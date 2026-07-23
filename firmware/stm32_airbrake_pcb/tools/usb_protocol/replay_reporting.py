"""Host clock, evidence-bundle, and GUI fan-out support for live replay.

Architecture references:
  [ARCH-3] The STM32 USB simulation stream is explicit and time-bounded.
  [ARCH-6] The replay process is the sole COM-port owner; a GUI receives an
           optional newline-JSON mirror over localhost UDP.
  [ARCH-8] A presentation run leaves machine-readable evidence and a verdict.

The module deliberately knows nothing about the rocket packet schema.  Its
caller supplies JSON-safe event dictionaries, which keeps serial decoding and
safety policy in ``replay_openrocket.py``.  File-write failures are allowed to
propagate so a motion run cannot silently continue after losing its requested
evidence trail.  UDP failures are counted but remain non-fatal because a GUI
must never interfere with serial cleanup or actuator safety.

Sections:
  1. High-resolution injectable host clock.
  2. Atomic JSON and localhost UDP helpers.
  3. Run observer used by the replay control path.
"""

from __future__ import annotations

import copy
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import queue
import socket
import threading
import time
from typing import Any, Callable, Mapping, Protocol


# ---------------------------------------------------------------------------
# High-resolution host clock
# ---------------------------------------------------------------------------


class HostClock(Protocol):
    """Minimal clock contract used by scheduling, freshness, and tests."""

    def now(self) -> float:
        """Return high-resolution host seconds from an arbitrary epoch."""

    def sleep(self, seconds: float) -> None:
        """Yield for up to the requested positive duration."""


class PerfCounterClock:
    """Production clock backed by ``time.perf_counter``.

    ``perf_counter`` is monotonic and has the highest available host resolution;
    its epoch is intentionally not compared with STM32 packet timestamps.
    """

    def now(self) -> float:
        return time.perf_counter()

    def sleep(self, seconds: float) -> None:
        if seconds > 0.0:
            time.sleep(seconds)


# ---------------------------------------------------------------------------
# Persistence and localhost GUI transport
# ---------------------------------------------------------------------------


def _utc_now_text() -> str:
    """Return an ISO-8601 UTC timestamp for human correlation in artifacts."""

    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _write_json_atomic(path: Path, value: Mapping[str, Any]) -> None:
    """Replace one JSON artifact atomically after its complete text is ready."""

    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


class LocalhostJsonPublisher:
    """Best-effort newline-JSON UDP publisher fixed to 127.0.0.1."""

    def __init__(self, port: int) -> None:
        if not 1 <= port <= 65535:
            raise ValueError("GUI UDP port must be within 1..65535")
        self.target = ("127.0.0.1", port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.errors = 0

    def publish(self, record: Mapping[str, Any]) -> None:
        """Send one self-contained datagram; a missing GUI is harmless for UDP."""

        payload = (
            json.dumps(record, separators=(",", ":"), sort_keys=True) + "\n"
        ).encode("utf-8")
        try:
            self.socket.sendto(payload, self.target)
        except OSError:
            self.errors += 1

    def close(self) -> None:
        self.socket.close()


# ---------------------------------------------------------------------------
# Replay run observer
# ---------------------------------------------------------------------------


class ReplayEvidenceWriterError(RuntimeError):
    """Raised when ordered replay evidence can no longer be persisted."""


class ReplayRunObserver:
    """Fan out replay events to a persistent bundle and/or localhost GUI.

    ``packets.jsonl`` intentionally includes control/timing records as well as
    wire packets.  This creates one ordered host-time transcript without
    pretending that host ``perf_counter`` time and STM32 ``time_ms`` share an
    epoch.

    Event indexing and snapshots happen on the replay thread. JSON encoding,
    JSONL writes/fsync, SQLite/event-sink work, and UDP publication happen on
    one bounded FIFO worker so durable evidence cannot stall the 50 Hz serial
    scheduler. ``finalize`` drains that FIFO before closing any output and is
    idempotent.
    """

    _STOP = object()

    def __init__(
        self,
        *,
        clock: HostClock,
        bundle_dir: str | Path | None = None,
        gui_udp_port: int | None = None,
        event_sink: Callable[[Mapping[str, Any]], None] | None = None,
        flush_interval_s: float = 1.0,
        event_log_name: str = "packets.jsonl",
        record_context: Mapping[str, Any] | None = None,
        max_pending_events: int = 4096,
    ) -> None:
        if flush_interval_s <= 0.0:
            raise ValueError("flush interval must be positive")
        if Path(event_log_name).name != event_log_name:
            raise ValueError("event log name must be a filename")
        if max_pending_events <= 0:
            raise ValueError("maximum pending events must be positive")
        self.clock = clock
        self.started_host_s = clock.now()
        self.bundle_dir = Path(bundle_dir).resolve() if bundle_dir is not None else None
        self.publisher = (
            LocalhostJsonPublisher(gui_udp_port)
            if gui_udp_port is not None
            else None
        )
        self.event_sink = event_sink
        self.flush_interval_s = flush_interval_s
        self.event_log_name = event_log_name
        self.record_context = dict(record_context or {})
        self.max_pending_events = max_pending_events
        self._packet_file = None
        self._manifest: dict[str, Any] | None = None
        self._finalized = False
        self._event_index = 0
        self._last_flush_s = self.started_host_s
        self._event_queue: queue.Queue[object] = queue.Queue(
            maxsize=max_pending_events
        )
        self._writer_thread: threading.Thread | None = None
        self._writer_error: BaseException | None = None
        self._writer_error_lock = threading.Lock()
        self._writer_broken = threading.Event()

    @property
    def enabled(self) -> bool:
        return (
            self.bundle_dir is not None
            or self.publisher is not None
            or self.event_sink is not None
        )

    def _flush_events(self) -> None:
        """Durably flush the append-only transcript at a bounded cadence."""

        if self._packet_file is None:
            return
        self._packet_file.flush()
        os.fsync(self._packet_file.fileno())
        self._last_flush_s = self.clock.now()

    def _set_writer_error(
        self,
        error: BaseException,
        *,
        writer_broken: bool,
    ) -> None:
        with self._writer_error_lock:
            if self._writer_error is None:
                self._writer_error = error
        if writer_broken:
            self._writer_broken.set()

    def _writer_failure(self) -> BaseException | None:
        with self._writer_error_lock:
            return self._writer_error

    @staticmethod
    def _writer_error_message(error: BaseException) -> str:
        return (
            "ordered replay evidence writer failed: "
            f"{type(error).__name__}: {error}"
        )

    def _raise_if_writer_failed(self) -> None:
        error = self._writer_failure()
        if error is not None:
            raise ReplayEvidenceWriterError(
                self._writer_error_message(error)
            ) from error

    def _write_record(self, record: Mapping[str, Any]) -> None:
        if self._packet_file is not None:
            self._packet_file.write(
                json.dumps(record, separators=(",", ":"), sort_keys=True)
                + "\n"
            )
            if self.clock.now() - self._last_flush_s >= self.flush_interval_s:
                self._flush_events()
        if self.event_sink is not None:
            self.event_sink(record)
        if self.publisher is not None:
            self.publisher.publish(record)

    def _finish_event_sink(self) -> None:
        if self.event_sink is None:
            return
        flush = getattr(self.event_sink, "flush", None)
        if callable(flush):
            flush()
        close = getattr(self.event_sink, "close", None)
        if callable(close):
            close()

    def _writer_main(self) -> None:
        while True:
            item = self._event_queue.get()
            if item is self._STOP:
                for operation in (self._flush_events, self._finish_event_sink):
                    try:
                        operation()
                    except BaseException as error:
                        self._set_writer_error(error, writer_broken=True)
                return
            if self._writer_broken.is_set():
                # Preserve the successfully written ordered prefix. The run
                # will fault, and recovery will never infer missing records.
                continue
            try:
                assert isinstance(item, Mapping)
                self._write_record(item)
            except BaseException as error:
                self._set_writer_error(error, writer_broken=True)

    def _start_writer(self) -> None:
        if not self.enabled or self._writer_thread is not None:
            return
        self._writer_thread = threading.Thread(
            target=self._writer_main,
            name="AMBAR-replay-evidence-writer",
            daemon=True,
        )
        self._writer_thread.start()

    def _stop_writer(self) -> None:
        thread = self._writer_thread
        if thread is None:
            return
        if thread.is_alive():
            # Finalization occurs after serial safety cleanup, so it is correct
            # to wait here until every accepted event has been handled.
            self._event_queue.put(self._STOP)
            thread.join()
        self._writer_thread = None

    def _enqueue(self, record: Mapping[str, Any]) -> None:
        if not self.enabled:
            return
        self._raise_if_writer_failed()
        try:
            self._event_queue.put_nowait(record)
        except queue.Full as error:
            overflow = ReplayEvidenceWriterError(
                "ordered replay evidence queue exceeded "
                f"{self.max_pending_events} pending events"
            )
            # The worker remains healthy and drains the already accepted FIFO
            # prefix; no later record is accepted after this fail-closed edge.
            self._set_writer_error(overflow, writer_broken=False)
            raise overflow from error

    @staticmethod
    def _failed_verdict(
        verdict: Mapping[str, Any],
        error: BaseException,
    ) -> dict[str, Any]:
        value = copy.deepcopy(dict(verdict))
        message = ReplayRunObserver._writer_error_message(error)
        value["status"] = "FAIL"
        value["passed"] = False
        value["failure"] = {
            "type": "ReplayEvidenceWriterError",
            "message": message,
        }
        checks = value.setdefault("checks", {})
        checks["evidence_writer"] = {
            "applicable": True,
            "passed": False,
            "expected": "ordered JSONL, SQLite, and dashboard fan-out completed",
            "actual": message,
        }
        return value

    def start(self, manifest: Mapping[str, Any]) -> None:
        """Create/truncate this run's artifacts before serial motion can begin."""

        if self._manifest is not None:
            raise RuntimeError("run observer has already started")
        self._manifest = {
            "schema": "ambar.live_replay_manifest.v1",
            "started_utc": _utc_now_text(),
            "status": "RUNNING",
            **dict(manifest),
        }
        if self.bundle_dir is not None:
            self.bundle_dir.mkdir(parents=True, exist_ok=True)
            _write_json_atomic(self.bundle_dir / "manifest.json", self._manifest)
            self._packet_file = (self.bundle_dir / self.event_log_name).open(
                "w",
                encoding="utf-8",
                newline="\n",
            )
        self._start_writer()
        self.emit("run_started")

    def emit(self, event: str, **fields: Any) -> dict[str, Any]:
        """Append and mirror one ordered event using host elapsed time only."""

        record: dict[str, Any] = {
            "schema": "ambar.live_replay_event.v1",
            "event_index": self._event_index,
            "event": event,
            "host_elapsed_s": round(self.clock.now() - self.started_host_s, 9),
            **self.record_context,
            **fields,
        }
        self._event_index += 1
        # Preserve the synchronous observer's snapshot semantics without doing
        # any filesystem, database, or socket work on the replay thread.
        queued_record = copy.deepcopy(record)
        self._enqueue(queued_record)
        return record

    def finalize(self, verdict: Mapping[str, Any]) -> None:
        """Persist the final verdict and close outputs exactly once.

        The verdict event is written before closing ``packets.jsonl`` so a
        consumer can reconstruct the complete run from that single stream.
        """

        if self._finalized:
            return
        self._finalized = True
        final_value = dict(verdict)
        writer_error = self._writer_failure()
        if writer_error is not None:
            final_value = self._failed_verdict(final_value, writer_error)
        else:
            try:
                self.emit("run_verdict", verdict=final_value)
            except ReplayEvidenceWriterError:
                writer_error = self._writer_failure()
                if writer_error is not None:
                    final_value = self._failed_verdict(
                        final_value,
                        writer_error,
                    )

        self._stop_writer()
        writer_error = self._writer_failure()
        if writer_error is not None:
            final_value = self._failed_verdict(final_value, writer_error)

        if self._packet_file is not None:
            self._packet_file.close()
            self._packet_file = None

        if self.bundle_dir is not None:
            _write_json_atomic(self.bundle_dir / "verdict.json", final_value)
            assert self._manifest is not None
            self._manifest["status"] = str(final_value.get("status", "UNKNOWN"))
            self._manifest["finished_utc"] = _utc_now_text()
            self._manifest["artifacts"] = [
                "manifest.json",
                self.event_log_name,
                "verdict.json",
            ]
            self._manifest["gui_udp_errors"] = (
                self.publisher.errors if self.publisher is not None else 0
            )
            _write_json_atomic(self.bundle_dir / "manifest.json", self._manifest)

        if self.publisher is not None:
            self.publisher.close()
            self.publisher = None

        if writer_error is not None:
            raise ReplayEvidenceWriterError(
                self._writer_error_message(writer_error)
            ) from writer_error
