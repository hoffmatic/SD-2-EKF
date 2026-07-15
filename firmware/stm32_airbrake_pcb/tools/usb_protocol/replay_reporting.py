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

from datetime import datetime, timezone
import json
from pathlib import Path
import socket
import time
from typing import Any, Mapping, Protocol


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


class ReplayRunObserver:
    """Fan out replay events to a persistent bundle and/or localhost GUI.

    ``packets.jsonl`` intentionally includes control/timing records as well as
    wire packets.  This creates one ordered host-time transcript without
    pretending that host ``perf_counter`` time and STM32 ``time_ms`` share an
    epoch.  ``finalize`` is idempotent and writes the verdict on both success
    and handled failure paths.
    """

    def __init__(
        self,
        *,
        clock: HostClock,
        bundle_dir: str | Path | None = None,
        gui_udp_port: int | None = None,
    ) -> None:
        self.clock = clock
        self.started_host_s = clock.now()
        self.bundle_dir = Path(bundle_dir).resolve() if bundle_dir is not None else None
        self.publisher = (
            LocalhostJsonPublisher(gui_udp_port)
            if gui_udp_port is not None
            else None
        )
        self._packet_file = None
        self._manifest: dict[str, Any] | None = None
        self._finalized = False

    @property
    def enabled(self) -> bool:
        return self.bundle_dir is not None or self.publisher is not None

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
            self._packet_file = (self.bundle_dir / "packets.jsonl").open(
                "w",
                encoding="utf-8",
                newline="\n",
            )
        self.emit("run_started")

    def emit(self, event: str, **fields: Any) -> dict[str, Any]:
        """Append and mirror one ordered event using host elapsed time only."""

        record: dict[str, Any] = {
            "schema": "ambar.live_replay_event.v1",
            "event": event,
            "host_elapsed_s": round(self.clock.now() - self.started_host_s, 9),
            **fields,
        }
        if self._packet_file is not None:
            self._packet_file.write(
                json.dumps(record, separators=(",", ":"), sort_keys=True) + "\n"
            )
        if self.publisher is not None:
            self.publisher.publish(record)
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
        self.emit("run_verdict", verdict=final_value)

        if self._packet_file is not None:
            self._packet_file.flush()
            self._packet_file.close()
            self._packet_file = None

        if self.bundle_dir is not None:
            _write_json_atomic(self.bundle_dir / "verdict.json", final_value)
            assert self._manifest is not None
            self._manifest["status"] = str(final_value.get("status", "UNKNOWN"))
            self._manifest["finished_utc"] = _utc_now_text()
            self._manifest["artifacts"] = [
                "manifest.json",
                "packets.jsonl",
                "verdict.json",
            ]
            self._manifest["gui_udp_errors"] = (
                self.publisher.errors if self.publisher is not None else 0
            )
            _write_json_atomic(self.bundle_dir / "manifest.json", self._manifest)

        if self.publisher is not None:
            self.publisher.close()
            self.publisher = None
