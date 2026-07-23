"""Validate and replay OpenRocket data over the AMBAR direct USB link.

Architecture references:
  [ARCH-3] This host owns the explicit, timeout-bounded USB simulation stream.
  [ARCH-6] It is the sole STM32 COM owner and may mirror decoded newline JSON
           to a GUI over localhost UDP; the GUI never opens COM independently.
  [ARCH-8] Scheduling uses an injectable high-resolution host clock, and an
           optional evidence bundle records packets, timing, and acceptance.

Safety and control flow:
  * Dry-run inspection remains the default and never opens a serial port.
  * Live use validates input and board feature/status gates before ARM.
  * Physical motion additionally requires two explicit operator acknowledgements.
  * Every live exit attempts DISARM then SIM_STOP; failure cleanup never performs
    a blind retract. Successful motion runs retract and verify energy is off.

Host ``perf_counter`` time schedules samples and checks freshness. STM32 packet
``time_ms`` is recorded as a separate device time domain and is never compared
directly with the host clock. The presentation position is TMC5240 XACTUAL, not
encoder feedback, so evidence proves the commanded/internal position path but
does not by itself prove physical shaft motion. See [ARCH-5] and [ARCH-8].

Sections follow the operator workflow: CSV/profile preparation, mechanical
metadata, serial monitoring/reporting, guarded replay, then CLI entry points.
"""

from __future__ import annotations

import argparse
import bisect
import csv
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re
import secrets
import struct
import sys
from typing import Any, Callable, Mapping, Sequence

from rocket_protocol import (
    ACK_OK,
    ACTUATOR_STATUS_CONTINUOUS_HIL_PROFILE,
    ACTUATOR_STATUS_FULL_ACTIVE,
    ACTUATOR_STATUS_GEOMETRY_PLAUSIBLE,
    ACTUATOR_STATUS_HOME_ACTIVE,
    ACTUATOR_STATUS_HIL_OVERRIDE_ACTIVE,
    ACTUATOR_STATUS_LIMITS_PLAUSIBLE,
    ACTUATOR_STATUS_SOFTWARE_FULL_ACTIVE,
    ACTUATOR_STATUS_SOFTWARE_HOME_ACTIVE,
    ACTUATOR_STATUS_STROKE_SEQUENCE_VERIFIED,
    CMD_HIL_SET_OVERRIDE,
    CMD_HOME,
    CMD_PING,
    CMD_RETRACT,
    CMD_REQUEST_SNAPSHOT,
    CMD_SET_ARMED,
    CMD_SET_TARGET_APOGEE,
    CMD_SIM_START,
    CMD_SIM_STOP,
    HIL_OVERRIDE_FORCE_FULL,
    HIL_OVERRIDE_FORCE_HOME,
    HIL_OVERRIDE_OFF,
    PKT_ACK,
    PKT_ACTUATOR_STATUS,
    PKT_COMMAND,
    PKT_EVENT,
    PKT_HEARTBEAT,
    PKT_SIMULATION,
    PKT_TELEMETRY,
    StreamDecoder,
    command_frame,
    decode_ack,
    decode_actuator_status,
    decode_event,
    decode_heartbeat,
    decode_telemetry,
    simulation_frame,
)
from replay_reporting import HostClock, PerfCounterClock, ReplayRunObserver


AMBAR_USB_VID = 0x0483
AMBAR_USB_PID = 0x5740

FEATURE_ACTUATOR = 1 << 3
FEATURE_BENCH_COMMANDS = 1 << 4
FEATURE_USB_PROTOCOL = 1 << 10
FEATURE_SIMULATION = 1 << 11
FEATURE_CONTINUOUS_HIL = 1 << 14
# Compatibility alias for the finite presentation replay workflow.
FEATURE_PRESENTATION_MOTION = FEATURE_CONTINUOUS_HIL
FEATURE_DIRECTION_INVERTED = 1 << 15

PRESENTATION_FULL_TRAVEL_STEPS = 3 * 200 * 256
PROJECT_ROOT = Path(__file__).resolve().parents[2]

ACTUATOR_FLAG_BUILD_ENABLED = 1 << 0
ACTUATOR_FLAG_BENCH_ENABLED = 1 << 1
ACTUATOR_FLAG_HOMED = 1 << 2
ACTUATOR_FLAG_DRIVER_OK = 1 << 3
ACTUATOR_FLAG_DRIVER_ENABLED = 1 << 4
ACTUATOR_FLAG_ESTOP = 1 << 5
ACTUATOR_FLAG_CONFIG_VALID = 1 << 6
ACTUATOR_FLAG_MANUAL_PENDING = 1 << 7
TELEMETRY_FLAG_SIMULATION_ACTIVE = 1 << 15

ACTUATOR_STATE_FAULT = 6
ACTUATOR_STATE_ESTOP = 7
ACTUATOR_POSITION_TOLERANCE_STEPS = 100
MAX_HOST_SCHEDULE_LAG_S = 0.100

TMC_DRV_STATUS_S2GB = 0x10000000
TMC_DRV_STATUS_S2GA = 0x08000000
TMC_DRV_STATUS_OTPW = 0x04000000
TMC_DRV_STATUS_OT = 0x02000000
TMC_DRV_STATUS_S2VSB = 0x00002000
TMC_DRV_STATUS_S2VSA = 0x00001000
TMC_DRV_STATUS_STOP_MASK = 0x1E003000
TMC_DRV_STATUS_THERMAL_MASK = TMC_DRV_STATUS_OTPW | TMC_DRV_STATUS_OT

PHASE_NAMES = {
    0: "PAD_IDLE",
    1: "BOOST",
    2: "COAST",
    3: "AIRBRAKE_ACTIVE",
    4: "RECOVERY",
    5: "FAULT",
}

EVENT_RE = re.compile(
    r"^Event\s+([A-Z0-9_]+)\s+occurred\s+at\s+t=([-+0-9.eE]+)\s+seconds$"
)


# ---------------------------------------------------------------------------
# Protocol/build constants and immutable replay models
# ---------------------------------------------------------------------------


def _shareable_project_path(path: str | Path) -> str:
    """Return a repo-relative provenance path without exposing a user profile."""

    resolved = Path(path).resolve()
    try:
        return resolved.relative_to(PROJECT_ROOT).as_posix()
    except ValueError:
        # External inputs retain a useful identity through filename plus SHA-256.
        return resolved.name


class ReplayError(RuntimeError):
    """Raised when the input or live board state is unsafe for replay."""


class ReplayDeadlineError(ReplayError):
    """Replay fault that preserves the measured over-limit scheduling lag."""

    def __init__(self, lag_s: float) -> None:
        self.lag_s = float(lag_s)
        super().__init__(
            "host missed a replay deadline by "
            f"{self.lag_s * 1000.0:.1f} ms"
        )


@dataclass(frozen=True)
class OpenRocketEvent:
    """One named OpenRocket event at source-relative seconds."""

    name: str
    time_s: float


@dataclass(frozen=True)
class OpenRocketDataset:
    """Validated numeric OpenRocket columns plus resolved semantic indexes."""

    path: Path
    headers: tuple[str, ...]
    columns: tuple[tuple[float, ...], ...]
    events: tuple[OpenRocketEvent, ...]
    time_index: int
    altitude_index: int
    vertical_velocity_index: int | None
    vertical_acceleration_index: int | None
    total_velocity_index: int | None
    total_acceleration_index: int | None

    @property
    def row_count(self) -> int:
        return len(self.columns[0])

    def values(self, index: int) -> tuple[float, ...]:
        return self.columns[index]

    def event_times(self, name: str) -> tuple[float, ...]:
        wanted = name.upper()
        return tuple(event.time_s for event in self.events if event.name == wanted)


@dataclass(frozen=True)
class ReplaySample:
    """One uniformly scheduled vertical input sample in SI units."""

    replay_time_s: float
    source_time_s: float
    altitude_m: float
    vertical_velocity_mps: float
    vertical_acceleration_mps2: float


@dataclass(frozen=True)
class ReplayProfile:
    """Bounded-rate replay samples and the provenance needed for inspection."""

    samples: tuple[ReplaySample, ...]
    rate_hz: float
    prepad_s: float
    source_stop_s: float
    vertical_source: str
    uses_derived_vertical: bool
    source_max_gap_s: float
    warnings: tuple[str, ...]


@dataclass(frozen=True)
class ReplayDeadlineResult:
    """One sample's host-scheduling result.

    ``rebase_s`` is nonzero only for continuous HIL.  It shifts all later
    deadlines by the tolerated delay so every input row is sent without a
    catch-up burst.
    """

    lag_s: float
    skip: bool
    rebase_s: float


# ---------------------------------------------------------------------------
# CSV validation, vertical reconstruction, and uniform resampling
# ---------------------------------------------------------------------------


def _canonical_header(value: str) -> str:
    """Normalize a CSV heading for tolerant semantic matching."""

    text = value.lower().replace("²", "2")
    return " ".join(re.sub(r"[^a-z0-9]+", " ", text).split())


def _find_header(headers: Sequence[str], phrase: str) -> int | None:
    """Return the first column whose normalized heading contains ``phrase``."""

    wanted = _canonical_header(phrase)
    for index, header in enumerate(headers):
        if wanted in _canonical_header(header):
            return index
    return None


def _normalized_unit(header: str) -> str | None:
    """Extract and normalize the trailing parenthesized unit from a heading."""

    match = re.search(r"\(([^()]*)\)\s*$", header)
    if match is None:
        return None
    unit = match.group(1).lower().replace(" ", "")
    unit = unit.replace("Â²", "^2").replace("²", "^2")
    return unit


def _require_unit(headers: Sequence[str], index: int, accepted: set[str]) -> None:
    """Reject a resolved column when its declared unit is absent or unsafe."""

    unit = _normalized_unit(headers[index])
    if unit not in accepted:
        expected = " or ".join(sorted(accepted))
        raise ReplayError(
            f"unsupported or missing units in '{headers[index]}'; expected {expected}"
        )


def load_openrocket_csv(path: str | Path) -> OpenRocketDataset:
    """Load numeric CSV data, event comments, semantic columns, and SI units."""

    source = Path(path)
    if not source.is_file():
        raise ReplayError(f"OpenRocket CSV not found: {source}")

    headers: list[str] | None = None
    rows: list[tuple[float, ...]] = []
    events: list[OpenRocketEvent] = []

    for line_number, raw_line in enumerate(
        source.read_text(encoding="utf-8-sig").splitlines(), start=1
    ):
        line = raw_line.strip()
        if not line:
            continue
        if line.startswith("#"):
            comment = line[1:].strip()
            event_match = EVENT_RE.match(comment)
            if event_match:
                events.append(OpenRocketEvent(event_match.group(1), float(event_match.group(2))))
            if headers is None and "," in comment and "time" in comment.lower():
                candidate = next(csv.reader([comment]))
                if len(candidate) >= 2:
                    headers = [item.strip() for item in candidate]
            continue

        fields = next(csv.reader([line]))
        if headers is None:
            try:
                tuple(float(value) for value in fields)
            except ValueError:
                headers = [item.strip() for item in fields]
                continue
            raise ReplayError(f"numeric data appeared before a header on line {line_number}")
        if len(fields) != len(headers):
            raise ReplayError(
                f"line {line_number} has {len(fields)} fields; expected {len(headers)}"
            )
        try:
            row = tuple(float(value) for value in fields)
        except ValueError as exc:
            raise ReplayError(f"line {line_number} contains a nonnumeric value") from exc
        if not all(math.isfinite(value) for value in row):
            raise ReplayError(f"line {line_number} contains NaN or infinity")
        rows.append(row)

    if headers is None or not rows:
        raise ReplayError("CSV does not contain a usable header and numeric data")

    time_index = _find_header(headers, "time")
    altitude_index = _find_header(headers, "altitude")
    if time_index is None or altitude_index is None:
        raise ReplayError("CSV must contain Time and Altitude columns")

    columns = tuple(tuple(row[index] for row in rows) for index in range(len(headers)))
    times = columns[time_index]
    if len(times) < 2:
        raise ReplayError("CSV must contain at least two numeric rows")
    if abs(times[0]) > 1e-6:
        raise ReplayError("the first timestamp must be 0 seconds")
    for index in range(1, len(times)):
        if times[index] <= times[index - 1]:
            raise ReplayError(
                f"timestamps must be strictly increasing; row {index + 1} is invalid"
            )

    vertical_velocity_index = _find_header(headers, "vertical velocity")
    vertical_acceleration_index = _find_header(headers, "vertical acceleration")
    total_velocity_index = _find_header(headers, "total velocity")
    total_acceleration_index = _find_header(headers, "total acceleration")

    _require_unit(headers, time_index, {"s"})
    _require_unit(headers, altitude_index, {"m"})
    for velocity_index in (vertical_velocity_index, total_velocity_index):
        if velocity_index is not None:
            _require_unit(headers, velocity_index, {"m/s"})
    for acceleration_index in (vertical_acceleration_index, total_acceleration_index):
        if acceleration_index is not None:
            _require_unit(headers, acceleration_index, {"m/s2", "m/s^2"})

    return OpenRocketDataset(
        path=source.resolve(),
        headers=tuple(headers),
        columns=columns,
        events=tuple(events),
        time_index=time_index,
        altitude_index=altitude_index,
        vertical_velocity_index=vertical_velocity_index,
        vertical_acceleration_index=vertical_acceleration_index,
        total_velocity_index=total_velocity_index,
        total_acceleration_index=total_acceleration_index,
    )


def _linear_interpolate(times: Sequence[float], values: Sequence[float], query: float) -> float:
    """Interpolate a bounded scalar channel on strictly increasing timestamps."""

    if query <= times[0]:
        return values[0]
    if query >= times[-1]:
        return values[-1]
    upper = bisect.bisect_right(times, query)
    lower = upper - 1
    span = times[upper] - times[lower]
    fraction = (query - times[lower]) / span
    return values[lower] + fraction * (values[upper] - values[lower])


def _solve_linear(matrix: list[list[float]], vector: list[float]) -> list[float]:
    """Solve the small normal equation used by the local altitude fit."""

    size = len(vector)
    augmented = [row[:] + [vector[index]] for index, row in enumerate(matrix)]
    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-12:
            raise ReplayError("altitude derivative fit is singular")
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        divisor = augmented[column][column]
        augmented[column] = [value / divisor for value in augmented[column]]
        for row in range(size):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [
                augmented[row][item] - factor * augmented[column][item]
                for item in range(size + 1)
            ]
    return [augmented[row][-1] for row in range(size)]


def _altitude_kinematics(
    times: Sequence[float],
    altitudes: Sequence[float],
    query: float,
    point_count: int,
) -> tuple[float, float, float]:
    """Estimate altitude and its first two derivatives around one query time."""

    count = min(max(point_count, 5), len(times))
    upper = bisect.bisect_left(times, query)
    start = max(0, min(len(times) - count, upper - count // 2))
    selected_times = times[start : start + count]
    selected_altitudes = altitudes[start : start + count]

    relative = [value - query for value in selected_times]
    scale = max(max(abs(value) for value in relative), 1e-6)
    normalized = [value / scale for value in relative]
    degree = min(3, count - 1)

    normal = [[0.0 for _ in range(degree + 1)] for _ in range(degree + 1)]
    rhs = [0.0 for _ in range(degree + 1)]
    for x_value, y_value in zip(normalized, selected_altitudes):
        powers = [1.0]
        for _ in range(2 * degree):
            powers.append(powers[-1] * x_value)
        for row in range(degree + 1):
            rhs[row] += y_value * powers[row]
            for column in range(degree + 1):
                normal[row][column] += powers[row + column]

    coefficients = _solve_linear(normal, rhs)
    altitude = coefficients[0]
    velocity = coefficients[1] / scale if degree >= 1 else 0.0
    acceleration = 2.0 * coefficients[2] / (scale * scale) if degree >= 2 else 0.0
    return altitude, velocity, acceleration


def build_replay_profile(
    dataset: OpenRocketDataset,
    *,
    rate_hz: float = 50.0,
    prepad_s: float = 1.0,
    stop_s: float | None = None,
    full_flight: bool = False,
    derivative_points: int = 11,
) -> ReplayProfile:
    """Resample the selected source interval onto an absolute 20-50 Hz grid."""

    if not 20.0 <= rate_hz <= 50.0:
        raise ReplayError("rate must be between 20 and 50 Hz")
    if prepad_s < 0.5:
        raise ReplayError("prepad must be at least 0.5 seconds")
    if derivative_points < 5 or derivative_points % 2 == 0:
        raise ReplayError("derivative point count must be an odd number of at least 5")

    times = dataset.values(dataset.time_index)
    altitude_raw = dataset.values(dataset.altitude_index)
    altitude_zero = altitude_raw[0]
    altitudes = tuple(value - altitude_zero for value in altitude_raw)
    max_gap = max(after - before for before, after in zip(times, times[1:]))

    if full_flight:
        selected_stop = times[-1]
    elif stop_s is not None:
        selected_stop = stop_s
    else:
        apogee_times = dataset.event_times("APOGEE")
        selected_stop = min(times[-1], (apogee_times[0] + 2.0) if apogee_times else times[-1])
    if selected_stop <= times[0] or selected_stop > times[-1]:
        raise ReplayError(f"stop time must be within ({times[0]}, {times[-1]}]")

    direct_velocity = (
        dataset.values(dataset.vertical_velocity_index)
        if dataset.vertical_velocity_index is not None
        else None
    )
    direct_acceleration = (
        dataset.values(dataset.vertical_acceleration_index)
        if dataset.vertical_acceleration_index is not None
        else None
    )
    total_acceleration = (
        dataset.values(dataset.total_acceleration_index)
        if dataset.total_acceleration_index is not None
        else None
    )

    uses_derived = direct_velocity is None or direct_acceleration is None
    if direct_velocity is not None and direct_acceleration is not None:
        vertical_source = "OpenRocket vertical velocity and vertical acceleration columns"
    elif total_acceleration is not None:
        vertical_source = (
            "DERIVED: altitude-fit vertical velocity plus signed Total acceleration magnitude"
        )
    else:
        vertical_source = "DERIVED: local polynomial derivatives of altitude"

    warnings: list[str] = []
    if uses_derived:
        warnings.append(
            "Vertical channels are derived because the export lacks explicit "
            "Vertical velocity/acceleration."
        )
    if max_gap >= 0.5:
        warnings.append(
            f"Raw source gap reaches {max_gap:.3f} s; uniform resampling is "
            "required to avoid SIMULATION_STALE."
        )
    tumble = dataset.event_times("TUMBLE")
    apogee = dataset.event_times("APOGEE")
    if tumble and apogee and tumble[0] <= apogee[0]:
        warnings.append(
            f"OpenRocket reports TUMBLE at {tumble[0]:.3f} s before apogee; "
            "verify the source model."
        )
    if not any("AIRBRAKE" in event.name for event in dataset.events):
        warnings.append(
            "The export has no airbrake event/position/drag channel; it is an "
            "open-loop trajectory replay."
        )
    if not apogee:
        warnings.append(
            "No APOGEE event was exported; maximum-altitude time is used for velocity signing."
        )

    liftoff_times = dataset.event_times("LIFTOFF")
    liftoff_s = liftoff_times[0] if liftoff_times else times[0]
    peak_altitude_index = max(range(len(altitudes)), key=altitudes.__getitem__)
    apogee_s = apogee[0] if apogee else times[peak_altitude_index]
    interval_s = 1.0 / rate_hz
    replay_stop = prepad_s + selected_stop
    sample_count = int(math.floor(replay_stop * rate_hz + 1e-9)) + 1
    replay_times = [index * interval_s for index in range(sample_count)]
    if replay_times[-1] < replay_stop - 1e-9:
        replay_times.append(replay_stop)

    samples: list[ReplaySample] = []
    last_acceleration_sign = 1.0
    for replay_time in replay_times:
        if replay_time < prepad_s:
            samples.append(ReplaySample(replay_time, 0.0, 0.0, 0.0, 0.0))
            continue

        source_time = min(selected_stop, replay_time - prepad_s)
        altitude = _linear_interpolate(times, altitudes, source_time)
        _, fitted_velocity, fitted_acceleration = _altitude_kinematics(
            times, altitudes, source_time, derivative_points
        )

        if direct_velocity is not None:
            velocity = _linear_interpolate(times, direct_velocity, source_time)
        else:
            velocity = fitted_velocity
            if source_time < apogee_s and velocity < 0.0:
                velocity = 0.0
            elif source_time > apogee_s and velocity > 0.0:
                velocity = 0.0

        if direct_acceleration is not None:
            acceleration = _linear_interpolate(times, direct_acceleration, source_time)
        elif total_acceleration is not None:
            magnitude = abs(_linear_interpolate(times, total_acceleration, source_time))
            if source_time <= liftoff_s:
                last_acceleration_sign = 1.0
            elif fitted_acceleration > 0.25:
                last_acceleration_sign = 1.0
            elif fitted_acceleration < -0.25:
                last_acceleration_sign = -1.0
            acceleration = magnitude * last_acceleration_sign
        else:
            acceleration = fitted_acceleration

        if not -1000.0 <= altitude <= 20000.0:
            raise ReplayError(f"altitude {altitude:.3f} m is outside the firmware limit")
        if not -500.0 <= acceleration <= 500.0:
            raise ReplayError(
                f"acceleration {acceleration:.3f} m/s^2 is outside the firmware limit"
            )
        if not -2000.0 <= velocity <= 2000.0:
            raise ReplayError(f"velocity {velocity:.3f} m/s is outside the firmware limit")

        samples.append(
            ReplaySample(replay_time, source_time, altitude, velocity, acceleration)
        )

    return ReplayProfile(
        samples=tuple(samples),
        rate_hz=rate_hz,
        prepad_s=prepad_s,
        source_stop_s=selected_stop,
        vertical_source=vertical_source,
        uses_derived_vertical=uses_derived,
        source_max_gap_s=max_gap,
        warnings=tuple(warnings),
    )


# ---------------------------------------------------------------------------
# Mechanical display metadata and normalized GUI export
# ---------------------------------------------------------------------------


def prototype_counts(
    deployment_percent: float,
    *,
    rotations: float,
    full_steps_per_revolution: int,
    microsteps: int,
    gear_ratio: float,
) -> tuple[float, int]:
    """Map a bounded deployment percentage to display rotations and counts."""

    fraction = min(max(deployment_percent / 100.0, 0.0), 1.0)
    intended_rotations = rotations * fraction
    full_counts = rotations * full_steps_per_revolution * microsteps * gear_ratio
    return intended_rotations, round(full_counts * fraction)


def validate_mechanical_assumptions(
    *,
    brake_count: int,
    force_per_brake_n: float,
    rotations: float,
    full_steps_per_revolution: int,
    microsteps: int,
    gear_ratio: float,
) -> None:
    """Validate user-supplied display assumptions before reporting/export."""

    if brake_count <= 0:
        raise ReplayError("brake count must be positive")
    if not math.isfinite(force_per_brake_n) or force_per_brake_n < 0.0:
        raise ReplayError("force per brake must be finite and nonnegative")
    if not math.isfinite(rotations) or rotations <= 0.0:
        raise ReplayError("full-extension rotations must be finite and positive")
    if full_steps_per_revolution <= 0 or microsteps <= 0:
        raise ReplayError("motor full steps and microsteps must be positive")
    if not math.isfinite(gear_ratio) or gear_ratio <= 0.0:
        raise ReplayError("gear ratio must be finite and positive")


def validate_presentation_motion_profile(
    *,
    rotations: float,
    full_steps_per_revolution: int,
    microsteps: int,
    gear_ratio: float,
) -> None:
    """Prevent host labels from disagreeing with the locked firmware travel."""
    exact_profile = (
        math.isclose(rotations, 3.0, rel_tol=0.0, abs_tol=1.0e-9)
        and full_steps_per_revolution == 200
        and microsteps == 256
        and math.isclose(gear_ratio, 1.0, rel_tol=0.0, abs_tol=1.0e-9)
    )
    if not exact_profile:
        _, requested_counts = prototype_counts(
            100.0,
            rotations=rotations,
            full_steps_per_revolution=full_steps_per_revolution,
            microsteps=microsteps,
            gear_ratio=gear_ratio,
        )
        raise ReplayError(
            "motion mode is locked to the reviewed firmware profile: "
            "3 rotations, 200 full steps/rev, 256 microsteps, 1:1 gearing "
            f"({PRESENTATION_FULL_TRAVEL_STEPS} counts); requested values imply "
            f"{requested_counts} counts"
        )


def export_replay_csv(
    path: str | Path,
    dataset: OpenRocketDataset,
    profile: ReplayProfile,
    *,
    barometer_stddev_m: float,
    target_apogee_m: float,
    brake_count: int,
    force_per_brake_n: float,
    rotations: float,
    full_steps_per_revolution: int,
    microsteps: int,
    gear_ratio: float,
) -> tuple[Path, Path]:
    """Write uniformly timed GUI input plus a machine-readable metadata sidecar."""

    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(
            (
                "simulation_time_s",
                "source_time_s",
                "altitude_m_agl",
                "vertical_velocity_mps_reference",
                "vertical_acceleration_mps2_gravity_removed",
                "barometer_stddev_m",
                "is_prepad",
            )
        )
        for sample in profile.samples:
            writer.writerow(
                (
                    f"{sample.replay_time_s:.6f}",
                    f"{sample.source_time_s:.6f}",
                    f"{sample.altitude_m:.6f}",
                    f"{sample.vertical_velocity_mps:.6f}",
                    f"{sample.vertical_acceleration_mps2:.6f}",
                    f"{barometer_stddev_m:.6f}",
                    "1" if sample.replay_time_s < profile.prepad_s else "0",
                )
            )
    _, full_counts = prototype_counts(
        100.0,
        rotations=rotations,
        full_steps_per_revolution=full_steps_per_revolution,
        microsteps=microsteps,
        gear_ratio=gear_ratio,
    )
    metadata_path = destination.with_suffix(destination.suffix + ".metadata.json")
    metadata = {
        "schema": "ambar.openrocket_replay.v1",
        "source": {
            "path": _shareable_project_path(dataset.path),
            "sha256": hashlib.sha256(dataset.path.read_bytes()).hexdigest().upper(),
            "numeric_rows": dataset.row_count,
            "headers": list(dataset.headers),
        },
        "replay": {
            "rate_hz": profile.rate_hz,
            "prepad_s": profile.prepad_s,
            "source_stop_s": profile.source_stop_s,
            "sample_count": len(profile.samples),
            "vertical_source": profile.vertical_source,
            "vertical_channels_derived": profile.uses_derived_vertical,
            "source_max_gap_s": profile.source_max_gap_s,
            "target_apogee_m": target_apogee_m,
            "barometer_stddev_m": barometer_stddev_m,
            "acceleration_convention": (
                "launch-frame vertical, positive up, pad-zeroed/gravity removed"
            ),
        },
        "mechanical_display_assumptions": {
            "verified": False,
            "brake_count": brake_count,
            "force_per_brake_n": force_per_brake_n,
            "combined_force_n": brake_count * force_per_brake_n,
            "full_extension_motor_rotations": rotations,
            "motor_full_steps_per_revolution": full_steps_per_revolution,
            "microsteps": microsteps,
            "gear_ratio": gear_ratio,
            "prototype_equivalent_full_counts": full_counts,
            "rotations_formula": (
                f"{rotations:g} * deployment_percent / 100"
            ),
            "counts_formula": (
                f"round({full_counts} * deployment_percent / 100)"
            ),
        },
    }
    metadata_path.write_text(
        json.dumps(metadata, indent=2) + "\n",
        encoding="utf-8",
    )
    return destination.resolve(), metadata_path.resolve()


def print_inspection(
    dataset: OpenRocketDataset,
    profile: ReplayProfile,
    *,
    brake_count: int,
    force_per_brake_n: float,
    rotations: float,
    full_steps_per_revolution: int,
    microsteps: int,
    gear_ratio: float,
) -> None:
    """Print an operator-readable dry-run audit without opening the COM port."""

    times = dataset.values(dataset.time_index)
    altitudes = dataset.values(dataset.altitude_index)
    peak_index = max(range(len(altitudes)), key=altitudes.__getitem__)
    intervals = [after - before for before, after in zip(times, times[1:])]
    accelerations = [sample.vertical_acceleration_mps2 for sample in profile.samples]
    velocities = [sample.vertical_velocity_mps for sample in profile.samples]
    full_rotations, full_counts = prototype_counts(
        100.0,
        rotations=rotations,
        full_steps_per_revolution=full_steps_per_revolution,
        microsteps=microsteps,
        gear_ratio=gear_ratio,
    )

    print(f"Source: {dataset.path}")
    print(f"Rows: {dataset.row_count}; duration: {times[-1] - times[0]:.4f} s")
    print(
        f"Source dt: min={min(intervals):.4f} s, max={max(intervals):.4f} s; "
        f"replay={profile.rate_hz:.1f} Hz"
    )
    print(
        f"Peak altitude: {altitudes[peak_index]:.4f} m at {times[peak_index]:.4f} s"
    )
    print(
        f"Replay segment: 0..{profile.source_stop_s:.3f} s plus "
        f"{profile.prepad_s:.2f} s pad; {len(profile.samples)} packets"
    )
    print(f"Vertical input: {profile.vertical_source}")
    print(
        f"Derived range: velocity {min(velocities):.2f}..{max(velocities):.2f} m/s; "
        f"acceleration {min(accelerations):.2f}..{max(accelerations):.2f} m/s^2"
    )
    print("Events:")
    for event in dataset.events:
        print(f"  {event.time_s:8.3f} s  {event.name}")
    print("Warnings:")
    for warning in profile.warnings:
        print(f"  - {warning}")
    print("Mechanical metadata (UNVERIFIED; not applied to trajectory):")
    print(
        f"  {brake_count} brakes x {force_per_brake_n:.1f} N = "
        f"{brake_count * force_per_brake_n:.1f} N assumed full-extension load envelope"
    )
    print(
        f"  100% deployment = {full_rotations:.3f} motor rotations = "
        f"{full_counts} prototype-equivalent counts"
    )


# ---------------------------------------------------------------------------
# Serial discovery, packet monitoring, and guarded wait helpers
# ---------------------------------------------------------------------------


def _require_serial():
    """Load pyserial lazily so dry-run inspection has no serial dependency."""

    try:
        import serial  # type: ignore
        from serial.tools import list_ports  # type: ignore
    except ImportError as exc:
        raise ReplayError(
            "pyserial is required for --live; install pyserial 3.5 in the selected Python runtime"
        ) from exc
    return serial, list_ports


def _choose_port(list_ports, requested: str | None) -> str:
    """Honor an explicit COM name or find the single reviewed STM32 VID:PID."""

    if requested:
        return requested
    matches = []
    for port in list_ports.comports():
        if port.vid == AMBAR_USB_VID and port.pid == AMBAR_USB_PID:
            matches.append(port)
    if len(matches) == 1:
        return str(matches[0].device)
    if len(matches) > 1:
        devices = ", ".join(str(port.device) for port in matches)
        raise ReplayError(
            "multiple AMBAR USB devices were found "
            f"({devices}); pass --port COMx explicitly"
        )
    raise ReplayError("AMBAR USB device 0483:5740 was not found; pass --port COMx")


# ---------------------------------------------------------------------------
# Live packet observation and high-resolution waits [ARCH-6] [ARCH-8]
# ---------------------------------------------------------------------------


def _host_packet_time_ms(clock: HostClock) -> int:
    """Convert the host clock to the protocol's wrapping diagnostic timestamp."""

    return round(clock.now() * 1000.0) & 0xFFFFFFFF


def _decoded_tx_payload(packet) -> dict[str, Any]:
    """Describe one host-originated packet without conflating its time domain."""

    if packet.packet_type == PKT_COMMAND and len(packet.payload) >= 2:
        declared_length = int(packet.payload[1])
        return {
            "command": int(packet.payload[0]),
            "declared_length": declared_length,
            "data_hex": packet.payload[2 : 2 + declared_length].hex().upper(),
        }
    if packet.packet_type == PKT_SIMULATION and len(packet.payload) == 16:
        flags, altitude_mm, acceleration_mmps2, velocity_mmps, stddev_cm = (
            struct.unpack("<HiiiH", packet.payload)
        )
        return {
            "flags": flags,
            "altitude_mm": altitude_mm,
            "acceleration_mmps2": acceleration_mmps2,
            "velocity_mmps": velocity_mmps,
            "barometer_stddev_cm": stddev_cm,
        }
    return {"payload_hex": packet.payload.hex().upper()}


class _ObservedSerialPort:
    """Record successful host writes while preserving the pyserial interface."""

    def __init__(
        self,
        port,
        *,
        clock: HostClock,
        observer: ReplayRunObserver,
    ) -> None:
        self._port = port
        self.clock = clock
        self.observer = observer
        self.decoder = StreamDecoder()

    def read(self, size: int) -> bytes:
        return self._port.read(size)

    def write(self, data: bytes) -> int:
        try:
            written = self._port.write(data)
        except Exception as exc:
            self.observer.emit(
                "serial_write_error",
                requested_bytes=len(data),
                error_type=type(exc).__name__,
                error=str(exc),
            )
            raise
        for packet in self.decoder.feed(bytes(data[:written])):
            self.observer.emit(
                "packet_tx",
                packet_type=packet.packet_type,
                sequence=packet.sequence,
                packet_time_ms=packet.time_ms,
                packet_time_domain="host_frame",
                decoded=_decoded_tx_payload(packet),
            )
        return written


class PacketMonitor:
    """Decode live board traffic and retain the evidence needed for acceptance."""

    def __init__(
        self,
        *,
        clock: HostClock | None = None,
        observer: ReplayRunObserver | None = None,
    ) -> None:
        self.clock = clock if clock is not None else PerfCounterClock()
        self.observer = observer
        self.decoder = StreamDecoder()
        self.acks: dict[int, dict[str, int]] = {}
        self.telemetry: dict[str, int | float] | None = None
        self.actuator: dict[str, int | bool] | None = None
        self.heartbeat: dict[str, int] | None = None
        self.initial_heartbeat: dict[str, int] | None = None
        self.events: list[dict[str, int]] = []
        self.phase_order: list[int] = []
        self.max_altitude_m = float("-inf")
        self.max_predicted_apogee_m = float("-inf")
        self.max_deployment_percent = 0
        self.max_abs_actuator_target_steps = 0
        self.max_abs_actuator_actual_steps = 0
        self.max_actuator_tracking_error_steps = 0
        self.telemetry_count = 0
        self.actuator_count = 0
        self.telemetry_received_s: float | None = None
        self.actuator_received_s: float | None = None
        self.telemetry_packet_time_ms: int | None = None
        self.actuator_packet_time_ms: int | None = None

    def begin_replay_metrics(self) -> None:
        """Exclude preflight state from flight-phase and motion extrema.

        Packet counters and heartbeat baselines remain continuous so freshness
        gates and transport-error deltas still span the complete connection.
        """

        self.phase_order.clear()
        self.max_altitude_m = float("-inf")
        self.max_predicted_apogee_m = float("-inf")
        self.max_deployment_percent = 0
        self.max_abs_actuator_target_steps = 0
        self.max_abs_actuator_actual_steps = 0
        self.max_actuator_tracking_error_steps = 0

    def poll(self, port) -> None:
        """Consume currently available bytes and update both state and evidence."""

        chunk = port.read(4096)
        errors_before = self.decoder.errors
        for packet in self.decoder.feed(chunk):
            decoded: dict[str, int | float]
            if packet.packet_type == PKT_ACK:
                ack = decode_ack(packet.payload)
                self.acks[ack["command_sequence"]] = ack
                decoded = ack
            elif packet.packet_type == PKT_TELEMETRY:
                telemetry = decode_telemetry(packet.payload)
                self.telemetry = telemetry
                self.telemetry_count += 1
                self.telemetry_received_s = self.clock.now()
                self.telemetry_packet_time_ms = packet.time_ms
                phase = int(telemetry["state"])
                if not self.phase_order or self.phase_order[-1] != phase:
                    self.phase_order.append(phase)
                self.max_altitude_m = max(self.max_altitude_m, float(telemetry["altitude_m"]))
                self.max_predicted_apogee_m = max(
                    self.max_predicted_apogee_m,
                    float(telemetry["predicted_apogee_m"]),
                )
                self.max_deployment_percent = max(
                    self.max_deployment_percent,
                    int(telemetry["deployment_percent"]),
                )
                decoded = telemetry
            elif packet.packet_type == PKT_ACTUATOR_STATUS:
                actuator = decode_actuator_status(packet.payload)
                self.actuator = actuator
                self.actuator_count += 1
                self.actuator_received_s = self.clock.now()
                self.actuator_packet_time_ms = packet.time_ms
                target = int(actuator["target_steps"])
                actual = int(actuator["actual_steps"])
                self.max_abs_actuator_target_steps = max(
                    self.max_abs_actuator_target_steps,
                    abs(target),
                )
                self.max_abs_actuator_actual_steps = max(
                    self.max_abs_actuator_actual_steps,
                    abs(actual),
                )
                self.max_actuator_tracking_error_steps = max(
                    self.max_actuator_tracking_error_steps,
                    abs(target - actual),
                )
                decoded = actuator
            elif packet.packet_type == PKT_HEARTBEAT:
                heartbeat = decode_heartbeat(packet.payload)
                if self.initial_heartbeat is None:
                    self.initial_heartbeat = dict(heartbeat)
                self.heartbeat = heartbeat
                decoded = heartbeat
            elif packet.packet_type == PKT_EVENT:
                event = decode_event(packet.payload)
                self.events.append(event)
                print(f"EVENT {event}")
                decoded = event
            else:
                decoded = {"payload_hex": packet.payload.hex().upper()}

            if self.observer is not None:
                self.observer.emit(
                    "packet_rx",
                    packet_type=packet.packet_type,
                    sequence=packet.sequence,
                    packet_time_ms=packet.time_ms,
                    packet_time_domain="stm32_device",
                    decoded=decoded,
                )

        if self.decoder.errors != errors_before and self.observer is not None:
            self.observer.emit(
                "decoder_error",
                new_errors=self.decoder.errors - errors_before,
                total_errors=self.decoder.errors,
            )


class SequenceCounter:
    """Generate wrapping uint16 protocol sequences from a nonzero random seed."""

    def __init__(self, seed: int | None = None) -> None:
        if seed is None:
            seed = secrets.randbelow(0xFFFF) + 1
        if not 0 <= seed <= 0xFFFF:
            raise ValueError("sequence seed must fit uint16")
        self.value = seed

    def take(self) -> int:
        current = self.value
        self.value = (self.value + 1) & 0xFFFF
        return current


def _send_command(
    port,
    sequence: SequenceCounter,
    command: int,
    data: bytes = b"",
    *,
    clock: HostClock | None = None,
) -> int:
    """Send one command using the same injectable host timebase as scheduling."""

    active_clock = clock if clock is not None else getattr(port, "clock", PerfCounterClock())
    command_sequence = sequence.take()
    port.write(
        command_frame(
            command_sequence,
            command,
            data,
            time_ms=_host_packet_time_ms(active_clock),
        )
    )
    return command_sequence


def _wait_for_ack(
    port,
    monitor: PacketMonitor,
    command_sequence: int,
    command: int,
    timeout_s: float = 1.0,
) -> dict[str, int]:
    """Wait for a matching successful ACK on the monitor's host clock."""

    deadline = monitor.clock.now() + timeout_s
    while monitor.clock.now() < deadline:
        monitor.poll(port)
        ack = monitor.acks.get(command_sequence)
        if ack is not None:
            if ack["command"] != command or ack["result"] != ACK_OK:
                raise ReplayError(f"command 0x{command:02X} failed: {ack}")
            return ack
        monitor.clock.sleep(0.005)
    raise ReplayError(f"timeout waiting for command 0x{command:02X} ACK")


def _set_hil_override(
    port,
    monitor: PacketMonitor,
    sequence: SequenceCounter,
    mode: int,
    *,
    observer: ReplayRunObserver,
    timeout_s: float = 0.25,
) -> None:
    """Set one firmware-gated HIL override and require its explicit ACK."""

    command_sequence = _send_command(
        port,
        sequence,
        CMD_HIL_SET_OVERRIDE,
        bytes((mode,)),
    )
    observer.emit(
        "hil_override_command",
        mode=mode,
        command_sequence=command_sequence,
    )
    _wait_for_ack(
        port,
        monitor,
        command_sequence,
        CMD_HIL_SET_OVERRIDE,
        timeout_s=timeout_s,
    )


def _collect_preflight_status(port, monitor: PacketMonitor, seconds: float = 1.25) -> None:
    """Collect heartbeat and actuator status without mixing device/host time."""

    deadline = monitor.clock.now() + seconds
    while monitor.clock.now() < deadline:
        monitor.poll(port)
        if monitor.heartbeat is not None and monitor.actuator is not None:
            return
        monitor.clock.sleep(0.005)


def _enforce_no_motion(monitor: PacketMonitor) -> None:
    """Require the new HIL image while proving all physical demand is inactive."""

    if monitor.heartbeat is None or monitor.actuator is None:
        raise ReplayError("did not receive heartbeat and actuator status during preflight")
    features = int(monitor.heartbeat["feature_flags"])
    required = FEATURE_USB_PROTOCOL | FEATURE_SIMULATION | FEATURE_CONTINUOUS_HIL
    if (features & required) != required:
        raise ReplayError(
            "no-motion live replay requires the USB, simulation, and "
            f"CONTINUOUS_HIL feature bits: 0x{features:08X}"
        )
    flags = int(monitor.actuator["flags"])
    if flags & (
        ACTUATOR_FLAG_DRIVER_ENABLED
        | ACTUATOR_FLAG_MANUAL_PENDING
        | ACTUATOR_FLAG_ESTOP
    ):
        raise ReplayError(
            f"refusing no-motion replay because actuator demand is active: 0x{flags:02X}"
        )
    reserved = int(monitor.actuator.get("reserved", 0))
    if not reserved & ACTUATOR_STATUS_CONTINUOUS_HIL_PROFILE:
        raise ReplayError("actuator status does not identify the CONTINUOUS_HIL image")
    if reserved & ACTUATOR_STATUS_HIL_OVERRIDE_ACTIVE:
        raise ReplayError("refusing no-motion replay while HIL override is active")


def _enforce_actuator_motion(
    monitor: PacketMonitor,
    *,
    require_geometry: bool = True,
) -> None:
    """Require the motion build, energy off, and optionally declared geometry."""
    if monitor.heartbeat is None or monitor.actuator is None:
        raise ReplayError("did not receive heartbeat and actuator status during preflight")

    features = int(monitor.heartbeat["feature_flags"])
    required_features = (
        FEATURE_USB_PROTOCOL
        | FEATURE_SIMULATION
        | FEATURE_ACTUATOR
        | FEATURE_BENCH_COMMANDS
        | FEATURE_PRESENTATION_MOTION
    )
    missing_features = required_features & ~features
    if missing_features:
        raise ReplayError(
            "motion replay requires the USB, simulation, actuator, bench, and "
            f"presentation feature bits; missing 0x{missing_features:08X}"
        )

    flags = int(monitor.actuator["flags"])
    required_flags = (
        ACTUATOR_FLAG_BUILD_ENABLED
        | ACTUATOR_FLAG_BENCH_ENABLED
        | ACTUATOR_FLAG_DRIVER_OK
        | ACTUATOR_FLAG_CONFIG_VALID
    )
    missing_flags = required_flags & ~flags
    if missing_flags:
        raise ReplayError(
            "actuator is not ready for software HOME declaration; "
            f"missing status flags 0x{missing_flags:02X}"
        )

    unsafe_flags = (
        ACTUATOR_FLAG_DRIVER_ENABLED
        | ACTUATOR_FLAG_ESTOP
        | ACTUATOR_FLAG_MANUAL_PENDING
    )
    if flags & unsafe_flags:
        raise ReplayError(
            f"actuator is not in an energy-off preflight state: 0x{flags:02X}"
        )
    if int(monitor.actuator.get("machine_state", 0)) in (
        ACTUATOR_STATE_FAULT,
        ACTUATOR_STATE_ESTOP,
    ):
        raise ReplayError(f"actuator reports fault state: {monitor.actuator}")
    reserved = int(monitor.actuator.get("reserved", 0))
    if not reserved & ACTUATOR_STATUS_CONTINUOUS_HIL_PROFILE:
        raise ReplayError("actuator status does not identify the CONTINUOUS_HIL image")
    if (
        require_geometry
        and not reserved & ACTUATOR_STATUS_GEOMETRY_PLAUSIBLE
    ):
        raise ReplayError("software actuator geometry is not plausible")
    if (
        reserved & ACTUATOR_STATUS_SOFTWARE_HOME_ACTIVE
        and reserved & ACTUATOR_STATUS_SOFTWARE_FULL_ACTIVE
    ):
        raise ReplayError(
            "software HOME and software FULL cannot both be active"
        )


def _actuator_ready_for_flight(monitor: PacketMonitor) -> bool:
    """Return whether post-HOME status satisfies all automatic-motion gates."""

    try:
        _enforce_actuator_motion(monitor)
    except ReplayError:
        return False
    assert monitor.actuator is not None
    reserved = int(monitor.actuator.get("reserved", 0))
    return (
        (int(monitor.actuator["flags"]) & ACTUATOR_FLAG_HOMED) != 0
        and (reserved & ACTUATOR_STATUS_SOFTWARE_HOME_ACTIVE) != 0
        and (reserved & ACTUATOR_STATUS_SOFTWARE_FULL_ACTIVE) == 0
        and (reserved & ACTUATOR_STATUS_GEOMETRY_PLAUSIBLE) != 0
        and abs(
            int(monitor.actuator.get("target_steps", 0))
            - int(monitor.actuator.get("actual_steps", 0))
        ) <= ACTUATOR_POSITION_TOLERANCE_STEPS
    )


def _enforce_continuous_hil(
    monitor: PacketMonitor,
    *,
    require_geometry: bool = True,
) -> None:
    """Require the HIL image, energy off, and optionally declared geometry."""

    if monitor.heartbeat is None or monitor.actuator is None:
        raise ReplayError("did not receive heartbeat and actuator status during preflight")
    features = int(monitor.heartbeat["feature_flags"])
    required_features = (
        FEATURE_USB_PROTOCOL
        | FEATURE_SIMULATION
        | FEATURE_ACTUATOR
        | FEATURE_CONTINUOUS_HIL
    )
    missing_features = required_features & ~features
    if missing_features:
        raise ReplayError(
            "continuous HIL requires the USB, simulation, actuator, and HIL "
            f"profile bits; missing 0x{missing_features:08X}"
        )

    status = monitor.actuator
    flags = int(status["flags"])
    required_flags = (
        ACTUATOR_FLAG_BUILD_ENABLED
        | ACTUATOR_FLAG_DRIVER_OK
        | ACTUATOR_FLAG_CONFIG_VALID
    )
    missing_flags = required_flags & ~flags
    if missing_flags:
        raise ReplayError(
            f"continuous-HIL actuator preflight is missing flags 0x{missing_flags:02X}"
        )
    if flags & (
        ACTUATOR_FLAG_DRIVER_ENABLED
        | ACTUATOR_FLAG_ESTOP
        | ACTUATOR_FLAG_MANUAL_PENDING
    ):
        raise ReplayError(
            f"continuous-HIL actuator is not energy-off and idle: 0x{flags:02X}"
        )
    if int(status.get("machine_state", 0)) in (
        ACTUATOR_STATE_FAULT,
        ACTUATOR_STATE_ESTOP,
    ):
        raise ReplayError(f"actuator reports fault state: {status}")
    if int(status.get("driver_status", 0)) & TMC_DRV_STATUS_STOP_MASK:
        raise ReplayError(
            "TMC5240 reports a short/thermal warning: "
            f"0x{int(status['driver_status']):08X}"
        )
    reserved = int(status.get("reserved", 0))
    if not reserved & ACTUATOR_STATUS_CONTINUOUS_HIL_PROFILE:
        raise ReplayError("actuator status does not identify the CONTINUOUS_HIL image")
    if (
        require_geometry
        and not reserved & ACTUATOR_STATUS_GEOMETRY_PLAUSIBLE
    ):
        raise ReplayError("software actuator geometry is not plausible")
    if (
        reserved & ACTUATOR_STATUS_SOFTWARE_HOME_ACTIVE
        and reserved & ACTUATOR_STATUS_SOFTWARE_FULL_ACTIVE
    ):
        raise ReplayError(
            "software HOME and software FULL cannot both be active"
        )


def _continuous_hil_ready_for_flight(monitor: PacketMonitor) -> bool:
    """Return whether the declared software zero remains ready and energy-off."""

    try:
        _enforce_continuous_hil(monitor)
    except ReplayError:
        return False
    assert monitor.actuator is not None
    status = monitor.actuator
    reserved = int(status.get("reserved", 0))
    return (
        (int(status["flags"]) & ACTUATOR_FLAG_HOMED) != 0
        and (reserved & ACTUATOR_STATUS_SOFTWARE_HOME_ACTIVE) != 0
        and (reserved & ACTUATOR_STATUS_SOFTWARE_FULL_ACTIVE) == 0
        and abs(int(status.get("target_steps", 0)))
        <= ACTUATOR_POSITION_TOLERANCE_STEPS
        and abs(int(status.get("actual_steps", 0)))
        <= ACTUATOR_POSITION_TOLERANCE_STEPS
    )


def _assert_motion_runtime_healthy(
    monitor: PacketMonitor,
    *,
    continuous_hil: bool = False,
) -> None:
    """Reject stale status or a newly unsafe actuator state during motion."""

    if monitor.actuator is None:
        raise ReplayError("actuator status was lost during motion replay")
    now = monitor.clock.now()
    if (
        monitor.actuator_received_s is None
        or now - monitor.actuator_received_s > 0.35
    ):
        raise ReplayError("actuator status became stale during motion replay")
    if (
        monitor.telemetry_received_s is None
        or now - monitor.telemetry_received_s > 0.35
    ):
        raise ReplayError("flight telemetry became stale during motion replay")
    flags = int(monitor.actuator["flags"])
    required = (
        ACTUATOR_FLAG_BUILD_ENABLED
        | ACTUATOR_FLAG_HOMED
        | ACTUATOR_FLAG_DRIVER_OK
        | ACTUATOR_FLAG_CONFIG_VALID
    )
    if not continuous_hil:
        required |= ACTUATOR_FLAG_BENCH_ENABLED
    if (flags & required) != required:
        raise ReplayError(f"actuator readiness changed during replay: 0x{flags:02X}")
    if flags & (ACTUATOR_FLAG_ESTOP | ACTUATOR_FLAG_MANUAL_PENDING):
        raise ReplayError(f"unsafe actuator state during replay: 0x{flags:02X}")
    if int(monitor.actuator.get("machine_state", 0)) in (
        ACTUATOR_STATE_FAULT,
        ACTUATOR_STATE_ESTOP,
    ):
        raise ReplayError(f"actuator faulted during replay: {monitor.actuator}")
    driver_status = int(monitor.actuator.get("driver_status", 0))
    if driver_status & TMC_DRV_STATUS_STOP_MASK:
        raise ReplayError(
            "TMC5240 short/thermal status during replay: "
            f"0x{driver_status:08X}"
        )
    if continuous_hil:
        reserved = int(monitor.actuator.get("reserved", 0))
        if not reserved & ACTUATOR_STATUS_CONTINUOUS_HIL_PROFILE:
            raise ReplayError("continuous-HIL build identity disappeared")
        if not reserved & ACTUATOR_STATUS_GEOMETRY_PLAUSIBLE:
            raise ReplayError("software actuator geometry became implausible")
        if (
            reserved & ACTUATOR_STATUS_SOFTWARE_HOME_ACTIVE
            and reserved & ACTUATOR_STATUS_SOFTWARE_FULL_ACTIVE
        ):
            raise ReplayError(
                "software HOME and software FULL became active together"
            )


def _wait_for_actuator(
    port,
    monitor: PacketMonitor,
    predicate,
    *,
    after_count: int,
    description: str,
    timeout_s: float,
) -> dict[str, int]:
    """Wait for a fresh matching actuator report and fail fast on a fault."""

    deadline = monitor.clock.now() + timeout_s
    while monitor.clock.now() < deadline:
        monitor.poll(port)
        if monitor.actuator_count > after_count and monitor.actuator is not None:
            status = monitor.actuator
            if (
                int(status.get("flags", 0)) & ACTUATOR_FLAG_ESTOP
                or int(status.get("machine_state", 0))
                in (ACTUATOR_STATE_FAULT, ACTUATOR_STATE_ESTOP)
            ):
                raise ReplayError(f"actuator faulted while waiting to {description}: {status}")
            if predicate(status):
                return status
        monitor.clock.sleep(0.005)
    raise ReplayError(f"timeout waiting for actuator to {description}")


class HilStrokeTracker:
    """Deterministic 0 -> 153600 -> 0 software-geometry state machine.

    The tracker intentionally keeps the STM32 controller's deployment request
    separate from the forced motor override. It observes XACTUAL only as
    TMC5240 ramp-generator state. The software HOME/FULL status bits are
    firmware-derived geometry flags, not switch, encoder, or independent
    evidence that the mechanism physically followed these counts.
    """

    def __init__(
        self,
        *,
        burn_time_s: float,
        post_burn_margin_s: float = 0.1,
        full_hold_s: float = 8.0,
        endpoint_timeout_s: float = 8.0,
        full_travel_steps: int = PRESENTATION_FULL_TRAVEL_STEPS,
        position_tolerance_steps: int = ACTUATOR_POSITION_TOLERANCE_STEPS,
    ) -> None:
        if burn_time_s < 0.0:
            raise ValueError("burn time must be nonnegative")
        if post_burn_margin_s < 0.0:
            raise ValueError("post-burn margin must be nonnegative")
        if full_hold_s <= 0.0 or endpoint_timeout_s <= 0.0:
            raise ValueError("HIL stroke timeouts must be positive")
        self.burn_time_s = burn_time_s
        self.post_burn_margin_s = post_burn_margin_s
        self.full_hold_s = full_hold_s
        self.endpoint_timeout_s = endpoint_timeout_s
        self.full_travel_steps = full_travel_steps
        self.position_tolerance_steps = position_tolerance_steps
        self.state = "WAITING_FOR_COAST"
        self.raw_controller_was_active = False
        self.full_command_host_s: float | None = None
        self.full_reached_host_s: float | None = None
        self.home_command_host_s: float | None = None
        self.home_reached_host_s: float | None = None
        self.full_status: dict[str, Any] | None = None
        self.home_status: dict[str, Any] | None = None

    @property
    def complete(self) -> bool:
        return self.state == "COMPLETE"

    @property
    def override_mode(self) -> int:
        if self.state in ("MOVING_FULL", "AT_FULL"):
            return HIL_OVERRIDE_FORCE_FULL
        if self.state == "MOVING_HOME":
            return HIL_OVERRIDE_FORCE_HOME
        return HIL_OVERRIDE_OFF

    def _at_full(self, status: Mapping[str, Any]) -> bool:
        reserved = int(status.get("reserved", 0))
        return (
            (reserved & ACTUATOR_STATUS_SOFTWARE_FULL_ACTIVE) != 0
            and (reserved & ACTUATOR_STATUS_SOFTWARE_HOME_ACTIVE) == 0
            and abs(
                abs(int(status.get("target_steps", 0)))
                - self.full_travel_steps
            )
            <= self.position_tolerance_steps
            and abs(
                abs(int(status.get("actual_steps", 0)))
                - self.full_travel_steps
            )
            <= self.position_tolerance_steps
        )

    def _at_home(self, status: Mapping[str, Any]) -> bool:
        reserved = int(status.get("reserved", 0))
        return (
            (reserved & ACTUATOR_STATUS_SOFTWARE_HOME_ACTIVE) != 0
            and (reserved & ACTUATOR_STATUS_SOFTWARE_FULL_ACTIVE) == 0
            and (reserved & ACTUATOR_STATUS_STROKE_SEQUENCE_VERIFIED) != 0
            and abs(int(status.get("target_steps", 0)))
            <= self.position_tolerance_steps
            and abs(int(status.get("actual_steps", 0)))
            <= self.position_tolerance_steps
        )

    def observe(
        self,
        *,
        source_time_s: float,
        telemetry: Mapping[str, Any] | None,
        actuator: Mapping[str, Any] | None,
        host_now_s: float,
    ) -> int | None:
        """Advance from fresh board status and return a newly requested mode."""

        if telemetry is not None:
            raw_percent = int(telemetry.get("deployment_percent", 0))
            if raw_percent > 0:
                self.raw_controller_was_active = True
            phase = int(telemetry.get("state", -1))
        else:
            raw_percent = 0
            phase = -1

        if self.state == "WAITING_FOR_COAST":
            if (
                source_time_s >= self.burn_time_s + self.post_burn_margin_s
                and phase in (2, 3)
            ):
                self.state = "MOVING_FULL"
                self.full_command_host_s = host_now_s
                return HIL_OVERRIDE_FORCE_FULL
            return None

        if self.state == "MOVING_FULL":
            assert self.full_command_host_s is not None
            if actuator is not None and self._at_full(actuator):
                self.state = "AT_FULL"
                self.full_reached_host_s = host_now_s
                self.full_status = dict(actuator)
                return None
            if host_now_s - self.full_command_host_s > self.endpoint_timeout_s:
                raise ReplayError(
                    "software FULL/XACTUAL target was not reached before "
                    "the travel timeout"
                )
            return None

        if self.state == "AT_FULL":
            assert self.full_reached_host_s is not None
            recovery = phase == 4
            raw_retracted = self.raw_controller_was_active and raw_percent == 0
            hold_expired = (
                host_now_s - self.full_reached_host_s >= self.full_hold_s
            )
            if recovery or raw_retracted or hold_expired:
                self.state = "MOVING_HOME"
                self.home_command_host_s = host_now_s
                return HIL_OVERRIDE_FORCE_HOME
            return None

        if self.state == "MOVING_HOME":
            assert self.home_command_host_s is not None
            if actuator is not None and self._at_home(actuator):
                self.state = "COMPLETE"
                self.home_reached_host_s = host_now_s
                self.home_status = dict(actuator)
                return None
            if host_now_s - self.home_command_host_s > self.endpoint_timeout_s:
                raise ReplayError(
                    "software HOME/XACTUAL zero was not reached before "
                    "the travel timeout"
                )
            return None

        return None

    def require_home(self, host_now_s: float) -> int | None:
        """Request HOME at the replay boundary or reject an incomplete stroke."""

        if self.state == "AT_FULL":
            self.state = "MOVING_HOME"
            self.home_command_host_s = host_now_s
            return HIL_OVERRIDE_FORCE_HOME
        if self.state in ("MOVING_FULL", "MOVING_HOME") or self.complete:
            return None
        raise ReplayError(
            "trajectory ended before software FULL/XACTUAL was observed"
        )

    def metrics(self) -> dict[str, Any]:
        open_time = (
            self.full_reached_host_s - self.full_command_host_s
            if self.full_reached_host_s is not None
            and self.full_command_host_s is not None
            else None
        )
        close_time = (
            self.home_reached_host_s - self.home_command_host_s
            if self.home_reached_host_s is not None
            and self.home_command_host_s is not None
            else None
        )
        return {
            "stroke_state": self.state,
            "full_command_host_s": self.full_command_host_s,
            "full_reached_host_s": self.full_reached_host_s,
            "home_command_host_s": self.home_command_host_s,
            "home_reached_host_s": self.home_reached_host_s,
            "open_time_s": open_time,
            "close_time_s": close_time,
            "raw_controller_was_active": self.raw_controller_was_active,
            "full_status": self.full_status,
            "home_status": self.home_status,
        }


def _emit_sample_observed(
    observer: ReplayRunObserver,
    *,
    sample_index: int,
    sample: ReplaySample,
    schedule_lag_s: float,
    monitor: PacketMonitor,
    hil_stroke: HilStrokeTracker | None,
    metadata: Mapping[str, Any] | None,
    skipped_host_samples: int,
) -> None:
    """Publish one chart-ready row without conflating controller and override."""

    telemetry = dict(monitor.telemetry or {})
    actuator = dict(monitor.actuator or {})
    sil = dict(metadata or {})
    override_mode = (
        hil_stroke.override_mode if hil_stroke is not None else HIL_OVERRIDE_OFF
    )
    forced_fraction: float | None
    if override_mode == HIL_OVERRIDE_FORCE_FULL:
        forced_fraction = 1.0
    elif override_mode == HIL_OVERRIDE_FORCE_HOME:
        forced_fraction = 0.0
    else:
        forced_fraction = None
    target_steps = (
        int(actuator["target_steps"]) if "target_steps" in actuator else None
    )
    actual_steps = (
        int(actuator["actual_steps"]) if "actual_steps" in actuator else None
    )
    observer.emit(
        "sample_observed",
        sample_index=sample_index,
        replay_time_s=sample.replay_time_s,
        source_time_s=sample.source_time_s,
        schedule_lag_s=schedule_lag_s,
        skipped_host_samples=skipped_host_samples,
        truth_altitude_m=sample.altitude_m,
        truth_velocity_mps=sample.vertical_velocity_mps,
        truth_acceleration_mps2=sample.vertical_acceleration_mps2,
        sil=sil,
        stm32_altitude_m=telemetry.get("altitude_m"),
        stm32_velocity_mps=telemetry.get("velocity_mps"),
        stm32_predicted_apogee_m=telemetry.get("predicted_apogee_m"),
        target_apogee_m=telemetry.get("target_apogee_m"),
        raw_controller_deployment_fraction=(
            float(telemetry["deployment_percent"]) / 100.0
            if "deployment_percent" in telemetry
            else None
        ),
        forced_hil_deployment_fraction=forced_fraction,
        hil_override_mode=override_mode,
        motor_target_steps=target_steps,
        motor_actual_steps=actual_steps,
        motor_tracking_error_steps=(
            target_steps - actual_steps
            if target_steps is not None and actual_steps is not None
            else None
        ),
        software_home_active=actuator.get(
            "software_home_active",
            actuator.get("home_active"),
        ),
        software_full_active=actuator.get(
            "software_full_active",
            actuator.get("full_active"),
        ),
        geometry_plausible=actuator.get(
            "geometry_plausible",
            actuator.get("limits_plausible"),
        ),
        stroke_sequence_verified=actuator.get(
            "stroke_sequence_verified",
            actuator.get("endpoint_sequence_verified"),
        ),
        # Keep the protocol-v2 evidence column names for existing databases.
        home_active=actuator.get("home_active"),
        full_active=actuator.get("full_active"),
        limits_plausible=actuator.get("limits_plausible"),
        endpoint_sequence_verified=actuator.get(
            "endpoint_sequence_verified"
        ),
        stm32_phase=telemetry.get("state"),
        stm32_phase_name=PHASE_NAMES.get(
            int(telemetry["state"])
            if "state" in telemetry
            else -1,
            "UNKNOWN",
        ),
        actuator_machine_state=actuator.get("machine_state"),
        actuator_flags=actuator.get("flags"),
        actuator_inhibit_flags=actuator.get("actuator_inhibit_flags"),
        driver_status=actuator.get("driver_status"),
        telemetry_packet_time_ms=monitor.telemetry_packet_time_ms,
        actuator_packet_time_ms=monitor.actuator_packet_time_ms,
    )


def _print_progress(
    replay_time_s: float,
    monitor: PacketMonitor,
    *,
    rotations: float,
    full_steps_per_revolution: int,
    microsteps: int,
    gear_ratio: float,
    actuator_motion: bool,
) -> None:
    """Render one compact operator line from the latest decoded board state."""

    telemetry = monitor.telemetry
    if telemetry is None:
        print(f"SIM {replay_time_s:6.2f}s | waiting for telemetry")
        return
    deployment = float(telemetry["deployment_percent"])
    intended_rotations, counts = prototype_counts(
        deployment,
        rotations=rotations,
        full_steps_per_revolution=full_steps_per_revolution,
        microsteps=microsteps,
        gear_ratio=gear_ratio,
    )
    if (
        actuator_motion
        and monitor.heartbeat is not None
        and int(monitor.heartbeat["feature_flags"]) & FEATURE_DIRECTION_INVERTED
    ):
        counts = -counts
    phase = PHASE_NAMES.get(int(telemetry["state"]), f"STATE_{telemetry['state']}")
    motor_text = "MOTOR INHIBITED"
    if actuator_motion:
        if monitor.actuator is None:
            motor_text = "MOTOR STATUS WAIT"
        else:
            target = int(monitor.actuator["target_steps"])
            actual = int(monitor.actuator["actual_steps"])
            enabled = (int(monitor.actuator["flags"]) & ACTUATOR_FLAG_DRIVER_ENABLED) != 0
            motor_text = (
                f"MOTOR {'MOVING' if enabled else 'READY'} "
                f"target={target} actual={actual}"
            )
    print(
        f"SIM {replay_time_s:6.2f}s | {phase:15} | "
        f"Alt {float(telemetry['altitude_m']):7.1f} m | "
        f"Vel {float(telemetry['velocity_mps']):7.1f} m/s | "
        f"Pred {float(telemetry['predicted_apogee_m']):7.1f} m | "
        f"Deploy {deployment:3.0f}% | {intended_rotations:4.2f} rev / {counts} counts | "
        f"{motor_text}"
    )


# ---------------------------------------------------------------------------
# Machine-readable presentation acceptance [ARCH-8]
# ---------------------------------------------------------------------------


PRESENTATION_PHASE_ORDER = (0, 1, 2, 3, 4)
# A small number of isolated Windows scheduling misses is tolerable, but losing
# more than 2% no longer represents the selected trajectory with useful fidelity.
MAX_HOST_SKIP_RATIO = 0.02


def _wait_for_replay_deadline(
    clock: HostClock,
    poll: Callable[[], None],
    *,
    start_s: float,
    replay_time_s: float,
    rate_hz: float,
    continuous_hil: bool,
) -> ReplayDeadlineResult:
    """Wait for one sample and classify a bounded host scheduling miss.

    Finite SIL replay preserves its absolute timeline by skipping a row that
    is more than half a period late. Continuous HIL must never skip a row, so
    a tolerated miss instead rebases later deadlines by the complete lag. This
    preserves nominal spacing after the interruption and prevents a catch-up
    USB burst. Both modes still fail closed above the 100 ms deadline ceiling.
    """

    deadline = start_s + replay_time_s
    now = clock.now()
    while now < deadline:
        poll()
        now = clock.now()
        if now < deadline:
            clock.sleep(min(0.002, deadline - now))
            now = clock.now()

    lag = now - deadline
    if lag > MAX_HOST_SCHEDULE_LAG_S:
        raise ReplayDeadlineError(lag)

    late_skip_threshold_s = 0.5 / rate_hz
    if continuous_hil:
        return ReplayDeadlineResult(
            lag_s=lag,
            skip=False,
            rebase_s=lag if lag > late_skip_threshold_s else 0.0,
        )
    return ReplayDeadlineResult(
        lag_s=lag,
        skip=lag > late_skip_threshold_s,
        rebase_s=0.0,
    )


def _contains_ordered(values: Sequence[int], required: Sequence[int]) -> bool:
    """Return true when every required value appears in order without rewinding."""

    next_required = 0
    for value in values:
        if next_required < len(required) and value == required[next_required]:
            next_required += 1
    return next_required == len(required)


def _finite_or_none(value: float) -> float | None:
    """Represent missing extrema as JSON null rather than nonstandard infinity."""

    return value if math.isfinite(value) else None


def _acceptance_check(
    passed: bool,
    *,
    expected: Any,
    actual: Any,
    applicable: bool = True,
    note: str | None = None,
) -> dict[str, Any]:
    """Create one uniform verdict check, including intentionally skipped checks."""

    result = {
        "applicable": applicable,
        "passed": bool(passed) if applicable else True,
        "expected": expected,
        "actual": actual,
    }
    if note is not None:
        result["note"] = note
    return result


def _actuator_energy_is_off(status: Mapping[str, Any]) -> bool:
    """Return true only for explicit driver-off and HIL-override-off evidence."""

    return (
        int(status.get("flags", 0)) & ACTUATOR_FLAG_DRIVER_ENABLED
    ) == 0 and (
        int(status.get("reserved", 0))
        & ACTUATOR_STATUS_HIL_OVERRIDE_ACTIVE
    ) == 0


def _perform_safety_cleanup(
    port,
    monitor: PacketMonitor,
    sequence: SequenceCounter,
    observer: ReplayRunObserver,
    clock: HostClock,
    *,
    continuous_hil: bool,
    timeout_s: float = 1.5,
) -> dict[str, Any]:
    """Stop in place and collect fresh post-cleanup state without retracting.

    Evidence callbacks are deliberately best effort in this path: a disk or
    dashboard failure must not prevent the override-off, DISARM, and SIM_STOP
    writes. A fresh actuator packet is required before motor energy is reported
    as verified off.
    """

    commands: list[tuple[int, bytes]] = []
    if continuous_hil:
        commands.append(
            (CMD_HIL_SET_OVERRIDE, bytes((HIL_OVERRIDE_OFF,)))
        )
    commands.extend(
        (
            (CMD_SET_ARMED, b"\x00"),
            (CMD_SIM_STOP, b""),
        )
    )
    before_count = monitor.actuator_count
    before_actuator = (
        dict(monitor.actuator) if monitor.actuator is not None else None
    )
    command_results: list[dict[str, Any]] = []
    observation_errors: list[dict[str, str]] = []

    try:
        observer.emit(
            "safety_cleanup",
            commands=[command for command, _data in commands],
            retract_attempted=False,
        )
    except Exception as error:
        observation_errors.append(
            {
                "stage": "cleanup_start_event",
                "type": type(error).__name__,
                "message": str(error),
            }
        )

    for command, data in commands:
        result: dict[str, Any] = {
            "command": command,
            "sequence": None,
            "write_completed": False,
            "acknowledged": False,
        }
        try:
            command_sequence = _send_command(
                port,
                sequence,
                command,
                data,
                clock=clock,
            )
            result["sequence"] = command_sequence
            result["write_completed"] = True
        except Exception as error:
            result["error_type"] = type(error).__name__
            result["error"] = str(error)
        command_results.append(result)

    post_cleanup_actuator: dict[str, Any] | None = None
    poll_error: dict[str, str] | None = None
    deadline = clock.now() + timeout_s
    while clock.now() < deadline:
        try:
            monitor.poll(port)
        except Exception as error:
            poll_error = {
                "type": type(error).__name__,
                "message": str(error),
            }
        if monitor.actuator_count > before_count and monitor.actuator is not None:
            candidate = dict(monitor.actuator)
            if _actuator_energy_is_off(candidate):
                post_cleanup_actuator = candidate
                break
        clock.sleep(0.005)

    for result in command_results:
        command_sequence = result.get("sequence")
        if command_sequence is None:
            continue
        ack = monitor.acks.get(int(command_sequence))
        result["acknowledged"] = bool(
            ack is not None
            and int(ack.get("command", -1)) == int(result["command"])
            and int(ack.get("result", -1)) == ACK_OK
        )

    cleanup = {
        "attempted": True,
        "retract_attempted": False,
        "policy": "stop_in_place_no_automatic_home_after_fault",
        "pre_cleanup_actuator": before_actuator,
        "fresh_actuator_status_received": post_cleanup_actuator is not None,
        "motor_energy_off_verified": post_cleanup_actuator is not None,
        "post_cleanup_actuator": post_cleanup_actuator,
        "command_results": command_results,
        "poll_error": poll_error,
        "observation_errors": observation_errors,
    }
    try:
        observer.emit("safety_cleanup_result", **cleanup)
    except Exception:
        # Safety writes and the in-memory cleanup result already exist. The
        # original evidence-write failure is allowed to propagate later.
        pass
    return cleanup


def _build_live_verdict(
    monitor: PacketMonitor,
    *,
    completed: bool,
    failure: BaseException | None,
    no_arm: bool,
    allow_actuator_motion: bool,
    max_schedule_lag_s: float,
    skipped_host_samples: int,
    sent_host_samples: int,
    continuous_hil: bool = False,
    hil_stroke: HilStrokeTracker | None = None,
    safety_cleanup: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Evaluate the presentation checks and preserve the evidence behind each.

    No-arm transport checks intentionally mark flight-phase and deployment checks
    not applicable. Motion acceptance uses XACTUAL as internal driver evidence;
    it is not represented as encoder-verified physical travel.
    """

    initial_heartbeat = monitor.initial_heartbeat or monitor.heartbeat
    final_heartbeat = monitor.heartbeat
    if initial_heartbeat is not None and final_heartbeat is not None:
        receive_error_delta: int | None = (
            int(final_heartbeat["receive_errors"])
            - int(initial_heartbeat["receive_errors"])
        )
        transmit_drop_delta: int | None = (
            int(final_heartbeat["transmit_drops"])
            - int(initial_heartbeat["transmit_drops"])
        )
    else:
        receive_error_delta = None
        transmit_drop_delta = None

    cleanup_attempted = bool(
        safety_cleanup is not None and safety_cleanup.get("attempted")
    )
    if cleanup_attempted:
        cleanup_actuator = safety_cleanup.get("post_cleanup_actuator")
        final_actuator = (
            dict(cleanup_actuator)
            if isinstance(cleanup_actuator, Mapping)
            else None
        )
    else:
        final_actuator = (
            dict(monitor.actuator) if monitor.actuator is not None else None
        )
    final_flags = int(final_actuator["flags"]) if final_actuator is not None else 0
    final_target = (
        int(final_actuator["target_steps"]) if final_actuator is not None else None
    )
    final_actual = (
        int(final_actuator["actual_steps"]) if final_actuator is not None else None
    )

    flight_checks_apply = not no_arm
    home_passed = (
        final_actuator is not None
        and (final_flags & ACTUATOR_FLAG_HOMED) != 0
        and final_target is not None
        and abs(final_target) <= ACTUATOR_POSITION_TOLERANCE_STEPS
        and final_actual is not None
        and abs(final_actual) <= ACTUATOR_POSITION_TOLERANCE_STEPS
    )
    driver_off_passed = (
        final_actuator is not None
        and (final_flags & ACTUATOR_FLAG_DRIVER_ENABLED) == 0
    )
    final_reserved = (
        int(final_actuator.get("reserved", 0))
        if final_actuator is not None
        else 0
    )
    hil_metrics = hil_stroke.metrics() if hil_stroke is not None else {}
    hil_full_passed = bool(
        hil_stroke is not None and hil_stroke.full_reached_host_s is not None
    )
    hil_home_passed = bool(
        hil_stroke is not None
        and hil_stroke.complete
        and final_reserved & ACTUATOR_STATUS_SOFTWARE_HOME_ACTIVE
        and final_reserved & ACTUATOR_STATUS_STROKE_SEQUENCE_VERIFIED
    )
    heartbeat_passed = (
        receive_error_delta == 0 and transmit_drop_delta == 0
    )
    scheduled_host_samples = sent_host_samples + skipped_host_samples
    skip_ratio = (
        skipped_host_samples / scheduled_host_samples
        if scheduled_host_samples > 0
        else None
    )

    checks = {
        "run_completed": _acceptance_check(
            completed,
            expected=True,
            actual=completed,
        ),
        "phase_order": _acceptance_check(
            _contains_ordered(monitor.phase_order, PRESENTATION_PHASE_ORDER),
            expected=[PHASE_NAMES[value] for value in PRESENTATION_PHASE_ORDER],
            actual=[PHASE_NAMES.get(value, str(value)) for value in monitor.phase_order],
            applicable=flight_checks_apply,
        ),
        "nonzero_deployment": _acceptance_check(
            monitor.max_deployment_percent > 0,
            expected="> 0 percent",
            actual=monitor.max_deployment_percent,
            applicable=flight_checks_apply,
        ),
        "final_home_position": _acceptance_check(
            home_passed,
            expected={
                "homed": True,
                "target_abs_max_steps": ACTUATOR_POSITION_TOLERANCE_STEPS,
                "actual_abs_max_steps": ACTUATOR_POSITION_TOLERANCE_STEPS,
            },
            actual=final_actuator,
            applicable=allow_actuator_motion,
            note="XACTUAL is internal TMC5240 position, not encoder feedback",
        ),
        "final_driver_off": _acceptance_check(
            driver_off_passed,
            expected=True,
            actual=(
                (final_flags & ACTUATOR_FLAG_DRIVER_ENABLED) == 0
                if final_actuator is not None
                else None
            ),
        ),
        "fault_cleanup_energy_off": _acceptance_check(
            bool(
                safety_cleanup is not None
                and safety_cleanup.get("motor_energy_off_verified")
            ),
            expected=True,
            actual=(
                safety_cleanup.get("motor_energy_off_verified")
                if safety_cleanup is not None
                else None
            ),
            applicable=cleanup_attempted,
            note=(
                "fault cleanup stops in place; it never performs a blind "
                "automatic retract or HOME"
            ),
        ),
        "hil_full_ramp_state": _acceptance_check(
            hil_full_passed,
            expected={
                "software_full": True,
                "evidence_boundary": (
                    "target and XACTUAL are internal TMC5240 ramp-generator "
                    "state, not endstop or encoder proof"
                ),
                "full_travel_steps": PRESENTATION_FULL_TRAVEL_STEPS,
            },
            actual=hil_metrics.get("full_status"),
            applicable=continuous_hil,
        ),
        "hil_home_ramp_sequence": _acceptance_check(
            hil_home_passed,
            expected={
                "software_home": True,
                "stroke_sequence_verified": True,
                "position_steps": 0,
                "evidence_boundary": (
                    "software geometry only; no independent mechanical "
                    "position measurement"
                ),
            },
            actual=hil_metrics.get("home_status"),
            applicable=continuous_hil,
        ),
        "decoder_errors": _acceptance_check(
            monitor.decoder.errors == 0,
            expected=0,
            actual=monitor.decoder.errors,
        ),
        "heartbeat_error_drop_deltas": _acceptance_check(
            heartbeat_passed,
            expected={"receive_errors": 0, "transmit_drops": 0},
            actual={
                "receive_errors": receive_error_delta,
                "transmit_drops": transmit_drop_delta,
            },
        ),
        "host_deadline": _acceptance_check(
            max_schedule_lag_s <= MAX_HOST_SCHEDULE_LAG_S,
            expected=f"<= {MAX_HOST_SCHEDULE_LAG_S:.3f} seconds",
            actual=max_schedule_lag_s,
        ),
        "host_skip_ratio": _acceptance_check(
            skip_ratio is not None
            and (
                skipped_host_samples == 0
                if continuous_hil
                else skip_ratio <= MAX_HOST_SKIP_RATIO
            ),
            expected=(
                "0 skipped samples"
                if continuous_hil
                else f"<= {MAX_HOST_SKIP_RATIO:.3f}"
            ),
            actual=skip_ratio,
            note=(
                "ratio is skipped samples divided by sent plus skipped samples; "
                "a zero-sample run fails"
            ),
        ),
    }
    passed = all(
        bool(check["passed"])
        for check in checks.values()
        if bool(check["applicable"])
    )
    return {
        "schema": "ambar.live_replay_verdict.v1",
        "status": "PASS" if passed else "FAIL",
        "passed": passed,
        "failure": (
            None
            if failure is None
            else {"type": type(failure).__name__, "message": str(failure)}
        ),
        "checks": checks,
        "metrics": {
            "phase_order": list(monitor.phase_order),
            "max_altitude_m": _finite_or_none(monitor.max_altitude_m),
            "max_predicted_apogee_m": _finite_or_none(
                monitor.max_predicted_apogee_m
            ),
            "max_deployment_percent": monitor.max_deployment_percent,
            "max_abs_actuator_target_steps": monitor.max_abs_actuator_target_steps,
            "max_abs_actuator_actual_steps": monitor.max_abs_actuator_actual_steps,
            "max_actuator_tracking_error_steps": (
                monitor.max_actuator_tracking_error_steps
            ),
            "max_schedule_lag_s": max_schedule_lag_s,
            "skipped_host_samples": skipped_host_samples,
            "sent_host_samples": sent_host_samples,
            "scheduled_host_samples": scheduled_host_samples,
            "host_skip_ratio": skip_ratio,
            "telemetry_packets": monitor.telemetry_count,
            "actuator_packets": monitor.actuator_count,
            "event_packets": len(monitor.events),
            "continuous_hil": continuous_hil,
            **hil_metrics,
        },
        "transport": {
            "initial_heartbeat": initial_heartbeat,
            "final_heartbeat": final_heartbeat,
            "receive_error_delta": receive_error_delta,
            "transmit_drop_delta": transmit_drop_delta,
            "decoder_errors": monitor.decoder.errors,
        },
        "final_actuator": final_actuator,
        "safety_cleanup": (
            dict(safety_cleanup)
            if safety_cleanup is not None
            else {
                "attempted": False,
                "retract_attempted": False,
                "policy": "normal_verified_shutdown",
            }
        ),
    }


# ---------------------------------------------------------------------------
# Guarded live replay, safety cleanup, and evidence finalization
# ---------------------------------------------------------------------------


def run_continuous_hil_preflight(
    *,
    port_name: str | None,
    accept_current_position_home: bool = False,
    clock: HostClock | None = None,
    home_timeout_s: float = 10.0,
) -> dict[str, Any]:
    """Verify the HIL image and declare one operator-confirmed software HOME.

    This bounded pass owns the serial port only for preflight. It checks
    communications, build identity, driver state, and software geometry before
    issuing ``CMD_HOME``. That command must only zero the current TMC5240 ramp
    position; it must not seek or move the motor. The explicit acknowledgement
    means the operator has manually placed the mechanism fully closed.

    This declaration happens once per supervisor process. Later replay cycles
    only verify that HOMED and XACTUAL/target near zero remain present. A
    restart, resume, board reboot, or lost HOMED state requires a fresh
    acknowledgement and a new declaration rather than an automatic re-zero.
    """

    if home_timeout_s <= 0.0:
        raise ReplayError("HOME preflight timeout must be positive")
    if not accept_current_position_home:
        raise ReplayError(
            "software HOME requires explicit confirmation that the mechanism "
            "is manually fully closed; no serial port was opened"
        )

    active_clock = clock if clock is not None else PerfCounterClock()
    started_host_s = active_clock.now()
    sequence = SequenceCounter()
    monitor = PacketMonitor(clock=active_clock)
    serial, list_ports = _require_serial()
    selected_port = _choose_port(list_ports, port_name)
    cleanup_required = True
    stages: list[dict[str, Any]] = []

    def record_stage(name: str) -> None:
        stages.append(
            {
                "stage": name,
                "host_elapsed_s": round(active_clock.now() - started_host_s, 9),
                "heartbeat": (
                    dict(monitor.heartbeat)
                    if monitor.heartbeat is not None
                    else None
                ),
                "actuator": (
                    dict(monitor.actuator)
                    if monitor.actuator is not None
                    else None
                ),
            }
        )

    with serial.Serial(
        selected_port,
        115200,
        timeout=0.0,
        write_timeout=1.0,
    ) as port:
        port.reset_input_buffer()
        try:
            ping_sequence = _send_command(
                port,
                sequence,
                CMD_PING,
                clock=active_clock,
            )
            _wait_for_ack(port, monitor, ping_sequence, CMD_PING)
            snapshot_sequence = _send_command(
                port,
                sequence,
                CMD_REQUEST_SNAPSHOT,
                clock=active_clock,
            )
            _wait_for_ack(
                port,
                monitor,
                snapshot_sequence,
                CMD_REQUEST_SNAPSHOT,
            )
            _collect_preflight_status(port, monitor)

            # No motion command is sent until every static safety gate passes.
            _enforce_continuous_hil(monitor, require_geometry=False)
            record_stage("static_gates_passed")

            actuator_count_before_stop = monitor.actuator_count
            disarm_sequence = _send_command(
                port,
                sequence,
                CMD_SET_ARMED,
                b"\x00",
                clock=active_clock,
            )
            _wait_for_ack(
                port,
                monitor,
                disarm_sequence,
                CMD_SET_ARMED,
            )
            stop_sequence = _send_command(
                port,
                sequence,
                CMD_SIM_STOP,
                clock=active_clock,
            )
            _wait_for_ack(port, monitor, stop_sequence, CMD_SIM_STOP)
            _wait_for_actuator(
                port,
                monitor,
                lambda status: (
                    int(status["flags"])
                    & (
                        ACTUATOR_FLAG_DRIVER_ENABLED
                        | ACTUATOR_FLAG_MANUAL_PENDING
                    )
                )
                == 0
                and (
                    int(status.get("reserved", 0))
                    & ACTUATOR_STATUS_HIL_OVERRIDE_ACTIVE
                )
                == 0,
                after_count=actuator_count_before_stop,
                description="confirm an energy-off preflight state",
                timeout_s=1.5,
            )
            _enforce_continuous_hil(monitor, require_geometry=False)
            record_stage("energy_off_confirmed")

            pre_zero_actuator = dict(monitor.actuator or {})
            actuator_count_before_home = monitor.actuator_count
            home_sequence = _send_command(
                port,
                sequence,
                CMD_HOME,
                clock=active_clock,
            )
            _wait_for_ack(port, monitor, home_sequence, CMD_HOME)
            _wait_for_actuator(
                port,
                monitor,
                lambda status: (
                    int(status["flags"]) & ACTUATOR_FLAG_HOMED
                )
                != 0
                and (
                    int(status["flags"]) & ACTUATOR_FLAG_DRIVER_ENABLED
                )
                == 0
                and (
                    int(status.get("reserved", 0))
                    & ACTUATOR_STATUS_SOFTWARE_HOME_ACTIVE
                )
                != 0
                and (
                    int(status.get("reserved", 0))
                    & ACTUATOR_STATUS_SOFTWARE_FULL_ACTIVE
                )
                == 0,
                after_count=actuator_count_before_home,
                description="declare the current fully closed position as zero",
                timeout_s=home_timeout_s,
            )
            if not _continuous_hil_ready_for_flight(monitor):
                raise ReplayError(
                    "software HOME completed without a zero reference and "
                    "energy-off actuator state"
                )
            record_stage("software_home_declared")

            actuator_count_before_final_stop = monitor.actuator_count
            disarm_sequence = _send_command(
                port,
                sequence,
                CMD_SET_ARMED,
                b"\x00",
                clock=active_clock,
            )
            _wait_for_ack(
                port,
                monitor,
                disarm_sequence,
                CMD_SET_ARMED,
            )
            stop_sequence = _send_command(
                port,
                sequence,
                CMD_SIM_STOP,
                clock=active_clock,
            )
            _wait_for_ack(port, monitor, stop_sequence, CMD_SIM_STOP)
            _wait_for_actuator(
                port,
                monitor,
                lambda status: (
                    int(status["flags"])
                    & (
                        ACTUATOR_FLAG_DRIVER_ENABLED
                        | ACTUATOR_FLAG_MANUAL_PENDING
                    )
                )
                == 0
                and (
                    int(status.get("reserved", 0))
                    & ACTUATOR_STATUS_SOFTWARE_HOME_ACTIVE
                )
                != 0,
                after_count=actuator_count_before_final_stop,
                description="confirm software HOME remains energy-off",
                timeout_s=1.5,
            )
            if not _continuous_hil_ready_for_flight(monitor):
                raise ReplayError(
                    "final HIL preflight state is not software-HOME-ready"
                )
            record_stage("final_energy_off")

            cleanup_required = False
            assert monitor.heartbeat is not None
            assert monitor.actuator is not None
            return {
                "schema": "ambar.continuous_hil_preflight.v1",
                "selected_port": selected_port,
                "feature_flags": int(monitor.heartbeat["feature_flags"]),
                "heartbeat": dict(monitor.heartbeat),
                "actuator": dict(monitor.actuator),
                "stages": stages,
                "home_verified": True,
                "software_home_declared": True,
                "operator_confirmed_manually_fully_closed": True,
                "home_method": "operator-confirmed current position set to XACTUAL=0",
                "pre_zero_actuator": pre_zero_actuator,
                "position_evidence_boundary": (
                    "TMC target/XACTUAL only; no encoder or endstop proof"
                ),
                "motor_energy_off": True,
            }
        finally:
            if cleanup_required:
                try:
                    port.write(
                        command_frame(
                            sequence.take(),
                            CMD_HIL_SET_OVERRIDE,
                            bytes((HIL_OVERRIDE_OFF,)),
                            time_ms=_host_packet_time_ms(active_clock),
                        )
                    )
                    port.write(
                        command_frame(
                            sequence.take(),
                            CMD_SET_ARMED,
                            b"\x00",
                            time_ms=_host_packet_time_ms(active_clock),
                        )
                    )
                    port.write(
                        command_frame(
                            sequence.take(),
                            CMD_SIM_STOP,
                            time_ms=_host_packet_time_ms(active_clock),
                        )
                    )
                    active_clock.sleep(0.1)
                except Exception:
                    pass


def run_live_replay(
    profile: ReplayProfile,
    *,
    port_name: str | None,
    barometer_stddev_m: float,
    arm_after_s: float,
    target_apogee_m: float,
    no_arm: bool,
    rotations: float,
    full_steps_per_revolution: int,
    microsteps: int,
    gear_ratio: float,
    allow_actuator_motion: bool,
    home_at_current_position: bool,
    clock: HostClock | None = None,
    run_bundle_dir: str | Path | None = None,
    gui_udp_port: int | None = None,
    run_metadata: Mapping[str, Any] | None = None,
    continuous_hil: bool = False,
    burn_time_s: float | None = None,
    post_burn_margin_s: float = 0.1,
    full_hold_s: float = 8.0,
    endpoint_timeout_s: float = 8.0,
    sample_metadata: Sequence[Mapping[str, Any]] | None = None,
    event_sink: Callable[[Mapping[str, Any]], None] | None = None,
    event_log_name: str = "packets.jsonl",
) -> dict[str, Any]:
    """Run one guarded USB replay and return its machine-readable verdict.

    The injected clock owns all host scheduling, freshness, wait, and host-frame
    timestamp decisions. The observer is initialized before serial access so a
    requested bundle captures validation/open failures as well as successful
    runs. Safety cleanup remains inside the serial scope and therefore precedes
    evidence finalization.
    """

    active_clock = clock if clock is not None else PerfCounterClock()
    observer = ReplayRunObserver(
        clock=active_clock,
        bundle_dir=run_bundle_dir,
        gui_udp_port=gui_udp_port,
        event_sink=event_sink,
        event_log_name=event_log_name,
        record_context=run_metadata,
    )
    sequence = SequenceCounter()
    monitor = PacketMonitor(clock=active_clock, observer=observer)
    max_schedule_lag_s = 0.0
    skipped_host_samples = 0
    sent_host_samples = 0
    completed = False
    failure: BaseException | None = None
    verdict: dict[str, Any]
    hil_stroke: HilStrokeTracker | None = None
    safety_cleanup: dict[str, Any] | None = None

    observer.start(
        {
            "clock": {
                "host_scheduler": "time.perf_counter",
                "host_epoch": "arbitrary_monotonic",
                "device_packet_time_domain": "stm32_HAL_GetTick",
            },
            "connection": {
                "requested_port": port_name,
                "baud": 115200,
                "usb_vid_pid": "0483:5740",
                "serial_owner": "replay_openrocket.py",
                "gui_transport": (
                    f"udp://127.0.0.1:{gui_udp_port}"
                    if gui_udp_port is not None
                    else None
                ),
            },
            "replay": {
                "rate_hz": profile.rate_hz,
                "prepad_s": profile.prepad_s,
                "source_stop_s": profile.source_stop_s,
                "sample_count": len(profile.samples),
                "vertical_source": profile.vertical_source,
                "vertical_channels_derived": profile.uses_derived_vertical,
                "barometer_stddev_m": barometer_stddev_m,
                "arm_after_s": arm_after_s,
                "target_apogee_m": target_apogee_m,
            },
            "motion": {
                "enabled": allow_actuator_motion,
                "continuous_hil": continuous_hil,
                "home_at_current_position_acknowledged": home_at_current_position,
                "software_home_method": (
                    "operator-confirmed fully closed position declared XACTUAL=0"
                    if home_at_current_position
                    else None
                ),
                "position_evidence_boundary": (
                    "TMC target/XACTUAL internal ramp state; no encoder or endstop"
                ),
                "no_arm": no_arm,
                "full_extension_rotations": rotations,
                "full_steps_per_revolution": full_steps_per_revolution,
                "microsteps": microsteps,
                "gear_ratio": gear_ratio,
                "full_travel_steps": PRESENTATION_FULL_TRAVEL_STEPS,
            },
            "sequence_seed": sequence.value,
            "run_metadata": dict(run_metadata or {}),
        }
    )

    try:
        if not 0.1 <= barometer_stddev_m <= 100.0:
            raise ReplayError("barometer standard deviation must be within 0.1..100 m")
        if not no_arm and not 0.1 <= arm_after_s < profile.prepad_s:
            raise ReplayError("arm-after must occur during the prepad interval")
        if not 10.0 <= target_apogee_m <= 6553.5:
            raise ReplayError("target apogee must be within 10.0..6553.5 m")
        if continuous_hil and not allow_actuator_motion:
            raise ReplayError("continuous HIL requires actuator motion")
        if continuous_hil and burn_time_s is None:
            raise ReplayError("continuous HIL requires the RocketPy motor burn time")
        if allow_actuator_motion and not home_at_current_position:
            raise ReplayError(
                "motion mode requires explicit acknowledgement that the "
                "mechanism was manually placed fully closed before software "
                "HOME was declared"
            )
        if home_at_current_position and not allow_actuator_motion:
            raise ReplayError(
                "--home-at-current-position requires --allow-actuator-motion"
            )
        if allow_actuator_motion and no_arm:
            raise ReplayError("--no-arm cannot produce actuator motion")
        if allow_actuator_motion:
            validate_presentation_motion_profile(
                rotations=rotations,
                full_steps_per_revolution=full_steps_per_revolution,
                microsteps=microsteps,
                gear_ratio=gear_ratio,
            )
        if continuous_hil:
            hil_stroke = HilStrokeTracker(
                burn_time_s=float(burn_time_s),
                post_burn_margin_s=post_burn_margin_s,
                full_hold_s=full_hold_s,
                endpoint_timeout_s=endpoint_timeout_s,
            )
            if sample_metadata is not None and len(sample_metadata) != len(
                profile.samples
            ):
                raise ReplayError(
                    "sample metadata must align one-for-one with replay samples"
                )

        serial, list_ports = _require_serial()
        selected_port = _choose_port(list_ports, port_name)
        observer.emit("serial_selected", port=selected_port)
        if allow_actuator_motion:
            print(
                f"Opening {selected_port} at 115200; MOTOR MOTION ENABLED."
            )
            if continuous_hil:
                print(
                    "Software HOME was declared by the supervisor from the "
                    "operator-confirmed fully closed position. This cycle will "
                    "verify it without re-zeroing."
                )
            else:
                print(
                    "CMD_HOME will declare the manually fully closed current "
                    "position as XACTUAL=0 without motor motion."
                )
        else:
            print(
                f"Opening {selected_port} at 115200; physical motion must remain disabled."
            )

        print(f"Session sequence seed: {sequence.value}")
        cleanup_required = True

        with serial.Serial(
            selected_port,
            115200,
            timeout=0.0,
            write_timeout=1.0,
        ) as raw_port:
            raw_port.reset_input_buffer()
            port = _ObservedSerialPort(
                raw_port,
                clock=active_clock,
                observer=observer,
            )
            try:
                ping_sequence = _send_command(port, sequence, CMD_PING)
                _wait_for_ack(port, monitor, ping_sequence, CMD_PING)
                snapshot_sequence = _send_command(
                    port,
                    sequence,
                    CMD_REQUEST_SNAPSHOT,
                )
                _wait_for_ack(
                    port,
                    monitor,
                    snapshot_sequence,
                    CMD_REQUEST_SNAPSHOT,
                )
                _collect_preflight_status(port, monitor)
                observer.emit(
                    "preflight_status",
                    heartbeat=monitor.heartbeat,
                    actuator=monitor.actuator,
                )
                if not allow_actuator_motion:
                    _enforce_no_motion(monitor)

                actuator_count_before_reset = monitor.actuator_count
                disarm_sequence = _send_command(
                    port,
                    sequence,
                    CMD_SET_ARMED,
                    b"\x00",
                )
                _wait_for_ack(port, monitor, disarm_sequence, CMD_SET_ARMED)
                reset_sequence = _send_command(port, sequence, CMD_SIM_STOP)
                _wait_for_ack(port, monitor, reset_sequence, CMD_SIM_STOP)
                if allow_actuator_motion:
                    _wait_for_actuator(
                        port,
                        monitor,
                        lambda _status: True,
                        after_count=actuator_count_before_reset,
                        description="report its stopped preflight state",
                        timeout_s=1.5,
                    )
                    if continuous_hil:
                        _enforce_continuous_hil(monitor)
                    else:
                        _enforce_actuator_motion(monitor)
                    assert monitor.heartbeat is not None
                    direction = (
                        "negative"
                        if int(monitor.heartbeat["feature_flags"])
                        & FEATURE_DIRECTION_INVERTED
                        else "positive"
                    )
                    print(
                        "Firmware actuator profile: "
                        f"{PRESENTATION_FULL_TRAVEL_STEPS} counts full travel; "
                        f"extension direction is {direction}."
                    )

                    if continuous_hil:
                        ready_for_flight = _continuous_hil_ready_for_flight(
                            monitor
                        )
                    else:
                        actuator_count_before_home = monitor.actuator_count
                        home_sequence = _send_command(
                            port,
                            sequence,
                            CMD_HOME,
                        )
                        _wait_for_ack(
                            port,
                            monitor,
                            home_sequence,
                            CMD_HOME,
                        )
                        _wait_for_actuator(
                            port,
                            monitor,
                            lambda status: (
                                int(status["flags"]) & ACTUATOR_FLAG_HOMED
                            )
                            != 0
                            and (
                                int(status.get("reserved", 0))
                                & ACTUATOR_STATUS_SOFTWARE_HOME_ACTIVE
                            )
                            != 0
                            and (
                                int(status.get("reserved", 0))
                                & ACTUATOR_STATUS_SOFTWARE_FULL_ACTIVE
                            )
                            == 0,
                            after_count=actuator_count_before_home,
                            description="declare software HOME",
                            timeout_s=1.5,
                        )
                        ready_for_flight = _actuator_ready_for_flight(
                            monitor
                        )
                    if not ready_for_flight:
                        raise ReplayError(
                            "software HOME is missing or no longer ready; "
                            "the supervisor will not re-zero automatically: "
                            f"{monitor.actuator}"
                        )
                    print(
                        "Software HOME verified at target/XACTUAL zero with "
                        "motor energy off."
                    )

                target_dm = round(target_apogee_m * 10.0)
                target_sequence = _send_command(
                    port,
                    sequence,
                    CMD_SET_TARGET_APOGEE,
                    struct.pack("<H", target_dm),
                )
                _wait_for_ack(
                    port,
                    monitor,
                    target_sequence,
                    CMD_SET_TARGET_APOGEE,
                )

                monitor.begin_replay_metrics()
                telemetry_count_before_start = monitor.telemetry_count
                sim_start_sequence = _send_command(port, sequence, CMD_SIM_START)
                arm_sequence: int | None = None
                arm_acknowledged = no_arm
                sim_start_acknowledged = False
                simulation_active_seen = False
                start = active_clock.now()
                next_progress_s = 0.0
                previous_hil_state = (
                    hil_stroke.state if hil_stroke is not None else None
                )

                for sample_index, sample in enumerate(profile.samples):
                    try:
                        deadline_result = _wait_for_replay_deadline(
                            active_clock,
                            lambda: monitor.poll(port),
                            start_s=start,
                            replay_time_s=sample.replay_time_s,
                            rate_hz=profile.rate_hz,
                            continuous_hil=continuous_hil,
                        )
                    except ReplayDeadlineError as deadline_error:
                        max_schedule_lag_s = max(
                            max_schedule_lag_s,
                            deadline_error.lag_s,
                        )
                        raise
                    lag = deadline_result.lag_s
                    max_schedule_lag_s = max(max_schedule_lag_s, lag)
                    if deadline_result.rebase_s > 0.0:
                        start += deadline_result.rebase_s
                    if deadline_result.skip:
                        # Preserve the absolute timeline. An overdue row is
                        # skipped instead of creating a catch-up USB burst.
                        monitor.poll(port)
                        skipped_host_samples += 1
                        observer.emit(
                            "sample_skipped",
                            sample_index=sample_index,
                            replay_time_s=sample.replay_time_s,
                            schedule_lag_s=lag,
                        )
                        continue

                    packet_sequence = sequence.take()
                    port.write(
                        simulation_frame(
                            packet_sequence,
                            altitude_m=sample.altitude_m,
                            acceleration_mps2=sample.vertical_acceleration_mps2,
                            velocity_mps=sample.vertical_velocity_mps,
                            barometer_stddev_m=barometer_stddev_m,
                            time_ms=_host_packet_time_ms(active_clock),
                        )
                    )
                    sent_host_samples += 1
                    observer.emit(
                        "sample_sent",
                        sample_index=sample_index,
                        replay_time_s=sample.replay_time_s,
                        source_time_s=sample.source_time_s,
                        schedule_lag_s=lag,
                        schedule_rebase_s=deadline_result.rebase_s,
                        sequence=packet_sequence,
                    )
                    monitor.poll(port)
                    if allow_actuator_motion:
                        _assert_motion_runtime_healthy(
                            monitor,
                            continuous_hil=continuous_hil,
                        )

                    start_ack = monitor.acks.get(sim_start_sequence)
                    if start_ack is not None:
                        if (
                            start_ack["command"] != CMD_SIM_START
                            or start_ack["result"] != ACK_OK
                        ):
                            raise ReplayError(f"SIM_START failed: {start_ack}")
                        sim_start_acknowledged = True
                    if (
                        monitor.telemetry_count > telemetry_count_before_start
                        and monitor.telemetry is not None
                        and (
                            int(monitor.telemetry["flags"])
                            & TELEMETRY_FLAG_SIMULATION_ACTIVE
                        )
                        != 0
                    ):
                        simulation_active_seen = True
                    if sample.replay_time_s > 0.75:
                        if not sim_start_acknowledged:
                            raise ReplayError(
                                "SIM_START was not acknowledged while pad samples "
                                "were streaming"
                            )
                        if not simulation_active_seen:
                            raise ReplayError(
                                "fresh telemetry did not confirm SIMULATION_ACTIVE"
                            )

                    if (
                        not no_arm
                        and arm_sequence is None
                        and sim_start_acknowledged
                        and simulation_active_seen
                        and sample.replay_time_s >= arm_after_s
                    ):
                        arm_sequence = _send_command(
                            port,
                            sequence,
                            CMD_SET_ARMED,
                            b"\x01",
                        )
                    if arm_sequence is not None:
                        arm_ack = monitor.acks.get(arm_sequence)
                        if arm_ack is not None:
                            if (
                                arm_ack["command"] != CMD_SET_ARMED
                                or arm_ack["result"] != ACK_OK
                            ):
                                raise ReplayError(f"SET_ARMED failed: {arm_ack}")
                            arm_acknowledged = True
                    if (
                        sample.replay_time_s > profile.prepad_s
                        and not arm_acknowledged
                    ):
                        raise ReplayError(
                            "arm command was not acknowledged before launch data began"
                        )

                    if (
                        continuous_hil
                        and arm_acknowledged
                        and hil_stroke is not None
                    ):
                        requested_override = hil_stroke.observe(
                            source_time_s=sample.source_time_s,
                            telemetry=monitor.telemetry,
                            actuator=monitor.actuator,
                            host_now_s=active_clock.now(),
                        )
                        if requested_override is not None:
                            _set_hil_override(
                                port,
                                monitor,
                                sequence,
                                requested_override,
                                observer=observer,
                            )
                        if hil_stroke.state != previous_hil_state:
                            observer.emit(
                                "hil_stroke_state",
                                previous_state=previous_hil_state,
                                current_state=hil_stroke.state,
                                source_time_s=sample.source_time_s,
                                actuator=monitor.actuator,
                            )
                            previous_hil_state = hil_stroke.state

                    _emit_sample_observed(
                        observer,
                        sample_index=sample_index,
                        sample=sample,
                        schedule_lag_s=lag,
                        monitor=monitor,
                        hil_stroke=hil_stroke,
                        metadata=(
                            sample_metadata[sample_index]
                            if sample_metadata is not None
                            else None
                        ),
                        skipped_host_samples=skipped_host_samples,
                    )

                    if sample.replay_time_s >= next_progress_s:
                        _print_progress(
                            sample.replay_time_s,
                            monitor,
                            rotations=rotations,
                            full_steps_per_revolution=full_steps_per_revolution,
                            microsteps=microsteps,
                            gear_ratio=gear_ratio,
                            actuator_motion=allow_actuator_motion,
                        )
                        next_progress_s += 1.0

                if continuous_hil:
                    assert hil_stroke is not None
                    requested_override = hil_stroke.require_home(
                        active_clock.now()
                    )
                    if requested_override is not None:
                        _set_hil_override(
                            port,
                            monitor,
                            sequence,
                            requested_override,
                            observer=observer,
                        )
                    last_sample = profile.samples[-1]
                    tail_index = len(profile.samples)
                    tail_deadline = (
                        active_clock.now()
                        + endpoint_timeout_s * 2.0
                        + full_hold_s
                        + 1.0
                    )
                    next_tail_send = active_clock.now()
                    while not hil_stroke.complete:
                        now = active_clock.now()
                        if now >= tail_deadline:
                            raise ReplayError(
                                "forced ramp-state sequence did not return to "
                                "software HOME before the "
                                "bounded replay tail deadline"
                            )
                        if now < next_tail_send:
                            monitor.poll(port)
                            active_clock.sleep(min(0.002, next_tail_send - now))
                            continue
                        packet_sequence = sequence.take()
                        port.write(
                            simulation_frame(
                                packet_sequence,
                                altitude_m=last_sample.altitude_m,
                                acceleration_mps2=(
                                    last_sample.vertical_acceleration_mps2
                                ),
                                velocity_mps=last_sample.vertical_velocity_mps,
                                barometer_stddev_m=barometer_stddev_m,
                                time_ms=_host_packet_time_ms(active_clock),
                            )
                        )
                        sent_host_samples += 1
                        monitor.poll(port)
                        _assert_motion_runtime_healthy(
                            monitor,
                            continuous_hil=True,
                        )
                        requested_override = hil_stroke.observe(
                            source_time_s=last_sample.source_time_s,
                            telemetry=monitor.telemetry,
                            actuator=monitor.actuator,
                            host_now_s=active_clock.now(),
                        )
                        if requested_override is not None:
                            _set_hil_override(
                                port,
                                monitor,
                                sequence,
                                requested_override,
                                observer=observer,
                            )
                        if hil_stroke.state != previous_hil_state:
                            observer.emit(
                                "hil_stroke_state",
                                previous_state=previous_hil_state,
                                current_state=hil_stroke.state,
                                source_time_s=last_sample.source_time_s,
                                actuator=monitor.actuator,
                            )
                            previous_hil_state = hil_stroke.state
                        _emit_sample_observed(
                            observer,
                            sample_index=tail_index,
                            sample=last_sample,
                            schedule_lag_s=0.0,
                            monitor=monitor,
                            hil_stroke=hil_stroke,
                            metadata=(
                                sample_metadata[-1]
                                if sample_metadata is not None
                                else None
                            ),
                            skipped_host_samples=skipped_host_samples,
                        )
                        tail_index += 1
                        next_tail_send += 1.0 / profile.rate_hz

                    actuator_count_before_off = monitor.actuator_count
                    _set_hil_override(
                        port,
                        monitor,
                        sequence,
                        HIL_OVERRIDE_OFF,
                        observer=observer,
                    )
                    _wait_for_actuator(
                        port,
                        monitor,
                        lambda status: (
                            int(status["flags"])
                            & ACTUATOR_FLAG_DRIVER_ENABLED
                        )
                        == 0
                        and (
                            int(status.get("reserved", 0))
                            & ACTUATOR_STATUS_HIL_OVERRIDE_ACTIVE
                        )
                        == 0,
                        after_count=actuator_count_before_off,
                        description="clear HIL override and remove motor energy",
                        timeout_s=1.5,
                    )

                disarm_sequence = _send_command(
                    port,
                    sequence,
                    CMD_SET_ARMED,
                    b"\x00",
                )
                _wait_for_ack(port, monitor, disarm_sequence, CMD_SET_ARMED)
                stop_sequence = _send_command(port, sequence, CMD_SIM_STOP)
                _wait_for_ack(port, monitor, stop_sequence, CMD_SIM_STOP)
                if allow_actuator_motion and not continuous_hil:
                    actuator_count_before_retract = monitor.actuator_count
                    retract_sequence = _send_command(port, sequence, CMD_RETRACT)
                    _wait_for_ack(port, monitor, retract_sequence, CMD_RETRACT)
                    retracted = _wait_for_actuator(
                        port,
                        monitor,
                        lambda status: (
                            int(status["flags"])
                            & (
                                ACTUATOR_FLAG_DRIVER_ENABLED
                                | ACTUATOR_FLAG_MANUAL_PENDING
                            )
                        )
                        == 0
                        and abs(
                            int(status["target_steps"])
                            - int(status["actual_steps"])
                        )
                        <= ACTUATOR_POSITION_TOLERANCE_STEPS,
                        after_count=actuator_count_before_retract,
                        description="retract and switch the driver off",
                        timeout_s=10.0,
                    )
                    print(
                        "Actuator retracted and de-energized: "
                        f"target={retracted['target_steps']} "
                        f"actual={retracted['actual_steps']}"
                    )
                cleanup_required = False
            finally:
                if cleanup_required:
                    safety_cleanup = _perform_safety_cleanup(
                        port,
                        monitor,
                        sequence,
                        observer,
                        active_clock,
                        continuous_hil=continuous_hil,
                    )

        phases = " -> ".join(
            PHASE_NAMES.get(value, str(value)) for value in monitor.phase_order
        )
        print("Replay summary:")
        print(f"  phase order: {phases or 'no telemetry'}")
        print(f"  peak estimated altitude: {monitor.max_altitude_m:.1f} m")
        print(
            f"  peak predicted apogee: {monitor.max_predicted_apogee_m:.1f} m"
        )
        print(
            f"  maximum deployment request: {monitor.max_deployment_percent}%"
        )
        print(
            "  maximum host scheduling lag: "
            f"{max_schedule_lag_s * 1000.0:.2f} ms"
        )
        print(
            "  host samples skipped to prevent catch-up bursts: "
            f"{skipped_host_samples}"
        )
        print(f"  decoder errors: {monitor.decoder.errors}")
        if allow_actuator_motion and monitor.actuator is not None:
            print(
                "  final actuator: "
                f"target={monitor.actuator['target_steps']} "
                f"actual={monitor.actuator['actual_steps']} "
                f"flags=0x{int(monitor.actuator['flags']):02X}"
            )
        completed = True
    except BaseException as exc:
        failure = exc
        observer.emit(
            "run_failed",
            error_type=type(exc).__name__,
            error=str(exc),
        )
        raise
    finally:
        verdict = _build_live_verdict(
            monitor,
            completed=completed,
            failure=failure,
            no_arm=no_arm,
            allow_actuator_motion=allow_actuator_motion,
            max_schedule_lag_s=max_schedule_lag_s,
            skipped_host_samples=skipped_host_samples,
            sent_host_samples=sent_host_samples,
            continuous_hil=continuous_hil,
            hil_stroke=hil_stroke,
            safety_cleanup=safety_cleanup,
        )
        observer.finalize(verdict)

    print(f"Replay acceptance verdict: {verdict['status']}")
    if not verdict["passed"]:
        failed_checks = [
            name
            for name, check in verdict["checks"].items()
            if check["applicable"] and not check["passed"]
        ]
        raise ReplayError(
            "live replay completed but acceptance failed: "
            + ", ".join(failed_checks)
        )
    return verdict


# ---------------------------------------------------------------------------
# Command-line interface
# ---------------------------------------------------------------------------


def build_argument_parser() -> argparse.ArgumentParser:
    """Build the backward-compatible dry-run/live operator CLI."""

    parser = argparse.ArgumentParser(
        description="Inspect or safely replay an OpenRocket CSV over AMBAR USB"
    )
    parser.add_argument("csv", type=Path, help="OpenRocket CSV export")
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--prepad-s", type=float, default=1.0)
    parser.add_argument("--stop-s", type=float)
    parser.add_argument("--full-flight", action="store_true")
    parser.add_argument("--derivative-points", type=int, default=11)
    parser.add_argument(
        "--export-csv",
        type=Path,
        help="write the normalized replay rows for a GUI without opening a serial port",
    )
    parser.add_argument("--live", action="store_true", help="transmit to the real STM32")
    parser.add_argument("--port", help="COM port; omit to auto-detect 0483:5740")
    parser.add_argument(
        "--run-bundle",
        type=Path,
        help=(
            "write manifest.json, packets.jsonl, and verdict.json for this live run"
        ),
    )
    parser.add_argument(
        "--gui-udp-port",
        type=int,
        help=(
            "mirror decoded newline JSON to 127.0.0.1:PORT while this process "
            "retains exclusive COM ownership"
        ),
    )
    parser.add_argument(
        "--allow-derived-vertical",
        action="store_true",
        help="acknowledge approximate vertical channels when the CSV only has Total values",
    )
    parser.add_argument("--no-arm", action="store_true")
    parser.add_argument(
        "--allow-actuator-motion",
        action="store_true",
        help=(
            "enable the separately gated motor path; requires the "
            "CONTINUOUS_HIL firmware and --home-at-current-position"
        ),
    )
    parser.add_argument(
        "--home-at-current-position",
        action="store_true",
        help=(
            "confirm the mechanism is manually fully closed and authorize "
            "CMD_HOME to declare the current TMC ramp position as XACTUAL=0"
        ),
    )
    parser.add_argument("--arm-after-s", type=float, default=0.5)
    parser.add_argument("--target-apogee-m", type=float, default=914.4)
    parser.add_argument("--barometer-stddev-m", type=float, default=1.5)
    parser.add_argument("--brake-count", type=int, default=4)
    parser.add_argument("--force-per-brake-n", type=float, default=55.0)
    parser.add_argument("--full-extension-rotations", type=float, default=3.0)
    parser.add_argument("--motor-full-steps-per-rev", type=int, default=200)
    parser.add_argument("--microsteps", type=int, default=256)
    parser.add_argument("--gear-ratio", type=float, default=1.0)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Validate CLI input, run inspection/export, and optionally own live COM."""

    args = build_argument_parser().parse_args(argv)
    try:
        if args.full_flight and args.stop_s is not None:
            raise ReplayError("--full-flight and --stop-s cannot be used together")
        if not 0.1 <= args.barometer_stddev_m <= 100.0:
            raise ReplayError("barometer standard deviation must be within 0.1..100 m")
        if not 10.0 <= args.target_apogee_m <= 6553.5:
            raise ReplayError("target apogee must be within 10.0..6553.5 m")
        validate_mechanical_assumptions(
            brake_count=args.brake_count,
            force_per_brake_n=args.force_per_brake_n,
            rotations=args.full_extension_rotations,
            full_steps_per_revolution=args.motor_full_steps_per_rev,
            microsteps=args.microsteps,
            gear_ratio=args.gear_ratio,
        )
        dataset = load_openrocket_csv(args.csv)
        profile = build_replay_profile(
            dataset,
            rate_hz=args.rate_hz,
            prepad_s=args.prepad_s,
            stop_s=args.stop_s,
            full_flight=args.full_flight,
            derivative_points=args.derivative_points,
        )
        print_inspection(
            dataset,
            profile,
            brake_count=args.brake_count,
            force_per_brake_n=args.force_per_brake_n,
            rotations=args.full_extension_rotations,
            full_steps_per_revolution=args.motor_full_steps_per_rev,
            microsteps=args.microsteps,
            gear_ratio=args.gear_ratio,
        )
        if args.export_csv is not None:
            exported, metadata = export_replay_csv(
                args.export_csv,
                dataset,
                profile,
                barometer_stddev_m=args.barometer_stddev_m,
                target_apogee_m=args.target_apogee_m,
                brake_count=args.brake_count,
                force_per_brake_n=args.force_per_brake_n,
                rotations=args.full_extension_rotations,
                full_steps_per_revolution=args.motor_full_steps_per_rev,
                microsteps=args.microsteps,
                gear_ratio=args.gear_ratio,
            )
            print(f"Normalized replay CSV: {exported}")
            print(f"Replay metadata JSON: {metadata}")
            if profile.uses_derived_vertical:
                print("  WARNING: this export contains provisional derived vertical channels.")
        if not args.live:
            print("DRY RUN ONLY: no serial port was opened and no command was sent.")
            return 0
        if profile.uses_derived_vertical and not args.allow_derived_vertical:
            raise ReplayError(
                "live replay is blocked because vertical channels are derived; "
                "re-export Vertical velocity/acceleration or pass --allow-derived-vertical"
            )
        run_live_replay(
            profile,
            port_name=args.port,
            barometer_stddev_m=args.barometer_stddev_m,
            arm_after_s=args.arm_after_s,
            target_apogee_m=args.target_apogee_m,
            no_arm=args.no_arm,
            rotations=args.full_extension_rotations,
            full_steps_per_revolution=args.motor_full_steps_per_rev,
            microsteps=args.microsteps,
            gear_ratio=args.gear_ratio,
            allow_actuator_motion=args.allow_actuator_motion,
            home_at_current_position=args.home_at_current_position,
            run_bundle_dir=args.run_bundle,
            gui_udp_port=args.gui_udp_port,
            run_metadata={
                "source": {
                    "path": _shareable_project_path(dataset.path),
                    "sha256": hashlib.sha256(dataset.path.read_bytes())
                    .hexdigest()
                    .upper(),
                    "numeric_rows": dataset.row_count,
                },
                "tool": {
                    "path": _shareable_project_path(Path(__file__)),
                    "sha256": hashlib.sha256(Path(__file__).read_bytes())
                    .hexdigest()
                    .upper(),
                },
                "cli_arguments": list(argv) if argv is not None else sys.argv[1:],
            },
        )
        return 0
    except (ReplayError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("Interrupted; requested DISARM and SIM_STOP before closing.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
