"""Run causally coupled 50 Hz AMBAR VARIABLE_HIL sessions.

At tick ``n`` this runner sends the *current* simulated sensor state, accepts
only the board reply correlated to that exact simulation sequence, and applies
the confirmed TMC5240 XACTUAL fraction to RocketPy's following integration
interval.  It never substitutes the controller request or target position for
actuator feedback and it never uses the forced continuous-HIL override.

Hardware access is lazy and requires all three explicit command-line gates:
``--hardware``, ``--allow-actuator-motion``, and
``--accept-current-position-home``.  Importing this module and its default fake
mode cannot enumerate or open a COM port.
"""

from __future__ import annotations

import argparse
import copy
from dataclasses import dataclass
from datetime import datetime, timezone
import importlib
import json
import math
from pathlib import Path
import secrets
import sys
import time
from typing import Any, Callable, Mapping, Protocol, Sequence

from variable_hil_store import (
    COUPLING_MODE,
    DEFAULT_TICK_HZ,
    VariableHilStore,
    default_results_root,
)


ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = ROOT / "sim" / "rocketpy"
USB_DIR = (
    ROOT
    / "firmware"
    / "stm32_airbrake_pcb"
    / "tools"
    / "usb_protocol"
)
DEFAULT_VEHICLE_CONFIG = MODEL_DIR / "ambar_reference_config.json"
DEFAULT_VARIABLE_CONFIG = MODEL_DIR / "variable_hil_m5_config.json"
TICK_HZ = 50.0
TICK_PERIOD_S = 1.0 / TICK_HZ
MAX_FEEDBACK_AGE_S = 0.100
FULL_TRAVEL_STEPS = 153_600
TMC_XACTUAL_SOURCE = 1
FEET_TO_METERS = 0.3048
# RocketActuatorStatusPayload.flags bit 4; bit 0 is BUILD_ENABLED.
ACTUATOR_FLAG_DRIVER_ENABLED = 1 << 4


def actuator_status_driver_enabled(status: Mapping[str, Any]) -> bool:
    """Decode motor-energy state from the actuator-status flag byte."""

    return bool(int(status.get("flags", 0)) & ACTUATOR_FLAG_DRIVER_ENABLED)


class VariableHilError(RuntimeError):
    """Base error for the causal host workflow."""


class VariableHilSafetyFault(VariableHilError):
    """Fault requiring immediate SIM_STOP and disarm."""


class UsbDisconnected(VariableHilSafetyFault):
    """The sole hardware transport disappeared or failed."""


class Clock(Protocol):
    def now(self) -> float: ...

    def sleep(self, seconds: float) -> None: ...


class MonotonicClock:
    def now(self) -> float:
        return time.perf_counter()

    def sleep(self, seconds: float) -> None:
        if seconds > 0.0:
            time.sleep(seconds)


@dataclass(frozen=True)
class SensorSample:
    simulation_time_s: float
    altitude_m: float
    velocity_mps: float
    acceleration_mps2: float
    truth_altitude_m: float
    truth_velocity_mps: float
    truth_acceleration_mps2: float
    barometer_stddev_m: float = 1.5


@dataclass(frozen=True)
class BoardFeedback:
    simulation_sequence: int
    packet_sequence: int
    received_at_s: float
    controller_request_fraction: float
    actuator_target_fraction: float
    actuator_xactual_fraction: float
    actuator_target_steps: int
    actuator_xactual_steps: int
    flight_inhibit_flags: int
    actuator_inhibit_flags: int
    driver_status: int
    config_crc32: int
    closed_predicted_apogee_m: float | None
    full_predicted_apogee_m: float | None
    target_apogee_m: float | None
    target_reachable: bool | None
    phase: int
    machine_state: int
    state_flags: int
    feedback_source_code: int
    driver_ok: bool
    config_valid: bool
    simulation_active: bool
    simulation_fresh: bool
    armed: bool
    estimated_altitude_m: float | None = None
    estimated_velocity_mps: float | None = None

    @property
    def feedback_source(self) -> str:
        if self.feedback_source_code == TMC_XACTUAL_SOURCE:
            return "TMC5240_XACTUAL"
        return f"UNKNOWN_{self.feedback_source_code}"


@dataclass(frozen=True)
class TickOutcome:
    tick_index: int
    physics_applied_fraction: float
    late_tick: bool
    consecutive_misses: int
    feedback: BoardFeedback | None


class VariableHilTransport(Protocol):
    is_hardware: bool

    def prepare(self) -> Mapping[str, Any]: ...

    def exchange(
        self,
        sequence: int,
        sample: SensorSample,
        *,
        deadline_s: float,
    ) -> BoardFeedback | None: ...

    def safe_stop(self, reason: str) -> Mapping[str, Any]: ...

    def close(self) -> None: ...


def _bounded_fraction(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result) or not 0.0 <= result <= 1.0:
        raise VariableHilSafetyFault(f"{name} is outside [0, 1]: {value!r}")
    return result


class CausalVariableHilStepper:
    """One wall-paced, no-catch-up, causal 50 Hz exchange state machine."""

    def __init__(
        self,
        transport: VariableHilTransport,
        store: VariableHilStore,
        hardware_run_id: str,
        *,
        clock: Clock | None = None,
        tick_hz: float = TICK_HZ,
        maximum_feedback_age_s: float = MAX_FEEDBACK_AGE_S,
        expected_config_crc32: int | None = None,
    ) -> None:
        if abs(float(tick_hz) - TICK_HZ) > 1e-9:
            raise ValueError("VARIABLE_HIL is fixed at 50 Hz")
        if maximum_feedback_age_s > MAX_FEEDBACK_AGE_S:
            raise ValueError("feedback-age limit cannot be weakened beyond 100 ms")
        self.transport = transport
        self.store = store
        self.hardware_run_id = hardware_run_id
        self.clock = clock if clock is not None else MonotonicClock()
        self.period_s = 1.0 / float(tick_hz)
        self.maximum_feedback_age_s = float(maximum_feedback_age_s)
        self.expected_config_crc32 = expected_config_crc32
        self.latched_config_crc32: int | None = expected_config_crc32
        self.epoch_s: float | None = None
        self.next_tick_s: float | None = None
        self.tick_index = 0
        self.sequence = 0
        self.last_confirmed_fraction = 0.0
        self.consecutive_misses = 0
        self.max_consecutive_misses = 0
        self.late_tick_count = 0
        self.maximum_feedback_age_observed_s = 0.0
        self.safety_fault: str | None = None
        self.emergency_cleanup: Mapping[str, Any] | None = None
        self.last_feedback: BoardFeedback | None = None

    @property
    def performance_timing_passed(self) -> bool:
        return self.late_tick_count == 0

    def _emergency_fault(self, message: str) -> None:
        self.safety_fault = message
        try:
            self.emergency_cleanup = self.transport.safe_stop(message)
        except BaseException as cleanup_error:
            self.emergency_cleanup = {
                "success": False,
                "error": f"{type(cleanup_error).__name__}: {cleanup_error}",
            }
        self.store.record_event(
            "variable_hil_safety_fault",
            hardware_run_id=self.hardware_run_id,
            host_elapsed_s=(
                None
                if self.epoch_s is None
                else max(0.0, self.clock.now() - self.epoch_s)
            ),
            message=message,
            cleanup=dict(self.emergency_cleanup),
        )
        raise VariableHilSafetyFault(message)

    def _validate_feedback(
        self,
        feedback: BoardFeedback,
        *,
        expected_sequence: int,
        sent_at_s: float,
    ) -> float:
        if feedback.packet_sequence != feedback.simulation_sequence:
            raise VariableHilSafetyFault(
                "VARIABLE_HIL packet/header correlation mismatch: "
                f"header={feedback.packet_sequence}, "
                f"payload={feedback.simulation_sequence}"
            )
        if feedback.simulation_sequence != expected_sequence:
            raise VariableHilSafetyFault(
                "VARIABLE_HIL sequence mismatch: "
                f"sent={expected_sequence}, confirmed={feedback.simulation_sequence}"
            )
        age_s = max(0.0, float(feedback.received_at_s) - sent_at_s)
        self.maximum_feedback_age_observed_s = max(
            self.maximum_feedback_age_observed_s, age_s
        )
        if age_s > self.maximum_feedback_age_s:
            raise VariableHilSafetyFault(
                f"actuator feedback age {age_s * 1000.0:.3f} ms exceeds 100 ms"
            )
        if feedback.feedback_source_code != TMC_XACTUAL_SOURCE:
            raise VariableHilSafetyFault(
                "causal physics requires TMC5240_XACTUAL feedback source 1; "
                f"received {feedback.feedback_source_code}"
            )
        if feedback.config_crc32 == 0:
            raise VariableHilSafetyFault("board reported an invalid zero config CRC32")
        if self.latched_config_crc32 is None:
            self.latched_config_crc32 = int(feedback.config_crc32)
            self.store.latch_board_config_crc32(self.latched_config_crc32)
        elif int(feedback.config_crc32) != int(self.latched_config_crc32):
            raise VariableHilSafetyFault(
                f"board config CRC32 mismatch: expected "
                f"0x{int(self.latched_config_crc32):08X}, got "
                f"0x{int(feedback.config_crc32):08X}"
            )
        required = {
            "driver_ok": feedback.driver_ok,
            "config_valid": feedback.config_valid,
            "simulation_active": feedback.simulation_active,
            "simulation_fresh": feedback.simulation_fresh,
            "armed": feedback.armed,
        }
        failed = [name for name, value in required.items() if not value]
        if failed:
            raise VariableHilSafetyFault(
                "board VARIABLE_HIL safety state is false: " + ", ".join(failed)
            )
        _bounded_fraction(
            feedback.controller_request_fraction, "controller request fraction"
        )
        _bounded_fraction(feedback.actuator_target_fraction, "actuator target fraction")
        _bounded_fraction(
            feedback.actuator_xactual_fraction, "actuator XACTUAL fraction"
        )
        return age_s

    def tick(self, sample: SensorSample) -> TickOutcome:
        if self.safety_fault is not None:
            raise VariableHilSafetyFault(
                f"VARIABLE_HIL is already faulted: {self.safety_fault}"
            )
        if self.epoch_s is None:
            self.epoch_s = self.clock.now()
            self.next_tick_s = self.epoch_s
        assert self.epoch_s is not None
        assert self.next_tick_s is not None
        index = self.tick_index
        scheduled_s = self.next_tick_s
        now = self.clock.now()
        late_tick = now >= scheduled_s + self.period_s
        if not late_tick and now < scheduled_s:
            self.clock.sleep(scheduled_s - now)
            now = self.clock.now()
        schedule_lag_s = max(0.0, now - scheduled_s)

        sequence: int | None = None
        sent_at_s: float | None = None
        feedback: BoardFeedback | None = None
        feedback_age_s: float | None = None
        safety_error: VariableHilSafetyFault | None = None

        if not late_tick:
            sequence = self.sequence
            self.sequence = (self.sequence + 1) & 0xFFFF
            sent_at_s = self.clock.now()
            # Every transmitted sample gets one complete 50 Hz response window.
            # The next send is paced from the actual send time, so a late host
            # callback can never create a compressed catch-up burst.
            tick_deadline_s = sent_at_s + self.period_s
            self.next_tick_s = tick_deadline_s
            try:
                feedback = self.transport.exchange(
                    sequence,
                    sample,
                    deadline_s=tick_deadline_s,
                )
            except VariableHilSafetyFault as error:
                safety_error = error
            except (OSError, IOError) as error:
                safety_error = UsbDisconnected(str(error))
            if safety_error is None and feedback is None:
                late_tick = True
            elif safety_error is None and feedback is not None:
                try:
                    feedback_age_s = self._validate_feedback(
                        feedback,
                        expected_sequence=sequence,
                        sent_at_s=sent_at_s,
                    )
                except VariableHilSafetyFault as error:
                    safety_error = error
        else:
            # Skip an entirely overdue physics callback without transmitting.
            # Rebase the wall schedule so the following callback cannot catch up
            # immediately behind this missed interval.
            self.next_tick_s = now + self.period_s

        if late_tick:
            self.consecutive_misses += 1
            self.late_tick_count += 1
        elif safety_error is None:
            self.consecutive_misses = 0
        self.max_consecutive_misses = max(
            self.max_consecutive_misses, self.consecutive_misses
        )

        if feedback is not None and safety_error is None and not late_tick:
            physics_fraction = _bounded_fraction(
                feedback.actuator_xactual_fraction,
                "actuator XACTUAL fraction",
            )
            self.last_confirmed_fraction = physics_fraction
            self.last_feedback = feedback
        else:
            # On one missed/late tick, retain the last confirmed physical state.
            # This marks the performance verdict FAIL but avoids inventing motion.
            physics_fraction = self.last_confirmed_fraction

        observed_at_s = self.clock.now()
        active_feedback = feedback if feedback is not None else self.last_feedback
        record = {
            "tick_index": index,
            "scheduled_time_s": scheduled_s - self.epoch_s,
            "host_elapsed_s": max(0.0, observed_at_s - self.epoch_s),
            "simulation_time_s": float(sample.simulation_time_s),
            "schedule_lag_s": schedule_lag_s,
            "late_tick": int(late_tick),
            "late_tick_count": self.late_tick_count,
            "consecutive_misses": self.consecutive_misses,
            "sensor_altitude_m": float(sample.altitude_m),
            "sensor_velocity_mps": float(sample.velocity_mps),
            "sensor_acceleration_mps2": float(sample.acceleration_mps2),
            "truth_altitude_m": float(sample.truth_altitude_m),
            "truth_velocity_mps": float(sample.truth_velocity_mps),
            "truth_acceleration_mps2": float(sample.truth_acceleration_mps2),
            "estimated_altitude_m": (
                None if active_feedback is None else active_feedback.estimated_altitude_m
            ),
            "estimated_velocity_mps": (
                None if active_feedback is None else active_feedback.estimated_velocity_mps
            ),
            "predicted_apogee_m": (
                None
                if active_feedback is None
                else active_feedback.closed_predicted_apogee_m
            ),
            "closed_predicted_apogee_m": (
                None
                if active_feedback is None
                else active_feedback.closed_predicted_apogee_m
            ),
            "full_predicted_apogee_m": (
                None
                if active_feedback is None
                else active_feedback.full_predicted_apogee_m
            ),
            "target_apogee_m": (
                None if active_feedback is None else active_feedback.target_apogee_m
            ),
            "target_reachable": (
                None
                if active_feedback is None or active_feedback.target_reachable is None
                else int(active_feedback.target_reachable)
            ),
            "controller_request_fraction": (
                None
                if active_feedback is None
                else active_feedback.controller_request_fraction
            ),
            "actuator_target_fraction": (
                None
                if active_feedback is None
                else active_feedback.actuator_target_fraction
            ),
            "actuator_target_steps": (
                None if active_feedback is None else active_feedback.actuator_target_steps
            ),
            "actuator_xactual_fraction": (
                None
                if active_feedback is None
                else active_feedback.actuator_xactual_fraction
            ),
            "actuator_xactual_steps": (
                None if active_feedback is None else active_feedback.actuator_xactual_steps
            ),
            "physics_applied_fraction": physics_fraction,
            "forced_hil_deployment_fraction": None,
            "feedback_source": (
                None if active_feedback is None else active_feedback.feedback_source
            ),
            "feedback_source_code": (
                None if active_feedback is None else active_feedback.feedback_source_code
            ),
            "feedback_age_s": feedback_age_s,
            "feedback_age_ms": (
                None if feedback_age_s is None else feedback_age_s * 1000.0
            ),
            "board_sequence_sent": sequence,
            "board_sequence_confirmed": (
                None if feedback is None else feedback.simulation_sequence
            ),
            "config_crc32": (
                None if active_feedback is None else active_feedback.config_crc32
            ),
            "flight_inhibit_flags": (
                None if active_feedback is None else active_feedback.flight_inhibit_flags
            ),
            "actuator_inhibit_flags": (
                None
                if active_feedback is None
                else active_feedback.actuator_inhibit_flags
            ),
            "driver_status": (
                None if active_feedback is None else active_feedback.driver_status
            ),
            "phase": None if active_feedback is None else active_feedback.phase,
            "machine_state": (
                None if active_feedback is None else active_feedback.machine_state
            ),
            "state_flags": (
                None if active_feedback is None else active_feedback.state_flags
            ),
            "hardware_ok": int(safety_error is None),
            "performance_ok": int(not late_tick),
            "fault_active": int(safety_error is not None),
        }
        self.store.record_sample(self.hardware_run_id, record)
        self.tick_index += 1

        if safety_error is not None:
            self._emergency_fault(str(safety_error))
        if self.consecutive_misses >= 2:
            self._emergency_fault(
                "two consecutive 50 Hz VARIABLE_HIL ticks missed; "
                "physics retained the last confirmed XACTUAL fraction"
            )
        return TickOutcome(
            tick_index=index,
            physics_applied_fraction=physics_fraction,
            late_tick=late_tick,
            consecutive_misses=self.consecutive_misses,
            feedback=feedback,
        )


class FakeClock:
    """Deterministic clock used only by fake transport/plant tests and demos."""

    def __init__(self, start_s: float = 0.0) -> None:
        self.value = float(start_s)

    def now(self) -> float:
        return self.value

    def sleep(self, seconds: float) -> None:
        self.value += max(0.0, float(seconds))

    def advance(self, seconds: float) -> None:
        self.value += float(seconds)


class FakeVariableHilTransport:
    """Deterministic transport that never imports pyserial or touches COM."""

    is_hardware = False

    def __init__(
        self,
        clock: Clock,
        *,
        config_crc32: int = 0x13579BDF,
        response_delays_s: Sequence[float | None] = (),
        sequence_offsets: Mapping[int, int] | None = None,
        disconnect_ticks: Sequence[int] = (),
        feedback_age_overrides_s: Mapping[int, float] | None = None,
        xactual_fractions: Sequence[float] = (),
        target_apogee_m: float = 914.4,
    ) -> None:
        if config_crc32 == 0:
            raise ValueError("fake config CRC32 must be nonzero")
        self.clock = clock
        self.config_crc32 = int(config_crc32)
        self.response_delays_s = tuple(response_delays_s)
        self.sequence_offsets = dict(sequence_offsets or {})
        self.disconnect_ticks = set(int(value) for value in disconnect_ticks)
        self.feedback_age_overrides_s = dict(feedback_age_overrides_s or {})
        self.xactual_fractions = tuple(float(value) for value in xactual_fractions)
        self.target_apogee_m = float(target_apogee_m)
        self.exchange_count = 0
        self.exchange_sequences: list[int] = []
        self.exchange_sent_at_s: list[float] = []
        self.exchange_deadline_budgets_s: list[float] = []
        self.stop_reasons: list[str] = []
        self.sim_stop_count = 0
        self.disarm_count = 0
        self.closed = False
        self._xactual = 0.0

    def prepare(self) -> Mapping[str, Any]:
        return {
            "schema": "ambar.variable_hil_config.fake.v1",
            "config_crc32": self.config_crc32,
            "target_apogee_m": self.target_apogee_m,
            "mission_tolerance_m": 30.48,
            "deployment_cda_m2": (0.012, 0.014, 0.017, 0.021, 0.026),
        }

    def exchange(
        self,
        sequence: int,
        sample: SensorSample,
        *,
        deadline_s: float,
    ) -> BoardFeedback | None:
        tick = self.exchange_count
        self.exchange_count += 1
        self.exchange_sequences.append(sequence)
        sent_at_s = self.clock.now()
        self.exchange_sent_at_s.append(sent_at_s)
        self.exchange_deadline_budgets_s.append(max(0.0, deadline_s - sent_at_s))
        if tick in self.disconnect_ticks:
            raise UsbDisconnected(f"fake USB disconnect at tick {tick}")
        delay = self.response_delays_s[tick] if tick < len(self.response_delays_s) else 0.0
        if delay is None:
            remaining = max(0.0, deadline_s - self.clock.now())
            self.clock.sleep(remaining)
            return None
        remaining = max(0.0, deadline_s - self.clock.now())
        if float(delay) >= remaining:
            self.clock.sleep(remaining)
            return None
        self.clock.sleep(float(delay))
        requested = max(
            0.0,
            min(1.0, (sample.altitude_m + max(0.0, sample.velocity_mps) * 5.0 - 700.0) / 350.0),
        )
        target = requested
        if tick < len(self.xactual_fractions):
            self._xactual = _bounded_fraction(
                self.xactual_fractions[tick], "fake XACTUAL fraction"
            )
        else:
            maximum_change = 0.05
            self._xactual += max(
                -maximum_change, min(maximum_change, target - self._xactual)
            )
        confirmed = (sequence + self.sequence_offsets.get(tick, 0)) & 0xFFFF
        received_at = self.clock.now() + self.feedback_age_overrides_s.get(tick, 0.0)
        closed_prediction = sample.altitude_m + max(0.0, sample.velocity_mps) ** 2 / 19.6133
        full_prediction = closed_prediction - 120.0
        return BoardFeedback(
            simulation_sequence=confirmed,
            packet_sequence=confirmed,
            received_at_s=received_at,
            controller_request_fraction=requested,
            actuator_target_fraction=target,
            actuator_xactual_fraction=self._xactual,
            actuator_target_steps=round(target * FULL_TRAVEL_STEPS),
            actuator_xactual_steps=round(self._xactual * FULL_TRAVEL_STEPS),
            flight_inhibit_flags=0,
            actuator_inhibit_flags=0,
            driver_status=0,
            config_crc32=self.config_crc32,
            closed_predicted_apogee_m=closed_prediction,
            full_predicted_apogee_m=full_prediction,
            target_apogee_m=self.target_apogee_m,
            target_reachable=full_prediction <= self.target_apogee_m <= closed_prediction,
            phase=3,
            machine_state=8,
            state_flags=0xBD,
            feedback_source_code=TMC_XACTUAL_SOURCE,
            driver_ok=True,
            config_valid=True,
            simulation_active=True,
            simulation_fresh=True,
            armed=True,
            estimated_altitude_m=sample.altitude_m,
            estimated_velocity_mps=sample.velocity_mps,
        )

    def safe_stop(self, reason: str) -> Mapping[str, Any]:
        self.stop_reasons.append(reason)
        self.sim_stop_count += 1
        self.disarm_count += 1
        return {
            "success": True,
            "sim_stop_attempted": True,
            "sim_stop_acknowledged": True,
            "disarm_attempted": True,
            "disarm_acknowledged": True,
            "driver_off_confirmed": True,
            "fake": True,
        }

    def close(self) -> None:
        self.closed = True


class DeterministicFakePlant:
    """Small deterministic plant for transport/timing tests, not flight evidence."""

    def __init__(self) -> None:
        self.time_s = 0.0
        self.altitude_m = 0.0
        self.velocity_mps = 80.0
        self.acceleration_mps2 = -9.80665
        self.maximum_altitude_m = 0.0

    def sample(self) -> SensorSample:
        return SensorSample(
            simulation_time_s=self.time_s,
            altitude_m=self.altitude_m,
            velocity_mps=self.velocity_mps,
            acceleration_mps2=self.acceleration_mps2,
            truth_altitude_m=self.altitude_m,
            truth_velocity_mps=self.velocity_mps,
            truth_acceleration_mps2=self.acceleration_mps2,
            barometer_stddev_m=0.0,
        )

    def advance(self, interval_s: float, deployment_fraction: float) -> None:
        deployment = _bounded_fraction(deployment_fraction, "plant deployment")
        self.acceleration_mps2 = -9.80665 - 8.0 * deployment
        self.velocity_mps += self.acceleration_mps2 * interval_s
        self.altitude_m = max(0.0, self.altitude_m + self.velocity_mps * interval_s)
        self.time_s += interval_s
        self.maximum_altitude_m = max(self.maximum_altitude_m, self.altitude_m)


def run_fake_plant(
    stepper: CausalVariableHilStepper,
    *,
    duration_s: float,
) -> tuple[float, list[TickOutcome]]:
    plant = DeterministicFakePlant()
    outcomes: list[TickOutcome] = []
    ticks = max(1, int(math.ceil(float(duration_s) * TICK_HZ)))
    for _ in range(ticks):
        outcome = stepper.tick(plant.sample())
        outcomes.append(outcome)
        plant.advance(TICK_PERIOD_S, outcome.physics_applied_fraction)
    return plant.maximum_altitude_m, outcomes


class VariableHilRocketPyController:
    """RocketPy air-brake callback that closes the physical causal loop."""

    def __init__(
        self,
        stepper: CausalVariableHilStepper,
        *,
        sensor_model: Any,
        environment_elevation_m: float,
        acceleration_projector: Callable[..., float],
        body_axis: Callable[[list[float]], tuple[float, float, float]],
    ) -> None:
        self.stepper = stepper
        self.sensor_model = sensor_model
        self.environment_elevation_m = float(environment_elevation_m)
        self.acceleration_projector = acceleration_projector
        self.body_axis = body_axis
        self.previous_time_s: float | None = None
        self.previous_velocity = (0.0, 0.0, 0.0)
        self.pad_reference_specific_force_mps2: float | None = None
        self.last_applied_fraction = 0.0
        self.log: list[dict[str, Any]] = []
        self.maximum_truth_altitude_m = 0.0

    def __call__(
        self,
        simulation_time_s: float,
        sampling_rate: float,
        state: list[float],
        state_history: list[list[float]],
        observed_variables: list[Any],
        air_brakes: Any,
        sensors: list[Any],
        controller_environment: Any,
    ) -> None:
        del state_history, observed_variables, sensors, controller_environment
        if abs(float(sampling_rate) - TICK_HZ) > 1e-6:
            raise VariableHilSafetyFault(
                f"RocketPy callback rate is {sampling_rate}, expected 50 Hz"
            )
        if (
            self.previous_time_s is not None
            and simulation_time_s <= self.previous_time_s + 1e-9
        ):
            air_brakes.deployment_level = self.last_applied_fraction
            return None

        velocity = (float(state[3]), float(state[4]), float(state[5]))
        truth_altitude = max(0.0, float(state[2]) - self.environment_elevation_m)
        axis = self.body_axis(state)
        if self.previous_time_s is None:
            truth_acceleration = 0.0
            self.pad_reference_specific_force_mps2 = 9.80665 * axis[2]
        else:
            dt_s = float(simulation_time_s) - self.previous_time_s
            acceleration_vector = tuple(
                (current - previous) / dt_s
                for current, previous in zip(velocity, self.previous_velocity)
            )
            truth_acceleration = self.acceleration_projector(
                acceleration_vector,
                axis,
                self.pad_reference_specific_force_mps2,
            )
        measured_altitude = self.sensor_model.altitude(truth_altitude)
        measured_acceleration = self.sensor_model.acceleration(truth_acceleration)
        sample = SensorSample(
            simulation_time_s=float(simulation_time_s),
            altitude_m=float(measured_altitude),
            velocity_mps=velocity[2],
            acceleration_mps2=float(measured_acceleration),
            truth_altitude_m=truth_altitude,
            truth_velocity_mps=velocity[2],
            truth_acceleration_mps2=truth_acceleration,
            barometer_stddev_m=float(
                self.sensor_model.values["barometer_measurement_std_dev_m"]
            ),
        )
        outcome = self.stepper.tick(sample)
        # RocketPy uses this value after the callback for the next ODE interval.
        air_brakes.deployment_level = outcome.physics_applied_fraction
        self.last_applied_fraction = outcome.physics_applied_fraction
        self.maximum_truth_altitude_m = max(
            self.maximum_truth_altitude_m, truth_altitude
        )
        self.log.append(
            {
                "simulation_time_s": float(simulation_time_s),
                "truth_altitude_m": truth_altitude,
                "truth_velocity_mps": velocity[2],
                "truth_acceleration_mps2": truth_acceleration,
                "physics_applied_fraction": outcome.physics_applied_fraction,
                "board_sequence": (
                    None
                    if outcome.feedback is None
                    else outcome.feedback.simulation_sequence
                ),
                "late_tick": outcome.late_tick,
            }
        )
        self.previous_time_s = float(simulation_time_s)
        self.previous_velocity = velocity
        return None


def _load_runtime_modules() -> tuple[Any, Any]:
    for directory in (MODEL_DIR, USB_DIR):
        text = str(directory)
        if text not in sys.path:
            sys.path.insert(0, text)
    return (
        importlib.import_module("run_rocketpy_sim"),
        importlib.import_module("rocket_protocol"),
    )


def _load_protocol_module() -> Any:
    text = str(USB_DIR)
    if text not in sys.path:
        sys.path.insert(0, text)
    return importlib.import_module("rocket_protocol")


def build_variable_hil_config(
    vehicle_config: Mapping[str, Any],
    protocol: Any,
) -> tuple[Any, bytes, dict[str, Any]]:
    """Build the wire config from the explicit merged M5 overlay.

    Values absent from the JSON model are the unchanged, named firmware M5
    factory defaults.  They are kept visible here rather than inferred from a
    prior run or a controller bridge.
    """

    controller = vehicle_config["controller"]
    predictor = vehicle_config["predictor"]
    control_mode_names = {"proportional": 0, "predictive": 1}
    control_mode_name = str(controller["control_mode"]).lower()
    if control_mode_name not in control_mode_names:
        raise VariableHilError(f"unsupported control mode {control_mode_name!r}")
    predictor_mode_names = {"ballistic": 0, "drag": 1}
    predictor_mode_name = str(predictor.get("mode", "drag")).lower()
    if predictor_mode_name not in predictor_mode_names:
        raise VariableHilError(f"unsupported predictor mode {predictor_mode_name!r}")
    cda = tuple(float(value) for value in predictor["deployment_drag_area_m2"])
    if len(cda) != 5:
        raise VariableHilError("predictor deployment_drag_area_m2 must have five points")
    config = protocol.VariableHilConfig(
        calibration_version=int(predictor["calibration_version"]),
        control_mode=control_mode_names[control_mode_name],
        predictor_mode=predictor_mode_names[predictor_mode_name],
        target_apogee_m=float(controller["target_apogee_ft"]) * FEET_TO_METERS,
        mission_tolerance_m=float(controller["mission_tolerance_ft"])
        * FEET_TO_METERS,
        control_deadband_m=float(controller["control_deadband_ft"]) * FEET_TO_METERS,
        full_deployment_error_m=float(
            controller.get("full_deployment_error_ft", 250.0)
        )
        * FEET_TO_METERS,
        minimum_deploy_altitude_m=float(
            controller.get("minimum_deploy_altitude_ft", 200.0)
        )
        * FEET_TO_METERS,
        minimum_flight_time_s=float(controller.get("minimum_flight_time_s", 1.0)),
        predictive_update_period_s=float(controller["predictive_update_period_s"]),
        coast_mass_kg=float(predictor["coast_mass_kg"]),
        maximum_deploy_fraction=float(
            controller.get("maximum_deploy_fraction", 1.0)
        ),
        deployment_hysteresis_fraction=float(
            controller["deployment_hysteresis_fraction"]
        ),
        deployment_cda_m2=cda,
        sea_level_air_density_kgpm3=float(
            predictor["sea_level_air_density_kgpm3"]
        ),
        density_scale_height_m=float(predictor["density_scale_height_m"]),
        launch_site_elevation_m=float(predictor["launch_site_elevation_m"]),
        actuator_delay_s=float(predictor["actuator_delay_s"]),
        actuator_open_rate_fraction_per_s=float(
            predictor["actuator_open_rate_fraction_per_s"]
        ),
        actuator_close_rate_fraction_per_s=float(
            predictor["actuator_close_rate_fraction_per_s"]
        ),
    )
    payload = protocol.encode_variable_hil_config_payload(config)
    decoded = dict(protocol.decode_variable_hil_config(payload))
    if int(decoded["config_crc32"]) == 0:
        raise VariableHilError("canonical VARIABLE_HIL config encoded a zero CRC32")
    return config, payload, decoded


def load_variable_hil_config(
    path: Path,
    protocol: Any,
    runtime: Any,
) -> tuple[dict[str, Any], Any, bytes, dict[str, Any]]:
    if not path.is_file():
        raise VariableHilError(
            f"canonical VARIABLE_HIL config is missing: {path}. "
            "Do not derive it from the legacy RocketPy reference config."
        )
    vehicle_config = runtime.load_config(path.resolve())
    sampling_rate = float(vehicle_config["airbrakes"]["sampling_rate_hz"])
    if abs(sampling_rate - TICK_HZ) > 1e-9:
        raise VariableHilError(
            f"VARIABLE_HIL overlay requests {sampling_rate} Hz, expected 50 Hz"
        )
    config, payload, decoded = build_variable_hil_config(vehicle_config, protocol)
    return vehicle_config, config, payload, decoded


class SerialVariableHilTransport:
    """Sole COM owner; constructed only after all hardware opt-ins pass."""

    is_hardware = True

    def __init__(
        self,
        *,
        port_name: str,
        clock: Clock,
        protocol: Any,
        serial_module: Any,
        variable_config: Any,
        expected_config_payload: bytes,
        target_apogee_m: float,
    ) -> None:
        self.clock = clock
        self.protocol = protocol
        self.variable_config = variable_config
        self.expected_config_payload = bytes(expected_config_payload)
        self.target_apogee_m = float(target_apogee_m)
        try:
            self.port = serial_module.Serial(
                port=port_name,
                baudrate=115200,
                timeout=0,
                write_timeout=0.2,
            )
        except BaseException as error:
            raise UsbDisconnected(f"could not open {port_name}: {error}") from error
        self.port_name = port_name
        self.decoder = protocol.StreamDecoder()
        # Firmware retains a small idempotency cache across host COM close/open
        # cycles. Start each host session at an unpredictable 16-bit sequence so
        # a prior session's cached command cannot be mistaken for this one.
        self.command_sequence = secrets.randbits(16)
        self.acks: dict[int, dict[str, int]] = {}
        self.states: list[tuple[int, dict[str, Any], float]] = []
        self.expired_simulation_sequence: int | None = None
        self.config_packets: list[tuple[int, bytes, dict[str, Any]]] = []
        self.heartbeat: dict[str, Any] | None = None
        self.actuator: dict[str, Any] | None = None
        self.actuator_generation = 0
        self.telemetry: dict[str, Any] | None = None
        self.prepared = False
        self.stopped = False

    @classmethod
    def open_after_opt_in(
        cls,
        *,
        hardware: bool,
        allow_actuator_motion: bool,
        accept_current_position_home: bool,
        port_name: str | None,
        clock: Clock,
        variable_config_path: Path,
        runtime: Any,
    ) -> tuple["SerialVariableHilTransport", dict[str, Any], dict[str, Any]]:
        validate_hardware_opt_in(
            hardware=hardware,
            allow_actuator_motion=allow_actuator_motion,
            accept_current_position_home=accept_current_position_home,
            port_name=port_name,
            variable_config_path=variable_config_path,
        )
        if not hardware:
            raise VariableHilError("serial transport requires --hardware")
        protocol = _load_protocol_module()
        vehicle_config, config, payload, decoded = load_variable_hil_config(
            variable_config_path, protocol, runtime
        )
        try:
            serial_module = importlib.import_module("serial")
        except ImportError as error:
            raise VariableHilError(
                "pyserial is required only for explicit VARIABLE_HIL hardware runs"
            ) from error
        return (
            cls(
                port_name=str(port_name),
                clock=clock,
                protocol=protocol,
                serial_module=serial_module,
                variable_config=config,
                expected_config_payload=payload,
                target_apogee_m=float(decoded["target_apogee_m"]),
            ),
            vehicle_config,
            decoded,
        )

    def _take_sequence(self) -> int:
        value = self.command_sequence
        self.command_sequence = (self.command_sequence + 1) & 0xFFFF
        return value

    def _write(self, frame: bytes) -> None:
        try:
            written = self.port.write(frame)
        except BaseException as error:
            raise UsbDisconnected(f"USB write failed: {error}") from error
        if written != len(frame):
            raise UsbDisconnected(
                f"short USB write: wrote {written} of {len(frame)} bytes"
            )

    def _poll(self) -> None:
        try:
            chunk = self.port.read(4096)
        except BaseException as error:
            raise UsbDisconnected(f"USB read failed: {error}") from error
        received_at = self.clock.now()
        decoder_errors_before = int(self.decoder.errors)
        packets = self.decoder.feed(chunk)
        if int(self.decoder.errors) != decoder_errors_before:
            raise VariableHilSafetyFault(
                "USB stream decoder rejected a corrupt or malformed packet"
            )
        try:
            for packet in packets:
                p = self.protocol
                if packet.packet_type == p.PKT_ACK:
                    ack = p.decode_ack(packet.payload)
                    self.acks[int(ack["command_sequence"])] = ack
                elif packet.packet_type == p.PKT_HEARTBEAT:
                    self.heartbeat = p.decode_heartbeat(packet.payload)
                elif packet.packet_type == p.PKT_ACTUATOR_STATUS:
                    self.actuator = p.decode_actuator_status(packet.payload)
                    self.actuator_generation += 1
                elif packet.packet_type == p.PKT_TELEMETRY:
                    self.telemetry = p.decode_telemetry(packet.payload)
                elif packet.packet_type == p.PKT_VARIABLE_HIL_STATE:
                    self.states.append(
                        (
                            int(packet.sequence),
                            dict(p.decode_variable_hil_state(packet.payload)),
                            received_at,
                        )
                    )
                elif packet.packet_type == p.PKT_VARIABLE_HIL_CONFIG:
                    self.config_packets.append(
                        (
                            int(packet.sequence),
                            bytes(packet.payload),
                            dict(p.decode_variable_hil_config(packet.payload)),
                        )
                    )
        except Exception as error:
            raise VariableHilSafetyFault(
                f"USB packet payload decoder failed: {error}"
            ) from error

    def _wait_until(self, predicate: Callable[[], Any], timeout_s: float, description: str) -> Any:
        deadline = self.clock.now() + float(timeout_s)
        while self.clock.now() < deadline:
            self._poll()
            result = predicate()
            if result is not None and result is not False:
                return result
            self.clock.sleep(min(0.001, max(0.0, deadline - self.clock.now())))
        raise VariableHilSafetyFault(f"timeout waiting for {description}")

    def _command(self, command: int, data: bytes = b"", timeout_s: float = 1.0) -> int:
        sequence = self._take_sequence()
        self._write(
            self.protocol.command_frame(
                sequence,
                command,
                data,
                time_ms=round(self.clock.now() * 1000.0) & 0xFFFFFFFF,
            )
        )

        def accepted() -> bool:
            ack = self.acks.get(sequence)
            if ack is None:
                return False
            if int(ack["command"]) != command or int(ack["result"]) != self.protocol.ACK_OK:
                raise VariableHilSafetyFault(
                    f"command 0x{command:02X} failed: {ack}"
                )
            return True

        self._wait_until(accepted, timeout_s, f"command 0x{command:02X} ACK")
        return sequence

    def _require_preflight(self) -> None:
        self._command(self.protocol.CMD_REQUEST_SNAPSHOT)
        self._wait_until(
            lambda: self.heartbeat is not None and self.actuator is not None,
            1.5,
            "heartbeat and actuator preflight",
        )
        assert self.heartbeat is not None and self.actuator is not None
        feature_bit = int(getattr(self.protocol, "FEATURE_VARIABLE_HIL", 1 << 16))
        if int(self.heartbeat["feature_flags"]) & feature_bit == 0:
            raise VariableHilSafetyFault("firmware does not advertise VARIABLE_HIL")
        if not bool(self.actuator.get("variable_hil_profile")):
            raise VariableHilSafetyFault("actuator status is not the VARIABLE_HIL profile")
        if bool(self.actuator.get("hil_override_active")):
            raise VariableHilSafetyFault("forced HIL override must be inactive")
        if actuator_status_driver_enabled(self.actuator):
            raise VariableHilSafetyFault("motor driver was enabled before VARIABLE_HIL start")

        # The caller explicitly confirmed the mechanism is fully closed. HOME
        # declares that position as ramp-generator zero; it does not seek.
        actuator_generation = self.actuator_generation
        self._command(self.protocol.CMD_HOME)

        def home_and_off() -> bool:
            if (
                self.actuator is None
                or self.actuator_generation == actuator_generation
            ):
                return False
            return (
                bool(self.actuator.get("software_home_active"))
                and int(self.actuator["target_steps"]) == 0
                and int(self.actuator["actual_steps"]) == 0
                and not actuator_status_driver_enabled(self.actuator)
            )

        self._wait_until(home_and_off, 1.5, "software HOME at zero with driver off")

    def _upload_and_verify_config(self) -> Mapping[str, Any]:
        sequence = self._take_sequence()
        self._write(
            self.protocol.variable_hil_config_upload_frame(
                sequence,
                self.variable_config,
                time_ms=round(self.clock.now() * 1000.0) & 0xFFFFFFFF,
            )
        )

        def upload_accepted() -> bool:
            ack = self.acks.get(sequence)
            if ack is None:
                return False
            expected_command = int(self.protocol.CMD_VARIABLE_HIL_CONFIG_UPLOAD)
            if (
                int(ack["command"]) != expected_command
                or int(ack["result"]) != self.protocol.ACK_OK
            ):
                raise VariableHilSafetyFault(f"atomic config upload failed: {ack}")
            return True

        self._wait_until(upload_accepted, 1.5, "atomic config-upload ACK")
        get_sequence = self._command(self.protocol.CMD_VARIABLE_HIL_GET_CONFIG)

        def matching_readback() -> Mapping[str, Any] | None:
            for packet_sequence, payload, decoded in self.config_packets:
                if packet_sequence != get_sequence:
                    continue
                if payload != self.expected_config_payload:
                    raise VariableHilSafetyFault(
                        "VARIABLE_HIL config readback bytes differ from uploaded bytes"
                    )
                return decoded
            return None

        decoded = self._wait_until(
            matching_readback, 1.5, "exact VARIABLE_HIL config readback"
        )
        if int(decoded["config_crc32"]) == 0:
            raise VariableHilSafetyFault("config readback CRC32 is zero")
        return dict(decoded)

    def _prime_fresh_simulation_input_before_arm(
        self,
        *,
        expected_config_crc32: int,
    ) -> None:
        """Establish the firmware's fresh-input arm gate without motion.

        SIM_START deliberately invalidates the previous simulation sample.  The
        VARIABLE_HIL image therefore must see one correlated, on-pad sample
        before it can accept ARM.  This packet is a preflight freshness sample,
        not a RocketPy physics tick; the first causal physics sample is still
        sent by :meth:`exchange` from RocketPy's controller callback.
        """

        sequence = self._take_sequence()
        self._write(
            self.protocol.simulation_frame(
                sequence,
                altitude_m=0.0,
                acceleration_mps2=0.0,
                velocity_mps=0.0,
                barometer_stddev_m=1.5,
                time_ms=round(self.clock.now() * 1000.0) & 0xFFFFFFFF,
            )
        )

        def correlated_fresh_state() -> bool:
            if not self.states:
                return False
            packet_sequence, state, _ = self.states.pop(0)
            payload_sequence = int(state["simulation_sequence"])
            if packet_sequence != sequence or payload_sequence != sequence:
                raise VariableHilSafetyFault(
                    "pre-arm VARIABLE_HIL sequence mismatch: "
                    f"sent={sequence}, header={packet_sequence}, "
                    f"payload={payload_sequence}"
                )
            required = {
                "config_valid": bool(state["config_valid"]),
                "simulation_active": bool(state["simulation_active"]),
                "simulation_fresh": bool(state["simulation_fresh"]),
                "software_home": bool(state["software_home"]),
                "driver_ok": bool(state["driver_ok"]),
            }
            failed = [name for name, value in required.items() if not value]
            if failed:
                raise VariableHilSafetyFault(
                    "pre-arm VARIABLE_HIL state is false: " + ", ".join(failed)
                )
            if bool(state["armed"]) or bool(state["driver_enabled"]):
                raise VariableHilSafetyFault(
                    "pre-arm VARIABLE_HIL unexpectedly reported arm or motor energy"
                )
            if int(state["config_crc32"]) != int(expected_config_crc32):
                raise VariableHilSafetyFault(
                    "pre-arm VARIABLE_HIL config CRC differs from exact readback"
                )
            if int(state["target_steps"]) != 0 or int(state["actual_steps"]) != 0:
                raise VariableHilSafetyFault(
                    "pre-arm VARIABLE_HIL is not HOME at exact XACTUAL zero"
                )
            return True

        self._wait_until(
            correlated_fresh_state,
            1.0,
            "correlated fresh simulation input before ARM",
        )

    def prepare(self) -> Mapping[str, Any]:
        self._require_preflight()
        decoded = self._upload_and_verify_config()
        self._command(self.protocol.CMD_SIM_START)
        self._prime_fresh_simulation_input_before_arm(
            expected_config_crc32=int(decoded["config_crc32"]),
        )
        self._command(self.protocol.CMD_SET_ARMED, b"\x01")
        self.prepared = True
        return decoded

    def exchange(
        self,
        sequence: int,
        sample: SensorSample,
        *,
        deadline_s: float,
    ) -> BoardFeedback | None:
        if not self.prepared or self.stopped:
            raise UsbDisconnected("VARIABLE_HIL transport is not active")
        immediately_previous_sequence = (sequence - 1) & 0xFFFF
        allowed_late_sequence = (
            self.expired_simulation_sequence
            if self.expired_simulation_sequence == immediately_previous_sequence
            else None
        )
        self.expired_simulation_sequence = None
        self._write(
            self.protocol.simulation_frame(
                sequence,
                altitude_m=sample.altitude_m,
                acceleration_mps2=sample.acceleration_mps2,
                velocity_mps=sample.velocity_mps,
                barometer_stddev_m=sample.barometer_stddev_m,
                time_ms=round(self.clock.now() * 1000.0) & 0xFFFFFFFF,
            )
        )
        while self.clock.now() < deadline_s:
            self._poll()
            while self.states:
                packet_sequence, state, received_at = self.states.pop(0)
                payload_sequence = int(state["simulation_sequence"])
                if (
                    allowed_late_sequence is not None
                    and packet_sequence == allowed_late_sequence
                    and payload_sequence == allowed_late_sequence
                ):
                    # The immediately preceding tick already failed when its
                    # complete 20 ms response window expired. Discard exactly
                    # one positively correlated late reply, then continue
                    # waiting for the current sequence within this tick's
                    # existing deadline. No frame is retransmitted.
                    allowed_late_sequence = None
                    continue
                telemetry = self.telemetry or {}
                return BoardFeedback(
                    simulation_sequence=payload_sequence,
                    packet_sequence=packet_sequence,
                    received_at_s=received_at,
                    controller_request_fraction=float(
                        state["controller_requested_fraction"]
                    ),
                    actuator_target_fraction=float(state["actuator_target_fraction"]),
                    actuator_xactual_fraction=float(state["xactual_fraction"]),
                    actuator_target_steps=int(state["target_steps"]),
                    actuator_xactual_steps=int(state["actual_steps"]),
                    flight_inhibit_flags=int(state["flight_inhibit_flags"]),
                    actuator_inhibit_flags=int(state["actuator_inhibit_flags"]),
                    driver_status=int(state["driver_status"]),
                    config_crc32=int(state["config_crc32"]),
                    closed_predicted_apogee_m=float(
                        state["closed_predicted_apogee_m"]
                    ),
                    full_predicted_apogee_m=float(
                        state["full_predicted_apogee_m"]
                    ),
                    target_apogee_m=(
                        self.target_apogee_m
                        if "target_apogee_m" not in telemetry
                        else float(telemetry["target_apogee_m"])
                    ),
                    target_reachable=bool(state["target_reachable"]),
                    phase=int(state["phase"]),
                    machine_state=int(state["machine_state"]),
                    state_flags=int(state["state_flags"]),
                    feedback_source_code=int(state["feedback_source"]),
                    driver_ok=bool(state["driver_ok"]),
                    config_valid=bool(state["config_valid"]),
                    simulation_active=bool(state["simulation_active"]),
                    simulation_fresh=bool(state["simulation_fresh"]),
                    armed=bool(state["armed"]),
                    estimated_altitude_m=(
                        None
                        if "altitude_m" not in telemetry
                        else float(telemetry["altitude_m"])
                    ),
                    estimated_velocity_mps=(
                        None
                        if "velocity_mps" not in telemetry
                        else float(telemetry["velocity_mps"])
                    ),
                )
            self.clock.sleep(min(0.0005, max(0.0, deadline_s - self.clock.now())))
        self.expired_simulation_sequence = sequence
        return None

    def safe_stop(self, reason: str) -> Mapping[str, Any]:
        if self.stopped:
            return {
                "success": True,
                "already_stopped": True,
                "reason": reason,
            }
        result: dict[str, Any] = {
            "success": True,
            "reason": reason,
            "sim_stop_attempted": True,
            "sim_stop_acknowledged": False,
            "disarm_attempted": True,
            "disarm_acknowledged": False,
            "driver_off_confirmed": False,
        }
        errors: list[str] = []
        try:
            self._command(self.protocol.CMD_SIM_STOP, timeout_s=0.5)
            result["sim_stop_acknowledged"] = True
        except BaseException as error:
            errors.append(f"SIM_STOP: {type(error).__name__}: {error}")
        try:
            self._command(self.protocol.CMD_SET_ARMED, b"\x00", timeout_s=0.5)
            result["disarm_acknowledged"] = True
        except BaseException as error:
            errors.append(f"DISARM: {type(error).__name__}: {error}")
        try:
            actuator_generation = self.actuator_generation
            self._command(self.protocol.CMD_REQUEST_SNAPSHOT, timeout_s=0.5)

            def home_and_driver_off() -> bool:
                return (
                    self.actuator is not None
                    and self.actuator_generation != actuator_generation
                    and not actuator_status_driver_enabled(self.actuator)
                    and int(self.actuator["target_steps"]) == 0
                    and int(self.actuator["actual_steps"]) == 0
                    and bool(self.actuator.get("software_home_active"))
                    and not bool(self.actuator.get("hil_override_active"))
                )

            self._wait_until(
                home_and_driver_off,
                0.75,
                "post-stop HOME/zero/override-off/driver-off status",
            )
            result["driver_off_confirmed"] = True
            result["final_actuator"] = dict(self.actuator or {})
        except BaseException as error:
            errors.append(f"DRIVER_OFF: {type(error).__name__}: {error}")
        result["errors"] = errors
        result["success"] = (
            not errors
            and bool(result["sim_stop_acknowledged"])
            and bool(result["disarm_acknowledged"])
            and bool(result["driver_off_confirmed"])
        )
        self.stopped = True
        return result

    def close(self) -> None:
        try:
            self.port.close()
        except BaseException:
            pass


def validate_hardware_opt_in(
    *,
    hardware: bool,
    allow_actuator_motion: bool,
    accept_current_position_home: bool,
    port_name: str | None,
    variable_config_path: Path | None,
) -> None:
    if not hardware:
        if allow_actuator_motion or accept_current_position_home or port_name:
            raise VariableHilError(
                "COM/motion options require the explicit --hardware gate"
            )
        return
    missing: list[str] = []
    if not allow_actuator_motion:
        missing.append("--allow-actuator-motion")
    if not accept_current_position_home:
        missing.append("--accept-current-position-home")
    if not port_name:
        missing.append("--port COMx")
    if variable_config_path is None or not variable_config_path.is_file():
        missing.append("--variable-config <canonical-json>")
    if missing:
        raise VariableHilError(
            "hardware VARIABLE_HIL remains disabled; missing " + ", ".join(missing)
        )


def _interpolate_curve(points: Sequence[float], deployment: float) -> float:
    value = max(0.0, min(1.0, float(deployment)))
    scaled = value * (len(points) - 1)
    left = min(len(points) - 2, int(math.floor(scaled)))
    fraction = scaled - left
    return float(points[left]) + fraction * (float(points[left + 1]) - float(points[left]))


def run_rocketpy_case(
    *,
    vehicle_config: Mapping[str, Any],
    board_config: Mapping[str, Any],
    transport: VariableHilTransport,
    stepper: CausalVariableHilStepper,
    maximum_time_s: float,
    runtime: Any,
) -> tuple[float, VariableHilRocketPyController, Mapping[str, Any]]:
    """Build once, then start hardware immediately before RocketPy integration."""

    environment = runtime.build_environment(vehicle_config)
    rocket = runtime.build_rocket(
        vehicle_config,
        runtime.build_motor(vehicle_config),
    )
    sensor_model = runtime.SensorModel(dict(vehicle_config))
    controller = VariableHilRocketPyController(
        stepper,
        sensor_model=sensor_model,
        environment_elevation_m=float(environment.elevation),
        acceleration_projector=runtime.pad_referenced_body_axis_acceleration,
        body_axis=runtime.rocket_body_z_axis,
    )
    cda_points = tuple(float(value) for value in board_config["deployment_cda_m2"])
    if len(cda_points) != 5:
        raise VariableHilError("VARIABLE_HIL deployment CdA curve must have five points")
    reference_area_m2 = math.pi * float(vehicle_config["rocket"]["radius_m"]) ** 2
    baseline_cda_m2 = cda_points[0]

    def airbrake_drag_coefficient(deployment: float, mach: float) -> float:
        del mach
        incremental_cda = max(
            0.0,
            _interpolate_curve(cda_points, deployment) - baseline_cda_m2,
        )
        return incremental_cda / reference_area_m2

    rocket.add_air_brakes(
        drag_coefficient_curve=airbrake_drag_coefficient,
        controller_function=controller,
        sampling_rate=TICK_HZ,
        clamp=True,
        override_rocket_drag=False,
        name="AMBAR VARIABLE_HIL XACTUAL-coupled Airbrakes",
    )

    verified_board_config = transport.prepare()
    if int(verified_board_config["config_crc32"]) != int(board_config["config_crc32"]):
        raise VariableHilSafetyFault("prepared board config CRC differs from frozen config")
    values = vehicle_config["environment"]
    runtime.Flight(
        rocket=rocket,
        environment=environment,
        rail_length=values["rail_length_m"],
        inclination=values["inclination_deg"],
        heading=values["heading_deg"],
        terminate_on_apogee=False,
        max_time=float(maximum_time_s),
        max_time_step=0.01,
        verbose=False,
    )
    return controller.maximum_truth_altitude_m, controller, verified_board_config


def _session_id_now() -> str:
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return f"{timestamp}-{secrets.token_hex(4)}"


ENERGY_CASE_FACTORS = {
    "low": 0.95,
    "nominal": 1.0,
    "high": 1.05,
}


def build_case_names(cycles: int, case_list: str) -> list[str]:
    if cycles <= 0:
        raise ValueError("cycle count must be positive")
    names = [name.strip().lower() for name in case_list.split(",") if name.strip()]
    if not names:
        raise ValueError("case list must contain low, nominal, or high")
    unsupported = [name for name in names if name not in ENERGY_CASE_FACTORS]
    if unsupported:
        raise ValueError(f"unsupported VARIABLE_HIL case names: {unsupported}")
    return [names[index % len(names)] for index in range(cycles)]


def apply_energy_case(
    base_config: Mapping[str, Any],
    case_name: str,
    case_index: int,
) -> dict[str, Any]:
    """Apply only the explicit +/-5% motor-energy commissioning bracket."""

    if case_name not in ENERGY_CASE_FACTORS:
        raise ValueError(f"unsupported case {case_name!r}")
    config = copy.deepcopy(dict(base_config))
    factor = ENERGY_CASE_FACTORS[case_name]
    nominal_impulse = float(config["motor"]["total_impulse_ns"])
    config["motor"]["total_impulse_ns"] = nominal_impulse * factor
    config["sensor_model"]["random_seed"] = int(
        config["sensor_model"]["random_seed"]
    ) + int(case_index)
    config["model_status"] = (
        f"{config['model_status']}; VARIABLE_HIL {case_name} energy case "
        f"({factor:.3f}x nominal motor total impulse)"
    )
    config["variable_hil_case"] = {
        "name": case_name,
        "case_index": int(case_index),
        "motor_total_impulse_factor": factor,
        "nominal_motor_total_impulse_ns": nominal_impulse,
        "applied_motor_total_impulse_ns": config["motor"]["total_impulse_ns"],
    }
    return config


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run causal 50 Hz RocketPy/STM32 VARIABLE_HIL"
    )
    parser.add_argument("--hardware", action="store_true")
    parser.add_argument("--allow-actuator-motion", action="store_true")
    parser.add_argument("--accept-current-position-home", action="store_true")
    parser.add_argument("--port", help="required explicit COM port in hardware mode")
    parser.add_argument("--variable-config", type=Path, default=DEFAULT_VARIABLE_CONFIG)
    parser.add_argument("--results-root", type=Path, default=default_results_root())
    parser.add_argument("--session-id")
    parser.add_argument("--max-time-s", type=float)
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument(
        "--case-list",
        default="low,nominal,high",
        help="comma-separated low/nominal/high sequence, repeated to --cycles",
    )
    parser.add_argument("--dwell-s", type=float, default=30.0)
    parser.add_argument("--fake-response-delay-ms", type=float, default=0.0)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    try:
        validate_hardware_opt_in(
            hardware=args.hardware,
            allow_actuator_motion=args.allow_actuator_motion,
            accept_current_position_home=args.accept_current_position_home,
            port_name=args.port,
            variable_config_path=args.variable_config if args.hardware else None,
        )
    except VariableHilError as error:
        raise SystemExit(str(error)) from error
    maximum_time_s = float(
        args.max_time_s if args.max_time_s is not None else (30.0 if args.hardware else 2.0)
    )
    if maximum_time_s <= 0.0:
        raise SystemExit("--max-time-s must be positive")
    if args.fake_response_delay_ms < 0.0:
        raise SystemExit("--fake-response-delay-ms must be nonnegative")
    if args.cycles <= 0:
        raise SystemExit("--cycles must be positive")
    if args.dwell_s < 0.0:
        raise SystemExit("--dwell-s must be nonnegative")
    try:
        case_names = build_case_names(args.cycles, args.case_list)
    except ValueError as error:
        raise SystemExit(str(error)) from error

    session_id = args.session_id or _session_id_now()
    session_dir = args.results_root.resolve() / session_id
    runtime = None
    protocol = None
    serial_module = None
    board_config: dict[str, Any]
    variable_config_object = None
    variable_config_payload = b""
    nominal_vehicle_config: dict[str, Any] | None = None

    if args.hardware:
        runtime, protocol = _load_runtime_modules()
        (
            nominal_vehicle_config,
            variable_config_object,
            variable_config_payload,
            board_config,
        ) = load_variable_hil_config(
            args.variable_config.resolve(), protocol, runtime
        )
        try:
            serial_module = importlib.import_module("serial")
        except ImportError as error:
            raise SystemExit("pyserial is required for --hardware") from error
    else:
        board_config = {
            "config_crc32": 0x13579BDF,
            "target_apogee_m": 914.4,
            "mission_tolerance_m": 30.48,
            "deployment_cda_m2": [0.012, 0.014, 0.017, 0.021, 0.026],
        }

    store = VariableHilStore.create(
        args.results_root,
        session_id,
        tick_hz=TICK_HZ,
        settings={
            "workflow": "VARIABLE_HIL",
            "coupling_mode": COUPLING_MODE,
            "hardware": bool(args.hardware),
            "motion_opt_in": bool(args.allow_actuator_motion),
            "serial_port": args.port if args.hardware else None,
            "feedback_source_boundary": (
                "TMC5240 XACTUAL ramp-generator state; not encoder, shaft, "
                "linkage, or airbrake-position evidence"
            ),
            "forced_hil_deployment_fraction": None,
            "tick_hz": TICK_HZ,
            "maximum_feedback_age_ms": MAX_FEEDBACK_AGE_S * 1000.0,
            "expected_runs": int(args.cycles),
            "case_list": case_names,
            "dwell_s": float(args.dwell_s),
            "variable_config": (
                str(args.variable_config.resolve()) if args.hardware else None
            ),
        },
        config_crc32=int(board_config["config_crc32"]),
    )

    campaign_fault: BaseException | None = None
    completed_runs = 0
    target_apogee_m = float(board_config["target_apogee_m"])
    target_tolerance_m = float(board_config["mission_tolerance_m"])
    try:
        for case_index, case_name in enumerate(case_names):
            simulation_case_id = f"{session_id}-case-{case_index:08d}"
            hardware_run_id = f"{session_id}-hw-{case_index:08d}"
            if args.hardware:
                assert nominal_vehicle_config is not None
                case_vehicle_config = apply_energy_case(
                    nominal_vehicle_config,
                    case_name,
                    case_index,
                )
                input_config: Mapping[str, Any] = {
                    "vehicle": case_vehicle_config,
                    "variable_hil": board_config,
                    "config_source": str(args.variable_config.resolve()),
                }
            else:
                case_vehicle_config = None
                input_config = {
                    "mode": "DETERMINISTIC_FAKE_PLANT",
                    "duration_s": maximum_time_s,
                    "case_name": case_name,
                    "case_index": case_index,
                    "board_config": board_config,
                }
            store.begin_run(
                simulation_case_id=simulation_case_id,
                hardware_run_id=hardware_run_id,
                case_index=case_index,
                input_config=input_config,
                target_apogee_m=target_apogee_m,
                mode=case_name,
            )

            clock: Clock = MonotonicClock()
            transport: VariableHilTransport | None = None
            stepper: CausalVariableHilStepper | None = None
            failure: BaseException | None = None
            truth_apogee_m: float | None = None
            cleanup: Mapping[str, Any] = {"success": False, "not_attempted": True}
            try:
                if args.hardware:
                    assert protocol is not None
                    assert serial_module is not None
                    assert variable_config_object is not None
                    transport = SerialVariableHilTransport(
                        port_name=str(args.port),
                        clock=clock,
                        protocol=protocol,
                        serial_module=serial_module,
                        variable_config=variable_config_object,
                        expected_config_payload=variable_config_payload,
                        target_apogee_m=target_apogee_m,
                    )
                else:
                    transport = FakeVariableHilTransport(
                        clock,
                        config_crc32=int(board_config["config_crc32"]),
                        response_delays_s=(args.fake_response_delay_ms / 1000.0,),
                        target_apogee_m=target_apogee_m,
                    )
                stepper = CausalVariableHilStepper(
                    transport,
                    store,
                    hardware_run_id,
                    clock=clock,
                    expected_config_crc32=int(board_config["config_crc32"]),
                )
                if args.hardware:
                    assert runtime is not None and case_vehicle_config is not None
                    truth_apogee_m, _, verified = run_rocketpy_case(
                        vehicle_config=case_vehicle_config,
                        board_config=board_config,
                        transport=transport,
                        stepper=stepper,
                        maximum_time_s=maximum_time_s,
                        runtime=runtime,
                    )
                    run_dir = session_dir / "runs" / hardware_run_id
                    run_dir.mkdir(parents=True, exist_ok=True)
                    (run_dir / "board_config_readback.json").write_text(
                        json.dumps(verified, indent=2, sort_keys=True) + "\n",
                        encoding="utf-8",
                    )
                else:
                    verified = transport.prepare()
                    store.latch_board_config_crc32(int(verified["config_crc32"]))
                    truth_apogee_m, _ = run_fake_plant(
                        stepper,
                        duration_s=maximum_time_s,
                    )
            except BaseException as error:
                failure = error
            finally:
                if transport is not None:
                    if stepper is not None and stepper.emergency_cleanup is not None:
                        cleanup = stepper.emergency_cleanup
                    else:
                        try:
                            cleanup = transport.safe_stop(
                                "normal completion"
                                if failure is None
                                else f"fault: {failure}"
                            )
                        except BaseException as cleanup_error:
                            cleanup = {
                                "success": False,
                                "error": (
                                    f"{type(cleanup_error).__name__}: {cleanup_error}"
                                ),
                            }
                            if failure is None:
                                failure = cleanup_error
                    transport.close()

            timing_pass = stepper is not None and stepper.performance_timing_passed
            safety_pass = (
                failure is None
                and stepper is not None
                and stepper.safety_fault is None
                and bool(cleanup.get("success"))
            )
            final_actuator = cleanup.get("final_actuator")
            if args.hardware:
                tracking_pass = (
                    safety_pass
                    and isinstance(final_actuator, Mapping)
                    and int(final_actuator.get("target_steps", 1)) == 0
                    and int(final_actuator.get("actual_steps", 1)) == 0
                    and bool(final_actuator.get("software_home_active"))
                    and not bool(final_actuator.get("hil_override_active"))
                    and not actuator_status_driver_enabled(final_actuator)
                )
            else:
                tracking_pass = False
            target_pass = (
                truth_apogee_m is not None
                and abs(float(truth_apogee_m) - target_apogee_m)
                <= target_tolerance_m
            )
            performance_pass = failure is None and bool(timing_pass) and target_pass
            hardware_verdict = {
                "schema": "ambar.variable_hil_hardware_verdict.v1",
                "status": (
                    "PASS"
                    if args.hardware and safety_pass and tracking_pass
                    else "FAIL" if args.hardware else "NOT_APPLICABLE"
                ),
                "passed": (safety_pass and tracking_pass) if args.hardware else None,
                "tracking_pass": tracking_pass,
                "hardware_exercised": bool(args.hardware),
                "coupling_mode": COUPLING_MODE,
                "feedback_source": "TMC5240_XACTUAL",
                "feedback_evidence_boundary": (
                    "TMC5240 ramp-generator state, not independent physical position"
                ),
                "config_crc32": int(board_config["config_crc32"]),
                "maximum_feedback_age_ms": (
                    0.0
                    if stepper is None
                    else stepper.maximum_feedback_age_observed_s * 1000.0
                ),
                "safety_fault": None if stepper is None else stepper.safety_fault,
                "protocol_failure": isinstance(
                    failure, (VariableHilSafetyFault, UsbDisconnected)
                ),
                "cleanup": dict(cleanup),
            }
            performance_verdict = {
                "schema": "ambar.variable_hil_performance_verdict.v1",
                "status": "PASS" if performance_pass else "FAIL",
                "passed": performance_pass,
                "case_name": case_name,
                "case_index": case_index,
                "tick_hz": TICK_HZ,
                "tick_count": 0 if stepper is None else stepper.tick_index,
                "late_tick_count": 0 if stepper is None else stepper.late_tick_count,
                "maximum_consecutive_misses": (
                    0 if stepper is None else stepper.max_consecutive_misses
                ),
                "timing_pass": bool(timing_pass),
                "truth_apogee_m": truth_apogee_m,
                "target_apogee_m": target_apogee_m,
                "target_tolerance_m": target_tolerance_m,
                "target_band_pass": target_pass,
                "forced_hil_deployment_fraction": None,
            }
            if failure is not None:
                store.record_event(
                    "variable_hil_run_failure",
                    hardware_run_id=hardware_run_id,
                    host_elapsed_s=None,
                    failure_type=type(failure).__name__,
                    failure_message=str(failure),
                )
            store.finalize_run(
                simulation_case_id=simulation_case_id,
                hardware_run_id=hardware_run_id,
                status=(
                    "FAULTED"
                    if failure is not None
                    or (args.hardware and not bool(hardware_verdict["passed"]))
                    else "PASS" if args.hardware else "SIMULATED"
                ),
                hardware_verdict=hardware_verdict,
                performance_verdict=performance_verdict,
                truth_apogee_m=truth_apogee_m,
                target_apogee_m=target_apogee_m,
                failure=failure,
                stop_reason=(
                    f"{type(failure).__name__}: {failure}"
                    if failure is not None
                    else f"bounded {case_name} causal run complete"
                ),
            )
            completed_runs += 1
            print(
                f"Completed VARIABLE_HIL case {case_index + 1}/{args.cycles} "
                f"({case_name}): hardware={hardware_verdict['status']}, "
                f"performance={performance_verdict['status']}"
            )
            if args.hardware and not bool(hardware_verdict["passed"]):
                campaign_fault = failure or VariableHilSafetyFault(
                    f"hardware/tracking verdict failed on case {case_index}"
                )
                break
            if (
                args.hardware
                and case_index + 1 < args.cycles
                and args.dwell_s > 0.0
            ):
                dwell_deadline = time.monotonic() + float(args.dwell_s)
                while time.monotonic() < dwell_deadline:
                    time.sleep(min(0.25, dwell_deadline - time.monotonic()))
    except BaseException as error:
        campaign_fault = error

    campaign = store.finalize_session(
        expected_runs=int(args.cycles),
        hardware_required=bool(args.hardware),
        stop_reason=(
            f"{type(campaign_fault).__name__}: {campaign_fault}"
            if campaign_fault is not None
            else f"bounded campaign complete ({completed_runs}/{args.cycles})"
        ),
    )
    store.close()

    print(f"AMBAR VARIABLE_HIL session: {session_id}")
    print(f"Results: {session_dir}")
    print(f"Campaign hardware safety verdict: {campaign['hardware']['status']}")
    print(f"Campaign performance verdict: {campaign['performance']['status']}")
    if campaign_fault is not None:
        print(
            f"VARIABLE_HIL CAMPAIGN FAULT: "
            f"{type(campaign_fault).__name__}: {campaign_fault}"
        )
        return 1
    hardware_failed = args.hardware and not bool(campaign["hardware"]["passed"])
    performance_failed = not bool(campaign["performance"]["passed"])
    return 1 if hardware_failed else 2 if performance_failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
