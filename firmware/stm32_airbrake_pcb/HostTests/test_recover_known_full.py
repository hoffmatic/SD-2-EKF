"""Pure host-side checks for the guarded known-FULL recovery utility."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path


TOOLS = Path(__file__).resolve().parents[1] / "tools" / "usb_protocol"
sys.path.insert(0, str(TOOLS))

from recover_known_full import (  # noqa: E402
    _recovery_complete,
    _require_lost_reference_at_known_full,
)
from replay_openrocket import (  # noqa: E402
    ACTUATOR_FLAG_CONFIG_VALID,
    ACTUATOR_FLAG_DRIVER_OK,
    ACTUATOR_FLAG_HOMED,
    ACTUATOR_STATUS_CONTINUOUS_HIL_PROFILE,
    ACTUATOR_STATUS_GEOMETRY_PLAUSIBLE,
    ACTUATOR_STATUS_SOFTWARE_HOME_ACTIVE,
    ReplayError,
)


class RecoverKnownFullTests(unittest.TestCase):
    def reset_zero_status(self) -> dict[str, object]:
        return {
            "flags": ACTUATOR_FLAG_DRIVER_OK | ACTUATOR_FLAG_CONFIG_VALID,
            "reserved": ACTUATOR_STATUS_CONTINUOUS_HIL_PROFILE,
            "target_steps": 0,
            "actual_steps": 0,
        }

    def test_accepts_only_reset_zero_unhomed_continuous_state(self) -> None:
        _require_lost_reference_at_known_full(self.reset_zero_status())

    def test_rejects_existing_home_reference(self) -> None:
        status = self.reset_zero_status()
        status["flags"] = int(status["flags"]) | ACTUATOR_FLAG_HOMED
        with self.assertRaisesRegex(ReplayError, "software HOME still exists"):
            _require_lost_reference_at_known_full(status)

    def test_rejects_nonzero_ramp_reference(self) -> None:
        status = self.reset_zero_status()
        status["actual_steps"] = 153_500
        with self.assertRaisesRegex(ReplayError, "reset-zero"):
            _require_lost_reference_at_known_full(status)

    def test_completion_requires_home_zero_geometry_and_energy_off(self) -> None:
        complete = {
            "flags": (
                ACTUATOR_FLAG_DRIVER_OK
                | ACTUATOR_FLAG_CONFIG_VALID
                | ACTUATOR_FLAG_HOMED
            ),
            "reserved": (
                ACTUATOR_STATUS_CONTINUOUS_HIL_PROFILE
                | ACTUATOR_STATUS_GEOMETRY_PLAUSIBLE
                | ACTUATOR_STATUS_SOFTWARE_HOME_ACTIVE
            ),
            "target_steps": 0,
            "actual_steps": 0,
        }
        self.assertTrue(_recovery_complete(complete))
        complete["actual_steps"] = 1_000
        self.assertFalse(_recovery_complete(complete))


if __name__ == "__main__":
    unittest.main()
