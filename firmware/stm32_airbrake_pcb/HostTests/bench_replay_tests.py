"""
PROJECT FILE OVERVIEW
Comment made: 2026-07-07 18:31:46 -04:00

What this file does:
  This is a small host-side replay check for the airbrake firmware math. It compares the old ballistic apogee estimate with the new drag-aware coast estimate.

Process flow:
  A simple simulated coast profile is generated, both apogee predictors are run, and the script prints pass/fail checks that can be repeated before changing firmware tuning.

Main variables and what can be changed:
  MASS_KG, DRAG_AREA_M2, AIR_DENSITY, START_ALT_M, and START_VEL_MPS are the main test knobs. Change them to match measured rocket values after bench work.

Assumptions:
  This script tests the predictor shape only. It does not compile or run the STM32 firmware, and it does not validate the real actuator.

What is missing:
  There is no canned real flight log replay yet and no automatic comparison against the original desktop C++ EKF.
"""

import math

G = 9.80665
MASS_KG = 5.0
DRAG_AREA_M2 = 0.012
AIR_DENSITY = 1.225
START_ALT_M = 300.0
START_VEL_MPS = 120.0


def ballistic_apogee(altitude_m: float, velocity_mps: float) -> float:
    if velocity_mps <= 0.0:
        return altitude_m
    return altitude_m + (velocity_mps * velocity_mps) / (2.0 * G)


def drag_apogee(altitude_m: float, velocity_mps: float, dt_s: float = 0.02) -> float:
    drag_k = 0.5 * AIR_DENSITY * DRAG_AREA_M2 / MASS_KG
    altitude = altitude_m
    velocity = velocity_mps
    elapsed = 0.0

    while velocity > 0.0 and elapsed < 30.0:
        accel = -G - drag_k * velocity * velocity
        altitude += velocity * dt_s + 0.5 * accel * dt_s * dt_s
        velocity += accel * dt_s
        elapsed += dt_s

    return max(altitude, altitude_m)


def main() -> int:
    ballistic = ballistic_apogee(START_ALT_M, START_VEL_MPS)
    drag = drag_apogee(START_ALT_M, START_VEL_MPS)

    print(f"ballistic_apogee_m={ballistic:.2f}")
    print(f"drag_apogee_m={drag:.2f}")

    checks = [
        ("drag_is_below_ballistic", drag < ballistic),
        ("drag_is_above_current_altitude", drag >= START_ALT_M),
        ("ballistic_is_reasonable", math.isfinite(ballistic) and ballistic > START_ALT_M),
        ("drag_is_reasonable", math.isfinite(drag) and drag > START_ALT_M),
    ]

    failed = [name for name, ok in checks if not ok]
    if failed:
        print("FAILED:", ", ".join(failed))
        return 1

    print("PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
