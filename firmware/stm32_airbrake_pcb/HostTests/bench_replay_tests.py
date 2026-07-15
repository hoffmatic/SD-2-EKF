"""Quick mathematical smoke test for the two apogee predictor models.

The script generates one synthetic upward coast, evaluates both the ballistic
and drag-aware equations, and checks broad physical invariants.  Constants at
the top are scenario inputs, not validated vehicle measurements.  This test is
useful before changing [ARCH-4] prediction logic, but it does not compile the
production C implementation, exercise USB, or prove physical actuator motion.
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
