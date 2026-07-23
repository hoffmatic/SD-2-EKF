/* Host checks for VARIABLE_HIL bracketing, eight-step bisection, saturation,
 * and command hysteresis using the production flight implementation. */

#include "ambar_flight.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#define CHECK(condition)                                                        \
    do                                                                          \
    {                                                                           \
        if (!(condition))                                                       \
        {                                                                       \
            (void)printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
            return false;                                                       \
        }                                                                       \
    } while (0)

static bool test_reachable_solution_matches_eight_bounded_iterations(void)
{
    AmbarFlightConfig_t config = AmbarFlight_M5VariableHilConfig();
    const float altitude_m = 250.0f;
    const float velocity_mps = 150.0f;
    const float current_fraction = 0.0f;
    const float closed = AmbarFlight_PredictApogee(
        altitude_m, velocity_mps, current_fraction, 0.0f, &config.apogee);
    const float full = AmbarFlight_PredictApogee(
        altitude_m, velocity_mps, current_fraction,
        config.controller.maximum_deploy_fraction, &config.apogee);
    float fraction = -1.0f;
    bool saturated = true;

    CHECK(closed > config.controller.target_apogee_m);
    CHECK(full < config.controller.target_apogee_m);
    CHECK(AmbarFlight_SolvePredictiveFraction(
        altitude_m,
        velocity_mps,
        current_fraction,
        closed,
        full,
        &config.controller,
        &config.apogee,
        &fraction,
        &saturated));
    CHECK(!saturated);
    CHECK(fraction > 0.0f && fraction < 1.0f);

    float lower = 0.0f;
    float upper = config.controller.maximum_deploy_fraction;
    for (uint32_t iteration = 0U;
         iteration < AMBAR_PREDICTIVE_BISECTION_ITERATIONS;
         ++iteration)
    {
        const float middle = 0.5f * (lower + upper);
        const float apogee = AmbarFlight_PredictApogee(
            altitude_m,
            velocity_mps,
            current_fraction,
            middle,
            &config.apogee);
        if (apogee > config.controller.target_apogee_m)
        {
            lower = middle;
        }
        else
        {
            upper = middle;
        }
    }
    CHECK(fabsf(fraction - 0.5f * (lower + upper)) < 1.0e-6f);
    CHECK((upper - lower)
          <= config.controller.maximum_deploy_fraction / 256.0f + 1.0e-6f);
    return true;
}

static bool test_saturation_occurs_only_outside_authority(void)
{
    AmbarFlightConfig_t config = AmbarFlight_M5VariableHilConfig();
    const float altitude_m = 250.0f;
    const float velocity_mps = 150.0f;
    const float closed = AmbarFlight_PredictApogee(
        altitude_m, velocity_mps, 0.0f, 0.0f, &config.apogee);
    const float full = AmbarFlight_PredictApogee(
        altitude_m, velocity_mps, 0.0f, 1.0f, &config.apogee);
    float fraction = -1.0f;
    bool saturated = false;

    config.controller.target_apogee_m = closed + 10.0f;
    CHECK(AmbarFlight_SolvePredictiveFraction(
        altitude_m, velocity_mps, 0.0f, closed, full,
        &config.controller, &config.apogee, &fraction, &saturated));
    CHECK(saturated && fraction == 0.0f);

    config.controller.target_apogee_m = full - 10.0f;
    CHECK(AmbarFlight_SolvePredictiveFraction(
        altitude_m, velocity_mps, 0.0f, closed, full,
        &config.controller, &config.apogee, &fraction, &saturated));
    CHECK(saturated
          && fraction == config.controller.maximum_deploy_fraction);

    config.controller.target_apogee_m = 0.5f * (closed + full);
    CHECK(!AmbarFlight_SolvePredictiveFraction(
        altitude_m, velocity_mps, 0.0f, full, closed,
        &config.controller, &config.apogee, &fraction, &saturated));
    return true;
}

static bool test_two_percent_hysteresis_and_saturation_bypass(void)
{
    const float previous = 0.50f;
    CHECK(AmbarFlight_ApplyDeploymentHysteresis(
              0.519f, previous, 0.02f, false)
          == previous);
    CHECK(AmbarFlight_ApplyDeploymentHysteresis(
              0.525f, previous, 0.02f, false)
          == 0.525f);
    CHECK(AmbarFlight_ApplyDeploymentHysteresis(
              0.501f, previous, 0.02f, true)
          == 0.501f);
    return true;
}

int main(void)
{
    unsigned passed = 0U;
    const unsigned total = 3U;
    passed += test_reachable_solution_matches_eight_bounded_iterations()
        ? 1U : 0U;
    passed += test_saturation_occurs_only_outside_authority() ? 1U : 0U;
    passed += test_two_percent_hysteresis_and_saturation_bypass() ? 1U : 0U;
    (void)printf("Predictive solver tests: %u/%u passed\n", passed, total);
    return passed == total ? 0 : 1;
}
