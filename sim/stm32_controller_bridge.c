/**
 * @file stm32_controller_bridge.c
 * @brief Expose the production STM32 flight core to desktop simulations.
 *
 * Overall role
 *   RocketPy and the Monte Carlo runner own the virtual atmosphere, vehicle,
 *   sensors, and actuator.  This bridge owns the exact `ambar_ekf.c` and
 *   `ambar_flight.c` sources compiled into the STM32 firmware.  A persistent
 *   line-oriented process keeps estimator and phase state between simulation
 *   samples without pulling HAL, USB, or motor-driver code into the host build.
 *
 * Protocol
 *   RESET <time_s>
 *     Reset the production flight core on the virtual pad and arm it for the
 *     simulation.  The physical board still requires its normal runtime gates.
 *
 *   STEP <time_s> <vertical_accel_mps2> <altitude_m>
 *        <barometer_stddev_m> <use_barometer_0_or_1>
 *     Advance the IMU path and optionally the barometer correction, then emit
 *     one STATE response.
 *
 *   QUIT
 *     Exit cleanly.
 *
 * STATE fields
 *   deploy_fraction, inhibit, inhibit_flags, estimated altitude, estimated
 *   vertical velocity, predicted apogee, health, and phase name.  The format is
 *   intentionally identical to the older C++ bridge so the physics adapter can
 *   switch to production firmware logic without a second parser.
 *
 * Validation boundary
 *   This proves production estimator/controller behavior in a closed software
 *   loop.  It does not execute STM32 scheduling, USB, sensor drivers, the
 *   TMC5240 driver, or physical airbrake motion; those remain separate HIL and
 *   bench-test layers.
 */

#include "ambar_flight.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** Print one stable machine-readable controller snapshot. */
static void write_output(void)
{
    const AmbarFlightOutput_t output = AmbarFlight_GetOutput();

    printf("STATE %.6f %u %lu %.6f %.6f %.6f %u %s\n",
           (double)output.airbrake_command.deploy_fraction,
           output.airbrake_command.inhibit ? 1U : 0U,
           (unsigned long)output.airbrake_command.inhibit_flags,
           (double)output.estimate.altitude_agl_m,
           (double)output.estimate.vertical_velocity_mps,
           (double)output.estimate.predicted_apogee_m,
           output.health.healthy ? 1U : 0U,
           AmbarFlight_PhaseName(output.phase));
    fflush(stdout);
}

/** Parse one finite floating-point command-line value. */
static int parse_finite_float(const char *text, float *value)
{
    char *end = NULL;
    const float parsed = strtof(text, &end);

    if (end == text || *end != '\0' || !isfinite(parsed))
    {
        return 0;
    }
    *value = parsed;
    return 1;
}

/** Apply the fixed runtime configuration selected before a study begins. */
static int apply_arguments(int argc, char **argv, AmbarFlightConfig_t *config)
{
    int index;

    for (index = 1; index < argc; index += 2)
    {
        float value;
        const char *option = argv[index];

        if (index + 1 >= argc || !parse_finite_float(argv[index + 1], &value))
        {
            fprintf(stderr, "Missing or invalid value for %s.\n", option);
            return 0;
        }

        if (strcmp(option, "--minimum-boost-time") == 0 && value >= 0.0f)
        {
            config->phase.minimum_boost_time_s = value;
        }
        else if (strcmp(option, "--target-apogee-m") == 0 && value > 0.0f)
        {
            config->controller.target_apogee_m = value;
        }
        else if (strcmp(option, "--target-tolerance-m") == 0 && value >= 0.0f)
        {
            config->controller.apogee_tolerance_m = value;
        }
        else
        {
            fprintf(stderr, "Unknown option or invalid value: %s.\n", option);
            return 0;
        }
    }
    return 1;
}

int main(int argc, char **argv)
{
    char line[512];
    AmbarFlightConfig_t config = AmbarFlight_DefaultConfig();

    if (!apply_arguments(argc, argv, &config))
    {
        fputs("Usage: ambar_stm32_controller_bridge"
              " [--minimum-boost-time seconds]"
              " [--target-apogee-m meters]"
              " [--target-tolerance-m meters]\n",
              stderr);
        return 2;
    }

    AmbarFlight_Init(&config);
    AmbarFlight_ResetOnPad(0.0f);
    AmbarFlight_SetArmed(true);

    while (fgets(line, sizeof(line), stdin) != NULL)
    {
        char command[16];

        if (sscanf(line, "%15s", command) != 1)
        {
            continue;
        }

        if (strcmp(command, "RESET") == 0)
        {
            float timestamp_s = 0.0f;
            if (sscanf(line, "%*s %f", &timestamp_s) != 1
                || !isfinite(timestamp_s))
            {
                fputs("ERROR malformed RESET\n", stdout);
                fflush(stdout);
                continue;
            }
            AmbarFlight_ResetOnPad(timestamp_s);
            AmbarFlight_SetArmed(true);
            write_output();
            continue;
        }

        if (strcmp(command, "STEP") == 0)
        {
            float timestamp_s;
            float acceleration_mps2;
            float altitude_m;
            float barometer_stddev_m;
            int use_barometer;

            if (sscanf(line,
                       "%*s %f %f %f %f %d",
                       &timestamp_s,
                       &acceleration_mps2,
                       &altitude_m,
                       &barometer_stddev_m,
                       &use_barometer) != 5)
            {
                fputs("ERROR malformed STEP\n", stdout);
                fflush(stdout);
                continue;
            }

            (void)AmbarFlight_UpdateImu(timestamp_s, acceleration_mps2);
            if (use_barometer != 0)
            {
                (void)AmbarFlight_UpdateBarometer(altitude_m,
                                                   barometer_stddev_m);
            }
            write_output();
            continue;
        }

        if (strcmp(command, "QUIT") == 0)
        {
            break;
        }

        fputs("ERROR unknown command\n", stdout);
        fflush(stdout);
    }

    return 0;
}
