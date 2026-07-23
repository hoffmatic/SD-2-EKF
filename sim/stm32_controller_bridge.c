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
 *   vertical velocity, selected/retracted/full predicted apogee, health,
 *   controller mode, predictive-valid flag, and phase name.
 *
 *   PREDICT <altitude_m> <velocity_mps> <current_fraction> <target_fraction>
 *     Evaluate the pure production coast predictor for calibration tooling.
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

    printf("STATE %.6f %u %lu %.6f %.6f %.6f %.6f %.6f %u %u %u %u %s\n",
           (double)output.airbrake_command.deploy_fraction,
           output.airbrake_command.inhibit ? 1U : 0U,
           (unsigned long)output.airbrake_command.inhibit_flags,
           (double)output.estimate.altitude_agl_m,
           (double)output.estimate.vertical_velocity_mps,
           (double)output.estimate.predicted_apogee_m,
           (double)output.closed_predicted_apogee_m,
           (double)output.full_predicted_apogee_m,
           output.health.healthy ? 1U : 0U,
           (unsigned)output.controller_mode_used,
           output.predictive_solution_valid ? 1U : 0U,
           output.target_reachable ? 1U : 0U,
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

static int parse_u32(const char *text, uint32_t *value)
{
    char *end = NULL;
    const unsigned long parsed = strtoul(text, &end, 10);

    if (end == text || *end != '\0' || parsed > 0xFFFFFFFFUL)
    {
        return 0;
    }
    *value = (uint32_t)parsed;
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

        if (index + 1 >= argc)
        {
            fprintf(stderr, "Missing or invalid value for %s.\n", option);
            return 0;
        }

        if (strcmp(option, "--calibration-version") == 0)
        {
            uint32_t version;
            if (!parse_u32(argv[index + 1], &version))
            {
                fprintf(stderr, "Missing or invalid value for %s.\n", option);
                return 0;
            }
            config->apogee.calibration_version = version;
            continue;
        }

        if (!parse_finite_float(argv[index + 1], &value))
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
            config->controller.mission_tolerance_m = value;
        }
        else if (strcmp(option, "--mission-tolerance-m") == 0 && value > 0.0f)
        {
            config->controller.apogee_tolerance_m = value;
            config->controller.mission_tolerance_m = value;
        }
        else if (strcmp(option, "--control-deadband-m") == 0 && value >= 0.0f)
        {
            config->controller.control_deadband_m = value;
        }
        else if (strcmp(option, "--deployment-hysteresis") == 0 && value >= 0.0f)
        {
            config->controller.deployment_hysteresis_fraction = value;
        }
        else if (strcmp(option, "--predictive-period-s") == 0 && value > 0.0f)
        {
            config->controller.predictive_update_period_s = value;
        }
        else if (strcmp(option, "--control-mode") == 0
                 && (value == 0.0f || value == 1.0f))
        {
            config->controller.control_mode = (AmbarAirbrakeControlMode_t)((int)value);
        }
        else if (strcmp(option, "--coast-mass-kg") == 0 && value > 0.0f)
        {
            config->apogee.coast_mass_kg = value;
            config->apogee.vehicle_mass_kg = value;
        }
        else if (strcmp(option, "--baseline-cda-m2") == 0 && value > 0.0f)
        {
            config->apogee.baseline_drag_area_m2 = value;
            config->apogee.drag_area_m2 = value;
        }
        else if (strcmp(option, "--cda-0-m2") == 0 && value > 0.0f)
        {
            config->apogee.deployment_drag_area_m2[0] = value;
        }
        else if (strcmp(option, "--cda-25-m2") == 0 && value > 0.0f)
        {
            config->apogee.deployment_drag_area_m2[1] = value;
        }
        else if (strcmp(option, "--cda-50-m2") == 0 && value > 0.0f)
        {
            config->apogee.deployment_drag_area_m2[2] = value;
        }
        else if (strcmp(option, "--cda-75-m2") == 0 && value > 0.0f)
        {
            config->apogee.deployment_drag_area_m2[3] = value;
        }
        else if (strcmp(option, "--cda-100-m2") == 0 && value > 0.0f)
        {
            config->apogee.deployment_drag_area_m2[4] = value;
        }
        else if (strcmp(option, "--sea-level-density-kgpm3") == 0 && value > 0.0f)
        {
            config->apogee.sea_level_air_density_kgpm3 = value;
            config->apogee.air_density_kgpm3 = value;
        }
        else if (strcmp(option, "--density-scale-height-m") == 0 && value > 0.0f)
        {
            config->apogee.density_scale_height_m = value;
        }
        else if (strcmp(option, "--launch-site-elevation-m") == 0)
        {
            config->apogee.launch_site_elevation_m = value;
        }
        else if (strcmp(option, "--predictor-time-step-s") == 0 && value > 0.0f)
        {
            config->apogee.time_step_s = value;
        }
        else if (strcmp(option, "--predictor-max-time-s") == 0 && value > 0.0f)
        {
            config->apogee.max_predict_time_s = value;
        }
        else if (strcmp(option, "--actuator-delay-s") == 0 && value >= 0.0f)
        {
            config->apogee.actuator_delay_s = value;
        }
        else if (strcmp(option, "--actuator-open-rate") == 0 && value > 0.0f)
        {
            config->apogee.actuator_open_rate_fraction_per_s = value;
        }
        else if (strcmp(option, "--actuator-close-rate") == 0 && value > 0.0f)
        {
            config->apogee.actuator_close_rate_fraction_per_s = value;
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

    if (!apply_arguments(argc, argv, &config)
        || !AmbarFlight_ValidateControlConfig(&config.controller,
                                              &config.apogee,
                                              NULL))
    {
        fputs("Usage: ambar_stm32_controller_bridge"
              " [--minimum-boost-time seconds]"
              " [--target-apogee-m meters]"
              " [--target-tolerance-m meters]"
              " [--control-deadband-m meters]"
              " [--control-mode 0_or_1]"
              " [--coast-mass-kg kilograms]"
              " [--cda-0-m2 value] ... [--cda-100-m2 value]"
              " [--actuator-delay-s seconds]"
              " [--actuator-open-rate fraction_per_s]"
              " [--actuator-close-rate fraction_per_s]\n",
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
            float actuator_fraction = 0.0f;
            int use_barometer;
            int parsed_fields;

            parsed_fields = sscanf(line,
                                   "%*s %f %f %f %f %d %f",
                                   &timestamp_s,
                                   &acceleration_mps2,
                                   &altitude_m,
                                   &barometer_stddev_m,
                                   &use_barometer,
                                   &actuator_fraction);
            if (parsed_fields != 5 && parsed_fields != 6)
            {
                fputs("ERROR malformed STEP\n", stdout);
                fflush(stdout);
                continue;
            }

            AmbarFlight_SetActuatorFraction(actuator_fraction,
                                             parsed_fields == 6);
            (void)AmbarFlight_UpdateImu(timestamp_s, acceleration_mps2);
            if (use_barometer != 0)
            {
                (void)AmbarFlight_UpdateBarometer(altitude_m,
                                                   barometer_stddev_m);
            }
            write_output();
            continue;
        }

        if (strcmp(command, "PREDICT") == 0)
        {
            float altitude_m;
            float velocity_mps;
            float current_fraction;
            float target_fraction;

            if (sscanf(line,
                       "%*s %f %f %f %f",
                       &altitude_m,
                       &velocity_mps,
                       &current_fraction,
                       &target_fraction) != 4)
            {
                fputs("ERROR malformed PREDICT\n", stdout);
                fflush(stdout);
                continue;
            }

            printf("PREDICTION %.6f\n",
                   (double)AmbarFlight_PredictApogee(
                       altitude_m,
                       velocity_mps,
                       current_fraction,
                       target_fraction,
                       &config.apogee));
            fflush(stdout);
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
