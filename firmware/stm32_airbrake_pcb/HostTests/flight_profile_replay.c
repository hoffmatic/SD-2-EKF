/**
 * @file flight_profile_replay.c
 * @brief Feed a normalized presentation profile through the production C flight stack.
 *
 * This executable compiles Core/Src/ambar_ekf.c and ambar_flight.c for the host,
 * reads one normalized CSV row at a time, and records phase/deployment coverage.
 * It validates the [ARCH-4] estimator/controller path without USB, HAL drivers,
 * timing jitter, or physical motion.  The input must use the normalized column
 * order emitted by replay_openrocket.py.
 */
#include "ambar_flight.h"

#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
    char line[512];
    FILE *input;
    AmbarFlightConfig_t config;
    AmbarFlightOutput_t output;
    float max_deploy = 0.0f;
    float max_predicted = 0.0f;
    unsigned phase_mask = 0U;
    unsigned rows = 0U;

    if (argc != 2)
    {
        fprintf(stderr, "usage: flight_profile_replay normalized.csv\n");
        return 2;
    }
    input = fopen(argv[1], "r");
    if (input == NULL)
    {
        perror(argv[1]);
        return 2;
    }
    if (fgets(line, sizeof(line), input) == NULL)
    {
        fclose(input);
        return 2;
    }

    config = AmbarFlight_DefaultConfig();
    AmbarFlight_Init(&config);
    AmbarFlight_ResetOnPad(0.0f);

    while (fgets(line, sizeof(line), input) != NULL)
    {
        float time_s;
        float source_time_s;
        float altitude_m;
        float reference_velocity_mps;
        float acceleration_mps2;
        float barometer_stddev_m;
        unsigned is_prepad;

        if (sscanf(line,
                   "%f,%f,%f,%f,%f,%f,%u",
                   &time_s,
                   &source_time_s,
                   &altitude_m,
                   &reference_velocity_mps,
                   &acceleration_mps2,
                   &barometer_stddev_m,
                   &is_prepad) != 7)
        {
            fprintf(stderr, "bad row: %s", line);
            fclose(input);
            return 2;
        }
        (void)source_time_s;
        (void)reference_velocity_mps;
        (void)is_prepad;

        if (time_s > 0.0f)
        {
            if (time_s >= 0.5f && !AmbarFlight_IsArmed())
            {
                AmbarFlight_SetArmed(true);
            }
            if (!AmbarFlight_UpdateImu(time_s, acceleration_mps2))
            {
                fprintf(stderr, "IMU rejection at %.6f s\n", time_s);
                fclose(input);
                return 1;
            }
            (void)AmbarFlight_UpdateBarometer(altitude_m, barometer_stddev_m);
        }

        output = AmbarFlight_GetOutput();
        phase_mask |= 1U << (unsigned)output.phase;
        if (output.airbrake_command.deploy_fraction > max_deploy)
        {
            max_deploy = output.airbrake_command.deploy_fraction;
        }
        if (output.estimate.predicted_apogee_m > max_predicted)
        {
            max_predicted = output.estimate.predicted_apogee_m;
        }
        ++rows;
    }
    fclose(input);

    output = AmbarFlight_GetOutput();
    printf("rows=%u phase=%s phase_mask=0x%02X max_predicted_m=%.2f max_deploy=%.3f\n",
           rows,
           AmbarFlight_PhaseName(output.phase),
           phase_mask,
           max_predicted,
           max_deploy);

    if ((phase_mask & (1U << AMBAR_PHASE_BOOST)) == 0U
        || (phase_mask & (1U << AMBAR_PHASE_COAST)) == 0U
        || (phase_mask & (1U << AMBAR_PHASE_AIRBRAKE_ACTIVE)) == 0U
        || (phase_mask & (1U << AMBAR_PHASE_FAULT)) != 0U
        || max_deploy <= 0.0f)
    {
        fputs("flight_profile_replay: FAIL\n", stderr);
        return 1;
    }

    puts("flight_profile_replay: PASS");
    return 0;
}
