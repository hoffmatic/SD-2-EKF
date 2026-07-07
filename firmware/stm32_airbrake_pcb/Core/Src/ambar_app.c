/*
 * PROJECT FILE OVERVIEW
 * Comment made: 2026-07-07 17:44:48 -04:00
 *
 * What this file does:
 *   This is the main flight scheduler for the Airbrake PCB firmware. It connects sensors, the EKF, telemetry, and the actuator safety gate without using the old blocking one-second loop.
 *
 * Process flow:
 *   Startup stores sensor, radio, and motor health; disables the motor; captures pad acceleration and pressure; then resets the filter. The loop reads IMU about every 8 ms, barometer every 20 ms, and telemetry every 200 ms.
 *
 * Main variables and what can be changed:
 *   AMBAR_IMU_PERIOD_MS, AMBAR_BARO_PERIOD_MS, and AMBAR_TELEMETRY_PERIOD_MS set rates. AMBAR_BARO_STDDEV_M controls how much the filter trusts barometer altitude.
 *
 * Assumptions:
 *   HAL_GetTick is running, the board is motionless during pad reference capture, and short radio transmit blocking is acceptable for first bench tests.
 *
 * What is missing:
 *   There is no persistent settings storage, command handling, flight replay, or real actuator motion control yet.
 */

#include "ambar_app.h"

#include "ambar_actuator.h"
#include "ambar_command.h"
#include "ambar_config.h"
#include "ambar_flight.h"
#include "ambar_log.h"
#include "radio_bridge.h"
#include "rocket_sensors.h"
#include "tmc5240.h"
#include "w25q64.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

/*
 * ===================== AMBAR EKF PCB INTEGRATION - NEW FILE =====================
 *
 * This file glues the board drivers to the plain-C flight estimator:
 *   - IMU propagation runs about every 8 ms, roughly 125 Hz.
 *   - Barometer correction runs every 20 ms, 50 Hz.
 *   - Radio telemetry runs every 200 ms, 5 Hz.
 *
 * The motor driver is intentionally kept disabled unless the actuator module says
 * every safety gate has passed.  With AMBAR_ENABLE_ACTUATOR left at 0, this code
 * will estimate and report deployment commands without moving hardware.
 */

#define AMBAR_IMU_PERIOD_MS       8U
#define AMBAR_BARO_PERIOD_MS      20U
#define AMBAR_TELEMETRY_PERIOD_MS 200U
#define AMBAR_LOG_PERIOD_MS       200U
#define AMBAR_BARO_STDDEV_M       1.5f

static RocketSensorRawData_t s_raw_sensor_data;
/* Latest raw samples are kept so telemetry can send exactly what the sensors returned. */
static RocketSensorConvertedData_t s_converted_sensor_data;
/* Converted samples are the SI-unit values used by the EKF. */
static HAL_StatusTypeDef s_sensor_status = HAL_ERROR;
/* Init/last-read status values are mirrored into telemetry messages and LEDs. */
static HAL_StatusTypeDef s_pad_reference_status = HAL_ERROR;
static HAL_StatusTypeDef s_radio_status = HAL_ERROR;
static HAL_StatusTypeDef s_imu_status = HAL_ERROR;
static HAL_StatusTypeDef s_mag_status = HAL_ERROR;
static HAL_StatusTypeDef s_baro_status = HAL_ERROR;
static HAL_StatusTypeDef s_telemetry_status = HAL_ERROR;
static HAL_StatusTypeDef s_flash_status = HAL_ERROR;
static HAL_StatusTypeDef s_config_load_status = HAL_ERROR;
static HAL_StatusTypeDef s_config_save_status = HAL_ERROR;
static HAL_StatusTypeDef s_log_status = HAL_ERROR;
static AmbarActuatorState_t s_actuator_state;
static AmbarConfig_t s_config;
static AmbarCommandResult_t s_last_command_result;
static AmbarActuatorDecision_t s_last_actuator_decision;
static W25Q64_JedecId_t s_flash_jedec;
/* Tick timestamps for the cooperative scheduler periods below. */
static uint32_t s_last_imu_ms = 0U;
static uint32_t s_last_baro_ms = 0U;
static uint32_t s_last_telemetry_ms = 0U;
static uint32_t s_last_log_ms = 0U;

static uint16_t ambar_u16_from_float(float value, float scale)
{
    /*
     * Legacy telemetry fields are unsigned 16-bit values.  Keep them populated
     * for existing ground tools, while detailed signed fixed-point EKF fields are
     * appended by PayloadPipelineWithFlight().
     */
    if (!isfinite(value) || value <= 0.0f)
    {
        return 0U;
    }

    const float scaled = value * scale;
    if (scaled >= 65535.0f)
    {
        return 65535U;
    }

    return (uint16_t)(scaled + 0.5f);
}

static uint8_t ambar_deployment_percent(const AmbarFlightOutput_t *output)
{
    /*
     * Deployment percent is a display-friendly value.  If the flight layer is
     * inhibited, percent is forced to zero even if the raw controller error would
     * otherwise request deployment.
     */
    if (output == NULL || output->airbrake_command.inhibit)
    {
        return 0U;
    }

    float percent = output->airbrake_command.deploy_fraction * 100.0f;
    if (percent < 0.0f)
    {
        percent = 0.0f;
    }
    if (percent > 100.0f)
    {
        percent = 100.0f;
    }

    return (uint8_t)(percent + 0.5f);
}

static void ambar_fill_legacy_calc(uint16_t calc_data[4],
                                   const AmbarFlightOutput_t *output,
                                   uint8_t deployment_percent)
{
    /*
     * BEGIN AMBAR EKF PCB INTEGRATION - LEGACY CALC COMPATIBILITY
     *
     * These four values replace the old placeholder calculated data while staying
     * in the same 4x uint16_t packet section:
     *   0: altitude AGL, centimeters
     *   1: predicted apogee, centimeters
     *   2: deployment command, percent
     *   3: flight phase enum
     */
    calc_data[0] = ambar_u16_from_float(output->estimate.altitude_agl_m, 10.0f);
    calc_data[1] = ambar_u16_from_float(output->estimate.predicted_apogee_m, 10.0f);
    calc_data[2] = (uint16_t)deployment_percent;
    calc_data[3] = (uint16_t)output->phase;
    /* END AMBAR EKF PCB INTEGRATION - LEGACY CALC COMPATIBILITY */
}

static const char *ambar_status_message(const AmbarFlightOutput_t *output)
{
    /*
     * Keep the text messages short because they share the same SX1280 packet as
     * raw sensor data and fixed-point estimator fields.
     */
    if (s_sensor_status != HAL_OK)
    {
        return "SENSOR_INIT_FAULT";
    }

    if (s_pad_reference_status != HAL_OK)
    {
        return "PAD_REF_FAULT";
    }

    if (output != NULL && !output->estimate.healthy)
    {
        return "EKF_FAULT";
    }

    return "OK";
}

static int32_t ambar_s32_from_float(float value, float scale)
{
    if (!isfinite(value))
    {
        return 0;
    }

    const float scaled = value * scale;
    if (scaled > 2147483647.0f)
    {
        return 2147483647;
    }
    if (scaled < -2147483648.0f)
    {
        return (int32_t)0x80000000UL;
    }

    return (int32_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

static void ambar_apply_runtime_config(uint8_t reset_estimator)
{
    /*
     * BEGIN AMBAR BENCH-GATED EXPANSION - CONFIG APPLY POINT
     *
     * The saved config fans out to three runtime areas:
     *   - sensor orientation,
     *   - flight/EKF/controller tuning,
     *   - actuator travel/current/speed limits.
     *
     * Reinitializing flight after SET_CONFIG is acceptable during bench work
     * because the operator can run PAD_RESET immediately after changing values.
     */
    AmbarFlightConfig_t flight_config;
    const bool was_armed = AmbarFlight_IsArmed();

    (void)RocketSensors_ApplyOrientationConfig(s_config.imu_vertical_axis_index,
                                               s_config.imu_vertical_axis_sign);

    AmbarConfig_ApplyToFlightConfig(&s_config, &flight_config);
    AmbarFlight_Init(&flight_config);
    AmbarFlight_SetArmed(was_armed);

    if (reset_estimator != 0U)
    {
        AmbarFlight_ResetOnPad((float)HAL_GetTick() * 0.001f);
    }

    (void)AmbarActuator_ApplyConfig(&s_actuator_state, &s_config.actuator);
    /* END AMBAR BENCH-GATED EXPANSION - CONFIG APPLY POINT */
}

static bool ambar_set_config_value(const char *key, float value, char *response, size_t response_len)
{
    if (key == NULL || response == NULL || response_len == 0U)
    {
        return false;
    }

    if (strcmp(key, "TARGET_APOGEE_M") == 0)
    {
        s_config.flight.controller.target_apogee_m = value;
    }
    else if (strcmp(key, "TARGET_APOGEE_FT") == 0)
    {
        s_config.flight.controller.target_apogee_m = value * AMBAR_FEET_TO_METERS;
    }
    else if (strcmp(key, "TARGET_TOLERANCE_M") == 0)
    {
        s_config.flight.controller.apogee_tolerance_m = value;
    }
    else if (strcmp(key, "MAX_DEPLOY") == 0)
    {
        s_config.flight.controller.maximum_deploy_fraction = value;
    }
    else if (strcmp(key, "FULL_DEPLOY_ERROR_M") == 0)
    {
        s_config.flight.controller.full_deployment_error_m = value;
    }
    else if (strcmp(key, "MIN_DEPLOY_ALT_M") == 0)
    {
        s_config.flight.controller.minimum_deploy_altitude_m = value;
    }
    else if (strcmp(key, "DRAG_MASS_KG") == 0)
    {
        s_config.flight.apogee.vehicle_mass_kg = value;
    }
    else if (strcmp(key, "DRAG_AREA_M2") == 0 || strcmp(key, "DRAG_CD_AREA") == 0)
    {
        s_config.flight.apogee.drag_area_m2 = value;
    }
    else if (strcmp(key, "DRAG_RHO") == 0)
    {
        s_config.flight.apogee.air_density_kgpm3 = value;
    }
    else if (strcmp(key, "EFFECTIVENESS") == 0)
    {
        s_config.flight.apogee.actuator_effectiveness = value;
    }
    else if (strcmp(key, "APOGEE_MODE") == 0)
    {
        s_config.flight.apogee.mode =
            (value >= 0.5f) ? AMBAR_APOGEE_MODE_DRAG : AMBAR_APOGEE_MODE_BALLISTIC;
    }
    else if (strcmp(key, "VERT_AXIS") == 0)
    {
        s_config.imu_vertical_axis_index = (int32_t)value;
    }
    else if (strcmp(key, "VERT_SIGN") == 0)
    {
        s_config.imu_vertical_axis_sign = (value >= 0.0f) ? 1.0f : -1.0f;
    }
    else if (strcmp(key, "HOME_STEPS") == 0)
    {
        s_config.actuator.home_position_steps = (int32_t)value;
    }
    else if (strcmp(key, "MAX_STEPS") == 0)
    {
        s_config.actuator.max_extension_steps = (int32_t)value;
    }
    else if (strcmp(key, "RUN_CURRENT_MA") == 0)
    {
        s_config.actuator.run_current_ma = (uint16_t)value;
    }
    else if (strcmp(key, "HOLD_CURRENT_MA") == 0)
    {
        s_config.actuator.hold_current_ma = (uint16_t)value;
    }
    else if (strcmp(key, "MAX_VELOCITY") == 0)
    {
        s_config.actuator.max_velocity_steps_per_s = (uint32_t)value;
    }
    else if (strcmp(key, "MAX_ACCEL") == 0)
    {
        s_config.actuator.max_accel_steps_per_s2 = (uint32_t)value;
    }
    else
    {
        (void)snprintf(response, response_len, "NACK:BAD_KEY:%s", key);
        return false;
    }

    ambar_apply_runtime_config(1U);
    (void)snprintf(response, response_len, "ACK:SET_CONFIG:%s", key);
    return true;
}

static void ambar_execute_command(const AmbarCommandResult_t *command)
{
    char response[AMBAR_COMMAND_RESPONSE_LEN];
    HAL_StatusTypeDef status = HAL_OK;

    if (command == NULL)
    {
        return;
    }

    s_last_command_result = *command;
    strncpy(response, command->response, sizeof(response) - 1U);
    response[sizeof(response) - 1U] = '\0';

    switch (command->action)
    {
    case AMBAR_COMMAND_PING:
        (void)snprintf(response, sizeof(response), "ACK:PONG");
        break;

    case AMBAR_COMMAND_STATUS:
    {
        const AmbarFlightOutput_t output = AmbarFlight_GetOutput();
        (void)snprintf(response,
                       sizeof(response),
                       "STATUS:ARM=%u PH=%s ALT=%ld LOG=%lu ACT=%d",
                       output.armed ? 1U : 0U,
                       AmbarFlight_PhaseName(output.phase),
                       (long)ambar_s32_from_float(output.estimate.altitude_agl_m, 100.0f),
                       (unsigned long)AmbarLog_GetStatusFlags(),
                       (int)s_actuator_state.machine_state);
        break;
    }

    case AMBAR_COMMAND_PAD_RESET:
        status = RocketSensors_ResetPadReference(ROCKET_PAD_REFERENCE_DEFAULT_SAMPLES);
        if (status == HAL_OK)
        {
            const RocketSensorCalibrationStatus_t cal = RocketSensors_GetCalibrationStatus();
            s_config.has_stored_pad_reference = 1UL;
            s_config.stored_pad_vertical_accel_mps2 = cal.pad_vertical_accel_mps2;
            s_config.stored_pad_pressure_pa = cal.pad_pressure_pa;
            AmbarFlight_ResetOnPad((float)HAL_GetTick() * 0.001f);
            (void)snprintf(response, sizeof(response), "ACK:PAD_RESET");
        }
        else
        {
            (void)snprintf(response, sizeof(response), "NACK:PAD_RESET");
        }
        break;

    case AMBAR_COMMAND_ARM:
        AmbarFlight_SetArmed(true);
        (void)snprintf(response, sizeof(response), "ACK:ARM");
        break;

    case AMBAR_COMMAND_DISARM:
        AmbarFlight_SetArmed(false);
        (void)snprintf(response, sizeof(response), "ACK:DISARM");
        break;

    case AMBAR_COMMAND_ESTOP:
        AmbarFlight_SetArmed(false);
        AmbarActuator_EStop(&s_actuator_state);
        (void)snprintf(response, sizeof(response), "ACK:ESTOP");
        break;

    case AMBAR_COMMAND_RETRACT:
        if (AmbarActuator_RequestRetract(&s_actuator_state))
        {
            (void)snprintf(response, sizeof(response), "ACK:RETRACT");
        }
        else
        {
            (void)snprintf(response, sizeof(response), "NACK:RETRACT");
        }
        break;

    case AMBAR_COMMAND_HOME:
        if (AmbarActuator_RequestHome(&s_actuator_state))
        {
            (void)snprintf(response, sizeof(response), "ACK:HOME");
        }
        else
        {
            (void)snprintf(response, sizeof(response), "NACK:HOME_INHIBITED");
        }
        break;

    case AMBAR_COMMAND_BENCH_MOVE:
        if (AmbarActuator_RequestBenchMove(&s_actuator_state, command->steps))
        {
            (void)snprintf(response, sizeof(response), "ACK:BENCH_MOVE:%ld", (long)command->steps);
        }
        else
        {
            (void)snprintf(response, sizeof(response), "NACK:BENCH_MOVE");
        }
        break;

    case AMBAR_COMMAND_SET_CONFIG:
        if (!ambar_set_config_value(command->key, command->value, response, sizeof(response)))
        {
            status = HAL_ERROR;
        }
        break;

    case AMBAR_COMMAND_SAVE_CONFIG:
        s_config_save_status = AmbarConfig_SaveToFlash(&s_config);
        status = s_config_save_status;
        (void)snprintf(response,
                       sizeof(response),
                       status == HAL_OK ? "ACK:SAVE_CONFIG" : "NACK:SAVE_CONFIG");
        break;

    case AMBAR_COMMAND_START_LOG:
        s_log_status = AmbarLog_Start();
        status = s_log_status;
        (void)snprintf(response,
                       sizeof(response),
                       status == HAL_OK ? "ACK:START_LOG" : "NACK:START_LOG");
        break;

    case AMBAR_COMMAND_STOP_LOG:
        s_log_status = AmbarLog_Stop();
        (void)snprintf(response, sizeof(response), "ACK:STOP_LOG");
        break;

    case AMBAR_COMMAND_ERASE_LOG:
        s_log_status = AmbarLog_Erase();
        status = s_log_status;
        (void)snprintf(response,
                       sizeof(response),
                       status == HAL_OK ? "ACK:ERASE_LOG" : "NACK:ERASE_LOG");
        break;

    case AMBAR_COMMAND_NONE:
    default:
        status = HAL_ERROR;
        (void)snprintf(response, sizeof(response), "NACK:NO_ACTION");
        break;
    }

    if (status != HAL_OK)
    {
        s_last_command_result.accepted = false;
        s_last_command_result.ack = AMBAR_COMMAND_ACK_BAD_ARGUMENT;
    }

    strncpy(s_last_command_result.response,
            response,
            sizeof(s_last_command_result.response) - 1U);
    s_last_command_result.response[sizeof(s_last_command_result.response) - 1U] = '\0';

    (void)RadioBridge_SendText(response);
}

static AmbarTelemetryExtra_t ambar_build_telemetry_extra(const AmbarFlightOutput_t *output,
                                                         const AmbarActuatorDecision_t *decision)
{
    AmbarTelemetryExtra_t extra;
    RocketSensorCalibrationStatus_t cal = RocketSensors_GetCalibrationStatus();

    memset(&extra, 0, sizeof(extra));
    extra.config_flags = AmbarConfig_GetStatusFlags();
    extra.flash_log_flags =
        (s_flash_status == HAL_OK ? 1UL : 0UL) | (AmbarLog_GetStatusFlags() << 8);
    extra.command_action = (uint32_t)s_last_command_result.action;
    extra.command_ack = (uint32_t)s_last_command_result.ack;
    extra.calibration_flags =
        (cal.pad_reference_valid ? 1UL : 0UL)
        | (((uint32_t)(cal.vertical_axis_index & 0xFF)) << 8)
        | ((cal.vertical_axis_sign >= 0.0f ? 1UL : 2UL) << 16)
        | ((uint32_t)cal.imu_status_reg << 24);
    extra.actuator_state = (int32_t)s_actuator_state.machine_state;
    extra.actuator_target_steps =
        (decision != NULL) ? decision->target_position_steps : s_actuator_state.requested_position_steps;
    extra.actuator_actual_steps = s_actuator_state.actual_position_steps;
    extra.tmc_driver_status = (int32_t)s_actuator_state.last_driver_status;
    extra.tmc_diag_pins =
        (int32_t)((s_actuator_state.diag0_level ? 1U : 0U)
        | (s_actuator_state.diag1_level ? 2U : 0U));
    if (output != NULL)
    {
        extra.ballistic_apogee_cm = ambar_s32_from_float(output->ballistic_apogee_m, 100.0f);
        extra.drag_apogee_cm = ambar_s32_from_float(output->drag_apogee_m, 100.0f);
    }
    extra.drag_area_u_m2 = ambar_s32_from_float(s_config.flight.apogee.drag_area_m2, 1000000.0f);
    extra.actuator_effectiveness_milli =
        ambar_s32_from_float(s_config.flight.apogee.actuator_effectiveness, 1000.0f);

    return extra;
}

static void ambar_append_log_snapshot(const AmbarFlightOutput_t *output,
                                      const AmbarActuatorDecision_t *decision)
{
    if (output == NULL || decision == NULL || !AmbarLog_IsActive())
    {
        return;
    }

    AmbarLogSnapshot_t snapshot;
    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.timestamp_ms = HAL_GetTick();
    memcpy(snapshot.imu, s_raw_sensor_data.imu, sizeof(snapshot.imu));
    memcpy(snapshot.mag, s_raw_sensor_data.mag, sizeof(snapshot.mag));
    memcpy(snapshot.baro, s_raw_sensor_data.baro, sizeof(snapshot.baro));
    snapshot.altitude_m = output->estimate.altitude_agl_m;
    snapshot.velocity_mps = output->estimate.vertical_velocity_mps;
    snapshot.acceleration_mps2 = output->estimate.vertical_acceleration_mps2;
    snapshot.predicted_apogee_m = output->estimate.predicted_apogee_m;
    snapshot.ballistic_apogee_m = output->ballistic_apogee_m;
    snapshot.drag_apogee_m = output->drag_apogee_m;
    snapshot.deploy_fraction = output->airbrake_command.deploy_fraction;
    snapshot.pressure_pa = s_converted_sensor_data.pressure_pa;
    snapshot.temperature_c = s_converted_sensor_data.temperature_c;
    snapshot.phase = (uint32_t)output->phase;
    snapshot.flight_inhibit_flags = output->airbrake_command.inhibit_flags;
    snapshot.actuator_state = (uint32_t)s_actuator_state.machine_state;
    snapshot.actuator_inhibit_flags = decision->inhibit_flags;
    snapshot.command_action = (uint32_t)s_last_command_result.action;
    snapshot.command_ack = (uint32_t)s_last_command_result.ack;
    snapshot.actuator_target_steps = decision->target_position_steps;
    snapshot.actuator_actual_steps = s_actuator_state.actual_position_steps;

    s_log_status = AmbarLog_AppendSnapshot(&snapshot);
}

void AmbarApp_Init(HAL_StatusTypeDef sensor_status,
                   HAL_StatusTypeDef radio_status,
                   uint8_t motor_driver_ok)
{
    memset(&s_raw_sensor_data, 0, sizeof(s_raw_sensor_data));
    memset(&s_converted_sensor_data, 0, sizeof(s_converted_sensor_data));

    s_sensor_status = sensor_status;
    s_radio_status = radio_status;
    s_imu_status = HAL_ERROR;
    s_mag_status = HAL_ERROR;
    s_baro_status = HAL_ERROR;
    s_telemetry_status = HAL_ERROR;
    s_flash_status = HAL_ERROR;
    s_config_load_status = HAL_ERROR;
    s_config_save_status = HAL_ERROR;
    s_log_status = HAL_ERROR;
    memset(&s_last_command_result, 0, sizeof(s_last_command_result));
    memset(&s_last_actuator_decision, 0, sizeof(s_last_actuator_decision));

    s_actuator_state = AmbarActuator_DefaultState();
    s_actuator_state.motor_driver_ok = motor_driver_ok != 0U;

    /*
     * BEGIN AMBAR EKF PCB INTEGRATION - MOTOR SAFE BY DEFAULT
     *
     * Even if the TMC5240 answers on SPI, leave DRV_ENN in the disabled state.
     * The actuator module can only enable it when the build flag, homing state,
     * driver health, and flight command all allow movement.
     */
    TMC5240_SetDriverEnabled(0U);
    if (s_actuator_state.motor_driver_ok)
    {
        (void)TMC5240_ConfigureSafeDefaults();
    }
    /* END AMBAR EKF PCB INTEGRATION - MOTOR SAFE BY DEFAULT */

    /*
     * BEGIN AMBAR BENCH-GATED EXPANSION - FLASH CONFIG AND LOG STARTUP
     *
     * Load defaults first so a missing or corrupt flash record cannot leave the
     * flight app with uninitialized tuning values.  Then try flash config and
     * initialize the log ring only if the W25Q64 answers correctly.
     */
    AmbarConfig_LoadDefaults(&s_config);
    s_flash_status = W25Q64_Init(&s_flash_jedec);
    if (s_flash_status == HAL_OK)
    {
        s_config_load_status = AmbarConfig_LoadFromFlash(&s_config);
        s_log_status = AmbarLog_Init(1U);
    }
    else
    {
        s_config_load_status = HAL_ERROR;
        s_log_status = AmbarLog_Init(0U);
    }
    ambar_apply_runtime_config(0U);
    /* END AMBAR BENCH-GATED EXPANSION - FLASH CONFIG AND LOG STARTUP */

    if (sensor_status == HAL_OK)
    {
        /*
         * BEGIN AMBAR EKF PCB INTEGRATION - REQUIRED PAD RESET
         *
         * The EKF starts from the pad with altitude and velocity set to zero.
         * Pad reference capture must succeed before real IMU/barometer samples
         * are trusted by the scheduler below.
         */
        s_pad_reference_status =
            RocketSensors_ResetPadReference(ROCKET_PAD_REFERENCE_DEFAULT_SAMPLES);
        /* END AMBAR EKF PCB INTEGRATION - REQUIRED PAD RESET */
    }
    else
    {
        s_pad_reference_status = HAL_ERROR;
    }

    AmbarFlight_ResetOnPad((float)HAL_GetTick() * 0.001f);

    const uint32_t now = HAL_GetTick();
    s_last_imu_ms = now;
    s_last_baro_ms = now;
    s_last_telemetry_ms = now;
    s_last_log_ms = now;
}

void AmbarApp_Task(void)
{
    const uint32_t now = HAL_GetTick();
    const float timestamp_s = (float)now * 0.001f;

    /*
     * Keep the radio receive/heartbeat service active every pass through main.
     * The estimator work below is scheduled by elapsed time instead of HAL_Delay().
     */
    RadioBridge_Task();

    AmbarCommandResult_t command;
    while (RadioBridge_TakeCommand(&command))
    {
        ambar_execute_command(&command);
    }

    {
        const AmbarFlightOutput_t actuator_output = AmbarFlight_GetOutput();
        s_last_actuator_decision = AmbarActuator_Task(&s_actuator_state, &actuator_output);
    }

    if (RocketSensors_ConsumeImuDataReady()
        || (uint32_t)(now - s_last_imu_ms) >= AMBAR_IMU_PERIOD_MS)
    {
        /*
         * BEGIN AMBAR EKF PCB INTEGRATION - 100+ HZ IMU PROPAGATION
         *
         * A valid pad-zeroed vertical acceleration sample advances the EKF.  A
         * failed IMU read feeds NAN, which the EKF rejects and counts as a health
         * fault instead of silently propagating garbage.
         */
        s_last_imu_ms = now;
        s_imu_status =
            RocketSensors_ReadImuConverted(&s_raw_sensor_data, &s_converted_sensor_data);

        if (s_imu_status == HAL_OK && s_converted_sensor_data.pad_reference_valid)
        {
            (void)AmbarFlight_UpdateImu(timestamp_s,
                                        s_converted_sensor_data.vertical_acceleration_mps2);
        }
        else
        {
            (void)AmbarFlight_UpdateImu(timestamp_s, NAN);
        }
        /* END AMBAR EKF PCB INTEGRATION - 100+ HZ IMU PROPAGATION */
    }

    if (RocketSensors_ConsumeBarometerDataReady()
        || (uint32_t)(now - s_last_baro_ms) >= AMBAR_BARO_PERIOD_MS)
    {
        /*
         * BEGIN AMBAR EKF PCB INTEGRATION - 50 HZ BAROMETER CORRECTION
         *
         * The barometer path returns altitude above the pad.  The EKF applies
         * its innovation gate, so disconnected/noisy pressure data becomes a
         * rejected sample counter and an inhibit flag rather than a bad command.
         */
        s_last_baro_ms = now;
        s_baro_status =
            RocketSensors_ReadBarometerConverted(&s_raw_sensor_data,
                                                 &s_converted_sensor_data);

        if (s_baro_status == HAL_OK && s_converted_sensor_data.pad_reference_valid)
        {
            (void)AmbarFlight_UpdateBarometer(s_converted_sensor_data.altitude_agl_m,
                                              AMBAR_BARO_STDDEV_M);
        }
        else
        {
            (void)AmbarFlight_UpdateBarometer(NAN, AMBAR_BARO_STDDEV_M);
        }
        /* END AMBAR EKF PCB INTEGRATION - 50 HZ BAROMETER CORRECTION */
    }

    if ((uint32_t)(now - s_last_telemetry_ms) >= AMBAR_TELEMETRY_PERIOD_MS)
    {
        /*
         * BEGIN AMBAR EKF PCB INTEGRATION - 5 HZ FLIGHT TELEMETRY
         *
         * Raw fields remain in the packet for bring-up.  The added flight fields
         * carry altitude, velocity, acceleration, predicted apogee, phase, command,
         * inhibit flags, and rejected-sample counters as fixed-point signed ints.
         */
        s_last_telemetry_ms = now;
        s_mag_status = RocketSensors_ReadMagnetometerRaw(&s_raw_sensor_data);
        s_converted_sensor_data.magnetometer_valid = s_mag_status == HAL_OK;

        const AmbarFlightOutput_t flight_output = AmbarFlight_GetOutput();
        const AmbarActuatorDecision_t actuator_decision = s_last_actuator_decision;
        const AmbarTelemetryExtra_t extra =
            ambar_build_telemetry_extra(&flight_output, &actuator_decision);

        const uint8_t deployment_percent = ambar_deployment_percent(&flight_output);
        uint16_t calc_data[4] = {0};
        ambar_fill_legacy_calc(calc_data, &flight_output, deployment_percent);

        const char *message[3] = {
            ambar_status_message(&flight_output),
            s_last_command_result.response[0] != '\0'
                ? s_last_command_result.response
                : AmbarFlight_PhaseName(flight_output.phase),
            actuator_decision.inhibit_flags == AMBAR_ACTUATOR_INHIBIT_NONE
                ? ""
                : "ACTUATOR_INHIBITED"
        };

        if (s_radio_status == HAL_OK)
        {
            s_telemetry_status = PayloadPipelineWithFlight(
                (uint16_t *)s_raw_sensor_data.imu,
                s_raw_sensor_data.baro,
                (uint16_t *)s_raw_sensor_data.mag,
                calc_data,
                message,
                deployment_percent,
                &flight_output,
                actuator_decision.inhibit_flags,
                &extra
            );
        }
        else
        {
            s_telemetry_status = HAL_ERROR;
        }

        HAL_GPIO_WritePin(LED_5_GPIO_Port,
                          LED_5_Pin,
                          s_telemetry_status == HAL_OK ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_TogglePin(LED_6_GPIO_Port, LED_6_Pin);

        if ((uint32_t)(now - s_last_log_ms) >= AMBAR_LOG_PERIOD_MS)
        {
            s_last_log_ms = now;
            ambar_append_log_snapshot(&flight_output, &actuator_decision);
        }
        /* END AMBAR EKF PCB INTEGRATION - 5 HZ FLIGHT TELEMETRY */
    }
}

void AmbarApp_HandleExtiPin(uint16_t GPIO_Pin)
{
    AmbarActuator_HandleExtiPin(&s_actuator_state, GPIO_Pin);
}
