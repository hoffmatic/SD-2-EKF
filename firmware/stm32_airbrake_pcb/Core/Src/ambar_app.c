/**
 * @file ambar_app.c
 * @brief Cooperative integration layer for sensors, flight, communications, and motion.
 *
 * OVERVIEW
 * --------
 *   Main cooperative integration layer. It connects board I/O, the estimator,
 *   [ARCH-4] phase/controller output, USB/radio/flash services, and the [ARCH-5]
 *   actuator authority.
 *
 * HOW IT WORKS
 * ------------
 *   AmbarApp_Init() records hardware health, forces the motor energy-off, loads
 *   configuration, captures the pad reference, and resets flight state.
 *   AmbarApp_Task() then services each subsystem with HAL_GetTick() deadlines as
 *   described by [ARCH-1].
 *
 * SECTION MAP
 * -----------
 *   1. Scheduler constants and private singleton state
 *   2. Numeric, configuration, and shared command helpers
 *   3. Direct-USB telemetry, command cache, and simulation service
 *   4. Radio telemetry extras and persistent log snapshots
 *   5. Initialization, cooperative task, and EXTI forwarding
 *
 * SCHEDULING AND TUNING
 * ---------------------
 *   The AMBAR_*_PERIOD_MS constants below set scheduler rates. Feature identity
 *   and hardware-motion locks live in ambar_features.h; flight and actuator
 *   tuning live in their respective configuration structures.
 *
 * ASSUMPTIONS
 * -----------
 *   HAL_GetTick() is running and main.c has initialized every referenced driver.
 *   No service may add an unbounded delay because USB, estimation, and actuator
 *   progress monitoring share this cooperative loop.
 *
 * SAFETY BEHAVIOR
 * ---------------
 *   DISARM, SIM_STOP, stale simulation input, USB disconnect, ESTOP, and faults
 *   use AmbarActuator_StopAndCancel()/EStop() and remain energy-off. Normal
 *   healthy/armed recovery is passed to the actuator task so its one-shot,
 *   bounded automatic return-to-HOME policy can run.
 *
 * INTEGRATED SUBSYSTEMS
 * ---------------------
 *   Persistent settings, flash logging, radio commands/telemetry, direct USB
 *   protocol v2, presentation simulation input, and bench-gated actuator
 *   control all run through this cooperative scheduler.
 */

#include "ambar_app.h"

#include "ambar_actuator.h"
#include "ambar_command.h"
#include "ambar_config.h"
#include "ambar_features.h"
#include "ambar_flight.h"
#include "ambar_log.h"
#include "ambar_usb.h"
#include "radio_bridge.h"
#include "rocket_protocol.h"
#include "rocket_sensors.h"
#include "tmc5240.h"
#include "w25q64.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

/* -------------------- Cooperative scheduler and protocol limits -------------------- */

#define AMBAR_IMU_PERIOD_MS       8U
#define AMBAR_BARO_PERIOD_MS      20U
#define AMBAR_TELEMETRY_PERIOD_MS 1000U
#define AMBAR_LOG_PERIOD_MS       200U
#define AMBAR_BARO_STDDEV_M       1.5f
#define AMBAR_USB_TELEMETRY_PERIOD_MS 50U
#define AMBAR_USB_ACTUATOR_PERIOD_MS  100U
#define AMBAR_USB_HEARTBEAT_PERIOD_MS 1000U
#define AMBAR_SIMULATION_TIMEOUT_MS   500U
#define AMBAR_USB_COMMAND_CACHE_DEPTH 8U
#define AMBAR_USB_TARGET_APOGEE_MIN_DM 100U

/* -------------------- Private application state -------------------- */

typedef struct
{
    bool valid;
    uint16_t sequence;
    uint8_t command;
    uint8_t payload_length;
    uint8_t payload[ROCKET_COMMAND_DATA_MAX];
    RocketAckPayload ack;
} AmbarUsbCommandCacheEntry_t;

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
static uint32_t s_last_usb_telemetry_ms = 0U;
static uint32_t s_last_usb_actuator_ms = 0U;
static uint32_t s_last_usb_heartbeat_ms = 0U;
static bool s_usb_initialized = false;
static bool s_usb_was_configured = false;
static bool s_usb_snapshot_requested = false;
static bool s_usb_event_state_valid = false;
static uint16_t s_usb_previous_flags = 0U;
static uint8_t s_usb_previous_state = 0U;
static uint8_t s_usb_pending_message = ROCKET_MSG_NONE;
static AmbarUsbCommandCacheEntry_t
    s_usb_command_cache[AMBAR_USB_COMMAND_CACHE_DEPTH];
static uint8_t s_usb_command_cache_next = 0U;
static bool s_simulation_active = false;
static bool s_simulation_sample_valid = false;
static bool s_simulation_barometer_pending = false;
static bool s_simulation_stale = false;
static uint32_t s_last_simulation_sample_ms = 0U;
static RocketSimulationPayload s_simulation_sample;

/* -------------------- Shared numeric and presentation helpers -------------------- */

static bool ambar_actuator_motion_active(void)
{
    /*
     * Flash maintenance and pad-reference work may block the cooperative loop.
     * Treat any energized or pending motion as active so those operations cannot
     * starve actuator servicing while the mechanism is moving.
     */
    return s_actuator_state.driver_enabled
        || s_actuator_state.manual_request_pending != 0U
        || s_actuator_state.automatic_motion_active != 0U
        || s_actuator_state.automatic_deployment_latched != 0U
        || s_actuator_state.automatic_retract_active != 0U;
}

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
}

#if AMBAR_FEATURE_VERBOSE_STATUS_TEXT
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
#endif

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

static void ambar_refresh_config_crc(AmbarConfig_t *config)
{
    if (config == NULL)
    {
        return;
    }

    config->crc32 = 0UL;
    config->crc32 = AmbarConfig_Crc32((const uint8_t *)config, sizeof(*config));
}

/* -------------------- Runtime configuration fan-out ([ARCH-2], [ARCH-4], [ARCH-5]) -------------------- */

static void ambar_apply_runtime_config(uint8_t reset_estimator)
{
    /*
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
}

static bool ambar_set_config_value(const char *key, float value, char *response, size_t response_len)
{
    AmbarConfig_t candidate;

    if (key == NULL || response == NULL || response_len == 0U)
    {
        return false;
    }

    if (AmbarFlight_IsArmed()
        || s_actuator_state.driver_enabled
        || s_actuator_state.manual_request_pending != 0U)
    {
        (void)snprintf(response, response_len, "NACK:CONFIG_WHILE_ACTIVE");
        return false;
    }

#if AMBAR_FEATURE_PRESENTATION_MOTION
    if (strcmp(key, "HOME_STEPS") == 0
        || strcmp(key, "MAX_STEPS") == 0
        || strcmp(key, "RUN_CURRENT_MA") == 0
        || strcmp(key, "HOLD_CURRENT_MA") == 0
        || strcmp(key, "MAX_VELOCITY") == 0
        || strcmp(key, "MAX_ACCEL") == 0)
    {
        (void)snprintf(response, response_len, "NACK:PRESENTATION_PROFILE_LOCKED");
        return false;
    }
#endif

    candidate = s_config;

    if (strcmp(key, "TARGET_APOGEE_M") == 0)
    {
        candidate.flight.controller.target_apogee_m = value;
    }
    else if (strcmp(key, "TARGET_APOGEE_FT") == 0)
    {
        candidate.flight.controller.target_apogee_m = value * AMBAR_FEET_TO_METERS;
    }
    else if (strcmp(key, "TARGET_TOLERANCE_M") == 0)
    {
        candidate.flight.controller.apogee_tolerance_m = value;
    }
    else if (strcmp(key, "MAX_DEPLOY") == 0)
    {
        candidate.flight.controller.maximum_deploy_fraction = value;
    }
    else if (strcmp(key, "FULL_DEPLOY_ERROR_M") == 0)
    {
        candidate.flight.controller.full_deployment_error_m = value;
    }
    else if (strcmp(key, "MIN_DEPLOY_ALT_M") == 0)
    {
        candidate.flight.controller.minimum_deploy_altitude_m = value;
    }
    else if (strcmp(key, "DRAG_MASS_KG") == 0)
    {
        candidate.flight.apogee.vehicle_mass_kg = value;
    }
    else if (strcmp(key, "DRAG_AREA_M2") == 0 || strcmp(key, "DRAG_CD_AREA") == 0)
    {
        candidate.flight.apogee.drag_area_m2 = value;
    }
    else if (strcmp(key, "DRAG_RHO") == 0)
    {
        candidate.flight.apogee.air_density_kgpm3 = value;
    }
    else if (strcmp(key, "EFFECTIVENESS") == 0)
    {
        candidate.flight.apogee.actuator_effectiveness = value;
    }
    else if (strcmp(key, "APOGEE_MODE") == 0)
    {
        candidate.flight.apogee.mode =
            (value >= 0.5f) ? AMBAR_APOGEE_MODE_DRAG : AMBAR_APOGEE_MODE_BALLISTIC;
    }
    else if (strcmp(key, "VERT_AXIS") == 0)
    {
        candidate.imu_vertical_axis_index = (int32_t)value;
    }
    else if (strcmp(key, "VERT_SIGN") == 0)
    {
        candidate.imu_vertical_axis_sign = (value >= 0.0f) ? 1.0f : -1.0f;
    }
    else if (strcmp(key, "HOME_STEPS") == 0)
    {
        candidate.actuator.home_position_steps = (int32_t)value;
    }
    else if (strcmp(key, "MAX_STEPS") == 0)
    {
        candidate.actuator.max_extension_steps = (int32_t)value;
    }
    else if (strcmp(key, "RUN_CURRENT_MA") == 0)
    {
        candidate.actuator.run_current_ma = (uint16_t)value;
    }
    else if (strcmp(key, "HOLD_CURRENT_MA") == 0)
    {
        candidate.actuator.hold_current_ma = (uint16_t)value;
    }
    else if (strcmp(key, "MAX_VELOCITY") == 0)
    {
        candidate.actuator.max_velocity_steps_per_s = (uint32_t)value;
    }
    else if (strcmp(key, "MAX_ACCEL") == 0)
    {
        candidate.actuator.max_accel_steps_per_s2 = (uint32_t)value;
    }
    else
    {
        (void)snprintf(response, response_len, "NACK:BAD_KEY:%s", key);
        return false;
    }

    ambar_refresh_config_crc(&candidate);
    if (!AmbarConfig_Validate(&candidate))
    {
        (void)snprintf(response, response_len, "NACK:BAD_VALUE:%s", key);
        return false;
    }

    s_config = candidate;
    ambar_apply_runtime_config(1U);
    (void)snprintf(response, response_len, "ACK:SET_CONFIG:%s", key);
    return true;
}

static HAL_StatusTypeDef ambar_execute_command(const AmbarCommandResult_t *command,
                                                bool send_radio_response)
{
    char response[AMBAR_COMMAND_RESPONSE_LEN];
    HAL_StatusTypeDef status = HAL_OK;

    if (command == NULL)
    {
        return HAL_ERROR;
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
                       "STATUS:ARM=%u PH=%s ALT=%ld LOG=%lu ACT=%d F=%03lX",
                       output.armed ? 1U : 0U,
                       AmbarFlight_PhaseName(output.phase),
                       (long)ambar_s32_from_float(output.estimate.altitude_agl_m, 100.0f),
                       (unsigned long)AmbarLog_GetStatusFlags(),
                       (int)s_actuator_state.machine_state,
                       (unsigned long)AMBAR_FEATURE_ENABLED_BITS);
        break;
    }

    case AMBAR_COMMAND_PAD_RESET:
        if (AmbarFlight_IsArmed()
            || s_simulation_active
            || ambar_actuator_motion_active())
        {
            status = HAL_BUSY;
            (void)snprintf(response, sizeof(response), "NACK:PAD_RESET_BUSY");
        }
        else
        {
            status = RocketSensors_ResetPadReference(ROCKET_PAD_REFERENCE_DEFAULT_SAMPLES);
            if (status == HAL_OK)
            {
                const RocketSensorCalibrationStatus_t cal =
                    RocketSensors_GetCalibrationStatus();
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
        }
        break;

    case AMBAR_COMMAND_ARM:
        if (s_simulation_active && !s_simulation_sample_valid)
        {
            status = HAL_BUSY;
            (void)snprintf(response, sizeof(response), "NACK:SIM_SAMPLE_REQUIRED");
        }
#if AMBAR_FEATURE_ACTUATOR
        else if (!AmbarActuator_IsReadyForFlight(&s_actuator_state))
        {
            status = HAL_BUSY;
            s_actuator_state.last_inhibit_flags =
                s_last_actuator_decision.inhibit_flags;
            (void)snprintf(response, sizeof(response), "NACK:ACTUATOR_NOT_READY");
        }
#endif
        else
        {
            AmbarFlight_SetArmed(true);
            (void)snprintf(response, sizeof(response), "ACK:ARM");
        }
        break;

    case AMBAR_COMMAND_DISARM:
        AmbarFlight_SetArmed(false);
        (void)AmbarActuator_StopAndCancel(&s_actuator_state);
        (void)snprintf(response, sizeof(response), "ACK:DISARM");
        break;

    case AMBAR_COMMAND_ESTOP:
        AmbarFlight_SetArmed(false);
        AmbarActuator_EStop(&s_actuator_state);
        (void)snprintf(response, sizeof(response), "ACK:ESTOP");
        break;

    case AMBAR_COMMAND_RETRACT:
        if (AmbarFlight_IsArmed())
        {
            status = HAL_BUSY;
            (void)snprintf(response, sizeof(response), "NACK:RETRACT_WHILE_ARMED");
        }
        else
        {
#if AMBAR_FEATURE_EFFECTIVE_BENCH_COMMANDS
            if (AmbarActuator_RequestRetract(&s_actuator_state))
            {
                (void)snprintf(response, sizeof(response), "ACK:RETRACT");
            }
            else
            {
                status = HAL_ERROR;
                (void)snprintf(response, sizeof(response), "NACK:RETRACT");
            }
#else
            status = HAL_ERROR;
            (void)snprintf(response, sizeof(response), "NACK:BENCH_DISABLED");
#endif
        }
        break;

    case AMBAR_COMMAND_HOME:
        if (AmbarFlight_IsArmed())
        {
            status = HAL_BUSY;
            (void)snprintf(response, sizeof(response), "NACK:HOME_WHILE_ARMED");
        }
        else
        {
#if AMBAR_FEATURE_EFFECTIVE_BENCH_COMMANDS
            if (AmbarActuator_RequestHome(&s_actuator_state))
            {
                (void)snprintf(response, sizeof(response), "ACK:HOME");
            }
            else
            {
                status = HAL_ERROR;
                (void)snprintf(response, sizeof(response), "NACK:HOME_INHIBITED");
            }
#else
            status = HAL_ERROR;
            (void)snprintf(response, sizeof(response), "NACK:BENCH_DISABLED");
#endif
        }
        break;

    case AMBAR_COMMAND_BENCH_MOVE:
        if (AmbarFlight_IsArmed())
        {
            status = HAL_BUSY;
            (void)snprintf(response, sizeof(response), "NACK:BENCH_MOVE_WHILE_ARMED");
        }
        else
        {
#if AMBAR_FEATURE_EFFECTIVE_BENCH_COMMANDS
            if (AmbarActuator_RequestBenchMove(&s_actuator_state, command->steps))
            {
                (void)snprintf(response,
                               sizeof(response),
                               "ACK:BENCH_MOVE:%ld",
                               (long)command->steps);
            }
            else
            {
                status = HAL_ERROR;
                (void)snprintf(response, sizeof(response), "NACK:BENCH_MOVE");
            }
#else
            status = HAL_ERROR;
            (void)snprintf(response, sizeof(response), "NACK:BENCH_DISABLED");
#endif
        }
        break;

    case AMBAR_COMMAND_SET_CONFIG:
        if (!ambar_set_config_value(command->key, command->value, response, sizeof(response)))
        {
            status = HAL_ERROR;
        }
        break;

    case AMBAR_COMMAND_SAVE_CONFIG:
        if (AmbarFlight_IsArmed() || ambar_actuator_motion_active())
        {
            status = HAL_BUSY;
            (void)snprintf(response, sizeof(response), "NACK:SAVE_CONFIG_WHILE_ACTIVE");
        }
        else
        {
            s_config_save_status = AmbarConfig_SaveToFlash(&s_config);
            status = s_config_save_status;
            (void)snprintf(response,
                           sizeof(response),
                           status == HAL_OK ? "ACK:SAVE_CONFIG" : "NACK:SAVE_CONFIG");
        }
        break;

    case AMBAR_COMMAND_START_LOG:
#if AMBAR_FEATURE_FLASH_LOGGING
        s_log_status = AmbarLog_Start();
        status = s_log_status;
        (void)snprintf(response,
                       sizeof(response),
                       status == HAL_OK ? "ACK:START_LOG" : "NACK:START_LOG");
#else
        status = HAL_ERROR;
        (void)snprintf(response, sizeof(response), "NACK:LOG_DISABLED");
#endif
        break;

    case AMBAR_COMMAND_STOP_LOG:
#if AMBAR_FEATURE_FLASH_LOGGING
        s_log_status = AmbarLog_Stop();
        (void)snprintf(response, sizeof(response), "ACK:STOP_LOG");
#else
        status = HAL_ERROR;
        (void)snprintf(response, sizeof(response), "NACK:LOG_DISABLED");
#endif
        break;

    case AMBAR_COMMAND_ERASE_LOG:
        if (AmbarFlight_IsArmed() || ambar_actuator_motion_active())
        {
            status = HAL_BUSY;
            (void)snprintf(response, sizeof(response), "NACK:ERASE_LOG_WHILE_ACTIVE");
        }
        else
        {
#if AMBAR_FEATURE_FLASH_LOGGING
#if AMBAR_FEATURE_WATCHDOG
            /* A full 1 MiB erase can exceed the watchdog window by design. */
            status = HAL_ERROR;
            (void)snprintf(response, sizeof(response), "NACK:ERASE_WDG_ACTIVE");
#else
            s_log_status = AmbarLog_Erase();
            status = s_log_status;
            (void)snprintf(response,
                           sizeof(response),
                           status == HAL_OK ? "ACK:ERASE_LOG" : "NACK:ERASE_LOG");
#endif
#else
            status = HAL_ERROR;
            (void)snprintf(response, sizeof(response), "NACK:LOG_DISABLED");
#endif
        }
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

#if AMBAR_FEATURE_RADIO
    if (send_radio_response)
    {
        (void)RadioBridge_SendText(response);
    }
#else
    (void)send_radio_response;
#endif

    return status;
}

/* -------------------- Direct-USB telemetry encoding ([ARCH-3], [ARCH-6]) -------------------- */

static int16_t ambar_i16_from_float(float value, float scale)
{
    if (!isfinite(value))
    {
        return 0;
    }

    const float scaled = value * scale;
    if (scaled >= 32767.0f)
    {
        return INT16_MAX;
    }
    if (scaled <= -32768.0f)
    {
        return INT16_MIN;
    }
    return (int16_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

static uint16_t ambar_u16_saturating_sum(uint32_t first, uint32_t second)
{
    if (first >= UINT16_MAX || second >= UINT16_MAX - first)
    {
        return UINT16_MAX;
    }
    return (uint16_t)(first + second);
}

static uint8_t ambar_usb_status_code(const AmbarFlightOutput_t *output)
{
    if (s_simulation_stale)
    {
        return ROCKET_STATUS_SIMULATION_STALE;
    }
    if (!s_simulation_active && s_sensor_status != HAL_OK)
    {
        return ROCKET_STATUS_SENSOR_INIT_FAILED;
    }
    if (output == NULL || !output->estimate.healthy)
    {
        return ROCKET_STATUS_ESTIMATOR_INVALID;
    }
    if (s_imu_status != HAL_OK || s_baro_status != HAL_OK)
    {
        return ROCKET_STATUS_SENSOR_READ_FAILED;
    }
#if AMBAR_FEATURE_RADIO
    if (s_radio_status != HAL_OK)
    {
        return ROCKET_STATUS_RADIO_ERROR;
    }
#endif
    return ROCKET_STATUS_OK;
}

static uint16_t ambar_usb_telemetry_flags(const AmbarFlightOutput_t *output)
{
    uint16_t flags = 0U;

    if (output == NULL)
    {
        return ROCKET_FLAG_SENSOR_FAULT;
    }
    if (output->estimate.initialized)
    {
        flags |= ROCKET_FLAG_INITIALIZED;
    }
    if (s_baro_status == HAL_OK
        && (s_simulation_active || s_converted_sensor_data.pad_reference_valid))
    {
        flags |= ROCKET_FLAG_BARO_VALID;
    }
    if (output->estimate.healthy)
    {
        flags |= ROCKET_FLAG_EKF_VALID;
    }
    if (output->armed)
    {
        flags |= ROCKET_FLAG_ARMED;
    }
    if (!output->airbrake_command.inhibit
        && output->airbrake_command.deploy_fraction > 0.0f)
    {
        flags |= ROCKET_FLAG_CONTROLLER_ACTIVE;
    }
    if (output->phase == AMBAR_PHASE_RECOVERY)
    {
        flags |= ROCKET_FLAG_APOGEE_REACHED;
    }
    if (s_imu_status != HAL_OK)
    {
        flags |= ROCKET_FLAG_ACCEL_IGNORED;
    }
#if !AMBAR_FEATURE_MAGNETOMETER_TELEMETRY
    flags |= ROCKET_FLAG_MAG_IGNORED;
#endif
    if ((!s_simulation_active && s_sensor_status != HAL_OK)
        || s_imu_status != HAL_OK
        || s_baro_status != HAL_OK
        || !output->estimate.healthy
        || s_simulation_stale)
    {
        flags |= ROCKET_FLAG_SENSOR_FAULT;
    }
    if (s_simulation_active)
    {
        flags |= ROCKET_FLAG_SIMULATION_ACTIVE;
    }
    return flags;
}

static uint8_t ambar_usb_phase_message(uint8_t previous_state, uint8_t current_state)
{
    const uint8_t previous_phase = previous_state & 0x0FU;
    const uint8_t current_phase = current_state & 0x0FU;

    if (previous_phase == AMBAR_PHASE_PAD_IDLE && current_phase == AMBAR_PHASE_BOOST)
    {
        return ROCKET_MSG_LAUNCH_DETECTED;
    }
    if (previous_phase == AMBAR_PHASE_BOOST
        && (current_phase == AMBAR_PHASE_COAST
            || current_phase == AMBAR_PHASE_AIRBRAKE_ACTIVE))
    {
        return ROCKET_MSG_BURNOUT_DETECTED;
    }
    if (current_phase == AMBAR_PHASE_AIRBRAKE_ACTIVE
        && previous_phase != AMBAR_PHASE_AIRBRAKE_ACTIVE)
    {
        return ROCKET_MSG_AIRBRAKE_DEPLOYED;
    }
    if (current_phase == AMBAR_PHASE_RECOVERY && previous_phase != AMBAR_PHASE_RECOVERY)
    {
        return ROCKET_MSG_APOGEE_REACHED;
    }
    return ROCKET_MSG_NONE;
}

static RocketTelemetryPayload ambar_build_usb_telemetry(const AmbarFlightOutput_t *output,
                                                         uint8_t message_code)
{
    RocketTelemetryPayload payload;
    uint8_t imu_health;
    uint8_t barometer_health;
    uint8_t magnetometer_health;
    uint8_t actuator_health;

    memset(&payload, 0, sizeof(payload));
    if (output == NULL)
    {
        payload.status_code = ROCKET_STATUS_UNKNOWN;
        return payload;
    }

    payload.flags = ambar_usb_telemetry_flags(output);
    payload.state = (uint8_t)output->phase & 0x0FU;
    payload.status_code = ambar_usb_status_code(output);
    payload.altitude_dm = ambar_i16_from_float(output->estimate.altitude_agl_m, 10.0f);
    payload.velocity_cms = ambar_i16_from_float(output->estimate.vertical_velocity_mps, 100.0f);
    payload.acceleration_cms2 =
        ambar_i16_from_float(output->estimate.vertical_acceleration_mps2, 100.0f);
    payload.predicted_apogee_dm =
        ambar_u16_from_float(output->estimate.predicted_apogee_m, 10.0f);
    payload.target_apogee_dm =
        ambar_u16_from_float(output->airbrake_command.target_apogee_m, 10.0f);
    payload.deployment_percent = ambar_deployment_percent(output);

    imu_health = s_imu_status == HAL_OK ? ROCKET_SENSOR_OK : ROCKET_SENSOR_FAULT;
    barometer_health = s_baro_status == HAL_OK ? ROCKET_SENSOR_OK : ROCKET_SENSOR_FAULT;
    if (s_simulation_active && !s_simulation_sample_valid)
    {
        imu_health = ROCKET_SENSOR_STALE;
        barometer_health = ROCKET_SENSOR_STALE;
    }
    if (s_simulation_stale)
    {
        imu_health = ROCKET_SENSOR_STALE;
        barometer_health = ROCKET_SENSOR_STALE;
    }
#if AMBAR_FEATURE_MAGNETOMETER_TELEMETRY
    magnetometer_health =
        s_mag_status == HAL_OK ? ROCKET_SENSOR_OK : ROCKET_SENSOR_UNKNOWN;
#else
    magnetometer_health = ROCKET_SENSOR_UNKNOWN;
#endif
#if AMBAR_FEATURE_ACTUATOR
    actuator_health = s_actuator_state.motor_driver_ok
        ? ROCKET_SENSOR_OK
        : ROCKET_SENSOR_FAULT;
#else
    actuator_health = ROCKET_SENSOR_UNKNOWN;
#endif
    payload.sensor_health = RocketProtocol_PackSensorHealth(imu_health,
                                                            barometer_health,
                                                            magnetometer_health,
                                                            actuator_health);
    payload.failed_reads = ambar_u16_saturating_sum(
        output->health.rejected_imu_samples,
        output->health.rejected_barometer_samples);
    payload.message_code = message_code;
    return payload;
}

static void ambar_usb_send_telemetry(uint8_t requested_message)
{
    const AmbarFlightOutput_t output = AmbarFlight_GetOutput();
    uint8_t message = requested_message;
    uint8_t encoded[ROCKET_TELEMETRY_PAYLOAD_SIZE];

    if (message == ROCKET_MSG_NONE)
    {
        message = s_usb_pending_message;
    }
    if (message == ROCKET_MSG_NONE && s_usb_event_state_valid)
    {
        message = ambar_usb_phase_message(s_usb_previous_state, (uint8_t)output.phase);
    }

    const RocketTelemetryPayload telemetry = ambar_build_usb_telemetry(&output, message);
    const size_t length = RocketProtocol_EncodeTelemetryPayload(encoded,
                                                                 sizeof(encoded),
                                                                 &telemetry);
    const bool queued = length != 0U
        && AmbarUsb_QueuePacket(ROCKET_PKT_TELEMETRY,
                                encoded,
                                length,
                                HAL_GetTick());

    if (queued
        && (!s_usb_event_state_valid
            || telemetry.flags != s_usb_previous_flags
            || telemetry.state != s_usb_previous_state
            || message != ROCKET_MSG_NONE))
    {
        RocketEventPayload event;
        uint8_t event_bytes[ROCKET_EVENT_PAYLOAD_SIZE];

        event.changed_flags = s_usb_event_state_valid
            ? (uint16_t)(telemetry.flags ^ s_usb_previous_flags)
            : telemetry.flags;
        event.current_flags = telemetry.flags;
        event.previous_state = s_usb_event_state_valid
            ? s_usb_previous_state
            : telemetry.state;
        event.current_state = telemetry.state;
        event.status_code = telemetry.status_code;
        event.message_code = message;
        event.detail = (uint16_t)(output.airbrake_command.inhibit_flags & 0xFFFFU);

        const size_t event_length = RocketProtocol_EncodeEventPayload(event_bytes,
                                                                      sizeof(event_bytes),
                                                                      &event);
        if (event_length != 0U)
        {
            (void)AmbarUsb_QueuePacket(ROCKET_PKT_EVENT,
                                       event_bytes,
                                       event_length,
                                       HAL_GetTick());
        }
    }

    if (queued)
    {
        s_usb_previous_flags = telemetry.flags;
        s_usb_previous_state = telemetry.state;
        s_usb_event_state_valid = true;
        s_usb_pending_message = ROCKET_MSG_NONE;
    }
}

static void ambar_usb_send_actuator_status(void)
{
    const AmbarFlightOutput_t output = AmbarFlight_GetOutput();
    RocketActuatorStatusPayload payload;
    uint8_t encoded[ROCKET_ACTUATOR_STATUS_PAYLOAD_SIZE];

    memset(&payload, 0, sizeof(payload));
    payload.actuator_inhibit_flags = s_last_actuator_decision.inhibit_flags;
    payload.flight_inhibit_flags = output.airbrake_command.inhibit_flags;
    payload.target_steps = s_last_actuator_decision.target_position_steps;
    payload.actual_steps = s_actuator_state.actual_position_steps;
    payload.driver_status = s_actuator_state.last_driver_status;
    payload.machine_state = (uint8_t)s_actuator_state.machine_state;
#if AMBAR_FEATURE_ACTUATOR
    payload.flags |= ROCKET_ACTUATOR_FLAG_BUILD_ENABLED;
#endif
#if AMBAR_FEATURE_EFFECTIVE_BENCH_COMMANDS
    payload.flags |= ROCKET_ACTUATOR_FLAG_BENCH_ENABLED;
#endif
    if (s_actuator_state.homed)
    {
        payload.flags |= ROCKET_ACTUATOR_FLAG_HOMED;
    }
    if (s_actuator_state.motor_driver_ok)
    {
        payload.flags |= ROCKET_ACTUATOR_FLAG_DRIVER_OK;
    }
    if (s_actuator_state.driver_enabled)
    {
        payload.flags |= ROCKET_ACTUATOR_FLAG_DRIVER_ENABLED;
    }
    if (s_actuator_state.estop_latched)
    {
        payload.flags |= ROCKET_ACTUATOR_FLAG_ESTOP;
    }
    if (s_actuator_state.config_valid)
    {
        payload.flags |= ROCKET_ACTUATOR_FLAG_CONFIG_VALID;
    }
    if (s_actuator_state.manual_request_pending != 0U)
    {
        payload.flags |= ROCKET_ACTUATOR_FLAG_MANUAL_PENDING;
    }

    const size_t length = RocketProtocol_EncodeActuatorStatusPayload(encoded,
                                                                      sizeof(encoded),
                                                                      &payload);
    if (length != 0U)
    {
        (void)AmbarUsb_QueuePacket(ROCKET_PKT_ACTUATOR_STATUS,
                                   encoded,
                                   length,
                                   HAL_GetTick());
    }
}

static void ambar_usb_send_heartbeat(void)
{
    const AmbarUsbStats_t stats = AmbarUsb_GetStats();
    RocketHeartbeatPayload payload;
    uint8_t encoded[ROCKET_HEARTBEAT_PAYLOAD_SIZE];

    payload.feature_flags = AMBAR_FEATURE_ENABLED_BITS;
    payload.receive_errors = ambar_u16_saturating_sum(stats.receive_errors,
                                                      stats.receive_queue_drops);
    payload.transmit_drops = stats.transmit_drops >= UINT16_MAX
        ? UINT16_MAX
        : (uint16_t)stats.transmit_drops;

    const size_t length = RocketProtocol_EncodeHeartbeatPayload(encoded,
                                                                 sizeof(encoded),
                                                                 &payload);
    if (length != 0U)
    {
        (void)AmbarUsb_QueuePacket(ROCKET_PKT_HEARTBEAT,
                                   encoded,
                                   length,
                                   HAL_GetTick());
    }
}

/* -------------------- Direct-USB command cache and dispatch -------------------- */

static void ambar_usb_reset_command_cache(void)
{
    memset(s_usb_command_cache, 0, sizeof(s_usb_command_cache));
    s_usb_command_cache_next = 0U;
}

static const AmbarUsbCommandCacheEntry_t *ambar_usb_find_cached_command(
    uint16_t sequence)
{
    for (uint8_t i = 0U; i < AMBAR_USB_COMMAND_CACHE_DEPTH; ++i)
    {
        if (s_usb_command_cache[i].valid
            && s_usb_command_cache[i].sequence == sequence)
        {
            return &s_usb_command_cache[i];
        }
    }

    return NULL;
}

static bool ambar_usb_cached_command_matches(
    const AmbarUsbCommandCacheEntry_t *cached,
    const RocketCommandPayload *command)
{
    if (cached == NULL || command == NULL
        || cached->command != command->command
        || cached->payload_length != command->payload_length)
    {
        return false;
    }

    return command->payload_length == 0U
        || memcmp(cached->payload, command->payload, command->payload_length) == 0;
}

static void ambar_usb_queue_ack(const RocketAckPayload *ack,
                                const RocketCommandPayload *cache_command)
{
    uint8_t encoded[ROCKET_ACK_PAYLOAD_SIZE];

    if (ack == NULL)
    {
        return;
    }

    const size_t length = RocketProtocol_EncodeAckPayload(encoded, sizeof(encoded), ack);

    if (length != 0U)
    {
        (void)AmbarUsb_QueuePacket(ROCKET_PKT_ACK, encoded, length, HAL_GetTick());
    }

    if (cache_command != NULL)
    {
        AmbarUsbCommandCacheEntry_t *cached =
            &s_usb_command_cache[s_usb_command_cache_next];
        memset(cached, 0, sizeof(*cached));
        cached->valid = true;
        cached->sequence = ack->command_sequence;
        cached->command = cache_command->command;
        cached->payload_length = cache_command->payload_length;
        if (cache_command->payload_length != 0U)
        {
            memcpy(cached->payload,
                   cache_command->payload,
                   cache_command->payload_length);
        }
        cached->ack = *ack;
        s_usb_command_cache_next = (uint8_t)(
            (s_usb_command_cache_next + 1U) % AMBAR_USB_COMMAND_CACHE_DEPTH);
    }
}

static void ambar_usb_complete_command(uint16_t sequence,
                                       uint8_t command,
                                       RocketAckCode result,
                                       uint16_t detail,
                                       const RocketCommandPayload *cache_command)
{
    RocketAckPayload ack;
    ack.command_sequence = sequence;
    ack.command = command;
    ack.result = (uint8_t)result;
    ack.detail = detail;
    ambar_usb_queue_ack(&ack, cache_command);
}

static void ambar_usb_prepare_app_command(AmbarCommandResult_t *request,
                                          AmbarCommandAction_t action)
{
    memset(request, 0, sizeof(*request));
    request->action = action;
    request->ack = AMBAR_COMMAND_ACK_OK;
    request->accepted = true;
}

static void ambar_usb_process_command(const RocketDecodedPacket *packet)
{
    RocketCommandPayload command;
    RocketAckCode result = ROCKET_ACK_OK;
    uint16_t detail = 0U;
    bool execute_app_command = false;
    AmbarCommandResult_t request;

    if (!RocketProtocol_DecodeCommandPayload(packet->payload,
                                             packet->payload_length,
                                             &command))
    {
        const uint8_t code = packet->payload_length != 0U
            ? packet->payload[0]
            : 0xFFU;
        ambar_usb_complete_command(packet->header.sequence,
                                   code,
                                   ROCKET_ACK_BAD_LENGTH,
                                   packet->payload_length,
                                   NULL);
        return;
    }

    const AmbarUsbCommandCacheEntry_t *cached =
        ambar_usb_find_cached_command(packet->header.sequence);
    if (cached != NULL)
    {
        if (ambar_usb_cached_command_matches(cached, &command))
        {
            ambar_usb_queue_ack(&cached->ack, NULL);
        }
        else
        {
            ambar_usb_complete_command(packet->header.sequence,
                                       command.command,
                                       ROCKET_ACK_BAD_VALUE,
                                       0xD001U,
                                       NULL);
        }
        return;
    }

    ambar_usb_prepare_app_command(&request, AMBAR_COMMAND_NONE);

    switch (command.command)
    {
    case ROCKET_CMD_NOP:
        if (command.payload_length != 0U)
        {
            result = ROCKET_ACK_BAD_LENGTH;
        }
        break;

    case ROCKET_CMD_PING:
        if (command.payload_length == 0U)
        {
            request.action = AMBAR_COMMAND_PING;
            execute_app_command = true;
        }
        else
        {
            result = ROCKET_ACK_BAD_LENGTH;
        }
        break;

    case ROCKET_CMD_REQUEST_SNAPSHOT:
        if (command.payload_length == 0U)
        {
            s_usb_snapshot_requested = true;
        }
        else
        {
            result = ROCKET_ACK_BAD_LENGTH;
        }
        break;

    case ROCKET_CMD_SET_TARGET_APOGEE:
        if (command.payload_length != 2U)
        {
            result = ROCKET_ACK_BAD_LENGTH;
        }
        else if (AmbarFlight_IsArmed())
        {
            result = ROCKET_ACK_BUSY;
        }
        else if (RocketProtocol_ReadU16(command.payload)
                 < AMBAR_USB_TARGET_APOGEE_MIN_DM)
        {
            result = ROCKET_ACK_BAD_VALUE;
            detail = RocketProtocol_ReadU16(command.payload);
        }
        else
        {
            request.action = AMBAR_COMMAND_SET_CONFIG;
            request.value = (float)RocketProtocol_ReadU16(command.payload) * 0.1f;
            (void)strncpy(request.key, "TARGET_APOGEE_M", sizeof(request.key) - 1U);
            execute_app_command = true;
        }
        break;

    case ROCKET_CMD_SET_CONTROLLER:
        if (command.payload_length != 1U)
        {
            result = ROCKET_ACK_BAD_LENGTH;
        }
        else if (command.payload[0] > 1U)
        {
            result = ROCKET_ACK_BAD_VALUE;
        }
        else if (command.payload[0] != 0U
                 && s_simulation_active
                 && !s_simulation_sample_valid)
        {
            result = ROCKET_ACK_BUSY;
        }
        else
        {
            request.action = command.payload[0] != 0U
                ? AMBAR_COMMAND_ARM
                : AMBAR_COMMAND_DISARM;
            execute_app_command = true;
        }
        break;

    case ROCKET_CMD_SET_MODE:
    case ROCKET_CMD_RETURN_STANDARD:
    case ROCKET_CMD_MANUAL_AIRBRAKE:
        result = ROCKET_ACK_UNSUPPORTED;
        break;

    case ROCKET_CMD_ESTOP:
        if (command.payload_length == 0U)
        {
            request.action = AMBAR_COMMAND_ESTOP;
            execute_app_command = true;
        }
        else
        {
            result = ROCKET_ACK_BAD_LENGTH;
        }
        break;

    case ROCKET_CMD_HOME:
    case ROCKET_CMD_RETRACT:
    case ROCKET_CMD_BENCH_MOVE_STEPS:
        if ((command.command == ROCKET_CMD_BENCH_MOVE_STEPS
             && command.payload_length != 4U)
            || (command.command != ROCKET_CMD_BENCH_MOVE_STEPS
                && command.payload_length != 0U))
        {
            result = ROCKET_ACK_BAD_LENGTH;
        }
        else if (AmbarFlight_IsArmed())
        {
            result = ROCKET_ACK_BUSY;
        }
#if AMBAR_FEATURE_EFFECTIVE_BENCH_COMMANDS
        else
        {
            request.action = command.command == ROCKET_CMD_HOME
                ? AMBAR_COMMAND_HOME
                : (command.command == ROCKET_CMD_RETRACT
                    ? AMBAR_COMMAND_RETRACT
                    : AMBAR_COMMAND_BENCH_MOVE);
            if (command.command == ROCKET_CMD_BENCH_MOVE_STEPS)
            {
                request.steps = RocketProtocol_ReadI32(command.payload);
            }
            execute_app_command = true;
        }
#else
        else
        {
            result = ROCKET_ACK_UNSUPPORTED;
            detail = (uint16_t)(AMBAR_ACTUATOR_INHIBIT_BUILD_FLAG
                                | AMBAR_ACTUATOR_INHIBIT_BENCH_FLAG);
        }
#endif
        break;

    case ROCKET_CMD_PAD_RESET:
        if (command.payload_length != 0U)
        {
            result = ROCKET_ACK_BAD_LENGTH;
        }
        else if (s_simulation_active
                 || AmbarFlight_IsArmed()
                 || ambar_actuator_motion_active())
        {
            result = ROCKET_ACK_BUSY;
        }
        else
        {
            request.action = AMBAR_COMMAND_PAD_RESET;
            execute_app_command = true;
        }
        break;

    case ROCKET_CMD_SAVE_CONFIG:
        if (command.payload_length != 0U)
        {
            result = ROCKET_ACK_BAD_LENGTH;
        }
#if AMBAR_FEATURE_EFFECTIVE_USB_FLASH_MAINTENANCE
        else if (AmbarFlight_IsArmed() || ambar_actuator_motion_active())
        {
            result = ROCKET_ACK_BUSY;
        }
        else
        {
            request.action = AMBAR_COMMAND_SAVE_CONFIG;
            execute_app_command = true;
        }
#else
        else
        {
            /* Flash maintenance is kept off the time-sensitive USB task. */
            result = ROCKET_ACK_UNSUPPORTED;
        }
#endif
        break;

    case ROCKET_CMD_SIM_START:
#if AMBAR_FEATURE_EFFECTIVE_SIMULATION
        if (command.payload_length == 0U)
        {
            (void)AmbarActuator_StopAndCancel(&s_actuator_state);
            s_simulation_active = true;
            s_simulation_sample_valid = false;
            s_simulation_barometer_pending = false;
            s_simulation_stale = false;
            s_last_simulation_sample_ms = HAL_GetTick();
            AmbarFlight_SetArmed(false);
            AmbarFlight_ResetOnPad((float)HAL_GetTick() * 0.001f);
            s_usb_pending_message = ROCKET_MSG_SIMULATION_STARTED;
        }
        else
        {
            result = ROCKET_ACK_BAD_LENGTH;
        }
#else
        result = ROCKET_ACK_UNSUPPORTED;
#endif
        break;

    case ROCKET_CMD_SIM_STOP:
#if AMBAR_FEATURE_EFFECTIVE_SIMULATION
        if (command.payload_length == 0U)
        {
            s_simulation_active = false;
            s_simulation_sample_valid = false;
            s_simulation_barometer_pending = false;
            s_simulation_stale = false;
            AmbarFlight_SetArmed(false);
            (void)AmbarActuator_StopAndCancel(&s_actuator_state);
            AmbarFlight_ResetOnPad((float)HAL_GetTick() * 0.001f);
            s_usb_pending_message = ROCKET_MSG_SIMULATION_STOPPED;
        }
        else
        {
            result = ROCKET_ACK_BAD_LENGTH;
        }
#else
        result = ROCKET_ACK_UNSUPPORTED;
#endif
        break;

    case ROCKET_CMD_START_LOG:
    case ROCKET_CMD_STOP_LOG:
    case ROCKET_CMD_ERASE_LOG:
        if (command.payload_length != 0U)
        {
            result = ROCKET_ACK_BAD_LENGTH;
        }
#if AMBAR_FEATURE_EFFECTIVE_USB_FLASH_MAINTENANCE
        else if (command.command == ROCKET_CMD_ERASE_LOG
                 && (AmbarFlight_IsArmed() || ambar_actuator_motion_active()))
        {
            result = ROCKET_ACK_BUSY;
        }
#else
        else if (command.command == ROCKET_CMD_ERASE_LOG)
        {
            /* Sector erases block long enough to starve USBX standalone tasks. */
            result = ROCKET_ACK_UNSUPPORTED;
        }
#endif
#if AMBAR_FEATURE_FLASH_LOGGING
        else
        {
            request.action = command.command == ROCKET_CMD_START_LOG
                ? AMBAR_COMMAND_START_LOG
                : (command.command == ROCKET_CMD_STOP_LOG
                    ? AMBAR_COMMAND_STOP_LOG
                    : AMBAR_COMMAND_ERASE_LOG);
            execute_app_command = true;
        }
#else
        else
        {
            result = ROCKET_ACK_UNSUPPORTED;
        }
#endif
        break;

    default:
        result = ROCKET_ACK_UNSUPPORTED;
        break;
    }

    if (execute_app_command)
    {
        const HAL_StatusTypeDef status = ambar_execute_command(&request, false);
        if (status != HAL_OK)
        {
            result = ROCKET_ACK_EXECUTION_ERROR;
            if (request.action == AMBAR_COMMAND_HOME
                || request.action == AMBAR_COMMAND_RETRACT
                || request.action == AMBAR_COMMAND_BENCH_MOVE)
            {
                detail = (uint16_t)(s_actuator_state.last_inhibit_flags & 0xFFFFU);
            }
        }
        else if (request.action == AMBAR_COMMAND_ARM)
        {
            s_usb_pending_message = ROCKET_MSG_CONTROLLER_ENABLED;
        }
        else if (request.action == AMBAR_COMMAND_DISARM
                 || request.action == AMBAR_COMMAND_ESTOP)
        {
            s_usb_pending_message = ROCKET_MSG_CONTROLLER_DISABLED;
        }
        else if (request.action == AMBAR_COMMAND_SET_CONFIG)
        {
            s_usb_pending_message = ROCKET_MSG_CONFIG_APPLIED;
        }
    }

    ambar_usb_complete_command(packet->header.sequence,
                               command.command,
                               result,
                               detail,
                               &command);
}

/* -------------------- Time-bounded simulation input service ([ARCH-3]) -------------------- */

static bool ambar_usb_simulation_value_valid(const RocketSimulationPayload *sample)
{
    const uint16_t known_flags = ROCKET_SIM_FLAG_ALTITUDE_VALID
        | ROCKET_SIM_FLAG_ACCELERATION_VALID
        | ROCKET_SIM_FLAG_VELOCITY_VALID
        | ROCKET_SIM_FLAG_END_OF_STREAM;
    const uint16_t required_sample_flags = ROCKET_SIM_FLAG_ALTITUDE_VALID
        | ROCKET_SIM_FLAG_ACCELERATION_VALID;

    if (sample == NULL || (sample->flags & (uint16_t)~known_flags) != 0U)
    {
        return false;
    }
    if ((sample->flags & ROCKET_SIM_FLAG_END_OF_STREAM) != 0U)
    {
        return true;
    }
    if ((sample->flags & required_sample_flags) != required_sample_flags)
    {
        return false;
    }
    if ((sample->flags & ROCKET_SIM_FLAG_ALTITUDE_VALID) != 0U
        && (sample->altitude_mm < -1000000L || sample->altitude_mm > 20000000L))
    {
        return false;
    }
    if ((sample->flags & ROCKET_SIM_FLAG_ACCELERATION_VALID) != 0U
        && (sample->acceleration_mmps2 < -500000L
            || sample->acceleration_mmps2 > 500000L))
    {
        return false;
    }
    if ((sample->flags & ROCKET_SIM_FLAG_VELOCITY_VALID) != 0U
        && (sample->velocity_mmps < -2000000L || sample->velocity_mmps > 2000000L))
    {
        return false;
    }
    return sample->barometer_stddev_cm <= 10000U;
}

static void ambar_usb_process_simulation(const RocketDecodedPacket *packet)
{
#if AMBAR_FEATURE_EFFECTIVE_SIMULATION
    RocketSimulationPayload sample;

    if (!s_simulation_active
        || !RocketProtocol_DecodeSimulationPayload(packet->payload,
                                                   packet->payload_length,
                                                   &sample)
        || !ambar_usb_simulation_value_valid(&sample))
    {
        return;
    }

    if ((sample.flags & ROCKET_SIM_FLAG_END_OF_STREAM) != 0U)
    {
        s_simulation_active = false;
        s_simulation_sample_valid = false;
        s_simulation_barometer_pending = false;
        s_simulation_stale = false;
        AmbarFlight_SetArmed(false);
        (void)AmbarActuator_StopAndCancel(&s_actuator_state);
        AmbarFlight_ResetOnPad((float)HAL_GetTick() * 0.001f);
        s_usb_pending_message = ROCKET_MSG_SIMULATION_STOPPED;
        return;
    }

    s_simulation_sample = sample;
    s_simulation_sample_valid = true;
    s_simulation_barometer_pending = true;
    s_simulation_stale = false;
    s_last_simulation_sample_ms = HAL_GetTick();
#else
    (void)packet;
#endif
}

static void ambar_usb_service_received_packets(void)
{
    RocketDecodedPacket packet;

    while (AmbarUsb_TakePacket(&packet))
    {
        if (packet.header.type == ROCKET_PKT_COMMAND)
        {
            ambar_usb_process_command(&packet);
        }
        else if (packet.header.type == ROCKET_PKT_SIMULATION)
        {
            ambar_usb_process_simulation(&packet);
        }
    }
}

static void ambar_check_simulation_timeout(uint32_t now)
{
#if AMBAR_FEATURE_EFFECTIVE_SIMULATION
    if (s_simulation_active
        && (uint32_t)(now - s_last_simulation_sample_ms) > AMBAR_SIMULATION_TIMEOUT_MS)
    {
        s_simulation_active = false;
        s_simulation_sample_valid = false;
        s_simulation_barometer_pending = false;
        s_simulation_stale = true;
        AmbarFlight_SetArmed(false);
        (void)AmbarActuator_StopAndCancel(&s_actuator_state);
        AmbarFlight_ResetOnPad((float)HAL_GetTick() * 0.001f);
        s_usb_pending_message = ROCKET_MSG_SIMULATION_STALE;
        s_usb_snapshot_requested = true;
    }
#else
    (void)now;
#endif
}

/* -------------------- Radio extras and persistent log snapshots ([ARCH-6], [ARCH-7]) -------------------- */

static AmbarTelemetryExtra_t ambar_build_telemetry_extra(const AmbarFlightOutput_t *output,
                                                         const AmbarActuatorDecision_t *decision)
{
    AmbarTelemetryExtra_t extra;
    RocketSensorCalibrationStatus_t cal = RocketSensors_GetCalibrationStatus();

    memset(&extra, 0, sizeof(extra));
    /* Preserve config status in bits 0..15; publish build features in 16..25. */
    extra.config_flags =
        AmbarConfig_GetStatusFlags() | AMBAR_FEATURE_CONFIG_STATUS_FLAGS;
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

#if AMBAR_FEATURE_FLASH_LOGGING
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
#endif

/* -------------------- Public application lifecycle ([ARCH-1]) -------------------- */

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
    memset(&s_simulation_sample, 0, sizeof(s_simulation_sample));
    ambar_usb_reset_command_cache();
    s_usb_initialized = false;
    s_usb_was_configured = false;
    s_usb_snapshot_requested = false;
    s_usb_event_state_valid = false;
    s_usb_previous_flags = 0U;
    s_usb_previous_state = 0U;
    s_usb_pending_message = ROCKET_MSG_NONE;
    s_simulation_active = false;
    s_simulation_sample_valid = false;
    s_simulation_barometer_pending = false;
    s_simulation_stale = false;
    s_last_simulation_sample_ms = 0U;

    s_actuator_state = AmbarActuator_DefaultState();
    s_actuator_state.motor_driver_ok = motor_driver_ok != 0U;

    /*
     * Even if the TMC5240 answers on SPI, leave DRV_ENN in the disabled state.
     * The actuator module can only enable it when the build flag, homing state,
     * driver health, and flight command all allow movement.
     */
    TMC5240_SetDriverEnabled(0U);
    if (s_actuator_state.motor_driver_ok)
    {
        if (TMC5240_ConfigureSafeDefaults() != HAL_OK)
        {
            s_actuator_state.motor_driver_ok = false;
            s_actuator_state.machine_state = AMBAR_ACTUATOR_STATE_FAULT;
            s_actuator_state.last_inhibit_flags |=
                AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT;
            TMC5240_SetDriverEnabled(0U);
        }
    }

    /*
     * Load defaults first so a missing or corrupt flash record cannot leave the
     * flight app with uninitialized tuning values.  Then try flash config and
     * initialize the log ring only if the W25Q64 answers correctly.
     */
    AmbarConfig_LoadDefaults(&s_config);
    s_flash_status = W25Q64_Init(&s_flash_jedec);
    if (s_flash_status == HAL_OK)
    {
        s_config_load_status = AmbarConfig_LoadFromFlash(&s_config);
#if AMBAR_FEATURE_FLASH_LOGGING
        s_log_status = AmbarLog_Init(1U);
#else
        s_log_status = HAL_OK;
#endif
    }
    else
    {
        s_config_load_status = HAL_ERROR;
#if AMBAR_FEATURE_FLASH_LOGGING
        s_log_status = AmbarLog_Init(0U);
#else
        s_log_status = HAL_OK;
#endif
    }
#if AMBAR_FEATURE_PRESENTATION_MOTION
    /*
     * A presentation-motion heartbeat is a promise that the actuator uses the
     * reviewed 153600-count prototype profile. Do not let an older/saved bench
     * actuator block silently change travel, direction, current, or ramp values.
     * Flight/EKF settings from flash remain available.
     */
    s_config.actuator = AmbarActuator_DefaultConfig();
    ambar_refresh_config_crc(&s_config);
#endif
    ambar_apply_runtime_config(0U);

    if (sensor_status == HAL_OK)
    {
        /*
         * The EKF starts from the pad with altitude and velocity set to zero.
         * Pad reference capture must succeed before real IMU/barometer samples
         * are trusted by the scheduler below.
         */
        s_pad_reference_status =
            RocketSensors_ResetPadReference(ROCKET_PAD_REFERENCE_DEFAULT_SAMPLES);
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
    s_last_usb_telemetry_ms = now;
    s_last_usb_actuator_ms = now;
    s_last_usb_heartbeat_ms = now;

#if AMBAR_FEATURE_USB_PROTOCOL
    s_usb_initialized = AmbarUsb_Init();
#endif
}

void AmbarApp_Task(void)
{
    const uint32_t now = HAL_GetTick();
    const float timestamp_s = (float)now * 0.001f;

#if AMBAR_FEATURE_USB_PROTOCOL
    if (s_usb_initialized)
    {
        AmbarUsb_Task();
        const bool configured = AmbarUsb_IsConfigured();
        if (configured && !s_usb_was_configured)
        {
            s_usb_snapshot_requested = true;
            s_usb_pending_message = ROCKET_MSG_BOOT;
            s_usb_event_state_valid = false;
        }
        else if (!configured && s_usb_was_configured)
        {
            s_usb_event_state_valid = false;
            ambar_usb_reset_command_cache();
#if AMBAR_FEATURE_ACTUATOR
            /* A lost presentation USB session is always an energy-off stop. */
            AmbarFlight_SetArmed(false);
            (void)AmbarActuator_StopAndCancel(&s_actuator_state);
#endif
        }
        s_usb_was_configured = configured;
        ambar_usb_service_received_packets();
    }
#endif

    ambar_check_simulation_timeout(HAL_GetTick());

    /*
     * Keep the radio receive/heartbeat service active every pass through main.
     * The estimator work below is scheduled by elapsed time instead of HAL_Delay().
     */
#if AMBAR_FEATURE_RADIO
    RadioBridge_Task();

    AmbarCommandResult_t command;
    while (RadioBridge_TakeCommand(&command))
    {
        (void)ambar_execute_command(&command, true);
    }
#endif

    {
        /*
         * The actuator sees the complete flight snapshot. A healthy, armed
         * recovery/on-target output can therefore request its stateful bounded
         * return to HOME; energy-off stop paths above never pass through here as
         * a substitute for disconnect, DISARM, ESTOP, or stale-input handling.
         */
        const AmbarFlightOutput_t actuator_output = AmbarFlight_GetOutput();
        s_last_actuator_decision = AmbarActuator_Task(&s_actuator_state, &actuator_output);
    }

    if (RocketSensors_ConsumeImuDataReady()
        || (uint32_t)(now - s_last_imu_ms) >= AMBAR_IMU_PERIOD_MS)
    {
        /*
         * A valid pad-zeroed vertical acceleration sample advances the EKF.  A
         * failed IMU read feeds NAN, which the EKF rejects and counts as a health
         * fault instead of silently propagating garbage.
         */
        s_last_imu_ms = now;
#if AMBAR_FEATURE_EFFECTIVE_SIMULATION
        if (s_simulation_active)
        {
            if (s_simulation_sample_valid
                && (s_simulation_sample.flags
                    & ROCKET_SIM_FLAG_ACCELERATION_VALID) != 0U)
            {
                s_converted_sensor_data.vertical_acceleration_mps2 =
                    (float)s_simulation_sample.acceleration_mmps2 * 0.001f;
                s_converted_sensor_data.pad_reference_valid = true;
                s_imu_status = HAL_OK;
                (void)AmbarFlight_UpdateImu(
                    timestamp_s,
                    s_converted_sensor_data.vertical_acceleration_mps2);
            }
            else
            {
                s_imu_status = HAL_ERROR;
            }
        }
        else
#endif
        {
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
        }
    }

    if (RocketSensors_ConsumeBarometerDataReady()
        || (uint32_t)(now - s_last_baro_ms) >= AMBAR_BARO_PERIOD_MS)
    {
        /*
         * The barometer path returns altitude above the pad.  The EKF applies
         * its innovation gate, so disconnected/noisy pressure data becomes a
         * rejected sample counter and an inhibit flag rather than a bad command.
         */
        s_last_baro_ms = now;
#if AMBAR_FEATURE_EFFECTIVE_SIMULATION
        if (s_simulation_active)
        {
            if (s_simulation_sample_valid
                && (s_simulation_sample.flags & ROCKET_SIM_FLAG_ALTITUDE_VALID) != 0U)
            {
                float stddev_m = (float)s_simulation_sample.barometer_stddev_cm * 0.01f;
                if (stddev_m < 0.1f)
                {
                    stddev_m = AMBAR_BARO_STDDEV_M;
                }
                s_converted_sensor_data.altitude_agl_m =
                    (float)s_simulation_sample.altitude_mm * 0.001f;
                s_converted_sensor_data.pad_reference_valid = true;
                s_baro_status = HAL_OK;
                if (s_simulation_barometer_pending)
                {
                    s_simulation_barometer_pending = false;
                    (void)AmbarFlight_UpdateBarometer(
                        s_converted_sensor_data.altitude_agl_m,
                        stddev_m);
                }
            }
            else
            {
                s_baro_status = HAL_ERROR;
            }
        }
        else
#endif
        {
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
        }
    }

#if AMBAR_FEATURE_USB_PROTOCOL
    if (s_usb_initialized && AmbarUsb_IsConfigured())
    {
        if (s_usb_snapshot_requested
            || (uint32_t)(now - s_last_usb_telemetry_ms) >= AMBAR_USB_TELEMETRY_PERIOD_MS)
        {
            uint8_t message = ROCKET_MSG_NONE;
            if (s_usb_snapshot_requested)
            {
                message = s_usb_pending_message != ROCKET_MSG_NONE
                    ? s_usb_pending_message
                    : ROCKET_MSG_SNAPSHOT;
            }
            s_usb_snapshot_requested = false;
            s_last_usb_telemetry_ms = now;
            ambar_usb_send_telemetry(message);
        }

        if ((uint32_t)(now - s_last_usb_actuator_ms) >= AMBAR_USB_ACTUATOR_PERIOD_MS)
        {
            s_last_usb_actuator_ms = now;
            ambar_usb_send_actuator_status();
        }

        if ((uint32_t)(now - s_last_usb_heartbeat_ms) >= AMBAR_USB_HEARTBEAT_PERIOD_MS)
        {
            s_last_usb_heartbeat_ms = now;
            ambar_usb_send_heartbeat();
        }
    }
#endif

#if AMBAR_FEATURE_EFFECTIVE_TELEMETRY
    if ((uint32_t)(now - s_last_telemetry_ms) >= AMBAR_TELEMETRY_PERIOD_MS)
    {
        /*
         * Raw fields remain in the packet for bring-up.  The added flight fields
         * carry altitude, velocity, acceleration, predicted apogee, phase, command,
         * inhibit flags, and rejected-sample counters as fixed-point signed ints.
         */
        s_last_telemetry_ms = now;
#if AMBAR_FEATURE_MAGNETOMETER_TELEMETRY
        s_mag_status = RocketSensors_ReadMagnetometerRaw(&s_raw_sensor_data);
        s_converted_sensor_data.magnetometer_valid = s_mag_status == HAL_OK;
#else
        memset(s_raw_sensor_data.mag, 0, sizeof(s_raw_sensor_data.mag));
        s_mag_status = HAL_ERROR;
        s_converted_sensor_data.magnetometer_valid = false;
#endif

        const AmbarFlightOutput_t flight_output = AmbarFlight_GetOutput();
        const AmbarActuatorDecision_t actuator_decision = s_last_actuator_decision;
        const AmbarTelemetryExtra_t extra =
            ambar_build_telemetry_extra(&flight_output, &actuator_decision);

        const uint8_t deployment_percent = ambar_deployment_percent(&flight_output);
        uint16_t calc_data[4] = {0};
        ambar_fill_legacy_calc(calc_data, &flight_output, deployment_percent);

#if AMBAR_FEATURE_VERBOSE_STATUS_TEXT
        const char *message[3] = {
            ambar_status_message(&flight_output),
            s_last_command_result.response[0] != '\0'
                ? s_last_command_result.response
                : AmbarFlight_PhaseName(flight_output.phase),
            actuator_decision.inhibit_flags == AMBAR_ACTUATOR_INHIBIT_NONE
                ? ""
                : "ACTUATOR_INHIBITED"
        };
#endif

        if (s_radio_status == HAL_OK)
        {
            s_telemetry_status = PayloadPipelineWithFlight(
                (uint16_t *)s_raw_sensor_data.imu,
                s_raw_sensor_data.baro,
                (uint16_t *)s_raw_sensor_data.mag,
                calc_data,
#if AMBAR_FEATURE_VERBOSE_STATUS_TEXT
                message,
#else
                NULL,
#endif
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

    }
#endif

    /* Keep flash logging at 5 Hz even though the long radio frame is sent at 1 Hz. */
#if AMBAR_FEATURE_FLASH_LOGGING
    if ((uint32_t)(now - s_last_log_ms) >= AMBAR_LOG_PERIOD_MS)
    {
        const AmbarFlightOutput_t flight_output = AmbarFlight_GetOutput();
        const AmbarActuatorDecision_t actuator_decision = s_last_actuator_decision;

        s_last_log_ms = now;
        ambar_append_log_snapshot(&flight_output, &actuator_decision);
    }
#endif
}

void AmbarApp_HandleExtiPin(uint16_t GPIO_Pin)
{
    AmbarActuator_HandleExtiPin(&s_actuator_state, GPIO_Pin);
}
