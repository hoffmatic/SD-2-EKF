/**
 * @file ambar_command.h
 * @brief Parse the legacy ASCII radio/bench command vocabulary.
 *
 * OVERVIEW
 * --------
 * This module translates one bounded text packet into a typed action plus an
 * ACK/NACK description.  Parsing never performs the action: ambar_app.c remains
 * responsible for arming rules, flash restrictions, HOME, ESTOP, and actuator
 * safety gates.
 *
 * HOW IT WORKS
 * ------------
 * radio_bridge.c receives bytes -> AmbarCommand_ProcessPacket() normalizes and
 * validates them -> AmbarApp consumes AmbarCommandResult_t -> the application
 * either performs the request or replaces the parser ACK with an execution
 * failure.  See CODE_GUIDE.md [ARCH-1], [ARCH-5], and [ARCH-6].
 *
 * SAFETY / LIMITS
 * ---------------
 * The text channel is convenient for bench radio work, but it is not
 * authenticated and has no replay protection.  It must not bypass the binary
 * USB protocol or the actuator's final hardware gates in a flight build.
 */

#ifndef AMBAR_COMMAND_H
#define AMBAR_COMMAND_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

/** Maximum SET_CONFIG key including its terminating NUL byte. */
#define AMBAR_COMMAND_KEY_LEN      24U

/** Maximum human-readable ACK/NACK response including its terminating NUL. */
#define AMBAR_COMMAND_RESPONSE_LEN 96U

typedef enum
{
    /* No recognized request. */
    AMBAR_COMMAND_NONE = 0,

    /* Read-only health/status requests. */
    AMBAR_COMMAND_PING,
    AMBAR_COMMAND_STATUS,

    /* Flight-state and emergency control requests. */
    AMBAR_COMMAND_PAD_RESET,
    AMBAR_COMMAND_ARM,
    AMBAR_COMMAND_DISARM,
    AMBAR_COMMAND_ESTOP,

    /* Explicit actuator maintenance requests; later safety gates still apply. */
    AMBAR_COMMAND_RETRACT,
    AMBAR_COMMAND_HOME,
    AMBAR_COMMAND_BENCH_MOVE,

    /* Configuration and flash-log maintenance requests. */
    AMBAR_COMMAND_SET_CONFIG,
    AMBAR_COMMAND_SAVE_CONFIG,
    AMBAR_COMMAND_START_LOG,
    AMBAR_COMMAND_STOP_LOG,
    AMBAR_COMMAND_ERASE_LOG
} AmbarCommandAction_t;

typedef enum
{
    /** Syntax and argument validation succeeded. */
    AMBAR_COMMAND_ACK_OK = 0,
    /** The first token does not name a supported command. */
    AMBAR_COMMAND_ACK_UNKNOWN = 1,
    /** A required value is absent or cannot be parsed. */
    AMBAR_COMMAND_ACK_BAD_ARGUMENT = 2,
    /** Input exceeded the bounded parser buffer. */
    AMBAR_COMMAND_ACK_TOO_LONG = 3
} AmbarCommandAck_t;

typedef struct
{
    /** Typed request selected by the first command token. */
    AmbarCommandAction_t action;
    /** Parser-level result; execution can still be refused by the application. */
    AmbarCommandAck_t ack;
    /** True only when command syntax and arguments were accepted. */
    bool accepted;
    /** Uppercase SET_CONFIG key, otherwise an empty string. */
    char key[AMBAR_COMMAND_KEY_LEN];
    /** Parsed SET_CONFIG floating-point value. */
    float value;
    /** Parsed BENCH_MOVE signed step target. */
    int32_t steps;
    /** Short radio-facing ACK/NACK explanation. */
    char response[AMBAR_COMMAND_RESPONSE_LEN];
} AmbarCommandResult_t;

/**
 * @brief Parse one non-NUL-terminated command packet.
 * @param packet Raw ASCII packet bytes.
 * @param length Number of valid bytes in @p packet.
 * @param result Fully initialized parse result, including failures.
 * @return HAL_OK for recognized valid syntax; HAL_ERROR otherwise.
 * @note This function has no hardware side effects.
 */
HAL_StatusTypeDef AmbarCommand_ProcessPacket(const uint8_t *packet,
                                             uint8_t length,
                                             AmbarCommandResult_t *result);

/** @brief Return the stable uppercase name used by logs and responses. */
const char *AmbarCommand_ActionName(AmbarCommandAction_t action);

#ifdef __cplusplus
}
#endif

#endif /* AMBAR_COMMAND_H */
