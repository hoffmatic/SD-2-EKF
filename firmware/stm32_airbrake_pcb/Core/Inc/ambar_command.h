/*
 * PROJECT FILE OVERVIEW
 * Comment made: 2026-07-07 18:31:46 -04:00
 *
 * What this file does:
 *   This header defines the simple radio command parser. It turns incoming text packets into safe actions for the app to accept or reject.
 *
 * Process flow:
 *   radio_bridge receives bytes, AmbarCommand_ProcessPacket trims and parses them, then the app consumes the parsed action and performs the actual state change.
 *
 * Main variables and what can be changed:
 *   AmbarCommandAction_t lists supported commands. AMBAR_COMMAND_RESPONSE_LEN limits the short ACK/NACK text returned over telemetry or direct radio messages.
 *
 * Assumptions:
 *   Commands are ASCII text during bench bring-up. This is not authenticated and should not be treated as a flight-ready command link.
 *
 * What is missing:
 *   There is no cryptographic authentication, sequence-number replay protection, binary command framing, or ground-station UI yet.
 */

#ifndef AMBAR_COMMAND_H
#define AMBAR_COMMAND_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

#define AMBAR_COMMAND_KEY_LEN      24U
#define AMBAR_COMMAND_RESPONSE_LEN 96U

typedef enum
{
    AMBAR_COMMAND_NONE = 0,
    AMBAR_COMMAND_PING,
    AMBAR_COMMAND_STATUS,
    AMBAR_COMMAND_PAD_RESET,
    AMBAR_COMMAND_ARM,
    AMBAR_COMMAND_DISARM,
    AMBAR_COMMAND_ESTOP,
    AMBAR_COMMAND_RETRACT,
    AMBAR_COMMAND_HOME,
    AMBAR_COMMAND_BENCH_MOVE,
    AMBAR_COMMAND_SET_CONFIG,
    AMBAR_COMMAND_SAVE_CONFIG,
    AMBAR_COMMAND_START_LOG,
    AMBAR_COMMAND_STOP_LOG,
    AMBAR_COMMAND_ERASE_LOG
} AmbarCommandAction_t;

typedef enum
{
    AMBAR_COMMAND_ACK_OK = 0,
    AMBAR_COMMAND_ACK_UNKNOWN = 1,
    AMBAR_COMMAND_ACK_BAD_ARGUMENT = 2,
    AMBAR_COMMAND_ACK_TOO_LONG = 3
} AmbarCommandAck_t;

typedef struct
{
    AmbarCommandAction_t action;
    AmbarCommandAck_t ack;
    bool accepted;
    char key[AMBAR_COMMAND_KEY_LEN];
    float value;
    int32_t steps;
    char response[AMBAR_COMMAND_RESPONSE_LEN];
} AmbarCommandResult_t;

HAL_StatusTypeDef AmbarCommand_ProcessPacket(const uint8_t *packet,
                                             uint8_t length,
                                             AmbarCommandResult_t *result);
const char *AmbarCommand_ActionName(AmbarCommandAction_t action);

#ifdef __cplusplus
}
#endif

#endif /* AMBAR_COMMAND_H */
