/**
 * @file ambar_command.c
 * @brief Bounded implementation of the legacy ASCII command parser.
 *
 * The file is organized as: parser limits -> normalization/conversion helpers
 * -> public packet parser -> action-name formatter.  Helpers deliberately have
 * no module state, which makes one packet independent of every previous packet.
 * See ambar_command.h and CODE_GUIDE.md [ARCH-6].
 */

#include "ambar_command.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Parser limits and private normalization helpers                            */
/* -------------------------------------------------------------------------- */

/** Largest accepted radio packet before the local NUL terminator is added. */
#define AMBAR_COMMAND_MAX_PACKET_LEN 120U

/** Reset every result field so failure paths cannot leak a prior command. */
static void ambar_command_clear(AmbarCommandResult_t *result)
{
    if (result != NULL)
    {
        memset(result, 0, sizeof(*result));
        result->action = AMBAR_COMMAND_NONE;
        result->ack = AMBAR_COMMAND_ACK_UNKNOWN;
        result->accepted = false;
    }
}

/** Trim leading/trailing ASCII whitespace in place and return the first token. */
static char *ambar_trim(char *text)
{
    char *end;

    while (*text != '\0' && isspace((unsigned char)*text))
    {
        ++text;
    }

    end = text + strlen(text);
    while (end > text && isspace((unsigned char)*(end - 1)))
    {
        --end;
    }
    *end = '\0';

    return text;
}

/** Normalize a NUL-terminated token for case-insensitive command matching. */
static void ambar_uppercase(char *text)
{
    while (*text != '\0')
    {
        *text = (char)toupper((unsigned char)*text);
        ++text;
    }
}

/** Parse one float token; range validation belongs to the config layer. */
static bool ambar_parse_float(const char *text, float *value)
{
    char *end = NULL;
    float parsed;

    if (text == NULL || value == NULL)
    {
        return false;
    }

    parsed = strtof(text, &end);
    if (end == text)
    {
        return false;
    }

    *value = parsed;
    return true;
}

/** Parse a decimal signed step value used by BENCH_MOVE. */
static bool ambar_parse_int32(const char *text, int32_t *value)
{
    char *end = NULL;
    long parsed;

    if (text == NULL || value == NULL)
    {
        return false;
    }

    parsed = strtol(text, &end, 10);
    if (end == text)
    {
        return false;
    }

    *value = (int32_t)parsed;
    return true;
}

/** Populate the common successful ACK fields for a recognized command. */
static HAL_StatusTypeDef ambar_command_finish(AmbarCommandResult_t *result,
                                              AmbarCommandAction_t action,
                                              const char *response)
{
    result->action = action;
    result->ack = AMBAR_COMMAND_ACK_OK;
    result->accepted = true;
    (void)snprintf(result->response,
                   sizeof(result->response),
                   "ACK:%s",
                   response != NULL ? response : AmbarCommand_ActionName(action));
    return HAL_OK;
}

/* -------------------------------------------------------------------------- */
/* Public parser API                                                          */
/* -------------------------------------------------------------------------- */

HAL_StatusTypeDef AmbarCommand_ProcessPacket(const uint8_t *packet,
                                             uint8_t length,
                                             AmbarCommandResult_t *result)
{
    char buffer[AMBAR_COMMAND_MAX_PACKET_LEN + 1U];
    char command[24] = {0};
    char arg1[AMBAR_COMMAND_KEY_LEN] = {0};
    char arg2[32] = {0};
    int field_count;

    if (result == NULL)
    {
        return HAL_ERROR;
    }

    ambar_command_clear(result);

    if (packet == NULL || length == 0U)
    {
        result->ack = AMBAR_COMMAND_ACK_BAD_ARGUMENT;
        (void)snprintf(result->response, sizeof(result->response), "NACK:EMPTY");
        return HAL_ERROR;
    }

    if (length > AMBAR_COMMAND_MAX_PACKET_LEN)
    {
        result->ack = AMBAR_COMMAND_ACK_TOO_LONG;
        (void)snprintf(result->response, sizeof(result->response), "NACK:TOO_LONG");
        return HAL_ERROR;
    }

    memcpy(buffer, packet, length);
    buffer[length] = '\0';

    /* At most three fields are accepted: COMMAND [KEY_OR_STEPS] [VALUE]. */
    char *trimmed = ambar_trim(buffer);
    field_count = sscanf(trimmed, "%23s %23s %31s", command, arg1, arg2);
    if (field_count <= 0)
    {
        result->ack = AMBAR_COMMAND_ACK_BAD_ARGUMENT;
        (void)snprintf(result->response, sizeof(result->response), "NACK:EMPTY");
        return HAL_ERROR;
    }

    ambar_uppercase(command);
    ambar_uppercase(arg1);

    if (strcmp(command, "PING") == 0)
    {
        return ambar_command_finish(result, AMBAR_COMMAND_PING, "PONG");
    }
    if (strcmp(command, "STATUS") == 0)
    {
        return ambar_command_finish(result, AMBAR_COMMAND_STATUS, "STATUS");
    }
    if (strcmp(command, "PAD_RESET") == 0 || strcmp(command, "PADRESET") == 0)
    {
        return ambar_command_finish(result, AMBAR_COMMAND_PAD_RESET, "PAD_RESET");
    }
    if (strcmp(command, "ARM") == 0)
    {
        return ambar_command_finish(result, AMBAR_COMMAND_ARM, "ARM");
    }
    if (strcmp(command, "DISARM") == 0)
    {
        return ambar_command_finish(result, AMBAR_COMMAND_DISARM, "DISARM");
    }
    if (strcmp(command, "ESTOP") == 0 || strcmp(command, "E_STOP") == 0)
    {
        return ambar_command_finish(result, AMBAR_COMMAND_ESTOP, "ESTOP");
    }
    if (strcmp(command, "RETRACT") == 0)
    {
        return ambar_command_finish(result, AMBAR_COMMAND_RETRACT, "RETRACT");
    }
    if (strcmp(command, "HOME") == 0)
    {
        return ambar_command_finish(result, AMBAR_COMMAND_HOME, "HOME");
    }
    if (strcmp(command, "SAVE_CONFIG") == 0 || strcmp(command, "SAVECONFIG") == 0)
    {
        return ambar_command_finish(result, AMBAR_COMMAND_SAVE_CONFIG, "SAVE_CONFIG");
    }
    if (strcmp(command, "START_LOG") == 0 || strcmp(command, "STARTLOG") == 0)
    {
        return ambar_command_finish(result, AMBAR_COMMAND_START_LOG, "START_LOG");
    }
    if (strcmp(command, "STOP_LOG") == 0 || strcmp(command, "STOPLOG") == 0)
    {
        return ambar_command_finish(result, AMBAR_COMMAND_STOP_LOG, "STOP_LOG");
    }
    if (strcmp(command, "ERASE_LOG") == 0 || strcmp(command, "ERASELOG") == 0)
    {
        return ambar_command_finish(result, AMBAR_COMMAND_ERASE_LOG, "ERASE_LOG");
    }
    if (strcmp(command, "BENCH_MOVE") == 0 || strcmp(command, "BENCHMOVE") == 0)
    {
        if (field_count < 2 || !ambar_parse_int32(arg1, &result->steps))
        {
            result->ack = AMBAR_COMMAND_ACK_BAD_ARGUMENT;
            (void)snprintf(result->response, sizeof(result->response), "NACK:BENCH_MOVE_STEPS");
            return HAL_ERROR;
        }
        return ambar_command_finish(result, AMBAR_COMMAND_BENCH_MOVE, "BENCH_MOVE");
    }
    if (strcmp(command, "SET_CONFIG") == 0 || strcmp(command, "SETCONFIG") == 0)
    {
        if (field_count < 3 || !ambar_parse_float(arg2, &result->value))
        {
            result->ack = AMBAR_COMMAND_ACK_BAD_ARGUMENT;
            (void)snprintf(result->response, sizeof(result->response), "NACK:SET_CONFIG_KEY_VALUE");
            return HAL_ERROR;
        }

        strncpy(result->key, arg1, sizeof(result->key) - 1U);
        return ambar_command_finish(result, AMBAR_COMMAND_SET_CONFIG, "SET_CONFIG");
    }

    result->ack = AMBAR_COMMAND_ACK_UNKNOWN;
    (void)snprintf(result->response, sizeof(result->response), "NACK:UNKNOWN:%s", command);
    return HAL_ERROR;
}

const char *AmbarCommand_ActionName(AmbarCommandAction_t action)
{
    switch (action)
    {
    case AMBAR_COMMAND_PING: return "PING";
    case AMBAR_COMMAND_STATUS: return "STATUS";
    case AMBAR_COMMAND_PAD_RESET: return "PAD_RESET";
    case AMBAR_COMMAND_ARM: return "ARM";
    case AMBAR_COMMAND_DISARM: return "DISARM";
    case AMBAR_COMMAND_ESTOP: return "ESTOP";
    case AMBAR_COMMAND_RETRACT: return "RETRACT";
    case AMBAR_COMMAND_HOME: return "HOME";
    case AMBAR_COMMAND_BENCH_MOVE: return "BENCH_MOVE";
    case AMBAR_COMMAND_SET_CONFIG: return "SET_CONFIG";
    case AMBAR_COMMAND_SAVE_CONFIG: return "SAVE_CONFIG";
    case AMBAR_COMMAND_START_LOG: return "START_LOG";
    case AMBAR_COMMAND_STOP_LOG: return "STOP_LOG";
    case AMBAR_COMMAND_ERASE_LOG: return "ERASE_LOG";
    case AMBAR_COMMAND_NONE:
    default:
        return "NONE";
    }
}
