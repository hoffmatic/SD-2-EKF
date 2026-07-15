/*
 * AMBAR LILYGO T3-S3 SX1280 GROUND STATION
 *
 * Purpose
 *   Provides the known-working Arduino/RadioLib ground station for the STM32
 *   SX1280 link.  It receives the firmware's version-1 radio telemetry, prints a
 *   detailed tag-by-tag decode and/or a compact operator summary, and forwards
 *   explicitly typed serial-console text to the rocket as a radio command.
 *
 * Data/control flow
 *   DIO1 only latches a receive flag.  loop() services the serial console and,
 *   when a packet is ready, reads it from RadioLib, records RSSI/SNR, recognizes
 *   binary telemetry versus plain text, parses bounded TLV sections, and prints
 *   the selected view.  "tx <payload>" temporarily leaves receive mode, sends
 *   those exact bytes, and restores reception.  See CODE_GUIDE.md [ARCH-6].
 *
 * Section map
 *   1. Board wiring, radio parameters, and version-1 tag constants
 *   2. Summary models and flag-name dictionaries
 *   3. Radio/session state and display mode
 *   4. Endian, label, and summary-capture helpers
 *   5. Compact operator summary renderer
 *   6. Detailed tag decoders and bounded TLV parser
 *   7. Radio interrupt, RX/TX, and serial-console service
 *   8. Arduino setup() and loop()
 *
 * Safety and assumptions
 *   Pin mapping and modulation settings must match the LILYGO T3-S3 SX1280PA
 *   hardware and STM32 radio configuration.  This sketch observes telemetry; it
 *   does not authorize actuator motion or bypass firmware safety gates.  Console
 *   transmit deliberately does not normalize or validate command text, so the
 *   operator is responsible for every payload.  Unknown/truncated tags are
 *   reported and parsing stops before an out-of-bounds read.  This radio/TLV
 *   format is separate from direct-USB rocket_protocol version 2.
 */

#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <stdio.h>
#include <string.h>

/* ===================== BOARD WIRING AND RADIO PARAMETERS ===================== */

// LILYGO T3-S3 SX1280PA wiring used by the known-working ground station.
#define LORA_SCK   5
#define LORA_MISO  3
#define LORA_MOSI  6
#define LORA_CS    7
#define LORA_RST   8
#define LORA_DIO1  9
#define LORA_BUSY  36
#define LORA_TXEN  10
#define LORA_RXEN  21

static const float RADIO_FREQUENCY_MHZ = 2445.0F;
static const float RADIO_BANDWIDTH_KHZ = 203.125F;
static const uint8_t RADIO_SPREADING_FACTOR = 7;
static const uint8_t RADIO_CODING_RATE_DENOMINATOR = 5;
static const int8_t RADIO_OUTPUT_POWER_DBM = 3;
static const uint16_t RADIO_PREAMBLE_LENGTH = 12;
static const size_t STM32_MAX_PAYLOAD_LEN = 200;

/* ===================== VERSION-1 RADIO TELEMETRY TAGS ===================== */

static const uint8_t PAYLOAD_MAGIC = 0xA5;
static const uint8_t PAYLOAD_TYPE_DATA = 0x01;

static const uint8_t TAG_IMU = 0x10;
static const uint8_t TAG_BARO = 0x20;
static const uint8_t TAG_MAGNET = 0x30;
static const uint8_t TAG_CALC = 0x40;
static const uint8_t TAG_STATUS_MSG = 0x50;
static const uint8_t TAG_COMMAND_MSG = 0x51;
static const uint8_t TAG_ERROR_MSG = 0x52;
static const uint8_t TAG_FLIGHT_ESTIMATE = 0x60;
static const uint8_t TAG_FLIGHT_COMMAND = 0x61;
static const uint8_t TAG_FLIGHT_HEALTH = 0x62;
static const uint8_t TAG_SYSTEM_STATUS = 0x63;
static const uint8_t TAG_ACTUATOR_STATUS = 0x64;
static const uint8_t TAG_APOGEE_DETAIL = 0x65;

/* ===================== SUMMARY MODELS AND FLAG DICTIONARIES ===================== */

struct NamedFlag {
  uint32_t mask;
  const char *name;
};

static const size_t SUMMARY_TEXT_CAPACITY = 96;
static const size_t SUMMARY_NOTICE_CAPACITY = 160;

struct S32TagSummary {
  bool seen;
  uint8_t count;
  int32_t values[5];
};

struct PacketSummary {
  // One receive-pass accumulator used by both compact and detailed views.
  uint32_t packetNumber;
  size_t packetLength;
  float rssiDbm;
  float snrDb;

  bool telemetry;
  bool headerValid;
  uint16_t sequence;
  uint32_t timeMs;
  uint8_t headerDeploymentPercent;

  S32TagSummary flightEstimate;
  S32TagSummary flightCommand;
  S32TagSummary flightHealth;
  S32TagSummary systemStatus;
  S32TagSummary actuatorStatus;
  S32TagSummary apogeeDetail;

  bool statusTextSeen;
  bool commandTextSeen;
  bool errorTextSeen;
  bool rawTextSeen;
  char statusText[SUMMARY_TEXT_CAPACITY];
  char commandText[SUMMARY_TEXT_CAPACITY];
  char errorText[SUMMARY_TEXT_CAPACITY];
  char rawText[SUMMARY_TEXT_CAPACITY];
  char parserNotice[SUMMARY_NOTICE_CAPACITY];
};

static const NamedFlag FLIGHT_INHIBIT_FLAGS[] = {
  {1UL << 0, "ESTIMATOR_UNHEALTHY"},
  {1UL << 1, "NOT_IN_COAST"},
  {1UL << 2, "BELOW_MINIMUM_ALTITUDE"},
  {1UL << 3, "BEFORE_MINIMUM_FLIGHT_TIME"},
  {1UL << 4, "DESCENDING"},
  {1UL << 5, "APOGEE_ON_TARGET"},
  {1UL << 6, "NOT_ARMED"}
};

static const NamedFlag ACTUATOR_INHIBIT_FLAGS[] = {
  {1UL << 0, "BUILD_FLAG"},
  {1UL << 1, "NOT_HOMED"},
  {1UL << 2, "DRIVER_FAULT"},
  {1UL << 3, "FLIGHT_COMMAND"},
  {1UL << 4, "BENCH_FLAG"},
  {1UL << 5, "CONFIG_INVALID"},
  {1UL << 6, "ESTOP"},
  {1UL << 7, "DIAG_FAULT"}
};

// Plain-language names used only in the compact operator summary.
static const NamedFlag FLIGHT_INHIBIT_SUMMARY_FLAGS[] = {
  {1UL << 0, "estimator unhealthy"},
  {1UL << 1, "not in coast"},
  {1UL << 2, "below minimum altitude"},
  {1UL << 3, "before minimum flight time"},
  {1UL << 4, "descending"},
  {1UL << 5, "apogee on target"},
  {1UL << 6, "not armed"}
};

static const NamedFlag ACTUATOR_INHIBIT_SUMMARY_FLAGS[] = {
  {1UL << 0, "build inhibit"},
  {1UL << 1, "not homed"},
  {1UL << 2, "driver fault"},
  {1UL << 3, "flight command inhibited"},
  {1UL << 4, "bench motion disabled"},
  {1UL << 5, "configuration invalid"},
  {1UL << 6, "emergency stop"},
  {1UL << 7, "diagnostic fault"}
};

static const NamedFlag CONFIG_STATUS_FLAGS[] = {
  {1UL << 0, "DEFAULTS_USED"},
  {1UL << 1, "FLASH_LOAD_OK"},
  {1UL << 2, "FLASH_SAVE_OK"},
  {1UL << 3, "CRC_ERROR"},
  {1UL << 4, "VALUE_ERROR"},
  {1UL << 16, "FEATURE_RADIO"},
  {1UL << 17, "FEATURE_TELEMETRY"},
  {1UL << 18, "FEATURE_FLASH_LOGGING"},
  {1UL << 19, "FEATURE_ACTUATOR"},
  {1UL << 20, "FEATURE_BENCH_ACTUATOR_COMMANDS"},
  {1UL << 21, "FEATURE_WATCHDOG"},
  {1UL << 22, "FEATURE_MAGNETOMETER_TELEMETRY"},
  {1UL << 23, "FEATURE_VERBOSE_STATUS_TEXT"},
  {1UL << 24, "FEATURE_AUTO_FLIGHT_PHASES"},
  {1UL << 25, "FEATURE_RADIO_HEARTBEAT"}
};

static const uint32_t REPORTED_FEATURE_MASK = 0x03FF0000UL;

static const NamedFlag LOG_STATUS_FLAGS[] = {
  {1UL << 0, "FLASH_OK"},
  {1UL << 1, "ACTIVE"},
  {1UL << 2, "FULL_OR_WRAPPED"},
  {1UL << 3, "LAST_WRITE_OK"},
  {1UL << 4, "LAST_WRITE_FAILED"}
};

/* ===================== RADIO, SESSION, AND VIEW STATE ===================== */

SX1280 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);

// Written by the DIO1 ISR and consumed by loop(); keep the ISR free of I/O.
volatile bool rxFlag = false;
volatile uint32_t dio1Count = 0;

// Cooperative-loop diagnostics; rollover-safe comparisons use unsigned deltas.
static uint32_t lastWaitingPrintMs = 0;
static uint32_t receivedPacketCount = 0;

enum ViewMode {
  VIEW_BOTH,
  VIEW_SUMMARY,
  VIEW_DETAIL
};

static ViewMode viewMode = VIEW_BOTH;

/* ===================== WIRE, LABEL, AND VIEW HELPERS ===================== */

bool detailOutputEnabled() {
  return viewMode == VIEW_BOTH || viewMode == VIEW_DETAIL;
}

bool summaryOutputEnabled() {
  return viewMode == VIEW_BOTH || viewMode == VIEW_SUMMARY;
}

void printHexByte(uint8_t value) {
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

void printHex32(uint32_t value) {
  Serial.print("0x");
  for (int shift = 28; shift >= 0; shift -= 4) {
    Serial.print((value >> shift) & 0x0FUL, HEX);
  }
}

uint16_t readU16LE(const uint8_t *data, size_t index) {
  return (uint16_t)data[index]
       | ((uint16_t)data[index + 1] << 8);
}

uint32_t readU32LE(const uint8_t *data, size_t index) {
  return (uint32_t)data[index]
       | ((uint32_t)data[index + 1] << 8)
       | ((uint32_t)data[index + 2] << 16)
       | ((uint32_t)data[index + 3] << 24);
}

int32_t readS32LE(const uint8_t *data, size_t index) {
  return (int32_t)readU32LE(data, index);
}

int32_t tagS32(const uint8_t *data, uint8_t fieldIndex) {
  return readS32LE(data, (size_t)fieldIndex * 4U);
}

const char *phaseName(int32_t phase) {
  switch (phase) {
    case 0: return "PAD_IDLE";
    case 1: return "BOOST";
    case 2: return "COAST";
    case 3: return "AIRBRAKE_ACTIVE";
    case 4: return "RECOVERY";
    case 5: return "FAULT";
    default: return "UNKNOWN";
  }
}

const char *actuatorStateName(int32_t state) {
  switch (state) {
    case 0: return "DISABLED";
    case 1: return "IDLE";
    case 2: return "HOMING";
    case 3: return "READY";
    case 4: return "MOVING";
    case 5: return "RETRACTING";
    case 6: return "FAULT";
    case 7: return "ESTOP";
    default: return "UNKNOWN";
  }
}

const char *commandActionName(int32_t action) {
  switch (action) {
    case 0: return "NONE";
    case 1: return "PING";
    case 2: return "STATUS";
    case 3: return "PAD_RESET";
    case 4: return "ARM";
    case 5: return "DISARM";
    case 6: return "ESTOP";
    case 7: return "RETRACT";
    case 8: return "HOME";
    case 9: return "BENCH_MOVE";
    case 10: return "SET_CONFIG";
    case 11: return "SAVE_CONFIG";
    case 12: return "START_LOG";
    case 13: return "STOP_LOG";
    case 14: return "ERASE_LOG";
    default: return "UNKNOWN";
  }
}

const char *commandAckName(int32_t ack) {
  switch (ack) {
    case 0: return "OK";
    case 1: return "UNKNOWN_COMMAND";
    case 2: return "BAD_ARGUMENT";
    case 3: return "TOO_LONG";
    default: return "UNKNOWN";
  }
}

/* ===================== SUMMARY CAPTURE HELPERS ===================== */

void initializePacketSummary(PacketSummary &summary,
                             uint32_t packetNumber,
                             size_t packetLength,
                             float rssiDbm,
                             float snrDb) {
  // Clear every optional field before capturing one newly received packet.
  memset(&summary, 0, sizeof(summary));
  summary.packetNumber = packetNumber;
  summary.packetLength = packetLength;
  summary.rssiDbm = rssiDbm;
  summary.snrDb = snrDb;
}

void appendParserNotice(PacketSummary &summary, const char *notice) {
  if (notice == NULL || notice[0] == '\0') {
    return;
  }

  const size_t used = strlen(summary.parserNotice);
  if (used >= sizeof(summary.parserNotice) - 1U) {
    return;
  }

  if (used > 0U) {
    strncat(summary.parserNotice,
            "; ",
            sizeof(summary.parserNotice) - strlen(summary.parserNotice) - 1U);
  }
  strncat(summary.parserNotice,
          notice,
          sizeof(summary.parserNotice) - strlen(summary.parserNotice) - 1U);
}

void copySummaryText(char *destination,
                     size_t capacity,
                     const uint8_t *data,
                     size_t length) {
  if (destination == NULL || capacity == 0U) {
    return;
  }

  size_t copyLength = length;
  bool truncated = false;
  if (copyLength >= capacity) {
    copyLength = capacity - 1U;
    truncated = true;
  }

  for (size_t i = 0; i < copyLength; ++i) {
    const uint8_t value = data[i];
    destination[i] = (value >= 0x20U && value <= 0x7EU) ? (char)value : '.';
  }
  destination[copyLength] = '\0';

  if (truncated && capacity >= 4U) {
    destination[capacity - 4U] = '.';
    destination[capacity - 3U] = '.';
    destination[capacity - 2U] = '.';
    destination[capacity - 1U] = '\0';
  }
}

void captureTextSummary(PacketSummary &summary,
                        uint8_t tag,
                        const uint8_t *data,
                        uint8_t length) {
  char *destination = NULL;
  bool *seen = NULL;

  if (tag == TAG_STATUS_MSG) {
    destination = summary.statusText;
    seen = &summary.statusTextSeen;
  } else if (tag == TAG_COMMAND_MSG) {
    destination = summary.commandText;
    seen = &summary.commandTextSeen;
  } else if (tag == TAG_ERROR_MSG) {
    destination = summary.errorText;
    seen = &summary.errorTextSeen;
  }

  if (destination != NULL && seen != NULL) {
    *seen = true;
    copySummaryText(destination, SUMMARY_TEXT_CAPACITY, data, length);
  }
}

bool captureS32Summary(PacketSummary &summary,
                       uint8_t tag,
                       const uint8_t *data,
                       uint8_t count) {
  // Retain known fixed-point flight tags for the end-of-packet summary.
  S32TagSummary *destination = NULL;
  uint8_t expectedCount = 0U;

  switch (tag) {
    case TAG_FLIGHT_ESTIMATE:
      destination = &summary.flightEstimate;
      expectedCount = 5U;
      break;
    case TAG_FLIGHT_COMMAND:
      destination = &summary.flightCommand;
      expectedCount = 5U;
      break;
    case TAG_FLIGHT_HEALTH:
      destination = &summary.flightHealth;
      expectedCount = 5U;
      break;
    case TAG_SYSTEM_STATUS:
      destination = &summary.systemStatus;
      expectedCount = 5U;
      break;
    case TAG_ACTUATOR_STATUS:
      destination = &summary.actuatorStatus;
      expectedCount = 5U;
      break;
    case TAG_APOGEE_DETAIL:
      destination = &summary.apogeeDetail;
      expectedCount = 4U;
      break;
    default:
      return false;
  }

  destination->seen = true;
  destination->count = count;
  const uint8_t storedCount = count < 5U ? count : 5U;
  for (uint8_t i = 0; i < storedCount; ++i) {
    destination->values[i] = tagS32(data, i);
  }

  if (count != expectedCount) {
    char notice[64];
    snprintf(notice,
             sizeof(notice),
             "tag 0x%02X expected %u fields, received %u",
             (unsigned int)tag,
             (unsigned int)expectedCount,
             (unsigned int)count);
    appendParserNotice(summary, notice);
  }
  return true;
}

bool summaryHasField(const S32TagSummary &tag, uint8_t fieldIndex) {
  return tag.seen && tag.count > fieldIndex && fieldIndex < 5U;
}

/* ===================== COMPACT OPERATOR SUMMARY ===================== */

void printSummaryScaled(bool available,
                        int32_t raw,
                        float divisor,
                        uint8_t decimalPlaces,
                        const char *unit) {
  if (!available) {
    Serial.print("N/A");
    return;
  }
  Serial.print((float)raw / divisor, decimalPlaces);
  if (unit != NULL && unit[0] != '\0') {
    Serial.print(' ');
    Serial.print(unit);
  }
}

void printSummaryUnsigned(bool available, uint32_t value) {
  if (available) {
    Serial.print(value);
  } else {
    Serial.print("N/A");
  }
}

void printSummaryText(bool available, const char *text) {
  if (!available) {
    Serial.print("N/A");
  } else if (text == NULL || text[0] == '\0') {
    Serial.print("(empty)");
  } else {
    Serial.print('"');
    Serial.print(text);
    Serial.print('"');
  }
}

void printInlineFlagNames(uint32_t value,
                          const NamedFlag *flags,
                          size_t flagCount) {
  if (value == 0U) {
    Serial.print("none");
    return;
  }

  bool first = true;
  uint32_t remaining = value;
  for (size_t i = 0; i < flagCount; ++i) {
    if ((value & flags[i].mask) != 0U) {
      if (!first) {
        Serial.print('|');
      }
      Serial.print(flags[i].name);
      first = false;
      remaining &= ~flags[i].mask;
    }
  }

  if (remaining != 0U) {
    if (!first) {
      Serial.print('|');
    }
    Serial.print("UNKNOWN(");
    printHex32(remaining);
    Serial.print(')');
  }
}

void printFeatureState(const char *label,
                       uint32_t featureFlags,
                       uint32_t mask) {
  Serial.print(label);
  Serial.print(' ');
  Serial.print((featureFlags & mask) != 0U ? "ON" : "OFF");
}

void printPacketSummary(const PacketSummary &summary) {
  // Present interpreted state after all TLVs have been captured from the packet.
  Serial.println("===== RECEIVED PACKET SUMMARY =====");

  Serial.print("Packet ");
  Serial.print(summary.packetNumber);
  Serial.print(" | sequence ");
  if (summary.headerValid) {
    Serial.print(summary.sequence);
  } else {
    Serial.print("N/A");
  }
  Serial.print(" | STM32 uptime ");
  if (summary.headerValid) {
    Serial.print(summary.timeMs);
    Serial.print(" ms");
  } else {
    Serial.print("N/A");
  }
  Serial.print(" | packet length ");
  Serial.print(summary.packetLength);
  Serial.println(" bytes");

  Serial.print("Flight: phase ");
  if (summaryHasField(summary.flightHealth, 0U)) {
    const int32_t phase = summary.flightHealth.values[0];
    Serial.print(phaseName(phase));
    if (strcmp(phaseName(phase), "UNKNOWN") == 0) {
      Serial.print('(');
      Serial.print(phase);
      Serial.print(')');
    }
  } else {
    Serial.print("N/A");
  }
  Serial.print(" | armed ");
  if (summaryHasField(summary.flightCommand, 3U)) {
    const uint32_t inhibits = (uint32_t)summary.flightCommand.values[3];
    Serial.print((inhibits & (1UL << 6)) == 0U ? "YES" : "NO");
  } else {
    Serial.print("N/A");
  }
  Serial.println();

  Serial.print("Motion: altitude ");
  printSummaryScaled(summaryHasField(summary.flightEstimate, 0U),
                     summary.flightEstimate.values[0], 100.0F, 2U, "m");
  Serial.print(" | vertical velocity ");
  printSummaryScaled(summaryHasField(summary.flightEstimate, 1U),
                     summary.flightEstimate.values[1], 100.0F, 2U, "m/s");
  Serial.print(" | vertical acceleration ");
  printSummaryScaled(summaryHasField(summary.flightEstimate, 2U),
                     summary.flightEstimate.values[2], 100.0F, 2U, "m/s^2");
  Serial.println();

  const bool hasPredictedEstimate = summaryHasField(summary.flightEstimate, 3U);
  const bool hasPredictedCommand = summaryHasField(summary.flightCommand, 1U);
  Serial.print("Apogee: predicted ");
  printSummaryScaled(hasPredictedEstimate || hasPredictedCommand,
                     hasPredictedEstimate
                         ? summary.flightEstimate.values[3]
                         : summary.flightCommand.values[1],
                     100.0F, 2U, "m");
  Serial.print(" | ballistic ");
  printSummaryScaled(summaryHasField(summary.apogeeDetail, 0U),
                     summary.apogeeDetail.values[0], 100.0F, 2U, "m");
  Serial.print(" | drag ");
  printSummaryScaled(summaryHasField(summary.apogeeDetail, 1U),
                     summary.apogeeDetail.values[1], 100.0F, 2U, "m");
  Serial.print(" | target ");
  printSummaryScaled(summaryHasField(summary.flightCommand, 2U),
                     summary.flightCommand.values[2], 100.0F, 2U, "m");
  Serial.println();

  const bool hasCommand = summaryHasField(summary.flightCommand, 0U);
  Serial.print("Airbrake: command ");
  if (hasCommand) {
    printSummaryScaled(true,
                       summary.flightCommand.values[0],
                       10.0F,
                       1U,
                       "%");
  } else if (summary.headerValid) {
    Serial.print(summary.headerDeploymentPercent);
    Serial.print(" %");
  } else {
    Serial.print("N/A");
  }
  Serial.print(" | effectiveness ");
  printSummaryScaled(summaryHasField(summary.apogeeDetail, 3U),
                     summary.apogeeDetail.values[3], 1000.0F, 3U, NULL);
  Serial.print(" | actuator state ");
  if (summaryHasField(summary.actuatorStatus, 0U)) {
    const int32_t state = summary.actuatorStatus.values[0];
    Serial.print(actuatorStateName(state));
    if (strcmp(actuatorStateName(state), "UNKNOWN") == 0) {
      Serial.print('(');
      Serial.print(state);
      Serial.print(')');
    }
  } else {
    Serial.print("N/A");
  }
  Serial.println();

  Serial.print("Sensors: estimator ");
  if (summaryHasField(summary.flightHealth, 1U)) {
    Serial.print(summary.flightHealth.values[1] != 0 ? "HEALTHY" : "UNHEALTHY");
  } else {
    Serial.print("N/A");
  }
  Serial.print(" | rejected IMU samples ");
  printSummaryUnsigned(summaryHasField(summary.flightHealth, 2U),
                       (uint32_t)summary.flightHealth.values[2]);
  Serial.print(" | rejected barometer samples ");
  printSummaryUnsigned(summaryHasField(summary.flightHealth, 3U),
                       (uint32_t)summary.flightHealth.values[3]);
  Serial.print(" | last barometer innovation ");
  printSummaryScaled(summaryHasField(summary.flightHealth, 4U),
                     summary.flightHealth.values[4], 100.0F, 2U, "m");
  Serial.println();

  Serial.print("Sensor setup: pad reference ");
  if (summaryHasField(summary.systemStatus, 4U)) {
    const uint32_t calibration = (uint32_t)summary.systemStatus.values[4];
    Serial.print((calibration & 0x01U) != 0U ? "VALID" : "NOT VALID");
    Serial.print(" | IMU status register 0x");
    printHexByte((uint8_t)((calibration >> 24) & 0xFFU));
  } else {
    Serial.print("N/A | IMU status register N/A");
  }
  Serial.print(" | configuration ");
  if (summaryHasField(summary.systemStatus, 0U)) {
    const uint32_t configFlags = (uint32_t)summary.systemStatus.values[0];
    const uint32_t configErrorMask = (1UL << 3) | (1UL << 4);
    Serial.print((configFlags & configErrorMask) == 0U ? "OK" : "ERROR");
    if ((configFlags & (1UL << 0)) != 0U) {
      Serial.print(" (defaults used)");
    }
  } else {
    Serial.print("N/A");
  }
  Serial.println();

  Serial.print("Features: ");
  const bool featureWordReported = summaryHasField(summary.systemStatus, 0U)
                                && (((uint32_t)summary.systemStatus.values[0]
                                     & REPORTED_FEATURE_MASK) != 0U);
  uint32_t featureFlags = 0U;
  if (featureWordReported) {
    featureFlags = (uint32_t)summary.systemStatus.values[0];
    printFeatureState("radio", featureFlags, 1UL << 16);
    Serial.print(" | ");
    printFeatureState("telemetry", featureFlags, 1UL << 17);
    Serial.print(" | ");
    printFeatureState("flash logging", featureFlags, 1UL << 18);
    Serial.print(" | ");
    printFeatureState("actuator", featureFlags, 1UL << 19);
    Serial.print(" | ");
    printFeatureState("bench actuator commands", featureFlags, 1UL << 20);
    Serial.print(" | ");
    printFeatureState("watchdog", featureFlags, 1UL << 21);
    Serial.print(" | ");
    printFeatureState("magnetometer telemetry", featureFlags, 1UL << 22);
    Serial.print(" | ");
    printFeatureState("verbose status", featureFlags, 1UL << 23);
    Serial.print(" | ");
    printFeatureState("automatic flight phases", featureFlags, 1UL << 24);
    Serial.print(" | ");
    printFeatureState("radio heartbeat", featureFlags, 1UL << 25);
  } else {
    Serial.print("N/A (not reported by this firmware)");
  }
  Serial.println();

  Serial.print("Logging: ");
  const bool loggingFeatureOff = featureWordReported
                              && (featureFlags & (1UL << 18)) == 0U;
  if (loggingFeatureOff) {
    Serial.print("intentionally disabled by build");
  } else if (summaryHasField(summary.systemStatus, 1U)) {
    const uint32_t flashAndLog = (uint32_t)summary.systemStatus.values[1];
    const uint32_t logFlags = flashAndLog >> 8;
    Serial.print("flash ");
    Serial.print((flashAndLog & 0x01U) != 0U ? "OK" : "not ready");
    Serial.print(" | ");
    Serial.print((logFlags & (1UL << 1)) != 0U ? "ACTIVE" : "inactive");
    Serial.print(" | storage ");
    Serial.print((logFlags & (1UL << 2)) != 0U ? "full/wrapped" : "space available");
    Serial.print(" | last write ");
    if ((logFlags & (1UL << 4)) != 0U) {
      Serial.print("FAILED");
    } else if ((logFlags & (1UL << 3)) != 0U) {
      Serial.print("OK");
    } else {
      Serial.print("N/A");
    }
  } else {
    Serial.print("N/A");
  }
  Serial.println();

  Serial.print("Radio: RSSI ");
  Serial.print(summary.rssiDbm, 2);
  Serial.print(" dBm | SNR ");
  Serial.print(summary.snrDb, 2);
  Serial.println(" dB");

  Serial.print("Safety: flight inhibits ");
  if (summaryHasField(summary.flightCommand, 3U)) {
    printInlineFlagNames((uint32_t)summary.flightCommand.values[3],
                         FLIGHT_INHIBIT_SUMMARY_FLAGS,
                         sizeof(FLIGHT_INHIBIT_SUMMARY_FLAGS)
                             / sizeof(FLIGHT_INHIBIT_SUMMARY_FLAGS[0]));
  } else {
    Serial.print("N/A");
  }
  Serial.print(" | actuator inhibits ");
  if (summaryHasField(summary.flightCommand, 4U)) {
    const uint32_t actuatorInhibits = (uint32_t)summary.flightCommand.values[4];
    const bool actuatorDisabledByBuild = (actuatorInhibits & (1UL << 0)) != 0U
                                      && featureWordReported
                                      && (featureFlags & (1UL << 19)) == 0U;
    if (actuatorDisabledByBuild) {
      Serial.print("intentionally disabled by build");
      const uint32_t otherInhibits = actuatorInhibits & ~(1UL << 0);
      if (otherInhibits != 0U) {
        Serial.print('|');
        printInlineFlagNames(otherInhibits,
                             ACTUATOR_INHIBIT_SUMMARY_FLAGS,
                             sizeof(ACTUATOR_INHIBIT_SUMMARY_FLAGS)
                                 / sizeof(ACTUATOR_INHIBIT_SUMMARY_FLAGS[0]));
      }
    } else {
      printInlineFlagNames(actuatorInhibits,
                           ACTUATOR_INHIBIT_SUMMARY_FLAGS,
                           sizeof(ACTUATOR_INHIBIT_SUMMARY_FLAGS)
                               / sizeof(ACTUATOR_INHIBIT_SUMMARY_FLAGS[0]));
    }
  } else {
    Serial.print("N/A");
  }
  Serial.println();

  if (summary.rawTextSeen) {
    Serial.print("Radio text: ");
    printSummaryText(true, summary.rawText);
    Serial.println();
  }

  Serial.print("Messages: status ");
  printSummaryText(summary.statusTextSeen, summary.statusText);
  Serial.print(" | last command ");
  printSummaryText(summary.commandTextSeen, summary.commandText);
  Serial.print(" | warning/error ");
  printSummaryText(summary.errorTextSeen, summary.errorText);
  Serial.println();

  Serial.print("Decoder notice: ");
  if (summary.parserNotice[0] == '\0') {
    Serial.println("none");
  } else {
    Serial.println(summary.parserNotice);
  }

  Serial.println("===== END RECEIVED PACKET SUMMARY =====");
}

/* ===================== DETAILED FLIGHT-TAG DECODERS ===================== */

void printNamedFlags(const char *label,
                     uint32_t value,
                     const NamedFlag *flags,
                     size_t flagCount) {
  Serial.print("  ");
  Serial.print(label);
  Serial.print('=');
  printHex32(value);
  Serial.print(" [");

  if (value == 0U) {
    Serial.print("NONE");
  } else {
    bool first = true;
    uint32_t remaining = value;
    for (size_t i = 0; i < flagCount; ++i) {
      if ((value & flags[i].mask) != 0U) {
        if (!first) {
          Serial.print('|');
        }
        Serial.print(flags[i].name);
        first = false;
        remaining &= ~flags[i].mask;
      }
    }
    if (remaining != 0U) {
      if (!first) {
        Serial.print('|');
      }
      Serial.print("UNKNOWN=");
      printHex32(remaining);
    }
  }

  Serial.println(']');
}

void printScaledField(const char *label,
                      int32_t raw,
                      float divisor,
                      uint8_t decimalPlaces) {
  Serial.print("  ");
  Serial.print(label);
  Serial.print('=');
  Serial.print((float)raw / divisor, decimalPlaces);
  Serial.print(" (raw=");
  Serial.print(raw);
  Serial.println(')');
}

void printIntegerField(const char *label, int32_t value) {
  Serial.print("  ");
  Serial.print(label);
  Serial.print('=');
  Serial.println(value);
}

void printCountWarning(uint8_t actual, uint8_t expected) {
  if (actual != expected) {
    Serial.print("  WARN,expected_count=");
    Serial.print(expected);
    Serial.print(",actual_count=");
    Serial.println(actual);
  }
}

void printExtraS32Fields(const uint8_t *data, uint8_t count, uint8_t knownCount) {
  for (uint8_t i = knownCount; i < count; ++i) {
    Serial.print("  extra[");
    Serial.print(i);
    Serial.print("]=");
    Serial.println(tagS32(data, i));
  }
}

void decodeFlightEstimate(const uint8_t *data, uint8_t count) {
  Serial.println("TAG_0x60,FLIGHT_ESTIMATE");
  printCountWarning(count, 5);
  if (count > 0) printScaledField("altitude_agl_m", tagS32(data, 0), 100.0F, 2);
  if (count > 1) printScaledField("vertical_velocity_mps", tagS32(data, 1), 100.0F, 2);
  if (count > 2) printScaledField("vertical_acceleration_mps2", tagS32(data, 2), 100.0F, 2);
  if (count > 3) printScaledField("predicted_apogee_m", tagS32(data, 3), 100.0F, 2);
  if (count > 4) printScaledField("barometer_bias_m", tagS32(data, 4), 100.0F, 2);
  printExtraS32Fields(data, count, 5);
}

void decodeFlightCommand(const uint8_t *data, uint8_t count) {
  Serial.println("TAG_0x61,FLIGHT_COMMAND");
  printCountWarning(count, 5);
  if (count > 0) {
    const int32_t raw = tagS32(data, 0);
    printScaledField("deploy_fraction", raw, 1000.0F, 3);
    printScaledField("deploy_percent", raw, 10.0F, 1);
  }
  if (count > 1) printScaledField("predicted_apogee_m", tagS32(data, 1), 100.0F, 2);
  if (count > 2) printScaledField("target_apogee_m", tagS32(data, 2), 100.0F, 2);
  if (count > 3) {
    printNamedFlags("flight_inhibit_flags",
                    (uint32_t)tagS32(data, 3),
                    FLIGHT_INHIBIT_FLAGS,
                    sizeof(FLIGHT_INHIBIT_FLAGS) / sizeof(FLIGHT_INHIBIT_FLAGS[0]));
  }
  if (count > 4) {
    printNamedFlags("actuator_inhibit_flags",
                    (uint32_t)tagS32(data, 4),
                    ACTUATOR_INHIBIT_FLAGS,
                    sizeof(ACTUATOR_INHIBIT_FLAGS) / sizeof(ACTUATOR_INHIBIT_FLAGS[0]));
  }
  printExtraS32Fields(data, count, 5);
}

void decodeFlightHealth(const uint8_t *data, uint8_t count) {
  Serial.println("TAG_0x62,FLIGHT_HEALTH");
  printCountWarning(count, 5);
  if (count > 0) {
    const int32_t phase = tagS32(data, 0);
    Serial.print("  phase=");
    Serial.print(phase);
    Serial.print(" [");
    Serial.print(phaseName(phase));
    Serial.println(']');
  }
  if (count > 1) {
    Serial.print("  estimate_healthy=");
    Serial.println(tagS32(data, 1) != 0 ? "true" : "false");
  }
  if (count > 2) {
    Serial.print("  rejected_imu_samples=");
    Serial.println((uint32_t)tagS32(data, 2));
  }
  if (count > 3) {
    Serial.print("  rejected_barometer_samples=");
    Serial.println((uint32_t)tagS32(data, 3));
  }
  if (count > 4) {
    printScaledField("last_barometer_innovation_m", tagS32(data, 4), 100.0F, 2);
  }
  printExtraS32Fields(data, count, 5);
}

void decodeSystemStatus(const uint8_t *data, uint8_t count) {
  Serial.println("TAG_0x63,SYSTEM_STATUS");
  printCountWarning(count, 5);
  if (count > 0) {
    printNamedFlags("config_flags",
                    (uint32_t)tagS32(data, 0),
                    CONFIG_STATUS_FLAGS,
                    sizeof(CONFIG_STATUS_FLAGS) / sizeof(CONFIG_STATUS_FLAGS[0]));
  }
  if (count > 1) {
    const uint32_t combined = (uint32_t)tagS32(data, 1);
    Serial.print("  flash_log_flags=");
    printHex32(combined);
    Serial.print(",stm32_flash_ok=");
    Serial.println((combined & 0x01U) != 0U ? "true" : "false");
    printNamedFlags("log_flags",
                    combined >> 8,
                    LOG_STATUS_FLAGS,
                    sizeof(LOG_STATUS_FLAGS) / sizeof(LOG_STATUS_FLAGS[0]));
  }
  if (count > 2) {
    const int32_t action = tagS32(data, 2);
    Serial.print("  command_action=");
    Serial.print(action);
    Serial.print(" [");
    Serial.print(commandActionName(action));
    Serial.println(']');
  }
  if (count > 3) {
    const int32_t ack = tagS32(data, 3);
    Serial.print("  command_ack=");
    Serial.print(ack);
    Serial.print(" [");
    Serial.print(commandAckName(ack));
    Serial.println(']');
  }
  if (count > 4) {
    const uint32_t flags = (uint32_t)tagS32(data, 4);
    const uint8_t axis = (uint8_t)((flags >> 8) & 0xFFU);
    const uint8_t signCode = (uint8_t)((flags >> 16) & 0xFFU);
    const uint8_t imuStatus = (uint8_t)((flags >> 24) & 0xFFU);
    Serial.print("  calibration_flags=");
    printHex32(flags);
    Serial.print(",pad_reference_valid=");
    Serial.print((flags & 0x01U) != 0U ? "true" : "false");
    Serial.print(",vertical_axis_index=");
    Serial.print(axis);
    Serial.print(",vertical_axis_sign=");
    if (signCode == 1U) {
      Serial.print("positive");
    } else if (signCode == 2U) {
      Serial.print("negative");
    } else {
      Serial.print("unknown");
    }
    Serial.print(",imu_status_reg=0x");
    printHexByte(imuStatus);
    Serial.println();
  }
  printExtraS32Fields(data, count, 5);
}

void decodeActuatorStatus(const uint8_t *data, uint8_t count) {
  Serial.println("TAG_0x64,ACTUATOR_STATUS");
  printCountWarning(count, 5);
  if (count > 0) {
    const int32_t state = tagS32(data, 0);
    Serial.print("  actuator_state=");
    Serial.print(state);
    Serial.print(" [");
    Serial.print(actuatorStateName(state));
    Serial.println(']');
  }
  if (count > 1) printIntegerField("actuator_target_steps", tagS32(data, 1));
  if (count > 2) printIntegerField("actuator_actual_steps", tagS32(data, 2));
  if (count > 3) {
    Serial.print("  tmc_driver_status=");
    printHex32((uint32_t)tagS32(data, 3));
    Serial.println();
  }
  if (count > 4) {
    const uint32_t pins = (uint32_t)tagS32(data, 4);
    Serial.print("  tmc_diag_pins=");
    printHex32(pins);
    Serial.print(",diag0=");
    Serial.print((pins & 0x01U) != 0U ? "HIGH" : "LOW");
    Serial.print(",diag1=");
    Serial.println((pins & 0x02U) != 0U ? "HIGH" : "LOW");
  }
  printExtraS32Fields(data, count, 5);
}

void decodeApogeeDetail(const uint8_t *data, uint8_t count) {
  Serial.println("TAG_0x65,APOGEE_DETAIL");
  printCountWarning(count, 4);
  if (count > 0) printScaledField("ballistic_apogee_m", tagS32(data, 0), 100.0F, 2);
  if (count > 1) printScaledField("drag_apogee_m", tagS32(data, 1), 100.0F, 2);
  if (count > 2) printScaledField("drag_area_m2", tagS32(data, 2), 1000000.0F, 6);
  if (count > 3) printScaledField("actuator_effectiveness", tagS32(data, 3), 1000.0F, 3);
  printExtraS32Fields(data, count, 4);
}

bool decodeS32Tag(uint8_t tag,
                  const uint8_t *data,
                  uint8_t count) {
  switch (tag) {
    case TAG_FLIGHT_ESTIMATE:
      decodeFlightEstimate(data, count);
      return true;
    case TAG_FLIGHT_COMMAND:
      decodeFlightCommand(data, count);
      return true;
    case TAG_FLIGHT_HEALTH:
      decodeFlightHealth(data, count);
      return true;
    case TAG_SYSTEM_STATUS:
      decodeSystemStatus(data, count);
      return true;
    case TAG_ACTUATOR_STATUS:
      decodeActuatorStatus(data, count);
      return true;
    case TAG_APOGEE_DETAIL:
      decodeApogeeDetail(data, count);
      return true;
    default:
      return false;
  }
}

/* ===================== PACKET RECOGNITION AND BOUNDED TLV PARSING ===================== */

void printHexPacket(const uint8_t *data, size_t length) {
  Serial.print("HEX=");
  for (size_t i = 0; i < length; ++i) {
    printHexByte(data[i]);
    if (i + 1U < length) {
      Serial.print(' ');
    }
  }
  Serial.println();
}

bool isTelemetryPacket(const uint8_t *data, size_t length) {
  if (length > 0U && data[0] == PAYLOAD_MAGIC) {
    return true;
  }
  return length > 1U
      && data[0] == PAYLOAD_TYPE_DATA
      && data[1] == PAYLOAD_MAGIC;
}

void printTextPacket(const uint8_t *data, size_t length) {
  Serial.print("TEXT=");
  for (size_t i = 0; i < length; ++i) {
    const uint8_t value = data[i];
    if (value >= 0x20U && value <= 0x7EU) {
      Serial.write(value);
    } else {
      Serial.print("\\x");
      printHexByte(value);
    }
  }
  Serial.println();
}

bool requireBytes(size_t index,
                  size_t required,
                  size_t length,
                  uint8_t tag,
                  bool printDetail) {
  // Central bounds check shared by every variable-length TLV branch.
  if (required <= length - index) {
    return true;
  }

  if (printDetail) {
    Serial.print("TELEM_ERR,truncated_tag=0x");
    printHexByte(tag);
    Serial.print(",needed=");
    Serial.print(required);
    Serial.print(",remaining=");
    Serial.println(length - index);
  }
  return false;
}

void appendTruncatedNotice(PacketSummary &summary,
                           uint8_t tag,
                           size_t required,
                           size_t remaining) {
  char notice[80];
  snprintf(notice,
           sizeof(notice),
           "truncated tag 0x%02X: needed %u bytes, received %u",
           (unsigned int)tag,
           (unsigned int)required,
           (unsigned int)remaining);
  appendParserNotice(summary, notice);
}

void decodeU16Section(uint8_t tag,
                      const uint8_t *data,
                      uint8_t count) {
  static const char *IMU_NAMES[] = {
    "gyro_x_raw", "gyro_y_raw", "gyro_z_raw",
    "accel_x_raw", "accel_y_raw", "accel_z_raw"
  };
  static const char *MAG_NAMES[] = {
    "mag_x_raw", "mag_y_raw", "mag_z_raw"
  };
  static const char *CALC_NAMES[] = {
    "legacy_altitude", "legacy_predicted_apogee",
    "deployment_percent", "flight_phase"
  };

  Serial.print("TAG_0x");
  printHexByte(tag);
  Serial.println(tag == TAG_IMU ? ",IMU_RAW"
                 : tag == TAG_MAGNET ? ",MAGNET_RAW"
                 : ",LEGACY_CALC");

  for (uint8_t n = 0; n < count; ++n) {
    const uint16_t raw = readU16LE(data, (size_t)n * 2U);
    const char *name = NULL;
    if (tag == TAG_IMU && n < 6U) name = IMU_NAMES[n];
    if (tag == TAG_MAGNET && n < 3U) name = MAG_NAMES[n];
    if (tag == TAG_CALC && n < 4U) name = CALC_NAMES[n];

    Serial.print("  ");
    if (name != NULL) {
      Serial.print(name);
    } else {
      Serial.print('[');
      Serial.print(n);
      Serial.print(']');
    }
    Serial.print("=");
    Serial.print(raw);
    if (tag == TAG_IMU || tag == TAG_MAGNET) {
      Serial.print(",signed=");
      Serial.print((int16_t)raw);
    }
    Serial.println();
  }
}

void decodeU32Section(const uint8_t *data, uint8_t count) {
  Serial.println("TAG_0x20,BARO_RAW");
  for (uint8_t n = 0; n < count; ++n) {
    Serial.print("  baro[");
    Serial.print(n);
    Serial.print("]=");
    Serial.println(readU32LE(data, (size_t)n * 4U));
  }
}

void decodeStringSection(uint8_t tag,
                         const uint8_t *data,
                         uint8_t length) {
  Serial.print("TAG_0x");
  printHexByte(tag);
  Serial.print(tag == TAG_STATUS_MSG ? ",STATUS_MSG="
               : tag == TAG_COMMAND_MSG ? ",COMMAND_MSG="
               : ",ERROR_MSG=");
  for (uint8_t n = 0; n < length; ++n) {
    Serial.write(data[n]);
  }
  Serial.println();
}

void parseTelemetryPacket(const uint8_t *data,
                          size_t length,
                          PacketSummary &summary) {
  // Accept the legacy optional leading type byte, then walk one TLV at a time.
  size_t index = 0;
  const bool printDetail = detailOutputEnabled();
  summary.telemetry = true;

  if (length > 1U
      && data[0] == PAYLOAD_TYPE_DATA
      && data[1] == PAYLOAD_MAGIC) {
    if (printDetail) {
      Serial.println("TELEM_NOTE,leading_payload_type=0x01");
    }
    index = 1;
  }

  // Header: magic, version, type, sequence (LE), time_ms (LE), deployment percent.
  if (length - index < 10U) {
    if (printDetail) {
      Serial.println("TELEM_ERR,packet_too_short_for_header");
    }
    appendParserNotice(summary, "packet too short for telemetry header");
    return;
  }
  if (data[index] != PAYLOAD_MAGIC) {
    if (printDetail) {
      Serial.println("TELEM_ERR,bad_magic");
    }
    appendParserNotice(summary, "bad telemetry magic");
    return;
  }

  const uint8_t version = data[index + 1U];
  const uint8_t type = data[index + 2U];
  const uint16_t sequence = readU16LE(data, index + 3U);
  index += 5U;
  const uint32_t timeMs = readU32LE(data, index);
  index += 4U;
  const uint8_t deploymentPercent = data[index++];

  summary.headerValid = true;
  summary.sequence = sequence;
  summary.timeMs = timeMs;
  summary.headerDeploymentPercent = deploymentPercent;
  if (version != 1U || type != PAYLOAD_TYPE_DATA) {
    appendParserNotice(summary, "unsupported telemetry version or type");
  }

  if (printDetail) {
    Serial.println("----- AMBAR TELEMETRY -----");
    Serial.print("HEADER,version=");
    Serial.print(version);
    Serial.print(",type=");
    Serial.print(type);
    Serial.print(",sequence=");
    Serial.print(sequence);
    Serial.print(",time_ms=");
    Serial.print(timeMs);
    Serial.print(",deployment_percent=");
    Serial.println(deploymentPercent);
  }

  while (index < length) {
    if (length - index < 2U) {
      if (printDetail) {
        Serial.println("TELEM_ERR,trailing_byte_without_tag_header");
      }
      appendParserNotice(summary, "trailing byte without tag header");
      break;
    }

    const uint8_t tag = data[index++];
    const uint8_t countOrLength = data[index++];

    if (tag == TAG_IMU || tag == TAG_MAGNET || tag == TAG_CALC) {
      const size_t byteCount = (size_t)countOrLength * 2U;
      if (!requireBytes(index, byteCount, length, tag, printDetail)) {
        appendTruncatedNotice(summary, tag, byteCount, length - index);
        break;
      }
      if (printDetail) {
        decodeU16Section(tag, &data[index], countOrLength);
      }
      index += byteCount;
    } else if (tag == TAG_BARO) {
      const size_t byteCount = (size_t)countOrLength * 4U;
      if (!requireBytes(index, byteCount, length, tag, printDetail)) {
        appendTruncatedNotice(summary, tag, byteCount, length - index);
        break;
      }
      if (printDetail) {
        decodeU32Section(&data[index], countOrLength);
      }
      index += byteCount;
    } else if (tag == TAG_STATUS_MSG
               || tag == TAG_COMMAND_MSG
               || tag == TAG_ERROR_MSG) {
      if (!requireBytes(index, countOrLength, length, tag, printDetail)) {
        appendTruncatedNotice(summary, tag, countOrLength, length - index);
        break;
      }
      if (printDetail) {
        decodeStringSection(tag, &data[index], countOrLength);
      }
      captureTextSummary(summary, tag, &data[index], countOrLength);
      index += countOrLength;
    } else if (tag >= TAG_FLIGHT_ESTIMATE && tag <= TAG_APOGEE_DETAIL) {
      const size_t byteCount = (size_t)countOrLength * 4U;
      if (!requireBytes(index, byteCount, length, tag, printDetail)) {
        appendTruncatedNotice(summary, tag, byteCount, length - index);
        break;
      }
      const bool supported = captureS32Summary(summary,
                                               tag,
                                               &data[index],
                                               countOrLength);
      if (printDetail && supported) {
        (void)decodeS32Tag(tag, &data[index], countOrLength);
      }
      if (!supported) {
        if (printDetail) {
          Serial.println("TELEM_ERR,unsupported_s32_tag");
        }
        appendParserNotice(summary, "unsupported signed telemetry tag");
        break;
      }
      index += byteCount;
    } else {
      if (printDetail) {
        Serial.print("TELEM_ERR,unknown_tag=0x");
        printHexByte(tag);
        Serial.print(",count_or_length=");
        Serial.println(countOrLength);
      }
      char notice[64];
      snprintf(notice,
               sizeof(notice),
               "unknown tag 0x%02X with count/length %u",
               (unsigned int)tag,
               (unsigned int)countOrLength);
      appendParserNotice(summary, notice);
      break;
    }
  }

  if (printDetail) {
    Serial.println("---------------------------");
  }
}

/* ===================== RADIO ISR, TRANSPORT, AND CONSOLE SERVICE ===================== */

void onRadioDio1() {
  // ISR callback: defer SPI reads and all Serial output to handleRadioReceive().
  rxFlag = true;
  ++dio1Count;
}

void startReceiveMode() {
  // Restore continuous receive after startup, a packet read, or blocking transmit.
  const int state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("RX_START_ERR,");
    Serial.println(state);
  }
}

void sendPayload(String payload) {
  // Forward exact console bytes within the STM32 radio-command payload limit.
  if (payload.length() == 0U) {
    Serial.println("TX_REJECT,reason=empty_payload");
    return;
  }
  if (payload.length() > STM32_MAX_PAYLOAD_LEN) {
    Serial.print("TX_REJECT,reason=payload_too_long,length=");
    Serial.print(payload.length());
    Serial.print(",max=");
    Serial.println(STM32_MAX_PAYLOAD_LEN);
    return;
  }

  Serial.print("TX_START,LEN=");
  Serial.print(payload.length());
  Serial.print(",PAYLOAD=");
  Serial.println(payload);

  rxFlag = false;
  radio.standby();
  const int state = radio.transmit(payload);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.print("TX_DONE,PAYLOAD=");
    Serial.println(payload);
  } else {
    Serial.print("TX_ERR,");
    Serial.println(state);
  }

  // A blocking transmit can also assert DIO1 for TX done; discard that flag.
  rxFlag = false;
  startReceiveMode();
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  tx <payload>   Send exactly the bytes after 'tx '");
  Serial.println("  view both      Detailed decoder followed by summary (default)");
  Serial.println("  view summary   End-of-packet summary only");
  Serial.println("  view detail    Detailed decoder only");
  Serial.println("  help           Show this help");
  Serial.println("Examples:");
  Serial.println("  tx PING");
  Serial.println("  tx STATUS");
  Serial.println("Missing summary values are shown as N/A.");
}

void handleSerialConsole() {
  // Process at most one newline-terminated operator command per loop pass.
  if (!Serial.available()) {
    return;
  }

  String line = Serial.readStringUntil('\n');
  if (line.endsWith("\r")) {
    line.remove(line.length() - 1U);
  }

  if (line.startsWith("tx ")) {
    // Do not trim or uppercase: the substring is the exact radio payload.
    sendPayload(line.substring(3));
  } else if (line == "view both") {
    viewMode = VIEW_BOTH;
    Serial.println("VIEW_MODE,both");
  } else if (line == "view summary") {
    viewMode = VIEW_SUMMARY;
    Serial.println("VIEW_MODE,summary");
  } else if (line == "view detail") {
    viewMode = VIEW_DETAIL;
    Serial.println("VIEW_MODE,detail");
  } else if (line == "help") {
    printHelp();
  } else if (line.length() > 0U) {
    Serial.println("CONSOLE_ERR,use: tx <payload>, view both, view summary, view detail, or help");
  }
}

void handleRadioReceive() {
  // Read and decode one latched packet, then always return the radio to RX mode.
  if (!rxFlag) {
    return;
  }
  rxFlag = false;

  uint8_t rxBuffer[255];
  size_t packetLength = radio.getPacketLength();
  if (packetLength == 0U) {
    Serial.println("RX_ERR,zero_length_packet");
    startReceiveMode();
    return;
  }
  if (packetLength > sizeof(rxBuffer)) {
    Serial.print("RX_ERR,packet_too_large,length=");
    Serial.println(packetLength);
    startReceiveMode();
    return;
  }

  const int state = radio.readData(rxBuffer, packetLength);
  if (state == RADIOLIB_ERR_NONE) {
    const float rssiDbm = radio.getRSSI();
    const float snrDb = radio.getSNR();
    ++receivedPacketCount;

    PacketSummary summary;
    initializePacketSummary(summary,
                            receivedPacketCount,
                            packetLength,
                            rssiDbm,
                            snrDb);

    if (detailOutputEnabled()) {
      Serial.print("RX,RSSI_DBM=");
      Serial.print(rssiDbm);
      Serial.print(",SNR_DB=");
      Serial.print(snrDb);
      Serial.print(",LEN=");
      Serial.println(packetLength);
      printHexPacket(rxBuffer, packetLength);
    }

    if (isTelemetryPacket(rxBuffer, packetLength)) {
      parseTelemetryPacket(rxBuffer, packetLength, summary);
    } else {
      if (detailOutputEnabled()) {
        printTextPacket(rxBuffer, packetLength);
      }
      summary.rawTextSeen = true;
      copySummaryText(summary.rawText,
                      sizeof(summary.rawText),
                      rxBuffer,
                      packetLength);
    }

    if (summaryOutputEnabled()) {
      printPacketSummary(summary);
    }
  } else {
    Serial.print("RX_ERR,");
    Serial.println(state);
  }

  startReceiveMode();
}

/* ===================== ARDUINO LIFECYCLE ===================== */

void setup() {
  // Initialize USB serial first so radio bring-up failures remain visible.
  Serial.begin(115200);
  Serial.setTimeout(50);
  delay(2000);

  Serial.println();
  Serial.println("AMBAR LILYGO T3-S3 SX1280PA ground station starting...");
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

  // RadioLib requires receive-enable first and transmit-enable second.
  radio.setRfSwitchPins(LORA_RXEN, LORA_TXEN);

  const int state = radio.begin(
    RADIO_FREQUENCY_MHZ,
    RADIO_BANDWIDTH_KHZ,
    RADIO_SPREADING_FACTOR,
    RADIO_CODING_RATE_DENOMINATOR,
    RADIOLIB_SX128X_SYNC_WORD_PRIVATE,
    RADIO_OUTPUT_POWER_DBM,
    RADIO_PREAMBLE_LENGTH
  );

  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("RADIO_INIT_ERR,");
    Serial.println(state);
    while (true) {
      delay(1000);
    }
  }

  radio.explicitHeader();
  radio.setCRC(2);
  radio.setDio1Action(onRadioDio1);

  Serial.println("RADIO_READY,2445MHz,SF7,BW203.125kHz,CR4/5,PRIVATE_SYNC,PREAMBLE12,CRC_ON");
  printHelp();
  startReceiveMode();
}

void loop() {
  // Cooperative service loop: console, pending radio packet, then quiet heartbeat.
  handleSerialConsole();
  handleRadioReceive();

  const uint32_t now = millis();
  if (viewMode != VIEW_SUMMARY
      && (uint32_t)(now - lastWaitingPrintMs) >= 5000U) {
    lastWaitingPrintMs = now;
    Serial.print("WAITING_RX,dio1_count=");
    Serial.println(dio1Count);
  }
}
