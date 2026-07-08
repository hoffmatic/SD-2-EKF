/*
  AMBAR SX1280 Ground Receiver
  Created: 2026-07-07

  What this sketch is for:
    This Arduino/LILYGO sketch receives packets from the STM32 Airbrake PCB
    firmware. It fixes the common "radio is receiving errors/no responses"
    problem caused by the receiver treating every packet as readable text.

  What the STM32 sends:
    1. Short readable ASCII messages, such as STM32_BOOT and STM32_HEARTBEAT_0.
    2. Binary telemetry packets. Newer packets may start with 0xA5, 0x01, 0x01.
       The current STM32 project copy may also put one leading 0x01 byte before
       that header, so this sketch accepts both forms.

  How to use:
    1. Install the RadioLib Arduino library.
    2. Select the correct ESP32/LILYGO board in Arduino IDE.
    3. Check the pin numbers below and edit them if your board is different.
    4. Upload this sketch.
    5. Open Serial Monitor at 115200 baud.

  Important:
    The pin numbers are the only part likely to need changing. The radio
    frequency/settings below match the STM32 firmware that Codex generated.
*/

#include <Arduino.h>
#include <RadioLib.h>

/*
  ===================== BOARD PIN SETTINGS =====================

  These defaults match the basic RadioLib SX1280 example:
    NSS/CS = 10, DIO1 = 2, RESET = 3, BUSY = 9

  Many LILYGO ESP32 SX1280 boards use different pins. If the Serial Monitor
  says "Radio init failed", update these four numbers to match Ethan's board.

  Common names in board docs:
    NSS may be called CS, LORA_CS, RADIO_CS, or SX1280_NSS.
    DIO1 may be called LORA_DIO1 or RADIO_DIO1.
    RESET may be called RST, NRST, LORA_RST, or RADIO_RST.
    BUSY may be called LORA_BUSY or RADIO_BUSY.
*/
#define RADIO_NSS_PIN   10
#define RADIO_DIO1_PIN   2
#define RADIO_RST_PIN    3
#define RADIO_BUSY_PIN   9

/*
  Some boards need custom SPI pins. If Ethan's board package already sets the
  right SPI pins, leave USE_CUSTOM_SPI_PINS at 0. If the radio will not init,
  set this to 1 and fill in the SCK/MISO/MOSI pins from the board schematic.
*/
#define USE_CUSTOM_SPI_PINS 0
#define RADIO_SCK_PIN      12
#define RADIO_MISO_PIN     13
#define RADIO_MOSI_PIN     11

/*
  ===================== STM32 RADIO SETTINGS =====================

  These match Core/Src/sx1280.c in the STM32 firmware:
    frequency = 2445 MHz
    bandwidth = 203.125 kHz
    spreading factor = SF7
    coding rate = 4/5
    sync word = 0x12
    CRC enabled
*/
static const float RADIO_FREQ_MHZ = 2445.0f;
static const float RADIO_BW_KHZ = 203.125f;
static const uint8_t RADIO_SF = 7;
static const uint8_t RADIO_CR = 5;
static const uint8_t RADIO_SYNC_WORD = 0x12;
static const int8_t RADIO_TX_POWER_DBM = 2;
static const uint16_t RADIO_PREAMBLE_SYMBOLS = 12;
static const RadioLibTime_t RX_TIMEOUT_US = 500000;

static const uint8_t PAYLOAD_MAGIC = 0xA5;
static const uint8_t PAYLOAD_VERSION = 0x01;
static const uint8_t PAYLOAD_TYPE_DATA = 0x01;
static const size_t MAX_PACKET_LEN = 255;

SX1280 radio = new Module(RADIO_NSS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);

static uint8_t rxBuffer[MAX_PACKET_LEN];

static uint16_t readU16LE(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t readU32LE(const uint8_t *p) {
  return (uint32_t)p[0]
      | ((uint32_t)p[1] << 8)
      | ((uint32_t)p[2] << 16)
      | ((uint32_t)p[3] << 24);
}

static int32_t readS32LE(const uint8_t *p) {
  return (int32_t)readU32LE(p);
}

static void printHexPreview(const uint8_t *data, size_t len) {
  Serial.print(F("hex="));
  const size_t n = (len < 24) ? len : 24;
  for (size_t i = 0; i < n; i++) {
    if (data[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(data[i], HEX);
    if (i + 1 < n) {
      Serial.print(' ');
    }
  }
  if (len > n) {
    Serial.print(F(" ..."));
  }
  Serial.println();
}

static bool looksLikeTextPacket(const uint8_t *data, size_t len) {
  if (len == 0) {
    return false;
  }

  for (size_t i = 0; i < len; i++) {
    const uint8_t c = data[i];
    if (c == '\r' || c == '\n' || c == '\t') {
      continue;
    }
    if (c < 32 || c > 126) {
      return false;
    }
  }
  return true;
}

static void printTextPacket(const uint8_t *data, size_t len) {
  Serial.print(F("TEXT len="));
  Serial.print(len);
  Serial.print(F("  \""));
  for (size_t i = 0; i < len; i++) {
    Serial.print((char)data[i]);
  }
  Serial.println(F("\""));
}

static void printS32Scaled(const __FlashStringHelper *label, int32_t value, float scale) {
  Serial.print(label);
  Serial.print((float)value / scale, 2);
}

static void printU16Array(const uint8_t *payload, size_t count) {
  for (size_t i = 0; i < count; i++) {
    if (i > 0) {
      Serial.print(F(", "));
    }
    Serial.print(readU16LE(&payload[i * 2]));
  }
}

static void printU32Array(const uint8_t *payload, size_t count) {
  for (size_t i = 0; i < count; i++) {
    if (i > 0) {
      Serial.print(F(", "));
    }
    Serial.print(readU32LE(&payload[i * 4]));
  }
}

static void printS32Array(const uint8_t *payload, size_t count) {
  for (size_t i = 0; i < count; i++) {
    if (i > 0) {
      Serial.print(F(", "));
    }
    Serial.print(readS32LE(&payload[i * 4]));
  }
}

static void printStringSection(const __FlashStringHelper *name, const uint8_t *payload, size_t count) {
  Serial.print(name);
  Serial.print(F(": \""));
  for (size_t i = 0; i < count; i++) {
    const char c = (char)payload[i];
    if (c >= 32 && c <= 126) {
      Serial.print(c);
    } else {
      Serial.print('.');
    }
  }
  Serial.println(F("\""));
}

static void printFlightEstimate(const uint8_t *payload, size_t count) {
  if (count < 5) {
    Serial.print(F("EKF estimate malformed count="));
    Serial.println(count);
    return;
  }
  const int32_t alt = readS32LE(&payload[0]);
  const int32_t vel = readS32LE(&payload[4]);
  const int32_t accel = readS32LE(&payload[8]);
  const int32_t apogee = readS32LE(&payload[12]);
  const int32_t baroBias = readS32LE(&payload[16]);

  Serial.print(F("EKF: "));
  printS32Scaled(F("alt_m="), alt, 100.0f);
  Serial.print(F(", "));
  printS32Scaled(F("vel_mps="), vel, 100.0f);
  Serial.print(F(", "));
  printS32Scaled(F("accel_mps2="), accel, 100.0f);
  Serial.print(F(", "));
  printS32Scaled(F("apogee_m="), apogee, 100.0f);
  Serial.print(F(", "));
  printS32Scaled(F("baro_bias_m="), baroBias, 100.0f);
  Serial.println();
}

static void printFlightCommand(const uint8_t *payload, size_t count) {
  if (count < 5) {
    Serial.print(F("Command malformed count="));
    Serial.println(count);
    return;
  }
  const int32_t deploy = readS32LE(&payload[0]);
  const int32_t predicted = readS32LE(&payload[4]);
  const int32_t target = readS32LE(&payload[8]);
  const uint32_t flightInhibit = (uint32_t)readS32LE(&payload[12]);
  const uint32_t actuatorInhibit = (uint32_t)readS32LE(&payload[16]);

  Serial.print(F("CMD: deploy="));
  Serial.print((float)deploy / 1000.0f, 3);
  Serial.print(F(", predicted_m="));
  Serial.print((float)predicted / 100.0f, 2);
  Serial.print(F(", target_m="));
  Serial.print((float)target / 100.0f, 2);
  Serial.print(F(", flight_inhibit=0x"));
  Serial.print(flightInhibit, HEX);
  Serial.print(F(", actuator_inhibit=0x"));
  Serial.println(actuatorInhibit, HEX);
}

static void printFlightHealth(const uint8_t *payload, size_t count) {
  if (count < 5) {
    Serial.print(F("Health malformed count="));
    Serial.println(count);
    return;
  }
  Serial.print(F("HEALTH: phase="));
  Serial.print(readS32LE(&payload[0]));
  Serial.print(F(", healthy="));
  Serial.print(readS32LE(&payload[4]));
  Serial.print(F(", rejected_imu="));
  Serial.print(readS32LE(&payload[8]));
  Serial.print(F(", rejected_baro="));
  Serial.print(readS32LE(&payload[12]));
  Serial.print(F(", last_baro_innovation_m="));
  Serial.println((float)readS32LE(&payload[16]) / 100.0f, 2);
}

static void printSystemStatus(const uint8_t *payload, size_t count) {
  if (count < 5) {
    Serial.print(F("System status malformed count="));
    Serial.println(count);
    return;
  }
  Serial.print(F("SYSTEM: config=0x"));
  Serial.print((uint32_t)readS32LE(&payload[0]), HEX);
  Serial.print(F(", flash_log=0x"));
  Serial.print((uint32_t)readS32LE(&payload[4]), HEX);
  Serial.print(F(", command_action="));
  Serial.print(readS32LE(&payload[8]));
  Serial.print(F(", command_ack="));
  Serial.print(readS32LE(&payload[12]));
  Serial.print(F(", calibration=0x"));
  Serial.println((uint32_t)readS32LE(&payload[16]), HEX);
}

static void printActuatorStatus(const uint8_t *payload, size_t count) {
  if (count < 5) {
    Serial.print(F("Actuator status malformed count="));
    Serial.println(count);
    return;
  }
  Serial.print(F("ACTUATOR: state="));
  Serial.print(readS32LE(&payload[0]));
  Serial.print(F(", target_steps="));
  Serial.print(readS32LE(&payload[4]));
  Serial.print(F(", actual_steps="));
  Serial.print(readS32LE(&payload[8]));
  Serial.print(F(", tmc_status=0x"));
  Serial.print((uint32_t)readS32LE(&payload[12]), HEX);
  Serial.print(F(", diag_pins=0x"));
  Serial.println((uint32_t)readS32LE(&payload[16]), HEX);
}

static void printApogeeDetail(const uint8_t *payload, size_t count) {
  if (count < 4) {
    Serial.print(F("Apogee detail malformed count="));
    Serial.println(count);
    return;
  }
  Serial.print(F("APOGEE: ballistic_m="));
  Serial.print((float)readS32LE(&payload[0]) / 100.0f, 2);
  Serial.print(F(", drag_m="));
  Serial.print((float)readS32LE(&payload[4]) / 100.0f, 2);
  Serial.print(F(", drag_area_m2="));
  Serial.print((float)readS32LE(&payload[8]) / 1000000.0f, 6);
  Serial.print(F(", actuator_effectiveness="));
  Serial.println((float)readS32LE(&payload[12]) / 1000.0f, 3);
}

static void parseTaggedSection(uint8_t tag, const uint8_t *payload, size_t count, size_t byteCount) {
  switch (tag) {
    case 0x10:
      Serial.print(F("IMU raw: "));
      printU16Array(payload, count);
      Serial.println();
      break;

    case 0x20:
      Serial.print(F("BARO raw: "));
      printU32Array(payload, count);
      Serial.println();
      break;

    case 0x30:
      Serial.print(F("MAG raw: "));
      printU16Array(payload, count);
      Serial.println();
      break;

    case 0x40:
      Serial.print(F("CALC legacy: "));
      printU16Array(payload, count);
      Serial.println();
      break;

    case 0x50:
      printStringSection(F("STATUS"), payload, count);
      break;

    case 0x51:
      printStringSection(F("COMMAND"), payload, count);
      break;

    case 0x52:
      printStringSection(F("ERROR"), payload, count);
      break;

    case 0x60:
      printFlightEstimate(payload, count);
      break;

    case 0x61:
      printFlightCommand(payload, count);
      break;

    case 0x62:
      printFlightHealth(payload, count);
      break;

    case 0x63:
      printSystemStatus(payload, count);
      break;

    case 0x64:
      printActuatorStatus(payload, count);
      break;

    case 0x65:
      printApogeeDetail(payload, count);
      break;

    default:
      Serial.print(F("Unknown tag 0x"));
      Serial.print(tag, HEX);
      Serial.print(F(", count="));
      Serial.print(count);
      Serial.print(F(", bytes="));
      Serial.print(byteCount);
      Serial.print(F(" "));
      printHexPreview(payload, byteCount);
      break;
  }
}

static size_t bytesPerElementForTag(uint8_t tag) {
  switch (tag) {
    case 0x10:
    case 0x30:
    case 0x40:
      return 2;

    case 0x20:
    case 0x60:
    case 0x61:
    case 0x62:
    case 0x63:
    case 0x64:
    case 0x65:
      return 4;

    case 0x50:
    case 0x51:
    case 0x52:
      return 1;

    default:
      return 1;
  }
}

static bool findTelemetryHeader(const uint8_t *data, size_t len, size_t *headerIndex) {
  /*
    The STM32 packet builder in the current project copy has one compatibility
    byte in front of the documented 0xA5 header. Older/cleaner packet builders
    may not. By searching only the first two positions, we stay tolerant without
    accidentally finding 0xA5 inside normal payload data.
  */
  if (len >= 3
      && data[0] == PAYLOAD_MAGIC
      && data[1] == PAYLOAD_VERSION
      && data[2] == PAYLOAD_TYPE_DATA) {
    *headerIndex = 0;
    return true;
  }

  if (len >= 4
      && data[0] == PAYLOAD_TYPE_DATA
      && data[1] == PAYLOAD_MAGIC
      && data[2] == PAYLOAD_VERSION
      && data[3] == PAYLOAD_TYPE_DATA) {
    *headerIndex = 1;
    return true;
  }

  return false;
}

static void parseTelemetryPacket(const uint8_t *data, size_t len, size_t headerIndex) {
  const uint8_t *packet = &data[headerIndex];
  const size_t packetLen = len - headerIndex;

  if (packetLen < 10) {
    Serial.print(F("Binary packet too short, len="));
    Serial.println(len);
    printHexPreview(data, len);
    return;
  }

  if (packet[0] != PAYLOAD_MAGIC || packet[1] != PAYLOAD_VERSION || packet[2] != PAYLOAD_TYPE_DATA) {
    Serial.print(F("Unknown binary packet, len="));
    Serial.print(len);
    Serial.print(' ');
    printHexPreview(data, len);
    return;
  }

  const uint16_t sequence = readU16LE(&packet[3]);
  const uint32_t timestampMs = readU32LE(&packet[5]);
  const uint8_t deploymentPercent = packet[9];

  Serial.println();
  Serial.println(F("========== AMBAR TELEMETRY =========="));
  if (headerIndex != 0) {
    Serial.print(F("leading_compat_bytes="));
    Serial.println(headerIndex);
  }
  Serial.print(F("seq="));
  Serial.print(sequence);
  Serial.print(F(", t_ms="));
  Serial.print(timestampMs);
  Serial.print(F(", deploy_percent="));
  Serial.print(deploymentPercent);
  Serial.print(F(", len="));
  Serial.println(len);

  size_t index = 10;
  while (index + 2 <= packetLen) {
    const uint8_t tag = packet[index++];
    const uint8_t count = packet[index++];
    const size_t bytesPerElement = bytesPerElementForTag(tag);
    const size_t byteCount = (size_t)count * bytesPerElement;

    if (index + byteCount > packetLen) {
      Serial.print(F("Truncated tag 0x"));
      Serial.print(tag, HEX);
      Serial.print(F(", count="));
      Serial.print(count);
      Serial.print(F(", wanted_bytes="));
      Serial.print(byteCount);
      Serial.print(F(", remaining="));
      Serial.println(packetLen - index);
      break;
    }

    parseTaggedSection(tag, &packet[index], count, byteCount);
    index += byteCount;
  }

  if (index != packetLen) {
    Serial.print(F("Trailing bytes: "));
    Serial.println(packetLen - index);
  }

  Serial.print(F("RSSI="));
  Serial.print(radio.getRSSI());
  Serial.print(F(" dBm, SNR="));
  Serial.print(radio.getSNR());
  Serial.print(F(" dB, freq_err="));
  Serial.print(radio.getFrequencyError());
  Serial.println(F(" Hz"));
  Serial.println(F("====================================="));
}

static void handlePacket(const uint8_t *data, size_t len) {
  if (len == 0) {
    return;
  }

  size_t headerIndex = 0;
  if (findTelemetryHeader(data, len, &headerIndex)) {
    parseTelemetryPacket(data, len, headerIndex);
    return;
  }

  if (looksLikeTextPacket(data, len)) {
    printTextPacket(data, len);
    return;
  }

  Serial.print(F("UNKNOWN len="));
  Serial.print(len);
  Serial.print(' ');
  printHexPreview(data, len);
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println(F("AMBAR SX1280 Ground Receiver"));
  Serial.println(F("Listening for STM32_BOOT, STM32_HEARTBEAT, and 0xA5 binary telemetry."));

#if USE_CUSTOM_SPI_PINS
  SPI.begin(RADIO_SCK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_NSS_PIN);
#endif

  Serial.print(F("[SX1280] Initializing ... "));
  int state = radio.begin(
    RADIO_FREQ_MHZ,
    RADIO_BW_KHZ,
    RADIO_SF,
    RADIO_CR,
    RADIO_SYNC_WORD,
    RADIO_TX_POWER_DBM,
    RADIO_PREAMBLE_SYMBOLS
  );

  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("failed, code "));
    Serial.println(state);
    Serial.println(F("If this fails, check the four RADIO_*_PIN values at the top of the file."));
    while (true) {
      delay(1000);
    }
  }

  /*
    For SX1280 LoRa in RadioLib 7.x:
      setCRC(0) = off
      setCRC(2) = on
    The STM32 SX1280 driver has CRC enabled, so this receiver must match.
  */
  radio.setCRC(2);

  Serial.println(F("success"));
  Serial.print(F("freq="));
  Serial.print(RADIO_FREQ_MHZ, 3);
  Serial.print(F(" MHz, bw="));
  Serial.print(RADIO_BW_KHZ, 3);
  Serial.print(F(" kHz, sf="));
  Serial.print(RADIO_SF);
  Serial.print(F(", cr=4/"));
  Serial.print(RADIO_CR);
  Serial.print(F(", sync=0x"));
  Serial.println(RADIO_SYNC_WORD, HEX);
  Serial.println(F("Waiting for packets..."));
}

void loop() {
  memset(rxBuffer, 0, sizeof(rxBuffer));

  /*
    Receive into a byte buffer, not a String. The STM32 telemetry packet is
    binary and can contain zero bytes, so String/strlen-based parsing breaks.
  */
  int state = radio.receive(rxBuffer, sizeof(rxBuffer), RX_TIMEOUT_US);

  if (state == RADIOLIB_ERR_NONE) {
    const size_t packetLen = radio.getPacketLength();
    handlePacket(rxBuffer, packetLen);
  } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
    // Normal when no packet arrives during the receive window.
  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    Serial.println(F("CRC mismatch: radio settings may not match, or packet was corrupted."));
  } else {
    Serial.print(F("Receive failed, RadioLib code "));
    Serial.println(state);
    delay(200);
  }
}
