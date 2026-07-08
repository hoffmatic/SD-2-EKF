# Arduino SX1280 Ground Receiver

This folder contains a ready-to-send Arduino sketch for the LILYGO/SX1280
ground receiver.

Use:

```text
tools/arduino_ground_receiver/ambar_sx1280_ground_receiver/ambar_sx1280_ground_receiver.ino
```

The sketch fixes the receiver-side issue where the Arduino code treats every
incoming packet as readable text. The STM32 firmware sends both readable text
packets and binary EKF telemetry packets, so the receiver must read into a byte
buffer and decode tagged fields.

It also accepts the current STM32 packet shape that includes one leading
compatibility byte before the documented `0xA5 0x01 0x01` telemetry header.

Before uploading, check the four pin defines at the top of the sketch:

```c
#define RADIO_NSS_PIN
#define RADIO_DIO1_PIN
#define RADIO_RST_PIN
#define RADIO_BUSY_PIN
```

If the radio does not initialize, those pins are the first thing to change for
the exact LILYGO board being used.

Serial Monitor should be set to `115200` baud. The radio settings are already
set to match the STM32 firmware: `2445 MHz`, LoRa `SF7`, `203.125 kHz`
bandwidth, coding rate `4/5`, sync word `0x12`, and CRC on.
