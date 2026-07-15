# AMBAR Ground Station

This sketch targets the LILYGO T3-S3 SX1280PA ground radio and matches the
STM32 airbrake firmware's 2445 MHz, SF7, 203.125 kHz, CR 4/5 link profile.
Open the serial monitor at 115200 baud.

## Serial console

- `view both` prints the detailed decoder output followed by one packet summary.
  This is the default.
- `view summary` prints only the presentation-friendly packet summary.
- `view detail` prints only the detailed decoder output.
- `tx <payload>` transmits exactly the bytes after `tx `. For example,
  `tx PING` sends `PING` to the STM32.
- `help` prints the command list.

## End-of-packet summary

Every successfully received packet ends with a clearly delimited summary when
the view is `both` or `summary`. It reports packet number, STM32 sequence and
uptime, flight phase and armed state, motion, predicted/ballistic/drag apogee,
airbrake command and effectiveness, sensor health, build features, logging,
safety inhibits, RSSI/SNR, and message text. A field that was not present in
that packet is printed as `N/A`.

The header time is STM32 uptime since boot, not confirmed time since launch.
The firmware does not serialize a separate armed Boolean; the sketch derives
armed state only when the `NOT_ARMED` flight-inhibit bit is present. Actuator
readiness remains a separate status.

Feature, logging, and safety status comes from numeric telemetry tags `0x61`
and `0x63`. Those numeric fields remain authoritative even when optional status,
command, or error text is omitted because the STM32 packet has reached its
200-byte limit. An actuator build inhibit is described as intentionally disabled
when the reported feature word says the actuator is off; it is not labeled as a
hardware failure.

## Arduino board selection

Use ESP32 core 3.3.10, RadioLib 7.7.1, and this board configuration:

```text
esp32:esp32:lilygo_t3s3:USBMode=hwcdc,CDCOnBoot=cdc,Revision=Radio_SX1280PA
```

The required radio pins are SCK 5, MISO 3, MOSI 6, NSS 7, RESET 8, DIO1 9,
BUSY 36, RX enable 21, and TX enable 10.
