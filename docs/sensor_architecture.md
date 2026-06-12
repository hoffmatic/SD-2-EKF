# Sensor Architecture Verification

## Verified Result

The current airbrake computer uses an IMU, barometer, and LIS2MDL magnetometer.
It does not contain a GPS/GNSS receiver. This is supported by both the latest
local M5 report and the current V3 KiCad schematic.

The magnetometer is not a functional replacement for GPS:

- The LIS2MDL measures the local magnetic field and can support heading,
  attitude alignment, and sensor-health checks.
- GPS provides geographic position and recovery tracking.
- M5 requirement RR 4.12 still assigns GPS to the independent recovery system.

The intended architecture is therefore:

| System | Sensors / Role |
| --- | --- |
| Airbrake computer | LSM6DSV32X IMU, BMP388 barometer, LIS2MDL magnetometer |
| Recovery/tracking system | Independent GPS flight computer(s) and GPS ground station |

## Source Evidence

- M5 sensor-package text lists IMU, barometer, and magnetometer.
- M5 FCR 5.9 specifies a 2.4 GHz flight-computer antenna; it no longer defines
  the earlier GPS update-rate requirement used by this repository.
- M5 RR 4.12 requires GPS for recovery/location.
- The V3 schematic contains `LIS2MDLTR` and `MAGNET_SCL`, `MAGNET_SDA`, and
  `MAGNET_INT` nets, with no GPS/GNSS component or net.

This distinction must remain visible in software and reviews: removing GPS from
the airbrake PCB does not remove the vehicle-level recovery GPS requirement.
