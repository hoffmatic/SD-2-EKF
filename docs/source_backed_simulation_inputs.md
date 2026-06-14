# Source-Backed Simulation Inputs

This note separates values that were found in project source material from
values that are still placeholders.

## Sources Checked

- `Project AMBAR M5 Report.docx`, latest local report reviewed on June 12, 2026.
- Current V3 KiCad project:
  `C:\Users\hoffm\OneDrive\Desktop\PCBs\Ethan PCV V3\NewAirbrakePCB\airbrake.kicad_sch`.
- Earlier June 1 KiCad project candidate:
  `C:\Users\hoffm\OneDrive\Desktop\PCBs\Airbreak PCB June 1\airbrake-PCB\airbrake.kicad_sch`.
- Datasheets under `C:\Users\hoffm\OneDrive\Desktop\Data sheets`.

The online SharePoint folder still requires UCF sign-in before it can be read
from an unauthenticated local checkout. The local sync currently exposed only
an older `V1` EKF file under
`C:\Users\hoffm\OneDrive\Documents\GitHub\Project AMBAR`.

## M5 Report Values Now Reflected in Code

- Target apogee: 3000 ft.
- Target apogee tolerance: +/-100 ft.
- Maximum altitude limit: 12500 ft.
- Ground hit velocity limit: <=35 ft/s, with 20 ft/s listed as target.
- Flight computer acceleration capability: >=30G.
- Flight computer recorded velocity capability: <=1125 ft/s.
- Flight computer footprint limit: 3.25 in x 1.125 in.
- Airbrake sensor package: IMU, barometer, and magnetometer.
- Independent recovery GPS remains required by RR 4.12.
- Airbrake full-deployment time: <=1 second.
- Airbrake control response target: 500 ms.
- Airbrake mass limit: <=5 lb.
- Ground-station range requirement: >=5000 ft.
- Airbrake and ground-station radio frequency: 2.4 GHz.
- Selected motor: AeroTech J420R.
- M5 OpenRocket comparison apogee for J420R: 4005 ft.
- Report concept actuator travel: about 1 inch for full 90-degree deployment.
- Report concept lead screw travel: about 1.5 lead-screw rotations for full
  deployment.
- Required logging duration: greater than 2 hours.

These constants live in `include/ambar_project_requirements.hpp`.

## Hardware Facts Used by the Electronics Sandbox

The current KiCad schematic candidate confirms these major parts:

- STM32H562RGT6 microcontroller
- BMP388 barometer
- LSM6DSV32XTR IMU
- LIS2MDL magnetometer
- SX1280IMLTRT radio
- TMC5240ATJ+T motor driver
- W25Q64JVSSIQ flash
- MPM3606AGQV-Z regulator
- ESDA7P60-1U1M USB protection
- IRLML6344TRPBF MOSFET in the newer PCB candidate

The electronics sandbox now checks additional report-level risks:

- LSM6DSV32X 32G accelerometer range against the M5 30G requirement.
- W25Q64 flash capacity against the two-hour logging requirement using the
  current virtual 64-byte record size.
- Current SX1280 2.4-2.5 GHz RF range against the M5 2.4 GHz requirement.
- Airbrake IMU/barometer/magnetometer architecture and independent recovery GPS.

## RocketPy Inputs

The RocketPy reference model now uses:

- RocketPy 1.12.1.
- Certified public-domain AeroTech J420R RASP thrust data.
- M5 3-inch vehicle diameter and 3000 ft target.
- M5 OpenRocket passive apogee of 4005 ft as a comparison/calibration point.
- The real C++ `AmbarFlightComputer` through a persistent process bridge.

## Values Still Not Available

These are still not source-backed and should not be treated as real predictions:

- Real OpenRocket `.ork` file or exported flight CSV.
- Final measured flight-ready mass, center of gravity, and inertia.
- Drag coefficient vs Mach/airbrake deployment.
- Airbrake deployed area vs command fraction.
- Vent/static-port pressure lag model.
- Final step/mm, microstepping, current limit, homing switch, and stall current.
- Final actuator friction/load model.
- Bench-measured 3V3 current draw.
- Bench-measured I2C/SPI timing and signal integrity.

Until those are supplied, RocketPy results are a physics-backed reference model,
not an independently validated prediction of the final vehicle.
