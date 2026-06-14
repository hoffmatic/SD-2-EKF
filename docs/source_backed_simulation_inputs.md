# Source-Backed Simulation Inputs

This note separates values that were found in project source material from
values that are still placeholders.

## Sources Checked

- `Project AMBAR M5 Report (1).docx`, the SharePoint-linked Milestone 5 Critical
  Design Evaluation dated June 14, 2026.
- `Project AMBAR M5 Report.docx`, an older June 11 full-report copy retained for
  requirement context and conflict checking.
- Current V3 KiCad project:
  `C:\Users\hoffm\OneDrive\Desktop\PCBs\Ethan PCV V3\NewAirbrakePCB\airbrake.kicad_sch`.
- Earlier June 1 KiCad project candidate:
  `C:\Users\hoffm\OneDrive\Desktop\PCBs\Airbreak PCB June 1\airbrake-PCB\airbrake.kicad_sch`.
- Datasheets under `C:\Users\hoffm\OneDrive\Desktop\Data sheets`.

The SharePoint URL required UCF sign-in in the automated browser, but the
June 14 document was already present in Downloads and was extracted locally.
See `docs/m5_report_data_extract.md` for the complete engineering-value map and
confidence labels.

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
- Current June 14 OpenRocket passive apogee: 3379 ft.
- Current reported maximum velocity: 579 ft/s.
- Current reported maximum Mach: 0.509.
- Current reported rail-exit velocity: 75.5 ft/s.
- Current launch-site inputs: 27.93469 deg latitude, -80.70953 deg longitude,
  23 ft elevation, 72 in rail, and 0 deg from vertical.
- Stabilizing-fin geometry: 3 fins, 6 in root chord, 2 in tip chord, and 3 in
  span.
- Airbrake load calculation: 54.8215 N per fin and 219.286 N total for four
  fins at 197.206 m/s.
- Analytical 3.3 V peak-current budget: approximately 430 mA, excluding the
  battery-rail actuator driver.
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
- June 14 M5 OpenRocket passive apogee of 3379 ft as the current comparison
  point. The provisional RocketPy model is intentionally allowed to fail this
  comparison rather than being silently retuned.
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

The older June 11 report's 4005 ft result describes an earlier rocket design.
It is no longer the current acceptance baseline, but it remains documented as a
source conflict rather than being deleted from the engineering record.

Until those are supplied, RocketPy results are a physics-backed reference model,
not an independently validated prediction of the final vehicle.
