# June 14 M5 Report Data Extract

Source reviewed: `Project AMBAR M5 Report (1).docx`, a local copy of the
SharePoint document titled *Milestone 5 - Critical Design Evaluation* and dated
June 14, 2026. This file records engineering data only; it does not reproduce
the report.

## Confidence Labels

- **Requirement:** a stated project limit or target.
- **Reported result:** an OpenRocket, CFD, FEA, or calculation result in the
  report. It has not been independently reproduced here.
- **Model input:** a value shown in the report's input table or screenshot.
- **Unresolved:** missing, contradictory, or lacking units/source detail.

## Flight And Launch Inputs

| Value | Report value | Classification | Repository use |
| --- | ---: | --- | --- |
| Target apogee | 3000 ft | Requirement | Controller target |
| Target tolerance | +/-100 ft | Requirement from the full M5 requirements table | Acceptance criterion |
| Passive OpenRocket apogee | 3379 ft | Reported result | Current comparison baseline |
| Maximum altitude | 12500 ft | Requirement | Safety envelope |
| Maximum velocity | 579 ft/s | Reported result | Reference comparison |
| Maximum Mach | 0.509 | Reported result | Compared with Mach 1 limit |
| Rail exit velocity | 75.5 ft/s | Reported result | Compared with 52 ft/s minimum |
| Launch rail | 72 in | Model input | RocketPy rail length |
| Launch angle | 0 deg from vertical | Model input | RocketPy uses 90 deg from horizontal |
| Launch heading/wind direction | 225 deg | Model input | RocketPy heading reference |
| Launch latitude | 27.93469 deg | Model input | RocketPy environment |
| Launch longitude | -80.70953 deg | Model input | RocketPy environment |
| Launch elevation | 23 ft | Model input | RocketPy environment |
| Average wind | 3.58 m/s, approximately 8 mph | Model input | Applied as constant nominal wind |
| Wind standard deviation | 0.358 m/s | Model input | Recorded for future RocketPy dispersion |
| Turbulence intensity | 10% | Model input | Recorded for future turbulence modeling |

The current RocketPy environment uses standard pressure/temperature profiles
with a constant 3.58 m/s horizontal wind from 225 degrees. Wind dispersion and
turbulence are not yet part of the RocketPy run.

## Rocket And Recovery Data

| Value | Report value | Classification |
| --- | ---: | --- |
| Vehicle diameter | 3 in | Requirement/design statement |
| Stabilizing-fin count | 3 | Reported design |
| Fin root chord | 6 in | Model input |
| Fin tip chord | 2 in | Model input |
| Fin span | 3 in | Model input |
| Fin thickness | 0.125 in | Model input |
| Fin flutter velocity | 1686.705 ft/s | Reported calculation |
| Fin flutter safety factor | 2.913 | Reported calculation |
| Static stability | 1.79 cal | Reported result |
| Dynamic stability | 1.00 to 2.41 cal | Reported result |
| Selected motor | AeroTech J420R | Design selection |
| Reported motor impulse | 648 Ns | Reported motor value |
| Ground-hit velocity | 20.1 ft/s | Reported result; 35 ft/s limit |
| Maximum snatch force | 631 N / 141.85 lbf | Reported calculation at 55 ft/s |
| Main ejection charge | approximately 1.28 g | Reported calculation; requires ground test |
| Drogue ejection charge | approximately 0.73 g | Reported calculation; requires ground test |

The June 2 OpenRocket file supplies current body/nose/fin geometry and component
layout, but final measured dry/loaded mass, center of gravity, moments of
inertia, and reusable drag exports remain unavailable.

## Airbrake Aerodynamic And Structural Data

| Value | Report value | Classification |
| --- | ---: | --- |
| Air density | 1.138 kg/m^3 | Calculation input |
| Flat-plate drag coefficient | 1.28 | Calculation input |
| Calculation velocity | 197.206 m/s | Calculation input |
| Area per airbrake fin | 0.001935 m^2 | Calculation input |
| Drag force per fin | 54.8215 N | Reported calculation |
| Drag force for four fins | 219.286 N | Reported calculation |
| Maximum polycarbonate fin deflection | 0.018 in | Reported FEA result |
| Reported FEA factor of safety | at least 15 | Reported FEA result |

The 197.206 m/s drag-load velocity is about 647 ft/s and conflicts with the
separate 579 ft/s expected-maximum-velocity value. The report also lists
`Max Q = 500` without a unit. Neither value should become a hard simulation
truth until the team confirms the intended flight case and units.

The report's coefficient of 1.28 is referenced to one airbrake fin area. It
cannot be inserted directly as RocketPy's vehicle-reference-area drag increment
without converting reference areas or obtaining a deployment-versus-Mach drag
curve.

## Electrical Inputs

| 3.3 V load | Reported maximum |
| --- | ---: |
| STM32 MCU | 350 mA |
| IMU | approximately 1 mA |
| Barometer | approximately 1 mA |
| Magnetometer | approximately 1 mA |
| NOR flash | 25 mA |
| LoRa radio | 50 mA |
| Total | approximately 430 mA |

The report recommends a regulator capable of 0.6-1.0 A sustained output. The
stepper motor driver draws from the battery rail and is explicitly excluded
from this 3.3 V total. These are analytical budget values, not bench
measurements.

## Conflicts And Trust Limits

1. The June 14 report gives a passive OpenRocket apogee of 3379 ft. The older
   June 11 full-report copy gives 4005 ft for an earlier design and later says
   the vehicle may reach around 5000 ft. The repository now treats 3379 ft as
   current and preserves the conflict in provenance.
2. The report claims the controller commonly reaches +/-10 ft across many
   rockets and environments, but it does not include a reproducible scenario
   matrix, configuration files, raw outputs, or uncertainty analysis. That
   statement is not used as a validation threshold.
3. The report defines four physical airbrake fins but three stabilizing fins.
   These are different components and must not be combined in the rocket model.
4. Several test procedures describe what should be measured but contain no
   completed measurements. They are test plans, not evidence of passing tests.
5. Reported CFD, FEA, and OpenRocket values remain source claims until their
   project files or exports can be rerun independently.

## Current Repository Cross-Check

After applying the June 14 launch conditions and stabilizing-fin geometry, the
provisional RocketPy model reports:

- Passive apogee: 3851 ft versus 3379 ft in the report, a +14.0% mismatch.
- Rail exit: 42.7 ft/s versus the 52 ft/s requirement and 75.5 ft/s report
  result.
- Maximum Mach: 0.494 versus 0.509 in the report.
- Closed-loop apogee: 2973 ft, but this is not a validated target result because
  the passive vehicle model fails its source comparison.

The passive-reference and rail-exit tests therefore fail. This is the intended
behavior until the OpenRocket configuration is rerun and the mass properties
and drag data explain the discrepancy.

## Data Still Needed

- Rerun current OpenRocket configuration and exported simulation CSV.
- Flight-ready mass, CG, and inertia with measurement uncertainty.
- Airbrake drag increment versus deployment and Mach using a declared reference
  area.
- Motor/lead-screw torque, pitch, efficiency, travel, current, and load tests.
- Real IMU/barometer rates, filters, orientation, noise, delay, and venting.
- Battery voltage, capacity, impedance, regulator efficiency, and transient
  captures during radio transmission and actuator motion.
