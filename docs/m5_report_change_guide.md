# M5 Report Manual Change Guide

This is advice for a human editor. It does not modify or replace the Word
report. Confirm section numbers against the current document before editing,
because Word numbering may move as content is added.

## Highest-Priority Technical Corrections

| Report location | Problem | Manual change | Evidence/type |
| --- | --- | --- | --- |
| `2.1.1.1` flight-control algorithm | The report describes a drag-aware `k/log1p` controller and consistent +/-10 ft results, but that source and reproducible evidence are not in GitHub or SharePoint. | State that the current embedded scaffold uses the ballistic predictor `h + v^2/(2g)`. Describe drag-aware prediction as planned work unless the missing implementation and test evidence are supplied. Remove the general +/-10 ft claim. | Factual correction; `src/ambar_airbrake.cpp`, `tests/ambar_core_tests.cpp` |
| `2.1.1.2 Airbrake Flight Computer Tests` | Section is empty. | Add a short test matrix covering core assertions, five flight cases, six electronics cases, four actuator cases, four fault/replay cases, and the 200-trial fixed-seed study. Explain that these are desktop software tests, not hardware qualification. | Missing evidence; `docs/project_status.md` |
| `1.5.1 Software-in-the-Loop` | Current wording can imply that the model predicts the complete chip and vehicle. | Explain that SIL executes shared C++ logic against simplified physics and fault models. Explicitly exclude PCB power integrity, real buses, raw IMU orientation, mechanism loads, RF behavior, and manufacturing variation. | Scope correction |
| `1.5.1.1 Flight Control Suite` | Screenshot lacks the updated fault/replay and Monte Carlo coverage. | Add the test conditions and pass rules from the new suites. Report the 200/200 safety result and the informational 15/200 target-hit count together; do not present 15/200 as a validated probability. | Current repository result |
| `1.5.1.2 RocketPy Physics` | Earlier text/results use the older geometry and truth-level sensors. | State that the current run uses June 2 OpenRocket geometry, 3.58 m/s constant wind from 225 degrees, and deterministic provisional sensor bias/noise/quantization/latency. Report 3851 ft passive, 2979 ft closed-loop, Mach 0.494, and 42.7 ft/s rail exit. | Current run; `build/rocketpy-last-run.json` is generated locally |
| RocketPy conclusions | A closed-loop result inside tolerance may be presented as proof of accuracy. | Say that -21 ft is a necessary target-band check only. The passive comparison fails by +14.0% and rail exit fails, so target accuracy is not validated. | Model limitation |

## Hardware and Architecture Corrections

| Report location | Problem | Manual change | Evidence/type |
| --- | --- | --- | --- |
| PCB design/status section | Wording implies a completed or manufacturable PCB, and may call it single-sided. | Identify the June 2 Rev 3 design as the provisional baseline. State that placement exists but routing is incomplete: zero copper tracks, zero vias, and no copper zones were present in the reviewed file. Remove "single-sided" because R9, R10, and U9 are on the back. | Factual correction; `docs/hardware_map.md` |
| Flight-computer component table | Older BOM parts conflict with the schematic. | Use STM32H562RGT6, BMP388, LSM6DSV32XTR, LIS2MDLTR, W25Q64JVSSIQ, SX1280IMLTRT, TMC5240ATJ+T, and MPM3606AGQV-Z. Label procurement discrepancies separately. | Factual correction |
| GPS/magnetometer discussion | Some older diagrams describe GPS inside the airbrake computer or imply the magnetometer replaces GPS. | State that LIS2MDL supports magnetic-field/attitude information. Independent recovery GPS remains required and is not replaced by the magnetometer. | Architecture correction; `docs/sensor_architecture.md` |
| Ground-station radio section | BOM hardware uses SX1262 while the flight PCB uses SX1280. | Mark the link as unresolved. SX1262 and SX1280 are not an interoperable radio pair. Specify an SX1280-compatible 2.4 GHz ground station before claiming telemetry compatibility. | Unresolved team decision |
| Power-budget section | The 430 mA estimate may be presented as validated margin against a 600 mA regulator. | Label 430 mA as an analytical estimate. Note the nominal 170 mA margin and require startup, flash-write, radio-transmit, transient, and thermal measurements. | Missing bench evidence |
| Actuator electrical section | TMC5240 braking resistor and motor limits are unresolved. | Add motor phase resistance/current, supply voltage, selected current limit, R9/R10 value and pulse/continuous power, lead-screw conversion, homing, and jam threshold when available. | Missing design data |

## Vehicle, Recovery, and Requirements Corrections

| Report location | Problem | Manual change | Evidence/type |
| --- | --- | --- | --- |
| Requirements table / `SR1.1` | Project documents conflict between 3000 ft and 4000 ft. | Use one approved requirement. GitHub currently uses 3000 +/-100 ft. If the advisor has not approved this, mark it as an open decision instead of silently mixing both targets. | Requirements conflict |
| OpenRocket results | The file contains cached results but its simulations are marked not simulated after model changes. | Rerun the selected J420R configuration in OpenRocket and export the result before calling it current. Record software version, file hash, motor configuration, mass, CG, and run date. | Missing reproducibility evidence |
| Recovery configuration | OpenRocket/report references 44-inch main and 18-inch drogue while the BOM lists 40-inch and 24-inch parachutes. | Physically inspect labels and purchase records. Update all report tables, OpenRocket recovery objects, descent calculations, and snatch-force calculations to the verified pair. | Unresolved hardware decision |
| Requirement verification matrix | Several requirements have no method, evidence, owner, or result; older IDs have drifted. | Give every requirement one canonical ID plus verification method, test procedure, evidence path, owner, result, and status. Do not reuse IDs for different requirements. | Traceability correction |
| `PR3.9` verification entry | Verification is blank. | Add the approved analysis/test method and evidence, or mark it explicitly `Not yet verified`. | Missing evidence |
| Black-powder tables `T6-T9` | Tables are blank although calculations exist elsewhere. | Enter the current calculated main/drogue values only as starting points and state that ground testing determines final charges. Do not present spreadsheet values alone as qualified. | Missing content/safety limitation |

## Project Management and Quality Corrections

| Report location | Problem | Manual change | Evidence/type |
| --- | --- | --- | --- |
| Gantt/development plan | Owners are blank, progress is stale, and a June 13 flight-computer test was scheduled before a routed board existed. | Add one owner and status to each task, show dependencies and critical path, and reschedule PCB release, assembly, bring-up, bench tests, HIL, and flight testing realistically. | Planning correction |
| BOM and budget | Flight-computer price, vendor totals, threaded rod, U-bolt, and radio selections conflict across files. | Reconcile one procurement ledger with quantity, ordered part, vendor, actual paid cost, received status, subsystem, and variance from plan. | Procurement correction |
| FMEA/risk register | Entries still mention servo and SD-card failures. | Replace them with stepper/TMC5240, lead-screw/homing/jam, W25Q64 flash, radio compatibility, sensor freshness, brownout, and unrouted-board release risks. Add prevention, detection, response, owner, and verification. | Architecture correction |
| ABET design-competence matrix | Matrix is empty. | Map specific design decisions and verification evidence to the required competencies. | Missing content |
| References | No complete references section was found. | Cite component datasheets, RocketPy 1.12.1, OpenRocket 24.12, motor data, safety codes, and any equations or external calculations used. | Attribution gap |

## Editing and Formatting Cleanup

- Remove `TODO: images .`, `For reference, delete later vvv`, raw drafting
  questions, and other editor notes.
- Replace informal phrases such as "MCU on verge of melting itself" with a
  measured engineering risk statement.
- Correct the broken Cade Donley sentence.
- Correct Ground Application numbering (`2.3.2` versus `2.3.1.x`), skipped
  heading numbers, table cross-references, and the final `FME`/`FMEA` caption.
- Check every figure/table number after Word updates the fields.
- Use past tense only for work actually completed and measured. Use future
  tense for planned tests and conditional language for provisional models.

## Recommended Report Language

For simulation conclusions, use language similar to:

> The software-in-the-loop tests verify deterministic software reactions to
> declared virtual inputs. They do not qualify the PCB, sensors, actuator, RF
> link, or final vehicle aerodynamics. The current RocketPy model produces a
> 2979 ft closed-loop apogee, but passive-model and rail-exit comparisons fail;
> therefore, target accuracy remains provisional pending model reconciliation
> and hardware testing.

For the PCB status, use language similar to:

> The June 2 Rev 3 KiCad project is the current provisional electrical
> baseline. Component placement is present, but routing and manufacturing
> release checks are incomplete. Firmware pin planning may use this revision,
> while fabrication claims must wait for routed copper, ERC/DRC review, and
> generated manufacturing outputs.
