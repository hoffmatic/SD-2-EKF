# Beginner Quick Start

This page assumes you have never used code or GitHub before.

## What GitHub Is

GitHub is the online folder where the team can see the project files. For this
project, GitHub holds the current AMBAR airbrake software, project notes,
hardware notes, and virtual sandboxes.

Project link:

```text
https://github.com/hoffmatic/SD-2-EKF
```

## The Easiest Way To Get The Files

1. Open the GitHub project link in your browser.
2. Click the green `Code` button.
3. Click `Download ZIP`.
4. Open your Downloads folder.
5. Right-click the ZIP file and choose `Extract All`.
6. Open the extracted folder.

That gives you a local copy of the project on your computer. If the team updates
GitHub later and you are not using Git yet, repeat the same download steps to get
the newest copy.

## The Easiest Way To Run Every Simulation On Windows

1. Open the project folder you downloaded.
2. Hold `Shift`, right-click an empty spot in the folder, and choose
   `Open in Terminal` or `Open PowerShell window here`.
3. Copy and paste this command:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run_all_simulations.ps1
```

4. Press Enter.

The first run may take a few minutes because it creates a private Python
environment inside the project and installs RocketPy. The script then builds
and runs everything currently useful on a normal computer:

- `rocket_airbrake_ekf.exe`: small demo of the flight computer logic.
- `sim_flight_sandbox.exe`: simulated rocket flight behavior.
- `sim_electronics_sandbox.exe`: virtual PCB/electronics bring-up checks.
- `sim_actuator_sandbox.exe`: virtual airbrake motor and jam/fault behavior.
- `sim_fault_replay_sandbox.exe`: timestamp, invalid-sensor, dropout, and log
  replay checks.
- `sim_monte_carlo_sandbox.exe`: 200 repeatable variations that check command
  safety and report how often the provisional model reaches the target band.
- `ambar_core_tests.exe`: focused checks of estimator and controller rules.
- `RocketPy physics sandbox`: a six-degree-of-freedom rocket trajectory whose
  virtual airbrakes are commanded by the real C++ AMBAR estimator/controller.

## The Easiest Way To Use The Visual Interface

1. In the same PowerShell window, run:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run_simulation_ui.ps1
```

2. Open `http://127.0.0.1:8765` in a browser.
3. Click `RocketPy Physics` to view that simulation.
4. Click `Run This Suite` to rerun it, or `Run All` to run every sandbox.

The visual interface labels each condition, pass rule, measurement, warning,
and result. A RocketPy pass means the software and provisional physics model
behaved as specified. It does not certify the final rocket until measured mass,
inertia, center of gravity, and airbrake drag data replace the placeholders.

## How To Read The Output

The sandboxes print test reports. You do not need to understand code to use the
first results.

- `PASS` means the software reacted the expected way for that virtual test.
- `WARN` means the project still has an open engineering question.
- `FAIL` means a stated criterion was not met. The current RocketPy passive
  reference and rail-exit checks intentionally fail because the model and
  OpenRocket data are not reconciled; this is an engineering finding, not a
  broken installation.
- `BLOCKED` means a virtual boot condition should stop flight logic or arming.
- `cmd_%` means the software command to the airbrake.
- `act_%` means the simulated physical airbrake position.
- `baro_rej` means how many bad barometer readings were rejected.

These are not flight predictions yet. They are virtual workbenches for catching
obvious software, sensor, electronics, and actuator problems before touching real
hardware.

## Magnetometer And GPS

The current V3 airbrake PCB contains an LIS2MDL magnetometer and does not contain
a GPS receiver. The magnetometer measures magnetic-field direction; it cannot
measure geographic position and is not a functional replacement for GPS. The
M5 report still requires a separate GPS device in the rocket recovery system.

## What To Open First

- `README.md`: the front page of the project.
- `docs\project_requirements.md`: what the rocket needs the software to do.
- `docs\project_status.md`: what is implemented, simulated, provisional, and
  still future work.
- `docs\hardware_map.md`: how the current PCB parts and STM32 pins map to code.
- `docs\datasheet_integration_notes.md`: datasheet facts reflected in code.
- `docs\simulation_sandboxes.md`: deeper explanation of the sandboxes.
- `docs\sensor_architecture.md`: why the magnetometer and recovery GPS are
  separate subsystems.
- `docs\m5_report_change_guide.md`: exact report corrections for you to make
  manually in Word.

## If Something Fails

If the script says it cannot find a C++ compiler, that means the computer does
not yet have the tool needed to build the code. Send the exact error text to the
team maintainer. The code is still there; the computer just needs the build tool
installed or configured.

If a sandbox prints a failure in its table, read the `notes` or `reason` column.
That is the simulated problem the software caught.
