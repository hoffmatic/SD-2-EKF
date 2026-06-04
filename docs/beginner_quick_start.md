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

## The Easiest Way To Run The Sandboxes On Windows

1. Open the project folder you downloaded.
2. Hold `Shift`, right-click an empty spot in the folder, and choose
   `Open in Terminal` or `Open PowerShell window here`.
3. Copy and paste this command:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run_sandboxes.ps1
```

4. Press Enter.

The script builds and runs everything currently useful on a normal computer:

- `rocket_airbrake_ekf.exe`: small demo of the flight computer logic.
- `sim_flight_sandbox.exe`: simulated rocket flight behavior.
- `sim_electronics_sandbox.exe`: virtual PCB/electronics bring-up checks.
- `sim_actuator_sandbox.exe`: virtual airbrake motor and jam/fault behavior.

## How To Read The Output

The sandboxes print tables. You do not need to understand code to use the first
results.

- `pass` means the virtual check looks okay.
- `warn` means the project still has an open engineering question.
- `fail` means that condition should block flight logic until fixed.
- `cmd_%` means the software command to the airbrake.
- `act_%` means the simulated physical airbrake position.
- `baro_rej` means how many bad barometer readings were rejected.

These are not flight predictions yet. They are virtual workbenches for catching
obvious software, sensor, electronics, and actuator problems before touching real
hardware.

## What To Open First

- `README.md`: the front page of the project.
- `docs\project_requirements.md`: what the rocket needs the software to do.
- `docs\hardware_map.md`: how the current PCB parts and STM32 pins map to code.
- `docs\datasheet_integration_notes.md`: datasheet facts reflected in code.
- `docs\simulation_sandboxes.md`: deeper explanation of the sandboxes.

## If Something Fails

If the script says it cannot find a C++ compiler, that means the computer does
not yet have the tool needed to build the code. Send the exact error text to the
team or to Codex. The code is still there; the computer just needs the build tool
installed or configured.

If a sandbox prints a failure in its table, read the `notes` or `reason` column.
That is the simulated problem the software caught.
