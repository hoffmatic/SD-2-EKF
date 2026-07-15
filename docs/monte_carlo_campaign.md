# Closed-Loop Baseline and Monte Carlo Campaign

## What This Adds

The campaign runs RocketPy trajectory physics in a closed loop with the
production STM32 C flight core:

```text
RocketPy truth
  -> pad-referenced body-axis IMU plus delayed/noisy barometer measurements
  -> production ambar_ekf.c + ambar_flight.c
  -> deployment command
  -> delayed/rate-limited virtual actuator
  -> RocketPy airbrake drag
  -> changed trajectory and apogee
```

This corrects the central limitation of the OpenRocket replay: the deployment
command now changes the next simulated vehicle state. The physical motor is not
used by this campaign.

The default campaign performs:

1. Two identical baseline runs to verify repeatability.
2. Fifty seeded Latin-hypercube runs with varied plant, sensor, atmosphere, and
   virtual-actuator assumptions.
3. A passive no-airbrake counterfactual and a controlled flight for every run.
4. CSV and JSON evidence output, including failures.

Fifty identical runs only test repeatability. Fifty randomized runs are the
first robustness screen. They are intentionally reported separately.

## Easiest Way to Run It

Double-click:

```text
Run Monte Carlo Simulation.cmd
```

The first run creates `.venv` and installs the pinned RocketPy version. The
script detects and bypasses the broken Windows Store `python` alias when a real
Python or the bundled Codex Python is available.

The default 52 paired flights normally take several minutes. Each completed
case prints elapsed time and an ETA.

The equivalent PowerShell command is:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run_monte_carlo.ps1
```

Default arguments are equivalent to:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run_monte_carlo.ps1 `
  -BaselineRuns 2 `
  -RandomRuns 50 `
  -Seed 20260715
```

To prove 50-run identical-input repeatability and then run 50 randomized cases:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run_monte_carlo.ps1 `
  -BaselineRuns 50 `
  -RandomRuns 50 `
  -Seed 20260715
```

Repeating 50 full RocketPy baselines is normally unnecessary; two identical
results are enough to detect nondeterminism in this software-in-the-loop path.

## Where Results Go

Every campaign gets a new evidence directory:

```text
build/monte-carlo/<UTC timestamp>-seed-<seed>/
```

The terminal prints the exact path. Files are never silently appended to an
older campaign directory.

### `runs.csv`

One row for every attempted run. An exception remains as an `ERROR` row and
still counts in the denominator. The file is replaced atomically after each
attempt, so a stopped campaign retains its completed rows. Important groups of
columns are:

- identity: campaign, run, mode, master seed, per-run seed, hashes;
- sampled inputs: mass, drag, wind, launch angle, burn time, airbrake drag,
  actuator rate/delay, and sensor error assumptions;
- results: passive/controlled apogee, target error, apogee reduction, Mach,
  rail-exit speed, command/deployment timing, estimator errors, and phase path;
- verdicts: safety, effectiveness, target band, envelope, and overall result.

### `parameters.csv`

The complete parameter table generated before simulations start. `sample_index`
is the index used to derive each randomized seed; `run_index` is its position in
the complete baseline-plus-random campaign. A run can be recreated from its
seed and exact resolved values even if later cases fail.

### Provenance and input snapshots

`manifest.json` starts in `running` state before trial 1 and is finalized only
after all rows complete. `resolved_base_config.json`,
`resolved_study_config.json`, and `input_snapshot/` preserve the exact bridge
binary, compiler metadata, C sources/headers, thrust curve, Python runners,
dependency pin, and any override file used by the campaign.

### `aggregate_metrics.csv`

Campaign pass rates, mean/median/p05/p95/worst results, the target-hit Wilson
confidence interval, and separate zero-safety-failure/zero-overall-failure
statistical bounds when applicable.

### `sensitivity.csv`

Exploratory Spearman rank correlations between every varied input and signed or
absolute target error. These identify which provisional assumptions should be
measured first; correlation is not proof of causation.

### Representative traces

`representative_timeseries.csv` retains the first baseline, median randomized
case, worst target-error case, and first safety failure. Matching files under
`representative_profiles/` use explicit vertical channels accepted by the
existing STM32 USB replay tool.

## What PASS Means

Safety is evaluated separately from target performance.

Every safety pass requires:

- monotonic, finite time-history data;
- bounded command and actual deployment from 0 to 1;
- no command before true motor burnout plus the configured margin;
- no command after true descent exceeds the provisional -5 m/s Recovery
  threshold or outside `Coast`/`AirbrakeActive`; command samples between apogee
  and -5 m/s remain visible as a separate metric;
- no illegal phase transition, including `Recovery -> Boost`;
- a healthy production estimator/controller;
- observed Recovery and final virtual deployment below 2%;
- virtual deployment retracting below 2% within 1.60 s of Recovery; deployment
  can physically lag the zero command near apogee and is recorded explicitly;
- controlled apogee no higher than its paired passive case.

The provisional campaign performance screen additionally requires:

- at least 96% of randomized runs inside 3000 +/-100 ft;
- 100% inside the M5 Mach and rail-exit envelope checks;
- 95th-percentile absolute target error no greater than 100 ft;
- worst absolute target error no greater than 200 ft;
- at least 10 ft apogee reduction when the passive case needs braking.

These thresholds are development gates, not flight qualification. The current
RocketPy passive model does not yet agree with the final OpenRocket report and
several uncertainty ranges are placeholders.

## Changing What Is Randomized

Edit:

```text
sim/rocketpy/monte_carlo_config.json
```

Each parameter contains:

- one or more exact configuration paths;
- uniform or triangular distribution bounds;
- units;
- a source/status statement.

Do not narrow a range merely to make the campaign pass. Replace provisional
ranges with measured mass/CG, exported drag data, sensor logs, and loaded
actuator timing as those become available.

The STM32 controller configuration remains fixed while truth is randomized.
Changing both together would give the controller information it would not know
during a real flight and create self-confirming results. The host bridge starts
from compiled defaults plus the fixed `controller` section in the base JSON. A
physical PCB can load different saved flash configuration, so compare board
configuration readback before calling a representative replay equivalent.

## Replaying Selected Cases Through the PCB

The Monte Carlo campaign itself never opens a COM port or moves the mechanism.
After reviewing the CSV, select only a few representative profiles:

1. baseline;
2. median randomized case;
3. worst target-error case;
4. first safety failure, if one exists.

Follow
[`firmware/stm32_airbrake_pcb/tools/usb_protocol/OPENROCKET_SIMULATION.md`](../firmware/stm32_airbrake_pcb/tools/usb_protocol/OPENROCKET_SIMULATION.md)
and run the chosen file through `replay_openrocket.py` with physical motion
inhibited first. Only repeat a small, manually supervised subset with motion
enabled after confirming the airbrakes are retracted, the mechanism is clear,
and HOME is correctly established.

Do not run 50 physical three-rotation cycles as part of this software campaign.
Loaded travel, current, force, temperature, repeatability, and limit behavior
belong in a separate actuator qualification test.

## Statistical Limitation

Fifty runs provide useful parameter coverage but weak rare-event evidence. Even
if all 50 pass, the exact one-sided 95% upper bound on an unobserved failure rate
is about 5.8%. Increase the supported `-RandomRuns` count after runtime is
acceptable, but measurement-backed input distributions matter more than a large
number of precisely simulated placeholder cases.

## Verification Commands

Fast orchestration tests:

```powershell
.\.venv\Scripts\python.exe -m unittest sim\rocketpy\test_monte_carlo.py
```

Small integration campaign:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run_monte_carlo.ps1 `
  -BaselineRuns 2 `
  -RandomRuns 3 `
  -Seed 12345
```

Validate an existing evidence directory independently from its CSV/JSON files:

```powershell
.\.venv\Scripts\python.exe .\scripts\validate_monte_carlo_output.py `
  .\build\monte-carlo\<campaign-directory>
```

Production bridge only:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\build_stm32_controller_bridge.ps1
```
