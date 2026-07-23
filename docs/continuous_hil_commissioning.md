# AMBAR Continuous HIL Commissioning

This procedure uses the existing PCB and mechanism without adding endpoint
switches or changing PA0/PA1. It qualifies a commanded TMC5240 ramp-state
sequence; it does not create independent position feedback.

## Safety boundary

- Guard the mechanism and keep hands, tools, and loose wiring clear.
- Use a current-limited bench supply for initial powered work.
- Keep an independent latching motor-power or `DRV_ENN` cutoff within reach.
- The configured current labels still depend on an uncalibrated register-scale
  conversion. Measure actual supply/phase current and temperature.
- TMC target and `XACTUAL` are internal ramp-generator values, not encoder,
  shaft, linkage, blade, switch, or endstop measurements.
- A TMC warning, overtemperature, driver fault, USB loss, stale replay sample,
  deadline failure, ESTOP, or evidence-write failure ends the session.

## Unchanged PCB

PA0 and PA1 retain their original LED functions:

| MCU pin | Existing net |
| --- | --- |
| PA0 | `LED_2` through R24 |
| PA1 | `LED_1` through R23 |

Do not depopulate R23/R24 and do not install a HOME/FULL harness for this
workflow.

## Software-HOME rule

1. Remove motor energy.
2. Manually place the mechanism at the known fully closed position.
3. Clear the mechanism and restore the guarded, current-limited power setup.
4. Start the launcher and type `CLOSED`.
5. The supervisor completes non-motion checks, then sends `CMD_HOME`.
6. Firmware sets the current TMC ramp position and target to zero without
   seeking or moving.

The supervisor performs this declaration only once per process. It does not
re-zero between successful cycles. Restart, resume, board reboot, or any fault
requires manual closure and a fresh acknowledgement.

If the mechanism is not actually fully closed when software HOME is declared,
all later targets inherit the wrong physical reference. The software cannot
detect that condition.

## Staged commissioning

### 1. Energy-off checks

1. Verify PA0/PA1 still operate as the existing LED circuits.
2. Verify the motor cutoff removes driver energy independently of software.
3. Probe the CONTINUOUS_HIL profile with motor power disconnected.
4. Confirm boot, profile verification, and dashboard startup cause no motion.

### 2. Direction check

1. Manually close and declare software HOME.
2. Run only a 2% checkout (3,072 counts).
3. Watch the mechanism continuously and use the cutoff immediately if motion
   is in the unsafe direction.
4. Require target and `XACTUAL` to return near zero and the driver to turn off.
5. Independently observe whether the mechanism physically returned closed.

### 3. One full command sequence

1. Manually close and start a fresh acknowledged supervisor process.
2. Run one 0 -> 153,600 -> 0 count sequence.
3. Observe or externally measure actual shaft/linkage travel.
4. Verify no hard stop is contacted and the mechanism has adequate margin.
5. Confirm final target and `XACTUAL` are near zero and motor energy is off.
6. Exercise ESTOP, USB removal, and the independent cutoff.

### 4. RocketPy replay

Run one generated case and verify the dashboard keeps these channels separate:

- RocketPy/SIL truth;
- raw STM32 controller demand;
- forced HIL demand;
- TMC target and `XACTUAL`;
- software zero/full flags and faults.

The physical actuator does not feed back into the precomputed trajectory.

### 5. Endurance

1. Run 20 supervised cycles with the default 30-second dwell.
2. Independently inspect closed/full travel, current, temperature, sound, and
   mechanical condition.
3. Stop if timing drifts, temperature fails to stabilize, the mechanism misses
   its observed positions, or any software/driver fault occurs.
4. After correcting any issue, restart from manual closure and a fresh
   software-HOME declaration.

### 6. Qualification

Run 100 supervised cycles. Acceptance requires:

- 100 complete 0 -> 153600 -> 0 target/XACTUAL sequences;
- independently observed acceptable mechanism motion on every cycle;
- zero TMC warnings, thermal warnings, or driver faults;
- zero protocol, stale-input, deadline, or skipped-sample failures;
- zero incomplete run records;
- stable command timing, current, and temperature.

This result is actuator endurance evidence under the commissioned setup. It is
not independent position-sensor validation.

### 7. Return to flight mode

Flash NORMAL and verify simulation/HIL/bench commands are rejected while real
sensors, radio, EKF, controller, actuator telemetry, watchdog, and the original
LED functions remain operational.
