/**
 * @file tmc5240.h
 * @brief Low-level SPI and pin-control interface for the TMC5240 stepper driver.
 *
 * OVERVIEW
 * --------
 * This module translates simple register, position, current-profile, and
 * status requests into TMC5240 SPI transactions.  It also owns the two direct
 * hardware controls: active-low DRV_ENN and active-low SLEEPN.
 *
 * HOW IT FITS INTO THE SYSTEM
 * ---------------------------
 * Startup calls TMC5240_Init() with motor outputs disabled to prove that the
 * driver responds consistently.  The actuator state machine then configures
 * and calls this module only after its higher-level safety gates pass.  See
 * CODE_GUIDE.md [ARCH-5] for the complete command-to-motion path.
 *
 * SAFETY BOUNDARY
 * ---------------
 * This file can remove motor energy immediately, but it does not decide when
 * movement is permitted.  Travel clamping, homing, timeouts, ESTOP handling,
 * and phase-based permissions belong to ambar_actuator.c.  XACTUAL is the
 * driver's internal ramp-generator position, not independent shaft feedback.
 * Current values remain uncalibrated until the board sense resistor and motor
 * are characterized.
 */
#ifndef TMC5240_H
#define TMC5240_H

#include "main.h"
#include <stdint.h>

/* -------------------------------------------------------------------------- */
/* Device identity and register map                                            */
/* -------------------------------------------------------------------------- */

/** IOIN.VERSION is read twice at startup to verify stable SPI communication. */
#define TMC5240_REG_IOIN 0x04
#define TMC5240_EXPECTED_VERSION 0x40
/** Stable value observed twice on the assembled presentation PCB. */
#define TMC5240_PRESENTATION_BOARD_VERSION 0x41

/** Registers used by the bounded position-mode implementation. */
#define TMC5240_REG_GCONF       0x00U
#define TMC5240_REG_GSTAT       0x01U
#define TMC5240_REG_DRV_CONF    0x0AU
#define TMC5240_REG_GLOBALSCALER 0x0BU
#define TMC5240_REG_IHOLD_IRUN  0x10U
#define TMC5240_REG_TPOWERDOWN  0x11U
#define TMC5240_REG_RAMPMODE    0x20U
#define TMC5240_REG_XACTUAL     0x21U
#define TMC5240_REG_VSTART      0x23U
#define TMC5240_REG_A1          0x24U
#define TMC5240_REG_V1          0x25U
#define TMC5240_REG_AMAX        0x26U
#define TMC5240_REG_VMAX        0x27U
#define TMC5240_REG_DMAX        0x28U
#define TMC5240_REG_D1          0x2AU
#define TMC5240_REG_VSTOP       0x2BU
#define TMC5240_REG_TZEROWAIT   0x2CU
#define TMC5240_REG_XTARGET     0x2DU
#define TMC5240_REG_RAMP_STAT   0x35U
#define TMC5240_REG_CHOPCONF    0x6CU
#define TMC5240_REG_DRV_STATUS  0x6FU

#define TMC5240_RAMPMODE_POSITION 0x00U

/** DRV_STATUS bits that require immediate energy-off fault handling. */
#define TMC5240_DRV_STATUS_S2GB  (1UL << 28U)
#define TMC5240_DRV_STATUS_S2GA  (1UL << 27U)
#define TMC5240_DRV_STATUS_OTPW  (1UL << 26U)
#define TMC5240_DRV_STATUS_OT    (1UL << 25U)
#define TMC5240_DRV_STATUS_S2VSB (1UL << 13U)
#define TMC5240_DRV_STATUS_S2VSA (1UL << 12U)
#define TMC5240_DRV_STATUS_HARD_FAULT_MASK \
    (TMC5240_DRV_STATUS_S2GB | TMC5240_DRV_STATUS_S2GA | \
     TMC5240_DRV_STATUS_OTPW | TMC5240_DRV_STATUS_OT | \
     TMC5240_DRV_STATUS_S2VSB | TMC5240_DRV_STATUS_S2VSA)
/* -------------------------------------------------------------------------- */
/* Raw device communication                                                    */
/* -------------------------------------------------------------------------- */

/** Read or write one 32-bit register using a five-byte SPI datagram. */
HAL_StatusTypeDef TMC5240_ReadRegister(uint8_t address, uint32_t *value);
HAL_StatusTypeDef TMC5240_WriteRegister(uint8_t address, uint32_t value);
/** Read the VERSION field from IOIN. */
HAL_StatusTypeDef TMC5240_ReadVersion(uint8_t *version);

/** Wake and read the driver twice without enabling the motor outputs. */
HAL_StatusTypeDef TMC5240_BringupTest(uint8_t *version);
/** Validate the detected version against the active build profile. */
HAL_StatusTypeDef TMC5240_Init(uint8_t *version);

/* -------------------------------------------------------------------------- */
/* Direct hardware gates                                                       */
/* -------------------------------------------------------------------------- */

/** Drive active-low DRV_ENN. Passing zero removes motor output energy. */
void TMC5240_SetDriverEnabled(uint8_t enabled);
/** Drive active-low SLEEPN. Passing zero places the driver in sleep. */
void TMC5240_SetAwake(uint8_t awake);

/* -------------------------------------------------------------------------- */
/* Driver profile, motion, and diagnostics                                     */
/* -------------------------------------------------------------------------- */

/** Load the conservative or presentation profile selected at compile time. */
HAL_StatusTypeDef TMC5240_ConfigureSafeDefaults(void);
/** Program placeholder current-scale fields; values are not yet calibrated. */
HAL_StatusTypeDef TMC5240_SetCurrentLimits(uint16_t hold_current_ma,
                                           uint16_t run_current_ma);
/** Set GLOBALSCALER (zero means the device's full-scale special value). */
HAL_StatusTypeDef TMC5240_SetGlobalScaler(uint8_t scaler);
/** Configure the ramp generator's bounded velocity and acceleration fields. */
HAL_StatusTypeDef TMC5240_SetMotionLimits(uint32_t max_velocity_steps_per_s,
                                          uint32_t max_accel_steps_per_s2);
/** Re-label the driver's internal ramp position; this is not encoder homing. */
HAL_StatusTypeDef TMC5240_SetActualPosition(int32_t position_steps);
/** Command an absolute internal-ramp target after the caller applies limits. */
HAL_StatusTypeDef TMC5240_SetTargetPosition(int32_t target_steps);
/** Read the internal ramp position; this is not independent shaft feedback. */
HAL_StatusTypeDef TMC5240_ReadActualPosition(int32_t *position_steps);
/** Read the current absolute ramp target used by guarded recovery preflight. */
HAL_StatusTypeDef TMC5240_ReadTargetPosition(int32_t *position_steps);
/** Read raw DRV_STATUS for fault interpretation by the actuator layer. */
HAL_StatusTypeDef TMC5240_ReadDriverStatus(uint32_t *driver_status);
/** Return nonzero when any status bit in HARD_FAULT_MASK is active. */
uint8_t TMC5240_DriverStatusHasHardFault(uint32_t driver_status);
/** Disable outputs first, then freeze the ramp target at its current position. */
HAL_StatusTypeDef TMC5240_Stop(void);
/** Return DIAG0 in bit 0 and DIAG1 in bit 1. */
uint8_t TMC5240_ReadDiagPins(void);

#endif
