/*
 * PROJECT FILE OVERVIEW
 * Comment made: 2026-07-07 17:44:48 -04:00
 *
 * What this file does:
 *   This header describes the low-level TMC5240 motor-driver interface. It can read and write driver registers and control enable and sleep pins.
 *
 * Process flow:
 *   main.c wakes the driver and reads its version during startup. The actuator safety layer later decides whether the enable pin may ever be released for movement.
 *
 * Main variables and what can be changed:
 *   TMC5240_EXPECTED_VERSION should only change if a different driver chip is used. Motion settings are not defined here yet.
 *
 * Assumptions:
 *   The driver is wired to SPI2 and uses active-low DRV_ENN and sleep-style control pins as named in main.h.
 *
 * What is missing:
 *   No motion profiles, current limits, homing, stall or limit handling, or step commands are exposed yet.
 */

/*
 * tmc5240.h
 *
 *  Created on: Jul 1, 2026
 *      Author: hoffman
 */
#ifndef TMC5240_H
#define TMC5240_H

#include "main.h"
#include <stdint.h>

/*
 * TMC5240 motor-driver interface.
 *
 * This layer is only the electrical/SPI access layer.  It does not decide when
 * the airbrakes are allowed to move; ambar_actuator.c owns that safety decision.
 * In the first EKF build, movement remains disabled unless AMBAR_ENABLE_ACTUATOR
 * is explicitly enabled and the homing/safety state is valid.
 */

/* IOIN.VERSION is the first register we read to confirm SPI communication. */
#define TMC5240_REG_IOIN 0x04
#define TMC5240_EXPECTED_VERSION 0x40

/*
 * BEGIN AMBAR BENCH-GATED EXPANSION - MOTION REGISTER MAP
 *
 * These register names let the actuator safety layer configure conservative
 * bench motion.  The addresses follow the TMC5240/TMC stepper-controller
 * register map family; comments in tmc5240.c explain the safety limits.
 */
#define TMC5240_REG_GCONF       0x00U
#define TMC5240_REG_GSTAT       0x01U
#define TMC5240_REG_IHOLD_IRUN  0x10U
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
#define TMC5240_REG_DRV_STATUS  0x6FU

#define TMC5240_RAMPMODE_POSITION 0x00U
/* END AMBAR BENCH-GATED EXPANSION - MOTION REGISTER MAP */

/* Low-level 40-bit TMC5240 register access over SPI2. */
HAL_StatusTypeDef TMC5240_ReadRegister(uint8_t address, uint32_t *value);
HAL_StatusTypeDef TMC5240_WriteRegister(uint8_t address, uint32_t value);
HAL_StatusTypeDef TMC5240_ReadVersion(uint8_t *version);

/* Safe first-board checks. These do not enable motor movement. */
HAL_StatusTypeDef TMC5240_BringupTest(uint8_t *version);
HAL_StatusTypeDef TMC5240_Init(uint8_t *version);

/* Hardware control pins for the driver enable and sleep signals. */
void TMC5240_SetDriverEnabled(uint8_t enabled);
void TMC5240_SetAwake(uint8_t awake);

/* Conservative bench-motion helpers.  They do not decide if motion is safe. */
HAL_StatusTypeDef TMC5240_ConfigureSafeDefaults(void);
HAL_StatusTypeDef TMC5240_SetCurrentLimits(uint16_t hold_current_ma,
                                           uint16_t run_current_ma);
HAL_StatusTypeDef TMC5240_SetMotionLimits(uint32_t max_velocity_steps_per_s,
                                          uint32_t max_accel_steps_per_s2);
HAL_StatusTypeDef TMC5240_SetActualPosition(int32_t position_steps);
HAL_StatusTypeDef TMC5240_SetTargetPosition(int32_t target_steps);
HAL_StatusTypeDef TMC5240_ReadActualPosition(int32_t *position_steps);
HAL_StatusTypeDef TMC5240_ReadDriverStatus(uint32_t *driver_status);
HAL_StatusTypeDef TMC5240_Stop(void);
uint8_t TMC5240_ReadDiagPins(void);

#endif
