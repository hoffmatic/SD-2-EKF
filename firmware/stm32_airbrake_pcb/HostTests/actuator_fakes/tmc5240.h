/**
 * @file tmc5240.h
 * @brief Host declarations for the observable fake TMC5240 used in actuator tests.
 *
 * Implementations live in actuator_autoretract_tests.c and record commands,
 * positions, and enable transitions.  The signatures deliberately match the
 * production low-level API so ambar_actuator.c is compiled unchanged.
 */
#ifndef HOSTTEST_ACTUATOR_FAKE_TMC5240_H
#define HOSTTEST_ACTUATOR_FAKE_TMC5240_H

#include "main.h"

#include <stdint.h>

void TMC5240_SetDriverEnabled(uint8_t enabled);
HAL_StatusTypeDef TMC5240_SetCurrentLimits(uint16_t hold_current_ma,
                                           uint16_t run_current_ma);
HAL_StatusTypeDef TMC5240_SetMotionLimits(uint32_t max_velocity_steps_per_s,
                                          uint32_t max_accel_steps_per_s2);
HAL_StatusTypeDef TMC5240_SetActualPosition(int32_t position_steps);
HAL_StatusTypeDef TMC5240_SetTargetPosition(int32_t target_steps);
HAL_StatusTypeDef TMC5240_ReadActualPosition(int32_t *position_steps);
HAL_StatusTypeDef TMC5240_ReadTargetPosition(int32_t *position_steps);
HAL_StatusTypeDef TMC5240_ReadDriverStatus(uint32_t *driver_status);
uint8_t TMC5240_DriverStatusHasHardFault(uint32_t driver_status);
HAL_StatusTypeDef TMC5240_Stop(void);

#endif /* HOSTTEST_ACTUATOR_FAKE_TMC5240_H */
