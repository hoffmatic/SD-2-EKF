/*
 * PROJECT FILE OVERVIEW
 * Comment made: 2026-07-07 17:44:48 -04:00
 *
 * What this file does:
 *   This is the simple public entry point for the flight application scheduler. main.c calls it after board startup, then calls the task function forever.
 *
 * Process flow:
 *   Init stores device health, captures the pad reference, resets the estimator, and keeps the motor disabled. The task function then schedules IMU, barometer, radio, and actuator checks.
 *
 * Main variables and what can be changed:
 *   There are no tuning values in this header. Scheduler periods and barometer standard deviation live in ambar_app.c.
 *
 * Assumptions:
 *   main.c has already initialized clocks, GPIO, I2C, SPI, USB, and chip drivers before this layer starts.
 *
 * What is missing:
 *   There is no command parser, persistent configuration, or flight-data logging interface yet.
 */

#ifndef AMBAR_APP_H
#define AMBAR_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdint.h>

/*
 * ===================== AMBAR EKF PCB INTEGRATION - NEW FILE =====================
 *
 * Small application scheduler that replaces the old blocking one-second sensor
 * loop.  main.c initializes the board as before, then hands timing-sensitive
 * flight work to AmbarApp_Task().
 */

void AmbarApp_Init(HAL_StatusTypeDef sensor_status,
                   HAL_StatusTypeDef radio_status,
                   uint8_t motor_driver_ok);
void AmbarApp_Task(void);
void AmbarApp_HandleExtiPin(uint16_t GPIO_Pin);

#ifdef __cplusplus
}
#endif

#endif /* AMBAR_APP_H */
