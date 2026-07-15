/**
 * @file ambar_app.h
 * @brief Public lifecycle API for the cooperative AMBAR application.
 *
 * OVERVIEW
 * --------
 *   Public lifecycle entry points for the [ARCH-1] cooperative application
 *   scheduler and its interrupt forwarding hook.
 *
 * HOW IT WORKS
 * ------------
 *   main.c initializes hardware, calls AmbarApp_Init() once, then calls
 *   AmbarApp_Task() continuously. The task joins [ARCH-4] flight output to the
 *   final [ARCH-5] actuator safety layer while servicing communications.
 *
 * OWNERSHIP
 * ---------
 *   This header intentionally exposes no tuning. Scheduler periods live in
 *   ambar_app.c; feature locks, flight tuning, and actuator limits live in their
 *   owning modules.
 *
 * SAFETY BOUNDARY
 * ---------------
 *   main.c has initialized clocks, GPIO, I2C, SPI, USB, and chip drivers. The
 *   task must run frequently because there is no RTOS or independent motor task.
 *
 *   The application may request motion but cannot bypass ambar_actuator.c.
 *   Disconnect, DISARM, simulation timeout, ESTOP, and fault paths remove motor
 *   energy immediately.
 */

#ifndef AMBAR_APP_H
#define AMBAR_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdint.h>

/* -------------------- Application lifecycle ([ARCH-1]) -------------------- */

/* Initialize singleton state after all board drivers have completed bring-up. */
void AmbarApp_Init(HAL_StatusTypeDef sensor_status,
                   HAL_StatusTypeDef radio_status,
                   uint8_t motor_driver_ok);

/* Run one bounded cooperative pass; main.c calls this continuously. */
void AmbarApp_Task(void);

/* Forward board EXTI pins to the owning subsystem; actuator DIAG faults latch here. */
void AmbarApp_HandleExtiPin(uint16_t GPIO_Pin);

#ifdef __cplusplus
}
#endif

#endif /* AMBAR_APP_H */
