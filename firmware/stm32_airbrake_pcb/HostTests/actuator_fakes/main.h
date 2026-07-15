/**
 * @file main.h
 * @brief Minimal HAL/CMSIS surface for host-side actuator state-machine tests.
 *
 * This fake intentionally exposes only symbols used by ambar_actuator.c.  It
 * makes the production [ARCH-5] source testable on Windows without silently
 * replacing any of its state-machine logic.
 */
#ifndef HOSTTEST_ACTUATOR_FAKE_MAIN_H
#define HOSTTEST_ACTUATOR_FAKE_MAIN_H

#include <stdint.h>

typedef enum
{
    HAL_OK = 0x00U,
    HAL_ERROR = 0x01U,
    HAL_BUSY = 0x02U,
    HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

typedef enum
{
    GPIO_PIN_RESET = 0U,
    GPIO_PIN_SET
} GPIO_PinState;

typedef struct
{
    uint32_t unused;
} GPIO_TypeDef;

extern GPIO_TypeDef g_fake_motor_diag_port;

#define MOTOR_DIAG0_GPIO_Port (&g_fake_motor_diag_port)
#define MOTOR_DIAG1_GPIO_Port (&g_fake_motor_diag_port)
#define MOTOR_DIAG0_Pin       ((uint16_t)0x0001U)
#define MOTOR_DIAG1_Pin       ((uint16_t)0x0002U)

uint32_t HAL_GetTick(void);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);

static inline uint32_t __get_PRIMASK(void)
{
    return 0U;
}

static inline void __disable_irq(void)
{
}

static inline void __set_PRIMASK(uint32_t value)
{
    (void)value;
}

static inline void __DMB(void)
{
}

#endif /* HOSTTEST_ACTUATOR_FAKE_MAIN_H */
