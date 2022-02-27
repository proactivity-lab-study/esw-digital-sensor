/**
 * @file gpio_handler.h
 *
 * @author Johannes Ehala, ProLab.
 * @license MIT
 *
 * Copyright ProLab, TTÜ. 2021
 */

#ifndef GPIO_HANDLER_H_
#define GPIO_HANDLER_H_

#include "cmsis_os2.h"
#include "i2c_handler.h" // Include I2C SDA and SCL port and pins

#define GPIO_IF_EXTI_NUM 0  // TODO Replace with actual IF number

// Public functions
void gpio_i2c_pin_init (void);
void gpio_external_interrupt_init(void);
void gpio_external_interrupt_enable(osThreadId_t tID, uint32_t tFlag);
void gpio_external_interrupt_disable(void);

#endif // GPIO_HANDLER_H_
