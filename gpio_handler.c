/**
 * @file gpio_handler.c
 *
 * @brief Initialize GPIO and configure for I2C data transfer (SDA and SCL). 
 *        Configure GPIO pin for external interrupts.
 *
 * @author Johannes Ehala, ProLab.
 * @license MIT
 *
 * Copyright ProLab, TTÜ. 2021
 */

#include "em_cmu.h"
#include "em_gpio.h"
#include "cmsis_os2.h"

#include "gpio_handler.h"

#include "loglevels.h"
#define __MODUUL__ "gpio"
#define __LOG_LEVEL__ (LOG_LEVEL_gpio & BASE_LOG_LEVEL)
#include "log.h"

volatile static osThreadId_t resumeThreadID;
volatile static uint32_t resumeThreadFlagID;

/**
 * @brief Initialize GPIO interface and configure for I2C communication. 
 *
 * Accelerometer sensor is connected to port A pin 2 (SCL) and pin 3 (SDA) on TTTW lab-kit.
 */
void gpio_i2c_pin_init (void)
{
    // TODO Enable GPIO peripheral

    // TODO Take control of SDC and SCL output pins.
}

/**
 * @brief Initialize GPIO interface and configure for external interrupts. 
 *
 * Accelerometer sensor interrupt 1 (INT1) is connected to port A pin 1 on TTTW lab-kit. 
 */
void gpio_external_interrupt_init (void)
{
    // TODO Enable GPIO peripheral

    // TODO Configure pin
    
    // TODO Configure external interrupts

}

void gpio_external_interrupt_disable ()
{
    GPIO_IntDisable(GPIO_IF_EXTI_NUM);
    NVIC_DisableIRQ(GPIO_ODD_IRQn);
}

/**
 * @brief Enable exteranl interrupts on GPIO pins.
 *
 * @param   tID, ID of the thread that handles the interrupt (deferred interrupt handling)
 * @param   tFlag, flag id for thread tID
 * 
 * @note    The EXTIx number (index) determines the external interrupt to use. GPIO ports and pins
 *          are wired to the interrupt controller (NVIC) based on the EXTI number (index). Only
 *          certain ports and pins are available for each external interrupt. Check EFR32MG12 manual
 *          p1114 and GPIO application note p11-12 for more information.
 *          EXTIx number also determines which IRQHandler function to use - GPIO_ODD_IRQHandler or
 *          GPIO_EVEN_IRQHandler.
 */
void gpio_external_interrupt_enable (osThreadId_t tID, uint32_t tFlag)
{
    resumeThreadID = tID;
    resumeThreadFlagID = tFlag;
    
    GPIO_IntClear(GPIO_IF_EXTI_NUM);
    
    NVIC_EnableIRQ(GPIO_ODD_IRQn);
    NVIC_SetPriority(GPIO_ODD_IRQn, 3); 

    GPIO_IntEnable(GPIO_IF_EXTI_NUM);
}

void GPIO_ODD_IRQHandler (void)
{
    // TODO Get pending interrupts
    // TODO Clear interrupt
}
