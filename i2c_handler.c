/**
 * @file i2c_handler.c
 *
 * @brief Init I2C0 and transmit-receive.
 
 * @note The accelerometer sensor is always turned on on the TTTW lab-kit. So
 * no power ON or enable has to be done.
 * 
 * @author Johannes Ehala, ProLab.
 * @license MIT
 *
 * Copyright ProLab, TTÜ. 2021
 */

#include "em_cmu.h"

#include "i2c_handler.h"
#include "gpio_handler.h"

/**
 * @brief Init I2C interface. 
 *
 * Accelerometer sensor is connected to port A pin 2 (SCL) and pin 3 (SDA), I2C0
 * must be routed to those pins.
 */
void i2c_init (void)
{
    // TODO Enable I2C clock.

    // TODO Initialize and configure SDA and SCL pins for I2C data transfer.
	
	// TODO Route I2C SDA and SCL to GPIO pins (efr32mg12-datasheet page 188).
    
    // TODO Initialize I2C. 
}

void i2c_enable (void)
{
    I2C_Enable(I2C0, true);
}

void i2c_disable (void)
{
    I2C_Enable(I2C0, false);
}

void i2c_reset (void)
{
    I2C_Reset(I2C0);
}

I2C_TransferSeq_TypeDef * i2c_transaction (I2C_TransferSeq_TypeDef * seq)
{
    // TODO Do a polled transfer.
    return seq;
}

