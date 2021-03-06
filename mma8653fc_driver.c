/**
 * @file mma8653fc_driver.c
 *
 * @note    I2C set-up must be done separately and before the usage of this driver.
 *          GPIO interrupt set-up must be done separately if MMA8653FC interrupts are used.
 *
 * @author Johannes Ehala, ProLab.
 * @license MIT
 *
 * Copyright ProLab, TTÜ. 2021
 */

#include "cmsis_os2.h" // For osDelay() in sensor_reset() function.
#include "mma8653fc_reg.h"
#include "mma8653fc_driver.h"
#include "em_i2c.h"
#include "i2c_handler.h"

#include "loglevels.h"
#define __MODUUL__ "sdrv"
#define __LOG_LEVEL__ (LOG_LEVEL_mmadrv & BASE_LOG_LEVEL)
#include "log.h"

static uint8_t read_registry(uint8_t regAddr);
static void read_multiple_registries(uint8_t startRegAddr, uint8_t *rxBuf, uint16_t rxBufLen);
static void write_registry(uint8_t regAddr, uint8_t regVal);

/**
 * @brief   Reset MMA8653FC sensor (software reset).
 */
void sensor_reset (void)
{
    uint8_t regVal;
    
    regVal = read_registry(MMA8653FC_REGADDR_CTRL_REG2);
    regVal = (regVal & ~MMA8653FC_CTRL_REG2_SOFTRST_MASK) | (MMA8653FC_CTRL_REG2_SOFTRST_EN << MMA8653FC_CTRL_REG2_SOFTRST_SHIFT);
    write_registry(MMA8653FC_REGADDR_CTRL_REG2, regVal);
    osDelay(5*osKernelGetTickFreq()/1000); // Wait a little for reset to finish.
}

/**
 * @brief   Sets sensor to active mode. 
 */
void set_sensor_active ()
{
    // TODO Change mode to ACTIVE
}

/**
 * @brief   Sets sensor to standby mode. Sensor must be in standby mode when writing to
 *          different config registries.
 */
void set_sensor_standby ()
{
    // TODO Change mode to STANDBY
}

/**
 * @brief   Configures MMA8653FC sensor to start collecting xyz acceleration data.
 *
 * @param   dataRate Set data rate (1.56, 6.26, 12, 50, 100, 200, 400, 800 Hz)
 * @param   range Set dynamic range (+- 2g, +- 4g, +- 8g)
 * @param   powerMod Set power mode (normal, low-noise-low-power, highres, low-power)
 * 
 * @return  -1 if sensor is not in standby mode
 *           0 if configuration succeeded (no check)
 */
int8_t configure_xyz_data (uint8_t dataRate, uint8_t range, uint8_t powerMod)
{
    // TODO Check if sensor is in standby mode, control registers can only be modified in standby mode.
    
    // TODO Set data rate.
    
    // TODO Set dynamic range.
    
    // TODO Set power mode (oversampling). 
    
    return 0;
}

/**
 * @brief   Configures MMA8653FC sensor to start collecting xyz acceleration data.
 *
 * @param   polarity Set interrupt pin polarity.
 * @param   pinmode Set interrupt pin pinmode.
 * @param   interrupt Set interrupts to use.
 * @param   int_select Route interrupts to selected pin.
 *
 * @return  -1 if sensor is not in standby mode
 *           0 if configuration succeeded (no check)
 */
int8_t configure_interrupt (uint8_t polarity, uint8_t pinmode, uint8_t interrupt, uint8_t int_select)
{
    // TODO Check if sensor is in standby mode, control registers can only be modified in standby mode.
    
    // TODO Configure interrupt pin pinmode and interrupt transition direction

    // TODO Enable data ready interrupt

    // TODO Route data ready interrupt to sensor INT1 output pin (connected to port PA1 on the TTTW uC)

    return 0;
}

/**
 * @brief   Reads MMA8653FC STATUS and data registries.
 *
 * @return  Returns value of STATUS registry and x, y, z, 10 bit raw values (left-justified 2's complement)
 */
xyz_rawdata_t get_xyz_data()
{
    xyz_rawdata_t data;
    // TODO Read multiple registries for status and x, y, z raw data
    
    return data;
}

/**
 * @brief   Read value of one registry of MMA8653FC.
 *
 * @param   regAddr Address of registry to read.
 *
 * @return  value of registry with address regAddr.
 */
static uint8_t read_registry(uint8_t regAddr)
{
    uint8_t reg;
    // TODO Configure I2C_TransferSeq_TypeDef
    
    // TODO Write value to MMA8653FC registry
    return reg;
}

/**
 * @brief   Write a value to one registry of MMA8653FC.
 *
 * @param   regAddr Address of registry to read.
 * @param   regVal Value to write to MMA8653FC registry.
 *
 * @note    rxBuf is not used I think, maybe replace with NULL pointer.
 */
static void write_registry(uint8_t regAddr, uint8_t regVal)
{
    // TODO Configure I2C_TransferSeq_TypeDef
    
    // TODO Write value to MMA8653FC registry
    
    return ;
}

/**
 * @brief   Read multiple registries of MMA8653FC in one go.
 * @note    MMA8653FC increments registry pointer to read internally according to its own logic. 
 *          Registries next to each other are not necessarily read in succession. Check MMA8653FC
 *          datasheet to see how registry pointer is incremented.
 *
 * @param   startRegAddr Address of registry to start reading from.
 * @param   *rxBuf Pointer to memory area where read values are stored.
 * @param   rxBufLen Length/size of rxBuf memory area.
 */
static void read_multiple_registries(uint8_t startRegAddr, uint8_t *rxBuf, uint16_t rxBufLen)
{
    // TODO Configure I2C_TransferSeq_TypeDef 
    
    // TODO Do I2C transaction
    
    return ;
}

/**
 * @brief   Converts MMA8653FC sensor output value (left-justified 10-bit 2's complement
 *          number) to decimal number representing MMA8653FC internal ADC read-out 
 *          (including bias).
 *          
 * @param raw_val   is expected to be left-justified 10-bit 2's complement number
 *
 * @return          decimal number ranging between -512 ... 511
 */
int16_t convert_to_count(uint16_t raw_val)
{
    uint16_t res;
    // TODO Convert raw sensor data to ADC readout (count) value
    
    return res;
}

/**
 * @brief   Converts MMA8653FC sensor output value (left-justified 10-bit 2's complement
 *          number) to floating point number representing acceleration rate in g.
 *
 * @param raw_val       is expected to be left-justified 10-bit 2's complement number
 * @param sensor_scale  sensor scale 2g, 4g or 8g
 *
 * @return          floating point number, value depending on chosen sensor range 
 *                  +/- 2g  ->  range -2 ... 1.996
 *                  +/- 4g  ->  range -4 ... 3.992
 *                  +/- 8g  ->  range -8 ... 7.984
 */
float convert_to_g(uint16_t raw_val, uint8_t sensor_scale)
{
    float res;
    // TODO Convert raw sensor data to g-force acceleration value
    
    return res;
}
