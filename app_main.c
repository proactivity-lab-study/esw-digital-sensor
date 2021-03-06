/**
 * @file app_main.c
 * 
 * @brief   Communicates with TTTW labkit accelerometer over I2C protocol. Writes 
 *          results to log output.
 *
 * MMA8653FC datasheet
 * https://www.nxp.com/docs/en/data-sheet/MMA8653FC.pdf
 * 
 * MMA8653FC application note
 * https://www.nxp.com/docs/en/application-note/AN4083.pdf
 * 
 * EFR32 Application Note on I2C
 * https://www.silabs.com/documents/public/application-notes/AN0011.pdf
 *
 * EFR32MG12 Wireless Gecko Reference Manual (I2C p501)
 * https://www.silabs.com/documents/public/reference-manuals/efr32xg12-rm.pdf
 * 
 * EFR32MG12 Wireless Gecko datasheet
 * https://www.silabs.com/documents/public/data-sheets/efr32mg12-datasheet.pdf
 *
 * GPIO API documentation 
 * https://docs.silabs.com/mcu/latest/efr32mg12/group-GPIO
 * 
 * ARM RTOS API
 * https://arm-software.github.io/CMSIS_5/RTOS2/html/group__CMSIS__RTOS.html
 *
 * @author Johannes Ehala, ProLab.
 * @license MIT
 *
 * Copyright Thinnect Inc. 2019
 * Copyright ProLab, TTÜ. 2021
 * 
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>

#include "retargetserial.h"

#include "cmsis_os2.h"

#include "platform.h"

#include "SignatureArea.h"
#include "DeviceSignature.h"

#include "loggers_ext.h"
#include "logger_fwrite.h"

#include "i2c_handler.h"
#include "mma8653fc_reg.h"
#include "gpio_handler.h"
#include "mma8653fc_driver.h"
#include "app_main.h"

#include "loglevels.h"
#define __MODUUL__ "main"
#define __LOG_LEVEL__ (LOG_LEVEL_main & BASE_LOG_LEVEL)
#include "log.h"

// Include the information header binary
#include "incbin.h"
INCBIN(Header, "header.bin");

static osThreadId_t dataReadyThreadId;

float calc_signal_energy(float buf[], uint32_t num_elements);

// Heartbeat loop - periodically print 'Heartbeat'
static void hb_loop (void *args)
{
    for (;;)
    {
        osDelay(10000);
        info1("Heartbeat");
    }
}

/**
 * @brief   Configures I2C, GPIO and sensor, wakes up on MMA8653FC data ready interrupt, fetches
 *          a batch of sensor data and analyzes data.
 */
static void mma_data_ready_loop (void *args)
{
    // TODO Initialize and enable I2C.

    // TODO Read Who-am-I registry
    
    // TODO To configure sensor put sensor in standby mode.
    
    // TODO Configure sensor for xyz data acquisition.
    
    // TODO Configure sensor to generate interrupt when new data becomes ready.
    
    // TODO Configure GPIO for external interrupts and enable external interrupts.
    
    // TODO Activate sensor.
    
    for (;;)
    {
        // TODO Wait for data ready interrupt signal from MMA8653FC sensor

        // TODO Get raw data
        
        // TODO Convert to engineering value
        
        // TODO Signal analysis

    }
}

int logger_fwrite_boot (const char *ptr, int len)
{
    fwrite(ptr, len, 1, stdout);
    fflush(stdout);
    return len;
}

int main ()
{
    PLATFORM_Init();

    // LEDs
    PLATFORM_LedsInit(); // This also enables GPIO peripheral.

    // Configure debug output.
    RETARGET_SerialInit();
    log_init(BASE_LOG_LEVEL, &logger_fwrite_boot, NULL);

    info1("Digi-sensor-demo "VERSION_STR" (%d.%d.%d)", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);

    // Initialize OS kernel.
    osKernelInitialize();

    // Create a thread.
    const osThreadAttr_t app_thread_attr = { .name = "heartbeat" , .priority = osPriorityNormal2 };
    osThreadNew(hb_loop, NULL, &app_thread_attr);

    // Create thread to receive data ready event and read data from sensor.
    const osThreadAttr_t data_ready_thread_attr = { .name = "data_ready_thread" };
    dataReadyThreadId = osThreadNew(mma_data_ready_loop, NULL, &data_ready_thread_attr);
    
    if (osKernelReady == osKernelGetState())
    {
        // Switch to a thread-safe logger
        logger_fwrite_init();
        log_init(BASE_LOG_LEVEL, &logger_fwrite, NULL);

        // Start the kernel
        osKernelStart();
    }
    else
    {
        err1("!osKernelReady");
    }

    for(;;);
}

/**
 * @brief 
 * Calculate energy of measured signal. 
 * 
 * @details 
 * Energy is calculated by subtracting bias from every sample and then adding 
 * together the square values of all samples. Energy is small if there is no 
 * signal (just measurement noise) and larger when a signal is present. 
 *
 * Disclaimer: The signal measured by the ADC is an elecrical signal, and its
 * unit would be joule, but since I don't know the exact load that the signal
 * is driving I can't account for the load. And so the energy I calculate here  
 * just indicates the presence or absence of a signal (and its relative 
 * strength), not the actual electrical energy in joules. 
 * Such a calculation can be done to all sorts of signals. There is probably
 * a more correct scientific term than energy for the result of this calculation
 * but I don't know what it is.
 *
 * Read about signal energy 
 * https://www.gaussianwaves.com/2013/12/power-and-energy-of-a-signal/
 *
 * @return Energy value.
 */
float calc_signal_energy(float buf[], uint32_t num_elements)
{
    static uint32_t i;
    static float signal_bias, signal_energy, res;

    signal_bias = signal_energy = res = 0;

    for (i = 0; i < num_elements; i++)
    {
        signal_bias += buf[i];
    }
    signal_bias /= num_elements;

    for (i = 0; i < num_elements; i++)
    {
        res = buf[i] - signal_bias; // Subtract bias
        signal_energy += res * res;
    }
    return signal_energy;
}
