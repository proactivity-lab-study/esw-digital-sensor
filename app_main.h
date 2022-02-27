/**
 * @file app_main.h
 * 
 * @author Johannes Ehala, ProLab.
 * @license MIT
 *
 * Copyright ProLab, TTÜ. 2021
 */

#ifndef ACCEL_APP_MAIN_H_
#define ACCEL_APP_MAIN_H_

// Number of data points in one batch of data
//#define ACC_XYZ_DATA_LEN        2400UL  // 3 sec @ 800Hz, 15 min 38 sec @ 1,56 Hz !!!
#define ACC_XYZ_DATA_LEN        40      // 3 sec @ 6,25Hz

#define CONVERT_TO_G    // This includes 'float' buffers for data, uncomment for 'int16_t' buffers

#define SENSOR_DATA_RANGE MMA8653FC_XYZ_DATA_CFG_2G_RANGE

#endif // ACCEL_APP_MAIN_H_
