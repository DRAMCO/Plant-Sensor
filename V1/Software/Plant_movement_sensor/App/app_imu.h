/*
 * app_imu.h
 *
 *  Created on: Nov 1, 2023
 *      Author: Sarah Goossens
 */

#ifndef APP_IMU_H_
#define APP_IMU_H_

#include "main.h"
#include "../Drivers/Invn/Devices/SensorTypes.h"
#include "../Drivers/Invn/Devices/DeviceIcm20948.h"

#define PLANTSENSOR_GRYO_FSR        2000 //250 // +- 500 dps
#define IMU_GYRO_DEFAULT_FSR        2000
#define PLANTSENSOR_ACCEL_FSR       2 // 4 G
#define IMU_ACCEL_DEFAULT_FSR       4
#define IMU_DEFAULT_SAMPL_FREQ  	22 // 22: 48.9/ 4: 225 Hz

#define RAW_Q_FORMAT_ACC_COMMA_BITS 10     // Number of bits used for comma part of raw data.
#define RAW_Q_FORMAT_GYR_COMMA_BITS 5    // Number of bits used for comma part of raw data.
#define RAW_Q_FORMAT_CMP_COMMA_BITS 4    // Number of bits used for comma part of raw data.


/// Fixed-point Format: 11.5 (16-bit)
typedef int32_t fixed_point_t;

typedef struct
{
    int32_t w;
    int32_t x;
    int32_t y;
    int32_t z;
} imu_quat_t;

void ImuThreadInit();
void ImuThreadStart(const void * params);
void ImuNotifyFromISR(uint32_t notValue);
//void ImuThreadNotify(uint32_t notValue);

void ICM20948_reset();
void ICM_20948_sleepModeEnable(int enable, void * context);
void calibration_callback();
int reset_DMP (void *context);

// Set FSR for Gyro and Accel
void imu_config_fsr_gyro(int fsr_in);
void imu_config_fsr_accel(int fsr_in);

fixed_point_t float_to_fixed_euler(float input);
fixed_point_t float_to_fixed_quat(float input);


void PollImuDevice(void);

#endif /* APP_IMU_H_ */
