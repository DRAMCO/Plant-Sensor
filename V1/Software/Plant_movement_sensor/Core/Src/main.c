/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

// network_connect_init
// [ICM20948Setup.c] inv_icm20948_poll_sensor()
//2024
// [app_gateway]  Current
// data event #
// [network_connect] Start listening to search for a new device. RTC PID controller is skipped during this time.
// [SX1280.c] [GoToStandby] Radio in standby mode.
// [app_rtc_sync] HOSTPP10S #
// [app_hal_sync] Error: Jump out of while loop to trim pidRtcOutput.
// [app_network_connect] Known module giving me a message.
// [app_hal_sync] RTCPPS #
// app_rtc] Wrong notification received while waiting to change the RTC
// [app_hal_sync] PID RTC done. Error =
// [app_hal_sync] RTOS time difference GNSSPPS #
// [app_rtc_sync] GNSSPP10S #
// [app_hal_sync] RTOS time difference GNSSPPS #
// xEpochLht5 =
// [network_connect]      Checksum done at
// [app_network_connect] [BroadCastMyID] DIOx pin time-out occurred...(>2s).
// [app_network_connect] [NetConThread] Started
// [app_hal_sync] Pairing process started or GNSSPPS is at 9s: no RTC PID
// [app_network_connect] [ConfirmPairing]
// aligned with its time stamp
// [app_rtc] No notification received from app
// No notification received from app_hal_sync to change
// [SX1280] [HalReset] Lora Radio status returns 0, try again to reset radio.
// [SX1280.c] LoRa radio reset done.
// 0x00008000
// [gateway] Super capacitor loaded. Ready to start.
// [SX1280.c] [SX1280WaitOnBusy] Error: Jump out of while loop (Radio busy for > 20s).
// [IMU read register] IMU SPI busy before reading register.
// [app_imu] data event
// [init] Notification received that IMU is initialized. Initialize radio.
// init] Error: no notification from IMU in the last 40s.
// Sensor started.
// [initThread] Sync word at reset time
// [app_network_connect] [NetConThread] Started.
// TIM2
// SysTick
// aligned with its time stamp
// RTOS time difference with previous GNSSPP10S
// to set RTC.    GNSS last received
// [app_rtc_sync] GNSSPPS #
// systick_callback
// SysTick_Handler
// TickType_t
// IWDG_KR
// CCCC
// HAL_FLASH_Program()
// HAL_FLASH_Unlock();
// HAL_FLASH_Lock();
// FLASH_Read
// 0x%
// erase
// HAL_FLASHEx_Erase()
// inv_sensor_event_t
// inv_device_get_sensor_config
// inv_device_set_sensor_config
// INV_SENSOR_CONFIG_OFFSET
// (int)(event->data.gyr.bias[0]*1000)


//11:25:02.831 -> 29259 [app_imu] [ImuThread] Started.
//11:25:02.831 -> Enter init2.
//11:25:02.831 -> compass state = 1.
//11:25:03.018 -> Setting-up ICM device with lower driver.
//11:25:03.018 -> Booting up Icm20948...
//11:25:03.018 -> Reading WHOAMI...
//11:25:03.018 -> WHOAMI value: 0xea.
//11:25:03.065 -> Putting Icm20948 in sleep mode...
//11:25:03.065 -> Initialize to SPI.
//11:25:03.065 -> Initialize lower driver.
//11:25:03.065 -> Read REG_USER_CTRL and REG_WHO_AM_I done.
//11:25:03.065 -> set_lowpower_or_highperformance done.
//11:25:03.065 -> Write to REG_USER_CTRL done.
//11:25:03.065 ->  [Icm20948LoadFirmware.c] Load firmware started.
//...
//11:25:36.045 -> Hex dump: 10 18 F5 00 B5 96 F5 18 BB AF D0 B7 9F E0 F0 D0
//11:25:36.090 -> Hex dump: F3 CF F2 CC D0 F3 CD F2 CA D0 F3 CB F2 C8 D0 F3
//11:25:36.090 -> Hex dump: C9 E0
//11:25:36.137 -> Image loaded to IMU.
//11:25:36.137 -> Load firmware done.
//11:25:36.137 -> Image loaded to IMU.
//11:25:36.137 -> Initialize auxiliary sensors
//11:25:36.137 -> Enter setup compass.
//11:25:36.185 -> Compass found.
//11:25:36.276 -> We're good to go !
//11:25:36.276 -> **********************************************************************
//11:25:36.320 -> Enter setup compass.
//11:25:36.366 -> Compass found.
//11:25:36.459 -> Ping SENSOR_ROTATION_VECTOR OK
//11:25:36.459 -> Starting SENSOR_ROTATION_VECTOR @ 50000 us
//11:25:36.459 -> Sensor started.
//11:25:36.459 -> 62907 [init] Notification received that IMU is initialized. Initialize radio.
//11:25:36.459 -> 62912 [init] IMU calibration process done.
//11:25:36.459 -> 62916 [app_supercap] Check Super capacitor voltage level.
//11:25:36.508 -> 62920 [app_supercap] Super capacitor loading (iteration #0), SCAP = 1798mV, BATT = 3300mV, BATTIN = 3300mV, Core = 899mV, Core temp = 22°C.
//11:25:36.508 -> [IMU] Notification to init given that IMU is initialized.

//17:41:41.368 -> Start calibration
//17:41:41.368 -> 68877 [app_imu] Sensor GYRO FSR (current): 2000
//17:41:41.368 -> 68881 [app_imu] Sensor GYRO FSR (already) set to: 2000
//17:41:41.368 -> 68885 [app_imu] Sensor GYRO FSR (current): 2000
//17:41:41.368 -> 68889 [app_imu] Sensor ACCEL FSR (current): 4
//17:41:41.368 -> 68893 [app_imu] Sensor ACCEL FSR (already) set to: 4
//17:41:41.368 -> 68897 [app_imu] Sensor ACCEL FSR (current): 4
//17:41:41.415 -> 68901 [app_imu] Calibration callback
//17:41:41.415 -> 68904 [app_imu] gyroAccuracy = 0, accelAccuracy = 0, magAccuracy = 0.
//17:41:41.415 -> 68909 [app_imu] Start timer 1s
//17:41:41.415 -> 68912 [app_imu] NOT CALIBRATED
//17:41:41.415 -> 68915 [app_imu] Start GYRO for calibration
//17:41:55.371 -> 82902 Cal status: gyro: 3
//17:41:55.418 -> 82905 [app_imu] Calibration callback
//17:41:55.418 -> 82908 [app_imu] gyroAccuracy = 3, accelAccuracy = 0, magAccuracy = 0.
//17:41:55.418 -> 82913 [app_imu] Start timer 500ms
//17:41:55.418 -> 82916 [app_imu] GYRO CALIBRATED
//17:41:55.418 -> 82919 [app_imu] Start ACCEL for calibration
//17:44:08.637 -> 216110 Cal status: accel: 3
//17:44:08.637 -> 216113 [app_imu] Calibration callback
//17:44:08.637 -> 216116 [app_imu] gyroAccuracy = 3, accelAccuracy = 3, magAccuracy = 0.
//17:44:08.637 -> 216121 [app_imu] Start timer 200ms
//17:44:08.637 -> 216124 [app_imu] ACCEL CALIBRATED
//17:44:08.637 -> 216127 [app_imu] Start MAG for calibration

//int inv_icm20948_set_bias(struct inv_icm20948 * s, enum inv_icm20948_sensor sensor, const void * bias)
//{
//	int bias_q16[3];
//	int bias_in[3];
//	int rc = 0;
//	short shift;
//	switch(sensor) {
//		case INV_ICM20948_SENSOR_ACCELEROMETER :
//			memcpy(bias_q16, bias, sizeof(bias_q16));
//			//convert from q16 to q25
//			bias_in[0] = bias_q16[0] << (25 - 16);
//			bias_in[1] = bias_q16[1] << (25 - 16);
//			bias_in[2] = bias_q16[2] << (25 - 16);
//			rc |= inv_icm20948_ctrl_set_acc_bias(s, bias_in);
//			break;
//		case INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED:
//		case INV_ICM20948_SENSOR_GYROSCOPE:
//			memcpy(bias_q16, bias, sizeof(bias_q16));
//			//convert from q16 to :
//			//Q19 => 2000dps
//			//Q20 => 1000dps
//			//Q21 => 500dps
//			//Q22 => 250dps
//			shift = ((20 + (MPU_FS_1000dps - inv_icm20948_get_gyro_fullscale(s))) - 16);
//			bias_in[0] = bias_q16[0] << shift;
//			bias_in[1] = bias_q16[1] << shift;
//			bias_in[2] = bias_q16[2] << shift;
//
//			rc |= inv_icm20948_ctrl_set_gyr_bias(s, bias_in);
//			break;
//		case INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
//		case INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD:
//			memcpy(bias_q16, bias, sizeof(bias_q16));
//			// bias is already in q16
//			rc |= inv_icm20948_ctrl_set_mag_bias(s, bias_q16);
//			break;
//		default :
//			rc = -1;
//			break;
//	}
//	return (rc == 0) ? 1 : rc;
//}
//
//int inv_icm20948_get_bias(struct inv_icm20948 * s, enum inv_icm20948_sensor sensor, void * bias)
//{
//	int bias_qx[3];
//	int bias_out[3];
//	int rc = 0;
//	short shift;
//	switch(sensor) {
//		case INV_ICM20948_SENSOR_ACCELEROMETER :
//			rc |= inv_icm20948_ctrl_get_acc_bias(s, bias_qx);
//			//convert from q25 to q16
//			bias_out[0] = bias_qx[0] >> (25 - 16);
//			bias_out[1] = bias_qx[1] >> (25 - 16);
//			bias_out[2] = bias_qx[2] >> (25 - 16);
//			memcpy(bias, bias_out, sizeof(bias_out));
//			break;
//		case INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED:
//		case INV_ICM20948_SENSOR_GYROSCOPE:
//			rc |= inv_icm20948_ctrl_get_gyr_bias(s, bias_qx);
//			//convert from qn to q16:
//			//Q19 => 2000dps
//			//Q20 => 1000dps
//			//Q21 => 500dps
//			//Q22 => 250dps
//			shift = ((20 + (MPU_FS_1000dps - inv_icm20948_get_gyro_fullscale(s))) - 16);
//			bias_out[0] = bias_qx[0] >> shift;
//			bias_out[1] = bias_qx[1] >> shift;
//			bias_out[2] = bias_qx[2] >> shift;
//
//			memcpy(bias, bias_out, sizeof(bias_out));
//			break;
//		case INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
//		case INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD:
//			rc |= inv_icm20948_ctrl_get_mag_bias(s, bias_qx);
//			// bias is already in q16
//			memcpy(bias, bias_qx, sizeof(bias_qx));
//			break;
//		default:
//			rc = -1;
//			break;
//	}
//	return (rc == 0) ? 3*sizeof(float) : rc;
//}


//00:04:04.940 -> 103584 [app_gateway] Put IMU to sleep.
//00:04:05.035 -> 103688 [app_imu] Sleep mode ON
//00:04:05.035 -> 103692 [app_imu] PWR_MGMT register at begin of sleepModeEnable: 21
//00:04:05.035 -> 103700 [app_imu] New reg: 61
//00:04:05.035 -> 103704 [app_imu] PWR_MGMT register at end of sleepModeEnable: 61

// 0b
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] PWR->CR1      = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif


//00:04:59.824 -> Icm20948Transport.c read_reg, data = FF F7 24.
//00:04:59.824 -> Icm20948Transport.c read_reg, data = 00 07 28.
//00:04:59.824 -> Icm20948Transport.c read_reg, data = 00 18 1C.
//00:04:59.824 -> 106205 [app_imu] data event #1, SENSOR_ROTATION_VECTOR (e-3): 8672 -4067 2762 -782
//00:04:59.872 -> Icm20948Transport.c read_reg, data = FF F7 24.
//00:04:59.872 -> Icm20948Transport.c read_reg, data = 00 07 28.
//00:04:59.872 -> Icm20948Transport.c read_reg, data = 00 18 1C.
//...
//00:05:05.387 -> .Icm20948Transport.c read_reg, data = FF F7 24.
//00:05:05.434 -> Icm20948Transport.c read_reg, data = 00 07 28.
//00:05:05.434 -> Icm20948Transport.c read_reg, data = 00 18 1C.
//00:05:05.434 -> .
//00:05:05.434 -> 111806 [app_gateway] Accelerometer offsets x = nan, y = 0.000000, z = 0.000000.
//00:05:05.434 -> 111814 [app_gateway] Accelerometer Bias x = -0.176194, y = 0.042968, z = 0.000000.
//00:05:05.434 -> 111822 [app_gateway]     Gyrometer Bias x = -0.176194, y = 0.042968, z = 0.000000.
//00:05:05.434 -> 111830 [app_gateway]  Magnetometer Bias x = -0.176194, y = 0.042968, z = 0.000000.


//YA_OFFS_H
//WIA2
// HXL

// below are the registers for the DMP of the IMU:

//// data output control
//#define DATA_OUT_CTL1 (4 * 16)
//#define DATA_OUT_CTL2 (4 * 16 + 2)
//#define DATA_INTR_CTL (4 * 16 + 12)
//#define FIFO_WATERMARK (31 * 16 + 14)
//// motion event control
//#define MOTION_EVENT_CTL (4 * 16 + 14)
//// indicates to DMP which sensors are available
//#define DATA_RDY_STATUS (8 * 16 + 10)
//// batch mode
//#define BM_BATCH_CNTR (27 * 16)
//#define BM_BATCH_THLD (19 * 16 + 12)
//#define BM_BATCH_MASK (21 * 16 + 14)
//// sensor output data rate
//#define ODR_ACCEL (11 * 16 + 14)
//#define ODR_GYRO (11 * 16 + 10)
//#define ODR_CPASS (11 * 16 + 6)
//#define ODR_ALS (11 * 16 + 2)
//#define ODR_QUAT6 (10 * 16 + 12)
//#define ODR_QUAT9 (10 * 16 + 8)
//#define ODR_PQUAT6 (10 * 16 + 4)
//#define ODR_GEOMAG (10 * 16 + 0)
//#define ODR_PRESSURE (11 * 16 + 12)
//#define ODR_GYRO_CALIBR (11 * 16 + 8)
//#define ODR_CPASS_CALIBR (11 * 16 + 4)
//// sensor output data rate counter
//#define ODR_CNTR_ACCEL (9 * 16 + 14)
//#define ODR_CNTR_GYRO (9 * 16 + 10)
//#define ODR_CNTR_CPASS (9 * 16 + 6)
//#define ODR_CNTR_ALS (9 * 16 + 2)
//#define ODR_CNTR_QUAT6 (8 * 16 + 12)
//#define ODR_CNTR_QUAT9 (8 * 16 + 8)
//#define ODR_CNTR_PQUAT6 (8 * 16 + 4)
//#define ODR_CNTR_GEOMAG (8 * 16 + 0)
//#define ODR_CNTR_PRESSURE (9 * 16 + 12)
//#define ODR_CNTR_GYRO_CALIBR (9 * 16 + 8)
//#define ODR_CNTR_CPASS_CALIBR (9 * 16 + 4)
//// mounting matrix
//#define CPASS_MTX_00 (23 * 16)
//#define CPASS_MTX_01 (23 * 16 + 4)
//#define CPASS_MTX_02 (23 * 16 + 8)
//#define CPASS_MTX_10 (23 * 16 + 12)
//#define CPASS_MTX_11 (24 * 16)
//#define CPASS_MTX_12 (24 * 16 + 4)
//#define CPASS_MTX_20 (24 * 16 + 8)
//#define CPASS_MTX_21 (24 * 16 + 12)
//#define CPASS_MTX_22 (25 * 16)
//// bias calibration
//#define GYRO_BIAS_X (139 * 16 + 4)
//#define GYRO_BIAS_Y (139 * 16 + 8)
//#define GYRO_BIAS_Z (139 * 16 + 12)
//#define ACCEL_BIAS_X (110 * 16 + 4)
//#define ACCEL_BIAS_Y (110 * 16 + 8)
//#define ACCEL_BIAS_Z (110 * 16 + 12)
//#define CPASS_BIAS_X (126 * 16 + 4)
//#define CPASS_BIAS_Y (126 * 16 + 8)
//#define CPASS_BIAS_Z (126 * 16 + 12)
//// Accel FSR
//#define ACC_SCALE (30 * 16 + 0)
//#define ACC_SCALE2 (79 * 16 + 4)
//// pedometer
//#define PEDSTD_STEPCTR (54 * 16)
//#define PEDSTD_TIMECTR (60 * 16 + 4)
//// Activity Recognition
//#define BAC_RATE (48 * 16 + 10)
//// parameters for accel calibration
//#define ACCEL_CAL_RATE (94 * 16 + 4)
//#define ACCEL_ALPHA_VAR (91 * 16)
//#define ACCEL_A_VAR (92 * 16)
//// parameters for compass calibration
//#define CPASS_TIME_BUFFER (112 * 16 + 14)
//// gains
//#define ACCEL_ONLY_GAIN (16 * 16 + 12)
//#define GYRO_SF (19 * 16)
//0x68
//TIM2

//Icm20948Dmp3Driver.c read_mems ACCEL_BIAS_X

// [payload_encrypt] Buffer Size =
// [app_gateway] Accelerometer BIAS_X dmp
// IWDG early warning given, shut down activated.
// PA9 SCAP_ON
// shut down activated at last refresh, EWI given at


/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os2.h"
#include "adc.h"
#include "i2c.h"
#include "icache.h"
#include "iwdg.h"
#include "lptim.h"
#include "memorymap.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "../../Middlewares/nanoprintf/nanoprintf.h"
#include "../../Drivers/SX1280/SX1280.h"
#include "../../App/app_radio_available.h"
#include "../../App/app_gateway.h"
#include "../../App/app_imu.h"
#include "../../App/app_adc.h"
#include "../../App/app_rtc.h"

#include "../../App/app_init.h"
#include "../../App/app_led.h"

#include <string.h>
//#include "../../Callback/timer_Callback.h"
#include "../../App/app_hal_sync.h"
#include "../../App/app_rtc_sync.h"
#include "../../App/app_network_connect.h"
#if !SENSOR_HOST
//#include "../../App/app_node_gnsspps.h"
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define NANOPRINTF_USE_FIELD_WIDTH_FORMAT_SPECIFIERS 1
#define NANOPRINTF_USE_PRECISION_FORMAT_SPECIFIERS 1
#define NANOPRINTF_USE_LARGE_FORMAT_SPECIFIERS 1
#define NANOPRINTF_USE_FLOAT_FORMAT_SPECIFIERS 1
#define NANOPRINTF_USE_BINARY_FORMAT_SPECIFIERS 1
#define NANOPRINTF_USE_WRITEBACK_FORMAT_SPECIFIERS 1



#define PRINTF_APP_MAIN 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//extern osThreadId rangingThreadHandler;
//extern osThreadId initThreadHandler;
//extern osThreadId ledThreadHandler;
//extern osThreadId rtcThreadHandler;
//extern osThreadId rtcSyncThreadHandler;
#if	STM32WBAUSED
extern TaskHandle_t halSyncThreadHandler;
#else
extern osThreadId halSyncThreadHandler;
#endif
//extern osThreadId netwConThreadHandler;

module   moduleTable[MAXNRMODULES]; // table containing a list of all modules in range, in record [0] the info of the own sensor is stored
char     uart_buf[200];
char     byteString[3];
uint8_t  uart_buf_length;
#if	STM32WBAUSED
uint8_t  huart1InUse            = 0;
#else
uint8_t  huart2InUse            = 0;
#endif
uint64_t timeStampInt;
double   halDriftCorrection     = 0; // Is the HAL/RTOS drift in us per ms. This is the output of the HAL pid controller value / 1000 (see app_hal_sync.c)
                                     // to spread the expected HAL pid error output correction over the NEXT second
#if TICKSYNCHRONIZATION
  double   adjustTick           = 0; // If synchronization is based on ticks: if this value is +1, then the incTick is skipped, if this is -1, then extra incTick.
  uint32_t substractedTicks     = 0; // number of Ticks which were subtracted from the Tick count
  uint32_t addedTicks           = 0; // number of Ticks which were added
  uint32_t subtractedIn1s         = 0;
  uint32_t addedIn1s              = 0;
#else
  double   adjustTIM2           = 0; // If synchronization is based on us level: used to adapt the TIM2 counter end value
  uint32_t substracteduSeconds  = 0; // number of us which were subtracted from the Tick counter timer TIM2 end value
  uint32_t addeduSeconds        = 0; // number of us which were added to the Tick counter timer TIM2 end value
  double subtracteduSecondsIn1s = 0;
  double addeduSecondsIn1s      = 0;
  int32_t tim2Table[1000];           // only used for debugging purposes, to store all the Tick counter end values during 1 second
  uint32_t tim2Position         = 0; // only used for debugging purposes, the tim2Table position pointer
  uint32_t TIM2Adapted          = 0;
  int32_t regVal               = 0;
#endif
  uint32_t SysTickCallbackHits  = 0;
  uint32_t subTickCountAtRTCPPS = 0;
  int32_t  subTickCounterRTCPPS = 0;
  uint32_t subTickCurrentValue  = 0;
  uint32_t subTickCounterTIM2   = 0;
  uint32_t newReloadSysTickValue  = 0;
  double   restReloadSysTickValue = 0;


double   compensateTicks        = 0; // = -pidHalError. This is to compensate for the ticks of the FORMER second, so that the absolute value of the tick
                                     // count remains correct. This is done ASAP (per ms 1 tick compensation)
                                     // As this resource is adapted in both main.c and app_hal_sync.c this resource is guarded with CompTicksMutex
                                     // If this value is negative, extra ticks will be given, if positive, extra ticks will be skipped.
                                     // In order to do a correct measurement of how many HAL/RTOS ticks 1 RTCPPS takes, this value needs to be added to
                                     // previousRtosTimeStampRTCPPS if this value is negative, or subtracted if positive.
uint32_t schedulerStartTime     = 0; // to synchronize RTOS time to STM32 time
uint64_t startOfTransmit        = 0; // this is a timestamp just before the radio starts to transmit. This is t1 for a Node and t5 for a Host
uint64_t rtosTimeStampChangeRTC = 0; // RTOS time stamp when changeClock is set to 1
uint64_t rtosTimeStampNotifyRTC = 0; // RTOS time stamp when app_gnss is notifying app_rtc to change the clock
uint64_t rtosTimeStampStopRTC   = 0; // RTOS time stamp when the RTC is being stopped
uint64_t rtosTimeStampStartRTC  = 0; // RTOS time stamp when the RTC is being started
uint64_t rtosTimeStampGPRMC     = 0; // RTOS time stamp when a $GPRMC message is received (see app_gnss.c)
uint64_t rtosTimeStampI2CInUse  = 0; // RTOS time stamp when the i2c bus is becoming in use by the GNSS module
uint64_t rtosTimeStampI2CFree   = 0; // RTOS time stamp when the i2c bus is released by the GNSS module
uint64_t rtosTimeStampBCastStrt = 0; // RTOS time stamp when the broadcast process is starting
uint64_t rtosTimeStampBCastDone = 0; // for HOST: RTOS time stamp when the broadcast message has been sent
                                     // for NODE: RTOS time stamp when the broadcast message has been received
uint64_t rtosTimeStampLedOn     = 0; // RTOS time stamp when the LED is being switched on
uint64_t timeStamp8             = 0; // t8 RTOS time stamp of RxDone (radio level) of message received by node when confirmation pairing was received
uint64_t xEpochLnt8             = 0; // t8  RTC time stamp of RXDone  (CPU level)  of message received by node when confirmation pairing was received
uint64_t timeStamp1             = 0; // t1 = RTOS time at start transmission (CPU level) of node to reply to ranging process
uint64_t xEpochLnt1             = 0; // RTC time stamp t1 at node when answer pairing was sent
uint64_t timeStamp5             = 0; // t5 = RTOS time at start transmission (CPU level) of clock reference (Host)
                                     // t5 is also used when the RTC clock needs to be set again. If the current time - t5 is more than 8s, then do not set the RTC clock.
uint64_t xEpochLht5             = 0; // received RTC time stamp t5 of host when answer pairing was received
uint64_t timeStamp4             = 0; // t4 = RTOS time at RxDone (radio level) of clock reference (Host) on reply of Node
uint64_t xEpochLht6             = 0; // received RTC time stamp t6 of host when confirm pairing was sent
time_t   gnssEpoch              = 0; // HOST: epoch time coming from GNSS module
                                     // NODE: epoch time coming from sensor HOST
time_t   rtc_epoch              = 0; // HOST & NODE: epoch time coming from RTC module, for the HOST this is used to forward the time to another sensor module
time_t   received_epoch         = 0; // received epoch from host
uint64_t receivedxEpoch         = 0; // received extended epoch from host
uint64_t rtosTimeStampRTCPPS    = 0; // protected RTOS time stamp taken each time a new RTC PPS arrives
uint64_t tickCountRTCPPS        = 0;

double sysTickCurrentValue      = 0;
double sysTickCurrentTimeStamp  = 0;
uint32_t rtc_pps                = 0; // counts the number of RTC PPS pulses, is reset when the RTC clock is started
uint64_t firstRTCPPS            = 0; // HAL time stamp for the first RTC PPS
uint64_t durationRTC10PPS       = 0; // duration in RTOS time between first and 10th RTC PPS pulse (is 10,000.000ms + x + x')
uint64_t rtosTimeStampGNSSPPS   = 0; // protected RTOS time stamp taken each time a new GNSS PPS arrives
uint32_t gnss_pps               = 0; // Counts the number of GNSS PPS pulses, is reset when the RTC clock is set
uint32_t duration_100_gnss_pps  = 0; // duration in HAL ms between first and 101st GNSS PPS pulse (is 100,000ms + x)
                                     // GNSS is the reference, 100,000ms, x is drift of HAL after 100s, x' is drift of RTC after 100s
uint32_t halSynchronized        = 0; // if HAL and RTOS are synchronized (5 cycles of max 1ms difference), then halSynchronized = 1
                                     // Only when halSynchronized = 1, the rtc clock can start.
uint32_t rtcSynchronized        = 0; // if RTC_PPS and GNSS_PPS are synchronized (max 2ms difference), then rtcSynchronized = 1
                                     // Only when rtcSynchronized = 1, the app_network_connect can start
uint8_t  rtosClockStopped       = 0; // To go back on the RTOS Tick, we stop the RTOS clock. This flag indicates that the clock stopped (=1)
uint8_t  changeClock            = 0; // if ChangeClock == 1, then the RTC clock can be changed
uint8_t  radioAvailable         = 0; // if the radio is available, radioAvailable = 1.
#if !STM32WBAUSED
uint8_t  setClock               = 0; // if RTC is being set, the GNSS module should not use the I2C bus, as this can prevent the clock to be properly set.
uint8_t  i2cInUseGNSS           = 0; // This will become 0 after setClock has been set to 1 and GNSS cycle(s) has done a notification to app_rtc to change the clock.
#endif
int      rtosTimeDifference     = 0; // This is the RTOS time stamp difference between the GNSS PPS (reference) and RTC PPS (rtosTimeStampGNSSPPS - rtosTimeStampRTCPPS)
                                     // this is used as input for the pid controller
uint32_t  subSecondsCorrection  = 0; // outcome of PID controller to set the subSeconds on the RTC
uint32_t  msCorrection          = 0; // outcome of PID controller to wait to start the clock
uint8_t   notifyOnlyOnceGNSS    = 1; // to make sure that their is only one notification from app_gnss to app_rtc to start the RTC set-up process.
                                     // There is an exception: when the RTC set-up process is postponed because a broad cast message is close to be executed,
                                     // then an extra notification needs to be given.
uint32_t  pairingOngoing        = 0; // If a pairing process is close to or ongoing (this variable = 1), then the RTC may not be changed (no RTCPID), otherwise the offset is not correctly calculated
uint32_t  imuAvailable          = 1; // If the IMU is not responding after 5 attempts, we consider that no IMU is available on board (becomes 0)
uint32_t  agingOffsetPCF2131    = 0;
uint64_t  previousrtosTimeStampGNSSPPS = 0;
uint32_t  rtosDriftToGNSS       = 0;
uint32_t  totalRtosDriftToGNSS  = 0;
uint32_t  gnssEpochAvailable    = 0; // becomes 1 when gnssEpoch is being available from the GNSS module. This is used to start-up the RTC PID controller

uint32_t  printReadMemsIMURegContent = 0; // Set to 1 if the contents of the registers being read needs to be printed with instruction inv_icm20948_read_mems()

uint8_t   dmpBiasFlash[100]     = {0};
uint8_t   otherFlash[64]        = {0};
uint8_t   stopLoadingSuperCap   = 0; // becomes 1 on the EWI of the watchdog and stops supercap to charge further. This is only needed for test purposes, but does not harm the normal working process
uint64_t  timeOfEWDGI           = 0;
uint32_t  LoRaDevAddrInit       = 0; // LoRa Device Address

float     tickSpeedToReference         = 0;

//uint32_t sensorID             = 0; // MyID, 3 bytes
char      receivedSensorID[8]      ; // Received sensorID
BaseType_t    xHigherPriorityTaskWoken = pdFALSE;

RadioStatus_t statusSX1280;
int           yaw                      = 0;
int           pitch                    = 0;
int           roll                     = 0;
int16_t		  quatW        			   = 0;
int16_t		  quatX					   = 0;
int16_t		  quatY        			   = 0;
int16_t		  quatZ					   = 0;
float 		  accX 					   = 0;
float		  accY 					   = 0;
float 	      accZ 					   = 0;
float		  magX 					   = 0;
float		  magY 					   = 0;
float		  magZ 					   = 0;
float		  gyrX 					   = 0;
float		  gyrY 					   = 0;
float		  gyrZ 					   = 0;
float         accBias[3];
float         gyrBias[3];
float         magBias[3];
inv_sensor_config_offset_t accOffset;


uint8_t       pollIMU                  = 0;
int           newImuData               = 0;
TickType_t    ledFreq                  = 2000; //Frequency of the LED: LedFreq =    0 --> Led OFF               --> used when calibration is done or STM32 sleeping
                                               //                      LedFreq = 2000 --> Led 0.2s on, 2.0s off --> used to indicate that Super capacitor is loading
                                               //                      LedFreq = 1500 --> Led 0.2s on, 1.5s off --> used when firmware of IMU is being loaded
                                               //                      LedFreq = 1000 --> Led 0.2s on, 1.0s off --> start of Gyroscope calibration
                                               //                      LedFreq =  500 --> Led 0.2s on, 0.5s off --> used to ask to start calibration of Accelerometer
                                               //                      LedFreq =  200 --> Led 0.2s on, 0.2s off --> Accelerometer calibration done, start calibration of Magnetometer
int 		  calibrationActive        = 1;
uint32_t      imuMeasurementNr         = 0;

ADC_ChannelConfTypeDef configADC       = {0};
uint16_t      installedBatteryVoltage  = 3300; // we start with 3300mV. If V_Core is less than 860mV, then it means that the installed Battery is 3600mV
uint16_t      supplyVoltageDetermined  = 0;    // when the installed battery voltage is determined, this value = 1. Only then, the super cap voltage level is correct.
uint16_t      batteryVoltageLevel      = 0;
uint16_t      batteryInVoltageLevel    = 0;
uint16_t      supercapVoltageLevel     = 0;
uint16_t      vCoreVoltageLevel        = 0;
uint16_t      vCoreTemperature         = 0;
uint64_t      startSuperCapOn          = 0;
uint64_t      superCapOnTime           = 0;
uint32_t      superCapAvailable        = 1;  // in app_supercap a check is being done if a supercap is available on the device
                                             // if superCapAvailable = 0   then no super capacitor is available, or its value is not being measured. To be sure, a fixed loading time will be used.
                                             // if superCapAvailable = 2   a super capacitor is installed on this device and its voltage level is measured
                                             // if superCapAvailable = 100 Radio's are directly powered, not via super capacitor.
uint32_t      radioBusy                = 0;



/* Variables for ADC conversion data */
__IO uint16_t uhADCxConvertedData = VAR_CONVERTED_DATA_INIT_VALUE; /* ADC group regular conversion data */

/* Variables for ADC conversion data computation to physical values */
//uint16_t uhADCxConvertedData_Voltage_mVolt = 0;  /* Value of voltage calculated from ADC conversion data (unit: mV) */

/* Variable to report status of ADC group regular unitary conversion          */
/*  0: ADC group regular unitary conversion is not completed                  */
/*  1: ADC group regular unitary conversion is completed                      */
/*  2: ADC group regular unitary conversion has not been started yet          */
/*     (initial state)                                                        */
//__IO uint8_t ubAdcGrpRegularUnitaryConvStatus = 2; /* Variable set into ADC interruption callback */
// [app_init] Error: battery voltage level is
// [app_adc] Error: Battery Voltage
// A super capacitor is installed on this device and its voltage
// (load time =

inv_device_icm20948_t device_icm20948; // States for ICM20948 device object

//SemaphoreHandle_t imuDataMutex;
SemaphoreHandle_t gnssEpochMutex;
SemaphoreHandle_t recEpochMutex;
SemaphoreHandle_t timeStampIntMutex;
#if !STM32WBAUSED
SemaphoreHandle_t i2cGNSSMutex;
#endif
SemaphoreHandle_t CompTicksMutex;

uint32_t rccCFGRSave;
uint32_t rccCRSave;
uint32_t sleepFlag = 0;

// moved out of SX1280.h:
/*
 * Frequency look up table :
 * To avoid Wifi channels, 40 Bluetooth channels are defined below (they already
 * avoid Wifi common channels) : from 2402 MHz to 2480 MHz, step 2 MHz.
 * User can define channel count for Ranging run, and it is optimized to have
 * several frequencies in the largest band as possible. Also the 40 frequencies
 * are generated by random sorting to preferate the 10 first in the largest band
 * as possible (10 is the shortest channel count the user can choose).
 */
//const uint32_t Channelf[] =
//{
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
// 2472000000,
//};

//const uint32_t Channelf[] =
//{
// 2450000000,
// 2476000000,
// 2436000000,
// 2430000000,
// 2468000000,
// 2458000000,
// 2416000000,
// 2424000000,
// 2478000000,
// 2456000000,
// 2448000000,
// 2462000000,
// 2472000000,
// 2432000000,
// 2446000000,
// 2422000000,
// 2442000000,
// 2460000000,
// 2474000000,
// 2414000000,
// 2464000000,
// 2454000000,
// 2444000000,
// 2404000000,
// 2434000000,
// 2410000000,
// 2408000000,
// 2440000000,
// 2452000000,
// 2480000000,
// 2426000000,
// 2428000000,
// 2466000000,
// 2418000000,
// 2412000000,
// 2406000000,
// 2470000000,
// 2438000000,
// 2420000000,
// 2402000000,
//};

// channels in the sequence of the BLE channel number 0...39 (the last three channels are the BLE advertising channels)
const uint32_t Channelf[] =
{
2403000000, //2404000000,
2479000000, //2406000000,
2425000000, //2408000000, rx1
2423000000, //2410000000, rx2
2412000000,
2414000000,
2416000000,
2418000000,
2420000000,
2422000000,
2424000000,
2428000000,
2430000000,
2432000000,
2434000000,
2436000000,
2438000000,
2440000000,
2442000000,
2444000000,
2446000000,
2448000000,
2450000000,
2452000000,
2454000000,
2456000000,
2458000000,
2460000000,
2462000000,
2464000000,
2466000000,
2468000000,
2470000000,
2472000000,
2474000000,
2476000000,
2478000000,
2402000000,
2426000000,
2480000000,
};

//Ranging raw factor                 SF5     SF6     SF7     SF8     SF9     SF10
const uint16_t RNG_CALIB_0400[] = { 10299,  10271,  10244,  10242,  10230,  10246  };
const uint16_t RNG_CALIB_0800[] = { 11486,  11474,  11453,  11426,  11417,  11401  };
const uint16_t RNG_CALIB_1600[] = { 13308,  13493,  13528,  13515,  13430,  13376  };
const double   RNG_FGRAD_0400[] = { -0.148, -0.214, -0.419, -0.853, -1.686, -3.423 };
const double   RNG_FGRAD_0800[] = { -0.041, -0.811, -0.218, -0.429, -0.853, -1.737 };
const double   RNG_FGRAD_1600[] = { 0.103,  -0.041, -0.101, -0.211, -0.424, -0.87  };



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the System Power */
  SystemPower_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init(); // Make sure that SCAP_ON_Pin is SET, the STM32 will not start up when the supercap is not loaded until a voltage level of
                  // roughly 700mV if an external XTAL is used. This behavior is different when the internal clock is being used!!!
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_ICACHE_Init();
  MX_LPTIM1_Init();
  MX_LPTIM2_Init();
  MX_ADC4_Init();
  MX_TIM2_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  //USART1->CR1 &= (0x0000); // Set OVER16 bit 0x00008000


//20240405  removed on Init scheduler: osKernelInitialize();
//20240405  removed osKernelStart(); and replaced it with vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Init scheduler */
//  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
//  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  vTaskStartScheduler();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEDiv = RCC_HSE_DIV2;
  RCC_OscInitStruct.LSIState = RCC_LSI1_ON;
  RCC_OscInitStruct.LSIDiv = RCC_LSI_DIV128;
  RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK7|RCC_CLOCKTYPE_HCLK5;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB7CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHB5_PLL1_CLKDivider = RCC_SYSCLK_PLL1_DIV1;
  RCC_ClkInitStruct.AHB5_HSEHSI_CLKDivider = RCC_SYSCLK_HSEHSI_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{
  /* WKUP_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(WKUP_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(WKUP_IRQn);
/* USER CODE BEGIN PWR */
/* USER CODE END PWR */
}

/* USER CODE BEGIN 4 */
void waitToPrint(void)
{
#if	STM32WBAUSED
  while (huart1InUse)
  {
    vTaskDelay(4);
  }
#else
  while (huart2InUse)
  {
    osDelay(4);
  }
#endif
}

void huart2print(char buf[], uint8_t length)
{
#if	STM32WBAUSED
  huart1InUse = 1;
  HAL_UART_Transmit(&huart1, (uint8_t *)buf, length, 100);
  vTaskDelay(2);
  huart1InUse = 0;
#else
  huart2InUse = 1;
  HAL_UART_Transmit(&huart2, (uint8_t *)buf, length, 100);
  oskDelay(2);
  huart2InUse = 0;
#endif
}


#if  PCF2131I2C
void IsI2C1Available(void)
{
  uint32_t whileIterations = 0; // to prevent lock into while loop
  uint32_t onlyOnce        = 1;
  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
  {
    if (onlyOnce)
    {
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [IsI2C1Available] Waiting for I2C1 to become available.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
      onlyOnce = 0;
    }
	vTaskDelay(2);
    if(whileIterations++ > 100)
    {
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [IsI2C1Available] Error: Jump out of while loop after 100 iterations.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
      return;
    }
  }
}
#else
void IsSPI3Available(void)
{
  uint32_t whileIterations = 0; // to prevent lock into while loop
  uint32_t onlyOnce        = 1;
  while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY)
  {
    if (onlyOnce)
    {
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [IsSPIAvailable] Waiting for SPI3 to become available.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
      onlyOnce = 0;
    }
    vTaskDelay(2);
    if(whileIterations++ > 100)
    {
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [IsSPI3Available] Error: Jump out of while loop after 100 iterations.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
      return;
    }
  }
}
#endif

void IsI2CAvailable(void)
{
  uint32_t whileIterations = 0; // to prevent lock into while loop
  uint32_t onlyOnce        = 1;
#if STM32WBAUSED
  while (HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY)
  {
    if (onlyOnce)
    {
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [IsI2CAvailable] Waiting for I2C3 to become available.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
      onlyOnce = 0;
    }
	vTaskDelay(2);
    if(whileIterations++ > 100)
    {
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [IsI2CAvailable] Error: Jump out of while loop after 100 iterations.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
      return;
    }
  }
#else
  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
  {
	vTaskDelay(2);
    if (onlyOnce)
    {
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [IsI2CAvailable] Waiting for I2C1 to become available.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
      onlyOnce = 0;
    }
    if(whileIterations++ > 100)
    {
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [IsI2CAvailable] Error: Jump out of while loop after 100 iterations.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
      return;
    }
  }
#endif
}

//void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
//{
//	if (sleepFlag == 2)
//		{
//	//	RCC->CR = rccCRSave;
//	//	RCC->CFGR1 = rccCFGRSave;
//	//	SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
//
//	    HAL_ResumeTick();
//
//	//    vTaskStepTick((TickType_t) 10000);
//
//
//	    sleepFlag = 1;
//	#if PRINTF_APP_MAIN
//	    waitToPrint();
//	    npf_snprintf(uart_buf, 200, "%u [app_imu] STM32 waking up!\r\n",(unsigned int) xTaskGetTickCountFromISR());
//	    huart2print(uart_buf, strlen(uart_buf));
//	    npf_snprintf(uart_buf, 200, ".\r\n");
//	#endif
//
//		}
//}
//void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
//{
//	if (sleepFlag == 2)
//	{
//	RCC->CR = rccCRSave;
//	RCC->CFGR1 = rccCFGRSave;
//	SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
//
//    HAL_ResumeTick();
//
////    vTaskStepTick((TickType_t) 10000);
//
//
//    sleepFlag = 1;
//#if PRINTF_APP_MAIN
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "%u [app_imu] STM32 waking up!\r\n",(unsigned int) xTaskGetTickCountFromISR());
//    huart2print(uart_buf, strlen(uart_buf));
//    npf_snprintf(uart_buf, 200, ".\r\n");
//#endif
//
//    NVIC_SystemReset();
//
//	}
//}

/**
  * @brief  Conversion transfer complete callback
  * @note   This function is executed when the transfer complete interrupt
  *         is generated
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  /* Update status variable of ADC unitary conversion                     */
//  ubAdcGrpRegularUnitaryConvStatus = 1;
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  xHigherPriorityTaskWoken = pdFALSE;
  AdcNotifyFromISR(NOTIFICATION_FROM_ADC);
}

//void i2c2read(char buf[],uint8_t length)
//{
//  i2cInUse = 1;
//
//}

/*!
 * @brief           Capture the system time in microseconds
 *
 * @return          system_current_time    current system timestamp in microseconds
 */
uint64_t get_timestamp_us()
{
  static uint64_t microseconds_since_start = 0;
////  microseconds_since_start += (uint64_t)(uwTick * 1000 - (uint32_t)(microseconds_since_start) + *((uint32_t *)0x40001424));
//  microseconds_since_start += (uint64_t)(uwTick * 1000 - (uint32_t)(microseconds_since_start) + TIM2->CNT);
  microseconds_since_start += (uint64_t)((uwTick - (uint32_t)(microseconds_since_start / 1000)) * 1000 + TIM2->CNT);
  return microseconds_since_start;
//  return (uint64_t)(uwTick * 1000 + TIM2->CNT);
}

void HAL_IWDG_EarlyWakeupCallback(IWDG_HandleTypeDef *hiwdg)
{
  timeOfEWDGI = (uint64_t) xTaskGetTickCountFromISR();
  HAL_GPIO_WritePin(SCAP_ON_GPIO_Port, SCAP_ON_Pin, GPIO_PIN_RESET);   // Supercap off
  ledFreq = 0; // led off
  stopLoadingSuperCap = 1;
  otherFlash[0] = 0xDA;
  otherFlash[1] = 0xDA;
  FLASH_EraseInitTypeDef flashEraseData;
  flashEraseData.TypeErase = FLASH_TYPEERASE_PAGES;
  flashEraseData.Page = 126;
  flashEraseData.NbPages = 1;
  uint32_t flashErasePageError = 0;
  // write value(s) to this location:
  HAL_FLASH_Unlock();
  HAL_FLASHEx_Erase(&flashEraseData, &flashErasePageError);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, 0x080FC000, (uint32_t) &otherFlash[0]); // QUADWORD = 16 bytes
  HAL_FLASH_Lock();

//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "%u [main] Early wake up notice.\r\n", (unsigned int) xTaskGetTickCount());
//      huart2print(uart_buf, strlen(uart_buf));


}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t n)
{// only RTC_PPS_Pin has falling callback, no need to check...
//  xSemaphoreTakeFromISR(timeStampIntMutex, &xHigherPriorityTaskWoken);
  //timeStampInt = xTaskGetTickCountFromISR();
  //rtosTimeStampRTCPPS = (uint64_t)(uwTick * 1000 + TIM2->CNT);


//  __disable_irq();


  subTickCounterTIM2 = TIM2->CNT; // number of counts during 1 RTCPPS to determine the mean value of the frequency during the last RTCPPS
  TIM2->CNT = 0; // could be moved to hal_sync, to reset after 10PPS, not every RTCPPS
  subTickCurrentValue = (uint32_t) SYSTICK_CURRENT_VALUE_REG;
//  SYSTICK_CURRENT_VALUE_REG = 0; // to align the start of the SysTick with the start of a new RTCPPS
  tickCountRTCPPS = xTaskGetTickCountFromISR();
  sysTickCurrentValue = (double) (SYSTICK_LOAD_REG - subTickCurrentValue); // the sysTickCounter counts down!
//  HAL_TIM_Base_Start(&htim2); // moved to ProjectInit()
//  subTickCountAtRTCPPS = (uint32_t) subTickCounterRTCPPS + (int32_t) SYSTICK_LOAD_REG - subTickCurrentValue;
//  subTickCounterRTCPPS = (int32_t) subTickCurrentValue - (int32_t) SYSTICK_LOAD_REG - (int32_t) 1;

//  __enable_irq();


//  rtosTimeStampRTCPPS = tickCountRTCPPS * 1000000 + (uint64_t)(sysTickCurrentValue * 62.5);
//  rtosTimeStampRTCPPS = xTaskGetTickCountFromISR() * 1000000 + (uint64_t)((double)SYSTICK_CURRENT_VALUE_REG * 62.5);
//rtosTimeStampRTCPPS = HAL_GetTick() * 1000 + (uint64_t)TIM2->CNT;
//  HAL_GetTick();
//  xTickCount =;
//  TickType_t;
//  epoch;
//  xSemaphoreGiveFromISR(timeStampIntMutex, &xHigherPriorityTaskWoken);
//  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//  if (n == RTC_PPS_Pin)
//  { // RTC_NINTA_Pin is available for both HOST and NODE
    app_hal_sync_notify_fromISR(NOTIFICATION_FROM_RTC_PPS);
//    if (halSyncThreadHandler != NULL)
//    { // synchronization of HAL is done by the RTC PPS signal
//      vTaskNotifyGiveFromISR(halSyncThreadHandler, &xHigherPriorityTaskWoken);
//      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//    }
//    app_rtc_notify_fromISR(NOTIFICATION_FROM_RTC_PPS_START_RTC);
//  }
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t n)
{
//  UBaseType_t uxSavedInterruptState;
  xSemaphoreTakeFromISR(timeStampIntMutex, &xHigherPriorityTaskWoken);
//  timeStampInt = xTaskGetTickCountFromISR();
//  timeStampInt = uwTick * 1000 + (uint64_t)TIM2->CNT;
//  timeStampInt = xTaskGetTickCountFromISR() * 1000 + (uint64_t)TIM2->CNT;
//  uxSavedInterruptState = portSET_INTERRUPT_MASK_FROM_ISR();

  sysTickCurrentTimeStamp = (double)(SYSTICK_CURRENT_VALUE_REG);
  timeStampInt = xTaskGetTickCountFromISR();

//  portCLEAR_INTERRUPT_MASK_FROM_ISR( uxSavedInterruptState );
  xSemaphoreGiveFromISR(timeStampIntMutex, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  //  xHigherPriorityTaskWoken = pdFALSE;
  if (n == INT1_ICM20948_Pin)
  {
	// 20240616:
    //ImuNotifyFromISR(NOTIFICATION_FROM_IMU);
	GatewayNotifyFromISR(NOTIFICATION_FROM_IMU);
  }
#if !IMUTEST // no RTC/GNSS function for IMU test purposes
#if SENSOR_HOST
  if (n == GNSS_PPS_Pin)
  { // GNSS_PPS_Pin is only available at a HOST
  //    if (rtcThreadHandler != NULL)
  //    { // this is used to synchronize the clock start-up, but this can only be used on a HOST. RTC_PPS can NOT be used as the clock is being
  //      // stopped when a new time/date is set. Another solution is needed (like timer from STM32)
  //      vTaskNotifyGiveFromISR(rtcThreadHandler, &xHigherPriorityTaskWoken);
  //      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  //    }
  //    HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
//	if (changeClock)
//	{ // give only a notification to app_rtc when change clock = 1.
//	  app_rtc_notify_fromISR(NOTIFICATION_FROM_GNSS_PPS);
//	}
    app_rtc_sync_notify_fromISR(NOTIFICATION_FROM_GNSS_PPS);
//    if (rtcSyncThreadHandler != NULL)
//    { // the GNSSPPS pin is only available at the HOST, this is used to synchronize the RTC clock with the GNSS PPS
//      vTaskNotifyGiveFromISR(rtcSyncThreadHandler, &xHigherPriorityTaskWoken);
//      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//    }


  }
#else
//  if (n == GNSS_PPS_Pin)
//  { // PPS_Pin is only available at a NODE for test purposes, see app_node_gnsspps.c
////20240405      app_gnsspps_notify_fromISR(NOTIFICATION_FROM_HOST_GNSS_PPS);
//  }
#endif

//  if (n == RTC_PPS_Pin)
//  { // RTC_NINTA_Pin is available for both HOST and NODE
//    app_hal_sync_notify_fromISR(NOTIFICATION_FROM_RTC_PPS);
////    if (halSyncThreadHandler != NULL)
////    { // synchronization of HAL is done by the RTC PPS signal
////      vTaskNotifyGiveFromISR(halSyncThreadHandler, &xHigherPriorityTaskWoken);
////      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
////    }
////    app_rtc_notify_fromISR(NOTIFICATION_FROM_RTC_PPS_START_RTC);
//  }
#endif // !IMUTEST
  if (n == DIO1_SX1280_Pin)
  {
#if PLANTSENSOR
    GatewayNotifyFromISR(NOTIFICATION_FROM_DIO1);
#else
    app_netw_con_notify_fromISR(NOTIFICATION_FROM_DIO1);
#endif
    //    waitToPrint();
    //    npf_snprintf(uart_buf, 200, "%u [HAL_GPIO_EXTI_Callback] DIO1_SX1280_Pin interrupt occurred.\r\n",(unsigned int) xTaskGetTickCount());
    //    huart2print(uart_buf, strlen(uart_buf));
  }
  if (n == DIO2_SX1280_Pin)
  {
#if PLANTSENSOR
    GatewayNotifyFromISR(NOTIFICATION_FROM_DIO2);
#endif
    //    waitToPrint();
    //    npf_snprintf(uart_buf, 200, "%u [HAL_GPIO_EXTI_Callback] DIO1_SX1280_Pin interrupt occurred.\r\n",(unsigned int) xTaskGetTickCount());
    //    huart2print(uart_buf, strlen(uart_buf));
  }
  if (n == BUSY_SX1280_Pin)
  {
    RadioAvailableNotifyFromISR(NOTIFICATION_FROM_RADIO_AVAILABLE);
  }

}



//void HAL_SYSTICK_Callback(void)
//{
////	SysTickCallbackHits++;
//    if (TIM2Adapted)
//    {
////      regVal = 999;
////      WRITE_REG(TIM2->ARR, regVal);
//      SYSTICK_LOAD_REG = 15999U;
//      TIM2Adapted = 0;
//    }
//    adjustTIM2 += halDriftCorrection;
//    adjustTIM2 += compensateTicks;
//
//    if(adjustTIM2 >= 62.5)
//    {
////	  addeduSecondsIn1s += adjustTIM2;
////      regVal = 998;
////      WRITE_REG(TIM2->ARR, regVal);
//      regVal = (uint32_t)(adjustTIM2 / 62.5);
//      SYSTICK_LOAD_REG = 15999U - regVal;
//      TIM2Adapted = 1;
//      adjustTIM2 -= 62.5 * (double)regVal;
//    }
//    else
//    {
//	  if(adjustTIM2 < -62.5)
//      {
////		subtracteduSecondsIn1s += adjustTIM2;
////        regVal = 1000;
////        WRITE_REG(TIM2->ARR, regVal);
//		regVal = (uint32_t)(adjustTIM2 / 62.5);
//        SYSTICK_LOAD_REG = 15999U + regVal;
//        TIM2Adapted = 1;
//        adjustTIM2 += 62.5 * (double)regVal;
//      }
//    }
//   // tim2Table[tim2Position] = (uint32_t) READ_REG(TIM2->ARR);
////    tim2Table[tim2Position] = (uint32_t) SYSTICK_LOAD_REG;
////    if (tim2Position++ == 100)
////    {
////      tim2Position = 0;
////    }
//
//}

///**
//  * @brief  Period elapsed callback in non blocking mode
//  * @note   This function is called  when TIM2 interrupt took place, inside
//  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
//  * a global variable "uwTick" used as application time base.
//  * @param  htim : TIM handle
//  * @retval None
//  */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//
//  /* USER CODE BEGIN Callback 0 */
//  if (htim->Instance == TIM2)
//  {
//#if TICKSYNCHRONIZATION
//    //Old code to adjust on tick level (ms level)
//    if (rtosClockStopped)
//    { // 1ms passed since the RTOS Clock was stopped, start the RTOS Clock again
//      SYSTICK_CTRL_REG |= SYSTICK_ENABLE;
//      rtosClockStopped = 0;
//      substractedTicks++;
//    }
//    adjustTick += halDriftCorrection;
//    adjustTick += compensateTicks;
//    compensateTicks = 0;
//    if (adjustTick < 1)
//    { // do a normal HAL_IncTick() (this is overriding the HAL_IncTick() process which is automatically generated)
//      HAL_IncTick();
//      if (adjustTick <= -1)
//      { // do an extra HAL_IncTick() to compensate for the drift
//        addedTicks++;
//#else
//#endif
//  // the code between below two comment lines is automatically generated but will now ONLY be executed when AdjustTick < -1
//  /* USER CODE END Callback 0 */
//  if (htim->Instance == TIM2) {
//    HAL_IncTick();
//  }
//  /* USER CODE BEGIN Callback 1 */
//#if TICKSYNCHRONIZATION
//        xTaskCatchUpTicks((TickType_t) 1U); // jump 1 tick forward in RTOS as well!
//        adjustTick++;
//      }
//    }
//    else
//    { // AdjustTick >= 1: in this case NO HAL_IncTick() may be given to compensate for the drift
//      // here we need to freeze the RTOS tick as well during 1ms!
//      // Stop the RTOS SysTick, this will be started again at the next TIM2 instance
//      SYSTICK_CTRL_REG &= ~SYSTICK_ENABLE;
//      rtosClockStopped = 1;
//      adjustTick--;
//    }
//  }
//#else
//  //20240812 new code based on adjustment of TIM2 count end value, so on us level:
//    if (TIM2Adapted)
//    {
////      regVal = 999;
////      WRITE_REG(TIM2->ARR, regVal);
//      SYSTICK_LOAD_REG = 15999U;
//      TIM2Adapted = 0;
//    }
//    adjustTIM2 += halDriftCorrection;
//    adjustTIM2 += compensateTicks;
//
//    if(adjustTIM2 >= 62.5)
//    {
////	  addeduSecondsIn1s += adjustTIM2;
////      regVal = 998;
////      WRITE_REG(TIM2->ARR, regVal);
//      regVal = (uint32_t)(adjustTIM2 / 62.5);
//      SYSTICK_LOAD_REG = 15999U - regVal;
//      TIM2Adapted = 1;
//      adjustTIM2 -= 62.5 * (double)regVal;
//    }
//    else
//    {
//	  if(adjustTIM2 < -62.5)
//      {
////		subtracteduSecondsIn1s += adjustTIM2;
////        regVal = 1000;
////        WRITE_REG(TIM2->ARR, regVal);
//		regVal = (uint32_t)(adjustTIM2 / 62.5);
//        SYSTICK_LOAD_REG = 15999U + regVal;
//        TIM2Adapted = 1;
//        adjustTIM2 += 62.5 * (double)regVal;
//      }
//    }
//   // tim2Table[tim2Position] = (uint32_t) READ_REG(TIM2->ARR);
//    tim2Table[tim2Position] = (uint32_t) SYSTICK_LOAD_REG;
//    if (tim2Position++ == 100)
//    {
//      tim2Position = 0;
//    }
//  }
//#endif
//  /* USER CODE END Callback 1 */
//}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();

  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [main] Error: entering Error handler infinite loop.\r\n",(unsigned int) xTaskGetTickCount());
  huart2print(uart_buf, strlen(uart_buf));
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
