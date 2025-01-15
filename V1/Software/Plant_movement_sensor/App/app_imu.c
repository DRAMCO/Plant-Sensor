/*
 * app_imu.c
 *
 *  Created on: Nov 1, 2023
 *      Author: Sarah Goossens
 */


#include "app_imu.h"
#include "usart.h"
#include <string.h>
//#include "ICM20948.h"
#include "idd_io_hal.h"
#include "lptim.h"
#include "../../Drivers/Invn/Devices/HostSerif.h"
#include "../../Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Defs.h"
#include "../../Drivers/Invn/Devices/Drivers/Icm20948/Icm20948MPUFifoControl.h"
#include "../../Drivers/Invn/Devices/Drivers/Icm20948/Icm20948DataBaseDriver.h"
#include "app_gateway.h"
#include "app_led.h"
#include "app_init.h"

#define PRINTF_APP_IMU 1
#define CALIBRATION    0
//SPI write not successful
/*
 * Set O/1 to start the following sensors in this example
 * NB: In case you are using IddWrapper (USE_IDDWRAPPER = 1), the following compile switch will have no effect.
 */
#define USE_RAW_ACC 0
#define USE_RAW_GYR 0
#define USE_GRV     0
#define USE_CAL_ACC 0
#define USE_CAL_GYR 0
#define USE_CAL_MAG 0
#define USE_UCAL_GYR 0
#define USE_UCAL_MAG 0
#define USE_RV      1    /* requires COMPASS*/ //quat 9 axis
#define USE_GEORV   0   /* requires COMPASS*/
#define USE_ORI     0    /* requires COMPASS*/ //euler angles
#define USE_STEPC   0
#define USE_STEPD   0
#define USE_SMD     0
#define USE_BAC     0
#define USE_TILT    0
#define USE_PICKUP  0
#define USE_GRA     0
#define USE_LINACC  0
#define USE_B2S     0

#define EXPECTED_WHOAMI 0xEA // WHOAMI value for ICM20948



#define FIXED_POINT_FRACTIONAL_BITS_EULER       16
#define FIXED_POINT_FRACTIONAL_BITS_QUAT        30
#define FIXED_POINT_FRACTIONAL_BITS             0

//fsr

/*
 * Sensor to start in this example
 */

static const struct {
	uint8_t  type;
	uint32_t period_us;
} sensor_list[] = {
#if USE_RAW_ACC
	{ INV_SENSOR_TYPE_RAW_ACCELEROMETER, 50000 /* 20 Hz */ },
#endif
#if USE_RAW_GYR
	{ INV_SENSOR_TYPE_RAW_GYROSCOPE,     50000 /* 20 Hz */ },
#endif
#if USE_CAL_ACC
	{ INV_SENSOR_TYPE_ACCELEROMETER, 50000 /* 20 Hz */ },
#endif
#if USE_CAL_GYR
	{ INV_SENSOR_TYPE_GYROSCOPE, 50000 /* 20 Hz */ },
#endif
#if USE_CAL_MAG
	{ INV_SENSOR_TYPE_MAGNETOMETER, 50000 /* 20 Hz */ },
#endif
#if USE_UCAL_GYR
	{ INV_SENSOR_TYPE_UNCAL_GYROSCOPE, 50000 /* 20 Hz */ },
#endif
#if USE_UCAL_MAG
	{ INV_SENSOR_TYPE_UNCAL_MAGNETOMETER, 50000 /* 20 Hz */ },
#endif
#if USE_GRV
	{ INV_SENSOR_TYPE_GAME_ROTATION_VECTOR, 50000 /* 20 Hz */ },
#endif
#if USE_RV
	{ INV_SENSOR_TYPE_ROTATION_VECTOR, 50000 /* 20 Hz */ },
#endif
#if USE_GEORV
	{ INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR, 50000 /* 20 Hz */ },
#endif
#if USE_ORI
	{ INV_SENSOR_TYPE_ORIENTATION, 50000 /* 20 Hz */ },
#endif
#if USE_STEPC
	{ INV_SENSOR_TYPE_STEP_COUNTER, ODR_NONE },
#endif
#if USE_STEPD
	{ INV_SENSOR_TYPE_STEP_DETECTOR, ODR_NONE},
#endif
#if USE_SMD
	{ INV_SENSOR_TYPE_SMD, ODR_NONE},
#endif
#if USE_BAC
	{ INV_SENSOR_TYPE_BAC, ODR_NONE},
#endif
#if USE_TILT
	{ INV_SENSOR_TYPE_TILT_DETECTOR, ODR_NONE},
#endif
#if USE_PICKUP
	{ INV_SENSOR_TYPE_PICK_UP_GESTURE, ODR_NONE},
#endif
#if USE_GRA
	{ INV_SENSOR_TYPE_GRAVITY, 50000 /* 20 Hz */},
#endif
#if USE_LINACC
	{ INV_SENSOR_TYPE_LINEAR_ACCELERATION, 50000 /* 20 Hz */},
#endif
#if USE_B2S
	{ INV_SENSOR_TYPE_B2S, ODR_NONE},
#endif
};


/* Forward declaration */
static void check_rc(int rc);
void inv_icm20948_sleep(int us);
//uint64_t inv_icm20948_get_time_us(void);
static void sensor_event_cb(const inv_sensor_event_t * event, void * arg);

void ext_interrupt_cb(void * context, int int_num);

//ICM_20948_REG_PWR_MGMT_1

uint64_t inv_icm20948_get_dataready_interrupt_time_us(void);

//static void msg_printer(int level, const char * str, va_list ap);

extern char     uart_buf[200];
uint32_t   notificationValue = 0;      // Used to identify where the notification is coming from.

TaskHandle_t imuThreadHandler;

extern int      pitch;
extern int      yaw;
extern int      roll;

int pitch0 = 0;
int yaw0   = 0;
int roll0  = 0;

extern int16_t  quatW;
extern int16_t	quatX;
extern int16_t	quatY;
extern int16_t	quatZ;

extern float accX;
extern float accY;
extern float accZ;
extern float magX;
extern float magY;
extern float magZ;
extern float gyrX;
extern float gyrY;
extern float gyrZ;

extern int      newImuData;
//extern SemaphoreHandle_t imuDataMutex;

extern uint32_t rccCFGRSave;
extern uint32_t rccCRSave;
extern uint32_t sleepFlag;
extern TickType_t ledFreq;

extern uint8_t pollIMU;

extern uint32_t imuMeasurementNr;

extern uint32_t imuAvailable;

extern uint8_t   dmpBiasFlash[100];
extern uint8_t   otherFlash[64];



int gyroAccuracy = 0;
int accelAccuracy = 0;
int magAccuracy = 0;

int gyroCalibrated = 0;
int accelCalibrated = 0;
int magCalibrated = 0;

int startGateway = 0;

extern int calibrationActive;
//56

static volatile int irq_from_device; // Flag set from device irq handler
//static const uint8_t EXPECTED_WHOAMI[] = { 0xEA }; // WHOAMI value for ICM20948

static const uint8_t dmp3_image[] =
{ // ICM20948 device requires a DMP image to be loaded on init: provide such images by means of a byte array
#include "../../Drivers/Invn/Images/icm20948_img.dmp3a.h"
};

extern inv_device_icm20948_t device_icm20948;
//static inv_device_icm20948_t device_icm20948;

extern float         accBias[3];
extern float         gyrBias[3];
extern float         magBias[3];
extern inv_sensor_config_offset_t accOffset;


static inv_device_t * device; // Just a handy variable to keep the handle to device object
static const inv_sensor_listener_t sensor_listener =
{ // A listener object to handle sensor events
  sensor_event_cb, /* callback that will receive sensor events */
  0                /* some pointer passed to the callback */
};
static volatile uint32_t last_irq_time = 0; // Last time at which ICM20948 IRQ was fired

imu_quat_t quat;

// to calculate the mean value of the measurements:
imu_quat_t imuMeasQuatArray[100];
int64_t quatW64b;
int64_t	quatX64b;
int64_t	quatY64b;
int64_t	quatZ64b;

void ImuThreadInit()
{
  if (xTaskCreate ((TaskFunction_t)ImuThreadStart, "ImuThread", (configSTACK_DEPTH_TYPE)512, NULL, osPriorityHigh, &imuThreadHandler) != pdPASS)
  {
#if PRINTF_APP_IMU
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_imu] IMU thread Could not be started.\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
}

void ImuThreadStart(const void * params)
{
//  uint32_t   notificationValue = 0;                     // Used to identify where the notification is coming from.
#if PRINTF_APP_IMU
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_imu] [ImuThread] Started.\n",(unsigned int) xTaskGetTickCount());
  huart2print(uart_buf, strlen(uart_buf));
#endif
  ledFreq = 1500; // led off time 1500ms, on time 200ms
  int rc = 0;
  unsigned i = 0;
  uint8_t whoami = 0xFF;
  uint32_t whileIterations = 0; // to prevent lock into while loop
  uint64_t available_sensor_mask; /* To keep track of available sensors*/

  //rc += inv_host_serif_open(idd_io_hal_get_serif_instance_spi());

/*
 * Create icm20948 Device
 * Pass to the driver:
 * - reference to serial interface object,
 * - reference to listener that will catch sensor events,
 * - a static buffer for the driver to use as a temporary buffer
 * - various driver option
 */

  select_userbank(ub_0);
  inv_device_icm20948_init(&device_icm20948, idd_io_hal_get_serif_instance_spi(), &sensor_listener, dmp3_image, sizeof(dmp3_image));

  vTaskDelay(100U);

  device = inv_device_icm20948_get_base(&device_icm20948); // Simply get generic device handle from icm20948 Device

  vTaskDelay(100U);

  /* Check WHOAMI */
  if((rc = inv_device_icm20948_whoami(device, &whoami)) != 0)
  {
    waitToPrint();
    npf_snprintf(uart_buf, 100, "Error %d when reading WHOAMI value\r\n", rc);
    huart2print(uart_buf, strlen(uart_buf));
    return rc;
  }
  if(whoami == 0 || whoami == 0xFF)
  {
    waitToPrint();
    npf_snprintf(uart_buf, 100, "Unexpected WHOAMI value 0x%X. Aborting setup.\r\n", whoami);
    huart2print(uart_buf, strlen(uart_buf));
    return INV_ERROR;
  }
  else
  {
    waitToPrint();
    npf_snprintf(uart_buf, 100, "WHOAMI value: 0x%X.\r\n", whoami);
    huart2print(uart_buf, strlen(uart_buf));
  }

#if PRINTF_APP_IMU
  waitToPrint();
  npf_snprintf(uart_buf, 100, "Setting-up ICM device with lower driver.\r\n");
  huart2print(uart_buf, strlen(uart_buf));
#endif

  while (HAL_SPI_GetState(ICM20948_SPI) != HAL_SPI_STATE_READY)
  {
    vTaskDelay(10);
    waitToPrint();
    npf_snprintf(uart_buf, 100, "[app_imu] IMU SPI busy before setting up device.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
  }

  rc = inv_device_icm20948_setup(&device_icm20948);

  while (HAL_SPI_GetState(ICM20948_SPI) != HAL_SPI_STATE_READY)
  {
    vTaskDelay(10);
    waitToPrint();
    npf_snprintf(uart_buf, 100, "[app_imu] IMU SPI busy after setting up device.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
  }


  while (rc == -1)
  {
    vTaskDelay(1000U);
    inv_device_icm20948_reset(&device_icm20948);

    reset_DMP(&device_icm20948);
    vTaskDelay(1000U);

    waitToPrint();
    npf_snprintf(uart_buf, 100, "[app_imu] IMU failed to set-up, try again.\r\n");
    if(whileIterations++ > 4)
    {
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_imu] Error: Jump out of while loop to set-up IMU. Continue without IMU on board.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
      imuAvailable = 0;
      rc = 0;
    }
    if (imuAvailable)
    {
      huart2print(uart_buf, strlen(uart_buf));
      while (HAL_SPI_GetState(ICM20948_SPI) != HAL_SPI_STATE_READY)
      {
        vTaskDelay(10);
        waitToPrint();
        npf_snprintf(uart_buf, 100, "[app_imu] IMU SPI busy before setting up device.\r\n");
        huart2print(uart_buf, strlen(uart_buf));
      }
      rc = inv_device_icm20948_setup(&device_icm20948);
      while (HAL_SPI_GetState(ICM20948_SPI) != HAL_SPI_STATE_READY)
      {
        vTaskDelay(10);
        waitToPrint();
        npf_snprintf(uart_buf, 100, "[app_imu] IMU SPI busy after setting up device.\r\n");
        huart2print(uart_buf, strlen(uart_buf));
      }
    }
  }

  if (imuAvailable)
  {
    check_rc(rc);
    /*
     * Now that Icm20948 device was inialized, we can proceed with DMP image loading
     * This step is mandatory as DMP image is not stored in non volatile memory
     */
    waitToPrint();
    npf_snprintf(uart_buf, 100, "**********************************************************************\r\n");
    huart2print(uart_buf, strlen(uart_buf));

//todo
//	// Set Gryo and Accel FSR (set in imu.h settings)
//	// imu_config_fsr_gyro_accel(NOMADE_GRYO_FSR, NOMADE_ACCEL_FSR);
//
//	imu_config_fsr_gyro(NOMADE_GRYO_FSR);
//	imu_config_fsr_accel(NOMADE_ACCEL_FSR);
//
//	// Apply stored IMU offsets from flash
//	apply_stored_offsets();

    reset_DMP(&device_icm20948);

#if CALIBRATION
    // Set to full scale range
    imu_config_fsr_gyro(IMU_GYRO_DEFAULT_FSR);
    imu_config_fsr_accel(IMU_ACCEL_DEFAULT_FSR);

    // Restore first from FLASH memory:

    for (int i = 0; i < 64; i++)
    {
      dmpBiasFlash[i] = *((uint32_t *)(0x080FE000 + i));
    }
    for (int i = 0; i < 64; i++)
    {
      otherFlash[i] = *((uint32_t *)(0x080FC000 + i));
    }
    // erase page 126
    FLASH_EraseInitTypeDef flashEraseData;
    flashEraseData.TypeErase = FLASH_TYPEERASE_PAGES;
    flashEraseData.Page = 126;
    flashEraseData.NbPages = 1;
    uint32_t flashErasePageError = 0;
    // write value(s) to this location:
    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&flashEraseData, &flashErasePageError);
    HAL_FLASH_Lock();

    if ((dmpBiasFlash[42] == 0xDA) && (dmpBiasFlash[43] == 0xDA) && (otherFlash[0] == 0xDA) && (otherFlash[1] == 0xDA))
    {
#if PRINTF_APP_IMU
      waitToPrint();
      npf_snprintf(uart_buf, 200, "Calibration settings are being restored. A reboot of the module happened.\r\n");
      huart2print(uart_buf, strlen(uart_buf));
#endif

#if PRINTF_APP_IMU
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_imu] Data restored from Flash: 0x",(unsigned int) xTaskGetTickCount());
      char     byteString[3];
      for (unsigned int i = 0; i < 48; i++)
      {
        npf_snprintf(byteString, 3, "%02X", dmpBiasFlash[i]);
        strcat(uart_buf, byteString);
      }
      strcat(uart_buf, ". \n");
      huart2print(uart_buf, strlen(uart_buf));
#endif

      inv_device_set_sensor_period(device, INV_SENSOR_TYPE_ACCELEROMETER, IMU_DEFAULT_SAMPL_FREQ);
      inv_device_set_sensor_period(device, INV_SENSOR_TYPE_GYROSCOPE,     IMU_DEFAULT_SAMPL_FREQ);
      inv_device_set_sensor_period(device, INV_SENSOR_TYPE_MAGNETOMETER,  IMU_DEFAULT_SAMPL_FREQ);

      uint32_t accOffset[3] = {0,0,0};
      uint32_t magOffset[3] = {0,0,0};
      uint32_t gyrOffset[3] = {0,0,0};

      inv_device_set_sensor_config(device, INV_SENSOR_TYPE_ACCELEROMETER, INV_SENSOR_CONFIG_OFFSET, &accOffset, sizeof(accOffset));
      inv_device_set_sensor_config(device, INV_SENSOR_TYPE_MAGNETOMETER,  INV_SENSOR_CONFIG_OFFSET, &magOffset, sizeof(magOffset));
      inv_device_set_sensor_config(device, INV_SENSOR_TYPE_GYROSCOPE,     INV_SENSOR_CONFIG_OFFSET, &gyrOffset, sizeof(gyrOffset));

      inv_device_get_sensor_config(device, INV_SENSOR_TYPE_ACCELEROMETER, INV_SENSOR_CONFIG_OFFSET, &accOffset, sizeof(accOffset));
      inv_device_get_sensor_config(device, INV_SENSOR_TYPE_MAGNETOMETER,  INV_SENSOR_CONFIG_OFFSET, &magOffset, sizeof(magOffset));
      inv_device_get_sensor_config(device, INV_SENSOR_TYPE_GYROSCOPE,     INV_SENSOR_CONFIG_OFFSET, &gyrOffset, sizeof(gyrOffset));

#if PRINTF_APP_IMU
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_imu] Accelerometer restored offsets x = %u, y = %u, z = %u.\r\n",
    		  (unsigned int) xTaskGetTickCount(), (unsigned int)accOffset[0], (unsigned int)accOffset[1], (unsigned int)accOffset[2]);
      huart2print(uart_buf, strlen(uart_buf));
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_imu]  Magnetometer restored offsets x = %u, y = %u, z = %u.\r\n",
    		  (unsigned int) xTaskGetTickCount(), (unsigned int)magOffset[0], (unsigned int)magOffset[1], (unsigned int)magOffset[2]);
      huart2print(uart_buf, strlen(uart_buf));
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_ime]   Gyroscope   restored offsets x = %u, y = %u, z = %u.\r\n",
    		  (unsigned int) xTaskGetTickCount(), (unsigned int)gyrOffset[0], (unsigned int)gyrOffset[1], (unsigned int)gyrOffset[2]);
      huart2print(uart_buf, strlen(uart_buf));
      gyroAccuracy  = 3;
      accelAccuracy = 3;
      magAccuracy   = 3;
#endif

      rc += inv_device_stop_sensor(device, INV_SENSOR_TYPE_GYROSCOPE);
      check_rc(rc);
      rc += inv_device_stop_sensor(device, INV_SENSOR_TYPE_ACCELEROMETER);
      check_rc(rc);
      rc += inv_device_stop_sensor(device, INV_SENSOR_TYPE_MAGNETOMETER);
      check_rc(rc);

    }
    else
    {
      //imu_enable_sensors
#if PRINTF_APP_IMU
      waitToPrint();
      npf_snprintf(uart_buf, 200, "Start calibration.\r\n");
      huart2print(uart_buf, strlen(uart_buf));
#endif

      calibrationActive = 1;

      // Reset status before starting calibration
      gyroAccuracy  = 0;
      accelAccuracy = 0;
      magAccuracy   = 0;

      calibration_callback();

#if PRINTF_APP_IMU
	  waitToPrint();
	  npf_snprintf(uart_buf, 200, "%u [app_imu] Start GYRO for calibration\r\n",(unsigned int) xTaskGetTickCount());
	  huart2print(uart_buf, strlen(uart_buf));
	  npf_snprintf(uart_buf, 200, ".\r\n");
#endif

      rc += inv_device_ping_sensor(device, INV_SENSOR_TYPE_GYROSCOPE);
	  check_rc(rc);
	  rc += inv_device_set_sensor_period(device, INV_SENSOR_TYPE_GYROSCOPE, IMU_DEFAULT_SAMPL_FREQ);
	  check_rc(rc);
	  rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_GYROSCOPE);
	  check_rc(rc);

	//vTaskDelay(50);
//#if PRINTF_APP_IMU
//        waitToPrint();
//        npf_snprintf(uart_buf, 200, "%u [app_imu] Start ACCEL for calibration\r\n",(unsigned int) xTaskGetTickCount());
//        huart2print(uart_buf, strlen(uart_buf));
//        npf_snprintf(uart_buf, 200, ".\r\n");
//#endif
//
//	rc += inv_device_ping_sensor(device, INV_SENSOR_TYPE_ACCELEROMETER);
//	check_rc(rc);
//	rc += inv_device_set_sensor_period(device, INV_SENSOR_TYPE_ACCELEROMETER, IMU_DEFAULT_SAMPL_FREQ);
//	check_rc(rc);
//	rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_ACCELEROMETER);
//	check_rc(rc);
//
//#if PRINTF_APP_IMU
//        waitToPrint();
//        npf_snprintf(uart_buf, 200, "%u [app_imu] Start MAG for calibration\r\n",(unsigned int) xTaskGetTickCount());
//        huart2print(uart_buf, strlen(uart_buf));
//        npf_snprintf(uart_buf, 200, ".\r\n");
//#endif
//
//	rc += inv_device_ping_sensor(device, INV_SENSOR_TYPE_MAGNETOMETER);
//	check_rc(rc);
//	rc += inv_device_set_sensor_period(device, INV_SENSOR_TYPE_MAGNETOMETER, IMU_DEFAULT_SAMPL_FREQ);
//	check_rc(rc);
//	rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_MAGNETOMETER);
//	check_rc(rc);

      while (calibrationActive)
      {
	    rc = inv_device_poll(device); // Poll device for data
	    vTaskDelay(20);
      }

// Calibration done
    }
#endif //CALIBRATION

    /*
     * Check sensor availibility
     * if rc value is 0, it means sensor is available,
     * if rc value is INV_ERROR or INV_ERROR_BAD_ARG, sensor is NA
     */
    available_sensor_mask = 0;
    for(i = 0; i < sizeof(sensor_list)/sizeof(sensor_list[0]); ++i)
    {
      const int rc = inv_device_ping_sensor(device, sensor_list[i].type);
      waitToPrint();
      npf_snprintf(uart_buf, 100, "Ping %s %s", inv_sensor_2str(sensor_list[i].type), (rc == 0) ? "OK\r\n" : "KO\r\n");
      huart2print(uart_buf, strlen(uart_buf));
      if (rc == 0)
      {
        available_sensor_mask |= (1ULL << sensor_list[i].type);
      }
    }
    /*
     * Start all available sensors from the sensor list
    */
    for (i = 0; i < sizeof(sensor_list)/sizeof(sensor_list[0]); ++i)
    {
      if (available_sensor_mask & (1ULL << sensor_list[i].type))
      {
        waitToPrint();
        npf_snprintf(uart_buf, 100, "Starting %s @ %u us\r\n", inv_sensor_2str(sensor_list[i].type), (unsigned int)sensor_list[i].period_us);
        huart2print(uart_buf, strlen(uart_buf));
        rc  = inv_device_set_sensor_period_us(device, sensor_list[i].type, sensor_list[i].period_us);
        check_rc(rc);
        rc += inv_device_start_sensor(device, sensor_list[i].type);
        check_rc(rc);

        waitToPrint();
        npf_snprintf(uart_buf, 100, "Sensor started.\r\n");
        huart2print(uart_buf, strlen(uart_buf));
      }
    }
  }

//  waitToPrint();
//  npf_snprintf(uart_buf, 100, "IMU Calibrated. Place IMU now in reference position to reset to zero. You have 30s to do that.\r\n");
//  huart2print(uart_buf, strlen(uart_buf));
//  vTaskDelay(30000);
//  waitToPrint();
//  npf_snprintf(uart_buf, 100, "Starting reset now, Do not move sensor until further notice.\r\n");
//  huart2print(uart_buf, strlen(uart_buf));
//
//  for (int i = 0; i < 80 ;i++)
//  {
//    imuMeasurementNr = 0;
//    rc = inv_device_poll(device); // Poll device for data
//    imuMeasurementNr++;
////    check_rc(rc);
//    vTaskDelay(100U);
//  }
//  pitch0 = pitch;
//  yaw0   = yaw;
//  roll0  = roll;
//  waitToPrint();
//  npf_snprintf(uart_buf, 100, "IMU reset done, the sensor can now be moved!\r\n");
//  huart2print(uart_buf, strlen(uart_buf));


//  if (startGateway)
//  {

//  }
//  vTaskDelay(1000);

  InitThreadNotify(NOTIFICATION_FROM_IMU_INITIALIZED);

//  vTaskDelay(10000);
  waitToPrint();
  npf_snprintf(uart_buf, 100, "[IMU] Notification to init given that IMU is initialized.\r\n");
  huart2print(uart_buf, strlen(uart_buf));

//    GatewayThreadInit();
  // 20240616
  vTaskSuspend(imuThreadHandler);

//  TickType_t xLastWakeTime = xTaskGetTickCount();
  for(;;)
  { // Infinite loop
//    HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);
//    vTaskDelay(500U);
//    HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);

//    if (irq_from_device & TO_MASK(GPIO_SENSOR_IRQ_D6))
//    {
//	ICM_20948_sleepModeEnable(0, &device_icm20948); //Putting Icm20948

//	vTaskDelay(3000);
	  //todo: terug aanzetten als gateway thread loopt!!!
//#if PRINTF_APP_IMU
//          waitToPrint();
//          npf_snprintf(uart_buf, 200, ".\r\n");
//          huart2print(uart_buf, strlen(uart_buf));
//#endif

    if (pollIMU && imuAvailable)
    {
//#if PRINTF_APP_IMU
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "%u [app_imu] Start of poll IMU.\r\n",(unsigned int) xTaskGetTickCount());
//      huart2print(uart_buf, strlen(uart_buf));
//#endif

      xTaskNotifyStateClear(imuThreadHandler);
      if (xTaskNotifyWait(0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(200)))
      {// No time-out
        if ((notificationValue & NOTIFICATION_FROM_IMU) == NOTIFICATION_FROM_IMU)
        { // Interrupt from INT1_ICM20948_Pin (see main HAL_GPIO_EXTI_Rising_Callback())
//#if PRINTF_APP_IMU
//          waitToPrint();
//          npf_snprintf(uart_buf, 200, "%u [app_imu] Poll IMU...\r\n",(unsigned int) xTaskGetTickCount());
//          huart2print(uart_buf, strlen(uart_buf));
//#endif



          rc = inv_device_poll(device); // Poll device for data
          check_rc(rc);

//#if USE_ORI
//          waitToPrint();
//          npf_snprintf(uart_buf, 200, "%u [app_imu] yaw = %d, pitch = %d, roll = %d\r\n", (unsigned int) xTaskGetTickCount(), yaw, pitch, roll);
//          huart2print(uart_buf, strlen(uart_buf));
//          npf_snprintf(uart_buf, 200, ".\r\n");
//#endif
//
//#if USE_RV
//          waitToPrint();
//          npf_snprintf(uart_buf, 200, "%u [app_imu] quat: w = %ld, x = %ld, y = %ld, z = %ld.\r\n", (unsigned int) xTaskGetTickCount(), quat.w, quat.x, quat.y, quat.z);
//          huart2print(uart_buf, strlen(uart_buf));
//          npf_snprintf(uart_buf, 200, ".\r\n");
//#endif
//#if USE_RV
//          waitToPrint();
//          npf_snprintf(uart_buf, 200, "%u [app_imu] int16_t format quat: w = %d, x = %d, y = %d, z = %d.\r\n", (unsigned int) xTaskGetTickCount(), quatW, quatX, quatY, quatZ);
//          huart2print(uart_buf, strlen(uart_buf));
//          npf_snprintf(uart_buf, 200, ".\r\n");
//#endif
        }
        else
        {
#if PRINTF_APP_IMU
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_imu] Wrong notification value!\r\n",(unsigned int) xTaskGetTickCount());
          huart2print(uart_buf, strlen(uart_buf));
//          npf_snprintf(uart_buf, 200, ".\r\n");
#endif
        }
      }
      else
      {
#if PRINTF_APP_IMU
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_imu] Time-out (>200ms) on interrupt from INT1_ICM20948_Pin!\r\n",(unsigned int) xTaskGetTickCount());
        huart2print(uart_buf, strlen(uart_buf));
        rc = inv_device_poll(device); // Poll device for data
#endif

      }
    }

// moved to app_gateway on 20231125
//      ICM_20948_sleepModeEnable(1, &device_icm20948);
//      HAL_LPTIM_Counter_Start_IT(&hlptim1);
//      __HAL_LPTIM_DISABLE_IT(&hlptim1, LPTIM_IT_ARROK);
//      HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

//      HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
//      //vTaskDelay(200);
////EXTI_CLEAR
//      rccCRSave = RCC->CR;
//      rccCFGRSave = RCC->CFGR1;
//      SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
//      DBGMCU->SCR = 0;
//      //PWR_CR1
//
//      sleepFlag++;
//#if PRINTF_APP_IMU
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "%u [app_imu] Going to STOP mode!\r\n",(unsigned int) xTaskGetTickCount());
//      huart2print(uart_buf, strlen(uart_buf));
//      npf_snprintf(uart_buf, 200, ".\r\n");
//#endif
//
//      __disable_irq();
//
//      for (unsigned int i = 0; i < 69; i++)
//      {
//        NVIC_ClearPendingIRQ(i);
//        vTaskDelay(1);
//      }
////      WWDG_IRQn                 = 0,      /*!< Window WatchDog interrupt                                         */
////        PVD_IRQn                  = 1,      /*!< PVD through EXTI Line detection Interrupt                         */
////        RTC_IRQn                  = 2,      /*!< RTC non-secure interrupt                                          */
////        RTC_S_IRQn                = 3,      /*!< RTC secure interrupt                                              */
////        TAMP_IRQn                 = 4,      /*!< Tamper global interrupt                                           */
////        RAMCFG_IRQn               = 5,      /*!< RAMCFG global interrupt                                           */
////        FLASH_IRQn                = 6,      /*!< FLASH non-secure global interrupt                                 */
////        FLASH_S_IRQn              = 7,      /*!< FLASH secure global interrupt                                     */
////        GTZC_IRQn                 = 8,      /*!< Global TrustZone Controller interrupt                             */
////        RCC_IRQn                  = 9,      /*!< RCC non secure global interrupt                                   */
////        RCC_S_IRQn                = 10,     /*!< RCC secure global interrupt                                       */
////        EXTI0_IRQn                = 11,     /*!< EXTI Line0 interrupt                                              */
////        EXTI1_IRQn                = 12,     /*!< EXTI Line1 interrupt                                              */
////        EXTI2_IRQn                = 13,     /*!< EXTI Line2 interrupt                                              */
////        EXTI3_IRQn                = 14,     /*!< EXTI Line3 interrupt                                              */
////        EXTI4_IRQn                = 15,     /*!< EXTI Line4 interrupt                                              */
////        EXTI5_IRQn                = 16,     /*!< EXTI Line5 interrupt                                              */
////        EXTI6_IRQn                = 17,     /*!< EXTI Line6 interrupt                                              */
////        EXTI7_IRQn                = 18,     /*!< EXTI Line7 interrupt                                              */
////        EXTI8_IRQn                = 19,     /*!< EXTI Line8 interrupt                                              */
////        EXTI9_IRQn                = 20,     /*!< EXTI Line9 interrupt                                              */
////        EXTI10_IRQn               = 21,     /*!< EXTI Line10 interrupt                                             */
////        EXTI11_IRQn               = 22,     /*!< EXTI Line11 interrupt                                             */
////        EXTI12_IRQn               = 23,     /*!< EXTI Line12 interrupt                                             */
////        EXTI13_IRQn               = 24,     /*!< EXTI Line13 interrupt                                             */
////        EXTI14_IRQn               = 25,     /*!< EXTI Line14 interrupt                                             */
////        EXTI15_IRQn               = 26,     /*!< EXTI Line15 interrupt                                             */
////        IWDG_IRQn                 = 27,     /*!< IWDG global interrupt                                             */
////        SAES_IRQn                 = 28,     /*!< Secure AES global interrupt                                       */
////        GPDMA1_Channel0_IRQn      = 29,     /*!< GPDMA1 Channel 0 global interrupt                                 */
////        GPDMA1_Channel1_IRQn      = 30,     /*!< GPDMA1 Channel 1 global interrupt                                 */
////        GPDMA1_Channel2_IRQn      = 31,     /*!< GPDMA1 Channel 2 global interrupt                                 */
////        GPDMA1_Channel3_IRQn      = 32,     /*!< GPDMA1 Channel 3 global interrupt                                 */
////        GPDMA1_Channel4_IRQn      = 33,     /*!< GPDMA1 Channel 4 global interrupt                                 */
////        GPDMA1_Channel5_IRQn      = 34,     /*!< GPDMA1 Channel 5 global interrupt                                 */
////        GPDMA1_Channel6_IRQn      = 35,     /*!< GPDMA1 Channel 6 global interrupt                                 */
////        GPDMA1_Channel7_IRQn      = 36,     /*!< GPDMA1 Channel 7 global interrupt                                 */
////        TIM1_BRK_IRQn             = 37,     /*!< TIM1 Break interrupt                                              */
////        TIM1_UP_IRQn              = 38,     /*!< TIM1 Update interrupt                                             */
////        TIM1_TRG_COM_IRQn         = 39,     /*!< TIM1 Trigger and Commutation interrupt                            */
////        TIM1_CC_IRQn              = 40,     /*!< TIM1 Capture Compare interrupt                                    */
////        TIM2_IRQn                 = 41,     /*!< TIM2 global interrupt                                             */
////        TIM3_IRQn                 = 42,     /*!< TIM3 global interrupt                                             */
////        I2C1_EV_IRQn              = 43,     /*!< I2C1 Event interrupt                                              */
////        I2C1_ER_IRQn              = 44,     /*!< I2C1 Error interrupt                                              */
////        SPI1_IRQn                 = 45,     /*!< SPI1 global interrupt                                             */
////        USART1_IRQn               = 46,     /*!< USART1 global interrupt                                           */
////        USART2_IRQn               = 47,     /*!< USART2 global interrupt                                           */
////        LPUART1_IRQn              = 48,     /*!< LPUART1 global interrupt                                          */
////        LPTIM1_IRQn               = 49,     /*!< LPTIM1 global interrupt                                           */
////        LPTIM2_IRQn               = 50,     /*!< LPTIM2 global interrupt                                           */
////        TIM16_IRQn                = 51,     /*!< TIM16 global interrupt                                            */
////        TIM17_IRQn                = 52,     /*!< TIM17 global interrupt                                            */
////        I2C3_EV_IRQn              = 54,     /*!< I2C3 Event interrupt                                              */
////        I2C3_ER_IRQn              = 55,     /*!< I2C3 Error interrupt                                              */
////        TSC_IRQn                  = 57,     /*!< Touch Sense Controller global interrupt                           */
////        AES_IRQn                  = 58,     /*!< AES global interrupt                                              */
////        RNG_IRQn                  = 59,     /*!< RNG global interrupt                                              */
////        FPU_IRQn                  = 60,     /*!< FPU global interrupt                                              */
////        HASH_IRQn                 = 61,     /*!< HASH global interrupt                                             */
////        PKA_IRQn                  = 62,     /*!< PKA global interrupt                                              */
////        SPI3_IRQn                 = 63,     /*!< SPI3 global interrupt                                             */
////        ICACHE_IRQn               = 64,     /*!< Instruction cache global interrupt                                */
////        ADC4_IRQn                 = 65,     /*!< ADC4 global interrupt                                             */
////        RADIO_IRQn                = 66,     /*!< 2.4GHz RADIO global interrupt                                     */
////        WKUP_IRQn                 = 67,     /*!< PWR global WKUP pin interrupt                                     */
////        HSEM_IRQn                 = 68,     /*!< HSEM non-secure global interrupt                                  */
////        HSEM_S_IRQn               = 69,     /*!< HSEM secure global interrupt                                      */
//      /* Enable Power Clock */
//      __HAL_RCC_PWR_CLK_ENABLE();
//
//      /* Disable all used wakeup sources: Pin1(PA.0) */
//      HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN6);
//      HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN7);
//
//
//      /* Enable all used wakeup sources: Pin1(PA.0) */
//      HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN6);
//      HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN7);
//
//      /* Clear all related wakeup flags */
//      __HAL_PWR_CLEAR_FLAG(RTC_CLEAR_WUTF); //WUTOC
//      __HAL_PWR_CLEAR_FLAG(PWR_FLAG_STOPF);
//      __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SBF);
//      __HAL_PWR_CLEAR_FLAG(0x06U);
//      __HAL_PWR_CLEAR_FLAG(PWR_WAKEUP_FLAG6);
//      __HAL_PWR_CLEAR_FLAG(PWR_WAKEUP_FLAG7);
//      __HAL_PWR_CLEAR_FLAG(PWR_WAKEUP_ALL_FLAG);
//      __HAL_GPIO_EXTI_CLEAR_RISING_IT(INT1_ICM20948_Pin);
//
//
////PWR_FLAG
//      //EXTI_RPR1
//      //WRITE_REG(RTC->SCR, RTC_SCR_CWUTF);
//
//      //WRITE_REG(RTC->SCR, RTC_CLEAR_WUTF);
//
////      HAL_SuspendTick();
////      HAL_PWR_EnterSTOPMode(0, PWR_STOPENTRY_WFI);
//      HAL_PWR_GetClearWakeupSource();
//
//
//
//      PWR->WUSCR = PWR_WUSCR_CWUF;
//      (void)PWR->WUSCR;
//
//
//      /* Select Stop mode */
//      MODIFY_REG(PWR->CR1, PWR_CR1_LPMS, PWR_LOWPOWERREGULATOR_ON);
//      (void)PWR->CR1;
//
//      __enable_irq();
//
//      /* Set SLEEPDEEP bit of Cortex System Control Register */
//      SET_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk);
//      (void)SCB->SCR;
//
//
//         /* Wait For Interrupt Request */
//
//        __DSB();
//        __WFI();
//
//
////      HAL_PWR_EnterSTANDBYMode();
//
//#if PRINTF_APP_IMU
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "%u [app_imu] Printing after STOP mode...\r\n",(unsigned int) xTaskGetTickCount());
//      huart2print(uart_buf, strlen(uart_buf));
//      npf_snprintf(uart_buf, 200, ".\r\n");
//#endif
//
//    }
//    else
//    {// time-out
//#if PRINTF_APP_IMU
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "%u [app_imu] INT1_ICM20948_Pin time-out occurred...(>10s).\r\n",(unsigned int) xTaskGetTickCount());
//      huart2print(uart_buf, strlen(uart_buf));
//#endif
//    }
//
////  if(rc >= 0)
////  {
////    __disable_irq();
////    irq_from_device &= ~TO_MASK(GPIO_SENSOR_IRQ_D6);
////    __enable_irq();
////  }
//    }
////    vTaskDelayUntil(&xLastWakeTime, 1000U);
//
    if (imuAvailable)
    {
      vTaskDelay(100);
    }
    else
    {
#if PRINTF_APP_IMU
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_imu] No IMU available.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
#endif
      vTaskDelay(10000);

    }

  }
}


static void check_rc(int rc)
{
  if(rc == -1)
  {
    waitToPrint();
    npf_snprintf(uart_buf, 100, "BAD RC=%d\r\n", rc);
    huart2print(uart_buf, strlen(uart_buf));
    //while(1);
  }
}

void inv_icm20948_sleep_us(int us)
{
  //delay_us(us);
  //TODO change into something with microseconds
  HAL_Delay(us/1000);
}

uint64_t inv_icm20948_get_time_us(void)
{
  //TODO: is this correct? with hal tick?
  //return timer_get_counter(TIMEBASE_TIMER);
  return xTaskGetTickCount();
}

/*
 * Callback called upon sensor event reception
 * This function is called in the same context as inv_device_poll()
 */
static void sensor_event_cb(const inv_sensor_event_t * event, void * arg)
{
  /* arg will contain the value provided at init time */
  (void)arg;
  /*
  * In normal mode, display sensor event over UART messages
  */
  //activityName
//#if PRINTF_APP_IMU
//  waitToPrint();
//  npf_snprintf(uart_buf, 100, "data event.\r\n");
//  huart2print(uart_buf, strlen(uart_buf));
//#endif




  if (event->status == INV_SENSOR_STATUS_DATA_UPDATED)
  {
    switch (INV_SENSOR_ID_TO_TYPE(event->sensor))
    {
      case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
      case INV_SENSOR_TYPE_RAW_GYROSCOPE:
//        waitToPrint();
//        npf_snprintf(uart_buf, 100, "data event %s (lsb): %lu %d %d %d\r\n", inv_sensor_str(event->sensor),
//            (long unsigned int)event->timestamp,
//            (int)event->data.raw3d.vect[0],
//            (int)event->data.raw3d.vect[1],
//            (int)event->data.raw3d.vect[2]);
//        huart2print(uart_buf, strlen(uart_buf));
	    break;
      case INV_SENSOR_TYPE_ACCELEROMETER:
      case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
      case INV_SENSOR_TYPE_GRAVITY:

//#if PRINTF_APP_IMU
//        waitToPrint();
//        npf_snprintf(uart_buf, 100, "data event %s (mg): %d %d %d\r\n", inv_sensor_str(event->sensor),
//            (int)(event->data.acc.vect[0]*1000),
//            (int)(event->data.acc.vect[1]*1000),
//            (int)(event->data.acc.vect[2]*1000));
//        huart2print(uart_buf, strlen(uart_buf));
//#endif

      	//accX = event->data.acc.vect[0]*1000;
      	//accY = event->data.acc.vect[1]*1000;
      	//accZ = event->data.acc.vect[2]*1000;

    	  //from jona:
//		imu_data.accel.x =      (int16_t)((event->data.acc.vect[0]) * (1 << RAW_Q_FORMAT_ACC_COMMA_BITS));
//		imu_data.accel.y =      (int16_t)((event->data.acc.vect[1]) * (1 << RAW_Q_FORMAT_ACC_COMMA_BITS));
//		imu_data.accel.z =      (int16_t)((event->data.acc.vect[2]) * (1 << RAW_Q_FORMAT_ACC_COMMA_BITS));

    	accX =      (event->data.acc.vect[0]*1000);
    	accY =      (event->data.acc.vect[1]*1000);
    	accZ =      (event->data.acc.vect[2]*1000);

        if (accelCalibrated && !calibrationActive)
        {
#if PRINTF_APP_IMU
	      waitToPrint();
	      npf_snprintf(uart_buf, 100, "%u Accelerometer: x = %f, y = %f, z = %f\r\n", (unsigned int) xTaskGetTickCount(), accX, accY, accZ);
	      huart2print(uart_buf, strlen(uart_buf));
#endif
        }

	    // Save accuracy flag if it changes
	    if(event->data.acc.accuracy_flag != accelAccuracy)
	    {
	      if(calibrationActive)
	      {
#if PRINTF_APP_IMU
	        waitToPrint();
	        npf_snprintf(uart_buf, 100, "%u Cal status: accel: %d\r\n", (unsigned int) xTaskGetTickCount(), event->data.acc.accuracy_flag);
	        huart2print(uart_buf, strlen(uart_buf));
#endif
	        accelAccuracy = event->data.acc.accuracy_flag;
	        calibration_callback();
	      }
	    }
        break;
      case INV_SENSOR_TYPE_GYROSCOPE:

//#if PRINTF_APP_IMU
//	waitToPrint();
//        npf_snprintf(uart_buf, 100, "data event %s (mdps): %d %d %d\r\n", inv_sensor_str(event->sensor),
//            (int)(event->data.gyr.vect[0]*1000),
//            (int)(event->data.gyr.vect[1]*1000),
//            (int)(event->data.gyr.vect[2]*1000));
//        huart2print(uart_buf, strlen(uart_buf));
//#endif

        gyrX = (event->data.gyr.vect[0]*1000);
        gyrY = (event->data.gyr.vect[1]*1000);
        gyrZ = (event->data.gyr.vect[2]*1000);

        if (gyroCalibrated && !calibrationActive)
        {
#if PRINTF_APP_IMU
          waitToPrint();
          npf_snprintf(uart_buf, 100, "%u Gyroscope: x = %f, y = %f, z = %f\r\n", (unsigned int) xTaskGetTickCount(), gyrX, gyrY, gyrZ);
          huart2print(uart_buf, strlen(uart_buf));
#endif
        }

        if (!gyroCalibrated)
        {
//#if PRINTF_APP_IMU
//	waitToPrint();
//	npf_snprintf(uart_buf, 100, "Cal accuracy: gyro: %d\r\n", event->data.gyr.accuracy_flag);
//	huart2print(uart_buf, strlen(uart_buf));
//#endif
        }


	// Save accuracy flag
	    if(event->data.gyr.accuracy_flag != gyroAccuracy)
	    {
	      if(calibrationActive)
	      {
#if PRINTF_APP_IMU
	        waitToPrint();
	        npf_snprintf(uart_buf, 100, "%u Cal status: gyro: %d\r\n", (unsigned int) xTaskGetTickCount(), event->data.gyr.accuracy_flag);
	        huart2print(uart_buf, strlen(uart_buf));
#endif
	        gyroAccuracy = event->data.gyr.accuracy_flag;
	        calibration_callback();
	      }
	    }
        break;
      case INV_SENSOR_TYPE_MAGNETOMETER:
//#if PRINTF_APP_IMU
//        waitToPrint();
//        npf_snprintf(uart_buf, 100, "data event %s (nT): %d %d %d\r\n", inv_sensor_str(event->sensor),
//            (int)(event->data.mag.vect[0]*1000),
//            (int)(event->data.mag.vect[1]*1000),
//            (int)(event->data.mag.vect[2]*1000));
//        huart2print(uart_buf, strlen(uart_buf));
//#endif
//	if (!magCalibrated)
//	{
////#if PRINTF_APP_IMU
////	waitToPrint();
////	npf_snprintf(uart_buf, 100, "Cal accuracy: mag: %d\r\n", event->data.mag.accuracy_flag);
////	huart2print(uart_buf, strlen(uart_buf));
////#endif
//	}

//  	magX = event->data.acc.vect[0]*1000;
//  	magY = event->data.acc.vect[1]*1000;
//  	magZ = event->data.acc.vect[2]*1000;

    	  //from jona:
//			imu_data.mag.y =   -(int16_t)((event->data.mag.vect[0]) * (1 << RAW_Q_FORMAT_CMP_COMMA_BITS)); // Changed axes and inverted. Corrected for rotation of axes.
//			imu_data.mag.x =    (int16_t)((event->data.mag.vect[1]) * (1 << RAW_Q_FORMAT_CMP_COMMA_BITS)); // Changed axes. Corrected for rotation of axes.
//			imu_data.mag.z =    (int16_t)((event->data.mag.vect[2]) * (1 << RAW_Q_FORMAT_CMP_COMMA_BITS));


//		magY =   -(event->data.mag.vect[0]); // Changed axes and inverted. Corrected for rotation of axes.
//		magX =    (event->data.mag.vect[1]); // Changed axes. Corrected for rotation of axes.
//		magZ =    (event->data.mag.vect[2]);

		magY =   -(event->data.mag.vect[0]*1000); // Changed axes and inverted. Corrected for rotation of axes.
		magX =    (event->data.mag.vect[1]*1000); // Changed axes. Corrected for rotation of axes.
		magZ =    (event->data.mag.vect[2]*1000);

	    if (magCalibrated)
	    {
#if PRINTF_APP_IMU
	      waitToPrint();
	      npf_snprintf(uart_buf, 100, "%u Magnetometer: x = %f, y = %f, z = %f\r\n", (unsigned int) xTaskGetTickCount(), magX, magY, magZ);
	      huart2print(uart_buf, strlen(uart_buf));
#endif
	    }

  // Save accuracy flag
	    if(event->data.mag.accuracy_flag != magAccuracy)
	    {
	      if(calibrationActive)
	      {
#if PRINTF_APP_IMU
	        waitToPrint();
	        npf_snprintf(uart_buf, 100, "%u Cal status: mag: %d\r\n", (unsigned int) xTaskGetTickCount(), event->data.mag.accuracy_flag);
	        huart2print(uart_buf, strlen(uart_buf));
#endif
	        magAccuracy = event->data.mag.accuracy_flag;

//

	        calibration_callback();
	      }
	    }

        break;
      case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
        waitToPrint();
        npf_snprintf(uart_buf, 100, "data event %s (mdps): %d %d %d %d %d %d\r\n", inv_sensor_str(event->sensor),
            (int)(event->data.gyr.vect[0]*1000),
            (int)(event->data.gyr.vect[1]*1000),
            (int)(event->data.gyr.vect[2]*1000),
            (int)(event->data.gyr.bias[0]*1000),
            (int)(event->data.gyr.bias[1]*1000),
            (int)(event->data.gyr.bias[2]*1000));
        huart2print(uart_buf, strlen(uart_buf));
        break;
      case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:
//        waitToPrint();
//        npf_snprintf(uart_buf, 100, "data event %s (nT): %d %d %d %d %d %d\r\n", inv_sensor_str(event->sensor),
//            (int)(event->data.mag.vect[0]*1000),
//            (int)(event->data.mag.vect[1]*1000),
//            (int)(event->data.mag.vect[2]*1000),
//            (int)(event->data.mag.bias[0]*1000),
//            (int)(event->data.mag.bias[1]*1000),
//            (int)(event->data.mag.bias[2]*1000));
//        huart2print(uart_buf, strlen(uart_buf));
        break;
      case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
      case INV_SENSOR_TYPE_ROTATION_VECTOR:
      case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
	    imuMeasurementNr++;

//	    if (imuMeasurementNr == 100)
//	    {
//          waitToPrint();
//          npf_snprintf(uart_buf, 100, "%u data event nr %u, %s (e-3): %f %f %f %f\r\n", (unsigned int) xTaskGetTickCount(), (unsigned int) imuMeasurementNr, inv_sensor_str(event->sensor), event->data.quaternion.quat[0], event->data.quaternion.quat[1], event->data.quaternion.quat[2], event->data.quaternion.quat[3]);
//          huart2print(uart_buf, strlen(uart_buf));
//        }
        quat.w = event->data.quaternion.quat[0]*10000;
        quat.x = event->data.quaternion.quat[1]*10000;
        quat.y = event->data.quaternion.quat[2]*10000;
        quat.z = event->data.quaternion.quat[3]*10000;

//        accBias[0] = event->data.acc.bias[0];
//        accBias[1] = event->data.acc.bias[1];
//        accBias[2] = event->data.acc.bias[2];
//        gyrBias[0] = event->data.gyr.bias[0];
//        gyrBias[1] = event->data.gyr.bias[1];
//        gyrBias[2] = event->data.gyr.bias[2];
//        magBias[0] = event->data.mag.bias[0];
//        magBias[1] = event->data.mag.bias[1];
//        magBias[2] = event->data.mag.bias[2];


#if PRINTF_APP_IMU
        if (imuMeasurementNr == 1)
        {

//          uint32_t gyrOffset2[3];
//          uint32_t accOffset2[3];
//          uint32_t magOffset2[3];
//          inv_device_get_sensor_config(device, INV_SENSOR_TYPE_ACCELEROMETER, INV_SENSOR_CONFIG_OFFSET, &accOffset2, sizeof(accOffset2));
//          inv_device_get_sensor_config(device,  INV_SENSOR_TYPE_MAGNETOMETER, INV_SENSOR_CONFIG_OFFSET, &magOffset2, sizeof(magOffset2));
//          inv_device_get_sensor_config(device,   INV_SENSOR_TYPE_GYROSCOPE,   INV_SENSOR_CONFIG_OFFSET, &gyrOffset2, sizeof(gyrOffset2));
//          waitToPrint();
//          npf_snprintf(uart_buf, 200, "\r\n%u [app_imu] Accelerometer offsets x = %u, y = %u, z = %u.\r\n",(unsigned int) xTaskGetTickCount(), (unsigned int)accOffset2[0], (unsigned int)accOffset2[1], (unsigned int)accOffset2[2]);
//          huart2print(uart_buf, strlen(uart_buf));
//          waitToPrint();
//          npf_snprintf(uart_buf, 200, "%u [app_imu]  Magnetometer offsets x = %u, y = %u, z = %u.\r\n",(unsigned int) xTaskGetTickCount(), (unsigned int)magOffset2[0], (unsigned int)magOffset2[1], (unsigned int)magOffset2[2]);
//          huart2print(uart_buf, strlen(uart_buf));
//          waitToPrint();
//          npf_snprintf(uart_buf, 200, "%u [app_imu]   Gyroscope   offsets x = %u, y = %u, z = %u.\r\n",(unsigned int) xTaskGetTickCount(), (unsigned int)gyrOffset2[0], (unsigned int)gyrOffset2[1], (unsigned int)gyrOffset2[2]);
//          huart2print(uart_buf, strlen(uart_buf));





          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_imu] data event #%u, %s (e-3): %d %d %d %d\r\n", (unsigned int) xTaskGetTickCount(), (unsigned int) imuMeasurementNr, inv_sensor_str(event->sensor),
              (int) quat.w, (int) quat.x, (int) quat.y, (int) quat.z);
          huart2print(uart_buf, strlen(uart_buf));
        }
        else
        {
          waitToPrint();
          npf_snprintf(uart_buf, 200, ".");
          huart2print(uart_buf, strlen(uart_buf));
        }
#endif
//        float f_quat[4];
//		memcpy(f_quat, (event->data.quaternion.quat), sizeof(event->data.quaternion.quat));
//
//		fixed_point_t p_quat[4];
//		p_quat[0] = float_to_fixed_quat(f_quat[0]);
//		p_quat[1] = float_to_fixed_quat(f_quat[1]);
//		p_quat[2] = float_to_fixed_quat(f_quat[2]);
//		p_quat[3] = float_to_fixed_quat(f_quat[3]);
//
//		quat.w = p_quat[0];
//		quat.x = p_quat[1];
//		quat.y = p_quat[2];
//		quat.z = p_quat[3];

        // store 100 measurements in imuMeasQuatArray:
        if ((imuMeasurementNr > 19) && (imuMeasurementNr < 121))
        {
          imuMeasQuatArray[imuMeasurementNr-20].w = quat.w;
          imuMeasQuatArray[imuMeasurementNr-20].x = quat.x;
          imuMeasQuatArray[imuMeasurementNr-20].y = quat.y;
          imuMeasQuatArray[imuMeasurementNr-20].z = quat.z;
        }
        if (imuMeasurementNr == 120)
        {
          quatW64b = 0;
          quatX64b = 0;
          quatY64b = 0;
          quatZ64b = 0;
          for (int i = 0; i < 100; i++)
          {
          	quatW64b += imuMeasQuatArray[i].w;
          	quatX64b += imuMeasQuatArray[i].x;
          	quatY64b += imuMeasQuatArray[i].y;
          	quatZ64b += imuMeasQuatArray[i].z;
          }
          quatW = (int16_t) ((quatW64b + 50) / 100);
          quatX = (int16_t) ((quatX64b + 50) / 100);
          quatY = (int16_t) ((quatY64b + 50) / 100);
          quatZ = (int16_t) ((quatZ64b + 50) / 100);

//          quatW64b = ((quatW64b + 50) / 100);
//          quatX64b = ((quatX64b + 50) / 100);
//          quatY64b = ((quatY64b + 50) / 100);
//          quatZ64b = ((quatZ64b + 50) / 100);
//#if PRINTF_APP_IMU
//           waitToPrint();
//           npf_snprintf(uart_buf, 200, "%u [app_imu] Measurement: %d, mean value of quaternions: w = %d, x = %d, y = %d, z = %d.\r\n", (unsigned int) xTaskGetTickCount(), (int) imuMeasurementNr, (int) quatW64b, (int) quatX64b, (int) quatY64b, (int) quatZ64b);
//           huart2print(uart_buf, strlen(uart_buf));
//#endif
        }
//        if (imuMeasurementNr == 100)
//        {
//          quatW = (int16_t) quat.w;
//          quatX = (int16_t) quat.x;
//          quatY = (int16_t) quat.y;
//          quatZ = (int16_t) quat.z;
//        }

    	  //            (int)(event->data.quaternion.quat[1]*1000),
    	  //            (int)(event->data.quaternion.quat[2]*1000),
    	  //            (int)(event->data.quaternion.quat[3]*1000));


        break;
      case INV_SENSOR_TYPE_ORIENTATION:
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_imu] data event %s (e-3): rot z=%d rot x=%d rot y=%d\r\n", (unsigned int) xTaskGetTickCount(), inv_sensor_str(event->sensor),
            (int)(event->data.orientation.x*1000),
            (int)(event->data.orientation.y*1000),
            (int)(event->data.orientation.z*1000));
        huart2print(uart_buf, strlen(uart_buf));
        npf_snprintf(uart_buf, 200, ".\r\n");



         yaw   = (int)(event->data.orientation.x*1000) - yaw0;   //yaw
         roll  = (int)(event->data.orientation.y*1000) - roll0;  //roll
         pitch = (int)(event->data.orientation.z*1000) - pitch0; //pitch

         waitToPrint();
         npf_snprintf(uart_buf, 200, "%u [app_imu] data event after zeroing %s (e-3): rot z=%d rot x=%d rot y=%d\r\n", (unsigned int) xTaskGetTickCount(), inv_sensor_str(event->sensor),
             (int)(yaw),
             (int)(roll),
             (int)(pitch));
         huart2print(uart_buf, strlen(uart_buf));
         npf_snprintf(uart_buf, 200, ".\r\n");


// software from Jona
//        npf_snprintf(uart_buf, 200, "%u [app_imu] data event %s (e-3): %d %d %d\r\n", (unsigned int) xTaskGetTickCount(), inv_sensor_str(event->sensor),
//			(int)(event->data.orientation.x),
//			(int)(event->data.orientation.y),
//			(int)(event->data.orientation.z));
//        huart2print(uart_buf, strlen(uart_buf));
//        npf_snprintf(uart_buf, 200, ".\r\n");
//
//        fixed_point_t p_euler[3];
//
//		float euler[3];
//		euler[0] = event->data.orientation.x;
//		euler[1] = event->data.orientation.y;
//		euler[2] = event->data.orientation.z;
//
//		p_euler[0] = float_to_fixed_euler(euler[0]);
//		p_euler[1] = float_to_fixed_euler(euler[1]);
//		p_euler[2] = float_to_fixed_euler(euler[2]);
//
//		yaw   = p_euler[0];
//		pitch = p_euler[1];
//		roll  = p_euler[2];




        break;
      case INV_SENSOR_TYPE_BAC:
//        waitToPrint();
//        npf_snprintf(uart_buf, 100, "data event %s : %d %s\r\n", inv_sensor_str(event->sensor), event->data.bac.event, activityName(event->data.bac.event));
//        huart2print(uart_buf, strlen(uart_buf));
        break;
      case INV_SENSOR_TYPE_STEP_COUNTER:
//        waitToPrint();
//        npf_snprintf(uart_buf, 100, "data event %s : %lu\r\n", inv_sensor_str(event->sensor), (unsigned long)event->data.step.count);
//        huart2print(uart_buf, strlen(uart_buf));
        break;
      case INV_SENSOR_TYPE_PICK_UP_GESTURE:
      case INV_SENSOR_TYPE_STEP_DETECTOR:
      case INV_SENSOR_TYPE_SMD:
      case INV_SENSOR_TYPE_B2S:
      case INV_SENSOR_TYPE_TILT_DETECTOR:
      default:
//        waitToPrint();
//        npf_snprintf(uart_buf, 100, "data event %s : ...\r\n", inv_sensor_str(event->sensor));
//        huart2print(uart_buf, strlen(uart_buf));
        break;
    }
  }
  else
  {
    waitToPrint();
    npf_snprintf(uart_buf, 100, "INV_SENSOR_STATUS_DATA is not updated.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
  }
}

void ImuNotifyFromISR(uint32_t notValue)
{
  BaseType_t xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  if (imuThreadHandler != NULL)
  {
    xTaskNotifyFromISR(imuThreadHandler, notValue, eSetBits, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ICM20948_reset()
{
/* Set H_RESET bit to initiate soft reset */
  //ICM_20948_registerWrite(ICM_20948_REG_PWR_MGMT_1, BIT_H_RESET);

  /* Wait 100ms to complete the reset sequence */
  //nrf_delay_ms(100);
  vTaskDelay(100);
}

void ICM_20948_sleepModeEnable(int enable, void * context)
{
  uint8_t reg;
  inv_device_icm20948_t * self = (inv_device_icm20948_t *)context;
  /* Read the Sleep Enable register */
  //ICM_20948_registerRead(B0_PWR_MGMT_1, 1, &reg); //inv_icm20948_write_reg
  select_userbank(ub_0);
  spi_master_read_register(REG_PWR_MGMT_1, 1, &reg);
#if PRINTF_APP_IMU
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_imu] PWR_MGMT_1 register at begin of sleepMode setting: %02X, 0b" BYTE_TO_BINARY_PATTERN "\r\n",
		  (unsigned int) xTaskGetTickCount(), reg, BYTE_TO_BINARY(reg));
  huart2print(uart_buf, strlen(uart_buf));
#endif
  if (enable)
  { // Sleep: set the SLEEP bit
    reg |= BIT_SLEEP;
  }
  else
  { // Wake up: clear the SLEEP bit
    reg &= ~BIT_SLEEP; /* this was the provided code */
  }
#if PRINTF_APP_IMU
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_imu] New reg: %02X , 0b" BYTE_TO_BINARY_PATTERN "\r\n", (unsigned int) xTaskGetTickCount(), reg, BYTE_TO_BINARY(reg));
  huart2print(uart_buf, strlen(uart_buf));
#endif
  //ICM_20948_registerWrite(ICM_20948_REG_PWR_MGMT_1, reg);
  select_userbank(ub_0);
  spi_master_write_register(REG_PWR_MGMT_1, 1, &reg);
//  if (enable)
//  {
//    inv_icm20948_set_chip_power_state(&self->icm20948_states, CHIP_LP_ENABLE, 1);
//    inv_icm20948_sleep_mems(&self->icm20948_states);
//  }
//  else
//  {
//    inv_icm20948_wakeup_mems(&self->icm20948_states);
//  }
  select_userbank(ub_0);
  spi_master_read_register(REG_PWR_MGMT_1, 1, &reg);
#if PRINTF_APP_IMU
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_imu] PWR_MGMT_1 register at end of sleepMode setting: %02X, 0b" BYTE_TO_BINARY_PATTERN "\r\n",
		  (unsigned int) xTaskGetTickCount(), reg, BYTE_TO_BINARY(reg));
  huart2print(uart_buf, strlen(uart_buf));
  npf_snprintf(uart_buf, 200, ".\r\n");
#endif
  if (enable)
  {
#if PRINTF_APP_IMU
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_imu] Sleep mode ON \r\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
  else
  {
#if PRINTF_APP_IMU
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_imu] Sleep mode OFF \r\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }

}

fixed_point_t float_to_fixed_euler(float input)
{
  return (fixed_point_t)(input * (1 << FIXED_POINT_FRACTIONAL_BITS_EULER));
  // return (fixed_point_t)(round(input * (1 << FIXED_POINT_FRACTIONAL_BITS)));
}

fixed_point_t float_to_fixed_quat(float input)
{
    return (fixed_point_t)(input * (1 << FIXED_POINT_FRACTIONAL_BITS_QUAT));
    // return (fixed_point_t)(round(input * (1 << FIXED_POINT_FRACTIONAL_BITS)));
}

void imu_config_fsr_gyro(int fsr_in)
{
  int rc = 0;

  inv_sensor_config_fsr_t fsr;
  fsr.fsr = (uint32_t) fsr_in;

  // Read previously configured FSR
  inv_sensor_config_fsr_t temp_fsr;
  inv_device_get_sensor_config(device, INV_SENSOR_TYPE_GYROSCOPE, INV_SENSOR_CONFIG_FSR, &temp_fsr, sizeof(temp_fsr));
  check_rc(rc);
#if PRINTF_APP_IMU
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_imu] Sensor GYRO FSR (current): %ld \r\n",(unsigned int) xTaskGetTickCount(), temp_fsr.fsr);
  huart2print(uart_buf, strlen(uart_buf));
  npf_snprintf(uart_buf, 200, ".\r\n");
#endif

  if(fsr.fsr != temp_fsr.fsr)
  {
    inv_device_set_sensor_config(device, INV_SENSOR_TYPE_GYROSCOPE, INV_SENSOR_CONFIG_FSR, &fsr, sizeof(fsr));
    check_rc(rc);
#if PRINTF_APP_IMU
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_imu] Sensor GYRO FSR (update) set to: %ld \r\n",(unsigned int) xTaskGetTickCount(), fsr.fsr);
    huart2print(uart_buf, strlen(uart_buf));
    npf_snprintf(uart_buf, 200, ".\r\n");
#endif
  }
  else
  {
#if PRINTF_APP_IMU
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_imu] Sensor GYRO FSR (already) set to: %ld \r\n",(unsigned int) xTaskGetTickCount(), temp_fsr.fsr);
    huart2print(uart_buf, strlen(uart_buf));
    npf_snprintf(uart_buf, 200, ".\r\n");
#endif
  }

	// Read previously configured FSR
  inv_device_get_sensor_config(device, INV_SENSOR_TYPE_GYROSCOPE, INV_SENSOR_CONFIG_FSR, &temp_fsr, sizeof(temp_fsr));
  check_rc(rc);
#if PRINTF_APP_IMU
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_imu] Sensor GYRO FSR (current): %ld \r\n",(unsigned int) xTaskGetTickCount(), temp_fsr.fsr);
  huart2print(uart_buf, strlen(uart_buf));
  npf_snprintf(uart_buf, 200, ".\r\n");
#endif

}

void imu_config_fsr_accel(int fsr_in)
{
  int rc = 0;

  inv_sensor_config_fsr_t fsr;
  fsr.fsr = (uint32_t) fsr_in;

  // Read previously configured FSR
  inv_sensor_config_fsr_t temp_fsr;
  inv_device_get_sensor_config(device, INV_SENSOR_TYPE_ACCELEROMETER, INV_SENSOR_CONFIG_FSR, &temp_fsr, sizeof(temp_fsr));
  check_rc(rc);
#if PRINTF_APP_IMU
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_imu] Sensor ACCEL FSR (current): %lu \r\n",(unsigned int) xTaskGetTickCount(), temp_fsr.fsr);
  huart2print(uart_buf, strlen(uart_buf));
  npf_snprintf(uart_buf, 200, ".\r\n");
#endif

  if(fsr.fsr != temp_fsr.fsr)
  {
    inv_device_set_sensor_config(device, INV_SENSOR_TYPE_ACCELEROMETER, INV_SENSOR_CONFIG_FSR, &fsr, sizeof(fsr));
    check_rc(rc);
#if PRINTF_APP_IMU
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_imu] SSensor ACCEL FSR (update) set to: %lu \r\n",(unsigned int) xTaskGetTickCount(), fsr.fsr);
    huart2print(uart_buf, strlen(uart_buf));
    npf_snprintf(uart_buf, 200, ".\r\n");
#endif
  }
  else
  {
#if PRINTF_APP_IMU
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_imu] Sensor ACCEL FSR (already) set to: %lu \r\n",(unsigned int) xTaskGetTickCount(), temp_fsr.fsr);
    huart2print(uart_buf, strlen(uart_buf));
    npf_snprintf(uart_buf, 200, ".\r\n");
#endif
  }

  // Read previously configured FSR
  inv_device_get_sensor_config(device, INV_SENSOR_TYPE_ACCELEROMETER, INV_SENSOR_CONFIG_FSR, &temp_fsr, sizeof(temp_fsr));
  check_rc(rc);
#if PRINTF_APP_IMU
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_imu] Sensor ACCEL FSR (current): %lu\r\n",(unsigned int) xTaskGetTickCount(), temp_fsr.fsr);
  huart2print(uart_buf, strlen(uart_buf));
  npf_snprintf(uart_buf, 200, ".\r\n");
#endif
}

void calibration_callback()
{
//  ret_code_t err_code;
  int rc = 0;
//  magAccuracy = 3; // ONLY FOR TEST PURPOSES!!!
//  accelAccuracy = 3; // ONLY FOR TEST PURPOSES!!!
#if PRINTF_APP_IMU
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_imu] Calibration callback\r\n",(unsigned int) xTaskGetTickCount());
  huart2print(uart_buf, strlen(uart_buf));
  npf_snprintf(uart_buf, 200, ".\r\n");
#endif

//  gyroAccuracy = inv_icm20948_get_gyro_accuracy();
//  accelAccuracy = inv_icm20948_get_accel_accuracy();
//  magAccuracy = inv_icm20948_get_mag_accuracy();

#if PRINTF_APP_IMU
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_imu] gyroAccuracy = %d, accelAccuracy = %d, magAccuracy = %d. \r\n",(unsigned int) xTaskGetTickCount(), gyroAccuracy, accelAccuracy, magAccuracy);
  huart2print(uart_buf, strlen(uart_buf));
  npf_snprintf(uart_buf, 200, ".\r\n");
#endif

// Set LED blink according to calibration scheme
// Blink slow when starting -> 1s
// Blink faster when gyro is calibrated -> 500ms
// Blink faster when accel is callibrated -> 200ms
// Full led when mag is callibrated -> 0ms

  if (gyroAccuracy != 3 && accelAccuracy != 3 && magAccuracy != 3)
  {
#if PRINTF_APP_IMU
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_imu] Start timer 1s\r\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
#endif

#if PRINTF_APP_IMU
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_imu] IMU NOT CALIBRATED\r\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
    npf_snprintf(uart_buf, 200, ".\r\n");
#endif

// Start timer with period of 1 sec
//start_calibration_timer(1000);
//todo check if correct!!!

    ledFreq = 1000; // led off time 1000ms, on time 200ms, start of Gyroscope calibration
    gyroCalibrated = 0;
    accelCalibrated = 0;
    magCalibrated = 0;

//send_calibration(true, false, false, false);
  }
  else
  {
    if (gyroAccuracy == 3 && accelAccuracy == 3 && magAccuracy == 3)
    {
#if PRINTF_APP_IMU
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_imu] Switch off Led, IMU FULLY CALIBRATED.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
#endif
//stop_calibration_timer();
//send_calibration(false, true, true, true);
      ledFreq = 0; // led off, calibration done
      gyroCalibrated = 1;
      accelCalibrated = 1;
      magCalibrated = 1;

      HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
// Stop IMU
//ble_tms_config_t temp;
//memset(&temp, 0, sizeof(temp)); // Reset values to zero -> Turn off all sensors
//err_code = imu_enable_sensors(&temp);
//APP_ERROR_CHECK(err_code);
#if PRINTF_APP_IMU
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_imu] Stop gyroscope.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
#endif
      rc += inv_device_stop_sensor(device, INV_SENSOR_TYPE_GYROSCOPE);
      check_rc(rc);
#if PRINTF_APP_IMU
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_imu] Stop accelerometer.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
#endif
      rc += inv_device_stop_sensor(device, INV_SENSOR_TYPE_ACCELEROMETER);
      check_rc(rc);
#if PRINTF_APP_IMU
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_imu] Stop magnetometer.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
#endif
      rc += inv_device_stop_sensor(device, INV_SENSOR_TYPE_MAGNETOMETER);
      check_rc(rc);
// Reset scale to user values
      imu_config_fsr_gyro(PLANTSENSOR_GRYO_FSR);
      imu_config_fsr_accel(PLANTSENSOR_ACCEL_FSR);
      HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
      calibrationActive = 0;
// Reset accuracy when fully calibrated
// imu_data.gyro_accuracy = 0;
// imu_data.accel_accuracy = 0;
// imu_data.mag_accuracy = 0;
// gyroAccuracy = 0;
// accelAccuracy = 0;
// magAccuracy = 0;
    }
    else
    {
      if (gyroAccuracy == 3 && accelAccuracy == 3)
      {
#if PRINTF_APP_IMU
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_imu] Start timer 200ms\r\n",(unsigned int) xTaskGetTickCount());
        huart2print(uart_buf, strlen(uart_buf));
#endif
#if PRINTF_APP_IMU
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_imu] ACCELEROMETER CALIBRATED\r\n",(unsigned int) xTaskGetTickCount());
        huart2print(uart_buf, strlen(uart_buf));
        npf_snprintf(uart_buf, 200, ".\r\n");
#endif
//stop_calibration_timer();
        ledFreq = 200; // led off time 200ms, on time 200ms: start of Magnetometer calibration
//start_calibration_timer(200);
//send_calibration(false, true, true, false);
        gyroCalibrated = 1;
        accelCalibrated = 1;
        magCalibrated = 0;
        calibrationActive = 1;
#if PRINTF_APP_IMU
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_imu] Start MAGNETOMETER for calibration\r\n",(unsigned int) xTaskGetTickCount());
        huart2print(uart_buf, strlen(uart_buf));
        npf_snprintf(uart_buf, 200, ".\r\n");
#endif
        rc += inv_device_ping_sensor(device, INV_SENSOR_TYPE_MAGNETOMETER);
        check_rc(rc);
        rc += inv_device_set_sensor_period(device, INV_SENSOR_TYPE_MAGNETOMETER, IMU_DEFAULT_SAMPL_FREQ);
        check_rc(rc);
        rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_MAGNETOMETER);
        check_rc(rc);
      }
      else
      {
        if (gyroAccuracy == 3)
        {
#if PRINTF_APP_IMU
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_imu] Start timer 500ms\r\n",(unsigned int) xTaskGetTickCount());
          huart2print(uart_buf, strlen(uart_buf));
#endif
#if PRINTF_APP_IMU
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_imu] GYROMETER CALIBRATED\r\n",(unsigned int) xTaskGetTickCount());
          huart2print(uart_buf, strlen(uart_buf));
          npf_snprintf(uart_buf, 200, ".\r\n");
#endif
//stop_calibration_timer();
//start_calibration_timer(500);
          ledFreq = 500;  // led off time 500ms, on time 200ms: start of Accelerometer calibration
//send_calibration(false, true, false, false);
          gyroCalibrated = 1;
          accelCalibrated = 0;
          magCalibrated = 0;
#if PRINTF_APP_IMU
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_imu] Start ACCELEROMETER for calibration\r\n",(unsigned int) xTaskGetTickCount());
          huart2print(uart_buf, strlen(uart_buf));
          npf_snprintf(uart_buf, 200, ".\r\n");
#endif
          rc += inv_device_ping_sensor(device, INV_SENSOR_TYPE_ACCELEROMETER);
    	  check_rc(rc);
    	  rc += inv_device_set_sensor_period(device, INV_SENSOR_TYPE_ACCELEROMETER, IMU_DEFAULT_SAMPL_FREQ);
    	  check_rc(rc);
    	  rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_ACCELEROMETER);
    	  check_rc(rc);
        } // endif (gyroAccuracy == 3)
      } // endif else, so not (gyroAccuracy == 3 && accelAccuracy == 3)
    } // endif else, so not (gyroAccuracy == 3 && accelAccuracy == 3 && magAccuracy == 3)
  } // endif else, so not (gyroAccuracy != 3 && accelAccuracy != 3 && magAccuracy != 3)
}

int reset_DMP (void *context)
{

  inv_device_icm20948_t * self = (inv_device_icm20948_t *) context;
  struct inv_icm20948 * s = &self->icm20948_states;
  int result = 0;

  s->base_state.user_ctrl |= BIT_DMP_RST;
  result |= inv_icm20948_write_single_mems_reg(s, REG_USER_CTRL, s->base_state.user_ctrl);
	//result |= inv_icm20948_write_single_mems_reg(device, REG_USER_CTRL,
  //                               (saved_regs->user_ctrl & (~BIT_FIFO_EN)) | BIT_DMP_RST);
	inv_icm20948_sleep_us(DMP_RESET_TIME*1000);

  result |=inv_icm20948_set_dmp_address(s);
  result |=inv_icm20948_set_secondary(s);
  result |=inv_icm20948_setup_compass_akm(s);
  result |= inv_icm20948_sleep_mems(s);

return result;
}

void PollImuDevice(void)
{
  inv_device_poll(device); // Poll device for data
}
