/*
 * app_init.c
 *
 *            Created on: Oct 22, 2022
 *  Adapted for STM32WBA: Oct 23, 2023
 *                Author: Sarah Goossens
 */

/* Standard libraries */
#include <string.h>
#include <time.h>
#include <stdio.h>

/* Own header */
#include "main.h"
#include "app_init.h"
#include "app_led.h"
//#include "app_ranging.h"
#include "tim.h"
#include "stm32wbaxx_hal_flash.h"

#if PCF2131USED
#include "../Drivers/PCF2131/PCF2131.h"  // to declare bcd2bin
#include "app_rtc.h"
#include "app_rtc_sync.h"
#include "app_hal_sync.h"
#endif

#if PLANTSENSOR
#include "app_gateway.h"
#else
#include "app_network_connect.h"
#if SENSOR_HOST
  #include "app_gnss.h"
#endif
#endif

#include "app_hal_pps.h"
#include "app_radio_available.h"
#include "app_node_gnsspps.h"
#include "app_imu.h"

#if SUPERCAPUSED
#include "app_supercap.h"
#include "app_adc.h"
#endif

#include "../Drivers/SX1280/SX1280.h"
#include "usart.h"                       // to declare huart1

#define PRINTF_APP_INIT      1
#define PRINTF_APP_INIT_ID   0
#define PRINTF_APP_INIT_TIM2 0  // to print the registers of TIM2


extern module            moduleTable[MAXNRMODULES];
extern char              uart_buf[200];
extern char              byteString[3];
extern uint64_t          timeStampInt;
//extern osThreadId        rangingThreadHandler;
extern uint32_t          schedulerStartTime;
extern SemaphoreHandle_t gnssEpochMutex;
extern SemaphoreHandle_t recEpochMutex;
extern SemaphoreHandle_t timeStampIntMutex;
extern BaseType_t        xHigherPriorityTaskWoken;
#if !STM32WBAUSED
extern SemaphoreHandle_t i2cGNSSMutex;
#endif
extern SemaphoreHandle_t CompTicksMutex;
extern RadioStatus_t     statusSX1280;
extern float             tickSpeedToReference;
extern uint32_t          LoRaDevAddrInit;

void InitThreadInit(void);
void InitThreadStart(void * params);

uint16_t YourID, MyID;
uint8_t  rangingMaster = 0; // 1 for master, 0 for slave
uint16_t r_count;
uint8_t  stay_quiet_mode;
RadioLoRaBandwidths_t bandwidth;

#if STM32WBAUSED
  TaskHandle_t initThreadHandler;
#else
  osThreadId initThreadHandler;
#endif


void ProjectInit(void)
{
  HAL_TIM_Base_Start(&htim2); // start the 16MHz timer to evaluate RTCPPS SubTicks
  /* Initialization of the common configuration */
  timeStampIntMutex = xSemaphoreCreateMutex(); // to guard shared resource timeStampInt
#if !PLANTSENSOR
  gnssEpochMutex    = xSemaphoreCreateMutex(); // to guard shared resource gnssEpoch (epoch from a gnss module on a host)
  recEpochMutex     = xSemaphoreCreateMutex(); // to guard shared resource received_epoch (epoch from a host to a node)
#endif
#if !STM32WBAUSED
  i2cGNSSMutex      = xSemaphoreCreateMutex(); // to guard shared resource i2cInUseGNSS
#endif
  CompTicksMutex    = xSemaphoreCreateMutex(); // to guard shared resource CompensateTicks
//  imuDataMutex = xSemaphoreCreateMutex(); // to guard shared variables: yaw, pitch and roll
  schedulerStartTime = HAL_GetTick();
//  schedulerStartTime++;  // add one more to align exactly (otherwise RTOS tick is one tick behind HAL tick)
  InitThreadInit();
}

void InitThreadInit(void)
{
#if STM32WBAUSED
  if (xTaskCreate ((TaskFunction_t)InitThreadStart, "InitThread", (configSTACK_DEPTH_TYPE)512, NULL, osPriorityRealtime, &initThreadHandler) != pdPASS)
  {
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_init] Could not be started.\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
  }
#else
  osThreadDef(InitThread, InitThreadStart, osPriorityNormal, 0, 128);
  initThreadHandler = osThreadCreate(osThread(InitThread), NULL);
#endif
}

void InitThreadStart(void * params)
{
#if PRINTF_APP_INIT
  waitToPrint();
#if SENSOR_HOST
  npf_snprintf(uart_buf, 200, "%u [init] Sensor HOST version %u.%u.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) VERSION,(unsigned int) SUBVERSION);
#else
#if PLANTSENSOR
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor version %u.%u.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) VERSION,(unsigned int) SUBVERSION);
#else
  npf_snprintf(uart_buf, 200, "%u [init] Sensor NODE version %u.%u.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) VERSION,(unsigned int) SUBVERSION);
#endif //PLANTSENSOR
#endif // SENSOR_HOST
  huart2print(uart_buf, strlen(uart_buf));

#if PLANTSENSOR
  waitToPrint();
#if myPlantSensor
  LoRaDevAddrInit = 0x260B1B80; //from original file
  npf_snprintf(uart_buf, 200, "%u [init] My Plant Sensor, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantSensor1
  LoRaDevAddrInit = 0x260BAA00;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor 1, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantSensor2
  LoRaDevAddrInit = 0x260B83A5; //260B1B80; //from original file
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor 2, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantSensor3
  LoRaDevAddrInit = 0x260BB657; //from original file
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor 3, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantSensor4
  LoRaDevAddrInit = 0x260BE244; //260B1B80; //from original file
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor 4, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
//set 1
#if plantsensor_s1_01
  LoRaDevAddrInit = 0x260B8764;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S1-01, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s1_02
  LoRaDevAddrInit = 0x260B7A8F;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S1-02, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s1_03
  LoRaDevAddrInit = 0x260BF8D3;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S1-03, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s1_04
  LoRaDevAddrInit = 0x260BD489;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S1-04, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s1_05
  LoRaDevAddrInit = 0x260BA1BC;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S1-05, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s1_06
  LoRaDevAddrInit = 0x260B91D4;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S1-06, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s1_07
  LoRaDevAddrInit = 0x260B4E21;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S1-07, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s1_08
  LoRaDevAddrInit = 0x260B71CB;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S1-08, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s1_09
  LoRaDevAddrInit = 0x260BE4E9;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S1-09, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s1_10
  LoRaDevAddrInit = 0x260BC643;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S1-10, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s1_11
  LoRaDevAddrInit = 0x260BA322;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S1-11, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s1_12
  LoRaDevAddrInit = 0x260B514C;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S1-12, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s1_13
  LoRaDevAddrInit = 0x260B6096;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S1-13, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s1_14
  LoRaDevAddrInit = 0x260BF336;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S1-14, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s1_15
  LoRaDevAddrInit = 0x260B440A;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S1-15, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
//set 2
#if plantsensor_s2_01
  LoRaDevAddrInit = 0x260BD10E;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S2-01, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s2_02
  LoRaDevAddrInit = 0x260B23F3;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S2-02, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s2_03
  LoRaDevAddrInit = 0x260B4BB2;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S2-03, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s2_04
  LoRaDevAddrInit = 0x260B8C8A;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S2-04, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s2_05
  LoRaDevAddrInit = 0x260B304B;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S2-05, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s2_06
  LoRaDevAddrInit = 0x260BFE32;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S2-06, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s2_07
  LoRaDevAddrInit = 0x260B4EA2;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S2-07, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s2_08
  LoRaDevAddrInit = 0x260BE59C;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S2-08, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s2_09
  LoRaDevAddrInit = 0x260B9044;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S2-09, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s2_10
  LoRaDevAddrInit = 0x260B1BA8;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S2-10, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s2_11
  LoRaDevAddrInit = 0x260B87FD;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S2-11, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s2_12
  LoRaDevAddrInit = 0x260BFF7F;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S2-12, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s2_13
  LoRaDevAddrInit = 0x260B4B53;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S2-13, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s2_14
  LoRaDevAddrInit = 0x260BCD20;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S2-14, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s2_15
  LoRaDevAddrInit = 0x260B93AA;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S2-15, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
//set 3
#if plantsensor_s3_01
  LoRaDevAddrInit = 0x260B97D2;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S3-01, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s3_02
  LoRaDevAddrInit = 0x260BCFA9;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S3-02, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s3_03
  LoRaDevAddrInit = 0x260BA56D;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S3-03, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s3_04
  LoRaDevAddrInit = 0x260B4CE1;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S3-04, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s3_05
  LoRaDevAddrInit = 0x260B89B3;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S3-05, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s3_06
  LoRaDevAddrInit = 0x260BB25D;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S3-06, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s3_07
  LoRaDevAddrInit = 0x260B89C4;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S3-07, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s3_08
  LoRaDevAddrInit = 0x260B276E;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S3-08, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s3_09
  LoRaDevAddrInit = 0x260B8495;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S3-09, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s3_10
  LoRaDevAddrInit = 0x260BAB7E;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S3-10, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s3_11
  LoRaDevAddrInit = 0x260B4F71;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S3-11, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s3_12
  LoRaDevAddrInit = 0x260B81BF;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S3-12, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s3_13
  LoRaDevAddrInit = 0x260BA4AD;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S3-13, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s3_14
  LoRaDevAddrInit = 0x260B7CBA;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S3-14, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s3_15
  LoRaDevAddrInit = 0x260B0F5D;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S3-15, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
//set 4
#if plantsensor_s4_01
  LoRaDevAddrInit = 0x260B3C1B;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S4-01, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s4_02
  LoRaDevAddrInit = 0x260BE04C;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S4-02, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s4_03
  LoRaDevAddrInit = 0x260B8032;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S4-03, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s4_04
  LoRaDevAddrInit = 0x260BA2BC;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S4-04, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s4_05
  LoRaDevAddrInit = 0x260B2810;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S4-05, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s4_06
  LoRaDevAddrInit = 0x260B3104;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S4-06, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s4_07
  LoRaDevAddrInit = 0x260B77FB;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S4-07, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s4_08
  LoRaDevAddrInit = 0x260BE17B;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S4-08, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s4_09
  LoRaDevAddrInit = 0x260BD596;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S4-09, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s4_10
  LoRaDevAddrInit = 0x260B316C;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S4-10, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s4_11
  LoRaDevAddrInit = 0x260B2E39;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S4-11, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s4_12
  LoRaDevAddrInit = 0x260B70B8;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S4-12, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s4_13
  LoRaDevAddrInit = 0x260B4A94;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S4-13, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s4_14
  LoRaDevAddrInit = 0x260B8AD1;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S4-14, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif
#if plantsensor_s4_15
  LoRaDevAddrInit = 0x260BFC3F;
  npf_snprintf(uart_buf, 200, "%u [init] Plant Sensor S4-15, LoRa Device Address: 0x%08X.\r\n",(unsigned int) HAL_GetTick(),(unsigned int) LoRaDevAddrInit);
#endif

  huart2print(uart_buf, strlen(uart_buf));
#endif //PLANTSENSOR

//  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, sizeof(uart_buf), 100);
#if PCF2131USED
  npf_snprintf(uart_buf, 200, "%u [init] External RTC is used.\r\n",(unsigned int) HAL_GetTick());
#else
  npf_snprintf(uart_buf, 200, "%u [init] No external RTC is used.\r\n",(unsigned int) HAL_GetTick());
#endif // PCF2131USED
  huart2print(uart_buf, strlen(uart_buf));
//  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, sizeof(uart_buf), 100);
#if PCF2131I2C
  npf_snprintf(uart_buf, 200, "%u [init] RTC works with I2C.\r\n",(unsigned int) HAL_GetTick());
#else
  npf_snprintf(uart_buf, 200, "%u [init] RTC works with SPI.\r\n",(unsigned int) HAL_GetTick());
#endif // PCF2131I2C
  huart2print(uart_buf, strlen(uart_buf));
//  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, sizeof(uart_buf), 100);
#endif // PRINTF_APP_INIT


  uint32_t regVal = 0;
  regVal = READ_REG(PWR->SR); // Wake Up and Interrupt pins
#if PRINTF_APP_INIT
  waitToPrint();
  npf_snprintf(uart_buf, 200, "[Init] PWR->SRSBF    = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
  huart2print(uart_buf, strlen(uart_buf));
#endif
  // check Bit 2: SBF: Standby flag. This bit is set by hardware when the device enters the Standby mode and the CPU restarts from its reset vector.
  if ((regVal & PWR_SR_SBF_Msk) == PWR_SR_SBF_Msk)
  {
#if PRINTF_APP_INIT
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[Init] PWR->SRSBF: the device is starting from standby mode.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
    //todo if STANDBY mode will be implemented, a different start up scenario needs to be implemented
  }
  else
  {
#if PRINTF_APP_INIT
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[Init] PWR->SRSBF: the device is starting from a hardware reset.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }

  // if plantsensor, the IWDG needs to reset after 7 minutes, this is standard configured to prescaler 4 = (1/62.5Hz) x 4096 = 65.536s



  // check if a variable stays in flash after IWDG reset:



//  uint32_t flashVariable[4] = {0,0,0,0};
//  uint32_t resistantVariable = 0;
//  resistantVariable = *((uint32_t *)0x080FE000); // first location of flash page 127
//  char* demo="Sensor Plant_movement_sensor";
//#if PRINTF_APP_INIT
//  waitToPrint();
//  npf_snprintf(uart_buf, 200, "[Init] resistantVariable = 0x%08X.\r\n", (unsigned int) resistantVariable);
//  huart2print(uart_buf, strlen(uart_buf));
//#endif
//  flashVariable[0] = 0x01234567;
//  flashVariable[1] = 0x89ABCDEF;
//  flashVariable[2] = 0xFEDCBA98;
//  flashVariable[3] = 0x76543210;
//
//  FLASH_EraseInitTypeDef flashEraseData;
//  flashEraseData.TypeErase = FLASH_TYPEERASE_PAGES;
//  flashEraseData.Page = 127;
//  flashEraseData.NbPages = 1;
//  uint32_t flashErasePageError = 0;
//
//  // write value(s) to this location:
//  HAL_FLASH_Unlock();
//  HAL_FLASHEx_Erase(&flashEraseData, &flashErasePageError);
////  HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, 0x080FE000, demo);
////  HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, 0x080FE000, (uint32_t) &resistantVariable);
//  HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, 0x080FE000, (uint32_t) &flashVariable);
//  HAL_FLASH_Lock();

#if STARTWITHEMPTYFLASH
    FLASH_EraseInitTypeDef flashEraseData;
    flashEraseData.TypeErase = FLASH_TYPEERASE_PAGES;
    flashEraseData.Page = 127;
    flashEraseData.NbPages = 1;
    uint32_t flashErasePageError = 0;

    // write value(s) to this location:
    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&flashEraseData, &flashErasePageError);
    HAL_FLASH_Lock();
#endif


  uint8_t    buf[5];
  uint8_t    rxbuf[5];
  uint16_t   testID;
  uint32_t   lSerNum;
  uint32_t   sertemp;
  uint8_t    IDreceived;
//  TickType_t xLastWakeTime = xTaskGetTickCount();
//  uint32_t ulInterruptStatus;
  char       printchar;
  uint32_t   notificationValue = 0;
//  uint32_t   superCapCounter = 0;
//  uint32_t   testValue = 0xF0F0F0F0;

  float      tickTime                     = 0;
  float      tickTimeReference            = 0.001;

  tickTime = 1/(float)configTICK_RATE_HZ;
  tickSpeedToReference = tickTimeReference / tickTime;

#if PRINTF_APP_INIT
  waitToPrint();
  if (tickSpeedToReference == 1)
  {
    npf_snprintf(uart_buf, 200, "%u [Init] TickRate = %uHz, 1 Tick = 1ms (= reference value).\r\n",(unsigned int) xTaskGetTickCount(), (unsigned int) configTICK_RATE_HZ);

  }
  else
  {
    npf_snprintf(uart_buf, 200, "%u [Init] TickRate = %uHz, 1 Tick = %fs, %f times faster than the reference of 1ms.\r\n",
      (unsigned int) xTaskGetTickCount(), (unsigned int) configTICK_RATE_HZ, tickTime, tickSpeedToReference);
  }
  huart2print(uart_buf, strlen(uart_buf));
#endif


  // show settings of TIM2:
//  __IO uint32_t CR1;         /*!< TIM control register 1,                   Address offset: 0x00 */
//  __IO uint32_t CR2;         /*!< TIM control register 2,                   Address offset: 0x04 */
//  __IO uint32_t SMCR;        /*!< TIM slave mode control register,          Address offset: 0x08 */
//  __IO uint32_t DIER;        /*!< TIM DMA/interrupt enable register,        Address offset: 0x0C */
//  __IO uint32_t SR;          /*!< TIM status register,                      Address offset: 0x10 */
//  __IO uint32_t EGR;         /*!< TIM event generation register,            Address offset: 0x14 */
//  __IO uint32_t CCMR1;       /*!< TIM capture/compare mode register 1,      Address offset: 0x18 */
//  __IO uint32_t CCMR2;       /*!< TIM capture/compare mode register 2,      Address offset: 0x1C */
//  __IO uint32_t CCER;        /*!< TIM capture/compare enable register,      Address offset: 0x20 */
//  __IO uint32_t CNT;         /*!< TIM counter register,                     Address offset: 0x24 */
//  __IO uint32_t PSC;         /*!< TIM prescaler,                            Address offset: 0x28 */
//  __IO uint32_t ARR;         /*!< TIM auto-reload register,                 Address offset: 0x2C */
//  __IO uint32_t RCR;         /*!< TIM repetition counter register,          Address offset: 0x30 */
//  __IO uint32_t CCR1;        /*!< TIM capture/compare register 1,           Address offset: 0x34 */
//  __IO uint32_t CCR2;        /*!< TIM capture/compare register 2,           Address offset: 0x38 */
//  __IO uint32_t CCR3;        /*!< TIM capture/compare register 3,           Address offset: 0x3C */
//  __IO uint32_t CCR4;        /*!< TIM capture/compare register 4,           Address offset: 0x40 */
//  __IO uint32_t BDTR;        /*!< TIM break and dead-time register,         Address offset: 0x44 */
//  __IO uint32_t CCR5;        /*!< TIM capture/compare register 5,           Address offset: 0x48 */
//  __IO uint32_t CCR6;        /*!< TIM capture/compare register 6,           Address offset: 0x4C */
//  __IO uint32_t CCMR3;       /*!< TIM capture/compare mode register 3,      Address offset: 0x50 */
//  __IO uint32_t DTR2;        /*!< TIM deadtime register 2,                  Address offset: 0x54 */
//  __IO uint32_t ECR;         /*!< TIM encoder control register,             Address offset: 0x58 */
//  __IO uint32_t TISEL;       /*!< TIM Input Selection register,             Address offset: 0x5C */
//  __IO uint32_t AF1;         /*!< TIM alternate function option register 1, Address offset: 0x60 */
//  __IO uint32_t AF2;         /*!< TIM alternate function option register 2, Address offset: 0x64 */
//  __IO uint32_t OR;          /*!< TIM option register,                      Address offset: 0x68 */
//       uint32_t RESERVED0[220];/*!< Reserved,                               Address offset: 0x68-0x3D8 */
//  __IO uint32_t DCR;         /*!< TIM DMA control register,                 Address offset: 0x3DC */
//  __IO uint32_t DMAR;        /*!< TIM DMA address for full transfer,        Address offset: 0x3E0 */

#if PRINTF_APP_INIT_TIM2
  regVal = READ_REG(TIM2->CR1);
  waitToPrint();
  npf_snprintf(uart_buf, 200, "[Init] TIM2->CR1    = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
  huart2print(uart_buf, strlen(uart_buf));
  regVal = READ_REG(TIM2->CR2);
  waitToPrint();
  npf_snprintf(uart_buf, 200, "[Init] TIM2->CR2    = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
  huart2print(uart_buf, strlen(uart_buf));
  regVal = READ_REG(TIM2->SR);
  waitToPrint();
  npf_snprintf(uart_buf, 200, "[Init] TIM2->SR     = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
  huart2print(uart_buf, strlen(uart_buf));
  regVal = READ_REG(TIM2->EGR);
  waitToPrint();
  npf_snprintf(uart_buf, 200, "[Init] TIM2->EGR    = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
  huart2print(uart_buf, strlen(uart_buf));
  regVal = READ_REG(TIM2->CCMR1);
  waitToPrint();
  npf_snprintf(uart_buf, 200, "[Init] TIM2->CCMR1  = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
  huart2print(uart_buf, strlen(uart_buf));
  regVal = READ_REG(TIM2->CCMR2);
  waitToPrint();
  npf_snprintf(uart_buf, 200, "[Init] TIM2->CCMR2  = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
  huart2print(uart_buf, strlen(uart_buf));
  regVal = READ_REG(TIM2->CCMR3);
  waitToPrint();
  npf_snprintf(uart_buf, 200, "[Init] TIM2->CCMR3  = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
  huart2print(uart_buf, strlen(uart_buf));
  regVal = READ_REG(TIM2->CCER);
  waitToPrint();
  npf_snprintf(uart_buf, 200, "[Init] TIM2->CCER   = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
  huart2print(uart_buf, strlen(uart_buf));
  regVal = READ_REG(TIM2->PSC);
  waitToPrint();
  npf_snprintf(uart_buf, 200, "[Init] TIM2->PSC    = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
  huart2print(uart_buf, strlen(uart_buf));
  regVal = READ_REG(TIM2->ARR);
  waitToPrint();
  npf_snprintf(uart_buf, 200, "[Init] TIM2->ARR    = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
  huart2print(uart_buf, strlen(uart_buf));
  regVal = READ_REG(TIM2->RCR);
  waitToPrint();
  npf_snprintf(uart_buf, 200, "[Init] TIM2->RCR    = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
  huart2print(uart_buf, strlen(uart_buf));
  regVal = READ_REG(TIM2->CCR1);
  waitToPrint();
  npf_snprintf(uart_buf, 200, "[Init] TIM2->CCR1   = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
  huart2print(uart_buf, strlen(uart_buf));
  regVal = READ_REG(TIM2->CCR2);
  waitToPrint();
  npf_snprintf(uart_buf, 200, "[Init] TIM2->CCR2   = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
  huart2print(uart_buf, strlen(uart_buf));
  regVal = READ_REG(TIM2->CCR3);
  waitToPrint();
  npf_snprintf(uart_buf, 200, "[Init] TIM2->CCR3   = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
  huart2print(uart_buf, strlen(uart_buf));
  regVal = READ_REG(TIM2->CCR4);
  waitToPrint();
  npf_snprintf(uart_buf, 200, "[Init] TIM2->CCR4   = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
  huart2print(uart_buf, strlen(uart_buf));
  regVal = READ_REG(TIM2->CCR5);
  waitToPrint();
  npf_snprintf(uart_buf, 200, "[Init] TIM2->CCR5   = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
  huart2print(uart_buf, strlen(uart_buf));
  regVal = READ_REG(TIM2->CCR6);
  waitToPrint();
  npf_snprintf(uart_buf, 200, "[Init] TIM2->CCR6   = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
  huart2print(uart_buf, strlen(uart_buf));
  regVal = READ_REG(TIM2->BDTR);
  waitToPrint();
  npf_snprintf(uart_buf, 200, "[Init] TIM2->BDTR   = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
  huart2print(uart_buf, strlen(uart_buf));
  regVal = READ_REG(TIM2->ECR);
  waitToPrint();
  npf_snprintf(uart_buf, 200, "[Init] TIM2->ECR    = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
  huart2print(uart_buf, strlen(uart_buf));
  regVal = READ_REG(TIM2->TISEL);
  waitToPrint();
  npf_snprintf(uart_buf, 200, "[Init] TIM2->TISEL  = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
  huart2print(uart_buf, strlen(uart_buf));
  regVal = READ_REG(TIM2->AF1);
  waitToPrint();
  npf_snprintf(uart_buf, 200, "[Init] TIM2->AF1    = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
  huart2print(uart_buf, strlen(uart_buf));
  regVal = READ_REG(TIM2->AF2);
  waitToPrint();
  npf_snprintf(uart_buf, 200, "[Init] TIM2->AF2    = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
  huart2print(uart_buf, strlen(uart_buf));
  regVal = READ_REG(TIM2->OR);
  waitToPrint();
  npf_snprintf(uart_buf, 200, "[Init] TIM2->OR     = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
  huart2print(uart_buf, strlen(uart_buf));

#endif

  xTaskCatchUpTicks((TickType_t) schedulerStartTime); // align RTOS tick with HAL tick

#if PRINTF_APP_INIT
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u %u [init] Started.\r\n",(unsigned int) xTaskGetTickCount(), (unsigned int)xTaskGetTickCount());
  huart2print(uart_buf, strlen(uart_buf));
#endif

  HAL_GPIO_WritePin(CS_SX1280_GPIO_Port,   CS_SX1280_Pin,   GPIO_PIN_SET);   // Radio SX1280 chip select off
  HAL_GPIO_WritePin(CS_ICM20948_GPIO_Port, CS_ICM20948_Pin, GPIO_PIN_SET);   // IMU ICM-20948 chip select off
  HAL_GPIO_WritePin(CS_BME280_GPIO_Port,   CS_BME280_Pin,   GPIO_PIN_SET);   // environmental sensor BME280 chip select off
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port,      CS_RTC_Pin,      GPIO_PIN_SET);   // RTC PCF2131 chip select off
  //HAL_GPIO_WritePin(GNSS_WakeUp_GPIO_Port, GNSS_WakeUp_Pin, GPIO_PIN_RESET); // GNSS WakeUp, in case a GNSS module is connected
  HAL_GPIO_WritePin(GNSS_Reset_GPIO_Port,  GNSS_Reset_Pin,  GPIO_PIN_SET);   // GNSS Reset, in case a GNSS module is connected
  HAL_GPIO_WritePin(USER_LED_GPIO_Port,    USER_LED_Pin,    GPIO_PIN_SET); // LED off
  HAL_GPIO_WritePin(ANT_SELECT_GPIO_Port,  ANT_SELECT_Pin,  GPIO_PIN_RESET); // RESET = Lora, SET = Bleutooth
  HAL_GPIO_WritePin(VSW1V8_On_GPIO_Port,   VSW1V8_On_Pin,   GPIO_PIN_SET);   // switch 1V8 for IMU, BME280 and VEML6035 on

#if PRINTF_APP_INIT
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [Init] SPI select pins switched off, LED off, VSW on, Lora antenna selected.\r\n",(unsigned int) xTaskGetTickCount());
  huart2print(uart_buf, strlen(uart_buf));
#endif
#if !IMUTEST
  LedThreadInit();
#endif

  uint32_t whileIterations = 0; // to prevent lock into while loop
#if !IMUTEST
#if SUPERCAPUSED
  AdcThreadInit();
  vTaskDelay(1000U); // give the ADC some time to start up and determine the BATTIN voltage level.
  ScapThreadInit();
  vTaskDelay(1000U); // give the ScapThread some time to start up
  ScapThreadNotify(NOTIFICATION_LOAD_SCAP);     // first make sure that the supercap is loaded
  xTaskNotifyStateClear(initThreadHandler);
  notificationValue = 0;
  while ((notificationValue & NOTIFICATION_FROM_SCAP_SUPERCAP_READY) != NOTIFICATION_FROM_SCAP_SUPERCAP_READY)
  { // waiting for a notification value from the app_supercap that the super capacitor is loaded
    xTaskNotifyWait( 0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(30000));
    if ((notificationValue & NOTIFICATION_FROM_SCAP_SUPERCAP_READY) == NOTIFICATION_FROM_SCAP_SUPERCAP_READY)
    {
#if PRINTF_APP_INIT
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [Init] Super capacitor loaded. Ready to start.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
#endif
    }
    else
    {
      if (!notificationValue)
      {
#if PRINTF_APP_INIT
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [Init] No notification from supercap in the last 30s.\r\n",(unsigned int) xTaskGetTickCount());
        huart2print(uart_buf, strlen(uart_buf));
#endif
      }
      else
      {
#if PRINTF_APP_INIT
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [Init] Wrong notification value received to load supercap: 0x%08X.\r\n",(unsigned int) xTaskGetTickCount(), (unsigned int) notificationValue);
        huart2print(uart_buf, strlen(uart_buf));
#endif
      }
      xTaskNotifyStateClear(initThreadHandler);
      notificationValue = 0;
    }
    if (whileIterations++ > 15)
    {
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [Init] Error: Jump out of while loop waiting for a notification from supercap.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
      notificationValue = NOTIFICATION_FROM_SCAP_SUPERCAP_READY;
      // todo: check what is wrong...
    }
  }
#else // !SUPERCAPUSED
#if PRINTF_APP_INIT
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [Init] No Supercap used. Ready to start.\r\n",(unsigned int) xTaskGetTickCount());
  huart2print(uart_buf, strlen(uart_buf));
#endif
#endif // SUPERCAPUSED
#endif // !IMUTEST

//  HAL_GPIO_WritePin(SCAP_ON_GPIO_Port, SCAP_ON_Pin, GPIO_PIN_SET);   // Keep Supercap on during this process to avoid extra waiting time afterwards

  // Creation of unique ID:
  // Base address: 0x1FF80064: U_ID(95:88):    Unique ID [ 7: 0] (0x00)                                   NOT USED
  // Base address: 0x1FF80064: U_ID(87:80):    Unique ID [ 7: 0] Y-coordinate on wafer                  Y 0 -       255 --> max  8 bits
  // Base address: 0x1FF80064: U_ID(79:72):    Unique ID [ 7: 0] (0x00)                                   NOT USED
  // Base address: 0x1FF80064: U_ID(71:64):    Unique ID [ 7: 0] X-coordinate on wafer                  X 0 -       255 --> max  8 bits
  // Base address: 0x1FF80050: U_ID(31:24): Wafer number [ 7: 0] (unsigned 8 bit)                       W 0 -       255 --> max  8 bits
  // Base address: 0x1FF80050: U_ID(23:16):   Lot number [55:48] (ASCII code, letter) if A = 1, Z = 26  L 0 -        26 --> max  5 bits
  // Base address: 0x1FF80050: U_ID(15: 0):   Lot number [47:32] (ASCII code, 2 numbers 0 - 9)
  // Base address: 0x1FF80054: U_ID(63:32):   Lot number [31: 0] (ASCII code, 4 numbers 0 - 9)          N 0 -   999.999 --> max 20 bits - max 1111 0100 0010 0011 1111
  // Control Flag HOST: indication of Host = 1 or Node = 0                                            CFH                        1 bit
  //            UNIQUE ID STOPS HERE, THE REST ARE STATUS FLAGS
  // Control Flag CLKCHNG: RTC has been adapted since last message = 1. 0 if not adapted              CFC                        1 bit
  // Control Flag POSITKN: Known position                                                             CFP                        1 bit
  // Control Flag CONNECT: Node is connected to a host (is only active for a node)                    CFN                        1 bit
  // Control Flag HSTATUS: Health status issue = 1, no issue = 0                                      CFS                        1 bit
  // Control Flag SPARE01                                                                                                        1 bit
  // Control Flag SPARE02                                                                                                        1 bit
  //                                                                                                    -------------------------------
  //                                                                                                                        --> 56 bits (7 bytes)
  // To come to 64 bits, another 8 bits are added in the message. For the moment as SPARE.
  //
  //                                                                                                    0b1111 1111 1111 1111 1111 1111 1111 1111 1111 1111 1111 1111 1111 1111 1111 1111
  //                                                                                                      + SPARE + +-- Y --+ +-- X --+ +-- W --+ +--------- N ----------+ + L -++------+
  //                                                                                                                                                                   CCC CCCC
  //                                                                                                                                                                   FFF FFFF
  //                                                                                                                                                                     S NPCH
  //
  // (0b0000 1111 0100 0101 0101 0110) or (0x0F4556) --> 3 bytes for unique code with 4 leading bits available for extra information
  // (0b0000 1111 1111 1111 1111 1111) or (0x0FFFFF) = 1.048.575, so in fact another 47.785 values not used...
  //  --> shift to left +1 for HOST (HOST has always 1 for the least significant bit)
  //  --> shift to left for NODE    (NODE has always 0 for the least significant bit)
  //
  // Example:
  //
  //          +-------------------> Base address 0x1FF8 0064 U_ID[95:64]
  //          |        +----------> Base address 0x1FF8 0054 U_ID[63:32]
  //          |        |        +-> Base address 0x1FF8 0050 U_ID[31:00]
  //          |        |        |
  // 0x003C001C 33353833 07473034
  //     ||  || |||||||| ||||||||
  //     ||  || |||||||| ||||||++-> 5th Lot number:         4, becomes  40000
  //     ||  || |||||||| ||||++---> 6th Lot number:         0, becomes      0 = 43583 = 0x0AA3F = 0b0000 1010 1010 0011 1111
  //     ||  || |||||||| ||++-----> Letter Lot    :         G, becomes      7 = (Max Z = 0b1 1010) G or 7 -> 0b0000 0111 becomes 0b0000 0011 and 0b1000 0000
  //     ||  || |||||||| ++-------> Wafer number  :         7, stays        7 = 0b0001 1010 0x07 = 0b0000 0111
  //     ||  || ||||||++----------> 1st Lot number:         3, becomes      3
  //     ||  || ||||++------------> 2nd Lot number:         8, becomes     80
  //     ||  || ||++--------------> 3rd Lot number:         5, becomes    500
  //     ||  || ++----------------> 4th Lot number:         3, becomes   3000
  //     ||  ++-------------------> X-coordinate on wafer: 28, stays       28 = 0x1C = 0b0001 1100
  //     ++-----------------------> Y-coordinate on wafer: 60, stays       60 = 0x3C = 0b0011 1100
  //  suppose this is a host                                                  = 1 at the end
  //  suppose that the clock was not set                                      = 0
  //                                                                                                                                                                   CCC CCCC
  //                            <--------------------------- Unique ID ----------------------0000000X
  //                  + SPARE + +-- Y --+ +-- X --+ +-- W --+ +--------- N ----------+ + L -+--S NPCH
  //                0b0000 0000 0011 1100 0001 1100 0000 0111 0000 1010 1010 0011 1111 0011 1000 0001
  // moduleTable[0].ID      [7]       [6]       [5]       [4]       [3]       [2]       [1]       [0]
  //                0x   0    0    3    C    1    C    0    7    0    A    A    3    F    3    8    1
  //                0x003C1C070AA3F381
  //                    81F3A30A073C1C
#if STM32WBAUSED
  sertemp = *((uint32_t *)0x0BF90700); // for STM32WBA used
#if PRINTF_APP_INIT_ID
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_init] [initThread] Unique ID [95:64] @0x0BF90700: 0x%08X.\r\n", (unsigned int) xTaskGetTickCount(), (unsigned int)sertemp);
  huart2print(uart_buf, strlen(uart_buf));
#endif
#else
  sertemp   = *((uint32_t *)0x1FF80064); // for STM32 used
#if PRINTF_APP_INIT_ID
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_init] [initThread] Unique ID [95:64] @0x1FF80064: 0x%08X.\r\n", (unsigned int) xTaskGetTickCount(), (unsigned int)sertemp);
  huart2print(uart_buf, strlen(uart_buf));
#endif
#endif
  moduleTable[0].ID[6] = (uint8_t)(sertemp & 0xFF);                    // take Y-coordinate on wafer (BCD format)
#if PRINTF_APP_INIT_ID
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_init] [initThread] Y-coordinate converts to: 0x%02X.\r\n", (unsigned int) xTaskGetTickCount(), (unsigned int)moduleTable[0].ID[6]);
  huart2print(uart_buf, strlen(uart_buf));
#endif
  moduleTable[0].ID[5] = (uint8_t)((sertemp >> 16) & 0xFF);           // take X-coordinate on wafer (BCD format)
#if PRINTF_APP_INIT_ID
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_init] [initThread] X-coordinate converts to: 0x%02X.\r\n", (unsigned int) xTaskGetTickCount(), (unsigned int)moduleTable[0].ID[5]);
  huart2print(uart_buf, strlen(uart_buf));
#endif

#if STM32WBAUSED
  lSerNum = *((uint32_t *)0x0BF90704); // for STM32WBA used
#if PRINTF_APP_INIT_ID
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_init] [initThread] Unique ID [63:32] @0x0BF90704: 0x%08X.\r\n", (unsigned int) xTaskGetTickCount(), (unsigned int)lSerNum);
  huart2print(uart_buf, strlen(uart_buf));
#endif
#else
  lSerNum = *((uint32_t *)0x1FF80054);
#if PRINTF_APP_INIT_ID
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_init] [initThread] Unique ID [63:32] @0x1FF80054: 0x%08X.\r\n", (unsigned int) xTaskGetTickCount(), (unsigned int)lSerNum);
  huart2print(uart_buf, strlen(uart_buf));
#endif
#endif
  sertemp = (uint32_t)(lSerNum & 0xFF) - 0x30;                      // take 1st Lot number (ASCII format, 0x30 = 0)
#if PRINTF_APP_INIT_ID
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_init] [initThread] 1st Lot number: 0x%02X.\r\n", (unsigned int) xTaskGetTickCount(), (char)(lSerNum & 0xFF));
  huart2print(uart_buf, strlen(uart_buf));
#endif
  sertemp += (uint32_t)((((lSerNum >> 8) & 0xFF) - 0x30) * 10);      // take 2nd Lot number (ASCII format, 0x30 = 0)
#if PRINTF_APP_INIT_ID
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_init] [initThread] 2nd Lot number: 0x%02X.\r\n", (unsigned int) xTaskGetTickCount(), (char)((lSerNum >> 8) & 0xFF));
  huart2print(uart_buf, strlen(uart_buf));
#endif
  sertemp += (uint32_t)((((lSerNum >> 16) & 0xFF) - 0x30) * 100);     // take 3rd Lot number (ASCII format, 0x30 = 0)
#if PRINTF_APP_INIT_ID
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_init] [initThread] 3rd Lot number: 0x%02X.\r\n", (unsigned int) xTaskGetTickCount(), (char)((lSerNum >> 16) & 0xFF));
  huart2print(uart_buf, strlen(uart_buf));
#endif
  sertemp += (uint32_t)((((lSerNum >> 24) & 0xFF) - 0x30) * 1000);    // take 4th Lot number (ASCII format, 0x30 = 0)
#if PRINTF_APP_INIT_ID
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_init] [initThread] 4th Lot number: 0x%02X.\r\n", (unsigned int) xTaskGetTickCount(), (char)((lSerNum >> 24) & 0xFF));
  huart2print(uart_buf, strlen(uart_buf));
#endif
#if STM32WBAUSED
  lSerNum = *((uint32_t *)0x0BF90708);
#if PRINTF_APP_INIT_ID
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_init] [initThread] Unique ID [31:00] @0x0BF90708: 0x%08X.\r\n", (unsigned int) xTaskGetTickCount(), (unsigned int)lSerNum);
  huart2print(uart_buf, strlen(uart_buf));
#endif
#else
  lSerNum = *((uint32_t *)0x1FF80050);
#if PRINTF_APP_INIT_ID
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_init] [initThread] Unique ID[31:00] @0x1FF80050: 0x%08X.\r\n", (unsigned int) xTaskGetTickCount(), (unsigned int)lSerNum);
  huart2print(uart_buf, strlen(uart_buf));
#endif
#endif
  sertemp += (uint32_t)(((lSerNum & 0xFF) - 0x30) * 10000);          // take 5th Lot number (ASCII format, 0x30 = 0)
#if PRINTF_APP_INIT_ID
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_init] [initThread] 5th Lot number: 0x%02X.\r\n", (unsigned int) xTaskGetTickCount(), (char)(lSerNum & 0xFF));
  huart2print(uart_buf, strlen(uart_buf));
#endif
  sertemp += (uint32_t)((((lSerNum >> 8) & 0xFF) - 0x30) * 100000);  // take 6th Lot number (ASCII format, 0x30 = 0)
#if PRINTF_APP_INIT_ID
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_init] [initThread] 6th Lot number: 0x%02X.\r\n", (unsigned int) xTaskGetTickCount(), (char)((lSerNum >> 8) & 0xFF));
  huart2print(uart_buf, strlen(uart_buf));
#endif
#if PRINTF_APP_INIT_ID
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_init] [initThread] Full lot number: %d.\r\n", (unsigned int) xTaskGetTickCount(), (unsigned int)sertemp);
  huart2print(uart_buf, strlen(uart_buf));
#endif
  moduleTable[0].ID[3] = (uint8_t)((sertemp >> 12) & 0xFF); // take first 8 bits of lot number
  moduleTable[0].ID[2] = (uint8_t)((sertemp >>  4) & 0xFF); // take second 8 bits of lot number
  moduleTable[0].ID[1] = (uint8_t)((sertemp      ) & 0x0F); // take last 4 bits of lot number
  sertemp = (uint32_t)(lSerNum >> 16  & 0xFF) - 0x40;       // take letter of Lot number (ASCII format, 0x41 = A, so A = 1, Z = 26)
  moduleTable[0].ID[1] = moduleTable[0].ID[1] << 4;
  moduleTable[0].ID[1] |= (uint8_t)((sertemp >> 1) & 0x0F); // take highest 4 bits of lot nr character
#if PRINTF_APP_INIT_ID
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_init] [initThread] Letter Lot number: 0x%02X.\r\n", (unsigned int) xTaskGetTickCount(), (char)sertemp);
  huart2print(uart_buf, strlen(uart_buf));
#endif
  moduleTable[0].ID[0] = (uint8_t)(sertemp & 0x01);         // take least significant bit of lot nr character
  moduleTable[0].ID[0] = moduleTable[0].ID[0] << 7;         // shift 7 positions to the left
  moduleTable[0].ID[4] = (uint8_t)((lSerNum >> 24) & 0xFF); // take Wafer number (number 0-255)
#if PRINTF_APP_INIT_ID
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_init] [initThread] Wafer number: %d.\r\n", (unsigned int) xTaskGetTickCount(), (unsigned int)((lSerNum >> 24) & 0xFF));
  huart2print(uart_buf, strlen(uart_buf));
#endif
#if SENSOR_HOST
  moduleTable[0].ID[0] |= 0x01;
  moduleTable[0].type = 1;
#else
  moduleTable[0].type = 0;
#endif

// remove status bits from ID:
// moduleTable[0].ID[0] &= 0b10000001;

#if PRINTF_APP_INIT
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [Init] [initThread] Sensor Unique ID: ",(unsigned int) xTaskGetTickCount());
  for (unsigned int i = 0; i < 8; i++)
  {
    npf_snprintf(byteString, 3, "%02X", moduleTable[0].ID[i]);
    strcat(uart_buf, byteString);
  }
  strcat(uart_buf, ".\n");
  huart2print(uart_buf, strlen(uart_buf));
#endif


//  //build MyID from Unique device ID register @ Base address: 0x1FF80064 (STM32L073RXT6)
//  lSerNum = *((uint32_t *)0x1FF80064);
//  sertemp = (uint8_t)   lSerNum       & 0xFF;      //save 8 LSbits
//  MyID    = (uint16_t)((lSerNum >> 8) & 0xFF00);   //save 8 MSbits
//  MyID   |= sertemp;
//  //read EEPROM - if never initialized then do it here
//  testID = *(__IO uint32_t *)(MY_DATA_EEPROM_BASE + EEPROM_MYID);//Read 32-bit word
//#if PRINTF_APP_INIT
//  waitToPrint();
//  npf_snprintf(uart_buf, 200, "%u [app_init] [initThread] Serial number: %d (0x%08X), test ID: %d (0x%04X), MyID: %d (0x%04X).\r\n",
//      (unsigned int) xTaskGetTickCount(), (unsigned int)lSerNum, (unsigned int)lSerNum,
//      (unsigned int)testID, (unsigned int)testID, (unsigned int)MyID, (unsigned int)MyID);
//  huart2print(uart_buf, strlen(uart_buf));
//#endif
//  if(testID != MyID)
//  {
//    //init EEPROM storage to -not- paired and LORA
//    EepromWrite( MY_DATA_EEPROM_BASE + EEPROM_MYID, (uint32_t) MyID); //MyID
//#if PRINTF_APP_INIT
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[app_init] [initThread] Eeprom write MyID done.\r\n");
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//    EepromWrite( MY_DATA_EEPROM_BASE + EEPROM_YOURID, (uint32_t) 0x0000); //YourID
//#if PRINTF_APP_INIT
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[app_init] [initThread] Eeprom write YourNewID done.\r\n");
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//  }
//  //read startup values for ID of paired module and current modulating mode
//  MyID   = *(__IO uint32_t *)(MY_DATA_EEPROM_BASE + EEPROM_MYID);   //Read 32-bit word
//  YourID = *(__IO uint32_t *)(MY_DATA_EEPROM_BASE + EEPROM_YOURID); //Read 32-bit word
//#if PRINTF_APP_INIT
//  waitToPrint();
//  npf_snprintf(uart_buf, 200, "%u [app_init] [initThread] MyID: %d, YourID: %d.\r\n",(unsigned int) xTaskGetTickCount(), MyID, YourID);
//  huart2print(uart_buf, strlen(uart_buf));
//#endif


//  YourNewID  = (uint16_t)buf[0];
//  temp       = (uint16_t)buf[1];
//  YourNewID |= (temp << 8);
//  EepromWrite( MY_DATA_EEPROM_BASE + EEPROM_MYID, (uint32_t) MyID); //MyID
//#if PRINTF_APP_INIT
//  waitToPrint();
//  npf_snprintf(uart_buf, 200, "%u [app_init] [initThread] Eeprom write MyID done.\n\r",(unsigned int) xTaskGetTickCount());
//  huart2print(uart_buf, strlen(uart_buf));
//#endif
//  EepromWrite( MY_DATA_EEPROM_BASE + EEPROM_YOURID, (uint32_t) YourNewID); //Your new ID
//#if PRINTF_APP_INIT
//  waitToPrint();
//  npf_snprintf(uart_buf, 200, "%u [app_init] [initThread] Eeprom write YourNewID done.\n\r",(unsigned int) xTaskGetTickCount());
//  huart2print(uart_buf, strlen(uart_buf));
//#endif
//  YourID = *(__IO uint32_t *)(MY_DATA_EEPROM_BASE + EEPROM_YOURID);//Read 32-bit word
//#if PRINTF_APP_INIT
//  waitToPrint();
//  npf_snprintf(uart_buf, 200, "%u [app_init] [initThread] ID 1st device: %d, ID 2nd device: %d\n\r",(unsigned int) xTaskGetTickCount(), MyID, YourID);
//  huart2print(uart_buf, strlen(uart_buf));
//  waitToPrint();
//  npf_snprintf(uart_buf, 200, "%u [app_init] [initThread] Pairing done: MyID: %d, YourID: %d.\r\n",(unsigned int) xTaskGetTickCount(), MyID, YourID);
//  huart2print(uart_buf, strlen(uart_buf));
//#endif
//  //RangeTestMasterMode(MyID, YourID);
//#endif //SENSOR_HOST
//
//  rtos_ranging_thread_init();


#if PLANTSENSOR
#if PCF2131USED
  HalPPSThreadInit();
  RtcThreadInit();
  HalSyncThreadInit();
#else
  ImuThreadInit();
#endif
  whileIterations = 0; // to prevent lock into while loop
  xTaskNotifyStateClear(initThreadHandler);
  notificationValue = 0;
  while ((notificationValue & NOTIFICATION_FROM_IMU_INITIALIZED) != NOTIFICATION_FROM_IMU_INITIALIZED)
  { // waiting for a notification value from the app_supercap that the super capacitor is loaded
    xTaskNotifyWait( 0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(600000));
    if ((notificationValue & NOTIFICATION_FROM_IMU_INITIALIZED) == NOTIFICATION_FROM_IMU_INITIALIZED)
    {
#if PRINTF_APP_INIT
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [init] Notification received that IMU is initialized. Initialize radio.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
#endif
    }
    else
    {
      if (!notificationValue)
      {
#if PRINTF_APP_INIT
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [init] PLEASE FINISH IMU CALIBRATION PROCESS!!!.\r\n",(unsigned int) xTaskGetTickCount());
        huart2print(uart_buf, strlen(uart_buf));
#endif
      }
      else
      {
#if PRINTF_APP_INIT
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [init] Wrong notification value received during IMU calibration process: 0x%08X.\r\n",(unsigned int) xTaskGetTickCount(), (unsigned int) notificationValue);
        huart2print(uart_buf, strlen(uart_buf));
#endif

      }
      xTaskNotifyStateClear(initThreadHandler);
      notificationValue = 0;
    }
    if (whileIterations++ > 20)
    {
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [init] Error: Jump out of while loop waiting to finish IMU calibration process.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
      notificationValue = NOTIFICATION_FROM_IMU_INITIALIZED;
    }
  }

#if PRINTF_APP_INIT
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [init] IMU calibration process done.\r\n",(unsigned int) xTaskGetTickCount());
  huart2print(uart_buf, strlen(uart_buf));
#endif


  whileIterations = 0; // to prevent lock into while loop
  xTaskNotifyStateClear(initThreadHandler);
  notificationValue = 0;

  ScapThreadNotify(NOTIFICATION_LOAD_SCAP);     // first make sure that the supercap is loaded

  while ((notificationValue & NOTIFICATION_FROM_SCAP_SUPERCAP_READY) != NOTIFICATION_FROM_SCAP_SUPERCAP_READY)
  { // waiting for a notification value from the app_supercap that the super capacitor is loaded
    xTaskNotifyWait( 0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(30000));
    if ((notificationValue & NOTIFICATION_FROM_SCAP_SUPERCAP_READY) == NOTIFICATION_FROM_SCAP_SUPERCAP_READY)
    {
#if PRINTF_APP_INIT
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [init] Super capacitor loaded. Ready to start the radio.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
#endif
    }
    else
    {
      if (!notificationValue)
      {
#if PRINTF_APP_INIT
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [init] No notification from supercap in the last 30s.\r\n",(unsigned int) xTaskGetTickCount());
        huart2print(uart_buf, strlen(uart_buf));
#endif
      }
      else
      {
#if PRINTF_APP_INIT
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [init] Wrong notification value received to load supercap: 0x%08X.\r\n",(unsigned int) xTaskGetTickCount(), (unsigned int) notificationValue);
        huart2print(uart_buf, strlen(uart_buf));
#endif
      }
      xTaskNotifyStateClear(initThreadHandler);
      notificationValue = 0;
    }
    if (whileIterations++ > 15)
    {
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [init] Error: Jump out of while loop waiting for a notification from supercap.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
      notificationValue = NOTIFICATION_FROM_SCAP_SUPERCAP_READY;
      // todo: check what is wrong...
    }
  }
  SX1280Init();
  GoToStandby(0);
  vTaskDelay(200U);
  SX1280SetSleep();
  vTaskDelay(500U);
  GatewayThreadInit();
#if PRINTF_APP_INIT
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_init] [initThread] Suspended.\r\n",(unsigned int) xTaskGetTickCount());
  huart2print(uart_buf, strlen(uart_buf));
#endif
  vTaskSuspend(initThreadHandler);

#else
  SX1280Init();
  GoToStandby(0);
  vTaskDelay(200U);
  SX1280SetSleep();
  vTaskDelay(1000U);
#if !SENSOR_HOST
  HalPPSThreadInit();
#endif
  //20240725 moved to app_hal_sync when tickSpeedToReference is defined
  RtcThreadInit();
  //20240623 moved to RtcThreadInit(), just before endless loop:
  //RtcSyncThreadInit();

  //20240623 moved to app_hal_sync when HAL/RTOS is synchronized:
//#if SENSOR_HOST
//  GNSSThreadInit();
//#else
////  GNSSPPSThreadInit();
//#endif

  //20240623 moved to app_rtc_sync, just before endless loop:
  //HalSyncThreadInit(); // wait until RTC_PPS is working


#if STM32WBAUSED
#if PRINTF_APP_INIT
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [Init] Suspended.\r\n",(unsigned int) xTaskGetTickCount());
  huart2print(uart_buf, strlen(uart_buf));
#endif
  vTaskSuspend(initThreadHandler);
#else // !STM32WBAUSED
#if PRINTF_APP_INIT
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [Init] Terminated.\r\n",(unsigned int) xTaskGetTickCount());
  huart2print(uart_buf, strlen(uart_buf));
#endif
  osThreadTerminate(initThreadHandler);
#endif //STM32WBAUSED

#endif //PLANTSENSOR


  /* Infinite loop */
//  for(;;)
//  {
//    //HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
//
//    vTaskDelayUntil(&xLastWakeTime, 100000U);
//  }
}

void InitThreadNotify(uint32_t notValue)
{
  if (initThreadHandler != NULL)
  {
    xTaskNotify(initThreadHandler, notValue, eSetBits);
  }
}

void app_init_notify_fromISR(uint32_t notValue)
{
  if (initThreadHandler != NULL)
  {
    xTaskNotifyFromISR(initThreadHandler, notValue, eSetBits, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void app_DIO1_notify_from_isr(uint32_t notValue)
{
#if STM32WBAUSED
  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
#else
  if (osKernelRunning())
#endif
  {
//    BaseType_t xHigherPriorityTaskWoken;
//    xHigherPriorityTaskWoken = pdFALSE;
    if (initThreadHandler != NULL)
    {
      xTaskNotifyFromISR(initThreadHandler, notValue, eSetBits, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
}
