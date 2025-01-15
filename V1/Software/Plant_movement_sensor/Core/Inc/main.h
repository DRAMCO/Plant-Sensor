/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wbaxx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#include "../../Middlewares/nanoprintf/nanoprintf.h"
#include "time.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
  /* Module status variables -------------------------------------------------*/
  typedef struct
  {
    uint32_t  number;          // Sequence number of module connection. If nothing is filled in, this is an empty record.
                               // For a Host: all nodes which are connected to this host have a sequence number when they were connected.
                               //             This is also giving how many nodes are connected to the Host
                               // For a Node: The connected host has number 1.
                               //             All other Hosts in reach are also listed as spare Hosts to connect to when the own Host should fail
                               // This is used by the host to know when data will be received from this module
    char      ID[8];	       // unique ID of the connected module
    uint32_t  alias;           // unique ID of the connected module defined by the host to simplify the identification of his connected nodes
    uint8_t   type;            // host = 1, node = 0
    uint8_t   status;          // these are the 8 status bits received from the paired module (see definition of Control Flags in app_init.c)
    time_t    epochConnected;  // epoch time of first connection request
    time_t    epochLast;       // for a Node: epoch time of the last synchronization received from a host
                               // for a Host: epoch time of the last received data from this connected node
    uint8_t   synchronized;    // for a Host and a Node: is 1 when synchronized
    uint32_t  sequence;        // The sequence number of the module connection at the paired module. A host is giving this to a Node at the acknowledge step during pairing.
                               // This is used by the Node to know when data transmission needs to be done
                               // This is used by the Host to know when he needs to listen for data from a node
                               // If the sequence number is not filled in, it means that the module connection to a host failed acknowledge.
    float     hostfei;         // Frequency Error Indication of the last transmission from the host
    float     hostrssi;        // RSSI value of the last transmission from the host
    float     hostsnr;         // SNR value of the last transmission from the host
    float     nodefei;         // Frequency Error Indication of the last transmission from the node
    float     noderssi;        // RSSI value of the last transmission from the node
    float     nodesnr;         // SNR value of the last transmission from the node
  } module;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void huart2print(char buf[], uint8_t length);
void waitToPrint(void);
#if  PCF2131I2C
void IsI2C1Available(void);
#else
void IsSPI3Available(void);
#endif
void IsI2CAvailable(void);
/*!
 * @brief           Capture the system time in microseconds
 *
 * @return          system_current_time    current system timestamp in microseconds
 */
uint64_t get_timestamp_us();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GNSS_PPS_Pin GPIO_PIN_11
#define GNSS_PPS_GPIO_Port GPIOB
#define GNSS_PPS_EXTI_IRQn EXTI11_IRQn
#define SCAP_LEVEL_Pin GPIO_PIN_8
#define SCAP_LEVEL_GPIO_Port GPIOA
#define BATTIN_LEVEL_Pin GPIO_PIN_5
#define BATTIN_LEVEL_GPIO_Port GPIOA
#define BATT_LEVEL_Pin GPIO_PIN_3
#define BATT_LEVEL_GPIO_Port GPIOA
#define GNSS_Reset_Pin GPIO_PIN_10
#define GNSS_Reset_GPIO_Port GPIOB
#define CS_ICM20948_Pin GPIO_PIN_2
#define CS_ICM20948_GPIO_Port GPIOA
#define CS_RTC_Pin GPIO_PIN_1
#define CS_RTC_GPIO_Port GPIOA
#define CS_BME280_Pin GPIO_PIN_13
#define CS_BME280_GPIO_Port GPIOC
#define RTC_PPS_Pin GPIO_PIN_7
#define RTC_PPS_GPIO_Port GPIOB
#define RTC_PPS_EXTI_IRQn EXTI7_IRQn
#define RTC_NINTB_Pin GPIO_PIN_6
#define RTC_NINTB_GPIO_Port GPIOB
#define INT1_ICM20948_Pin GPIO_PIN_5
#define INT1_ICM20948_GPIO_Port GPIOB
#define INT1_ICM20948_EXTI_IRQn EXTI5_IRQn
#define RESET_SX1280_Pin GPIO_PIN_12
#define RESET_SX1280_GPIO_Port GPIOA
#define DIO2_SX1280_Pin GPIO_PIN_2
#define DIO2_SX1280_GPIO_Port GPIOB
#define DIO2_SX1280_EXTI_IRQn EXTI2_IRQn
#define DIO1_SX1280_Pin GPIO_PIN_1
#define DIO1_SX1280_GPIO_Port GPIOB
#define DIO1_SX1280_EXTI_IRQn EXTI1_IRQn
#define BUSY_SX1280_Pin GPIO_PIN_0
#define BUSY_SX1280_GPIO_Port GPIOB
#define BUSY_SX1280_EXTI_IRQn EXTI0_IRQn
#define ANT_SELECT_Pin GPIO_PIN_15
#define ANT_SELECT_GPIO_Port GPIOB
#define CS_SX1280_Pin GPIO_PIN_10
#define CS_SX1280_GPIO_Port GPIOA
#define SCAP_ON_Pin GPIO_PIN_9
#define SCAP_ON_GPIO_Port GPIOA
#define VSW1V8_On_Pin GPIO_PIN_14
#define VSW1V8_On_GPIO_Port GPIOB
#define USER_LED_Pin GPIO_PIN_13
#define USER_LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define VERSION             1
#define SUBVERSION          3     // version 3 release 20241115

#define SENSOR_HOST         0 // 0 0 0 if 1 = code for sensor host, if 0 = code for sensor node or plantsensor
#define PLANTSENSOR         1 // 1 1 1 if 1 = plantsensor specific code (put SENSOR_HOST to 0), otherwise PHD specific code
#define LISTENTOPLANTSENSOR 0 // 0 0 0 this is only valid if node (SENSOR_HOST = 0 and PLANTSENSOR = 0) and is to listen to a PLANTSENSOR to see if RF messages are being received
#define PCF2131USED         0 // 1 0 0 if 1 = external RTC is used, if 0 = no external RTC is used
#define PCF2131I2C          0 // 0 0 0 RTC works with I2C (1), or with SPI (0) (plant movement sensor)
#define STM32WBAUSED        1 // 1 1 1 if STM32WBA is used (plant movement sensor)
#define RANGECALIBRATION    0 // 0 0 0 if 1 = code to determine specific design delays to be placed into RxTxDelay register
#define ANTENNARSSITEST     0 // 0 0 0 to do specific tests to show the properties (eg RSS value) of communication between one host and one node
#define IMUTEST             0 // 0 0 0 if 1 = code reduced for IMU test purposes (no radio, no supercap, no RTC,...)
#define AUTOMATESUPERCAP    0 // 0 0 0 if 1 = supercap is loaded automatically when the value is below 1.82V
#define SUPERCAPUSED        1 // 0 0 1 if 1 = supercap is available on the PCB
#define TICKSYNCHRONIZATION 0 // 0 0 0 if 1, synchronization process is done on tick level. If 0 = synchronization process is done on us level
#define SLEEPTIME1MINUTE    0 // 1 1 0 if 1 = STM32 sleeptime is only 1 minute for test purposes. If 0, sleep time is 5 minutes
#define STARTWITHEMPTYFLASH 0 // 1 0 0 if 1 = the DMP FLASH will be ignored with a new start up. This is ONLY for test purposes
#define SENDGATEWAYMESSAGE  1 // 1 1 1 if 1 = send a message to the gateway (normal plantsensor setting). If 0, no transmission is being done, this is for test purposes only
#define KEEPCPUAWAKE        0 // 0 0 0 if 1 = do not put the CPU into sleep mode, this is for test purposes only. If 0, normal operation
#define TESTIWDG            0 // 0 0 0 if 1 = no refresh of IWDG will be given after 3 steps

#define myPlantSensor 0
#define plantSensor1  1
#define plantSensor2  0
#define plantSensor3  0
#define plantSensor4  0

#define plantsensor_s1_01 0  // delivered - updated to version 1.3
#define plantsensor_s1_02 0  // delivered
#define plantsensor_s1_03 0  // delivered
#define plantsensor_s1_04 0  // delivered
#define plantsensor_s1_05 0  // delivered
#define plantsensor_s1_06 0  // delivered
#define plantsensor_s1_07 0  // programmed, delivered second set
#define plantsensor_s1_08 0  // programmed, delivered second set
#define plantsensor_s1_09 0  // programmed, delivered second set
#define plantsensor_s1_10 0  // programmed, delivered second set
#define plantsensor_s1_11 0  // programmed, module 1 used 260BA322
#define plantsensor_s1_12 0  // programmed, module 23 used 260B514C
#define plantsensor_s1_13 0
#define plantsensor_s1_14 0
#define plantsensor_s1_15 0

#define plantsensor_s2_01 0  // delivered
#define plantsensor_s2_02 0  // delivered
#define plantsensor_s2_03 0  // delivered
#define plantsensor_s2_04 0  // delivered
#define plantsensor_s2_05 0  // delivered
#define plantsensor_s2_06 0  // delivered
#define plantsensor_s2_07 0  // programmed, delivered second set
#define plantsensor_s2_08 0  // programmed, delivered second set
#define plantsensor_s2_09 0  // programmed, delivered second set
#define plantsensor_s2_10 0  // programmed, delivered second set
#define plantsensor_s2_11 0  // programmed, module 12 used 260B87FD
#define plantsensor_s2_12 0  // programmed, module 11 used 260BFF7F
#define plantsensor_s2_13 0
#define plantsensor_s2_14 0
#define plantsensor_s2_15 0

#define plantsensor_s3_01 0  // delivered
#define plantsensor_s3_02 0  // delivered
#define plantsensor_s3_03 0  // delivered
#define plantsensor_s3_04 0  // delivered
#define plantsensor_s3_05 0  // delivered
#define plantsensor_s3_06 0  // delivered
#define plantsensor_s3_07 0  // programmed, delivered second set
#define plantsensor_s3_08 0  // programmed, delivered second set
#define plantsensor_s3_09 0  // programmed, delivered second set
#define plantsensor_s3_10 0  // programmed, delivered second set
#define plantsensor_s3_11 0  // programmed, module 13 used 260B4F71
#define plantsensor_s3_12 0  // programmed, module 10 used, nothing received, replaced with other module (not yet in box3)
#define plantsensor_s3_13 0
#define plantsensor_s3_14 0
#define plantsensor_s3_15 0

#define plantsensor_s4_01 0  // delivered
#define plantsensor_s4_02 0  // delivered
#define plantsensor_s4_03 0  // delivered
#define plantsensor_s4_04 0  // delivered
#define plantsensor_s4_05 0  // delivered
#define plantsensor_s4_06 0  // delivered
#define plantsensor_s4_07 0  // programmed, delivered second set
#define plantsensor_s4_08 0  // programmed, delivered second set
#define plantsensor_s4_09 0  // programmed, delivered second set
#define plantsensor_s4_10 0  // programmed, delivered second set
#define plantsensor_s4_11 0  // programmed, module 14 used 260B2E39
#define plantsensor_s4_12 0  // programmed, module 17 used
#define plantsensor_s4_13 0
#define plantsensor_s4_14 0
#define plantsensor_s4_15 0



#define NANOPRINTF_IMPLEMENTATION  // define nanoprintf configuration
#define MAXNRMODULES      0x0000000B // max number of connected modules

// app_network protocol commands
#define STARTDELIMITER    0x53    // "S" Message Start Delimiter
#define CONFIRMPAIRING    0x43    // "C" Confirm Pairing
#define ENDDELIMITER      0x45    // "E" Message End delimiter
#define HOSTISTHEREANYONE 0x50    // "P" pairing command, "Hello, is there anyone?"
#define NODEIAMHERE       0x59    // "Y" pairing ANSWER command, "Yes, I'm here!"

// sysTick defines
#define SYSTICK_CTRL_REG             ( *( ( volatile uint32_t * ) 0xe000e010 ) ) // for ARM Cortex M0, M33
#define SYSTICK_LOAD_REG             ( *( ( volatile uint32_t * ) 0xe000e014 ) ) // for ARM Cortex M0, M33 = reload value
#define SYSTICK_CURRENT_VALUE_REG    ( *( ( volatile uint32_t * ) 0xe000e018 ) ) // for ARM Cortex M0, M33 = current counter value
#define SYSTICK_ENABLE               0x00000001

/* Definitions of environment analog values */
/* Value of analog reference voltage (Vref+), connected to analog voltage   */
/* supply Vdda (unit: mV).                                                  */
//#define VBATT3300                       (3300UL) // in case of Battery: 3600UL, in case of Nucleo board or Silicon Labs: 3300UL
//#define VBATT3600                       (3600UL)
// above defined values have been changed using variable installedBatteryVoltage.
/* Init variable out of expected ADC conversion data range */
#define VAR_CONVERTED_DATA_INIT_VALUE    (__LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B) + 1)

// conversion of uint8_t to binary format
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')
//usage example:
//waitToPrint();
//npf_snprintf(uart_buf, 200, "[PCF2131] SEC_ALARM = 0x%02X 0b" BYTE_TO_BINARY_PATTERN "\r\n",pcf2131Buf[1], BYTE_TO_BINARY(pcf2131Buf[1]));
//huart2print(uart_buf, strlen(uart_buf));

// conversion of uint32_t to binary format
#define UINT32_T_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c"
#define UINT32_T_TO_BINARY(val)  \
  ((val) & 0x80000000 ? '1' : '0'), \
  ((val) & 0x40000000 ? '1' : '0'), \
  ((val) & 0x20000000 ? '1' : '0'), \
  ((val) & 0x10000000 ? '1' : '0'), \
  ((val) & 0x08000000 ? '1' : '0'), \
  ((val) & 0x04000000 ? '1' : '0'), \
  ((val) & 0x02000000 ? '1' : '0'), \
  ((val) & 0x01000000 ? '1' : '0'), \
  ((val) & 0x00800000 ? '1' : '0'), \
  ((val) & 0x00400000 ? '1' : '0'), \
  ((val) & 0x00200000 ? '1' : '0'), \
  ((val) & 0x00100000 ? '1' : '0'), \
  ((val) & 0x00080000 ? '1' : '0'), \
  ((val) & 0x00040000 ? '1' : '0'), \
  ((val) & 0x00020000 ? '1' : '0'), \
  ((val) & 0x00010000 ? '1' : '0'), \
  ((val) & 0x00008000 ? '1' : '0'), \
  ((val) & 0x00004000 ? '1' : '0'), \
  ((val) & 0x00002000 ? '1' : '0'), \
  ((val) & 0x00001000 ? '1' : '0'), \
  ((val) & 0x00000800 ? '1' : '0'), \
  ((val) & 0x00000400 ? '1' : '0'), \
  ((val) & 0x00000200 ? '1' : '0'), \
  ((val) & 0x00000100 ? '1' : '0'), \
  ((val) & 0x00000080 ? '1' : '0'), \
  ((val) & 0x00000040 ? '1' : '0'), \
  ((val) & 0x00000020 ? '1' : '0'), \
  ((val) & 0x00000010 ? '1' : '0'), \
  ((val) & 0x00000008 ? '1' : '0'), \
  ((val) & 0x00000004 ? '1' : '0'), \
  ((val) & 0x00000002 ? '1' : '0'), \
  ((val) & 0x00000001 ? '1' : '0')
// usage example:
// uint32_t regVal = READ_REG(PWR->CR1);
// waitToPrint();
// npf_snprintf(uart_buf, 200, "[Gateway] PWR->CR1 = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",regVal, UINT32_T_TO_BINARY(regVal));
// huart2print(uart_buf, strlen(uart_buf));


//printf("Leading text "BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(val));
//For multi-byte types
//printf("m: "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(m>>8), BYTE_TO_BINARY(m));



// Notifications
#define NOTIFICATION_FROM_APP_GNSS                      0x00000001   // see app_gnss.c: notification when app_gnns is refreshing gnss_epoch (epoch coming from the gnss module in case of host).
#define NOTIFICATION_FROM_GNSS_PPS                      0x00000002   // see main.c: notification when PPS_pin from the gnss module is refreshing (only valid in case of host)
#define NOTIFICATION_FROM_HAL_PPS                       0x00000004   // see app_hal_pps.c: notification when PPS_pin from the current module is refreshing
#define NOTIFICATION_FROM_RTC_PPS                       0x00000008   // see app_hal_sync.c: notification when RTC PPS from the current module is refreshing
#define NOTIFICATION_FROM_DIO1                          0x00000010   // see main.c and app_network_connect.c : notification of DIOx pin (transmit or receive data of radio)
#define NOTIFICATION_FROM_DIO2                          0x00000020   // see main.c and app_network_connect.c : notification of DIOx pin (transmit or receive data of radio)
//#define NOTIFICATION_FROM_DIO3                          0x00000040   // see main.c and app_network_connect.c : notification of DIOx pin (transmit or receive data of radio)
#define NOTIFICATION_FROM_APP_NETWORK_CONNECT_SET_RTC   0x00000080   // see app_network_connect.c: notification when app_network_connectc is refreshing received_epoch (epoch coming from a host in case of node).
#define NOTIFICATION_FROM_APP_NETWORK_CONNECT_START_RTC 0x00000100   // see app_network_connect.c: notification when app_network_connectc is refreshing received_epoch (epoch coming from a host in case of node).
#define NOTIFICATION_FROM_RADIO_AVAILABLE               0x00000200   // see app_network_connect.c: notification that the radio is available (RADIO_BUSY pin goes low)
#define NOTIFICATION_FROM_RTC_PPS_SET_RTC               0x00000400   // see app_rtc.c to make sure that the RTC is being set AFTER an RTC_PPS
#define NOTIFICATION_FROM_HOST_PP10S                    0x00000800   // see app_network_connect.c if a node is receiving a broadcast message from a his host, then notify app_rtc_sync
#define NOTIFICATION_FROM_APP_HAL_SYNC_TO_START_LED     0x00001000
#define NOTIFICATION_FROM_HOST_GNSS_PPS                 0x00002000
#define NOTIFICATION_FROM_RTC_PPS_START_RTC             0x00004000   // see app_rtc.c to make start the RTC in the beginning of the program.
#define NOTIFICATION_FROM_IMU                           0x00008000   // interrupt from IMU that data is available (INT1_ICM20948_Pin), see main.c and app_imu.c
#define NOTIFICATION_FROM_ADC                           0x00010000   // interrupt from ADC that data is available, see main.c and app_adc.c
#define NOTIFICATION_FROM_ADC_SUPERCAP_READY            0x00020000   // notification given to app_init that the voltage of the super capapcitor is more then 1800mV
#define NOTIFICATION_LOAD_SCAP                          0x00040000   // notification from either app_gateway or app_network_connect to load the supercap
#define NOTIFICATION_FROM_SCAP_SUPERCAP_READY           0x00080000   // notification from app_supercap that the supercap is loaded
#define NOTIFICATION_FROM_APP_HAL_SYNC_TO_SET_RTC       0x00100000   // notification
#define NOTIFICATION_FROM_GNSSPP10S_TO_START_BROADCAST  0x00200000   // notification from app_rtc_sync that the broadcast process can start
#define NOTIFICATION_FROM_APP_RTC_SYNC_START_RTC        0x00400000   // see app_network_connect.c: notification when app_network_connectc is refreshing received_epoch (epoch coming from a host in case of node).
#define NOTIFICATION_FROM_IMU_INITIALIZED               0x00800000   // see app_init: notofocation from app_imu that the IMU is initialized


// PRINCIPLE HOW TO USE NOTIFICATIONS ORIGINATING FROM DIFFERENT APPS TO A THREAD WHICH IS WAITING FOR AN EVENT
// AND NEEDS TO DISTINGUISH WHERE THE EVENT IS COMING FROM
// ==> in app_rtc.h:
// #define NOTIFICATION_FROM_APP_GNSS 0x02
// #define NOTIFICATION_FROM_GNSS_PPS 0x03
// void rtos_rtc_thread_init(void);
// void rtos_rtc_thread(const void * params)
// void app_rtc_notify(uint32_t notValue);
// void app_rtc_notify_fromISR(uint32_t notValue);
// ==> in app_gnss.c:
//     #include "app_rtc.h"
//     ...
//     app_rtc_notify(NOTIFICATION_FROM_APP_GNSS);
// ==> in HAL_GPIO_EXTI_Callback (main.c):
//     #include "app_rtc.h"
//     app_rtc_notify_fromISR(NOTIFICATION_FROM_GNSS_PPS);
//     ...
// ==> in app_rtc.c (receiving app of notification):
//   ==> in the thread to receive the notification:
//       osThreadId rtcThreadHandler;
//       void rtos_rtc_thread_init()
//       {
//         osThreadDef(rtcThread, rtos_rtc_thread, osPriorityNormal, 0, 128);
//         rtcThreadHandler = osThreadCreate(osThread(rtcThread), NULL);
//       }
//       void rtos_rtc_thread(const void * params)
//       {
//         uint32_t   NotificationValue = 0;                  // Used to identify where the notification is coming from.
//         TickType_t xMaxBlockTime     = pdMS_TO_TICKS(400); // The maximum block time state of the task in ms when no notification is received.
//         ...
//         for(;;)
//         {
//           ...
//           if (xTaskNotifyWait( 0x00,               // Don't clear any bits on entry.
//                                0xffffffff,         // Clear all bits on exit. (long max).
//                                &NotificationValue, // Receives the notification value.
//                                xMaxBlockTime ))    // Block 400 ms
//           {
//             if ((ulNotificationValue & NOTIFICATION_FROM_APP_GNSS) == NOTIFICATION_FROM_APP_GNSS)
//             { /* Received notification from syncThread (app_gnss.c). This happens each time a new date/time is received */
//               ...
//               check if the clock needs to be changed (every TIME_TO_ADAPT_RTC)
//               use gnss_epoch to set the time
//               ...
//             }
//             if ((ulNotificationValue & NOTIFICATION_FROM_GNSS_PPS) == NOTIFICATION_FROM_GNSS_PPS)
//             { /* Received notification from HAL_GPIO_EXTI_Callback (main.c). This happens when a gnss PPS is received  */
//               ...
//             }
//           }
//           ...
//         }
//       }
//   ==> to define app_rtc_notify():
//       void app_rtc_notify(uint32_t notValue)
//       {
//         if (rtcThreadHandler != NULL)
//       {
//         xTaskNotify(rtcThreadHandler, notValue, eSetBits);
//       }
//   ==> to define app_rtc_notify_fromISR():
//       void app_rtc_notify_fromISR(uint32_t notValue)
//       {
//         BaseType_t xHigherPriorityTaskWoken;
//         xHigherPriorityTaskWoken = pdFALSE;
//         if (rtcThreadHandler != NULL)
//         {
//           xTaskNotifyFromISR(rtcThreadHandler, notValue, eSetBits, &xHigherPriorityTaskWoken);
//         }
//         portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//       }

// changing Ticks to 64 bit:

// in stm32wbaxx_hal.h:
// change extern __IO uint32_t            uwTick; to extern __IO uint64_t            uwTick;
// change uwTick += (uint32_t)uwTickFreq; to uwTick += (uint64_t)uwTickFreq;

// change __IO uint32_t uwTick to __IO uint64_t uwTick;
// in portmacrocommon.h:
// change typedef uint32_t     TickType_t; to typedef uint64_t     TickType_t;

// adaptations to port.c:

//#include "main.h"
//extern uint32_t SysTickCallbackHits;
//extern __IO uint64_t uwTick;
//extern double   adjustTIM2;
//extern int32_t tim2Table[1000];
//extern uint32_t tim2Position;
//extern int32_t regVal;
//extern uint32_t TIM2Adapted;
//extern double   compensateTicks;
//extern double   halDriftCorrection;
//extern int32_t subTickCounterRTCPPS;
//
//void SysTick_Handler( void ) /* PRIVILEGED_FUNCTION */
//{
//    subTickCounterRTCPPS += (int32_t) SYSTICK_LOAD_REG + (int32_t) 1;   // added SG
//
//    uint32_t ulPreviousMask;
//
//    ulPreviousMask = portSET_INTERRUPT_MASK_FROM_ISR();
//    {
//        /* Increment the RTOS tick. */
//        if( xTaskIncrementTick() != pdFALSE )
//        {
//            /* Pend a context switch. */
//            portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
//        }
////        uwTick++;
//
//
//
////        SYSTICK_LOAD_REG = 15999U;
////        adjustTIM2 += halDriftCorrection;
////        adjustTIM2 += compensateTicks;
//
//
//
////        compensateTicks = 0;
////        if(adjustTIM2 >= 62.5)
////        {
////          regVal = (int32_t)(adjustTIM2 / -62.5);
////          SYSTICK_LOAD_REG = 15999U + regVal;
////          TIM2Adapted = 1;
////          adjustTIM2 += 62.5 * (double)regVal;
////        }
////        else
////        {
////    	  if(adjustTIM2 < -62.5)
////          {
////    		regVal = (int32_t)(adjustTIM2 / -62.5);
////            SYSTICK_LOAD_REG = 15999U + regVal;
////            TIM2Adapted = 1;
////            adjustTIM2 += 62.5 * (double)regVal;
////          }
////        }
//
//
//
////		regVal = (int32_t)(-adjustTIM2);
////        SYSTICK_LOAD_REG = 15999U + regVal;
////        adjustTIM2 += (double)regVal;
//
//
//
////        adjustTIM2 += 62.5 * (double)regVal;
////        tim2Table[tim2Position] = (uint32_t) SYSTICK_LOAD_REG;
////        tim2Table[tim2Position] = (int32_t) adjustTIM2;
////        tim2Table[tim2Position] = (uint32_t) regVal;
//    }
//    portCLEAR_INTERRUPT_MASK_FROM_ISR( ulPreviousMask );
//}
//Version of 20240910:
//#include "main.h"
//extern uint32_t      SysTickCallbackHits;
//extern __IO uint64_t uwTick;
//extern double        adjustTIM2;
//extern int32_t       tim2Table[1000];
//extern uint32_t      tim2Position;
//extern int32_t       regVal;
//extern uint32_t      TIM2Adapted;
//extern double        compensateTicks;
//extern double        halDriftCorrection;
//extern int32_t       subTickCounterRTCPPS;
//extern uint32_t      newReloadSysTickValue;
//extern double        restReloadSysTickValue;
//double               adjustsysTickCounter = 0;

//void SysTick_Handler( void ) /* PRIVILEGED_FUNCTION */
//{
////	subTickCounterRTCPPS += (int32_t) SYSTICK_LOAD_REG + (int32_t) 1;   // added SG
//
//    uint32_t ulPreviousMask;
//
//    ulPreviousMask = portSET_INTERRUPT_MASK_FROM_ISR();
//    {
//        /* Increment the RTOS tick. */
//        if( xTaskIncrementTick() != pdFALSE )
//        {
//            /* Pend a context switch. */
//            portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
//        }
//
//        adjustsysTickCounter += restReloadSysTickValue + halDriftCorrection;
//
//        if (halDriftCorrection)  // use a better way as this might be zero...
//        {
//          SYSTICK_LOAD_REG = (uint32_t) newReloadSysTickValue + (uint32_t)adjustsysTickCounter;
//          adjustsysTickCounter -= (uint32_t)adjustsysTickCounter;
//        }
////        adjustTIM2 += halDriftCorrection;
////        adjustTIM2 += compensateTicks;
//
//
//    }
//    portCLEAR_INTERRUPT_MASK_FROM_ISR( ulPreviousMask );
//}



//Adaptations to tasks.c:

//#include "main.h"
//extern __IO uint64_t uwTick;
//extern int32_t subTickCounterRTCPPS;
//
//// under: BaseType_t xTaskIncrementTick( void ):
//
//xTickCount = xConstTickCount;
//uwTick = xConstTickCount;                                           // added SG
////        subTickCounterRTCPPS += (int32_t) SYSTICK_LOAD_REG + (int32_t) 1;   // added SG




/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
