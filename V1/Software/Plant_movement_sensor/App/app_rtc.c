/*
 * app_rtc.c
 *
 *  Created on: Feb 19, 2023
 *      Author: Sarah Goossens
 *
 *  This app is stopping, setting and starting a new RTC date and time value
 *
 */

#include "main.h"
#include "app_rtc.h"
#include "usart.h"              // to declare huart2
#include <string.h>

#include "../Drivers/PCF2131/PCF2131.h"
//#include "app_gnss.h"
#include "time.h"
#include "app_network_connect.h"
#include "app_rtc_sync.h"
#if  PCF2131I2C
#include "i2c.h"
#else
#include "spi.h"
#endif

#define PRINTF_APP_RTC         1
#define PRINTF_APP_RTC_DETAILS 0

extern char              uart_buf[100];
extern char              byteString[3];
extern uint8_t           pcf2131Buf[54];
#if  !PCF2131I2C
extern uint8_t           pcf2131TxBuffer[17]; // only needed if PCF2131 with SPI interface
#endif
extern time_t            gnssEpoch;
extern uint64_t          StartClockHALTick;
extern time_t            received_epoch;   // received epoch from external module: if host, this will be used for time synchronization purposes
                                           //                                      if node, to check if node needs to be synchronized
extern uint64_t          receivedxEpoch;
extern SemaphoreHandle_t gnssEpochMutex;
//extern SemaphoreHandle_t recEpochMutex;
extern uint8_t           changeClock;
extern uint8_t           recSubSeconds;     // received sub seconds from Host at node (see app_network_connect.c)
extern uint64_t          timeStamp5;
extern uint8_t           firstGPRMC;
extern uint64_t          rtosTimeStampGNSSPPS;
extern uint64_t          rtosTimeStampRTCPPS;
extern uint64_t          rtosTimeStampChangeRTC;
extern uint64_t          rtosTimeStampNotifyRTC;
extern uint64_t          rtosTimeStampStopRTC;
extern uint64_t          rtosTimeStampStartRTC;
extern uint8_t           rtcSynchronized;
extern uint32_t          gnss_pps;
extern uint32_t          rtc_pps;
extern uint64_t          rtosTimeStampBCastStrt;
//extern uint8_t           notifyOnlyOnceGNSS;
extern module            moduleTable[MAXNRMODULES];
extern float             tickSpeedToReference;
#if !STM32WBAUSED
extern uint8_t           setClock;
extern uint8_t           i2cInUseGNSS;
extern SemaphoreHandle_t i2cGNSSMutex;
#endif

#if STM32WBAUSED
  TaskHandle_t rtcThreadHandler;
#else
  osThreadId rtcThreadHandler;
#endif

void RtcThreadInit()
{
#if STM32WBAUSED
  if (xTaskCreate ((TaskFunction_t)StartRtcThread, "RtcThread", (configSTACK_DEPTH_TYPE)512, NULL, osPriorityRealtime, &rtcThreadHandler) != pdPASS)
  {
#if PRINTF_APP_RTC
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_rtc] [RtcThread] Could not be started.\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
#else
  osThreadDef(rtcThread, rtos_rtc_thread, osPriorityHigh, 0, 128);
  rtcThreadHandler = osThreadCreate(osThread(rtcThread), NULL);
#endif
}

void StartRtcThread(const void * params)
{
#if PRINTF_APP_RTC
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_rtc] [RtcThread] Started.\n", (unsigned int) xTaskGetTickCount());
  huart2print(uart_buf, strlen(uart_buf));
#endif
  vTaskDelay(500); // wait until RTC started

  pcf2131_get_status(pcf2131Buf, 16, 0);

#if PRINTF_APP_RTC
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_rtc] PCF2131 read out first 16 registers: ", (unsigned int) xTaskGetTickCount());
  for (unsigned int i = 0; i < 16; i++)
  {
    npf_snprintf(byteString, 3, "%02X", pcf2131Buf[i]);
    strcat(uart_buf, byteString);
  }
  strcat(uart_buf, ".\n");
  huart2print(uart_buf, strlen(uart_buf));
#endif

  // check if RTC is running and if not, start the clock!
  while (!pcf2131_is_running())
  {
    pcf2131_start_clock();
#if PRINTF_APP_RTC
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_rtc] Clock is not running at the start of app_rtc! Start the clock again.\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
    vTaskDelay(20);
#endif
  }
#if PRINTF_APP_RTC
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_rtc] Clock runs at the start of app_rtc.\n",(unsigned int) xTaskGetTickCount());
  huart2print(uart_buf, strlen(uart_buf));
#endif

  pcf2131_get_status(pcf2131Buf, 16, 0);

#if PRINTF_APP_RTC
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_rtc] PCF2131 status: ", (unsigned int) xTaskGetTickCount());
  for (unsigned int i = 0; i < 15; i++)
  {
    npf_snprintf(byteString, 3, "%02X", pcf2131Buf[i]);
    strcat(uart_buf, byteString);
  }
  strcat(uart_buf, ".\n");
  huart2print(uart_buf, strlen(uart_buf));
#endif


  if ((pcf2131Buf[PCF2131_REG_CTRL1] & PCF2131_BIT_CTRL1_POR_OVRD) == PCF2131_BIT_CTRL1_POR_OVRD)
  {
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_rtc] [rtos_rtc_thread] PORO Enabled. Disable for normal operation.\r\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
    pcf2131_disable_poro();
  }
  else
  {
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_rtc] [rtos_rtc_thread] Power On Reset Override (PORO) Disabled.\r\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
  }
  pcf2131_reset();
  vTaskDelay(2000);
  pcf2131_get_status(pcf2131Buf, 16, 0);
#if PRINTF_APP_RTC
  waitToPrint();
  npf_snprintf(uart_buf, 200, "    [app_rtc] Raw data: REG_CTRL1 = 0x%02X " BYTE_TO_BINARY_PATTERN "b\r\n					 ",pcf2131Buf[PCF2131_REG_CTRL1], BYTE_TO_BINARY(pcf2131Buf[PCF2131_REG_CTRL1]));
  huart2print(uart_buf, strlen(uart_buf));
#if PRINTF_APP_RTC_DETAILS
  waitToPrint();
  if ((pcf2131Buf[PCF2131_REG_CTRL1] & PCF2131_BIT_CTRL1_SI) == PCF2131_BIT_CTRL1_SI)
  {
    npf_snprintf(uart_buf, 200, "|||||||+-> Second interrupt Enabled.\r\n");
  }
  else
  {
    npf_snprintf(uart_buf, 200, "|||||||+-> Second interrupt Disabled.\r\n");
  }
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  if ((pcf2131Buf[PCF2131_REG_CTRL1] & PCF2131_BIT_CTRL1_MI) == PCF2131_BIT_CTRL1_MI)
  {
    npf_snprintf(uart_buf, 200, "					 ||||||+--> Minute interrupt Enabled.\r\n");
  }
  else
  {
    npf_snprintf(uart_buf, 200, "					 ||||||+--> Minute interrupt Disabled.\r\n");
  }
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  if ((pcf2131Buf[PCF2131_REG_CTRL1] & PCF2131_BIT_CTRL1_12_24) == PCF2131_BIT_CTRL1_12_24)
  {
    npf_snprintf(uart_buf, 200, "					 |||||+---> 12-hour mode.\r\n");
  }
  else
  {
    npf_snprintf(uart_buf, 200, "					 |||||+---> 24-hour mode.\r\n");
  }
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  if ((pcf2131Buf[PCF2131_REG_CTRL1] & PCF2131_BIT_CTRL1_POR_OVRD) == PCF2131_BIT_CTRL1_POR_OVRD)
  {
    npf_snprintf(uart_buf, 200, "					 ||||+----> Power On Reset Override (PORO) Enabled.\r\n");
  }
  else
  {
    npf_snprintf(uart_buf, 200, "					 ||||+----> Power On Reset Override (PORO) Disabled.\r\n");
  }
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  if ((pcf2131Buf[PCF2131_REG_CTRL1] & PCF2131_BIT_CTRL1_100TH_S_DIS) == PCF2131_BIT_CTRL1_100TH_S_DIS)
  {
    npf_snprintf(uart_buf, 200, "					 |||+-----> 100th seconds counter Disabled.\r\n");
  }
  else
  {
    npf_snprintf(uart_buf, 200, "					 |||+-----> 100th seconds counter Enabled.\r\n");
  }
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  if ((pcf2131Buf[PCF2131_REG_CTRL1] & PCF2131_BIT_CTRL1_STOP) == PCF2131_BIT_CTRL1_STOP)
  {
    npf_snprintf(uart_buf, 200, "					 ||+------> RTC clock stopped.\r\n");
  }
  else
  {
    npf_snprintf(uart_buf, 200, "					 ||+------> RTC clock Runs.\r\n");
  }
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  if ((pcf2131Buf[PCF2131_REG_CTRL1] & PCF2131_BIT_CTRL1_TC_DIS) == PCF2131_BIT_CTRL1_TC_DIS)
  {
    npf_snprintf(uart_buf, 200, "					 |+-------> Temperature Compensation Disabled.\r\n");
  }
  else
  {
    npf_snprintf(uart_buf, 200, "					 |+-------> Temperature Compensation Enabled.\r\n");
  }
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  if ((pcf2131Buf[PCF2131_REG_CTRL1] & PCF2131_BIT_CTRL1_EXT_TEST) == PCF2131_BIT_CTRL1_EXT_TEST)
  {
    npf_snprintf(uart_buf, 200, "					 +--------> External clock test mode.\r\n\r\n");
  }
  else
  {
    npf_snprintf(uart_buf, 200, "					 +--------> Normal mode.\r\n\r\n");
  }
  huart2print(uart_buf, strlen(uart_buf));
#endif // PRINTF_APP_RTC_DETAILS
  waitToPrint();
  npf_snprintf(uart_buf, 200, "\r\n    [app_rtc] Raw data: REG_CTRL2 = 0x%02X " BYTE_TO_BINARY_PATTERN "b\r\n					 ",pcf2131Buf[PCF2131_REG_CTRL2], BYTE_TO_BINARY(pcf2131Buf[PCF2131_REG_CTRL2]));
  huart2print(uart_buf, strlen(uart_buf));
#if PRINTF_APP_RTC_DETAILS
  waitToPrint();
  npf_snprintf(uart_buf, 200, "|||||||+-> Not used.\r\n");
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  if ((pcf2131Buf[PCF2131_REG_CTRL2] & PCF2131_BIT_CTRL2_AIE) == PCF2131_BIT_CTRL2_AIE)
  {
    npf_snprintf(uart_buf, 200, "					 ||||||+--> Alarm Interrupt Enabled.\r\n");
  }
  else
  {
    npf_snprintf(uart_buf, 200, "					 ||||||+--> No interrupt generated from the alarm flag.\r\n");
  }
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  npf_snprintf(uart_buf, 200, "					 |||||+---> Not used.\r\n");
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  npf_snprintf(uart_buf, 200, "					 ||||+----> Not used.\r\n");
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  if ((pcf2131Buf[PCF2131_REG_CTRL2] & PCF2131_BIT_CTRL2_AF) == PCF2131_BIT_CTRL2_AF)
  {
    npf_snprintf(uart_buf, 200, "					 |||+-----> Alarm Flag set when alarm triggered, must be cleared.\r\n");
  }
  else
  {
    npf_snprintf(uart_buf, 200, "					 |||+-----> No alarm interrupt triggered.\r\n");
  }
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  npf_snprintf(uart_buf, 200, "					 ||+------> Not used.\r\n");
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  if ((pcf2131Buf[PCF2131_REG_CTRL2] & PCF2131_BIT_CTRL2_WDTF) == PCF2131_BIT_CTRL2_WDTF)
  {
    npf_snprintf(uart_buf, 200, "					 |+-------> Watchdog timer interrupt will be generated.\r\n");
  }
  else
  {
    npf_snprintf(uart_buf, 200, "					 |+-------> No watchdog timer interrupt will be generated.\r\n");
  }
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  if ((pcf2131Buf[PCF2131_REG_CTRL2] & PCF2131_BIT_CTRL2_MSF) == PCF2131_BIT_CTRL2_MSF)
  {
    npf_snprintf(uart_buf, 200, "					 +--------> Minute or Second interrupt will be generated.\r\n\r\n");
  }
  else
  {
    npf_snprintf(uart_buf, 200, "					 +--------> No Minute or Second interrupt will be generated.\r\n\r\n");
  }
  huart2print(uart_buf, strlen(uart_buf));
#endif // PRINTF_APP_RTC_DETAILS
  waitToPrint();
  npf_snprintf(uart_buf, 200, "\r\n    [app_rtc] Raw data: REG_CTRL3 = 0x%02X " BYTE_TO_BINARY_PATTERN "b\r\n					 ",pcf2131Buf[PCF2131_REG_CTRL3], BYTE_TO_BINARY(pcf2131Buf[PCF2131_REG_CTRL3]));
  huart2print(uart_buf, strlen(uart_buf));
#if PRINTF_APP_RTC_DETAILS
  waitToPrint();
  if ((pcf2131Buf[PCF2131_REG_CTRL3] & PCF2131_BIT_CTRL3_BLIE) == PCF2131_BIT_CTRL3_BLIE)
  {
    npf_snprintf(uart_buf, 200, "| |||||+-> An interrupt will be generated when Battery is low.\r\n");
  }
  else
  {
    npf_snprintf(uart_buf, 200, "| |||||+-> No interrupt will be generated when Battery is low.\r\n");
  }
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  if ((pcf2131Buf[PCF2131_REG_CTRL3] & PCF2131_BIT_CTRL3_BIE) == PCF2131_BIT_CTRL3_BIE)
  {
    npf_snprintf(uart_buf, 200, "					 | ||||+--> battery flag (BF) is set: interrupt will be generated.\r\n");
  }
  else
  {
    npf_snprintf(uart_buf, 200, "					 | ||||+--> no interrupt generated from the battery flag (BF).\r\n");
  }
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  if ((pcf2131Buf[PCF2131_REG_CTRL3] & PCF2131_BIT_CTRL3_BLF) == PCF2131_BIT_CTRL3_BLF)
  {
    npf_snprintf(uart_buf, 200, "					 | |||+---> Battery Low, flag is automatically cleared with battery replacement.\r\n");
  }
  else
  {
    npf_snprintf(uart_buf, 200, "					 | |||+---> Battery status OK.\r\n");
  }
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  if ((pcf2131Buf[PCF2131_REG_CTRL3] & PCF2131_BIT_CTRL3_BF) == PCF2131_BIT_CTRL3_BF)
  {
    npf_snprintf(uart_buf, 200, "					 | ||+----> Battery switch-over interrupt occurred, must be cleared.\r\n");
  }
  else
  {
    npf_snprintf(uart_buf, 200, "					 | ||+----> No battery switch-over interrupt occurred.\r\n");
  }
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  if ((pcf2131Buf[PCF2131_REG_CTRL3] & PCF2131_BIT_CTRL3_BTSE) == PCF2131_BIT_CTRL3_BTSE)
  {
    npf_snprintf(uart_buf, 200, "					 | |+-----> Time stamp will be generated when battery switch-over interrupt occurs.\r\n");
  }
  else
  {
    npf_snprintf(uart_buf, 200, "					 | |+-----> No Time stamp will be generated when battery switch-over interrupt occurs.\r\n");
  }
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  uint8_t pwrmgt = (pcf2131Buf[PCF2131_REG_CTRL3] & 0b11100000) >> 5;
  switch (pwrmgt)
  {
    case 0b000:
      npf_snprintf(uart_buf, 200, "					 +-+------> battery switch-over function enabled in standard mode, battery low detection function enabled.\r\n");
      break;
    case 0b001:
      npf_snprintf(uart_buf, 200, "					 +-+------> battery switch-over function enabled in standard mode, battery low detection function disabled.\r\n");
      break;
    case 0b010:
      npf_snprintf(uart_buf, 200, "					 +-+------> battery switch-over function enabled in standard mode, battery low detection function disabled.\r\n");
      break;
    case 0b011:
      npf_snprintf(uart_buf, 200, "					 +-+------> battery switch-over function enabled in direct switching mode, battery low detection function enabled.\r\n");
      break;
    case 0b100:
      npf_snprintf(uart_buf, 200, "					 +-+------> battery switch-over function enabled in direct switching mode, battery low detection function disabled.\r\n");
      break;
    case 0b101:
      npf_snprintf(uart_buf, 200, "					 +-+------> battery switch-over function enabled in direct switching mode, battery low detection function disabled.\r\n");
      break;
    case 0b110:
      npf_snprintf(uart_buf, 200, "					 +-+------> battery switch-over function disabled (only power supply), battery low detection function disabled.\r\n");
      break;
    case 0b111:
      npf_snprintf(uart_buf, 200, "					 +-+------> battery switch-over function disabled (only power supply), battery low detection function disabled.\r\n");
  }
  huart2print(uart_buf, strlen(uart_buf));
#endif // PRINTF_APP_RTC_DETAILS
  waitToPrint();
  npf_snprintf(uart_buf, 200, "\r\n    [app_rtc] Raw data: REG_CTRL4 = 0x%02X " BYTE_TO_BINARY_PATTERN "b\r\n					 ",pcf2131Buf[PCF2131_REG_CTRL4], BYTE_TO_BINARY(pcf2131Buf[PCF2131_REG_CTRL4]));
  huart2print(uart_buf, strlen(uart_buf));
#if PRINTF_APP_RTC_DETAILS
  waitToPrint();
  npf_snprintf(uart_buf, 200, "|||||||+-> Not used.\r\n");
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  npf_snprintf(uart_buf, 200, "					 ||||||+--> Not used.\r\n");
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  npf_snprintf(uart_buf, 200, "					 |||||+---> Not used.\r\n");
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  npf_snprintf(uart_buf, 200, "					 ||||+----> Not used.\r\n");
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  if ((pcf2131Buf[PCF2131_REG_CTRL4] & PCF2131_BIT_CTRL4_TSF4) == PCF2131_BIT_CTRL4_TSF4)
  {
    npf_snprintf(uart_buf, 200, "					 |||+-----> Time stamp interrupt generated for pin NTS4.\r\n");
  }
  else
  {
    npf_snprintf(uart_buf, 200, "					 |||+-----> No time stamp interrupt generated for pin NTS4.\r\n");
  }
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  if ((pcf2131Buf[PCF2131_REG_CTRL4] & PCF2131_BIT_CTRL4_TSF3) == PCF2131_BIT_CTRL4_TSF3)
  {
    npf_snprintf(uart_buf, 200, "					 ||+------> Time stamp interrupt generated for pin NTS3.\r\n");
  }
  else
  {
    npf_snprintf(uart_buf, 200, "					 ||+------> No time stamp interrupt generated for pin NTS3.\r\n");
  }
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  if ((pcf2131Buf[PCF2131_REG_CTRL4] & PCF2131_BIT_CTRL4_TSF2) == PCF2131_BIT_CTRL4_TSF2)
  {
    npf_snprintf(uart_buf, 200, "					 |+-------> Time stamp interrupt generated for pin NTS2.\r\n");
  }
  else
  {
    npf_snprintf(uart_buf, 200, "					 |+-------> No time stamp interrupt generated for pin NTS2.\r\n");
  }
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  if ((pcf2131Buf[PCF2131_REG_CTRL4] & PCF2131_BIT_CTRL4_TSF1) == PCF2131_BIT_CTRL4_TSF1)
  {
    npf_snprintf(uart_buf, 200, "					 +--------> Time stamp interrupt generated for pin NTS1.\r\n");
  }
  else
  {
    npf_snprintf(uart_buf, 200, "					 +--------> No time stamp interrupt generated for pin NTS1.\r\n");
  }
  huart2print(uart_buf, strlen(uart_buf));
#endif // PRINTF_APP_RTC_DETAILS
  waitToPrint();
  npf_snprintf(uart_buf, 200, "\r\n    [app_rtc] Raw data: REG_CTRL5 = 0x%02X " BYTE_TO_BINARY_PATTERN "b\r\n					 ",pcf2131Buf[PCF2131_REG_CTRL5], BYTE_TO_BINARY(pcf2131Buf[PCF2131_REG_CTRL5]));
  huart2print(uart_buf, strlen(uart_buf));
#if PRINTF_APP_RTC_DETAILS
  waitToPrint();
  npf_snprintf(uart_buf, 200, "|||||||+-> Not used.\r\n");
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  npf_snprintf(uart_buf, 200, "					 ||||||+--> Not used.\r\n");
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  npf_snprintf(uart_buf, 200, "					 |||||+---> Not used.\r\n");
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  npf_snprintf(uart_buf, 200, "					 ||||+----> Not used.\r\n");
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  if ((pcf2131Buf[PCF2131_REG_CTRL5] & PCF2131_BIT_CTRL5_TSIE4) == PCF2131_BIT_CTRL5_TSIE4)
  {
    npf_snprintf(uart_buf, 200, "					 |||+-----> Interrupt will be generated when time stamp flag is set of NTS4.\r\n");
  }
  else
  {
    npf_snprintf(uart_buf, 200, "					 |||+-----> No interrupt will be generated from time stamp flag of NTS4.\r\n");
  }
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  if ((pcf2131Buf[PCF2131_REG_CTRL5] & PCF2131_BIT_CTRL5_TSIE3) == PCF2131_BIT_CTRL5_TSIE3)
  {
    npf_snprintf(uart_buf, 200, "					 ||+------> Interrupt will be generated when time stamp flag is set of NTS3.\r\n");
  }
  else
  {
    npf_snprintf(uart_buf, 200, "					 ||+------> No interrupt will be generated from time stamp flag of NTS3.\r\n");
  }
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  if ((pcf2131Buf[PCF2131_REG_CTRL5] & PCF2131_BIT_CTRL5_TSIE2) == PCF2131_BIT_CTRL5_TSIE2)
  {
    npf_snprintf(uart_buf, 200, "					 |+-------> Interrupt will be generated when time stamp flag is set of NTS2.\r\n");
  }
  else
  {
    npf_snprintf(uart_buf, 200, "					 |+-------> No interrupt will be generated from time stamp flag of NTS2.\r\n");
  }
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  if ((pcf2131Buf[PCF2131_REG_CTRL5] & PCF2131_BIT_CTRL5_TSIE1) == PCF2131_BIT_CTRL5_TSIE1)
  {
    npf_snprintf(uart_buf, 200, "					 +--------> Interrupt will be generated when time stamp flag is set of NTS1.\r\n\r\n");
  }
  else
  {
    npf_snprintf(uart_buf, 200, "					 +--------> No interrupt will be generated from time stamp flag of NTS1.\r\n\r\n");
  }
  huart2print(uart_buf, strlen(uart_buf));
#endif // PRINTF_APP_RTC_DETAILS
  waitToPrint();
  if ((pcf2131Buf[PCF2131_REG_SR_RESET] & 0b00100100) == 0b00100100)
  {
    npf_snprintf(uart_buf, 200, "\r\n    [app_rtc] Raw data: Software reset register value correct = 0x%02X " BYTE_TO_BINARY_PATTERN "b\r\n",pcf2131Buf[PCF2131_REG_SR_RESET], BYTE_TO_BINARY(pcf2131Buf[PCF2131_REG_SR_RESET]));
  }
  else
  {
    npf_snprintf(uart_buf, 200, "\r\n    [app_rtc] Raw data: Software reset register error! 0b00100100 expected, value = 0x%02X " BYTE_TO_BINARY_PATTERN "b\r\n",pcf2131Buf[PCF2131_REG_SR_RESET], BYTE_TO_BINARY(pcf2131Buf[PCF2131_REG_SR_RESET]));
  }
  huart2print(uart_buf, strlen(uart_buf));
#endif

  uint32_t notificationValue = 0;                     // Used to identify where the notification is coming from.
  PCF2131_time_t rtc_time;
  PCF2131_time_t gnss_time;
  struct tm *gnss_tmStructure;
  time_t gnssEpochCopy = 0;
  uint8_t RTCFirstTimeOn = 1; // If the RTC is set for the first time, then this variable will become 0.

#if PLANTSENSOR
  vTaskSuspend(rtcThreadHandler);
#endif

  RtcSyncThreadInit();

  for(;;)
  { // Infinite loop
    notificationValue = 0;
    xTaskNotifyStateClear(rtcThreadHandler);
    //***************
    // 1st time waiting for a notification from the RTC PID controller in app_hal_sync to start a new iteration to set a new date/time in the RTC
    //***************
#if SENSOR_HOST
    while ((notificationValue & NOTIFICATION_FROM_APP_HAL_SYNC_TO_SET_RTC) != NOTIFICATION_FROM_APP_HAL_SYNC_TO_SET_RTC)
    { // waiting for a notification value from app_hal_sync that the RTC needs to be changed
      //xTaskNotifyWait( 0x00, 0xffffffff, &notificationValue, (TickType_t) (50000 * ((int)tickSpeedToReference)));
      xTaskNotifyWait( 0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(50000));
      if ((notificationValue & NOTIFICATION_FROM_APP_HAL_SYNC_TO_SET_RTC) == NOTIFICATION_FROM_APP_HAL_SYNC_TO_SET_RTC)
      {
        rtosTimeStampNotifyRTC = xTaskGetTickCount();
        xSemaphoreTake(gnssEpochMutex, pdMS_TO_TICKS(400));
        gnssEpochCopy = gnssEpoch;
        xSemaphoreGive(gnssEpochMutex);
#if !STM32WBAUSED
        xSemaphoreTake(i2cGNSSMutex, pdMS_TO_TICKS(400));
        i2cInUseGNSS = 0;
        xSemaphoreGive(i2cGNSSMutex);
#endif
        gnss_tmStructure = gmtime(&gnssEpochCopy);
#if PRINTF_APP_RTC
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_rtc] Notification received @%uticks to set RTC.    GNSS last received Date: %d/%d/%d, UTC Time: %d:%d:%d.\r\n",
            (unsigned int) xTaskGetTickCount(), (unsigned int) rtosTimeStampNotifyRTC,
            gnss_tmStructure->tm_mday, gnss_tmStructure->tm_mon + 1, gnss_tmStructure->tm_year + 1900,
            gnss_tmStructure->tm_hour, gnss_tmStructure->tm_min, gnss_tmStructure->tm_sec);
        huart2print(uart_buf, strlen(uart_buf));
#endif
      }
      else
      {
        if(notificationValue)
        {
#if PRINTF_APP_RTC
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_rtc] Wrong notification received while waiting to change the RTC: 0x%08X.\r\n", (unsigned int) xTaskGetTickCount(), (unsigned int) notificationValue);
        huart2print(uart_buf, strlen(uart_buf));
#endif
          xTaskNotifyStateClear(rtcThreadHandler);
        }
        else
        {
#if PRINTF_APP_RTC
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_rtc] No notification received from app_hal_sync to change the RTC in the last 50s.\r\n", (unsigned int) xTaskGetTickCount());
          huart2print(uart_buf, strlen(uart_buf));
#endif
          xTaskNotifyStateClear(rtcThreadHandler);
        }
      }
    }
    gnssEpochCopy++;  // +1 because the clock will start again at the NEXT GNSSPPS
    gnss_tmStructure = gmtime(&gnssEpochCopy);
    pcf2131_get_time(&rtc_time);
    gnss_time.year          = (uint16_t)(gnss_tmStructure->tm_year - 100); // only year, no century: 0-99 (tm_year is since 1900)
    gnss_time.month         = (uint8_t) (gnss_tmStructure->tm_mon  + 1);   // 1-12 (tm_mon is 0-11)
    gnss_time.dayOfMonth    = (uint8_t)  gnss_tmStructure->tm_mday;        // 1-31
    gnss_time.dayOfWeek     = (uint8_t)  gnss_tmStructure->tm_wday;        // Sunday = 0 - 6
    gnss_time.hours         = (uint8_t)  gnss_tmStructure->tm_hour;
    gnss_time.minutes       = (uint8_t)  gnss_tmStructure->tm_min;
    gnss_time.seconds       = (uint8_t)  gnss_tmStructure->tm_sec;
    gnss_time.subSeconds    = 0x00;                                        // for Host there are no sub seconds
    gnss_time.mode          = 0;
    pcf2131_set_time(&gnss_time);
#if PRINTF_APP_RTC
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_rtc] RTC stopped at %uticks, %ums later than Notification. RTC Current Date: %d/%d/%d, UTC Time: %d:%d:%d.%d.\r\n",
        (unsigned int) xTaskGetTickCount(), (unsigned int) rtosTimeStampStopRTC, (unsigned int) ((rtosTimeStampStopRTC - rtosTimeStampNotifyRTC) / ((int)tickSpeedToReference)),
        rtc_time.dayOfMonth, rtc_time.month, rtc_time.year, rtc_time.hours, rtc_time.minutes, rtc_time.seconds, rtc_time.subSeconds);
    huart2print(uart_buf, strlen(uart_buf));
#endif
    notificationValue = 0;
    //***************
    // 2nd time waiting for a notification from the gnss_pps (in case of host) or ??? (only HAL_pps is available) (in case of node) to start the clock. The time set is 1 second later than received.
    //***************
    xTaskNotifyStateClear(rtcThreadHandler);
    while ((notificationValue & NOTIFICATION_FROM_APP_RTC_SYNC_START_RTC) != NOTIFICATION_FROM_APP_RTC_SYNC_START_RTC)
    { // waiting for a notification value from app_gnss.c that the i2c bus is not in use by the GNSS module. There will be also a notification from GNSS_PPS which is not needed here.
      xTaskNotifyWait( 0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(2000));
      if ((notificationValue & NOTIFICATION_FROM_APP_RTC_SYNC_START_RTC) == NOTIFICATION_FROM_APP_RTC_SYNC_START_RTC)
      {
        pcf2131_start_clock();
        if (pcf2131_is_running())
        { // check if the RTC is running
          changeClock = 0;
          pcf2131_get_time(&rtc_time);
          if (RTCFirstTimeOn)
          { // RTC is set for the first time with actual GNSS information:
            RTCFirstTimeOn = 0;
            moduleTable[0].epochConnected = gnssEpochCopy;  // store the start time of the Host to the module table:
          }
#if PRINTF_APP_RTC
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_rtc] RTC started @ tick %u, %ums later than GNSSPPS #%u. Total time RTC was off: %ums, RTC new UTC Time: %d:%d:%d.%d.\r\n",
              (unsigned int) xTaskGetTickCount(), (unsigned int) rtosTimeStampStartRTC, (int) ((rtosTimeStampStartRTC - rtosTimeStampGNSSPPS) / ((int)tickSpeedToReference)), (unsigned int) gnss_pps,
              (unsigned int) ((rtosTimeStampStartRTC - rtosTimeStampStopRTC) / ((int)tickSpeedToReference)),rtc_time.hours, rtc_time.minutes, rtc_time.seconds, rtc_time.subSeconds );
          huart2print(uart_buf, strlen(uart_buf));
          //npf_snprintf(uart_buf, 200, ".\r\n");
#endif
        }
        else
        { // RTC is not running... do a new attempt (do not zero changeClock)
#if PRINTF_APP_RTC
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_rtc] Clock was not running at the start of a new RTC sync! New attempt...\n",(unsigned int) xTaskGetTickCount());
          huart2print(uart_buf, strlen(uart_buf));
#endif
        }
      }
      else
      {
        if (notificationValue)
        {
#if PRINTF_APP_RTC
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_rtc] Wrong notification received to start the RTC: %08X.\r\n", (unsigned int) xTaskGetTickCount(), (unsigned int) notificationValue);
          huart2print(uart_buf, strlen(uart_buf));
#endif
          xTaskNotifyStateClear(rtcThreadHandler);
        }
        else
        {
#if PRINTF_APP_RTC
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_rtc] No notification received from rtc_sync that the RTC can be started in the last 2s.\r\n", (unsigned int) xTaskGetTickCount());
        huart2print(uart_buf, strlen(uart_buf));
#endif
        }
      }
    }
//#if PRINTF_APP_RTC
//          waitToPrint();
//          npf_snprintf(uart_buf, 200, "%u [app_rtc] GNSSPPS %ums, RTCPPS %ums, Change %ums, Notify %ums, Stop %ums, Start %ums.\n",
//        		(unsigned int) xTaskGetTickCount(),
//				(unsigned int) rtosTimeStampGNSSPPS,
//				(unsigned int) rtosTimeStampRTCPPS,
//				(unsigned int) rtosTimeStampChangeRTC,
//				(unsigned int) rtosTimeStampNotifyRTC,
//				(unsigned int) rtosTimeStampStopRTC,
//				(unsigned int) rtosTimeStampStartRTC);
//          huart2print(uart_buf, strlen(uart_buf));
//#endif

#else
//      pcf2131_get_status(pcf2131Buf, 16, 0);
//      pcf2131_get_time(&rtc_time);
//#if PRINTF_APP_RTC
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "%u [app_rtc]        RTC Current Date: %d/%d/%d, UTC Time: %d:%d:%d.%d.\r\n", (unsigned int) xTaskGetTickCount(),
//          rtc_time.dayOfMonth, rtc_time.month, rtc_time.year, rtc_time.hours, rtc_time.minutes, rtc_time.seconds, rtc_time.subSeconds);
//      huart2print(uart_buf, strlen(uart_buf));
//#endif

      vTaskDelay(2000);
//    while ((notificationValue & NOTIFICATION_FROM_APP_NETWORK_CONNECT_SET_RTC) != NOTIFICATION_FROM_APP_NETWORK_CONNECT_SET_RTC)
//    { // waiting for a notification value from app_gnss.c There will be also a notification from GNSS_PPS which is not needed here.
//      xTaskNotifyWait( 0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(2000));
//    }
//    if ((notificationValue & NOTIFICATION_FROM_APP_NETWORK_CONNECT_SET_RTC) == NOTIFICATION_FROM_APP_NETWORK_CONNECT_SET_RTC)
//    { // Received notification from syncThread (app_gnss.c). This happens when the clock needs to be refreshed.
//      xSemaphoreTake(recEpochMutex, pdMS_TO_TICKS(400));
//      received_epoch = ( time_t) (receivedxEpoch / 100);
//      recSubSeconds  = (uint8_t) (receivedxEpoch % 100);
//      xSemaphoreGive(recEpochMutex);
//      gnss_tmStructure = gmtime(&received_epoch);
//#if PRINTF_APP_RTC
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "%u [app_rtc] notification from app_network_connect to start the clock.\n",(unsigned int) xTaskGetTickCount());
//      huart2print(uart_buf, strlen(uart_buf));
//#endif
//      pcf2131_get_time(&rtc_time);
//#if PRINTF_APP_RTC
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "%u [app_rtc] HOST last received Date: %d/%d/%d, UTC Time: %d:%d:%d.\r\n", (unsigned int) xTaskGetTickCount(),
//          gnss_tmStructure->tm_mday, gnss_tmStructure->tm_mon + 1, gnss_tmStructure->tm_year + 1900,
//          gnss_tmStructure->tm_hour, gnss_tmStructure->tm_min, gnss_tmStructure->tm_sec);
//      huart2print(uart_buf, strlen(uart_buf));
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "%u [app_rtc]        RTC Current Date: %d/%d/%d, UTC Time: %d:%d:%d.%d.\r\n", (unsigned int) xTaskGetTickCount(),
//          rtc_time.dayOfMonth, rtc_time.month, rtc_time.year, rtc_time.hours, rtc_time.minutes, rtc_time.seconds, rtc_time.subSeconds);
//      huart2print(uart_buf, strlen(uart_buf));
//#endif
//      gnss_time.year          = (uint16_t)(gnss_tmStructure->tm_year - 100); // only year, no century: 0-99 (tm_year is since 1900)
//      gnss_time.month         = (uint8_t) (gnss_tmStructure->tm_mon  + 1);   // 1-12 (tm_mon is 0-11)
//      gnss_time.dayOfMonth    = (uint8_t)  gnss_tmStructure->tm_mday;        // 1-31
//      gnss_time.dayOfWeek     = (uint8_t)  gnss_tmStructure->tm_wday;        // Sunday = 0 - 6
//      gnss_time.hours         = (uint8_t)  gnss_tmStructure->tm_hour;
//      gnss_time.minutes       = (uint8_t)  gnss_tmStructure->tm_min;
//      gnss_time.seconds       = (uint8_t)  gnss_tmStructure->tm_sec;
//      gnss_time.subSeconds    = recSubSeconds;
//      gnss_time.mode          = 0;
//      pcf2131_set_time(&gnss_time);
//      notificationValue = 0;
//      xTaskNotifyStateClear(rtcThreadHandler);
////      while ((notificationValue & NOTIFICATION_FROM_APP_NETWORK_CONNECT_START_RTC) != NOTIFICATION_FROM_APP_NETWORK_CONNECT_START_RTC)
////      { // waiting for a notification from HAL_PPS (see app_hal_pps.c).
//        xTaskNotifyWait( 0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(2100));
////      }
//      if ((notificationValue & NOTIFICATION_FROM_APP_NETWORK_CONNECT_START_RTC) == NOTIFICATION_FROM_APP_NETWORK_CONNECT_START_RTC)
//      { // hal_PPS (see app_hal_pps.c)
//        pcf2131_start_clock();
//      }
//      // check if the RTC is running and if not, start the RTC! (this should never happen as it will not be in sync anymore)
//      if (!pcf2131_is_running())
//      {
//        pcf2131_start_clock();
//#if PRINTF_APP_RTC
//        waitToPrint();
//        npf_snprintf(uart_buf, 200, "%u [app_rtc] Clock is not running at the start of a new RTC sync! Start the clock.\n",(unsigned int) xTaskGetTickCount());
//        huart2print(uart_buf, strlen(uart_buf));
//#endif
//      }
//      changeClock        = 0;
#if !STM32WBAUSED
      setClock           = 0;
#endif
//    }
#endif
  } // end Infinite loop
}

void app_rtc_notify(uint32_t notValue)
{
  if (rtcThreadHandler != NULL)
  {
    xTaskNotify(rtcThreadHandler, notValue, eSetBits);
  }
}

void app_rtc_notify_fromISR(uint32_t notValue)
{
  BaseType_t xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  if (rtcThreadHandler != NULL)
  {
    xTaskNotifyFromISR(rtcThreadHandler, notValue, eSetBits, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
