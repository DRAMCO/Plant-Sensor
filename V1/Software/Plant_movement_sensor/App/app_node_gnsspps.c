/*
 * app_node_gnsspps.c
 *
 *  Created on: Oct 3, 2023
 *      Author: Sarah Goossens
 *
 *  ONLY used for test purposes and on a NODE
 *
 *  To measure the time difference in RTOS ticks (ms) between the RTCPPS, synchronized via a host and a wired GNSSPPS (from the same GNSS module
 *  that the host is using)
 *
 */

#include "main.h"
#include "FreeRTOS.h"
#if !STM32WBAUSED
  #include "cmsis_os.h"
#endif

#include "app_rtc.h"
#include "app_node_gnsspps.h"
#include "usart.h"              // to declare huart2
#include <string.h>

#define PRINTF_APP_NODE_GNSSPPS 1

extern char              uart_buf[200];
extern uint64_t          timeStampInt; // RTOS tick at the time of the external interrupt (in this case NINTA or RTC_PPS)
extern uint64_t          timeStamp8;
extern uint8_t           rtc_pps;
extern uint32_t          gnss_pps;
extern uint32_t          duration_100_gnss_pps;
extern uint64_t          rtosTimeStampGNSSPPS;
extern uint64_t          rtosTimeStampRTCPPS;
extern uint64_t          rtosTimeStampLedOn;
extern time_t            gnssEpoch;
extern SemaphoreHandle_t gnssEpochMutex;
extern SemaphoreHandle_t timeStampIntMutex;

#if STM32WBAUSED
   TaskHandle_t gnssppsThreadHandler;
#else
  osThreadId gnssppsThreadHandler;
#endif


void GNSSPPSThreadInit()
{
#if STM32WBAUSED
  if (xTaskCreate ((TaskFunction_t)StartGNSSPPSThread, "GNSSPPSThread", (configSTACK_DEPTH_TYPE)512, NULL, osPriorityHigh, &gnssppsThreadHandler) != pdPASS)
  {
#if PRINTF_APP_NODE_GNSSPPS
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_node_gnsspps] [GNSSPPSThread] Could not be started.\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
#else
  osThreadDef(GNSSPPSThread, StartGNSSPPSThread, osPriorityHigh, 0, 128);
  gnssppsThreadHandler = osThreadCreate(osThread(GNSSPPSThread), NULL);
#endif
}

void StartGNSSPPSThread(const void * params)
{
#if PRINTF_APP_NODE_GNSSPPS
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_node_gnsspps] [rtos_gnsspps_thread] Started.\n",(unsigned int) xTaskGetTickCount());
  huart2print(uart_buf, strlen(uart_buf));
#endif
  uint32_t   notificationValue            = 0;                     // Used to identify where the notification is coming from.
  uint64_t   rtosTimeSHostGNSSPPS         = 0;
  uint64_t   previousRtosTimeSHostGNSSPPS = 0;
  int32_t    differenceHostGNSSPPSLedOn   = 0;
  for(;;)
  { // Infinite loop
    notificationValue = 0;
    xTaskNotifyStateClear(gnssppsThreadHandler);
    while ((notificationValue & NOTIFICATION_FROM_HOST_GNSS_PPS) != NOTIFICATION_FROM_HOST_GNSS_PPS)
    { // waiting for a notification value from GNSS_PPS - Interrupt from GNSS PPS_Pin - see main HAL_GPIO_EXTI_Callback().
      xTaskNotifyWait( 0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(2000) );
    }
    xSemaphoreTake(timeStampIntMutex, pdMS_TO_TICKS(400));
    rtosTimeSHostGNSSPPS = timeStampInt;
    xSemaphoreGive(timeStampIntMutex);
    gnss_pps++;
    differenceHostGNSSPPSLedOn = (int32_t) (rtosTimeSHostGNSSPPS - rtosTimeStampLedOn);
//    differenceHostGNSSPPSLedOn += 1000;
    while (differenceHostGNSSPPSLedOn >= 500)
    {
	differenceHostGNSSPPSLedOn -= 1000;
    }
//    differenceHostGNSSPPSLedOn -= 1000;
    while (differenceHostGNSSPPSLedOn <= -500)
    {
	differenceHostGNSSPPSLedOn += 1000;
    }

#if PRINTF_APP_NODE_GNSSPPS
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[app_node_gnsspps]   HOST GNSSPPS #%u at %ums -> time reference. RTCPPS #%u at %ums, LedOn at %ums -> HOST - Led = %dms.\r\n",
        (unsigned int) gnss_pps, (unsigned int) rtosTimeSHostGNSSPPS, (unsigned int) rtc_pps, (unsigned int) rtosTimeStampRTCPPS, (unsigned int) rtosTimeStampLedOn, (int) differenceHostGNSSPPSLedOn);
    huart2print(uart_buf, strlen(uart_buf));
#endif

//    previousRtosTimeSHostGNSSPPS = rtosTimeSHostGNSSPPS;
  }
}

void app_gnsspps_notify_fromISR(uint32_t notValue)
{
  BaseType_t xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  if (gnssppsThreadHandler != NULL)
  {
    xTaskNotifyFromISR(gnssppsThreadHandler, notValue, eSetBits, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void app_gnsspps_notify(uint32_t notValue)
{
  if (gnssppsThreadHandler != NULL)
  {
    xTaskNotify(gnssppsThreadHandler, notValue, eSetBits);
  }
}

