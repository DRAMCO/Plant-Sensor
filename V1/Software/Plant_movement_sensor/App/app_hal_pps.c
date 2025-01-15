/*
 * app_hal_pps.c
 *
 *  Created on: 22 apr. 2023
 *      Author: Sarah Goossens
 *
 *  creates a notification every HAL second.
 *  This is being used to start the clock, as only the HAL clock remains working for both Host and Node.
 *
 */

#include "main.h"
#include "app_hal_pps.h"
#include "usart.h"              // to declare huart2
#include <string.h>
#include "app_rtc.h"

#define PRINTF_APP_HAL_PPS 1

extern char    uart_buf[200];

#if STM32WBAUSED
  TaskHandle_t halPPSThreadHandler;
#else
  osThreadId halPPSThreadHandler;
#endif

void HalPPSThreadInit()
{
#if STM32WBAUSED
  if (xTaskCreate ((TaskFunction_t)StartHalPPSThread, "HalPPSThread", (configSTACK_DEPTH_TYPE)512, NULL, osPriorityHigh, &halPPSThreadHandler) != pdPASS)
  {
#if PRINTF_APP_HAL_PPS
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_hal_pps] [HalPPSThread] Could not be started.\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
#else
  osThreadDef(HalPPSThread, StartHalPPSThread, osPriorityHigh, 0, 128);
  halPPSThreadHandler = osThreadCreate(osThread(halPPSThread), NULL);
#endif
}

void StartHalPPSThread(const void * params)
{
#if PRINTF_APP_HAL_PPS
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_hal_pps] [HalPPSThread] Started.\n",(unsigned int) xTaskGetTickCount());
  huart2print(uart_buf, strlen(uart_buf));
#endif
  uint32_t   hal_pps           = 0;
  uint32_t   notificationValue = 0;                     // Used to identify where the notification is coming from.
  TickType_t xMaxBlockTime     = pdMS_TO_TICKS(180000); // The maximum block time state of the task in ms when no notification is received (here set to 3m).
  TickType_t xLastWakeTime     = xTaskGetTickCount();
  for(;;)
  { // Infinite loop
    xTaskNotifyStateClear(halPPSThreadHandler);
    hal_pps++;
    if (hal_pps < 10)
    {
      app_rtc_notify(NOTIFICATION_FROM_HAL_PPS);
#if PRINTF_APP_HAL_PPS
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_hal_pps] halPPS.\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
#endif
    }
    else
    {
#if STM32WBAUSED
#if PRINTF_APP_HAL_PPS
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_hal_pps] Suspended.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
#endif
//        vTaskDelete(halPPSThreadHandler);
      vTaskSuspend(halPPSThreadHandler);
#else
#if PRINTF_APP_HAL_PPS
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_hal_pps] Terminated.\r\n",(unsigned int) xTaskGetTickCount());
        huart2print(uart_buf, strlen(uart_buf));
#endif
        osThreadTerminate(halPPSThreadHandler);
#endif
    }
    vTaskDelayUntil(&xLastWakeTime, 1000U);
  }
}

void app_hal_pps_notify(uint32_t notValue)
{
  if (halPPSThreadHandler != NULL)
  {
    xTaskNotify(halPPSThreadHandler, notValue, eSetBits);
  }
}


