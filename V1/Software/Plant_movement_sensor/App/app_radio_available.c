/*
 * app_radio_available.c
 *
 *  Created on: May 13, 2023
 *      Author: Sarah Goossens
 */

#include "app_radio_available.h"
#include "usart.h"              // to declare huart2
#include <string.h>
//#include "app_rtc.h"
#include "app_init.h"

#define PRINTF_APP_RADIO_AVAILABLE 1

extern char uart_buf[200];
extern uint8_t  radioAvailable;
//extern osThreadId initThreadHandler;
extern TaskHandle_t * initThreadHandler;

//osThreadId radioAvailableThreadHandler;
TaskHandle_t radioAvailableThreadHandler;

void RadioAvailableThreadInit()
{
//  osThreadDef(radioAvailableThread, rtos_radio_available_thread, osPriorityNormal, 0, 128);
//  radioAvailableThreadHandler = osThreadCreate(osThread(radioAvailableThread), NULL);

  if (xTaskCreate ((TaskFunction_t)RadioAvailableThreadStart, "RadioAvailableThread", (configSTACK_DEPTH_TYPE)512, NULL, osPriorityNormal, &radioAvailableThreadHandler) != pdPASS)
  {
#if PRINTF_APP_RADIO_AVAILABLE
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_radio_available] Could not be started.\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
}

void RadioAvailableThreadStart(const void * params)
{
#if PRINTF_APP_RADIO_AVAILABLE
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_radio_available] [rtos_radio_available_thread] Started.\n",(unsigned int) xTaskGetTickCount());
  huart2print(uart_buf, strlen(uart_buf));
#endif
  uint32_t   notificationValue = 0;                     // Used to identify where the notification is coming from.
//  BaseType_t xHigherPriorityTaskWoken;
//  xHigherPriorityTaskWoken = pdFALSE;
  for(;;)
  { // Infinite loop
//    xTaskNotifyStateClear(radioAvailableThreadHandler);
    if (xTaskNotifyWait(0x00, 0xffffffff, &notificationValue, 20000))
    {
      if ((notificationValue & NOTIFICATION_FROM_RADIO_AVAILABLE) == NOTIFICATION_FROM_RADIO_AVAILABLE)
      { // Interrupt from RADIO_BUSY_Pin (see main HAL_GPIO_EXTI_Callback()) that the Radio is ready
	radioAvailable = 1;
#if PRINTF_APP_RADIO_AVAILABLE
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_radio_available] Radio available.\r\n",(unsigned int) xTaskGetTickCount());
        huart2print(uart_buf, strlen(uart_buf));
#endif

        if (initThreadHandler != NULL)
        {
          xTaskNotify(initThreadHandler, notificationValue, eSetBits);
//          portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}

      }
      else
      {
#if PRINTF_APP_RADIO_AVAILABLE
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_radio_available] Wrong notification value received.\r\n",(unsigned int) xTaskGetTickCount());
        huart2print(uart_buf, strlen(uart_buf));
#endif
      }
    }
    else
    {
#if PRINTF_APP_RADIO_AVAILABLE
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_radio_available] Radio Busy since last 20s.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
#endif
    }
  }
}

void RadioAvailableNotifyFromISR(uint32_t notValue)
{
  BaseType_t xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  if (radioAvailableThreadHandler != NULL)
  {
    xTaskNotifyFromISR(radioAvailableThreadHandler, notValue, eSetBits, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
