/*
 * app_led.c
 *
 *               Created on: Oct 22, 2022
 *  Adapted for STM32WBA on: Oct 23, 2023
 *                   Author: Sarah Goossens
 */

#include "main.h"
#include "app_led.h"
#include "usart.h"              // to declare huart1
#include <string.h>
#include "app_rtc.h"

#define PRINTF_APP_LED 1

extern char     uart_buf[200];
extern uint64_t rtosTimeStampLedOn;
extern TickType_t ledFreq;
extern int calibrationActive;

#if STM32WBAUSED
  TaskHandle_t ledThreadHandler;
#else
  osThreadId ledThreadHandler;
#endif

void LedThreadInit()
{
#if STM32WBAUSED
  if (xTaskCreate ((TaskFunction_t)StartLedThread, "LedThread", (configSTACK_DEPTH_TYPE)512, NULL, osPriorityHigh, &ledThreadHandler) != pdPASS)
  {
#if PRINTF_APP_LED
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_led] [LedThread] Could not be started.\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
#else
  osThreadDef(LedThread, StartLedThread, osPriorityHigh, 0, 128);
  ledThreadHandler = osThreadCreate(osThread(LedThread), NULL);
#endif
}

void StartLedThread(const void * params)
{
  uint32_t notificationValue = 0; // Used to identify where the notification is coming from.
  uint32_t whileIterations   = 0; // to prevent lock into while loop
#if PRINTF_APP_LED
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_led] [LedThread] Started.\n",(unsigned int) xTaskGetTickCount());
  huart2print(uart_buf, strlen(uart_buf));
#endif
  for(;;)
  { // Infinite loop
#if PLANTSENSOR
    if (calibrationActive)
    {
      if (ledFreq)
      {
        HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);
        vTaskDelay(pdMS_TO_TICKS(200U));
      }
      HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
      vTaskDelay(pdMS_TO_TICKS(ledFreq));
    }
#else
    whileIterations   = 0;
    xTaskNotifyStateClear(ledThreadHandler);
    notificationValue = 0;
    while ((notificationValue & NOTIFICATION_FROM_APP_HAL_SYNC_TO_START_LED) != NOTIFICATION_FROM_APP_HAL_SYNC_TO_START_LED)
    { // waiting for a notification value from app_hal_sync
      xTaskNotifyWait( 0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(5000));
      if ((notificationValue & NOTIFICATION_FROM_APP_HAL_SYNC_TO_START_LED) == NOTIFICATION_FROM_APP_HAL_SYNC_TO_START_LED)
      {
#if SENSOR_HOST
        vTaskDelay(500U);
#else
        vTaskDelay(500U);
#endif
        rtosTimeStampLedOn = xTaskGetTickCount();
        HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
        vTaskDelay(490U);
        HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
      }
      else
      {
        if (!notificationValue)
        {
#if PRINTF_APP_LED
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_led] No notification from app_hal_sync in the last 5s to start the LED.\r\n",(unsigned int) xTaskGetTickCount());
          huart2print(uart_buf, strlen(uart_buf));
#endif
        }
        else
        {
#if PRINTF_APP_LED
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_led] Wrong notification value received to start the LED: 0x%08X.\r\n",(unsigned int) xTaskGetTickCount(), (unsigned int) notificationValue);
          huart2print(uart_buf, strlen(uart_buf));
#endif
        }
        xTaskNotifyStateClear(ledThreadHandler);
        notificationValue = 0;
      }
      if (whileIterations++ > 3)
      {
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_led] Error: Jump out of while loop waiting for a notification from app_hal_sync to start the LED.\r\n",(unsigned int) xTaskGetTickCount());
        huart2print(uart_buf, strlen(uart_buf));
        notificationValue = NOTIFICATION_FROM_APP_HAL_SYNC_TO_START_LED;
      }
    }
#endif
  }
}

//void app_led_notify_fromISR(uint32_t notValue)
//{
//  BaseType_t xHigherPriorityTaskWoken;
//  xHigherPriorityTaskWoken = pdFALSE;
//  if (ledThreadHandler != NULL)
//  {
//    xTaskNotifyFromISR(ledThreadHandler, notValue, eSetBits, &xHigherPriorityTaskWoken);
//  }
//  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//}

void LedThreadNotify(uint32_t notValue)
{
  if (ledThreadHandler != NULL)
  {
    xTaskNotify(ledThreadHandler, notValue, eSetBits);
  }
}

