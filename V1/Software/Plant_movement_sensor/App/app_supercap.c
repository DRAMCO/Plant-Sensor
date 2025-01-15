/*
 * app_supercap.c
 *
 *  Created on: Apr 13, 2024
 *      Author: Sarah Goossens
 *
 *      App to load supercap
 *
 */

/* Standard libraries */
#include <string.h>
#include <stdio.h>

/* Others */
#include "main.h"
#include "app_supercap.h"
#include "app_init.h"
#include "app_led.h"
#include "app_network_connect.h"
#include "app_gateway.h"
#include "app_adc.h"
#include "usart.h"                       // to declare huart1

#define PRINTF_APP_SCAP    1

extern char                  uart_buf[200];
extern uint16_t              batteryVoltageLevel;
extern uint16_t              batteryInVoltageLevel;
extern uint16_t              supercapVoltageLevel;
extern uint16_t              vCoreVoltageLevel;
extern uint16_t              vCoreTemperature;

extern uint64_t              startSuperCapOn;
extern uint64_t              superCapOnTime;
extern uint32_t              superCapAvailable;
extern uint32_t              superCapLoaded;

extern TickType_t            ledFreq;


TaskHandle_t scapThreadHandler;

void ScapThreadInit()
{
  if (xTaskCreate ((TaskFunction_t)StartScapThread, "ScapThread", (configSTACK_DEPTH_TYPE)512, NULL, osPriorityHigh, &scapThreadHandler) != pdPASS)
  {
#if PRINTF_APP_SCAP
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_supercap] [ScapThread] Could not be started.\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
}

void StartScapThread(const void * params)
{
#if PRINTF_APP_SCAP
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_supercap] [ScapThread] Started.\n",(unsigned int) xTaskGetTickCount());
  huart2print(uart_buf, strlen(uart_buf));
#endif
  uint32_t             superCapCounter = 0;
  uint32_t           notificationValue = 0;      // Used to identify where the notification is coming from.
  uint32_t   supercapVoltageDifference = 0;
  uint32_t     oldSupercapVoltageLevel = 0;
  uint32_t initialSupercapVoltageLevel = 0;

  for(;;)
  { // Infinite loop
#if AUTOMATESUPERCAP
	if (supercapVoltageLevel < 1850)
	{
      if (superCapAvailable != 100)
      { // if the radio's are being powered with the super capacitor, loading starts, otherwise skip loading process
#if PRINTF_APP_SCAP
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_supercap] Super capacitor voltage level below 1850mV, start loading process.\r\n", (unsigned int) xTaskGetTickCount());
        huart2print(uart_buf, strlen(uart_buf));
#endif
        //
        // switch on the super capacitor until voltage level is 1850mV
        //
        HAL_GPIO_WritePin(SCAP_ON_GPIO_Port, SCAP_ON_Pin, GPIO_PIN_SET);   // Supercap on
        ledFreq = 2000; // led off time 2000ms, on time 200ms
#if PRINTF_APP_SCAP
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_supercap] Super capacitor loading (iteration #%u), SCAP = %umV, BATT = %umV, BATTIN = %umV, Core = %umV, Core temp = %u°C.\r\n",
            (unsigned int) xTaskGetTickCount(), (unsigned int) superCapCounter, (unsigned int) supercapVoltageLevel, (unsigned int) batteryVoltageLevel,
            (unsigned int) batteryInVoltageLevel,(unsigned int) vCoreVoltageLevel, (unsigned int) vCoreTemperature);
        huart2print(uart_buf, strlen(uart_buf));
#endif
        if (superCapCounter)
        {
          supercapVoltageDifference = supercapVoltageLevel - oldSupercapVoltageLevel;
        }
        else
        {
          initialSupercapVoltageLevel = supercapVoltageLevel;
        }
        oldSupercapVoltageLevel = supercapVoltageLevel;
        startSuperCapOn = xTaskGetTickCount();
        xTaskNotifyStateClear(scapThreadHandler);
        notificationValue = 0;
        while ((notificationValue & NOTIFICATION_FROM_ADC_SUPERCAP_READY) != NOTIFICATION_FROM_ADC_SUPERCAP_READY)
        { // waiting for a notification value from the ADC that the voltage of the super capacitor is 1800mV
          xTaskNotifyWait( 0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(1000));
          if ((notificationValue & NOTIFICATION_FROM_ADC_SUPERCAP_READY) != NOTIFICATION_FROM_ADC_SUPERCAP_READY)
          {
            superCapCounter++;
#if PRINTF_APP_SCAP
            waitToPrint();
            npf_snprintf(uart_buf, 200, "%u [app_supercap] Super capacitor loading (iteration #%u), SCAP = %umV, BATT = %umV, BATTIN = %umV, Core = %umV, Core temp = %u°C.\r\n",
                (unsigned int) xTaskGetTickCount(), (unsigned int) superCapCounter, (unsigned int) supercapVoltageLevel, (unsigned int) batteryVoltageLevel,
                (unsigned int) batteryInVoltageLevel,(unsigned int) vCoreVoltageLevel, (unsigned int) vCoreTemperature);
            huart2print(uart_buf, strlen(uart_buf));
#endif
            xTaskNotifyStateClear(scapThreadHandler);
            notificationValue = 0;
            if ((superCapCounter > 15) && ((supercapVoltageLevel - initialSupercapVoltageLevel) < 50))
            {
#if PRINTF_APP_SCAP
  	        waitToPrint();
   	        npf_snprintf(uart_buf, 200, "%u [app_supercap] It looks that the Radio's are directly powered, not via super capacitor.\r\n", (unsigned int) xTaskGetTickCount());
            huart2print(uart_buf, strlen(uart_buf));
#endif
  	        superCapAvailable = 100;
            notificationValue = NOTIFICATION_FROM_ADC_SUPERCAP_READY;
          }
        }
      } // end of while notificationValue != NOTIFICATION_FROM_ADC_SUPERCAP_READY
      HAL_GPIO_WritePin(SCAP_ON_GPIO_Port, SCAP_ON_Pin, GPIO_PIN_RESET); // Super Capacitor off
      ledFreq = 0; // led off
      superCapOnTime = xTaskGetTickCount() - startSuperCapOn;
      superCapCounter = 0;
#if PRINTF_APP_SCAP
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_supercap] Super capacitor loaded (load time = %ums), SCAP = %umV, BATT = %umV, BATTIN = %umV, Core = %umV, Core temp = %u°C.\r\n",
          (unsigned int) xTaskGetTickCount(), (unsigned int) superCapOnTime, (unsigned int) supercapVoltageLevel, (unsigned int) batteryVoltageLevel,
          (unsigned int) batteryInVoltageLevel, (unsigned int) vCoreVoltageLevel, (unsigned int) vCoreTemperature);
      huart2print(uart_buf, strlen(uart_buf));
#endif
      }
      else
      {
#if PRINTF_APP_SCAP
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_supercap] Radio's are not powered via super cap, skip loading process.\r\n", (unsigned int) xTaskGetTickCount());
        huart2print(uart_buf, strlen(uart_buf));
#endif
      }
      InitThreadNotify(NOTIFICATION_FROM_SCAP_SUPERCAP_READY);  // notify app_init that the supercap is loaded
#if PLANTSENSOR
      GatewayNotify(NOTIFICATION_FROM_SCAP_SUPERCAP_READY);     // notify app_gateway that the supercap is loaded
#else
      NwConThreadNotify(NOTIFICATION_FROM_SCAP_SUPERCAP_READY); // notify app_network_connect that the supercap is loaded
#endif
    }
    vTaskDelay(500);
#else
	xTaskNotifyStateClear(scapThreadHandler);
    while ((notificationValue & NOTIFICATION_LOAD_SCAP) != NOTIFICATION_LOAD_SCAP)
    { // waiting for a notification value from either app_gateway or appnetwork_connect to start loading the supercap
      xTaskNotifyWait( 0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(60000));
      if ((notificationValue & NOTIFICATION_LOAD_SCAP) == NOTIFICATION_LOAD_SCAP)
      {
        if (superCapAvailable != 100)
        { // if the radio's are being powered with the super capacitor, loading starts, otherwise skip loading process
#if PRINTF_APP_SCAP
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_supercap] Check Super capacitor voltage level.\r\n", (unsigned int) xTaskGetTickCount());
          huart2print(uart_buf, strlen(uart_buf));
#endif
          //
          // switch on the super capacitor until voltage level is 1850mV
          //
          HAL_GPIO_WritePin(SCAP_ON_GPIO_Port, SCAP_ON_Pin, GPIO_PIN_SET);   // Supercap on
          ledFreq = 2000; // led off time 2000ms, on time 200ms
#if PRINTF_APP_SCAP
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_supercap] Super capacitor loading (iteration #%u), SCAP = %umV, BATT = %umV, BATTIN = %umV, Core = %umV, Core temp = %u°C.\r\n",
              (unsigned int) xTaskGetTickCount(), (unsigned int) superCapCounter, (unsigned int) supercapVoltageLevel, (unsigned int) batteryVoltageLevel,
              (unsigned int) batteryInVoltageLevel,(unsigned int) vCoreVoltageLevel, (unsigned int) vCoreTemperature);
          huart2print(uart_buf, strlen(uart_buf));
#endif
          if (superCapCounter)
          {
        	supercapVoltageDifference = supercapVoltageLevel - oldSupercapVoltageLevel;
          }
          else
          {
        	initialSupercapVoltageLevel = supercapVoltageLevel;
          }
          oldSupercapVoltageLevel = supercapVoltageLevel;
          startSuperCapOn = xTaskGetTickCount();
          xTaskNotifyStateClear(scapThreadHandler);
          notificationValue = 0;
          while ((notificationValue & NOTIFICATION_FROM_ADC_SUPERCAP_READY) != NOTIFICATION_FROM_ADC_SUPERCAP_READY)
          { // waiting for a notification value from the ADC that the voltage of the super capacitor is 1800mV
            xTaskNotifyWait( 0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(1000));
            if ((notificationValue & NOTIFICATION_FROM_ADC_SUPERCAP_READY) != NOTIFICATION_FROM_ADC_SUPERCAP_READY)
            {
              superCapCounter++;
#if PRINTF_APP_SCAP
              waitToPrint();
              npf_snprintf(uart_buf, 200, "%u [app_supercap] Super capacitor loading (iteration #%u), SCAP = %umV, BATT = %umV, BATTIN = %umV, Core = %umV, Core temp = %u°C.\r\n",
                  (unsigned int) xTaskGetTickCount(), (unsigned int) superCapCounter, (unsigned int) supercapVoltageLevel, (unsigned int) batteryVoltageLevel,
                  (unsigned int) batteryInVoltageLevel,(unsigned int) vCoreVoltageLevel, (unsigned int) vCoreTemperature);
              huart2print(uart_buf, strlen(uart_buf));
#endif
              xTaskNotifyStateClear(scapThreadHandler);
              notificationValue = 0;
#if !SUPERCAPUSED
              if ((superCapCounter > 15) && ((supercapVoltageLevel - initialSupercapVoltageLevel) < 50))
              {
#if PRINTF_APP_SCAP
       	        waitToPrint();
       	        npf_snprintf(uart_buf, 200, "%u [app_supercap] It looks that the Radio's are directly powered, not via super capacitor.\r\n", (unsigned int) xTaskGetTickCount());
       	        huart2print(uart_buf, strlen(uart_buf));
#endif
       	        superCapAvailable = 100;
                notificationValue = NOTIFICATION_FROM_ADC_SUPERCAP_READY;
              }
#endif
            }
          } // end of while notificationValue != NOTIFICATION_FROM_ADC_SUPERCAP_READY
          HAL_GPIO_WritePin(SCAP_ON_GPIO_Port, SCAP_ON_Pin, GPIO_PIN_RESET); // Super Capacitor off
          ledFreq = 0; // led off
          superCapOnTime = xTaskGetTickCount() - startSuperCapOn;
          superCapCounter = 0;
#if PRINTF_APP_SCAP
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_supercap] Super capacitor loaded (load time = %ums), SCAP = %umV, BATT = %umV, BATTIN = %umV, Core = %umV, Core temp = %u°C.\r\n",
              (unsigned int) xTaskGetTickCount(), (unsigned int) superCapOnTime, (unsigned int) supercapVoltageLevel, (unsigned int) batteryVoltageLevel,
              (unsigned int) batteryInVoltageLevel, (unsigned int) vCoreVoltageLevel, (unsigned int) vCoreTemperature);
          huart2print(uart_buf, strlen(uart_buf));
#endif
        }
        else
        {
#if PRINTF_APP_SCAP
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_supercap] Radio's are not powered via super cap, skip loading process.\r\n", (unsigned int) xTaskGetTickCount());
          huart2print(uart_buf, strlen(uart_buf));
#endif
        }
        InitThreadNotify(NOTIFICATION_FROM_SCAP_SUPERCAP_READY);  // notify app_init that the supercap is loaded
#if PLANTSENSOR
        GatewayNotify(NOTIFICATION_FROM_SCAP_SUPERCAP_READY);     // notify app_gateway that the supercap is loaded
#else
        NwConThreadNotify(NOTIFICATION_FROM_SCAP_SUPERCAP_READY); // notify app_network_connect that the supercap is loaded
#endif
      }
      else
      { // notificationValue != NOTIFICATION_LOAD_SCAP
        if(notificationValue)
        {
          if (notificationValue != NOTIFICATION_FROM_ADC_SUPERCAP_READY)
          {
#if PRINTF_APP_SCAP
            waitToPrint();
            npf_snprintf(uart_buf, 200, "%u [app_supercap] Wrong notification value received: 0x%02X, SCAP = %umV, BATT = %umV, BATTIN = %umV, Core = %umV, Core temp = %u°C.\r\n",
              (unsigned int) xTaskGetTickCount(), (unsigned int) notificationValue, (unsigned int) supercapVoltageLevel, (unsigned int) batteryVoltageLevel,
              (unsigned int) batteryInVoltageLevel,(unsigned int) vCoreVoltageLevel, (unsigned int) vCoreTemperature);
            huart2print(uart_buf, strlen(uart_buf));
#endif
          }
        }
        else
        {
        // todo check the voltage level. If the voltage level is below 1200mV, load the supercap...
#if PRINTF_APP_SCAP
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_supercap] No request received in last 60s to load SCAP, SCAP = %umV, BATT = %umV, BATTIN = %umV, Core = %umV, Core temp = %u°C.\r\n",
            (unsigned int) xTaskGetTickCount(), (unsigned int) supercapVoltageLevel, (unsigned int) batteryVoltageLevel,
            (unsigned int) batteryInVoltageLevel,(unsigned int) vCoreVoltageLevel, (unsigned int) vCoreTemperature);
          huart2print(uart_buf, strlen(uart_buf));
#endif

        }
      }
    } // end of while notificationValue != NOTIFICATION_LOAD_SCAP
#endif //AUTOMATESUPERCAP
  } // end of infinite loop
}

void ScapThreadNotify(uint32_t notValue)
{
  if (scapThreadHandler != NULL)
  {
    xTaskNotify(scapThreadHandler, notValue, eSetBits);
  }
}


