/*
 * app_adc.c
 *
 *  Created on: Dec 15, 2023
 *      Author: Sarah Goossens
 */

#include "main.h"
#include "app_adc.h"
#include "usart.h"              // to declare huart1
#include "adc.h"
#include <string.h>
#include "app_init.h"
#include "app_gateway.h"
#include "app_supercap.h"

#define PRINTF_APP_ADC 1

extern char                   uart_buf[200];
extern ADC_ChannelConfTypeDef configADC;
extern uint16_t               installedBatteryVoltage;
extern uint16_t               batteryVoltageLevel;
extern uint16_t               batteryInVoltageLevel;
extern uint16_t               supercapVoltageLevel;
extern uint16_t               vCoreVoltageLevel;
extern uint16_t               vCoreTemperature;
extern uint16_t               uhADCxConvertedData;
extern uint16_t               supplyVoltageDetermined;
//extern uint32_t               radioBusy;

TaskHandle_t adcThreadHandler;

void AdcThreadInit()
{
  if (xTaskCreate ((TaskFunction_t)StartAdcThread, "AdcThread", (configSTACK_DEPTH_TYPE)512, NULL, osPriorityHigh, &adcThreadHandler) != pdPASS)
  {
#if PRINTF_APP_ADC
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_adc] [AdcThread] Could not be started.\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
}

void StartAdcThread(const void * params)
{
  float wrongVoltageFactor               = 0;
  uint32_t tempInstalledBatteryVoltage   = 0;
  uint32_t finalBatteryVoltageLevelCount = 0;
  uint32_t whileIterations               = 0; // to prevent lock into while loop

#if PRINTF_APP_ADC
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_adc] [AdcThread] Started.\n",(unsigned int) xTaskGetTickCount());
  huart2print(uart_buf, strlen(uart_buf));
#endif
  /* Perform ADC calibration */
  if (HAL_ADCEx_Calibration_Start(&hadc4) != HAL_OK)
  { // Calibration Error
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [main] ADC calibration error.\r\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
  }
  else
  {
#if PRINTF_APP_ADC
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [main] ADC calibration done.\r\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
  uint32_t   notificationValue = 0;
  TickType_t xLastWakeTime     = xTaskGetTickCount();
  for(;;)
  { // Infinite loop
    //******************************************************
	// Configure ADC channel to measure V-CORE voltage level
	// This is to determine the supplied voltage level
    //******************************************************

    while (!supplyVoltageDetermined)
    {
      configADC.Channel = ADC_CHANNEL_VCORE;
      if (HAL_ADC_ConfigChannel(&hadc4, &configADC) != HAL_OK)
      {  // Error: ADC regular configuration could not be performed
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_adc] Error: ADC regular configuration for V-Core could not be done.\r\n",(unsigned int) xTaskGetTickCount());
        huart2print(uart_buf, strlen(uart_buf));
      }
      if (HAL_ADC_Start_IT(&hadc4) != HAL_OK)  // Start measure V-Core Voltage level
      { // Error: ADC conversion start could not be performed
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_adc] Error: ADC conversion start for V-Core could not be performed.\r\n",(unsigned int) xTaskGetTickCount());
        huart2print(uart_buf, strlen(uart_buf));
      }
      xTaskNotifyStateClear(adcThreadHandler);
      notificationValue = 0;
      xTaskNotifyWait(0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(1000));  // Wait until ADC conversion is done
      if ((notificationValue & NOTIFICATION_FROM_ADC) == NOTIFICATION_FROM_ADC)
      { // conversion notification received from ADC
        uhADCxConvertedData = HAL_ADC_GetValue(&hadc4);  // Retrieve ADC conversion data
        vCoreVoltageLevel = __LL_ADC_CALC_DATA_TO_VOLTAGE(installedBatteryVoltage, uhADCxConvertedData, LL_ADC_RESOLUTION_12B);  // from RAW to physical value
#if PRINTF_APP_ADC
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_adc] Measured V_Core = %umV, reference voltage level = %umV.\r\n",(unsigned int) xTaskGetTickCount(), (unsigned int) vCoreVoltageLevel, (unsigned int) installedBatteryVoltage);
        huart2print(uart_buf, strlen(uart_buf));
#endif
        if (vCoreVoltageLevel == 900)
        { // check if vCore Voltage is 900mV, if not, adapt the Battery voltage level
          finalBatteryVoltageLevelCount++;
        }
        else
        {
          finalBatteryVoltageLevelCount = 0;
          if (vCoreVoltageLevel < 900)
          { // supplied voltage level is greater than 3300mV
            wrongVoltageFactor = (float) (((float) 900 - (float) vCoreVoltageLevel) / (float) 900);
            tempInstalledBatteryVoltage = (unsigned int) ((float)installedBatteryVoltage * (1 + wrongVoltageFactor));
#if PRINTF_APP_ADC
            waitToPrint();
            npf_snprintf(uart_buf, 200, "%u [app_adc] Vcore Voltage level too low, wrongVoltageFactor = %f, (%u * (1 + wrongVoltageFactor)) = %umV.\r\n",(unsigned int) xTaskGetTickCount(), wrongVoltageFactor, (unsigned int) installedBatteryVoltage, (unsigned int) tempInstalledBatteryVoltage);
            huart2print(uart_buf, strlen(uart_buf));
#endif
          }
          else
          { // supplied voltage level is less then 3300mV
            wrongVoltageFactor = (float) (((float) vCoreVoltageLevel - (float) 900) / (float) vCoreVoltageLevel);
            tempInstalledBatteryVoltage = (unsigned int) ((float)installedBatteryVoltage / (1 + wrongVoltageFactor));
#if PRINTF_APP_ADC
            waitToPrint();
            npf_snprintf(uart_buf, 200, "%u [app_adc] Vcore Voltage level too high, wrongVoltageFactor = %f, (%u / (1 + wrongVoltageFactor)) = %umV.\r\n",(unsigned int) xTaskGetTickCount(), wrongVoltageFactor, (unsigned int) installedBatteryVoltage, (unsigned int) tempInstalledBatteryVoltage);
            huart2print(uart_buf, strlen(uart_buf));
#endif
          }
          if (wrongVoltageFactor < 0.003)
          { // accuracy is good enough
#if PRINTF_APP_ADC
            waitToPrint();
            npf_snprintf(uart_buf, 200, "%u [app_adc] Accuracy good enough to determine supplied voltage level.\r\n",(unsigned int) xTaskGetTickCount());
            huart2print(uart_buf, strlen(uart_buf));
#endif
            finalBatteryVoltageLevelCount = 4;
          }

        }
        if (tempInstalledBatteryVoltage)
        {
          installedBatteryVoltage = tempInstalledBatteryVoltage;
        }
        else
        {
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_adc] Error on wrongVoltageFactor = %f, (%u / (1 + wrongVoltageFactor)) = %umV. This error did not take effect\r\n",(unsigned int) xTaskGetTickCount(), wrongVoltageFactor, (unsigned int) installedBatteryVoltage, (unsigned int) tempInstalledBatteryVoltage);
          huart2print(uart_buf, strlen(uart_buf));
        }
        if (finalBatteryVoltageLevelCount == 4)
        {
          if ((installedBatteryVoltage > 3000) && (installedBatteryVoltage < 3400))
          {
            installedBatteryVoltage = 3300;
          }
          // todo include also other possible battery voltage levels!
#if PRINTF_APP_ADC
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_adc] Battery voltage fixed to %umV.\r\n",(unsigned int) xTaskGetTickCount(), (unsigned int) installedBatteryVoltage);
          huart2print(uart_buf, strlen(uart_buf));
#endif
          supplyVoltageDetermined = 1;
        }
      }
      else
      { // wrong notification value received
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_adc] Error: Wrong notification received during V-Core voltage measurement: %u. Raw ADC data = %u.\r\n",
       		 (unsigned int) xTaskGetTickCount(), (unsigned int) notificationValue, (unsigned int) uhADCxConvertedData);
        huart2print(uart_buf, strlen(uart_buf));

      }
      if(whileIterations++ > 100)
      {
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_adc] Error: Jump out of while loop to determine Battery Voltage after 100 iterations.\r\n",(unsigned int) xTaskGetTickCount());
        huart2print(uart_buf, strlen(uart_buf));
        supplyVoltageDetermined = 1;
      }
    }
	//**************************************************************************************
	// Configure ADC channel IN4 (Battery In voltage level)
    // PA5     ------> ADC4_IN4 ---> BATTIN_LEVEL, this is before the resistor
	//**************************************************************************************
    configADC.Channel = ADC_CHANNEL_4;
    configADC.Rank = ADC_REGULAR_RANK_1;
    configADC.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
    if (HAL_ADC_ConfigChannel(&hadc4, &configADC) != HAL_OK)
    { // Error: ADC regular configuration could not be performed
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_adc] Error: ADC regular configuration for channel 4 (BATTIN_LEVEL) could not be done.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
    }
    xTaskNotifyStateClear(adcThreadHandler);
    notificationValue = 0;
    if (HAL_ADC_Start_IT(&hadc4) != HAL_OK) // Start measure Battery Voltage level
    { // Error: ADC conversion start could not be performed
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_adc] Error: ADC conversion start for channel 4 (BATTIN_LEVEL) could not be performed.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
    }
    xTaskNotifyWait( 0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(1000)); // Wait until ADC conversion is done
    if ((notificationValue & NOTIFICATION_FROM_ADC) == NOTIFICATION_FROM_ADC)
    { // conversion notification received from ADC
      uhADCxConvertedData = HAL_ADC_GetValue(&hadc4); // Retrieve ADC conversion data
      batteryInVoltageLevel = __LL_ADC_CALC_DATA_TO_VOLTAGE(installedBatteryVoltage, uhADCxConvertedData, LL_ADC_RESOLUTION_12B); // from RAW to physical value
      if (!batteryInVoltageLevel)
      {
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_adc] Error: BATTIN_LEVEL Voltage is zero. Raw ADC data = %u.\r\n", (unsigned int) xTaskGetTickCount(), (unsigned int) uhADCxConvertedData);
        huart2print(uart_buf, strlen(uart_buf));
      }
    }
    else
    {
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_adc] Error: Wrong notification received during BATTIN_LEVEL voltage measurement: %u. Raw ADC data = %u.\r\n",
     		 (unsigned int) xTaskGetTickCount(), (unsigned int) notificationValue, (unsigned int) uhADCxConvertedData);
      huart2print(uart_buf, strlen(uart_buf));
    }
    //********************************************************************************************************************************
	// Configure ADC channel IN6 (+BATT)
    // PA3     ------> ADC4_IN6 ---> BATTERY_LEVEL, this is voltage behind the resistor (resistor of 1R between BATTIN and +BATT)
    //********************************************************************************************************************************
    configADC.Channel = ADC_CHANNEL_6;
    if (HAL_ADC_ConfigChannel(&hadc4, &configADC) != HAL_OK)
    { // Error: ADC regular configuration could not be performed
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_adc] Error: ADC regular configuration for channel 6 (+BATT) could not be done.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
    }
    xTaskNotifyStateClear(adcThreadHandler);
    notificationValue = 0;
    if (HAL_ADC_Start_IT(&hadc4) != HAL_OK)  // Start measure Battery-in Voltage level
    { // Error: ADC regular configuration could not be performed
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_adc] Error: ADC conversion start for channel 6 (+BATT) could not be performed.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
    }
    xTaskNotifyWait( 0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(1000)); // Wait until ADC conversion is done
    if ((notificationValue & NOTIFICATION_FROM_ADC) == NOTIFICATION_FROM_ADC)
    { // conversion notification received from ADC
      uhADCxConvertedData = HAL_ADC_GetValue(&hadc4); // Retrieve ADC conversion data
      batteryVoltageLevel = __LL_ADC_CALC_DATA_TO_VOLTAGE(installedBatteryVoltage, uhADCxConvertedData, LL_ADC_RESOLUTION_12B);  // from RAW to physical value
      if (!batteryVoltageLevel)
      {
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_adc] Error: BATT_LEVEL Voltage is zero. Raw ADC data = %u. This module does not measure BATT_LEVEL.\r\n", (unsigned int) xTaskGetTickCount(), (unsigned int) uhADCxConvertedData);
        huart2print(uart_buf, strlen(uart_buf));
      }
    }
    else
    {
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_adc] Error: Wrong notification received during Battery-in voltage measurement: %u. Raw ADC data = %u.\r\n",
   		 (unsigned int) xTaskGetTickCount(), (unsigned int) notificationValue, (unsigned int) uhADCxConvertedData);
      huart2print(uart_buf, strlen(uart_buf));
    }
    //***************************************************************************************
	// Configure ADC channel IN1 (+VDC or super capacitor voltage level)
    // PA8     ------> ADC4_IN1 ---> SCAP_LEVEL     this is +VDC or voltage over the supercap
    //***************************************************************************************
    configADC.Channel = ADC_CHANNEL_1;
    if (HAL_ADC_ConfigChannel(&hadc4, &configADC) != HAL_OK)
    {
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [main] ADC regular configuration for channel 1 (+VDC) could not be done.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
    }
    xTaskNotifyStateClear(adcThreadHandler);
    notificationValue = 0;
    if (HAL_ADC_Start_IT(&hadc4) != HAL_OK)
    {
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [main] ADC conversion start for channel 1 (+VDC) could not be performed.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
    }
    xTaskNotifyWait( 0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(1000));
    if ((notificationValue & NOTIFICATION_FROM_ADC) == NOTIFICATION_FROM_ADC)
    { // conversion notification received from ADC
      uhADCxConvertedData = HAL_ADC_GetValue(&hadc4);
      supercapVoltageLevel = __LL_ADC_CALC_DATA_TO_VOLTAGE(installedBatteryVoltage, uhADCxConvertedData, LL_ADC_RESOLUTION_12B);
      if ((supercapVoltageLevel > 1840) && supplyVoltageDetermined)
      { // super capacitor loaded, notify app_init (needed at start-up) and app_gateway (during execution)
//        InitThreadNotify(NOTIFICATION_FROM_ADC_SUPERCAP_READY);
//        GatewayNotify(NOTIFICATION_FROM_ADC_SUPERCAP_READY);
        ScapThreadNotify(NOTIFICATION_FROM_ADC_SUPERCAP_READY);
      }
    }
    else
    {
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_adc] Error: Wrong notification received during SuperCap voltage measurement: %u. Raw ADC data = %u.\r\n",
   		 (unsigned int) xTaskGetTickCount(), (unsigned int) notificationValue, (unsigned int) uhADCxConvertedData);
      huart2print(uart_buf, strlen(uart_buf));
    }
    //*********************************
	// Configure ADC channel TEMPSENSOR
    //*********************************
    configADC.Channel = ADC_CHANNEL_TEMPSENSOR;
    if (HAL_ADC_ConfigChannel(&hadc4, &configADC) != HAL_OK)
    {
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [main] ADC regular configuration for V-Core could not be done.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
    }
    if (HAL_ADC_Start_IT(&hadc4) != HAL_OK)
    {
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [main] ADC conversion start for V-Core could not be performed.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
    }
    xTaskNotifyStateClear(adcThreadHandler);
    notificationValue = 0;
    xTaskNotifyWait( 0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(1000));
    if ((notificationValue & NOTIFICATION_FROM_ADC) == NOTIFICATION_FROM_ADC)
    { // conversion notification received from ADC
      uhADCxConvertedData = HAL_ADC_GetValue(&hadc4);
      vCoreTemperature = __LL_ADC_CALC_TEMPERATURE(installedBatteryVoltage, uhADCxConvertedData, LL_ADC_RESOLUTION_12B);
    }
    else
    {
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_adc] Error: Wrong notification received during V-Core voltage measurement: %u. Raw ADC data = %u.\r\n",
   		 (unsigned int) xTaskGetTickCount(), (unsigned int) notificationValue, (unsigned int) uhADCxConvertedData);
      huart2print(uart_buf, strlen(uart_buf));
    }

//#if PRINTF_APP_ADC
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "%u [main] ADC conversion done (mV): BATT %umV, BATTIN %umV, SCAP %umV, V-Core %umV, Core temp = %u°C.\r\n",
//   		(unsigned int) xTaskGetTickCount(), (unsigned int) batteryVoltageLevel, (unsigned int) batteryInVoltageLevel,
//		(unsigned int) supercapVoltageLevel, (unsigned int) vCoreVoltageLevel, (unsigned int) vCoreTemperature);
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
    vTaskDelayUntil(&xLastWakeTime, 200U);
  }
}

void AdcNotifyFromISR(uint32_t notValue)
{
  BaseType_t xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  if (adcThreadHandler != NULL)
  {
    xTaskNotifyFromISR(adcThreadHandler, notValue, eSetBits, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void AdcThreadNotify(uint32_t notValue)
{
  if (adcThreadHandler != NULL)
  {
    xTaskNotify(adcThreadHandler, notValue, eSetBits);
  }
}
