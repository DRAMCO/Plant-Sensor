/*
 * app_rtc_sync.c
 *
 *  Created on: Apr 11, 2023
 *      Author: Sarah Goossens
 *
 *  Synchronization of the RTC clock with an external clock (either GNSS clock in case of sensor host or host clock in case of sensor node.
 *  The intention is that this compensation value is implemented in the RTC itself. This part is not yet implemented
 *
 */

#include "main.h"
#include "FreeRTOS.h"
#if !STM32WBAUSED
  #include "cmsis_os.h"
#endif
#include "app_rtc_sync.h"
#include "app_rtc.h"
#include "app_hal_sync.h"
#include "usart.h"              // to declare huart2
#include "app_network_connect.h"
#include <string.h>
#include "../Drivers/PCF2131/PCF2131.h"

#define PRINTF_APP_RTC_SYNC 1

extern char              uart_buf[200];
extern uint64_t          timeStampInt; // RTOS tick at the time of the external interrupt (in this case NINTA or RTC_PPS)
extern uint64_t          timeStamp8;
extern uint32_t          gnss_pps;
extern uint32_t          duration_100_gnss_pps;
extern uint64_t          rtosTimeStampGNSSPPS;
extern uint64_t          rtosTimeStampLedOn;
extern time_t            gnssEpoch;
extern SemaphoreHandle_t gnssEpochMutex;
extern SemaphoreHandle_t timeStampIntMutex;
extern uint8_t           halSynchronized;
extern uint64_t          rtosTimeStampRTCPPS;
extern uint8_t           changeClock;
extern uint32_t          agingOffsetPCF2131;
extern uint32_t          pairingOngoing;
extern uint64_t          previousrtosTimeStampGNSSPPS;
extern uint32_t          rtosDriftToGNSS;
extern uint32_t          totalRtosDriftToGNSS;
extern uint32_t          rtcSynchronized;
extern float             tickSpeedToReference;
extern double            sysTickCurrentTimeStamp;


#if STM32WBAUSED
   TaskHandle_t rtcSyncThreadHandler;
#else
  osThreadId rtcSyncThreadHandler;
#endif

void RtcSyncThreadInit()
{
#if STM32WBAUSED
  if (xTaskCreate ((TaskFunction_t)StartRtcSyncThread, "RtcSyncThread", (configSTACK_DEPTH_TYPE)512, NULL, osPriorityRealtime, &rtcSyncThreadHandler) != pdPASS)
  {
#if PRINTF_APP_RTC_SYNC
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_rtc_sync] [RtcSyncThread] Could not be started.\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
#else
  osThreadDef(rtcSyncThread, StartRtcSyncThread, osPriorityHigh, 0, 128);
  rtcSyncThreadHandler = osThreadCreate(osThread(rtcSyncThread), NULL);
#endif
}

void StartRtcSyncThread(const void * params)
{
#if PRINTF_APP_RTC_SYNC
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_rtc_sync] [StartRtcSyncThread] Started.\n",(unsigned int) xTaskGetTickCount());
  huart2print(uart_buf, strlen(uart_buf));
#endif
//  float      rtcDrift          = 0;
  uint32_t   notificationValue            = 0; // Used to identify where the notification is coming from.
  uint32_t   GNSSPPSAlignedWithTimeStamp  = 0; // to align once the gnss_pps value with rtosTimeStampGNSSPPS
  uint32_t   whileIterations              = 0; // to prevent lock into while loop
  uint32_t   netConStarted                = 0;
#if !SENSOR_HOST
  int32_t    differenceGNSSPPSRTCPPS      = 0;
  int32_t    statsDifferenceGNSSPPSRTCPPS = 0;
  int32_t    statisticsRTCAccuracy[11]    = {0,0,0,0,0,0,0,0,0,0,0};
#endif
  int32_t    staticDifferenceGNSSPPSrtosTimeStamp = 0;

  uint64_t sysTicknsValueGNSSPPS          = 0;

HalSyncThreadInit(); // wait until RTC_PPS is working

  for(;;)
  { // Infinite loop
    whileIterations   = 0;
    notificationValue = 0;
    xTaskNotifyStateClear(rtcSyncThreadHandler);
#if SENSOR_HOST
    while ((notificationValue & NOTIFICATION_FROM_GNSS_PPS) != NOTIFICATION_FROM_GNSS_PPS)
    { // waiting for a notification value from GNSS_PPS - Interrupt from GNSS PPS_Pin - see main HAL_GPIO_EXTI_Callback().
      xTaskNotifyWait(0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(1200));
      if ((notificationValue & NOTIFICATION_FROM_GNSS_PPS) == NOTIFICATION_FROM_GNSS_PPS)
      {
   	    xSemaphoreTake(timeStampIntMutex, pdMS_TO_TICKS(400));
   	    sysTicknsValueGNSSPPS = (uint64_t)(sysTickCurrentTimeStamp * 62.5);
   	    rtosTimeStampGNSSPPS = timeStampInt * 1000000 + sysTicknsValueGNSSPPS;
   	    xSemaphoreGive(timeStampIntMutex);
   		if (changeClock)
   		{ // give a notification to app_rtc to start the RTC when change clock = 1.
   		  app_rtc_notify(NOTIFICATION_FROM_APP_RTC_SYNC_START_RTC);
   		}
   	    //    xSemaphoreTake(gnssEpochMutex, pdMS_TO_TICKS(400));
   	    //    gnssEpoch++;
   	    //    xSemaphoreGive(gnssEpochMutex);
   	    gnss_pps++;

   	    //20240811 Align gnss_pps statically with rtos tick:
//        if (!GNSSPPSAlignedWithTimeStamp)
//        {
//          gnss_pps = (uint32_t) (rtosTimeStampGNSSPPS / 1000000);
//          gnss_pps++;
//          GNSSPPSAlignedWithTimeStamp = 1;
//          staticDifferenceGNSSPPSrtosTimeStamp = gnss_pps * 1000 - rtosTimeStampGNSSPPS / 1000;
//          for (int i = 1; i < staticDifferenceGNSSPPSrtosTimeStamp; i++)
//          {
//            HAL_IncTick();
//          }
//          xTaskCatchUpTicks((TickType_t) staticDifferenceGNSSPPSrtosTimeStamp);
//        }
//        else
//        {
#if PRINTF_APP_RTC_SYNC
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u %u [app_rtc_sync] GNSSPPS #%u @ RTOS time: %uus.\r\n",
             (unsigned int) HAL_GetTick(), (unsigned int) xTaskGetTickCount(), (unsigned int) gnss_pps, (unsigned int) (rtosTimeStampGNSSPPS / 1000));
          huart2print(uart_buf, strlen(uart_buf));
#endif

//        }

   	    if (rtcSynchronized)
   	    {
          if (gnss_pps%10 == 9)
          { // no RTCPID should be done, as the RTC should not be changed when we are close to a broadcast
        	if (!netConStarted)
            {
              NetConThreadInit(); // Now app_network_connect for the Host can be started
              netConStarted = 1;
            }
            pairingOngoing = 1;
//#if PRINTF_APP_RTC_SYNC
//            waitToPrint();
//            npf_snprintf(uart_buf, 200, "%u [app_rtc_sync] Pairing process started.\r\n", (unsigned int) xTaskGetTickCount());
//            huart2print(uart_buf, strlen(uart_buf));
//#endif
          }
          if (!(gnss_pps%10))
          {// do a broadcast every GNSSPP10S
            NwConThreadNotify(NOTIFICATION_FROM_GNSSPP10S_TO_START_BROADCAST);
//#if PRINTF_APP_RTC_SYNC
//            waitToPrint();
//            npf_snprintf(uart_buf, 200, "%u [app_rtc_sync] GNSSPPS #%u, start broadcast.\r\n",(unsigned int) xTaskGetTickCount(), (unsigned int) gnss_pps);
//            huart2print(uart_buf, strlen(uart_buf));
//#endif
            rtosDriftToGNSS = rtosTimeStampGNSSPPS - previousrtosTimeStampGNSSPPS;
            totalRtosDriftToGNSS += (rtosDriftToGNSS - 10000000);  // it should be 10000ms, so this is the reference value to compare with
            // 20240622 if the following line is printed, this will cause 7ms delay on the broadcast. Moved this message AFTER broadcast, see app_network_connect
#if PRINTF_APP_RTC_SYNC
            waitToPrint();
            npf_snprintf(uart_buf, 200, "%u [app_rtc_sync] GNSSPP10S #%u, @%uus. RTOS time difference with previous GNSSPP10S = %uus. Total drift since RTOS synchronization = %dus.\r\n",
              (unsigned int) xTaskGetTickCount(), (unsigned int) gnss_pps, (unsigned int) rtosTimeStampGNSSPPS, (unsigned int)rtosDriftToGNSS, (int) totalRtosDriftToGNSS);
            huart2print(uart_buf, strlen(uart_buf));
#endif
//          pcf2131SetAging(agingOffsetPCF2131);
//#if PRINTF_APP_RTC_SYNC
//          waitToPrint();
//          npf_snprintf(uart_buf, 200, "%u [app_rtc_sync] Aging Offset PCF2131 set to = %u.\r\n", (unsigned int) xTaskGetTickCount(), (unsigned int) agingOffsetPCF2131);
//          huart2print(uart_buf, strlen(uart_buf));
//#endif
//          agingOffsetPCF2131++;
//          if (agingOffsetPCF2131 == 0x0F)
//          {
//            agingOffsetPCF2131 = 0;
//          }
          }
   	    } // endif rtcSynchronized
      }
      else
      { // notificationValue != NOTIFICATION_FROM_GNSS_PPS
      	if (!notificationValue)
      	{
#if PRINTF_APP_RTC_SYNC
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_rtc_sync] No GNSSPPS in the last 1.2s.\r\n",(unsigned int) xTaskGetTickCount());
          huart2print(uart_buf, strlen(uart_buf));
#endif
      	}
      	else
      	{
#if PRINTF_APP_RTC_SYNC
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_rtc_sync] Expected GNSSPPS, but wrong notification value received: 0x%08X.\r\n",(unsigned int) xTaskGetTickCount(), (unsigned int) notificationValue);
          huart2print(uart_buf, strlen(uart_buf));
#endif
      	}
        if (whileIterations++ > 3)
        {
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_rtc_sync] Error: Jump out of while loop waiting for GNSSPPS.\r\n",(unsigned int) xTaskGetTickCount());
          huart2print(uart_buf, strlen(uart_buf));
          notificationValue = NOTIFICATION_FROM_GNSS_PPS;
          // todo reset GNSS module
        }
      	xTaskNotifyStateClear(rtcSyncThreadHandler);
        notificationValue = 0;
      }
    }
#else
    while ((notificationValue & NOTIFICATION_FROM_HOST_PP10S) != NOTIFICATION_FROM_HOST_PP10S)
    { // waiting for a notification value from the host - Received broadcast message - see app_network_connect.c
      xTaskNotifyWait( 0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(12000));
      if ((notificationValue & NOTIFICATION_FROM_HOST_PP10S) == NOTIFICATION_FROM_HOST_PP10S)
      {
        xSemaphoreTake(timeStampIntMutex, pdMS_TO_TICKS(400));
        rtosTimeStampGNSSPPS = timeStamp8;
        xSemaphoreGive(timeStampIntMutex);
        gnss_pps++;
        //20240509 for the node, the statistics are not (yet) calculated in app_hal_sync
    	differenceGNSSPPSRTCPPS = (rtosTimeStampGNSSPPS - rtosTimeStampRTCPPS) % 1000; // value between -999 and +999. Correct value or set point is -500 (RTCPPS should be 500ms later than GNSSPPS)
    																					   // since this endless loop is starting with a GNSSPPS, the difference GNSSPPS - RTCPPS should be +500.
    	statsDifferenceGNSSPPSRTCPPS = differenceGNSSPPSRTCPPS;
    	if (differenceGNSSPPSRTCPPS < 0)
    	{
    	  statsDifferenceGNSSPPSRTCPPS += 500;
    	}
    	else
    	{
    	  statsDifferenceGNSSPPSRTCPPS -= 500;
    	}
    	if (halSynchronized)
    	{
    	  switch (statsDifferenceGNSSPPSRTCPPS)
    	  {
    		case -4:
    		  statisticsRTCAccuracy[1]++;
    		  break;
    		case -3:
    		  statisticsRTCAccuracy[2]++;
    		  break;
    		case -2:
    		  statisticsRTCAccuracy[3]++;
    		  break;
    		case -1:
    		  statisticsRTCAccuracy[4]++;
    		  break;
    		case 0:
    		  statisticsRTCAccuracy[5]++;
    		  break;
    		case 1:
    		  statisticsRTCAccuracy[6]++;
    		  break;
    		case 2:
    		  statisticsRTCAccuracy[7]++;
    		  break;
    		case 3:
    		  statisticsRTCAccuracy[8]++;
    		  break;
    		case 4:
    		  statisticsRTCAccuracy[9]++;
    		  break;
    	  }
    	  if (statsDifferenceGNSSPPSRTCPPS > 4)
    	  {
    		statisticsRTCAccuracy[10]++;
    	  }
    	  if (statsDifferenceGNSSPPSRTCPPS < -4)
    	  {
    		statisticsRTCAccuracy[0]++;
    	  }
    	}
#if PRINTF_APP_RTC_SYNC
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_rtc_sync] HOSTPP10S #%u, @%ums (time reference for node). RTOS time difference HOSTPP10S - RTCPPS = %dms, Stats: %d %d %d %d %d %d %d %d %d %d %d.\r\n",
            (unsigned int) xTaskGetTickCount(), (unsigned int) gnss_pps, (unsigned int) rtosTimeStampGNSSPPS, (int) differenceGNSSPPSRTCPPS,
            (int) statisticsRTCAccuracy[0], (int) statisticsRTCAccuracy[1], (int) statisticsRTCAccuracy[2], (int) statisticsRTCAccuracy[3], (int) statisticsRTCAccuracy[4],
            (int) statisticsRTCAccuracy[5], (int) statisticsRTCAccuracy[6], (int) statisticsRTCAccuracy[7], (int) statisticsRTCAccuracy[8], (int) statisticsRTCAccuracy[9],
            (int) statisticsRTCAccuracy[10]);
        huart2print(uart_buf, strlen(uart_buf));
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_rtc_sync] HOSTPP10S #%u (time reference for node). RTOS time difference with previous HOSTPP10S = %ums.\r\n",
            (unsigned int) xTaskGetTickCount(), (unsigned int) gnss_pps, (unsigned int)(rtosTimeStampGNSSPPS - previousrtosTimeStampGNSSPPS));
        huart2print(uart_buf, strlen(uart_buf));
#endif
      }
      else
      { // notificationValue != NOTIFICATION_FROM_HOST_PP10S
  	    if (!notificationValue)
      	{
#if PRINTF_APP_RTC_SYNC
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_rtc_sync] No HOSTPP10S in the last 12s.\r\n",(unsigned int) xTaskGetTickCount());
          huart2print(uart_buf, strlen(uart_buf));
#endif
      	}
      	else
      	{
#if PRINTF_APP_RTC_SYNC
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_rtc_sync] Expected HOSTPP10S, but wrong notification value received: 0x%08X.\r\n",(unsigned int) xTaskGetTickCount(), (unsigned int) notificationValue);
          huart2print(uart_buf, strlen(uart_buf));
#endif
      	}
        if (whileIterations++ > 3)
        {
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_rtc_sync] Error: Jump out of while loop waiting for HOSTPP10S.\r\n",(unsigned int) xTaskGetTickCount());
          huart2print(uart_buf, strlen(uart_buf));
          notificationValue = NOTIFICATION_FROM_HOST_PP10S;
          // todo reset node module?
        }
      	xTaskNotifyStateClear(rtcSyncThreadHandler);
        notificationValue = 0;
      }
    }
#endif

#if SENSOR_HOST
    if (!(gnss_pps%10))
    {// do this every 10 GNSSPPS
      previousrtosTimeStampGNSSPPS = rtosTimeStampGNSSPPS; // this is in fact HOSTPP10S
    }
#else
    previousrtosTimeStampGNSSPPS = rtosTimeStampGNSSPPS; // on a node this is HOSTPP10S
#endif
  }
}

void app_rtc_sync_notify_fromISR(uint32_t notValue)
{
  BaseType_t xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  if (rtcSyncThreadHandler != NULL)
  {
    xTaskNotifyFromISR(rtcSyncThreadHandler, notValue, eSetBits, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void app_rtc_sync_notify(uint32_t notValue)
{
  if (rtcSyncThreadHandler != NULL)
  {
    xTaskNotify(rtcSyncThreadHandler, notValue, eSetBits);
  }
}
