/*
 * app_hal_sync.c
 *
 *  Created on: Apr 11, 2023
 *      Author: Sarah Goossens
 *
 *  Synchronization of the STM32 clock is done with the RTC_PPS signal because this is available for both the HOST and NODE
 *  A notification is received each time an RTC_PPS pulse is given.
 *
 *  This thread also evaluates when the RTC needs to be refreshed with the epoch time received from the gnss module (gnssEpoch - in case of
 *  host) or epoch time from the host (received_epoch - in case of node)
 *
 */

#include "main.h"
#include "app_hal_sync.h"
#include "app_rtc.h"
#include "usart.h"            // to declare huart2
#include <string.h>
#include "../Drivers/PCF2131/PCF2131.h"
#include "app_network_connect.h"
#include "app_led.h"
#include "app_imu.h"
#include "app_gnss.h"
#include "FreeRTOSConfig.h"
#include "iwdg.h"

#define PRINTF_APP_HAL_SYNC 1

extern IWDG_HandleTypeDef hiwdg;
extern char              uart_buf[200];
extern uint64_t          timeStampInt; // RTOS tick at the time of the external interrupt (in this case NINTA or RTC_PPS)
extern double            halDriftCorrection;
extern double            compensateTicks;
extern SemaphoreHandle_t CompTicksMutex;
#if TICKSYNCHRONIZATION
  extern double          adjustTick;
  extern uint32_t        subtractedIn1s;
  extern uint32_t        addedIn1s;
  extern uint32_t        substractedTicks;
  extern uint32_t        addedTicks;
#else
  extern double          adjustTIM2;
  extern uint32_t        substracteduseconds;
  extern uint32_t        addeduseconds;
  extern double        subtracteduSecondsIn1s;
  extern double        addeduSecondsIn1s;
  extern int32_t        tim2Table[1000];
  extern uint32_t        tim2Position;
  extern int32_t        regVal;
#endif
extern   uint32_t SysTickCallbackHits;
extern uint32_t          schedulerStartTime;
extern uint32_t          rtc_pps;
extern uint64_t          firstRTCPPS;
extern uint64_t          durationRTC10PPS;
extern uint64_t          rtosTimeStampGNSSPPS;
extern uint64_t          rtosTimeStampRTCPPS;
extern uint64_t 		 tickCountRTCPPS;
extern double            sysTickCurrentValue;
extern uint32_t          gnss_pps;
extern uint32_t          duration_100_gnss_pps;
extern uint8_t           halSynchronized;
extern uint8_t           rtcSynchronized;
//extern osThreadId        ledThreadHandler;
extern SemaphoreHandle_t timeStampIntMutex;
extern uint8_t           changeClock;
#if !STM32WBAUSED
extern uint8_t           setClock;
#endif
extern int               rtosTimeDifference;
extern uint8_t           subSecondsCorrection;
extern uint8_t           msCorrection;
extern uint64_t          rtosTimeStampChangeRTC;
extern uint64_t          rtosTimeStampGPRMC;
extern uint8_t           notifyOnlyOnceGNSS;
extern uint32_t          pairingOngoing;
extern uint64_t          previousrtosTimeStampGNSSPPS;
extern uint32_t          gnssEpochAvailable;
extern float             tickSpeedToReference;

extern uint32_t          subTickCountAtRTCPPS;
extern uint32_t          subTickCounterTIM2;
extern uint32_t          newReloadSysTickValue;
extern double            restReloadSysTickValue;





#if STM32WBAUSED
  TaskHandle_t halSyncThreadHandler;
#else
  osThreadId halSyncThreadHandler;
#endif

void HalSyncThreadInit()
{
#if STM32WBAUSED
  if (xTaskCreate ((TaskFunction_t)StartHalSyncThread, "HalSyncThread", (configSTACK_DEPTH_TYPE)512, NULL, osPriorityHigh7, &halSyncThreadHandler) != pdPASS)
  {
#if PRINTF_APP_HAL_SYNC
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [hal_sync] [HalSynchread] Could not be started.\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
#else
  osThreadDef(HalSyncThread, StartHalSyncThread, osPriorityRealtime, 0, 128);
  halSyncThreadHandler = osThreadCreate(osThread(halSyncThread), NULL);
#endif
}

void StartHalSyncThread(const void * params)
{
#if PRINTF_APP_HAL_SYNC
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [hal_sync] [HalSyncThread] Started.\n",(unsigned int) xTaskGetTickCount());
  huart2print(uart_buf, strlen(uart_buf));
#endif
 double    halDrift                    = 0;
  uint32_t notificationValue           = 0;  // Used to identify where the notification is coming from.
  uint64_t previousRtosTimeStampRTCPPS = 0;  // to calculate how many RTOS Ticks 1 RTCPPS takes
  uint8_t  checkHalSynchronized        = 0;
  uint8_t  checkHalAligned             = 0;
  uint8_t  checkRTCSynchronized        = 0;
  int      rtosTimeDifferenceReference = 0;
  int      rtosTimeDifferenceCheck     = 0;
//  uint64_t rtosTimeStampStartWait      = 0;
  int32_t  gprmcCorrection             = 0;
  uint64_t sysTicknsValue              = 0;
  uint64_t previousTickCountRTCPP      = 0;
  uint64_t previousSysTicknsValue      = 0;
  int      tickAdded                   = 0;

  // Definition of HAL synchronisation with RTCPPS PID controller parameters and variables:
float pidHalKp                 = 0.1;    // was 0.3 Proportional Gain: determines the strength of the proportional control.
                                         // It scales the error term to produce an output that is directly proportional to the current error.
                                         // Increasing Kp makes the controller more responsive but may lead to overshoot or oscillations,
                                         // while decreasing it may result in sluggish response.
float pidHalKi                 = 0.03;   // was 0.03 Integral Gain: determines the strength of the integral control.
                                         // It integrates the accumulated error over time, helping to eliminate steady-state errors.
                                         // It is particularly useful in cases where there is a constant bias or offset in the system.
float pidHalKd                 = 0.01;   // was 0.01 Derivative Gain: determines the strength of the derivative control.
                                         // It considers the rate of change of the error and helps dampen the response, reducing overshoot
                                         // and oscillations. It provides a predictive element that can anticipate the future behavior of the
                                         // system. Setting Kd too high can lead to a high-frequency response that amplifies noise.
 int32_t pidHalError           = 0;      // The difference between the desired setpoint and the current value of the process variable.
                                         // It serves as the primary input to the PID controller and drives the control action.
 int64_t pidHalIntegral        = 0;    // This variable keeps track of the accumulated error over time. It integrates the error term
                                         // by continuously summing the error values. It helps eliminate steady-state errors and ensures
                                         // the controller reaches the desired setpoint.
 int64_t pidHalDerivative      = 0;    // This variable calculates the rate of change of the error. It helps anticipate the future behavior
                                         // of the system by considering the rate at which the error is changing. It contributes to damping the
                                         // response and reducing overshoot.
 int64_t pidHalPreviousError   = 0;    // Previous error value. It is used to calculate the derivative term by finding the difference between
                                         // the current error and the previous error.
 double pidHalOutput           = 0.0;    // Output of the PID controller. It is calculated by summing the proportional, integral, and
                                         // derivative terms. The output is applied to the system being controlled to adjust its behavior.
 int32_t pidHalSetpoint        = 1000000000.0;    // Target value for the process variable. The PID controller aims to minimize the error between the
                                         // setpoint and the process variable by adjusting the output.
                                         // HAL/RTOS Ticks needs to be aligned to RTCPPS: if there is an RTCPPS, then HAL/RTOS ticks needs to be at 0
 int32_t pidHalProcessVariable = 0;      // Current value of the system being controlled. The PID controller measures this variable and
                                         // compares it to the setpoint to calculate the error and determine the appropriate control action.
  // Definition of RTC synchronization with GNSSPPS PID controller parameters and variables (ONLY HOST):
#define pidRtcKp 0.3                     // was 0.4 Proportional Gain: determines the strength of the proportional control. //was 0.4
#define pidRtcKi 0.05                     // was 0.3 Integral Gain: determines the strength of the integral control.//was 0.1
#define pidRtcKd 0.01                    // was 0.01 Derivative Gain: determines the strength of the derivative control. //was 0.02
 double pidRtcError            = 0.0;    // The difference between the desired setpoint and the current value of the process variable.
 double pidRtcIntegral         = 0.0;    // This variable keeps track of the accumulated error over time.
 double pidRtcDerivative       = 0.0;    // This variable calculates the rate of change of the error.
 double pidRtcPrevious_error   = 0.0;    // Previous error value.
 double pidRtcOutput           = 0.0;    // Output of the PID controller.
 double pidRtcSetpoint         = -500.0; //20240424 was -500. Target value for the process variable. 500ms difference between RTCPPS and GNSSPPS
                                         // if there is a GNSSPPS, then RTCPPS needs to be at 500
 double pidRtcProcess_variable = 0.0;    // Current value of the system being controlled.
  uint32_t pidRtcIterations                = 0;
  uint8_t  startpidRtc                     = 0;
  int32_t  halAlignment                    = 0;
  uint8_t  halAligned                      = 0;
  int32_t  alignRTOSTicks                  = 0;
  int32_t  statisticsRTOStoRTCAccuracy[11] = {0,0,0,0,0,0,0,0,0,0,0};
  int32_t  statisticsRTCtoGNSSAccuracy[11] = {0,0,0,0,0,0,0,0,0,0,0};

  uint32_t whileIterations               = 0; // to prevent lock into while loop

  uint32_t formerSubSecondsCorrection   = 0;
  uint32_t formerMsCorrection           = 0;
  uint32_t tickDiffDefRTCGNSS           = 500; // RTCPPS needs to be 500ms difference with GNSSPPS. When a tick rate different than 1ms is chosen, this needs to be adapted.
                                               // if the tick rate is 2000Hz, then the tickDiffDefRTCGNSS will become 1000.
  double   timeDifferenceRTCGNSSPPS     = 0;


  //RtcThreadInit();
  tickDiffDefRTCGNSS = tickDiffDefRTCGNSS * (uint32_t) tickSpeedToReference;


  double   CumAvSubTicksCounter         = 0;
  double   CumAvSubTicksTIM2            = 0;

#if PRINTF_APP_HAL_SYNC
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [hal_sync] SYSTICK_LOAD_REG value = #%u.\r\n", (unsigned int) xTaskGetTickCount(), (unsigned int) SYSTICK_LOAD_REG);
  huart2print(uart_buf, strlen(uart_buf));
#endif


  for(;;)
  { // Infinite loop
    xTaskNotifyStateClear(halSyncThreadHandler);
    notificationValue = 0;
    whileIterations   = 0;
    while ((notificationValue & NOTIFICATION_FROM_RTC_PPS) != NOTIFICATION_FROM_RTC_PPS)
    { // waiting for a notification value from RTC_PPS - Interrupt from RTC_NINTA - see main HAL_GPIO_EXTI_Callback()
      xTaskNotifyWait( 0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(3000));
      if ((notificationValue & NOTIFICATION_FROM_RTC_PPS) == NOTIFICATION_FROM_RTC_PPS)
      {
//        xSemaphoreTake(timeStampIntMutex, pdMS_TO_TICKS(400));
//        rtosTimeStampRTCPPS = timeStampInt;
//        xSemaphoreGive(timeStampIntMutex);
    	  sysTicknsValue = (uint64_t)(sysTickCurrentValue * ((double)1000000000 / (double)subTickCounterTIM2));
    	  rtosTimeStampRTCPPS = tickCountRTCPPS * 1000000 + sysTicknsValue;

//    	  if (sysTicknsValue > 900000)
//    	  {
//            rtosTimeStampRTCPPS = --tickCountRTCPPS * 1000000 + sysTicknsValue;
//            tickAdded = -1;
//    	  }
//    	  else
//    	  {
////    	    if (sysTicknsValue < 100000)
////    	    {
////              rtosTimeStampRTCPPS = tickCountRTCPPS-- * 1000000 + sysTicknsValue;
////              tickAdded = -1;
////    	    }
////    	    else
////    	    {
//              rtosTimeStampRTCPPS = tickCountRTCPPS * 1000000 + sysTicknsValue;
//              tickAdded = 0;
////    	    }
//    	  }

        if (!rtc_pps)
        {
          firstRTCPPS = rtosTimeStampRTCPPS;
        }
        else
        {
          if (!(rtc_pps % 10))
          {
//            TIM2->CNT = 0;
            durationRTC10PPS = rtosTimeStampRTCPPS - firstRTCPPS;
            firstRTCPPS = rtosTimeStampRTCPPS;
#if SENSOR_HOST
#if PRINTF_APP_HAL_SYNC
            waitToPrint();
            npf_snprintf(uart_buf, 200, "%u [hal_sync] RTCPP10S #%u (to check RTOS drift to RTC). RTOS time difference with previous RTCPP10S = %uus.\r\n",
              (unsigned int) xTaskGetTickCount(), (unsigned int) rtc_pps, (unsigned int) (durationRTC10PPS/1000));
            huart2print(uart_buf, strlen(uart_buf));
#endif
#endif
          }
        }
        rtc_pps++;
      }
      else
      {
    	if (!notificationValue)
    	{
#if PRINTF_APP_HAL_SYNC
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [hal_sync] No RTCPPS in the last 3s.\r\n",(unsigned int) xTaskGetTickCount());
          huart2print(uart_buf, strlen(uart_buf));
#endif
    	}
    	else
    	{
#if PRINTF_APP_HAL_SYNC
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [hal_sync] Expected RTCPPS, but wrong notification value received: 0x%08X.\r\n",(unsigned int) xTaskGetTickCount(), (unsigned int) notificationValue);
          huart2print(uart_buf, strlen(uart_buf));
#endif
    	}
    	xTaskNotifyStateClear(halSyncThreadHandler);
        notificationValue = 0;
      }
      if(whileIterations++ > 3)
      {
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [hal_sync] Error: Jump out of while loop waiting for RTCPPS.\r\n",(unsigned int) xTaskGetTickCount());
        huart2print(uart_buf, strlen(uart_buf));
        notificationValue = NOTIFICATION_FROM_RTC_PPS;
        // todo: reset RTC as there is no RTCPPS
      }
    }
//#if SENSOR_HOST
//#if PRINTF_APP_HAL_SYNC
//    if (rtosTimeStampRTCPPS > rtosTimeStampGNSSPPS)
//    {
////      if (((rtosTimeStampRTCPPS - rtosTimeStampGNSSPPS) % tickDiffDefRTCGNSS) || (rtosTimeStampRTCPPS % tickDiffDefRTCGNSS))
////      {// print only if RTCPPS is not @ 500ms or RTCPPS - GNSSPPS is not 500ms
//    	timeDifferenceRTCGNSSPPS = ((float)(rtosTimeStampRTCPPS - rtosTimeStampGNSSPPS)) / tickSpeedToReference;
//        waitToPrint();
//        npf_snprintf(uart_buf, 200, "%u [hal_sync] RTCPPS #%u, @%uus, %fus later than GNSSPPS #%u.\r\n",
//            (unsigned int) xTaskGetTickCount(), (unsigned int) rtc_pps, (unsigned int) rtosTimeStampRTCPPS, timeDifferenceRTCGNSSPPS, (unsigned int) gnss_pps);
//        huart2print(uart_buf, strlen(uart_buf));
////      }
//    }
//    else
//    {
////      if (((rtosTimeStampGNSSPPS - rtosTimeStampRTCPPS) % tickDiffDefRTCGNSS) || (rtosTimeStampRTCPPS % tickDiffDefRTCGNSS))
////      {// print only if RTCPPS is not @ 500ms or RTCPPS - GNSSPPS is not 500ms
//    	timeDifferenceRTCGNSSPPS = ((float)(rtosTimeStampGNSSPPS - rtosTimeStampRTCPPS)) / tickSpeedToReference;
//        waitToPrint();
//        npf_snprintf(uart_buf, 200, "%u [hal_sync] RTCPPS #%u, @%uus, %fus earlier than GNSSPPS #%u.\r\n",
//            (unsigned int) xTaskGetTickCount(), (unsigned int) rtc_pps, (unsigned int) rtosTimeStampRTCPPS, timeDifferenceRTCGNSSPPS, (unsigned int) gnss_pps);
//        huart2print(uart_buf, strlen(uart_buf));
////      }
//    }
//#endif
//#endif





    LedThreadNotify(NOTIFICATION_FROM_APP_HAL_SYNC_TO_START_LED);
#if !SENSOR_HOST
    if (rtc_pps == 2)
    { // align RTOS and HAL with rtc_pps
      alignRTOSTicks = xTaskGetTickCount();
      alignRTOSTicks = alignRTOSTicks % 1000;
//      xSemaphoreTake(CompTicksMutex, pdMS_TO_TICKS(400));
      if (alignRTOSTicks > 500)
      {
        alignRTOSTicks = 1000 - alignRTOSTicks;
        compensateTicks = (float) -alignRTOSTicks;
      }
      else
      {
        compensateTicks = (float) alignRTOSTicks;
      }
//      xSemaphoreGive(CompTicksMutex);
//#if PRINTF_APP_HAL_SYNC
//        waitToPrint();
//        npf_snprintf(uart_buf, 200, "%u [hal_sync] Compensate %d ticks.\r\n",(unsigned int) xTaskGetTickCount(), (int) compensateTicks);
//        huart2print(uart_buf, strlen(uart_buf));
//#endif
//      while (compensateTicks)
//      {
        vTaskDelay(500);
//     vTaskDelay((TickType_t)(alignRTOSTicks+10)); //20240407 to give HAL_TIM_PeriodElapsedCallback() enough time to adapt the ticks
//      }
//     compensateTicks = 0;
    }
#endif

    // start PID controller for synchronization between RTOS/HAL time base and RTC time base:

#if SENSOR_HOST
    if (rtc_pps > 2)


#else
//    if (!halSynchronized && (rtc_pps > 14)) // the HAL PID controller should start later, otherwise a big error compensation will be given when the RTC first starts up with HOST time
    if (rtc_pps > 5) // the HAL PID controller should start later, otherwise a big error compensation will be given when the RTC first starts up with HOST time
#endif
    { // wait a few cycles at start-up so that the PID controller can work properly. Once HAL is synchronized, this pid controller must always work,
      // but when rtc_pps = 1 then we need to subtract 1000ms since we are missing one RTCPPS during the setting of the clock
//      if (rtc_pps == 1)
//      {
//	  rtosTimeStampRTCPPS -= 1000;
//      }

      //timeDifferenceRTCGNSSPPS = ((double)(rtosTimeStampRTCPPS - rtosTimeStampGNSSPPS)) / tickSpeedToReference;

      // update process variable to be controlled
//      pidHalProcessVariable = (int32_t)(rtosTimeStampRTCPPS - previousRtosTimeStampRTCPPS);// + (int32_t)compensateTicks * 1000;
//      pidHalProcessVariable = (int32_t)(sysTicknsValue - previousSysTicknsValue);// + (int32_t)compensateTicks * 1000;
      //pidHalProcessVariable += 1000000000;

//      if (pidHalProcessVariable > 500000)
//      {
//	    pidHalProcessVariable -= 1000000;
//      }
//      else
//      {
//        if (pidHalProcessVariable < -500000)
//        {
//          pidHalProcessVariable += 1000000;
//        }
//      }

      CumAvSubTicksTIM2 = CumAvSubTicksTIM2 + ((double)subTickCounterTIM2 - CumAvSubTicksTIM2) / (CumAvSubTicksCounter + (double) 1);
      CumAvSubTicksCounter += (double) 1;

      pidHalSetpoint = (uint32_t) CumAvSubTicksTIM2;
      pidHalProcessVariable = (float)subTickCounterTIM2;

      waitToPrint();
//      npf_snprintf(uart_buf, 200, "%u [hal_sync] RTCPPS #%u, tickCount = %u, sysTickCurrentValue = %uns, delta previous RTCPPS = %ins, #subTicks = %u*62.5= %uns, #subTicks TIM2 = %u.\r\n",
//	    (unsigned int) xTaskGetTickCount(), (unsigned int) rtc_pps, (unsigned int) tickCountRTCPPS, (unsigned int) sysTicknsValue, (int)(rtosTimeStampRTCPPS - previousRtosTimeStampRTCPPS),
//		(unsigned int) subTickCountAtRTCPPS, (unsigned int)((double)subTickCountAtRTCPPS * (double)62.5), (unsigned int) subTickCounterTIM2);
//      huart2print(uart_buf, strlen(uart_buf));

      npf_snprintf(uart_buf, 200, "%u [hal_sync] RTCPPS #%u, tickCount = %u, sysTickCurrentValue = %uns, RTCPPS = %ins, #subTicks TIM2 = %u. Cumulative Average = %8.7f.\r\n",
	    (unsigned int) xTaskGetTickCount(), (unsigned int) rtc_pps, (unsigned int) tickCountRTCPPS, (unsigned int) sysTicknsValue, (int)(rtosTimeStampRTCPPS - previousRtosTimeStampRTCPPS),
		(unsigned int) subTickCounterTIM2, CumAvSubTicksTIM2);
      huart2print(uart_buf, strlen(uart_buf));


      // todo check if not better always against RTCPPS?
//      pidHalProcessVariable = (float)(rtosTimeStampRTCPPS % 1000);
      //20240726  pidHalProcessVariable = (float)(rtosTimeStampRTCPPS % (1000 * ((int)tickSpeedToReference)));
//      if (pidHalProcessVariable > 499)
//      //20240726  if (pidHalProcessVariable > ((500 * tickSpeedToReference)-1))
//      {
//    	pidHalProcessVariable -= 1000;
//    	//20240726  pidHalProcessVariable -= (1000 * tickSpeedToReference);
//      }
      pidHalError            =  pidHalProcessVariable - pidHalSetpoint;                                           // Calculate error
//      pidHalError            =  pidHalProcessVariable;                                           // Calculate error

//      if (!halSynchronized && checkHalSynchronized && (pidHalError < 1000) && (pidHalPreviousError < 1000))
//      {// keep the haldrift correction and start a new PID conotroller
//        pidHalIntegral = 0;
//        pidHalOutput = 0;
//        pidHalPreviousError = 0;
//        pidHalKp = pidHalKp / 10.0;
//        pidHalKi = pidHalKi / 10.0;
//        pidHalKd = pidHalKd / 10.0;
//        npf_snprintf(uart_buf, 200, "%u [hal_sync] PID RTOS-RTCPPS adapted.\r\n",
//            (unsigned int) xTaskGetTickCount());
//        huart2print(uart_buf, strlen(uart_buf));
//
//      }
//      pidHalError            = pidHalProcessVariable;                                           // Calculate error (pidHalSetpoint = 0!!!)
      pidHalIntegral        += (int64_t)pidHalError;                                                                      // Calculate integral
      pidHalDerivative       = (int64_t)pidHalError - pidHalPreviousError;                                                // Calculate derivative
      pidHalOutput           = pidHalKp * (double)pidHalError + pidHalKi * (double)pidHalIntegral + pidHalKd * (double)pidHalDerivative; // Calculate output
      pidHalPreviousError    = (int64_t)pidHalError;                                                                      // Update previous error
      halDriftCorrection     = -pidHalOutput / (double)1000.0;// / -62500.0; // * tickSpeedToReference??;


 //     halDriftCorrection     = (double)pidHalSetpoint / (double)1000.0;

      newReloadSysTickValue  = (uint32_t) ((double)pidHalSetpoint / (double)1000.0);
      restReloadSysTickValue = ((double) (pidHalSetpoint - newReloadSysTickValue * 1000)) / (double) 1000.0;
      newReloadSysTickValue--; // one less as the sysTick counter counts down till 0


      // Refreh the Watchdog
//      HAL_IWDG_Refresh(&hiwdg);


//      halDriftCorrection     = 0;
////      xSemaphoreTake(CompTicksMutex, pdMS_TO_TICKS(400));
//      if (tickCountRTCPPS - previousTickCountRTCPP - 1000)
//      {
//        compensateTicks = -16.0;
//      }
//      compensateTicks        = pidHalOutput / 1000.0;
//      compensateTicks        = ((double)(-pidHalError)) / (double)(1000.0);
//      tim2Position = 0;
//      regVal = 1000U;
//      WRITE_REG(TIM2->ARR, regVal);

////      xSemaphoreGive(CompTicksMutex);
//      if (halSynchronized)
//      {
//        switch ((int) pidHalError)
//        {
//          case -4:
//            statisticsRTOStoRTCAccuracy[1]++;
//            break;
//          case -3:
//            statisticsRTOStoRTCAccuracy[2]++;
//            break;
//          case -2:
//            statisticsRTOStoRTCAccuracy[3]++;
//            break;
//          case -1:
//            statisticsRTOStoRTCAccuracy[4]++;
//            break;
//          case 0:
//            statisticsRTOStoRTCAccuracy[5]++;
//            break;
//          case 1:
//            statisticsRTOStoRTCAccuracy[6]++;
//            break;
//          case 2:
//            statisticsRTOStoRTCAccuracy[7]++;
//            break;
//          case 3:
//            statisticsRTOStoRTCAccuracy[8]++;
//            break;
//          case 4:
//            statisticsRTOStoRTCAccuracy[9]++;
//            break;
//        }
//        if (pidHalError > 4)
//        {
//          statisticsRTOStoRTCAccuracy[10]++;
//        }
//        if (pidHalError < -4)
//        {
//          statisticsRTOStoRTCAccuracy[0]++;
//        }
//      }

      if (pidHalError < 0)
      {
        if (pidHalError < -500)
        {
          statisticsRTOStoRTCAccuracy[0]++;
        }
        else
        {
          if (pidHalError < -100)
          {
            statisticsRTOStoRTCAccuracy[1]++;
          }
          else
          {
            if (pidHalError < -50)
            {
              statisticsRTOStoRTCAccuracy[2]++;
            }
            else
            {
              if (pidHalError < -10)
              {
                statisticsRTOStoRTCAccuracy[3]++;
              }
              else
              {
                if (pidHalError < -5)
                {
                  statisticsRTOStoRTCAccuracy[4]++;
                }
                else
                {
                  statisticsRTOStoRTCAccuracy[5]++;
                }
              }
            }
          }
        }
      }
      else
      {
        if (pidHalError > 500)
        {
          statisticsRTOStoRTCAccuracy[10]++;
        }
        else
        {
          if (pidHalError > 100)
          {
            statisticsRTOStoRTCAccuracy[9]++;
          }
          else
          {
            if (pidHalError > 50)
            {
              statisticsRTOStoRTCAccuracy[8]++;
            }
            else
            {
              if (pidHalError > 10)
              {
                statisticsRTOStoRTCAccuracy[7]++;
              }
              else
              {
                if (pidHalError > 5)
                {
                  statisticsRTOStoRTCAccuracy[6]++;
                }
                else
                {
                  statisticsRTOStoRTCAccuracy[5]++;
                }
              }
            }
          }
        }
      }
//#if !ANTENNARSSITEST
//#if PRINTF_APP_HAL_SYNC
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "%u [hal_sync] PID RTOS-RTCPPS done, RTOS error to RTCPPS = %03.3fus, corr = %03.5fus/ms, subtractedIn1s = %uus, addedIn1s = %uus, #subtr = %u, #added = %u, Stats: %d %d %d %d %d %d %d %d %d %d %d.\r\n",
//          (unsigned int) xTaskGetTickCount(), pidHalError / tickSpeedToReference, halDriftCorrection, (unsigned int) subtractedIn1s, (unsigned int) addedIn1s,  (unsigned int) substractedTicks, (unsigned int) addedTicks,
//	      (int) statisticsRTOStoRTCAccuracy[0], (int) statisticsRTOStoRTCAccuracy[1], (int) statisticsRTOStoRTCAccuracy[2], (int) statisticsRTOStoRTCAccuracy[3], (int) statisticsRTOStoRTCAccuracy[4],
//	      (int) statisticsRTOStoRTCAccuracy[5], (int) statisticsRTOStoRTCAccuracy[6], (int) statisticsRTOStoRTCAccuracy[7], (int) statisticsRTOStoRTCAccuracy[8], (int) statisticsRTOStoRTCAccuracy[9],
//	      (int) statisticsRTOStoRTCAccuracy[10]);
//      huart2print(uart_buf, strlen(uart_buf));
//#endif
//#endif
#if !ANTENNARSSITEST
#if PRINTF_APP_HAL_SYNC
      waitToPrint();
#if TICKSYNCHRONIZATION
      npf_snprintf(uart_buf, 200, "%u [hal_sync] PID RTOS-RTCPPS done, Process Variable: %uus, RTOS error to RTCPPS = %ius, corr = %03.5fus/ms, subtractedIn1s = %uus, addedIn1s = %uus, #subtr = %u, #added = %u.\r\n",
          (unsigned int) xTaskGetTickCount(), (unsigned int)pidHalProcessVariable, (int)pidHalError, halDriftCorrection, (unsigned int) subtractedIn1s, (unsigned int) addedIn1s,  (unsigned int) substractedTicks,
		  (unsigned int) addedTicks);
      huart2print(uart_buf, strlen(uart_buf));
#else
//      npf_snprintf(uart_buf, 200, "%u [hal_sync] PID RTOS-RTCPPS done, Process Variable: %uus, RTOS error to RTCPPS = %ius, corr = %03.5fus/ms, subtractedIn1s = %03.5fus, addedIn1s = %03.5fus.\r\n",
//          (unsigned int) xTaskGetTickCount(), (unsigned int)pidHalProcessVariable, (int)pidHalError, halDriftCorrection, subtracteduSecondsIn1s, addeduSecondsIn1s);
//      huart2print(uart_buf, strlen(uart_buf));
      npf_snprintf(uart_buf, 200, "%u [hal_sync] PID RTOS-RTCPPS done, error to CA = %isubTicks, correction = %03.5fsubTicks/ms, compensation= %03.5fus/ms. ReloadVal = %usubTicks, RestVal = %03.5fsubTicks/ms.\r\n",
          (unsigned int) xTaskGetTickCount(), (int)pidHalError, halDriftCorrection, compensateTicks, (unsigned int) newReloadSysTickValue, restReloadSysTickValue);
      huart2print(uart_buf, strlen(uart_buf));

#endif
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [hal_sync] Stats: %d %d %d %d %d %d %d %d %d %d %d.\r\n",
          (unsigned int) xTaskGetTickCount(),
	      (int) statisticsRTOStoRTCAccuracy[0], (int) statisticsRTOStoRTCAccuracy[1], (int) statisticsRTOStoRTCAccuracy[2], (int) statisticsRTOStoRTCAccuracy[3], (int) statisticsRTOStoRTCAccuracy[4],
	      (int) statisticsRTOStoRTCAccuracy[5], (int) statisticsRTOStoRTCAccuracy[6], (int) statisticsRTOStoRTCAccuracy[7], (int) statisticsRTOStoRTCAccuracy[8], (int) statisticsRTOStoRTCAccuracy[9],
	      (int) statisticsRTOStoRTCAccuracy[10]);
      huart2print(uart_buf, strlen(uart_buf));
#endif
#endif

#if TICKSYNCHRONIZATION
      subtractedIn1s = 0;
      addedIn1s      = 0;
#else
      subtracteduSecondsIn1s = 0;
      addeduSecondsIn1s      = 0;

#endif
//#if !TICKSYNCHRONIZATION
//#if PRINTF_APP_HAL_SYNC
//      for (int i = 0;i < 1000; i++)
//      {
//    	if (tim2Table[i] != 15999)
////        if (tim2Table[i])
//    	{
//          waitToPrint();
//          npf_snprintf(uart_buf, 200, "%u [hal_sync] tim2Table[%u] = %d.\r\n",
//              (unsigned int) xTaskGetTickCount(), (unsigned int) i, (int) tim2Table[i]);
//          huart2print(uart_buf, strlen(uart_buf));
//    	}
//      }
//#endif
//#endif

#if SENSOR_HOST
      if (!halSynchronized && ((pidHalError == 1) || (pidHalError == -1) || !pidHalError ))
#else
      if (!halSynchronized && (!pidHalError))
#endif
      { // if 5 following iterations are giving a value of max 1 RTOS Tick difference per RTCPPS, then we consider that HAL is synchronized with the RTC
	    checkHalSynchronized++;
#if PRINTF_APP_HAL_SYNC
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [hal_sync] # of following RTOS seconds with max 1 tick drift to RTCPPS: %u.\r\n",(unsigned int) xTaskGetTickCount(), (unsigned int) checkHalSynchronized);
        huart2print(uart_buf, strlen(uart_buf));
#endif
#if SENSOR_HOST
        if (checkHalSynchronized > 4)
#else
        if (checkHalSynchronized > 3)
#endif
        {
#if PRINTF_APP_HAL_SYNC
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [hal_sync] HAL/RTOS synchronized.\r\n",(unsigned int) xTaskGetTickCount());
          huart2print(uart_buf, strlen(uart_buf));
#endif
          halSynchronized = 1;

#if SENSOR_HOST
//20241002          GNSSThreadInit();
#else
//        GNSSPPSThreadInit();
#endif




#if PLANTSENSOR
          ImuThreadInit();
          pcf2131_si_off();
#if PRINTF_APP_HAL_SYNC
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [hal_sync] Suspended.\r\n",(unsigned int) xTaskGetTickCount());
          huart2print(uart_buf, strlen(uart_buf));
#endif
          vTaskSuspend(halSyncThreadHandler);
#else

#if !SENSOR_HOST
          NetConThreadInit(); // Now app_network_connect for the Node can be started:
#endif
#endif
        }
      }
      else
      {
        checkHalSynchronized = 0;
      }
    }

//    if (!halAligned && halSynchronized)
//    {
//      checkHalAligned++;
//      halAlignment = rtosTimeStampGNSSPPS / 1000;
//      halAlignment = rtosTimeStampGNSSPPS - (halAlignment * 1000);
//      if (halAlignment < 500)
//      {
//	halDriftCorrection  -= (halAlignment / 1000);
//	rtosTimeStampRTCPPS -= halAlignment;
//#if PRINTF_APP_HAL_SYNC
//        waitToPrint();
//        npf_snprintf(uart_buf, 200, "%u [hal_sync] RTOS/HAL alignment with start of GNSS_PPS done (move %u ticks backward).\r\n",(unsigned int) xTaskGetTickCount(), (unsigned int) halAlignment);
//        huart2print(uart_buf, strlen(uart_buf));
//#endif
//      }
//      else
//      {
//	halAlignment = 1000 - halAlignment;
//	halDriftCorrection  += (halAlignment / 1000);
//	rtosTimeStampRTCPPS += halAlignment;
//#if PRINTF_APP_HAL_SYNC
//        waitToPrint();
//        npf_snprintf(uart_buf, 200, "%u [hal_sync] RTOS/HAL alignment with start of GNSS_PPS done (move %u ticks forward).\r\n",(unsigned int) xTaskGetTickCount(), (unsigned int) halAlignment);
//        huart2print(uart_buf, strlen(uart_buf));
//#endif
//      }
//      if (checkHalAligned > 5)
//      {
//        halAligned = 1;
//      }
//    }

//     start PID controller for synchronization between RTCPPS and GNSSPPS (or HOSTPP10S):
//     for a HOST, the GNSS pulses are coming every second
//     for a NODE, the HOSTPP10S is coming every 10s, meaning that this evaluation can only be done every 10 seconds
#if SENSOR_HOST
    rtosTimeDifference = ((int) (rtosTimeStampGNSSPPS - rtosTimeStampRTCPPS)) % (1000 * ((int)tickSpeedToReference));
#else
    if (!(rtc_pps % 10))
    {
       rtosTimeDifference = ((int) (rtosTimeStampGNSSPPS - rtosTimeStampRTCPPS)) % (1000 * ((int)tickSpeedToReference));
#if PRINTF_APP_HAL_SYNC
       waitToPrint();
       npf_snprintf(uart_buf, 200, "%u [hal_sync]  RTOS time difference HOSTPP10S - RTCPPS before adaptation = %dms.\r\n",
          (unsigned int) xTaskGetTickCount(), rtosTimeDifference);
       huart2print(uart_buf, strlen(uart_buf));
#endif
    }
#endif
//    while (rtosTimeDifference < -1000)
//    {
//      rtosTimeDifference += 1000;
//    }
//    while (rtosTimeDifference > 1000)
//    {
//      rtosTimeDifference -= 1000;
//    }
    if (rtosTimeDifference < 0)
    {
      rtosTimeDifferenceCheck = rtosTimeDifference + (500 * ((int)tickSpeedToReference));
    }
    else
    {
      rtosTimeDifferenceCheck = rtosTimeDifference - (500 * ((int)tickSpeedToReference));
    }
    if (rtcSynchronized)
    {
      switch (rtosTimeDifferenceCheck)
      {
        case -4:
          statisticsRTCtoGNSSAccuracy[1]++;
          break;
        case -3:
          statisticsRTCtoGNSSAccuracy[2]++;
          break;
        case -2:
          statisticsRTCtoGNSSAccuracy[3]++;
          break;
        case -1:
    	  statisticsRTCtoGNSSAccuracy[4]++;
          break;
        case 0:
  	      statisticsRTCtoGNSSAccuracy[5]++;
          break;
        case 1:
          statisticsRTCtoGNSSAccuracy[6]++;
          break;
        case 2:
   	      statisticsRTCtoGNSSAccuracy[7]++;
          break;
        case 3:
          statisticsRTCtoGNSSAccuracy[8]++;
          break;
        case 4:
  	    statisticsRTCtoGNSSAccuracy[9]++;
      }
      if (rtosTimeDifferenceCheck > 4)
      {
        statisticsRTCtoGNSSAccuracy[10]++;
      }
      if (rtosTimeDifferenceCheck < -4)
      {
        statisticsRTCtoGNSSAccuracy[0]++;
      }
      if (rtosTimeDifferenceCheck)
      {
#if SENSOR_HOST
#if PRINTF_APP_HAL_SYNC
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [hal_sync] RTOS time difference GNSSPPS #%u - RTCPPS #%u = %fms. Check value = %dticks. Stats: %d %d %d %d %d %d %d %d %d %d %d.\r\n",
          (unsigned int) xTaskGetTickCount(), (unsigned int) gnss_pps, (unsigned int) rtc_pps, ((double)rtosTimeDifference) / tickSpeedToReference, rtosTimeDifferenceCheck,
	      (int) statisticsRTCtoGNSSAccuracy[0], (int) statisticsRTCtoGNSSAccuracy[1], (int) statisticsRTCtoGNSSAccuracy[2], (int) statisticsRTCtoGNSSAccuracy[3], (int) statisticsRTCtoGNSSAccuracy[4],
	      (int) statisticsRTCtoGNSSAccuracy[5], (int) statisticsRTCtoGNSSAccuracy[6], (int) statisticsRTCtoGNSSAccuracy[7], (int) statisticsRTCtoGNSSAccuracy[8], (int) statisticsRTCtoGNSSAccuracy[9],
	      (int) statisticsRTCtoGNSSAccuracy[10]);
      huart2print(uart_buf, strlen(uart_buf));
#endif
#else
    if (!(rtc_pps % 10))
    {
#if PRINTF_APP_HAL_SYNC
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [hal_sync] RTOS time difference HOSTPP10S - RTCPPS after adaptation = %fms. RTOS time difference check = %dticks.\r\n",
          (unsigned int) xTaskGetTickCount(), ((float)rtosTimeDifference) / tickSpeedToReference, rtosTimeDifferenceCheck);
      huart2print(uart_buf, strlen(uart_buf));
#endif
    }
#endif
      }
      if (rtosTimeDifferenceCheck < 0)
      {
        rtosTimeDifferenceCheck = -rtosTimeDifferenceCheck;
      }
    }
    // the intention of this pid controller is to trim the start of the RTC so that the RTC PPS is 500ms shifted with GNSS PPS in case of host
    // if we try to have them as close as possible to each other, the program is crashing
    // this pid controller should only work only when HAL/RTOS is synchronized
    // the output of this pid controller is used to calculate the subseconds (per 10ms) setting of the RTC and an extra delay (per ms) before to start the RTC
    // In case of node, we use the same PID controller, but this time we can not use GNSSPPS. In stead, we use the broadcast message coming from the host of the
    // node. As this signal is coming every 10S, we call this HOSTPP10S (HOST Pulse Per 10s)
#if SENSOR_HOST

    //if (halSynchronized) // 20240623 changed to:
    if (gnssEpochAvailable)
    {
      if ((!rtosTimeDifferenceCheck || (rtosTimeDifferenceCheck == 1)) && !rtcSynchronized)
      {// if 5 following iterations are giving a value of max 1tick RTCPPS difference per GNSSPPS, then we consider that the RTCPPS is synchronized with the GNSSPSS
        if (++checkRTCSynchronized == 5)
        {
#if PRINTF_APP_HAL_SYNC
          waitToPrint();
 	      npf_snprintf(uart_buf, 200, "%u [hal_sync] # of following iterations max 1tick difference between GNSSPPS and RTCPPS: %u.\r\n",
              (unsigned int) xTaskGetTickCount(), (unsigned int) checkRTCSynchronized);
  	      huart2print(uart_buf, strlen(uart_buf));
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [hal_sync] RTC is synchronized, start network connect at start of first broadcast process.\r\n", (unsigned int) xTaskGetTickCount());
          huart2print(uart_buf, strlen(uart_buf));
#endif
          rtcSynchronized = 1;
        }
        else
        {
#if PRINTF_APP_HAL_SYNC
  	  waitToPrint();
 	  npf_snprintf(uart_buf, 200, "%u [hal_sync] # of following iterations max 1ms difference between GNSSPPS and RTCPPS: %u.\r\n",
              (unsigned int) xTaskGetTickCount(), (unsigned int) checkRTCSynchronized);
  	  huart2print(uart_buf, strlen(uart_buf));
#endif
        }
      }
      else
      {
        checkRTCSynchronized = 0;
      }
      //if (!pairingOngoing && (gnss_pps%10 != 9)) // 20240623 changed to (gnss_pps%10 ==9) now evaluated in app_rtc_sync:
      if (!pairingOngoing)
      {// if a pairing process has started, the RTC should not be changed. To prevent this, the RTC PID controller will not be started.
       // if we are close to a broadcast message (which happens at GNSSPP10), also no RTCPID cycle should be planned, as this could cause that the RTC starts only AFTER the broadcast event
        if (((rtosTimeDifferenceCheck > 0) || !rtcSynchronized) && startpidRtc)
        { // set the RTC if error is more or equal than 1ms or not yet 5 times synchronized
          pidRtcOutput = 0;
          if (startpidRtc == 1)
          {
            startpidRtc = 2;
            if (rtosTimeDifference < 0)
            {
              rtosTimeDifferenceReference = (500 * ((int)tickSpeedToReference)) - rtosTimeDifference;
            }
            else
            {
              rtosTimeDifferenceReference = (500 * ((int)tickSpeedToReference)) - rtosTimeDifference;
            }
          }
          else
          {
            pidRtcIterations++;
            pidRtcProcess_variable   = (double) rtosTimeDifference;                                                       // update process variable to be controlled
            pidRtcError              = pidRtcSetpoint - pidRtcProcess_variable;                                          // Calculate error
            pidRtcIntegral          += pidRtcError;                                                                      // Calculate integral
            pidRtcDerivative         = pidRtcError - pidRtcPrevious_error;                                               // Calculate derivative
            pidRtcOutput             = pidRtcKp * pidRtcError + pidRtcKi * pidRtcIntegral + pidRtcKd * pidRtcDerivative; // Calculate output
            pidRtcPrevious_error     = pidRtcError;                                                                      // Update previous error
            // There is no control when the software exactly started with respect to the GNSSPPS. This however is important to set the RTC.
            // When a GNSSPPS is given, a $GPRMC message is created and we wait to read this out from the GNSS before to set the clock.
            // If it takes too long, then we miss an RTCPPS and the setting of RTC takes more than 2s
            // To overcome this, we estimate when the next $GPRMC message will come (variable rtosTimeStampGPRMC + 1000) and wait to give the Change Clock request command
            // so that we know that the next $GPRMC message will come within 80-100ms.
            rtosTimeStampChangeRTC = xTaskGetTickCount();
          }
          pidRtcOutput += (double) (rtosTimeDifferenceReference);
          whileIterations = 0;
          while (pidRtcOutput < (-1000 * ((int)tickSpeedToReference)))
          {
        	pidRtcOutput += 1000 * ((int)tickSpeedToReference);
            if(whileIterations++ > 10)
            {
              waitToPrint();
              npf_snprintf(uart_buf, 200, "%u [hal_sync] Error: Jump out of while loop to trim pidRtcOutput.\r\n",(unsigned int) xTaskGetTickCount());
              huart2print(uart_buf, strlen(uart_buf));
              pidRtcOutput = 0;
            }
          }
          whileIterations = 0;
          while (pidRtcOutput > (1000 * ((int)tickSpeedToReference)))
          {
        	pidRtcOutput -= 1000 * ((int)tickSpeedToReference);
            if(whileIterations++ > 10)
            {
              waitToPrint();
              npf_snprintf(uart_buf, 200, "%u [hal_sync] Error: Jump out of while loop to trim pidRtcOutput.\r\n",(unsigned int) xTaskGetTickCount());
              huart2print(uart_buf, strlen(uart_buf));
              pidRtcOutput = 0;
            }

          }
          if (pidRtcOutput < 0)
          {
            subSecondsCorrection     = (uint32_t) ((-pidRtcOutput - 0.5) / 10);
            msCorrection             = (uint32_t)  (-pidRtcOutput - 0.5) - subSecondsCorrection * 10; // in fact, this is tick level correction
          }
          else
          {
            subSecondsCorrection     = (uint32_t) ((pidRtcOutput + 0.5) / 10);
            msCorrection             = (uint32_t)  (pidRtcOutput + 0.5) - subSecondsCorrection * 10;
          }
          if (msCorrection)
          { // if msCorrection is > 0, then add 10ms to the RTC setting and wait to start the RTC with 10ms - msCorrection
            // todo what if subSecondsCorrection = 99??
        	subSecondsCorrection++;
       	    msCorrection = 10 - msCorrection;
          }
#if !ANTENNARSSITEST
#if PRINTF_APP_HAL_SYNC
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [hal_sync] PID RTC done. Error = %03.3fms, PIDOutput =  %03.3fms, compss = %dds, compms = %dms.\r\n",
              (unsigned int) xTaskGetTickCount(), pidRtcError, pidRtcOutput, subSecondsCorrection / ((int)tickSpeedToReference), msCorrection / ((int)tickSpeedToReference));
              huart2print(uart_buf, strlen(uart_buf));
#endif
#endif
#if !STM32WBAUSED
          setClock    = 1; // Stop the i2c bus use of the gnss module until the RTC is running again
#endif
//          if (!changeClock)
//          { // give only once a notification to change clock.
//        	if ((formerSubSecondsCorrection == subSecondsCorrection) && (formerMsCorrection == msCorrection))
//        	{
//#if PRINTF_APP_HAL_SYNC
//              waitToPrint();
//              npf_snprintf(uart_buf, 200, "%u [hal_sync] RTC Will not be changed as the compensation values are identical to the former values.\r\n", (unsigned int) xTaskGetTickCount());
//              huart2print(uart_buf, strlen(uart_buf));
//#endif
//        	}
//        	else
//        	{
////20240725 start
//              changeClock = 1;
//              app_rtc_notify(NOTIFICATION_FROM_APP_HAL_SYNC_TO_SET_RTC); // Now the RTC can be notified to start to change of the RTC (new iteration in app_rtc endless loop)
////20240725 end
////#if PRINTF_APP_HAL_SYNC
////            waitToPrint();
////            npf_snprintf(uart_buf, 200, "[hal_sync] RTC Change Command at %ums.\r\n", (unsigned int) xTaskGetTickCount());
////            huart2print(uart_buf, strlen(uart_buf));
////#endif
//              formerSubSecondsCorrection = subSecondsCorrection;
//              formerMsCorrection = msCorrection;
//        	}
//          }
        }
      } // endif !pairingOngoing
      if (!startpidRtc)
      { // set the clock for the first time after HAL/RTOS is synchronized
        startpidRtc = 1;
#if !STM32WBAUSED
        setClock    = 1; // Stop the i2c bus use of the gnss module until the RTC is running again
#endif
        changeClock = 1;
        app_rtc_notify(NOTIFICATION_FROM_APP_HAL_SYNC_TO_SET_RTC); //20240423 Now the RTC can be notified to change the RTC (first iteration in app_rtc endless loop)
        rtosTimeStampChangeRTC = xTaskGetTickCount();
#if PRINTF_APP_HAL_SYNC
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [hal_sync] 1st RTC Change Command at %ums. Difference GNSSPPS - RTCPPS = %dms. Now start also RTC PID.\r\n",
        		(unsigned int) xTaskGetTickCount(), (unsigned int) (rtosTimeStampChangeRTC / ((int)tickSpeedToReference)), (int) (rtosTimeDifference / ((int)tickSpeedToReference)));
        huart2print(uart_buf, strlen(uart_buf));
#endif
      }
    }
#endif //SENSOR_HOST
    previousRtosTimeStampRTCPPS = rtosTimeStampRTCPPS;
    previousSysTicknsValue = sysTicknsValue;
    previousTickCountRTCPP = tickCountRTCPPS;
  }
}

void app_hal_sync_notify_fromISR(uint32_t notValue)
{
  BaseType_t xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  if (halSyncThreadHandler != NULL)
  {
    xTaskNotifyFromISR(halSyncThreadHandler, notValue, eSetBits, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


// Example of PID controller:

//#include <stdio.h>
//
//// Define constants
//#define Kp 1.0
//#define Ki 0.5
//#define Kd 0.1
//
//// Initialize variables
//float error = 0.0;
//float integral = 0.0;
//float derivative = 0.0;
//float previous_error = 0.0;
//float output = 0.0;
//float setpoint = 50.0;
//float process_variable = 0.0;
//
//int main()
//{
//    // Loop until process variable reaches setpoint
//    while (process_variable != setpoint) {
//        // Calculate error
//        error = setpoint - process_variable;
//
//        // Calculate integral
//        integral += error;
//
//        // Calculate derivative
//        derivative = error - previous_error;
//
//        // Calculate output
//        output = Kp * error + Ki * integral + Kd * derivative;
//
//        // Update previous error
//        previous_error = error;
//
//        // Update process variable
//        process_variable += output;
//
//        // Print output
//        printf("Output: %f\n", output);
//    }
//
//    return 0;
//}
