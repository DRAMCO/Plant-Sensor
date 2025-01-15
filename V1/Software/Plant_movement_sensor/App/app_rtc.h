/*
 * app_rtc.h
 *
 *  Created on: Feb 19, 2023
 *      Author: Sarah Goossens
 */

#ifndef APP_RTC_H_
#define APP_RTC_H_

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "main.h"

#define TIME_DELAY_TO_START_RTC               0x08   // this is the extra (STM32 and I2C) delay between start command and actual start of the RTC, was 0x06
#define TIME_TO_CHANGE_CLOCK                   10U  // nr of seconds between refreshing (setting) the RTC (now 2 minutes)


void RtcThreadInit(void);
void StartRtcThread(const void * params);
void app_rtc_notify(uint32_t notValue);
void app_rtc_notify_fromISR(uint32_t notValue);



#endif /* APP_RTC_H_ */
