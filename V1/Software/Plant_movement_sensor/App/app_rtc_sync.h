/*
 * app_rtc_sync.h
 *
 *  Created on: Apr 11, 2023
 *      Author: Sarah Goossens
 */

#ifndef APP_RTC_SYNC_H_
#define APP_RTC_SYNC_H_

#include "FreeRTOS.h"
#if !STM32WBAUSED
  #include "cmsis_os.h"
#endif
#include "main.h"

void RtcSyncThreadInit();
void StartRtcSyncThread();
void app_rtc_sync_notify_fromISR(uint32_t notValue);
void app_rtc_sync_notify(uint32_t notValue);


#endif /* APP_RTC_SYNC_H_ */
