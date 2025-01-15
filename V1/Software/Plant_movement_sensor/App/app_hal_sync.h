/*
 * app_hal_sync.h
 *
 *  Created on: Apr 11, 2023
 *      Author: Sarah Goossens
 */

#ifndef APP_HAL_SYNC_H_
#define APP_HAL_SYNC_H_

#include "FreeRTOS.h"
#if !STM32WBAUSED
  #include "cmsis_os.h"
#endif
#include "main.h"

void HalSyncThreadInit();
void StartHalSyncThread();
void app_hal_sync_notify_fromISR(uint32_t notValue);


#endif /* APP_HAL_SYNC_H_ */
