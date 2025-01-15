/*
 * app_hal_pps.h
 *
 *  Created on: 22 apr. 2023
 *      Author: Sarah Goossens
 */

#ifndef APP_HAL_PPS_H_
#define APP_HAL_PPS_H_

#include "FreeRTOS.h"
#if !STM32WBAUSED
  #include "cmsis_os.h"
#endif
#include "main.h"


void HalPPSThreadInit(void);
void StartHalPPSThread(const void * params);
void app_hal_pps_notify(uint32_t notValue);

#endif /* APP_HAL_PPS_H_ */
