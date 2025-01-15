/*
 * app_node_gnsspps.h
 *
 *  Created on: Oct 3, 2023
 *      Author: Sarah Goossens
 */

#ifndef APP_NODE_GNSSPPS_H_
#define APP_NODE_GNSSPPS_H_

#include "FreeRTOS.h"
#if !STM32WBAUSED
  #include "cmsis_os.h"
#endif

#include "main.h"

void GNSSPPSThreadInit();
void StartGNSSPPSThread();
void app_gnsspps_notify_fromISR(uint32_t notValue);
void app_gnsspps_notify(uint32_t notValue);


#endif /* APP_NODE_GNSSPPS_H_ */
