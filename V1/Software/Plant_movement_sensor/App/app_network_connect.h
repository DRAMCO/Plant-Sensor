/*
 * app_network_connect.h
 *
 *  Created on: Apr 15, 2023
 *      Author: Sarah Goossens
 *
 */

#ifndef APP_NETWORK_CONNECT_H_
#define APP_NETWORK_CONNECT_H_

#include "FreeRTOS.h"
#if !STM32WBAUSED
  #include "cmsis_os.h"
#endif

#include "main.h"

void NetConThreadInit();
void StartNetConThread();
void app_netw_con_notify_fromISR(uint32_t notValue);
void NwConThreadNotify(uint32_t notValue);

#endif /* APP_NETWORK_CONNECT_H_ */
