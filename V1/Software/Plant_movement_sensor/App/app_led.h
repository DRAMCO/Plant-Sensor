/*
 * app_led.h
 *
 *  Created on: Oct 22, 2022
 *      Author: Sarah Goossens
 */

#ifndef APP_LED_H_
#define APP_LED_H_

#include "main.h"

void LedThreadInit();
void StartLedThread(const void * params);
//void app_led_notify_fromISR(uint32_t notValue);
void LedThreadNotify(uint32_t notValue);



#endif /* APP_LED_H_ */
