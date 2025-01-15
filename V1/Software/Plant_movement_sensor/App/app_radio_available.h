/*
 * app_radio_available.h
 *
 *  Created on: May 13, 2023
 *      Author: Sarah Goossens
 */

#ifndef APP_RADIO_AVAILABLE_H_
#define APP_RADIO_AVAILABLE_H_

//#include "FreeRTOS.h"
//#include "cmsis_os2.h"
#include "main.h"

void RadioAvailableThreadInit();
void RadioAvailableThreadStart();
void RadioAvailableNotifyFromISR(uint32_t notValue);



#endif /* APP_RADIO_AVAILABLE_H_ */
