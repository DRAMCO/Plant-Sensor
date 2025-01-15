/*
 * app_init.h
 *
 *  Created on: Oct 22, 2022
 *      Author: Sarah Goossens
 */

#ifndef APP_INIT_H_
#define APP_INIT_H_

#include "main.h"
//#include "FreeRTOS.h"
//#include "cmsis_os2.h"

/* Initialisation of the whole project */
void ProjectInit(void);
void InitThreadNotify(uint32_t notValue);
void app_DIO1_notify_from_isr(uint32_t notValue);
void app_init_notify_fromISR(uint32_t notValue);

#endif /* APP_INIT_H_ */
