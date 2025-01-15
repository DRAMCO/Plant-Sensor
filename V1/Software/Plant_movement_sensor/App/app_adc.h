/*
 * app_adc.h
 *
 *  Created on: Dec 15, 2023
 *      Author: Sarah Goossens
 */

#ifndef APP_ADC_H_
#define APP_ADC_H_

void AdcThreadInit();
void StartAdcThread(const void * params);
void AdcNotifyFromISR(uint32_t notValue);
void AdcThreadNotify(uint32_t notValue);

#endif /* APP_ADC_H_ */
