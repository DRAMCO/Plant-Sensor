/*
 * app_supercap.h
 *
 *  Created on: Apr 13, 2024
 *      Author: Sarah Goossens
 */

#ifndef APP_SUPERCAP_H_
#define APP_SUPERCAP_H_

void ScapThreadInit();
void StartScapThread(const void * params);
void ScapThreadNotify(uint32_t notValue);


#endif /* APP_SUPERCAP_H_ */
