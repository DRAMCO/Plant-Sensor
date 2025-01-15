/*
 * app_gateway.h
 *
 *  Created on: 17 Sep 2023
 *      Author: Sarah Goossens
 */

#ifndef APP_GATEWAY_H_
#define APP_GATEWAY_H_

#include "main.h"
//#include "../../App/app_lorawan_defs.h"

#define CALL_LR1MAC_PERIOD_MS 400

void GatewayThreadInit();
void GatewayThreadStart();
void GatewayNotifyFromISR(uint32_t notValue);
void GatewayNotify(uint32_t notValue);
//lr1mac_states_t lr1mac_core_process( lr1mac_states_t  lr1mac_state, user_rx_packet_type_t* available_rx_packet );
//lr1mac_states_t lr1mac_core_process( user_rx_packet_type_t* available_rx_packet );

#endif /* APP_GATEWAY_H_ */
