/*
 * task_ble_dispatch.h
 *
 *  Created on: Dec 8, 2015
 *      Author: titan
 */

#include "asf.h"
#include "ble_msg.h"
#include "../shared.h"


#ifndef TASK_BLE_DISPATCH_H_
#define TASK_BLE_DISPATCH_H_

extern QueueHandle_t xDispatchQueue;
extern uint16_t xBleDispatchMonitorCounter;

void task_ble_dispatch_start();
//bool btle_handle_le_packet(char * buffer, btle_msg_t *);
uint8_t btle_parse_nybble(char c);

void read_changeset();

//bool handle_router_config_packet(char * buffer, router_config_t * conf);
bool handle_router_config_packet2(char * buffer, router_config_t * conf);

bool btle_handle_le_packet2(char * buffer, btle_msg_t * btle_msg);
void system_class();
void attr_db_class();
void gap_class();
void connection_class();
void reset_transmit();
void update_rssi_threshold();

#endif /* TASK_BLE_DISPATCH_H_ */
