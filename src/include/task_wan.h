/*
 * task_wan.h
 *
 *  Created on: Dec 8, 2015
 *      Author: titan
 */

#ifndef TASK_WAN_H_
#define TASK_WAN_H_

#include "asf.h"
#include "conf_board.h"
#include "task_ble_dispatch.h"
#include "ble_msg.h"
#include "wan_msg.h"
#include "router_status_msg.h"

extern QueueHandle_t xWANQueue;
extern TaskHandle_t xWanTaskHandle;
extern uint16_t xWanMonitorCounter;

void config_done_handler();
void configure_wan();
void task_wan_start();
void decode_cobs(const unsigned char *ptr, unsigned long length, unsigned char *dst);
void build_app_msg(btle_msg_t *btle_msg, tag_msg_t *msg);
void send_router_status_msg(router_msg_t * msg);
void send_tag_message(btle_msg_t *msg);


#endif /* TASK_WAN_H_ */
