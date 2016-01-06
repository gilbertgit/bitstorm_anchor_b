/*
 * task_wan_dispatch.h
 *
 *  Created on: Dec 17, 2015
 *      Author: titan
 */

#ifndef TASK_WAN_DISPATCH_H_
#define TASK_WAN_DISPATCH_H_

extern QueueHandle_t xWanDispatchQueue;
extern uint16_t xWanDispatchMonitorCounter;

void synchronize_zigbit();

void task_wan_dispatch_start(UBaseType_t uxPriority);

#endif /* TASK_WAN_DISPATCH_H_ */
