/*
 * shared.h
 *
 *  Created on: Dec 8, 2015
 *      Author: titan
 */

#ifndef SHARED_H_
#define SHARED_H_

#include <stdint.h>

typedef struct
{
	uint8_t magic;
	uint64_t mac;
	uint16_t pan_id;
	uint8_t channel;
}router_config_t;

typedef struct
{
	uint8_t threshold;
}rssi_threshold_t;

extern router_config_t router_config;
extern uint8_t sync_count;
extern uint32_t my_tick_count;
extern router_config_t router_config;
extern rssi_threshold_t rssi_threshold;




#endif /* SHARED_H_ */
