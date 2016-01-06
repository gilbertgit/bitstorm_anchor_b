/*
 * wan_config.c
 *
 *  Created on: Dec 8, 2015
 *      Author: titan
 */

#include "wan.h"
#include "wan_msg.h"
#include "task_ble_dispatch.h"
#include "../shared.h"

void wan_config_network() {
	cmd_config_ntw_t config_ntw;

	config_ntw.command = CMD_CONFIG_NETWORK;
	config_ntw.pan_id = 0x1973; // router_config.pan_id;
	config_ntw.short_id = 0x1989; // router_config.mac & 0x0000FFFF;
	config_ntw.channel = 0x16; // router_config.channel;

	uint8_t frame[10];
	frame[0] = sizeof(config_ntw) + 1; // size of message

	int frame_index = 1;
	//config
	for (int i = 0; i < sizeof(config_ntw); i++) {
		frame[frame_index++] = ((uint8_t *) (&config_ntw))[i];
	}
	// checksum
	uint8_t cs = 0;
	for (int i = 0; i < frame_index; cs ^= frame[i++])
		;
	frame[frame_index++] = cs;

	for (int i = 0; i < frame_index;) {
		wan_tx(frame[i++]);
	}
}
