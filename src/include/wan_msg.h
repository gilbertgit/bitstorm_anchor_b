/*
 * wan_msg.h
 *
 *  Created on: Dec 8, 2015
 *      Author: titan
 */

#ifndef WAN_MSG_H_
#define WAN_MSG_H_

#define MSG_TYPE_NORM 		1
#define MSG_TYPE_IN_PROX 	5
#define MSG_TYPE_OUT_PROX 	6

typedef struct {
	uint8_t rssi;
	uint64_t mac;
	uint16_t batt;
	uint16_t temp;

} wan_msg_t;

typedef struct __attribute__((packed)){
	uint8_t command;
	uint16_t pan_id;
	uint16_t short_id;
	uint8_t channel;
} cmd_config_ntw_t;

enum {
	CMD_SEND = 0x01,
	CMD_ACK_SEND = 0x02,
	CMD_CONFIG_NETWORK = 0x03,
	CMD_GET_ADDRESS = 0x04,
	CMD_IN_PROX = 0x05,
	CMD_OUT_PROX = 0x06,
	CMD_CONFIG_DONE = 0x07,
	CMD_ROUTER_STATUS = 0X08

};

typedef struct __attribute__((packed)) tag_msg_t {
	uint8_t messageType;		// 0x01 for tag
	uint64_t routerMac;			// full MAC or source router
	uint16_t routerShort;		// short id of MAC source router - used for two-way comm
	uint64_t tagMac;			// full MAC of ble tag
	uint16_t tagConfigSet;		// current configuration set id of tag
	uint16_t tagSerial;			// sequential serial number of tag’s report - used to group pings
	uint16_t tagStatus;			// tag status value (bit mask) within adv data
	uint8_t tagLqi;				// link quality indicator of tag-to-router signal
	uint8_t tagRssi;			// signal strength of tag-to-router signal
	uint32_t tagBattery;		// raw ADC value of battery read - currently not reliable but kept for future uses
	uint32_t tagTemperature;	// raw ADC value of temp sensor on tag - reserved for future use

} tag_msg_t;

#endif /* WAN_MSG_H_ */
