/*
 * task_ble_dispatch.c
 *
 *  Created on: Dec 8, 2015
 *      Author: titan
 */

#include "task_ble_dispatch.h"
#include "task_ble_serial.h"
#include "task_wan.h"
#include "../shared.h"
#include "ble_msg.h"

#define BUFFER_MAX		50
#define BUFFER_SIZE     (BUFFER_MAX + 1)
#define QUEUE_TICKS		10
#define RSSI_INDEX 			0
#define PACKET_TYPE_INDEX 	1
#define ADDRESS_INDEX 		2
#define ADDRESS_TYPE_INDEX 	8
#define BOND_INDEX 			9
#define DATA_LENGTH_INDEX 	10
#define DATA_INDEX 			11

#define TASK_BLE_DISPATCH_STACK_SIZE                (1024/sizeof(portSTACK_TYPE))
#define TASK_BLE_DISPATCH_STACK_PRIORITY            (tskIDLE_PRIORITY)

static signed char outBuffer[BUFFER_SIZE];

static BaseType_t result;
static btle_msg_t msg;

uint16_t xBleDispatchMonitorCounter;
QueueHandle_t xDispatchQueue;
router_config_t router_config;
rssi_threshold_t rssi_threshold;
//changeset_t cs_v;

static bool is_configured = false;
static bool is_configuring = false;
static bool is_connected = false;
static uint8_t configCmd[] = { 0x06, 0x02 };
static uint8_t helloTestCmd[] = { 0x07, 0x03 };

//static uint8_t configWanCmd[] = { 0x09, 0x09 };

const static TickType_t xDelay = 500 / portTICK_PERIOD_MS;
char HEX_DIGITS[] = "0123456789abcdef";

enum classes
{
	SYSTEM_CLASS = 0x00, ATTR_DB_CLASS = 0x02, CONNECTION = 0x03, GAP_CLASS = 0x06
};
uint64_t router_addr_temp;

static portTASK_FUNCTION(task_dispatch, params)
{
	//read_config();
	//read_changeset();
	//read_rssi_threshold();
	for (;;) {

		xBleDispatchMonitorCounter++;
		result = xQueueReceive(xDispatchQueue, outBuffer, QUEUE_TICKS);
		if (result == pdTRUE) {
			switch (outBuffer[2])
			{
			case ATTR_DB_CLASS:
				attr_db_class();
				break;
			case GAP_CLASS:
				gap_class();
				break;
			case SYSTEM_CLASS:
				system_class();
				break;
			case CONNECTION:
				connection_class();
				break;
			}
			memset(outBuffer, 0, BUFFER_SIZE);

			// TODO: check if the tag that sent the message has the correct changeset
			// if not correct, put in ble queue that changeset needs to be updated for this tag
//			if (msg.cs_id == changeset.id)
//				led_alert_on();

		}
	}
}

void system_class()
{
	switch (outBuffer[3])
	{
	case 0x02: // address response
		if (router_config.magic == 1) {
			outBuffer[10] = 0x00;
			outBuffer[11] = 0x00;
			router_config.mac = *(uint64_t*) &outBuffer[4];
			router_config.magic = 2;
			//write_config();
		}
		break;
	case 0x00: // ble boot
		//TODO: if we missed this, we need to figure out a way to resend it
		xQueueSendToBack(xBleQueue, helloTestCmd, 0);
		break;
	case 0x01: // ble boot
			   //TODO: if we missed this, we need to figure out a way to resend it
		xQueueSendToBack(xBleQueue, configCmd, 0);
		break;
	}
}

static uint8_t getAddrCmd[] = { 0x05, 0x00, 0x00, 0x00, 0x02 };
#define CONFIG_HANDLE 23
#define RSSI_THRESHOLD_HANDLE 26

void attr_db_class()
{

	switch (outBuffer[3])
	{
	case 0x00: // GATT value change
		is_connected = true;
		is_configuring = true;
		switch ((uint16_t) outBuffer[6])
		{
		case CONFIG_HANDLE:
			handle_router_config_packet2((char*) outBuffer, &router_config);
			// get the ble address to finish the config
			xQueueSendToBack(xBleQueue, getAddrCmd, 0);
			break;
		case RSSI_THRESHOLD_HANDLE:
			update_rssi_threshold();
			break;
		}
		break;
	}
}

uint8_t discoverParams[] = { 0x0A, 0x00, 0x05, 0x06, 0x07, 0x40, 0x00, 0x32, 0x00, 0x00 };
uint8_t discoverCmd[] = { 0x06, 0x00, 0x01, 0x06, 0x02, 0x01 };
uint8_t modeCmd[] = { 0x07, 0x00, 0x02, 0x06, 0x01, 0x02, 0x02 };

void gap_class()
{
	// what is the method
	switch (outBuffer[3])
	{
	case 0x00: // discover event
		// TODO: filter data that is not from BS tags //// 0x54, 0x67 = TG
		if (outBuffer[20] == 0x54 && outBuffer[21] == 0x67) {
			// check if the message is within the rssi threshold (default is -99)
			if (abs(outBuffer[4]) <= rssi_threshold.threshold) {
				// start reading after the header on the 5th byte
				btle_handle_le_packet2((char *) &outBuffer[4], &msg);

				// if the router is configured and the BTLE is not connected to, send message to WAN queue
				if (router_config.magic == 2 && is_connected == false) {
					msg.type = 0x01;
					result = xQueueSendToBack(xWANQueue, &msg, 0);
				}
			}
		}

		break;
	case 0x03: // connect response

		break;
	case 0x04: // end discover resp
		xQueueSendToBack(xBleQueue, discoverParams, 0);
		break;
	case 0x07: // discover params resp
		// send discover cmd
		xQueueSendToBack(xBleQueue, discoverCmd, 0);
		break;
	case 0x02: // discover cmd resp
		// send mode cmd
		xQueueSendToBack(xBleQueue, modeCmd, 0);
		is_configured = true;
		break;
	}
}

void connection_class()
{
	switch (outBuffer[3])
	{
	case 0x04: // disconnect event
		is_connected = false;

		if (router_config.magic == 2) {
			if (is_configuring) {
				// just restart the wan after we get the config
				// the wan_task will take care of configuring it
//				kill_wan();
//				vTaskDelay(xDelay);
//				init_wan();
				while (1)
					;
			}
		}
		xQueueSendToBack(xBleQueue, configCmd, 0);
		is_configuring = false;
		break;
	}
}

void update_rssi_threshold()
{
	rssi_threshold.threshold = (uint8_t) outBuffer[11];
	//write_rssi_threshold();
}

bool btle_handle_le_packet2(char * msg, btle_msg_t * btle_msg)
{
	memset(btle_msg, 0, sizeof(btle_msg_t));

	btle_msg->rssi = msg[RSSI_INDEX];
	btle_msg->mac = (uint64_t) msg[6] << 32 | (uint64_t) msg[5] << 24 | (uint64_t) msg[4] << 16 | (uint64_t) msg[3] << 8 | (uint64_t) msg[2];
	btle_msg->batt = 0;
	btle_msg->temp = 0;
	btle_msg->cs_id = (uint16_t) msg[23] << 8 | (uint16_t) msg[22];
	btle_msg->tagSerial = (uint16_t) msg[25] << 8 | (uint16_t) msg[24];
	btle_msg->tagStatus = (uint16_t) msg[27] << 8 | (uint16_t) msg[26];

	return true;
}

bool handle_router_config_packet2(char * buffer, router_config_t * conf)
{
	memset(conf, 0, sizeof(router_config_t));
	//conf->pan_id = *((uint16_t*) &buffer[11]);
	conf->pan_id = (uint16_t) buffer[11] << 8 | (uint16_t) buffer[12];
	conf->channel = buffer[13];
	conf->magic = 1;
	return true;
}

uint8_t btle_parse_nybble(char c)
{
	if (c >= 'A' && c <= 'F')
		c = c | 0x20;
	for (uint8_t i = 0; i < 16; i++) {
		if (HEX_DIGITS[i] == c)
			return i;
	}
	return 0x80;
}

void task_ble_dispatch_start()
{
	xDispatchQueue = xQueueCreate(10, BUFFER_SIZE);
	if (xDispatchQueue == 0) {
		//led_alert_on();
	}
	else {
		xTaskCreate(task_dispatch, "dispatch", TASK_BLE_DISPATCH_STACK_SIZE, NULL, TASK_BLE_DISPATCH_STACK_PRIORITY, ( TaskHandle_t * ) NULL);
	}
}
