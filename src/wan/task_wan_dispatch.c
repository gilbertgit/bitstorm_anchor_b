/*
 * task_wan_dispatch.c
 *
 *  Created on: Dec 17, 2015
 *      Author: titan
 */
#include "asf.h"
#include "task_wan_dispatch.h"
#include "task_ble_serial.h"
#include "../shared.h"

#define TASK_WAN_DISPATCH_STACK_SIZE                (1024/sizeof(portSTACK_TYPE))
#define TASK_WAN_DISPATCH_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define BUFFER_MAX		50
#define BUFFER_SIZE     (BUFFER_MAX + 1)
#define QUEUE_TICKS		10

enum comands {
	CONFIG_RESP = 0x03, CHANGESET = 0X09, WAN_STATUS = 0xEF
};
enum wan_statuses {
	MSG_TIMEOUT = 0x0A, CONF_RETRIES = 0x0B, MSG_ERROR = 0x0C, INVALID_MSG = 0x0D, SYNC_CONF = 0x0E, FATAL = 0xEE
};

QueueHandle_t xWanDispatchQueue;

static signed char outBuffer[BUFFER_SIZE];
static BaseType_t result;

uint8_t errorMsgCounter = 0;
bool queue_created = false;


static portTASK_FUNCTION(task_wan_dispatch, params)
{
	uint8_t cmd;
	uint8_t wan_status;

	for (;;)
		{
			xWanDispatchMonitorCounter++;
			result = xQueueReceive( xWanDispatchQueue, outBuffer, QUEUE_TICKS);
			if (result == pdTRUE )
			{
				cmd = outBuffer[0];
				switch (cmd)
				{
				case CHANGESET:
					//update_changeset();
					xQueueSendToBack(xBleQueue, outBuffer, 0);
					break;
				case WAN_STATUS:
					// successful sync will clear the counter
					wan_status = outBuffer[1];
					switch (wan_status)
					{
					case SYNC_CONF:
						sync_count = 0;
						errorMsgCounter++;
						break;
					case CONF_RETRIES:
						// not cool. LWM had an issue and couldn't get a
						// message to the coordinator after 3 retries. REBOOT!
						//reboot_1284(CONF_RETRIES);
						break;
					case MSG_TIMEOUT:
						errorMsgCounter++;
						break;
					case INVALID_MSG:
						errorMsgCounter++;
						break;
					}
					break;
				}
			}
		}

}

void task_wan_dispatch_start(UBaseType_t uxPriority)
{
	if (!queue_created)
		xWanDispatchQueue = xQueueCreate( 10, BUFFER_SIZE );

	if (xWanDispatchQueue == 0)
	{
//		led_alert_on();
	} else
	{
		queue_created = true;
		xTaskCreate(task_wan_dispatch, "wan_dispatch", TASK_WAN_DISPATCH_STACK_SIZE, NULL, TASK_WAN_DISPATCH_STACK_PRIORITY, ( TaskHandle_t * ) NULL);

	}
}
