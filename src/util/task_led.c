/*
 * task_led.c
 *
 *  Created on: Dec 8, 2015
 *      Author: titan
 */

#include "led.h"

#define TASK_LED_STACK_SIZE                (1024/sizeof(portSTACK_TYPE))
#define TASK_LED_STACK_PRIORITY            (tskIDLE_PRIORITY)

static void task_led(void *pvParameters) {
	UNUSED(pvParameters);
	for (;;) {
		pio_toggle_pin(LED_STATUS_IDX);
		vTaskDelay(1000);
	}
}

void task_led_start()
{
	if (xTaskCreate(task_led, "Led", TASK_LED_STACK_SIZE, NULL, TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
			printf("Failed to create test led task\r\n");
		}
}
