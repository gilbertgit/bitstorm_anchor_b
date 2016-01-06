/*
 * task_wan.c
 *
 *  Created on: Dec 8, 2015
 *      Author: titan
 */

#include "task_wan.h"
#include "wan_msg.h"
#include "ble_msg.h"
#include "task_wan_dispatch.h"
#include "router_status_msg.h"
#include "wan.h"
#include "flash.h"
#include "../shared.h"
#include "ncp_spi.h"

#define TASK_WAN_STACK_SIZE                (1024/sizeof(portSTACK_TYPE))
#define TASK_WAN_STACK_PRIORITY            (tskIDLE_PRIORITY)
#define TASK_WAN_RX_STACK_SIZE             (1024/sizeof(portSTACK_TYPE))
#define TASK_WAN_RX_STACK_PRIORITY         (tskIDLE_PRIORITY)

#define BUFFER_MAX		50
#define BUFFER_SIZE     (BUFFER_MAX + 1)
#define QUEUE_TICKS		10
#define WAN_CONFIG_RETRY_MAX	3

QueueHandle_t xWANQueue;
TaskHandle_t xWanTaskHandle;
uint16_t xWanMonitorCounter;
router_config_t router_config;

static int wan_config_retry = 0;
static int frame_index = 0;
static signed char outBuffer[BUFFER_SIZE];
static uint8_t inBuffer[BUFFER_SIZE];
static uint16_t router_serial;
static uint32_t message_counter;
static bool is_wan_configuring = false;
static uint8_t frame[80];
static uint8_t status_msg_frame[50];
static cmd_send_header_t cmd_header;
static tag_msg_t tag_msg;

uint8_t cobs_buffer[60];
uint8_t decoded_msg[60];

uint8_t cobs_buffer[60];
uint8_t decoded_msg[60];
uint8_t sync_count = 0;

//static bool queue_created = false;

//void pin_edge_handler(const uint32_t id, const uint32_t index)
//{
//	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//
//	if ((id == ID_PIOA) && (index == PIO_PA16)) {
//		if (pio_get(PIOA, PIO_TYPE_PIO_INPUT, PIO_PA16)) {
//			vTaskNotifyGiveFromISR(xWanTaskHandle, &xHigherPriorityTaskWoken);
//
//			if (xHigherPriorityTaskWoken != pdFALSE) {
//				taskYIELD();
//			}
//		}
//	}
//}

void config_done_handler() //const uint32_t id, const uint32_t index)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (pio_get(PIOA, PIO_TYPE_PIO_INPUT, PIO_PA8)) {

		vTaskNotifyGiveFromISR(xWanTaskHandle, &xHigherPriorityTaskWoken);

		if (xHigherPriorityTaskWoken != pdFALSE) {
			taskYIELD();
		}
	}
}

//void nHost_INT_handler()
//{
//
//}

//void init_pins_temp()
//{
//	// enable host int
//	pmc_enable_periph_clk(ID_PIOA);
//	pio_set_input(PIOA, PIO_PA16, PIO_PULLUP);
//	pio_handler_set(PIOA, ID_PIOA, PIO_PA16, PIO_IT_EDGE, nHost_INT_handler);
//	pio_enable_interrupt(PIOA, PIO_PA16);

// CTS0 - WAN CONFIG LINE
//	pmc_enable_periph_clk(ID_PIOA);
//	pio_set_input(PIOA, PIO_PA8, PIO_PULLUP);
//	//pio_handler_set(PIOA, ID_PIOA, PIO_PA8, PIO_IT_EDGE, pin_edge_handler);
//	pio_handler_set(PIOA, ID_PIOA, PIO_PA8, PIO_IT_EDGE, config_done_handler);
//	pio_enable_interrupt(PIOA, PIO_PA8);

	//NVIC_EnableIRQ(PIOA_IRQn);
//}

void testSaveSettings()
{
	cmd_config_ntw_t config_ntw;
	config_ntw.command = 0x07;
	config_ntw.pan_id = 0x1975;
	config_ntw.short_id = 0xAAAA;
	config_ntw.channel = 0x16;

	write_flash(&config_ntw, sizeof(config_ntw));

	flash_read();

}

 static int spi_wake() {
	status_code_t result = STATUS_OK;
	uint16_t buff;
	uint8_t pcs = 0;

	spi_set_peripheral_chip_select_value(NCP_SPI, (~(1U << NCP_CHIP_SELECT)));

	//NOTE: Requires a change in ASF FreeRTOS SPI code freertos_spi_master.c line 423
	//      This is because by default, the ASF code was blasting the buffer with 0xff for the read,
	//      but in this case, we need the "dummy" bytes of the header to remain.  So I just commented out
	//      the memset() line in the ASF code.

	do{
		buff = 0xFF;
//	if (freertos_spi_read_packet(NCP_SPI, &buff, 1, NCP_SPI_MAX_BLOCK_TIME) != STATUS_OK) {
//		result = STATUS_ERR_TIMEOUT;
//	}
		spi_write(NCP_SPI, buff, pcs, 0);
		spi_read(NCP_SPI, &buff, &pcs);
	}
	while ((buff & 0xFF) == 0xFF);

	spi_set_peripheral_chip_select_value(NCP_SPI, NCP_NONE_CHIP_SELECT_VALUE);
	spi_set_lastxfer(NCP_SPI);

	if (result != STATUS_OK) {
		printf("readfromspi_serial timeout\r\n");
		for (;;) {
		}
	}

	return result;
}

static void task_wan(void *pvParameters)
{
	UNUSED(pvParameters);
	init_pins_temp();
	const uint8_t *buffer;
	uint32_t length;
	uint8_t ff_buffer[6];
	uint8_t header[2];                                       // buffer to compose header in
	int cnt = 2;
	int ff_cnt = 6;

	header[0] = 0x0A;
	header[1] = 0xA7;

	ff_buffer[0] = 0xFF;
	ff_buffer[1] = 0xFF;
	ff_buffer[2] = 0xFF;
	ff_buffer[3] = 0xFF;
	ff_buffer[4] = 0xFF;
	ff_buffer[5] = 0xFF;

	uint16_t byte_b = 0xFF;
	uint8_t pcs = 0;

	for (;;) {
		//testSaveSettings();
		pio_toggle_pin(LED_ALERT_IDX);
		writetospi(cnt, header, length, buffer);
		while (pio_get_pin_value(PIO_PA16_IDX))
			;
		spi_wake();

//		do {
//
//			readfromspi_serial(1, ff_buffer, 1, buffer);
//		}
//		while (buffer[0] == 0xFF);

		vTaskDelay(1000);
	}

}

static void task_wan_rx(void *pvParameters)
{
	UNUSED(pvParameters);

}

void decode_cobs(const unsigned char *ptr, unsigned long length, unsigned char *dst)
{
	const unsigned char *end = ptr + length;
	while (ptr < end) {
		int i, code = *ptr++;
		for (i = 1; i < code; i++)
			*dst++ = *ptr++;
		if (code < 0xFF)
			*dst++ = 0;
	}
}

void configure_wan()
{
	//PCMSK1 |= WAN_CONFIG_ISR_MSK;

	uint32_t ulNotificationValue;
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS(50);

	wan_config_retry = 0;
	for (;;) {
		//ERIC: Perhaps move the PCMSK set/unset closer to this line rather than outside retry loop.
		// now that we have the mac from the wan, send entire nwk config
		wan_config_network();

		// wait for a response
		ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);

		if (ulNotificationValue == 1) {
			/* The transmission ended as expected. */
			frame_index = 0;
			is_wan_configuring = false;
			break;
		}
		else {
			/* The call to ulTaskNotifyTake() timed out. */
			wan_config_retry++;

			if (wan_config_retry > WAN_CONFIG_RETRY_MAX) {
				//reboot_1284(WAN_CONFIG_RT);
				break;
			}
		}
	}
	//PCMSK1 &= ~WAN_CONFIG_ISR_MSK;
}

void send_router_status_msg(router_msg_t * msg)
{

	msg->routerMac = router_config.mac;
	msg->routerShort = router_config.mac & 0x0000FFFF;
	msg->routerSerial = router_serial;
	msg->routerConfigSet = 0x00;
	msg->routerMsgCount = message_counter;
	msg->routerUptime = my_tick_count;
	msg->routerBattery = 0x00;
	msg->routerTemperature = 0x00;
	status_msg_frame[0] = sizeof(cmd_header) + sizeof(router_msg_t) + 1;

	cmd_header.command = CMD_SEND;
	cmd_header.pan_id = router_config.pan_id;
	cmd_header.short_id = router_config.mac & 0x0000FFFF;
	cmd_header.message_length = sizeof(router_msg_t);

	int frame_index = 1;
// header
	for (int i = 0; i < sizeof(cmd_header); i++) {
		status_msg_frame[frame_index++] = ((uint8_t *) (&cmd_header))[i];
	}
// message
	for (int i = 0; i < sizeof(router_msg_t); i++) {
		status_msg_frame[frame_index++] = ((uint8_t *) (msg))[i];
	}
// checksum
	uint8_t cs = 0;
	for (int i = 0; i < frame_index; cs ^= status_msg_frame[i++])
		;
	status_msg_frame[frame_index++] = cs;

	for (int i = 0; i < frame_index;) {
		wan_tx(status_msg_frame[i++]);
	}
	router_serial++;
}

void send_tag_message(btle_msg_t *msg)
{

	build_app_msg(msg, &tag_msg);

	frame[0] = sizeof(cmd_header) + sizeof(tag_msg) + 1;

	if (msg->type == MSG_TYPE_IN_PROX)
		tag_msg.messageType = CMD_IN_PROX;
	else if (msg->type == MSG_TYPE_OUT_PROX)
		tag_msg.messageType = CMD_OUT_PROX;

#ifdef ZB_ACK
	cmd_header.command = CMD_ACK_SEND;
#else
	cmd_header.command = CMD_SEND;
#endif
	cmd_header.pan_id = router_config.pan_id;
	cmd_header.short_id = 0x00;
	cmd_header.message_length = sizeof(tag_msg);

	int frame_index = 1;
// header
	for (int i = 0; i < sizeof(cmd_header); i++) {
		frame[frame_index++] = ((uint8_t *) (&cmd_header))[i];
	}
// message
	for (int i = 0; i < sizeof(tag_msg_t); i++) {
		frame[frame_index++] = ((uint8_t *) (&tag_msg))[i];
	}
// checksum
	uint8_t cs = 0;
	for (int i = 0; i < frame_index; cs ^= frame[i++])
		;
	frame[frame_index++] = cs;

// send bytes
	for (int i = 0; i < frame_index;) {
		wan_tx(frame[i++]);
	}

// keep track of how many messages we have sent out
	router_serial++;
//	hwm = uxTaskGetStackHighWaterMark(NULL);
//	sprintf((char*)frame, "A:%d\r\n", hwm);
//	for (int i=0; frame[i]; xSerialPutChar(pxWan, frame[i++], 5));
}

void build_app_msg(btle_msg_t *btle_msg, tag_msg_t *msg)
{

	msg->messageType = 0x01;
	msg->routerMac = router_config.mac;
	msg->routerShort = router_config.mac & 0x0000FFFF;
	msg->tagMac = btle_msg->mac;
	msg->tagConfigSet = btle_msg->cs_id;
	msg->tagSerial = btle_msg->tagSerial;
	msg->tagStatus = btle_msg->tagStatus;
	msg->tagLqi = 0x00;
	msg->tagRssi = btle_msg->rssi;
	msg->tagBattery = 0x00;
	msg->tagTemperature = 0x00;
}

void task_wan_start()
{
//	if (!queue_created)
	//xWANQueue = xQueueCreate(10, BUFFER_SIZE);
//
//	if (xWANQueue == 0) {
//		//led_alert_on();
//	} else {
	//queue_created = true;
	xTaskCreate(task_wan, "wan", TASK_WAN_STACK_SIZE, NULL, TASK_WAN_STACK_PRIORITY, xWanTaskHandle);
	//xTaskCreate(task_wan_rx, "wan_rx", TASK_WAN_RX_STACK_SIZE, NULL, TASK_WAN_RX_STACK_PRIORITY, NULL);
	//}
}
