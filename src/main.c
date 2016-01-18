#include "asf.h"
#include "conf_board.h"

#include "task_wan.h"
#include "task_ble_serial.h"
#include "led.h"
#include "flash.h"
#include "ncp_spi.h"
#include "string.h"
#include "ezsp_api.h"

#define BUFFER_MAX		20
#define BUFFER_SIZE     (BUFFER_MAX + 1)

#define TASK_MONITOR_STACK_SIZE            (2048/sizeof(portSTACK_TYPE))
#define TASK_MONITOR_STACK_PRIORITY        (tskIDLE_PRIORITY)
#define TASK_CLI_STACK_SIZE                (2048/sizeof(portSTACK_TYPE))
#define TASK_CLI_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define ROUTER
//#define COORDINATOR

static volatile bool bMonitorPaused = true;
uint32_t my_tick_count;
static bool is_awaiting_resp = false;
static bool is_ready = false;
static bool join_sent = false;
static bool send_join = false;

static uint8_t resp_buff[BUFFER_SIZE];
static volatile bool uWait = true;
int frame_index = 0;

void read_ncp_data();

extern void vApplicationMallocFailedHook(void)
{
	taskDISABLE_INTERRUPTS();
	for (;;)
		;
}

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *) pcTaskName);
	for (;;)
		;
}

extern void vApplicationIdleHook(void)
{
}

extern void vApplicationTickHook(void)
{
	my_tick_count++;
}

#define RESET_IDX			PIO_PA15_IDX

#define PINS_RESET_PIO		PIOA
#define PINS_RESET_TYPE		PIO_OUTPUT_1
#define PINS_RESET_MASK		PIO_PA15
#define PINS_RESET_ATTR		PIO_DEFAULT

#define nHOST_INT_PIN		PIO_PA16_IDX
#define nHOST_INT_PIO		PIOA
#define nHOST_INT_TYPE		PIO_TYPE_PIO_INPUT
#define nHOST_INT_MASK		PIO_PA16
#define nHOST_INT_ATTR		PIO_DEFAULT

void reset_ncp()
{
	pio_set_pin_low(RESET_IDX);
	delay_ms(1);
	pio_set_pin_high(RESET_IDX);
}

static void task_monitor(void *pvParameters)
{
	static portCHAR szList[256];
	UNUSED(pvParameters);

	for (;;) {
		if (bMonitorPaused == false) {
			printf("--- task ## %u\r\n", (unsigned int) uxTaskGetNumberOfTasks());
			vTaskList((signed portCHAR *) szList);
			printf(szList);
		}
		vTaskDelay(1000);
	}
}

// CHEAP CLI task to exercise getchar()
// A better solution is to use FreeRTOS+CLI
static void task_cli(void *pvParameters)
{
	uint8_t c;
	UNUSED(pvParameters);
	for (;;) {
		c = getchar();
		if (c == 'm' || c == 'M') {
			bMonitorPaused = (bMonitorPaused ? false : true);
			printf("bMonitorPaused is now %d\r\n", (int) bMonitorPaused);
		}
	}
}

static void configure_console(void)
{
	usart_serial_options_t uart_serial_options = { .baudrate = CONF_UART_BAUDRATE, .paritytype = CONF_UART_PARITY, };
	usart_serial_init(CONF_UART, &uart_serial_options);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

void send_ezsp_msg(uint8_t spi_byte, uint8_t length_byte, void* payload)
{
	uint8_t pcs = 0;
	uint16_t buff;
	uint16_t eof = 0xA7;
	uint16_t resp_spi_byte;
	uint16_t resp_length;
	uint16_t data;

	// ASSERT CS
	spi_set_peripheral_chip_select_value(NCP_SPI, (~(1U << NCP_CHIP_SELECT)));

	// WRITE DATA WITHOUT RESET
	spi_write(NCP_SPI, spi_byte, pcs, 0);
	spi_read(NCP_SPI, &buff, &pcs);
	spi_write(NCP_SPI, length_byte, pcs, 0);
	spi_read(NCP_SPI, &buff, &pcs);

	// SEND THE PAYLOAD
	for (int i = 0; i < length_byte; i++) {
		spi_write(NCP_SPI, ((uint8_t*) payload)[i], pcs, 0);
		spi_read(NCP_SPI, &buff, &pcs);
	}
	// SEND END OF FRAME
	spi_write(NCP_SPI, eof, pcs, 0);
	spi_read(NCP_SPI, &buff, &pcs);

	is_awaiting_resp = true;
	while (uWait)
		;
	read_ncp_data();
	printf("\r\nEZSP-SPI-Response\r\n");
	for (int j = 0; j < frame_index; j++) {
		printf("0x%02x, ", resp_buff[j]);
	}
//	while(pio_get_pin_value(PIO_PA16_IDX) == 1);
	//spi_set_clock_phase(NCP_SPI, NCP_CHIP_SELECT, 1);

// WRITE 0xFF UNTILL WE GET SOMETHING OTHER THAN 0xFF
//			do {
//				buff = 0xFF;
//				spi_write(NCP_SPI, buff, pcs, 0);
//				spi_read(NCP_SPI, &buff, &pcs);
//			}
//			while ((buff & 0xFF) == 0xFF);
//			resp_spi_byte = buff; // FIRST BYTE IS RESPONCE SPI BYTE
//			buff = 0xFF;
//
//			// GET THE MESSAGE LENGTH
//			spi_write(NCP_SPI, buff, pcs, 0);
//			spi_read(NCP_SPI, &buff, &pcs);
//			resp_length = buff;
//			buff = 0xFF;
//			int i = 0;
//			for (; i < resp_length; i++) {
//				spi_write(NCP_SPI, buff, pcs, 0);
//				spi_read(NCP_SPI, &data, &pcs);
//
//				resp_buff[i] = (uint8_t) data;
//			}
//
//			spi_write(NCP_SPI, 0xFF, pcs, 1);
//			spi_read(NCP_SPI, &data, &pcs);
//
//			resp_buff[i] = (uint8_t) data;
//
//			// DEASSERT CS
//			spi_set_peripheral_chip_select_value(NCP_SPI, NCP_NONE_CHIP_SELECT_VALUE);
//
//			//spi_set_clock_phase(NCP_SPI, NCP_CHIP_SELECT, NCP_CLOCK_PHASE);
//
//			printf("\r\nEZSP-SPI-Response\r\n");
//			printf("0x%02x, ", resp_spi_byte);
//			printf("0x%02x, ", resp_length);
//
//			for (int i = 0; i < resp_length + 1; i++) {
//				printf("0x%02x, ", resp_buff[i]);
//			}
	memset(resp_buff, 0, BUFFER_SIZE);

}

void ncp_init_procedure()
{
	uint8_t pcs2 = 0;
	uint16_t data[2];
	data[0] = 0x0A;
	data[1] = 0xA7;

	uint16_t resp;
	int i = 0;

	// RESET NCP
	reset_ncp();

	// WAIT FOR nHOST_INT TO ASSERT
	while (pio_get_pin_value(PIO_PA16_IDX))
		;
//			delay_ms(5);

	// ASSET CS
	spi_set_peripheral_chip_select_value(NCP_SPI, (~(1U << NCP_CHIP_SELECT)));

	// WRITE DATA
	spi_write(NCP_SPI, data[0], pcs2, 0);
	spi_read(NCP_SPI, &resp, &pcs2);

	spi_write(NCP_SPI, data[1], pcs2, 0);
	spi_read(NCP_SPI, &resp, &pcs2);

	is_ready = true;
	is_awaiting_resp = true;
	while (uWait)
		;
	read_ncp_data();
	printf("\r\nEZSP-SPI-Response\r\n");
	for (int j = 0; j < frame_index; j++) {
		printf("0x%02x, ", resp_buff[j]);
	}

//			uint16_t buff;
//			uint16_t ff_buff = 0xFF;
//
//			do {
//				buff = 0xFF;
//				spi_write(NCP_SPI, buff, pcs2, 0);
//				spi_read(NCP_SPI, &buff, &pcs2);
//			}
//			while ((buff & 0xFF) == 0xFF);
//			resp_buff2[i++] = buff;
//
//			spi_write(NCP_SPI, ff_buff, pcs2, 0);
//			spi_read(NCP_SPI, &resp, &pcs2);
//			resp_buff2[i++] = (uint8_t) resp;
//
//			spi_write(NCP_SPI, ff_buff, pcs2, 1);
//			spi_read(NCP_SPI, &resp, &pcs2);
//			resp_buff2[i++] = (uint8_t) resp;
//
//			// DEASSERT CS
//			spi_set_peripheral_chip_select_value(NCP_SPI, NCP_NONE_CHIP_SELECT_VALUE);
//
//			//spi_set_clock_phase(NCP_SPI, NCP_CHIP_SELECT, NCP_CLOCK_PHASE);
//
//			printf("\r\nCMD After Reset\r\n");
//			for (int j = 0; j < i; j++) {
//				printf("0x%02x, ", resp_buff2[j]);
//			}
//			i = 0;
//////////////////////////////////////////////////////////////////////////////////
	delay_ms(5);

	// ASSERT CS
	spi_set_peripheral_chip_select_value(NCP_SPI, (~(1U << NCP_CHIP_SELECT)));

	// WRITE DATA WITHOUT RESET
	spi_write(NCP_SPI, data[0], pcs2, 0);
	spi_read(NCP_SPI, &resp, &pcs2);
	spi_write(NCP_SPI, data[1], pcs2, 0);
	spi_read(NCP_SPI, &resp, &pcs2);

	is_awaiting_resp = true;
	while (uWait)
		;
	read_ncp_data();
	printf("\r\nEZSP-SPI-Response\r\n");
	for (int j = 0; j < frame_index; j++) {
		printf("0x%02x, ", resp_buff[j]);
	}
//			do {
//				buff = 0xFF;
//				spi_write(NCP_SPI, buff, pcs2, 0);
//				spi_read(NCP_SPI, &buff, &pcs2);
//			}
//			while ((buff & 0xFF) == 0xFF);
//			resp_buff2[i++] = buff;
//
//			//spi_write(NCP_SPI, ff_buff, pcs2, 0);
//			//spi_write(NCP_SPI, ff_buff, pcs2, 1);
//
//			spi_write(NCP_SPI, ff_buff, pcs2, 0);
//			spi_read(NCP_SPI, &resp, &pcs2);
//			resp_buff2[i++] = (uint8_t) resp;
//
//			spi_set_peripheral_chip_select_value(NCP_SPI, NCP_NONE_CHIP_SELECT_VALUE);
//
//			//spi_set_clock_phase(NCP_SPI, NCP_CHIP_SELECT, NCP_CLOCK_PHASE);
//
//			printf("\r\nCMD NO Reset\r\n");
//			for (int j = 0; j < i; j++) {
//				printf("0x%02x, ", resp_buff2[j]);
//			}
//			i = 0;
	//////////////////////////////////////////////////////////////////////////////////
	delay_ms(5);

	// SEND EZSP STATUS REQUEST
	uint16_t data_status[2];
	data_status[0] = 0x0B;
	data_status[1] = 0xA7;

	// ASSERT CS
	spi_set_peripheral_chip_select_value(NCP_SPI, (~(1U << NCP_CHIP_SELECT)));

	// WRITE DATA WITHOUT RESET
	spi_write(NCP_SPI, data_status[0], pcs2, 0);
	spi_read(NCP_SPI, &resp, &pcs2);
	spi_write(NCP_SPI, data_status[1], pcs2, 0);
	spi_read(NCP_SPI, &resp, &pcs2);

	is_awaiting_resp = true;
	while (uWait)
		;
	read_ncp_data();
	printf("\r\nEZSP-SPI-Response\r\n");
	for (int j = 0; j < frame_index; j++) {
		printf("0x%02x, ", resp_buff[j]);
	}

//			do {
//				buff = 0xFF;
//				spi_write(NCP_SPI, buff, pcs2, 0);
//				spi_read(NCP_SPI, &buff, &pcs2);
//			}
//			while ((buff & 0xFF) == 0xFF);
//			resp_buff2[i++] = buff;
//
//			spi_write(NCP_SPI, ff_buff, pcs2, 0);
//			spi_read(NCP_SPI, &resp, &pcs2);
//			resp_buff2[i++] = (uint8_t) resp;
//
//			spi_set_peripheral_chip_select_value(NCP_SPI, NCP_NONE_CHIP_SELECT_VALUE);
//
//			printf("\r\nStatus CMD \r\n");
//			for (int j = 0; j < i; j++) {
//				printf("0x%02x, ", resp_buff2[j]);
//			}
//			i = 0;

	memset(resp_buff, 0, BUFFER_SIZE);
}

void nHost_INT_handler()
{
	uWait = false;
}

void read_ncp_data()
{
	uint8_t pcs = 0;
	uint16_t buff = 0xFF;
	uint16_t ff_buff = 0xFF;

	uint16_t resp;
	uint8_t resp_spi_byte;
	uint8_t resp_length;
	frame_index = 0;

	if (is_ready) {
		pio_toggle_pin(LED_STATUS_IDX);

		if (!is_awaiting_resp) {
			// ASSERT CS
			spi_set_peripheral_chip_select_value(NCP_SPI, (~(1U << NCP_CHIP_SELECT)));
		}

		do {
			spi_write(NCP_SPI, ff_buff, pcs, 0);
			spi_read(NCP_SPI, &buff, &pcs);
		}
		while ((buff & 0xFF) == 0xFF);
		resp_buff[frame_index++] = (uint8_t) buff;
		resp_spi_byte = (uint8_t) buff;

		if (resp_spi_byte == 0xFE) {
			// get response length (second byte)
			spi_write(NCP_SPI, ff_buff, pcs, 0);
			spi_read(NCP_SPI, &resp, &pcs);
			resp_buff[frame_index++] = (uint8_t) resp;
			resp_length = (uint8_t) resp;

			// get the rest of the data
			for (int j = 0; j < resp_length + 1; j++) {
				spi_write(NCP_SPI, ff_buff, pcs, 0);
				spi_read(NCP_SPI, &resp, &pcs);
				resp_buff[frame_index++] = (uint8_t) resp;
			}

			if (resp_buff[4] == 0x1C) {
				send_join = true;
			}
//			else if (resp_buff[4] == 0x18) {
//				if (resp_buff[5] == 0x00)
//					send_join = true;
//			}
		}
		else if (resp_spi_byte == 0x00) {
			spi_write(NCP_SPI, ff_buff, pcs, 0);
			spi_read(NCP_SPI, &resp, &pcs);
			resp_buff[frame_index++] = (uint8_t) resp;

			spi_write(NCP_SPI, ff_buff, pcs, 1);
			spi_read(NCP_SPI, &resp, &pcs);
			resp_buff[frame_index++] = (uint8_t) resp;
		}
		else {
			spi_write(NCP_SPI, ff_buff, pcs, 0);
			spi_read(NCP_SPI, &resp, &pcs);
			resp_buff[frame_index++] = (uint8_t) resp;
		}

		spi_set_peripheral_chip_select_value(NCP_SPI, NCP_NONE_CHIP_SELECT_VALUE);

		is_awaiting_resp = false;
	}
}

void init_pins_temp()
{
	// enable host int
	pio_configure_pin(PIO_PA16_IDX, PIO_PA16);
	pio_pull_down(PIOA, PIO_PA16, true);
	pio_handler_set(PIOA, ID_PIOA, PIO_PA16, PIO_IT_FALL_EDGE, nHost_INT_handler);
	NVIC_EnableIRQ((IRQn_Type) ID_PIOA);
	pio_enable_interrupt(PIOA, PIO_PA16);

	// NCP Reset pin confgure
	pio_configure(PINS_RESET_PIO, PINS_RESET_TYPE, PINS_RESET_MASK, PINS_RESET_ATTR);

	pmc_enable_periph_clk(ID_PIOA);
}

#ifdef TEST

int main(void)
{
	const uint8_t *buffer;
	uint32_t length;
	uint8_t header[2];                                       // buffer to compose header in
	int cnt = 2;
	int ff_cnt = 6;

	header[0] = 0x0A;
	header[1] = 0xA7;
	sysclk_init();
	board_init();

	configure_console();

	spi_peripheral_init();
	init_pins_temp();
	//init_flash();

	pio_configure(PINS_RESET_PIO, PINS_RESET_TYPE, PINS_RESET_MASK, PINS_RESET_ATTR);

	/* Output demo infomation. */
	printf("-- Freertos Example --\n\r");
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);
	printf("-- Press M to toggle Task Monitor\r\n");

	/* Create task to monitor processor activity */
	if (xTaskCreate(task_monitor, "Monitor", TASK_MONITOR_STACK_SIZE, NULL,TASK_MONITOR_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Monitor task\r\n");
	}

	/* Create task to make led blink */
	//task_led_start();
	//task_wan_start();
	//task_ble_serial_start();
	//task_ble_dispatch_start();
	/* Create task for cheap CLI */
	if (xTaskCreate(task_cli, "CLI", TASK_CLI_STACK_SIZE, NULL, TASK_CLI_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create CLI task\r\n");
	}

	/* Start the scheduler. */
	//vTaskStartScheduler();
	/* Will only get here if there was insufficient memory to create the idle task. */

	version_command.header.sequence_byte = 0x00;
	version_command.header.control_byte = 0x00;
	version_command.header.frame_id = 0x00;
	version_command.desired_protocol_version = 0x04;

	delay_ms(1000);
	ncp_init_procedure();
	delay_ms(5000);
	send_ezsp_msg(0xFE, sizeof(version_command_t), &version_command);
	while (1) {
		//status_msg();

//		while (pio_get_pin_value(PIO_PA16_IDX))
//			;
//		spi_wake();
		//ex1_msg();
		//delay_ms(1000);

	}
	return 0;
}
#endif

enum ezsp_frame_id
{
	SPI_BYTE = 0xFE,
	PROTOCOL_VERSION = 0x00,
	INITIAL_SECURITY_STATE = 0x68,
	NETWORK_INIT = 0x17,
	GET_NETWORK_PARAMS = 0x28,
	FORM_NETWORK = 0x1E,
	PERMIT_JOINING = 0x22,
	JOIN_NETWORK = 0x1F,
	NETWORK_STATE = 0x18,
	START_SCAN = 0x1A,
	STOP_SCAN = 0x1D,
	CALLBACK_CMD = 0x06

};

int main(void)
{
	sysclk_init();
	board_init();

	configure_console();

	init_pins_temp();

	/* Output demo infomation. */
	printf("\r\n-- Decawave SPI TEST --\n\r");

	spi_set_master_mode(NCP_SPI);

	pio_configure_pin(PIO_PA12_IDX, PIO_PERIPH_A); // MISO
	pio_configure_pin(PIO_PA13_IDX, PIO_PERIPH_A); // MOSI
	pio_configure_pin(PIO_PA14_IDX, PIO_PERIPH_A); // SPCK
	pio_configure_pin(NCP_CSn_PIO_IDX, NCP_CSn_PIO_PERIPH);
	pmc_enable_periph_clk(ID_SPI);

	spi_disable(NCP_SPI);
	spi_set_clock_polarity(NCP_SPI, NCP_CHIP_SELECT, NCP_CLOCK_POLARITY);
	spi_set_clock_phase(NCP_SPI, NCP_CHIP_SELECT, NCP_CLOCK_PHASE);
	spi_set_baudrate_div(NCP_SPI, NCP_CHIP_SELECT, (sysclk_get_cpu_hz() / NCP_SPI_BAUD_SLOW));
	spi_set_transfer_delay(NCP_SPI, NCP_CHIP_SELECT, NCP_DELAY_BEFORE, NCP_DELAY_BETWEEN);
	spi_configure_cs_behavior(NCP_SPI, NCP_CHIP_SELECT, SPI_CS_KEEP_LOW);
	//spi_set_bits_per_transfer(NCP_SPI, NCP_CHIP_SELECT, 0);
	spi_set_peripheral_chip_select_value(NCP_SPI, NCP_CHIP_SELECT_VALUE);

	spi_enable(NCP_SPI);

	uint8_t ext_panid[] = { 0x03, 0x05, 0x09, 0x03, 0x09, 0x02, 0x19, 0x89 };

	version_command.header.sequence_byte = 0x00;
	version_command.header.control_byte = 0x00;
	version_command.header.frame_id = PROTOCOL_VERSION;
	version_command.desired_protocol_version = 0x04;

	initial_security_state.header.sequence_byte = 0x00;
	initial_security_state.header.control_byte = 0x00;
	initial_security_state.header.frame_id = INITIAL_SECURITY_STATE;
#ifdef COORDINATOR
	initial_security_state.bitmask = 0x0208; //0x0008;
	initial_security_state.preconfigured_key = 0x0030;
	initial_security_state.network_key = 0x0040;
	initial_security_state.sequence_number = 0x00;
	initial_security_state.trust_center_eui64 = 0x00000000;
#else
	initial_security_state.bitmask = 0x0B00; //0x0100;
	initial_security_state.preconfigured_key = 0xAAD1;
	initial_security_state.network_key = 0xA7B7;
	initial_security_state.sequence_number = 0x00;
	initial_security_state.trust_center_eui64 = 0x00000000;
#endif

//	set_channel_cmd.header.sequence_byte = 0x00;
//	set_channel_cmd.header.control_byte = 0x00;
//	set_channel_cmd.header.frame_id = 0x8A;
//	set_channel_cmd.channel = 0x10; // channel 16;

	network_init_cmd.header.sequence_byte = 0x00;
	network_init_cmd.header.control_byte = 0x00;
	network_init_cmd.header.frame_id = NETWORK_INIT;

	get_network_params_cmd.header.sequence_byte = 0x00;
	get_network_params_cmd.header.control_byte = 0x00;
	get_network_params_cmd.header.frame_id = GET_NETWORK_PARAMS;

	form_network.header.sequence_byte = 0x00;
	form_network.header.control_byte = 0x00;
	form_network.header.frame_id = FORM_NETWORK;
	memcpy(form_network.net_params.extendedPanId, ext_panid, 8);
	form_network.net_params.panId = 0x1989;
	form_network.net_params.radioTxPower = 0x03;
	form_network.net_params.radioChannel = 0x0B;
	form_network.net_params.joinMethod = 0x00;
//	ember_network_parameters.nwkManagerId = 0x0000;
//	ember_network_parameters.nwkUpdateId = 0x00;
//	ember_network_parameters.channels = 0x00001000; // only scan for channel 16

	join_network.header.sequence_byte = 0x00;
	join_network.header.control_byte = 0x00;
	join_network.header.frame_id = JOIN_NETWORK;
	join_network.node_type = 0x02;
	memcpy(join_network.net_params.extendedPanId, ext_panid, 8);
	join_network.net_params.panId = 0x1989;
	join_network.net_params.radioTxPower = 0x03;
	join_network.net_params.radioChannel = 0x0B;
	join_network.net_params.joinMethod = 0x00;

	permit_joining.header.sequence_byte = 0x00;
	permit_joining.header.control_byte = 0x00;
	permit_joining.header.frame_id = PERMIT_JOINING;
	permit_joining.duration = 0xFF;

	network_state.header.sequence_byte = 0x00;
	network_state.header.control_byte = 0x00;
	network_state.header.frame_id = NETWORK_STATE;

	start_scan.header.sequence_byte = 0x00;
	start_scan.header.control_byte = 0x00;
	start_scan.header.frame_id = START_SCAN;
	start_scan.scan_type = 0x01;
	start_scan.channel_mask = 0x00000800;
	start_scan.duration = 0x0B;

	stop_scan.header.sequence_byte = 0x00;
	stop_scan.header.control_byte = 0x00;
	stop_scan.header.frame_id = STOP_SCAN;

	callback_cmd.header.sequence_byte = 0x00;
	callback_cmd.header.control_byte = 0x00;
	callback_cmd.header.frame_id = CALLBACK_CMD;

	delay_ms(5000);
	ncp_init_procedure();
	delay_ms(10);
	send_ezsp_msg(SPI_BYTE, sizeof(version_command_t), &version_command);
	delay_ms(1000);

#ifdef COORDINATOR
//			send_ezsp_msg(0xFE, sizeof(network_init_cmd_t), &network_init_cmd);
//			delay_ms(10);
	send_ezsp_msg(0xFE, sizeof(initial_security_state_t), &initial_security_state);
	delay_ms(10);
	send_ezsp_msg(0xFE, sizeof(form_network_t), &form_network);
	delay_ms(10);
	send_ezsp_msg(0xFE, sizeof(permit_joining_t), &permit_joining);
	delay_ms(10);
#endif
#ifdef ROUTER
	send_ezsp_msg(SPI_BYTE, sizeof(start_scan_t), &start_scan);
	while (uWait)
		;
	send_ezsp_msg(SPI_BYTE, sizeof(callback_cmd_t), &callback_cmd);

#endif
	//send_ezsp_msg(0xFE, sizeof(get_network_params_cmd_t), &get_network_params_cmd);

	while (1) {
#ifdef ROUTER

		if (send_join) {
			// send join will trigger when the scan is complete. CB = 1C
			send_ezsp_msg(SPI_BYTE, sizeof(initial_security_state_t), &initial_security_state);
			send_ezsp_msg(SPI_BYTE, sizeof(join_network_t), &join_network);
			while (uWait)
					;
			join_sent = true;
			send_join = false;
		}
		else {
			send_ezsp_msg(SPI_BYTE, sizeof(callback_cmd_t), &callback_cmd);
			delay_ms(1000);
		}

//		if (join_sent == true && send_join == false) {
//			//send_ezsp_msg(SPI_BYTE, sizeof(network_state_t), &network_state);
//			send_ezsp_msg(SPI_BYTE, sizeof(callback_cmd_t), &callback_cmd);
//			delay_ms(1000);
//		}
//		else if (send_join) {
//			if (!join_sent) {
//				send_ezsp_msg(SPI_BYTE, sizeof(stop_scan_t), &stop_scan);
//				delay_ms(10);
//				send_ezsp_msg(SPI_BYTE, sizeof(initial_security_state_t), &initial_security_state);
//				delay_ms(10);
//			}
//			send_ezsp_msg(SPI_BYTE, sizeof(join_network_t), &join_network);
//			join_sent = true;
//			send_join = false;
//			while (uWait)
//					;
//			send_ezsp_msg(SPI_BYTE, sizeof(callback_cmd_t), &callback_cmd);
//		}
//		else {
//			send_ezsp_msg(SPI_BYTE, sizeof(callback_cmd_t), &callback_cmd);
//			delay_ms(1000);
//		}
#else
//		send_ezsp_msg(0xFE, sizeof(network_state_t), &network_state);
		send_ezsp_msg(SPI_BYTE, sizeof(callback_cmd_t), &callback_cmd);

		delay_ms(1000);
#endif
	}

}
