#include "asf.h"
#include "conf_board.h"

#include "task_wan.h"
#include "task_ble_serial.h"
#include "led.h"
#include "flash.h"
#include "ncp_spi.h"
#include "string.h"

#define TASK_MONITOR_STACK_SIZE            (2048/sizeof(portSTACK_TYPE))
#define TASK_MONITOR_STACK_PRIORITY        (tskIDLE_PRIORITY)
#define TASK_CLI_STACK_SIZE                (2048/sizeof(portSTACK_TYPE))
#define TASK_CLI_STACK_PRIORITY            (tskIDLE_PRIORITY)

static volatile bool bMonitorPaused = true;
uint32_t my_tick_count;

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

#define RESET_IDX		PIO_PA15_IDX

#define PINS_RESET_PIO		PIOA
#define PINS_RESET_TYPE		PIO_OUTPUT_1
#define PINS_RESET_MASK		PIO_PA15
#define PINS_RESET_ATTR		PIO_DEFAULT

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

typedef struct
{
	uint8_t sequence_byte;
	uint8_t control_byte;
	uint8_t frame_id;

} ezsp_header_t;
ezsp_header_t ezsp_header;
typedef struct
{
	ezsp_header_t header;
	uint8_t desired_protocol_version;
} version_command_t;
version_command_t version_command;

typedef struct
{
	ezsp_header_t header;
	uint16_t bitmask;
	uint16_t preconfigured_key;
	uint16_t network_key;
	uint8_t sequence_number;
	uint64_t trust_center_eui64;
} initial_security_state_t;
initial_security_state_t initial_security_state;

typedef struct
{
	ezsp_header_t header;
} network_init_cmd_t;
network_init_cmd_t network_init_cmd;

typedef struct
{
	ezsp_header_t header;
	uint8_t channel;
} set_channel_cmd_t;
set_channel_cmd_t set_channel_cmd;

typedef struct
{
	ezsp_header_t header;
} get_network_params_cmd_t;
get_network_params_cmd_t get_network_params_cmd;

#define PACKED __attribute__((packed))

typedef struct
	PACKED {
		ezsp_header_t header;
		uint8_t extendedPanId[8];
		uint16_t panId;
		uint8_t radioTxPower;
		uint8_t radioChannel;
		uint8_t joinMethod;
//	uint16_t nwkManagerId;
//	uint8_t nwkUpdateId;
//	uint32_t channels;
	} ember_network_parameters_t;
	ember_network_parameters_t ember_network_parameters;

	void send_ezsp_msg(uint8_t spi_byte, uint8_t length_byte, void* payload)
	{
		uint8_t pcs = 0;
		uint16_t buff;
		uint16_t eof = 0xA7;
		uint16_t resp_spi_byte;
		uint16_t resp_length;
		uint8_t resp_buff[20];

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

//	while(pio_get_pin_value(PIO_PA16_IDX) == 1);
		//spi_set_clock_phase(NCP_SPI, NCP_CHIP_SELECT, 1);

// WRITE 0xFF UNTILL WE GET SOMETHING OTHER THAN 0xFF
		do {
			buff = 0xFF;
			spi_write(NCP_SPI, buff, pcs, 0);
			spi_read(NCP_SPI, &buff, &pcs);
		}
		while ((buff & 0xFF) == 0xFF);
		resp_spi_byte = buff; // FIRST BYTE IS RESPONCE SPI BYTE
		buff = 0xFF;

		// GET THE MESSAGE LENGTH
		spi_write(NCP_SPI, buff, pcs, 0);
		spi_read(NCP_SPI, &buff, &pcs);
		resp_length = buff;
		buff = 0xFF;
		int i = 0;
		for (; i < resp_length; i++) {
			spi_write(NCP_SPI, buff, pcs, 0);
			spi_read(NCP_SPI, &data, &pcs);

			resp_buff[i] = (uint8_t) data;
		}

		spi_write(NCP_SPI, 0xFF, pcs, 1);
		spi_read(NCP_SPI, &data, &pcs);

		resp_buff[i] = (uint8_t) data;

		// DEASSERT CS
		spi_set_peripheral_chip_select_value(NCP_SPI, NCP_NONE_CHIP_SELECT_VALUE);

		//spi_set_clock_phase(NCP_SPI, NCP_CHIP_SELECT, NCP_CLOCK_PHASE);

		printf("\r\nEZSP-SPI-Response\r\n");
		printf("0x%02x, ", resp_spi_byte);
		printf("0x%02x, ", resp_length);

		for (int i = 0; i < resp_length + 1; i++) {
			printf("0x%02x, ", resp_buff[i]);
		}
	}

	void ncp_init_procedure()
	{
		uint8_t pcs2 = 0;
		uint16_t data[2];
		data[0] = 0x0A;
		data[1] = 0xA7;
		uint8_t resp_buff2[20];
		uint16_t resp;
		int i = 0;

		// RESET NCP
		reset_ncp();

		// WAIT FOR nHOST_INT TO ASSERT
		while (pio_get_pin_value(PIO_PA16_IDX))
			;

		// ASSET CS
		spi_set_peripheral_chip_select_value(NCP_SPI, (~(1U << NCP_CHIP_SELECT)));

		// WRITE DATA
		spi_write(NCP_SPI, data[0], pcs2, 0);
		spi_read(NCP_SPI, &resp, &pcs2);

		spi_write(NCP_SPI, data[1], pcs2, 0);
		spi_read(NCP_SPI, &resp, &pcs2);

		status_code_t result = STATUS_OK;
		uint16_t buff;
		uint16_t ff_buff = 0xFF;

		//spi_set_clock_phase(NCP_SPI, NCP_CHIP_SELECT, 1);

		do {
			buff = 0xFF;
			spi_write(NCP_SPI, buff, pcs2, 0);
			spi_read(NCP_SPI, &buff, &pcs2);
		}
		while ((buff & 0xFF) == 0xFF);
		resp_buff2[i++] = buff;

		spi_write(NCP_SPI, ff_buff, pcs2, 0);
		spi_read(NCP_SPI, &resp, &pcs2);
		resp_buff2[i++] = (uint8_t) resp;

		spi_write(NCP_SPI, ff_buff, pcs2, 1);
		spi_read(NCP_SPI, &resp, &pcs2);
		resp_buff2[i++] = (uint8_t) resp;

		// DEASSERT CS
		spi_set_peripheral_chip_select_value(NCP_SPI, NCP_NONE_CHIP_SELECT_VALUE);

		//spi_set_clock_phase(NCP_SPI, NCP_CHIP_SELECT, NCP_CLOCK_PHASE);

		printf("\r\nCMD After Reset\r\n");
		for (int j = 0; j < i; j++) {
			printf("0x%02x, ", resp_buff2[j]);
		}
		i = 0;
//////////////////////////////////////////////////////////////////////////////////
		delay_ms(5);

		// ASSERT CS
		spi_set_peripheral_chip_select_value(NCP_SPI, (~(1U << NCP_CHIP_SELECT)));

		// WRITE DATA WITHOUT RESET
		spi_write(NCP_SPI, data[0], pcs2, 0);
		spi_read(NCP_SPI, &resp, &pcs2);
		spi_write(NCP_SPI, data[1], pcs2, 0);
		spi_read(NCP_SPI, &resp, &pcs2);

		//spi_set_clock_phase(NCP_SPI, NCP_CHIP_SELECT, 1);

		do {
			buff = 0xFF;
			spi_write(NCP_SPI, buff, pcs2, 0);
			spi_read(NCP_SPI, &buff, &pcs2);
		}
		while ((buff & 0xFF) == 0xFF);
		resp_buff2[i++] = buff;

		//spi_write(NCP_SPI, ff_buff, pcs2, 0);
		//spi_write(NCP_SPI, ff_buff, pcs2, 1);

		spi_write(NCP_SPI, ff_buff, pcs2, 0);
		spi_read(NCP_SPI, &resp, &pcs2);
		resp_buff2[i++] = (uint8_t) resp;

		spi_set_peripheral_chip_select_value(NCP_SPI, NCP_NONE_CHIP_SELECT_VALUE);

		//spi_set_clock_phase(NCP_SPI, NCP_CHIP_SELECT, NCP_CLOCK_PHASE);

		printf("\r\nCMD NO Reset\r\n");
		for (int j = 0; j < i; j++) {
			printf("0x%02x, ", resp_buff2[j]);
		}
		i = 0;
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

		//spi_set_clock_phase(NCP_SPI, NCP_CHIP_SELECT, 1);

		do {
			buff = 0xFF;
			spi_write(NCP_SPI, buff, pcs2, 0);
			spi_read(NCP_SPI, &buff, &pcs2);
		}
		while ((buff & 0xFF) == 0xFF);
		resp_buff2[i++] = buff;

		spi_write(NCP_SPI, ff_buff, pcs2, 0);
		spi_read(NCP_SPI, &resp, &pcs2);
		resp_buff2[i++] = (uint8_t) resp;
//	spi_write(NCP_SPI, ff_buff, pcs2, 0);
//	spi_write(NCP_SPI, ff_buff, pcs2, 1);
		spi_set_peripheral_chip_select_value(NCP_SPI, NCP_NONE_CHIP_SELECT_VALUE);

		//spi_set_clock_phase(NCP_SPI, NCP_CHIP_SELECT, NCP_CLOCK_PHASE);

		printf("\r\nStatus CMD \r\n");
		for (int j = 0; j < i; j++) {
			printf("0x%02x, ", resp_buff2[j]);
		}
		i = 0;
	}

	void nHost_INT_handler()
	{
		//spi_wake();
	}

	void init_pins_temp()
	{
		// enable host int

		pio_configure_pin(PIO_PA16_IDX, PIO_PA16);
		//pio_set_input(PIOA, PIO_PA16);
		pio_pull_down(PIOA, PIO_PA16, true);
		pio_handler_set(PIOA, ID_PIOA, PIO_PA16, PIO_IT_FALL_EDGE, nHost_INT_handler);
		pio_enable_interrupt(PIOA, PIO_PA16);
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
	uint64_t u8tou64(uint8_t const u8[static 8])
	{
		uint64_t u64;
		memcpy(&u64, u8, sizeof u64);
		return u64;
	}
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
		spi_configure_cs_behavior(NCP_SPI, NCP_CHIP_SELECT, SPI_CS_RISE_NO_TX);
		spi_set_peripheral_chip_select_value(NCP_SPI, NCP_CHIP_SELECT_VALUE);

		spi_enable(NCP_SPI);

//	uint8_t pcs = NCP_CHIP_SELECT;
//	uint16_t data = 0;
//	uint8_t buff[5];
//
//	spi_set_peripheral_chip_select_value(NCP_SPI, (~(1U << NCP_CHIP_SELECT)));
//
//	spi_write(NCP_SPI, 0, pcs, 0);
//	spi_read(NCP_SPI, &data, &pcs);
//	buff[0] = data & 0xFF;
//
//	spi_write(NCP_SPI, 0, pcs, 0);
//	spi_read(NCP_SPI, &data, &pcs);
//	buff[1] = data & 0xFF;
//
//	spi_write(NCP_SPI, 0, pcs, 0);
//	spi_read(NCP_SPI, &data, &pcs);
//	buff[2] = data & 0xFF;
//
//	spi_write(NCP_SPI, 0, pcs, 0);
//	spi_read(NCP_SPI, &data, &pcs);
//	buff[3] = data & 0xFF;
//
//	spi_write(NCP_SPI, 0, pcs, 0);
//	spi_read(NCP_SPI, &data, &pcs);
//	buff[4] = data & 0xFF;
//
//	spi_set_peripheral_chip_select_value(NCP_SPI, NCP_NONE_CHIP_SELECT_VALUE);
//
//	printf("returned %02X%02X%02X%02X\r\n", buff[1], buff[2], buff[3], buff[4]);

		uint8_t ext_panid[] = { 0x03, 0x05, 0x09, 0x03, 0x09, 0x02, 0x19, 0x89 };

		version_command.header.sequence_byte = 0x00;
		version_command.header.control_byte = 0x00;
		version_command.header.frame_id = 0x00;
		version_command.desired_protocol_version = 0x04;

		initial_security_state.header.sequence_byte = 0x00;
		initial_security_state.header.control_byte = 0x00;
		initial_security_state.header.frame_id = 0x68;
		initial_security_state.bitmask = 0x0000;
		initial_security_state.preconfigured_key = 0x0000;
		initial_security_state.network_key = 0x0000;
		initial_security_state.sequence_number = 0x00;
		initial_security_state.trust_center_eui64 = 0x00000000;

//	set_channel_cmd.header.sequence_byte = 0x00;
//	set_channel_cmd.header.control_byte = 0x00;
//	set_channel_cmd.header.frame_id = 0x8A;
//	set_channel_cmd.channel = 0x10; // channel 16;

		network_init_cmd.header.sequence_byte = 0x00;
		network_init_cmd.header.control_byte = 0x00;
		network_init_cmd.header.frame_id = 0x17;

		get_network_params_cmd.header.sequence_byte = 0x00;
		get_network_params_cmd.header.control_byte = 0x00;
		get_network_params_cmd.header.frame_id = 0x28;

		ember_network_parameters.header.sequence_byte = 0x00;
		ember_network_parameters.header.control_byte = 0x00;
		ember_network_parameters.header.frame_id = 0x1E;
		//ember_network_parameters.extendedPanId = u8tou64(ext_panid);
		memcpy(ember_network_parameters.extendedPanId, ext_panid, 8);
		ember_network_parameters.panId = 0x1989;
		ember_network_parameters.radioTxPower = 0x03;
		ember_network_parameters.radioChannel = 0x0B; // channel 16
		ember_network_parameters.joinMethod = 0x00;
//	ember_network_parameters.nwkManagerId = 0x0000;
//	ember_network_parameters.nwkUpdateId = 0x00;
//	ember_network_parameters.channels = 0x00001000; // only scan for channel 16

		delay_ms(1000);
		ncp_init_procedure();
		delay_ms(10);
		send_ezsp_msg(0xFE, sizeof(version_command_t), &version_command);
		delay_ms(10);
		send_ezsp_msg(0xFE, sizeof(initial_security_state_t), &initial_security_state);
		delay_ms(5000);
//	send_ezsp_msg(0xFE, sizeof(network_init_cmd_t), &network_init_cmd);
//	delay_ms(10);
//	send_ezsp_msg(0xFE, sizeof(set_channel_cmd_t), &set_channel_cmd);
//	delay_ms(10);
		send_ezsp_msg(0xFE, sizeof(ember_network_parameters_t), &ember_network_parameters);
		delay_ms(10);
		send_ezsp_msg(0xFE, sizeof(get_network_params_cmd_t), &get_network_params_cmd);

//	delay_ms(10);
//	send_ezsp_msg(0xFE, sizeof(version_command_t), &version_command);
		while (1)
			;

	}
