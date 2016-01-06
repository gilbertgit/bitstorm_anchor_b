#include "asf.h"
#include "conf_board.h"

#include "task_wan.h"
#include "task_ble_serial.h"
#include "led.h"
#include "flash.h"
#include "ncp_spi.h"

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

void ex1_msg()
{
	uint8_t pcs2 = 0;
	uint16_t data[7];
	data[0] = 0x0A;
	data[1] = 0xA7;

	// RESET NCP
	reset_ncp();

	// WAIT FOR nHOST_INT TO ASSERT
	while (pio_get_pin_value(PIO_PA16_IDX))
		;

	// ASSET CS
	spi_set_peripheral_chip_select_value(NCP_SPI, (~(1U << NCP_CHIP_SELECT)));

	// WRITE DATA
	spi_write(NCP_SPI, data[0], pcs2, 0);
	spi_write(NCP_SPI, data[1], pcs2, 0);

	status_code_t result = STATUS_OK;
	uint16_t buff;
	uint16_t ff_buff = 0xFF;

	do {
		buff = 0xFF;
		spi_write(NCP_SPI, buff, pcs2, 0);
		spi_read(NCP_SPI, &buff, &pcs2);
	}
	while ((buff & 0xFF) == 0xFF);
	spi_write(NCP_SPI, ff_buff, pcs2, 1);

	spi_set_peripheral_chip_select_value(NCP_SPI, NCP_NONE_CHIP_SELECT_VALUE);

}

void status_msg()
{
	uint8_t pcs2 = 0;
	uint16_t buff;
	uint16_t ff_buff = 0xFF;

	// SEND EZSP STATUS REQUEST
	uint16_t data_status[2];
	data_status[0] = 0x0B;
	data_status[1] = 0xA7;

	// ASSET CS
	spi_set_peripheral_chip_select_value(NCP_SPI, (~(1U << NCP_CHIP_SELECT)));

	// WRITE DATA WITHOUT RESET
	spi_write(NCP_SPI, data_status[0], pcs2, 0);
	spi_write(NCP_SPI, data_status[1], pcs2, 0);

	//while (pio_get_pin_value(PIO_PA16_IDX));

	do {
		buff = 0xFF;
		spi_write(NCP_SPI, buff, pcs2, 0);
		spi_read(NCP_SPI, &buff, &pcs2);
	}
	while ((buff & 0xFF) == 0xFF);
	spi_write(NCP_SPI, ff_buff, pcs2, 1);

	spi_set_peripheral_chip_select_value(NCP_SPI, NCP_NONE_CHIP_SELECT_VALUE);
}

void ncp_init_procedure()
{
	uint8_t pcs2 = 0;
	uint16_t data[2];
	data[0] = 0x0A;
	data[1] = 0xA7;

	// RESET NCP
	reset_ncp();

	// WAIT FOR nHOST_INT TO ASSERT
	while (pio_get_pin_value(PIO_PA16_IDX))
		;

	// ASSET CS
	spi_set_peripheral_chip_select_value(NCP_SPI, (~(1U << NCP_CHIP_SELECT)));

	// WRITE DATA
	spi_write(NCP_SPI, data[0], pcs2, 0);
	spi_write(NCP_SPI, data[1], pcs2, 0);

	status_code_t result = STATUS_OK;
	uint16_t buff;
	uint16_t ff_buff = 0xFF;

	do {
		buff = 0xFF;
		spi_write(NCP_SPI, buff, pcs2, 0);
		spi_read(NCP_SPI, &buff, &pcs2);
	}
	while ((buff & 0xFF) == 0xFF);
	spi_write(NCP_SPI, ff_buff, pcs2, 0);
	spi_write(NCP_SPI, ff_buff, pcs2, 1);

	spi_set_peripheral_chip_select_value(NCP_SPI, NCP_NONE_CHIP_SELECT_VALUE);

	delay_ms(5);

	// DEASSERT CS
	spi_set_peripheral_chip_select_value(NCP_SPI, (~(1U << NCP_CHIP_SELECT)));

	// WRITE DATA WITHOUT RESET
	spi_write(NCP_SPI, data[0], pcs2, 0);
	spi_write(NCP_SPI, data[1], pcs2, 0);

	do {
		buff = 0xFF;
		spi_write(NCP_SPI, buff, pcs2, 0);
		spi_read(NCP_SPI, &buff, &pcs2);
	}
	while ((buff & 0xFF) == 0xFF);
	//spi_write(NCP_SPI, ff_buff, pcs2, 0);
	//spi_write(NCP_SPI, ff_buff, pcs2, 1);

	spi_set_peripheral_chip_select_value(NCP_SPI, NCP_NONE_CHIP_SELECT_VALUE);

	delay_ms(5);

	// SEND EZSP STATUS REQUEST
	uint16_t data_status[2];
	data_status[0] = 0x0B;
	data_status[1] = 0xA7;

	// ASSERT CS
	spi_set_peripheral_chip_select_value(NCP_SPI, (~(1U << NCP_CHIP_SELECT)));

	// WRITE DATA WITHOUT RESET
	spi_write(NCP_SPI, data_status[0], pcs2, 0);
	spi_write(NCP_SPI, data_status[1], pcs2, 0);

	do {
		buff = 0xFF;
		spi_write(NCP_SPI, buff, pcs2, 0);
		spi_read(NCP_SPI, &buff, &pcs2);
	}
	while ((buff & 0xFF) == 0xFF);
//	spi_write(NCP_SPI, ff_buff, pcs2, 0);
//	spi_write(NCP_SPI, ff_buff, pcs2, 1);
	spi_set_peripheral_chip_select_value(NCP_SPI, NCP_NONE_CHIP_SELECT_VALUE);

}

void send_msg()
{
	uint8_t pcs = 0;
	uint16_t data[7];
	data[0] = 0xFE;
	data[1] = 0x04;
	data[2] = 0x00;
	data[3] = 0x00;
	data[4] = 0x00;
	data[5] = 0x04;
	data[6] = 0xA7;

	spi_set_peripheral_chip_select_value(NCP_SPI, (~(1U << NCP_CHIP_SELECT)));

	spi_write(NCP_SPI, data[0], pcs, 0);
	spi_write(NCP_SPI, data[1], pcs, 0);
	spi_write(NCP_SPI, data[2], pcs, 0);
	spi_write(NCP_SPI, data[3], pcs, 0);
	spi_write(NCP_SPI, data[4], pcs, 0);
	spi_write(NCP_SPI, data[5], pcs, 0);
	spi_write(NCP_SPI, data[6], pcs, 0);
	//spi_write(NCP_SPI, 0xFF, pcs, 1);

	//spi_set_peripheral_chip_select_value(NCP_SPI, NCP_NONE_CHIP_SELECT_VALUE);
	//spi_set_lastxfer(NCP_SPI);
}

static int spi_wake()
{
	status_code_t result = STATUS_OK;
	uint16_t buff;
	uint16_t ff_buff = 0xFF;

	uint8_t pcs = 0;

	//spi_set_peripheral_chip_select_value(NCP_SPI, (~(1U << NCP_CHIP_SELECT)));

	//NOTE: Requires a change in ASF FreeRTOS SPI code freertos_spi_master.c line 423
	//      This is because by default, the ASF code was blasting the buffer with 0xff for the read,
	//      but in this case, we need the "dummy" bytes of the header to remain.  So I just commented out
	//      the memset() line in the ASF code.

	do {
		buff = 0xFF;
//	if (freertos_spi_read_packet(NCP_SPI, &buff, 1, NCP_SPI_MAX_BLOCK_TIME) != STATUS_OK) {
//		result = STATUS_ERR_TIMEOUT;
//	}
		spi_write(NCP_SPI, buff, pcs, 0);
		spi_read(NCP_SPI, &buff, &pcs);
	}
	while ((buff & 0xFF) == 0xFF);

	for (int i = 0; i < 10; i++) {
		spi_write(NCP_SPI, ff_buff, pcs, 0);

	}
	spi_write(NCP_SPI, ff_buff, pcs, 1);
	//spi_write(NCP_SPI, ff_buff, pcs, 1);
	//spi_configure_cs_behavior(NCP_SPI, NCP_CHIP_SELECT, SPI_CS_RISE_NO_TX);
	spi_set_peripheral_chip_select_value(NCP_SPI, NCP_NONE_CHIP_SELECT_VALUE);
	//spi_set_lastxfer(NCP_SPI);
	spi_write(NCP_SPI, ff_buff, pcs, 0);
	if (result != STATUS_OK) {
		printf("readfromspi_serial timeout\r\n");
		for (;;) {
		}
	}

	return result;
}

void nHost_INT_handler()
{
	spi_wake();
}

void init_pins_temp()
{
	// enable host int

	pio_configure_pin(PIO_PA16_IDX, PIO_PA16);
	//pio_set_input(PIOA, PIO_PA16);
	pio_pull_down(PIOA, PIO_PA16, true);
	pio_handler_set(PIOA, ID_PIOA, PIO_PA16, PIO_IT_FALL_EDGE, nHost_INT_handler);
	pio_enable_interrupt(PIOA, PIO_PA16);

	pmc_enable_periph_clk(ID_PIOA);
}

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

	delay_ms(5000);
	ncp_init_procedure();
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
