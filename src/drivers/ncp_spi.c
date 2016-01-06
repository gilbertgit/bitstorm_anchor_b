/*
 * spi.c
 *
 *  Created on: Dec 21, 2015
 *      Author: titan
 */

#include "asf.h"
#include "ncp_spi.h"

void * SPI_Configuration(void) {

	freertos_spi_if spi_if;

	const freertos_peripheral_options_t driver_options = {
	/* No receive buffer pointer needed for SPI */
	NULL,

	/* No receive buffer size needed for SPI */
	0,

	/* Cortex-M4 priority */
	configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,

	/* Operation mode - MASTER */
	SPI_MASTER,

	/* Blocking (not async) */
	(USE_TX_ACCESS_MUTEX | USE_RX_ACCESS_MUTEX | WAIT_TX_COMPLETE | WAIT_RX_COMPLETE) };

	spi_if = freertos_spi_master_init(NCP_SPI, &driver_options);

	if (spi_if != NULL) {
		pio_configure_pin(PIO_PA12_IDX, PIO_PERIPH_A);	// MISO
		pio_configure_pin(PIO_PA13_IDX, PIO_PERIPH_A);	// MOSI
		pio_configure_pin(PIO_PA14_IDX, PIO_PERIPH_A);	// SPCK
//		pio_configure_pin(PIO_PC4_IDX, PIO_PERIPH_B);	// NPCS1
		pio_configure_pin(NCP_CSn_PIO_IDX, NCP_CSn_PIO_PERIPH);
		pmc_enable_periph_clk(ID_SPI);

		spi_disable(NCP_SPI);
		spi_set_clock_polarity(NCP_SPI, NCP_CHIP_SELECT, NCP_CLOCK_POLARITY);
		spi_set_clock_phase(NCP_SPI, NCP_CHIP_SELECT, NCP_CLOCK_PHASE);
		spi_set_baudrate_div(NCP_SPI, NCP_CHIP_SELECT, (sysclk_get_cpu_hz() / NCP_SPI_BAUD_SLOW));
		//spi_set_baudrate_div(NCP_SPI, NCP_CHIP_SELECT, 20);
		spi_set_transfer_delay(NCP_SPI, NCP_CHIP_SELECT, NCP_DELAY_BEFORE,		NCP_DELAY_BETWEEN);
		spi_configure_cs_behavior(NCP_SPI, NCP_CHIP_SELECT, SPI_CS_KEEP_LOW);

		//spi_configure_cs_behavior(NCP_SPI, NCP_CHIP_SELECT, SPI_CS_RISE_NO_TX);
		spi_set_peripheral_chip_select_value(NCP_SPI, NCP_CHIP_SELECT_VALUE);

		spi_enable(NCP_SPI);
	}

	return spi_if;
}

#include <string.h>
static uint8_t spi_buffer[1024];

int writetospi_serial(uint16_t headerLength, const uint8_t *headerBuffer, uint32_t bodylength, const uint8_t *bodyBuffer) {
	status_code_t result = STATUS_OK;

	memcpy(spi_buffer, headerBuffer, headerLength);
	memcpy(&spi_buffer[headerLength], bodyBuffer, bodylength);

	spi_set_peripheral_chip_select_value(NCP_SPI, (~(1U << NCP_CHIP_SELECT)));

	if (freertos_spi_write_packet(NCP_SPI, spi_buffer, bodylength + headerLength, NCP_SPI_MAX_BLOCK_TIME) != STATUS_OK) {
		result = STATUS_ERR_TIMEOUT;
	}

	spi_set_peripheral_chip_select_value(NCP_SPI, NCP_NONE_CHIP_SELECT_VALUE);
	spi_set_lastxfer(NCP_SPI);

	if (result != STATUS_OK) {
		printf("writetospi_serial timeout\r\n");
		for (;;) {
		}
	}

	return result;
}

int readfromspi_serial(uint16_t headerLength, const uint8_t *headerBuffer, uint32_t readlength, uint8_t *readBuffer) {
	status_code_t result = STATUS_OK;

	memcpy(spi_buffer, headerBuffer, headerLength);
	memset(&spi_buffer[headerLength],0xff,readlength);

	spi_set_peripheral_chip_select_value(NCP_SPI, (~(1U << NCP_CHIP_SELECT)));

	//NOTE: Requires a change in ASF FreeRTOS SPI code freertos_spi_master.c line 423
	//      This is because by default, the ASF code was blasting the buffer with 0xff for the read,
	//      but in this case, we need the "dummy" bytes of the header to remain.  So I just commented out
	//      the memset() line in the ASF code.

	if (freertos_spi_read_packet(NCP_SPI, spi_buffer, readlength + headerLength, NCP_SPI_MAX_BLOCK_TIME) != STATUS_OK) {
		result = STATUS_ERR_TIMEOUT;
	}
	else {
		memcpy(readBuffer, &spi_buffer[headerLength], readlength);
	}

	spi_set_peripheral_chip_select_value(NCP_SPI, NCP_NONE_CHIP_SELECT_VALUE);
	spi_set_lastxfer(NCP_SPI);

	if (result != STATUS_OK) {
		printf("readfromspi_serial timeout\r\n");
		for (;;) {
		}
	}

	return result;
}

void port_SPIx_clear_chip_select(void) {
	// Is this the best way to control the CS line?
	uint8_t dummy_buf[1];
	dummy_buf[0] = 0x00;
	spi_set_peripheral_chip_select_value(NCP_SPI, (~(1U << NCP_CHIP_SELECT)));
	freertos_spi_write_packet(NCP_SPI, dummy_buf, 1, NCP_SPI_MAX_BLOCK_TIME);
}

void port_SPIx_set_chip_select(void) {
	spi_set_peripheral_chip_select_value(NCP_SPI, NCP_NONE_CHIP_SELECT_VALUE);
	spi_set_lastxfer(NCP_SPI);
}

void * spi_peripheral_init() {
	return SPI_Configuration();
}
