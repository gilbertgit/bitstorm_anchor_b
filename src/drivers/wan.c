/*
 * wan.c
 *
 *  Created on: Dec 8, 2015
 *      Author: titan
 */

#include "wan.h"

void init_wan(void)
{
	// RESET PIN CONFIG
	pio_configure(WAN_RESET_PIO, WAN_RESET_TYPE, WAN_RESET_MASK, WAN_RESET_ATTR);
	wan_reset();
}

void init_uart_wan(void)
{
	//sysclk_init();

	// set the pins to use the uart peripheral
	pio_configure(PINS_USART0_PIO_WAN, PINS_USART0_TYPE_WAN, PINS_USART0_MASK_WAN, PINS_USART0_ATTR_WAN);

	//enable the uart peripherial clock
	pmc_enable_periph_clk(ID_USART0);
	const sam_usart_opt_t usart_console_settings = {
	USART_SERIAL_BAUDRATE,
	USART_SERIAL_CHAR_LENGTH,
	USART_SERIAL_PARITY,
	USART_SERIAL_STOP_BIT,
	US_MR_CHMODE_NORMAL };

	pmc_enable_periph_clk(USART_SERIAL_ID_WAN);

	usart_init_rs232(USART_SERIAL_WAN, &usart_console_settings, sysclk_get_main_hz());
	usart_enable_tx(USART_SERIAL_WAN);
	usart_enable_rx(USART_SERIAL_WAN);

}

void wan_tx(uint8_t data)
{
	usart_putchar(USART0, data);
	//usart_write(USART0, data);
}

uint8_t wan_rx_handler(signed char *rx_byte)
{
	uint32_t dw_status = uart_get_status(UART1);

	if (dw_status & UART_SR_RXRDY) {

		uart_read(USART0, &rx_byte);
		return 1;
	}
	return 0;
}

void wan_reset()
{
	pio_toggle_pin(WAN_RESET_PIN);
}
