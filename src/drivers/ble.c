/*
 * ble.c
 *
 *  Created on: Dec 17, 2015
 *      Author: titan
 */

#include "ble.h"

void init_ble()
{
	pio_configure(BLE_CTS_PIN, BLE_CTS_TYPE, BLE_CTS_MASK, BLE_CTS_ATTR);
	pio_configure(BLE_RTS_PIN, BLE_RTS_TYPE, BLE_RTS_MASK, BLE_RTS_ATTR);
	pio_configure(BLE_RESET_PIN, BLE_RESET_TYPE, BLE_RESET_MASK, BLE_RESET_ATTR);

	ble_reset();

	pio_set_pin_low(BLE_CTS_PIN);
	//pio_clear(PIOA, BLE_CTS_MASK);
}

void init_uart_ble(void)
{
	//sysclk_init();

	// set the pins to use the uart peripheral
	pio_configure(PINS_USART1_PIO_BLE, PINS_USART1_TYPE_BLE, PINS_USART1_MASK_BLE, PINS_USART1_ATTR_BLE);

	//enable the uart peripherial clock
	pmc_enable_periph_clk(ID_USART1);
	const sam_usart_opt_t usart_console_settings = {
	USART_SERIAL_BAUDRATE,
	USART_SERIAL_CHAR_LENGTH,
	USART_SERIAL_PARITY,
	USART_SERIAL_STOP_BIT,
	US_MR_CHMODE_NORMAL };

	pmc_enable_periph_clk(USART_SERIAL_ID_BLE);

	//uart_init(UART1, &usart_console_settings);
	usart_init_rs232(USART_SERIAL_BLE, &usart_console_settings, sysclk_get_main_hz());
	usart_enable_tx(USART_SERIAL_BLE);
	usart_enable_rx(USART_SERIAL_BLE);

}

void ble_tx(uint8_t data)
{
	usart_putchar(USART1, data);
}

uint8_t ble_rx_handler(signed char *rx_byte)
{
	uint32_t dw_status = uart_get_status(USART1);

	if (dw_status & UART_SR_RXRDY) {

		uart_read(USART1, rx_byte);
		return 1;
	}
	return 0;
}

void ble_not_ready_to_rcv()
{
	pio_set_pin_high(BLE_CTS_PIN);
	//pio_set(PIOA, PIO_PA25);
}

void ble_ready_to_rcv()
{
	pio_set_pin_low(BLE_CTS_PIN);
	//pio_clear(PIOA, PIO_PA25);
}

void ble_reset()
{
	pio_toggle_pin(BLE_RESET_PIN);
}
