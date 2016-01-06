/*
 * ble.h
 *
 *  Created on: Dec 17, 2015
 *      Author: titan
 */
#include "asf.h"
#ifndef BLE_H_
#define BLE_H_

#define BLE_RESET_PIN			PIO_PA26_IDX
#define BLE_RESET_PIO			PIOA
#define BLE_RESET_TYPE		PIO_OUTPUT_0
#define BLE_RESET_MASK		PIO_PA26
#define BLE_RESET_ATTR		PIO_DEFAULT

#define BLE_CTS_PIN			PIO_PA25_IDX
#define BLE_CTS_PIO			PIOA
#define BLE_CTS_TYPE		PIO_OUTPUT_0
#define BLE_CTS_MASK		PIO_PA25
#define BLE_CTS_ATTR		PIO_DEFAULT

#define BLE_RTS_PIN			PIO_PA24_IDX
#define BLE_RTS_PIO			PIOA
#define BLE_RTS_TYPE		PIO_PERIPH_A
#define BLE_RTS_MASK		PIO_PA24
#define BLE_RTS_ATTR		PIO_DEFAULT


#define PINS_USART1_MASK_BLE     	(PIO_PA21A_RXD1 | PIO_PA22A_TXD1)
#define PINS_USART1_PIO_BLE      	PIOA
#define PINS_USART1_TYPE_BLE      	PIO_PERIPH_A
#define PINS_USART1_ATTR_BLE      	PIO_DEFAULT


#define USART_SERIAL_BLE                 USART1
#define USART_SERIAL_ID_BLE              ID_USART1
#define USART_SERIAL_PIO             PINS_USART_PIO
#define USART_SERIAL_TYPE            PINS_USART_TYPE
#define USART_SERIAL_PINS            PINS_USART_PINS
#define USART_SERIAL_MASK            PINS_USART_MASK
#define USART_SERIAL_BAUDRATE        38400
#define USART_SERIAL_CHAR_LENGTH     US_MR_CHRL_8_BIT
#define USART_SERIAL_PARITY          US_MR_PAR_NO
#define USART_SERIAL_STOP_BIT        US_MR_NBSTOP_1_BIT

void init_ble(void);
void init_uart_ble(void);
void kill_ble(void);
void ble_tx(uint8_t data);
uint8_t ble_rx_handler(signed char *rx_byte);
void ble_not_ready_to_rcv();
void ble_ready_to_rcv();
void ble_reset();


#endif /* BLE_H_ */
