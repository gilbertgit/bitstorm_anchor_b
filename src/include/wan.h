/*
 * wan.h
 *
 *  Created on: Dec 8, 2015
 *      Author: titan
 */

#ifndef WAN_H_
#define WAN_H_

#include "asf.h"
#include "wan_config.h"

#define PINS_USART0_MASK_WAN     	(PIO_PA5A_RXD0 | PIO_PA6A_TXD0)
#define PINS_USART0_PIO_WAN       	PIOA
#define PINS_USART0_TYPE_WAN      	PIO_PERIPH_A
#define PINS_USART0_ATTR_WAN      	PIO_DEFAULT


#define USART_SERIAL_WAN                 USART0
#define USART_SERIAL_ID_WAN              ID_USART0
#define USART_SERIAL_PIO             PINS_USART_PIO
#define USART_SERIAL_TYPE            PINS_USART_TYPE
#define USART_SERIAL_PINS            PINS_USART_PINS
#define USART_SERIAL_MASK            PINS_USART_MASK
#define USART_SERIAL_BAUDRATE        38400
#define USART_SERIAL_CHAR_LENGTH     US_MR_CHRL_8_BIT
#define USART_SERIAL_PARITY          US_MR_PAR_NO
#define USART_SERIAL_STOP_BIT        US_MR_NBSTOP_1_BIT

#define NWK_CONFIG_PIN				PIO_PA8_IDX

#define WAN_RESET_PIN				PIO_PA15_IDX
#define WAN_RESET_PIO		PIOA
#define WAN_RESET_TYPE		PIO_OUTPUT_0
#define WAN_RESET_MASK		PIO_PA15
#define WAN_RESET_ATTR		PIO_DEFAULT

void init_wan(void);
void init_uart_wan(void);
uint8_t wan_rx_handler(signed char *rx_byte);
void wan_tx(uint8_t data);
void wan_reset();



#endif /* WAN_H_ */
