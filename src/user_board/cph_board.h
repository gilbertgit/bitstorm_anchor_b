
#ifndef _CPH_BOARD_H
#define _CPH_BOARD_H

#include "compiler.h"
#include "system_sam4s.h"
#include "exceptions.h"
#include "pio.h"

// Decawave SPI
#define NCP_SPI_BAUD_FAST			5000000UL
#define NCP_SPI_BAUD_SLOW			1000000UL
#define NCP_SPI						SPI
#define NCP_SPI_MAX_BLOCK_TIME 		(50 / portTICK_RATE_MS)

#define NCP_CSn_PIO_IDX				PIO_PA11_IDX
#define NCP_CSn_PIO_PERIPH			PIO_PERIPH_A
#define NCP_CHIP_SELECT				0
#define NCP_CHIP_SELECT_VALUE		0x03
#define NCP_NONE_CHIP_SELECT_VALUE  0x0f

#define NCP_DELAY_BEFORE			0x00
#define NCP_DELAY_BETWEEN			0x10
#define NCP_CLOCK_POLARITY			0
#define NCP_CLOCK_PHASE				1

/** Board oscillator settings */
#define BOARD_FREQ_SLCK_XTAL        (32768U)
#define BOARD_FREQ_SLCK_BYPASS      (32768U)
#define BOARD_FREQ_MAINCK_XTAL      (12000000U)
#define BOARD_FREQ_MAINCK_BYPASS    (12000000U)

/** Master clock frequency */
#define BOARD_MCK                   CHIP_FREQ_CPU_MAX

/** board main clock xtal statup time */
#define BOARD_OSC_STARTUP_US   15625

/** Name of the board */
#define BOARD_NAME "BitStorm_Anchor_A"
/** Board definition */
#define cphBoard
/** Family definition (already defined) */
#define sam4s
/** Core definition */
#define cortexm4

/* Test page start address. */
#define FLASH_PAGE_ADDRESS (IFLASH0_ADDR + IFLASH0_SIZE - IFLASH0_PAGE_SIZE * 4)

/*----------------------------------------------------------------------------*/
/*	CONSOLE																	  */
/*----------------------------------------------------------------------------*/

#define CONSOLE_UART        UART0
#define CONSOLE_UART_ID     ID_UART0

#define PINS_UART0_PIO 		PIOA
#define PINS_UART0_TYPE 	PIO_PERIPH_A
#define PINS_UART0_MASK 	PIO_PA9A_URXD0|PIO_PA10A_UTXD0
#define PINS_UART0_ATTR 	PIO_DEFAULT


/*----------------------------------------------------------------------------*/
/*	LEDS																	  */
/*----------------------------------------------------------------------------*/

#define LED_STATUS_IDX		PIO_PB0_IDX

#define PINS_LED0_PIO		PIOB
#define PINS_LED0_TYPE		PIO_OUTPUT_0
#define PINS_LED0_MASK		PIO_PB0
#define PINS_LED0_ATTR		PIO_DEFAULT

#define LED_ALERT_IDX		PIO_PB1_IDX

#define PINS_LED1_PIO		PIOB
#define PINS_LED1_TYPE		PIO_OUTPUT_0
#define PINS_LED1_MASK		PIO_PB1
#define PINS_LED1_ATTR		PIO_DEFAULT




#endif  // _CPH_BOARD_H
