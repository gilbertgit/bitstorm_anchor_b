/*
 * spi.h
 *
 *  Created on: Dec 21, 2015
 *      Author: titan
 */

#ifndef SPI_H_
#define SPI_H_

#include "spi.h"

extern int writetospi_serial(uint16_t headerLength, const uint8_t *headerBuffer, uint32_t bodylength, const uint8_t *bodyBuffer);
extern int readfromspi_serial(uint16_t headerLength, const uint8_t *headerBuffer, uint32_t readlength, uint8_t *readBuffer);

#define writetospi		writetospi_serial
#define readfromspi		readfromspi_serial

void * SPI_Configuration(void);


void * spi_peripheral_init(void);


#endif /* SPI_H_ */
