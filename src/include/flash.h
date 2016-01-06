/*
 * flash.h
 *
 *  Created on: Dec 11, 2015
 *      Author: titan
 */

#ifndef FLASH_H_
#define FLASH_H_

void init_flash();
uint8_t flash_read();
uint8_t write_flash(void* data, uint8_t size);


#endif /* FLASH_H_ */
