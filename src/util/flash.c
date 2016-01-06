/*
 * flash.c
 *
 *  Created on: Dec 9, 2015
 *      Author: titan
 */

#include "asf.h"
#include "cph_board.h"
#include "wan.h"

uint32_t ul_test_page_addr = FLASH_PAGE_ADDRESS;
uint8_t *pul_test_page;
uint32_t ul_rc;
uint32_t ul_idx;
uint8_t uc_key;
uint32_t ul_page_buffer[IFLASH0_PAGE_SIZE / sizeof(uint32_t)];

typedef unsigned long UL;

void init_flash()
{
	pul_test_page = (uint32_t *) ul_test_page_addr;

	/* Initialize flash: 6 wait states for flash writing. */
	ul_rc = flash_init(FLASH_ACCESS_MODE_128, 6);
	if (ul_rc != FLASH_RC_OK) {
		printf("-F- Initialization error %lu\n\r", (UL) ul_rc);
	}
}

uint8_t write_flash(void* data, uint32_t size)
{

	/* Unlock page */
	//printf("-I- This is a test");
//	printf("-I- Unlocking test page: 0x%08x\r\n", ul_test_page_addr);
	ul_rc = flash_unlock(ul_test_page_addr, ul_test_page_addr + IFLASH0_PAGE_SIZE - 1, 0, 0);
	if (ul_rc != FLASH_RC_OK) {
		printf("-F- Unlock error %lu\n\r", (UL) ul_rc);
		return 0;
	}

	/* The EWP command is not supported for non-8KByte sectors in all devices
	 *  SAM4 series, so an erase command is requried before the write operation.
	 */
	ul_rc = flash_erase_sector(ul_test_page_addr);
	if (ul_rc != FLASH_RC_OK) {
		printf("-F- Flash programming error %lu\n\r", (UL) ul_rc);
		return 0;
	}

	// white data to flash
	ul_rc = flash_write(ul_test_page_addr, data, size, 0);

	if (ul_rc != FLASH_RC_OK) {
		printf("-F- Flash programming error %lu\n\r", (UL) ul_rc);
		return 0;
	}

	/* Lock page */
	//printf("-I- Locking page\n\r");
	ul_rc = flash_lock(ul_test_page_addr, ul_test_page_addr + IFLASH0_PAGE_SIZE - 1, 0, 0);
	if (ul_rc != FLASH_RC_OK) {
		printf("-F- Flash locking error %lu\n\r", (UL) ul_rc);
		return 0;
	}

	return 1;
}

void * flash_read()
{
//	for (ul_idx = 0; ul_idx < (IFLASH0_PAGE_SIZE / 4); ul_idx++) {
//
//			return pul_test_page[ul_idx];
//
//
//	}
	return pul_test_page;
}
