/*
 * bsp_spi_ram.h
 *
 *  Created on: Jun 26, 2025
 *      Author: Admin
 */

#ifndef BSP_SPI_RAM_H_
#define BSP_SPI_RAM_H_

#include "board.h"
void bsp_spi_ram_write_polling(uint32_t address, uint32_t size, uint8_t *buffer);
void bsp_spi_ram_read_polling(uint32_t address, uint32_t size, uint8_t *buffer);
void bsp_spi_ram_read_id(uint8_t * buffer);

#endif /* BSUPPORT_BSP_BSP_SPI_RAM_BSP_SPI_RAM_H_ */
