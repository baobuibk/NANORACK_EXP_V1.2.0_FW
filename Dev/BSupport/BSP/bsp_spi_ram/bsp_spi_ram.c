/*
 * bsp_spi_ram.c
 *
 *  Created on: Jun 25, 2025
 *      Author: Admin
 */

#include "IS66WVS4M8BLL.h"
// Byte dummy cho DMA TX



// Khởi tạo cấu hình SRAM
IS66_t IS66WV = {
    .spi = SPI6,
    .cs_port = GPIOD,
    .cs_pin = LL_GPIO_PIN_7, // Dùng PD7 làm CS
    .transfer_size = 0,
    .transfer_done = 0,
    .dma = DMA2, // Dùng DMA2
    .dma_stream_tx = LL_DMA_STREAM_5, // Stream 5 cho TX
    .dma_stream_rx = LL_DMA_STREAM_6, // Stream 6 cho RX
    .dma_channel = LL_DMA_CHANNEL_0 // Channel 0 chung
};

void bsp_spi_ram_write(uint32_t address, uint32_t size, uint8_t *buffer)
{
	SRAM_Write(&IS66WV, address, size, buffer);
}
void bsp_spi_ram_read(uint32_t address, uint32_t size, uint8_t *buffer)
{
	SRAM_Read(&IS66WV, address, size, buffer);
}
