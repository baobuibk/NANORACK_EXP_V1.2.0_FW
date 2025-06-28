#include "is66wvs4m8bll.h"
#include "stm32f7xx_ll_bus.h"
#include "stm32f7xx_ll_dma.h"

static uint8_t data_dummy = 0xAA;

// Hàm khởi tạo SRAM
void SRAM_Initialize(IS66_t *config)
{
	LL_DMA_SetMode(config->dma, config->dma_stream_tx, LL_DMA_MODE_NORMAL);
	LL_DMA_SetMode(config->dma, config->dma_stream_rx, LL_DMA_MODE_NORMAL);

	if (LL_DMA_IsActiveFlag_TC6(config->dma))
	{
		LL_DMA_ClearFlag_TC6(config->dma);
	}
	if (LL_DMA_IsActiveFlag_TC5(config->dma))
	{
		LL_DMA_ClearFlag_TC5(config->dma);
	}

	LL_DMA_EnableIT_TC(config->dma, config->dma_stream_rx);		// Kích hoạt ngắt DMA hoàn tất (cho RX)
	LL_SPI_EnableDMAReq_TX(config->spi);
	LL_SPI_EnableDMAReq_RX(config->spi);
}


void SRAM_read_id(IS66_t *config, uint8_t *buffer)
{
	uint32_t i;
    uint8_t cmd[4] = {SRAM_READ_ID_CMD, 0, 0, 0};
    LL_GPIO_ResetOutputPin(config->cs_port, config->cs_pin); // CS thấp
    for (i = 0; i < 4; i++) {
        while (!LL_SPI_IsActiveFlag_TXE(config->spi));
        LL_SPI_TransmitData8(config->spi, cmd[i]);
        while (!LL_SPI_IsActiveFlag_RXNE(config->spi));
        LL_SPI_ReceiveData8(config->spi); // Đọc bỏ dummy
    }
    for (i = 0; i < 8; i++) {
        while (!LL_SPI_IsActiveFlag_TXE(config->spi));
        LL_SPI_TransmitData8(config->spi, 0xAA);
        while (!LL_SPI_IsActiveFlag_RXNE(config->spi));
        buffer[i] = LL_SPI_ReceiveData8(config->spi); // Đọc bỏ dummy
    }
    LL_GPIO_SetOutputPin(config->cs_port, config->cs_pin); // CS thấp

}

// Hàm ghi dữ liệu vào SRAM với DMA
void SRAM_write_polling(IS66_t *config, uint32_t address, uint32_t size, uint8_t *buffer) {

	uint32_t i;
	uint8_t cmd[4] = {SRAM_WRITE_CMD, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF};
	LL_GPIO_SetOutputPin(config->cs_port, config->cs_pin); // make sure CS is high
	LL_GPIO_ResetOutputPin(config->cs_port, config->cs_pin); // CS thấp
	    for (i = 0; i < 4; i++) {
	        while (!LL_SPI_IsActiveFlag_TXE(config->spi));
	        LL_SPI_TransmitData8(config->spi, cmd[i]);
	        while (!LL_SPI_IsActiveFlag_RXNE(config->spi));
	        LL_SPI_ReceiveData8(config->spi); // Đọc bỏ dummy
	    }
	    for (i = 0; i < size; i++) {
	        while (!LL_SPI_IsActiveFlag_TXE(config->spi));
	        LL_SPI_TransmitData8(config->spi, buffer[i]);
	        while (!LL_SPI_IsActiveFlag_RXNE(config->spi)){};
	         LL_SPI_ReceiveData8(config->spi); // Đọc bỏ dummy
	    }
	    LL_GPIO_SetOutputPin(config->cs_port, config->cs_pin); // CS thấp
}

void SRAM_read_polling(IS66_t *config, uint32_t address, uint32_t size, uint8_t *buffer) {

	uint32_t i;
	uint8_t cmd[4] = {SRAM_READ_CMD, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF};
	LL_GPIO_SetOutputPin(config->cs_port, config->cs_pin); // make sure CS is high
	LL_GPIO_ResetOutputPin(config->cs_port, config->cs_pin); // CS thấp
	    for (i = 0; i < 4; i++) {
	        while (!LL_SPI_IsActiveFlag_TXE(config->spi));
	        LL_SPI_TransmitData8(config->spi, cmd[i]);
	        while (!LL_SPI_IsActiveFlag_RXNE(config->spi));
	        LL_SPI_ReceiveData8(config->spi); // Đọc bỏ dummy
	    }
	    for (i = 0; i < size; i++) {
	        while (!LL_SPI_IsActiveFlag_TXE(config->spi));
	        LL_SPI_TransmitData8(config->spi, 0xAA);
	        while (!LL_SPI_IsActiveFlag_RXNE(config->spi));
	        buffer[i] = LL_SPI_ReceiveData8(config->spi); // Đọc bỏ dummy
	    }
	    LL_GPIO_SetOutputPin(config->cs_port, config->cs_pin); // CS thấp
}


void SRAM_fast_read_polling(IS66_t *config, uint32_t address, uint32_t size, uint8_t *buffer) {

	uint32_t i;
	uint8_t cmd[5] = {SRAM_FAST_READ_CMD, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF,0};
	LL_GPIO_SetOutputPin(config->cs_port, config->cs_pin); // make sure CS is high
	LL_GPIO_ResetOutputPin(config->cs_port, config->cs_pin); // CS thấp
	    for (i = 0; i < 5; i++) {
	        while (!LL_SPI_IsActiveFlag_TXE(config->spi));
	        LL_SPI_TransmitData8(config->spi, cmd[i]);
	        while (!LL_SPI_IsActiveFlag_RXNE(config->spi));
	        LL_SPI_ReceiveData8(config->spi); // Đọc bỏ dummy
	    }
	    for (i = 0; i < size; i++) {
	        while (!LL_SPI_IsActiveFlag_TXE(config->spi));
	        LL_SPI_TransmitData8(config->spi, 0xAA);
	        while (!LL_SPI_IsActiveFlag_RXNE(config->spi));
	        buffer[i] = LL_SPI_ReceiveData8(config->spi); // Đọc bỏ dummy
	    }
	    LL_GPIO_SetOutputPin(config->cs_port, config->cs_pin); // CS thấp
}




/*
 * Ghi dữ liệu vào SRAM sử dụng DMA qua SPI.
 * @param config: Con trỏ đến cấu trúc thiết bị IS66 chứa thông tin SPI và DMA.
 * @param address: Địa chỉ trong SRAM để ghi dữ liệu.
 * @param size: Kích thước dữ liệu cần ghi (đơn vị: byte).
 * @param buffer: Con trỏ đến bộ đệm chứa dữ liệu cần ghi.
 */
void SRAM_write_DMA(IS66_t *config, uint32_t address, uint32_t size, uint8_t *buffer)
{

	uint32_t i;
	uint8_t cmd[4] = {SRAM_WRITE_CMD, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF};
	config->transfer_done = 0;

	config->cs_port->BSRR = config->cs_pin;
	//LL_GPIO_SetOutputPin(config->cs_port, config->cs_pin); // make sure CS is high
	// start transfer
	config->cs_port->BSRR = config->cs_pin << 16;
	//LL_GPIO_ResetOutputPin(config->cs_port, config->cs_pin); // CS thấp

	for (i = 0; i < 4; i++) {
		while (!LL_SPI_IsActiveFlag_TXE(config->spi));
		LL_SPI_TransmitData8(config->spi, cmd[i]);
		while (!LL_SPI_IsActiveFlag_RXNE(config->spi));
		LL_SPI_ReceiveData8(config->spi); // Đọc bỏ dummy
	}

	//Config stream tx
	LL_DMA_ConfigAddresses(	config->dma,
							config->dma_stream_tx,
							(uint32_t)buffer,
							(uint32_t)&(config->spi->DR),
							LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetDataLength(config->dma, config->dma_stream_tx, size);
	LL_DMA_SetMemoryIncMode(config->dma, config->dma_stream_tx, LL_DMA_MEMORY_INCREMENT);

	//Config stream rx
	LL_DMA_ConfigAddresses(	config->dma,
							config->dma_stream_rx,
							(uint32_t)&(config->spi->DR),
							(uint32_t)&data_dummy,
							LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetDataLength(config->dma, config->dma_stream_rx, size);
	LL_DMA_SetMemoryIncMode(config->dma, config->dma_stream_rx, LL_DMA_MEMORY_NOINCREMENT);

	// Kích hoạt DMA
	LL_DMA_EnableStream(config->dma, config->dma_stream_rx); 	// RX trước
	LL_DMA_EnableStream(config->dma, config->dma_stream_tx); 	// TX sau


}


void SRAM_read_DMA(IS66_t *config, uint32_t address, uint32_t size, uint8_t *buffer) {

	uint32_t i;
	uint8_t cmd[5] = {SRAM_FAST_READ_CMD, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF,0};
	config->transfer_done = 0;
	LL_GPIO_SetOutputPin(config->cs_port, config->cs_pin); // make sure CS is high

	LL_GPIO_ResetOutputPin(config->cs_port, config->cs_pin); // CS thấp

	for (i = 0; i < 5; i++)
	{
		while (!LL_SPI_IsActiveFlag_TXE(config->spi));
		LL_SPI_TransmitData8(config->spi, cmd[i]);
		while (!LL_SPI_IsActiveFlag_RXNE(config->spi));
		LL_SPI_ReceiveData8(config->spi); // Đọc bỏ dummy
	}

	//Config stream tx
	//LL_DMA_SetMode(config->dma, config->dma_stream_tx, LL_DMA_MODE_NORMAL);
	LL_DMA_ConfigAddresses(	config->dma,
							config->dma_stream_tx,
							(uint32_t)&data_dummy,
							(uint32_t)&(config->spi->DR),
							LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetDataLength(config->dma, config->dma_stream_tx, size);
	LL_DMA_SetMemoryIncMode(config->dma, config->dma_stream_tx, LL_DMA_MEMORY_NOINCREMENT);

	//Config stream rx
	//LL_DMA_SetMode(config->dma, config->dma_stream_rx, LL_DMA_MODE_NORMAL);
	LL_DMA_ConfigAddresses(	config->dma,
							config->dma_stream_rx,
							(uint32_t)&(config->spi->DR),
							(uint32_t)buffer,
							LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetDataLength(config->dma, config->dma_stream_rx, size);
	LL_DMA_SetMemoryIncMode(config->dma, config->dma_stream_rx, LL_DMA_MEMORY_INCREMENT);

	// Kích hoạt DMA
	LL_DMA_EnableStream(config->dma, config->dma_stream_rx); 	// RX trước
	LL_DMA_EnableStream(config->dma, config->dma_stream_tx); 	// TX sau
}


void DMA_RX_callback(IS66_t *dev)
{
	LL_GPIO_SetOutputPin(dev->cs_port, dev->cs_pin); // CS cao
	dev->transfer_done = 1; // Báo hoàn tất

	LL_DMA_DisableStream(dev->dma, dev->dma_stream_rx);
	LL_DMA_DisableStream(dev->dma, dev->dma_stream_tx);
}


// Hàm kiểm tra trạng thái truyền
uint8_t SRAM_IsTransferDone(IS66_t *config)
{
    return config->transfer_done;
}
