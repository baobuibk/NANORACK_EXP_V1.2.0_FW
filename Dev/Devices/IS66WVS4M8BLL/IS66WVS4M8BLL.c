#include "is66wvs4m8bll.h"
#include "stm32f7xx_ll_bus.h"

static uint8_t data_dummy = 0x00;

// Hàm khởi tạo SRAM
void SRAM_Initialize(IS66_t *config) {
    // Cấp bộ đệm

    // 1. Kích hoạt clock
//    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD); // GPIOD (PD7)
//    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB); // GPIOB (PB13-15)
//    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2); // SPI2
//    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2); // DMA2
//
//    // 2. Cấu hình GPIO PD7 (CS)
//    LL_GPIO_SetPinMode(config->cs_port, config->cs_pin, LL_GPIO_MODE_OUTPUT);
//    LL_GPIO_SetPinOutputType(config->cs_port, config->cs_pin, LL_GPIO_OUTPUT_PUSHPULL);
//    LL_GPIO_SetPinSpeed(config->cs_port, config->cs_pin, LL_GPIO_SPEED_FREQ_VERY_HIGH);
//    LL_GPIO_SetPinPull(config->cs_port, config->cs_pin, LL_GPIO_PULL_NO);
//    LL_GPIO_SetOutputPin(config->cs_port, config->cs_pin); // CS cao
//
//    // 3. Cấu hình GPIO PB13 (SCK), PB14 (MISO), PB15 (MOSI)
//    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15, LL_GPIO_MODE_ALTERNATE);
//    LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15, LL_GPIO_AF_5);
//    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15, LL_GPIO_SPEED_FREQ_VERY_HIGH);
//    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_14, LL_GPIO_PULL_UP); // MISO pull-up
//
//    // 4. Cấu hình SPI2
//    LL_SPI_SetMode(config->spi, LL_SPI_MODE_MASTER);
//    LL_SPI_SetDataWidth(config->spi, LL_SPI_DATAWIDTH_8BIT);
//    LL_SPI_SetClockPolarity(config->spi, LL_SPI_POLARITY_LOW); // Mode 0
//    LL_SPI_SetClockPhase(config->spi, LL_SPI_PHASE_1EDGE);     // Mode 0
//    LL_SPI_SetNSSMode(config->spi, LL_SPI_NSS_SOFT);
//    LL_SPI_SetBaudRatePrescaler(config->spi, LL_SPI_BAUDRATEPRESCALER_DIV4); // ~25 MHz
//    LL_SPI_SetTransferDirection(config->spi, LL_SPI_FULL_DUPLEX);
//    LL_SPI_SetRxFIFOThreshold(config->spi, LL_SPI_RX_FIFO_TH_QUARTER);
//    LL_SPI_EnableDMAReq_TX(config->spi);
//    LL_SPI_EnableDMAReq_RX(config->spi);
//    LL_SPI_Enable(config->spi);

//     5. Cấu hình DMA Stream TX
    LL_DMA_SetChannelSelection(config->dma, config->dma_stream_tx, config->dma_channel);
    LL_DMA_SetDataTransferDirection(config->dma, config->dma_stream_tx, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetStreamPriorityLevel(config->dma, config->dma_stream_tx, LL_DMA_PRIORITY_HIGH);
    LL_DMA_SetMode(config->dma, config->dma_stream_tx, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(config->dma, config->dma_stream_tx, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(config->dma, config->dma_stream_tx, LL_DMA_MEMORY_NOINCREMENT); // Không tăng địa chỉ
    LL_DMA_SetPeriphSize(config->dma, config->dma_stream_tx, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(config->dma, config->dma_stream_tx, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(config->dma, config->dma_stream_tx);
    LL_DMA_SetPeriphAddress(config->dma, config->dma_stream_tx, (uint32_t)&(config->spi->DR));

    // 6. Cấu hình DMA Stream RX
    LL_DMA_SetChannelSelection(config->dma, config->dma_stream_rx, config->dma_channel);
    LL_DMA_SetDataTransferDirection(config->dma, config->dma_stream_rx, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetStreamPriorityLevel(config->dma, config->dma_stream_rx, LL_DMA_PRIORITY_HIGH);
    LL_DMA_SetMode(config->dma, config->dma_stream_rx, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(config->dma, config->dma_stream_rx, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(config->dma, config->dma_stream_rx, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(config->dma, config->dma_stream_rx, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(config->dma, config->dma_stream_rx, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(config->dma, config->dma_stream_rx);
    LL_DMA_SetPeriphAddress(config->dma, config->dma_stream_rx, (uint32_t)&(config->spi->DR));
//
//    // 7. Cấu hình NVIC
//    NVIC_SetPriority(DMA2_Stream6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 3)); // Stream 6 cho RX
//    NVIC_EnableIRQ(DMA2_Stream6_IRQn);
//    NVIC_SetPriority(DMA2_Stream5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 2)); // Stream 5 cho TX
//    NVIC_EnableIRQ(DMA2_Stream5_IRQn);
}

// Hàm đọc dữ liệu từ SRAM với DMA
void SRAM_Read(IS66_t *config, uint32_t address, uint32_t size, uint8_t *buffer) {

    config->transfer_size = size;
    config->transfer_done = 0;

    // Gửi lệnh đọc và địa chỉ
    uint8_t cmd[4] = {SRAM_READ_CMD, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF};
    LL_GPIO_ResetOutputPin(config->cs_port, config->cs_pin); // CS thấp
    for (uint8_t i = 0; i < 4; i++) {
        while (!LL_SPI_IsActiveFlag_TXE(config->spi));
        LL_SPI_TransmitData8(config->spi, cmd[i]);
        while (!LL_SPI_IsActiveFlag_RXNE(config->spi));
        LL_SPI_ReceiveData8(config->spi); // Đọc bỏ dummy
    }

    // Cấu hình DMA TX (gửi dummy byte)
    LL_DMA_DisableStream(config->dma, config->dma_stream_tx);
    LL_DMA_SetDataLength(config->dma, config->dma_stream_tx, size);
    LL_DMA_SetMemoryAddress(config->dma, config->dma_stream_tx, (uint32_t)&data_dummy);
    LL_DMA_EnableStream(config->dma, config->dma_stream_tx);

    // Cấu hình DMA RX (nhận dữ liệu)
    LL_DMA_DisableStream(config->dma, config->dma_stream_rx);
    LL_DMA_SetDataLength(config->dma, config->dma_stream_rx, size);
    LL_DMA_SetMemoryAddress(config->dma, config->dma_stream_rx, (uint32_t)buffer);
    LL_DMA_EnableIT_TC(config->dma, config->dma_stream_rx);
    LL_DMA_EnableStream(config->dma, config->dma_stream_rx);
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
	        while (!LL_SPI_IsActiveFlag_RXNE(config->spi));
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


// Hàm xử lý ngắt DMA RX (SPI2_RX)
void DMA_RX_callback(IS66_t *dev) {

        LL_DMA_DisableStream(dev->dma, dev->dma_stream_rx);
        LL_DMA_DisableStream(dev->dma, dev->dma_stream_tx); // Tắt DMA TX
        LL_GPIO_SetOutputPin(dev->cs_port, dev->cs_pin); // CS cao
        dev->transfer_done = 1; // Báo hoàn tất

}

// Hàm xử lý ngắt DMA TX (SPI2_TX)
void DMA_TX_callback(IS66_t *dev)  {

        LL_DMA_DisableStream(dev->dma, dev->dma_stream_tx);
        // Không đặt CS cao, chờ DMA RX

}

// Hàm kiểm tra trạng thái truyền
uint8_t SRAM_IsTransferDone(IS66_t *config) {
    return config->transfer_done;
}
