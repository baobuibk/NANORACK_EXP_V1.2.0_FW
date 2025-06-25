#include "is66wvs4m8bll.h"
#include "stm32f7xx_ll_bus.h"

// Byte dummy cho DMA TX
static uint8_t dummy_byte = 0x00;

// Khởi tạo cấu hình SRAM
static SRAM_Config_t SRAM_Config = {
    .spi = SPI2,
    .cs_port = GPIOD,
    .cs_pin = LL_GPIO_PIN_8,
    .buffer = NULL,
    .buffer_size = 50000,
    .transfer_size = 0,
    .transfer_done = 0,
    .dma = DMA1,
    .dma_stream_tx = LL_DMA_STREAM_4,
    .dma_stream_rx = LL_DMA_STREAM_3,
    .dma_channel_tx = LL_DMA_CHANNEL_0,
    .dma_channel_rx = LL_DMA_CHANNEL_0
};

// Hàm khởi tạo SRAM
void SRAM_Init(SRAM_Config_t *config) {
    // Cấp bộ đệm
    static uint8_t sram_buffer[50000];
    config->buffer = sram_buffer;

    // 1. Kích hoạt clock
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD); // GPIOD (PD8)
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB); // GPIOB (PB13-15)
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2); // SPI2
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1); // DMA1

    // 2. Cấu hình GPIO PD8 (CS)
    LL_GPIO_SetPinMode(config->cs_port, config->cs_pin, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(config->cs_port, config->cs_pin, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinSpeed(config->cs_port, config->cs_pin, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinPull(config->cs_port, config->cs_pin, LL_GPIO_PULL_NO);
    LL_GPIO_SetOutputPin(config->cs_port, config->cs_pin); // CS cao

    // 3. Cấu hình GPIO PB13 (SCK), PB14 (MISO), PB15 (MOSI)
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15, LL_GPIO_AF_5);
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_14, LL_GPIO_PULL_UP); // MISO pull-up

    // 4. Cấu hình SPI2
    LL_SPI_SetMode(config->spi, LL_SPI_MODE_MASTER);
    LL_SPI_SetDataWidth(config->spi, LL_SPI_DATAWIDTH_8BIT);
    LL_SPI_SetClockPolarity(config->spi, LL_SPI_POLARITY_LOW); // Mode 0
    LL_SPI_SetClockPhase(config->spi, LL_SPI_PHASE_1EDGE);     // Mode 0
    LL_SPI_SetNSSMode(config->spi, LL_SPI_NSS_SOFT);
    LL_SPI_SetBaudRatePrescaler(config->spi, LL_SPI_BAUDRATEPRESCALER_DIV4); // ~25 MHz
    LL_SPI_SetTransferDirection(config->spi, LL_SPI_FULL_DUPLEX);
    LL_SPI_SetRxFIFOThreshold(config->spi, LL_SPI_RX_FIFO_TH_QUARTER);
    LL_SPI_EnableDMAReq_TX(config->spi);
    LL_SPI_EnableDMAReq_RX(config->spi);
    LL_SPI_Enable(config->spi);

    // 5. Cấu hình DMA Stream TX
    LL_DMA_SetChannelSelection(config->dma, config->dma_stream_tx, config->dma_channel_tx);
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
    LL_DMA_SetChannelSelection(config->dma, config->dma_stream_rx, config->dma_channel_rx);
    LL_DMA_SetDataTransferDirection(config->dma, config->dma_stream_rx, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetStreamPriorityLevel(config->dma, config->dma_stream_rx, LL_DMA_PRIORITY_HIGH);
    LL_DMA_SetMode(config->dma, config->dma_stream_rx, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(config->dma, config->dma_stream_rx, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(config->dma, config->dma_stream_rx, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(config->dma, config->dma_stream_rx, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(config->dma, config->dma_stream_rx, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(config->dma, config->dma_stream_rx);
    LL_DMA_SetPeriphAddress(config->dma, config->dma_stream_rx, (uint32_t)&(config->spi->DR));

    // 7. Cấu hình NVIC
    NVIC_SetPriority(DMA1_Stream3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 3));
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    NVIC_SetPriority(DMA1_Stream4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 2));
    NVIC_EnableIRQ(DMA1_Stream4_IRQn);
}

// Hàm đọc dữ liệu từ SRAM với DMA
void SRAM_Read(SRAM_Config_t *config, uint32_t address, uint32_t size, uint8_t *buffer) {
    if (size > config->buffer_size) {
        size = config->buffer_size;
    }
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
    LL_DMA_SetMemoryAddress(config->dma, config->dma_stream_tx, (uint32_t)&dummy_byte);
    LL_DMA_EnableStream(config->dma, config->dma_stream_tx);

    // Cấu hình DMA RX (nhận dữ liệu)
    LL_DMA_DisableStream(config->dma, config->dma_stream_rx);
    LL_DMA_SetDataLength(config->dma, config->dma_stream_rx, size);
    LL_DMA_SetMemoryAddress(config->dma, config->dma_stream_rx, (uint32_t)buffer);
    LL_DMA_EnableIT_TC(config->dma, config->dma_stream_rx);
    LL_DMA_EnableStream(config->dma, config->dma_stream_rx);
}

// Hàm ghi dữ liệu vào SRAM với DMA
void SRAM_Write(SRAM_Config_t *config, uint32_t address, uint32_t size, uint8_t *buffer) {
    if (size > config->buffer_size) {
        size = config->buffer_size;
    }
    config->transfer_size = size;
    config->transfer_done = 0;

    // Gửi lệnh ghi và địa chỉ
    uint8_t cmd[4] = {SRAM_WRITE_CMD, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF};
    LL_GPIO_ResetOutputPin(config->cs_port, config->cs_pin); // CS thấp
    for (uint8_t i = 0; i < 4; i++) {
        while (!LL_SPI_IsActiveFlag_TXE(config->spi));
        LL_SPI_TransmitData8(config->spi, cmd[i]);
        while (!LL_SPI_IsActiveFlag_RXNE(config->spi));
        LL_SPI_ReceiveData8(config->spi); // Đọc bỏ dummy
    }

    // Cấu hình DMA TX
    LL_DMA_DisableStream(config->dma, config->dma_stream_tx);
    LL_DMA_SetDataLength(config->dma, config->dma_stream_tx, size);
    LL_DMA_SetMemoryAddress(config->dma, config->dma_stream_tx, (uint32_t)buffer);
    LL_DMA_SetMemoryIncMode(config->dma, config->dma_stream_tx, LL_DMA_MEMORY_INCREMENT); // Tăng địa chỉ cho ghi
    LL_DMA_EnableIT_TC(config->dma, config->dma_stream_tx);
    LL_DMA_EnableStream(config->dma, config->dma_stream_tx);
}

// Hàm xử lý ngắt DMA RX (SPI2_RX)
void DMA1_Stream3_IRQHandler(void) {
    if (LL_DMA_IsActiveFlag_TC3(DMA1)) {
        LL_DMA_ClearFlag_TC3(DMA1); // Xóa cờ ngắt
        LL_DMA_DisableStream(DMA1, SRAM_Config.dma_stream_rx);
        LL_DMA_DisableStream(DMA1, SRAM_Config.dma_stream_tx); // Tắt DMA TX
        LL_GPIO_SetOutputPin(SRAM_Config.cs_port, SRAM_Config.cs_pin); // CS cao
        SRAM_Config.transfer_done = 1; // Báo hoàn tất
    }
}

// Hàm xử lý ngắt DMA TX (SPI2_TX)
void DMA1_Stream4_IRQHandler(void) {
    if (LL_DMA_IsActiveFlag_TC4(DMA1)) {
        LL_DMA_ClearFlag_TC4(DMA1); // Xóa cờ ngắt
        LL_DMA_DisableStream(DMA1, SRAM_Config.dma_stream_tx);
        // Không đặt CS cao, chờ DMA RX
    }
}

// Hàm kiểm tra trạng thái truyền
uint8_t SRAM_IsTransferComplete(SRAM_Config_t *config) {
    return config->transfer_done;
}
