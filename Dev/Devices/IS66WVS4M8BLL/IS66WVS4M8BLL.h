#ifndef IS66WVS4M8BLL_H
#define IS66WVS4M8BLL_H

#include "stm32f7xx_ll_gpio.h"
#include "stm32f7xx_ll_spi.h"
#include "stm32f7xx_ll_dma.h"

// Định nghĩa lệnh
#define SRAM_READ_CMD  0x03 // Lệnh đọc
#define SRAM_WRITE_CMD 0x02 // Lệnh ghi

// Struct cấu hình SRAM
typedef struct {
    SPI_TypeDef *spi;               // Instance SPI
    GPIO_TypeDef *cs_port;          // Cổng CS
    uint32_t cs_pin;                // Chân CS
    uint8_t *buffer;                // Bộ đệm DMA
    uint32_t buffer_size;           // Kích thước bộ đệm
    volatile uint32_t transfer_size; // Kích thước truyền
    volatile uint8_t transfer_done; // Cờ hoàn tất
    DMA_TypeDef *dma;               // Instance DMA
    uint32_t dma_stream_tx;         // Stream TX
    uint32_t dma_stream_rx;         // Stream RX
    uint32_t dma_channel_tx;        // Channel TX
    uint32_t dma_channel_rx;        // Channel RX
} SRAM_Config_t;

// Prototype hàm
void SRAM_Init(SRAM_Config_t *config);
void SRAM_Read(SRAM_Config_t *config, uint32_t address, uint32_t size, uint8_t *buffer);
void SRAM_Write(SRAM_Config_t *config, uint32_t address, uint32_t size, uint8_t *buffer);
uint8_t SRAM_IsTransferComplete(SRAM_Config_t *config);

#endif // IS66WVS4M8BLL_H
