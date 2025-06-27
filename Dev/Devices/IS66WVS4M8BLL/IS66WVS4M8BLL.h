#ifndef IS66WVS4M8BLL_H
#define IS66WVS4M8BLL_H

#include "stm32f7xx_ll_gpio.h"
#include "stm32f7xx_ll_spi.h"
#include "stm32f7xx_ll_dma.h"

// Định nghĩa lệnh
#define SRAM_READ_CMD  0x03 // Lệnh đọc
#define SRAM_WRITE_CMD 0x02 // Lệnh ghi
#define SRAM_READ_ID_CMD 0x9F // Lệnh ghi
#define SRAM_FAST_READ_CMD 0x0B
// Struct cấu hình SRAM
typedef struct {
    SPI_TypeDef *spi;               // Instance SPI (SPI2)
    GPIO_TypeDef *cs_port;          // Cổng CS (GPIOD)
    uint32_t cs_pin;                // Chân CS (PD7)
    uint32_t transfer_size;         // Kích thước truyền
    uint8_t transfer_done;          // Cờ hoàn thành
    DMA_TypeDef *dma;               // Instance DMA (DMA2)
    uint32_t dma_stream_tx;         // Stream TX (Stream 5)
    uint32_t dma_stream_rx;         // Stream RX (Stream 6)
    uint32_t dma_channel;           // Channel (0)
} IS66_t;

// Prototype hàm
void SRAM_Initialize(IS66_t *config);
void SRAM_read_polling(IS66_t *config, uint32_t address, uint32_t size, uint8_t *buffer);
void SRAM_fast_read_polling(IS66_t *config, uint32_t address, uint32_t size, uint8_t *buffer);
void SRAM_write_polling(IS66_t *config, uint32_t address, uint32_t size, uint8_t *buffer);
void SRAM_read_id(IS66_t *config, uint8_t *buffer);

void SRAM_write_DMA(IS66_t *config, uint32_t address, uint32_t size, uint8_t *buffer) ;
void SRAM_read_DMA(IS66_t *config, uint32_t address, uint32_t size, uint8_t *buffer) ;



uint8_t SRAM_IsTransferDone(IS66_t *config);
void DMA_RX_callback(IS66_t *dev);

#endif // IS66WVS4M8BLL_H
