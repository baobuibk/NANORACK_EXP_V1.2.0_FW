/*
 * bsp_photodiode.c
 *
 *  Created on: Jun 24, 2025
 *      Author: Admin
 */

#include "bsp_photodiode.h"
#include "bsp_laser.h"
#include "main.h"
#include "experiment_task.h"
#include "app_signals.h"
#include "bsp_spi_ram.h"
#define PHOTO_DMA DMA1
#define PHOTO_SPI SPI2
#define PHOTO_TIMER TIM2



uint16_t photo_data_buffer[BUFFER_FULL_SIZE];
uint16_t * const upper_data_buffer = photo_data_buffer + BUFFER_HALF_SIZE;

photo_diode_t photo_diode_adc = {
		.spi = PHOTO_SPI,
		.dma = DMA1,
		.dma_stream_rx = LL_DMA_STREAM_1
};
typedef struct bsp_photodiode_timer_param_t
{
	uint32_t pre_time_ARR;
	uint32_t sampling_time_ARR;
	uint32_t post_time_ARR;
	uint32_t sampling_period_ARR;

}bsp_photodiode_timer_param_t;


ADG1414_Device_t photo_sw;
ADS8327_Device_t ads8327_dev = {
		.spi = PHOTO_SPI,
		.cs_port = PHOTO_ADC_CS_GPIO_Port,
		.cs_pin = PHOTO_ADC_CS_Pin
};

static photo_state_t photo_diode_state = PHOTO_SAMPLED_PRE;


bsp_photodiode_timer_param_t timer_timing;

static experiment_evt_t const finish_pre_phase_evt = {.super = {.sig = EVT_EXPERIMENT_FINISH_PRE_SAMPLING} };
static experiment_evt_t const finish_sampling_phase_evt = {.super = {.sig = EVT_EXPERIMENT_FINISH_SAMPLING} };
static experiment_evt_t const finish_post_phase_evt = {.super = {.sig = EVT_EXPERIMENT_FINISH_POST_SAMPLING} };

extern experiment_task_t experiment_task_inst;

void bsp_photodiode_timer1_init(uint32_t period_ns);
void bsp_photodiode_set_time_tim2(uint32_t period_ns);

void bsp_photodiode_sample_rate_timer_init();
void bsp_photodiode_set_pre_time();
void bsp_photodiode_set_sampling_time();
void bsp_photodiode_set_post_time();
void bsp_photodiode_timer1_init(uint32_t period_ns);
void bsp_photodiode_start_dma(photo_diode_t *config, uint32_t *buffer, uint32_t size);


void bsp_photo_set_time(bsp_photodiode_time_t * init_photo_time)
{
	photo_diode_adc.timing = *init_photo_time;
	bsp_photodiode_time_t * photo_time = &photo_diode_adc.timing;
	/*
	 * timer2 clock is 100 Mhz, meam 100 tick = 1us
	 */
	timer_timing.post_time_ARR = photo_time->post_time * 100;				// tick
	timer_timing.pre_time_ARR = photo_time->pre_time * 100;
	timer_timing.sampling_time_ARR = photo_time->sampling_time * 100;

	//sampling rate in Khz, timer1 clock is 200 Mhz
	timer_timing.sampling_period_ARR = (1000 * 200 / photo_time->sampling_rate);
}

void bsp_photo_prepare_pre_sampling()
{
	bsp_photodiode_sample_rate_timer_init();
	bsp_photodiode_set_pre_time();
    // Xóa cờ ngắt update
	TIM1->SR = ~TIM_SR_UIF; // Xóa cờ ngắt
    TIM2->SR = ~TIM_SR_UIF;
	TIM1->CNT = 0;
	TIM2->CNT = 0;
//	TIM1->CR1 |= TIM_CR1_CEN;
//	TIM2->CR1 |= TIM_CR1_CEN;
//	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
//	NVIC_EnableIRQ(TIM2_IRQn);
}
void bsp_photo_sw_init(void)
{
	bsp_photodiode_sw_spi_change_mode();
	ADG1414_Chain_Init(&photo_sw, PHOTO_SPI, PHOTO_SW_CS_GPIO_Port, PHOTO_SW_CS_Pin, INTERNAL_CHAIN_SWITCH_NUM);

}

void bsp_photo_adc_init()
{
	bsp_photodiode_adc_spi_change_mode();
	ADS8327_Device_Init(&ads8327_dev);
}

void bsp_photodiode_set_spi_mode(spi_mode_t spi_mode)
{
	switch(spi_mode)
	{
		case SPI_MODE_0:
			LL_SPI_SetClockPolarity(PHOTO_SPI, LL_SPI_POLARITY_LOW);
			LL_SPI_SetClockPhase(PHOTO_SPI, LL_SPI_PHASE_1EDGE);
		break;

		case SPI_MODE_1:
			LL_SPI_SetClockPolarity(PHOTO_SPI, LL_SPI_POLARITY_LOW);
			LL_SPI_SetClockPhase(PHOTO_SPI, LL_SPI_PHASE_2EDGE);
		break;

		case SPI_MODE_2:
			LL_SPI_SetClockPolarity(PHOTO_SPI, LL_SPI_POLARITY_HIGH);
			LL_SPI_SetClockPhase(PHOTO_SPI, LL_SPI_PHASE_1EDGE);
		break;

		case SPI_MODE_3:
			LL_SPI_SetClockPolarity(PHOTO_SPI, LL_SPI_POLARITY_HIGH);
			LL_SPI_SetClockPhase(PHOTO_SPI, LL_SPI_PHASE_2EDGE);
		break;
	}
}

void bsp_photodiode_set_spi_data_len(uint32_t DataWidth)
{
    LL_SPI_SetDataWidth(PHOTO_SPI, DataWidth);
    if (DataWidth < LL_SPI_DATAWIDTH_9BIT)
	{
    	LL_SPI_SetRxFIFOThreshold(PHOTO_SPI, LL_SPI_RX_FIFO_TH_QUARTER);
	}
    else
    {
    	LL_SPI_SetRxFIFOThreshold(PHOTO_SPI, LL_SPI_RX_FIFO_TH_HALF);
    }
}

void bsp_photodiode_set_spi_prescaler(uint32_t Prescaler)
{
    LL_SPI_SetBaudRatePrescaler(PHOTO_SPI, Prescaler);
}

void bsp_photodiode_sw_spi_change_mode()
{
	LL_SPI_Disable(PHOTO_SPI);
	bsp_photodiode_set_spi_data_len(LL_SPI_DATAWIDTH_8BIT);
	bsp_photodiode_set_spi_mode(SPI_MODE_1);
	bsp_photodiode_set_spi_prescaler(LL_SPI_BAUDRATEPRESCALER_DIV16);
	LL_SPI_Enable(PHOTO_SPI);
}

void bsp_photodiode_adc_spi_change_mode()
{
	LL_SPI_Disable(PHOTO_SPI);
	bsp_photodiode_set_spi_data_len(LL_SPI_DATAWIDTH_16BIT);
	bsp_photodiode_set_spi_mode(SPI_MODE_0);
	bsp_photodiode_set_spi_prescaler(LL_SPI_BAUDRATEPRESCALER_DIV2);
	LL_SPI_Enable(PHOTO_SPI);
}

void bsp_photo_switch_on(uint32_t channel_idx)
{
	ADG1414_Chain_SwitchOn(&photo_sw, channel_idx);
}
void bsp_photo_switch_off_all(void)
{
	ADG1414_Chain_SwitchAllOff(&photo_sw);
}
/*
 *timer interrupt->start Conv
 *timer 1 for sampling
 *timer
 */
void bsp_photodiode_sample_rate_timer_init()
{
	bsp_photodiode_timer1_init(timer_timing.sampling_period_ARR);
}
void bsp_photodiode_set_pre_time()
{
	bsp_photodiode_set_time_tim2(timer_timing.pre_time_ARR);
}
void bsp_photodiode_set_sampling_time()
{
	bsp_photodiode_set_time_tim2(timer_timing.sampling_time_ARR);
}
void bsp_photodiode_set_post_time()
{
	bsp_photodiode_set_time_tim2(timer_timing.post_time_ARR);
}
void bsp_photodiode_timer1_init(uint32_t period) {

    uint32_t ticks = period ; // Số tick cần cho period

    uint16_t prescaler = 0;
    uint16_t arr = ticks - 1; // Giá trị ARR = số tick - 1

    // Nếu ticks lớn hơn 0xFFFF, cần điều chỉnh prescaler
    if (ticks > 0xFFFF) {
        prescaler = (ticks / 0xFFFF) + 1; // Làm tròn lên
        arr = (ticks / prescaler) - 1;
    }

    // Cấu hình Timer 1
    //TIM1->CR1 = 0; // Xóa thanh ghi điều khiển
    TIM1->ARR = arr; // Cài đặt giá trị auto-reload
    TIM1->PSC = prescaler; // Cài đặt prescaler
    TIM1->EGR = TIM_EGR_UG; // Tạo sự kiện update để áp dụng ngay
    TIM1->DIER &= ~TIM_DIER_UIE; // off ngắt update
    TIM1->CNT = 0;
    // Cấu hình NVIC cho ngắt Timer 1
    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0); // Ưu tiên ngắt cao nhất
    NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
}


void bsp_photodiode_set_time_tim2(uint32_t period) {

    uint32_t ticks = period; // Số tick cần cho period ns
    // Tính prescaler và auto-reload (ARR)
    uint16_t prescaler = 0;
    uint32_t arr = ticks - 1; // Giá trị ARR = số tick - 1 (TIM2 hỗ trợ 32-bit ARR)
    // Cấu hình Timer 2
    //TIM2->CR1 = 0; // Xóa thanh ghi điều khiển
    TIM2->ARR = arr; // Cài đặt giá trị auto-reload
    TIM2->PSC = prescaler; // Cài đặt prescaler
    TIM2->EGR = TIM_EGR_UG; // Tạo sự kiện update để áp dụng ngay
    TIM2->DIER &= ~TIM_DIER_UIE;
    TIM2->CNT = 0;
    // 4. Cấu hình NVIC cho ngắt Timer 2
    NVIC_SetPriority(TIM2_IRQn, 1); // Ưu tiên ngắt 1
     NVIC_DisableIRQ(TIM2_IRQn);
}
void bsp_photodiode_sample_start()
{
	bsp_photodiode_adc_spi_change_mode();
	bsp_photo_prepare_pre_sampling();
	bsp_photodiode_time_t * timing = &photo_diode_adc.timing;
	uint32_t num_sample = ((timing->pre_time + timing->sampling_time + timing->post_time) * timing->sampling_rate) / 1000;
	if (num_sample % BUFFER_FULL_SIZE) photo_diode_adc.block_count = (num_sample / BUFFER_FULL_SIZE) + 1;
	else photo_diode_adc.block_count = (num_sample / BUFFER_FULL_SIZE);
	photo_diode_adc.ram_current_address = 0;

	bsp_photodiode_start_dma(&photo_diode_adc,(uint32_t *)&photo_data_buffer,BUFFER_FULL_SIZE);

	TIM2->CNT = 0;
	TIM1->CNT = 0;
	TIM1->SR &= ~TIM_SR_UIF; // Xóa cờ ngắt
    TIM2->SR &= ~TIM_SR_UIF;
    TIM1->DIER |= TIM_DIER_UIE;
    TIM2->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
	    // 5. Kích hoạt Timer 1
	TIM2->CR1 |= TIM_CR1_CEN;
    TIM1->CR1 |= TIM_CR1_CEN;
    photo_diode_state = PHOTO_SAMPLED_PRE;


}

void bsp_photodiode_start_dma(photo_diode_t *config, uint32_t *buffer, uint32_t size)
{
	//Config stream rx
	LL_DMA_SetMode(config->dma, config->dma_stream_rx, LL_DMA_MODE_CIRCULAR);
	LL_DMA_ConfigAddresses(	config->dma,
							config->dma_stream_rx,
							(uint32_t)&(config->spi->DR),
							(uint32_t)buffer,
							LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetDataLength(config->dma, config->dma_stream_rx, size);
	LL_DMA_SetMemoryIncMode(config->dma, config->dma_stream_rx, LL_DMA_MEMORY_INCREMENT);

	// Kích hoạt DMA
	LL_DMA_EnableIT_TC(config->dma, config->dma_stream_rx);		// Kích hoạt ngắt DMA hoàn tất (cho RX)
	LL_DMA_EnableIT_HT(config->dma, config->dma_stream_rx);		// Kích hoạt ngắt DMA hoàn tất (cho RX)
	LL_SPI_EnableDMAReq_RX(config->spi);
	LL_DMA_EnableStream(config->dma, config->dma_stream_rx); 	// RX trước
}





// Hàm xử lí DMA ADC
void DMA1_Stream1_IRQHandler(void)
{
	TIM1->CR1 &= ~TIM_CR1_CEN;		// Stop timer trigger
	GPIOD->BSRR = GPIO_BSRR_BS_9; // Đặt CS lên 1
	// Kiểm tra cờ ngắt HT
	if (DMA1->LISR & DMA_LISR_HTIF1)
	{
		DMA1->LIFCR = DMA_LIFCR_CHTIF1;	// Clear HT flag
		bsp_spi_ram_write_dma(photo_diode_adc.ram_current_address, BUFFER_HALF_SIZE_BYTE, (uint8_t *)photo_data_buffer);
		photo_diode_adc.ram_current_address += BUFFER_HALF_SIZE_BYTE;
		TIM1->CR1 |= TIM_CR1_CEN;		// Continue start timer trigger
	}

	// Kiểm tra cờ ngắt TC
	else if (DMA1->LISR & DMA_LISR_TCIF1)
	{
		DMA1->LIFCR = DMA_LIFCR_CTCIF1;	// Clear TC flag
		//DMA_TC_RX_callback(&photo_diode_adc);
		bsp_spi_ram_write_dma(photo_diode_adc.ram_current_address, BUFFER_HALF_SIZE_BYTE, (uint8_t *)upper_data_buffer);
		photo_diode_adc.ram_current_address += BUFFER_HALF_SIZE_BYTE;
		photo_diode_adc.block_count --;
		TIM1->CR1 |= TIM_CR1_CEN;		// Continue start timer trigger
		if ((photo_diode_adc.block_count) == 0)
		{
			PHOTO_TIMER->CR1 &= ~TIM_CR1_CEN;		// stop timer 2
			PHOTO_TIMER->DIER &= ~TIM_DIER_UIE;
			TIM1->CR1 &= ~TIM_CR1_CEN;				// Timer1 disable counter
			TIM1->DIER &= ~TIM_DIER_UIE;			// Timer1 disable IT Update
			LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);
			LL_DMA_DisableIT_TC(DMA1, LL_DMA_STREAM_1);
			LL_DMA_DisableIT_HT(DMA1, LL_DMA_STREAM_1);
			LL_SPI_DisableDMAReq_RX(photo_diode_adc.spi);
			SST_Task_post((SST_Task *)&experiment_task_inst.super, (SST_Evt *)&finish_post_phase_evt);
		}
	}


}

// Hàm xử lý ngắt Timer ADC trigger
void TIM1_UP_TIM10_IRQHandler(void)
{
	// Tạo xung trên PD10
	GPIOD->BSRR = GPIO_BSRR_BS_9; // Đặt CS lên 1
	GPIOD->BSRR = GPIO_BSRR_BR_10; // Đặt CV xuống 0
	__NOP();
	__NOP();
	__NOP();
	__NOP();

	//PHOTO_SPI->DR = 0xAAAA;
	GPIOD->BSRR = GPIO_BSRR_BR_9; // CS=0
	LL_SPI_TransmitData16(PHOTO_SPI, 0xAAAA);
	GPIOD->BSRR = GPIO_BSRR_BS_10; // Đặt PD10 lên 1
	TIM1->SR = ~TIM_SR_UIF; // Xóa cờ ngắt
}

void TIM2_IRQHandler(void)
{
	if (PHOTO_TIMER->SR & TIM_SR_UIF)
	{
		PHOTO_TIMER->SR = ~TIM_SR_UIF;	// Xóa cờ ngắt update
		switch (photo_diode_state)
		{
			case PHOTO_SAMPLED_PRE:
				bsp_laser_int_switch_on(photo_diode_adc.timing.pos);
				photo_diode_state = PHOTO_SAMPLED_SAMPLING;
//				bsp_photodiode_set_sampling_time();
				PHOTO_TIMER->ARR = timer_timing.sampling_time_ARR - 1;
				PHOTO_TIMER->CNT = 0;
				SST_Task_post((SST_Task *)&experiment_task_inst.super, (SST_Evt *)&finish_pre_phase_evt);
			break;

			case PHOTO_SAMPLED_SAMPLING:
				bsp_laser_int_switch_off_all();
				photo_diode_state = PHOTO_SAMPLING_STOP;
//				bsp_photodiode_set_sampling_time();
				PHOTO_TIMER->ARR = timer_timing.post_time_ARR-1;
				PHOTO_TIMER->CNT = 0;
				SST_Task_post((SST_Task *)&experiment_task_inst.super, (SST_Evt *)&finish_sampling_phase_evt);
				PHOTO_TIMER->CR1 &= ~TIM_CR1_CEN;		// stop timer
			break;
			default: break;
		}
	}
}
