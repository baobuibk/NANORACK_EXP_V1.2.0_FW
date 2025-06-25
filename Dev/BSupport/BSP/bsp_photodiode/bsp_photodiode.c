/*
 * bsp_photodiode.c
 *
 *  Created on: Jun 24, 2025
 *      Author: Admin
 */

#include "bsp_photodiode.h"
#include "main.h"

ADS8327_Device_t photo_adc = {
	    .spi = SPI2,              // Peripheral SPI (SPI1, SPI2, v.v.)
	    .cs_port = PHOTO_ADC_CS_GPIO_Port,         // Port của chân CS
	    .cs_pin = PHOTO_ADC_CS_Pin,               // Chân CS
		.convst_port = PHOTO_ADC_CONV_GPIO_Port,     // Port của chân convst
		.convst_pin = PHOTO_ADC_CONV_Pin,           // Chân convst

};
ADG1414_Device_t photo_sw;
static uint32_t num_sample;


void bsp_photo_sw_init(void)
{
	  ADG1414_Chain_Init(&photo_sw, SPI2, PHOTO_SW_CS_GPIO_Port, PHOTO_SW_CS_Pin, INTERNAL_CHAIN_SWITCH_NUM);

}

void bsp_photo_sw_spi_change_mode(void)
{

}


void bsp_photo_adc_spi_change_mode(void)
{

}
/*
 *timer interrupt->start Conv
 *EXTI (EOC) falling edge
 *EXTI Pull CSADC low
 *EXTI SPI send 2 byte to get 2 byte
 */


void bsp_photodiode_timer1_init(uint32_t period) {
    // 1. Kích hoạt clock cho Timer 1
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // 2. Tính toán giá trị prescaler và period
    // Xung nhịp APB2 = 200 MHz (giả sử không phân tần)
    // Chu kỳ 1 tick = 1 / 200 MHz = 5 ns
    // Số tick cần cho period ns = period / 5
    uint32_t ticks = period / 5; // Số tick cần cho period ns

    // Tính prescaler và auto-reload (ARR)
    uint16_t prescaler = 0;
    uint16_t arr = ticks - 1; // Giá trị ARR = số tick - 1

    // Nếu ticks lớn hơn 0xFFFF, cần điều chỉnh prescaler
    if (ticks > 0xFFFF) {
        prescaler = (ticks / 0xFFFF) + 1; // Làm tròn lên
        arr = (ticks / prescaler) - 1;
    }

    // 3. Cấu hình Timer 1
    TIM1->CR1 = 0; // Xóa thanh ghi điều khiển
    TIM1->ARR = arr; // Cài đặt giá trị auto-reload
    TIM1->PSC = prescaler; // Cài đặt prescaler
    TIM1->EGR = TIM_EGR_UG; // Tạo sự kiện update để áp dụng ngay
    TIM1->DIER |= TIM_DIER_UIE; // Kích hoạt ngắt update
    TIM1->CNT = 0;
    // 4. Cấu hình NVIC cho ngắt Timer 1
    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0); // Ưu tiên ngắt cao nhất
 //   NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

    // 5. Kích hoạt Timer 1
    //TIM1->CR1 |= TIM_CR1_CEN;
}

void bsp_photodiode_sample_start(uint32_t init_num_sample)
{
	num_sample = init_num_sample;
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

	    // 5. Kích hoạt Timer 1
    TIM1->CR1 |= TIM_CR1_CEN;
}

/*
 * start the pre-sampling phase
 */
void bsp_photodiode_start_pre_sampling(uint32_t num_sample)
{
	Photodiode_DMA_config(num_sample);
	Photodiode_timer_start(num_sample);
}

// Hàm xử lý ngắt Timer 1
void TIM1_UP_TIM10_IRQHandler(void) {

        // Tạo xung trên PD10
	    GPIOD->BSRR = GPIO_BSRR_BS_9; // Đặt CS lên 1
        GPIOD->BSRR = GPIO_BSRR_BR_10; // Đặt PD10 xuống 0

            __NOP(); // Lệnh NOP để đảm bảo trễ
            __NOP(); // Lệnh NOP để đảm bảo trễ
            __NOP(); // Lệnh NOP để đảm bảo trễ
            __NOP(); // Lệnh NOP để đảm bảo trễ
            GPIOD->BSRR = GPIO_BSRR_BR_9; // CS=0
            LL_SPI_TransmitData16(SPI2, 0xAAAA);
        GPIOD->BSRR = GPIO_BSRR_BS_10; // Đặt PD10 lên 1
        TIM1->SR &= ~TIM_SR_UIF; // Xóa cờ ngắt


}
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_11) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_11);
    /* USER CODE BEGIN LL_EXTI_LINE_11 */

    /* USER CODE END LL_EXTI_LINE_11 */
  }
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}
