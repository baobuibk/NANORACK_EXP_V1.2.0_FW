/*
 * bsp_ntc.c
 *
 *  Created on: Jun 22, 2025
 *      Author: Admin
 */


#include "bsp_ntc.h"
#include "ntc.h"
#include "stm32f7xx_hal_dma.h"
#include "temperature_monitor.h"
#include "app_signals.h"
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern temperature_monitor_task_t temperature_monitor_task_inst ;


uint16_t	ntc_ADC_value[8] = {0,0,0,0,0,0,0,0};
uint16_t	ntc_ADC_value_average[8] = {0,0,0,0,0,0,0,0};
static uint32_t sample_count = 0;
static uint32_t first_sample = 1;

static temperature_monitor_evt_t const adc_evt = {.super = {.sig = EVT_TEMPERATURE_MONITOR_NTC_ADC_COMPLETED} };

void bsp_ntc_trigger_adc()
{


        // Cấu hình DMA để truyền dữ liệu ADC vào buffer

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ntc_ADC_value, 8);

}

int16_t bsp_ntc_get_temperature(uint32_t ntc_channel)
{
	return ntc_convert(ntc_ADC_value[ntc_channel]);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc == &hadc1)
	{
		if (first_sample)
		{
			for (uint32_t i = 0; i< 8; i++ ) ntc_ADC_value_average[i] = ntc_ADC_value[i];

		}
		else
		{
			for (uint32_t i = 0; i< 8; i++ ) ntc_ADC_value_average[i] = (ntc_ADC_value[i] + ntc_ADC_value_average[i])/2;
		}
		if (sample_count < 10) sample_count++;
		else
		{
			sample_count = 0;
			SST_Task_post(&temperature_monitor_task_inst.super, (SST_Evt *)&adc_evt); //post to temperature monitor task
		}
	}
}
