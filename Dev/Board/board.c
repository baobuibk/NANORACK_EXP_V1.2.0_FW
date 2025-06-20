/*
 * board.c
 *
 *  Created on: Apr 4, 2025
 *      Author: HTSANG
 */


#include "board.h"
#include "lt8722.h"
#include "adg1414.h"
#include "ads8327.h"
#include "mb85rs2mt.h"

struct lt8722_dev tec_0 = {
		  .hspi = SPI3,
		  .cs_port = TEC_1_CS_GPIO_Port,
		  .cs_pin = TEC_1_CS_Pin,
		  .en_port = TEC_1_EN_GPIO_Port,
		  .en_pin = TEC_1_EN_Pin,
		  .swen_port = TEC_1_SWEN_GPIO_Port,
		  .swen_pin = TEC_1_SWEN_Pin,
		  .start_up_sequence = 1,
		  .status = 1
};

struct lt8722_dev tec_1 = {
		  .hspi = SPI3,
		  .cs_port = TEC_2_CS_GPIO_Port,
		  .cs_pin = TEC_2_CS_Pin,
		  .en_port = TEC_2_EN_GPIO_Port,
		  .en_pin = TEC_2_EN_Pin,
		  .swen_port = TEC_2_SWEN_GPIO_Port,
		  .swen_pin = TEC_2_SWEN_Pin,
		  .start_up_sequence = 2,
		  .status = 2
};

struct lt8722_dev tec_2 = {
		  .hspi = SPI3,
		  .cs_port = TEC_3_CS_GPIO_Port,
		  .cs_pin = TEC_3_CS_Pin,
		  .en_port = TEC_3_EN_GPIO_Port,
		  .en_pin = TEC_3_EN_Pin,
		  .swen_port = TEC_3_SWEN_GPIO_Port,
		  .swen_pin = TEC_3_SWEN_Pin,
		  .start_up_sequence = 3,
		  .status = 3
};

struct lt8722_dev tec_3 = {
		  .hspi = SPI3,
		  .cs_port = TEC_4_CS_GPIO_Port,
		  .cs_pin = TEC_4_CS_Pin,
		  .en_port = TEC_4_EN_GPIO_Port,
		  .en_pin = TEC_4_EN_Pin,
		  .swen_port = TEC_4_SWEN_GPIO_Port,
		  .swen_pin = TEC_4_SWEN_Pin,
		  .start_up_sequence = 4,
		  .status = 4
};

struct lt8722_dev * tec_table[] = {&tec_0, &tec_1, &tec_2, &tec_3};

struct mb85rs2mt_dev fram = {
		  .hspi = SPI6,
		  .cs_port = FRAM_CS_GPIO_Port,
		  .cs_pin = FRAM_CS_Pin
};

MCP4902_Device_t DAC_device;
ADG1414_Device_t laser_int;
ADG1414_Device_t laser_ext;
//ADC_DMA_Device_t laser_adc;
ADG1414_Device_t photo_sw;
ADS8327_Device_t photo_adc;
