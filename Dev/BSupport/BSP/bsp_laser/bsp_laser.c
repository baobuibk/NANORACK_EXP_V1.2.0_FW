/*
 * bsp_laser.c
 *
 *  Created on: Jun 25, 2025
 *      Author: Admin
 */
#include "bsp_laser.h"

MCP4902_Device_t DAC_device;
ADG1414_Device_t laser_int;
ADG1414_Device_t laser_ext;

void bsp_laser_init(void)
{

	  MCP4902_Device_Init(&DAC_device, SPI4, LASER_DAC_CS_GPIO_Port, LASER_DAC_CS_Pin, LASER_DAC_LATCH_GPIO_Port, LASER_DAC_LATCH_Pin);
	  ADG1414_Chain_Init(&laser_int, SPI4, LASER_INT_SW_CS_GPIO_Port, LASER_INT_SW_CS_Pin, INTERNAL_CHAIN_SWITCH_NUM);
	  ADG1414_Chain_Init(&laser_ext, SPI4, LASER_EXT_SW_CS_GPIO_Port, LASER_EXT_SW_CS_Pin, EXTERNAL_CHAIN_SWITCH_NUM);
}

void bsp_laser_int_switch_on(uint32_t channel_idx)
{
	ADG1414_Chain_SwitchOn(&laser_int, channel_idx);
}

void bsp_laser_int_switch_off_all(void){
	ADG1414_Chain_SwitchAllOff(&laser_int);
}

void bsp_laser_ext_switch_on(uint32_t channel_idx)
{
	ADG1414_Chain_SwitchOn(&laser_ext, channel_idx);
}

void bsp_laser_ext_switch_off_all(void){
	ADG1414_Chain_SwitchAllOff(&laser_ext);
}
/*
 * current source has 250 ohm shunt
 * with maximum voltage of 3V, we calculate the voltage for ADC and send to ADC
 */

void bsp_laser_int_set_current(uint32_t percent)
{
	if (percent > 100) percent = 100;
	MCP4902_Set_Voltage(&DAC_device, 0, 33*percent);
}

void bsp_laser_ext_set_current(uint32_t percent)
{
	if (percent > 100) percent = 100;
	MCP4902_Set_Voltage(&DAC_device, 1, 33*percent);
}
void bsp_laser_set_current(uint32_t id, uint32_t percent)
{
	if (id ==0)  bsp_laser_int_set_current(percent);
	else bsp_laser_ext_set_current(percent);

}
