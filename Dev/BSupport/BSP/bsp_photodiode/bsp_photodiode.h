/*
 * bsp_photodiode.h
 *
 *  Created on: Jun 24, 2025
 *      Author: Admin
 */

#ifndef BSUPPORT_BSP_BSP_PHOTODIODE_BSP_PHOTODIODE_H_
#define BSUPPORT_BSP_BSP_PHOTODIODE_BSP_PHOTODIODE_H_

#include "board.h"
#include "ads8327.h"
void bsp_photodiode_timer1_init(uint32_t period) ;
void bsp_photodiode_sample_start(uint32_t init_num_sample);
extern ADS8327_Device_t photo_adc;
#endif /* BSUPPORT_BSP_BSP_PHOTODIODE_BSP_PHOTODIODE_H_ */
