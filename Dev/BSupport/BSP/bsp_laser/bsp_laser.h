/*
 * bsp_laser.h
 *
 *  Created on: Jun 25, 2025
 *      Author: Admin
 */

#ifndef BSUPPORT_BSP_BSP_LASER_BSP_LASER_H_
#define BSUPPORT_BSP_BSP_LASER_BSP_LASER_H_

#include "board.h"
#include "adg1414.h"
#include "mcp4902.h"



void bsp_laser_set_spi_prescaler(uint32_t Prescaler);
void bsp_laser_set_spi_mode(spi_mode_t spi_mode);

void bsp_laser_int_set_current(uint32_t percent);
void bsp_laser_ext_set_current(uint32_t percent);

void bsp_laser_init(void);
void bsp_laser_set_current(uint32_t id, uint32_t percent);
void bsp_laser_int_switch_on(uint32_t channel_idx);
void bsp_laser_int_switch_off_all(void);
void bsp_laser_ext_switch_on(uint32_t channel_idx);
void bsp_laser_ext_switch_off_all(void);
#endif /* BSUPPORT_BSP_BSP_LASER_BSP_LASER_H_ */
