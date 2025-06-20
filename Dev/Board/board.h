/*
 * board.h
 *
 *  Created on: Nov 21, 2024
 *      Author: SANG HUYNH
 */

#ifndef BOARD_BOARD_H_
#define BOARD_BOARD_H_
#include <stm32f765xx.h>

#include "adg1414.h"
#include "ads8327.h"
#include "mcp4902.h"

//*****************************************************************************
// EXP_UART_RS485
//*****************************************************************************
#define EXP_UART_RS485_HANDLE			USART1
#define EXP_UART_RS495_IRQ				USART1_IRQn


//*****************************************************************************
// EXP_UART_CONSOLE
//*****************************************************************************
#define EXP_UART_CONSOLE_HANDLE			USART6
#define EXP_UART_CONSOLE_IRQ			USART6_IRQn

//*****************************************************************************
// OBC_EXP_UART
//*****************************************************************************
#define EXP_UART_OBC_HANDLE				UART7
#define EXP_UART_OBC_IRQ				UART7_IRQn


//*****************************************************************************
// I2C_SENSOR
//*****************************************************************************
#define EXP_I2C_SENSOR_HANDLE			I2C1
#define EXP_I2C_SENSOR_IRQ				I2C1_EV_IRQn


//*****************************************************************************
// I2C_COPC
//*****************************************************************************
#define EXP_I2C_OBC_HANDLE				I2C2
#define EXP_I2C_OBC_IRQ					I2C2_EV_IRQn


//*****************************************************************************
// SPI_TEC
//*****************************************************************************
#define SPI_TEC				SPI3

extern struct lt8722_dev tec_0;
extern struct lt8722_dev tec_1;
extern struct lt8722_dev tec_2;
extern struct lt8722_dev tec_3;
extern struct lt8722_dev * tec_table[];
extern struct mb85rs2mt_dev fram;

extern MCP4902_Device_t DAC_device;
extern ADG1414_Device_t laser_int;
extern ADG1414_Device_t laser_ext;
//extern ADC_DMA_Device_t laser_adc;
extern ADG1414_Device_t photo_sw;
extern ADS8327_Device_t photo_adc;

#endif /* BOARD_BOARD_H_ */
