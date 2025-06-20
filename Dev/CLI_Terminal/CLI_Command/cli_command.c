/*
 * cli_command.c
 *
 *  Created on: Apr 1, 2025
 *      Author: HTSANG
 */

#include "cli_command.h"
#include "uart_driver.h"
#include "sensor_i2c.h"
#include "ntc.h"
#include "lt8722.h"
#include "temperature.h"
#include <stdlib.h>
#include <string.h>

#include "scheduler.h"
#include "k33.h"
#include "board.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct _CLI_Command_TaskContextTypedef_ {
	SCH_TASK_HANDLE taskHandle;
	SCH_TaskPropertyTypedef taskProperty;
} CLI_Command_TaskContextTypedef;

/* Private function ----------------------------------------------------------*/
static void CLI_Command_Task_Update(void);


static CLI_Command_TaskContextTypedef s_CLI_CommandTaskContext = {
		SCH_INVALID_TASK_HANDLE, 	// Will be updated by Schedular
		{ SCH_TASK_SYNC,        	// taskType;
		SCH_TASK_PRIO_0,         	// taskPriority;
		10,                      	// taskPeriodInMS;
		CLI_Command_Task_Update, 	// taskFunction;
		9 }
};

/*************************************************
 *                Private variable                 *
 *************************************************/
static uint8_t photo_index = 0;
static uint32_t samp_rate = 0;

/*************************************************
 *                Command Define                 *
 *************************************************/
#define ROUND(x) ((x) >= 0 ? (int)(x + 0.5) : (int)(x - 0.5))

static int32_t Map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static void crc16_CCITT_update(uint16_t *crc, uint16_t data)
{
    uint8_t bytes[2] = {data >> 8, data & 0xFF};
    for (uint8_t i = 0; i < 2; i++)
    {
        *crc ^= bytes[i] << 8;
        for (uint8_t j = 0; j < 8; j++)
            *crc = (*crc & 0x8000) ? (*crc << 1) ^ 0x1021 : *crc << 1;
    }
}

static void htoa(uint16_t hex_value, char *output)
{
    const char hex_chars[] = "0123456789ABCDEF";
    output[0] = hex_chars[(hex_value >> 12) & 0x0F];
    output[1] = hex_chars[(hex_value >> 8) & 0x0F];
    output[2] = hex_chars[(hex_value >> 4) & 0x0F];
    output[3] = hex_chars[hex_value & 0x0F];
    output[4] = ' ';
}

/*************************************************
 *                Command Define                 *
 *************************************************/
static void CMD_Clear_CLI(EmbeddedCli *cli, char *args, void *context);
static void CMD_Reset(EmbeddedCli *cli, char *args, void *context);

static void CMD_NTC_Get_Temp(EmbeddedCli *cli, char *args, void *context);
static void CMD_PWR_5V_Set(EmbeddedCli *cli, char *args, void *context);
static void CMD_PWR_5V_Get(EmbeddedCli *cli, char *args, void *context);
static void CMD_TEC_Init(EmbeddedCli *cli, char *args, void *context);
static void CMD_TEC_Set_Volt(EmbeddedCli *cli, char *args, void *context);
static void CMD_TEC_Get_Volt(EmbeddedCli *cli, char *args, void *context);
static void CMD_TEC_Set_Dir(EmbeddedCli *cli, char *args, void *context);
static void CMD_TEC_Get_Dir(EmbeddedCli *cli, char *args, void *context);
static void CMD_HTR_Set_Duty(EmbeddedCli *cli, char *args, void *context);
static void CMD_HTR_Get_Duty(EmbeddedCli *cli, char *args, void *context);
static void CMD_Ref_Set_Temp(EmbeddedCli *cli, char *args, void *context);
static void CMD_Ref_Get_Temp(EmbeddedCli *cli, char *args, void *context);
static void CMD_Ref_Set_NTC(EmbeddedCli *cli, char *args, void *context);
static void CMD_Ref_Get_NTC(EmbeddedCli *cli, char *args, void *context);
static void CMD_TEC_Set_Auto(EmbeddedCli *cli, char *args, void *context);
static void CMD_TEC_Get_Auto(EmbeddedCli *cli, char *args, void *context);
static void CMD_HTR_Set_Auto(EmbeddedCli *cli, char *args, void *context);
static void CMD_HTR_Get_Auto(EmbeddedCli *cli, char *args, void *context);
static void CMD_Temp_Set_Auto(EmbeddedCli *cli, char *args, void *context);
static void CMD_Temp_Get_Auto(EmbeddedCli *cli, char *args, void *context);

static void CMD_Sens_List(EmbeddedCli *cli, char *args, void *context);
static void CMD_LSMSens_Get(EmbeddedCli *cli, char *args, void *context);
static void CMD_H3LSens_Get(EmbeddedCli *cli, char *args, void *context);
static void CMD_BMESens_Get(EmbeddedCli *cli, char *args, void *context);
static void CMD_H250Sens_Get(EmbeddedCli *cli, char *args, void *context);
static void CMD_K33Sens_Get(EmbeddedCli *cli, char *args, void *context);

// Laser Photo
static void CMD_Set_Laser(EmbeddedCli *cli, char *args, void *context);
static void CMD_Get_Current(EmbeddedCli *cli, char *args, void *context);
static void CMD_PD_Get(EmbeddedCli *cli, char *args, void *context);
static void CMD_Sample_Set_PD(EmbeddedCli *cli, char *args, void *context);
static void CMD_Sample_Set_Rate(EmbeddedCli *cli, char *args, void *context);
static void CMD_Sample_Trig(EmbeddedCli *cli, char *args, void *context);
static void CMD_Sample_Status_Get(EmbeddedCli *cli, char *args, void *context);
static void CMD_Sample_Get(EmbeddedCli *cli, char *args, void *context);
static void CMD_Sample_Get_Char(EmbeddedCli *cli, char *args, void *context);
static void CMD_Sample_Get_Buf(EmbeddedCli *cli, char *args, void *context);
static void CMD_Sample_Get_Buf_Char(EmbeddedCli *cli, char *args, void *context);

/*************************************************
 *                 Command  Array                *
 *************************************************/
// Guide: Command bindings are declared in the following order:
// { category, name, help, tokenizeArgs, context, binding }
// - category: Command group; set to NULL if grouping is not needed.
// - name: Command name (required)
// - help: Help string describing the command (required)
// - tokenizeArgs: Set to true to automatically split arguments when the command is called.
// - context: Pointer to a command-specific context; can be NULL.
// - binding: Callback function that handles the command.

static const CliCommandBinding cliStaticBindings_internal[] = {
    // Common
    { "Ultis", "help",         "Print list of all available CLI commands [Firmware: 1]", false,  NULL, CMD_Help },
    { "Ultis", "cls",          "Clear the console output screen",                        false,  NULL, CMD_Clear_CLI },
    { "Ultis", "reset",        "Perform MCU software reset",                             false,  NULL, CMD_Reset },

    // NTC
    { "NTC",   "ntc_get_temp", "Read temperature value from NTC sensor [ch: 0-7, a=all]", true,   NULL, CMD_NTC_Get_Temp },

    // Power
    { "PWR",   "pwr_5v_set",   "Turn ON/OFF 5V power supply [0:OFF / 1:ON]",             true,   NULL, CMD_PWR_5V_Set },
    { "PWR",   "pwr_5v_get",   "Read current state of 5V power supply",                  false,  NULL, CMD_PWR_5V_Get },

    // TEC
    { "TEC",   "tec_init",     "Initialize TEC driver for specified channel [0-3, a=all]", true,  NULL, CMD_TEC_Init },
    { "TEC",   "tec_volt_set", "Set voltage for TEC channel: tec[0-3]_volt [500~2500mV]", true,  NULL, CMD_TEC_Set_Volt },
    { "TEC",   "tec_volt_get", "Read current output voltage of TEC [0-3, a=all]",         true,  NULL, CMD_TEC_Get_Volt },
    { "TEC",   "tec_dir_set",  "Set direction of TEC [0-3]: 0=Cooling, 1=Heating",        true,  NULL, CMD_TEC_Set_Dir },
    { "TEC",   "tec_dir_get",  "Read direction setting of TEC [0-3, a=all]",              true,  NULL, CMD_TEC_Get_Dir },

    // Heater
    { "Heater", "htr_duty_set","Set duty cycle for heater [0-3]: [0~100%]",              true,  NULL, CMD_HTR_Set_Duty },
    { "Heater", "htr_duty_get","Read current duty cycle setting of heater [0-3, a=all]", true,  NULL, CMD_HTR_Get_Duty },

    // Reference temperature
    { "Ref",   "ref_temp_set", "Set reference temperature for control logic (°C)",       true,  NULL, CMD_Ref_Set_Temp },
    { "Ref",   "ref_temp_get", "Read current reference temperature setting",              true,  NULL, CMD_Ref_Get_Temp },
    { "Ref",   "ref_ntc_set",  "Select NTC channel used for temperature feedback [0-7]",  true,  NULL, CMD_Ref_Set_NTC },
    { "Ref",   "ref_ntc_get",  "Get currently selected NTC channel for control",          true,  NULL, CMD_Ref_Get_NTC },

    // Automatic control
    { "Auto",  "auto_tec_set", "Enable/disable TECs for auto control [tecX_en 0/1]",      true,  NULL, CMD_TEC_Set_Auto },
    { "Auto",  "auto_tec_get", "List enabled TEC channels for auto control [0-3, a]",     true,  NULL, CMD_TEC_Get_Auto },
    { "Auto",  "auto_htr_set", "Enable/disable heaters for auto control [htrX_en 0/1]",   true,  NULL, CMD_HTR_Set_Auto },
    { "Auto",  "auto_htr_get", "List enabled heater channels for auto control [0-3, a]",  true,  NULL, CMD_HTR_Get_Auto },
    { "Auto",  "auto_temp_set","Enable/disable full temperature control routine [0/1]",   true,  NULL, CMD_Temp_Set_Auto },
    { "Auto",  "auto_temp_get","Read current state of temperature control routine",       true,  NULL, CMD_Temp_Get_Auto },

    // Sensors
    { "Sensor","sens_list",    "List all available connected sensors",                    true,  NULL, CMD_Sens_List },
    { "Sensor","lsm_sens_get", "Read LSM6DSOX data [0=all, 1=acc, 2=gyro]",               true,  NULL, CMD_LSMSens_Get },
    { "Sensor","h3l_sens_get", "Read high-g acceleration data from H3LIS331 sensor",      true,  NULL, CMD_H3LSens_Get },
    { "Sensor","bme_sens_get", "Read BME280 sensor [0=all, 1=temp, 2=RH, 3=pressure]",     true,  NULL, CMD_BMESens_Get },
    { "Sensor","h250_sens_get","Read CO₂ concentration from H-250(G)-3V sensor",          false, NULL, CMD_H250Sens_Get },
    { "Sensor","k33_sens_get", "Read CO₂ concentration from K33 ICB-F 10% sensor",        false, NULL, CMD_K33Sens_Get },

	// Laser Photo
    { NULL, "set_laser",    "format: set_laser [int/ext] [laser_index] [dac_val]",             true, NULL, CMD_Set_Laser },
    { NULL, "get_current",  "format: get_current [int/ext]",                                   true, NULL, CMD_Get_Current },
    { NULL, "pd_get",       "format: pd_get [pd_index]",                                       true, NULL, CMD_PD_Get },
    { NULL, "sp_set_pd",    "format: sp_set_pd [photo_index]",                                 true, NULL, CMD_Sample_Set_PD },
    { NULL, "sp_set_rate",  "format: sp_set_rate [sampling_rate] [num_samples]",               true, NULL, CMD_Sample_Set_Rate },
    { NULL, "sp_trig",      "format: sp_trig",                                                 true, NULL, CMD_Sample_Trig },
    { NULL, "sp_status",    "format: sp_status",                                               true, NULL, CMD_Sample_Status_Get },
    { NULL, "sp_get",       "format: sp_get [num_samples]",                                    true, NULL, CMD_Sample_Get },
    { NULL, "sp_get_c",     "format: sp_get_c [num_samples]",                                  true, NULL, CMD_Sample_Get_Char },
    { NULL, "sp_get_buf",   "format: sp_get_buf",                                              true, NULL, CMD_Sample_Get_Buf },
    { NULL, "sp_get_buf_c", "format: sp_get_buf_c",                                            true, NULL, CMD_Sample_Get_Buf_Char },
};

/*************************************************
 *             Command List Function             *
 *************************************************/
void CLI_Command_Init(USART_TypeDef *handle_uart) {
	SystemCLI_Init();
	UART_Driver_SendString(handle_uart, "\r\nEXP_V110:~ HELLO! $ \r\n");
}

void CLI_Command_CreateTask(void) {
	SCH_TASK_CreateTask(&s_CLI_CommandTaskContext.taskHandle, &s_CLI_CommandTaskContext.taskProperty);
}

static void CLI_Command_Task_Update(void) {
	char rxData;
	if (UART_Driver_IsDataAvailable(EXP_UART_CONSOLE_HANDLE)) {
		rxData = UART_Driver_Read(EXP_UART_CONSOLE_HANDLE);
		embeddedCliReceiveChar(get_EXPConsole_CliPointer(), (char)rxData);
		embeddedCliProcess(get_EXPConsole_CliPointer());
	}
}
static void CMD_Clear_CLI(EmbeddedCli *cli, char *args, void *context) {
    char buffer[10];
    snprintf(buffer, sizeof(buffer), "\33[2J");
    embeddedCliPrint(cli, buffer);
}

static void CMD_Reset(EmbeddedCli *cli, char *args, void *context) {
	NVIC_SystemReset();
    embeddedCliPrint(cli, "");
}

static void CMD_NTC_Get_Temp(EmbeddedCli *cli, char *args, void *context) {
    // TODO: Implement NTC temperature get logic
	const char *arg1 = embeddedCliGetToken(args, 1);
	if (*arg1 == 'a' || *arg1 == '\0') {
		int16_t temp = 0;
		char buffer[80] = "ntc:";
		char tempo_buf[10];
		NTC_get_temperature(NTC_Temperature);
		for (uint8_t channel = 0; channel < 8; channel++) {
			temp = NTC_Temperature[channel];
			if (temp == 0x7FFF) {
				snprintf(tempo_buf, sizeof(tempo_buf), "f ");
			} else {
				snprintf(tempo_buf, sizeof(tempo_buf), "%i ", temp);
			}
			strcat(buffer, tempo_buf);
		}

		size_t len = strlen(buffer);
		if (len >= 1 && buffer[len - 1] == ' ') {
		    buffer[len - 1] = '\0';
		}

		strcat(buffer, "(C)");
		embeddedCliPrint(cli, buffer);
	}
	else if (*arg1 == '0' || *arg1 == '1' || *arg1 == '2' || *arg1 == '3' || *arg1 == '4' || *arg1 == '5' || *arg1 == '6' || *arg1 == '7') {
		int channel = atoi(arg1);
		char buffer[80];
		NTC_get_temperature(NTC_Temperature);
		int16_t temp = NTC_Temperature[channel];
		if (temp == 0x7FFF) {
			snprintf(buffer, sizeof(buffer), "ntc[%d]:f", channel);
		} else {
			snprintf(buffer, sizeof(buffer), "ntc[%d]:%i(C)", channel, temp);
		}
		embeddedCliPrint(cli, buffer);
	}
	embeddedCliPrint(cli, "");
}

static void CMD_PWR_5V_Set(EmbeddedCli *cli, char *args, void *context) {
	const char *arg1 = embeddedCliGetToken(args, 1);
	s_Temperature_CurrentState.Pwr_status = (uint8_t)atoi(arg1);
	if (s_Temperature_CurrentState.Pwr_status) {
		LL_GPIO_SetOutputPin(EF_5_EN_GPIO_Port, EF_5_EN_Pin);
		embeddedCliPrint(cli, "Pwr 5V: on");
	}
	else {
		LL_GPIO_ResetOutputPin(EF_5_EN_GPIO_Port, EF_5_EN_Pin);
		embeddedCliPrint(cli, "Pwr 5V: off");
	}
	embeddedCliPrint(cli, "");
}

static void CMD_PWR_5V_Get(EmbeddedCli *cli, char *args, void *context) {
	if (s_Temperature_CurrentState.Pwr_status) {
		embeddedCliPrint(cli, "Pwr 5V: on");
	}
	else {
		embeddedCliPrint(cli, "Pwr 5V: off");
	}
	embeddedCliPrint(cli, "");
}

static void CMD_TEC_Init(EmbeddedCli *cli, char *args, void *context) {
    // TODO: Implement TEC initialization logic
	const char *arg1 = embeddedCliGetToken(args, 1);
	char buffer[60];
	int8_t tec_init_channel = 0;
	struct lt8722_dev *p_tec_dev;
	struct lt8722_dev *p_tec_dev_table[] = {&tec_0, &tec_1, &tec_2, &tec_3};
	if (*arg1 == 'a' || *arg1 == '\0') {
		/* Init TEC 0 -> 3 */
		for (uint8_t channel = 0; channel < 4; channel++) {
			p_tec_dev = p_tec_dev_table[channel];
			tec_init_channel = lt8722_init(p_tec_dev);
			LL_mDelay(10);
			// if init is success
			if (!tec_init_channel) {
				lt8722_set_swen_req(p_tec_dev, LT8722_SWEN_REQ_DISABLED);
				snprintf(buffer, sizeof(buffer), "Tec %d init success", channel);
			}
			else
				snprintf(buffer, sizeof(buffer), "Tec %d init fail", channel);
			embeddedCliPrint(cli, buffer);
		}
	}
	else if (*arg1 == '0' || *arg1 == '1' || *arg1 == '2' || *arg1 == '3') {
		int channel = atoi(arg1);
		p_tec_dev = p_tec_dev_table[channel];
		tec_init_channel = lt8722_init(p_tec_dev);
		LL_mDelay(10);
		// if init is success
		if (!tec_init_channel) {
			lt8722_set_swen_req(p_tec_dev, LT8722_SWEN_REQ_DISABLED);
			snprintf(buffer, sizeof(buffer), "Tec %d init success", channel);
		}
		else
			snprintf(buffer, sizeof(buffer), "Tec %d init fail", channel);
		embeddedCliPrint(cli, buffer);
	}
	embeddedCliPrint(cli, "");
}

static void CMD_TEC_Set_Volt(EmbeddedCli *cli, char *args, void *context) {
    // TODO: Implement TEC voltage set logic
	const char *arg1 = embeddedCliGetToken(args, 1);
	const char *arg2 = embeddedCliGetToken(args, 2);
	const char *arg3 = embeddedCliGetToken(args, 3);
	const char *arg4 = embeddedCliGetToken(args, 4);
	int volt[4];
	volt[0] = atoi(arg1);
	volt[1] = atoi(arg2);
	volt[2] = atoi(arg3);
	volt[3] = atoi(arg4);
	char buffer[100];
	for (uint8_t i = 0; i < 4; i++) {
		temperature_set_tec_vol(i, volt[i]);
		if (volt[i] > 3000)
			snprintf(buffer, sizeof(buffer), "Tec[%d]: set %i mV OverVolt --> Tec[%d]: set 3000 mV", i, volt[i], i);
		else
			snprintf(buffer, sizeof(buffer), "Tec[%d]: set %i mV", i, volt[i]);
		embeddedCliPrint(cli, buffer);
	}
	embeddedCliPrint(cli, "");
}

static void CMD_TEC_Get_Volt(EmbeddedCli *cli, char *args, void *context) {
    // TODO: Implement TEC voltage get logic
	const char *arg1 = embeddedCliGetToken(args, 1);
	char buffer[60];
	uint16_t vol_set = 0;
	uint16_t vol_adc = 0;
	if (*arg1 == 'a' || *arg1 == '\0') {
		char buffer[80];
		for (uint8_t i = 0; i < 4; i++) {
			vol_set = temperature_get_tec_vol_set(i);
			vol_adc = temperature_get_tec_vol_adc(i);
			snprintf(buffer, sizeof(buffer), "Tec[%d]: set %imV, ADC %imV", i, vol_set, vol_adc);
			embeddedCliPrint(cli, buffer);
		}
	}
	else if (*arg1 == '0' || *arg1 == '1' || *arg1 == '2' || *arg1 == '3') {
		int channel = atoi(arg1);
		vol_set = temperature_get_tec_vol_set(channel);
		vol_adc = temperature_get_tec_vol_adc(channel);
		snprintf(buffer, sizeof(buffer), "Tec[%d]: set %imV, ADC %imV", channel, vol_set, vol_adc);
		embeddedCliPrint(cli, buffer);
	}
	embeddedCliPrint(cli, "");
}

static void CMD_TEC_Set_Dir(EmbeddedCli *cli, char *args, void *context) {
    // TODO: Implement TEC direction set logic
	const char *arg1 = embeddedCliGetToken(args, 1);
	const char *arg2 = embeddedCliGetToken(args, 2);
	const char *arg3 = embeddedCliGetToken(args, 3);
	const char *arg4 = embeddedCliGetToken(args, 4);
	tec_dir_t dir_0 = atoi(arg1) ? TEC_HEAT : TEC_COOL;
	tec_dir_t dir_1 = atoi(arg2) ? TEC_HEAT : TEC_COOL;
	tec_dir_t dir_2 = atoi(arg3) ? TEC_HEAT : TEC_COOL;
	tec_dir_t dir_3 = atoi(arg4) ? TEC_HEAT : TEC_COOL;
	temperature_set_tec_dir(dir_0, dir_1, dir_2, dir_3);
	if (dir_0 == TEC_COOL) embeddedCliPrint(cli, "TEC 0 set cool mode");
	else embeddedCliPrint(cli, "TEC 0 set heat mode");
	if (dir_1 == TEC_COOL) embeddedCliPrint(cli, "TEC 1 set cool mode");
	else embeddedCliPrint(cli, "TEC 1 set heat mode");
	if (dir_2 == TEC_COOL) embeddedCliPrint(cli, "TEC 2 set cool mode");
	else embeddedCliPrint(cli, "TEC 2 set heat mode");
	if (dir_3 == TEC_COOL) embeddedCliPrint(cli, "TEC 3 set cool mode");
	else embeddedCliPrint(cli, "TEC 3 set heat mode");
	embeddedCliPrint(cli, "");
}

static void CMD_TEC_Get_Dir(EmbeddedCli *cli, char *args, void *context) {
    // TODO: Implement TEC direction get logic
	const char *arg1 = embeddedCliGetToken(args, 1);
	tec_dir_t dir[4] = {TEC_COOL, TEC_COOL, TEC_COOL, TEC_COOL};
	temperature_get_tec_dir(&dir[0], &dir[1], &dir[2], &dir[3]);
	if (*arg1 == 'a' || *arg1 == '\0') {
		if (dir[0] == TEC_COOL) embeddedCliPrint(cli, "TEC 0 is cool mode");
		else embeddedCliPrint(cli, "TEC 0 is heat mode");
		if (dir[1] == TEC_COOL) embeddedCliPrint(cli, "TEC 1 is cool mode");
		else embeddedCliPrint(cli, "TEC 1 is heat mode");
		if (dir[2] == TEC_COOL) embeddedCliPrint(cli, "TEC 2 is cool mode");
		else embeddedCliPrint(cli, "TEC 2 is heat mode");
		if (dir[3] == TEC_COOL) embeddedCliPrint(cli, "TEC 3 is cool mode");
		else embeddedCliPrint(cli, "TEC 3 is heat mode");
	}
	else if (*arg1 == '0' || *arg1 == '1' || *arg1 == '2' || *arg1 == '3') {
		char buffer[40];
		int channel = atoi(arg1);
		if (dir[channel] == TEC_COOL)
			snprintf(buffer, sizeof(buffer), "TEC %d is cool mode", channel);
		else
			snprintf(buffer, sizeof(buffer), "TEC %d is heat mode", channel);
		embeddedCliPrint(cli, buffer);
	}
	embeddedCliPrint(cli, "");
}

static void CMD_HTR_Set_Duty(EmbeddedCli *cli, char *args, void *context) {
    // TODO: Implement Heater duty cycle set logic
	const char *arg1 = embeddedCliGetToken(args, 1);
	const char *arg2 = embeddedCliGetToken(args, 2);
	const char *arg3 = embeddedCliGetToken(args, 3);
	const char *arg4 = embeddedCliGetToken(args, 4);
	int duty[4];
	duty[0] = atoi(arg1);
	duty[1] = atoi(arg2);
	duty[2] = atoi(arg3);
	duty[3] = atoi(arg4);
	char buffer[80];
	for (uint8_t i = 0; i < 4; i++) {
		if (duty[i] > 100) duty[i] = 100;
		temperature_set_heater_duty(i, duty[i]);
		snprintf(buffer, sizeof(buffer), "Heater duty[%d]: %i%%", i, duty[i]);
		embeddedCliPrint(cli, buffer);
	}
	embeddedCliPrint(cli, "");
}

static void CMD_HTR_Get_Duty(EmbeddedCli *cli, char *args, void *context) {
    // TODO: Implement Heater duty cycle get logic
	const char *arg1 = embeddedCliGetToken(args, 1);
	char buffer[60];
	uint8_t duty;
	if (*arg1 == 'a' || *arg1 == '\0') {
		for (uint8_t i = 0; i < 4; i++) {
			duty = temperature_get_heater_duty(i);
			snprintf(buffer, sizeof(buffer), "Heater duty[%d]: %i%%", i, duty);
			embeddedCliPrint(cli, buffer);
		}
	}
	else if (*arg1 == '0' || *arg1 == '1' || *arg1 == '2' || *arg1 == '3') {
		int channel = atoi(arg1);
		duty = temperature_get_heater_duty(channel);
		snprintf(buffer, sizeof(buffer), "Heater duty[%d]: %i%%", channel, duty);
		embeddedCliPrint(cli, buffer);
	}
	embeddedCliPrint(cli, "");
}

static void CMD_Ref_Set_Temp(EmbeddedCli *cli, char *args, void *context) {
    // TODO: Implement reference temperature set logic
	const char *arg1 = embeddedCliGetToken(args, 1);
	int setpoint = atoi(arg1);
	char buffer[40];
	temperature_set_setpoint(setpoint);
	snprintf(buffer, sizeof(buffer), "Reference Temperature: %.2f *C", (float)setpoint/10);
	embeddedCliPrint(cli, buffer);
	embeddedCliPrint(cli, "");
}

static void CMD_Ref_Get_Temp(EmbeddedCli *cli, char *args, void *context) {
    // TODO: Implement reference temperature get logic
	char buffer[60];
	int16_t setpoint = temperature_get_setpoint();
	snprintf(buffer, sizeof(buffer), "Reference Temperature: %.2f *C", (float)setpoint/10);
	embeddedCliPrint(cli, buffer);
	embeddedCliPrint(cli, "");
}

static void CMD_Ref_Set_NTC(EmbeddedCli *cli, char *args, void *context) {
    // TODO: Implement reference NTC set logic
	const char *arg1 = embeddedCliGetToken(args, 1);
	int NTC_Ref = atoi(arg1);
	temperature_set_ntc_ref(NTC_Ref);
	char buffer[60];
	snprintf(buffer, sizeof(buffer), "NTC Ref is %d", NTC_Ref);
	embeddedCliPrint(cli, buffer);
	embeddedCliPrint(cli, "");
}

static void CMD_Ref_Get_NTC(EmbeddedCli *cli, char *args, void *context) {
    // TODO: Implement reference NTC get logic
	uint8_t NTC_Ref = 0;
	temperature_get_ntc_ref(&NTC_Ref);
	char buffer[60];
	snprintf(buffer, sizeof(buffer), "NTC Ref is %d", NTC_Ref);
	embeddedCliPrint(cli, buffer);
	embeddedCliPrint(cli, "");
}

static void CMD_TEC_Set_Auto(EmbeddedCli *cli, char *args, void *context) {
    // TODO: Implement TEC auto mode set logic
	const char *arg1 = embeddedCliGetToken(args, 1);
	const char *arg2 = embeddedCliGetToken(args, 2);
	const char *arg3 = embeddedCliGetToken(args, 3);
	const char *arg4 = embeddedCliGetToken(args, 4);
	uint8_t tec_0_en = atoi(arg1);
	uint8_t tec_1_en = atoi(arg2);
	uint8_t tec_2_en = atoi(arg3);
	uint8_t tec_3_en = atoi(arg4);
	temperature_set_tec_auto(tec_0_en, tec_1_en, tec_2_en, tec_3_en);
	char buffer[60];
	if (tec_0_en) {
		snprintf(buffer, sizeof(buffer), "TEC 0 is ena");
		embeddedCliPrint(cli, buffer);
	}
	if (tec_1_en) {
		snprintf(buffer, sizeof(buffer), "TEC 1 is ena");
		embeddedCliPrint(cli, buffer);
	}
	if (tec_2_en) {
		snprintf(buffer, sizeof(buffer), "TEC 2 is ena");
		embeddedCliPrint(cli, buffer);
	}
	if (tec_3_en) {
		snprintf(buffer, sizeof(buffer), "TEC 3 is ena");
		embeddedCliPrint(cli, buffer);
	}
	embeddedCliPrint(cli, "");
}

static void CMD_TEC_Get_Auto(EmbeddedCli *cli, char *args, void *context) {
    // TODO: Implement TEC auto mode get logic
	const char *arg1 = embeddedCliGetToken(args, 1);
	uint8_t tec_en[4];
	temperature_get_tec_auto(&tec_en[0], &tec_en[1], &tec_en[2], &tec_en[3]);
	char buffer[60];
	if (*arg1 == 'a' || *arg1 == '\0') {
		for (uint8_t channel = 0; channel < 4; channel++) {
			if (tec_en[channel])
				snprintf(buffer, sizeof(buffer), "TEC %d is ena", channel);
			else
				snprintf(buffer, sizeof(buffer), "TEC %d is dis", channel);
			embeddedCliPrint(cli, buffer);
		}
	}
	else if (*arg1 == '0' || *arg1 == '1' || *arg1 == '2' || *arg1 == '3') {
		int channel = atoi(arg1);
		if (tec_en[channel])
			snprintf(buffer, sizeof(buffer), "TEC %d is ena", channel);
		else
			snprintf(buffer, sizeof(buffer), "TEC %d is dis", channel);
		embeddedCliPrint(cli, buffer);
	}
	embeddedCliPrint(cli, "");
}

static void CMD_HTR_Set_Auto(EmbeddedCli *cli, char *args, void *context) {
    // TODO: Implement Heater auto mode set logic
	const char *arg1 = embeddedCliGetToken(args, 1);
	const char *arg2 = embeddedCliGetToken(args, 2);
	const char *arg3 = embeddedCliGetToken(args, 3);
	const char *arg4 = embeddedCliGetToken(args, 4);
	uint8_t htr_en[4] = {atoi(arg1), atoi(arg2), atoi(arg3), atoi(arg4)};
	temperature_set_heater_auto(htr_en[0], htr_en[1], htr_en[2], htr_en[3]);
	char buffer[60];
	for (uint8_t channel = 0; channel < 4; channel++) {
		if (htr_en[channel]) {
			snprintf(buffer, sizeof(buffer), "Heater %d is ena", channel);
			embeddedCliPrint(cli, buffer);
		}
	}
	embeddedCliPrint(cli, "");
}

static void CMD_HTR_Get_Auto(EmbeddedCli *cli, char *args, void *context) {
    // TODO: Implement Heater auto mode get logic
	const char *arg1 = embeddedCliGetToken(args, 1);
	uint8_t htr_en[4];
	temperature_get_heater_auto(&htr_en[0], &htr_en[1], &htr_en[2], &htr_en[3]);
	char buffer[60];
	if (*arg1 == 'a' || *arg1 == '\0') {
		for (uint8_t channel = 0; channel < 4; channel++) {
			if (htr_en[channel])
				snprintf(buffer, sizeof(buffer), "Heater %d is ena", channel);
			else
				snprintf(buffer, sizeof(buffer), "Heater %d is dis", channel);
			embeddedCliPrint(cli, buffer);
		}
	}
	else if (*arg1 == '0' || *arg1 == '1' || *arg1 == '2' || *arg1 == '3') {
		int channel = atoi(arg1);
		if (htr_en[channel])
			snprintf(buffer, sizeof(buffer), "Heater %d is ena", channel);
		else
			snprintf(buffer, sizeof(buffer), "Heater %d is dis", channel);
		embeddedCliPrint(cli, buffer);
	}
	embeddedCliPrint(cli, "");
}

static void CMD_Temp_Set_Auto(EmbeddedCli *cli, char *args, void *context) {
    // TODO: Implement auto temperature set logic
	const char *arg1 = embeddedCliGetToken(args, 1);
	uint8_t Temp_auto = atoi(arg1)? 1: 0;
	temperature_set_auto_ctrl(Temp_auto);
	if (Temp_auto)
		embeddedCliPrint(cli, "Temp is auto ctrl");
	else
		embeddedCliPrint(cli, "Temp isn't auto ctrl");
	embeddedCliPrint(cli, "");
}

static void CMD_Temp_Get_Auto(EmbeddedCli *cli, char *args, void *context) {
    // TODO: Implement auto temperature get logic
	uint8_t Temp_auto = 0;
	temperature_get_auto_ctrl(&Temp_auto);
	if (Temp_auto)
		embeddedCliPrint(cli, "Temp is auto ctrl");
	else
		embeddedCliPrint(cli, "Temp isn't auto ctrl");
	embeddedCliPrint(cli, "");
}


static void CMD_Sens_List(EmbeddedCli *cli, char *args, void *context) {
	// TODO:
	char buffer[100];
	NTC_get_temperature(NTC_Temperature);
	int16_t temp;
	for (uint8_t channel = 0; channel < 8; channel++) {
		temp = NTC_Temperature[channel];
		if (temp != 0x7FFF) {
			Sensor_list.ntc = 1;
			break;
		}
	}
	Sensor_I2C_Init();
	strcpy(buffer, "sensor:");
	if (Sensor_list.ntc) {
		strcat(buffer, "ntc,");
	}
	if (Sensor_list.lsm) {
		strcat(buffer, "lsm,");
	}
	if (Sensor_list.bmp) {
		strcat(buffer, "bmp,");
	}
	if (Sensor_list.bme) {
		strcat(buffer, "bme,");
	}
	if (Sensor_list.h3l) {
		strcat(buffer, "h3l,");
	}
	if (Sensor_list.h250) {
		strcat(buffer, "h250,");
	}
	if (Sensor_list.k33) {
		strcat(buffer, "k33,");
	}
	if (Sensor_list.sfc) {
		strcat(buffer, "sfc,");
	}

	size_t len = strlen(buffer);
	if (len >= 1 && buffer[len - 1] == ',') {
	    buffer[len - 1] = '\0';
	}

	embeddedCliPrint(cli, buffer);
	embeddedCliPrint(cli, "");
}

static void CMD_LSMSens_Get(EmbeddedCli *cli, char *args, void *context) {
    // TODO: Implement LSM sensor get logic
	LSM6DSOX_Read_Data(&LSM6DSOX_Data);
    char buffer[80];

    const char *arg1 = embeddedCliGetToken(args, 1);
	if (*arg1 == 'a' || *arg1 == '\0')
		snprintf(buffer, sizeof(buffer), "accel:%d %d %d(g),gyro:%d %d %d(dps)",
								LSM6DSOX_Data.Accel.x, LSM6DSOX_Data.Accel.y, LSM6DSOX_Data.Accel.z,
								LSM6DSOX_Data.Gyro.x, LSM6DSOX_Data.Gyro.y, LSM6DSOX_Data.Gyro.z);
	else if (*arg1 == '0')
		snprintf(buffer, sizeof(buffer), "accel:%d %d %d(g)", LSM6DSOX_Data.Accel.x, LSM6DSOX_Data.Accel.y, LSM6DSOX_Data.Accel.z);
	else if (*arg1 == '1')
		snprintf(buffer, sizeof(buffer), "gyro:%d %d %d(dps)", LSM6DSOX_Data.Gyro.x, LSM6DSOX_Data.Gyro.y, LSM6DSOX_Data.Gyro.z);
	embeddedCliPrint(cli, buffer);
	embeddedCliPrint(cli, "");
}

static void CMD_H3LSens_Get(EmbeddedCli *cli, char *args, void *context) {
    // TODO: Implement H3L sensor get logic
	H3LIS331DL_Get_Accel(&H3LIS331DL_Data);
	char buffer[50];
	snprintf(buffer, sizeof(buffer), "accel:%d %d %d(g)", (int16_t)H3LIS331DL_Data.x, (int16_t)H3LIS331DL_Data.y, (int16_t)H3LIS331DL_Data.z);
	embeddedCliPrint(cli, buffer);
	embeddedCliPrint(cli, "");
}

static void CMD_BMESens_Get(EmbeddedCli *cli, char *args, void *context) {
    // TODO: Implement BME sensor get logic
	BME280_Read_Data(&BME280_Data);
	char buffer[80];

	const char *arg1 = embeddedCliGetToken(args, 1);
	if (*arg1 == 'a' || *arg1 == '\0')
		snprintf(buffer, sizeof(buffer), "temp:%.2f(C),humid:%.2f(%%),press:%.2f(hPa)", BME280_Data.temperature, BME280_Data.humidity, BME280_Data.pressure);
	else if (*arg1 == '0')
		snprintf(buffer, sizeof(buffer), "temp:%.2f(C)", BME280_Data.temperature);
	else if (*arg1 == '1')
		snprintf(buffer, sizeof(buffer), "humid:%.2f(%%)", BME280_Data.humidity);
	else if (*arg1 == '2')
		snprintf(buffer, sizeof(buffer), "press:%.2f(hPa)", BME280_Data.pressure);
	embeddedCliPrint(cli, buffer);
	embeddedCliPrint(cli, "");
}

static void CMD_H250Sens_Get(EmbeddedCli *cli, char *args, void *context) {
	// TODO: Implement H250 sensor get logic
	H250_I2C_Read_Data(&H250_I2C_Data);
	char buffer[30];
	snprintf(buffer, sizeof(buffer), "co2:%.2f(%%)", (float)H250_I2C_Data/100.0f);
	embeddedCliPrint(cli, buffer);
	embeddedCliPrint(cli, "");
}

static void CMD_K33Sens_Get(EmbeddedCli *cli, char *args, void *context) {
    // TODO: Implement K33 sensor get logic
	K33_Read_Data(&K33_Data);
	char buffer[80];

	const char *arg1 = embeddedCliGetToken(args, 1);
	if (*arg1 == 'a' || *arg1 == '\0')
		snprintf(buffer, sizeof(buffer), "co2:%.2f(%%),temp:%.2f(C),humid:%.2f(%%)", (float)(K33_Data.CO2/1000.0f), (float)(K33_Data.Temp/100.0f), (float)(K33_Data.RH/100.0f));
	else if (*arg1 == '0')
		snprintf(buffer, sizeof(buffer), "co2:%.2f(%%)", (float)(K33_Data.CO2/1000.0f));
	else if (*arg1 == '1')
		snprintf(buffer, sizeof(buffer), "temp:%.2f(C)", (float)(K33_Data.Temp/100.0f));
	else if (*arg1 == '2')
		snprintf(buffer, sizeof(buffer), "humid:%.2f(%%)", (float)(K33_Data.RH/100.0f));
	embeddedCliPrint(cli, buffer);
	embeddedCliPrint(cli, "");
}

static void CMD_Set_Laser(EmbeddedCli *cli, char *args, void *context) {
    int argc = embeddedCliGetTokenCount(args);
    if (argc < 3) {
        embeddedCliPrint(cli, "Too few args. Format: set_laser [int/ext] [laser_index] [dac_val]\n");
        return;
    }

    const char *mode = embeddedCliGetToken(args, 1);
    uint8_t laser_ind = (uint8_t)atoi(embeddedCliGetToken(args, 2));

    if (strcmp(mode, "int") == 0) {
        if (laser_ind == 0) {
            if (argc != 3) {
                embeddedCliPrint(cli, "Usage: set_laser int 0\n");
                return;
            }
            MCP4902_Shutdown(&DAC_device, MCP4902_CHA);
            ADG1414_Chain_SwitchAllOff(&laser_int);
        } else if (laser_ind <= 36) {
//            if (argc != 4) {
//                embeddedCliPrint(cli, "Usage: set_laser int [1-36] [dac_val_percent]\n");
//                return;
//            }

            float percent = (float)atof(embeddedCliGetToken(args, 3));
            if (percent < 0 || percent > 100) {
                embeddedCliPrint(cli, "Percent must be 0-100\n");
                return;
            }

            uint8_t dac_val = Map(percent, 0, 100, 0, 255);
            MCP4902_Set_DAC(&DAC_device, MCP4902_CHA, dac_val);
            ADG1414_Chain_SwitchOn(&laser_int, laser_ind);
        } else {
            embeddedCliPrint(cli, "Laser index out of range for int (1-36)\n");
            return;
        }
    } else if (strcmp(mode, "ext") == 0) {
        if (laser_ind == 0) {
            if (argc != 3) {
                embeddedCliPrint(cli, "Usage: set_laser ext 0\n");
                return;
            }
            MCP4902_Shutdown(&DAC_device, MCP4902_CHB);
            ADG1414_Chain_SwitchAllOff(&laser_ext);
        } else if (laser_ind <= 8) {
//            if (argc != 4) {
//                embeddedCliPrint(cli, "Usage: set_laser ext [1-8] [dac_val_percent]\n");
//                return;
//            }

            float percent = (float)atof(embeddedCliGetToken(args, 3));
            if (percent < 0 || percent > 100) {
                embeddedCliPrint(cli, "Percent must be 0-100\n");
                return;
            }

            uint8_t dac_val = Map(percent, 0, 100, 0, 255);
            MCP4902_Set_DAC(&DAC_device, MCP4902_CHB, dac_val);
            ADG1414_Chain_SwitchOn(&laser_ext, laser_ind);
        } else {
            embeddedCliPrint(cli, "Laser index out of range for ext (1-8)\n");
            return;
        }
    } else {
        embeddedCliPrint(cli, "Invalid mode. Use 'int' or 'ext'\n");
        return;
    }

    embeddedCliPrint(cli, "Laser set successfully\n");
}

static void CMD_Get_Current(EmbeddedCli *cli, char *args, void *context) {
    return;
}

static void CMD_PD_Get(EmbeddedCli *cli, char *args, void *context) {
//    int argc = embeddedCliGetTokenCount(args);
//    if (argc != 2) {
//        embeddedCliPrint(cli, "Usage: pd_get [pd_index]\n");
//        return;
//    }

    uint8_t pd_ind = (uint8_t)atoi(embeddedCliGetToken(args, 1));
    if (pd_ind < 1 || pd_ind > 36) {
        embeddedCliPrint(cli, "Invalid pd_index. Must be in 1–36\n");
        return;
    }

//    SPI_SetDataLength(SPI2, LL_SPI_DATAWIDTH_8BIT);
//    SPI_SetPrescaler(SPI2, LL_SPI_BAUDRATEPRESCALER_DIV16);
    ADG1414_Chain_SwitchOn(&photo_sw, pd_ind);
    LL_mDelay(10);

    ADS8327_Read_Data_Polling(&photo_adc, 1000);

    char msg[64];
    snprintf(msg, sizeof(msg), "\r\nPD_index[%d]: %d\n", pd_ind, (uint16_t)photo_adc.ADC_val);
    embeddedCliPrint(cli, msg);
}

static void CMD_Sample_Set_PD(EmbeddedCli *cli, char *args, void *context) {
//    int argc = embeddedCliGetTokenCount(args);
//    if (argc < 2) {
//        embeddedCliPrint(cli, "Too few arguments. Format: sample_set_pd [pd_index]\n");
//        return;
//    }
//    if (argc > 2) {
//        embeddedCliPrint(cli, "Too many arguments. Format: sample_set_pd [pd_index]\n");
//        return;
//    }

    const char *arg1 = embeddedCliGetToken(args, 1);
    uint8_t pd_ind = (uint8_t)atoi(arg1);

    if (pd_ind < 1 || pd_ind > 36) {
        embeddedCliPrint(cli, "Invalid PD index. Must be between 1 and 36.\n");
        return;
    }

//    SPI_SetDataLength(SPI2, LL_SPI_DATAWIDTH_8BIT);
//    SPI_SetPrescaler(SPI2, LL_SPI_BAUDRATEPRESCALER_DIV16);
    ADG1414_Chain_SwitchOn(&photo_sw, pd_ind);
    photo_index = pd_ind;

    char buf[64];
    snprintf(buf, sizeof(buf), "PD index %d selected successfully.\n", pd_ind);
    embeddedCliPrint(cli, buf);
}

static void CMD_Sample_Set_Rate(EmbeddedCli *cli, char *args, void *context) {
    int argc = embeddedCliGetTokenCount(args);
    if (argc < 3) {
        embeddedCliPrint(cli, "Too few arguments. Format: sp_set_rate [sampling_rate] [num_samples]\n");
        return;
    }

    uint32_t sp_rate = (uint32_t)atoi(embeddedCliGetToken(args, 1));
    uint32_t num_sample = (uint32_t)atoi(embeddedCliGetToken(args, 2));

    if (sp_rate < 1 || sp_rate > 330000 || num_sample < 1 || num_sample > 50000) {
        embeddedCliPrint(cli, "Invalid sampling rate or number of samples.\n");
        return;
    }

    uint32_t AutoReload = ROUND(1000000.0f / sp_rate) - 1;
    LL_TIM_DisableIT_UPDATE(TIM1);
    LL_TIM_DisableCounter(TIM1);
    LL_TIM_SetAutoReload(TIM1, AutoReload);

    adc_rec_total = num_sample;
    samp_rate = sp_rate;

    embeddedCliPrint(cli, "Sampling rate and sample count set successfully.\n");
}

static void CMD_Sample_Trig(EmbeddedCli *cli, char *args, void *context) {
    if (!adc_rec_total || !samp_rate) {
        embeddedCliPrint(cli, "Sampling rate or sample count is not set.\n");
        return;
    }

    memset(adc_rec_buf, 0x00, adc_rec_total * 2);
    adc_ptr = adc_rec_buf;
    adc_rec_ind = 0;

    SPI_SetDataLength(SPI2, LL_SPI_DATAWIDTH_16BIT);
    SPI_SetPrescaler(SPI2, LL_SPI_BAUDRATEPRESCALER_DIV2);

    LL_TIM_SetCounter(TIM1, 0);
    LL_TIM_EnableIT_UPDATE(TIM1);
    LL_TIM_EnableCounter(TIM1);

    embeddedCliPrint(cli, "Sampling started.\n");
}

static void CMD_Sample_Status_Get(EmbeddedCli *cli, char *args, void *context) {
    char buf[128];
    snprintf(buf, sizeof(buf), "Photo: %d   Sampling_Rate: %ld SPS   Num_Samples: %ld S\n",
             photo_index, samp_rate, adc_rec_total);
    embeddedCliPrint(cli, buf);

    if (adc_rec_ind == adc_rec_total) {
        embeddedCliPrint(cli, "-> ADC Data ready to get!\n");
    } else {
        embeddedCliPrint(cli, "-> ADC Data is not ready!\n");
    }
}

static void CMD_Sample_Get(EmbeddedCli *cli, char *args, void *context) {
    if (!adc_rec_ind) {
        embeddedCliPrint(cli, "Please send cmd 'sp_trig' first!\n");
        return;
    }

    uint32_t num_sample = (uint32_t)atoi(embeddedCliGetToken(args, 1));
    if (num_sample < 1 || num_sample > 50000) {
        embeddedCliPrint(cli, "Invalid number of samples.\n");
        return;
    }

    uint16_t crc_val = 0xffff;
    uint8_t bytes_temp[3];
    uint32_t header = (0x000FFFFF & num_sample) | 0xFFF00000;
    bytes_temp[0] = (uint8_t)(header >> 16);
    bytes_temp[1] = (uint8_t)(header >> 8);
    bytes_temp[2] = (uint8_t)header;
    UART_Driver_SendString(USART6, (const char *)bytes_temp);

    for (uint32_t i = 0; i < num_sample; i++) {
        crc16_CCITT_update(&crc_val, adc_rec_buf[i]);
        bytes_temp[0] = adc_rec_buf[i] >> 8;
        bytes_temp[1] = adc_rec_buf[i] & 0xFF;
        UART_Driver_SendString(USART6, (const char *)bytes_temp);
    }

    bytes_temp[0] = crc_val >> 8;
    bytes_temp[1] = crc_val & 0xFF;
    UART_Driver_SendString(USART6, (const char *)bytes_temp);
}

static void CMD_Sample_Get_Char(EmbeddedCli *cli, char *args, void *context) {
    if (!adc_rec_ind) {
        embeddedCliPrint(cli, "Please send cmd 'sp_trig' first!\n");
        return;
    }

    uint32_t num_sample = (uint32_t)atoi(embeddedCliGetToken(args, 1));
    if (num_sample < 1 || num_sample > 50000) {
        embeddedCliPrint(cli, "Invalid number of samples.\n");
        return;
    }

    uint16_t crc_val = 0xffff;
    char ascii_buf[5];
    embeddedCliPrint(cli, "\n");

    for (uint32_t i = 0; i < num_sample; i++) {
        crc16_CCITT_update(&crc_val, adc_rec_buf[i]);
        htoa(adc_rec_buf[i], ascii_buf);
        UART_Driver_SendString(USART6, ascii_buf);
    }

    htoa(crc_val, ascii_buf);
    UART_Driver_SendString(USART6, ascii_buf);
}

static void CMD_Sample_Get_Buf(EmbeddedCli *cli, char *args, void *context) {
    if (!adc_rec_ind) {
        embeddedCliPrint(cli, "Please send cmd 'sp_trig' first!\n");
        return;
    }

    uint16_t crc_val = 0xffff;
    uint8_t bytes_temp[3];
    uint32_t header = (0x000FFFFF & adc_rec_total) | 0xFFF00000;

    bytes_temp[0] = (uint8_t)(header >> 16);
    bytes_temp[1] = (uint8_t)(header >> 8);
    bytes_temp[2] = (uint8_t)header;
    UART_Driver_SendString(USART6, (const char *)bytes_temp);

    for (uint32_t i = 0; i < adc_rec_total; i++) {
        crc16_CCITT_update(&crc_val, adc_rec_buf[i]);
        bytes_temp[0] = adc_rec_buf[i] >> 8;
        bytes_temp[1] = adc_rec_buf[i] & 0xFF;
        UART_Driver_SendString(USART6, (const char *)bytes_temp);
    }

    bytes_temp[0] = crc_val >> 8;
    bytes_temp[1] = crc_val & 0xFF;
    UART_Driver_SendString(USART6, (const char *)bytes_temp);
}

static void CMD_Sample_Get_Buf_Char(EmbeddedCli *cli, char *args, void *context) {
    if (!adc_rec_ind) {
        embeddedCliPrint(cli, "Please send cmd 'sp_trig' first!\n");
        return;
    }

    uint16_t crc_val = 0xffff;
    char ascii_buf[5];
    embeddedCliPrint(cli, "\n");

    for (uint32_t i = 0; i < adc_rec_total; i++) {
        crc16_CCITT_update(&crc_val, adc_rec_buf[i]);
        htoa(adc_rec_buf[i], ascii_buf);
        UART_Driver_SendString(USART6, ascii_buf);
    }

    htoa(crc_val, ascii_buf);
    UART_Driver_SendString(USART6, ascii_buf);
}
/*************************************************
 *                  End CMD List                 *
 *************************************************/

/*************************************************
 *                Getter - Helper                *
 *************************************************/
const CliCommandBinding *getCliStaticBindings(void) {
    return cliStaticBindings_internal;
}

uint16_t getCliStaticBindingCount(void) {
    return sizeof(cliStaticBindings_internal) / sizeof(cliStaticBindings_internal[0]);
}
