/*
 * experiment_task.h
 *
 *  Created on: Jun 25, 2025
 *      Author: Admin
 */

#ifndef APP_EXPERIMENT_EXPERIMENT_TASK_H_
#define APP_EXPERIMENT_EXPERIMENT_TASK_H_

#include "sst.h"
#include "fsm.h"

#define EXPERIMENT_COMMAND_PAYLOAD_LENGTH 16
typedef struct experiment_task_t experiment_task_t;
typedef struct experiment_evt_t experiment_evt_t;
typedef struct experiment_task_init_t experiment_task_init_t;
typedef state_t (*experiment_task_handler_t)(experiment_task_t  * const me, experiment_evt_t const * const e);
enum {S_PRE_SAMPLING, S_DATA_SAMPLING, S_POST_SAMPLING, S_AQUI_ERROR,S_AQUI_TIMEOUT,NO_SUBSTATE};
enum {EXPERIMENT_TEMPERATURE_AQUI_START,EXPERIMENT_TEMPERATURE_AQUI_SEND_DATA};

struct experiment_evt_t{
	SST_Evt super;
	uint8_t cmd;
	uint8_t payload[EXPERIMENT_COMMAND_PAYLOAD_LENGTH];
};

struct experiment_task_t{
	SST_Task super;
	experiment_task_handler_t state;
	SST_TimeEvt timeout_timer;
	uint8_t	sub_state;
	uint8_t  laser_current[2]; //current for internal/external laser
	uint8_t  int_laser_pos; //0-36, 0xFF for switched OFF all
	uint8_t  ext_laser_pos; //0-8, 0xFF for switched OFF all

};

struct experiment_task_init_t {
	experiment_task_handler_t init_state;
	experiment_evt_t * current_evt;
	circular_buffer_t * event_buffer;
	uint8_t	sub_state;
};

uint32_t experiment_task_laser_set_current(experiment_task_t * const me, uint32_t laser_id, uint32_t percent);
uint32_t experiment_task_laser_get_current(experiment_task_t * const me, uint32_t laser_id);
uint32_t experiment_task_int_laser_switchon(experiment_task_t * const me, uint32_t laser_id);
uint32_t experiment_task_ext_laser_switchon(experiment_task_t * const me, uint32_t laser_id);
uint32_t experiment_task_int_laser_switchoff(experiment_task_t * const me);
uint32_t experiment_task_ext_laser_switchoff(experiment_task_t * const me);

#endif /* APP_EXPERIMENT_EXPERIMENT_TASK_H_ */
