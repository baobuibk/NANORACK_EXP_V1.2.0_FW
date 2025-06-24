/*
 * temperature_monitor.h
 *
 *  Created on: Jun 21, 2025
 *      Author: Admin
 */

#ifndef APP_TEMPERATURE_MONITOR_TEMPERATURE_MONITOR_H_
#define APP_TEMPERATURE_MONITOR_TEMPERATURE_MONITOR_H_

#include "sst.h"
#include "fsm.h"

typedef struct temperature_monitor_task_t temperature_monitor_task_t;
typedef struct temperature_monitor_evt_t  temperature_monitor_evt_t;
typedef struct temperature_monitor_init_t  temperature_monitor_init_t;
typedef struct temperature_monitor_data_t {
	int16_t	ntc_temperature[8]; //8 channels
}temperature_monitor_data_t;
typedef state_t (* temperature_monitor_state_handler_t )(struct temperature_monitor_task_t * const me, temperature_monitor_evt_t const * const e);

struct temperature_monitor_evt_t{
	SST_Evt super;
};
struct temperature_monitor_task_t{
	SST_Task super;
	SST_TimeEvt temperature_task_timer;
	temperature_monitor_state_handler_t state; /* the "state variable" */
	temperature_monitor_data_t ntc_data;

};

struct temperature_monitor_init_t {
	temperature_monitor_state_handler_t init_state;
	temperature_monitor_evt_t * current_evt;
	circular_buffer_t * event_buffer;
};

int16_t temperature_monitor_get_ntc_temperature(uint32_t channel);
void temperature_monitor_task_ctor_singleton(void);
void temperature_monitor_task_start(uint8_t priority);
#endif /* APP_TEMPERATURE_MONITOR_TEMPERATURE_MONITOR_H_ */
