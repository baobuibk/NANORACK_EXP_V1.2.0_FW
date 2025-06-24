/*
 * temperature_monitor.c
 *
 *  Created on: Jun 21, 2025
 *      Author: Admin
 */

#include "temperature_monitor.h"
#include "bsp_ntc.h"
#include "dbc_assert.h"
#include "app_signals.h"
#include "error_codes.h"
#include "uart_dbg.h"
#include "stddef.h"
#include "ntc.h"

DBC_MODULE_NAME("temperature_monitor")

temperature_monitor_task_t temperature_monitor_task_inst ;
#define TEMPERATURE_MONITOR_TASK_NUM_EVENTS 4
temperature_monitor_evt_t current_temperature_monitor_e = {0}; // Current event being processed
temperature_monitor_evt_t temperature_monitor_e_buffer[TEMPERATURE_MONITOR_TASK_NUM_EVENTS] = {0}; // Array to hold shell events
circular_buffer_t temperature_monitor_e_queue = {0}; // Circular buffer to hold shell events


static state_t temperature_monitor_state_process_handler(temperature_monitor_task_t * const me, temperature_monitor_evt_t const * const e);


static void temperature_monitor_task_init(temperature_monitor_task_t * const me, temperature_monitor_evt_t const * const e)
{
	SST_TimeEvt_arm(&me->temperature_task_timer, 10, 10); //trigger every 10 tick
}


void temperature_monitor_task_ctor(temperature_monitor_task_t * const me, temperature_monitor_init_t const * const init) {
    DBC_ASSERT(0u, me != NULL);
    SST_Task_ctor(&me->super, (SST_Handler) temperature_monitor_task_init, (SST_Handler)temperature_monitor_state_process_handler, \
                                (SST_Evt *)init->current_evt, init->event_buffer);
    SST_TimeEvt_ctor(&me->temperature_task_timer, EVT_TEMPERATURE_MONITOR_NTC_TRIGGER_TIME, &(me->super));

    me->state = init->init_state;

}

void temperature_monitor_task_ctor_singleton()
{
 circular_buffer_init(&temperature_monitor_e_queue,(uint8_t *)&temperature_monitor_e_buffer,sizeof(temperature_monitor_e_buffer),TEMPERATURE_MONITOR_TASK_NUM_EVENTS,sizeof(temperature_monitor_evt_t));
 temperature_monitor_init_t init = {
		 .init_state = temperature_monitor_state_process_handler,
		 .event_buffer = &temperature_monitor_e_queue,
		 .current_evt = &current_temperature_monitor_e
 };
 temperature_monitor_task_ctor(&temperature_monitor_task_inst, &init);
}

void temperature_monitor_task_start(uint8_t priority)
{
	SST_Task_start(&temperature_monitor_task_inst.super,priority);
}
static state_t temperature_monitor_state_process_handler(temperature_monitor_task_t * const me, temperature_monitor_evt_t const * const e)
{
	switch (e->super.sig)
	{
	case EVT_TEMPERATURE_MONITOR_NTC_ADC_COMPLETED:
		{
			for (uint8_t i = 0; i < 8; i++ )
			{
				me->ntc_data.ntc_temperature[i] = bsp_ntc_get_temperature(i);
			}
			return HANDLED_STATUS;
		}
	case EVT_TEMPERATURE_MONITOR_NTC_TRIGGER_TIME:
		{
			bsp_ntc_trigger_adc();
			return HANDLED_STATUS;
		}
	default:
		return IGNORED_STATUS;
	}
}


int16_t temperature_monitor_get_ntc_temperature(uint32_t channel)
{
	return temperature_monitor_task_inst.ntc_data.ntc_temperature[channel];
}
