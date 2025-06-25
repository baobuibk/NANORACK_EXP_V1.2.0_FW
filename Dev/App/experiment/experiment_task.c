/*
 * experiment_task.c
 *
 *  Created on: Jun 25, 2025
 *      Author: Admin
 */
#include "experiment_task.h"
#include "dbc_assert.h"
#include "app_signals.h"
#include "error_codes.h"
#include "uart_dbg.h"
#include "stddef.h"

DBC_MODULE_NAME("experiment_task")

#define EXPERIMENT_TASK_NUM_EVENTS  	2
#define EXPERIMENT_TASK_AQUI_TIMEOUT	2000

experiment_task_t experiment_task_inst;

static experiment_evt_t const entry_evt = {.super = {.sig = SIG_ENTRY} };
static experiment_evt_t const exit_evt = {.super = {.sig = SIG_EXIT} };

experiment_evt_t experiment_task_current_event = {0}; // Current event being processed
experiment_evt_t experiment_task_event_buffer[EXPERIMENT_TASK_NUM_EVENTS];

circular_buffer_t experiment_task_event_queue = {0}; // Circular buffer to hold shell events

static state_t experiment_task_state_manual_handler(experiment_task_t * const me, experiment_evt_t const * const e);
static state_t experiment_task_state_data_aqui_handler(experiment_task_t * const me, experiment_evt_t const * const e);
static state_t experiment_task_state_data_send_handler(experiment_task_t * const me, experiment_evt_t const * const e);

static void experiment_task_init(experiment_task_t * const me,experiment_evt_t const * const e)
{
	DBG(DBG_LEVEL_INFO,"experiment_task init\r\n");
}
static void experiment_task_dispatch(experiment_task_t * const me,experiment_evt_t const * const e)
{
    experiment_task_handler_t prev_state = me->state; /* save for later */
    state_t status = (me->state)(me, e);

    if (status == TRAN_STATUS) { /* transition taken? */
        (prev_state)(me, &exit_evt);
        (me->state)(me, &entry_evt);
    }
}
void experiment_task_ctor(experiment_task_t * const me, experiment_task_init_t const * const init) {
    DBC_ASSERT(0u, me != NULL);
    SST_Task_ctor(&me->super, (SST_Handler) experiment_task_init, (SST_Handler)experiment_task_dispatch, \
                                (SST_Evt *)init->current_evt, init->event_buffer);
    SST_TimeEvt_ctor(&me->timeout_timer, EVT_EXPERIMENT_DATA_AQUISITION_TIMEOUT, &(me->super));
    SST_TimeEvt_disarm(&me->timeout_timer); // Disarm the timeout timer
    me->sub_state = init->sub_state;
}
void experiment_task_singleton_ctor(void)
{
	circular_buffer_init(&experiment_task_event_queue, (uint8_t * )&experiment_task_event_buffer, sizeof(experiment_task_event_buffer), EXPERIMENT_TASK_NUM_EVENTS, sizeof(experiment_evt_t));
	experiment_task_init_t init = {
			.init_state = experiment_task_state_manual_handler,
			.current_evt = &experiment_task_current_event,
			.event_buffer = &experiment_task_event_queue,
			.sub_state = NO_SUBSTATE
	};
	experiment_task_ctor(&experiment_task_inst,&init);
}
void experiment_task_start(uint8_t priority)
{
	SST_Task_start(&experiment_task_inst.super,priority);
}

//only handle command from shell
static state_t experiment_task_state_manual_handler(experiment_task_t * const me, experiment_evt_t const * const e)
{
	switch (e->super.sig)
	{
		case SIG_ENTRY:
		{
			DBG(DBG_LEVEL_INFO,"entry experiment_task_state_manual_handler\r\n");
			SST_TimeEvt_disarm(&me->timeout_timer); //disable the timeout

		case EVT_EXPERIMENT_HAS_COMMAND:
		{
			switch (e->cmd){
			case EXPERIMENT_TEMPERATURE_AQUI_START:
			{
				me->state = experiment_task_state_data_aqui_handler;
				return TRAN_STATUS;
			}
			case EXPERIMENT_TEMPERATURE_AQUI_SEND_DATA:
			{
				me->state = experiment_task_state_data_send_handler;
				return TRAN_STATUS;
			}
			default:
				return IGNORED_STATUS;
		}
	}
 }
}
	return IGNORED_STATUS;
}


static state_t experiment_task_state_data_aqui_handler(experiment_task_t * const me, experiment_evt_t const * const e)
{
	switch (e->super.sig)
	{
		case SIG_ENTRY:
		{
			DBG(DBG_LEVEL_INFO,"entry experiment_task_state_data_aqui_handler\r\n");
			SST_TimeEvt_arm(&me->timeout_timer, EXPERIMENT_TASK_AQUI_TIMEOUT, 0);
			me->sub_state = S_PRE_SAMPLING;
			return HANDLED_STATUS;
		}
		case SIG_EXIT:
		{
			DBG(DBG_LEVEL_INFO,"exit experiment_task_state_data_aqui_handler\r\n");
			SST_TimeEvt_disarm(&me->timeout_timer);
			return HANDLED_STATUS;
		}
		case EVT_EXPERIMENT_FINISH_PRE_SAMPLING:
		{
			if (me->sub_state == S_PRE_SAMPLING)
			{
				me->sub_state = S_DATA_SAMPLING;
			}
			else me->sub_state = S_AQUI_ERROR;
			return HANDLED_STATUS;
		}
		case EVT_EXPERIMENT_FINISH_SAMPLING:
		{
			if (me->sub_state == S_DATA_SAMPLING)
			{
				me->sub_state = S_POST_SAMPLING;
			}
			else me->sub_state = S_AQUI_ERROR;
			return HANDLED_STATUS;
		}
		case EVT_EXPERIMENT_FINISH_POST_SAMPLING:
		{
			if (me->sub_state == S_POST_SAMPLING)
			{
				me->sub_state = NO_SUBSTATE;
			}
			else me->sub_state = S_AQUI_ERROR;
			me->state = experiment_task_state_manual_handler;
			return TRAN_STATUS;
		}
		case EVT_EXPERIMENT_DATA_AQUISITION_TIMEOUT:
		{
			me->sub_state = S_AQUI_TIMEOUT;
			me->state = experiment_task_state_manual_handler;
			return TRAN_STATUS;
		}
		default:
			return IGNORED_STATUS;
	}
 }

static state_t experiment_task_state_data_send_handler(experiment_task_t * const me, experiment_evt_t const * const e)
{

}


