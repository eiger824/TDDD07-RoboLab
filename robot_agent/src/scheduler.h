/**
 * @file	scheduler.h
 * @author  Eriks Zaharans
 * @date	1 Jul 2013
 *
 * @section DESCRIPTION
 *
 * Cyclic executive scheduler library header file.
 */

#ifndef __SCHEDULER_H
#define __SCHEDULER_H

#include "def.h"
/**
 * @brief Scheduler structure
 */
typedef struct s_SCHEDULER_STRUCT
{
	unsigned int minor; // Minor cycle in miliseconds (ms)

	struct timeval tv_started; // Timer that registers when scheduler started
	struct timeval tv_cycle; // Timer for cycle sleeps/interrupts

} scheduler_t;

/* -- Function Prototypes -- */

/*
 * Function:	    scheduler_init
 * Brief:	        Initialize cyclic executive scheduler
 * @param minor:	The minor cycle to use
 * @asserts:        If major cycle is not a multiple of the minor cycle
 * Returns:	        Nothing
*/
scheduler_t *scheduler_init(unsigned minor); // Initialize cyclic executive scheduler
void scheduler_destroy(scheduler_t *ces); // Deinitialize cyclic executive scheduler
void scheduler_start(scheduler_t *ces); // Start scheduler
void scheduler_wait_for_timer(scheduler_t *ces); // Wait (sleep) till end of minor cycle
void scheduler_exec_task(int task_id); // Execute task
void scheduler_run(scheduler_t *ces); // Run scheduler
int  scheduler_get_deadline(int task_id); // Get deadline for specific task
// Wrapper for task execution: init timers, calculate deadline & exec time
void scheduler_process_task(int task_id, struct timeval* timer);
// Dump runtime statistics. No scheduler parameter is given since
// this function shall be called from outside main routine
void scheduler_dump_statistics(scheduler_t *ces);

// Get the overall task count
cnt_t scheduler_get_all_task_cnt();
// Get the overall deadline overrun count
cnt_t scheduler_get_all_deadline_overruns();

#endif /* __SCHEDULER_H */
