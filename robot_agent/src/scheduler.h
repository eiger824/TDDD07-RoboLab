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

/* -- Includes -- */

/* -- Enumurations -- */

/* -- Types -- */

/**
 * @brief Scheduler structure
 */
typedef struct s_SCHEDULER_STRUCT
{
	unsigned int minor; // Minor cycle in miliseconds (ms)

	struct timeval tv_started; // Timer that registers when scheduler started
	struct timeval tv_cycle; // Timer for cycle sleeps/interrupts

} scheduler_t;

/* -- Constants -- */

/* -- Function Prototypes -- */
scheduler_t *scheduler_init(void); // Initialize cyclic executive scheduler
void scheduler_destroy(scheduler_t *ces); // Deinitialize cyclic executive scheduler
void scheduler_start(scheduler_t *ces); // Start scheduler
void scheduler_wait_for_timer(scheduler_t *ces); // Wait (sleep) till end of minor cycle
void scheduler_exec_task(scheduler_t *ces, int task_id); // Execute task
void scheduler_run(scheduler_t *ces); // Run scheduler

#endif /* __SCHEDULER_H */
