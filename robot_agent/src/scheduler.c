/**
 * @file	scheduler.c
 * @author  Eriks Zaharans and Massimiiliano Raciti
 * @date    1 Jul 2013
 *
 * @section DESCRIPTION
 *
 * Cyclic executive scheduler library.
 */

/* -- Includes -- */
/* system libraries */
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
/* project libraries */
#include "scheduler.h"
#include "task.h"
#include "timelib.h"

/* -- Defines -- */
#define		WCET_TASK_MISSION		
#define		WCET_TASK_NAVIGATE		
#define		WCET_TASK_CONTROL		
#define		WCET_TASK_REFINE		
#define		WCET_TASK_REPORT		
#define		WCET_TASK_COMMUNICATE	
#define		WCET_TASK_AVOID			

/* -- Functions -- */

/*
 * Function: get_minor_cycle
 * Description: calculates the minor cycle of the scheduler
 * based on the calculated WCET's
 */

double get_minor_cycle();

/**
 * Initialize cyclic executive scheduler
 * @param minor Minor cycle in miliseconds (ms)
 * @return Pointer to scheduler structure
 */
scheduler_t *scheduler_init(void)
{
	// Allocate memory for Scheduler structure
	scheduler_t *ces = (scheduler_t *) malloc(sizeof(scheduler_t));

	return ces;
}

/**
 * Deinitialize cyclic executive scheduler
 * @param ces Pointer to scheduler structure
 * @return Void
 */
void scheduler_destroy(scheduler_t *ces)
{
	// Free memory
	free(ces);
}

/**
 * Start scheduler
 * @param ces Pointer to scheduler structure
 * @return Void
 */
void scheduler_start(scheduler_t *ces)
{
	// Set timers
	timelib_timer_set(&ces->tv_started);
	timelib_timer_set(&ces->tv_cycle);
}

/**
 * Wait (sleep) till end of minor cycle
 * @param ces Pointer to scheduler structure
 * @return Void
 */
void scheduler_wait_for_timer(scheduler_t *ces)
{
	int sleep_time; // Sleep time in microseconds

	// Calculate time till end of the minor cycle
	sleep_time = (ces->minor * 1000) - (int)(timelib_timer_get(ces->tv_cycle) * 1000);

	// Add minor cycle period to timer
	timelib_timer_add_ms(&ces->tv_cycle, ces->minor);

	// Check for overrun and execute sleep only if there is no
	if(sleep_time > 0)
	{
		// Go to sleep (multipy with 1000 to get miliseconds)
		usleep(sleep_time);
	}
}

/**
 * Execute task
 * @param ces Pointer to scheduler structure
 * @param task_id Task ID
 * @return Void
 */
void scheduler_exec_task(scheduler_t *ces, int task_id)
{
	switch(task_id)
	{
		// Mission
		case s_TASK_MISSION_ID :
			task_mission();
			break;
			// Navigate
		case s_TASK_NAVIGATE_ID :
			task_navigate();
			break;
			// Control
		case s_TASK_CONTROL_ID :
			task_control();
			break;
			// Refine
		case s_TASK_REFINE_ID :
			task_refine();
			break;
			// Report
		case s_TASK_REPORT_ID :
			task_report();
			break;
			// Communicate
		case s_TASK_COMMUNICATE_ID :
			task_communicate();
			break;
			// Collision detection
		case s_TASK_AVOID_ID :
			task_avoid();
			break;
			// Other
		default :
			// Do nothing
			break;
	}
}

/**
 * Run scheduler
 * @param ces Pointer to scheduler structure
 * @return Void
 */
void scheduler_run(scheduler_t *ces)
{
	/* --- Local variables (define variables here) --- */

	/* --- Set minor cycle period --- */
	//ces->minor = ...;

	// Run start the time struct
	timelib_timer_set(&ces->tv_started);
	
	/* --- Write your code here --- */

	unsigned k;
	struct timeval t0, t1;
	for (k=1; k<8; ++k)
	{

		timelib_timer_set(&t0);
		scheduler_exec_task(ces, k);
		timelib_timer_set(&t1);
		printf("Time difference (for task %d): %f\n", k, timelib_timer_diff(t0, t1));
	}

	/*
	 * Refine + Avoid
	 */
	timelib_timer_set(&t0);
	scheduler_exec_task(ces, 4);
	scheduler_exec_task(ces, 7);
	timelib_timer_set(&t1);
	printf("Time difference (for task REFINE AND AVOID ): %f\n"
			, timelib_timer_diff(t0, t1));
}

