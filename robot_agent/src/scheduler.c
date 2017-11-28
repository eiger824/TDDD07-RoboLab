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
#include <math.h>
/* project libraries */
#include "scheduler.h"
#include "task.h"
#include "timelib.h"

// Nr tasks
#define NR_TASKS_TO_HANDLE      7

/*
 * Factor to use when calculating the deadlines, multiple of the WCET
 */
#define HIGH_PRIO_FACTOR        1.5
#define LOW_PRIO_FACTOR         2.0

// Obtained WCETs from our measurements
#define wcet_TASK_MISSION       1
#define wcet_TASK_NAVIGATE      1
#define wcet_TASK_CONTROL       3
#define wcet_TASK_REFINE        11
#define wcet_TASK_REPORT        1
#define wcet_TASK_COMMUNICATE   5
#define wcet_TASK_AVOID         17

int never_printed_before = 1;

// Array holding the deadline overruns for every task
// There are only 7 valid tasks, but there is a NOP
// task defined in task.h, so let's just take the
// advantage for compatibility purposes
static cnt_t deadline_overruns[NR_TASKS_TO_HANDLE + 1]  = {0};

// A big array which counts the performed tasks over
// the execution time of the program
static cnt_t runtime_tasks[NR_TASKS_TO_HANDLE + 1]      = {0};
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
    struct timeval t0;
    struct timeval t1_stop;
    t1_stop.tv_sec = 0;
    t1_stop.tv_usec = 0;
    /* --- Set minor cycle period --- */
    int major_cycle, nr_minor_cycles;
    ces->minor = 100;

    major_cycle = 1000;
    nr_minor_cycles = major_cycle / ces->minor;

    // Run start the time struct
    scheduler_start(ces);

    // Loop through all minor cycles in a big major cycle
    int i;
    while(1)
    {
        for (i=0; i<nr_minor_cycles; ++i)
        {
            // Control task runs every 500
            if (i % 5 == 0)
            {
                timelib_timer_set(&t0);
                scheduler_exec_task(ces, s_TASK_CONTROL_ID);
                if (timelib_timer_get(t0) > scheduler_get_deadline(s_TASK_CONTROL_ID))
                    ++deadline_overruns[s_TASK_CONTROL_ID];
                ++runtime_tasks[s_TASK_CONTROL_ID];

                if (g_task_control.enabled == s_FALSE && (t1_stop.tv_sec != 0 && t1_stop.tv_usec != 0))
                {
                    if (never_printed_before)
                    {
                        printf("Elapsed milliseconds between STOP CMD and motors stopped: %f\n",
                            timelib_timer_get(t1_stop));
                        never_printed_before = 0;
                    }
                    
                }
            }
            // Avoid task
            timelib_timer_set(&t0);
            scheduler_exec_task(ces, s_TASK_AVOID_ID);
            if (timelib_timer_get(t0) > scheduler_get_deadline(s_TASK_AVOID_ID))
                ++deadline_overruns[s_TASK_AVOID_ID];
            ++runtime_tasks[s_TASK_AVOID_ID];

            // Navigate task
            timelib_timer_set(&t0);
            scheduler_exec_task(ces, s_TASK_NAVIGATE_ID);
            if (timelib_timer_get(t0) > scheduler_get_deadline(s_TASK_NAVIGATE_ID))
                ++deadline_overruns[s_TASK_NAVIGATE_ID];
            ++runtime_tasks[s_TASK_NAVIGATE_ID];

            // Refine task
            timelib_timer_set(&t0);
            scheduler_exec_task(ces, s_TASK_REFINE_ID);
            if (timelib_timer_get(t0) > scheduler_get_deadline(s_TASK_REFINE_ID))
                ++deadline_overruns[s_TASK_REFINE_ID];
            ++runtime_tasks[s_TASK_REFINE_ID];

            // Report task
            timelib_timer_set(&t0);
            scheduler_exec_task(ces, s_TASK_REPORT_ID);
            if (timelib_timer_get(t0) > scheduler_get_deadline(s_TASK_REPORT_ID))
                ++deadline_overruns[s_TASK_REPORT_ID];
            ++runtime_tasks[s_TASK_REPORT_ID];

            // Mission task
            timelib_timer_set(&t0);
            scheduler_exec_task(ces, s_TASK_MISSION_ID);
            if (timelib_timer_get(t0) > scheduler_get_deadline(s_TASK_MISSION_ID))
                ++deadline_overruns[s_TASK_MISSION_ID];
            ++runtime_tasks[s_TASK_MISSION_ID];


            // Communicate task runs every 1000, at the first minor cycle
            if (i == 0)
            {
                // Communicate task
                timelib_timer_set(&t0);
                scheduler_exec_task(ces, s_TASK_COMMUNICATE_ID);
                if (timelib_timer_get(t0) > scheduler_get_deadline(s_TASK_COMMUNICATE_ID))
                    ++deadline_overruns[s_TASK_COMMUNICATE_ID];
                ++runtime_tasks[s_TASK_COMMUNICATE_ID];

                if (g_task_control.enabled == s_FALSE)
                {
                    timelib_timer_set(&t1_stop);
                }
            }
            // Wait until the end of the current minor cycle
            scheduler_wait_for_timer(ces);
        }
    }


}

/*
 * Function:	scheduler_get_deadline
 * Brief:	Given a task id, it returns its computed deadline
 * @param task_id:	Guess what
 * Returns:	The deadline of the input task, in milliseconds.
 *          If the calculation would return a floating-point
 *          value, it will be rounded to the closest integer
 */
int scheduler_get_deadline(int task_id)
{
    /*
     * We are going to assume a constant factor to calculate the
     * deadlines of the tasks based on the WCETs.
     *
     * DEADLINE_k = WCET_k * [LOW|HIGH]_PRIO_FACTOR
     *
     * So, for instance, if [LOW|HIGH]_PRIO_FACTOR = 1, it will mean that D_t = WCET_t;
     *
     * Whether it's LOW or HIGH depends on if the task is a critical task
     * as in Control and communicate, for instance.
     * These two tasks are to be completed "as soon as possible", so
     * it's not a good idea to be very permissive with their deadlines.
     */
    switch (task_id)
    {
        // Mission
        case s_TASK_MISSION_ID :
            return roundf((float)wcet_TASK_MISSION * (float)LOW_PRIO_FACTOR * 10 ) / 10;
            // Navigate
        case s_TASK_NAVIGATE_ID :
            return roundf((float)wcet_TASK_NAVIGATE * (float)LOW_PRIO_FACTOR * 10 ) / 10;
            // Control
        case s_TASK_CONTROL_ID :
            return roundf((float)wcet_TASK_CONTROL * (float)HIGH_PRIO_FACTOR * 10 ) / 10;
            // Refine
        case s_TASK_REFINE_ID :
            return roundf((float)wcet_TASK_REFINE * (float)LOW_PRIO_FACTOR * 10 ) / 10;
            // Report
        case s_TASK_REPORT_ID :
            return roundf((float)wcet_TASK_REPORT* (float)LOW_PRIO_FACTOR * 10 ) / 10;
            // Communicate
        case s_TASK_COMMUNICATE_ID :
            return roundf((float)wcet_TASK_COMMUNICATE* (float)HIGH_PRIO_FACTOR * 10 ) / 10;
            // Collision detection
        case s_TASK_AVOID_ID :
            return roundf((float)wcet_TASK_AVOID* (float)LOW_PRIO_FACTOR * 10 ) / 10;
            // Other
        default :
            // Wrong
            return -1;
    }
}

cnt_t scheduler_get_all_task_cnt()
{
    cnt_t sum = 0;
    unsigned i;
    for (i=1; i<NR_TASKS_TO_HANDLE + 1; ++i)
    {
        sum += runtime_tasks[i];
    }
    return sum;
}

cnt_t scheduler_get_all_deadline_overruns()
{
    cnt_t sum = 0;
    unsigned i;
    for (i=1; i<NR_TASKS_TO_HANDLE + 1; ++i)
    {
        sum += deadline_overruns[i];
    }
    return sum;
}

void scheduler_dump_statistics()
{
    // First: output the number of tasks that were run
    printf("\n****************************************************************\n");
    printf("Nr. of performed tasks:\t\t%llu\n", scheduler_get_all_task_cnt());
    printf("Nr. of detected overruns:\t%llu\n\n", scheduler_get_all_deadline_overruns());
    printf("Summary of parameters:\n");
    printf("#_runs:\tNumber of times a given task has run\n");
    printf("#_do:\tNumber of deadline overruns a given task has experienced\n");
    printf("%%_self:\tPercentage of overruns with respect to the number of\n");
    printf("\ttimes that task ran\n");
    printf("%%_all:\tPercentage of overruns with respect to the global number\n");
    printf("\tof overruns\n\n");
    printf("SUMMARY\n-------------------\n");
    printf("\tMISS\t\tNAV\t\tCON\t\tREF\t\tREP\t\tCOM\t\tAVO\n");
    printf("\t----\t\t---\t\t---\t\t---\t\t---\t\t---\t\t---\n");
    unsigned i;
    printf("#_runs\t");
    for (i=1; i<NR_TASKS_TO_HANDLE + 1; ++i)
    {
        printf("%llu\t\t", runtime_tasks[i]);
    }
    printf("\n#_do\t");
    for (i=1; i<NR_TASKS_TO_HANDLE + 1; ++i)
    {
        printf("%llu\t\t", deadline_overruns[i]);
    }
    printf("\n%%_self\t");
    for (i=1; i<NR_TASKS_TO_HANDLE + 1; ++i)
    {
        printf("%.2f%%\t\t",
                100 * ((float)deadline_overruns[i] / (float)runtime_tasks[i]));
    }
    printf("\n%%_all\t");
    for (i=1; i<NR_TASKS_TO_HANDLE + 1; ++i)
    {
        printf("%.2f%%\t\t",
                100 * ((float)deadline_overruns[i] / (float)scheduler_get_all_deadline_overruns()));
    }
    printf("\n\nOVERALL PERFORMANCE OF SCHEDULER:\t%.2f%%\n",
            100 * ( 1 - ((float)scheduler_get_all_deadline_overruns() / (float)scheduler_get_all_task_cnt())));
    printf("OVERALL DEADLINE OVERRUNS OF SCHEDULER:\t%.2f%%\n",
            100 * (  ((float)scheduler_get_all_deadline_overruns() / (float)scheduler_get_all_task_cnt())));
    printf("****************************************************************\n");
}

