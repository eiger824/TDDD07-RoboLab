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
#include <assert.h>
/* project libraries */
#include "scheduler.h"
#include "task.h"
#include "timelib.h"

// Nr tasks
#define     NR_TASKS_TO_HANDLE              7

// Major cycle to use
#define     SCHEDULER_MAJOR_CYCLE           1000

// Factor to use when calculating the deadlines
#define     HIGH_PRIO_FACTOR                1.5
#define     LOW_PRIO_FACTOR                 2.0

// Obtained WCETs from our measurements
#define     wcet_TASK_MISSION               1
#define     wcet_TASK_NAVIGATE              1
#define     wcet_TASK_CONTROL               5
#define     wcet_TASK_REFINE                11
#define     wcet_TASK_REPORT                1
#define     wcet_TASK_COMMUNICATE           5
#define     wcet_TASK_AVOID                 17

// Array holding the deadline overruns for every task
// There are only 7 valid tasks, but there is a NOP
// task defined in task.h, so let's just take the
// advantage for compatibility purposes
static cnt_t deadline_overruns[NR_TASKS_TO_HANDLE + 1]  = {0};

// A big array which counts the performed tasks over
// the execution time of the program
static cnt_t runtime_tasks[NR_TASKS_TO_HANDLE + 1]      = {0};

// global constant representing the average period for the avoid task
static float runtime_average_avoid_task = 0.0;
// global constant representing the average waiting time every minor cycle
// when all tasks have completed (i.e. IDLE time)
static float runtime_average_sleep_time = 0.0;

/**
 * Initialize cyclic executive scheduler
 * @param minor Minor cycle in miliseconds (ms)
 * @return Pointer to scheduler structure
 */
scheduler_t *scheduler_init(unsigned minor)
{
    // Allocate memory for Scheduler structure
    scheduler_t *ces = (scheduler_t *) malloc(sizeof(scheduler_t));
    assert(SCHEDULER_MAJOR_CYCLE % minor == 0);
    ces->minor = minor;
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
void scheduler_exec_task(int task_id)
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
    // Local variables (define variables here)
    struct timeval task_exec_time;
    struct timeval avoid_period;
    struct timeval remaining_time;

    // Scheduler details
    int nr_minor_cycles;

    // Runtime sample of avoid task period
    float current_sample_avoid_task = 0.0;
    // Runtime IDLE time every minor cycle
    float current_measurement = 0.0;

    int first_time = 1;
    int second_time = 1;

    int first_measurement = 1;

    nr_minor_cycles = SCHEDULER_MAJOR_CYCLE / ces->minor;

    // Run start the time struct
    scheduler_start(ces);

    int i;
    // Loop through all minor cycles in a big major cycle
    while(1)
    {
        for (i=0; i<nr_minor_cycles; ++i)
        {
            /************************ Navigate task *************************/
            scheduler_process_task(s_TASK_NAVIGATE_ID, &task_exec_time);
            /****************************************************************/

            /************************ Control task **************************/

            // To be run every 500ms. But when this task does NOT run
            // (i.e. in minor cycles 1,2,3,4,6,7,8,9),
            // we want to keep this slot free so the rest of the tasks'
            // periods are respected
            if (i % 5 == 0)
            {
                scheduler_process_task(s_TASK_CONTROL_ID, &task_exec_time);
            }
            else
            {
                usleep(scheduler_get_deadline(s_TASK_CONTROL_ID));
            }
            /****************************************************************/

            /************************ Avoid task ****************************/

            // To be run every second minor cycle
            if (i % 2 == 0)
            {

                if (first_time)
                {
                    // The first time, just set the timer
                    timelib_timer_set(&avoid_period);
                    first_time = 0;
                }
                else
                {
                    // After the first time, always get the elapsed time
                    current_sample_avoid_task = timelib_timer_reset(&avoid_period);
                    if (second_time)
                    {
                        // The second time, compute elapsed time AND set average to first sample
                        runtime_average_avoid_task = current_sample_avoid_task;
                        second_time = 0;
                    }
                    else
                    {
                        // Update runtime average of avoid task
                        runtime_average_avoid_task = (runtime_average_avoid_task + current_sample_avoid_task) / 2;
                    }
                }
                scheduler_process_task(s_TASK_AVOID_ID, &task_exec_time);
            }
            else
            {
                usleep(scheduler_get_deadline(s_TASK_AVOID_ID));
            }
            /****************************************************************/

            /************************ Refine task ***************************/
            scheduler_process_task(s_TASK_REFINE_ID, &task_exec_time);
            /****************************************************************/

            /************************ Report task ***************************/
            scheduler_process_task(s_TASK_REPORT_ID, &task_exec_time);
            /****************************************************************/

            /************************ Communicate task **********************/
            // Communicate task runs every 1000, at the first minor cycle
            if (i == 0)
            {
                scheduler_process_task(s_TASK_COMMUNICATE_ID, &task_exec_time);
            }
            else
            {
                usleep(scheduler_get_deadline(s_TASK_COMMUNICATE_ID));
            }
            /****************************************************************/

            /************************ Mission task **************************/
            scheduler_process_task(s_TASK_MISSION_ID, &task_exec_time);
            /****************************************************************/
            /*********************** IDLE time ******************************/
            // Wait until the end of the current minor cycle
            // We calculate a 2-sample average of waiting times
            timelib_timer_set(&remaining_time);
            scheduler_wait_for_timer(ces);
            current_measurement = timelib_timer_get(remaining_time);

            if (first_measurement)
            {
                runtime_average_sleep_time = current_measurement;
                first_measurement = 0;
            }
            else
            {
                runtime_average_sleep_time = (current_measurement + runtime_average_sleep_time) / 2;
            }
            /****************************************************************/
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
        case 1 :
            return ceil((float)wcet_TASK_MISSION * (float)LOW_PRIO_FACTOR);
            // Navigate
        case 2:
            return ceil((float)wcet_TASK_NAVIGATE * (float)LOW_PRIO_FACTOR);
            // Control
        case 3:
            return ceil((float)wcet_TASK_CONTROL * (float)HIGH_PRIO_FACTOR);
            // Refine
        case 4:
            return ceil((float)wcet_TASK_REFINE * (float)LOW_PRIO_FACTOR);
            // Report
        case 5:
            return ceil((float)wcet_TASK_REPORT * (float)LOW_PRIO_FACTOR);
            // Communicate
        case 6:
            return ceil((float)wcet_TASK_COMMUNICATE * (float)HIGH_PRIO_FACTOR);
            // Collision detection
        case 7:
            return ceil((float)wcet_TASK_AVOID * (float)LOW_PRIO_FACTOR);
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

void scheduler_dump_statistics(scheduler_t *ces)
{
    // First: output the number of tasks that were run
    printf("\n****************************************************************\n");
    printf("Scheduler minor cycle: %d ms\n", ces->minor);
    printf("Scheduler run-time: %.2f s\n", timelib_timer_get(ces->tv_started));
    printf("Nr. of performed tasks:\t\t%llu\n", scheduler_get_all_task_cnt());
    printf("Nr. of detected overruns:\t%llu\n\n", scheduler_get_all_deadline_overruns());
    printf("Application requirements:\n");
    printf("[Req 1] Avoid task call rate: %f ms\n", runtime_average_avoid_task);
    printf("[Req 2] See messages printed to stdout (starting with \"[Req 2]\")\n");
    printf("[Req 3] See messages printed to stdout (starting with \"[Req 3]\")\n\n");
    printf("Some extra parameters:\n");
    printf("Number of illegal communications attempted (w/o go_ahead): %llu (%.2f %%)\n",
            illegal_communications, 100 * ((float) illegal_communications / (float) total_communications));
    printf("Number of legal communications made (w go_ahead): %llu (%.2f %%)\n",
            total_communications - illegal_communications,
            100 * ((float) (total_communications - illegal_communications) / (float) total_communications));
    printf("Average IDLE time every minor cycle: %f ms\n\n", runtime_average_sleep_time);
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

void scheduler_process_task(int task_id, struct timeval *timer)
{
    int deadline;
    double exec_time;

    // Set timer structure
    timelib_timer_set(timer);
    // Execute the task
    scheduler_exec_task(task_id);
    // Obtain execution time
    exec_time = timelib_timer_get(*timer);
    // Fetch deadline
    deadline = scheduler_get_deadline(task_id);
    // Check for deadline overrun: if detected, add up counter; if not,
    // just sleep until the deadline
    if (exec_time > deadline)
    {
        ++deadline_overruns[task_id];
    }
    else
    {
        usleep((float)(deadline - exec_time) * 1000);
    }
    ++runtime_tasks[task_id];
}


