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

// Proposed periods (deadlines)
#define     t_TASK_MISSION                  100
#define     t_TASK_NAVIGATE                 100
#define     t_TASK_CONTROL                  500
#define     t_TASK_REFINE                   100
#define     t_TASK_REPORT                   100
#define     t_TASK_COMMUNICATE              1000
#define     t_TASK_AVOID                    500

// Array holding the deadline overruns for every task
// There are only 7 valid tasks, but there is a NOP
// task defined in task.h, so let's just take the
// advantage for compatibility purposes
static cnt_t deadline_overruns[NR_TASKS_TO_HANDLE + 1]  = {0};

// A big array which counts the performed tasks over
// the execution time of the program
static cnt_t runtime_tasks[NR_TASKS_TO_HANDLE + 1]      = {0};

//sleep time to sync with mission countrol
static useconds_t sync_sleep_time = 0;

cnt_t total_data_count[4] = {0};
cnt_t actual_data_count[4] = {0};
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
    // Set minor cycle
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
    // Scheduler details
    unsigned nr_minor_cycles;

    nr_minor_cycles = SCHEDULER_MAJOR_CYCLE / ces->minor;

    unsigned i;
    // Get UNIX timestamp to be sychronized with the clock of the router, in seconds
    double timestamp = timelib_unix_timestamp() / 1e3;
    // Compute the difference in microseconds with the next higher second
    double diff = (ceil(timestamp) - timestamp) * 1e6;
    // Round the obtained microsecond difference to a useconds_t type, which is what
    // we will haveto sleep to synchronize
    sync_sleep_time = (useconds_t) round(diff);
    // And sleep the scheduler
    usleep(sync_sleep_time);

    // Run start the time struct
    scheduler_start(ces);

    // Loop through all minor cycles in a big major cycle
    while (1)
    {
        for (i=0; i<nr_minor_cycles; ++i)
        {
            // Set timer structure for deadline calculation at the start
            // of every minor cycle
            timelib_timer_set(&task_exec_time);

            // Communicate task runs every 1000, at the fifth minor
            // cycle(tdma slot 5)
            if (i == g_config.robot_id)
            {
                scheduler_process_task(s_TASK_COMMUNICATE_ID, &task_exec_time);
            }

            /************************ Navigate task *************************/
            scheduler_process_task(s_TASK_NAVIGATE_ID, &task_exec_time);
            /****************************************************************/

            // To be run every 500ms
            if (i % 5 == 0)
            {
                /************************ Control task **************************/
                scheduler_process_task(s_TASK_CONTROL_ID, &task_exec_time);
                /************************ Avoid task ****************************/
                scheduler_process_task(s_TASK_AVOID_ID, &task_exec_time);
                /****************************************************************/
            }

            /************************ Refine task ***************************/
            scheduler_process_task(s_TASK_REFINE_ID, &task_exec_time);
            /****************************************************************/

            /************************ Report task ***************************/
            scheduler_process_task(s_TASK_REPORT_ID, &task_exec_time);
            /****************************************************************/

            /************************ Mission task **************************/
            scheduler_process_task(s_TASK_MISSION_ID, &task_exec_time);
            /****************************************************************/

            /*********************** IDLE time ******************************/
            // Wait until the end of the current minor cycle
            scheduler_wait_for_timer(ces);
            /****************************************************************/
        }
    }
}

/*
 * Function:	    scheduler_get_deadline
 * Brief:	        Given a task id, it returns its computed deadline
 * @param task_id:	Guess what
 * Returns:	        The deadline of the input task, in milliseconds.
 *                  If the calculation would return a floating-point
 *                  value, it will be rounded to the next higher integer
 */
int scheduler_get_deadline(int task_id)
{
    /*
     * We are going to assume the deadlines of the tasks to be 
     * the equal to the periods.
     */
    switch (task_id)
    {
        case 1 :// Mission
            return t_TASK_MISSION;
        case 2:// Navigate
            return t_TASK_NAVIGATE;
        case 3:// Control
            return t_TASK_CONTROL;
        case 4:// Refine
            return t_TASK_REFINE;
        case 5:// Report
            return t_TASK_REPORT;
        case 6:// Communicate
            return t_TASK_COMMUNICATE;
        case 7:// Avoid
            return t_TASK_AVOID;
        default:// Wrong
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
    unsigned i;
    double scheduler_run_time =  (double)(timelib_timer_get(ces->tv_started) / 1000.0);
    // First: output the number of tasks that were run
    printf("\n****************************************************************\n");
    printf("Scheduler minor cycle:\t\t%d ms\n", ces->minor);
    printf("Scheduler run-time:\t\t%.2f s\n", scheduler_run_time);
    printf("Scheduler sync-time:\t\t%.2f ms\n", (float)(sync_sleep_time) / 1000.0);
    printf("Nr. of performed tasks:\t\t%llu\n", scheduler_get_all_task_cnt());
    printf("Nr. of detected overruns:\t%llu\n\n", scheduler_get_all_deadline_overruns());
    printf("Application requirements:\n");
    printf("[Req 1] Avoid task call rate: %f ms\n", 1e3 / ((float)runtime_tasks[s_TASK_AVOID_ID] / (float)scheduler_run_time));
    printf("[Req 2] See messages printed to stdout (starting with \"[Req 2]\")\n");
    printf("[Req 3] See messages printed to stdout (starting with \"[Req 3]\")\n\n");
    printf("Some extra parameters:\n");
    printf("Robot ID:\t\t\t\t\t\t\t%d\n", g_config.robot_id);
    printf("Number of illegal communications attempted (w/o go_ahead):\t%llu (%.2f %%)\n",
            illegal_communications, 100 * ((float) illegal_communications / (float) total_communications));
    printf("Number of legal communications made (w go_ahead):\t\t%llu (%.2f %%)\n",
            total_communications - illegal_communications,
            100 * ((float) (total_communications - illegal_communications) / (float) total_communications));
    printf("Number of total victim reports:\t\t\t\t%llu \n", total_victims);
    printf("Number of inaccurate victim position reports:\t\t\t%llu (%.2f %%)\n",
            inaccurate_victims,
            100 * (((float)inaccurate_victims / (float)total_victims)));
    printf("\nCommunication statistics:\n");
    printf("data_type\t\tROBOT\tVICTIM\tPHEROM\tSTREAM\n");
    printf("#_packets_2_send\t");
    for (i=0; i<4; ++i)
    {
        printf("%llu\t", total_data_count[i]);
    }
    printf("\n#_packets_sent\t\t");
    for (i=0; i<4; ++i)
    {
        printf("%llu\t", actual_data_count[i]);
    }
    printf("\n%%_communications\t");
    for (i=0; i<4; ++i)
    {
        printf("%.2f%%\t", 100 * (float)(actual_data_count[i]) / (float)(total_data_count[i]));
    }
    printf("\n\nSummary of scheduler parameters:\n");
    printf("#_runs:\tNumber of times a given task has run\n");
    printf("#_do:\tNumber of deadline overruns a given task has experienced\n");
    printf("%%_self:\tPercentage of overruns with respect to the number of\n");
    printf("\ttimes that task ran\n");
    printf("%%_all:\tPercentage of overruns with respect to the global number\n");
    printf("\tof overruns\n\n");
    printf("SUMMARY\n-------------------\n");
    printf("\tMISS\t\tNAV\t\tCON\t\tREF\t\tREP\t\tCOM\t\tAVO\n");
    printf("\t----\t\t---\t\t---\t\t---\t\t---\t\t---\t\t---\n");
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
    ++runtime_tasks[task_id];
}


