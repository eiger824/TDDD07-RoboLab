/* @file	main.c
 * @author	Eriks Zaharans
 * @date	1 Jul 2013
 *
 * @section DESCRIPTION
 *
 * Application main file.
 */

/* -- Includes -- */
/* system libraries */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
/* project libraries */
#include "src/config.h"
#include "src/def.h"
#include "src/scheduler.h"
#include "src/task.h"
#include "src/timelib.h"

#include "src/robot.h"
#include "src/doublylinkedlist.h"

/* -- Functions -- */
/*
 * Function:	    sig_handler
 * Brief:	        The signal handler function to use when aborting our program with Ctrl-C.
 *                  Its main purpose is to be able to get some statistics from the scheduler
 *                  even if the program was interrupted (if a SIGINT signal was sent via Ctrl-C).
 * @param signo:	The signal to catch
 * Returns:	        Shall return 0 for the moment
*/
int sig_handler(int signo);

/**
 * @brief Main application
 */
int main()
{
    // Say hello!
    printf("Hello world!\n");

    // Register our signal handler
    if (signal(SIGINT, (void(*)(int))sig_handler) == SIG_ERR)
    {
        fprintf(stderr, "Warning: won't catch SIGINT\n");
    }

    // Initialization
    // Load Configuration
    config_load();
    // Init tasks
    task_init(1);
    // Init scheduler (Set minor and mayor cycle)
    scheduler_t *ces = scheduler_init();

    // Run scheduler
    scheduler_run(ces);

    // Before end application deinitialize and free memory
    // Deinit tasks
    task_destroy();
    // Deinit scheduler
    // Dump some nice stats
    scheduler_dump_statistics();

    // Destroy scheduler
    scheduler_destroy(ces);

    // Say goodbye!
    printf("Goodbye!\n");

    // End application
    return 0;
}

int sig_handler(int signo)
{
    if (signo == SIGINT)
    {
        fprintf(stderr, "SIGINT received!\n");
        scheduler_dump_statistics();

        printf("Goodbye from signal handler!\n");
        // Exit our program
        exit(1);
    }
}


