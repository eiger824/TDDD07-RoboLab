/**
 * @file	main.c
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
/* project libraries */
#include "src/config.h"
#include "src/def.h"
#include "src/scheduler.h"
#include "src/task.h"
#include "src/timelib.h"

#include "src/robot.h"
#include "src/doublylinkedlist.h"

/* -- Functions -- */

 /**
 * @brief Main application
 */
int main()
{
	// Say hello!
	printf("Hello world!\n");

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
	scheduler_destroy(ces);

	// Say goodbye!
	printf("Goodbye!\n");

	// End application
	return 0;
}
