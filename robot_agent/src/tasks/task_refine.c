/**
 * @file	task_refine.c
 * @author  Eriks Zaharans
 * @date    31 Oct 2013
 *
 * @section DESCRIPTION
 *
 * Refine Task.
 */

/* -- Includes -- */
/* system libraries */

/* project libraries */
#include "task.h"

/**
 * Refine position, localization
 */
void task_refine(void)
{
	// Check if task is enabled
	if(g_task_refine.enabled == s_TRUE)
	{
		// Local variables
		int res;

		// Ping RFID reader
#ifndef s_CONFIG_TEST_ENABLE
		rfid_read(g_rfids);
#endif

		// Check RFID tag
		res = enviroment_tag_check(g_envs, g_rfids->id);
		// If tag is known -> weight particles and resample
		if(res >= 0)
		{
			// Measurement Update (Particle filter)
			pf_weight_tag(g_pfs, g_envs, res);
			pf_resample(g_pfs);
			pf_estimate(g_pfs, g_robot);
			pf_random(g_pfs, g_envs, res);
			//pf_estimate(g_pfs, g_robot); // !!!
		}
		// If s_ENVIROMENT_TAG_UNKNOWN, tag is unknown. Most probably a victim
		else if(res == s_ENVIROMENT_TAG_UNKNOWN)
		{
			// Redirect to task_report()
			// Copy ID to pipe
			strncpy(g_tp_refine_report.victim_id, g_rfids->id, 11);
			// Set event
			g_tp_refine_report.event = s_TASK_EVENT_SET;
		}
		else if(res == s_ENVIROMENT_TAG_DISABLED)
		{
			// Do nothing
			debug_printf("disabled tag red.\n");
		}

	}
}