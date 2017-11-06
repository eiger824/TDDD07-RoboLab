/**
 * @file	task_report.c
 * @author  Eriks Zaharans
 * @date    31 Oct 2013
 *
 * @section DESCRIPTION
 *
 * Report Task.
 */

/* -- Includes -- */
/* system libraries */

/* project libraries */
#include "task.h"

/**
 * Report victim
 */
void task_report(void)
 {
	// Check if task is enabled
	if(g_task_report.enabled == s_TRUE)
	{
		// Check pipe for event
		if(g_tp_refine_report.event == s_TASK_EVENT_SET)
		{
			// Local variables
			int i;
			victim_t victim;

			debug_printf("rfid: %s, %d\n", g_tp_refine_report.victim_id, g_tp_refine_report.victim_id[10]);

			// Check if this victim is new
			for(i = 0; i < g_task_mission_data.victim_count; i++)
			{
				// If there is victim with the same ID -> stop searching
				if(strcmp(g_task_mission_data.victims[i].id, g_tp_refine_report.victim_id) == 0)
				{
					break;
				}
			}
			// If counter is the same as victim count -> new victim is found
			// Report victim
			if(i == g_task_mission_data.victim_count)
			{
				// Save victim position
				victim.x = g_robot->x;
				victim.y = g_robot->y;
				// Save victim ID
				strncpy(victim.id, g_tp_refine_report.victim_id, 11);

				// Redirect victim data to mission
				g_tp_report_mission.victim = victim;
				// Set event
				g_tp_report_mission.event = s_TASK_EVENT_SET;

				// Send victim data (add data to communication queue)
				doublylinkedlist_insert_end(g_list_send, &victim, s_DATA_STRUCT_TYPE_VICTIM);
			}

			// Reset event
			g_tp_refine_report.event = s_TASK_EVENT_RESET;
		}

	}
}