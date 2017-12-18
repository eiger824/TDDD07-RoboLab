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
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/* project libraries */
#include "task.h"
#include "def.h"

#define         TOTAL_VICTIMS       24
// Table with hardcoded victims to check for accuracy
// This table is in the victims.pdf file
static const victim_t VICTIM_TABLE[TOTAL_VICTIMS] =
{
    { 340,      340,    "020058F5BD\0" },
    { 975,      1115,   "020053A537\0" },
    { 1845,     925,    "020053E0BA\0" },
    { 2670,     355,    "01004B835E\0" },
    { 3395,     870,    "020053C80E\0" },
    { 4645,     910,    "020058100D\0" },
    { 4800,     250,    "0200580B96\0" },
    { 5395,     1060,   "02005345B6\0" },
    { 5830,     1895,   "020058F121\0" },
    { 5110,     2390,   "0200581B9E\0" },
    { 5770,     3790,   "020058066F\0" },
    { 4500,     3190,   "020058212D\0" },
    { 4315,     3200,   "020058022D\0" },
    { 4150,     1810,   "0200581542\0" },
    { 3720,     3710,   "0200534E5C\0" },
    { 2580,     3770,   "020053AB2C\0" },
    { 2970,     2805,   "01004A11E8\0" },
    { 3030,     2070,   "020053E282\0" },
    { 3120,     1965,   "0200553505\0" },
    { 2880,     1840,   "01004751A2\0" },
    { 1890,     2580,   "02005097C0\0" },
    { 985,      3020,   "020053BF78\0" },
    { 730,      3175,   "020056D0EF\0" },
    { 320,      1800,   "01004BDF7B\0" }
};
static double current_offset_value = 0.0;
static int first_time = 1;

/*
 * Function:	        check_accuracy_victim_location
 * Brief:	            Given a found victim, it checks whether its coordinates match
 *                      with the hardcoded values
 * @param found_victim:	Victim struct representing the victim found by the RFID reader
 * @param dx:           The X difference between the found victim and the hardcoded value
 * @param dy:           The Y difference between the found victim and the hardcoded value
 * Returns:	            On success, 0 will be returned to indicate that the location
 *                      of the victim is correct, 1 otherwise
*/
int check_accuracy_victim_location(victim_t found_victim, int *dx, int *dy);

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
            // Victim found: start timer
            printf("[Req.2] Victim was found!\n");
            gettimeofday(&notify_victim_time, NULL);
			// Redirect to task_report()
			// Copy ID to pipe
			strncpy(g_tp_refine_report.victim_id, g_rfids->id, 11);
			// Set event
			g_tp_refine_report.event = s_TASK_EVENT_SET;

            // Check the accuracy of the found victim's location
            victim_t current_victim;
            current_victim.x = g_robot->x;
            current_victim.y = g_robot->y;
            memcpy(&current_victim.id, &g_rfids->id, 11);

            int dx, dy;
            if (!check_accuracy_victim_location(current_victim, &dx, &dy))
            {
                printf("Found victim's position is accurate: [%d,%d], with ID %s\n",
                        current_victim.x, current_victim.y, current_victim.id);
            }
            else
            {
                fprintf(stderr, "Found victim's position innacurate (ID %s)\n",
                        current_victim.id);
                // Add up failure counter
                ++inaccurate_victims;
                current_offset_value = sqrt(pow(dx,2) + pow(dx,2));
                if (first_time)
                {
                    victim_offset_average = current_offset_value;
                    first_time = 0;
                }
                else
                {
                    victim_offset_average = (victim_offset_average + current_offset_value) / 2;
                }
            }
            // Add up victim counter
            ++total_victims;
		}
		else if(res == s_ENVIROMENT_TAG_DISABLED)
		{
			// Do nothing
			debug_printf("disabled tag red.\n");
		}

	}
}

int check_accuracy_victim_location(victim_t found_victim, int *dx, int *dy)
{
    unsigned index;
    int accuracy;
    // First, find index of the found victim in the table
    for (index = 0; index < TOTAL_VICTIMS; ++index)
    {
        if (!strcmp(found_victim.id, VICTIM_TABLE[index].id))
            break;
    }
    // Last check, if tag (misteriously not found)
    if (index == TOTAL_VICTIMS)
    {
        fprintf(stderr, "Victim not found in table...\n");
        return 1;
    }
    else
    {
        accuracy = !(VICTIM_TABLE[index].x == found_victim.x &&
                VICTIM_TABLE[index].y == found_victim.y);
        // Victim location wrong, compute differences
        if (accuracy)
        {
            *dx = found_victim.x - VICTIM_TABLE[index].x;
            *dy = found_victim.y - VICTIM_TABLE[index].y;
        }
        // And return result
        return accuracy;
    }
}
