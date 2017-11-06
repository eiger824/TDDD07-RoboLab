/**
 * @file	task_avoid.c
 * @author  Eriks Zaharans
 * @date    31 Oct 2013
 *
 * @section DESCRIPTION
 *
 * Collision detection (avoid) Task.
 */

/* -- Includes -- */
/* system libraries */

/* project libraries */
#include "task.h"

 /**
 * Check the bump sensors for collision
 */
void task_avoid(void)
{
	if(g_task_avoid.enabled == s_TRUE)
	{
		// Update Bump sensor
#ifndef s_CONFIG_TEST_ENABLE
		openinterface_sensors_update(g_ois, s_OI_SENSOR_PACKET_1, s_OI_SENSOR_PACKET_1_SIZE);
#endif

		// Check collision
        if(g_ois->oiss->wheeldrop_bump > 0)
		{
			debug_printf("Collision\n");
		}

		// Choose direction of rotation depending on the side it hit
		// Right Bump
		if(g_ois->oiss->wheeldrop_bump == 1)
		{
			openinterface_drive(g_ois, g_config.robot_speed, 0x0001);
			return;
		}
		// Left Bump
		else if(g_ois->oiss->wheeldrop_bump == 2)
		{
			openinterface_drive(g_ois, g_config.robot_speed, 0xFFFF);
			return;
		}
		// Center Bump
		else if(g_ois->oiss->wheeldrop_bump == 3)
		{
			if((rand() % 10) > 5)
				openinterface_drive(g_ois, g_config.robot_speed, 0xFFFF);
			else
				openinterface_drive(g_ois, g_config.robot_speed, 0x0001);

			return;
		}
	}
}