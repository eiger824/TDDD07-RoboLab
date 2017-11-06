/**
 * @file	task_control.c
 * @author  Eriks Zaharans
 * @date    31 Oct 2013
 *
 * @section DESCRIPTION
 *
 * Control Task.
 */

/* -- Includes -- */
/* system libraries */

/* project libraries */
#include "task.h"

 /**
 * Sense, control, localization
 */
void task_control(void)
{
	// Check if task is enabled
	if(g_task_control.enabled == s_TRUE)
	{
		int speed;
		int req_time;

		// Update sensors Odometer
#ifndef s_CONFIG_TEST_ENABLE
		// Request only if the allowed period has passed
		req_time = (int)timelib_timer_get(g_task_control_data.request_timer);
		if(req_time > s_CONFIG_OI_REQUEST_PERIOD)
		{
			openinterface_sensors_update(g_ois, s_OI_SENSOR_PACKET_2, s_OI_SENSOR_PACKET_2_SIZE);
			pf_drive(g_pfs, g_ois->oiss->distance, g_ois->oiss->angle, 0);
			timelib_timer_reset(&g_task_control_data.request_timer);
		}
		else
		{
			debug_printf("Control: runs too often!\n");

			g_ois->oiss->distance = 0;
			g_ois->oiss->angle = 0;
			pf_drive(g_pfs, g_ois->oiss->distance, g_ois->oiss->angle, 1);
		}
#endif
		// Motion Update (Particle filter)
		pf_estimate(g_pfs, g_robot);

		if(pf_accuracy(g_pfs, g_envs) < s_CONFIG_ACCURACY_LIMIT)
		{
			speed = g_config.robot_speed / 2;
		}
		else
		{
			speed = g_config.robot_speed;
		}

		// Send robot pose (add data to communication queue)
		debug_printf("Pose: %d, %d, %f\n", g_robot->x, g_robot->y, g_robot->a);

		doublylinkedlist_insert_end(g_list_send, g_robot, s_DATA_STRUCT_TYPE_ROBOT);

		// Check pipe for event
		if(g_tp_navigate_control.event == s_TASK_EVENT_SET)
		{

			// If there is new move then send drive command to robot
			switch(g_tp_navigate_control.move)
			{
			case s_PH_LEFT :
				debug_printf("PH_LEFT\n");
				openinterface_drive(g_ois, speed, 200);
				break;
			case s_PH_TOP_LEFT :
				debug_printf("PH_TOP_LEFT\n");
				openinterface_drive(g_ois, speed, 100);
				break;
			case s_PH_TOP :
				debug_printf("PH_TOP\n");
				openinterface_drive(g_ois, speed, 0x8000);
				break;
			case s_PH_TOP_RIGHT :
				debug_printf("PH_TOP_RIGHT\n");
				openinterface_drive(g_ois, speed, 0xFFFF - 100);
				break;
			case s_PH_RIGHT :
				debug_printf("PH_RIGHT\n");
				openinterface_drive(g_ois, speed, 0xFFFF - 200);
				break;
			case s_PH_NONE :
				debug_printf("PH_NONE\n");
				openinterface_drive(g_ois, 0, 0);
			default :
				// Do nothing
				break;
			}

			// Reset event
			g_tp_navigate_control.event = s_TASK_EVENT_RESET;
		}
	}
}