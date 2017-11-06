/**
 * @file	task_mission.c
 * @author  Eriks Zaharans
 * @date    31 Oct 2013
 *
 * @section DESCRIPTION
 *
 * Mission Task.
 */

/* -- Includes -- */
/* system libraries */

/* project libraries */
#include "task.h"

/**
 * Control mission
 */
void task_mission(void)
{
	// Check if task is enabled
	if(g_task_mission.enabled == s_TRUE)
	{
		// -- Loacal variables --
		int i, j;
		void *data; // Void pointer for data
		int data_type; // Data type
		int stream_time, stream_time_reminder, stream_packet_count; // Stream time to be sent
		int go_ahead_time; // Go ahead time passed
		stream_t stream;

		// -- Check go ahead timer --
		go_ahead_time = (int)timelib_timer_get(g_task_mission_data.go_ahead_timer);
		if(go_ahead_time > s_CONFIG_GO_AHEAD_TIME)
		{
			//g_go_ahead = 0;
		}

		// -- Process all data in the queue --
		while(g_queue_mission->count != 0)
		{
			// Allocate memory for data structure
			switch(g_queue_mission->head->data_type)
			{
			// Victim information
			case s_DATA_STRUCT_TYPE_VICTIM :
				data = (void *)malloc(sizeof(victim_t));
				break;
			// Command
			case s_DATA_STRUCT_TYPE_CMD :
				data = (void *)malloc(sizeof(command_t));
				break;
			// Other
			default :
				// Do nothing
				break;
			}

			// Get data from the queue
			queue_dequeue(g_queue_mission, data, &data_type);

			// Choose action depending on data type
			switch(data_type)
			{
			// Victim information
			case s_DATA_STRUCT_TYPE_VICTIM :

				// Check if this victim is new
				for(i = 0; i < g_task_mission_data.victim_count; i++)
				{
					// If there is victim with the same ID -> stop searching
					if(strcmp(g_task_mission_data.victims[i].id, ((victim_t *)data)->id) == 0)
					{
						break;
					}
				}
				// If counter is the same as victim count -> new victim is found
				// Add it to the list
				if(i == g_task_mission_data.victim_count)
				{
					g_task_mission_data.victims[g_task_mission_data.victim_count] = *((victim_t *)data);
					g_task_mission_data.victim_count++;
				}

				break;
			// Command
			case s_DATA_STRUCT_TYPE_CMD :

				// Choose action depending on command
				switch(((command_t *)data)->cmd)
				{
				// Start tasks
				case s_CMD_START :
					g_task_mission.enabled 			= s_TRUE;
					g_task_navigate.enabled 		= s_TRUE;
					g_task_control.enabled 			= s_TRUE;
					g_task_refine.enabled 			= s_TRUE;
					g_task_report.enabled 			= s_TRUE;
					g_task_communicate.enabled 		= s_TRUE;
					g_task_avoid.enabled 			= s_TRUE;

                    debug_printf("cmd start\n");

					break;
				// Stop tasks
				case s_CMD_STOP :
					g_task_mission.enabled 			= s_TRUE;
					g_task_navigate.enabled 		= s_FALSE;
					g_task_control.enabled 			= s_FALSE;
					g_task_avoid.enabled 			= s_FALSE;
					g_task_refine.enabled 			= s_FALSE;
					g_task_report.enabled 			= s_FALSE;
					g_task_communicate.enabled 		= s_TRUE;

					debug_printf("cmd stop\n");

					// Get full control over robot
					openinterface_drive(g_ois, 0, 0x8000);
					break;
				case s_CMD_GO_AHEAD :
					// Go Ahead received
					g_go_ahead = 1;
					timelib_timer_reset(&g_task_mission_data.go_ahead_timer);
					break;
				// Other
				default :
					// Do nothing
					break;
				}

				break;
			// Other
			default :
				// Do nothing
				break;
			}

			// Free memory
			free(data);
		}

		// -- Check pipe for event --
		if(g_tp_report_mission.event == s_TASK_EVENT_SET)
		{

			// Check if this victim is new
			for(i = 0; i < g_task_mission_data.victim_count; i++)
			{
				// If there is victim with the same ID -> stop searching
				if(strcmp(g_task_mission_data.victims[i].id, g_tp_report_mission.victim.id) == 0)
				{
					break;
				}
			}
			// If counter is the same as victim count -> new victim is found
			// Add it to the list
			if(i == g_task_mission_data.victim_count)
			{
				g_task_mission_data.victims[g_task_mission_data.victim_count] = g_tp_report_mission.victim;
				g_task_mission_data.victim_count++;
			}

			// Reset event
			g_tp_report_mission.event = s_TASK_EVENT_RESET;
		}

		// -- Generate stream data --
		// Get time for stream
		stream_time = (int)timelib_timer_reset(&g_task_mission_data.stream_timer);
		stream_packet_count = (int)(stream_time / (1000 / (float)s_CONFIG_STREAM_RATE));

		//stream_time_reminder = (int)fmod(stream_time, (1000 / (float)s_CONFIG_STREAM_RATE)); // Calculate reminder not to loose miliseconds
		//timelib_timer_add_ms(&g_task_mission_data.stream_timer, stream_time_reminder); // Add them back

		//debug_printf("STREAM: %d, %d\n", stream_time, stream_packet_count);

		// Generate and send stream
		for(i = 0; i < stream_packet_count; i++)
		{
			stream.counter = g_task_mission_data.stream_counter;
			stream.size = s_CONFIG_STREAM_SIZE;

			// Gnerate random data
			for(j = 0; j < s_CONFIG_STREAM_SIZE; j++)
			{
				// For test copy just ones
				stream.data[j] = 1;
			}

			// Place data in a list
			doublylinkedlist_insert_end(g_list_send, &stream, s_DATA_STRUCT_TYPE_STREAM);
			// Increment stream packet counter
			g_task_mission_data.stream_counter++;
		}

	}
}
