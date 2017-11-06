/**
 * @file	task_navigate.c
 * @author  Eriks Zaharans
 * @date    31 Oct 2013
 *
 * @section DESCRIPTION
 *
 * Navigate Task.
 */

/* -- Includes -- */
/* system libraries */

/* project libraries */
#include "task.h"

 /**
 * Control navigation
 */
void task_navigate(void)
{
	// Check if task is enabled
	if(g_task_navigate.enabled == s_TRUE)
	{
		// Local vaiables
		int i;
		void *data; // Void pointer for data
		int data_type; // Data type

		// Check Queue for new Pheromone map sectors
		// If present -> Update Pheromone map
		// For testing purposes process all data in the queue
		while(g_queue_navigate->count != 0)
		{

			// Allocate memory for data structure
			if(g_queue_navigate->head->data_type == s_DATA_STRUCT_TYPE_PHEROMONE)
			{
				// Allocate memory
				data = (void *)malloc(sizeof(pheromone_map_sector_t));
				// Get data from the queue
				queue_dequeue(g_queue_navigate, data, &data_type);
				// Update Pheromone map
				pheromone_map_update(g_phs, data);
				// Free data
				free(data);
			}
			else
			{
				// Throw out of the queue !!!
			}
		}

		// Put pheromone only of position is reasonably accurate 
		if(pf_accuracy(g_pfs, g_envs) > s_CONFIG_ACCURACY_LIMIT)
		{
			pheromone_put(g_phs, g_robot->x, g_robot->y);
		}



		// Extract Pheromone map into sectors
		pheromone_map_sector_t **phms = pheromone_map_extract(g_phs);
		// Send pheromone map sectors (add data to communication queue)
		for(i = 0; i < g_phs->sector_count; i++)
		{
			doublylinkedlist_insert_end(g_list_send, phms[i], s_DATA_STRUCT_TYPE_PHEROMONE);
		}
		// Free memory
		pheromone_map_destroy(g_phs, phms);

		//Massi:check the go_ahead
		if(g_go_ahead){
			// Decide next move and send it to control
			g_tp_navigate_control.move = pheromone_eval(g_phs, g_robot);

		}else{
		// Decide next move and send it to control
			g_tp_navigate_control.move = s_PH_NONE;
		}

		// Set event
		g_tp_navigate_control.event = s_TASK_EVENT_SET;

	}
}