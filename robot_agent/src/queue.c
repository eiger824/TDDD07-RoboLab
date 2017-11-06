/**
 * @file	queue.c
 * @author  Eriks Zaharans
 * @date    4 Jul 2013
 *
 * @section DESCRIPTION
 *
 * FIFO queue library.
 */

/* -- Includes -- */
/* system libraries */
#include <stdio.h>
#include <stdlib.h>
/* project libraries */
#include "queue.h"
#include "def.h"
#include "robot.h"
#include "pheromone.h"

/* -- Defines -- */

/* -- Functions -- */

/**
 * Initialize FIFO queue
 * @return Pointer to queue structure
 */
queue_t *queue_init(void)
{
	// Allocate memory for Queue structure
	queue_t *qs = (queue_t *) malloc(sizeof(queue_t));

	// Init queue
	qs->head = NULL;
	qs->tail = NULL;
	qs->count = 0;

	return qs;
}

/**
 * Deinitialize FIFO queue
 * @param qs Pointer to queue structure
 * @return Void
 */
void queue_destroy(queue_t *qs)
{
	// Empty the queue
	queue_empty(qs);
	// Free memory
	free(qs);
}

/**
 * Add data structure to the end of the queue
 * @param qs Pointer to queue structure
 * @param data Pointer to data structure
 * @param data_type Data type
 * @return Void
 */
void queue_enqueue(queue_t *qs, void *data, int data_type)
{
	// Allocate memory for queue node structure
	queue_node_t *node = (queue_node_t *) malloc(sizeof(queue_node_t));

	// Increase count of nodes in the queue
	qs->count++;

	// Check if somebody is in the queue
	if(qs->head == NULL)
	{
		// If queue is empty, place node as head and tail
		qs->head = node;
		qs->tail = node;
	}
	else
	{
		// If there is, then inform tail node about new node behind it.
		qs->tail->next = node;
		// And make new node as the tail
		qs->tail = node;
	}


	// Init pointer to next
	node->next = NULL;
	// Save data type in node
	node->data_type = data_type;

	// Allocate memory for data structure and copy data
	switch(data_type)
	{
	// Robot pose
	case s_DATA_STRUCT_TYPE_ROBOT :
		node->data = (void *)malloc(sizeof(robot_t));
		*(robot_t *)node->data = *(robot_t *)data;
		break;
	// Victim information
	case s_DATA_STRUCT_TYPE_VICTIM :
		node->data = (void *)malloc(sizeof(victim_t));
		*(victim_t *)node->data = *(victim_t *)data;
		break;
	// Pheromone map
	case s_DATA_STRUCT_TYPE_PHEROMONE :
		node->data = (void *)malloc(sizeof(pheromone_map_sector_t));
		*(pheromone_map_sector_t *)node->data = *(pheromone_map_sector_t *)data;
		break;
	// Command
	case s_DATA_STRUCT_TYPE_CMD :
		node->data = (void *)malloc(sizeof(command_t));
		*(command_t *)node->data = *(command_t *)data;
		break;
    case s_DATA_STRUCT_TYPE_STREAM :
        node->data = (void *)malloc(sizeof(stream_t));
		*(stream_t *)node->data = *(stream_t *)data;
	// Other
	default :
		// Do nothing
		break;
	}

}

/**
 * Remove data structure from queue
 * @param qs Pointer to queue structure
 * @param data Pointer to memory to where save data structure
 * @param data_type Pointer to memory where to save data type
 * @return Void
 */
void queue_dequeue(queue_t *qs, void *data, int *data_type)
{
	// If queue is not empty, take out node and let next be the head
	if(qs->head != NULL)
	{
		// Save pointer to head node for later to be able to free the memory
		queue_node_t *temp_head_node = qs->head;

		// Decrease count of nodes in the queue
		qs->count--;

		// Save data type
		*data_type = qs->head->data_type;
		// Save data according to the data type
		switch(*data_type)
		{
		// Robot pose
		case s_DATA_STRUCT_TYPE_ROBOT :
			*(robot_t *)data = *(robot_t *)qs->head->data;
			break;
		// Victim information
		case s_DATA_STRUCT_TYPE_VICTIM :
			*(victim_t *)data = *(victim_t *)qs->head->data;
			break;
		// Pheromone map
		case s_DATA_STRUCT_TYPE_PHEROMONE :
			*(pheromone_map_sector_t *)data = *(pheromone_map_sector_t *)qs->head->data;
			break;
		// Command
		case s_DATA_STRUCT_TYPE_CMD :
			*(command_t *)data = *(command_t *)qs->head->data;
			break;
        case s_DATA_STRUCT_TYPE_STREAM :
            *(stream_t *)data = *(stream_t *)qs->head->data;
            break;
		// Other
		default :
			// Do nothing
			break;
		}

		// Make next node to be the head
		qs->head = qs->head->next;

		// Free memory (data)
		free(temp_head_node->data);
		// Free memory (node)
		free(temp_head_node);
	}
	else
	{
		// Otherwise tail empty
		qs->tail = NULL;
	}
}

/**
 * Empty the queue
 * @param qs Pointer to queue structure
 * @return Void
 */
void queue_empty(queue_t *qs)
{
	// Temporary pointer to head node
	queue_node_t *temp_head_node;

	// Take nodes out untill queue is empty
	while(qs->head != NULL)
	{
		// Save pointer to head node for later to be able to free the memory
		temp_head_node = qs->head;
		// Make next node to be the head
		qs->head = qs->head->next;
		// Free memory (data)
		free(temp_head_node->data);
		// Free memory (node)
		free(temp_head_node);
	}

	// Set number of nodes in the queue to zero
	qs->count = 0;

	// Deinit tail
	qs->tail = NULL;
}
