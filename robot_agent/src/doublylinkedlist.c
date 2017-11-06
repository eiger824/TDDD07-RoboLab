/**
 * @file	doublylinkedlist.c
 * @author  Eriks Zaharans
 * @date	21 Oct 2013
 *
 * @section DESCRIPTION
 *
 * Doubly linked list library.
 */

/* -- Includes -- */
/* system libraries */
#include <stdio.h>
#include <stdlib.h>
/* project libraries */
#include "doublylinkedlist.h"
#include "def.h"
#include "robot.h"
#include "pheromone.h"

/* -- Defines -- */

/* -- Functions -- */

/**
 * Initialize doubly linked list
 * @return Pointer to doubly linked list structure
 */
doublylinkedlist_t *doublylinkedlist_init(void)
{
	// Allocate memory for doubly linked list structure
	doublylinkedlist_t *dlls = (doublylinkedlist_t *) malloc(sizeof(doublylinkedlist_t));

	// Init doubly linked list
	dlls->first = NULL;
	dlls->last = NULL;
	dlls->count = 0;

	return dlls;
}

/**
 * Deinitialize doubly linked list
 * @param dll Pointer to doubly linked list structure
 * @return Void
 */
void doublylinkedlist_destroy(doublylinkedlist_t *dlls)
{
	// Empty the doubly linked list
	doublylinkedlist_empty(dlls);
	// Free memory
	free(dlls);
}

/**
 * Add a new node after specified node
 * @param dlls Pointer to doubly linked list structure
 * @param node Pointer to node structure after which a new node should be placed
 * @param data Pointer to data structure
 * @param data_type Data type
 * @return Void
 */
void doublylinkedlist_insert_after(doublylinkedlist_t *dlls, doublylinkedlist_node_t *node, void *data, int data_type)
{
	// Allocate memory for new node structure
	doublylinkedlist_node_t *new_node = (doublylinkedlist_node_t *) malloc(sizeof(doublylinkedlist_node_t));

	// Increase count of nodes in the doubly linked list
	dlls->count++;

	// Set pointers to next and previous node for the new node
	new_node->prev = node;
	new_node->next = node->next;

	// Check if the node was in the end of the list
	if(node->next == NULL)
	{
		// If yes, than it is placed as last
		dlls->last = new_node;
	}
	else
	{
		// Otherwise next node is informed about change of its previous node
		node->next->prev = new_node;
	}
	// Update nodes next node to new as it is placed in front of it
	node->next = new_node;

	// Save data type in node
	new_node->data_type = data_type;

	// Allocate memory for data structure and copy data
	switch(data_type)
	{
	// Robot pose
	case s_DATA_STRUCT_TYPE_ROBOT :
		new_node->data = (void *)malloc(sizeof(robot_t));
		*(robot_t *)new_node->data = *(robot_t *)data;
		break;
	// Victim information
	case s_DATA_STRUCT_TYPE_VICTIM :
		new_node->data = (void *)malloc(sizeof(victim_t));
		*(victim_t *)new_node->data = *(victim_t *)data;
		break;
	// Pheromone map
	case s_DATA_STRUCT_TYPE_PHEROMONE :
		new_node->data = (void *)malloc(sizeof(pheromone_map_sector_t));
		*(pheromone_map_sector_t *)new_node->data = *(pheromone_map_sector_t *)data;
		break;
	// Command
	case s_DATA_STRUCT_TYPE_CMD :
		new_node->data = (void *)malloc(sizeof(command_t));
		*(command_t *)new_node->data = *(command_t *)data;
		break;
    case s_DATA_STRUCT_TYPE_STREAM :
        new_node->data = (void *)malloc(sizeof(stream_t));
		*(stream_t *)new_node->data = *(stream_t *)data;
	// Other
	default :
		// Do nothing
		break;
	}

}

/**
 * Add a new node before specified node
 * @param dlls Pointer to doubly linked list structure
 * @param node Pointer to node structure before which a new node should be placed
 * @param data Pointer to data structure
 * @param data_type Data type
 * @return Void
 */
void doublylinkedlist_insert_before(doublylinkedlist_t *dlls, doublylinkedlist_node_t *node, void *data, int data_type)
{
	// Allocate memory for new node structure
	doublylinkedlist_node_t *new_node = (doublylinkedlist_node_t *) malloc(sizeof(doublylinkedlist_node_t));

	// Increase count of nodes in the doubly linked list
	dlls->count++;

	// Set pointers to next and previous node for the new node
	new_node->prev = node->prev;
	new_node->next = node;

	// Check if the node was in the beginning of the list
	if(node->prev == NULL)
	{
		// If yes, than it is placed as last
		dlls->first = new_node;
	}
	else
	{
		// Otherwise previous node is informed about change of its next node
		node->prev->next = new_node;
	}
	// Update nodes previous node to new as it is placed in back of it
	node->prev = new_node;

	// Save data type in node
	new_node->data_type = data_type;

	// Allocate memory for data structure and copy data
	switch(data_type)
	{
	// Robot pose
	case s_DATA_STRUCT_TYPE_ROBOT :
		new_node->data = (void *)malloc(sizeof(robot_t));
		*(robot_t *)new_node->data = *(robot_t *)data;
		break;
	// Victim information
	case s_DATA_STRUCT_TYPE_VICTIM :
		new_node->data = (void *)malloc(sizeof(victim_t));
		*(victim_t *)new_node->data = *(victim_t *)data;
		break;
	// Pheromone map
	case s_DATA_STRUCT_TYPE_PHEROMONE :
		new_node->data = (void *)malloc(sizeof(pheromone_map_sector_t));
		*(pheromone_map_sector_t *)new_node->data = *(pheromone_map_sector_t *)data;
		break;
	// Command
	case s_DATA_STRUCT_TYPE_CMD :
		new_node->data = (void *)malloc(sizeof(command_t));
		*(command_t *)new_node->data = *(command_t *)data;
		break;
    case s_DATA_STRUCT_TYPE_STREAM :
        new_node->data = (void *)malloc(sizeof(stream_t));
		*(stream_t *)new_node->data = *(stream_t *)data;
	// Other
	default :
		// Do nothing
		break;
	}

}


/**
 * Add a new node in the beginning of the doubly linked list
 * @param dlls Pointer to doubly linked list structure
 * @param data Pointer to data structure
 * @param data_type Data type
 * @return Void
 */
void doublylinkedlist_insert_beginning(doublylinkedlist_t *dlls, void *data, int data_type)
{

	// Check if there are any nodes in the doubly linked list
	// If no, then create first node
	if(dlls->first == NULL)
	{
		// Allocate memory for new node structure
		doublylinkedlist_node_t *new_node = (doublylinkedlist_node_t *) malloc(sizeof(doublylinkedlist_node_t));

		// Increase count of nodes in the doubly linked list
		dlls->count++;

		// As this is the first and only node in the list, then it is also logically the last one too.
		dlls->first = new_node;
		dlls->last = new_node;

		// There are no previous or next nodes.
		new_node->next = NULL;
		new_node->prev = NULL;

		// Save data type in node
		new_node->data_type = data_type;

		// Allocate memory for data structure and copy data
		switch(data_type)
		{
		// Robot pose
		case s_DATA_STRUCT_TYPE_ROBOT :
			new_node->data = (void *)malloc(sizeof(robot_t));
			*(robot_t *)new_node->data = *(robot_t *)data;
			break;
		// Victim information
		case s_DATA_STRUCT_TYPE_VICTIM :
			new_node->data = (void *)malloc(sizeof(victim_t));
			*(victim_t *)new_node->data = *(victim_t *)data;
			break;
		// Pheromone map
		case s_DATA_STRUCT_TYPE_PHEROMONE :
			new_node->data = (void *)malloc(sizeof(pheromone_map_sector_t));
			*(pheromone_map_sector_t *)new_node->data = *(pheromone_map_sector_t *)data;
			break;
		// Command
		case s_DATA_STRUCT_TYPE_CMD :
			new_node->data = (void *)malloc(sizeof(command_t));
			*(command_t *)new_node->data = *(command_t *)data;
			break;
	    case s_DATA_STRUCT_TYPE_STREAM :
	        new_node->data = (void *)malloc(sizeof(stream_t));
			*(stream_t *)new_node->data = *(stream_t *)data;
		// Other
		default :
			// Do nothing
			break;
		}
	}
	// Else place a new node before the first node
	else
	{
		doublylinkedlist_insert_before(dlls, dlls->first, data, data_type);
	}
}

/**
 * Add a new node in the end of the doubly linked list
 * @param dlls Pointer to doubly linked list structure
 * @param data Pointer to data structure
 * @param data_type Data type
 * @return Void
 */
void doublylinkedlist_insert_end(doublylinkedlist_t *dlls, void *data, int data_type)
{
	// Check if there are any nodes in the doubly linked list
	// If no, then create first node
	if(dlls->last == NULL)
	{
		doublylinkedlist_insert_beginning(dlls, data, data_type);
	}
	// Else place a new node after the last node
	else
	{
		doublylinkedlist_insert_after(dlls, dlls->last, data, data_type);
	}
}

/**
 * Remove a node and retrieve the data from the doubly linked list structure
 * @param dlls Pointer to doubly linked list structure
 * @param node Pointer to node structure which should be removed
 * @param data Pointer to memory to where save data structure
 * @param data_type Pointer to memory where to save data type
 * @return Void
 */
void doublylinkedlist_remove(doublylinkedlist_t *dlls, doublylinkedlist_node_t *node, void *data, int *data_type)
{
	// Save data type
	*data_type = node->data_type;
	// Save data according to the data type
	switch(*data_type)
	{
	// Robot pose
	case s_DATA_STRUCT_TYPE_ROBOT :
		*(robot_t *)data = *(robot_t *)node->data;
		break;
	// Victim information
	case s_DATA_STRUCT_TYPE_VICTIM :
		*(victim_t *)data = *(victim_t *)node->data;
		break;
	// Pheromone map
	case s_DATA_STRUCT_TYPE_PHEROMONE :
		*(pheromone_map_sector_t *)data = *(pheromone_map_sector_t *)node->data;
		break;
	// Command
	case s_DATA_STRUCT_TYPE_CMD :
		*(command_t *)data = *(command_t *)node->data;
		break;
	case s_DATA_STRUCT_TYPE_STREAM :
		*(stream_t *)data = *(stream_t *)node->data;
		break;
	// Other
	default :
		// Do nothing
		break;
	}

	// Delete the node
	doublylinkedlist_delete(dlls, node);
}

/**
 * Delete a node from the doubly linked list structure
 * @param dlls Pointer to doubly linked list structure
 * @param node Pointer to node structure which should be removed
 * @return Void
 */
void doublylinkedlist_delete(doublylinkedlist_t *dlls, doublylinkedlist_node_t *node)
{
	// Check if the node is the first one in the list
	// If yes, then make next node as the first now
	if(node->prev == NULL)
	{
		dlls->first = node->next;
	}
	// Else inform previous node that it has a new next node
	else
	{
		node->prev->next = node->next;
	}

	// Check if the node is the last one in the list
	// If yes, then make previous node as the last now
	if(node->next == NULL)
	{
		dlls->last = node->prev;
	}
	// Else inform next node that it has a new previous node
	else
	{
		node->next->prev = node->prev;
	}

	// Free memory (data)
	free(node->data);
	// Free memory (node)
	free(node);
	// Decrease count of nodes in the doubly linked list
	dlls->count--;
}

/**
 * Empty the doubly linked list
 * @param dlls Pointer to doubly linked list structure
 * @return Void
 */
void doublylinkedlist_empty(doublylinkedlist_t *dlls)
{
	// Temporary pointer to first node
	doublylinkedlist_node_t *temp_first_node;

	// Take nodes out until doubly linked list is empty
	while(dlls->first != NULL)
	{
		// Save pointer to first node for later to be able to free the memory
		temp_first_node = dlls->first;
		// Make next node to be the first
		dlls->first = dlls->first->next;
		// Free memory (data)
		free(temp_first_node->data);
		// Free memory (node)
		free(temp_first_node);
	}

	// Set number of nodes in the doubly linked list to zero
	dlls->count = 0;

	// Deinit last
	dlls->last = NULL;
}
