/**
 * @file	doublylinkedlist.h
 * @author  Eriks Zaharans
 * @date	21 Oct 2013
 *
 * @section DESCRIPTION
 *
 * Doubly linked list library header file.
 */

#ifndef __DOUBLYLINKEDLIST_H
#define __DOUBLYLINKEDLIST_H

/* -- Includes -- */

/* -- Enumurations -- */

/* -- Types -- */

/**
 * @brief Doubly linked list node structure
 */
typedef struct s_DOUBLYLINKEDLIST_NODE_STRUCT
{
	void *data;
	int data_type;
	struct s_DOUBLYLINKEDLIST_NODE_STRUCT *next;
	struct s_DOUBLYLINKEDLIST_NODE_STRUCT *prev;

} doublylinkedlist_node_t;

/**
 * @brief Doubly linked list structure
 */
typedef struct s_DOUBLYLINKEDLIST_STRUCT
{
	doublylinkedlist_node_t *first; // Pointer to the first node of the doubly linked list
	doublylinkedlist_node_t *last; // Pointer to the last node of the doubly linked list

	int count; // Number of nodes in the doubly linked list

} doublylinkedlist_t;

/* -- Constants -- */

/* -- Function Prototypes -- */

doublylinkedlist_t *doublylinkedlist_init(void); // Initialize doubly linked list
void doublylinkedlist_destroy(doublylinkedlist_t *dlls); // Deinitialize doubly linked list

void doublylinkedlist_insert_after(doublylinkedlist_t *dlls, doublylinkedlist_node_t *node, void *data, int data_type); // Add a new node after specified node
void doublylinkedlist_insert_before(doublylinkedlist_t *dlls, doublylinkedlist_node_t *node, void *data, int data_type); // Add a new node before specified node

void doublylinkedlist_insert_beginning(doublylinkedlist_t *dlls, void *data, int data_type); // Add a new node in the beginning of the doubly linked list
void doublylinkedlist_insert_end(doublylinkedlist_t *dlls, void *data, int data_type); // Add a new node in the end of the doubly linked list

void doublylinkedlist_remove(doublylinkedlist_t *dlls, doublylinkedlist_node_t *node, void *data, int *data_type); // Remove a node and retrieve the data from the doubly linked list structure
void doublylinkedlist_delete(doublylinkedlist_t *dlls, doublylinkedlist_node_t *node); // Delete a node from the doubly linked list structure

void doublylinkedlist_empty(doublylinkedlist_t *dlls); // Empty the doubly linked list

#endif /* __DOUBLYLINKEDLIST_H */
