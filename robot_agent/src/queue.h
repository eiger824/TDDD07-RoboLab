/**
 * @file	queue.h
 * @author  Eriks Zaharans
 * @date	4 Jul 2013
 *
 * @section DESCRIPTION
 *
 * FIFO queue library header file.
 */

#ifndef __QUEUE_H
#define __QUEUE_H

/* -- Includes -- */

/* -- Enumurations -- */

/* -- Types -- */

/**
 * @brief Queue node structure
 */
typedef struct s_QUEUE_NODE_STRUCT
{
	void *data;
	int data_type;
	struct s_QUEUE_NODE_STRUCT *next;

} queue_node_t;

/**
 * @brief Queue structure
 */
typedef struct s_QUEUE_STRUCT
{
	queue_node_t *head; // Pointer to the head node of the queue
	queue_node_t *tail; // Pointer to the tail node of the queue

	int count; // Number of nodes in the queue

} queue_t;

/* -- Constants -- */

/* -- Function Prototypes -- */

queue_t *queue_init(void); // Initialize FIFO queue
void queue_destroy(queue_t *qs); // Deinitialize FIFO queue
void queue_enqueue(queue_t *qs, void *data, int data_type); // Add data structure to the end of the queue
void queue_dequeue(queue_t *qs, void *data, int *data_type); // Remove data structure from queue
void queue_empty(queue_t *qs); // Empty the queue

#endif /* __QUEUE_H */
