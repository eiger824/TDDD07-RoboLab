/**
 * @file    def.h
 * @author  Eriks Zaharans
 * @date    1 Jul 2013
 *
 * @section DESCRIPTION
 *
 * Definition file.
 */

#ifndef __DEF_H
#define __DEF_H

/* -- Includes -- */
/* system libraries */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
/* project libraries */
#include "config.h"

/* -- Types -- */


/**
 * @brief Victim structure
 */
typedef struct s_VICTIM_STRUCT
{
	int x; // X coordinate
	int y; // Y coordinate
	char id[11]; // ID

} victim_t;

/**
 * @brief Command structure
 */
typedef struct s_COMMAND_STRUCT
{
	int cmd; // command

} command_t;

/**
 * @brief Stream structure
 */
typedef struct s_STREAM_STRUCT
{
	long counter; // Stream counter
	int size; // Stream size
	unsigned char data[s_CONFIG_STREAM_SIZE]; // Stream data

} stream_t;


/* -- Defines -- */

/* Boolean */
#define s_TRUE              1
#define s_FALSE             0

/* Function returns */
#define s_OK                0
#define s_ERROR             -1

/* Data Structure types */
#define s_DATA_STRUCT_TYPE_ROBOT		0
#define s_DATA_STRUCT_TYPE_VICTIM		1
#define s_DATA_STRUCT_TYPE_PHEROMONE	2
#define s_DATA_STRUCT_TYPE_CMD			3
#define s_DATA_STRUCT_TYPE_STREAM		4

/* Commands */
#define s_CMD_START		0
#define s_CMD_STOP		1
#define s_CMD_GO_AHEAD	2


#endif /* __DEF_H */
