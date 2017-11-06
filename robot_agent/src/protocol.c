/**
 * @file	protocol.c
 * @author  Eriks Zaharans
 * @date	5 Jul 2013
 *
 * @section DESCRIPTION
 *
 * Functions to encode/decode communication packets.
 */

 /* -- Includes -- */
/* system libraries */
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <stdlib.h>
#include <math.h>
/* project libraries */
#include "protocol.h"
#include "config.h"
#include "def.h"
#include "rfid.h"
#include "openinterface.h"
#include "robot.h"
#include "pheromone.h"
#include "timelib.h"

/* -- Defines -- */

/* -- Functions -- */

/**
 * Encode data structure into string to be sent through UDP
 * @param udp_packet
 * @param len
 * @param recv_id
 * @param send_id
 * @param send_team
 * @param type
 * @param data_type
 * @param data
 * @return s_OK if successful, s_ERROR if failed
 */
int protocol_encode(char *udp_packet,
					int *len,
					int recv_id,
					int send_id,
					int send_team,
                    char type,
                    int seqno,
					int seqid,
					int seq_lid,
					int data_type,
					void *data)
{
	// Reset length
	*len = 0;


    int timestamp = (long long)timelib_unix_timestamp() % 60000;

	// Encode Reciever ID, Sender ID, Type, Timestamp, SequenceNumber, SequenceID, SequenceLastID
	*len += sprintf(udp_packet,"%d,%d,%d,%c,%d,%d,%d,%d",recv_id, send_id, send_team, type,timestamp,seqno,seqid,seq_lid);

    // Continue depending on the packet type
	switch(type)
	{
	// ACK
	case s_PROTOCOL_TYPE_ACK :

		// Do nothing
		break;

    //Go ahead
    case s_PROTOCOL_TYPE_GO_AHEAD :
        //This should not be done by the robot
        break;

	// Data
	case s_PROTOCOL_TYPE_DATA :

		// Encode Data type
		*len += sprintf(udp_packet + *len,",%d",data_type);

		// Continue depending on the data type
		switch(data_type)
		{
		// Robot
		case s_DATA_STRUCT_TYPE_ROBOT :

			// Encode Estimated robot pose
			*len += sprintf(udp_packet + *len,",%d,%d,%d",
				((robot_t *)data)->x,
				((robot_t *)data)->y,
				(int)(((robot_t *)data)->a * (180/M_PI)));

			break;
		// Victim
		case s_DATA_STRUCT_TYPE_VICTIM :

			// Encode Victim information
			*len += sprintf(udp_packet + *len,",%d,%d,%s",
				((victim_t *)data)->x,
				((victim_t *)data)->y,
				((victim_t *)data)->id);

			break;
		// Pheromone map
		case s_DATA_STRUCT_TYPE_PHEROMONE :

			// Encode pheromone information
			*len += sprintf(udp_packet + *len,",%d,%d,%d,",
				((pheromone_map_sector_t *)data)->num,
				((pheromone_map_sector_t *)data)->size,
				((pheromone_map_sector_t *)data)->timestamp);

			// Encode pheromone map data
			memcpy(udp_packet + *len, ((pheromone_map_sector_t *)data)->data, ((pheromone_map_sector_t *)data)->size);
			*len += ((pheromone_map_sector_t *)data)->size;

			break;
		// Command
		case s_DATA_STRUCT_TYPE_CMD :

			 // Encode Command information
			*len += sprintf(udp_packet + *len,",%d",
				((command_t *)data)->cmd);

			break;
        case s_DATA_STRUCT_TYPE_STREAM :

			 // Encode stream data information
			*len += sprintf(udp_packet + *len,",%ld,",
				((stream_t *)data)->counter);

			// Encode stream data
			memcpy(udp_packet + *len, ((stream_t *)data)->data, ((stream_t *)data)->size);
			*len += ((stream_t *)data)->size;

            break;
		// Other ?
		default:
			// Return error as unknown data type
			return s_ERROR;
			break;
		}
		break;
	// Other ?
	default:
		// Return error as unknown packet type
		return s_ERROR;
		break;
	}

	// End string
	udp_packet[*len] = '\0';

	return s_OK;
}

/**
 * Decode string received from UDP into interpretable data
 * @param packet
 * @param udp_packet
 * @param len
 * @param robot_id
 * @param robot_team
 * @return Pointer to protocol packet containing data
 */
int protocol_decode(protocol_t *packet, char *udp_packet, int len, int robot_id, int robot_team)
{
	// Local variables
	char *end_ptr, *pch;

	// Process data into struct
	// Save Reciever ID
	pch = strtok(udp_packet, ",");
	packet->recv_id = strtol(pch, &end_ptr, 10);
	// Save Sender ID
	pch = strtok(NULL, ",");
	packet->send_id = strtol(pch, &end_ptr, 10);
	// Check if it is not sent by yourself. If yes, drop.
	if(packet->send_id == robot_id)
		return s_ERROR;
	// Save Sender Team
	pch = strtok(NULL, ",");
	packet->send_team = strtol(pch, &end_ptr, 10);
	// Check if it is not sent by other team. If yes, drop.
	// Or if send_team = 0, then ignore team ID
	if(packet->send_team != robot_team && robot_team != 0)
		return s_ERROR;

	// Save Packet type
	pch = strtok(NULL, ",");
	memcpy(&packet->type, pch, 1);

    //Save packet timestamp
    pch = strtok(NULL, ",");
	packet->send_time = strtol(pch, &end_ptr, 10);

	//Save the packet sequence number
	pch = strtok(NULL, ",");
	packet->pkt_seqno= strtol(pch, &end_ptr, 10);

    //Save the sequence id
	pch = strtok(NULL, ",");
	packet->pkt_seqid= strtol(pch, &end_ptr, 10);

	//Save the last sequence id
	pch = strtok(NULL, ",");
	packet->seq_last_id= strtol(pch, &end_ptr, 10);

	// Now decoding depends on the type of the packet
	switch(packet->type)
	{
	// ACK
	case s_PROTOCOL_TYPE_ACK :
		// Do nothing
		break;

    case s_PROTOCOL_TYPE_GO_AHEAD :
        //There is no data

        break;

	// Data
	case s_PROTOCOL_TYPE_DATA :
		// Save Data type
		pch = strtok(NULL, ",");
		packet->data_type = strtol(pch, &end_ptr, 10);

		// Continue depending on the data type
		switch(packet->data_type)
		{
		// Robot
		case s_DATA_STRUCT_TYPE_ROBOT :

			// Allocate memory for robot structure
			packet->data = (void *)malloc(sizeof(robot_t));

			// Save data in structure
			// Robot X
			pch = strtok(NULL, ",");
			((robot_t *)packet->data)->x = strtol(pch, &end_ptr, 10);
			// Robot Y
			pch = strtok(NULL, ",");
			((robot_t *)packet->data)->y = strtol(pch, &end_ptr, 10);
			// Robot heading angle
			pch = strtok(NULL, ",");
			((robot_t *)packet->data)->a = (float)strtol(pch, &end_ptr, 10) * (float)(M_PI/180);

			break;
		// Victim
		case s_DATA_STRUCT_TYPE_VICTIM :

			// Allocate memory for victim structure
			packet->data = (void *)malloc(sizeof(victim_t));

			// Save data in structure
			// Victims X
			pch = strtok(NULL, ",");
			((victim_t *)packet->data)->x = strtol(pch, &end_ptr, 10);
			// Victims Y
			pch = strtok(NULL, ",");
			((victim_t *)packet->data)->y = strtol(pch, &end_ptr, 10);
			// Victims ID
			pch = strtok(NULL, ",");
			memcpy(((victim_t *)packet->data)->id, pch, 11);
			((victim_t *)packet->data)->id[10] = 0;

			break;
		// Pheromone map
		case s_DATA_STRUCT_TYPE_PHEROMONE :

			// Allocate memory for pheromone map sector structure
			packet->data = (void *)malloc(sizeof(pheromone_map_sector_t));

			// Save data in structure
			// Map sector number
			pch = strtok(NULL, ",");
			((pheromone_map_sector_t *)packet->data)->num = strtol(pch, &end_ptr, 10);
			// Map sector size
			pch = strtok(NULL, ",");
			((pheromone_map_sector_t *)packet->data)->size = strtol(pch, &end_ptr, 10);
			// Map sector timestamp
			pch = strtok(NULL, ",");
			((pheromone_map_sector_t *)packet->data)->timestamp = strtol(pch, &end_ptr, 10);


			//printf("??? len: %d\n", len);
			//printf("memcpy -|%c|%c|%x|%x|\n", *(end_ptr-2),*(end_ptr-1),*(end_ptr),*(end_ptr+1));
			// Map sector data
			pch = strtok(NULL, ",");
			// Copy data
			memcpy(((pheromone_map_sector_t *)packet->data)->data, pch, ((pheromone_map_sector_t *)packet->data)->size);
			//printf("memcpy_end\n");


			break;
		// Command
		case s_DATA_STRUCT_TYPE_CMD :

			// Allocate memory for command structure
			packet->data = (void *)malloc(sizeof(command_t));

			// Save data in structure
			// Command
			pch = strtok(NULL, " ,");
			((command_t *)packet->data)->cmd = strtol(pch, &end_ptr, 10);

			break;


        case s_DATA_STRUCT_TYPE_STREAM :
            // Allocate memory for stream structure
			packet->data = (void *)malloc(sizeof(stream_t));

			// Save data in structure
			// Stream counter
			pch = strtok(NULL, ",");
			((stream_t *)packet->data)->counter = strtol(pch, &end_ptr, 10);
			// Stream size
			pch = strtok(NULL, ",");
			((stream_t *)packet->data)->size = strtol(pch, &end_ptr, 10);

			// Stream data
			pch = strtok(NULL, ",");
			// Copy data
			memcpy(((stream_t *)packet->data)->data, pch, ((stream_t *)packet->data)->size);

			break;

		// Other ?
		default:
			// Return error as unknown data type
			return s_ERROR;
			break;
		}

		break;
	// Other ?
	default:
		// Return error as unknown packet type
		return s_ERROR;
		break;
	}

	return s_OK;
}

/**
 *
 * @param
 * @param
 * @return
 */
int protocol_destroy()
{
	return 0;
}
