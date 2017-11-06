/**
 * @file	protocol.h
 * @author  Eriks Zaharans
 * @date	5 Jul 2013
 *
 * @section DESCRIPTION
 *
 * Functions to decode communication packets header file.
 */


#ifndef __PROTOCOL_H
#define __PROTOCOL_H

/* -- Includes -- */

/* -- Types -- */

 /**
 * @brief UDP structure
 */
typedef struct s_PROTOCOL_STRUCT
{
	int recv_id; // Receiver ID
	int send_id; // Sender ID
	int send_team; // Sender Team
	char type; // Packet type
	int send_time;
	int pkt_seqno; //Sequence number of the packet
    int pkt_seqid; //Sequence id of the packet.
	int seq_last_id; //The last id of the sequence
	char data_type; // Data type
	void *data; // Pointer to data structure

} protocol_t;

/* -- Constants -- */

/* Protocol Addresses */
#define s_PROTOCOL_ADDR_SERVER		0
#define s_PROTOCOL_ADDR_BROADCAST	99

/* Packet types */
#define s_PROTOCOL_TYPE_ACK			'a'
#define s_PROTOCOL_TYPE_DATA		'd'
#define s_PROTOCOL_TYPE_GO_AHEAD    'g'

/* -- Function Prototypes -- */
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
					void *data); // Encode data structure into string to be sent through UDP

/* Decode the data structure contanied in the udp packet */
int protocol_decode(protocol_t *packet, char *udp_packet, int len, int robot_id, int robot_team);



 #endif /* __PROTOCOL_H */
