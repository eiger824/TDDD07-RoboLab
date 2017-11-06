/**
 * @file	udp.h
 * @author  Eriks Zaharans
 * @date    4 Jul 2013
 *
 * @section DESCRIPTION
 *
 * UDP socket function library header file.
 */


#ifndef __UDP_H
#define __UDP_H

/* -- Includes -- */
/* system libraries */
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

/* -- Types -- */

 /**
 * @brief UDP structure
 */
typedef struct s_UDP_STRUCT
{
	// Socket address
	struct sockaddr_in sock_send;
	struct sockaddr_in sock_recv;
	struct sockaddr_in sock_conn;

	//
	socklen_t sock_conn_len;

	// Socket descriptors
	int sd_send;
	int sd_recv;

	// UDP settings
	char broadcast_ip[16];
	int port;
	int packet_size;


} udp_t;

/* -- Constants -- */

/* -- Function Prototypes -- */

int udp_init_broadcast(udp_t *udp);
int udp_init_receive(udp_t *udp);
udp_t *udp_open(char *broadcast_ip, int port, int packet_size); // Open UDP broadcast and recieve sockets
int udp_close(udp_t *udp); // Deinitialize UDP
int udp_broadcast(udp_t *udp, char *packet, int len); // Broadcast packet through UDP
int udp_receive(udp_t *udp, char *packet, int *len); // Receive packet through UDP
 


 #endif /* __UDP_H */