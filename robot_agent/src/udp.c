/**
 * @file	udp.c
 * @author  Eriks Zaharans
 * @date    4 Jul 2013
 *
 * @section DESCRIPTION
 *
 * UDP socket function library.
 */

 /* -- Includes -- */
/* system libraries */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
/* project libraries */
#include "udp.h"
#include "config.h"
#include "def.h"

 /* -- Defines -- */

/* -- Functions -- */


/**
 * Initialize UDP broadcasting
 * @param udp Pointer to UDP structure
 * @return s_OK if successful, s_ERROR if failed
 */
int udp_init_broadcast(udp_t *udp)
{
	// Local variables
	int broadcastEnable = 1;

	// Open socket
	if((udp->sd_send = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == s_ERROR) {
		printf("udp_init_broadcast: could not open a send socket.\n");
		return s_ERROR;
	}

	// Enable broadcast
	if(setsockopt(udp->sd_send, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable)) == s_ERROR) {
		printf("udp_init_broadcast: could not enable broadcast.\n");
		return s_ERROR;
	}

	// Configure IP and port
	memset((char *) &udp->sock_send, 0, sizeof(udp->sock_send));
	udp->sock_send.sin_family = AF_INET;
	udp->sock_send.sin_port = htons(udp->port);

	if(inet_pton(AF_INET, udp->broadcast_ip, &udp->sock_send.sin_addr) == s_ERROR) {
        printf("udp_init_broadcast: inet_pton failed.\n");
		return s_ERROR;
	}


	return s_OK;
}

/**
 * Initialize UDP recive
 * @param udp Pointer to UDP structure
 * @return s_OK if successful, s_ERROR if failed
 */
int udp_init_receive(udp_t *udp)
{
	// Open socket
	if((udp->sd_recv = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == s_ERROR) {
		printf("udp_init_receive: could not open a receive socket.\n");
		return s_ERROR;
	}

	// Configure IP and port
	memset((char *) &udp->sock_recv, 0, sizeof(udp->sock_recv));
	udp->sock_recv.sin_family = AF_INET;
	udp->sock_recv.sin_port = htons(udp->port);
	udp->sock_recv.sin_addr.s_addr = htonl(INADDR_ANY);

	// Bind socket
	if(bind(udp->sd_recv, (struct sockaddr *)&udp->sock_recv, sizeof(udp->sock_recv)) == s_ERROR) {
		printf("udp_init_receive: could not bind a socket.\n");
		return s_ERROR;
	}

	// Sender socket len
	udp->sock_conn_len = sizeof(udp->sock_conn);

	return s_OK;
}

/**
 * Open UDP broadcast and recieve sockets
 * @param broadcast_ip UDP broadcast IP address
 * @param port UDP port
 * @param packet_size UDP packet size
 * @return Pointer to UDP structure if successful, Null pointer if failed
 */
udp_t *udp_open(char *broadcast_ip, int port, int packet_size)
{
	// Allocate memory for UDP structure
	udp_t *udp = (udp_t *)malloc(sizeof(udp_t));

	strcpy(udp->broadcast_ip, broadcast_ip);
	udp->port = port;
	udp->packet_size = packet_size;

	// Initialize broadcasting
	if(udp_init_broadcast(udp) == s_ERROR)
		return NULL;

	// Initialize reception
	if(udp_init_receive(udp) == s_ERROR)
		return NULL;

	return udp;
}

/**
 * Deinitialize UDP
 * @param udp Pointer to UDP structure
 * @return s_OK if successful, s_ERROR if failed
 */
int udp_close(udp_t *udp)
{
	int res;

	// Close sockets
	res = close(udp->sd_send);
	res += close(udp->sd_recv);

	// Free memory
	free(udp);

	if(res < 0)
	{
		printf("udp_close: could not close UDP sockets.\n");
		return s_ERROR;
	}
	else
	{
		return s_OK;
	}
}

/**
 * Broadcast packet through UDP
 * @param udp Pointer to UDP structure
 * @param packet Pointer to data packet to be broadcasted
 * @param len Number of bytes to be broadcasted
 * @return s_OK if successful, s_ERROR if failed
 */
int udp_broadcast(udp_t *udp, char *packet, int len)
{

	// Broadcast UDP packet/datagram
	if(sendto(udp->sd_send, packet, len, 0, (struct sockaddr *)&udp->sock_send, sizeof(udp->sock_send)) == s_ERROR) {
		printf("udp_broadcast: could not send UDP packet.\n");
		return s_ERROR;
	}

	return s_OK;
}

/**
 * Receive packet through UDP
 * @param udp Pointer to UDP structure
 * @param packet Pointer to memory where to save received packet
 * @param len Pointer to memory where to save number of received bytes
 * @return s_OK if successful, s_ERROR if failed
 */
int udp_receive(udp_t *udp, char *packet, int *len)
{
	// Local variables
	int n;
	struct timeval tv;

	// Configure timer for select()
    tv.tv_sec = 0;
    tv.tv_usec = 1000;
    fd_set infds;
    FD_ZERO(&infds);
    FD_SET(udp->sd_recv, &infds);

	// Receive
	// while
	if((n = select(udp->sd_recv + 1, &infds, NULL, NULL, &tv)))
	{
		// Receive packet/datagram
        if((*len = recvfrom(	udp->sd_recv,
        						packet,
        						udp->packet_size,
        						0,
        						(struct sockaddr *)&udp->sock_conn,
        						&udp->sock_conn_len)) == s_ERROR)
        {
        	printf("udp_receive: could not receive UDP packet.\n");
        	return s_ERROR;
        }

        // End string
        packet[*len] = '\0';

        // Print received packet content and information
		/*printf("recv from %s:%d - data: %s\n",
			inet_ntoa(udp->sock_conn.sin_addr),
			ntohs(udp->sock_conn.sin_port),
			packet);*/

		return s_OK;
	}

	return s_ERROR;
}
