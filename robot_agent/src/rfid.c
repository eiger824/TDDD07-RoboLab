/**
 * @file	rfid.c
 * @author  Eriks Zaharans
 * @date    1 Jul 2013
 *
 * @section DESCRIPTION
 *
 * RFID library.
 */


/* -- Includes -- */
/* system libraries */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <sys/time.h>
#include <sys/select.h>
/* project libraries */
#include "config.h"
#include "def.h"
#include "serialport.h"
#include "rfid.h"

/* -- Defines -- */

/* -- Functions -- */

/**
 * Open RFID Reader connection
 * @param device_path Path to device
 * @return Pointer to RFID structure
 */
rfid_t *rfid_open(char *device_path)
{
	// RFID structure
	rfid_t *rfids = (rfid_t *) malloc(sizeof(rfid_t));

	// Open serial port
	rfids->sps = serialport_open(device_path);
	// Configure serial port
	serialport_config(rfids->sps, 2400);

	return rfids;
}

/**
 * Close RFID Reader connection
 * @param rfids Pointer to RFID structure
 * @return s_OK if successful, s_ERROR if failed
 */
int rfid_close(rfid_t *rfids)
{
	int res;

	// Close serial port
	res = serialport_close(rfids->sps);
	// Free memory
	free(rfids);

	return res;
}

/**
 * Read ID from RFID Reader
 * @param rfids Pointer to RFID structure
 * @return ...
 */
int rfid_read_locked(rfid_t *rfids)
{
	char buffer[32];
	char *bufptr;           // Current char in buffer
	int  nbytes, tbytes;    // Number of bytes read

	// Read characters into buffer until we get a CR or NL
	bufptr = buffer;

	//!!! Shoud add select!!!

	while ((nbytes = (int)read(rfids->sps->descriptor, bufptr, 1)) > 0)
	{
		// Wait for start byte -> "NL"
		if(*bufptr == '\n') {
			//printf("__NL");
			tbytes = 0;
		}
		// If "CR" received then check if 10 bytes collected if yes -> finished
		else if(*bufptr == '\r') {
			//printf("CR");
			*bufptr = '\0';
			if (tbytes == 10) {
				break;
			}
		}
		// Collect bytes
		else {
			//printf("%s : %d\n",buffer,tbytes);
			// Check overflow
			if (tbytes < 11) {
				tbytes += nbytes;
				bufptr += nbytes;
			}
			else
			{
				break;
			}
		}
	}

	//Copy ID to structure
	strncpy(rfids->last_id, buffer, 11);

	//!!! Check errors

	return 0;
}

/**
 * Read ID from RFID Reader
 * @param rfids Pointer to RFID structure
 * @return ...
 */
int rfid_read(rfid_t *rfids)
{
	char buffer[32];
	char *bufptr;           // Current char in buffer
	int  nbytes, tbytes, n;    // Number of bytes read

	// Prepare select()
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 200000;
	fd_set infds;
	FD_ZERO(&infds);
	FD_SET(rfids->sps->descriptor, &infds);

	struct timeval tv2;
	tv2.tv_sec = 0;
	tv2.tv_usec = 10000;
	fd_set infds2;
	FD_ZERO(&infds2);
	FD_SET(rfids->sps->descriptor, &infds2);

	// Read characters into buffer until we get a CR or NL
	bufptr = buffer;

	if((n = select(rfids->sps->descriptor + 1, &infds2, NULL, NULL, &tv2)) > 0)
	{
		//while((n = select(rfids->sps.descriptor + 1, &infds, NULL, NULL, &tv)) > 0)
		while ((nbytes = (int)read(rfids->sps->descriptor, bufptr, 1)) > 0)
		{
			//nbytes = (int)read(rfids->sps.descriptor, bufptr, 1);
			
			// Wait for start byte -> "NL"
			if(*bufptr == '\n') {
				tbytes = 0;
			}
			// If "CR" received then check if 10 bytes collected if yes -> finished
			else if(*bufptr == '\r') {
				*bufptr = '\0';
				if (tbytes == 10) {
					strncpy(rfids->id, buffer, 11);
					strncpy(rfids->last_id, buffer, 11);

					// Flush input buffer
					//printf("FLUSHED: %d\n", serialport_flush_input(rfids->sps));

					return 0;
				}
			}
			// Collect bytes
			else {
				//printf("%s : %d\n",buffer,tbytes);
				// Check overflow
				if (tbytes < 11) {
					tbytes += nbytes;
					bufptr += nbytes;
				}
				else
				{
					break;
				}
			}
		}
	}
	else
	{
		//Copy ID to structure
		strncpy(rfids->id, s_CONFIG_RFID_EMPTY_TAG, 11);
	}

	//!!! Check errors

	return 0;
}

