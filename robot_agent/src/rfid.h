/**
 * @file	rfid.h
 * @author  Eriks Zaharans
 * @date    1 Jul 2013
 *
 * @section DESCRIPTION
 *
 * RFID library header file.
 */


#ifndef __RFID_H
#define __RFID_H

/* -- Includes -- */
/* project libraries */
#include "serialport.h"

/* -- Types -- */

/**
 * @brief RFID structure
 */
typedef struct s_RFID_STRUCT
{
	serialport_t *sps;			// serial port structure
	char id[11];				// current id read
	char last_id[11];			// last id read

} rfid_t;

/* -- Constants -- */

/* -- Function Prototypes -- */
rfid_t *rfid_open(char *device_path); // Open RFID connection
int rfid_close(rfid_t *rfids); // Close RFID connection
int rfid_read_locked(rfid_t *rfids); // Read RFID tag ID (locks program until tag is read)
int rfid_read(rfid_t *rfids); // Read RFID tag ID


#endif /* __RFID_H */
