/**
 * @file	serialport.h
 * @author  Eriks Zaharans
 * @date    1 Jul 2013
 *
 * @section DESCRIPTION
 *
 * Serial Port library header file.
 */

#ifndef __SERIALPORT_H
#define __SERIALPORT_H

/* -- Includes -- */

/* -- Enumurations -- */

/* -- Types -- */

/**
 * @brief Serial Port structure
 */
typedef struct s_SERIALPORT_STRUCT
{
    int descriptor;				//descriptor
    char name[32];				//name
	unsigned int baudrate;		//baudrate
	
} serialport_t;


/* -- Constants -- */


/* -- Function Prototypes -- */
serialport_t *serialport_open(const char *name); // Open serial port
int serialport_close(serialport_t *sps); // Close serial port
int serialport_config(serialport_t *sps, unsigned int baud); // Configure serial port
int serialport_write(serialport_t *sps, unsigned char *data, unsigned int bytes); // Write data to serial port
int serialport_byte(serialport_t *sps, unsigned char byte); // Write one byt to serial port
int serialport_flush_input(serialport_t *sps); // Flush input buffer
//int serialport_read(serialport_t *sps, char *data, int bytes); // Read dara from serial port

#endif /* __SERIALPORT_H */
