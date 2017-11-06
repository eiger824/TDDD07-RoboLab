/**
 * @file	serialport.c
 * @author  Eriks Zaharans
 * @date    1 Jul 2013
 *
 * @section DESCRIPTION
 *
 * Serial Port library.
 */

/* -- Includes -- */
/* system libraries */
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <stdlib.h>
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
/* project libraries */
#include "serialport.h"
#include "def.h"

/* -- Defines -- */

/* -- Functions -- */

/**
 * Open serial port
 * @param name Serial port name ("/dev/tty...")
 * @return Pointer to serial port structure
 */
serialport_t *serialport_open(const char *name)
{
	serialport_t *sps = (serialport_t *) malloc(sizeof(serialport_t)); // Serial port structure

	strcpy(sps->name, name); // Set name
	sps->descriptor = open(sps->name, O_RDWR | O_NOCTTY | O_NDELAY);

	if (sps->descriptor == s_ERROR)
	{
		// sCould not open the port.
		printf("serialport_open: Unable to open port - \"%s\".\n", sps->name);
	}
	else
	{
		fcntl(sps->descriptor, F_SETFL, 0);
	}

	return sps;
}

/**
 * Close serial port
 * @param sps Pointer to serial port structure
 * @return s_OK if successful, s_ERROR if failed
 */
int serialport_close(serialport_t *sps)
{
	int res; // Result

	res = close(sps->descriptor);

	if (res == s_ERROR)
	{
		//Could not close the port.
		printf("serialport_close: Unable to close port - \"%s\".\n", sps->name);
	}

	// Free memory
	free(sps);

	return res;
}

/**
 * Configure serial port
 * @param sps Pointer to serial port structure
 * @param baud Baudrate of serial port communication
 * @return s_OK if successful, s_ERROR if failed
 */
int serialport_config(serialport_t *sps, unsigned int baud)
{
	int res; // Result
	struct termios options;
	speed_t baudrate;

	// Get the current options for the serial port
	tcgetattr(sps->descriptor, &options);

	// Update baudrate in structure
	sps->baudrate = baud;

	// Set the baud rate
	switch (sps->baudrate) {
		case 2400 :
			baudrate = B2400;
			break;
		case 4800 :
			baudrate = B4800;
			break;
		case 9600 :
			baudrate = B9600;
			break;
		case 19200 :
			baudrate = B19200;
			break;
		case 38400 :
			baudrate = B38400;
			break;
		case 57600 :
			baudrate = B57600;
			break;
		case 115200 :
			baudrate = B115200;
			break;
		default:
			printf("serialport_config: Invalid baud rate. Baud rate set to 9600.\n");
			baudrate = B9600;
			break;
	}
	cfsetispeed(&options, baudrate);
	cfsetospeed(&options, baudrate);

	//Enable the receiver and set local mode
	options.c_cflag |= (CLOCAL | CREAD);

	// Set 8 data bits / no parity bit / 1 stop bit
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;

	// Local options
	//options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_lflag = 0;

	// Input options
	//options.c_iflag &= ~(ICRNL);
	options.c_iflag = 0;

	// Output options
	//options.c_oflag &= ~OPOST;
	options.c_oflag = 0;

	// Set the new options for the serial port
	res = tcsetattr(sps->descriptor, TCSANOW, &options);

	// Error message
	if(res == s_ERROR)
	{
		printf("serialport_config: Could not apply new configuration.\n");
	}

	return res;
}


/**
 * Write data to serial port
 * @param sps Pointer to serial port structure
 * @param data Data to be written
 * @param bytes Number of bytes to be written
 * @return The number of bytes written, otherwise s_ERROR
 */
int serialport_write(serialport_t *sps, unsigned char *data, unsigned int bytes)
{
	// Write data
	return write(sps->descriptor, data, bytes);
}

/**
 * Write byte to serial port
 * @param sps Pointer to serial port structure
 * @param byte Byte to be written
 * @return The number of bytes written, otherwise s_ERROR
 */
int serialport_byte(serialport_t *sps, unsigned char byte)
{
	// Write one byte
	return write(sps->descriptor, &byte, 1);
}

/**
 * Flush input buffer
 * @param sps Pointer to serial port structure
 * @return number of flushed bytes, otherwise s_ERROR
 */
int serialport_flush_input(serialport_t *sps)
{
	// Loacal variables
	char buffer[1000];

	// Uselessly read input buffer
	return read(sps->descriptor, &buffer, sizeof(buffer));
}

/**
 * Read data from serial port
 * @param sps Pointer to serial port structure
 * @param data Pointer to memory where to store read data
 * @param bytes Number of bytes to be read
 * @return ...
 */
/*int serialport_read(serialport_t *sps, char *data, int bytes)
{
	char *bufptr; // Current char in buffer
	int  nbytes, bytes_left; // Number of bytes read

	bytes_left = bytes;

	// Read characters into buffer until we get a CR or NL
	bufptr = data;
	while ((nbytes = (int)read(sps->descriptor, bufptr, 1)) > 0)
	{
		bufptr += nbytes;
		bytes_left -= nbytes;
		if (bytes_left <= 0)
			break;
	}

	// Terminate the string with nul
	*bufptr = '\0';

	return 0;
}*/


