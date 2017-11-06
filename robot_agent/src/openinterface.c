/**
 * @file	openinterface.c
 * @author  Eriks Zaharans
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * Open Interface library.
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
#include "openinterface.h"
#include "config.h"
#include "def.h"
#include "general.h"

/* -- Defines -- */

/* -- Functions -- */

/**
 * Open OpenInterface connection
 * @param device_path Path to device
 * @return Pointer to openInterface structure
 */
openinterface_t *openinterface_open(char *device_path)
{
	// Openinterface structure
	openinterface_t *ois = (openinterface_t *) malloc(sizeof(openinterface_t));
	// Openinterface sensor structure
	ois->oiss = (openinterface_sensor_t *) malloc(sizeof(openinterface_sensor_t));

	// Open serial port
	ois->sps = serialport_open(device_path);

	// Configure serial port
	serialport_config(ois->sps, 57600);

	// Start Open Interface
	openinterface_start(ois);
	// Get full control over robot
	openinterface_full(ois);

	// Turn on LEDs
	openinterface_LEDs(ois, 1, 1, 255, 255);

	return ois;
}

/**
 * Close OpenInterface connection
 * @param opis Pointer to openInterface structure
 * @return s_OK if successful, s_ERROR if failed
 */
int openinterface_close(openinterface_t *ois)
{
	int res;

	// Stop robot if it still runs
	openinterface_full(ois);
	// Stop Motion
	openinterface_drive(ois, 0, 0x8000);
	// Turn on LEDs (So that user know that robot is still ON)
	openinterface_LEDs(ois, 1, 1, 255, 255);

	// Close serial port
	res = serialport_close(ois->sps);
	// Free memory (OpenInterface Sensor struct)
	free(ois->oiss);
	// Free memory (OpenInterface struct)
	free(ois);

	return res;
}

/**
 * Start OpenInterface connection
 * @param opis Pointer to OpenInterface structure
 * @return s_OK if successful, s_ERROR if failed
 */
int openinterface_start(openinterface_t *ois)
{
	// Start OI
	if(serialport_byte(ois->sps, s_OI_CMD_START) == s_ERROR)
		return s_ERROR;
	else
		return s_OK;
}

/**
 * Enable full control of OpenInterface
 * @param opis Pointer to OpenInterface structure
 * @return s_OK if successful, s_ERROR if failed
 */
int openinterface_full(openinterface_t *ois)
{
	// Enable full control of OI
	if(serialport_byte(ois->sps, s_OI_CMD_MODE_FULL) == s_ERROR)
		return s_ERROR;
	else
		return s_OK;
}

/**
 * Sends Drive Command
 * @param opis Pointer to OpenInterface structure
 * @param velocity Drive velocity
 * @param radius Radius of circle (curve)
 * @return s_OK if successful, s_ERROR if failed
 */
int openinterface_drive(openinterface_t *ois, int velocity, int radius)
{
	int res = 0;
	unsigned char velocity_data[2], radius_data[2];

	// Convert velocity int to bytes
	general_int2bytes(velocity, &velocity_data[0], &velocity_data[1]);
	// Convert radius int to bytes
	general_int2bytes(radius, &radius_data[0], &radius_data[1]);

	// Send command with drive parameters
	res += serialport_byte(ois->sps, s_OI_CMD_DRIVE);
	res += serialport_byte(ois->sps, velocity_data[0]);
	res += serialport_byte(ois->sps, velocity_data[1]);
	res += serialport_byte(ois->sps, radius_data[0]);
	res += serialport_byte(ois->sps, radius_data[1]);

	// Error check
	if(res < 5)
		return s_ERROR;
	else
		return s_OK;
}

/**
 * Sends Direct Drive Command
 * @param opis Pointer to OpenInterface structure
 * @param right Right wheel velocity
 * @param left left wheel velocity
 * @return s_OK if successful, s_ERROR if failed
 */
int openinterface_drive_direct(openinterface_t *ois, int right, int left)
{
	int res = 0;
	unsigned char right_data[2], left_data[2];

	// Convert right int to bytes
	general_int2bytes(right, &right_data[0], &right_data[1]);
	// Convert left int to bytes
	general_int2bytes(left, &left_data[0], &left_data[1]);

	// Send command with drive parameters
	res += serialport_byte(ois->sps, s_OI_CMD_DRIVE_DIRECT);
	res += serialport_byte(ois->sps, right_data[0]);
	res += serialport_byte(ois->sps, right_data[1]);
	res += serialport_byte(ois->sps, left_data[0]);
	res += serialport_byte(ois->sps, left_data[1]);

	// Error check
	if(res < 5)
		return s_ERROR;
	else
		return s_OK;
}

/**
 * Get one sensor request readings
 * @param opis Pointer to OpenInterface structure
 * @param id ID of the sensor packet
 * @param len Number of bytes to be read
 * @param data Ponter to memory where to store data
 * @return s_OK if successful, s_ERROR if failed
 */
int openinterface_sensor_get(openinterface_t *ois, const unsigned char id, const unsigned char len, unsigned char *data)
{
	// Request packet
	unsigned char req[2] = {s_OI_CMD_SENSORS, id};
	// Read packet
	unsigned char *bufptr;     // Current char in buffer
	int  nbytes, tbytes, n;    // Number of bytes read

	// Prepare select()
	struct timeval tv;
	tv.tv_sec = 1;      // Set two second timeout
	tv.tv_usec = 0;
	fd_set infds;
	FD_ZERO(&infds);
	FD_SET(ois->sps->descriptor, &infds);

	// Request sensor update
	serialport_write(ois->sps, req, 2);

	// Read sensor data
	tbytes = 0;
	bufptr = data;

	while((n = select(ois->sps->descriptor + 1, &infds, NULL, NULL, &tv)) > 0)
	{
		nbytes = (int)read(ois->sps->descriptor, bufptr, 1);

		if (tbytes < (len + 1)) {
			tbytes += nbytes;
			bufptr += nbytes;
		}

		if (tbytes == len) {
			break;
		}
	}

	return s_OK;
}


/**
 * Updates all sensor data
 * @param opis Pointer to OpenInterface structure
 * @param oiss Pointer to OpenInterface sensor structure
 * @param sensor_id ID of sensor packet
 * @param size Number of bytes in sensor packet
 * @return s_OK if successful, s_ERROR if failed
 */
int openinterface_sensors_update(openinterface_t *ois, const unsigned int sensor_id, const unsigned int size)
{
	// Array for data acquisition
	unsigned char data[52];

	// Request all sensor data
	openinterface_sensor_get(ois, sensor_id, size, data);


	// Update sensor measurement values
	// !!! FUNCTION IS NOT COMPLETE. UPDATES ONLY FEW SENSORS
	switch(sensor_id)
	{
	// Sensor Packet 0
	case s_OI_SENSOR_PACKET_0 :
		ois->oiss->wheeldrop_bump			= data[0];
		ois->oiss->wall						= data[1];
		ois->oiss->distance					= general_bytes2int(data[12], data[13]);
		ois->oiss->angle					= general_bytes2int(data[14], data[15]);
		break;
	// Sensor Packet 1
	case s_OI_SENSOR_PACKET_1 :
		ois->oiss->wheeldrop_bump			= data[0];
		ois->oiss->wall						= data[1];
		break;
	// Sensor Packet 2
	case s_OI_SENSOR_PACKET_2 :
		ois->oiss->distance					= general_bytes2int(data[2], data[3]);
		ois->oiss->angle					= general_bytes2int(data[4], data[5]);
		break;
	// Sensor Packet 3
	case s_OI_SENSOR_PACKET_3 :
		break;
	// Sensor Packet 4
	case s_OI_SENSOR_PACKET_4 :
		ois->oiss->wall_signal				= general_bytes2uint(data[0], data[1]);
		ois->oiss->cliff_left_signal		= general_bytes2uint(data[2], data[3]);
		ois->oiss->cliff_front_left_signal	= general_bytes2uint(data[4], data[5]);
		ois->oiss->cliff_front_right_signal	= general_bytes2uint(data[6], data[7]);
		ois->oiss->cliff_right_signal		= general_bytes2uint(data[8], data[9]);
		break;
	// Sensor Packet 5
	case s_OI_SENSOR_PACKET_5 :
		break;
	// Sensor Packet 6
	case s_OI_SENSOR_PACKET_6 :
		ois->oiss->wheeldrop_bump			= data[0];
		ois->oiss->wall						= data[1];
		ois->oiss->distance					= general_bytes2int(data[12], data[13]);
		ois->oiss->angle					= general_bytes2int(data[14], data[15]);
		ois->oiss->wall_signal				= general_bytes2uint(data[26], data[27]);
		ois->oiss->cliff_left_signal		= general_bytes2uint(data[28], data[29]);
		ois->oiss->cliff_front_left_signal	= general_bytes2uint(data[30], data[31]);
		ois->oiss->cliff_front_right_signal	= general_bytes2uint(data[32], data[33]);
		ois->oiss->cliff_right_signal		= general_bytes2uint(data[34], data[35]);
		break;
	// Other
	default :
		return s_ERROR;
		break;
	}

	return s_OK;
}

/**
 * Get travelled distance
 * @param opis Pointer to OpenInterface structure
 * @return travelled distance (mm)
 */
int openinterface_distance_get(openinterface_t *ois)
{
	unsigned char data[2];
	int distance;

	// Request travelled distance measurement
	openinterface_sensor_get(ois, 19, 2, data);

	//Convert to int (two's complement)
	distance = general_bytes2int(data[0], data[1]);

	return distance;
}

/**
 * Get change of angle
 * @param opis Pointer to OpenInterface structure
 * @return change of angle (degree)
 */
int openinterface_angle_get(openinterface_t *ois)
{
	unsigned char data[2];
	int angle;

	// Request angle change measurement
	openinterface_sensor_get(ois, 20, 2, data);

	//Convert to int (two's complement)
	angle = general_bytes2int(data[0], data[1]);

	return angle;
}

/**
 * Control LEDs
 * @param opis Pointer to OpenInterface structure
 * @param play Light up PLAY LED (0 - off, 1 - on)
 * @param advance Light up ADVANCE LED (0 - off, 1 - on)
 * @param power_color Color of POWER LED (0 - green, 255 - red, intermediate values - intermediate colors)
 * @param power_intensity Intensity of POWER LED (0-255)
 * @return s_OK if successful, s_ERROR if failed
 */
int openinterface_LEDs(	openinterface_t *ois,
						int play,
						int advance,
						unsigned char power_color,
						unsigned char power_intensity)
{
	int res = 0;
	unsigned char leds = 0;

	if(play > 0)
		leds += 2;
	if(advance > 0)
		leds += 8;

	// Send command with drive parameters
	res += serialport_byte(ois->sps, s_OI_CMD_LEDS);
	res += serialport_byte(ois->sps, leds);
	res += serialport_byte(ois->sps, power_color);
	res += serialport_byte(ois->sps, power_intensity);

	// Error check
	if(res < 4)
		return s_ERROR;
	else
		return s_OK;
}
