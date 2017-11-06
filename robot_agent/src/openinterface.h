/**
 * @file	openinterface.h
 * @author  Eriks Zaharans
 * @date    1 Jul 2013
 *
 * @section DESCRIPTION
 *
 * Open Interface library header file.
 */


#ifndef __OPENINTERFACE_H
#define __OPENINTERFACE_H

/* -- Includes -- */
#include "serialport.h"

/* -- Types -- */

/**
 * @brief Open Interface Sensor structure
 */
typedef struct s_OPENINTERFACE_SENSOR_STRUCT
{
	// ID: 7
	unsigned char wheeldrop_bump;
	// ID: 8
	unsigned char wall;
	// ID: 9
	unsigned char cliff_left;
	// ID: 10
	unsigned char cliff_front_left;
	// ID: 11
	unsigned char cliff_front_right;
	// ID: 12
	unsigned char cliff_right;
	// ID: 13
	unsigned char virtual_wall;
	// ID: 14
	unsigned char overcurrent;
	// ID: 15 UNUSED
	// ID: 16 UNUSED
	// ID: 17
	unsigned char infrared;
	// ID: 18
	unsigned char buttons;
	// ID: 19
	int distance;
	// ID: 20
	int angle;
	// ID: 21
	unsigned char charging_state;
	// ID: 22
	unsigned int voltage;
	// ID: 23
	int current;
	// ID: 24
	int battery_temp;
	// ID: 25
	unsigned int battery_charge;
	// ID: 26
	unsigned int battery_capacity;
	// ID: 27
	unsigned int wall_signal;
	// ID: 28
	unsigned int cliff_left_signal;
	// ID: 29
	unsigned int cliff_front_left_signal;
	// ID: 30
	unsigned int cliff_front_right_signal;
	// ID: 31
	unsigned int cliff_right_signal;
	// ID: 32
	unsigned char cargo_bay_digital_inputs;
	// ID: 33
	unsigned int cargo_bay_analog_signal;
	// ID: 34
	unsigned char charging_source_available;
	// ID: 35
	unsigned char io_mode;
	// ID: 36
	unsigned char song_number;
	// ID: 37
	unsigned char song_playing;
	// ID: 38
	unsigned char stream_packets_number;
	// ID: 39
	int requested_velocity;
	// ID: 40
	int requested_radius;
	// ID: 41
	int requested_velocity_right;
	// ID: 42
	int requested_velocity_left;
	
} openinterface_sensor_t;

/**
 * @brief Open Interface structure
 */
typedef struct s_OPENINTERFACE_STRUCT
{
	serialport_t *sps; // Pointer to serial port structure
	openinterface_sensor_t *oiss; // Pointer to Open Interface sensor structure

} openinterface_t;

/* -- Constants -- */

// Commands
#define s_OI_CMD_START					0x80
#define s_OI_CMD_MODE_SAFE				0x83
#define s_OI_CMD_MODE_FULL				0x84
#define s_OI_CMD_MODE_PASSIVE			0x80
#define s_OI_CMD_DRIVE					0x89
#define s_OI_CMD_DRIVE_DIRECT			0x91
#define s_OI_CMD_LEDS					0x8B
#define s_OI_CMD_DIGITAL_OUTPUTS		0x93
#define s_OI_CMD_PWM_LSD				0x90
#define s_OI_CMD_LSD					0x8A
#define s_OI_CMD_SEND_IR				0x97
#define s_OI_CMD_SONG					0x8C
#define s_OI_CMD_PLAY_SONG				0x8D
#define s_OI_CMD_SENSORS				0x8E
#define s_OI_CMD_QUERY_LIST				0x95
#define s_OI_CMD_STREAM					0x94
#define s_OI_CMD_PAUSE_RESUME_STREAM	0x96
#define s_OI_CMD_SCRIPT					0x98
#define s_OI_CMD_PLAY_SCRIPT			0x99
#define s_OI_CMD_SHOW_SCRIPT			0x9A
#define s_OI_CMD_WAIT_TIME				0x9B
#define s_OI_CMD_WAIT_DIST				0x9C
#define s_OI_CMD_WAIT_ANGLE				0x9D
#define s_OI_CMD_WAIT_EVENT				0x9E

// Sensor packet ID
#define s_OI_SENSOR_PACKET_0			0 // Fisrt half of the sensors
#define s_OI_SENSOR_PACKET_1			1 // 1st block of sensors
#define s_OI_SENSOR_PACKET_2			2 // 2nd block of sensors
#define s_OI_SENSOR_PACKET_3			3 // 3rd block of sensors
#define s_OI_SENSOR_PACKET_4			4 // 4th block of sensors
#define s_OI_SENSOR_PACKET_5			5 // 5th block of sensors
#define s_OI_SENSOR_PACKET_6			6 // All sensors

// Sensor packet size (See iRobot Create OpenInterface PDF)
#define s_OI_SENSOR_PACKET_0_SIZE		26 // Fisrt half of the sensors
#define s_OI_SENSOR_PACKET_1_SIZE		10 // 1st block of sensors
#define s_OI_SENSOR_PACKET_2_SIZE		6 // 2nd block of sensors
#define s_OI_SENSOR_PACKET_3_SIZE		10 // 3rd block of sensors
#define s_OI_SENSOR_PACKET_4_SIZE		14 // 4th block of sensors
#define s_OI_SENSOR_PACKET_5_SIZE		12 // 5th block of sensors
#define s_OI_SENSOR_PACKET_6_SIZE		52 // All sensors

/* -- Function Prototypes -- */
// Communication
openinterface_t *openinterface_open(char *device_path); // Open OpenInterface connection
int openinterface_close(openinterface_t *ois); // Close OpenInteface connection

// Comands
int openinterface_start(openinterface_t *ois); // Start OI
int openinterface_full(openinterface_t *ois); // Enable full control of OI

//??
int openinterface_drive(openinterface_t *ois, int velocity, int radius); // Sends drive command
int openinterface_drive_direct(openinterface_t *ois, int right, int left); // Sends Direct Drive Command
int openinterface_sensor_get(openinterface_t *ois, const unsigned char id, const unsigned char len, unsigned char *data); // Get one sensor request readings
int openinterface_sensors_update(openinterface_t *ois, const unsigned int sensor_id, const unsigned int size); // Update one sensor packet
int openinterface_distance_get(openinterface_t *ois); // Get travelled distance
int openinterface_angle_get(openinterface_t *ois); // Get change of angle

int openinterface_LEDs(	openinterface_t *ois, 
						int play, 
						int advance, 
						unsigned char power_color,
						unsigned char power_intensity); // Control LEDs

#endif /* __OPENINTERFACE_H */
