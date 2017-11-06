/**
 * @file	config.h
 * @author  Eriks Zaharans
 * @date	1 Jul 2013
 *
 * @section DESCRIPTION
 *
 * Configuration file.
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/* -- Includes -- */
/* system libraries */

/* project libraries */


/* -- Types -- */


/**
 * @brief Configuration structure
 */
typedef struct s_CONFIG_STRUCT
{
	// serialport
	char serialport_rfid_port_path[256];
	char serialport_openinterface_port_path[256];

	// enviroment
	char enviroment_room_def_path[256];
	char enviroment_tags_def_path[256];

	// particle filter
	int pf_particles_num; // Number of particles
	int pf_noise_move; // Error created during forward or backward motion (mm)
	int pf_noise_turn; // Error created during turning (degrees)
	int pf_noise_tag; // Error of tag read (mm)
	int pf_noise_wall; // Error of wall read (mm)

	// robot
	int robot_id;
	int robot_team;
	int robot_radius;
	int robot_init_x;
	int robot_init_y;
	int robot_init_angle;
	int robot_speed;

	// pheromones
	int pheromone_width;
	int pheromone_lifetime;
	int pheromone_pheromone_radius;
	int pheromone_eval_radius;
	int pheromone_eval_dist;
	int pheromone_sector_max_size;

	// UDP
	int udp_port;
	int udp_packet_size;
	char udp_broadcast_ip[16];

	// scenario
	int scenario_victims_max;
	int scenario_victims_num;

	// network
	int network_bitrate; // Network bit-rate
	int network_tdma_slot_num; // Number of slots in TDMA period
	int network_tdma_period; // TDMA period length in milliseconds


} config_t;



/* -- Defines -- */

/* RFID */
#define s_CONFIG_RFID_SENSE_RADIUS			50
#define s_CONFIG_RFID_EMPTY_TAG 			"0000000000\0"
/* PHEROMONE */
#define s_CONFIG_PHEROMONE_SECTOR_MAX_SIZE	200
/* VICTIM */
#define s_CONFIG_VICTIM_MAX_COUNT			100
/* STREAM */
#define s_CONFIG_STREAM_SIZE				20
#define s_CONFIG_STREAM_RATE				20 // How many packets in a second
/* GO AHEAD */
#define s_CONFIG_GO_AHEAD_TIME				2000 // Time in ms how long "go ahead" msg is valid
/* ALLOWED OPENINTERFACE REQUEST PERIOD */
#define s_CONFIG_OI_REQUEST_PERIOD			300 // Allowed period between requests
/* ACCURACY LIMIT */
#define s_CONFIG_ACCURACY_LIMIT				30


/* PARTICLE FILTER */
#define s_CONFIG_PF_DISTANCE_UNCERTANITY	200
#define s_CONFIG_PF_ANGLE_UNCERTANITY		10

/* DEBUG */
//#define s_CONFIG_OUTPUT_ENABLE			1
//#define s_CONFIG_TEST_ENABLE				1
//#define s_CONFIG_DEBUG_ENABLE				1



 /* DEFAULT CONFIGURATION */

// -- Serial port --
#define S_CONFIG_DEFAULT_SERIALPORT_RFID_PORT_PATH				"/dev/ttyUSB0"
#define s_CONFIG_DEFAULT_SERIALPORT_OPENINTERFACE_PORT_PATH		"/dev/ttyUSB1"

// -- Environment --
#define s_CONFIG_DEFAULT_ENVIRONMENT_ROOM_DEF_PATH				"./res/large_room.dat"
#define s_CONFIG_DEFAULT_ENVIRONMENT_TAGS_DEF_PATH				"./res/large_tags.dat"

// -- Particle filter --
#define s_CONFIG_DEFAULT_PF_PARTICLES_NUM						1000
#define s_CONFIG_DEFAULT_PF_NOISE_MOVE							20
#define s_CONFIG_DEFAULT_PF_NOISE_TURN							6
#define s_CONFIG_DEFAULT_PF_NOISE_TAG							80
#define s_CONFIG_DEFAULT_PF_NOISE_WALL							1

// -- Robot --
#define s_CONFIG_DEFAULT_ROBOT_ID								1
#define s_CONFIG_DEFAULT_ROBOT_TEAM								1
#define s_CONFIG_DEFAULT_ROBOT_RADIUS							160
#define s_CONFIG_DEFAULT_ROBOT_INIT_X							160
#define s_CONFIG_DEFAULT_ROBOT_INIT_Y							160
#define s_CONFIG_DEFAULT_ROBOT_INIT_ANGLE						0
#define s_CONFIG_DEFAULT_ROBOT_SPEED							300

// -- Pheromone --
#define s_CONFIG_DEFAULT_PHEROMONE_WIDTH						100
#define s_CONFIG_DEFAULT_PHEROMONE_LIFETIME						1
#define s_CONFIG_DEFAULT_PHEROMONE_PHEROMONE_RADIUS				100
#define s_CONFIG_DEFAULT_PHEROMONE_EVAL_RADIUS					300
#define s_CONFIG_DEFAULT_PHEROMONE_EVAL_DIST					400
#define s_CONFIG_DEFAULT_PHEROMONE_SECTOR_MAX_SIZE				100

// -- UDP --
#define s_CONFIG_DEFAULT_UDP_BROADCAST_IP						"255.255.255.255"
#define s_CONFIG_DEFAULT_UDP_PORT								454545
#define s_CONFIG_DEFAULT_UDP_PACKET_SIZE						512

// -- Network --
#define s_CONFIG_DEFAULT_NETWORK_BITRATE						153600
#define s_CONFIG_DEFAULT_NETWORK_TDMA_SLOT_NUM					8
#define s_CONFIG_DEFAULT_NETWORK_TDMA_PERIOD					1000

/* -- Shared Variables -- */
extern config_t g_config;

/* -- Function Prototypes -- */
void config_load(void);

#endif /* __CONFIG_H */
