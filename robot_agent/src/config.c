/**
 * @file	config.c
 * @author  Eriks Zaharans
 * @date    17 Aug 2013
 *
 * @section DESCRIPTION
 *
 * Configuration handling library file.
 */

/* -- Includes -- */
/* system libraries */

/* project libraries */
#include "lib/iniparser/iniparser.h"
#include "config.h"
#include "def.h"

/* -- Defines -- */

/* -- Global Variables -- */
config_t g_config;

/* -- Functions -- */

void config_load(void)
{
	// Local variables
	dictionary *ini;
	char *s;
	
	ini = iniparser_load("./res/config.ini");
	if (ini == NULL) {
        fprintf(stderr, "cannot parse file");
        return -1 ;
    }
    //iniparser_dump(ini, stderr);
	
    // Load configurations
	
	// -- Serial port --
    s = iniparser_getstring(ini, "serialport:rfid_port_path", S_CONFIG_DEFAULT_SERIALPORT_RFID_PORT_PATH);
    strcpy(g_config.serialport_rfid_port_path, s);
    s = iniparser_getstring(ini, "serialport:openinterface_port_path", s_CONFIG_DEFAULT_SERIALPORT_OPENINTERFACE_PORT_PATH);
   	strcpy(g_config.serialport_openinterface_port_path, s);

	// -- Environment --
	s = iniparser_getstring(ini, "environment:room_def_path", s_CONFIG_DEFAULT_ENVIRONMENT_ROOM_DEF_PATH);
	strcpy(g_config.enviroment_room_def_path, s);
	s = iniparser_getstring(ini, "environment:tags_def_path", s_CONFIG_DEFAULT_ENVIRONMENT_TAGS_DEF_PATH);
	strcpy(g_config.enviroment_tags_def_path, s);

	// -- Particle filter -- 
	g_config.pf_particles_num = iniparser_getint(ini, "pf:particles_num", s_CONFIG_DEFAULT_PF_PARTICLES_NUM);
	g_config.pf_noise_move = iniparser_getint(ini, "pf:noise_move", s_CONFIG_DEFAULT_PF_NOISE_MOVE);
	g_config.pf_noise_turn = iniparser_getint(ini, "pf:noise_turn", s_CONFIG_DEFAULT_PF_NOISE_TURN);
	g_config.pf_noise_tag = iniparser_getint(ini, "pf:noise_tag", s_CONFIG_DEFAULT_PF_NOISE_TAG);
	g_config.pf_noise_wall = iniparser_getint(ini, "pf:noise_wall", s_CONFIG_DEFAULT_PF_NOISE_WALL);

	// -- Robot --
	g_config.robot_id = iniparser_getint(ini, "robot:id", s_CONFIG_DEFAULT_ROBOT_ID);
	g_config.robot_team = iniparser_getint(ini, "robot:team", s_CONFIG_DEFAULT_ROBOT_TEAM);
	g_config.robot_radius = iniparser_getint(ini, "robot:radius", s_CONFIG_DEFAULT_ROBOT_RADIUS);
	g_config.robot_init_x = iniparser_getint(ini, "robot:init_x", s_CONFIG_DEFAULT_ROBOT_INIT_X);
	g_config.robot_init_y = iniparser_getint(ini, "robot:init_y", s_CONFIG_DEFAULT_ROBOT_INIT_Y);
	g_config.robot_init_angle = iniparser_getint(ini, "robot:init_angle", s_CONFIG_DEFAULT_ROBOT_INIT_ANGLE);
	g_config.robot_speed = iniparser_getint(ini, "robot:speed", s_CONFIG_DEFAULT_ROBOT_SPEED);

	// -- Pheromone --
	g_config.pheromone_width = iniparser_getint(ini, "pheromone:width", s_CONFIG_DEFAULT_PHEROMONE_WIDTH);
	g_config.pheromone_lifetime = iniparser_getint(ini, "pheromone:lifetime", s_CONFIG_DEFAULT_PHEROMONE_LIFETIME);
	g_config.pheromone_pheromone_radius = iniparser_getint(ini, "pheromone:pheromone_radius", s_CONFIG_DEFAULT_PHEROMONE_PHEROMONE_RADIUS);
	g_config.pheromone_eval_radius = iniparser_getint(ini, "pheromone:eval_radius", s_CONFIG_DEFAULT_PHEROMONE_EVAL_RADIUS);
	g_config.pheromone_eval_dist = iniparser_getint(ini, "pheromone:eval_dist", s_CONFIG_DEFAULT_PHEROMONE_EVAL_DIST);
	g_config.pheromone_sector_max_size = iniparser_getint(ini, "pheromone:sector_max_size", s_CONFIG_DEFAULT_PHEROMONE_SECTOR_MAX_SIZE);

	// -- UDP --
	s = iniparser_getstring(ini, "udp:broadcast_ip", s_CONFIG_DEFAULT_UDP_BROADCAST_IP);
	strcpy(g_config.udp_broadcast_ip, s);
	g_config.udp_port = iniparser_getint(ini, "udp:port", s_CONFIG_DEFAULT_UDP_PORT);
	g_config.udp_packet_size = iniparser_getint(ini, "udp:packet_size", s_CONFIG_DEFAULT_UDP_PACKET_SIZE);

	// -- Netwokr --
	g_config.network_bitrate = iniparser_getint(ini, "network:bitrate", s_CONFIG_DEFAULT_NETWORK_BITRATE);
	g_config.network_tdma_slot_num = iniparser_getint(ini, "network:tdma_slot_num", s_CONFIG_DEFAULT_NETWORK_TDMA_SLOT_NUM);
	g_config.network_tdma_period = iniparser_getint(ini, "network:tdma_period", s_CONFIG_DEFAULT_NETWORK_TDMA_PERIOD);



	/*// -- Scenario --
	int scenario_victims_max;
	int scenario_victims_num;*/



	// Deinit iniparser
    iniparser_freedict(ini);
}