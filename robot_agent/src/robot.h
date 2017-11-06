/**
 * @file	robot.h
 * @author  Eriks Zaharans
 * @date	1 Jul 2013
 *
 * @section DESCRIPTION
 *
 * Robot model function library header file.
 */


#ifndef __ROBOT_H
#define __ROBOT_H

/* -- Includes -- */
/* project libraries */
#include "rfid.h"
#include "enviroment.h"

/* -- Types -- */

/**
 * @brief Robot structure
 */
typedef struct s_ROBOT_STRUCT
{
    int x; // X coordinate
	int y; // Y coordinate
	float a; // Heading direction

	float weight; // Weight

	int move_noise;	// Move noise
	float turn_noise; // Turn noise
	int sense_tag_noise; // Tag sense noise
	int sense_wall_noise; // Wall sense noise

	int radius; // Robot radius in mm

	int tag_num;

} robot_t;

/* -- Constants -- */

/* -- Function Prototypes -- */
robot_t *robot_init(int x, int y, int a,
					int move_noise, 
					float turn_noise, 
					int sense_tag_noise, 
					int sense_wall_noise,
					int radius); // Initialize robot
void robot_destroy(robot_t *robot); // Deinitialize robot
void robot_set_pose(robot_t *robot, int x, int y, int a); // Set pose of the robot (x, y, heading direction)
void robot_set_noise(robot_t *robot, int move_noise,
									 float turn_noise,
									 int sense_tag_noise,
									 int sense_wall_noise); // Set error/noise for motion and sensors
void robot_drive(robot_t *robot, int distance, int angle, int uncertain); // Drive robot
float robot_sense_tag(robot_t *robot, rfid_t *rfids, enviroment_t *envs); // Calculate distance to read tag
float robot_sense_wall(robot_t *robot, enviroment_t *envs); // Calculate distance to read tag
float robot_eval_tag(robot_t *robot, enviroment_t *envs); // Evaluate tag
float robot_eval_wall(robot_t *robot, enviroment_t *envs); // Evaluate wall




 #endif /* __ROBOT_H */
