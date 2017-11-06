/**
 * @file	robot.c
 * @author  Eriks Zaharans
 * @date	1 Jul 2013
 *
 * @section DESCRIPTION
 *
 * Robot model function library.
 */

 /* -- Includes -- */
/* system libraries */
#include <math.h>
#include <string.h>
#include <stdlib.h>
/* project libraries */
#include "robot.h"
#include "general.h"
#include "config.h"
#include "def.h"

 /* -- Defines -- */

/* -- Functions -- */

/**
 * Initialize robot
 * @param x X coordinate of position (mm)
 * @param y Y coordinate of position (mm)
 * @param a Heading direction (Radians)
 * @param move_noise Error created during forward or backward motion (mm)
 * @param turn_noise Error created during turning (Radians)
 * @param sense_tag_noise Error of tag read (mm)
 * @param sense_wall_noise Error of wall read (mm)
 * @param radius Robot radius (mm)
 * @return Pointer to robot structure
 */
robot_t *robot_init(int x, int y, int a,
					int move_noise, 
					float turn_noise, 
					int sense_tag_noise, 
					int sense_wall_noise,
					int radius)
{
	// Allocate memory for robot structure
	robot_t *robot = (robot_t *) malloc(sizeof(robot_t));

	// Set initial pose
	robot->x = x;
	robot->y = y;
	robot->a = a;

	// Set noise
	robot->move_noise = move_noise;
	robot->turn_noise = turn_noise;
	robot->sense_tag_noise = sense_tag_noise;
	robot->sense_wall_noise = sense_wall_noise;

	// Set radius
	robot->radius = radius;

	return robot;
}

/**
 * Deinitialize robot
 * @param robot Pointer to robot structure
 * @return Void
 */
void robot_destroy(robot_t *robot)
{
	// Free memory
	free(robot);
}

/**
 * Set pose of the robot (x, y, heading direction)
 * @param robot Pointer to robot structure
 * @param x X coordinate of position (mm)
 * @param y Y coordinate of position (mm)
 * @param a Heading direction (Radians)
 * @return Void
 */
void robot_set_pose(robot_t *robot, int x, int y, int a)
{
	robot->x = x;
	robot->y = y;
	robot->a = a;
}

/**
 * Set error/noise for motion and sensors
 * @param robot Pointer to robot structure
 * @param move_noise Error created during forward or backward motion (mm)
 * @param turn_noise Error created during turning (Radians)
 * @param sense_tag_noise Error of tag read (mm)
 * @param sense_wall_noise Error of wall read (mm)
 * @return Void
 */
void robot_set_noise(robot_t *robot, int move_noise, float turn_noise, int sense_tag_noise, int sense_wall_noise)
{
	robot->move_noise = move_noise;
	robot->turn_noise = turn_noise;
	robot->sense_tag_noise = sense_tag_noise;
	robot->sense_wall_noise = sense_wall_noise;
}

/**
 * Drive robot
 * @param robot Pointer to robot structure
 * @param distance Distance traveled (mm)
 * @param angle Angle change (Degrees)
 * @return Void
 */
void robot_drive(robot_t *robot, int distance, int angle, int uncertain)
{
	// Update angle
    if(angle != 0)
    {
        robot->a = robot->a - ((angle + general_gaussrand(0, robot->turn_noise)) * M_PI/180);
        robot->a = fmod(robot->a, M_PI * 2);
    }

    // Update position
    if(distance != 0)
    {
        int dist_with_noise = distance + general_gaussrand(0, robot->move_noise);
        robot->x += cos(robot->a) * dist_with_noise;
        robot->y += sin(robot->a) * dist_with_noise;
    }

    // Robot is uncertain about its action, add just large noise
    if(uncertain > 0)
    {
    	robot->a = robot->a - (general_gaussrand(0, s_CONFIG_PF_ANGLE_UNCERTANITY) * M_PI/180);
        robot->a = fmod(robot->a, M_PI * 2);

        robot->x += cos(robot->a) * s_CONFIG_PF_DISTANCE_UNCERTANITY;
        robot->y += sin(robot->a) * s_CONFIG_PF_DISTANCE_UNCERTANITY;
    }
}

/**
 * Calculate distance to read tag
 * @param robot Pointer to robot structure
 * @param rfids Pointer to RFID structure
 * @param envs Pointer to enviroment structure
 * @return Distance to read tag (mm)
 */
float robot_sense_tag(robot_t *robot, rfid_t *rfids, enviroment_t *envs)
{
	int i;
	float dist;

	// Set robot tag as unknown
	robot->tag_num = -1;

	// No Tag is read (just heart beat)
	if(strcmp(rfids->id, s_CONFIG_RFID_EMPTY_TAG) == 0)
	{
		return -1;
	}

	// Search for read tag
	for(i = 0; i < envs->tags_num; i++)
	{
		if(strcmp(envs->tags[i].id, rfids->id) == 0)
		{
			robot->tag_num = i;
			break;
		}
	}

	// Calculate distance to the tag
	dist = sqrt(pow(robot->x - envs->tags[robot->tag_num].x, 2) +
				pow(robot->y - envs->tags[robot->tag_num].y, 2));
	dist += general_gaussrand(0, robot->sense_tag_noise);

	return dist;
}

/**
 * Calculate distance to read tag
 * @param robot Pointer to robot structure
 * @param envs Pointer to enviroment structure
 * @return Distance to closest wall (mm)
 */
float robot_sense_wall(robot_t *robot, enviroment_t *envs)
{
	int i;
	float res = 99999, dist;

	// Calculate distance to closest wall
	for(i = 0; i < envs->room_num; i++)
	{
		dist = sqrt(general_dist2seg(robot->x,
									 robot->y,
									 envs->room[i].point,
									 envs->room[(i + 1) % envs->room_num].point));
		if(res > dist) {
			res = dist;
		}
	}

	return res;
}

/**
 * Evaluate tag
 * @param robot Pointer to robot structure
 * @param envs Pointer to enviroment structure
 * @return Probability of reading the tag
 */
float robot_eval_tag(robot_t *robot, enviroment_t *envs)
{
	float prob = 1;
	float dist;

	dist = sqrt(pow(robot->x - envs->tags[robot->tag_num].x, 2) +
				pow(robot->y - envs->tags[robot->tag_num].y, 2));

	if( robot->x > envs->room_max_width ||
		robot->y > envs->room_max_height ||
		robot->x < 0 ||
		robot->y < 0)
	{
		prob = 0.00001;
		robot->x = rand() % envs->room_max_width;
		robot->y = rand() % envs->room_max_height;
		robot->a = fmod(rand(), M_PI * 2);
	}
	else
	{
		prob = general_gaussian(dist, robot->sense_tag_noise, rand() % s_CONFIG_RFID_SENSE_RADIUS); // !!!
		if(prob == 0)
			prob = 0.00001;
	}

	robot->weight = prob;

	return prob;
}

/**
 * Evaluate wall
 * @param robot Pointer to robot structure
 * @param envs Pointer to enviroment structure
 * @return Probability of hitting the wall
 */
float robot_eval_wall(robot_t *robot, enviroment_t *envs)
{
	float prob = 1;
	float dist;

	dist = robot_sense_wall(robot, envs);

	if( robot->x > envs->room_max_width ||
		robot->y > envs->room_max_height ||
		robot->x < 0 ||
		robot->y < 0)
	{
		prob = 0.00001;
		robot->x = rand() % envs->room_max_width;
		robot->y = rand() % envs->room_max_height;
		robot->a = fmod(rand(), M_PI * 2);
	}
	else
	{
		if(dist > (robot->radius - 10) && dist < (robot->radius + 10)) {
			prob = 1;
		}
		else 
		{
			prob = 0.1;
		}
	}

	robot->weight = prob;

	return prob;
}

