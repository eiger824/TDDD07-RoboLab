/**
 * @file	pf.c
 * @author  Eriks Zaharans
 * @date	1 Jul 2013
 *
 * @section DESCRIPTION
 *
 * Particle filter library.
 */

 /* -- Includes -- */
/* system libraries */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
/* project libraries */
#include "pf.h"
#include "robot.h"
#include "enviroment.h"
#include "debug.h"

 /* -- Defines -- */

/* -- Functions -- */

/**
 * Initialize particle filter
 * @param num Number of particles
 * @param envs Pointer to enviroment structure
 * @param move_noise Error created during forward or backward motion (mm)
 * @param turn_noise Error created during turning (Radians)
 * @param sense_tag_noise Error of tag read (mm)
 * @param sense_wall_noise Error of wall read (mm)
 * @param radius Particle radius - should be same as robot (mm)
 * @return Pointer to particle filter structure
 */
pf_t *pf_init(	int num,
				enviroment_t *envs,
				int move_noise,
				float turn_noise,
				int sense_tag_noise,
				int sense_wall_noise,
				int radius)
{
	int i;

	// Allocate Particle filter structure
	pf_t *pfs = (pf_t *) malloc(sizeof(pf_t));

	// Set number of particles
	pfs->num = num;

	// Allocate memory
	pfs->particles = malloc(pfs->num  * sizeof(robot_t));

	for(i = 0; i < pfs->num ; i++)
	{
		// Set random initial pose for particles (robots)
		robot_set_pose(&pfs->particles[i],	rand() % envs->room_max_width,
										 	rand() % envs->room_max_height,
										 	fmod(rand(), M_PI * 2));
		// Set noise
		robot_set_noise(&pfs->particles[i],	move_noise,
										  	turn_noise,
										  	sense_tag_noise,
										  	sense_wall_noise);

		// Set initial weight
		pfs->particles[i].radius = radius;

		// Set initial weight
		pfs->particles[i].weight = (float)1 / (float)pfs->num;
	}

	return pfs;
}

/**
 * Deinitialize particle filter
 * @param pfs Pointer to particle filter structure
 * @return Void
 */
void pf_destroy(pf_t *pfs)
{
	// Free memory (Robot struct - particles)
	free(pfs->particles);
	// Free memory (Particle filter struct)
	free(pfs);
}

/**
 * Move particles (drive particles) - Motion update
 * @param pfs Pointer to particle filter structure
 * @param distance Distance traveled (mm)
 * @param angle Angle change (Degrees)
 * @return Void
 */
void pf_drive(pf_t *pfs, int distance, int angle, int uncertain)
{
	int i;

	// Move all particles
	for(i = 0; i < pfs->num; i++)
	{
		robot_drive(&pfs->particles[i], distance, angle, uncertain);
	}
}

/**
 * Evaluate particles depending on RFID tag
 * @param pfs Pointer to particle filter structure
 * @param envs Pointer to enviroment structure
 * @param tag_num RFID tag number
 * @return Void
 */
void pf_weight_tag(pf_t *pfs, enviroment_t *envs, int tag_num)
{
	int i;

	// Evaluate each particle
	for(i = 0; i < pfs->num; i++)
	{
		pfs->particles[i].tag_num = tag_num;
		robot_eval_tag(&pfs->particles[i], envs);
	}
}

/**
 * Evaluate particles depending on how close to wall particle is
 * @param pfs Pointer to particle filter structure
 * @param envs Pointer to enviroment structure
 * @param tag_num RFID tag number
 * @return Void
 */
void pf_weight_wall(pf_t *pfs, enviroment_t *envs)
{
	int i;

	// Evaluate each particle
	for(i = 0; i < pfs->num; i++)
	{
		robot_eval_wall(&pfs->particles[i], envs);
	}
}

/**
 * Resample particles
 * @param pfs Pointer to particle filter structure
 * @return Void
 */
void pf_resample(pf_t *pfs)
{
	// Local variables
	robot_t **resampled;
	int i, index = (rand() % pfs->num);
	float beta = 0;
	float mw = 0, w_sum = 0;

	// Allocate memory
	resampled = (robot_t **)malloc(pfs->num * sizeof(robot_t *));
	for(i = 0; i < pfs->num; i++)
	{
		resampled[i] = (robot_t *)malloc(sizeof(robot_t));
	}

	// Find max weight
	for(i = 0; i < pfs->num; i++)
	{
		if(mw < pfs->particles[i].weight)
			mw = pfs->particles[i].weight;
	}

	// TEST !!!
	debug_printf("MAX WEIGHT: %f\n", mw);

	// Resample
	for(i = 0; i < pfs->num; i++)
	{
		beta += (float)rand() / ((float)RAND_MAX / (mw * 2));
		while(beta > pfs->particles[index].weight)
		{
			beta -= pfs->particles[index].weight;
			index = (index + 1) % pfs->num;
		}

		memcpy(resampled[i], &pfs->particles[index], sizeof(robot_t));

		// Calculate sum for normalization (so that sum of all weights is one)
		w_sum += pfs->particles[index].weight;

	}

	// Restore to particles array
	for(i = 0; i < pfs->num; i++)
	{
		memcpy(&pfs->particles[i], resampled[i], sizeof(robot_t));
		pfs->particles[i].weight = resampled[i]->weight / w_sum;
	}

	// Free memory
	for(i = 0; i < pfs->num; i++)
	{
		free(resampled[i]);
	}
	free(resampled);
}

/**
 * Estimate robot pose according to particles
 * @param pfs Pointer to particle filter structure
 * @param robot Pointer to robot structure
 * @return Void
 */
void pf_estimate(pf_t *pfs, robot_t *robot)
{
	int i;
	float ex = 0, ey = 0;
	float vx = 0, vy = 0, va = 0;

	// Loop through all particles
	for(i = 0; i < pfs->num; i++)
	{
		// Calculate position taking into account weights
		ex += (float)pfs->particles[i].x * pfs->particles[i].weight;
		ey += (float)pfs->particles[i].y * pfs->particles[i].weight;

		// Calculate vector for heading direction
		vx += pfs->particles[i].weight * cos(pfs->particles[i].a);
		vy += pfs->particles[i].weight * sin(pfs->particles[i].a);
	}

	// Calculate angle from the vector
	if(vy >= 0 && vx > 0)
		va = atan(vy / vx);
	else if(vy <= 0 && vx > 0)
		va = 2 * M_PI + atan(vy / vx);
	else if(vx != 0)
		va = M_PI + atan(vy / vx);
	else if(vy > 0)
		va = M_PI / 2;
	else if(vy < 0)
		va = (3 * M_PI) / 2;
	else
		va = 0;

	// Save estimated values
	robot->x = (int)ex;
	robot->y = (int)ey;
	robot->a = va;
}


/**
 * Generate random particles near read RFID tag
 * @param pfs Pointer to particle filter structure
 * @param envs Pointer to enviroment structure
 * @param tag_num RFID tag number
 * @return Void
 */
void pf_random(pf_t *pfs, enviroment_t *envs, int tag_num)
{
	int i;
	int num = rand() % 100; // Number of randomly drawn particles
	int particle_id;

	// Draw random particles and place them close to the read RFID tag
	for(i = 0; i < num; i++)
	{
		// Randomly draw id of particle
		particle_id = rand() % pfs->num;

		// Set random position near the read tag
		pfs->particles[particle_id].x = envs->tags[tag_num].x + (300 - (rand() % 600));
		pfs->particles[particle_id].y = envs->tags[tag_num].y + (300 - (rand() % 600));

		// Randomize angle only once a while
		/*if((rand() % 10) > 5)
		{
			pfs->particles[particle_id].a = fmod(rand(), M_PI * 2);
		}*/

		// Give a low probability
		pfs->particles[particle_id].weight = 0.00001;
	}
}

/**
 * Evaluate particle filter accuracy
 * @param pfs Pointer to particle filter structure
 * @param envs Pointer to enviroment structure
 * @return Accuracy in percent
 */
int pf_accuracy(pf_t *pfs, enviroment_t *envs)
{
	int i;
	int xmin = 99999, xmax = 0, ymin = 99999, ymax = 0;
	int width, height, area, field_area;
	float accuracy;
	int percent;

	//
	for(i = 0; i < pfs->num; i++)
	{
		// Look only at good particles
		if(pfs->particles[i].weight > 0.00001)
		{
			// Find MIN
			if(xmin > pfs->particles[i].x)
				xmin = pfs->particles[i].x;

			if(ymin > pfs->particles[i].y)
				ymin = pfs->particles[i].y;

			// Find MAX
			if(xmax < pfs->particles[i].x)
				xmax = pfs->particles[i].x;

			if(ymax < pfs->particles[i].y)
				ymax = pfs->particles[i].y;
		}
	}

	width = xmax - xmin;
	height = ymax - ymin;
	area = width * height;
	field_area = envs->room_max_width * envs->room_max_height;

	accuracy = 1 - ((float)area / (float)field_area);
	if(accuracy < 0)
	{
	    percent = 0;
	}
	else
	{
	    percent = (int)(accuracy * 100);
	}


	debug_printf("accuracy: %d [%f] - %d [%d x %d], %d\n", percent, accuracy, area, width, height, field_area);

	return percent;
}
