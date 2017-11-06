/**
 * @file	pf.h
 * @author  Eriks Zaharans
 * @date	1 Jul 2013
 *
 * @section DESCRIPTION
 *
 * Particle filter library header file.
 */


#ifndef __PF_H
#define __PF_H

/* -- Includes -- */
// Project libraries
#include "robot.h"

/* -- Types -- */

/**
 * @brief Particle filter structure
 */
typedef struct s_PF_STRUCT
{
	robot_t *particles; // Array of particles (robots)

	int num; // Number of particles
    
} pf_t;

/* -- Constants -- */

/* -- Function Prototypes -- */

pf_t *pf_init(	int num, 
				enviroment_t *envs, 
				int move_noise, 
				float turn_noise, 
				int sense_tag_noise, 
				int sense_wall_noise,
				int radius); // Initialize particle filter
void pf_destroy(pf_t *pfs); // Deinitialize particle filter
void pf_drive(pf_t *pfs, int distance, int angle, int uncertain); // Move particles (drive particles) - Motion update
void pf_weight_tag(pf_t *pfs, enviroment_t *envs, int tag_num); // Evaluate particles depending on RFID tag
void pf_weight_wall(pf_t *pfs, enviroment_t *envs); // Evaluate particles depending on how close to wall particle is
void pf_resample(pf_t *pfs); // Resample particles
void pf_estimate(pf_t *pfs, robot_t *robot); // Estimate robot pose according to particles
void pf_random(pf_t *pfs, enviroment_t *envs, int tag_num); // Generate random particles near read RFID tag
int pf_accuracy(pf_t *pfs, enviroment_t *envs); // Evaluate particle filter accuracy


 #endif /* __PF_H */