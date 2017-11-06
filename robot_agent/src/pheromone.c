/**
 * @file	pheromone.c
 * @author  Eriks Zaharans
 * @date    18 Jul 2013
 *
 * @section DESCRIPTION
 *
 * Pheromone library.
 */

 /* -- Includes -- */
/* system libraries */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
/* project libraries */
#include "pheromone.h"
#include "enviroment.h"
#include "general.h"
#include "def.h"
#include "debug.h"

/* -- Defines -- */

/* -- Functions -- */

/**
 * Initialize pheromones
 * @param ph Pointer to pheromone structure
 * @param env nviroment structure
 * @param width
 * @param lifetime
 * @param smell_radius
 * @param scan_radius
 * @param sector_max_size
 * @return Pointer to pheromone structure
 */
pheromone_t *pheromone_init(enviroment_t *env, 
							int width, 
							int lifetime,
							int pheromone_radius, 
							int eval_radius, 
							int eval_dist,
							int sector_max_size)
{
	// Local variables
	int i, j;

	// Allocate memory for robot structure
	pheromone_t *ph = (pheromone_t *) malloc(sizeof(pheromone_t));

	// Save parameter values
	ph->width = width;
	ph->lifetime = lifetime;
	ph->pheromone_radius = pheromone_radius;
	ph->eval_radius = eval_radius;
	ph->eval_dist = eval_dist;

	// Calculate how many grid cells fit in the room
	ph->x_cells = ceil((float)env->room_max_width / (float)width);
	ph->y_cells = ceil((float)env->room_max_height / (float)width);

	// Allocate memory
	ph->map = (int **)malloc(ph->x_cells * sizeof(int *));
	for(i = 0; i < ph->x_cells; i++)
	{
		ph->map[i] = (int *)malloc(ph->y_cells * sizeof(int));
		for(j = 0; j < ph->y_cells; j++)
		{
			ph->map[i][j] = 0;
		}
	}

	// How many cells fit in the smell stencil
	ph->eval_cells = ceil((float)(eval_radius * 2) / (float)width);
	// Smell cells should be odd number (circle draw algorithm limitation)
	if(ph->eval_cells % 2 == 0)
		ph->eval_cells++;
	// Make smell stencil
	pheromone_make_stencil(&ph->eval_stencil, ph->eval_cells);

	// How many cells fit in the pheromone stencil
	ph->pheromone_cells = ceil((float)(pheromone_radius * 2) / (float)width);
	// Smell cells should be odd number (circle draw algorithm limitation)
	if(ph->pheromone_cells % 2 == 0)
		ph->pheromone_cells++;
	// Make smell stencil
	pheromone_make_stencil(&ph->pheromone_stencil, ph->pheromone_cells);

	// Calculate sectors
	ph->sector_max_size = sector_max_size;
	ph->sector_size = floor((float)sector_max_size / (float)ph->y_cells);
    ph->sector_count = ceil((float)ph->x_cells / (float)ph->sector_size);
    ph->sector_size_last = ph->x_cells % ph->sector_size;

    debug_printf("map: %d, %d\n",ph->x_cells, ph->y_cells);
    debug_printf("sec: %d, %d, %d, %d\n", ph->y_cells, ph->sector_size, ph->sector_count, ph->sector_size_last);

	return ph;
}

/**
 * Deinitialize pheromones
 * @param ph Pointer to pheromone structure
 * @return Void
 */
void pheromone_destroy(pheromone_t *ph)
{
	// Local variables
	int i;

	// Free pheromone map
	for(i = 0; i < ph->x_cells; i++)
	{
		free(ph->map[i]);
	}
	free(ph->map);

	// Free smell stencil
	for(i = 0; i < ph->eval_cells; i++)
	{
		free(ph->eval_stencil[i]);
	}
	free(ph->eval_stencil);

	// Free pheromone stencil
	for(i = 0; i < ph->pheromone_cells; i++)
	{
		free(ph->pheromone_stencil[i]);
	}
	free(ph->pheromone_stencil);

	// Free pheromone structure
	free(ph);

}

/**
 * Make stencil - generate circle area in 2D array
 * @param stencil Pointer to 2D array
 * @param size Size of the stencil
 * @return Void
 */
void pheromone_make_stencil(int ***stencil, int size)
{
	// Local variables
	int i, j;

	// Allocate memory for smell area
	(*stencil) = (int **)malloc(size * sizeof(int *));
	for(i = 0; i < size; ++i)
	{
		(*stencil)[i] = (int *)malloc(size * sizeof(int));
		for(j = 0; j < size; j++)
		{
			(*stencil)[i][j] = 0;
		}
	}

	// Draw circle
	general_circle((*stencil), (size - 1)/2, (size - 1)/2, (size - 1)/2);
}

/**
 * Put pheromone in pheromone map
 * @param ph Pointer to pheromone structure
 * @param x X coordinate of pheromone center
 * @param y Y coordinate of pheromone center
 * @param time Time of placment
 * @return Void
 */
void pheromone_put(pheromone_t *ph, int x, int y)
{
	// Local variables
	int i, j;
	int cx, cy;

	// Calculate xy position to fit center (shift stencil so it fits in center)
	cx = (x - ph->pheromone_radius);
	cy = (y - ph->pheromone_radius);
	if(cx >= 0 && cy >= 0)
	{
		cx = cx / ph->width;
		cy = cy / ph->width;
	}
	else
	{
		return;
	}

	// Apply pheromone stencil to pheromone map
	for(i = 0; i < ph->pheromone_cells; i++)
	{
		for(j = 0; j < ph->pheromone_cells; j++)
		{
			// Check if stencil pixel is set
			if(ph->pheromone_stencil[i][j] != 0)
			{
				// Check pheromone map borders
				if((cx + i) >= 0 && (cy + j) >= 0 && (cx + i) < ph->x_cells && (cy + j) < ph->y_cells)
				{
					// Set pheromone with its timestamp
					ph->map[cx + i][cy + j] = time(NULL) / ph->lifetime;
					//printf("pheromone: %d, %d, %d\n", cx + i, cy + j, ph->map[cx + i][cy + j]);
				}
			}
		}
	}
}

/**
 * Evaluate smell in pheromone map and return new heading direction
 * @param ph Pointer to pheromone structure
 * @param robot Robot structure
 * @return Direction to move
 */
int pheromone_eval(pheromone_t *ph, robot_t *robot)
{
	int i, j, k;
	int x;
	int y;
	float a[5] = {-M_PI/2, -M_PI/4, 0, M_PI/4, M_PI/2};

	int t;
	int s[5] = {0, 0, 0, 0, 0};
	int s_t = 0;
	float p[5] = {0, 0, 0, 0, 0};
	float p_t = 0, p_r;

	// Test
	float max = 0;
	int max_i = 0;

	// Evaluate smell in 5 areas
	for(k = 0; k < 5; k++)
	{
		// Calculate position of smell area
		x = cos(robot->a + a[k]) * ((ph->pheromone_radius + ph->eval_radius) / ph->width) + (robot->x - ph->eval_radius) / ph->width;
		y = sin(robot->a + a[k]) * ((ph->pheromone_radius + ph->eval_radius) / ph->width) + (robot->y - ph->eval_radius) / ph->width;

		// Applay smell stencil and evaluate smell in area
		for(i = 0; i < ph->eval_cells; i++)
		{
			for(j = 0; j < ph->eval_cells; j++)
			{
				// Check if stencill element is full
				if(ph->eval_stencil[i][j] != 0)
				{
					// Check if in range
					if((x + i) >= 0 && (y + j) >= 0 && (x + i) < ph->x_cells && (y + j) < ph->y_cells)
					{
						t = ((time(NULL) / ph->lifetime) - ph->map[x + i][y + j]);
						if(t > 255) {
							t = 255;
						}
						else if(t < 0) {
							t = 0;
						}
						s[k] += (256 - t) * 100; // !!!
					}
					else
					{
						s[k] += 999999;
					}
				}
			}
		}
		s_t += s[k];
	}

	
	// Calculate probability
	// It is calculated other way around, it should be normalized but it works well without it.
	for(k = 0; k < 5; k++)
	{
		p[k] = ((float)s_t / (float)s[k]);
		p_t += p[k];

		if(p[k] > max)
		{
			max = p[k];
			max_i = k;
		}
		else if(p[k] == max)
		{
			if((rand() % 10) > 5)
			{
				max_i = k;
			}
		}
	}

	debug_printf("%f, %f, %f, %f, %f\n", p[0], p[1], p[2], p[3], p[4]);

	if(p[0] == p[1] && p[0] == p[2]  && p[0] == p[3]  && p[0] == p[4])
	{
		return (rand() % 5);
	}
	else
	{
		return max_i;
	}

	// Randomly choose direction based on probability
	//p_r = fmod(rand(), p_t);

	// Check which direction was choosen and return it
	if(p_r >= 0 && p_r < (p[0])) {
		return s_PH_LEFT;
	}
	else if(p_r >= (p[0]) && p_r < (p[0] + p[1])) {
		return s_PH_TOP_LEFT;
	}
	else if(p_r >= (p[0] + p[1]) && p_r < (p[0] + p[1] + p[2])){
		return s_PH_TOP;
	}
	else if(p_r >= (p[0] + p[1] + p[2]) && p_r < (p[0] + p[1] + p[2] + p[3])) {
		return s_PH_TOP_RIGHT;
	}
	else if(p_r >= (p[0] + p[1] + p[2] + p[3]) && p_r < (p[0] + p[1] + p[2] + p[3] + p[4])) {
		return s_PH_RIGHT;
	}

	// Return Error otherwise
	return s_ERROR;
}

/**
 * Update pheromone map
 * @param ph Pointer to pheromone structure
 * @param phms Pointer to pheromone map sector structure
 * @return Void
 */
void pheromone_map_update(pheromone_t *ph, pheromone_map_sector_t *phms)
{
	// Local variables
	int i,j, y = 0;

	// Output sector data starting at the right column
	for(i = phms->num * ph->sector_size; i < ph->x_cells; i++)
	{
		for(j = 0; j < ph->y_cells; j++)
		{
			// Replace only if data is newer
			if(ph->map[i][j] < (phms->timestamp - phms->data[y]))
			{
				ph->map[i][j] = phms->timestamp - phms->data[y];
			}

			// Increase sector data counter
			y++;
		}

		// Finish if sector is done
		if(y >= (ph->y_cells * ph->sector_size))
		{
			break;
		}
	}

	// If for cycle exits without "break", it was last sector, that was not full
}

/**
 * Extract array of pheromone map sectors
 * @param ph Pointer to pheromone structure
 * @return Pointer to array of pheromone map sector structures
 */
pheromone_map_sector_t **pheromone_map_extract(pheromone_t *ph)
{
	// Local variables
	int i, j, x = 0, y = 0;
	int t, t_curr;
	unsigned char cell;

	// Allocate memory for pheromone map sector structures
	pheromone_map_sector_t **phms = (pheromone_map_sector_t **)malloc(ph->sector_count * sizeof(pheromone_map_sector_t *));
	for(i = 0; i < ph->sector_count; i++)
	{
		phms[i] = (pheromone_map_sector_t *)malloc(sizeof(pheromone_map_sector_t));
	}

	// Get current timestamp
	t_curr = time(NULL) / ph->lifetime;

	// Save data about first sector
	phms[x]->num = x;
	phms[x]->size = ph->y_cells * ph->sector_size;
	phms[x]->timestamp = t_curr;
	memset(phms[x]->data, 0xFF, ph->sector_max_size);

	// Loop through map data and fill in the sectors
	for(i = 0; i < ph->x_cells; i++)
	{
		for(j = 0; j < ph->y_cells; j++)
		{
			// Calculate cell value
			t = t_curr - ph->map[i][j];
			if(t > 255) {
				cell = 255;
			}
			else if(t < 1) {
				cell = 1;
			}
			// THIS IS FOR TEST (SKIP COMMA CODE) !!!
			else if(t == ',') {
				cell = ',' + 1;
			}
			else {
				cell = (unsigned char)t;
			}

			// Save cell value in sector
			phms[x]->data[y] = cell;

			// Increase sector data counter
			y++;
			// Check if sector is full. If yes then ...
			if(y >= (ph->y_cells * ph->sector_size))
			{
				// Increase sector number
				x++;
				// Reset sector data counter
				y = 0;

				// Save data about new sector
				if(x < ph->sector_count)
				{
					phms[x]->num = x;
					phms[x]->size = ph->y_cells * ph->sector_size;
					phms[x]->timestamp = t_curr;
					memset(phms[x]->data, 0xFF, ph->sector_max_size);
				}
			}
		}
	}

	return phms;
}

/**
 * Destroy (free memory) array of pheromone map sectors
 * @param ph Pointer to pheromone structure
 * @param phms Pointer to array of pheromone map sector structures
 * @return Void
 */
void pheromone_map_destroy(pheromone_t *ph, pheromone_map_sector_t **phms)
{
	// Local variables
	int i;

	// Free Pheromone map sectors
	for (i = 0; i < ph->sector_count; ++i)
	{
		free(phms[i]);
	}

	// Free Pheromone map sector array
	free(phms);
}