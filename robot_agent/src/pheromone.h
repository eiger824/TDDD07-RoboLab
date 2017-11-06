/**
 * @file	pheromone.h
 * @author  Eriks Zaharans
 * @date    18 Jul 2013
 *
 * @section DESCRIPTION
 *
 * Pheromone library header file.
 */


#ifndef __PHEROMONE_H
#define __PHEROMONE_H

/* -- Includes -- */
/* project libraries */
#include "enviroment.h"
#include "robot.h"
#include "config.h"


/* -- Types -- */

/**
 * @brief Pheromone structure
 */
typedef struct s_PHEROMONE_STRUCT
{
	int **map; // Pointer to 2D array of pheromone
	int **eval_stencil; // Pointer to 2D array of smell stencil
	int **pheromone_stencil; // Pointer to 2D array of pheromone stencil

	int width; // Width/size of pheromone grid cell

	int x_cells; // Number of grid cells in x direction
	int y_cells; // NUmber of grid cells in y direction

	int eval_cells; // Number of cells in x and y direction in smell stencil (area is square)
	int pheromone_cells; // Number of cells in x and y direction in pheromone stencil (area is square)

	int lifetime; // Lifetime of a pheromone

	int pheromone_radius; // Radius of pheromone (approx. because it will be fitted to the grid)

	int eval_radius; // Radius of evaluation area (approx. because it will be fitted to the grid)
	int eval_dist; // Distance from robot center to evaluation area (approx. because it will be fitted to the grid)

	int sector_max_size;
	int sector_count;
	int sector_size;
	int sector_size_last;

} pheromone_t;

/**
 * @brief Pheromone Map Sector structure
 */
typedef struct s_PHEROMONE_MAP_SECTOR_STRUCT
{
	int num; // Sector number
	int size; // Sector size
	int timestamp; // Sector base timestamp
	unsigned char data[s_CONFIG_PHEROMONE_SECTOR_MAX_SIZE]; // Sector data

} pheromone_map_sector_t;

/* -- Enumerates -- */

/**
* @brief Enum for directions
*/
enum s_PHEROMONE_DIR
{
	s_PH_LEFT = 0,
	s_PH_TOP_LEFT,
	s_PH_TOP,
	s_PH_TOP_RIGHT,
	s_PH_RIGHT,
	s_PH_NONE
};

/* -- Constants -- */

/* -- Function Prototypes -- */

pheromone_t *pheromone_init(enviroment_t *env,
							int width,
							int lifetime,
							int pheromone_radius,
							int eval_radius,
							int eval_dist,
							int sector_max_size); // Initialize pheromones
void pheromone_destroy(pheromone_t *ph); // Deinitialize pheromones
void pheromone_make_stencil(int ***stencil, int size); // Make stencil - generate circle area in 2D array
void pheromone_put(pheromone_t *ph, int x, int y); // Put pheromone in pheromone map
int pheromone_eval(pheromone_t *ph, robot_t *robot); // Evaluate smell in pheromone map and return new heading direction

void pheromone_map_update(pheromone_t *ph, pheromone_map_sector_t *phms); // Update pheromone map
pheromone_map_sector_t **pheromone_map_extract(pheromone_t *ph); // Extract array of pheromone map sectors
void pheromone_map_destroy(pheromone_t *ph, pheromone_map_sector_t **phms); // Destroy (free memory) array of pheromone map sectors

 #endif /* __PHEROMONE_H */
