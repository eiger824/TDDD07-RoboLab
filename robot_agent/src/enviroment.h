/**
 * @file	enviroment.h
 * @author  Eriks Zaharans
 * @date    1 Jul 2013
 *
 * @section DESCRIPTION
 *
 * Enviroment related function library header file.
 */


#ifndef __ENVIROMENT_H
#define __ENVIROMENT_H

/* -- Includes -- */

/* -- Types -- */

/**
 * @brief Room point definition - x, y
 */
typedef union s_ENVIROMENT_ROOM_POINT_UNION
{
	struct
	{
		int x; // X
		int y; // Y
	};

	int point[2]; // Array with x and y coordinates (alternative way to acces x and y)

} enviroment_room_point_t;

/**
 * @brief Tag definition - x, y, id
 */
typedef struct s_ENVIROMENT_TAG_STRUCT
{
	int x; // X 
	int y; // Y
	char id[11]; // ID of tag 10 chars + '\0'
	unsigned char enable;

} enviroment_tag_t;

/**
 * @brief Enviroment Structure - room points, tags, room point number, tag number
 */
typedef struct s_ENVIROMENT_STRUCT
{
	enviroment_room_point_t *room; // Array of room point structures
	enviroment_tag_t *tags; // Array of RFID tag structures
	
	int room_num; // Number of room points
	int tags_num; // Number of tags

	int room_max_width; // Max room width
	int room_max_height; // Max room height

} enviroment_t;

/* -- Constants -- */
#define s_ENVIROMENT_TAG_ZEROS 		-1
#define s_ENVIROMENT_TAG_UNKNOWN	-2
#define s_ENVIROMENT_TAG_DISABLED 	-3

/* -- Function Prototypes -- */
enviroment_t *enviroment_load(char *room_def_path, char *tags_def_path); // Load enviroment definitions
void enviroment_destroy(enviroment_t *env); // Free dynamicly allocated memory
int enviroment_room_load(enviroment_room_point_t **room, const char *filename); // Load room definition
int enviroment_tags_load(enviroment_tag_t **tags, const char *filename); // Load tags
void enviroment_calc_room_dimensions(enviroment_t *env); // Calculate room dimensions
int enviroment_tag_check(enviroment_t *env, char tag_id[11]);  // Check weather RFID tag exists in enviroment

 #endif /* __ENVIROMENT_H */