/**
 * @file	enviroment.c
 * @author  Eriks Zaharans
 * @date    1 Jul 2013
 *
 * @section DESCRIPTION
 *
 * Enviroment related function library.
 */

 /* -- Includes -- */
/* system libraries */
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <stdlib.h>
/* project libraries */
#include "enviroment.h"
#include "file.h"
#include "config.h"

/* -- Defines -- */

/* -- Functions -- */

/**
 * Load enviroment definitions
 * @param room_def_path Path to room definition
 * @param tags_def_path Path to tags definition
 * @return Pointer to enviroment structure
 */
enviroment_t *enviroment_load(char *room_def_path, char *tags_def_path)
{
	// Init Enviroment
	enviroment_t *env = (enviroment_t *) malloc(sizeof(enviroment_t));
	// Load room points
	env->room_num = enviroment_room_load(&env->room, room_def_path);
	// Load tags
	env->tags_num = enviroment_tags_load(&env->tags, tags_def_path);
	// Calculate dimensions
	enviroment_calc_room_dimensions(env);

	return env;
}

/**
 * Free dynamicly allocated memory
 * @param env Enviroment structure
 * @return Void
 */
void enviroment_destroy(enviroment_t *env)
{
	// Room definition
	free(env->room);
	// Tag definition
	free(env->tags);
	// Enviroment definition
	free(env);
}

/**
 * Load room definition
 * @param room Pointer to pointer of room definition array
 * @param filename Path to the file containing room definition
 * @return Number of points
 */
int enviroment_room_load(enviroment_room_point_t **room, const char *filename)
{
	char line[128];
	char *end_ptr, *pch;
	int chars_read = 0, i = 0, num = 0;
	file_t *fs;

	// One file
	fs = file_open(filename, s_FILE_MODE_READ);

	// Read till End Of File
	while(chars_read != EOF)
	{
		// Read line
		chars_read = file_readln(fs, line, sizeof(line));
		// If line is empty or commented (first char is hash ('#') sign) then continue to next line
		if(chars_read == 0 || chars_read == EOF || line[0] == '#')
		{
			continue;
		}
		else
		{
			// First number in file specifies number of points
			if(num == 0)
			{
				// Save number of points as a integer
				num = strtol(line, &end_ptr, 10);

				// Allocate memory
				(*room) = malloc(num * sizeof(enviroment_room_point_t));
			}
			else
			{
				// Save each point in struct
				pch = strtok(line, " ,");
				(*room)[i].x = strtol(pch, &end_ptr, 10);
				pch = strtok(NULL, " ,");
				(*room)[i].y = strtol(pch, &end_ptr, 10);

				i++;
			}
		}
	}

	// Close file
	file_close(fs);

	// Return number of points
	return num;
}

/**
 * Load tags
 * @param tags Pointer to pointer of tag array
 * @param filename Path to the file containing RFID tag definition
 * @return Number of tags
 */
int enviroment_tags_load(enviroment_tag_t **tags, const char *filename)
{

	char line[128];
	char *end_ptr, *pch;
	int chars_read = 0, i = 0, num = 0;
	file_t *fs;

	// One file
	fs = file_open(filename, s_FILE_MODE_READ);

	// Read till End Of File
	while(chars_read != EOF)
	{
		// Read line
		chars_read = file_readln(fs, line, sizeof(line));
		// If line is empty or commented (first char is hash ('#') sign) then continue to next line
		if(chars_read == 0 || chars_read == EOF || line[0] == '#')
		{
			continue;
		}
		else
		{
			// First number in file specifies number of points
			if(num == 0)
			{
				// Save number of points as a integer
				num = strtol(line, &end_ptr, 10);

				// Allocate memory
				(*tags) = malloc(num * sizeof(enviroment_tag_t));
			}
			else
			{
				// Save each point in struct
				// X
				pch = strtok(line, " ,");
				(*tags)[i].x = strtol(pch, &end_ptr, 10);
				// Y
				pch = strtok(NULL, " ,");
				(*tags)[i].y = strtol(pch, &end_ptr, 10);
				// ID
				pch = strtok(NULL, " ,");
				memcpy((*tags)[i].id, pch, 10);
				(*tags)[i].id[10] = '\0';
				// Enable
				pch = strtok(line, " ,");
				(*tags)[i].enable = strtol(pch, &end_ptr, 10);

				// Increase counter
				i++;
			}
		}
	}

	// Close file
	file_close(fs);

	// Return number of points
	return num;
}

/**
 * Calculate room dimensions
 * @param env Enviroment structure
 * @return Void
 */
void enviroment_calc_room_dimensions(enviroment_t *env)
{
	unsigned int i, max_width = 0, max_height = 0;

	// Find room size
	for(i = 0; i < env->room_num; i++)
	{
		// Width
		if(max_width < env->room[i].x)
		{
			max_width = env->room[i].x;
		}
		// Height
		if(max_height < env->room[i].y)
		{
			max_height = env->room[i].y;
		}
	}

	// Set room dimensions
	env->room_max_width = max_width;
	env->room_max_height = max_height;
}

/**
 * Check weather RFID tag exists in enviroment
 * @param env Pointer to enviroment structure
 * @return Return tag number in array if found, s_ENVIROMENT_TAG_ZEROS if zeros, s_ENVIROMENT_TAG_UNKNOWN if unknown
 */
int enviroment_tag_check(enviroment_t *env, char tag_id[11])
{
	int i;

	// No Tag is read (just heart beat)
	if(strcmp(tag_id, s_CONFIG_RFID_EMPTY_TAG) == 0)
	{
		return s_ENVIROMENT_TAG_ZEROS;
	}

	// Search for read tag
	for(i = 0; i < env->tags_num; i++)
	{
		if(strcmp(env->tags[i].id, tag_id) == 0)
		{
			// Check if tag is enabled
			if(env->tags[i].enable > 0)
			{
				return i;
			}
			else
			{
				return s_ENVIROMENT_TAG_DISABLED;
			}
		}
	}

	return s_ENVIROMENT_TAG_UNKNOWN;
}