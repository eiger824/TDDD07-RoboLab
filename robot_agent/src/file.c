/**
 * @file	file.c
 * @author  Eriks Zaharans
 * @date    1 Jul 2013
 *
 * @section DESCRIPTION
 *
 * File handling library file.
 */

/* -- Includes -- */
/* system libraries */
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <stdlib.h>
#include <time.h>
/* project libraries */
#include "file.h"
#include "def.h"

/* -- Defines -- */

/* -- Functions -- */

/**
 * Open file
 * @param name File file name
 * @param mode File file open mode (r, w, a, r+, w+, a+)
 * @return Pointer to file structure
 */
file_t *file_open(const char *name, const char *mode)
{
	file_t *fs = (file_t *) malloc(sizeof(file_t)); // File structure

	memset(fs->name, 0, sizeof(fs->name));
	memcpy(fs->name, name, strlen(name)); // Set name

	// Open file
	fs->fd = fopen(fs->name, mode);

	// Return file structure
	return fs;
}

/**
 * Open file - name includes datetime (use only to create files)
 * @param name File name
 * @param ext File extension ('txt', 'dat', etc.)
 * @param mode File open mode (r, w, a, r+, w+, a+)
 * @return Pointer to file structure
 */
file_t *file_open_time(const char *name, const char *ext, const char *mode)
{
	file_t *fs = (file_t *) malloc(sizeof(file_t));; // File structure

	// Time variables
	time_t now;
	struct tm *gmt;
	char formatted_gmt [32];

	// Get time
	now = time (NULL);
	gmt = gmtime (&now);

	// Format time
	strftime(formatted_gmt, sizeof(formatted_gmt), "%d%m%y_%H%M%S", gmt);
	// Set name
	sprintf(fs->name,"%s_%s.%s", name, formatted_gmt, ext);

	// Open file
	fs->fd = fopen(fs->name, mode);

	// Return file structure
	return fs;
}

/**
 * Close file
 * @param fs Pointer to file structure
 * @return s_OK if successfully closed, otherwise s_ERROR
 */
int file_close(file_t *fs)
{
	int res;

	// Close File
	res = fclose(fs->fd);
	// Free memory
	free(fs);

	return res;
}

/**
 * Write to file
 * @param fs Pointer to file structure
 * @param data Data to write to file
 * @param len Length of data
 * @return The total number of elements successfully written
 */
int file_write(file_t *fs, char *data, int len)
{
	// Write to file
	return fwrite(data, 1, len, fs->fd);;
}

/**
 * Write and flush to file
 * @param fs Pointer to file structure
 * @param data Data to write to file
 * @param len Length of data
 * @return The total number of elements successfully written
 */
int file_write_direct(file_t *fs, char *data, int len)
{
	int res;

	// Write to file
	res = fwrite(data, 1, len, fs->fd);
	// Flush data
	if(fflush(fs->fd) == s_OK)
		return res;
	else
		return s_ERROR;
}

/**
 * Flush file
 * @param fs Pointer to file structure
 * @return s_OK if successfully flushed, otherwise s_ERROR
 */
int file_flush(file_t *fs)
{
	// Flush data
	return fflush(fs->fd);
}

/**
 * Read line form file
 * @param fs Pointer to file structure
 * @param buffer Buffer where to save read line
 * @param buflen Size of buffer
 * @return Number of char read
 */
int file_readln(file_t *fs, char *buffer, size_t buflen)
{
	char *end = buffer + buflen - 1; // Allow space for null terminator
	char *dst = buffer;
	int c;

	while ((c = getc(fs->fd)) != EOF && c != '\n' && dst < end)
	{
		*dst++ = c;
	}
	*dst = '\0';

	return((c == EOF && dst == buffer) ? EOF : dst - buffer);
}