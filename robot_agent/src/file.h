/**
 * @file	file.h
 * @author  Eriks Zaharans
 * @date	1 Jul 2013
 *
 * @section DESCRIPTION
 *
 * File handling library header file.
 */

#ifndef __FILE_H
#define __FILE_H

/* -- Includes -- */

/* -- Enumurations -- */

/* -- Types -- */

/**
 * @brief File structure
 */
typedef struct s_FILE_STRUCT
{
    FILE *fd;			// file descriptor
    char name[256];		//name

} file_t;

/* -- Constants -- */
#define s_FILE_MODE_READ         "r"     // open for reading
#define s_FILE_MODE_WRITE        "w"     // open for writing (file need not exist)
#define s_FILE_MODE_APPEND       "a"     // open for appending (file need not exist)
#define s_FILE_MODE_RW_START     "r+"    // open for reading and writing, start at beginning
#define s_FILE_MODE_RW_OVER      "w+"    // open for reading and writing (overwrite file)
#define s_FILE_MODE_RW_APPEND    "a+"    // open for reading and writing (append if file exists)

/* -- Function Prototypes -- */
file_t *file_open(const char *name, const char *mode); // Open file
file_t *file_open_time(const char *name, const char *ext, const char *mode); // Open file - name includes datetime
int file_close(file_t *fs); // Close file
int file_write(file_t *fs, char *data, int len); // Write to file
int file_write_direct(file_t *fs, char *data, int len); // Write and flush to file
int file_flush(file_t *fs); // Flush file
int file_readln(file_t *fs, char *buffer, size_t buflen); // Read line form file

#endif /* __FILE_H */