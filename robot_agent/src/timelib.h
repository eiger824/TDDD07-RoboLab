/**
 * @file	timelib.h
 * @author  Eriks Zaharans
 * @date	1 Jul 2013
 *
 * @section DESCRIPTION
 *
 * Time library header file.
 */

#ifndef __TIMELIB_H
#define __TIMELIB_H

/* -- Includes -- */

/* -- Enumurations -- */

/* -- Types -- */

/* -- Constants -- */

/* -- Function Prototypes -- */
int timelib_timer_set(struct timeval *tv); // Set timer value
double timelib_timer_get(struct timeval tv); // Get elapsed time in miliseconds
double timelib_timer_reset(struct timeval *tv); // Reset timer value
void timelib_timer_add_ms(struct timeval *tv, unsigned int ms); // Add time to timer in milliseconds
double timelib_timer_diff(struct timeval tv1, struct timeval tv2); // Get time difference in milliseconds
double timelib_unix_timestamp(); // Get UNIX timestamp in miliseconds

#endif /* __TIMELIB_H */
