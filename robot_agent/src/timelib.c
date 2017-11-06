/**
 * @file	timelib.c
 * @author  Eriks Zaharans
 * @date    1 Jul 2013
 *
 * @section DESCRIPTION
 *
 * Time library.
 */

/* -- Includes -- */
/* system libraries */
#include <stdio.h>
#include <sys/time.h>
/* project libraries */
#include "timelib.h"

/* -- Defines -- */

/* -- Functions -- */


/**
 * Set timer value
 * @param timeval structure
 * @return ... error???
 */
int timelib_timer_set(struct timeval *tv)
{
	// Set time to now
	gettimeofday(tv, NULL);

	// Check error!!!

	return 0;
}

/**
 * Get elapsed time in miliseconds
 * @param timeval structure
 * @return elapsed time in miliseconds
 */
double timelib_timer_get(struct timeval tv)
{
	struct timeval tv_now;
	double time_elapsed;

	// Get current time
	gettimeofday(&tv_now, NULL);

	time_elapsed = (tv_now.tv_sec - tv.tv_sec) * 1000.0;        // sec to ms
	time_elapsed += (tv_now.tv_usec - tv.tv_usec) / 1000.0;     // us to ms

	// Check error!!!

	return time_elapsed;
}

/**
 * Reset timer value
 * @param timeval structure
 * @return elapsed time in miliseconds
 */
double timelib_timer_reset(struct timeval *tv)
{
	struct timeval tv_now;
	double time_elapsed;

	// Get current time
	gettimeofday(&tv_now, NULL);

	time_elapsed = (tv_now.tv_sec - tv->tv_sec) * 1000.0;        // sec to ms
	time_elapsed += (tv_now.tv_usec - tv->tv_usec) / 1000.0;     // us to ms

	// Set current time to timeval
	*tv = tv_now;

	// Check error!!!

	return time_elapsed;
}


/**
 * Add time to timer in milliseconds
 * @param timeval Timeval structure
 * @param int Milliseconds to add
 * @return Void
 */
void timelib_timer_add_ms(struct timeval *tv, unsigned int ms)
{
	// Increase time
	tv->tv_usec = tv->tv_usec + (ms * 1000);
	// Normalize tv_usec if required
	if(tv->tv_usec >= 1000000)
	{
		tv->tv_sec++;
		tv->tv_usec -= 1000000;
	}
}


/**
 * Get time difference in milliseconds
 * @param timeval structure (first)
 * @param timeval structure (second)
 * @return time difference in miliseconds
 */
double timelib_timer_diff(struct timeval tv1, struct timeval tv2)
{
	double time_difference;

	time_difference = (tv2.tv_sec - tv1.tv_sec) * 1000.0;        // sec to ms
	time_difference += (tv2.tv_usec - tv1.tv_usec) / 1000.0;     // us to ms

	return time_difference;
}

/**
 * Get UNIX timestamp in miliseconds
 * @return UNIX timestamp in miliseconds
 */
double timelib_unix_timestamp()
{
	struct timeval tv_now;
	double time_now;

	// Get current time
	gettimeofday(&tv_now, NULL);

	time_now = (double)tv_now.tv_sec * 1000; // sec to ms
	time_now += (double)tv_now.tv_usec / 1000; // us to ms

	// Check error!!!

	return time_now;
}

