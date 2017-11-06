/**
 * @file	debug.c
 * @author  Eriks Zaharans
 * @date    4 Nov 2013
 *
 * @section DESCRIPTION
 *
 * Debug function library.
 */

/* -- Includes -- */
/* system libraries */
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
/* project libraries */
#include "debug.h"
#include "config.h"

/* -- Defines -- */

/* -- Functions -- */


void debug_printf(const char *format, ...)
{
#ifdef s_CONFIG_DEBUG_ENABLE

	va_list arg;
	int done;

	va_start(arg, format);
	done = printf(format, arg);
	va_end(arg);

	return done;

#endif
}
