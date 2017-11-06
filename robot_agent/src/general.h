/**
 * @file	general.h
 * @author  Eriks Zaharans
 * @date    1 Jul 2013
 *
 * @section DESCRIPTION
 *
 * General function library header file.
 */


#ifndef __GENERAL_H
#define __GENERAL_H

/* -- Includes -- */

/* -- Types -- */

/* -- Constants -- */

/* -- Function Prototypes -- */

int general_bytes2int(unsigned char byte_high, unsigned char byte_low); // Convert two bytes to int (two's complement)
unsigned int general_bytes2uint(unsigned char byte_high, unsigned char byte_low); // Convert two bytes to unsigned int
void general_int2bytes(int integer, unsigned char *byte_high, unsigned char *byte_low); // Convert int to two bytes (two's complement)

double general_gaussrand(double mu, double sigma); // Random number based on normal (gaussian) distribution
float general_gaussian(float mu, float sigma, int x); // Calculate Gaussian

float general_dist_squared(int x1, int y1, int x2, int y2); // Calculate squared distance between two points
float general_dist2seg(int x, int y, int p1[], int p2[]); // Calculate distance from point to line segment

void general_circle(int **arr, int cx, int cy, int radius); // Draw (fill) filled circle in 2D array
void general_circle_eight_points(int **arr, int cx, int cy, int x, int y); // Draw (fill) eight points - Part of general_circle() function
void general_circle_four_points(int **arr, int cx, int cy, int x, int y); // Draw (fill) four points - Part of general_circle() function
void general_circle_horizontal_line(int **arr, int x, int y, int len); // Draw (fill) horizontal line - Part of general_circle() function

#endif /* __GENERAL_H */