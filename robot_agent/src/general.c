/**
 * @file	general.c
 * @author  Eriks Zaharans
 * @date    1 Jul 2013
 *
 * @section DESCRIPTION
 *
 * General function library.
 */

 /* -- Includes -- */
/* system libraries */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
/* project libraries */
#include "general.h"

/* -- Defines -- */

/* -- Functions -- */

/**
 * Convert two bytes to int (two's complement)
 * @param byte_high High byte of data
 * @param byte_low Low byte of data
 * @return Integer (two's complement)
 */
int general_bytes2int(unsigned char byte_high, unsigned char byte_low)
{
    int num;

    //Convert to int (two's complement)
    if((byte_high & (1 << 7)) == (1 << 7))
    {
        num = (byte_low + (byte_high << 8)) - 0xFFFF;
    }
    else
    {
        num = byte_low + (byte_high << 8);
    }

    return num;
}

 /**
 * Convert two bytes to unsigned int
 * @param byte_high High byte of data
 * @param byte_low Low byte of data
 * @return Unsigned integer
 */
unsigned int general_bytes2uint(unsigned char byte_high, unsigned char byte_low)
{
    int num;

    //Convert to unsigned int
    num = byte_low + (byte_high << 8);

    return num;
}

/**
 * Convert int to two bytes (two's complement)
 * @param integer Integer to be converted
 * @param byte_high Pointer to high byte of data
 * @param byte_low Pointer to low byte of data
 * @return Integer (two's complement)
 */
void general_int2bytes(int integer, unsigned char *byte_high, unsigned char *byte_low)
{
    int num = integer;

    // Çonevrt integer to 2 bytes
    // If number is negative
    if(num < 0)
    {
        num += 0xFFFF;
    }
    // Split
    *byte_high = (num >> 8) & 0xFF;
    *byte_low = (num) & 0xFF;
}

/**
 * Random number based on normal (gaussian) distribution
 * @source http://c-faq.com/lib/gaussian.html [Donald E. Knuth]
 * @param mu Mean of gaussian
 * @param sigma Variance of gaussian
 * @return Random number based on normal distribution
 */
double general_gaussrand(double mu, double sigma)
{
    static double V1, V2, S;
    static int phase = 0;
    double X;

    if(phase == 0)
    {
        do
        {
            double U1 = (double)rand() / RAND_MAX;
            double U2 = (double)rand() / RAND_MAX;

            V1 = 2 * U1 - 1;
            V2 = 2 * U2 - 1;
            S = V1 * V1 + V2 * V2;
        } while(S >= 1 || S == 0);

        X = V1 * sqrt(-2 * log(S) / S);
    }
    else
    {
        X = V2 * sqrt(-2 * log(S) / S);
    }

    phase = 1 - phase;

    return X * sigma + mu;
}

/**
 * Calculate Gaussian
 * @param mu Mean of gaussian
 * @param sigma Variance of gaussian
 * @param x Position on x axis
 * @return Gaussian value at x
 */
float general_gaussian(float mu, float sigma, int x)
{
    return exp(- (pow((mu - x), 2)) / (pow(sigma, 2)) / 2.0) / sqrt(M_PI * 2 * (pow(sigma, 2)));
}

/**
 * Calculate squared distance between two points
 * @param x1 X coordinate of 1st point
 * @param y1 Y coordinate of 1st point
 * @param x2 X coordinate of 2nd point
 * @param y2 Y coordinate of 1st point
 * @return Squared distance between points
 */
float general_dist_squared(int x1, int y1, int x2, int y2)
{
    return pow(x1 - x2, 2) + pow(y1 - y2, 2);
}

/**
 * Calculate distance from point to line segment
 * @param x X coordinate of point
 * @param y Y coordinate of point
 * @param p1 Array of x and y coordinates for 1st line point
 * @param p2 Array of x and y coordinates for 2nd line point
 * @return Distance from point to line segment
 */
float general_dist2seg(int x, int y, int p1[], int p2[])
{
    float l2 = general_dist_squared(p1[0], p1[1], p2[0], p2[1]);

    if (l2 == 0)
        return general_dist_squared(x, y, p1[0], p1[1]);

    float t = ((x - p1[0]) * (p2[0] - p1[0]) + (y - p1[1]) * (p2[1] - p1[1])) / l2;

    if (t < 0)
        return general_dist_squared(x, y, p1[0], p1[1]);

    if (t > 1)
        return general_dist_squared(x, y, p2[0], p2[1]);

    return general_dist_squared(x, y, (p1[0] + t * (p2[0] - p1[0])), (p1[1] + t * (p2[1] - p1[1])));
}

/**
 * Draw (fill) filled circle in 2D array
 * @source http://en.wikipedia.org/wiki/Midpoint_circle_algorithm // Code modified
 * @param cx Center x coordinate
 * @param cy Center y coordinate
 * @param radius Radius of circle
 * @return Void
 */
void general_circle(int **arr, int cx, int cy, int radius)
{
    int error = -radius;
    int x = radius;
    int y = 0;

    while (x >= y)
    {
        general_circle_eight_points(arr, cx, cy, x, y);

        error += y;
        ++y;
        error += y;

        if (error >= 0)
        {
            error -= x;
            --x;
            error -= x;
        }
    }
}

/**
 * Draw (fill) eight points - Part of general_circle() function
 * @source http://en.wikipedia.org/wiki/Midpoint_circle_algorithm // Code modified
 * @param cx Center x coordinate
 * @param cy Center y coordinate
 * @param x X coordinate
 * @param y Y coordinate
 * @return Void
 */
void general_circle_eight_points(int **arr, int cx, int cy, int x, int y)
{
    general_circle_four_points(arr, cx, cy, x, y);
    if (x != y)
    {
        general_circle_four_points(arr, cx, cy, y, x);
    }
}

/**
 * Draw (fill) four points - Part of general_circle() function
 * @source http://en.wikipedia.org/wiki/Midpoint_circle_algorithm // Code modified
 * @param cx Center x coordinate
 * @param cy Center y coordinate
 * @param x X coordinate
 * @param y Y coordinate
 * @return Void
 */
void general_circle_four_points(int **arr, int cx, int cy, int x, int y)
{
    general_circle_horizontal_line(arr, cx - x, cy + y, 2 * x + 1);
    general_circle_horizontal_line(arr, cx - x, cy - y, 2 * x + 1);
}

/**
 * Draw (fill) horizontal line - Part of general_circle() function
 * @source http://en.wikipedia.org/wiki/Midpoint_circle_algorithm // Code modified
 * @param x Start x coordinate
 * @param y Start y coordinate
 * @param len Length of line
 * @return Void
 */
void general_circle_horizontal_line(int **arr, int x, int y, int len)
{
	// Local variables
	int i;
	
    for(i = 0; i < len; i++)
    {
        arr[x + i][y] = 1;
    }
}


