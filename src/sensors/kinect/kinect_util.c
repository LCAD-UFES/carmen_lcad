#include "kinect_util.h"
#include <malloc.h>
#include <stdio.h>

/**
 * http://nicolas.burrus.name/index.php/Research/KinectCalibration
 */
float
convert_kinect_depth_raw_to_meters(uint16_t raw_depth)
{
	if (raw_depth <= 2047)
	{
		return (float)(((double)1.0) / (((double)raw_depth) * -0.0030711016 + 3.3309495161));
	}
	return 0.0;
}

float
convert_kinect_depth_mm_to_meters(uint16_t raw_depth)
{
	return ((double)raw_depth)/1000.0;
}


uint16_t
convert_kinect_depth_meters_to_raw(float depth_in_meters)
{
	double x_inv = -1.0 / 0.0030711016;
	double y = 3.3309495161;
	if (depth_in_meters > 0.0)
	{
		return (uint16_t)( ( ((double)1.0) / ((double)depth_in_meters) - y) * x_inv);
	}
	return 0;
}

char *
convert_double_to_string(double value)
{
	int size= 25; //To hold . and null
	char *string = (char*)malloc(size * sizeof(char));
	sprintf(string,"%20.7f",value);
	return string;
}
