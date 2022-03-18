
#ifndef __OUSTER_CONFIG_H__
#define __OUSTER_CONFIG_H__

#include "libouster_conn/os1.h"
#include "libouster_conn/os1_packet.h"

enum IntensityType
{
    INTENSITY=1,
    REFLECTIVITY,
    NOISE,
};

// @filipe: these values can be automatically obtained from the sensor. How to make them available to other modules besides the driver?
const int W = 1024;
//const int H = ouster::OS1::pixels_per_column;
const ouster::OS1::lidar_mode mode = ouster::OS1::MODE_1024x10;

// in degrees
const double ouster64_altitude_angles[64] = {
    16.611,  16.084,  15.557,  15.029,  14.502,  13.975,  13.447,  12.920,
    12.393,  11.865,  11.338,  10.811,  10.283,  9.756,   9.229,   8.701,
    8.174,   7.646,   7.119,   6.592,   6.064,   5.537,   5.010,   4.482,
    3.955,   3.428,   2.900,   2.373,   1.846,   1.318,   0.791,   0.264,
    -0.264,  -0.791,  -1.318,  -1.846,  -2.373,  -2.900,  -3.428,  -3.955,
    -4.482,  -5.010,  -5.537,  -6.064,  -6.592,  -7.119,  -7.646,  -8.174,
    -8.701,  -9.229,  -9.756,  -10.283, -10.811, -11.338, -11.865, -12.393,
    -12.920, -13.447, -13.975, -14.502, -15.029, -15.557, -16.084, -16.611,
};

// in degrees
const double ouster64_azimuth_offsets[64] = {
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
};


// in degrees - Carmen-ini lidar0_vertical
const double ouster32_altitude_angles[32] = {
	20.6, 19.34, 18.07, 16.8, 15.49, 14.19, 12.84, 11.48,
	10.13, 8.74, 7.36, 5.97, 4.56, 3.17, 1.77, 0.34,
	-1.06, -2.47, -3.87, -5.27, -6.66, -8.050000000000001, -9.449999999999999, -10.81,
	-12.16, -13.51, -14.84, -16.15, -17.45, -18.74, -19.97, -21.22
};

// in degrees
const double ouster32_azimuth_offsets[32] = {
	-1.40, -1.41, -1.40, -1.39, -1.41, -1.4, -1.41, -1.41,
	-1.41, -1.42, -1.40, -1.40, -1.41, -1.41, -1.41, -1.42,
	-1.41, -1.41, -1.40, -1.40, -1.40, -1.40, -1.41, -1.40,
	-1.40, -1.39, -1.40, -1.40, -1.41, -1.41, -1.38, -1.4
};


// in degrees - Carmen-ini lidar0_vertical
const double ousterOS032_altitude_angles[32] = {
	43.88, 40.91, 37.95, 35.01, 32.08, 29.17, 26.27, 23.38,
	20.52, 17.67, 14.84, 12.01, 9.19, 6.38, 3.57, 0.77,
	-2.04, -4.85, -7.65, -10.46, -13.29, -16.12, -18.96, -21.81,
	-24.68, -27.56, -30.46, -33.38, -36.32, -39.29, -42.28, -45.32
};


// in degrees
const double ousterOS032_azimuth_offsets[32] = {
		-3.28, -3.19, -3.11, -3.04, -2.98, -2.93, -2.88, -2.85,
		-2.83, -2.80, -2.79, -2.78, -2.77, -2.77, -2.77, -2.78,
		-2.80, -2.82, -2.84, -2.88, -2.90, -2.95, -2.99, -3.04,
		-3.11, -3.16, -3.25, -3.35, -3.44, -3.56, -3.7, -3.86
};



const char* 
intensity_type_to_string(int intensity_type)
{
    if (intensity_type == INTENSITY) return "INTENSITY";
    else if (intensity_type == REFLECTIVITY) return "REFLECTIVITY";
    else if (intensity_type == NOISE) return "NOISE";
    else return "INVALID";
}

#endif
