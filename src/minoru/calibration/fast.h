/*
 FAST corner detection
 Based upon code originally written by Ed Rosten
 Copyright (C) 2009 Bob Mottram and Giacomo Spigler
 fuzzgun@gmail.com

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>
#include <math.h>

#ifndef FAST_H_
#define FAST_H_

#define FAST_MAX_CORNERS 8192
#define FAST_MAX_CORNERS_PREVIOUS 255 // should be no greater than 255
#define FAST_MAX_IMAGE_HEIGHT 1024
#define FAST_MIN_CORNERS 50
#define FAST_IMAGE_SCALES 3
#define FAST_DESCRIPTOR_PIXELS 30
#define FAST_SUBPIXEL 32
#define FAST_PREVIOUS_BUFFER 8
#define FAST_DESCRIPTOR_RADIUS 5
#define FAST_DESCRIPTOR_WIDTH 25
#define Compare(X, Y) ((X)>=(Y))
#define fastpixindex(xx, yy)  ((yy * img_width + xx) * 3)

typedef struct { int x, y; } xy;

class fast {
private:
	int corner_score(unsigned char* p, int* pixel, int bstart);
	void detect(unsigned char* im, int xsize, int ysize, int stride, int b, int* ret_num_corners);
	void score(unsigned char* i, int stride, xy* corners, int num_corners, int b);
	void detect_nonmax(unsigned char* im, int xsize, int ysize, int stride, int b, int* ret_num_corners);
	void nonmax_suppression(xy* corners, int* scores, int num_corners);
	void make_offsets(int* pixel, int row_stride);
	void match_temporal(unsigned char* img_mono, int img_width, int img_height, int current_no_of_corners, xy* current_corners, int prev_no_of_corners, xy* prev_corners, unsigned char* matches, int max_disparity);
	bool FileExists(std::string filename);
	void compute_descriptor(unsigned char *img, int img_width, int img_height, int x, int y, int radius, int descriptor_index, unsigned int* descriptor, unsigned char* descriptor_colour, unsigned char* descriptor_direction);
	void create_descriptor_lookup(int radius, int width, int* lookup);

	int* previous_no_of_corners;
	xy** previous_corners;
	xy* corners;
	int* scores;
	int* row_start;
	unsigned char* img_mono;
	unsigned char* prev_img_mono;
	int threshold;
	unsigned char** temporal_matches;
	unsigned short* interocular_disparity;
	unsigned short** previous_interocular_disparity;
	int temporal_offset_x;
	int temporal_offset_y;
	int track_ctr;

	xy* nonmax;
	int num_nonmax;

public:
	void show(unsigned char *outbuf, int img_width, int img_height, int show_tracking);
	int update(unsigned char* img, int img_width, int img_height, int desired_features, int use_tracking);
	void save_matches(std::string filename, unsigned char* img, int img_width, bool colour);
	void load_matches(std::string filename, bool colour);
	void match_interocular(int img_width, int img_height, int no_of_stereo_matches, unsigned int* stereo_matches);
	void estimate_pan_tilt(int img_width, int img_height, int fov_degrees, int angle_multiplier);

	void update_descriptors(unsigned char *img, int img_width, int img_height, unsigned int* descriptor, unsigned char* descriptor_colour, unsigned char* descriptor_direction);
	int save_descriptors(std::string filename, unsigned char *img, int img_width, int img_height);

	int get_no_of_corners();
	int get_previous_no_of_corners();
	int get_no_of_disparities();
	int* get_corners();
	int* get_previous_corners();
	unsigned char* get_temporal_matches();

	fast();
	virtual ~fast();
};

#endif /* FAST_H_ */
