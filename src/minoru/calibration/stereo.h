/*
    stereo
    Functions for simple sparse stereo
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

#ifndef STEREO_H_
#define STEREO_H_

#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <fstream>
#include "polynomial.h"
#include "linefit.h"

#define SVS_MAX_FEATURES         8000
#define SVS_MAX_MATCHES          2000
#define SVS_MAX_IMAGE_WIDTH      1024
#define SVS_MAX_IMAGE_HEIGHT     1024
#define SVS_VERTICAL_SAMPLING    2
#define SVS_HORIZONTAL_SAMPLING  8
#define SVS_DESCRIPTOR_PIXELS    30
#define SVS_MAX_LINES            200
#define SVS_PEAKS_HISTORY        10
#define SVS_FILTER_SAMPLING      40
#define SVS_SUB_PIXEL            32
#define SVS_PEAK_WIDTH           6

#define SVS_MAX_REGIONS          200
#define SVS_REGION_HISTORY       100

#define pixindex(xx, yy)  ((yy * imgWidth + xx) * 3)

class svs {
public:
    unsigned int imgWidth, imgHeight;

    /* array storing x coordinates of detected features */
    short int* feature_x;

    /* array storing y coordinates of detected features */
    short int* feature_y;

    /* array storing the number of features detected on each row */
    unsigned short int* features_per_row;

    /* array storing the number of features detected on each column */
    unsigned short int* features_per_col;

    /* Array storing a binary descriptor, 32bits in length, for each detected feature.
     * This will be used for matching purposes.*/
    unsigned int* descriptor;

    /* mean luminance for each feature */
    unsigned char* mean;

    /* buffer which stores sliding sum */
    int* row_sum;

    /* low cotrast areas of the image */
    unsigned short* low_contrast;
    int enable_segmentation;
    int enable_region_tracking;

    /* region volume in pixels */
    unsigned int* region_volume;

    /* centre of each region */
    unsigned int* region_centre;
    unsigned short** prev_region_centre;
    int region_history_index;

    unsigned char* region_disparity;

    /* bounding box of each region */
    unsigned short* region_bounding_box;

    /* colour of each region */
    unsigned int* region_colour;

    /* number of detected regions */
    int no_of_regions;

    /* buffer used to find peaks in edge space */
    unsigned int* row_peaks;
    unsigned int* temp_row_peaks;

    /* array stores matching probabilities (prob,x,y,disp) */
    unsigned int* svs_matches;

    /* used during filtering */
    unsigned char* valid_quadrants;

    /* array used to store a disparity histogram */
    unsigned short int* disparity_histogram;
    int* disparity_histogram_plane;
    int* disparity_plane_fit;

    /* number of detected planes found during the filtering step */
    int no_of_planes;
    int* plane;

    /* maps raw image pixels to rectified pixels */
    int* calibration_map;

    unsigned int av_peaks;

    /* non zero if ground plane is to be used */
    int enable_ground_priors;

    /* percent of the vertical resolution indicating
     * the ground plane position */
    int ground_y_percent;

    int update_sums(int cols, int y, unsigned char* rectified_frame_buf, int segment);
    void non_max(int cols, int inhibition_radius, unsigned int min_response);
    int compute_descriptor(int px, int py, unsigned char* rectified_frame_buf, int no_of_features, int row_mean);
    int get_features_horizontal(unsigned char* rectified_frame_buf, int inhibition_radius, unsigned int minimum_response, int calibration_offset_x, int calibration_offset_y, int segment);
    int get_features_vertical(unsigned char* rectified_frame_buf, int inhibition_radius, unsigned int minimum_response, int calibration_offset_x, int calibration_offset_y, int segment);
    void filter_plane(int no_of_possible_matches, int max_disparity_pixels);
    int match(svs* other, int ideal_no_of_matches, int max_disparity_percent, int learnDesc, int learnLuma, int learnDisp, int learnGrad, int groundPrior, int use_priors);
    int fit_plane(int no_of_matches, int max_deviation, int no_of_samples);
    void segment(unsigned char* rectified_frame_buf, int no_of_matches);

    void calibrate_offsets(unsigned char* left_image, unsigned char* right_image, int x_range, int y_range, int& calibration_offset_x, int& calibration_offset_y);
    void make_map(float centre_of_distortion_x, float centre_of_distortion_y, float coeff_0, float coeff_1, float coeff_2, float rotation, float scale);
    void make_map_int(long centre_of_distortion_x, long centre_of_distortion_y, long* coeff, long scale_num, long scale_denom);
    void rectify(unsigned char* raw_image, unsigned char* rectified_frame_buf);
    void flip(unsigned char* raw_image, unsigned char* flipped_frame_buf);

    bool FileExists(std::string filename);
    void save_matches(std::string filename, unsigned char* rectified_frame_buf, int no_of_matches, bool colour);
    bool log_matches(std::string filename, unsigned char* rectified_frame_buf, int no_of_matches, bool colour);
    static void histogram_equalise(IplImage* hist_image, unsigned char* img, int img_width, int img_height);

    svs(int width, int height);
    ~svs();
};

#endif

