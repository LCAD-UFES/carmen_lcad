#ifndef CARMEN_LIB_V_DISPARITY_H
#define CARMEN_LIB_V_DISPARITY_H 1

/* Carmen includes */
#include <carmen/global.h>

/* Stereo includes */
#include <carmen/stereo_util.h>

/* OpenCV Includes */
#include<opencv/cv.h>
#include<omp.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#define RED_OBSTACLES_DISTANCE  10.0
#define GREEN_OBSTACLES_DISTANCE  20.0
#define MINIMUM_OBSTACLE_HEIGHT(camera_height)  (0.1 - camera_height)
#define MAXIMUM_OBSTACLE_HEIGHT(camera_height)  (4.5 - camera_height)

#ifdef __cplusplus
extern "C" {
#endif

/* A line segment represented as start and end points */
typedef struct
{
  CvPoint A;
  CvPoint B;
} cvLineSegment;

typedef struct{
  stereo_util stereo_util_instance;
  int stereo_disparity;
  CvMemStorage *memory_storage, *polygon_storage;
  CvSeq *polygon;
} v_disparity;

/* Constructor and Destructor */
v_disparity get_v_disparity_instance(stereo_util stereo_util_instance, int stereo_disparity);
void destroy_v_disparity_instance(v_disparity);

void build_polygon(CvPoint *edges, int n_edges, v_disparity instance);

cvLineSegment *find_hough_lines(const IplImage *image, int *n_lines, v_disparity instance);
double get_slope_normalized_in_degrees(CvPoint A, CvPoint B, int image_height);
void compute_height_and_pitch(IplImage *v_disparity_map, double slope_threshould, cvLineSegment *lines, int n_lines, double *height, double *pitch, double *horizon_line, v_disparity instance);

/* V-Disparity */
int get_v_disparity_size(v_disparity);

unsigned short int *alloc_v_disparity_map(v_disparity instance);
void fill_v_disparity_map(unsigned short int *v_disparity_data, float *disparity_map, v_disparity instance);
void fill_v_disparity_image(unsigned short int *v_disparity_data, IplImage *v_disparity_map, v_disparity instance);

IplImage *alloc_v_disparity_map_image(v_disparity instance);
int draw_road_profile_lines_in_v_disparity(IplImage *road_profile_image, IplImage *v_disparity_map, double slope_threshould, v_disparity instance);

/* U-Disparity */
int get_u_disparity_size(v_disparity);

unsigned short int *alloc_u_disparity_map(v_disparity instance);
void fill_u_disparity_map(unsigned short int *u_disparity_data, float *disparity_map, v_disparity instance);

IplImage *alloc_u_disparity_map_image(v_disparity instance);
void fill_u_disparity_image(unsigned short int *u_disparity_data, IplImage *u_disparity_map, v_disparity instance);

void compute_v_disparity_info(unsigned short int *v_disparity_data, IplImage *v_disparity_map, double slope_threshould, float *disparity_map, double *height, double *pitch, double *horizon_line, v_disparity instance);

void print_obstacles(IplImage *dst_image, IplImage *v_disparity_map, IplImage *road_profile_image,
    float *disparity_map, double camera_height, double camera_pitch,
    v_disparity instance);

void print_u_disparity_obstacles(IplImage *dst_image, IplImage *u_disparity_map, float *disparity_map, double camera_height, double camera_pitch,
    v_disparity instance);

int get_road_pixels_list(IplImage *src_image, CvPoint *points, CvScalar *samples,
    IplImage *road_profile_image, float *disparity_map, int only_inside_polygon, v_disparity instance);

#ifdef __cplusplus
}
#endif

#endif
