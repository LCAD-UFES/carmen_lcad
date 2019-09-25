#ifndef CARMEN_STEREO_UTIL_H
#define CARMEN_STEREO_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif

#define BB2_INDEX 1
#define XB3_INDEX 1
#define BB2_NAME "BB2"
#define XB3_NAME "XB3"

typedef struct{
  double baseline;//distance between cameras (in meters)
  double vfov; //vertical FieldOfView (in radians)
  double hfov;//horizontal FieldOfView (in radians)
  float fx; // horizontal focal length (in pixels)
  float fy; // vertical focal length (in pixels)
  //principal point of the imager
  float xc;
  float yc;
  //image size
  int width;
  int height;
  //camera model
  int camera_model;
  // stride to make stereo_velodyne more sparse
  float stereo_stride_x;
  float stereo_stride_y;
} stereo_util;

stereo_util get_stereo_instance(int camera, int width, int height);
double get_focal_lenght_in_meters(stereo_util instance);
double get_accuracy_in_X(stereo_util instance, int disparity);
carmen_vector_3D_t camera_to_world(carmen_position_t right_point, double camera_height, double camera_pitch, stereo_util instance);
void getDepthMap(carmen_vector_3D_t *image3D, float *depthMap, stereo_util instance);
void reproject_to_3D(float *disparityMap, carmen_vector_3D_t *image3D, double theta, stereo_util instance);
void get_depth_map_from_disparity_map(float *disparityMap, unsigned short* depthMap, stereo_util instance, double MAX_DEPTH);
void get_pointcloud_from_disparity_map(float *disparityMap, carmen_vector_3D_t* pointcloud, stereo_util instance, double *position, double *rotation);
carmen_vector_3D_p
reproject_single_point_to_3D(const stereo_util *camera, carmen_position_t p2D, double disparity);
carmen_vector_3D_p
reproject_single_point_to_3D_with_depth(stereo_util instance, carmen_position_t right_point, unsigned short depth);
carmen_vector_3D_p
transform_camera_to_world_frame(const carmen_vector_3D_p camera_point, double theta);

carmen_vector_3D_p
reproject_single_point_to_3D_in_left_camera_frame(carmen_position_t left_camera_projection, carmen_position_t right_camera_projection, double baseline, double focal_length);
carmen_vector_3D_p
reproject_single_point_to_3D_in_left_camera_frame_with_bias_reduction(carmen_position_t left_camera_projection, carmen_position_t right_camera_projection, double baseline, double focal_length, double bias_std_deviation);

#ifdef __cplusplus
}
#endif

#endif
