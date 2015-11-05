#ifndef CARMEN_STEREO_MAPPING_MAP_H
#define CARMEN_STEREO_MAPPING_MAP_H

#ifdef __cplusplus
extern "C" {
#endif

#define N_DISTANCES 4

void compute_map_size();
double grid_x_map(ProbabilisticMapParams map_params, int x);
double grid_y_map(ProbabilisticMapParams map_params, int y);
int map_grid_x(ProbabilisticMapParams map_params, double x);
int map_grid_y(ProbabilisticMapParams map_params, double y);
int is_map_grid_cell_valid(ProbabilisticMapParams map_params, int x, int y);
void inverse_perspective_mapping(ProbabilisticMapParams map_params, IplImage *dst, IplImage *src, double height, double pitch, int horizon_line, int *perceptual_mask, stereo_util stereo_util_instance);
void reverse_inverse_perspective_mapping(ProbabilisticMapParams map_params, IplImage *dst, IplImage *src, double height, double pitch, stereo_util stereo_util_instance);
void camera_to_world_build_map(ProbabilisticMapParams map_params, IplImage *dst, IplImage *src, double camera_height, double camera_pitch, int horizon_line, stereo_util stereo_util_instance);
void opencv_birds_eye_remap(ProbabilisticMapParams map_params, const IplImage *src_image, IplImage *dst_image, double camera_height, double camera_pitch, stereo_util stereo_util_instance);
void draw_lateral_offset(ProbabilisticMapParams map_params, IplImage *dst_image);
void update_probabilistic_map(ProbabilisticMap probabilistic_map, ProbabilisticMapParams map_params, carmen_point_t xt, carmen_point_t xt_1, float *zt);
void compare_results(ProbabilisticMapParams map_params, IplImage *ground_truth, IplImage *result_map, int *mask);

#ifdef __cplusplus
}
#endif

#endif
