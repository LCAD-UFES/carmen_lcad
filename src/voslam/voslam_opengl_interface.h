#ifndef CARMEN_VOSLAM_OPENGL_INTERFACE_H_
#define CARMEN_VOSLAM_OPENGL_INTERFACE_H_

#define GL_GLEXT_PROTOTYPES

#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <math.h>
#include <vector>

#include <GL/glu.h>
#include <GL/freeglut.h>
#include <GL/glext.h>

#ifndef NO_CUDA

#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#endif

#include "voslam_messages.h"
#include "voslam_util.h"
#include "voslam_opengl_camera.h"

#ifdef _WIN32
#define DLL_EXPORT __declspec(dllexport)
#else
#define DLL_EXPORT
#endif

void init_gl();
void initLights();
int  init_glut(int argc, char **argv, void (*external_callback)(int));
bool init_global_variables(int number_of_point_clouds, int point_cloud_size, int point_cloud_dim);
void setCamera(float posX, float posY, float posZ, float targetX, float targetY, float targetZ);
void delete_vbo(const int vboId);
void drawString(const char *str, int x, int y, float color[4], void *font);
void drawString3D(const char *str, float pos[3], float color[4], void *font);
void showInfo();

extern "C" DLL_EXPORT void initialize_point_cloud_viewer(int argc, char **argv, int point_cloud_size, int point_cloud_dim, int number_of_point_clouds, void (*handler_t)(int), int is_cuda_enable, int device);
extern "C" DLL_EXPORT void run_glut();
extern "C" DLL_EXPORT void upload_data_to_vbo_array();
extern "C" DLL_EXPORT void allocate_vbo_arrays();
extern "C" DLL_EXPORT void clear_global_variables();
extern "C" DLL_EXPORT void copy_data_to_colors(float* c);
extern "C" DLL_EXPORT void copy_data_to_vertices(float* v);

#ifndef NO_CUDA
extern "C" DLL_EXPORT void upload_chain_depth_data_to_vbo_cuda_cloud(std::vector<carmen_voslam_pointcloud_t> keyframe_chain, int depth_width, int depth_height, float depth_focal_length_x, float depth_focal_length_y, float rgb_focal_length_x, float rgb_focal_length_y, float rgb_center_x, float rgb_center_y);
#endif

#endif /* SLAM6D_OPENGL_INTERFACE_H_ */
