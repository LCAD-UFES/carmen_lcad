/*
    Point cloud functions
    Copyright (C) 2010 Bob Mottram and Giacomo Spigler
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

#ifndef POINT_CLOUD_H_
#define POINT_CLOUD_H_

#include <cctype>
#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <omp.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <math.h>

using namespace std;

#define POINT_CLOUD_VERSION		1.0
#define POINT_CLOUD_X_AXIS		0
#define POINT_CLOUD_Y_AXIS		2
#define POINT_CLOUD_Z_AXIS		1

#define POINT_CLOUD_FORMAT_POINTS	0
#define POINT_CLOUD_FORMAT_STL		1
#define POINT_CLOUD_FORMAT_X3D		2

#define rgb15(r,g,b)			((b>>4)|((g>>4)<<4)|((r>>4)<<8))

class pointcloud {
protected:
    static CvMat* matMul(const CvMat* A, const CvMat* B);

    static void remove_duplicate_surfaces(
        std::vector<std::vector<cv::Point> > &surfaces,
        std::vector<int> &surface_heights);

    static void virtual_camera_show_axes(
        CvMat * intrinsic_matrix,
        CvMat * distortion_coeffs,
        CvMat * &translation,
        CvMat * &rotation_vector,
        int image_width,
        int image_height,
        unsigned char * img_output);

    static void fill_surface(
        int * height,
        int dimension,
        int x, int y,
        int surface_height_mm,
        int cell_size_mm,
        std::vector<cv::Point> &surface,
        int depth);

    static bool inside_surface(
        std::vector<cv::Point> &vertices,
        float x, float y);

    static float surface_area(
        std::vector<cv::Point> &surface);

public:
    static void save(
        std::vector<float> &point,
        std::vector<unsigned char> &point_colour,
        int max_range_mm,
        CvMat * pose,
        int image_width,
        int image_height,
        float baseline,
        std::string point_cloud_filename);

     static void save(
        unsigned char * img_left,
        IplImage * points_image, 
        int max_range_mm,
        CvMat * pose,
        float baseline,
        std::string point_cloud_filename);

    static bool load(
        std::string point_cloud_filename,
        CvMat * &pose,
        int &image_width,
        int &image_height,
        float &baseline,
        std::vector<float> &point,
        std::vector<unsigned char> &point_colour);

    static void disparity_map_to_3d_points(
        float * disparity_map,
        int img_width,
        int img_height,
        CvMat * disparity_to_depth,
        CvMat * pose,
        IplImage * &disparity_image,
        IplImage * &points_image);

    static void show(
        IplImage * points_image,
        float * disparity_map,
        unsigned char * img_left,
        CvMat * pose,
        float max_range_mm,
        float max_height_mm,
        int view_type,
        int output_image_width,
        int output_image_height,
        unsigned char * img_output);

    static void virtual_camera(
        unsigned char * img,
        IplImage * points_image,
        CvMat * pose,
        CvMat * intrinsic_matrix,
        CvMat * distortion_coeffs,
        float max_range_mm,
        bool view_point_cloud,
        unsigned char * img_output);

    static void virtual_camera(
        std::vector<float> &point,
        std::vector<unsigned char> &point_colour,
        CvMat * pose,
        CvMat * intrinsic_matrix,
        CvMat * distortion_coeffs,
        float max_range_mm,
        float * &depth,
        CvMat * &rotation_matrix,
        CvMat * &translation,
        CvMat * &rotation_vector,
        CvMat * &points,
        CvMat * &image_points,
        bool view_point_cloud,
        bool show_axes,
        int image_width,
        int image_height,
        unsigned char * img_output);

    static void virtual_camera(
        unsigned char * img,
        IplImage * points_image,
        CvMat * pose,
        CvMat * intrinsic_matrix,
        CvMat * distortion_coeffs,
        float max_range_mm,
        float * &depth,
        CvMat * &rotation_matrix,
        CvMat * &translation,
        CvMat * &rotation_vector,
        CvMat * &points,
        CvMat * &image_points,
        bool view_point_cloud,
        unsigned char * img_output);

    static void obstacle_map(
        IplImage * points_image,
        int map_dimension,
        int map_cell_size_mm,
        CvMat * pose,
        float relative_x_mm,
        float relative_y_mm,
        int threshold,
        float tilt_degrees,
        int * map);

    static void fill(
        int id,
        int * map,
        int map_dimension,
        int x,
        int y,
        int depth,
        int &ctr,
        int threshold);

    static void find_objects(
        int format,
        unsigned char * img,
        IplImage * points_image,
        int map_dimension,
        int map_cell_size_mm,
        CvMat * pose,
        float relative_x_mm,
        float relative_y_mm,
        int threshold,
        float tilt_degrees,
        int * map,
        int min_area_mm2,
        int max_area_mm2,
        bool BGR,
        std::vector<std::vector<float> > &objects);

    static int get_object_id(
        int x,
        int y,
        int width,
        float cos_tilt,
        float sin_tilt,
        int centre,
        float mult,
        float relative_x_mm,
        float relative_y_mm,
        int map_dimension,
        int threshold,
        int * map,
        float * points_image_data,
        float pose_x,
        float pose_y,
        float pose_z,
        float &x2,
        float &y2,
        float &z2);

    static void surface_normal(
        float x0, float y0, float z0,
        float x1, float y1, float z1,
        float x2, float y2, float z2,
        float &nx, float &ny, float &nz);

    static void save_point_cloud_x3d(
        std::string filename,
        int image_width,
        int image_height,
        CvMat * pose,
        float baseline,
        std::vector<float> &point,
        std::vector<unsigned char> &point_colour);

    static void save_mesh_x3d(
        std::string filename,
        int image_width,
        int image_height,
        CvMat * pose,
        float baseline,
        std::vector<float> &facets);

    static void save_stl_binary(
        std::string filename,
        int image_width,
        int image_height,
        CvMat * pose,
        float baseline,
        std::vector<float> &facets);

    static void save_stl_ascii(
        std::string filename,
        int image_width,
        int image_height,
        CvMat * pose,
        float baseline,
        std::vector<float> &facets);

    static void save_largest_object(
        std::string filename,
        int format,
        bool binary,
        int image_width,
        int image_height,
        CvMat * pose,
        float baseline,
        std::vector<std::vector<float> > &objects);

    static void export_points(
        int format,
        unsigned char * img,
        IplImage * points_image,
        CvMat * pose,
        float tilt_degrees,
        bool BGR,
        std::vector<float> &points,
        int max_range_mm);

    static void enlarge_surface(
        std::vector<cv::Point> &surface,
        std::vector<cv::Point> &enlarged,
        int enlarge_percent);

    static bool connected_surfaces(
        std::vector<cv::Point> &surface1,
        std::vector<cv::Point> &surface2);

    static bool join_surfaces(
        std::vector<std::vector<cv::Point> > &surfaces,
        std::vector<int> &surface_heights,
        int max_height_difference);

    static void height_field(
        std::vector<float> &point,
        int camera_height_mm,
        int map_dimension_mm,
        int cell_size_mm,
        int min_height_mm,
        int max_height_mm,
        int * height);

    static void find_horizontal_surfaces(
        int map_dimension_mm,
        int cell_size_mm,
        int min_height_mm,
        int patch_surface_area_mm2,
        int min_surface_area_mm2,
        int * height,
        std::vector<std::vector<cv::Point> > &surfaces,
        std::vector<int> &surface_heights);

    static void overhead_occupancy(
        std::vector<float> &point,
        int map_dimension_mm,
        int cell_size_mm,
        int * map);

    static void detect_horizontal_surfaces(
        std::vector<float> &point,
        int camera_height_mm,
        int map_dimension_mm,
        int cell_size_mm,
        int min_height_mm,
        int max_height_mm,
        int patch_surface_area_mm2,
        int min_surface_area_mm2,
        std::vector<std::vector<cv::Point> > &surfaces,
        std::vector<int> &surface_heights);

    static void horizontal_surfaces_points(
        std::vector<float> &point,
        std::vector<unsigned char> &point_colour,
        int camera_height_mm,
        int map_dimension_mm,
        int cell_size_mm,
        int min_height_mm,
        int max_height_mm,
        int patch_surface_area_mm2,
        int min_surface_area_mm2,
        std::vector<float> &horizontal_point,
        std::vector<unsigned char> &horizontal_point_colour);

    static void colour_surfaces_points(
        std::vector<float> &point,
        std::vector<unsigned char> &point_colour,
        int camera_height_mm,
        int map_dimension_mm,
        int cell_size_mm,
        int min_height_mm,
        int max_height_mm,
        int patch_surface_area_mm2,
        int min_surface_area_mm2,
        int r, int g, int b);

    static void colour_surface_objects(
        std::vector<float> &point,
        std::vector<unsigned char> &point_colour,
        int camera_height_mm,
        int map_dimension_mm,
        int cell_size_mm,
        int min_height_mm,
        int max_height_mm,
        int patch_surface_area_mm2,
        int min_surface_area_mm2,
        int r, int g, int b);

    static void prune(
        std::vector<float> &point,
        std::vector<unsigned char> &point_colour,
        int map_dimension_mm,
        int map_height_mm,
        int cell_size_mm,
        int camera_height_mm,
        int min_threshold);

};

#endif

