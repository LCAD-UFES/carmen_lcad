/*    
    3D occupancy grid map
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

#ifndef GRID_MAP_3D_H_
#define GRID_MAP_3D_H_

#include <stdio.h>
#include <omp.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <math.h>

#define GRID_CELL_DATA_TYPE	unsigned char
#define CELL_INDEX(x,y,z,dim,h)	((y*dim*h) + (x*h) + z)
#define CELL_STATE_UNKNOWN	127
#define CELL_MAX                255
#define PROB_MAX		32

class gridmap3d {
protected:
    // probabilities as log odds
    GRID_CELL_DATA_TYPE * prob_log;
    unsigned char * colour;

    void get_occupancy_model(
        int length,
        int &model_length,
        int &model_peak,
        int &model_magnitude);

    void update_vacancy(
        int camera_centre_x, int camera_centre_y, int camera_centre_z,
        int point_x, int point_y, int point_z);

    void update_occupancy(
        int view_centre_x, int view_centre_y, int view_centre_z,
        int point_x, int point_y, int point_z,
        unsigned char r, unsigned char g, unsigned char b);

public:
    // position of the grid
    float position[3];

    // grid dimensions
    int dimension_cells;
    int height_cells;

    // size of each grid voxel
    int cell_dimension_mm;

    void show(
        int image_width,
        int image_height,
        unsigned char * img,
        int view);

    void insert(
        int camera_centre_x, int camera_centre_y, int camera_centre_z,
        float * points,    
        int image_width,
        int image_height,
        unsigned char * img);

    // insert ray
    void insert(
        int camera_centre_x, int camera_centre_y, int camera_centre_z,
        int point_x, int point_y, int point_z,
        unsigned char r, unsigned char g, unsigned char b);

    gridmap3d(
        int dimension_cells,
        int height_cells,
        int cell_dimension_mm);

    ~gridmap3d();
};

#endif

