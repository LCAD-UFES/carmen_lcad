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

#include "gridmap3d.h"

gridmap3d::gridmap3d(
    int dimension_cells,
    int height_cells,
    int cell_dimension_mm)
{
    this->dimension_cells = dimension_cells;
    this->height_cells = height_cells;
    this->cell_dimension_mm = cell_dimension_mm;

    position[0] = 0;
    position[1] = 0;
    position[2] = 0;

    int n = dimension_cells * dimension_cells * height_cells;
    prob_log = new GRID_CELL_DATA_TYPE[n];
    for (int i = 0; i < n; i++) prob_log[i] = CELL_STATE_UNKNOWN;

    colour = new unsigned char[dimension_cells*dimension_cells*3];
}

gridmap3d::~gridmap3d()
{
    delete [] prob_log;
    delete [] colour;
}

void gridmap3d::update_vacancy(
    int camera_centre_x, int camera_centre_y, int camera_centre_z,
    int point_x, int point_y, int point_z)
{
    int start_x = ((camera_centre_x-position[0]) / cell_dimension_mm) + (dimension_cells>>1);
    int start_y = ((camera_centre_y-position[1]) / cell_dimension_mm) + (dimension_cells>>1);
    int start_z = ((camera_centre_z-position[2]) / cell_dimension_mm) + (height_cells>>1);

    int dx = (point_x-camera_centre_x) / cell_dimension_mm;
    int dy = (point_y-camera_centre_y) / cell_dimension_mm;
    int dz = (point_z-camera_centre_z) / cell_dimension_mm;
    int length = (int)sqrt(dx*dx + dy*dy + dz*dz);

    if (length > 0) {
        for (int i=0;i<length;i++) {
            int x = start_x + (i*dx/length);
            if ((x>-1) && (x<dimension_cells)) {
                int y = start_y + (i*dy/length);
                if ((y>-1) && (y<dimension_cells)) {
                    int z = start_z + (i*dz/length);
                    if ((z>-1) && (z<height_cells)) {
                        int idx = CELL_INDEX(x, y, z, dimension_cells, height_cells);
                        if (prob_log[idx] > 0) prob_log[idx]--;
                    }
                    else {
                        break;
                    }
                }
                else {
                    break;
                }
            }
            else {
                break;
            }
        }
    }
}

void gridmap3d::get_occupancy_model(
    int length,
    int &model_length,
    int &model_peak,
    int &model_magnitude)
{
    model_length = length/10;
    if (model_length<1) model_length=1;
    model_length *= model_length;
    model_peak = model_length >> 2;
    if (model_peak<1) model_peak=1;
    model_magnitude = 1+(PROB_MAX/(1+(model_length>>2)));
}

void gridmap3d::update_occupancy(
    int view_centre_x, int view_centre_y, int view_centre_z,
    int point_x, int point_y, int point_z,
    unsigned char r, unsigned char g, unsigned char b)
{
    int start_x = ((view_centre_x-position[0]) / cell_dimension_mm) + (dimension_cells>>1);
    int start_y = ((view_centre_y-position[1]) / cell_dimension_mm) + (dimension_cells>>1);
    int start_z = ((view_centre_z-position[2]) / cell_dimension_mm) + (height_cells>>1);

    int dx = (point_x-view_centre_x) / cell_dimension_mm;
    int dy = (point_y-view_centre_y) / cell_dimension_mm;
    int dz = (point_z-view_centre_z) / cell_dimension_mm;
    int length = (int)sqrt(dx*dx + dy*dy + dz*dz);
    if (length > 0) {

        int model_length=1;
        int model_peak=0;
        int model_magnitude=1;
        get_occupancy_model(length,model_length,model_peak,model_magnitude);
        int length_max = length + 2;//model_length;
        float mult0 = model_magnitude / (float)model_peak;
        float mult1 = model_magnitude / (float)(length - model_peak);

        for (int i = length; i < length_max; i++) {
            int x = start_x + (i*dx/length);
            if ((x > -1) && (x < dimension_cells)) {
                int y = start_y + (i*dy/length);
                if ((y > -1) && (y < dimension_cells)) {
                    int z = start_z + (i*dz/length);
                    if ((z > -1) && (z < height_cells)) {

                        // adjust evidence
                        int idx = CELL_INDEX(x, y, z, dimension_cells, height_cells);
                        int mag = 0;
                        if (length <= model_peak) {
                            mag = (int)(i * mult0);
                        }
                        else {
                            mag = model_magnitude - (int)((i - model_peak) * mult1);
                        }
                        if (prob_log[idx] + mag <= CELL_MAX) {
                            prob_log[idx] += mag;
                        }
                        else {
                            prob_log[idx] = CELL_MAX;
                        }

                        // adjust colour
                        idx = (y*dimension_cells + x)*3;
                        if (colour[idx] == 0) {
                            colour[idx] = r;
                            colour[idx+1] = g;
                            colour[idx+2] = b;
                        }
                        else {
                            if (colour[idx] < r) colour[idx]++;
                            if (colour[idx] > r) colour[idx]--;
                            if (colour[idx+1] < g) colour[idx+1]++;
                            if (colour[idx+1] > g) colour[idx+1]--;
                            if (colour[idx+2] < b) colour[idx+2]++;
                            if (colour[idx+2] > b) colour[idx+2]--;
                        }
                    }
                    else {
                        break;
                    }
                }
                else {
                    break;
                }
            }
            else {
                break;
            }
        }
    }
}

void gridmap3d::insert(
    int camera_centre_x, int camera_centre_y, int camera_centre_z,
    float * points,    
    int image_width,
    int image_height,
    unsigned char * img)
{
    #pragma omp parallel for
    for (int y = 0; y < image_height; y++) {
        int n = y*image_width*3;
        for (int x = 0; x < image_width; x++, n+=3) {
            insert(
                camera_centre_x, camera_centre_y, camera_centre_z,
                (int)points[n], (int)points[n+1], (int)points[n+2],
                img[n], img[n+1], img[n+2]);
        }
    }
}


void gridmap3d::insert(
    int camera_centre_x, int camera_centre_y, int camera_centre_z,
    int point_x, int point_y, int point_z,
    unsigned char r, unsigned char g, unsigned char b)
{
    #pragma omp parallel for
    for (int i = 0; i < 2; i++) {
        if (i == 0) {
            update_vacancy(
                camera_centre_x, camera_centre_y, camera_centre_z,
                point_x, point_y, point_z);
        }
        else {
            update_occupancy(
                camera_centre_x, camera_centre_y, camera_centre_z,
                point_x, point_y, point_z, r, g, b);
        }
    }
}

void gridmap3d::show(
    int image_width,
    int image_height,
    unsigned char * img,
    int view)
{
    int x,y,cell_x,cell_y,cell_z,n,i=0;

    memset((void*)img,'\0',image_width*image_height*3);

    for (y = 0; y < image_height; y++) {
        for (x = 0; x < image_width; x++, i += 3) {
            switch(view) {
                case 0: {
                    cell_x = x * dimension_cells / image_width;
                    cell_z = y * height_cells / image_height;
                    for (cell_y = 0; cell_y < dimension_cells; cell_y++) {
                        n = CELL_INDEX(cell_x,cell_y,cell_z,dimension_cells,height_cells);
                        if (prob_log[n] > CELL_STATE_UNKNOWN) {
                            n = (cell_y*dimension_cells + cell_x)*3;
                            img[i] = colour[n];
                            img[i+1] = colour[n+1];
                            img[i+1] = colour[n+2];
                            break;
                        }
                    }
                    break;
                }
                case 1: {
                    cell_y = x * dimension_cells / image_width;
                    cell_z = y * height_cells / image_height;
                    for (cell_x = 0; cell_x < dimension_cells; cell_x++) {
                        n = CELL_INDEX(cell_x,cell_y,cell_z,dimension_cells,height_cells);
                        if (prob_log[n] > CELL_STATE_UNKNOWN) {
                            n = (cell_y*dimension_cells + cell_x)*3;
                            img[i] = colour[n];
                            img[i+1] = colour[n+1];
                            img[i+1] = colour[n+2];
                            break;
                        }
                    }
                    break;
                }
                case 2: {
                    cell_x = x * dimension_cells / image_width;
                    cell_y = y * dimension_cells / image_width;
                    n = (cell_y*dimension_cells + cell_x)*3;
                    if (colour[n]+colour[n+1]+colour[n+2]>0) {
                        img[i] = colour[n];
                        img[i+1] = colour[n+1];
                        img[i+1] = colour[n+2];
                    }
                    break;
                }
            }
            
        }
    }
}




