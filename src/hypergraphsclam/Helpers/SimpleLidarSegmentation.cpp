#include <SimpleLidarSegmentation.hpp>
#include <iostream>

using namespace hyper;

// basic constructor
SimpleLidarSegmentation::SimpleLidarSegmentation() :
    xsize(0),
    ysize(0),
    ox(0),
    oy(0),
    minz(0.0f),
    maxz(0.0f),
    inverse_res(1.0 / LIDAR_GRID_CELL_RESOLUTION),
    minz_grid(nullptr),
    maxz_grid(nullptr),
    count_grid(nullptr) {}

// basic destructor
SimpleLidarSegmentation::~SimpleLidarSegmentation() {

    // delete the old grids
    RemoveGrid2D<double>(minz_grid);
    RemoveGrid2D<double>(maxz_grid);
    RemoveGrid2D<unsigned>(count_grid);

}

// update the grid size
void SimpleLidarSegmentation::UpdateGrids(double absx, double absy) {

    // get the cloud dimension
    unsigned mxsize = 2 * unsigned(std::floor(absx * inverse_res + 2.0));
    unsigned mysize = 2 * unsigned(std::floor(absy * inverse_res + 2.0));

    if (xsize < mxsize || ysize < mysize) {

        // delete the old grids
        RemoveGrid2D<double>(minz_grid);
        RemoveGrid2D<double>(maxz_grid);
        RemoveGrid2D<unsigned>(count_grid);

        // update the grid size
        xsize = mxsize;
        ysize = mysize;

        // update the origin
        ox = xsize / 2;
        oy = ysize / 2;

        // rebuild the min, max and count grids
        minz_grid = BuildGrid2D<double>(xsize, ysize, std::numeric_limits<double>::max());
        maxz_grid = BuildGrid2D<double>(xsize, ysize, -std::numeric_limits<double>::max());
        count_grid = BuildGrid2D<unsigned>(xsize, ysize, unsigned(0));

    } else {

        ClearGrid2D<double>(minz_grid, std::numeric_limits<double>::max());
        ClearGrid2D<double>(maxz_grid, -std::numeric_limits<double>::max());
        ClearGrid2D<unsigned>(count_grid, 0);

    }

}

// iterate the entire cloud and update the min/max/count values
void SimpleLidarSegmentation::FirstPassAnalysis(PointCloudHSV &cloud) {

    // iterators
    PointCloudHSV::iterator it(cloud.begin());
    PointCloudHSV::iterator end(cloud.end());

    while (end != it) {

        // direct access
        pcl::PointXYZHSV &p(*it);

        // compute the coordinates
        unsigned xi = unsigned(std::floor(p.x * inverse_res + 0.5) + double(ox));
        unsigned yi = unsigned(std::floor(p.y * inverse_res + 0.5) + double(oy));

        // get the min max values
        double &min(minz_grid[xi][yi]);
        double &max(maxz_grid[xi][yi]);

        // update the min max values
        if (min > p.z) min = p.z;
        if (max < p.z) max = p.z;

        // update the counter
        count_grid[xi][yi] += 1;

        ++it;

    }

}

// iterate the entire cloud and update the points
void SimpleLidarSegmentation::SecondPassAnalysis(PointCloudHSV &cloud) {

    // iterators
    PointCloudHSV::iterator it(cloud.begin());
    PointCloudHSV::iterator end(cloud.end());

    pcl::PointXYZHSV reference(*it);

    while (end != it) {

        // direct access
        pcl::PointXYZHSV &p(*it);

        // compute the coordinates
        unsigned xi = unsigned(std::floor(p.x * inverse_res + 0.5) + double(ox));
        unsigned yi = unsigned(std::floor(p.y * inverse_res + 0.5) + double(oy));

        // the sparse counter
        unsigned s = count_grid[xi][yi];

        for (int i = -5; i < 6; ++i) {

            int xipi = xi + i;

            if (unsigned(xipi) < xsize && -1 < xipi) {

                for (int j = -5; j < 6; ++j) {

                    if (0 != i && 0 != j) {

                        int yipj = yi + j;

                        if (unsigned(yipj) < ysize && -1 < yipj) {

                            s += count_grid[xipi][yipj];

                        }

                    }

                }

            }

        }

        // get the min max values
        double &min(minz_grid[xi][yi]);
        double &max(maxz_grid[xi][yi]);

        // compute the z displacement
        double dmmz = std::fabs(max - min);

        // is it a tall object?
        bool tall = 2.2f + minz < max || 3.10 < dmmz;

        // is it a ground object?
        bool ground = 1.00f + minz > max && 0.25f > dmmz;

        if (2 > s || (!tall && !ground)) {

            p.x = reference.x;
            p.y = reference.y;
            p.z = reference.z;

        } else {

            // verify if it's a tall object
            if (tall) {

                p.h = 96.0f;
                p.s = 1.0f;
                p.v = 0.5f;

            } else if (ground) {

                p.h = 23.0f;
                p.s = 1.0f;
                p.v = 0.5f;

            }

        }

        ++it;

    }

}

// type segmentation
void SimpleLidarSegmentation::PointTypeSegmentation(PointCloudHSV &cloud, double absx, double absy, double _minz, double _maxz) {

    // reset the min max z valures
    minz = _minz;
    maxz = _maxz;

    // verify and update the current grid size
    UpdateGrids(absx, absy);

    // fill the min/max and counter grids
    FirstPassAnalysis(cloud);

    // update the point colors
    // and move the undesired points to the first one
    SecondPassAnalysis(cloud);

}