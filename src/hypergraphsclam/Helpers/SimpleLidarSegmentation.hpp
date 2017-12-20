#ifndef HYPERGRAPHSLAM_SIMPLE_LIDAR_SEGMENTATION_HPP
#define HYPERGRAPHSLAM_SIMPLE_LIDAR_SEGMENTATION_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

namespace hyper {

#define LIDAR_GRID_CELL_RESOLUTION 0.3f

// syntactic sugar
typedef pcl::PointCloud<pcl::PointXYZHSV> PointCloudHSV;

class SimpleLidarSegmentation {

    private:

        // the grid sizes
        unsigned xsize, ysize;

        // the grid origin
        unsigned ox, oy;

        // the min max z valures
        double minz, maxz;

        // the inverse resolution
        double inverse_res;

        // the min/max grid
        double **minz_grid, **maxz_grid;

        // the count grid
        unsigned **count_grid;

        // build a new 2D grid
        template<typename T>
        T** BuildGrid2D(unsigned _xsize, unsigned _ysize, T value) {

            T **grid = new T*[_xsize];
            T *contiguous = new T[_xsize * _ysize];

            for (unsigned i = 0; i < _xsize; ++i) {

                unsigned displacement = _ysize * i;

                grid[i] = contiguous + displacement;

                for (unsigned j = 0; j < _ysize; ++j) {

                    contiguous[j + displacement] = value;

                }

            }

            return grid;

        }

        // clear a given 2D grid
        template<typename T>
        void ClearGrid2D(T **grid, T value) {

            for (unsigned i = 0; i < xsize * ysize; ++i) {

                grid[0][i] = value;

            }

        }

        // remove the 2d grid
        template<typename T>
        void RemoveGrid2D(T **grid) {

            if (nullptr != grid) {

                // remove the contiguous array
                delete [] grid[0];

                // remove the pointers
                delete [] grid;

            }

        }

        // update the grid size
        void UpdateGrids(double absx, double absy);

        // iterate the entire cloud and update the min/max/count values
        void FirstPassAnalysis(PointCloudHSV &cloud);

        // iterate the entire cloud and update the points
        void SecondPassAnalysis(PointCloudHSV &cloud);

    public:

        // basic constructor
        SimpleLidarSegmentation();

        // basic destructor
        ~SimpleLidarSegmentation();

        // type segmentation
        void PointTypeSegmentation(PointCloudHSV &cloud, double absx, double absy, double minz, double maxz);

};

}

#endif