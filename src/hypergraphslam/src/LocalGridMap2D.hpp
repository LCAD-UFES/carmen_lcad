#ifndef LOCAL_GRID_MAP_3D_HPP
#define LOCAL_GRID_MAP_3D_HPP

#include <iostream>
#include <stdexcept>
#include <cmath>

#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <GridCell.hpp>

namespace hyper {

template<typename T>
class LocalGridMap2D {

    private:

        // static value
        static constexpr T base_res = 0.1;

        // the local map
        hyper::GridCellMap2D<T> map;

        // the resolution
        T res;

        // the inverse resolution
        T inv_res;

        // the grid map width
        unsigned width;

        // the grid map height
        unsigned height;

        // the origin
        hyper::GridCellIndex2D origin;

        // map update
        void UpdateLocalMap(const Eigen::Matrix3d &transform) {}

    public:

        // custom constructor
        LocalGridMap2D(unsigned res_mult, T x_range, T y_range) : map(nullptr), res(base_res * T(res_mult)), inv_res(1.0/res), origin() {

            if (0.0 == res_mult || 0.0 == x_range || 0.0 == y_range) {

                // error
                throw std::invalid_argument("Check the input arguments");

            }

            // get the grid map width
            width = unsigned(2.0 * x_range * inv_res);

            if (0 == width % 2) {

                width += 1;

            }

            // get the grid map depth
            height = unsigned(2.0 * y_range * inv_res);

            if (0 == height % 2) {

                height += 1;

            }


            // allocate the grid map collumns
            map = new GridCell<T>*[width];

            for (unsigned i = 0; i < width; ++i) {

                // allocate the grid map rows
                map[i] = new GridCell<T>[depth];

            }

            // reset the probabilities
            Reset();

            // set the origin
            // the z value stays at zero
            origin.x = width / 2;
            origin.y = height / 2;

            // show the dimension
            std::cout << "Values: " << width << ", " << height << std::endl;
            std::cout << "Default origin: " << origin.x << ", " << origin.y << std::endl;

        }

        // custom destructor
        ~LocalGridMap2D() {

            if (nullptr != map) {

                // deallocate the map
                for (unsigned i = 0; i < width; ++i) {

                    // remove the entire row
                    delete [] map[i];

                }

                // remove all collumns
                delete [] map;

            }


        }

        // clear the entire grid map
        void Reset() {

            if (nullptr != map) {

                for (unsigned i = 0; i < width; ++i) {

                    for (unsigned j = 0; j < height; ++j) {

                        map[i][j].occupancy = 0.5;

                    }

                }

            }

        }

        // set a custom origin
        void SetOrigin(hyper::GridCellIndex2D o) {

            if (width > o.x && height > o.y) {

                // update
                origin = o;

            }

        }

        // map matching
        g2o::SE2 MapMatching(const Eigen::Matrix3d &guess, const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {

            (void) guess;
            (void) cloud;

            // create the random sampling

            // map matching

            // update map

            // return the best parameter
            return g2o::Se2(0.0, 0.0, 0.0);

        }

};

}

#endif