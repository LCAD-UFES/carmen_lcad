#ifndef LOCAL_GRID_MAP_3D_HPP
#define LOCAL_GRID_MAP_3D_HPP

#include <stdexcept>
#include <cmath>

#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <GridCell.hpp>
#include "../Helpers/Wrap2pi.hpp"

#include <unistd.h>

namespace hyper {

typedef std::vector<pcl::PointXYZHSV, Eigen::aligned_allocator<pcl::PointXYZHSV>> PointXYZHSVVector;

template<typename T, unsigned S = 7>
class LocalGridMap3D {

    private:

        // static value
        static constexpr T base_res = 0.1;

        // max value
        static constexpr T max_value = std::numeric_limits<T>::infinity();

        // the alpha parameter
        static constexpr T alpha = 0.9;

        // how many samples
        static const unsigned samples = S + 1;

        // the max angle sampling distance
        static constexpr T masd = 0.261799;

        // the max displacement sampling distance
        static constexpr T mdsd = 0.05;

        // the occupancy threshold
        static constexpr T occupancy_threshold = 0.9;

        // the actual local map containers
        hyper::GridCellMap3D<T> current_map, next_map;

        // the resolution
        T res;

        // a helper
        T res_2;

        // the inverse resolution
        T inv_res;

        // the x range
        T x_range;

        // the y range
        T y_range;

        // the z range
        T z_range;

        // the grid map width
        unsigned width;

        // the grid map depth
        unsigned depth;

        // the grid map height
        unsigned height;

        // the origin
        hyper::GridCellIndex3D origin;

        // the last transform matrix
        Eigen::Matrix<T, 4, 4> last_transform;

        // the unit x
        Eigen::Matrix<T, 3, 1> unit_x;

        // the unit y
        Eigen::Matrix<T, 3, 1> unit_y;

        // the unit z
        Eigen::Matrix<T, 3, 1> unit_z;

        // how many samples
        T multiplier[S + 1];

        // remove a map from memmory
        void Remove(hyper::GridCellMap3D<T> map) {

            if (nullptr != map) {

                // deallocate the map
                for (unsigned i = 0; i < width; ++i) {

                    for (unsigned j = 0; j < depth; ++j) {

                        // remove the entire layer
                        delete [] map[i][j];

                    }

                    // remove the entire row
                    delete [] map[i];

                }

                // remove all collumns
                delete [] map;

            }

        }

        // update the old local grid map to the vehicle coordinate frame
        void TransformLocalGridMap(const Eigen::Matrix<T, 4, 4> &transform) {

            // iterate over the current map
            for (int i = 0; i < width; ++i) {

                for (int j = 0; j < depth; ++j) {

                    for (int k = 0; k < height; ++k) {

                        // ge te next map direct access
                        hyper::GridCellRef<T> cell(next_map[i][j][k]);

                        // reset the values
                        cell.occupancy = 0.5;
                        cell.nocc = cell.nfree = 0;

                        // Convert to eigen vector
                        Eigen::Matrix<T, 4, 1> pos(T((i - int(origin.x)) * res), T((j - int(origin.y)) * res), T((k - int(origin.z)) * res), 1.0);

                        // rotate it
                        pos = (transform * pos) * inv_res;

                        // get the x, y and z coordinates
                        unsigned x = unsigned(int(origin.x) + int(pos(0)));
                        unsigned y = unsigned(int(origin.y) + int(pos(1)));
                        unsigned z = unsigned(int(origin.z) + int(pos(2)));

                        if (x < width && y < depth && z < height) {

                            // copy the values
                            cell = current_map[x][y][z];

                        }

                    }

                }

            }

            // swap the maps
            hyper::GridCellMap3D<T> tmp_map = next_map;
            next_map = current_map;
            current_map = tmp_map;

        }

        // DDA algorithm 3D version
        void UpdateLine(const pcl::PointXYZHSV &p) {

            // avoids very short beams
            if (res < std::fabs(p.x) || res < std::fabs(p.y) || res < std::fabs(p.z)) {

                T dx = p.x * inv_res;
                T dy = p.y * inv_res;
                T dz = p.z * inv_res;

                // build the target index
                hyper::GridCellIndex3D target;

                // get the correct index
                target.x = unsigned(int(origin.x) + int(dx));
                target.y = unsigned(int(origin.y) + int(dy));
                target.z = unsigned(int(origin.z) + int(dz));

                if (target.x < width && target.y < depth && target.z < height) {

                    // the origin voxel
                    unsigned ox = origin.x;
                    unsigned oy = origin.y;
                    unsigned oz = origin.z;

                    // the displacement
                    int dx = T(p.x) * inv_res;
                    int dy = T(p.y) * inv_res;
                    int dz = T(p.z) * inv_res;

                    // the increments
                    int step_x = 0 > dx ? -1 : 1;
                    int step_y = 0 > dy ? -1 : 1;
                    int step_z = 0 > dz ? -1 : 1;

                    // the required time to cross any cell
                    T time_x, time_y, time_z;

                    // the required time to cross the current cell
                    T tx, ty, tz;

                    if (0 != dx) {

                        // update the time value
                        time_x = 1.0 / std::fabs(dx);

                        // the laser beam starts in the cell's center
                        tx = res_2 * time_x;

                    } else {

                        // set the max value
                        time_x = tx = max_value;

                    }

                    if (0 != dy) {

                        // update the time value
                        time_y = 1.0 / std::fabs(dy);

                        // the laser beam starts in the cell's center
                        ty = res_2 * time_y;

                    } else {

                        // set the max value
                        time_y = ty = max_value;

                    }

                    if (0 != dz) {

                        // update the time value
                        time_z = 1.0 / std::fabs(dz);

                        // the laser beam starts in the cell's center
                        tz = res_2 * time_z;

                    } else {

                        // set the max value
                        time_z = tz = max_value;

                    }

                    // the main loop
                    while (ox != target.x && oy != target.y && oz != target.z) {

                        // get the cell reference
                        hyper::GridCellRef<T> c(current_map[ox][oy][oz]);

                        // increase the free space counter
                        c.nfree = c.nocc = 0;
                        c.occupancy = 0.0;

                        // the alpha
                        //T alpha_n = std::pow(alpha, n);

                        // update the probability
                        //c.occupancy = 0.5 * alpha_n + (1 - alpha_n) * (T(c.nocc) / T(n));
                        // c.occupancy = (T(c.nfree) / T(n));

                        if (tx < ty) {

                            if (tx < tz) {

                                // go to the next cell
                                ox += step_x;

                                // increment tx
                                tx += time_x;

                            } else {

                                // go to the next cell
                                oz += step_z;

                                // increment tz
                                tz += time_z;

                            }

                        } else {

                            if (ty < tz) {

                                // go to the next cell
                                oy += step_y;

                                // increment ty
                                ty += time_y;

                            } else {

                                // go to the next cell
                                oz += step_z;

                                // increment tz
                                tz += time_z;

                            }

                        }

                    }

                    // get the target voxel reference
                    hyper::GridCellRef<T> tc(current_map[target.x][target.y][target.z]);

                    // increase the occupied space counter
                    tc.nocc += 1;

                    // the total
                    unsigned tn = tc.nfree + tc.nocc;

                    // the alpha
                    // T talpha_n = std::pow(alpha, tn);

                    // update the probability
                    tc.occupancy = (T(tc.nocc) / T(tn));// 0.5 * talpha_n + (1 - talpha_n) *(T(tc.nocc) / T(tn));

                }

            }

        }

        // the current map mathching algorithm
        unsigned Match(const pcl::PointCloud<pcl::PointXYZHSV> &cloud, const Eigen::Transform<T, 3, Eigen::Affine> &transform) {

            // the hit counter
            unsigned hits = 0;

            // the new transformed cloud
            pcl::PointCloud<pcl::PointXYZHSV> transformed_cloud;

            // unset the dense flag
            transformed_cloud.is_dense = false;

            // transform the cloud
            pcl::transformPointCloud(cloud, transformed_cloud, transform);

            // get the points direct access
            const PointXYZHSVVector &ps(transformed_cloud.points);

            // helpers
            PointXYZHSVVector::const_iterator it = ps.begin();
            PointXYZHSVVector::const_iterator end = ps.end();

            while (end != it) {

                // the point access
                const pcl::PointXYZHSV &p(*it);

                // avoid very short ranges
                if (res < std::fabs(p.x) || res < std::fabs(p.y) || res < std::fabs(p.z)) {

                    // get the correct index
                    unsigned x = unsigned(int(origin.x) + int(p.x * inv_res));
                    unsigned y = unsigned(int(origin.y) + int(p.y * inv_res));
                    unsigned z = unsigned(int(origin.z) + int(p.z * inv_res));

                    if (x < width && y < depth && z < height && occupancy_threshold <= current_map[x][y][z].occupancy) {

                        ++hits;

                    }


                }

                // go to the next point
                ++it;

            }

            return hits;

        }

        // build an affine transform
        Eigen::Transform<T, 3, Eigen::Affine> GetAffineTransform(T roll, T pitch, T yaw, T dx, T dy, T dz) {

            // the roll matrix
            Eigen::Transform<T, 3, Eigen::Affine> R(Eigen::AngleAxis<T>(roll, unit_x));

            // the pitch matrix
            Eigen::Transform<T, 3, Eigen::Affine> P(Eigen::AngleAxis<T>(pitch, unit_y));

            // the yaw matrix
            Eigen::Transform<T, 3, Eigen::Affine> Y(Eigen::AngleAxis<T>(yaw, unit_z));

            // the translation
            Eigen::Translation<T, 3> translation(Eigen::Matrix<T, 3, 1>(dx, dy, dz));

            // return the desired transform
            return translation * R * P * Y;

        }

        // align a given scan and the local grid map
        Eigen::Matrix<T, 4, 4> Align(const pcl::PointCloud<pcl::PointXYZHSV> &cloud, const Eigen::Matrix<T, 4, 4> &guess) {

            // get the euler angles
            T R = std::atan2(guess(2,1), guess(2, 2));
            T P = std::atan2(-guess(2, 0), std::sqrt(std::pow(guess(2, 1), 2) + std::pow(guess(2, 2), 2)));
            T Y = std::atan2(guess(1, 0), guess(0, 0));

            // the translation values
            T dx = guess(0, 3);
            T dy = guess(1, 3);
            T dz = guess(2, 3);

            // get the best transform
            Eigen::Transform<T, 3, Eigen::Affine> best_transform;
            best_transform.matrix() = guess;

            // the ICP guess transform analysis
            unsigned best_score = Match(cloud, best_transform);

            // sampling
            for (int m = 1; m > -2; m -= 2) {

                for (unsigned i = 0; i < S; ++i) {

                    // get a new dx
                    T ndx = dx + T(m) * multiplier[i] * mdsd;

                    for (unsigned j = 0; j < S; ++j) {

                        // get a new dy
                        T ndy = dy + T(m) * multiplier[j] * mdsd;

                        for (unsigned k = 0; k < S; ++k) {

                            // get a new pitch angle
                            T nP = mrpt::math::wrapToPi<T>(P + T(m) * multiplier[i] * masd);

                            for (unsigned l = 0; l < S; ++l) {

                                // scape the duplicated case
                                // it occurs when all indexes are zero (i, j, k and l are equal to zero)
                                // considering the short circuit
                                if (-1 != m || (0 != l || (0 != k || (0 != j || (0 != i))))) {

                                    // get a new yaw angle
                                    T nY = mrpt::math::wrapToPi<T>(Y + T(m) * multiplier[i] * masd);

                                    // get a new transform
                                    Eigen::Transform<T, 3, Eigen::Affine> transform(GetAffineTransform(R, nP, nY, ndx, ndy, dz));

                                    // map matching algorithm
                                    const unsigned score = Match(cloud, transform);

                                    // is it a better transform?
                                    if (score > best_score) {

                                        // udpate the best score
                                        best_score = score;

                                        // save the new transform
                                        best_transform = transform;

                                    }

                                }

                            }

                        }

                    }

                }

            }

            // default value
            return best_transform.matrix();

        }

    public:

        // custom constructor
        LocalGridMap3D(unsigned res_m, T x_rng, T y_rng, T z_rng) :
            current_map(nullptr),
            next_map(nullptr),
            res(base_res * T(res_m)),
            res_2(res * 0.5),
            inv_res(1.0 / res),
            x_range(x_rng),
            y_range(y_rng),
            z_range(z_rng),
            width(0),
            depth(0),
            height(0),
            origin(),
            last_transform(Eigen::Matrix<T, 4, 4>::Identity()),
            unit_x(Eigen::Matrix<T, 3, 1>::UnitX()),
            unit_y(Eigen::Matrix<T, 3, 1>::UnitY()),
            unit_z(Eigen::Matrix<T, 3, 1>::UnitZ()),
            multiplier()
        {

            if (0.0 == res_m || 0.0 == x_rng || 0.0 == y_rng || 0.0 == z_rng) {

                // error
                throw std::invalid_argument("Check the input arguments");

            }

            // get the grid map width
            width = unsigned(2.0 * x_rng * inv_res);

            if (0 == width % 2) {

                width += 1;

            }

            // get the grid map depth
            depth = unsigned(2.0 * y_rng * inv_res);

            if (0 == depth % 2) {

                depth += 1;

            }

            // get the grid map height
            height = unsigned(2.0 * z_rng * inv_res);

            if (0 == height % 2) {

                height += 1;

            }

            // allocate the old grid map collumns
            unsigned size = width * depth * height;

            // contiguous
            GridCell<T> *local_contiguous_grid = new GridCell<T>[size];

            current_map = new GridCell<T>**[width];

            for (unsigned i = 0; i < width; ++i) {

                // allocate the grid map rows
                current_map[i] = new GridCell<T>*[depth];

                for (unsigned j = 0; j < depth; ++j) {

                    // allocate the grid map layers
                    current_map[i][j] = new GridCell<T>[height];

                }

            }

            // allocate the updated grid map collumns
            next_map = new GridCell<T>**[width];

            for (unsigned i = 0; i < width; ++i) {

                // allocate the grid map rows
                next_map[i] = new GridCell<T>*[depth];

                for (unsigned j = 0; j < depth; ++j) {

                    // allocate the grid map layers
                    next_map[i][j] = new GridCell<T>[height];

                }

            }

            // set the origin
            origin.x = width / 2;
            origin.y = depth / 2;
            origin.z = height / 2;

            // set the default value at the origin
            current_map[origin.x][origin.y][origin.z].occupancy = 0.0;

            // the step increment
            T inc = 1.0 / T(S);

            for (unsigned i = 0; i < samples; ++i) {

                // set the multiplier
                multiplier[i] = T(i) * inc;

            }

        }

        // custom destructor
        ~LocalGridMap3D() {

            // remove the current map
            Remove(current_map);

            // remove the next map
            Remove(next_map);

        }

        // clear the entire grid map
        void Reset() {

            if (nullptr != current_map) {

                for (unsigned i = 0; i < width; ++i) {

                    for (unsigned j = 0; j < depth; ++j) {

                        for (unsigned k = 0; k < height; ++k) {

                            // direct access
                            hyper::GridCellRef<T> c(current_map[i][j][k]);

                            // update the values
                            c.occupancy = 0.5;
                            c.nocc = 0;
                            c.nfree = 0;

                        }

                    }

                }

            }

        }

        // set a custom origin
        void SetOrigin(hyper::GridCellIndex3D o) {

            if (width > o.x && depth > o.y && height > o.z) {

                // update
                origin = o;

            }

        }

        // set the custom sick origin
        void SetSickOrigin() {

            // reset the y coordinate
            origin.y = 0;

        }

        // map update
        void UpdateLocalGridMap(const Eigen::Matrix<T, 4, 4> &transform, pcl::PointCloud<pcl::PointXYZHSV> &cloud) {

            // update the old local grid map to the vehicle coordinate frame
            TransformLocalGridMap(transform);

            // get the point access
            PointXYZHSVVector &ps(cloud.points);

            // helpers
            PointXYZHSVVector::const_iterator it = ps.begin();
            PointXYZHSVVector::const_iterator end = ps.end();

            while (end != it) {

                // a fast voxel traversal algorithm
                UpdateLine(*it);

                // go to the next point
                ++it;

            }

        }

        // map matching
        Eigen::Matrix<T, 4, 4> MapMatching(
                                    const Eigen::Matrix<T, 4, 4> &guess,
                                    pcl::PointCloud<pcl::PointXYZHSV> &source,
                                    pcl::PointCloud<pcl::PointXYZHSV> &target,
                                    bool update)
        {

            if (!update) {

                // loop closure case

                // get the alignment transform matrix
                return Align(target, guess);

            }

            // general lidar odometry case
            // we want to tu update the local grid map
            UpdateLocalGridMap(last_transform, source);

            // map matching
            last_transform = Align(target, guess);

            // return the best parameter
            return last_transform;

        }

        // save local grid map as a point cloud
        void SaveLocalGridMap(const std::string &base_path, unsigned cloud_id) {
            // creates a new RGB point cloud
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

            // set the dense flag
            map_cloud->is_dense = false;

            // the current point
            pcl::PointXYZRGB p;

            //
            for (unsigned i = 0; i < width; ++i) {


                for (unsigned j = 0; j < depth; ++j) {


                    for (unsigned k = 0; k < height; ++k) {

                        // direct access
                        hyper::GridCellRef<T> c(current_map[i][j][k]);

                        // verify the voxel occupancy
                        if (0.8 < c.occupancy) {

                            // se the x coordinate
                            p.x = res * (int(i) - int(origin.x));

                            // se the x coordinate
                            p.y = res * (int(j) - int(origin.y));

                            // set the z coordinate
                            p.z = res * (int(k) - int(origin.z));

                            p.r = 255;

                            p.g = 128;

                            p.b = 64;

                            // save the point
                            map_cloud->push_back(p);

                        }

                    }

                }

            }

            if (0 < map_cloud->size()) {

                // the string stream
                std::stringstream ss;

                // the number
                ss << base_path << "cloud" << cloud_id << ".pcd";

                // save the cloud
                if (-1 == pcl::io::savePCDFile(ss.str(), *map_cloud, true)) {

                    throw std::runtime_error("Could not save the map cloud");

                }

            }

        }

};

}

#endif