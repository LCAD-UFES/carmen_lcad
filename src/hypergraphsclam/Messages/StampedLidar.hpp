#ifndef HYPERGRAPHSLAM_STAMPED_LIDAR_HPP
#define HYPERGRAPHSLAM_STAMPED_LIDAR_HPP

#include <vector>

#include <StampedMessage.hpp>

#include <StringHelper.hpp>
#include <SimpleLidarSegmentation.hpp>

namespace hyper {

    using VoxelGridFilter = pcl::VoxelGrid<pcl::PointXYZHSV>;

    class StampedLidar : virtual public StampedMessage {

        protected:

            static double to_degree;

            double minx, maxx, absx;
            double miny, maxy, absy;
            double minz, maxz, absz;

            // the segmentation class
            static hyper::SimpleLidarSegmentation segm;

            // convert from spherical coordinates
            pcl::PointXYZHSV FromSpherical(double phi, double theta, double radius);

        public:

            // the default leaf size
            static double vg_leaf;

            static VoxelGridFilter grid_filtering;

            // the current speed, for filtering purpose
            double speed;

            // the point cloud filepath
            std::string path;

            // the sequential ICP measure
            g2o::SE2 seq_measurement;

            // the seq id
            unsigned seq_id;

            // the current lidar estimate
            g2o::SE2 lidar_estimate;

            // the current gps sync estimate
            g2o::SE2 gps_sync_estimate;

            // the loop restriction measure
            g2o::SE2 loop_measurement;

            // the loop cloure id
            unsigned loop_closure_id;

            // the external loop restriction measure
            g2o::SE2 external_loop_measurement;

            // the external loop closure id
            unsigned external_loop_closure_id;

            static std::string filepath_prefix;

            // the basic constructor
            StampedLidar(unsigned msg_id, const std::string &base_path);

            // the basic destructor
            virtual ~StampedLidar();

            virtual void LoadPointCloud(PointCloudHSV &cloud) = 0;

            // parse the pose from string stream
            virtual bool FromCarmenLog(std::stringstream &ss) = 0;

            // save the point cloud
            static void SavePointCloud(const std::string &base_path, unsigned cloud_id, const PointCloudHSV &cloud);

            // custom point cloud saving process
            static void SavePointCloud(const std::string &cloud_path, const PointCloudHSV &cloud);

            // custom point cloud loading process
            static void LoadPointCloud(const std::string &cloud_path, PointCloudHSV &cloud);

            // remove undesired points
            void RemoveUndesiredPoints(PointCloudHSV &cloud);

            void RemoveUndesiredPoints(SimpleLidarSegmentation &_segm, PointCloudHSV &cloud);

    };

    // syntactic sugar
    using StampedLidarPtr = StampedLidar*;
    using StampedLidarRef = StampedLidar&;

    // define the standard vector type
    using StampedLidarPtrVector = std::vector<StampedLidarPtr>;

}

#endif
