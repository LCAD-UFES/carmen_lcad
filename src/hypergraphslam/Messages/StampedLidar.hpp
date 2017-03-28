#ifndef HYPERGRAPHSLAM_STAMPED_LIDAR_HPP
#define HYPERGRAPHSLAM_STAMPED_LIDAR_HPP

#include <vector>

#include <StampedMessage.hpp>

#include <StringHelper.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

namespace hyper {

// syntactic sugar
typedef pcl::PointCloud<pcl::PointXYZHSV> PointCloudHSV;
typedef pcl::VoxelGrid<pcl::PointXYZHSV> VoxelGridFilter;

class StampedLidar : virtual public StampedMessage {

    protected:

        // to degree
        static double to_degree;

        // the default leaf size
        static double vg_leaf;

        // convert from spherical coordinates
        pcl::PointXYZHSV FromSpherical(double phi, double theta, double radius);

    public:

        // the cloud filtering object
        static VoxelGridFilter grid_filtering;

        // the current speed, for filtering purpose
        double speed;

        // the point cloud filepath
        std::string path;

        // the sequential ICP measure
        g2o::SE2 seq_measure;

        // the seq id
        unsigned seq_id;

        // the inverse translation confidence factor
        double itcf;

        // the inverse rotation confidence factor
        double ircf;

        // the loop restriction measure
        g2o::SE2 loop_measure;

        // the loop cloure id
        unsigned loop_closure_id;

        // the basic constructor
        StampedLidar(unsigned msg_id, const std::string &base_path);

        // the basic destructor
        virtual ~StampedLidar();

        // parse the pose from string stream
        virtual bool FromCarmenLog(std::stringstream &ss) =0;

        // save the point cloud
        static void SavePointCloud(const std::string &base_path, unsigned cloud_id, const PointCloudHSV &cloud);

        // custom point cloud saving process
        static void SavePointCloud(const std::string &cloud_path, const PointCloudHSV &cloud);

        // custom point cloud loading process
        static void LoadPointCloud(const std::string &cloud_path, PointCloudHSV &cloud);



};

// syntactic sugar
typedef StampedLidar* StampedLidarPtr;
typedef StampedLidar& StampedLidarRef;

// define the standard vector type
typedef std::vector<StampedLidarPtr> StampedLidarPtrVector;

}

#endif
