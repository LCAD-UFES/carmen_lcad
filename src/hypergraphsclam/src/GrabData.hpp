#ifndef HYPERGRAPHSLAM_GRAB_DATA_HPP
#define HYPERGRAPHSLAM_GRAB_DATA_HPP

#include <g2o/types/slam2d/se2.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/types_slam2d.h>

#include <EdgeGPS.hpp>

#include <pcl/registration/gicp.h>

#include <string>
#include <vector>

#include <StampedOdometry.hpp>
#include <StampedXSENS.hpp>
#include <StampedGPSPose.hpp>
#include <StampedGPSOrientation.hpp>
#include <StampedSICK.hpp>
#include <StampedVelodyne.hpp>
#include <VehicleModel.hpp>
#include <LocalGridMap3D.hpp>
#include <StringHelper.hpp>
#include <Wrap2pi.hpp>

#include <carmen/carmen.h>

#include <mutex>

namespace hyper {

#define MINIMUM_VEL_SCANS 0
#define GPS_FILTER_THRESHOLD 40.0
#define LOOP_REQUIRED_TIME 300.0
#define LOOP_REQUIRED_SQR_DISTANCE 16.0
#define CORRESPONDENCE_FACTOR 2.0
#define ICP_THREADS_POOL_SIZE 8
#define ICP_THREAD_BLOCK_SIZE 100
#define LIDAR_ODOMETRY_MIN_DISTANCE 0.3
#define ICP_TRANSLATION_CONFIDENCE_FACTOR 1.00

// define the gicp
typedef pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZHSV, pcl::PointXYZHSV> GeneralizedICP;

class GrabData {

    private:

        // the raw input message list
        StampedMessagePtrVector raw_messages;

        // the main and general message queue
        StampedMessagePtrVector messages;

        // a gps list to help the filtering process
        StampedGPSPosePtrVector gps_messages;

        // a velodyne list to help the ICP process
        StampedLidarPtrVector velodyne_messages;

        // a SICK list to help the ICP process
        StampedLidarPtrVector sick_messages;

        // the current cloud to be processed
        StampedLidarPtrVector *point_cloud_lidar_messages;

        // an Odometry list
        StampedOdometryPtrVector odometry_messages;

        // the gps origin
        Eigen::Vector2d gps_origin;

        // the current lidar message iterator index to be used in the ICP measure methods
        unsigned icp_start_index, icp_end_index;

        // mutex to avoid racing conditions
        std::mutex icp_mutex;

        // helper
        double dmax;

        // separate the gps, sick and velodyne messages
        void SeparateMessages();

        // get the gps estimation
        void GetNearestOrientation(
                    StampedMessagePtrVector::iterator it,
                    StampedMessagePtrVector::iterator end,
                    int adv,
                    double timestamp,
                    double &h,
                    double &dt);

        // get the gps full measure
        Eigen::Rotation2Dd GetGPSRotation(
                    StampedMessagePtrVector::iterator begin,
                    StampedMessagePtrVector::iterator it,
                    StampedMessagePtrVector::iterator end,
                    double timestamp);

        // get the first gps position
        Eigen::Vector2d GetFirstGPSPosition();

        // find the nearest orientation
        double FindGPSOrientation(StampedMessagePtrVector::iterator gps);

        // filter the entire gps positions
        void GPSFiltering();

        // iterate over the entire message list and build the measures and estimates
        void BuildGPSMeasures();

        // iterate over the entire message list and build the measures and estimates
        void BuildOdometryMeasures();

        // build the initial estimates
        void BuildOdometryEstimates();

        // build an icp measure
        bool BuildLidarOdometryMeasure(
                GeneralizedICP &gicp,
                VoxelGridFilter &grid_filtering,
                double cf,
                const g2o::SE2 &odom,
                PointCloudHSV::Ptr source_cloud,
                PointCloudHSV::Ptr target_cloud,
                g2o::SE2 &icp_measure);

        // build an icp measure
        bool BuildLidarLoopMeasure(
                GeneralizedICP &gicp,
                double cf,
                PointCloudHSV::Ptr source_cloud,
                PointCloudHSV::Ptr target_cloud,
                g2o::SE2 &loop_measure);

        // get the next lidar block
        bool GetNextLidarBlock(unsigned &first_index, unsigned &last_index);

        // it results in a safe region
        bool GetNextICPIterators(StampedLidarPtrVector::iterator &begin, StampedLidarPtrVector::iterator &end);

        // the main icp measure method, multithreading version
        void BuildLidarMeasuresMT();

        // build sequential and loop restriction ICP measures
        void BuildLidarOdometryMeasuresWithThreads(StampedLidarPtrVector &lidar_messages);

        // build the lidar odometry estimates,
        // we should call this method after the BuildOdometryEstimates
        void BuildLidarOdometryEstimates(StampedLidarPtrVector &lidar_messages);

        // compute the loop closure measure
        void BuildLidarLoopClosureMeasures(StampedLidarPtrVector &lidar_messages);

        // save all vertices to the external file
        void SaveAllVertices(std::ofstream &os);

        // save the odometry edges
        void SaveOdometryEdges(std::ofstream &os);

        // save the current odometry estimates to odom.txt file
        void SaveOdometryEstimates();

        // save the gps edges
        void SaveGPSEdges(std::ofstream &os);

        // save the gps edges
        void SaveGPSEstimates();

        // save icp edges
        void SaveLidarEdges(const std::string &msg_name, std::ofstream &os, const StampedLidarPtrVector &lidar_messages);

        // save icp edges
        void SaveICPEdges(std::ofstream &os);

        // save the lidar estimates
        void SaveLidarEstimates(const std::string &filename, const StampedLidarPtrVector &lidar_messages);

        // save the curvature constraint edges
        void SaveCurvatureEdges(std::ofstream &os);

        // build Eigen homogeneous matrix from g2o SE2
        Eigen::Matrix4f BuildMatrixFromSE2(const g2o::SE2 &transform);

        // get SE2 transform from Eigen homogeneous coordinate matrix
        g2o::SE2 BuildSE2FromMatrix(const Eigen::Matrix4f &matrix);

        // removing the copy constructor
        GrabData(const GrabData&) = delete;

        // removing the assignment operator overloading
        void operator=(const GrabData&);

    public:

        // the main constructor
        GrabData();

        // the main destructor
        ~GrabData();

        // parse the log file
        bool ParseLogFile(const std::string &input_filename);

        // sync all messages and process each one
        // it builds the all estimates and measures
        // and constructs the hypergraph
        void BuildHyperGraph();

        // save the hyper graph to the output file
        void SaveHyperGraph(const std::string &output_filename);

        // clear the entire object
        void Clear();

};

}

#endif
