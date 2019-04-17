#ifndef HYPERGRAPHSLAM_GRAB_DATA_HPP
#define HYPERGRAPHSLAM_GRAB_DATA_HPP

#include <string>
#include <vector>
#include <mutex>

#include <g2o/types/slam2d/se2.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/types_slam2d.h>

#include <pcl/registration/gicp.h>

#include <StampedOdometry.hpp>
#include <StampedXSENS.hpp>
#include <StampedGPSPose.hpp>
#include <StampedGPSOrientation.hpp>
#include <StampedSICK.hpp>
#include <StampedVelodyne.hpp>
#include <StampedBumblebee.hpp>
#include <EdgeGPS.hpp>

#include <VehicleModel.hpp>
#include <LocalGridMap3D.hpp>
#include <StringHelper.hpp>
#include <Wrap2pi.hpp>

#include <matrix.h>

#include <carmen/carmen.h>

namespace hyper {

#define MAXIMUM_VEL_SCANS 0
#define LOOP_REQUIRED_TIME 300.0
#define LOOP_REQUIRED_SQR_DISTANCE 25.0
#define ICP_THREADS_POOL_SIZE 3
#define ICP_THREAD_BLOCK_SIZE 400
#define LIDAR_ODOMETRY_MIN_DISTANCE 0.3
#define VISUAL_ODOMETRY_MIN_DISTANCE 0.1
#define ICP_TRANSLATION_CONFIDENCE_FACTOR 1.00
#define CURVATURE_REQUIRED_TIME 0.0001

    // define the gicp
    typedef pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZHSV, pcl::PointXYZHSV> GeneralizedICP;

    class GrabData
    {
        private:

            // the raw input message list
            StampedMessagePtrVector raw_messages;

            // the main and general message queue
            StampedMessagePtrVector messages;

            // a gps list to help the filtering process
            StampedGPSPosePtrVector gps_messages;

            // the xsens messages
            StampedXSENSPtrVector xsens_messages;

            // a velodyne list to help the ICP process
            StampedLidarPtrVector velodyne_messages;

            // a velodyne list to help the ICP process
            StampedLidarPtrVector used_velodyne;

            // a SICK list to help the ICP process
            StampedLidarPtrVector sick_messages;

            // the used sick messages
            StampedLidarPtrVector used_sick;

            // the current cloud to be processed
            StampedLidarPtrVector *point_cloud_lidar_messages;

            // an Odometry list
            StampedOdometryPtrVector odometry_messages;

            // the bumblebee messages
            StampedBumblebeePtrVector bumblebee_messages;

            // used bumblebee messages
            StampedBumblebeePtrVector used_frames;

            // the gps origin
            Eigen::Vector2d gps_origin;

            // the current lidar message iterator index to be used in the ICP measure methods
            unsigned icp_start_index, icp_end_index;

            // mutex to avoid racing conditions
            std::mutex icp_mutex, first_last_mutex, error_increment_mutex;

            // the icp error counter
            unsigned icp_errors;

            // helper
            double dmax;

            // parameters
            unsigned maximum_vel_scans;
            double loop_required_time;
            double loop_required_sqr_distance;
            unsigned icp_threads_pool_size;
            unsigned icp_thread_block_size;
            double lidar_odometry_min_distance;
            double visual_odometry_min_distance;
            double icp_translation_confidence_factor;
            bool save_accumulated_point_clouds;

            bool use_velodyne_odometry;
            bool use_sick_odometry;
            bool use_bumblebee_odometry;
            
            bool use_velodyne_loop;
            bool use_sick_loop;
            bool use_bumblebee_loop;

            // separate the gps, sick and velodyne messages
            void SeparateMessages();

            // get the gps antena position in relation to sensor board
            void SetGPSPose(std::string carmen_home);

            // get the gps estimation
            g2o::SE2 GetNearestGPSMeasure(
                        StampedMessagePtrVector::iterator it,
                        StampedMessagePtrVector::iterator end,
                        int adv,
                        double timestamp,
                        double &dt);

            // get the gps estimation
            g2o::SE2 GetGPSMeasure(
                        StampedMessagePtrVector::iterator begin,
                        StampedMessagePtrVector::iterator gps,
                        StampedMessagePtrVector::iterator end,
                        double timestamp);

            // get the gps estimation
            void GetNearestOrientation(
                        StampedMessagePtrVector::iterator it,
                        StampedMessagePtrVector::iterator end,
                        int adv,
                        double timestamp,
                        double &h,
                        double &dt);

            // get the gps full measure
            Eigen::Rotation2Dd GetGPSOrientation(
                        StampedMessagePtrVector::iterator begin,
                        StampedMessagePtrVector::iterator it,
                        StampedMessagePtrVector::iterator end,
                        double timestamp);

            // get the first gps position
            Eigen::Vector2d GetFirstGPSPosition();

            // find the nearest orientation
            double FindGPSOrientation(StampedMessagePtrVector::iterator gps);

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

            // remove the unused lidar messages
            void LidarMessagesFiltering(StampedLidarPtrVector &lidar_messages, StampedLidarPtrVector &used_lidar);

            // build the gps sync lidar estimates
            void BuildLidarOdometryGPSEstimates();

            // build the lidar odometry estimates,
            // we should call this method after the BuildOdometryEstimates
            void BuildRawLidarOdometryEstimates(StampedLidarPtrVector &lidar_messages, StampedLidarPtrVector &used_lidar);

            // build the visual odometry estimates, we should call this method after the BuildOdometryEstimates
            void BuildVisualOdometryEstimates();

            // compute the loop closure measure
            void BuildLidarLoopClosureMeasures(StampedLidarPtrVector &lidar_messages);

            // compute the bumblebee measure
            void BuildVisualOdometryMeasures();

            // save all vertices to the external file
            void SaveAllVertices(std::ofstream &os);

            // save the odometry edges
            void SaveOdometryEdges(std::ofstream &os);

            // save the current odometry estimates to odom.txt file
            void SaveOdometryEstimates(const std::string &output_filename, bool raw_version = false);

            // save the gps edges
            void SaveGPSEdges(std::ofstream &os);

            // save the xsens edges
            void SaveXSENSEdges(std::ofstream &os);

            // save the gps edges
            void SaveGPSEstimates();

            // save icp edges
            void SaveLidarEdges(const std::string &msg_name, std::ofstream &os, const StampedLidarPtrVector &lidar_messages, bool use_lidar_odometry, bool use_lidar_loop);

            // save visual odometry edges
            void SaveVisualOdometryEdges(std::ofstream &os);

            // save icp edges
            void SaveICPEdges(std::ofstream &os);

            // save the lidar estimates
            void SaveRawLidarEstimates(const std::string &filename, const StampedLidarPtrVector &lidar_messages);

            // save the visual odometry estimates
            void SaveVisualOdometryEstimates();

            // build Eigen homogeneous matrix from g2o SE2
            Eigen::Matrix4f BuildEigenMatrixFromSE2(const g2o::SE2 &transform);

            // get SE2 transform from Eigen homogeneous coordinate matrix
            g2o::SE2 GetSE2FromEigenMatrix(const Eigen::Matrix4f &matrix);

            // get SE2 transform from libviso homogeneous coordinate matrix
            g2o::SE2 GetSE2FromVisoMatrix(const Matrix &matrix);

            // removing the copy constructor
            GrabData(const GrabData&) = delete;

            // removing the assignment operator overloading
            void operator=(const GrabData&);

        public:

            // the main constructor
            GrabData();

            // the main destructor
            ~GrabData();

            // configuration
            void Configure(std::string config_filename, std::string carmen_home);

            // parse the log file
            bool ParseLogFile(const std::string &input_filename);

            // sync all messages and process each one
            // it builds the all estimates and measures
            // and constructs the hypergraph
            void BuildHyperGraph();

            // save the hyper graph to the output file
            void SaveHyperGraph(const std::string &output_filename);

            // save the estimates to external files
            void SaveEstimates();

            // clear the entire object
            void Clear();

    };

}

#endif
