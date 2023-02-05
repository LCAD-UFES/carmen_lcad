#include <fstream>
#include <math.h>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <optional>
#include <vector>

#include <carmen/carmen.h>
#include <carmen/velodyne_interface.h>
#include <carmen/gps_nmea_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/carmen_gps.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/crop_box.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <eigen3/Eigen/Dense>

#include <ceres/ceres.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "common.h"
#include "tic_toc.h"

#include "Scancontext.h"
#include "scanRegistration.h"
#include "laserOdometry.h"
#include "laserMapping.h"

using namespace gtsam;

using std::cout;
using std::endl;

double keyframeMeterGap;
double keyframeDegGap, keyframeRadGap;
double translationAccumulated = 1000000.0; // large value means must add the first given frame.
double rotaionAccumulated = 1000000.0; // large value means must add the first given frame.

bool isNowKeyFrame = false;

Pose6D odom_pose_prev {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init
Pose6D odom_pose_curr {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init pose is zero

std::queue<carmen_xsens_global_quat_message *> odometryBuf;
std::queue<carmen_velodyne_partial_scan_message *> fullResBuf;
std::queue<carmen_gps_xyz_message *> gpsBuf;
std::queue<std::pair<int, int> > scLoopICPBuf;

std::mutex mBuf;
std::mutex mKF;

double timeLaserOdometry = 0.0;
double timeLaser = 0.0;

pcl::PointCloud<PointType>::Ptr laserCloudFullRes_2(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudMapAfterPGO(new pcl::PointCloud<PointType>());

std::vector<pcl::PointCloud<PointType>::Ptr> keyframeLaserClouds;
std::vector<Pose6D> keyframePoses;
std::vector<Pose6D> keyframePosesUpdated;
std::vector<double> keyframeTimes;
int recentIdxUpdated = 0;

gtsam::NonlinearFactorGraph gtSAMgraph;
bool gtSAMgraphMade = false;
gtsam::Values initialEstimate;
gtsam::ISAM2 *isam;
gtsam::Values isamCurrentEstimate;

noiseModel::Diagonal::shared_ptr priorNoise;
noiseModel::Diagonal::shared_ptr odomNoise;
noiseModel::Base::shared_ptr robustLoopNoise;
noiseModel::Base::shared_ptr robustGPSNoise;

pcl::VoxelGrid<PointType> downSizeFilterScancontext;
SCManager scManager;
double scDistThres, scMaximumRadius;

pcl::VoxelGrid<PointType> downSizeFilterICP;
std::mutex mtxICP;
std::mutex mtxPosegraph;
std::mutex mtxRecentPose;

pcl::PointCloud<PointType>::Ptr laserCloudMapPGO(new pcl::PointCloud<PointType>());
pcl::VoxelGrid<PointType> downSizeFilterMapPGO;
bool laserCloudMapPGORedraw = true;

bool useGPS = true;
// bool useGPS = false;
carmen_gps_xyz_message *currGPS;
bool hasGPSforThisKF = false;
bool gpsOffsetInitialized = false;
double gpsAltitudeInitOffset = 0.0;
double recentOptimizedX = 0.0;
double recentOptimizedY = 0.0;

/*ros::Publisher pubMapAftPGO, pubOdomAftPGO, pubPathAftPGO;
ros::Publisher pubLoopScanLocal, pubLoopSubmapLocal;
ros::Publisher pubOdomRepubVerifier;*/

std::string save_directory;
std::string pgKITTIformat, pgScansDirectory, pgSCDsDirectory;
std::string odomKITTIformat;
std::fstream pgG2oSaveStream, pgTimeSaveStream;

std::vector<std::string> edges_str; // used in writeEdge



std::string
padZeros(int val, int num_digits = 6)
{
  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
  return out.str();
}

std::string
getVertexStr(const int _node_idx, const gtsam::Pose3& _Pose)
{
    gtsam::Point3 t = _Pose.translation();
    gtsam::Rot3 R = _Pose.rotation();

    std::string curVertexInfo {
        "VERTEX_SE3:QUAT " + std::to_string(_node_idx) + " "
        + std::to_string(t.x()) + " " + std::to_string(t.y()) + " " + std::to_string(t.z())  + " "
        + std::to_string(R.toQuaternion().x()) + " " + std::to_string(R.toQuaternion().y()) + " "
        + std::to_string(R.toQuaternion().z()) + " " + std::to_string(R.toQuaternion().w()) };

    // pgVertexSaveStream << curVertexInfo << std::endl;
    // vertices_str.emplace_back(curVertexInfo);
    return curVertexInfo;
}

void
writeEdge(const std::pair<int, int> _node_idx_pair, const gtsam::Pose3& _relPose, std::vector<std::string>& edges_str)
{
    gtsam::Point3 t = _relPose.translation();
    gtsam::Rot3 R = _relPose.rotation();

    std::string curEdgeInfo {
        "EDGE_SE3:QUAT " + std::to_string(_node_idx_pair.first) + " " + std::to_string(_node_idx_pair.second) + " "
        + std::to_string(t.x()) + " " + std::to_string(t.y()) + " " + std::to_string(t.z())  + " "
        + std::to_string(R.toQuaternion().x()) + " " + std::to_string(R.toQuaternion().y()) + " "
        + std::to_string(R.toQuaternion().z()) + " " + std::to_string(R.toQuaternion().w()) };

    // pgEdgeSaveStream << curEdgeInfo << std::endl;
    edges_str.emplace_back(curEdgeInfo);
}

void
saveSCD(std::string fileName, Eigen::MatrixXd matrix, std::string delimiter = " ")
{
    // delimiter: ", " or " " etc.

    int precision = 3; // or Eigen::FullPrecision, but SCD does not require such accruate precisions so 3 is enough.
    const static Eigen::IOFormat the_format(precision, Eigen::DontAlignCols, delimiter, "\n");

    std::ofstream file(fileName);
    if (file.is_open())
    {
        file << matrix.format(the_format);
        file.close();
    }
}

gtsam::Pose3
Pose6DtoGTSAMPose3(const Pose6D& p)
{
    return gtsam::Pose3( gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z) );
} // Pose6DtoGTSAMPose3

void
saveGTSAMgraphG2oFormat(const gtsam::Values& _estimates)
{
    // save pose graph (runs when programe is closing)
    // cout << "****************************************************" << endl;
    cout << "Saving the posegraph ..." << endl; // giseop

    pgG2oSaveStream = std::fstream(save_directory + "singlesession_posegraph.g2o", std::fstream::out);

    int pose_idx = 0;
    for(const auto& _pose6d: keyframePoses)
    {
        gtsam::Pose3 pose = Pose6DtoGTSAMPose3(_pose6d);
        pgG2oSaveStream << getVertexStr(pose_idx, pose) << endl;
        pose_idx++;
    }
    for(auto& _line: edges_str)
        pgG2oSaveStream << _line << std::endl;

    pgG2oSaveStream.close();
}

void
saveOdometryVerticesKITTIformat(std::string _filename)
{
    // ref from gtsam's original code "dataset.cpp"
    std::fstream stream(_filename.c_str(), std::fstream::out);
    for(const auto& _pose6d: keyframePoses)
    {
        gtsam::Pose3 pose = Pose6DtoGTSAMPose3(_pose6d);
        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3
        //std::fixed;

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << std::fixed << t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
    }
}

void
saveOptimizedVerticesKITTIformat(gtsam::Values _estimates, std::string _filename)
{
    using namespace gtsam;

    // ref from gtsam's original code "dataset.cpp"
    std::fstream stream(_filename.c_str(), std::fstream::out);

    for(const auto& key_value: _estimates)
    {
        auto p = dynamic_cast<const GenericValue<Pose3>*>(&key_value.value);
        if (!p) continue;

        const Pose3& pose = p->value();

        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3
        //std::fixed;
        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " <<  std::fixed <<t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
    }
}

/*
void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &_laserOdometry)
{
	mBuf.lock();
	odometryBuf.push(_laserOdometry);
	mBuf.unlock();
} // laserOdometryHandler

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &_laserCloudFullRes)
{
	mBuf.lock();
	fullResBuf.push(_laserCloudFullRes);
	mBuf.unlock();
} // laserCloudFullResHandler

void gpsHandler(const sensor_msgs::NavSatFix::ConstPtr &_gps)
{
    if(useGPS) {
        mBuf.lock();
        gpsBuf.push(_gps);
        mBuf.unlock();
    }
} // gpsHandler
*/

void
arrange_velodyne_vertical_angles_to_true_position(carmen_velodyne_partial_scan_message *velodyne_message)
{
	const int column_correspondence[32] =
	{
		0, 16, 1, 17, 2, 18, 3, 19, 4, 20, 5, 21, 6, 22, 7, 23, 8,
		24, 9, 25, 10, 26, 11, 27, 12, 28, 13, 29, 14, 30, 15, 31
	};

	int i, j;
	unsigned short original_distances[32];
	unsigned char original_intensities[32];

	for (i = 0; i < velodyne_message->number_of_32_laser_shots; i++)
	{
		memcpy(original_distances, velodyne_message->partial_scan[i].distance, 32 * sizeof(unsigned short));
		memcpy(original_intensities, velodyne_message->partial_scan[i].intensity, 32 * sizeof(unsigned char));

		for (j = 0; j < 32; j++)
		{
			velodyne_message->partial_scan[i].distance[column_correspondence[j]] = original_distances[j];
			velodyne_message->partial_scan[i].intensity[column_correspondence[j]] = original_intensities[j];
		}
	}
}



void
velodyne_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
	fullResBuf.push(velodyne_message);
}

void
xsens_matrix_handler(carmen_xsens_global_quat_message *msg)
{
	odometryBuf.push(msg);

}

void
carmen_gps_gpgga_message_handler(carmen_gps_gpgga_message *gps_gpgga)
{
	double latitude = 0;
	double longitude = 0;
	carmen_gps_xyz_message *gps_xyz_message;
	gps_xyz_message =(carmen_gps_xyz_message *) malloc (sizeof (carmen_gps_xyz_message));
	gps_xyz_message->nr = gps_gpgga->nr;
	gps_xyz_message->utc = gps_gpgga->utc;
	gps_xyz_message->latitude = gps_gpgga->latitude;
	gps_xyz_message->latitude_dm = gps_gpgga->latitude_dm;
	gps_xyz_message->lat_orient = gps_gpgga->lat_orient;
	gps_xyz_message->longitude = gps_gpgga->longitude;
	gps_xyz_message->longitude_dm = gps_gpgga->longitude_dm;
	gps_xyz_message->long_orient = gps_gpgga->long_orient;
	gps_xyz_message->gps_quality = gps_gpgga->gps_quality;
	gps_xyz_message->num_satellites = gps_gpgga->num_satellites;
	gps_xyz_message->hdop = gps_gpgga->hdop;
	gps_xyz_message->sea_level = gps_gpgga->sea_level;
	gps_xyz_message->altitude = gps_gpgga->altitude;
	gps_xyz_message->geo_sea_level = gps_gpgga->geo_sea_level;
	gps_xyz_message->geo_sep = gps_gpgga->geo_sep;
	gps_xyz_message->data_age = gps_gpgga->data_age;

	if (gps_gpgga->lat_orient == 'S') latitude = -gps_gpgga->latitude;
	if (gps_gpgga->long_orient == 'W') longitude = -gps_gpgga->longitude;
// Transformando o z utilizando como altitude o sea_level
	Gdc_Coord_3d gdc = Gdc_Coord_3d(latitude, longitude, gps_gpgga->sea_level);


// Transformando o z utilizando como altitude a altitude mesmo - que esta vindo como zero
	//Gdc_Coord_3d gdc = Gdc_Coord_3d(latitude, longitude, gps_gpgga->altitude);
	Utm_Coord_3d utm;
	Gdc_To_Utm_Converter::Init();
	Gdc_To_Utm_Converter::Convert(gdc,utm);

	//gps_xyz_message.x = utm.x;
	//gps_xyz_message.y = utm.y;
	//gps_xyz_message.z = utm.z;

	gps_xyz_message->x = utm.y;
	gps_xyz_message->y = -utm.x;

	gps_xyz_message->z = 0.0;
	gps_xyz_message->zone = (double)utm.zone;

	// printf("%lf %lf -> %lf %lf %lf\n", latitude, longitude, gps_xyz_message.x, gps_xyz_message.y, gps_xyz_message.z);

	if (utm.hemisphere_north == true)
		gps_xyz_message->hemisphere_north = 1;
	else
		gps_xyz_message->hemisphere_north = 0;

	gps_xyz_message->timestamp = gps_gpgga->timestamp;
	gps_xyz_message->host = carmen_get_host();
	gpsBuf.push(gps_xyz_message);
}

void
subscribe_messages()
{

	carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t) velodyne_handler, CARMEN_SUBSCRIBE_LATEST);
	//carmen_xsens_subscribe_xsens_global_quat_message(NULL, (carmen_handler_t) xsens_matrix_handler, CARMEN_SUBSCRIBE_ALL);
	//carmen_gps_subscribe_nmea_message(NULL, (carmen_handler_t) carmen_gps_gpgga_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


void
initNoises( void )
{
    gtsam::Vector priorNoiseVector6(6);
    priorNoiseVector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
    priorNoise = noiseModel::Diagonal::Variances(priorNoiseVector6);

    gtsam::Vector odomNoiseVector6(6);
    // odomNoiseVector6 << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
    odomNoiseVector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
    odomNoise = noiseModel::Diagonal::Variances(odomNoiseVector6);

    double loopNoiseScore = 0.5; // constant is ok...
    gtsam::Vector robustNoiseVector6(6); // gtsam::Pose3 factor has 6 elements (6D)
    robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore;
    robustLoopNoise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6) );

    double bigNoiseTolerentToXY = 1000000000.0; // 1e9
    double gpsAltitudeNoiseScore = 250.0; // if height is misaligned after loop clsosing, use this value bigger
    gtsam::Vector robustNoiseVector3(3); // gps factor has 3 elements (xyz)
    robustNoiseVector3 << bigNoiseTolerentToXY, bigNoiseTolerentToXY, gpsAltitudeNoiseScore; // means only caring altitude here. (because LOAM-like-methods tends to be asymptotically flyging)
    robustGPSNoise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector3) );

} // initNoises

Pose6D
getOdom(std::pair<carmen_vector_3D_t, carmen_quaternion_t> pair_final)
{
    auto tx = pair_final.first.x;
    auto ty = pair_final.first.y;
    auto tz = pair_final.first.z;

    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(pair_final.second.q0,pair_final.second.q1, pair_final.second.q2, pair_final.second.q3)).getRPY(roll, pitch, yaw);

    return Pose6D{tx, ty, tz, roll, pitch, yaw};
} // getOdom

Pose6D
diffTransformation(const Pose6D& _p1, const Pose6D& _p2)
{
    Eigen::Affine3f SE3_p1 = pcl::getTransformation(_p1.x, _p1.y, _p1.z, _p1.roll, _p1.pitch, _p1.yaw);
    Eigen::Affine3f SE3_p2 = pcl::getTransformation(_p2.x, _p2.y, _p2.z, _p2.roll, _p2.pitch, _p2.yaw);
    Eigen::Matrix4f SE3_delta0 = SE3_p1.matrix().inverse() * SE3_p2.matrix();
    Eigen::Affine3f SE3_delta; SE3_delta.matrix() = SE3_delta0;
    float dx, dy, dz, droll, dpitch, dyaw;
    pcl::getTranslationAndEulerAngles (SE3_delta, dx, dy, dz, droll, dpitch, dyaw);
    // std::cout << "delta : " << dx << ", " << dy << ", " << dz << ", " << droll << ", " << dpitch << ", " << dyaw << std::endl;

    return Pose6D{double(abs(dx)), double(abs(dy)), double(abs(dz)), double(abs(droll)), double(abs(dpitch)), double(abs(dyaw))};
} // SE3Diff

pcl::PointCloud<PointType>::Ptr
local2global(const pcl::PointCloud<PointType>::Ptr &cloudIn, const Pose6D& tf)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);

    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }

    return cloudOut;
}

void
publish_globalpos(double x, double y, double theta, double timestamp)
{
	IPC_RETURN_TYPE err;
	carmen_localize_ackerman_globalpos_message globalpos;

	globalpos.globalpos.x = x;
	globalpos.globalpos.y = y;
	globalpos.globalpos.theta = theta;

	globalpos.pose.position.x = globalpos.globalpos.x;
	globalpos.pose.position.y = globalpos.globalpos.y;
	globalpos.pose.position.z = 0.0;
	globalpos.pose.orientation.yaw = globalpos.globalpos.theta;
	globalpos.pose.orientation.pitch = globalpos.pose.orientation.roll = 0.0;

	globalpos.v = 0.0;
	globalpos.phi = 0.0;

	globalpos.velocity.x = globalpos.velocity.y = globalpos.velocity.z = 0.0;

	globalpos.globalpos_std.x = 0.001;
	globalpos.globalpos_std.y = 0.001;
	globalpos.globalpos_std.theta = 0.001;
	globalpos.globalpos_xy_cov = 0.001;

	globalpos.odometrypos.x = x;
	globalpos.odometrypos.y = y;
	globalpos.odometrypos.theta = theta;

	globalpos.converged = 1;

	globalpos.semi_trailer_type = 0;
	globalpos.semi_trailer_engaged = 0;

	globalpos.timestamp = timestamp;
	globalpos.host = carmen_get_host();

	err = IPC_publishData(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, &globalpos);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME);
}


void
pubPath( void )
{
    // pub odom and path
    /*nav_msgs::Odometry odomAftPGO;
    nav_msgs::Path pathAftPGO;
    pathAftPGO.header.frame_id = "/camera_init";*/
	carmen_localize_ackerman_globalpos_message msg_global_pos;
    mKF.lock();
    tf::Quaternion q_aux;
    Pose6D pose_est;
    // for (int node_idx=0; node_idx < int(keyframePosesUpdated.size()) - 1; node_idx++) // -1 is just delayed visualization (because sometimes mutexed while adding(push_back) a new one)
    for (int node_idx=0; node_idx < recentIdxUpdated; node_idx++) // -1 is just delayed visualization (because sometimes mutexed while adding(push_back) a new one)
    {
        pose_est = keyframePosesUpdated.at(node_idx); // upodated poses
		std::cout << "X " << pose_est.x << std::endl << "Y " << pose_est.y <<std::endl;
        q_aux = tf::createQuaternionFromRPY(pose_est.roll, pose_est.pitch, pose_est.yaw);
        publish_globalpos(pose_est.x, pose_est.y, pose_est.yaw, carmen_get_time());
	}
    mKF.unlock();

} // pubPath


void
updatePoses(void)
{
    mKF.lock();
    for (int node_idx=0; node_idx < int(isamCurrentEstimate.size()); node_idx++)
    {
        Pose6D& p =keyframePosesUpdated[node_idx];
        p.x = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().x();
        p.y = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().y();
        p.z = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().z();
        p.roll = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().roll();
        p.pitch = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().pitch();
        p.yaw = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().yaw();
    }
    mKF.unlock();

    mtxRecentPose.lock();
    const gtsam::Pose3& lastOptimizedPose = isamCurrentEstimate.at<gtsam::Pose3>(int(isamCurrentEstimate.size())-1);
    recentOptimizedX = lastOptimizedPose.translation().x();
    recentOptimizedY = lastOptimizedPose.translation().y();

    recentIdxUpdated = int(keyframePosesUpdated.size()) - 1;

    mtxRecentPose.unlock();
} // updatePoses

void
runISAM2opt(void)
{
    // called when a variable added
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();

    gtSAMgraph.resize(0);
    initialEstimate.clear();

    isamCurrentEstimate = isam->calculateEstimate();
    updatePoses();
}

pcl::PointCloud<PointType>::Ptr
transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, gtsam::Pose3 transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    PointType *pointFrom;

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(
                                    transformIn.translation().x(), transformIn.translation().y(), transformIn.translation().z(),
                                    transformIn.rotation().roll(), transformIn.rotation().pitch(), transformIn.rotation().yaw() );

    //int numberOfCores = 8; // TODO move to yaml
    for (int i = 0; i < cloudSize; ++i)
    {
        pointFrom = &cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom->x + transCur(0,1) * pointFrom->y + transCur(0,2) * pointFrom->z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom->x + transCur(1,1) * pointFrom->y + transCur(1,2) * pointFrom->z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom->x + transCur(2,1) * pointFrom->y + transCur(2,2) * pointFrom->z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom->intensity;
    }
    return cloudOut;
} // transformPointCloud

void
loopFindNearKeyframesCloud( pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& submap_size, const int& root_idx)
{
    // extract and stacking near keyframes (in global coord)
    nearKeyframes->clear();
    for (int i = -submap_size; i <= submap_size; ++i)
    {
        int keyNear = key + i; // see https://github.com/gisbi-kim/SC-A-LOAM/issues/7 ack. @QiMingZhenFan found the error and modified as below.
        if (keyNear < 0 || keyNear >= int(keyframeLaserClouds.size()) )
            continue;

        mKF.lock();
        *nearKeyframes += * local2global(keyframeLaserClouds[keyNear], keyframePosesUpdated[root_idx]);
        mKF.unlock();
    }

    if (nearKeyframes->empty())
        return;

    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
} // loopFindNearKeyframesCloud


void
doICPVirtualRelative( int _loop_kf_idx, int _curr_kf_idx, gtsam::Pose3 *pose_gt )
{
    // parse pointclouds
    int historyKeyframeSearchNum = 25; // enough. ex. [-25, 25] covers submap length of 50x1 = 50m if every kf gap is 1m
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr targetKeyframeCloud(new pcl::PointCloud<PointType>());
    loopFindNearKeyframesCloud(cureKeyframeCloud, _curr_kf_idx, 0, _loop_kf_idx); // use same root of loop kf idx
    loopFindNearKeyframesCloud(targetKeyframeCloud, _loop_kf_idx, historyKeyframeSearchNum, _loop_kf_idx);

    // loop verification
    /*sensor_msgs::PointCloud2 cureKeyframeCloudMsg;
    pcl::toROSMsg(*cureKeyframeCloud, cureKeyframeCloudMsg);
    cureKeyframeCloudMsg.header.frame_id = "/camera_init";
    pubLoopScanLocal.publish(cureKeyframeCloudMsg);

    sensor_msgs::PointCloud2 targetKeyframeCloudMsg;
    pcl::toROSMsg(*targetKeyframeCloud, targetKeyframeCloudMsg);
    targetKeyframeCloudMsg.header.frame_id = "/camera_init";
    pubLoopSubmapLocal.publish(targetKeyframeCloudMsg);*/

    // ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(150); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align pointclouds
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(targetKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    float loopFitnessScoreThreshold = 0.3; // user parameter but fixed low value is safe.
    if (icp.hasConverged() == false || icp.getFitnessScore() > loopFitnessScoreThreshold) {
        std::cout << "[SC loop] ICP fitness test failed (" << icp.getFitnessScore() << " > " << loopFitnessScoreThreshold << "). Reject this SC loop." << std::endl;
        pose_gt =  NULL;
    } else
    {
        std::cout << "[SC loop] ICP fitness test passed (" << icp.getFitnessScore() << " < " << loopFitnessScoreThreshold << "). Add this SC loop." << std::endl;
    }

    // Get pose transformation
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    pcl::getTranslationAndEulerAngles (correctionLidarFrame, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
    pose_gt = (gtsam::Pose3 *) malloc (sizeof (gtsam::Pose3));
    pose_gt[0] = poseFrom.between(poseTo);
} // doICPVirtualRelative

void
carmen_convert_vector_3d_point_cloud_to_pcl_point_cloud(carmen_velodyne_partial_scan_message carmen_velodyne_pcl, pcl::PointCloud<PointType> &pcl_point_cloud)
{
	const static double sorted_vertical_angles[32] =
	{
		-30.67, -29.33, -28.0, -26.67, -25.33, -24.0, -22.67, -21.33, -20.0,
		-18.67, -17.33, -16.0, -14.67, -13.33, -12.0, -10.67, -9.3299999, -8.0,
		-6.6700001, -5.3299999, -4.0, -2.6700001, -1.33, 0.0, 1.33, 2.6700001, 4.0,
		5.3299999, 6.6700001, 8.0, 9.3299999, 10.67
	};

	arrange_velodyne_vertical_angles_to_true_position(&carmen_velodyne_pcl);
	for(int i = 0; i < carmen_velodyne_pcl.number_of_32_laser_shots; i++)
	{
		for(int j = 0; j < 32; j++)
		{
			carmen_vector_3D_t point_3d;

			carmen_sphere_coord_t sphere_point;
			double hangle = carmen_normalize_theta(carmen_degrees_to_radians(carmen_velodyne_pcl.partial_scan[i].angle));
			double vangle = carmen_normalize_theta(carmen_degrees_to_radians(sorted_vertical_angles[j]));
			double range = carmen_velodyne_pcl.partial_scan[i].distance[j] / 500.0;
			float intensity = (float)carmen_velodyne_pcl.partial_scan[i].intensity[j] / 255.0;
			sphere_point.horizontal_angle = -hangle;
			sphere_point.vertical_angle = vangle;
			sphere_point.length = range;

			point_3d = carmen_covert_sphere_to_cartesian_coord(sphere_point);
			PointType pointcloud_point;

			pointcloud_point.x = point_3d.x;
			pointcloud_point.y = point_3d.y;
			pointcloud_point.z = point_3d.z;
			pointcloud_point.intensity = intensity;
			pcl_point_cloud.push_back(pointcloud_point);
		}
	}
}

void
process_pg()
{
	int count_i = 0;
    int frame_count = 0;
    while(1)
    {
    	//std::cout << "tam odom " << odometryBuf.size() << "\n";
    	//std::cout << "tam lidar " << fullResBuf.size() << "\n";
		while (  !fullResBuf.empty() )
        {
            //
            // pop and check keyframe is or not
            //
			/*mBuf.lock();
            while (!odometryBuf.empty() && odometryBuf.front()->timestamp < fullResBuf.front()->timestamp)
                odometryBuf.pop();
            if (odometryBuf.empty())
            {
                mBuf.unlock();
                break;
            }*/

            // Time equal check
            //timeLaserOdometry = odometryBuf.front()->timestamp;
			//std::cout << "TAM BUFF " << fullResBuf.size() << " \n";
            timeLaser = fullResBuf.front()->timestamp;
            // TODO

            laserCloudFullRes_2->clear();
            pcl::PointCloud<PointType> thisKeyFrame;
            //pcl::fromROSMsg(*fullResBuf.front(), *thisKeyFrame);
            carmen_convert_vector_3d_point_cloud_to_pcl_point_cloud(fullResBuf.front()[0], thisKeyFrame);
            //printf("COUNT %d\n", count_i);
            count_i++;
            fullResBuf.pop();
            std::vector<pcl::PointCloud<PointType>> vector_cloud = laserCloudHandler(thisKeyFrame);
            std::vector<pcl::PointCloud<PointType>> vector_cloud_1;
            std::pair<carmen_vector_3D_t, carmen_quaternion_t> pair_aux;
            //printf("VOU FAZER ODOMMMMM\n");
            pair_aux = process_odom(vector_cloud, vector_cloud_1, frame_count);
            //printf("SAIRR ODOMMMM\n");
            frame_count++;
            if(frame_count > 8)
            	frame_count = 0;
            std::pair<carmen_vector_3D_t, carmen_quaternion_t> pair_final;
            if(!vector_cloud_1.empty())
            {
            	thisKeyFrame.clear();
            	//printf("ENTREI FUNC LMMMMMM!!!!\n");
            	for (unsigned int i = 0; i < vector_cloud_1[2].size(); i++)
            		thisKeyFrame.push_back(vector_cloud_1[2].points[i]);
            	pair_final = process_lm(vector_cloud_1, pair_aux, thisKeyFrame);
            }else
            	continue;

            // find nearest gps
            double eps = 0.1; // find a gps topioc arrived within eps second
            while (!gpsBuf.empty())
            {
                auto thisGPS = gpsBuf.front();
                auto thisGPSTime = thisGPS->timestamp;
                if( abs(thisGPSTime - timeLaserOdometry) < eps )
                {
                    currGPS = thisGPS;
                    hasGPSforThisKF = true;
                    break;
                } else
                {
                    hasGPSforThisKF = false;
                }
                gpsBuf.pop();
            }
            //mBuf.unlock();
            /*odometryBuf.front()->m_acc.x = currGPS->x;
            odometryBuf.front()->m_acc.y = currGPS->y;
            odometryBuf.front()->m_acc.z = currGPS->z;*/
            Pose6D pose_curr = getOdom(pair_final);
			//odometryBuf.pop();

            //
            // Early reject by counting local delta movement (for equi-spereated kf drop)
            //
            odom_pose_prev = odom_pose_curr;
            odom_pose_curr = pose_curr;
            Pose6D dtf = diffTransformation(odom_pose_prev, odom_pose_curr); // dtf means delta_transform

            double delta_translation = sqrt(dtf.x * dtf.x + dtf.y * dtf.y + dtf.z * dtf.z); // note: absolute value.
            translationAccumulated += delta_translation;
            rotaionAccumulated += (dtf.roll + dtf.pitch + dtf.yaw); // sum just naive approach.

            if( translationAccumulated > keyframeMeterGap || rotaionAccumulated > keyframeRadGap )
            {
                isNowKeyFrame = true;
                translationAccumulated = 0.0; // reset
                rotaionAccumulated = 0.0; // reset
            } else
            {
                isNowKeyFrame = false;
            }

            if( ! isNowKeyFrame )
                continue;

            if( !gpsOffsetInitialized )
            {
                if(hasGPSforThisKF)
                { // if the very first frame
                    gpsAltitudeInitOffset = currGPS->altitude;
                    gpsOffsetInitialized = true;
                }
            }

            //
            // Save data and Add consecutive node
            //
            pcl::PointCloud<PointType>::Ptr Frame_pcl(new pcl::PointCloud<PointType>());
            for(unsigned int k = 0; k < thisKeyFrame.size(); k++)
            	Frame_pcl->push_back(thisKeyFrame.points[k]);
            pcl::PointCloud<PointType>::Ptr thisKeyFrameDS(new pcl::PointCloud<PointType>());
            downSizeFilterScancontext.setInputCloud(Frame_pcl);
            downSizeFilterScancontext.filter(*thisKeyFrameDS);

            mKF.lock();
            keyframeLaserClouds.push_back(thisKeyFrameDS);
            keyframePoses.push_back(pose_curr);
            keyframePosesUpdated.push_back(pose_curr); // init
            //keyframeTimes.push_back(timeLaserOdometry);

            scManager.makeAndSaveScancontextAndKeys(*thisKeyFrameDS);

            laserCloudMapPGORedraw = true;
            mKF.unlock();

            const int prev_node_idx = keyframePoses.size() - 2;
            const int curr_node_idx = keyframePoses.size() - 1; // becuase cpp starts with 0 (actually this index could be any number, but for simple implementation, we follow sequential indexing)
            if( ! gtSAMgraphMade /* prior node */)
            {
                const int init_node_idx = 0;
                gtsam::Pose3 poseOrigin = Pose6DtoGTSAMPose3(keyframePoses.at(init_node_idx));
                // auto poseOrigin = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));

                mtxPosegraph.lock();
                {
                    // prior factor
                    gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(init_node_idx, poseOrigin, priorNoise));
                    initialEstimate.insert(init_node_idx, poseOrigin);
                    // runISAM2opt();
                }
                mtxPosegraph.unlock();

                gtSAMgraphMade = true;

                cout << "posegraph prior node " << init_node_idx << " added" << endl;
            } else /* consecutive node (and odom factor) after the prior added */
            { // == keyframePoses.size() > 1
                gtsam::Pose3 poseFrom = Pose6DtoGTSAMPose3(keyframePoses.at(prev_node_idx));
                gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(keyframePoses.at(curr_node_idx));

                mtxPosegraph.lock();
                {
                    // odom factor
                    gtsam::Pose3 relPose = poseFrom.between(poseTo);
                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, relPose, odomNoise));

                    // gps factor
                    if(hasGPSforThisKF) {
                        double curr_altitude_offseted = currGPS->altitude - gpsAltitudeInitOffset;
                        mtxRecentPose.lock();
                        gtsam::Point3 gpsConstraint(recentOptimizedX, recentOptimizedY, curr_altitude_offseted); // in this example, only adjusting altitude (for x and y, very big noises are set)
                        mtxRecentPose.unlock();
                        gtSAMgraph.add(gtsam::GPSFactor(curr_node_idx, gpsConstraint, robustGPSNoise));
                        cout << "GPS factor added at node " << curr_node_idx << endl;
                    }
                    initialEstimate.insert(curr_node_idx, poseTo);
                    writeEdge({prev_node_idx, curr_node_idx}, relPose, edges_str); // giseop
                    // runISAM2opt();
                }
                mtxPosegraph.unlock();

                if(curr_node_idx % 100 == 0)
                    cout << "posegraph odom node " << curr_node_idx << " added." << endl;
            }
            // if want to print the current graph, use gtSAMgraph.print("\nFactor Graph:\n");

            // save utility
            std::string curr_node_idx_str = padZeros(curr_node_idx);
            pcl::io::savePCDFileBinary(pgScansDirectory + curr_node_idx_str + ".pcd", thisKeyFrame); // scan

            const auto& curr_scd = scManager.getConstRefRecentSCD();
            saveSCD(pgSCDsDirectory + curr_node_idx_str + ".scd", curr_scd);

            pgTimeSaveStream << timeLaser << std::endl; // path
            thisKeyFrame.clear();
			vector_cloud.clear();
			vector_cloud_1.clear();

        }

		//cout<< "TERMINEI LOOOPPPP!!!!\n";
        // ps.
        // scan context detector is running in another thread (in constant Hz, e.g., 1 Hz)
        // pub path and point cloud in another thread

        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(10);
        std::this_thread::sleep_for(dura);
    }
} // process_pg

void
performSCLoopClosure(void)
{
    if( int(keyframePoses.size()) < scManager.NUM_EXCLUDE_RECENT) // do not try too early
        return;

    auto detectResult = scManager.detectLoopClosureID(); // first: nn index, second: yaw diff
    int SCclosestHistoryFrameID = detectResult.first;
    if( SCclosestHistoryFrameID != -1 )
    {
        const int prev_node_idx = SCclosestHistoryFrameID;
        const int curr_node_idx = keyframePoses.size() - 1; // because cpp starts 0 and ends n-1
        cout << "Loop detected! - between " << prev_node_idx << " and " << curr_node_idx << "" << endl;

        mBuf.lock();
        scLoopICPBuf.push(std::pair<int, int>(prev_node_idx, curr_node_idx));
        // addding actual 6D constraints in the other thread, icp_calculation.
        mBuf.unlock();
    }
} // performSCLoopClosure

void
process_lcd(void)
{
    //float loopClosureFrequency = 1.0; // can change
    //ros::Rate rate(loopClosureFrequency);
    while (1)
    {
        performSCLoopClosure();
		std::chrono::milliseconds dura(600);
		std::this_thread::sleep_for(dura);
		/*if((1 - time_elapsed) > 0)
		{
			usleep((useconds_t) ((1 - time_elapsed) * 1000000));
			carmen_ipc_dispatch_nonblocking();
		}*/
        // performRSLoopClosure(); // TODO
    }
} // process_lcd

void
process_icp(void)
{
    while(1)
    {
		while ( !scLoopICPBuf.empty() )
        {
            if( scLoopICPBuf.size() > 30 )
            {
                printf("Too many loop clousre candidates to be ICPed is waiting ... Do process_lcd less frequently (adjust loopClosureFrequency)");
            }

            mBuf.lock();
            std::pair<int, int> loop_idx_pair = scLoopICPBuf.front();
            scLoopICPBuf.pop();
            mBuf.unlock();

            const int prev_node_idx = loop_idx_pair.first;
            const int curr_node_idx = loop_idx_pair.second;
            gtsam::Pose3 *relative_pose_optional = NULL;
			doICPVirtualRelative(prev_node_idx, curr_node_idx, relative_pose_optional);
            if(relative_pose_optional != NULL) {
                gtsam::Pose3 relative_pose = relative_pose_optional[0];
                mtxPosegraph.lock();
                gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, relative_pose, robustLoopNoise));
                writeEdge({prev_node_idx, curr_node_idx}, relative_pose, edges_str); // giseop
                // runISAM2opt();
                mtxPosegraph.unlock();
            }
        }

        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(1);
        std::this_thread::sleep_for(dura);
    }
} // process_icp

void
process_viz_path(void)
{
    //ros::Rate rate(hz);
    while (1)
    {
        //carmen_ipc_sleep(0.1);
    	std::cout << "CHECK FLAG " << recentIdxUpdated << "\n";
        if(recentIdxUpdated > 1)
        {
            pubPath();
        }
    }
}

void
process_isam(void)
{
    //float hz = 1;
    //ros::Rate rate(hz);
    while (1)
    {

        if( gtSAMgraphMade )
        {
            mtxPosegraph.lock();
            runISAM2opt();
            cout << "running isam2 optimization ..." << endl;
            mtxPosegraph.unlock();

            saveOptimizedVerticesKITTIformat(isamCurrentEstimate, pgKITTIformat); // pose
            saveOdometryVerticesKITTIformat(odomKITTIformat); // pose
            saveGTSAMgraphG2oFormat(isamCurrentEstimate);
        }
        std::chrono::milliseconds dura(400);
		std::this_thread::sleep_for(dura);
        /*double time_final = carmen_get_time();
        double time_elapsed = time_final - time_init;
        if((1 - time_elapsed) > 0)
		{
			usleep((useconds_t) ((1 - time_elapsed) * 1000000));
			carmen_ipc_dispatch_nonblocking();
		}*/
    }
}

/*void
pubMap(void)
{
    int SKIP_FRAMES = 2; // sparse map visulalization to save computations
    int counter = 0;

    laserCloudMapPGO->clear();

    mKF.lock();
    // for (int node_idx=0; node_idx < int(keyframePosesUpdated.size()); node_idx++) {
    for (int node_idx=0; node_idx < recentIdxUpdated; node_idx++) {
        if(counter % SKIP_FRAMES == 0) {
            *laserCloudMapPGO += *local2global(keyframeLaserClouds[node_idx], keyframePosesUpdated[node_idx]);
        }
        counter++;
    }
    mKF.unlock();

    downSizeFilterMapPGO.setInputCloud(laserCloudMapPGO);
    downSizeFilterMapPGO.filter(*laserCloudMapPGO);

    sensor_msgs::PointCloud2 laserCloudMapPGOMsg;
    pcl::toROSMsg(*laserCloudMapPGO, laserCloudMapPGOMsg);
    laserCloudMapPGOMsg.header.frame_id = "/camera_init";
    pubMapAftPGO.publish(laserCloudMapPGOMsg);
}*/

/*void
process_viz_map(void)
{
    float vizmapFrequency = 0.1; // 0.1 means run onces every 10s
    ros::Rate rate(vizmapFrequency);
    while (ros::ok()) {
        rate.sleep();
        if(recentIdxUpdated > 1) {
            pubMap();
        }
    }
} // pointcloud_viz
*/

void
define_messages()
{
	IPC_RETURN_TYPE err;

	carmen_localize_ackerman_define_globalpos_messages();

	err = IPC_defineMsg(CARMEN_FUSED_ODOMETRY_PARTICLE_NAME, IPC_VARIABLE_LENGTH, CARMEN_FUSED_ODOMETRY_PARTICLE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_FUSED_ODOMETRY_PARTICLE_NAME);
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	define_messages();

	//carmen_ipc_dispatch();

    // save directories
	//nh.param<std::string>("save_directory", save_directory, "/"); // pose assignment every k m move
	save_directory = "/media/lume/f3bce44c-b721-4f2d-8fd6-91d76b9581b5/02_result/";
    pgKITTIformat = save_directory + "optimized_poses.txt";
    odomKITTIformat = save_directory + "odom_poses.txt";

    // pgG2oSaveStream = std::fstream(save_directory + "singlesession_posegraph.g2o", std::fstream::out);

    pgTimeSaveStream = std::fstream(save_directory + "times.txt", std::fstream::out);
    pgTimeSaveStream.precision(std::numeric_limits<double>::max_digits10);

    pgScansDirectory = save_directory + "Scans/";
    system((std::string("exec rm -r ") + pgScansDirectory).c_str());
    system((std::string("mkdir -p ") + pgScansDirectory).c_str());

    pgSCDsDirectory = save_directory + "SCDs/"; // SCD: scan context descriptor
    system((std::string("exec rm -r ") + pgSCDsDirectory).c_str());
    system((std::string("mkdir -p ") + pgSCDsDirectory).c_str());

    // system params
	//nh.param<double>("keyframe_meter_gap", keyframeMeterGap, 2.0); // pose assignment every k m move
    double keyframeMeterGap = 2.0;
	//nh.param<double>("keyframe_deg_gap", keyframeDegGap, 10.0); // pose assignment every k deg rot
    double keyframeDegGap = 10.0;
    keyframeRadGap = deg2rad(keyframeDegGap);

	//nh.param<double>("sc_dist_thres", scDistThres, 0.2);
    double scDistThres = 0.2;
	//nh.param<double>("sc_max_radius", scMaximumRadius, 80.0); // 80 is recommended for outdoor, and lower (ex, 20, 40) values are recommended for indoor
    double scMaximumRadius = 80.0;
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);

    initNoises();

    scManager.setSCdistThres(scDistThres);
    scManager.setMaximumRadius(scMaximumRadius);

    float filter_size = 0.4;
    downSizeFilterScancontext.setLeafSize(filter_size, filter_size, filter_size);
    downSizeFilterICP.setLeafSize(filter_size, filter_size, filter_size);

    /*double mapVizFilterSize;
	nh.param<double>("mapviz_filter_size", mapVizFilterSize, 0.4); // pose assignment every k frames
    downSizeFilterMapPGO.setLeafSize(mapVizFilterSize, mapVizFilterSize, mapVizFilterSize);

	ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_registered_local", 100, laserCloudFullResHandler);
	ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 100, laserOdometryHandler);
	ros::Subscriber subGPS = nh.subscribe<sensor_msgs::NavSatFix>("/gps/fix", 100, gpsHandler);

	pubOdomAftPGO = nh.advertise<nav_msgs::Odometry>("/aft_pgo_odom", 100);
	pubOdomRepubVerifier = nh.advertise<nav_msgs::Odometry>("/repub_odom", 100);
	pubPathAftPGO = nh.advertise<nav_msgs::Path>("/aft_pgo_path", 100);
	pubMapAftPGO = nh.advertise<sensor_msgs::PointCloud2>("/aft_pgo_map", 100);

	pubLoopScanLocal = nh.advertise<sensor_msgs::PointCloud2>("/loop_scan_local", 100);
	pubLoopSubmapLocal = nh.advertise<sensor_msgs::PointCloud2>("/loop_submap_local", 100);*/
    //std::thread subs(subscribe_messages);
    subscribe_messages();
    std::thread posegraph_slam(process_pg);
    std::thread lc_detection(process_lcd); // loop closure detection
	std::thread icp_calculation(process_icp); // loop constraint calculation via icp
	std::thread isam_update(process_isam); // if you want to call less isam2 run (for saving redundant computations and no real-time visulization is required), uncommment this and comment all the above runisam2opt when node is added.
	//std::thread viz_path {process_viz_path}; // visualization - path (high frequency)
	carmen_ipc_dispatch();

	//s0td::thread viz_map {process_viz_map}; // visualization - map (low frequency because it is heavy)


	return 0;
}
