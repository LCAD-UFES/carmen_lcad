
#ifndef _SEGMAP_LIBSEGMAP_SEGMAP_DATASET_H_
#define _SEGMAP_LIBSEGMAP_SEGMAP_DATASET_H_

#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv/cv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <carmen/synchronized_data_package.h>
#include <carmen/odom_calib_data.h>
#include <carmen/segmap_pose2d.h>


class NewCarmenDataset
{
public:
	
	enum SyncSensor
	{
		SYNC_BY_CAMERA = 0,
		SYNC_BY_VELODYNE
	};

	enum SyncMode
	{
		// First store the messages of all sensors in different queues.
		// Then, for each message of the reference sensor, search in all queues
		// the messages with nearest time, even if they are posterior to the reference
		// sensor message. Messages can be repeated in the sense that they can be
		// the nearest messages to different reference sensor messages.
		SYNC_BY_NEAREST = 0,

		// This method is the most similar to what is found in real applications. Messages
		// from all sensors are stored in queues until a message from the reference sensor is
		// received. Then, we search for the messages with nearest time, and then the queue are cleared.
		SYNC_BY_NEAREST_BEFORE,
	};

	NewCarmenDataset(std::string path,
	                 std::string odom_calib_path = "",
									 std::string poses_path = "",
									 int gps_id = 1,
									 NewCarmenDataset::SyncSensor sync_sensor = SYNC_BY_CAMERA,
									 NewCarmenDataset::SyncMode sync_mode = SYNC_BY_NEAREST,
	                 std::string lidar_calib_path = "");

	~NewCarmenDataset();

	// Returns a matrix to transform from lidar to camera frame.
	// Points shall be in homogeneous coordinates.
	// To obtain the pixel positions, multiply the transformed x-coordinate
	// by the image width, and y-coordinate by the image height.
	Eigen::Matrix<double, 4, 4> vel2cam();

	// matrix to project from camera frame to image coordinates.
	Eigen::Matrix<double, 3, 4> projection_matrix();

	// Returns a matrix to transfrom from lidar to car frame.
	Eigen::Matrix<double, 4, 4> vel2car();

	// Returns a matrix to transfrom from xsens to car frame.
	Eigen::Matrix<double, 4, 4> xsens2car();

	unsigned char ***intensity_calibration;

	// Number of messages of the sensor used for synchronization.
	// For example, if the velodyne is the reference sensor, the method returns
	// the number of velodyne messages in the log.
	int size() const;

	// returns the i-th synchronized data package.
	DataSample* operator[](int i) const;
	DataSample* at(int i) const;

	double initial_angle() const;

protected:

	static const long _MAX_LINE_LENGTH = (5*4000000);

	std::string _images_dir;
	std::string _velodyne_dir;

	SyncSensor _sync_sensor;
	SyncMode _sync_mode;
	int _gps_id;

	OdomCalib _calib;
	std::vector<DataSample*> _data;
	std::vector<Pose2d> _poses;

	std::vector<std::string> _imu_messages;
	std::vector<std::string> _gps_position_messages;
	std::vector<std::string> _gps_orientation_messages;
	std::vector<std::string> _odom_messages;
	std::vector<std::string> _camera_messages;
	std::vector<std::string> _velodyne_messages;

	std::vector<double> _imu_times;
	std::vector<double> _gps_position_times;
	std::vector<double> _gps_orientation_times;
	std::vector<double> _odom_times;
	std::vector<double> _camera_times;
	std::vector<double> _velodyne_times;

	// Returns a matrix to transfrom from the board to the car frame.
	Eigen::Matrix<double, 4, 4> _board2car();

	void _load_log(std::string &path);
	void _load_odometry_calibration(std::string &path);
	void _load_intensity_calibration(std::string &path);
	void _load_poses(std::string &path, std::vector<Pose2d> *poses);
	void _update_data_with_poses();

	void _load_synchronizing_by_nearest(FILE *fptr);
	void _load_synchronizing_by_nearest_before(FILE *fptr);

	void _read_until_reference_sensor_message(FILE *fptr);
	void _read_and_enqueue_one_message(FILE *fptr);

	void _synchronize_messages_by_times(std::vector<double> &reference_sensor_times);
	DataSample* _create_synchronized_data_package(double ref_time);

	void _clear_synchronization_queues();
	void _add_message_to_queue(std::string);

	static std::vector<std::string> _find_nearest(std::vector<std::string> &queue,
																								std::vector<double> &times, double ref_time);

	static unsigned char*** _allocate_calibration_table();
	static void _free_calibration_table(unsigned char ***table);

	static void _parse_odom(std::vector<std::string> data, DataSample *sample);
	static void _parse_imu(std::vector<std::string> data, DataSample *sample);
	static void _parse_velodyne(std::vector<std::string> data, DataSample *sample, std::string velodyne_path);
	static void _parse_camera(std::vector<std::string> data, DataSample *sample, std::string image_path);
	static void _parse_gps_position(std::vector<std::string> data, DataSample *sample);
	static void _parse_gps_orientation(std::vector<std::string> data, DataSample *sample);

};


std::string default_odom_calib_path(const char *log_path);
std::string default_fused_odom_path(const char *log_path);
std::string default_graphslam_path(const char *log_path);
std::string default_graphslam_to_map_path(const char *log_path);


#endif


