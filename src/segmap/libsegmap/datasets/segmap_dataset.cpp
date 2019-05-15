
#include <carmen/carmen.h>
#include <carmen/Gdc_Coord_3d.h>
#include <carmen/Utm_Coord_3d.h>
#include <carmen/Gdc_To_Utm_Converter.h>

#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv/cv.hpp>

#include <carmen/util_io.h>
#include <carmen/util_math.h>
#include <carmen/util_strings.h>

#include <carmen/segmap_pose2d.h>
#include <carmen/segmap_dataset.h>
#include <carmen/segmap_conversions.h>

using namespace cv;
using namespace std;
using namespace pcl;
using namespace Eigen;


NewCarmenDataset::NewCarmenDataset(std::string path,
																	 std::string carmen_ini,
                                   std::string odom_calib_path,
																	 std::string poses_path,
																	 double camera_latency,
																	 int gps_id,
																	 NewCarmenDataset::SyncSensor sync_sensor,
																	 NewCarmenDataset::SyncMode sync_mode)
{

	printf("sync_sensor: %d\n", sync_sensor);
	_gps_id = gps_id;
	_sync_sensor = sync_sensor;
	_sync_mode = sync_mode;
	_camera_latency = camera_latency;

	_velodyne_dir = string(path) + "_velodyne";
	_images_dir = string(path) + "_bumblebee";

	_params = new CarmenParamFile(carmen_ini.c_str());
	_load_odometry_calibration(odom_calib_path);

	// reading the log requires odom calibration data.
	_load_log(path);

	_load_poses(poses_path, &_poses);
	_update_data_with_poses();
}


NewCarmenDataset::~NewCarmenDataset()
{
	for (int i = 0; i < _data.size(); i++)
		delete(_data[i]);

	delete(_params);
}


int
NewCarmenDataset::size() const
{
	return _data.size();
}


DataSample*
NewCarmenDataset::operator[](int i) const
{
	return at(i);
}


DataSample*
NewCarmenDataset::at(int i) const
{
	if (i >= size())
		exit(printf("Error: Trying to access data package %d of a log with %d packages\n", i, size()));

	return _data[i];
}


double
NewCarmenDataset::initial_angle() const
{
	return _calib.init_angle;
}


Matrix<double, 4, 4> 
NewCarmenDataset::vel2cam()
{
	// *****************************************************
	// Transform according to carmen parameters file.
	// *****************************************************
	//Matrix<double, 4, 4> velodyne2board;
	//Matrix<double, 4, 4> cam2board;
	//
	//velodyne2board = pose6d_to_matrix(0.145, 0., 0.48, 0.0, -0.0227, -0.01);
	//cam2board = pose6d_to_matrix(0.245, -0.04, 0.210, -0.017453, 0.026037, -0.023562 + carmen_degrees_to_radians(1.35));
	//
	//_vel2cam = projection * pose6d_to_matrix(0.04, 0.115, -0.27, -M_PI/2-0.052360, -0.034907, -M_PI/2-0.008727).inverse();
	//return cam2board.inverse() * velodyne2board;

	// *****************************************************
	// Transform from manual calibration.
	// *****************************************************
	return pose6d_to_matrix(-0.020000, 0.125000, -0.27, -0.015708, 0.048869, 0.005236).inverse();
}


Matrix<double, 3, 4> 
NewCarmenDataset::projection_matrix()
{
	Matrix<double, 3, 4> projection;
	Matrix<double, 4, 4> R;

	// Rotation to change the reference system from x: forward, y: left, z: up to x: right, y: down, z: forward.
	R = pose6d_to_matrix(0., 0., 0., -M_PI/2., 0, -M_PI/2).inverse();

	double fx_factor = 0.764749;
	double fy_factor = 1.01966;
	double cu_factor = 0.505423;
	double cv_factor = 0.493814;
	double pixel_size = 0.00000375;

	double fx_meters = fx_factor * pixel_size;
	double fy_meters = fy_factor * pixel_size;

	double cu = cu_factor;
	double cv = cv_factor;

	// see http://www.cvlibs.net/publications/Geiger2013IJRR.pdf
	// Note: Storing cu and cv in the 3rd column instead of the 4th is a trick.
	// To compute the pixel coordinates we divide the first two
	// dimensions of the point in homogeneous coordinates by the third one (which is Z).
	projection << fx_meters / pixel_size, 0, cu, 0,
			0, fy_meters / pixel_size, cv, 0,
			0, 0, 1, 0.;

	return projection * R;				  
}


Matrix<double, 4, 4> 
NewCarmenDataset::vel2car()
{
	Matrix<double, 4, 4> velodyne2board;

	//velodyne2board = pose6d_to_matrix(0.145, 0., 0.48, 0.0, -0.0227, -0.01);
	velodyne2board = pose6d_to_matrix(
		_params->get<double>("velodyne_x"),
		_params->get<double>("velodyne_y"),
		_params->get<double>("velodyne_z"),
		_params->get<double>("velodyne_roll"),
		_params->get<double>("velodyne_pitch"),
		_params->get<double>("velodyne_yaw")
	);

	return _board2car() * velodyne2board;
}


Matrix<double, 4, 4>
NewCarmenDataset::xsens2car()
{
	Matrix<double, 4, 4> xsens2board;

	//xsens2board = pose6d_to_matrix(0.175, -0.01, 0.25, 0, 0, 0);
	xsens2board = pose6d_to_matrix(
		_params->get<double>("xsens_x"),
		_params->get<double>("xsens_y"),
		_params->get<double>("xsens_z"),
		_params->get<double>("xsens_roll"),
		_params->get<double>("xsens_pitch"),
		_params->get<double>("xsens_yaw")
	);


	return _board2car() * xsens2board;
}


// Returns a matrix to transfrom from gps to car frame.
Eigen::Matrix<double, 4, 4>
NewCarmenDataset::car2gps()
{
	Matrix<double, 4, 4> gps2car_t = gps2car();
	return gps2car_t.inverse();
}


Eigen::Matrix<double, 4, 4>
NewCarmenDataset::gps2car()
{
	Matrix<double, 4, 4> gps2board;

	char gps_name[128];
	sprintf(gps_name, "gps_nmea_%d", _gps_id);
	string gps_name_str = string(gps_name);

	//gps2board = pose6d_to_matrix(0.175, -0.01, 0.25, 0, 0, 0);
	gps2board = pose6d_to_matrix(
		_params->get<double>(gps_name_str + "_x"),
		_params->get<double>(gps_name_str + "_y"),
		_params->get<double>(gps_name_str + "_z"),
		_params->get<double>(gps_name_str + "_roll"),
		_params->get<double>(gps_name_str + "_pitch"),
		_params->get<double>(gps_name_str + "_yaw")
	);

	return _board2car() * gps2board;
}


Eigen::Matrix<double, 4, 4>
NewCarmenDataset::_board2car()
{
	Matrix<double, 4, 4> board2car;

	//board2car = pose6d_to_matrix(0.572, 0, 1.394, 0.0, 0.0122173048, 0.0);
	board2car = pose6d_to_matrix(
		_params->get<double>("sensor_board_1_x"),
		_params->get<double>("sensor_board_1_y"),
		_params->get<double>("sensor_board_1_z"),
		_params->get<double>("sensor_board_1_roll"),
		_params->get<double>("sensor_board_1_pitch"),
		_params->get<double>("sensor_board_1_yaw")
	);

	return board2car;
}


void 
NewCarmenDataset::_load_odometry_calibration(std::string &path)
{
	int success = false;

	if (path.size() > 0)
	{
		FILE *f = fopen(path.c_str(), "r");

		if (f != NULL)
		{
			string line = read_line(f);
			vector<string> tokens = string_split(line, " ");
			
			// "bias v: %lf %lf bias phi: %lf %lf Initial Angle: %lf"
			if (tokens.size() == 11)
			{
				_calib.mult_v = from_string<double>(tokens[2]);
				_calib.add_v = from_string<double>(tokens[3]);
				_calib.mult_phi = from_string<double>(tokens[6]);
				_calib.add_phi = from_string<double>(tokens[7]);
				_calib.init_angle = from_string<double>(tokens[10]);

				success = true;
			}
			// "v (multiplier bias): (%lf %lf),  phi (multiplier bias): (%lf %lf),  Initial Angle: %lf"
			else if (tokens.size() == 13)
			{
				_calib.mult_v = from_string<double>(replace(tokens[3], "(", ""));
				_calib.add_v = from_string<double>(replace(tokens[4], "),", ""));
				_calib.mult_phi = from_string<double>(replace(tokens[8], "(", ""));
				_calib.add_phi = from_string<double>(replace(tokens[9], "),", ""));
				_calib.init_angle = from_string<double>(tokens[12]);

				success = true;
			}

			fclose(f);
		}
	}

	if (!success)
	{
		fprintf(stderr, "Warning: failed load odometry calibration from '%s'. Assuming default values.\n", path.c_str());

		_calib.mult_phi = _calib.mult_v = 1.0;
		_calib.add_phi = _calib.add_v = 0.;
		_calib.init_angle = 0.;
	}
	else
		printf("Odometry calibration successfully loaded from '%s'\n", path.c_str());

	printf("Odom calibration: bias v: %lf %lf bias phi: %lf %lf\n", 
				 _calib.mult_v, _calib.add_v, _calib.mult_phi, _calib.add_phi);
}


void
NewCarmenDataset::_load_poses(std::string &path, std::vector<Pose2d> *poses)
{
	bool success = false;
	char dummy[64];

	if (path.size() > 0)
	{
		FILE *f = fopen(path.c_str(), "r");

		if (f != NULL)
		{
			while (!feof(f))
			{
				Pose2d pose;

				fscanf(f, "\n%s %lf %lf %lf %s %s %s\n",
							 dummy, &pose.x, &pose.y, &pose.th,
							 dummy, dummy, dummy);

				//printf("%lf %lf %lf\n", pose.x, pose.y, pose.th);
				poses->push_back(pose);
			}

			if (poses->size() == size())
				success = 1;
			else
				fprintf(stderr, "Warning: number of poses %ld is different from log size %d\n",
								poses->size(), size());

			fclose(f);
		}
	}

	if (!success)
	{
		fprintf(stderr, "Warning: failed to load poses from '%s'. Returning default values.\n", path.c_str());
		poses->clear();
	}
	else
		fprintf(stderr, "Poses successfully loaded from '%s'.\n", path.c_str());
}


void
NewCarmenDataset::_update_data_with_poses()
{
	assert(_poses.size() == _data.size() || _poses.size() == 0);

	if (_poses.size() == _data.size())
	{
		for (int i = 0; i < size(); i++)
			at(i)->pose = _poses[i];
	}
	else
	{
		for (int i = 0; i < size(); i++)
			at(i)->pose = Pose2d(0, 0, 0);
	}
}


void
NewCarmenDataset::_load_log(std::string &path)
{
	printf("Loading log '%s'. It may take a few seconds.\n", path.c_str());

	FILE *fptr = safe_fopen(path.c_str(), "r");

	if (_sync_mode == SYNC_BY_NEAREST)
		_load_synchronizing_by_nearest(fptr);
	else if (_sync_mode == SYNC_BY_NEAREST_BEFORE)
		_load_synchronizing_by_nearest_before(fptr);
	else
		exit(printf("Error: Invalid synchronization mode '%d'.\n", (int) _sync_mode));

	fclose(fptr);

	_clear_synchronization_queues();
}


void
NewCarmenDataset::_load_synchronizing_by_nearest(FILE *fptr)
{
	// load all messages to different queues
	while (!feof(fptr))
		_read_and_enqueue_one_message(fptr);

	if (_sync_sensor == SYNC_BY_VELODYNE)
		_synchronize_messages_by_times(_velodyne_times);
	else if (_sync_sensor == SYNC_BY_CAMERA)
		_synchronize_messages_by_times(_camera_times);
	else
		exit(printf("Error: Invalid synchronization sensor '%d'\n", (int) _sync_sensor));
}


void
NewCarmenDataset::_load_synchronizing_by_nearest_before(FILE *fptr)
{
	double time;
	int reference_sensor_received = 0;
	DataSample *sample;

	while (!feof(fptr))
	{
		_read_and_enqueue_one_message(fptr);

		if ((_sync_sensor == SYNC_BY_VELODYNE) && (_velodyne_times.size() > 0))
		{
			time = _velodyne_times[_velodyne_times.size() - 1];
			reference_sensor_received = 1;
		}
		else if ((_sync_sensor == SYNC_BY_CAMERA) && (_camera_messages.size() > 0))
		{
			time = _camera_times[_camera_times.size() - 1];
			reference_sensor_received = 1;
		}
		else
			reference_sensor_received = 0;

		// as soon as a message from the reference sensor is received,
		// we created a data package.
		if (reference_sensor_received)
		{
			sample = _create_synchronized_data_package(time);
			_data.push_back(sample);
			_clear_synchronization_queues();
		}
	}
}


void
NewCarmenDataset::_synchronize_messages_by_times(vector<double> &reference_sensor_times)
{
	DataSample *sample;

	for (int i = 0; i < reference_sensor_times.size(); i++)
	{
		sample = _create_synchronized_data_package(reference_sensor_times[i]);
		_data.push_back(sample);
	}
}


DataSample*
NewCarmenDataset::_create_synchronized_data_package(double ref_time)
{
	DataSample *sample = new DataSample();

	if (_velodyne_messages.size())
		_parse_velodyne(_find_nearest(_velodyne_messages, _velodyne_times, ref_time), sample, _velodyne_dir);

	if (_odom_messages.size())
		_parse_odom(_find_nearest(_odom_messages, _odom_times, ref_time), sample);

	if (_imu_messages.size())
		_parse_imu(_find_nearest(_imu_messages, _imu_times, ref_time), sample);

	if (_gps_position_messages.size())
		_parse_gps_position(_find_nearest(_gps_position_messages, _gps_position_times, ref_time), sample);

	if (_gps_orientation_messages.size())
		_parse_gps_orientation(_find_nearest(_gps_orientation_messages, _gps_orientation_times, ref_time), sample);

	if (_camera_messages.size())
		_parse_camera(_find_nearest(_camera_messages, _camera_times, ref_time + _camera_latency), sample, _images_dir);

	sample->v = sample->v * _calib.mult_v + _calib.add_v;
	sample->phi = normalize_theta(sample->phi * _calib.mult_phi + _calib.add_phi);
	sample->time = sample->velodyne_time;

	return sample;
}


void
NewCarmenDataset::_read_and_enqueue_one_message(FILE *fptr)
{
	// static to prevent reallocation
	static char line[_MAX_LINE_LENGTH];

	fscanf(fptr, "\n%[^\n]\n", line);
	string copy_as_string = string(line);
	_add_message_to_queue(copy_as_string);
}


void 
NewCarmenDataset::_clear_synchronization_queues()
{
	_imu_messages.clear();
	_gps_position_messages.clear();
	_gps_orientation_messages.clear();
	_odom_messages.clear();
	_camera_messages.clear();
	_velodyne_messages.clear();

	_imu_times.clear();
	_gps_position_times.clear();
	_gps_orientation_times.clear();
	_odom_times.clear();
	_camera_times.clear();
	_velodyne_times.clear();
}


void 
NewCarmenDataset::_add_message_to_queue(string line)
{
	// ignore small lines
	if (line.size() > 10)
	{
		const char *cline = line.c_str();
		vector<string> splitted = string_split(line, " ");

		// ignore lines with less than 4 fields separated by spaces.
		if (splitted.size() < 4)
			return;

		double time = atof(splitted[splitted.size() - 3].c_str());

		if (!strncmp("NMEAGGA", cline, strlen("NMEAGGA"))
				&& (cline[strlen("NMEAGGA") + 1] - '0') == _gps_id)
		{
			_gps_position_messages.push_back(cline);
			_gps_position_times.push_back(time);
		}
		else if (!strncmp("NMEAHDT", cline, strlen("NMEAHDT")))
		{
			_gps_orientation_messages.push_back(cline);
			_gps_orientation_times.push_back(time);
		}
		else if (!strncmp("ROBOTVELOCITY_ACK", cline, strlen("ROBOTVELOCITY_ACK")))
		{
			_odom_messages.push_back(cline);
			_odom_times.push_back(time);
		}
		else if (!strncmp("XSENS_QUAT", cline, strlen("XSENS_QUAT")))
		{
			_imu_messages.push_back(cline);
			_imu_times.push_back(time);
		}
		else if (!strncmp("BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE3", cline, strlen("BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE3")))
		{
			_camera_messages.push_back(cline);
			_camera_times.push_back(time);
		}
		else if (!strncmp("VELODYNE_PARTIAL_SCAN_IN_FILE", cline, strlen("VELODYNE_PARTIAL_SCAN_IN_FILE")))
		{
			_velodyne_messages.push_back(cline);
			_velodyne_times.push_back(time);
		}
	}
}


vector<string>
NewCarmenDataset::_find_nearest(vector<string> &queue, vector<double> &times, double ref_time)
{
	double msg_time, nearest_time;
	vector<string> splitted;
	vector<string> most_sync;

	nearest_time = 0;
	int id = 0;

	for (int i = 0; i < queue.size(); i++)
	{
		msg_time = times[i];

		if (fabs(msg_time - ref_time) < fabs(nearest_time - ref_time))
		{
			id = i;
			nearest_time = msg_time;
		}
	}

	return string_split(queue[id], " ");
}


void 
NewCarmenDataset::_parse_odom(vector<string> data, DataSample *sample)
{
	assert(data.size() == 6);

	sample->v = atof(data[1].c_str());
	sample->phi = atof(data[2].c_str());
	sample->odom_time = atof(data[3].c_str());
}


void 
NewCarmenDataset::_parse_imu(vector<string> data, DataSample *sample)
{
	assert(data.size() == 19);

	sample->xsens = Quaterniond(
			atof(data[4].c_str()),
			atof(data[5].c_str()),
			atof(data[6].c_str()),
			atof(data[7].c_str())
	);

	sample->xsens_time = atof(data[data.size() - 3].c_str());
}


void
NewCarmenDataset::_parse_velodyne(vector<string> data, DataSample *sample, string velodyne_path)
{
	assert(data.size() == 6);

	vector<string> splitted = string_split(data[1], "/");
	int n = splitted.size();

	string path = velodyne_path + "/" + 
			splitted[n - 3] + "/" +
			splitted[n - 2] + "/" +
			splitted[n - 1];

	sample->n_laser_shots = atoi(data[2].c_str());
	sample->velodyne_path = path;
	sample->velodyne_time =  atof(data[data.size() - 3].c_str());
}


void 
NewCarmenDataset::_parse_camera(vector<string> data, DataSample *sample, string image_path)
{
	assert(data.size() == 9);

	vector<string> splitted = string_split(data[1], "/");
	int n = splitted.size();

	string path = image_path + "/" + 
			splitted[n - 3] + "/" +
			splitted[n - 2] + "/" +
			splitted[n - 1];

	sample->image_path = path;
	sample->image_height = atoi(data[3].c_str());
	sample->image_width = atoi(data[2].c_str());
	sample->image_time = atof(data[data.size() - 3].c_str());
}


void 
NewCarmenDataset::_parse_gps_position(vector<string> data, DataSample *sample)
{
	double lt = carmen_global_convert_degmin_to_double(atof(data[3].c_str()));
	double lg = carmen_global_convert_degmin_to_double(atof(data[5].c_str()));

	// verify the latitude and longitude orientations
	if ('S' == data[4][0]) lt = -lt;
	if ('W' == data[6][0]) lg = -lg;

	// convert to x and y coordinates
	Gdc_Coord_3d gdc = Gdc_Coord_3d(lt, lg, atof(data[10].c_str()));

	// Transformando o z utilizando como altitude a altitude mesmo - que esta vindo como zero
	Utm_Coord_3d utm;
	Gdc_To_Utm_Converter::Init();
	Gdc_To_Utm_Converter::Convert(gdc , utm);

	double offset_x = 7762735.177110;
	double offset_y = -358558.484606;

	sample->gps.x = utm.y - offset_x;
	sample->gps.y = -utm.x - offset_y;

	sample->gps_quality = atoi(data[7].c_str());
	sample->gps_time = atof(data[data.size() - 3].c_str());
}


void 
NewCarmenDataset::_parse_gps_orientation(vector<string> data, DataSample *sample)
{
	sample->gps.th = atof(data[2].c_str());
	sample->gps_orientation_quality = atoi(data[3].c_str());
}


std::string
default_odom_calib_path(const char *log_path)
{
	std::string log_name = file_name_from_path(log_path);
	return (string("/dados/data2/data_") + log_name + string("/odom_calib.txt"));
}


std::string
default_fused_odom_path(const char *log_path)
{
	std::string log_name = file_name_from_path(log_path);
	return (string("/dados/data2/data_") + log_name + string("/fused_odom.txt"));
}


std::string
default_graphslam_path(const char *log_path)
{
	std::string log_name = file_name_from_path(log_path);
	return (string("/dados/data2/data_") + log_name + string("/graphslam.txt"));
}


std::string
default_graphslam_to_map_path(const char *log_path)
{
	std::string log_name = file_name_from_path(log_path);
	return (string("/dados/data2/data_") + log_name + string("/graphslam_to_map.txt"));
}


std::string
default_intensity_calib_path(const char *log_path)
{
	//std::string log_name = file_name_from_path(log_path);
	//return (string("/dados/data2/data_") + log_name + string("/intensity_calibration.txt"));
	return string(getenv("CARMEN_HOME")) + "/bin/calibration_table.txt";
}

