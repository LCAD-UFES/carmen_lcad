
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
                                   std::string odom_calib_path,
																	 std::string fused_odom_path,
																	 int gps_id,
																	 NewCarmenDataset::SyncSensor sync_sensor,
																	 NewCarmenDataset::SyncMode sync_mode,
                                   std::string intensity_calib_path)
{
	_gps_id = gps_id;
	_sync_sensor = sync_sensor;
	_sync_mode = sync_mode;

	_velodyne_dir = string(path) + "_velodyne";
	_images_dir = string(path) + "_bumblebee";

	intensity_calibration = _allocate_calibration_table();

	_load_intensity_calibration(intensity_calib_path);
	_load_odometry_calibration(odom_calib_path);

	// reading the log requires odom calibration data.
	_load_log(path);

	_load_poses(fused_odom_path, &_fused_odom);
	_update_data_with_poses();
}


NewCarmenDataset::~NewCarmenDataset()
{
	_free_calibration_table(intensity_calibration);

	for (int i = 0; i < _data.size(); i++)
		delete(_data[i]);
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
	velodyne2board = pose6d_to_matrix(0.145, 0., 0.48, 0.0, -0.0227, -0.01);
	return _board2car() * velodyne2board;
}


Matrix<double, 4, 4>
NewCarmenDataset::xsens2car()
{
	Matrix<double, 4, 4> xsens2board;
	xsens2board = pose6d_to_matrix(0.175, -0.01, 0.25, 0, 0, 0);
	return _board2car() * xsens2board;
}


Eigen::Matrix<double, 4, 4>
NewCarmenDataset::_board2car()
{
	Matrix<double, 4, 4> board2car;
	board2car = pose6d_to_matrix(0.572, 0, 1.394, 0.0, 0.0122173048, 0.0);
	return board2car;
}


void
NewCarmenDataset::_load_log(std::string &path)
{
	printf("Loading log '%s'\n", path.c_str());
	FILE *fptr = safe_fopen(path.c_str(), "r");

	_read_log_msgs(fptr);

	/*
	DataSample *sample;

	while ((sample = _next_data_package(fptr)))
		_data.push_back(sample);

	 */
	_clear_synchronization_queues();

	fclose(fptr);
}


void 
NewCarmenDataset::_load_odometry_calibration(std::string &path)
{
	printf("Loading odometry calibration '%s'\n", path.c_str());

	int success = false;

	if (path.size() > 0)
	{
		FILE *f = fopen(path.c_str(), "r");

		if (f != NULL)
		{
			int n = fscanf(f, "bias v: %lf %lf bias phi: %lf %lf Initial Angle: %lf",
										 &_calib.mult_v,
										 &_calib.add_v,
										 &_calib.mult_phi,
										 &_calib.add_phi,
										 &_calib.init_angle);

			if (n == 5)
				success = true;

			fclose(f);
		}
	}

	if (!success)
	{
		fprintf(stderr, "Warning: fail to load odometry calibration from '%s'. Assuming default values.\n", path.c_str());

		_calib.mult_phi = _calib.mult_v = 1.0;
		_calib.add_phi = _calib.add_v = 0.;
		_calib.init_angle = 0.;
	}

	printf("Odom calibration: bias v: %lf %lf bias phi: %lf %lf\n", 
				 _calib.mult_v, _calib.add_v, _calib.mult_phi, _calib.add_phi);
}


void
NewCarmenDataset::_load_poses(std::string &path, std::vector<Pose2d> *poses)
{
	bool success = false;
	char dummy[64];

	printf("Loading poses from '%s'\n", path.c_str());

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
		else
		{
			fprintf(stderr, "Warning: failed to open file '%s'\n", path.c_str());
		}
	}

	if (!success)
	{
		fprintf(stderr, "Warning: failed to load poses from '%s'. Returning default values.\n", path.c_str());
		poses->clear();
	}
}


void
NewCarmenDataset::_update_data_with_poses()
{
	for (int i = 0; i < size(); i++)
	{
		if (i < _fused_odom.size()) at(i)->pose = _fused_odom[i];
		else at(i)->pose = Pose2d(0, 0, 0);
	}
}


unsigned char***
NewCarmenDataset::_allocate_calibration_table()
{
	unsigned char ***table = (unsigned char ***) calloc(32, sizeof(unsigned char **));

	for (int i = 0; i < 32; i++)
	{
		table[i] = (unsigned char **) calloc(10, sizeof(unsigned char *));

		for (int j = 0; j < 10; j++)
		{
			table[i][j] = (unsigned char *) calloc(256, sizeof(unsigned char));

			// assign default values.
			for (int k = 0; k < 256; k++)
				table[i][j][k] = (unsigned char) k;
		}
	}

	return table;
}


void 
NewCarmenDataset::_free_calibration_table(unsigned char ***table)
{
	for (int i = 0; i < 32; i++)
	{
		for (int j = 0; j < 10; j++)
			free(table[i][j]);

		free(table[i]);
	}

	free(table);
}


void
NewCarmenDataset::_load_intensity_calibration(std::string &path)
{
	printf("Loading intensity calibration '%s'\n", path.c_str());
	FILE *calibration_file_bin = fopen(path.c_str(), "r");

	if (calibration_file_bin == NULL)
	{
		fprintf(stderr, "Warning: failed to read intensity calibration file '%s'. Using uncalibrated values.\n", path.c_str());
		return;
	}

	int laser, ray_size, intensity;
	long accumulated_intennsity, count;
	float val, max_val = 0.0, min_val = 255.0;

	while (fscanf(calibration_file_bin, "%d %d %d %f %ld %ld", &laser, &ray_size, &intensity, &val, &accumulated_intennsity, &count) == 6)
	{
		intensity_calibration[laser][ray_size][intensity] = (uchar) val;

		if (val > max_val)
			max_val = val;

		if (val < min_val)
			min_val = val;
	}

	for (int i = 0; i < 32; i++)
	{
		for (int j = 0; j < 10; j++)
		{
			for (int k = 0; k < 256; k++)
			{
				val = intensity_calibration[i][j][k];
				val = (val - min_val) / (max_val - min_val);

				if (val > 1.0)
					val = 1.0;

				if (val < 0.0)
					val = 0.0;

				intensity_calibration[i][j][k] = (uchar) (val * 255.);
			}
		}
	}

	fclose(calibration_file_bin);
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
}


void 
NewCarmenDataset::_add_message_to_queue(string line)
{
	// ignore small lines
	if (line.size() > 10)
	{
		const char *cline = line.c_str();
		vector<string> splitted = string_split(line, " ");

		if (splitted.size() < 4)
			return;

		double time = atof(splitted[splitted.size() - 3].c_str());

		if (!strncmp("NMEAGGA", cline, strlen("NMEAGGA")) && (cline[strlen("NMEAGGA") + 1] - '0') == _gps_id)
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
		//splitted = string_split(queue[i], " ");
		//msg_time = atof(splitted[splitted.size() - 3].c_str());
		msg_time = times[i];

		if (fabs(msg_time - ref_time) < fabs(nearest_time - ref_time))
		{
			id = i;
			nearest_time = msg_time;
			//most_sync = vector<string>(splitted);
		}
	}

	return string_split(queue[id], " "); //most_sync;
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

	sample->gps.x = utm.y;
	sample->gps.y = -utm.x;

	sample->gps_quality = atoi(data[7].c_str());
	sample->gps_time = atof(data[data.size() - 3].c_str());
}


void 
NewCarmenDataset::_parse_gps_orientation(vector<string> data, DataSample *sample)
{
	sample->gps.th = atof(data[2].c_str());
	sample->gps_orientation_quality = atoi(data[3].c_str());
}


void
NewCarmenDataset::_read_log_msgs(FILE *fptr)
{
	static char line[_MAX_LINE_LENGTH];

	_clear_synchronization_queues();

	while (!feof(fptr))
	{
		fscanf(fptr, "\n%[^\n]\n", line);

		string copy_as_string = string(line);
		_add_message_to_queue(copy_as_string);
	}

	assert(_velodyne_messages.size() > 0);
	assert(_odom_messages.size() > 0);
	assert(_gps_position_messages.size() > 0);
	assert(_gps_orientation_messages.size() > 0);

	for (int i = 0; i < _camera_messages.size(); i++)
	{
		vector<string> msg_splitted;
		msg_splitted = string_split(_camera_messages[i], " ");

		int time_field = msg_splitted.size() - 3;
		double ref_time = atof(msg_splitted[time_field].c_str());

		DataSample *sample = new DataSample;

		_parse_velodyne(_find_nearest(_velodyne_messages, _velodyne_times, ref_time), sample, _velodyne_dir);
		_parse_odom(_find_nearest(_odom_messages, _odom_times, ref_time), sample);
		_parse_imu(_find_nearest(_imu_messages, _imu_times, ref_time), sample);
		_parse_gps_position(_find_nearest(_gps_position_messages, _gps_position_times, ref_time), sample);
		_parse_gps_orientation(_find_nearest(_gps_orientation_messages, _gps_orientation_times, ref_time), sample);
		_parse_camera(msg_splitted, sample, _images_dir);

		sample->v = sample->v * _calib.mult_v + _calib.add_v;
		sample->phi = normalize_theta(sample->phi * _calib.mult_phi + _calib.add_phi);

		_data.push_back(sample);
	}
}


DataSample*
NewCarmenDataset::_next_data_package(FILE *fptr)
{
	int all_sensors_were_received;
	static char line[_MAX_LINE_LENGTH];

	_clear_synchronization_queues();
	all_sensors_were_received = 0;

	while (!all_sensors_were_received)
	{
		if (feof(fptr))
			break;

		fscanf(fptr, "\n%[^\n]\n", line);

		string copy_as_string = string(line);
		_add_message_to_queue(copy_as_string);

		if ((_velodyne_messages.size() > 0) && (_imu_messages.size() > 0) &&
				(_odom_messages.size() > 0) && (_gps_position_messages.size() > 0) &&
				(_gps_orientation_messages.size() > 0))
		{
			// to support logs that do not contain camera data, we ignore
			// the need for camera data when synchronizing by velodyne.
			if (_sync_sensor == SYNC_BY_VELODYNE || _camera_messages.size() > 0)
				all_sensors_were_received = 1;
		}
	}

	// a data package could not be created because the log is over.
	if (!all_sensors_were_received) 
		return NULL;

	return _assemble_data_package_from_queues();
}


DataSample*
NewCarmenDataset::_assemble_data_package_from_queues()
{
	return NULL;
//
//	int time_field;
//	double ref_time;
//	vector<string> msg_splitted;
//	DataSample *sample = new DataSample;
//
//	if (_sync_type == SYNC_BY_CAMERA)
//		msg_splitted = string_split(_camera_messages[_camera_messages.size() - 1], " ");
//	else if (_sync_type == SYNC_BY_VELODYNE)
//		msg_splitted = string_split(_velodyne_messages[_velodyne_messages.size() - 1], " ");
//	else
//		exit(printf("Error: invalid sync type: '%d'\n", _sync_type));
//
//	time_field = msg_splitted.size() - 3;
//	ref_time = atof(msg_splitted[time_field].c_str());
//
//	_parse_velodyne(_find_nearest(_velodyne_messages, ref_time), sample, _velodyne_dir);
//	_parse_odom(_find_nearest(_odom_messages, ref_time), sample);
//	_parse_imu(_find_nearest(_imu_messages, ref_time), sample);
//	_parse_gps_position(_find_nearest(_gps_position_messages, ref_time), sample);
//	_parse_gps_orientation(_find_nearest(_gps_orientation_messages, ref_time), sample);
//
//	// to support logs that do not contain camera data, we ignore
//	// the need for camera data when synchronizing by velodyne.
//	if (_sync_type == SYNC_BY_VELODYNE || _camera_messages.size() > 0)
//		_parse_camera(_find_nearest(_camera_messages, ref_time), sample, _images_dir);
//
//	sample->v = sample->v * _calib.mult_v + _calib.add_v;
//	sample->phi = normalize_theta(sample->phi * _calib.mult_phi + _calib.add_phi);
//
//	return sample;
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
