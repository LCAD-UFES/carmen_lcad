/*
 *
 * road_mapper_log_tracker -f0 -f1 maplist.txt
 */

#include <carmen/carmen.h>
//#include <carmen/readlog.h>
#include <vector>
#include <string>
#include <algorithm>
using namespace std;
FILE * open_file(char *fname);

char *log_filename = NULL;

// logger/playback.c
carmen_logfile_index_p
load_logindex_file(char *index_filename)
{
	FILE *index_file = open_file(index_filename);
	carmen_logfile_index_p logfile_index = (carmen_logfile_index_p) malloc(sizeof(carmen_logfile_index_t));
	carmen_test_alloc(logfile_index);
	fread(logfile_index, sizeof(carmen_logfile_index_t), 1, index_file);
	// carmen_logfile_index_messages() (in readlog.c) set file size as last offset
	// so, offset array contains one element more than messages
	// it is required by carmen_logfile_read_line to read the last line
	logfile_index->offset = (off_t *) malloc((logfile_index->num_messages + 1) * sizeof(off_t));
	carmen_test_alloc(logfile_index->offset);
	fread(logfile_index->offset, sizeof(off_t), logfile_index->num_messages + 1, index_file);
	logfile_index->current_position = 0;
	fclose(index_file);
	return logfile_index;
}


// logger/playback.c
struct carmen_logger_param_message
{
	char *module_variable;
	char *value;
	double timestamp;
	char *host;
} logger_param;


char *
carmen_string_to_logger_param_message(char *string,	carmen_logger_param_message *logger_param)
{
	char *current_pos = string;
	copy_host_string(&logger_param->module_variable, &current_pos);
	copy_host_string(&logger_param->value, &current_pos);
	logger_param->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&logger_param->host, &current_pos);
	return current_pos;
}


carmen_laser_ldmrs_message laser_ldmrs;
carmen_laser_ldmrs_new_message laser_ldmrs_new;
carmen_laser_ldmrs_objects_message laser_ldmrs_objects;
carmen_laser_ldmrs_objects_data_message laser_ldmrs_objects_data;
carmen_laser_laser_message rawlaser1, rawlaser2, rawlaser3, rawlaser4, rawlaser5;
carmen_robot_ackerman_laser_message laser_ackerman1, laser_ackerman2, laser_ackerman3, laser_ackerman4, laser_ackerman5;
carmen_base_ackerman_odometry_message odometry_ackerman;
carmen_robot_ackerman_velocity_message velocity_ackerman;
carmen_visual_odometry_pose6d_message visual_odometry;
carmen_simulator_ackerman_truepos_message truepos_ackerman;
carmen_imu_message imu;
carmen_gps_gpgga_message gpsgga;
carmen_gps_gphdt_message gpshdt;
carmen_gps_gprmc_message gpsrmc;
carmen_kinect_depth_message raw_depth_kinect_0, raw_depth_kinect_1;
carmen_kinect_video_message raw_video_kinect_0, raw_video_kinect_1;
carmen_velodyne_partial_scan_message velodyne_partial_scan;
carmen_velodyne_variable_scan_message velodyne_variable_scan;
carmen_velodyne_gps_message velodyne_gps;
carmen_xsens_global_euler_message xsens_euler;
carmen_xsens_global_quat_message xsens_quat;
carmen_xsens_global_matrix_message xsens_matrix;
carmen_xsens_mtig_message xsens_mtig;
carmen_bumblebee_basic_stereoimage_message bumblebee_basic_stereoimage1, bumblebee_basic_stereoimage2, bumblebee_basic_stereoimage3, bumblebee_basic_stereoimage4, bumblebee_basic_stereoimage5, bumblebee_basic_stereoimage6, bumblebee_basic_stereoimage7, bumblebee_basic_stereoimage8, bumblebee_basic_stereoimage9;
carmen_web_cam_message web_cam_message;
carmen_base_ackerman_motion_command_message ackerman_motion_message;
carmen_ultrasonic_sonar_sensor_message ultrasonic_message;
carmen_ford_escape_status_message ford_escape_status;

typedef char *(*converter_func)(char *, void *);

typedef struct {
	char const *logger_message_name;
	char const *ipc_message_name;
	converter_func conv_func;
	void *message_data;
	int interpreted;
	double *timestamp;
} logger_callback_timestamp_t;


logger_callback_timestamp_t logger_callbacks[] =
{
	{"PARAM", "carmen_logger_param", (converter_func) carmen_string_to_logger_param_message, &logger_param, 0, &logger_param.timestamp},
	{"LASER_LDMRS", CARMEN_LASER_LDMRS_NAME, (converter_func) carmen_string_to_laser_ldmrs_message, &laser_ldmrs, 0, &laser_ldmrs.timestamp},
	{"LASER_LDMRS_NEW", CARMEN_LASER_LDMRS_NEW_NAME, (converter_func) carmen_string_to_laser_ldmrs_new_message, &laser_ldmrs_new, 0, &laser_ldmrs_new.timestamp},
	{"LASER_LDMRS_OBJECTS", CARMEN_LASER_LDMRS_OBJECTS_NAME, (converter_func) carmen_string_to_laser_ldmrs_objects_message, &laser_ldmrs_objects, 0, &laser_ldmrs_objects.timestamp},
	{"LASER_LDMRS_OBJECTS_DATA", CARMEN_LASER_LDMRS_OBJECTS_DATA_NAME, (converter_func) carmen_string_to_laser_ldmrs_objects_data_message, &laser_ldmrs_objects_data, 0, &laser_ldmrs_objects_data.timestamp},
	{"RAWLASER1", CARMEN_LASER_FRONTLASER_NAME, (converter_func) carmen_string_to_laser_laser_message, &rawlaser1, 0, &rawlaser1.timestamp},
	{"RAWLASER2", CARMEN_LASER_REARLASER_NAME, (converter_func) carmen_string_to_laser_laser_message, &rawlaser2, 0, &rawlaser2.timestamp},
	{"RAWLASER3", CARMEN_LASER_LASER3_NAME, (converter_func) carmen_string_to_laser_laser_message, &rawlaser3, 0, &rawlaser3.timestamp},
	{"RAWLASER4", CARMEN_LASER_LASER4_NAME, (converter_func) carmen_string_to_laser_laser_message, &rawlaser4, 0, &rawlaser4.timestamp},
	{"RAWLASER5", CARMEN_LASER_LASER5_NAME, (converter_func) carmen_string_to_laser_laser_message, &rawlaser5, 0, &rawlaser5.timestamp},
	{"ROBOTLASER_ACK1", CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME, (converter_func) carmen_string_to_robot_ackerman_laser_message, &laser_ackerman1, 0, &laser_ackerman1.timestamp},
	{"ROBOTLASER_ACK2", CARMEN_ROBOT_ACKERMAN_REARLASER_NAME, (converter_func) carmen_string_to_robot_ackerman_laser_message, &laser_ackerman2, 0, &laser_ackerman2.timestamp},
	{"ROBOTLASER_ACK3", CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME, (converter_func) carmen_string_to_robot_ackerman_laser_message, &laser_ackerman3, 0, &laser_ackerman3.timestamp},
	{"ROBOTLASER_ACK4", CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME, (converter_func) carmen_string_to_robot_ackerman_laser_message, &laser_ackerman4, 0, &laser_ackerman4.timestamp},
	{"ROBOTLASER_ACK5", CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME, (converter_func) carmen_string_to_robot_ackerman_laser_message, &laser_ackerman5, 0, &laser_ackerman5.timestamp},
	{"ODOM_ACK", CARMEN_BASE_ACKERMAN_ODOMETRY_NAME, (converter_func) carmen_string_to_base_ackerman_odometry_message, &odometry_ackerman, 0, &odometry_ackerman.timestamp},
	{"ROBOTVELOCITY_ACK", CARMEN_ROBOT_ACKERMAN_VELOCITY_NAME, (converter_func) carmen_string_to_robot_ackerman_velocity_message, &velocity_ackerman, 0, &velocity_ackerman.timestamp},
	{"VISUAL_ODOMETRY", CARMEN_VISUAL_ODOMETRY_POSE6D_MESSAGE_NAME, (converter_func) carmen_string_to_visual_odometry_message, &visual_odometry, 0, &visual_odometry.timestamp},
	{"TRUEPOS_ACK", CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_NAME, (converter_func) carmen_string_to_simulator_ackerman_truepos_message, &truepos_ackerman, 0, &truepos_ackerman.timestamp},
	{"IMU", CARMEN_IMU_MESSAGE_NAME, (converter_func) carmen_string_to_imu_message, &imu, 0, &imu.timestamp},
	{"NMEAGGA", CARMEN_GPS_GPGGA_MESSAGE_NAME, (converter_func) carmen_string_to_gps_gpgga_message, &gpsgga, 0, &gpsgga.timestamp},
	{"NMEAHDT", CARMEN_GPS_GPHDT_MESSAGE_NAME, (converter_func) carmen_string_to_gps_gphdt_message, &gpshdt, 0, &gpshdt.timestamp},
	{"NMEARMC", CARMEN_GPS_GPRMC_MESSAGE_NAME, (converter_func) carmen_string_to_gps_gprmc_message, &gpsrmc, 0, &gpsrmc.timestamp},
	{"RAW_KINECT_DEPTH0", CARMEN_KINECT_DEPTH_MSG_0_NAME, (converter_func) carmen_string_to_kinect_depth_message, &raw_depth_kinect_0, 0, &raw_depth_kinect_0.timestamp},
	{"RAW_KINECT_DEPTH1", CARMEN_KINECT_DEPTH_MSG_1_NAME, (converter_func) carmen_string_to_kinect_depth_message, &raw_depth_kinect_1, 0, &raw_depth_kinect_1.timestamp},
	{"RAW_KINECT_VIDEO0", CARMEN_KINECT_VIDEO_MSG_0_NAME, (converter_func) carmen_string_to_kinect_video_message, &raw_video_kinect_0, 0, &raw_video_kinect_0.timestamp},
	{"RAW_KINECT_VIDEO1", CARMEN_KINECT_VIDEO_MSG_1_NAME, (converter_func) carmen_string_to_kinect_video_message, &raw_video_kinect_1, 0, &raw_video_kinect_1.timestamp},
	{"VELODYNE_PARTIAL_SCAN", CARMEN_VELODYNE_PARTIAL_SCAN_MESSAGE_NAME, (converter_func) carmen_string_to_velodyne_partial_scan_message, &velodyne_partial_scan, 0, &velodyne_partial_scan.timestamp},
	{"VELODYNE_PARTIAL_SCAN_IN_FILE", CARMEN_VELODYNE_PARTIAL_SCAN_MESSAGE_NAME, (converter_func) carmen_string_and_file_to_velodyne_partial_scan_message, &velodyne_partial_scan, 0, &velodyne_partial_scan.timestamp},
	{"VARIABLE_VELODYNE_SCAN", "carmen_stereo_velodyne_scan_message8", (converter_func) carmen_string_to_variable_velodyne_scan_message, &velodyne_variable_scan, 0, &velodyne_variable_scan.timestamp},
	{"VELODYNE_GPS", CARMEN_VELODYNE_GPS_MESSAGE_NAME, (converter_func) carmen_string_to_velodyne_gps_message, &velodyne_gps, 0, &velodyne_gps.timestamp},
	{"XSENS_EULER", CARMEN_XSENS_GLOBAL_EULER_NAME, (converter_func) carmen_string_to_xsens_euler_message, &xsens_euler, 0, &xsens_euler.timestamp},
	{"XSENS_QUAT", CARMEN_XSENS_GLOBAL_QUAT_NAME, (converter_func) carmen_string_to_xsens_quat_message, &xsens_quat, 0, &xsens_quat.timestamp},
	{"XSENS_MATRIX", CARMEN_XSENS_GLOBAL_MATRIX_NAME, (converter_func) carmen_string_to_xsens_matrix_message, &xsens_matrix, 0, &xsens_matrix.timestamp},
	{"XSENS_MTIG", CARMEN_XSENS_MTIG_NAME, (converter_func) carmen_string_to_xsens_mtig_message, &xsens_mtig, 0, &xsens_mtig.timestamp},
	{"BUMBLEBEE_BASIC_STEREOIMAGE1", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE1_NAME, (converter_func) carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage1, 0, &bumblebee_basic_stereoimage1.timestamp},
	{"BUMBLEBEE_BASIC_STEREOIMAGE2", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE2_NAME, (converter_func) carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage2, 0, &bumblebee_basic_stereoimage2.timestamp},
	{"BUMBLEBEE_BASIC_STEREOIMAGE3", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE3_NAME, (converter_func) carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage3, 0, &bumblebee_basic_stereoimage3.timestamp},
	{"BUMBLEBEE_BASIC_STEREOIMAGE4", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE4_NAME, (converter_func) carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage4, 0, &bumblebee_basic_stereoimage4.timestamp},
	{"BUMBLEBEE_BASIC_STEREOIMAGE5", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE5_NAME, (converter_func) carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage5, 0, &bumblebee_basic_stereoimage5.timestamp},
	{"BUMBLEBEE_BASIC_STEREOIMAGE6", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE6_NAME, (converter_func) carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage6, 0, &bumblebee_basic_stereoimage6.timestamp},
	{"BUMBLEBEE_BASIC_STEREOIMAGE7", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE7_NAME, (converter_func) carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage7, 0, &bumblebee_basic_stereoimage7.timestamp},
	{"BUMBLEBEE_BASIC_STEREOIMAGE8", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE8_NAME, (converter_func) carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage8, 0, &bumblebee_basic_stereoimage8.timestamp},
	{"BUMBLEBEE_BASIC_STEREOIMAGE9", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE9_NAME, (converter_func) carmen_string_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage9, 0, &bumblebee_basic_stereoimage9.timestamp},
	{"BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE1", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE1_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage1, 0, &bumblebee_basic_stereoimage1.timestamp},
	{"BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE2", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE2_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage2, 0, &bumblebee_basic_stereoimage2.timestamp},
	{"BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE3", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE3_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage3, 0, &bumblebee_basic_stereoimage3.timestamp},
	{"BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE4", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE4_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage4, 0, &bumblebee_basic_stereoimage4.timestamp},
	{"BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE5", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE5_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage5, 0, &bumblebee_basic_stereoimage5.timestamp},
	{"BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE6", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE6_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage6, 0, &bumblebee_basic_stereoimage6.timestamp},
	{"BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE7", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE7_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage7, 0, &bumblebee_basic_stereoimage7.timestamp},
	{"BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE8", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE8_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage8, 0, &bumblebee_basic_stereoimage8.timestamp},
	{"BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE9", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE9_NAME, (converter_func) carmen_string_and_file_to_bumblebee_basic_stereoimage_message, &bumblebee_basic_stereoimage9, 0, &bumblebee_basic_stereoimage9.timestamp},
	{"WEB_CAM_IMAGE", CARMEN_WEB_CAM_MESSAGE_NAME, (converter_func) carmen_string_to_web_cam_message, &web_cam_message, 0, &web_cam_message.timestamp},
	{"BASEMOTION_ACK", CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME, (converter_func) carmen_string_to_base_ackerman_motion_message, &ackerman_motion_message, 0, &ackerman_motion_message.timestamp},
	{"ULTRASONIC_SONAR_SENSOR", CARMEN_ULTRASONIC_SONAR_SENSOR_NAME, (converter_func) carmen_string_to_ultrasonic_message, &ultrasonic_message, 0, &ultrasonic_message.timestamp},
	{"FORD_ESCAPE_STATUS", CARMEN_FORD_ESCAPE_STATUS_NAME, (converter_func) carmen_string_to_ford_escape_estatus_message, &ford_escape_status, 0, &ford_escape_status.timestamp},
};


int
read_message(carmen_logfile_index_p logfile_index, carmen_FILE *log_file, int message, double *timestamp)
{
	#define MAX_LINE_LENGTH (1000000)
	static char line[MAX_LINE_LENGTH + 1];
	static double last_playback_timestamp = -1.;
	static double last_timestamp = -1.;
	double playback_timestamp = -1.;
	*timestamp = -1.;

	if (carmen_logfile_read_line(logfile_index, log_file, message, MAX_LINE_LENGTH, line) <= 0)
		return -1;
	if (line[0] == '#')
		return 0;
	char *current_pos = carmen_next_word(line);
	for(int i = 0; i < (int)(sizeof(logger_callbacks) / sizeof(logger_callback_timestamp_t)); i++)
	{
		int message_name_len = strlen(logger_callbacks[i].logger_message_name);
		if(strncmp(line, logger_callbacks[i].logger_message_name, message_name_len) == 0 && line[message_name_len] == ' ')
		{
			current_pos = logger_callbacks[i].conv_func(current_pos, logger_callbacks[i].message_data);
			playback_timestamp = atof(current_pos);
			if (playback_timestamp < last_playback_timestamp && last_playback_timestamp != last_timestamp)
			{
				fprintf(stderr, "\nLog message # %d: unexpected playback timestamp: %lf (last %lf)\n", message, playback_timestamp, last_playback_timestamp);
				fprintf(stderr, "%s\n", line);
			}
			last_playback_timestamp = playback_timestamp;
			*timestamp = *logger_callbacks[i].timestamp;
			if (fabs(*timestamp - last_timestamp) > 1. && last_timestamp > 0.)
			{
				fprintf(stderr, "\nLog message # %d: unexpected IPC timestamp: %lf (last %lf)\n", message, *timestamp, last_timestamp);
				fprintf(stderr, "%s\n", line);
			}
			last_timestamp = *timestamp;
			return 0;
		}
	}
	fprintf(stderr, "\nLog message # %d: unexpected message name:\n%s\n", message, line);
	return 0;
}


typedef pair<carmen_point_t, double> pose_rec_t;
// graphslam/graphslam_publish_main.cpp
void
graphslam_load_corrected_poses(char *pose_filename, vector<pose_rec_t> &poses)
{
	carmen_point_t point;
	double timestamp;
	FILE *graphslam_file = open_file(pose_filename);

	while(!feof(graphslam_file))
	{
		if(fscanf(graphslam_file, "%lf %lf %lf %lf\n",
			&point.x, &point.y, &point.theta, &timestamp) == 4)
			poses.push_back(pose_rec_t(point, timestamp));
		else
			fprintf(stderr, "Error in pose file %s: could not read record # %ld in expected format: <x> <y> <theta> <timestamp>\n", pose_filename, poses.size());
	}
	fclose(graphslam_file);
	return;
}


int
find_pose_by_timestamp(double timestamp, vector<pose_rec_t> &poses)
{
	#define pose_cmp(t1, t2) ((t1 < t2) ? -1 : (t1 > t2))

	int index_low = 0;
	int sign_low = pose_cmp(poses[index_low].second, timestamp);
	if (sign_low == 0)
		return index_low;

	int index_high = poses.size() - 1;
	int sign_high = pose_cmp(poses[index_high].second, timestamp);
	if (sign_high == 0)
		return index_high;

	if (sign_low > 0 || sign_high < 0)
		return -1;

	// Binary search
    while ((index_high - index_low) > 1)
    {
    	int index_mid = (index_low + index_high) / 2;
    	int sign_mid = pose_cmp(poses[index_mid].second, timestamp);
    	if (sign_mid == 0)
    		return index_mid;
    	if (sign_mid < 0)
        	index_low = index_mid;
    	else
        	index_high = index_mid;
    }
    // Return the pose index which has the nearest timestamp below.
    return index_low;
}


struct map_rec_t
{
	carmen_point_t point;
	string name;
	int play_message, stop_message;
	map_rec_t(const carmen_point_t p, const string& s) : point(p), name(s), play_message(-1), stop_message(-1) {}
    bool operator< (const map_rec_t &rec) const
    {
    	if(point.x != rec.point.x)
            return (point.x < rec.point.x);
    	return (point.y < rec.point.y);
    }
};


bool
order_by_message(const map_rec_t &rec1, const map_rec_t &rec2)
{
	return ((unsigned int) rec1.play_message < (unsigned int) rec2.play_message);
}


void
load_maplist(char *map_list_filename, vector<map_rec_t> &maps)
{
	carmen_point_t point;
	char map_path[2000];
	FILE *map_list_file = open_file(map_list_filename);

	while(!feof(map_list_file))
	{
		map_path[0] = '\0';
		fscanf(map_list_file, "%s\n", map_path);
		size_t i = string(map_path).rfind('/');
		if(sscanf(&map_path[i+1], "%*c%lf_%lf.%*s", &point.x, &point.y) == 2)
			maps.push_back(map_rec_t(point, string(map_path)));
		else
			fprintf(stderr, "Error in map file list %s: could not read record # %ld in expected format: <type><x>_<y>.<ext>\n", map_list_filename, maps.size());
	}
	fclose(map_list_file);

	sort(maps.begin(), maps.end());
	vector<map_rec_t>::iterator last_pos = maps.begin(), pos = maps.begin();
	if(!maps.empty())
		pos++;
	while(pos != maps.end())
	{
		if((*pos).point.x == (*last_pos).point.x && (*pos).point.y == (*last_pos).point.y)
		{
			fprintf(stderr, "Error in map file list %s: discarding duplicate record: %s\n", map_list_filename, (*pos).name.c_str());
			pos = maps.erase(pos);
		}
		else
		{
			last_pos = pos;
			pos++;
		}
	}
	return;
}


int
find_map_by_pose(carmen_point_t point, vector<map_rec_t> &maps, int map_x_size, int map_y_size)
{
	#define map_cmp(map_x, map_y, x, y) (((map_x + map_x_size) <= x) ? -1 : (map_x > x) ? 1 : ((map_y + map_y_size) <= y) ? -1 : (map_y > y))

	int index_low = 0;
	int sign_low = map_cmp(maps[index_low].point.x, maps[index_low].point.y, point.x, point.y);
	if (sign_low == 0)
		return index_low;

	int index_high = maps.size() - 1;
	int sign_high = map_cmp(maps[index_high].point.x, maps[index_high].point.y, point.x, point.y);
	if (sign_high == 0)
		return index_high;

	if (sign_low > 0 || sign_high < 0)
		return -1;

	// Binary search
    while ((index_high - index_low) > 1)
    {
    	int index_mid = (index_low + index_high) / 2;
    	int sign_mid = map_cmp(maps[index_mid].point.x, maps[index_mid].point.y, point.x, point.y);
    	if (sign_mid == 0)
    		return index_mid;
    	if (sign_mid < 0)
        	index_low = index_mid;
    	else
        	index_high = index_mid;
    }
    return -1;
}


char *
get_filename(char *arg, const char **common, int n_common_files)
{
	if (strncmp(arg, "-f", 2) == 0)
	{
		int i = atoi(&arg[2]);
		if (i < 0 || i >= n_common_files)
		{
			fprintf(stderr, "\nInvalid common file index: %s (max=%d)\n", arg, n_common_files - 1);
			exit(-1);
		}
		return (char *) common[i];
	}
	return arg;
}


FILE *
open_file(char *fname)
{
	FILE *file = fopen(fname, "r");
	if (!file)
	{
		fprintf(stderr, "\nCannot open file for reading: %s\n", fname);
		exit(-1);
	}
	return file;
}


int
main(int argc, char **argv)
{
	#define MAP_X_SIZE (70) // meters
	#define MAP_Y_SIZE (70) // meters
	const char *usage = "<log_file> <pose_file> <map_list_file>";
	const char *common_files[] =
	{
		"/dados/log_guarapari-20170403-2_no-bumblebee.txt",
		"../../data/graphslam/poses_opt-log_guarapari-20170403-2.txt",
	};
	int n_files = (int) (sizeof(common_files) / sizeof(char *));

	if (argc != 4)
	{
		fprintf(stderr, "\nUsage: %s %s\nInvalid number of arguments\n", argv[0], usage);
		exit(-1);
	}
	log_filename = get_filename(argv[1], common_files, n_files);
	char *pose_filename = get_filename(argv[2], common_files, n_files);
	char *map_list_filename = get_filename(argv[3], common_files, n_files);

	char index_filename[2000];
	strcpy(index_filename, log_filename);
	strcat(index_filename, ".index");
	carmen_logfile_index_p logfile_index = load_logindex_file(index_filename);
	if(logfile_index->num_messages <= 0)
	{
		fprintf(stderr, "\nError: Log file %s: Invalid number of messages: %d\n", log_filename, logfile_index->num_messages);
		exit(-1);
	}
	printf("\nLog file %s contains %d messages.\n", log_filename, logfile_index->num_messages);

	vector<pose_rec_t> poses;
	graphslam_load_corrected_poses(pose_filename, poses);
	if(poses.size() <= 0)
	{
		fprintf(stderr, "\nError: Pose file %s: Invalid number of poses: %ld\n", pose_filename, poses.size());
		exit(-1);
	}
	printf("\nPose file %s contains %ld poses.\n", pose_filename, poses.size());

	vector<map_rec_t> maps;
	load_maplist(map_list_filename, maps);
	if(maps.size() <= 0)
	{
		fprintf(stderr, "\nError: Map list file %s: Invalid number of maps: %ld\n", map_list_filename, maps.size());
		exit(-1);
	}
	printf("\nMap list file %s contains %ld maps.\n", map_list_filename, maps.size());

	carmen_FILE *log_file = carmen_fopen(log_filename, "r");
	if(!log_file)
	{
		fprintf(stderr, "\nCannot open file for reading: %s\n", log_filename);
		exit(-1);
	}

	printf("\nLog messages:\n\n");
	enum sequence_type
	{
		LOG_EOF,
		LOG_READ_ERROR,
		NO_LOG_TIMESTAMP,
		NO_GRAPHSLAM_POSE,
		NO_MAP_FILE,
		MAP_SEQUENCE_OK,
	} sequence, last_sequence = (sequence_type) -1;
	carmen_point_t map_origin, last_map_origin;
	double timestamp, last_timestamp, sequence_timestamp, map_index_timestamp;
	int message, last_message = -1, sequence_message = 0, map_index_message = -1, pose_index, map_index, last_map_index = -1, count_files = 0;
	for(message = 0; message <= logfile_index->num_messages; message++)
	{
		timestamp = -1.;
		map_index = -1;
		map_origin.x = 0., map_origin.y = 0.;
		if(message == logfile_index->num_messages)
		{
			sequence = LOG_EOF;
		}
		else if(read_message(logfile_index, log_file, message, &timestamp))
		{
			sequence = LOG_READ_ERROR;
		}
		else if(timestamp < 0.)
		{
			sequence = NO_LOG_TIMESTAMP;
		}
		else
		{
			pose_index = find_pose_by_timestamp(timestamp, poses);
			if(pose_index < 0)
			{
				sequence = NO_GRAPHSLAM_POSE;
			}
			else
			{
				map_index = find_map_by_pose(poses[pose_index].first, maps, MAP_X_SIZE, MAP_Y_SIZE);
				if(map_index < 0)
				{
					sequence = NO_MAP_FILE;
					map_origin.x = floor(poses[pose_index].first.x / MAP_X_SIZE) * MAP_X_SIZE;
					map_origin.y = floor(poses[pose_index].first.y / MAP_Y_SIZE) * MAP_Y_SIZE;
				}
				else
				{
					sequence = MAP_SEQUENCE_OK;
				}
			}
		}

		if(message == 0)
		{
			// Start a new sequence
			last_sequence = sequence;
			sequence_message = message;
			sequence_timestamp = timestamp;
			count_files = 0;
		}
		else if(sequence != last_sequence)
		{
			// Finish the last sequence
			switch(last_sequence)
			{
				case LOG_EOF:
					break;
				case LOG_READ_ERROR:
					printf("%8d\t%8d\tFailed to read log file %s\n", sequence_message, last_message, log_filename);
					break;
				case NO_LOG_TIMESTAMP:
					printf("%8d\t%8d\tLog record does not contain a timestamp\n", sequence_message, last_message);
					break;
				case NO_GRAPHSLAM_POSE:
					printf("%8d\t%8d\t%17.6lf\t%17.6lf\tLog record timestamp is out of pose file's timestamp range\n", sequence_message, last_message, sequence_timestamp, last_timestamp);
					break;
				case NO_MAP_FILE:
					printf("%8d\t%8d\t%17.6lf\t%17.6lf\t %.0lf_%.0lf\t(map file unavailable)\n", map_index_message, last_message, map_index_timestamp, last_timestamp, last_map_origin.x, last_map_origin.y);
					break;
				case MAP_SEQUENCE_OK:
					printf("%8d\t%8d\t%17.6lf\t%17.6lf\t%s\n", map_index_message, last_message, map_index_timestamp, last_timestamp, maps[last_map_index].name.c_str());
					if(maps[last_map_index].play_message < 0)
					{
						maps[last_map_index].play_message = map_index_message;
						maps[last_map_index].stop_message = last_message;
						count_files++;
					}
					printf(">>>>> Map sequence (%d files):\t%lf\t%lf\t-play_message %d\t-stop_message %d\n", count_files, sequence_timestamp, last_timestamp, sequence_message, last_message);
					break;
			}
			// Start a new sequence
			last_sequence = sequence;
			sequence_message = message;
			sequence_timestamp = timestamp;
			count_files = 0;
		}

		if(sequence == MAP_SEQUENCE_OK)
		{
			if(message == sequence_message)
			{
				// Start a new map
				last_map_index = map_index;
				map_index_message = message;
				map_index_timestamp = timestamp;
			}
			else if(map_index != last_map_index)
			{
				// Finish the last map
				printf("%8d\t%8d\t%17.6lf\t%17.6lf\t%s\n", map_index_message, last_message, map_index_timestamp, last_timestamp, maps[last_map_index].name.c_str());
				if(maps[last_map_index].play_message < 0)
				{
					maps[last_map_index].play_message = map_index_message;
					maps[last_map_index].stop_message = last_message;
					count_files++;
				}
				// Start a new map
				last_map_index = map_index;
				map_index_message = message;
				map_index_timestamp = timestamp;
			}
		}
		else if(sequence == NO_MAP_FILE)
		{
			if(message == sequence_message)
			{
				// Start a new missing map
				last_map_origin = map_origin;
				map_index_message = message;
				map_index_timestamp = timestamp;
			}
			else if(map_origin.x != last_map_origin.x || map_origin.y != last_map_origin.y)
			{
				// Finish the last missing map
				printf("%8d\t%8d\t%17.6lf\t%17.6lf\t %.0lf_%.0lf\t(map file unavailable)\n", map_index_message, last_message, map_index_timestamp, last_timestamp, last_map_origin.x, last_map_origin.y);
				// Start a new missing map
				last_map_origin = map_origin;
				map_index_message = message;
				map_index_timestamp = timestamp;
			}
		}

		last_message = message;
		last_timestamp = timestamp;
	}
	carmen_fclose(log_file);

	printf("\n\nMaps out of log file %s:\n\n", log_filename);
	count_files = 0;
	for(map_index = 0; map_index < (int)maps.size(); map_index++)
	{
		if(maps[map_index].play_message < 0)
		{
			printf("\t%s\n", maps[map_index].name.c_str());
			count_files++;
		}
	}
	printf("\nTotal maps out of log: %d maps\n\n", count_files);

	return 0;
}
