#include <stdio.h>

#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/lane_analysis_interface.h> // implement if needed
#include <carmen/lane_analysis_messages.h>  // implement when needed

#include <opencv2/opencv.hpp>
#include "ELAS/ELAS.h"

#define SHOW_DISPLAY
// #define TEST_MESSAGES_PUBLISH

using namespace cv;
using namespace std;


enum CameraSide {
	LEFT = 0,
	RIGHT = 1
};

// camera
static int camera = 8;
static int camera_side = CameraSide::RIGHT;
static int image_width;
static int image_height;

// lane analysis
static string CARMEN_HOME;
static ELAS::raw_elas_message _raw_elas_message;
static carmen_elas_lane_analysis_message lane_analysis_message;

// car pose
static carmen_pose_3D_t *car_pose = nullptr;

// TODO: incorporate into ELAS
static double scale_x = 0.076; // default: 1px -> 1m (1px -> x meters)
static double scale_y = 0.25; // default: 1px -> 1m (1px -> y meters)

carmen_vector_2D_t toVector2D(const Point2d &p) {
	carmen_vector_2D_t p_carmen;
	p_carmen.x = p.x;
	p_carmen.y = p.y;
	return p_carmen;
}

// calculates the real world position of the estimated lane positions
// and add it to a trail that will be drawn
void ipm_to_world_coordinates(vector<carmen_vector_2D_t> &left_ipm, vector<carmen_vector_2D_t> &right_ipm, double _lane_width, double lane_deviation,
		vector<carmen_vector_3D_t> *left_world, vector<carmen_vector_3D_t> *right_world) {

	// set the distance from the pose to the visible region on the camera
	const int shift = 7;
	const double lane_width = _lane_width * scale_x;
	const double deviation = lane_deviation;

	// calculates the distance of the left lane markings from the car center
	const double dist_left = (1 + deviation) * (lane_width / 2.0);
	const double dist_right = (1 - deviation) * (lane_width / 2.0);

	// calculates the unit vector of the car orientation
	carmen_vector_3D_t car_pos = car_pose->position;
	cv::Point2d _orientation_unit;
	const double _theta = car_pose->orientation.yaw;
	_orientation_unit.x = cos(_theta);
	_orientation_unit.y = sin(_theta);
	_orientation_unit *= 1.0/cv::norm(_orientation_unit); // convert to unit vector

	// calculates the unit vector orthogonal to the car orientation, to draw lane position
	carmen_vector_2D_t _orthogonal_unit;
	_orthogonal_unit.x = -1 * _orientation_unit.y;
	_orthogonal_unit.y = _orientation_unit.x;

	// left point
	carmen_vector_3D_t _left;
	_left.x = car_pos.x + shift * _orientation_unit.x + dist_left * _orthogonal_unit.x;
	_left.y = car_pos.y + shift * _orientation_unit.y + dist_left * _orthogonal_unit.y;
	_left.z = car_pos.z;

	// right point
	carmen_vector_3D_t _right;
	_right.x = car_pos.x + shift * _orientation_unit.x - dist_right * _orthogonal_unit.x;
	_right.y = car_pos.y + shift * _orientation_unit.y - dist_right * _orthogonal_unit.y;
	_right.z = car_pos.z;

	// calculate the points ahead
	// they will be calculated based on their relative distances to the bottom points

	// first point is the bottom one
	left_world->push_back(_left); // first point is the bottom one
	for (int i = 1; i < (int)left_ipm.size(); i++) {
		// calculates the difference to the base point in pixels
		carmen_vector_2D_t _diff;
		_diff.x = left_ipm[0].x - left_ipm[i].x;
		_diff.y = left_ipm[0].y - left_ipm[i].y;

		// convert to meters
		_diff.x *= scale_x;
		_diff.y *= scale_y; // message->scale_y;

		// calculates the point position given the relative distance
		carmen_vector_3D_t _point;
		_point.x = (*left_world)[0].x + _diff.y * _orientation_unit.x + _diff.x * _orthogonal_unit.x;
		_point.y = (*left_world)[0].y + _diff.y * _orientation_unit.y + _diff.x * _orthogonal_unit.y;
		_point.z = (*left_world)[0].z;

		// add the point to the vector
		left_world->push_back(_point);
	}

	// first point is the bottom one
	right_world->push_back(_right);
	for (int i = 1; i < (int)right_ipm.size(); i++) {
		// calculates the difference to the base point in pixels
		carmen_vector_2D_t _diff;
		_diff.x = right_ipm[0].x - right_ipm[i].x;
		_diff.y = right_ipm[0].y - right_ipm[i].y;

		// convert to meters
		_diff.x *= scale_x;
		_diff.y *= scale_y; // message->scale_y;

		// calculates the point position given the relative distance
		carmen_vector_3D_t _point;
		_point.x = (*right_world)[0].x + _diff.y * _orientation_unit.x + _diff.x * _orthogonal_unit.x;
		_point.y = (*right_world)[0].y + _diff.y * _orientation_unit.y + _diff.x * _orthogonal_unit.y;
		_point.z = (*right_world)[0].z;

		// add the point to the vector
		right_world->push_back(_point);
	}
}

void lane_analysis_publish_messages(double _timestamp) {
	// stamps the message
	lane_analysis_message.host = carmen_get_host();
	lane_analysis_message.timestamp = _timestamp;

	// both sides must have the same number of points
	lane_analysis_message.num_control_points = (int)_raw_elas_message.lane_position.left.size();

	// TODO: if there is no points, return something more useful
	if (lane_analysis_message.num_control_points == 0) return;

	// get the points on the IPM
	ConfigXML * _cfg = ELAS::getConfigXML();
	vector<carmen_vector_2D_t> pos_left_ipm, pos_right_ipm;
	Point2d _tl = Point2d(_cfg->roi.tl().x, _cfg->roi.tl().y);
	for(Point2d p : _raw_elas_message.lane_position.left) pos_left_ipm.push_back(toVector2D(_cfg->ipm->applyHomography(p - _tl)));
	for(Point2d p : _raw_elas_message.lane_position.right) pos_right_ipm.push_back(toVector2D(_cfg->ipm->applyHomography(p -_tl)));
	reverse(pos_left_ipm.begin(), pos_left_ipm.end());
	reverse(pos_right_ipm.begin(), pos_right_ipm.end());

	// convert IPM points to world coordinates
	vector<carmen_vector_3D_t> left_world, right_world;
	ipm_to_world_coordinates(pos_left_ipm, pos_right_ipm, _raw_elas_message.lane_base.width, _raw_elas_message.lane_deviation, &left_world, &right_world);
	lane_analysis_message.left_control_points = left_world.data();
	lane_analysis_message.right_control_points = right_world.data();

	// lane markings type
	lane_analysis_message.left_lmt = _raw_elas_message.lmt.left;
	lane_analysis_message.right_lmt = _raw_elas_message.lmt.right;

	// publish!
	carmen_elas_lane_analysis_publish_message(&lane_analysis_message);
}

static int fnumber = 0;
void lane_analysis_handler(carmen_bumblebee_basic_stereoimage_message * stereo_image) {

	// if there is no car pose yet, return
	if (car_pose == nullptr) {
		printf("I do not see any car pose at the moment... sorry!\n");
		return;
	}

	// get the image from the bumblebee
	Mat3b image;
	if (stereo_image->image_size == 3686400) image = Mat3b(960, 1280);
	else image = Mat3b(480, 640);

	if (camera_side == CameraSide::LEFT) image.data = (uchar *) stereo_image->raw_left;
	else if(camera_side == CameraSide::RIGHT) image.data = (uchar *) stereo_image->raw_right;
	else image.data = (uchar *) stereo_image->raw_right;

	cvtColor(image, image, CV_RGB2BGR);
	cv::resize(image, image, Size(640,480));

	fnumber++;
	if (!image.empty()) {
		cout << "frame: " << fnumber << endl;
		// run ELAS
		ELAS::run(image);
		printf("CARMEN::ELAS... done!\n");

		// get the raw message
		printf("get_raw_message()... ");
		_raw_elas_message = ELAS::get_raw_message();
		_raw_elas_message.idx_frame = fnumber;
		printf("done!\n");

		// publish messages
		lane_analysis_publish_messages(stereo_image->timestamp);

#ifdef SHOW_DISPLAY
		// display viz
		ELAS::display(image, &_raw_elas_message);
#endif

	} else {
		printf("End of dataset!\n");
	}
}

static int read_parameters(int argc, char **argv) {
    int num_items;
    char bumblebee_string[256];

    sprintf(bumblebee_string, "%s%d", "bumblebee_basic", camera); // bumblebee_basic8
    carmen_param_t param_list[] = {
        { bumblebee_string, (char *) "width", CARMEN_PARAM_INT, &image_width, 0, NULL},
        { bumblebee_string, (char *) "height", CARMEN_PARAM_INT, &image_height, 0, NULL}
    };
    num_items = sizeof (param_list) / sizeof (param_list[0]);
    carmen_param_install_params(argc, argv, param_list, num_items);

    return EXIT_SUCCESS;
}

// from: http://stackoverflow.com/a/5866166/4228275
string GetEnv( const string & var ) {
	     const char * val = ::getenv( var.c_str() );
	     if ( val == 0 ) {
	         return "";
	     }
	     else {
	         return val;
	     }
	}

bool set_carmen_home() {
	CARMEN_HOME = GetEnv("CARMEN_HOME");
	return (CARMEN_HOME != "");
}

bool lane_analysis_init() {

	// init vars
	if (!set_carmen_home()) return false;
	string DATA_DIR = CARMEN_HOME + "/src/lane_analysis/data/";
	string config_xml_fname = CARMEN_HOME + "/src/lane_analysis/data/elas-iara-retadapenha.xml"; // -> elas-iara-config
	string DATASETS_DIR = ""; // just a workaround atm, ELAS was made to work with offline datasets

    ELAS::init(DATASETS_DIR, DATA_DIR, config_xml_fname);

#ifdef TEST_OFFLINE_DATASET_AVOID
    // read calibration params from file
    FileStorage fs("data/elas_calibration.xml", FileStorage::READ);
	Size origSize, dstSize;
	vector<Point2f> origPts, dstPts;
	fs["origSize"] >> origSize;
	fs["dstSize"] >> dstSize;
	fs["calibration_points"] >> origPts;
	fs["dst_points"] >> dstPts;
	fs["scale_x"] >> scale_x;
	fs["scale_y"] >> scale_y;

	// overwrite IPM config
	ELAS::setIPM(origSize, dstSize, origPts, dstPts);
#endif

	return true;
}

void localize_ackerman_handler(carmen_localize_ackerman_globalpos_message* localize_ackerman_message) {
	car_pose = &(localize_ackerman_message->pose);
}

// subscribes: what does this module need?
void subscribe_elas_messages() {
    carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) lane_analysis_handler, CARMEN_SUBSCRIBE_LATEST);
    carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_handler, CARMEN_SUBSCRIBE_LATEST);
}

static void register_ipc_messages(void) {
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_ELAS_LANE_ANALYSIS_NAME, IPC_VARIABLE_LENGTH, CARMEN_ELAS_LANE_ANALYSIS_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_ELAS_LANE_ANALYSIS_NAME);
}

int main(int argc, char ** argv) {
	// connect to IPC server
	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

    // read the params (camera and calibration params from config file)
	read_parameters(argc, argv);

    // define the messages
	register_ipc_messages();

	IPC_defineFormat("vector_2D", "{double,double}");
	IPC_defineFormat("vector_3D", "{double,double,double}");

	// subscribes
	subscribe_elas_messages();

	if (!lane_analysis_init()) {
		printf("Lane Analysis module could not init!\n");
		return EXIT_FAILURE;
	}

	// Loop forever waiting for messages
	carmen_ipc_dispatch();

    ELAS::finishProgram();

	return EXIT_SUCCESS;
}
