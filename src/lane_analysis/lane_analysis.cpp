#include <stdio.h>

#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/lane_analysis_interface.h> // implement if needed
#include <carmen/lane_analysis_messages.h>  // implement when needed

#include <opencv2/opencv.hpp>

#include "ELAS/ELAS.h"

// #define TEST_OFFLINE_DATASET
// #define DATASET_GOPRO // GoPro or UFES
// #define TEST_MESSAGES_PUBLISH

using namespace cv;
using namespace std;

// camera
static int camera = 8;
static int image_width;
static int image_height;

// lane analysis
static string config_fname = "/dados/berriel/elas-carmen.config";
#ifdef DATASET_GOPRO
	static string config_xml_fname = "/dados/berriel/elas-carmen-config.xml";
#else
	static string config_xml_fname = "/dados/berriel/elas-carmen-config-log-rodrigo.xml";
#endif
static ELAS::raw_elas_message _raw_elas_message;

// messages
static carmen_elas_lane_estimation_message lane_estimation_message;
static carmen_elas_lane_markings_type_message lmt_message;
static carmen_elas_adjacent_lanes_message adj_lanes_message;

// TODO: incorporate into ELAS
static double scale_x = 0.076; // default: 1px -> 1m (1px -> x meters)
static double scale_y = 0.025; // default: 1px -> 1m (1px -> y meters)

// stamps the messages in a more readable way
template<typename T>
void stamps_message(T& message, char* _host, double _timestamp) {
	message.host = _host;
	message.timestamp = _timestamp;
}
template void stamps_message<carmen_elas_lane_estimation_message>(carmen_elas_lane_estimation_message&, char*, double);
template void stamps_message<carmen_elas_lane_markings_type_message>(carmen_elas_lane_markings_type_message&, char*, double);
template void stamps_message<carmen_elas_adjacent_lanes_message>(carmen_elas_adjacent_lanes_message&, char*, double);

carmen_vector_2D_t toVector2D(const Point2d &p) {
	carmen_vector_2D_t p_carmen;
	p_carmen.x = p.x;
	p_carmen.y = p.y;
	return p_carmen;
}

void
lane_analysis_publish_messages(double _timestamp)
{
    // get the raw message
    printf("\nget_raw_message()... ");
    _raw_elas_message = ELAS::get_raw_message();
    printf("done!\n");

    // stamps the messages
    char * _host = carmen_get_host();
    stamps_message(lane_estimation_message, _host, _timestamp);
    stamps_message(lmt_message, _host, _timestamp);
    stamps_message(adj_lanes_message, _host, _timestamp);

    // lane position and lane base message
    lane_estimation_message.num_outputs_left = (int)_raw_elas_message.lane_position.left.size();
    lane_estimation_message.num_outputs_right = (int)_raw_elas_message.lane_position.right.size();
    vector<carmen_vector_2D_t> pos_left, pos_right;
	for(Point2d p : _raw_elas_message.lane_position.left) pos_left.push_back(toVector2D(p));
	for(Point2d p : _raw_elas_message.lane_position.right) pos_right.push_back(toVector2D(p));
	reverse(pos_left.begin(), pos_left.end());
	reverse(pos_right.begin(), pos_right.end());
	lane_estimation_message.left = pos_left.data();
	lane_estimation_message.right = pos_right.data();

	ConfigXML * _cfg = ELAS::getConfigXML();
	vector<carmen_vector_2D_t> pos_left_ipm, pos_right_ipm;
	Point2d _tl = Point2d(_cfg->roi.tl().x, _cfg->roi.tl().y);
	for(Point2d p : _raw_elas_message.lane_position.left) pos_left_ipm.push_back(toVector2D(_cfg->ipm->applyHomography(p - _tl)));
	for(Point2d p : _raw_elas_message.lane_position.right) pos_right_ipm.push_back(toVector2D(_cfg->ipm->applyHomography(p -_tl)));
	reverse(pos_left_ipm.begin(), pos_left_ipm.end());
	reverse(pos_right_ipm.begin(), pos_right_ipm.end());
	lane_estimation_message.left_ipm = pos_left_ipm.data();
	lane_estimation_message.right_ipm = pos_right_ipm.data();

	// lane estimation - others
	lane_estimation_message.lane_deviation = _raw_elas_message.lane_deviation;
	lane_estimation_message.lane_width = _raw_elas_message.lane_base.width;
	lane_estimation_message.direction = toVector2D(_raw_elas_message.lane_base.direction);
	lane_estimation_message.point_bottom = toVector2D(_raw_elas_message.lane_base.point_bottom);
	lane_estimation_message.point_top = toVector2D(_raw_elas_message.lane_base.point_top);
	lane_estimation_message.trustworthy_height = _raw_elas_message.trustworthy_height;
	lane_estimation_message.scale_x = scale_x;
	lane_estimation_message.scale_y = scale_y;

	// lane markings type
	lmt_message.left = _raw_elas_message.lmt.left;
	lmt_message.right = _raw_elas_message.lmt.right;

	// adjacent lanes
	adj_lanes_message.left = _raw_elas_message.adjacent_lanes.left;
	adj_lanes_message.right = _raw_elas_message.adjacent_lanes.right;

	// MESSAGES DEBUG
	printf("\tlane_width: %.2f meters\n", lane_estimation_message.lane_width * scale_x);
	printf("\tLMT: left(%d), right(%d)\n", lmt_message.left, lmt_message.right);
	printf("\tadj: left(%d), right(%d)\n", adj_lanes_message.left, adj_lanes_message.right);

/*
    lane_analysis_message.car_position_x = _raw_elas_message.car_position_x;
    lane_analysis_message.execution_time = _raw_elas_message.execution_time;
    lane_analysis_message.isKalmanNull = _raw_elas_message.isKalmanNull;
    lane_analysis_message.lane_change = _raw_elas_message.lane_change;
*/

	printf("publishing messages:\n");

	printf("\tlane position...");
	carmen_elas_lane_estimation_publish_message(&lane_estimation_message);
	printf(" done!\n");

	printf("\tlane markings type...");
	carmen_elas_lane_markings_type_publish_message(&lmt_message);
	printf(" done!\n");

	printf("\tadjacent lanes...");
	carmen_elas_adjacent_lanes_publish_message(&adj_lanes_message);
	printf(" done!\n");

    printf("messages published!\n");
}

static int fnumber = 0;
static const int publish_after_frame = 100; // global_pos is incorrect in the beginning
void
lane_analysis_handler(carmen_bumblebee_basic_stereoimage_message * stereo_image)
{

#ifdef TEST_OFFLINE_DATASET
	// read the frame
#ifdef DATASET_GOPRO
	Mat3b image = imread("/dados/berriel/MEGA/datasets/VIX_S05/images/lane_" + to_string(fnumber) + ".png");
#else
	Mat3b image = imread("/dados/berriel/datasets/log-rodrigo-ufes/images/lane_" + to_string(fnumber) + ".png");
#endif
#else
	// get the image from the bumblebee
	Mat3b image(960, 1280);
	image.data = (uchar *) stereo_image->raw_left;
	cvtColor(image, image, CV_RGB2BGR);
	cv::resize(image, image, Size(640,480));
	// imwrite("/dados/log-rodrigo/lane_" + to_string(fnumber) + ".png", image);
#endif
	fnumber++;
	if (!image.empty()) {
		// run ELAS
		ELAS::run(image);
		printf("CARMEN::ELAS... done!\n");

		// publish messages
		if (fnumber > publish_after_frame)
			lane_analysis_publish_messages(stereo_image->timestamp);

		// viz here? to debug?
		imshow("ELAS - CARMEN", image);
		waitKey(1);
	} else {
		printf("End of dataset!\n");
	}
}

#ifdef TEST_MESSAGES_PUBLISH
void
self_elas_estimation_handler(carmen_elas_lane_estimation_message * message) {
	printf("car center deviation: %.2f\n", message->lane_deviation);
	printf("lane width: %.2f\n", message->lane_width);
}
#endif

static int
read_parameters(int argc, char **argv)
{
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

void
lane_analysis_init()
{
    ELAS::init(config_fname, config_xml_fname);

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

}

// subscribes: what does this module need?
void
subscribe_elas_messages()
{
    carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) lane_analysis_handler, CARMEN_SUBSCRIBE_LATEST);

#ifdef TEST_MESSAGES_PUBLISH
    carmen_elas_lane_estimation_subscribe(NULL, (carmen_handler_t) self_elas_estimation_handler, CARMEN_SUBSCRIBE_LATEST);
#endif

}

static void
register_ipc_messages(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_ELAS_LANE_ESTIMATION_NAME, IPC_VARIABLE_LENGTH, CARMEN_ELAS_LANE_ESTIMATION_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_ELAS_LANE_ESTIMATION_NAME);

	err = IPC_defineMsg(CARMEN_ELAS_LANE_MARKINGS_TYPE_NAME, IPC_VARIABLE_LENGTH, CARMEN_ELAS_LANE_MARKINGS_TYPE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_ELAS_LANE_MARKINGS_TYPE_NAME);

	err = IPC_defineMsg(CARMEN_ELAS_ADJACENT_LANES_NAME, IPC_VARIABLE_LENGTH, CARMEN_ELAS_ADJACENT_LANES_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_ELAS_ADJACENT_LANES_NAME);
}

int
main(int argc, char * argv[])
{
	// connect to IPC server
	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

    // read the params (camera and calibration params from config file)
	read_parameters(argc, argv);

    // define the messages
	register_ipc_messages();

	IPC_defineFormat("vector_2D", "{double,double}");

	// subscribes
	subscribe_elas_messages();

	lane_analysis_init();

	// Loop forever waiting for messages
	carmen_ipc_dispatch();

    ELAS::finishProgram();

	return EXIT_SUCCESS;
}
