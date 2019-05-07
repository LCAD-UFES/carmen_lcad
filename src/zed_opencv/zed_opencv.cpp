#include <stdio.h>
#include <iostream>
#include <sstream>
#include <opencv2/opencv.hpp>

#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/stereo_interface.h>

#include "calibration.hpp"

// the only ugly global variable
static bool running_app = true;

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

void
publish_stereoimage_message(int camera_id, cv::Mat &cvL, cv::Mat &cvR)
{
    static int first = 1;

    static cv::Mat frameL3;
    static cv::Mat frameR3;

    if (first)
    {
        frameL3 = cv::Mat(cv::Size(cvL.cols, cvL.rows), CV_8UC3);
        frameR3 = cv::Mat(cv::Size(cvL.cols, cvL.rows), CV_8UC3);
        first = 0;
    }

    cvtColor(cvL, frameL3, CV_BGRA2RGB);
    cvtColor(cvR, frameR3, CV_BGRA2RGB);

    carmen_bumblebee_basic_stereoimage_message stereo_msg;

    stereo_msg.timestamp = carmen_get_time();
    stereo_msg.host = carmen_get_host();
    stereo_msg.width  = cvL.cols;
    stereo_msg.height = cvL.rows;
    stereo_msg.image_size = cvL.rows * cvL.cols * 3;
    stereo_msg.isRectified = 1;
    stereo_msg.raw_left  = frameL3.data;
    stereo_msg.raw_right = frameR3.data;

    carmen_bumblebee_basic_publish_message(camera_id, &stereo_msg);
}

static int
read_parameters(
    int argc,
    char **argv,
    int camera_id,
    int &serial_number,
    int &fps,
    cv::Size2i &image_size)
{
    int num_items;
    char bb_name[64];

    sprintf(bb_name, "bumblebee_basic%d", camera_id);

    carmen_param_t param_list[] =
    {
        { bb_name, (char*) "zed_fps",           CARMEN_PARAM_INT, &fps,                 0, NULL},
        { bb_name, (char*) "width",             CARMEN_PARAM_INT, &image_size.width,    0, NULL},
        { bb_name, (char*) "height",            CARMEN_PARAM_INT, &image_size.height,   0, NULL},
        { bb_name, (char*) "zed_serial_number", CARMEN_PARAM_INT, &serial_number,       0, NULL}
    };

    num_items = sizeof(param_list)/sizeof(param_list[0]);
    carmen_param_install_params(argc, argv, param_list, num_items);

    return 0;
}

void
shutdown_module(int signo)
{
    if(signo == SIGINT)
    {
        running_app = false;
        carmen_ipc_disconnect();
        printf("zed_camera_sensor: disconnected.\n");
        exit(0);
    }
}

int
get_device_id(int argc, char** argv)
{
    int device_id = 0;
    if (1 < argc) {
        std::stringstream ss;
        ss << argv[1];
        ss >> device_id;
    }
    return device_id;
}

int
publish(
    int argc,
    char **argv,
    int camera_id,
    int &serial_number,
    int &fps,
    cv::Size2i &image_size)
{
    int device_id = get_device_id(argc, argv);
    std::string calibration_file(argv[2]);

    cv::Mat map_left_x, map_left_y;
    cv::Mat map_right_x, map_right_y;
    cv::Mat cameraMatrix_left, cameraMatrix_right;

    initCalibration(calibration_file, image_size, map_left_x, map_left_y, map_right_x, map_right_y, cameraMatrix_left, cameraMatrix_right);

    std::cout << " Camera Matrix L: \n" << cameraMatrix_left << std::endl << std::endl;
    std::cout << " Camera Matrix R: \n" << cameraMatrix_right << std::endl << std::endl;

    cv::VideoCapture cap;

    cap.open(device_id);

    if (!cap.isOpened()) return -1;

    // Set the video resolution (2*Width * Height)
    cap.set(CV_CAP_PROP_FRAME_WIDTH,  image_size.width * 2);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, image_size.height);

    cap.grab();

    cv::Mat frame, left_raw, left_rect, right_raw, right_rect;

    while (running_app) {

        // Get a new frame from camera
        cap >> frame;

        // Extract left and right images from side-by-side
        left_raw =  frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
        right_raw = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));

        // rectify images
        cv::remap(left_raw,  left_rect,  map_left_x,  map_left_y,  cv::INTER_LINEAR);
        cv::remap(right_raw, right_rect, map_right_x, map_right_y, cv::INTER_LINEAR);

        // publish the rectified stereo image
        publish_stereoimage_message(camera_id, left_rect, right_rect);
    }

    return 0;
}

int main(int argc, char **argv)
{
    if (3 > argc)
    {
        std::cout << "Usage: ./zed <device_id> <calibration_file>" << std::endl;
        return 1;
    }

    // default camera id
    int camera_id = 4;

    // hard coded serial number value, but it can be replaced by the carmen param daemon
    int serial_number = 19436;

    int fps = 15;

    /* Connect to IPC Server */
    carmen_ipc_initialize(argc, argv);

    /* Check the param server version */
    carmen_param_check_version(argv[0]);

    /* Register shutdown cleaner handler */
    signal(SIGINT, shutdown_module);

    cv::Size2i image_size = cv::Size2i(1280, 720);

    /* Read application specific parameters (Optional) */
    read_parameters(argc, argv, camera_id, serial_number, fps, image_size);

    // the  basic bumblebee stereo message
    carmen_bumblebee_basic_define_messages(camera_id);

    // the disparity map message
    carmen_stereo_define_messages(camera_id);

    // the main publish loop method
    return publish(argc, argv, camera_id, serial_number, fps, image_size);
}