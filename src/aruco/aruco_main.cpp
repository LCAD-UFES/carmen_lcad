#include <carmen/carmen.h>
#include <aruco/aruco.h>
#include <carmen/camera_drivers_interface.h>
#include <carmen/bumblebee_basic_interface.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <list>

#include "aruco_interface.h"
#include "aruco_messages.h"


int SHOW_ON_TERMINAL = 0;

double _fx = 0., _fy = 0., _cu = 0., _cv = 0.;
double _k1 = 0., _k2 = 0., _p1 = 0., _p2 = 0., _k3 = 0.;
int use_left = 1, use_right = 0;
char *side_to_use = NULL;
double msg_fps = 0;

int show_output = 1;
int detector_id = 1;
char* camera_name = NULL;
int message_id = 1;
char* board_config_file = NULL;
char* aruco_dictionary = NULL; // "ARUCO_MIP_36h12"
double marker_size_in_meters = 0.2;


carmen_aruco_pose
setup_aruco_pose(int n_markers_detected, int _id_markers_detected, cv::Mat _tvec, cv::Mat _rvec)
{
    carmen_aruco_pose pose;
    int *ids_markers_detected = (int*) malloc(sizeof(int));
    ids_markers_detected[0] = _id_markers_detected;

    pose.n_markers_detected = n_markers_detected;
    pose.ids_markers_detected = ids_markers_detected;
    pose.rvec[0] = (double) _tvec.at<float>(0); pose.rvec[1] = (double) _tvec.at<float>(1); pose.rvec[2] = (double) _tvec.at<float>(2);
    pose.tvec[0] = (double) _rvec.at<float>(0); pose.tvec[1] = (double) _rvec.at<float>(1); pose.tvec[2] = (double) _rvec.at<float>(2);
    return pose;
}


carmen_aruco_pose
setup_aruco_pose(int n_markers_detected, std::vector<int> _ids_markers_detected, cv::Mat _tvec, cv::Mat _rvec)
{
    carmen_aruco_pose pose;
    int *ids_markers_detected = (int*) malloc(n_markers_detected*sizeof(int));
    for (size_t i = 0; i < _ids_markers_detected.size(); i++)
        ids_markers_detected[i] = _ids_markers_detected[i]+1;

    pose.n_markers_detected = n_markers_detected;
    pose.ids_markers_detected = ids_markers_detected;
    pose.rvec[0] = (double) _tvec.at<float>(0); pose.rvec[1] = (double) _tvec.at<float>(1); pose.rvec[2] = (double) _tvec.at<float>(2);
    pose.tvec[0] = (double) _rvec.at<float>(0); pose.tvec[1] = (double) _rvec.at<float>(1); pose.tvec[2] = (double) _rvec.at<float>(2);
    return pose;
}


void 
publish_message(int detector_id, std::vector<carmen_aruco_pose> _poses)
{
    aruco_define_message(detector_id);

    carmen_aruco_message message;
    carmen_aruco_pose* poses = (carmen_aruco_pose*) malloc(_poses.size()*sizeof(carmen_aruco_pose));
    for (size_t i = 0; i < _poses.size(); i++)
        poses[i] = _poses[i];
    
	message.n_poses = _poses.size();
	message.poses = poses;
	message.host = carmen_get_host();
    message.timestamp = carmen_get_time();

    aruco_publish_message(detector_id, &message);
}


void 
detect_posetracker(cv::Mat image)
{
    static int first = 1;
    int n_markers_detected = 0;
    static aruco::MarkerMap mmap;
    static aruco::CameraParameters camera;
    static aruco::MarkerDetector Detector;
    static aruco::MarkerMapPoseTracker MMTracker;
    std::vector<carmen_aruco_pose> poses;
    
    cv::Mat bgr, _rvec, _tvec;

    int pos;

    if (first)
    {
        cv::Mat camMatrix = (cv::Mat_<double>(3, 3) << _fx, .0, _cu, .0, _fy, _cv, .0, .0, 1.);
        cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << _k1, _k2, _p1, _p2, _k3);

        camera.setParams(camMatrix, distCoeffs, image.size());
        mmap.readFromFile(board_config_file);
        mmap = mmap.convertToMeters(marker_size_in_meters);
        MMTracker.setParams(camera, mmap, marker_size_in_meters);
        Detector.setDictionary(mmap.getDictionary());

        first = 0;
    }

    std::vector<aruco::Marker> markers = Detector.detect(image);

    std::vector<int> markers_from_set = mmap.getIndices(markers);
    n_markers_detected = markers_from_set.size();
    for (size_t i = 0; (i < n_markers_detected) && show_output; i++)
            markers[i].draw(image, cv::Scalar(0, 0, 255), 1.5);

    if (MMTracker.estimatePose(markers))
    {
        _rvec = MMTracker.getRvec();
        _tvec = MMTracker.getTvec();

        // +X is Right on the screen, +Y is Up, +Z is INTO the screen
        if (show_output)
            aruco::CvDrawingUtils::draw3dAxis(image, camera, _rvec, _tvec, mmap[0].getMarkerSize());

        if (SHOW_ON_TERMINAL)
        {
            for (size_t i = 0; i < n_markers_detected; i++)
                std::cout << markers_from_set[i]+1 << " ";
            std::cout << std::endl;
            std::cout << _tvec.at<float>(0) << " " << _tvec.at<float>(1) << " " << _tvec.at<float>(2) << std::endl;
            std::cout << _rvec.at<float>(0) << " " << _rvec.at<float>(1) << " " << _rvec.at<float>(2) << std::endl;
            std::cout << std::endl;
        }

        poses.push_back(setup_aruco_pose(n_markers_detected, markers_from_set, _tvec, _rvec));
    }

    if (show_output)
    {
        // cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
        cv::resize(image, image, cv::Size(), 640.0/(double)image.cols, 480.0/(double)image.rows);
        cv::imshow(camera_name, image);
        cv::waitKey(1);
    }

    if (poses.size() > 0)
        publish_message(detector_id, poses);
}


void 
detect_markers(cv::Mat &image)
{
    static int first = 1;
    int n_markers_detected = 0;
    static aruco::CameraParameters camera;
    static aruco::MarkerDetector Detector;
    cv::Mat bgr;
    std::vector<carmen_aruco_pose> poses;

    if (first)
    {
        cv::Mat camMatrix = (cv::Mat_<double>(3, 3) << _fx, .0, _cu, .0, _fy, _cv, .0, .0, 1.);
        cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << _k1, _k2, _p1, _p2, _k3);

        camera.setParams(camMatrix, distCoeffs, image.size());
        Detector.setDictionary(aruco_dictionary || "ARUCO_MIP_36h12");

        first = 0;
    }

    std::vector<aruco::Marker> markers = Detector.detect(image, camera, marker_size_in_meters);
    n_markers_detected = markers.size();
    for (size_t i = 0; i < n_markers_detected; i++)
    {
        poses.push_back(setup_aruco_pose(1, markers[i].id, markers[i].Tvec, markers[i].Rvec));
        if (SHOW_ON_TERMINAL)
        {
            std::cout << markers[i].id << std::endl;
            std::cout << markers[i].Tvec.at<float>(0) << " " <<  markers[i].Tvec.at<float>(1) << " " <<  markers[i].Tvec.at<float>(2) << std::endl;
            std::cout << markers[i].Rvec.at<float>(0) << " " << markers[i].Rvec.at<float>(1) << " " << markers[i].Rvec.at<float>(2) << std::endl;
            std::cout << std::endl;
        }
    }
    if (SHOW_ON_TERMINAL)
        std::cout << std::endl;

    if (camera.isValid() && (marker_size_in_meters > 0) && show_output)
        for (size_t i = 0; i < n_markers_detected; i++)
        {
            aruco::CvDrawingUtils::draw3dAxis(image, markers[i], camera);
            markers[i].draw(image, cv::Scalar(0, 0, 255), 1.5);
        }

    if (show_output)
    {
        // cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
        char info[25];
        sprintf(info, "%dx%d", image.cols, image.rows);
        cv::resize(image, image, cv::Size(), 640.0/(double)image.cols, 480.0/(double)image.rows);

        putText(image, info, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, .7, cv::Scalar(0, 0, 0), 4);
        putText(image, info, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, .7, cv::Scalar(255, 255, 255), 2);

        sprintf(info, "%.2fFPS", msg_fps);
        putText(image, info, cv::Point(10, 55), cv::FONT_HERSHEY_SIMPLEX, .7, cv::Scalar(0, 0, 0), 4);
        putText(image, info, cv::Point(10, 55), cv::FONT_HERSHEY_SIMPLEX, .7, cv::Scalar(255, 255, 255), 2);

        cv::imshow(camera_name, image);
        cv::waitKey(1);
    }

    if (poses.size() > 0)
        publish_message(detector_id, poses);
}


double
filter_fps(double timestamp)
{
    static std::list <double> timestamp_list;
    static unsigned int max_list_size = 5;
    double sum = 0.0;
    double cont = 0.0;

    if (timestamp_list.size() > max_list_size)
        timestamp_list.pop_front();

    timestamp_list.push_back(timestamp);

    std::list <double>::iterator it = timestamp_list.begin();
    std::list <double>::iterator next_it = timestamp_list.begin();
    next_it++;

    while(1)
    {
        if (next_it == timestamp_list.end())
            break;

        sum += *next_it - *it;
        it++;
        next_it++;
        cont += 1.0;
    }
    double fps = 1 / (sum / cont);

    if (round(fps) > max_list_size)
        max_list_size = round(fps);

    return (fps);
}


void 
image_handler_intelbras(camera_message *msg)
{
    // static int first_time = 1;
	camera_image *stereo_image = msg->images;
	cv::Mat cv_image = cv::Mat(stereo_image->height, stereo_image->width, CV_8UC3, stereo_image->raw_data, 0);
    
    // if (first_time)
    // {
    //     _fx = _fx_factor*cv_image.cols;
    //     _fy = _fy_factor*cv_image.rows;
    //     _cu = _cu_factor*cv_image.cols;
    //     _cv = _cv_factor*cv_image.rows;
    //     first_time = 0;
    // }
    msg_fps = filter_fps(msg->timestamp);

    if (board_config_file)
        detect_posetracker(cv_image);
    else
        detect_markers(cv_image);
}


static void
image_handler_bumblebee(carmen_bumblebee_basic_stereoimage_message* msg)
{
    cv::Mat src_image_left, src_image_right, concat;

    if (use_left)
    	src_image_left = cv::Mat(cv::Size(msg->width, msg->height), CV_8UC3, msg->raw_left);
    if (use_right)
    	src_image_right = cv::Mat(cv::Size(msg->width, msg->height), CV_8UC3, msg->raw_right);

    msg_fps = filter_fps(msg->timestamp);

    if (use_left && use_right)
    	cv::hconcat(src_image_left, src_image_right, concat);
    else
    	concat = use_left ? src_image_left : src_image_right;
    
    cvtColor(concat, concat, cv::COLOR_BGR2RGB);
    if (board_config_file)
        detect_posetracker(concat);
    else
        detect_markers(concat);
}


void 
shutdown_module(int signo)
{
    if (signo == SIGINT)
    {
        carmen_ipc_disconnect();
        cv::destroyAllWindows();

        printf("Signal %d received, exiting program ...\n", signo);
        exit(0);
    }
}


int 
read_parameters(int argc, char **argv)
{
    int num_items;
    char board_string[256];
    char semitrailer_string[256];

    carmen_param_allow_unfound_variables(0);
    carmen_param_t param_list_cm[] =
        {
            {(char *) "commandline", (char *) "camera_name",        CARMEN_PARAM_STRING, &camera_name,           0, NULL},
            {(char *) "commandline", (char *) "message_id",         CARMEN_PARAM_INT,    &message_id,            0, NULL},
            {(char *) "commandline", (char *) "marker_size",        CARMEN_PARAM_DOUBLE, &marker_size_in_meters, 0, NULL},
            {(char *) "commandline", (char *) "detector_id",        CARMEN_PARAM_INT,    &detector_id,           0, NULL},
        };
    num_items = sizeof(param_list_cm) / sizeof(param_list_cm[0]);
    carmen_param_install_params(argc, argv, param_list_cm, num_items);

    carmen_param_allow_unfound_variables(1);
    carmen_param_t param_list_cm_[] =
        {
            {(char *) "commandline", (char *) "board",          CARMEN_PARAM_STRING, &board_config_file,     0, NULL},
            {(char *) "commandline", (char *) "show_output",    CARMEN_PARAM_ONOFF,  &show_output,           0, NULL},
            {(char *) "commandline", (char *) "side",           CARMEN_PARAM_STRING, &side_to_use,           0, NULL},
        };
    num_items = sizeof(param_list_cm_) / sizeof(param_list_cm_[0]);
    carmen_param_install_params(argc, argv, param_list_cm_, num_items);
    if (side_to_use != NULL)
    {
        if (strcmp(side_to_use, "L") == 0 || strcmp(side_to_use, "l") == 0 || strcmp(side_to_use, "left") == 0)
			use_left = 1, use_right = 0;
		else if (strcmp(side_to_use, "R") == 0 || strcmp(side_to_use, "r") == 0 || strcmp(side_to_use, "right") == 0)
			use_left = 0, use_right = 1;
		else if (strcmp(side_to_use, "S") == 0 || strcmp(side_to_use, "s") == 0 || strcmp(side_to_use, "stereo") == 0)
			use_left = 1, use_right = 1;
    }

    if (!board_config_file)
    {
        carmen_param_allow_unfound_variables(0);
        carmen_param_t param_list_cm__[] =
            {
                {(char *) "commandline", (char *) "dictionary",  CARMEN_PARAM_STRING, &aruco_dictionary,      0, NULL},
            };
        num_items = sizeof(param_list_cm__) / sizeof(param_list_cm__[0]);
        carmen_param_install_params(argc, argv, param_list_cm__, num_items);

        std::cout << "detecting by single markers" << std::endl;
    }
    else
        std::cout << "detecting by board config " << board_config_file << std::endl;

    carmen_param_allow_unfound_variables(0);
    carmen_param_t param_list[] =
        {
            {camera_name, (char *)"fx", CARMEN_PARAM_DOUBLE, &_fx, 0, NULL},
            {camera_name, (char *)"fy", CARMEN_PARAM_DOUBLE, &_fy, 0, NULL},
            {camera_name, (char *)"cu", CARMEN_PARAM_DOUBLE, &_cu, 0, NULL},
            {camera_name, (char *)"cv", CARMEN_PARAM_DOUBLE, &_cv, 0, NULL},
            {camera_name, (char *)"k1", CARMEN_PARAM_DOUBLE, &_k1, 0, NULL},
            {camera_name, (char *)"k2", CARMEN_PARAM_DOUBLE, &_k2, 0, NULL},
            {camera_name, (char *)"p1", CARMEN_PARAM_DOUBLE, &_p1, 0, NULL},
            {camera_name, (char *)"p2", CARMEN_PARAM_DOUBLE, &_p2, 0, NULL},
            {camera_name, (char *)"k3", CARMEN_PARAM_DOUBLE, &_k3, 0, NULL},
        };

    num_items = sizeof(param_list) / sizeof(param_list[0]);
    carmen_param_install_params(argc, argv, param_list, num_items);

    return 0;
}


void 
subscribe_messages()
{
    if (strncmp("intelbras", camera_name, 9) == 0)
        camera_drivers_subscribe_message(message_id, NULL, (carmen_handler_t) image_handler_intelbras, CARMEN_SUBSCRIBE_LATEST);
    else
        carmen_bumblebee_basic_subscribe_stereoimage(message_id, NULL, (carmen_handler_t) image_handler_bumblebee, CARMEN_SUBSCRIBE_LATEST);
}


int 
main(int argc, char **argv)
{
    carmen_ipc_initialize(argc, argv);
    carmen_param_check_version(argv[0]);

    signal(SIGINT, shutdown_module);

    read_parameters(argc, argv);

    subscribe_messages();
    carmen_ipc_dispatch();

    return 0;
}
