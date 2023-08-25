#define BOOST_SIGNALS_NO_DEPRECATION_WARNING

#include <carmen/tf.h>
#include <carmen/carmen.h>
#include <aruco/aruco.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <carmen/camera_drivers_interface.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/localize_ackerman_interface.h>

#include "aruco_theta_tracker.hpp"


#define BOARD1 1
#define THETA1 1


aruco_theta_tracker_parameters params;

tf::Transformer transformer;
carmen_pose_3D_t board_pose;
carmen_pose_3D_t camera_pose;
carmen_pose_3D_t semitrailer_pose;

#define GLOBALPOS_QUEUE_SIZE 5
carmen_localize_ackerman_globalpos_message last_globalpos;
carmen_localize_ackerman_globalpos_message globalpos_queue[GLOBALPOS_QUEUE_SIZE];


void 
initialize_transformations()
{
    tf::Transform car_to_semitrailer_pose;
    tf::Transform car_to_board_pose;
    tf::Transform board_to_camera_pose;

    tf::Time::init();

    // king pin pose with respect to the car
    car_to_semitrailer_pose.setOrigin(tf::Vector3(semitrailer_pose.position.x, semitrailer_pose.position.y, semitrailer_pose.position.z));
    car_to_semitrailer_pose.setRotation(tf::Quaternion(semitrailer_pose.orientation.yaw, semitrailer_pose.orientation.pitch, semitrailer_pose.orientation.roll)); // yaw, pitch, roll
    tf::StampedTransform car_to_semitrailer_transform(car_to_semitrailer_pose, tf::Time(0), "/car", "/semitrailer");
    transformer.setTransform(car_to_semitrailer_transform, "car_to_semitrailer_transform");

    // board pose with respect to the car
    car_to_board_pose.setOrigin(tf::Vector3(board_pose.position.x, board_pose.position.y, board_pose.position.z));
    car_to_board_pose.setRotation(tf::Quaternion(board_pose.orientation.yaw, board_pose.orientation.pitch, board_pose.orientation.roll)); // yaw, pitch, roll
    tf::StampedTransform car_to_board_transform(car_to_board_pose, tf::Time(0), "/car", "/board");
    transformer.setTransform(car_to_board_transform, "car_to_board_transform");

    // camera pose with respect to the board
    board_to_camera_pose.setOrigin(tf::Vector3(camera_pose.position.x, camera_pose.position.y, camera_pose.position.z));
    board_to_camera_pose.setRotation(tf::Quaternion(camera_pose.orientation.yaw, camera_pose.orientation.pitch, camera_pose.orientation.roll)); // yaw, pitch, roll			// yaw, pitch, roll
    tf::StampedTransform board_to_camera_transform(board_to_camera_pose, tf::Time(0), "/board", "/camera");
    transformer.setTransform(board_to_camera_transform, "board_to_camera_transform");
}


tf::Transform
Rvec_Tvec_toTf(cv::Mat _tvec, cv::Mat _rvec, tf::Vector3 &marker_origin, tf::Quaternion &marker_quaternion)
{
    cv::Mat _tvec_t = (cv::Mat_<float>(3, 1) << - _tvec.at<float>(2), - _tvec.at<float>(0), _tvec.at<float>(1));
    cv::Mat _rvec_t = (cv::Mat_<float>(3, 1) << - _rvec.at<float>(2), - _rvec.at<float>(0), _rvec.at<float>(1));
    cv::Mat marker_rotation(3, 3, CV_32FC1);
    cv::Rodrigues(_rvec_t, marker_rotation);

    cv::Mat rotate_to_ros(3, 3 ,CV_32FC1);
    rotate_to_ros.at<float>(0,0) = -1.0;
    rotate_to_ros.at<float>(0,1) = 0;
    rotate_to_ros.at<float>(0,2) = 0;
    rotate_to_ros.at<float>(1,0) = 0;
    rotate_to_ros.at<float>(1,1) = 0;
    rotate_to_ros.at<float>(1,2) = 1.0;
    rotate_to_ros.at<float>(2,0) = 0.0;
    rotate_to_ros.at<float>(2,1) = 1.0;
    rotate_to_ros.at<float>(2,2) = 0.0;

    marker_rotation = marker_rotation * rotate_to_ros.t();

    // Origin solution
    tf::Matrix3x3 marker_tf_rot(marker_rotation.at<float>(0,0),marker_rotation.at<float>(0,1),marker_rotation.at<float>(0,2),
                                marker_rotation.at<float>(1,0),marker_rotation.at<float>(1,1),marker_rotation.at<float>(1,2),
                                marker_rotation.at<float>(2,0),marker_rotation.at<float>(2,1),marker_rotation.at<float>(2,2));

    tf::Vector3 marker_tf_tran(_tvec_t.at<float>(0),
                               _tvec_t.at<float>(1),
                               _tvec_t.at<float>(2));

    tf::Transform transform(marker_tf_rot, marker_tf_tran);
    return transform;
}


void 
to_marker_transformer(cv::Mat _tvec, cv::Mat _rvec, carmen_vector_3D_t &Tvec, carmen_orientation_3D_t &Rvec)
{
    static int first = 1;
    if (first)
    {
        initialize_transformations();
        first = 0;
    }

    tf::Vector3 marker_origin;
    tf::Quaternion marker_quaternion;
    tf::StampedTransform semitrailer_to_marker_pose;

    tf::Transform camera_to_marker_pose = Rvec_Tvec_toTf(_tvec, _rvec, marker_origin, marker_quaternion);
    tf::StampedTransform camera_to_marker_transform(camera_to_marker_pose, tf::Time(0), "/camera", "/marker");
    transformer.setTransform(camera_to_marker_transform, "camera_to_marker_transform");

    transformer.lookupTransform("/semitrailer", "/marker", tf::Time(0), semitrailer_to_marker_pose);

    Tvec.x = semitrailer_to_marker_pose.getOrigin().x();
    Tvec.y = semitrailer_to_marker_pose.getOrigin().y();
    Tvec.z = semitrailer_to_marker_pose.getOrigin().z();
    tf::Matrix3x3(semitrailer_to_marker_pose.getRotation()).getEulerYPR(Rvec.yaw, Rvec.pitch, Rvec.roll);
}


void 
plot_beta_diff(double _beta_velodyne, double _beta_aruco)
{
    static FILE *gnuplot_pipe = NULL;
    static int count = 0;
    static bool first_time = true;

    if (first_time)
    {
        first_time = false;
        gnuplot_pipe = popen("taskset -c 0 gnuplot", "w"); // -persist to keep last plot after program closes
        fprintf(gnuplot_pipe, "set yrange [-300:300]\n");
        remove("beta_comparison.dat");
    }
    fprintf(gnuplot_pipe, "set xrange [0:%d]\n", ++count);

    FILE *graph_file;
    graph_file = fopen("beta_comparison.dat", "a");
    fprintf(graph_file, "%d\t%lf\t%lf\n", 
            count,
            _beta_velodyne * 57.2958, 
            _beta_aruco * 57.2958);

    fclose(graph_file);
    fprintf(gnuplot_pipe, "plot "
                          "'beta_comparison.dat' using 1:2 title 'velodyne', "
                          "'beta_comparison.dat' using 1:3 title 'aruco'"
                          "\n");
    fflush(gnuplot_pipe);
}


void 
publish_message(int detector_id, int _n_markers_detected, cv::Mat _tvec, cv::Mat _rvec, double timestamp, double offset)
{
    if (_n_markers_detected <= 0)
        return;

    double estimated_theta1;
    carmen_vector_3D_t Tvec;
    carmen_orientation_3D_t Rvec;

    to_marker_transformer(_tvec, _rvec, Tvec, Rvec);

    // estimated_theta1 = convert_beta_to_theta1(last_globalpos.globalpos.theta, carmen_normalize_theta(Rvec.yaw + offset));
    estimated_theta1 = carmen_normalize_theta(convert_beta_to_theta1(last_globalpos.globalpos.theta, Rvec.yaw + offset));
    plot_beta_diff(carmen_normalize_theta(last_globalpos.trailer_theta[0]), estimated_theta1);

    // plot_beta_diff(last_globalpos.trailer_theta[0], - estimated_theta1 + M_PI);
    // printf("%lf\t%lf\t%lf\n", estimated_theta1 * 57.2958, last_globalpos.trailer_theta[0] * 57.2958, timestamp - last_globalpos.timestamp);
}



void 
image_handler_intelbras(camera_message *msg)
{
    double offset = .0;
    cv::Mat tvec, rvec;
    int n_markers_detected = 0;
    aruco_theta_tracker_detect(msg, params, tvec, rvec, n_markers_detected, offset);

    last_globalpos = globalpos_queue[0];
    for (size_t i = 1; i < GLOBALPOS_QUEUE_SIZE; i++)
        if (fabs(globalpos_queue[i].timestamp - msg->timestamp) < fabs(last_globalpos.timestamp - msg->timestamp))
            last_globalpos = globalpos_queue[i];

    publish_message(params.detector_id, n_markers_detected, tvec, rvec, msg->timestamp, offset);
}


void 
image_handler_bumblebee(carmen_bumblebee_basic_stereoimage_message *msg)
{
    double offset = .0;
    cv::Mat tvec, rvec;
    int n_markers_detected = 0;
    aruco_theta_tracker_detect(msg, params, tvec, rvec, n_markers_detected, offset);

    last_globalpos = globalpos_queue[0];
    for (size_t i = 1; i < GLOBALPOS_QUEUE_SIZE; i++)
        if (fabs(globalpos_queue[i].timestamp - msg->timestamp) < fabs(last_globalpos.timestamp - msg->timestamp))
            last_globalpos = globalpos_queue[i];

    publish_message(params.detector_id, n_markers_detected, tvec, rvec, msg->timestamp, offset);
}


void
localize_handler(carmen_localize_ackerman_globalpos_message *msg)
{
    carmen_localize_ackerman_globalpos_message local;
    local.globalpos = msg->globalpos;
    local.trailer_theta[0] = msg->trailer_theta[0];
    local.timestamp = msg->timestamp;
    local.host = NULL;

    static int idx = 0, first_time = 1;
    if (first_time)
    {
        for (size_t i = 0; i < GLOBALPOS_QUEUE_SIZE; i++)
            globalpos_queue[i] = local;

        first_time = 0;
    }
    globalpos_queue[idx] = local;
    idx = (idx + 1) % GLOBALPOS_QUEUE_SIZE;
}


void 
shutdown_module(int signo)
{
    if (signo == SIGINT)
    {
        carmen_ipc_disconnect();
        cv::destroyAllWindows();

        printf("\e[0;31msignal %d received, exiting program ...\e[0m\n", signo);
        exit(0);
    }
}


int 
read_parameters(int argc, char **argv)
{
    int num_items, semi_trailer_type;
    char board_string[256], semitrailer_string[256];

    // transform parameters
    carmen_param_allow_unfound_variables(0);
    carmen_param_t param_list_ini[] =
        {
            {(char *) "semi_trailer", (char *) "initial_type", CARMEN_PARAM_INT, &semi_trailer_type}
        };
    num_items = sizeof(param_list_ini) / sizeof(param_list_ini[0]);
    carmen_param_install_params(argc, argv, param_list_ini, num_items);

    sprintf(board_string, "%s%d", "sensor_board_", BOARD1);
    sprintf(semitrailer_string, "%s%d", "semi_trailer", semi_trailer_type);

    carmen_param_allow_unfound_variables(0);
    carmen_param_t param_list_pos[] =
        {
            {board_string, (char *)"x",     CARMEN_PARAM_DOUBLE, &board_pose.position.x, 0, NULL},
            {board_string, (char *)"y",     CARMEN_PARAM_DOUBLE, &board_pose.position.y, 0, NULL},
            {board_string, (char *)"z",     CARMEN_PARAM_DOUBLE, &board_pose.position.z, 0, NULL},
            {board_string, (char *)"roll",  CARMEN_PARAM_DOUBLE, &board_pose.orientation.roll, 0, NULL},
            {board_string, (char *)"pitch", CARMEN_PARAM_DOUBLE, &board_pose.orientation.pitch, 0, NULL},
            {board_string, (char *)"yaw",   CARMEN_PARAM_DOUBLE, &board_pose.orientation.yaw, 0, NULL},

            {(char*) params.camera_name, (char *)"x",      CARMEN_PARAM_DOUBLE, &camera_pose.position.x, 0, NULL},
            {(char*) params.camera_name, (char *)"y",      CARMEN_PARAM_DOUBLE, &camera_pose.position.y, 0, NULL},
            {(char*) params.camera_name, (char *)"z",      CARMEN_PARAM_DOUBLE, &camera_pose.position.z, 0, NULL},
            {(char*) params.camera_name, (char *)"roll",   CARMEN_PARAM_DOUBLE, &camera_pose.orientation.roll, 0, NULL},
            {(char*) params.camera_name, (char *)"pitch",  CARMEN_PARAM_DOUBLE, &camera_pose.orientation.pitch, 0, NULL},
            {(char*) params.camera_name, (char *)"yaw",    CARMEN_PARAM_DOUBLE, &camera_pose.orientation.yaw, 0, NULL},

            {semitrailer_string, (char *)"M", CARMEN_PARAM_DOUBLE, &semitrailer_pose.position.x, 0, NULL},
        };

    num_items = sizeof(param_list_pos) / sizeof(param_list_pos[0]);
    carmen_param_install_params(argc, argv, param_list_pos, num_items);

    semitrailer_pose.position.x = -semitrailer_pose.position.x;
    semitrailer_pose.position.y = 0.0;
    semitrailer_pose.position.z = 0.0;
    semitrailer_pose.orientation.roll = 0.0;
    semitrailer_pose.orientation.pitch = 0.0;
    semitrailer_pose.orientation.yaw = 0.0;

    return 0;
}

void 
subscribe_messages()
{
    if (strncmp("intelbras", params.camera_name, 9) == 0)
       camera_drivers_subscribe_message(params.camera_id, NULL, (carmen_handler_t)image_handler_intelbras, CARMEN_SUBSCRIBE_LATEST);
    else
        carmen_bumblebee_basic_subscribe_stereoimage(params.camera_id, NULL, (carmen_handler_t)image_handler_bumblebee, CARMEN_SUBSCRIBE_LATEST);

    carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_handler, CARMEN_SUBSCRIBE_LATEST);
}


int 
main(int argc, char **argv)
{
    carmen_ipc_initialize(argc, argv);
    carmen_param_check_version(argv[0]);

    signal(SIGINT, shutdown_module);

    aruco_theta_tracker_read_parameters(argc, argv, params);

    read_parameters(argc, argv);
    initialize_transformations();

    subscribe_messages();
    carmen_ipc_dispatch();

    return 0;
}
