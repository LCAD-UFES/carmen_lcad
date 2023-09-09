#define BOOST_SIGNALS_NO_DEPRECATION_WARNING

#include <vector>
#include <fstream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <jsoncpp/json/json.h>

#include "aruco_theta_tracker.hpp"


#define BOARD1 1
#define THETA1 1


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
check_image_index(camera_message *msg, aruco_theta_tracker_parameters params)
{
    if (params.image_index_to_show > (msg->number_of_images - 1))
    {
        printf("\nthe image index exceeds the maximun %d! \n \n setting the image index to 0!\n", msg->number_of_images - 1);
    }
    params.image_index_to_show = 0;
}


void 
aruco_theta_tracker_detect(camera_message *msg, aruco_theta_tracker_parameters params, cv::Mat &tvec, cv::Mat &rvec, int &n_markers_detected, double &offset)
{
    check_image_index(msg, params);

    int width, height;
    static int first_time = 1;

    width = msg->images[params.image_index_to_show].width;
    height = msg->images[params.image_index_to_show].height;

    cv::Mat cv_image = cv::Mat(height, width, CV_8UC3, msg->images[params.image_index_to_show].raw_data, 0);
    if (height*width <= 0)
        return;

    if (msg->undistorted == 2) // retrocompatibilidade
        cvtColor(cv_image, cv_image, cv::COLOR_RGB2BGR);

    if (first_time)
    {
        aruco::CameraParameters camera;
        cv::Mat camMatrix = (cv::Mat_<double>(3, 3) << params._fx, .0, params._cu, .0, params._fy, params._cv, .0, .0, 1.);
        cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << params._k1, params._k2, params._p1, params._p2, params._k3);

        camera.setParams(camMatrix, distCoeffs, cv::Size(height, width));

        for (auto it : params.detectors)
            it->setCamera(camera);

        first_time = 0;
    }

    int max_identified = 0;
    for (auto it : params.detectors)
    {
        n_markers_detected = it->detect(cv_image);
        if (n_markers_detected > 0)
        {
            if (n_markers_detected > max_identified)
            {
                tvec = it->getTvec();
                rvec = it->getRvec();
                offset = it->offset;
            }
        }
    }

    if (params.show_output)
    {
        char info[32];
        cv::Mat imag_rcz;
        cv::resize(cv_image, imag_rcz, cv::Size(), 640.0/(double)cv_image.cols, 480.0/(double)cv_image.rows);

        sprintf(info, "%dx%d", width, height);
        putText(imag_rcz, info, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, .7, cv::Scalar(0, 0, 0), 4);
        putText(imag_rcz, info, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, .7, cv::Scalar(255, 255, 255), 2);

        sprintf(info, "%.2fFPS", filter_fps(msg->timestamp));
        putText(imag_rcz, info, cv::Point(10, 55), cv::FONT_HERSHEY_SIMPLEX, .7, cv::Scalar(0, 0, 0), 4);
        putText(imag_rcz, info, cv::Point(10, 55), cv::FONT_HERSHEY_SIMPLEX, .7, cv::Scalar(255, 255, 255), 2);

        sprintf(info, "Aruco %d", params.detector_id);

        cv::imshow(info, imag_rcz);
        cv::waitKey(1);
    }
}


void 
aruco_theta_tracker_detect(carmen_bumblebee_basic_stereoimage_message *msg, aruco_theta_tracker_parameters params, cv::Mat &tvec, cv::Mat &rvec, int &n_markers_detected, double &offset)
{
    int width, height;
    static int first_time = 1;

    width = msg->width;
    height = msg->height;

    cv::Mat cv_image = cv::Mat(height, width, CV_8UC3, msg->raw_left, 0);
    if (height*width <= 0)
        return;

    cvtColor(cv_image, cv_image, cv::COLOR_RGB2BGR);

    if (first_time)
    {
        aruco::CameraParameters camera;
        cv::Mat camMatrix = (cv::Mat_<double>(3, 3) << params._fx, .0, params._cu, .0, params._fy, params._cv, .0, .0, 1.);
        cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << params._k1, params._k2, params._p1, params._p2, params._k3);

        camera.setParams(camMatrix, distCoeffs, cv::Size(height, width));

        for (auto it : params.detectors)
            it->setCamera(camera);

        first_time = 0;
    }

    int max_identified = 0;
    for (auto it : params.detectors)
    {
        n_markers_detected = it->detect(cv_image);
        if (n_markers_detected > 0)
        {
            if (n_markers_detected > max_identified)
            {
                tvec = it->getTvec();
                rvec = it->getRvec();
                offset = it->offset;

                max_identified = n_markers_detected;
            }
        }
    }
    n_markers_detected = max_identified;

    if (params.show_output)
    {
        char info[32];
        cv::Mat imag_rcz;
        cv::resize(cv_image, imag_rcz, cv::Size(), 640.0/(double)cv_image.cols, 480.0/(double)cv_image.rows);

        sprintf(info, "%dx%d", width, height);
        putText(imag_rcz, info, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, .7, cv::Scalar(0, 0, 0), 4);
        putText(imag_rcz, info, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, .7, cv::Scalar(255, 255, 255), 2);

        sprintf(info, "%.2fFPS", filter_fps(msg->timestamp));
        putText(imag_rcz, info, cv::Point(10, 55), cv::FONT_HERSHEY_SIMPLEX, .7, cv::Scalar(0, 0, 0), 4);
        putText(imag_rcz, info, cv::Point(10, 55), cv::FONT_HERSHEY_SIMPLEX, .7, cv::Scalar(255, 255, 255), 2);

        sprintf(info, "Aruco %d", params.detector_id);

        cv::imshow(info, imag_rcz);
        cv::waitKey(1);
    }
}


int
read_json_from_file(char* infile, Json::Value &root) {
    std::ifstream ifs;

    ifs.open(infile);
    if (!ifs)
        return EXIT_FAILURE;

    Json::CharReaderBuilder builder;
    builder["collectComments"] = true;
    JSONCPP_STRING errs;
    if (!parseFromStream(builder, ifs, &root, &errs))
    return EXIT_FAILURE;
  
    return EXIT_SUCCESS;
}


int 
aruco_theta_tracker_read_parameters(int argc, char **argv, aruco_theta_tracker_parameters &params)
{
    Json::Value values;
    double marker_size_in_meters;
    int num_items;
    char *json_file = NULL, *board_config_file = NULL, *aruco_dictionary = NULL;

    // ArUCO parameters
    carmen_param_allow_unfound_variables(1);
    carmen_param_t param_list_json[] =
        {
            {(char *) "commandline", (char *) "json",        CARMEN_PARAM_STRING, &json_file,           0, NULL},
        };
    num_items = sizeof(param_list_json) / sizeof(param_list_json[0]);
    carmen_param_install_params(argc, argv, param_list_json, num_items);

    if (json_file)
    {
        read_json_from_file(json_file, values);

        params.camera_name = (char*) malloc(strlen(values["camera_name"].asCString())*sizeof(char));
        strcpy(params.camera_name, values["camera_name"].asCString());
        params.camera_id = values["camera_id"].asInt();
        params.detector_id = values["detector_id"].asInt();
        
        for (auto it : values["detectors"])
        {
            if (it["board"].asString().size())
                params.detectors.push_back(new PoseTracker(it["marker_size"].asDouble(), it["board"].asCString(), it["offset"].asDouble()));
            else if (it["dictionary"].asString().size())
                params.detectors.push_back(new PoseTracker(it["marker_size"].asDouble(), it["dictionary"].asCString(), it["offset"].asDouble()));
        }

        std::cout << "detecting by loaded json \e[0;32m" << json_file << "\e[0m" << std::endl;
    }
    else
    {
        carmen_param_allow_unfound_variables(0);
        carmen_param_t param_list_cm[] =
            {
                {(char *) "commandline", (char *) "camera_name",        CARMEN_PARAM_STRING, &params.camera_name,           0, NULL},
                {(char *) "commandline", (char *) "camera_id",          CARMEN_PARAM_INT,    &params.camera_id,             0, NULL},
                {(char *) "commandline", (char *) "marker_size",        CARMEN_PARAM_DOUBLE, &marker_size_in_meters, 0, NULL},
                {(char *) "commandline", (char *) "detector_id",        CARMEN_PARAM_INT,    &params.detector_id,           0, NULL},
            };
        num_items = sizeof(param_list_cm) / sizeof(param_list_cm[0]);
        carmen_param_install_params(argc, argv, param_list_cm, num_items);

        carmen_param_allow_unfound_variables(1);
        carmen_param_t param_list_cm_[] =
            {
                {(char *) "commandline", (char *) "board",               CARMEN_PARAM_STRING, &board_config_file,     0, NULL},
                {(char *) "commandline", (char *) "show",         CARMEN_PARAM_ONOFF,  &params.show_output,           0, NULL},
                {(char *) "commandline", (char *) "dictionary",          CARMEN_PARAM_STRING, &aruco_dictionary,      0, NULL},
                {(char *) "commandline", (char *) "image_index_to_show", CARMEN_PARAM_INT,    &params.image_index_to_show,   0, NULL},
            };
        num_items = sizeof(param_list_cm_) / sizeof(param_list_cm_[0]);
        carmen_param_install_params(argc, argv, param_list_cm_, num_items);

        if (board_config_file)
            params.detectors.push_back(new PoseTracker(marker_size_in_meters, board_config_file));

        else
        {
            if (aruco_dictionary == NULL)
            {
                aruco_dictionary = (char*) malloc(sizeof(char)*16);
                strcpy(aruco_dictionary, "ARUCO_MIP_36h12");
            }
            params.detectors.push_back(new PoseTracker(marker_size_in_meters, aruco_dictionary));
        }

        if (!board_config_file)
        {
            std::cout << "detecting by single markers" << std::endl;
            std::cout << "dictionary: " << aruco_dictionary << std::endl;
        }
        else
            std::cout << "detecting by board config " << board_config_file << std::endl;
    }
    carmen_param_allow_unfound_variables(1);
    carmen_param_t param_list_show[] =
        {
            {(char *) "commandline", (char *) "show",         CARMEN_PARAM_ONOFF,  &params.show_output,           0, NULL},
        };
    num_items = sizeof(param_list_show) / sizeof(param_list_show[0]);
    carmen_param_install_params(argc, argv, param_list_show, num_items);

    // camera parameters
    carmen_param_allow_unfound_variables(0);
    carmen_param_t param_list[] =
        {
            {(char*) params.camera_name, (char *)"fx", CARMEN_PARAM_DOUBLE, &params._fx, 0, NULL},
            {(char*) params.camera_name, (char *)"fy", CARMEN_PARAM_DOUBLE, &params._fy, 0, NULL},
            {(char*) params.camera_name, (char *)"cu", CARMEN_PARAM_DOUBLE, &params._cu, 0, NULL},
            {(char*) params.camera_name, (char *)"cv", CARMEN_PARAM_DOUBLE, &params._cv, 0, NULL},
            {(char*) params.camera_name, (char *)"k1", CARMEN_PARAM_DOUBLE, &params._k1, 0, NULL},
            {(char*) params.camera_name, (char *)"k2", CARMEN_PARAM_DOUBLE, &params._k2, 0, NULL},
            {(char*) params.camera_name, (char *)"p1", CARMEN_PARAM_DOUBLE, &params._p1, 0, NULL},
            {(char*) params.camera_name, (char *)"p2", CARMEN_PARAM_DOUBLE, &params._p2, 0, NULL},
            {(char*) params.camera_name, (char *)"k3", CARMEN_PARAM_DOUBLE, &params._k3, 0, NULL},
        };

    num_items = sizeof(param_list) / sizeof(param_list[0]);
    carmen_param_install_params(argc, argv, param_list, num_items);

    return 0;
}
