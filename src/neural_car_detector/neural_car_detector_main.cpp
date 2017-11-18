/*
 * neural_car_detector_main.cpp
 *
 *  Created on: 3 de jul de 2017
 *      Author: luan
 */

#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/velodyne_camera_calibration.h>
#include <carmen/camera_boxes_to_world.h>

// moving objects
#include <carmen/moving_objects_messages.h>
#include <carmen/moving_objects_interface.h>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// Module specific
#include "DetectNet.hpp"
#include "Darknet.hpp" /*< Yolo V2 */
#include "neural_car_detector.hpp"

#include <cstdlib> /*< std::getenv */
#include <fstream>

#define SHOW_DETECTIONS

// camera number and side
int camera;
int camera_side;

// velodyne buffer
const unsigned int maxPositions = 50;
carmen_velodyne_partial_scan_message *velodyne_message_arrange;
std::vector<carmen_velodyne_partial_scan_message> velodyne_vector;

#define USE_YOLO_V2 	1
#define USE_DETECTNET 	0

#if USE_DETECTNET
DetectNet *detectNet;
#elif USE_YOLO_V2
Detector *darknet;
std::vector<std::string> obj_names;
#endif


// Moving objects message
carmen_moving_objects_point_clouds_message moving_objects_point_clouds_message;
carmen_point_t globalpos;

/*
 This function find the closest velodyne message with the camera message
 */
carmen_velodyne_partial_scan_message
find_velodyne_most_sync_with_cam(double bumblebee_timestamp)
{
    carmen_velodyne_partial_scan_message velodyne;
    double minTimestampDiff = DBL_MAX;
    int minTimestampIndex = -1;
    for (unsigned int i = 0; i < velodyne_vector.size(); i++) {
        if (fabs(velodyne_vector[i].timestamp - bumblebee_timestamp) < minTimestampDiff) {
            minTimestampIndex = i;
            minTimestampDiff = fabs(velodyne_vector[i].timestamp - bumblebee_timestamp);
        }
    }
    velodyne = velodyne_vector[minTimestampIndex];
    return (velodyne);
}


void
build_moving_objects_message(std::vector<carmen_tracked_cluster_t> clusters)
{

    moving_objects_point_clouds_message.num_point_clouds = clusters.size();
    moving_objects_point_clouds_message.point_clouds = (t_point_cloud_struct *) (malloc(
            moving_objects_point_clouds_message.num_point_clouds * sizeof(t_point_cloud_struct)));


    for (int i = 0; i < moving_objects_point_clouds_message.num_point_clouds; i++) {
        carmen_vector_3D_t box_centroid = compute_centroid(clusters[i].points);
        carmen_vector_3D_t offset;

        offset.x = 0.572;
        offset.y = 0.0;
        offset.z = 2.154;

        box_centroid = translate_point(box_centroid, offset);
        box_centroid = rotate_point(box_centroid, globalpos.theta);

        offset.x = globalpos.x;
        offset.y = globalpos.y;
        offset.z = 0.0;

        box_centroid = translate_point(box_centroid, offset);

        moving_objects_point_clouds_message.point_clouds[i].r = 1.0;
        moving_objects_point_clouds_message.point_clouds[i].g = 1.0;
        moving_objects_point_clouds_message.point_clouds[i].b = 0.0;

        moving_objects_point_clouds_message.point_clouds[i].linear_velocity = 0;//clusters[i].linear_velocity;
        moving_objects_point_clouds_message.point_clouds[i].orientation = globalpos.theta;//clusters[i].orientation;

        moving_objects_point_clouds_message.point_clouds[i].object_pose.x = box_centroid.x;
        moving_objects_point_clouds_message.point_clouds[i].object_pose.y = box_centroid.y;
        moving_objects_point_clouds_message.point_clouds[i].object_pose.z = box_centroid.z;

        moving_objects_point_clouds_message.point_clouds[i].height = 1.8;
        moving_objects_point_clouds_message.point_clouds[i].length = 4.5;
        moving_objects_point_clouds_message.point_clouds[i].width = 1.6;

        switch (clusters[i].cluster_type) {

            case carmen_moving_object_type::pedestrian:
                moving_objects_point_clouds_message.point_clouds[i].geometric_model = 0;
                moving_objects_point_clouds_message.point_clouds[i].model_features.geometry.height = 1.8;
                moving_objects_point_clouds_message.point_clouds[i].model_features.geometry.length = 1.0;
                moving_objects_point_clouds_message.point_clouds[i].model_features.geometry.width = 1.0;
                moving_objects_point_clouds_message.point_clouds[i].model_features.red = 1.0;
                moving_objects_point_clouds_message.point_clouds[i].model_features.green = 1.0;
                moving_objects_point_clouds_message.point_clouds[i].model_features.blue = 0.8;
                moving_objects_point_clouds_message.point_clouds[i].model_features.model_name = (char *) "pedestrian";

                break;

            case carmen_moving_object_type::car:
                moving_objects_point_clouds_message.point_clouds[i].geometric_model = 0;
                moving_objects_point_clouds_message.point_clouds[i].model_features.geometry.height = 1.8;
                moving_objects_point_clouds_message.point_clouds[i].model_features.geometry.length = 4.5;
                moving_objects_point_clouds_message.point_clouds[i].model_features.geometry.width = 1.6;
                moving_objects_point_clouds_message.point_clouds[i].model_features.red = 1.0;
                moving_objects_point_clouds_message.point_clouds[i].model_features.green = 0.0;
                moving_objects_point_clouds_message.point_clouds[i].model_features.blue = 0.8;
                moving_objects_point_clouds_message.point_clouds[i].model_features.model_name = (char *) "car";

                break;
            default:
                moving_objects_point_clouds_message.point_clouds[i].geometric_model = 0;
                moving_objects_point_clouds_message.point_clouds[i].model_features.geometry.height = 1.8;
                moving_objects_point_clouds_message.point_clouds[i].model_features.geometry.length = 4.5;
                moving_objects_point_clouds_message.point_clouds[i].model_features.geometry.width = 1.6;
                moving_objects_point_clouds_message.point_clouds[i].model_features.red = 1.0;
                moving_objects_point_clouds_message.point_clouds[i].model_features.green = 1.0;
                moving_objects_point_clouds_message.point_clouds[i].model_features.blue = 0.0;
                moving_objects_point_clouds_message.point_clouds[i].model_features.model_name = (char *) "other";


                break;
        }

        moving_objects_point_clouds_message.point_clouds[i].num_associated = clusters[i].track_id;

        // fill the points
        moving_objects_point_clouds_message.point_clouds[i].point_size = clusters[i].points.size();
        moving_objects_point_clouds_message.point_clouds[i].points = (carmen_vector_3D_t *)
                malloc(moving_objects_point_clouds_message.point_clouds[i].point_size * sizeof(carmen_vector_3D_t));
        for (int j = 0; j < moving_objects_point_clouds_message.point_clouds[i].point_size; j++) {
            //TODO modificar isso
            carmen_vector_3D_t p;

            p.x = clusters[i].points[j].x;
            p.y = clusters[i].points[j].y;
            p.z = clusters[i].points[j].z;

            offset.x = 0.572;
            offset.y = 0.0;
            offset.z = 2.154;

            p = translate_point(p, offset);

            p = rotate_point(p, globalpos.theta);

            offset.x = globalpos.x;
            offset.y = globalpos.y;
            offset.z = 0.0;

            p = translate_point(p, offset);

            moving_objects_point_clouds_message.point_clouds[i].points[j] = p;
        }

    }

}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
publish_moving_objects(double timestamp)
{

    moving_objects_point_clouds_message.timestamp = timestamp;
    moving_objects_point_clouds_message.host = carmen_get_host();

    carmen_moving_objects_point_clouds_publish_message(&moving_objects_point_clouds_message);

    for (int i = 0; i < moving_objects_point_clouds_message.num_point_clouds; i++) {
        free(moving_objects_point_clouds_message.point_clouds[i].points);
    }
    free(moving_objects_point_clouds_message.point_clouds);
}

///////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
    cv::Mat src_image = cv::Mat(cv::Size(image_msg->width, image_msg->height - image_msg->height * 0.25), CV_8UC3);
    cv::Mat rgb_image = cv::Mat(cv::Size(image_msg->width, image_msg->height - image_msg->height * 0.25), CV_8UC3);

    double start_time, end_time, delta_t, fps;

    start_time = carmen_get_time();

    if (camera_side == 0)
        memcpy(src_image.data, image_msg->raw_left, image_msg->image_size * sizeof(char) - image_msg->image_size * 0.25 * sizeof(char));
    else
        memcpy(src_image.data, image_msg->raw_right, image_msg->image_size * sizeof(char) - image_msg->image_size * 0.25 * sizeof(char));

    carmen_velodyne_partial_scan_message velodyne_sync_with_cam;
    std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> points_lasers_in_cam_with_obstacle;

    if (velodyne_vector.size() > 0)
        velodyne_sync_with_cam = find_velodyne_most_sync_with_cam(image_msg->timestamp);
    else
        return;

    points_lasers_in_cam_with_obstacle = carmen_velodyne_camera_calibration_lasers_points_in_camera_with_obstacle_and_display(
            &velodyne_sync_with_cam, image_msg->width, image_msg->height);

    cv::cvtColor(src_image, rgb_image, cv::COLOR_RGB2BGR);

    std::vector<carmen_tracked_cluster_t> clusters;

    std::vector<bounding_box> bouding_boxes_list;
    // detect the objects in image
#if USE_DETECTNET
    //crop image
    float inv_aspect = 380.0 / 1250.0;

    cv::Rect roi;
    roi.width = rgb_image.cols;
    roi.height = rgb_image.cols * inv_aspect;
    roi.x = 0;
    roi.y = (rgb_image->rows - roi.height) / 2;

    cv::Mat crop = (rgb_image)(roi);

    std::vector<float> result = detectNet->Predict(crop);

    float correction_x = crop.cols / 1250.0;
    float correction_y = crop.rows / 380.0;

    for (int i = 0; i < 10; i++)
    {
        int xt = result[5*i] * correction_x;
        int yt = result[5*i + 1] * correction_y;

        int xb = result[5*i + 2] * correction_x;
        int yb = result[5*i + 3] * correction_y;

        if (result[5*i + 4] > 0.0)
        {
            bounding_box bbox;
            bbox.pt1.x = xt;
            bbox.pt1.y = yt + roi.y;
            bbox.pt2.x = xb;
            bbox.pt2.y = yb + roi.y;

            bouding_boxes_list.push_back(bbox);
        }
    }
#elif USE_YOLO_V2

    //0.3 threshold is good, more than this and it starts to miss some obstacles (very bad)
    std::vector<bbox_t> predictions = darknet->detect(src_image, 0.2);
    //TODO: change this to the better tracker
    predictions = darknet->tracking(predictions); /*< Coment this line if object tracking is not necessary */

    /* The bouding box returned by the detector is different than what is
	*	expected by this module, so we have to convert
    */
    for (const auto &box : predictions)
    {
        bounding_box bbox;

        bbox.pt1.x = box.x;
        bbox.pt1.y = box.y;
        bbox.pt2.x = box.x + box.w;
        bbox.pt2.y = box.y + box.h;

        bouding_boxes_list.push_back(bbox);
    }

#endif

    auto laser_points_in_camera_box_list = velodyne_points_in_boxes(bouding_boxes_list, &velodyne_sync_with_cam,
                                                                    image_msg->width, image_msg->height);

    std::vector<std::vector<carmen_vector_3D_t> > cluster_list = get_cluster_list(laser_points_in_camera_box_list);
    filter_points_in_clusters(&cluster_list);

    for (int i = 0; i < cluster_list.size(); i++)
    {
        carmen_moving_object_type tp = find_cluster_type_by_obj_id(obj_names, predictions.at(i).obj_id);

        //TODO: Isso eh pois so queremos mexer com pedestres no momento
        //if(tp != carmen_moving_object_type::pedestrian)
        //  continue;

        int cluster_id = predictions.at(i).track_id;

        carmen_tracked_cluster_t clust;

        clust.points = cluster_list.at(i);

        //TODO: Calcular velocidade e orientacao corretas (provavelmente usando um tracker)
        clust.orientation = globalpos.theta;
        clust.linear_velocity = 0.0;
        clust.track_id = cluster_id;
        clust.last_detection_timestamp = image_msg->timestamp;
        clust.cluster_type = tp;

        clusters.push_back(clust);
    }

    end_time = carmen_get_time();

    build_moving_objects_message(clusters);
    publish_moving_objects(image_msg->timestamp);

    delta_t = end_time - start_time;
    fps = 1.0 / delta_t;

#ifdef SHOW_DETECTIONS
    char confianca[7];
    char frame_rate[25];

    sprintf(frame_rate, "FPS = %.2f", fps);

    cv::putText(rgb_image, frame_rate,
                cv::Point(10, 25),
                cv::FONT_HERSHEY_PLAIN, 2, cvScalar(0, 255, 0), 2);

    for (unsigned int i = 0; i < laser_points_in_camera_box_list.size(); i++) {
        for (unsigned int j = 0; j < laser_points_in_camera_box_list[i].size(); j++) {
            cv::circle(rgb_image, cv::Point(laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipx,
                                            laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy), 1,
                       cv::Scalar(0, 0, 255), 1);
        }

        cv::Scalar object_color;


#if USE_DETECTNET
        sprintf(confianca, "%.3f", result[5*i + 4]);
        object_color = cv::Scalar(0,0,255);
#elif USE_YOLO_V2
        sprintf(confianca, "%.3f", predictions.at(i).prob);

        int obj_id = predictions.at(i).obj_id;

        std::string obj_name;
        if (obj_names.size() > obj_id)
            obj_name = obj_names[obj_id];

        if (obj_name.compare("car") == 0)
            object_color = cv::Scalar(0, 0, 255);
        else
            object_color = cv::Scalar(255, 0, 255);
#endif

        cv::rectangle(rgb_image,
                      cv::Point(bouding_boxes_list[i].pt1.x, bouding_boxes_list[i].pt1.y),
                      cv::Point(bouding_boxes_list[i].pt2.x, bouding_boxes_list[i].pt2.y),
                      object_color, 1);

        cv::putText(rgb_image, obj_name,
                    cv::Point(bouding_boxes_list[i].pt2.x + 1, bouding_boxes_list[i].pt1.y - 3),
                    cv::FONT_HERSHEY_PLAIN, 1, cvScalar(0, 0, 255), 1);

        cv::putText(rgb_image, confianca,
                    cv::Point(bouding_boxes_list[i].pt1.x + 1, bouding_boxes_list[i].pt1.y - 3),
                    cv::FONT_HERSHEY_PLAIN, 1, cvScalar(255, 255, 0), 1);

    }

    cv::Mat resized_image(cv::Size(640, 480 - 480 * 0.25), CV_8UC3);
    cv::resize(rgb_image, resized_image, resized_image.size());

    cv::imshow("Neural car detector", resized_image);
    cv::waitKey(1);

    resized_image.release();

#endif
}


void
velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
    velodyne_message_arrange = velodyne_message;
    carmen_velodyne_camera_calibration_arrange_velodyne_vertical_angles_to_true_position(velodyne_message_arrange);

    carmen_velodyne_partial_scan_message velodyne_copy;

    velodyne_copy.host = velodyne_message_arrange->host;
    velodyne_copy.number_of_32_laser_shots = velodyne_message_arrange->number_of_32_laser_shots;
    velodyne_copy.partial_scan = (carmen_velodyne_32_laser_shot *) malloc(
            sizeof(carmen_velodyne_32_laser_shot) * velodyne_message_arrange->number_of_32_laser_shots);
    memcpy(velodyne_copy.partial_scan, velodyne_message_arrange->partial_scan,
           sizeof(carmen_velodyne_32_laser_shot) * velodyne_message_arrange->number_of_32_laser_shots);
    velodyne_copy.timestamp = velodyne_message_arrange->timestamp;

    velodyne_vector.push_back(velodyne_copy);

    if (velodyne_vector.size() > maxPositions) {
        free(velodyne_vector.begin()->partial_scan);
        velodyne_vector.erase(velodyne_vector.begin());
    }
    //	show_velodyne(velodyne_message);
}


void
carmen_localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *globalpos_message)
{
    globalpos.theta = globalpos_message->globalpos.theta;
    globalpos.x = globalpos_message->globalpos.x;
    globalpos.y = globalpos_message->globalpos.y;
}


void
shutdown_module(int signo)
{
    if (signo == SIGINT) {
        carmen_ipc_disconnect();
        cvDestroyAllWindows();

        printf("Neural car detector: disconnected.\n");
        exit(0);
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////

std::vector<std::string> objects_names_from_file(std::string const filename)
{
    std::ifstream file(filename);
    std::vector<std::string> file_lines;
    if (!file.is_open()) return file_lines;
    for (std::string line; getline(file, line);) file_lines.push_back(line);
    std::cout << "object names loaded \n";
    return file_lines;
}

void
subscribe_messages()
{
    carmen_bumblebee_basic_subscribe_stereoimage(camera,
                                                 NULL, (carmen_handler_t) image_handler,
                                                 CARMEN_SUBSCRIBE_LATEST);

    carmen_velodyne_subscribe_partial_scan_message(NULL,
                                                   (carmen_handler_t) velodyne_partial_scan_message_handler,
                                                   CARMEN_SUBSCRIBE_LATEST);

    carmen_localize_ackerman_subscribe_globalpos_message(NULL,
                                                         (carmen_handler_t) carmen_localize_ackerman_globalpos_message_handler,
                                                         CARMEN_SUBSCRIBE_LATEST);

}


int
main(int argc, char **argv)
{

    if (argc != 3) {
        fprintf(stderr,
                "%s: Wrong number of parameters. Neural_car_detector requires 2 parameter and received %d. \n Usage: %s <camera_number> <camera_side(0-left; 1-right)\n>",
                argv[0], argc - 1, argv[0]);
        exit(1);
    }


    int gpu = 1;
    int device_id = 0;

    /**** DETECTNET PARAMETERS ****/
    std::string model_file = "deploy.prototxt";
    std::string trained_file = "snapshot_iter_21600.caffemodel";

    /**** YOLO PARAMETERS ****/
    std::string darknet_home = std::getenv("DARKNET_HOME"); /*< get environment variable pointing path of darknet*/
    if (darknet_home.empty())
        printf("Cannot find darknet path. Check if you have correctly set DARKNET_HOME environment variable.\n");
    std::string cfg_filename = darknet_home + "/cfg/yolo.cfg";
    std::string weight_filename = darknet_home + "/data/yolo.weights";
    std::string voc_names = darknet_home + "/data/coco.names";
    obj_names = objects_names_from_file(voc_names);

#if USE_DETECTNET
    detectNet = new DetectNet(model_file, trained_file, gpu, device_id);
    carmen_test_alloc(detectNet);
#elif USE_YOLO_V2
    darknet = new Detector(cfg_filename, weight_filename, device_id);
    carmen_test_alloc(darknet);
#endif

#ifdef SHOW_DETECTIONS
    cv::namedWindow("Neural car detector", cv::WINDOW_AUTOSIZE);
#endif

    setlocale(LC_ALL, "C");

    camera = atoi(argv[1]);
    camera_side = atoi(argv[2]);

    carmen_ipc_initialize(argc, argv);
    signal(SIGINT, shutdown_module);

    subscribe_messages();
    carmen_ipc_dispatch();

    return 0;
}
