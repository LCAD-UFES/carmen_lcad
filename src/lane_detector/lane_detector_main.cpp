/*
 * Lane_car_detector_main.cpp
 *
 *  Created on: 28 de fev de 2018
 *      Author: marcelo
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
//#include "DetectNet.hpp"
#include "Darknet.hpp" /*< Yolo V2 */
#include "lane_detector.hpp"

#include <carmen/lane_detector_messages.h>
#include <carmen/lane_detector_interface.h>

#include <cstdlib> /*< std::getenv */
#include <fstream>
#include <math.h>

#define SHOW_DETECTIONS
#define SHOW_BBOX true
// camera number and side
int camera;
int camera_side;

carmen_camera_parameters camera_parameters;
carmen_pose_3D_t velodyne_pose;
carmen_pose_3D_t camera_pose;

// velodyne buffer
const unsigned int maxPositions = 50;
carmen_velodyne_partial_scan_message *velodyne_message_arrange;
std::vector<carmen_velodyne_partial_scan_message> velodyne_vector;

#define USE_YOLO_V2 publish	1
#define USE_DETECTNET 	0


Detector *darknet;
std::vector<std::string> obj_names;


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
    for (unsigned int i = 0; i < velodyne_vector.size(); i++)
    {
        if (fabs(velodyne_vector[i].timestamp - bumblebee_timestamp) < minTimestampDiff)
        {
            minTimestampIndex = i;
            minTimestampDiff = fabs(velodyne_vector[i].timestamp - bumblebee_timestamp);
        }
    }
    velodyne = velodyne_vector[minTimestampIndex];
    return (velodyne);
}





///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

static void
register_ipc_messages(void)
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_LANE_NAME, IPC_VARIABLE_LENGTH, CARMEN_LANE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LANE_NAME);
}

void
lane_publish_messages(double _timestamp, std::vector< std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> > &laser_points_in_camera_box_list,
		std::vector<bool> &left_or_right)
{
	// stamps the message
	carmen_lane_detector_lane_message_t message;
	message.host = carmen_get_host();
	message.timestamp = _timestamp;
	message.lane_vector_size = laser_points_in_camera_box_list.size();
	if (laser_points_in_camera_box_list.size() == 0)
		return ;
	message.lane_vector = (carmen_lane_detector_lane_t*) malloc ( message.lane_vector_size * sizeof (carmen_lane_detector_lane_t) );

	for (int i = 0; i < laser_points_in_camera_box_list.size(); i++)
	{
		unsigned int idx_pt1 = 0, idx_pt2 = 0;
		unsigned int x_min = 10000, x_max = 0, y_min = 10000, y_max = 0, idx_pt1_b, idx_pt2_b;
		for(int j = 0; j < laser_points_in_camera_box_list[i].size(); j++)
		{
			if (left_or_right[i] == false)
			{
				if (laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipx < x_min)
					if (laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy < y_min)
					{
						x_min = laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipx;
						y_min = laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy;
						idx_pt1_b = idx_pt1 = j;
					}
				if (laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipx > x_max)
					if (laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy > y_max)
					{
						x_max = laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipx;
						y_max = laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy;
						idx_pt2_b = idx_pt2 = j;
					}
			}else
			{
				if (laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipx < x_min)
					if (laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy > y_max)
					{
						x_min = laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipx;
						y_max = laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy;
						idx_pt2_b = idx_pt2 = j;
					}
				if (laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipx > x_max)
					if (laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy < y_min)
					{
						x_max = laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipx;
						y_min = laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy;
						idx_pt1_b = idx_pt1 = j;
					}
			}
		}
		for(int j = 0; j < laser_points_in_camera_box_list[i].size(); j++)
		{
			if (left_or_right[i] == false)
			{
				if (laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipx < x_min )
					if (laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy < y_min &&
							laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy != laser_points_in_camera_box_list[i][idx_pt1_b].velodyne_points_in_cam.ipy )
					{
						x_min = laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipx;
						y_min = laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy;
						idx_pt1 = j;
					}
				if (laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipx > x_max)
					if (laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy > y_max &&
							laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy != laser_points_in_camera_box_list[i][idx_pt2_b].velodyne_points_in_cam.ipy)
					{
						x_max = laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipx;
						y_max = laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy;
						idx_pt2 = j;
					}
			}else
			{
				if (laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipx < x_min )
					if (laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy > y_max &&
							laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy != laser_points_in_camera_box_list[i][idx_pt2_b].velodyne_points_in_cam.ipy)
					{
						x_min = laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipx;
						y_max = laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy;
						idx_pt2 = j;
					}
				if (laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipx > x_max )
					if (laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy < y_min &&
							laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy != laser_points_in_camera_box_list[i][idx_pt1_b].velodyne_points_in_cam.ipy)
					{
						x_max = laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipx;
						y_min = laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy;
						idx_pt1 = j;
					}
			}
		}
		carmen_vector_3D_t p1, p2;
		carmen_vector_3D_t offset;
		p1 = carmen_covert_sphere_to_cartesian_coord(laser_points_in_camera_box_list[i][idx_pt1].velodyne_points_in_cam.laser_polar);
		p2 = carmen_covert_sphere_to_cartesian_coord(laser_points_in_camera_box_list[i][idx_pt2].velodyne_points_in_cam.laser_polar);
	/*	std:: cout <<"i " << i << '\n';
		std:: cout <<"p1 x " << laser_points_in_camera_box_list[i][idx_pt1].velodyne_points_in_cam.ipx << " y " << laser_points_in_camera_box_list[i][idx_pt1].velodyne_points_in_cam.ipy << '\n';
		std:: cout <<"p2 x " << laser_points_in_camera_box_list[i][idx_pt2].velodyne_points_in_cam.ipx << " y " << laser_points_in_camera_box_list[i][idx_pt2].velodyne_points_in_cam.ipy << '\n';*/
        offset.x = 0.572;
        offset.y = 0.0;
        offset.z = 2.154;

        p1 = translate_point(p1, offset);
        p2 = translate_point(p2, offset);

        p1 = rotate_point(p1, globalpos.theta);
        p2 = rotate_point(p2, globalpos.theta);

        offset.x = globalpos.x;
        offset.y = globalpos.y;
        offset.z = 0.0;

        p1 = translate_point(p1, offset);
        p2 = translate_point(p2, offset);

		message.lane_vector[i].lane_segment_position1.x = p1.x;
		message.lane_vector[i].lane_segment_position1.y = p1.y;
		message.lane_vector[i].lane_segment_position2.x = p2.x;
		message.lane_vector[i].lane_segment_position2.y = p2.y;
		message.lane_vector[i].lane_class = 0;

		if (left_or_right[i] == true)
		{
			message.lane_vector[i].left = 1;
		}else
		{
			message.lane_vector[i].left = 0;
		}
	}
	// publish!
	carmen_lane_publish_message(&message);
	free(message.lane_vector);

}


std::vector< std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> >
velodyne_points_in_lane_bboxes(std::vector<bounding_box> bouding_boxes_list,
		carmen_velodyne_partial_scan_message *velodyne_sync_with_cam,
        carmen_camera_parameters camera_parameters,
		carmen_pose_3D_t velodyne_pose, carmen_pose_3D_t camera_pose,
		unsigned int width, unsigned int height, std::vector<bool> *left_or_right)
{

	std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> points_lasers_in_cam_with_obstacle;

	// Removes the ground, Removes points outside cameras field of view and Returns the points that are obstacles
	points_lasers_in_cam_with_obstacle = carmen_velodyne_camera_calibration_lasers_points_in_camera_with_obstacle_and_display(
				velodyne_sync_with_cam,camera_parameters, velodyne_pose,camera_pose, width, height);

	std::vector< std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> > laser_points_in_camera_box_list;
	for (unsigned int i = 0; i < bouding_boxes_list.size(); i++)
	{
		std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> laser_points_in_camera_box;
		for (unsigned int j = 0; j < points_lasers_in_cam_with_obstacle.size(); j++)
		{
			if (points_lasers_in_cam_with_obstacle[j].velodyne_points_in_cam.ipx > bouding_boxes_list[i].pt1.x
					&& points_lasers_in_cam_with_obstacle[j].velodyne_points_in_cam.ipx < bouding_boxes_list[i].pt2.x)
			{
				if (left_or_right[0][i] == false)
				{
					if (points_lasers_in_cam_with_obstacle[j].velodyne_points_in_cam.ipy > bouding_boxes_list[i].pt1.y
						&& points_lasers_in_cam_with_obstacle[j].velodyne_points_in_cam.ipy < bouding_boxes_list[i].pt2.y)
							laser_points_in_camera_box.push_back(points_lasers_in_cam_with_obstacle[j]);
				}else
				{
					if (points_lasers_in_cam_with_obstacle[j].velodyne_points_in_cam.ipy < bouding_boxes_list[i].pt1.y
						&& points_lasers_in_cam_with_obstacle[j].velodyne_points_in_cam.ipy > bouding_boxes_list[i].pt2.y)
							laser_points_in_camera_box.push_back(points_lasers_in_cam_with_obstacle[j]);
				}
			}
		}

		laser_points_in_camera_box_list.push_back(laser_points_in_camera_box);
	}
	return (laser_points_in_camera_box_list);
}

void
put_the_lane_dectections_in_image(std::vector< std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> > laser_points_in_camera_box_list,
	const cv::Mat& rgb_image, std::vector<bbox_t> predictions, std::vector<bounding_box> bouding_boxes_list, int end_time, int start_time) {
	double fps;
	int delta_t = end_time - start_time;
	fps = 1.0 / delta_t;
	char confianca[7];
	char frame_rate[25];
	sprintf(frame_rate, "FPS = %.2f", fps);
	cv::putText(rgb_image, frame_rate, cv::Point(10, 25), cv::FONT_HERSHEY_PLAIN, 2, cvScalar(0, 255, 0), 2);
	if(SHOW_BBOX)
	{
		for (unsigned int i = 0; i < laser_points_in_camera_box_list.size(); i++)
		{
			cv::rectangle(rgb_image, cv::Point(bouding_boxes_list[i].pt1.x, bouding_boxes_list[i].pt1.y),
			cv::Point(bouding_boxes_list[i].pt2.x, bouding_boxes_list[i].pt2.y), cvScalar(0, 255, 0), 3);
		}
	}else
	{
		for (unsigned int i = 0; i < laser_points_in_camera_box_list.size(); i++)
		{
			cv::line(rgb_image, cv::Point(bouding_boxes_list[i].pt1.x, bouding_boxes_list[i].pt1.y),
			cv::Point(bouding_boxes_list[i].pt2.x, bouding_boxes_list[i].pt2.y), cvScalar(0, 255, 0), 3);
		}
	}
	for (unsigned int i = 0; i < laser_points_in_camera_box_list.size(); i++)
	{
		for (unsigned int j = 0; j < laser_points_in_camera_box_list[i].size(); j++)
		{
			cv::circle(rgb_image, cv::Point(laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipx,
			laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy), 1, cv::Scalar(255, 0, 0), 3, 0);
		}
	}
	cv::Mat resized_image(cv::Size(640, 480), CV_8UC3);
	cv::resize(rgb_image, resized_image, resized_image.size());
	cv::imshow("Lane detector", resized_image);
	cv::waitKey(1);
	resized_image.release();
}

std::vector<bbox_t>
detection_of_the_lanes(std::vector<bounding_box> *bouding_boxes_list, cv::Mat& rgb_image, std::vector<bool> *left_or_right) {
	// detect the objects in image
	std::vector<bbox_t> predictions = darknet->detect(rgb_image, 0.3);	//find x min and x max for orientation of the lines
	unsigned int x_min = 10000, x_max = 0;
	for (int i = 0; i < predictions.size(); i++)
	{
		if (predictions[i].x < x_min)
			x_min = predictions[i].x;

		if ((predictions[i].x + predictions[i].w) > x_max)
			x_max = (predictions[i].x + predictions[i].w);
	}
	//Conversion from yolo format to bounding_box format
	for (const auto& box : predictions)
	{
		bounding_box bbox;
		bbox.pt1.x = box.x;
		bbox.pt2.x = box.x + box.w;
		if (box.x + box.w > x_min + (x_max - x_min) / 2)
		{
			bbox.pt1.y = box.y;
			bbox.pt2.y = box.y + box.h;
			left_or_right->push_back(false);
		} else
		{
			bbox.pt2.y = box.y;
			bbox.pt1.y = box.y + box.h;
			left_or_right->push_back(true);
		}
		bouding_boxes_list->push_back(bbox);
	}
	return (predictions);
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
	cv::Mat src_image = cv::Mat(cv::Size(image_msg->width, image_msg->height), CV_8UC3);
	cv::Mat rgb_image = cv::Mat(cv::Size(image_msg->width, image_msg->height), CV_8UC3);

	double start_time, end_time, delta_t, fps;

	start_time = carmen_get_time();

	if (camera_side == 0) {
		memcpy(src_image.data, image_msg->raw_left, image_msg->image_size * sizeof(char));
	} else {
		memcpy(src_image.data, image_msg->raw_right, image_msg->image_size * sizeof(char));
	}
    carmen_velodyne_partial_scan_message velodyne_sync_with_cam;

    if (velodyne_vector.size() > 0)
        velodyne_sync_with_cam = find_velodyne_most_sync_with_cam(image_msg->timestamp);
    else
        return;
    cv::cvtColor(src_image, rgb_image, cv::COLOR_RGB2BGR);
    std::vector<bounding_box> bouding_boxes_list;
    //left = true e right = false
    std::vector<bool> left_or_right;
    // detect the objects in image
	std::vector<bbox_t> predictions = detection_of_the_lanes(&bouding_boxes_list, src_image, &left_or_right);
    std::vector< std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> > laser_points_in_camera_box_list = velodyne_points_in_lane_bboxes(bouding_boxes_list,
    		&velodyne_sync_with_cam, camera_parameters, velodyne_pose, camera_pose, image_msg->width, image_msg->height, &left_or_right);
    end_time = carmen_get_time();
	put_the_lane_dectections_in_image(laser_points_in_camera_box_list, rgb_image, predictions, bouding_boxes_list, end_time, start_time);
	lane_publish_messages(image_msg-> timestamp, laser_points_in_camera_box_list, left_or_right);
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
}




void
shutdown_module(int signo)
{
    if (signo == SIGINT)
    {
        carmen_ipc_disconnect();
        cvDestroyAllWindows();
        printf("Lane detector: disconnected.\n");
        exit(0);
    }
}


std::vector<std::string>
objects_names_from_file(std::string const filename)
{
    std::ifstream file(filename);
    std::vector<std::string> file_lines;
    if (!file.is_open()) return file_lines;
    for (std::string line; getline(file, line);) file_lines.push_back(line);
    std::cout << "object names loaded \n";

    return (file_lines);
}

void
localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *globalpos_message)
{
    globalpos.theta = globalpos_message->globalpos.theta;
    globalpos.x = globalpos_message->globalpos.x;
    globalpos.y = globalpos_message->globalpos.y;
}

void
subscribe_messages()
{
    carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) image_handler,
         CARMEN_SUBSCRIBE_LATEST);

    carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t) velodyne_partial_scan_message_handler,
          CARMEN_SUBSCRIBE_LATEST);

    carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);

}


int
read_parameters(int argc, char **argv)
{
    /* defining the camera to be used */
    camera = atoi(argv[1]);
    camera_side = atoi(argv[2]);

    int num_items;
    char bumblebee_string[256];
    char camera_string[256];

    sprintf(bumblebee_string, "%s%d", "bumblebee_basic", camera);
    sprintf(camera_string, "%s%d", "camera", camera);

    carmen_param_t param_list[] =
    {
		{ bumblebee_string, (char*) "fx", CARMEN_PARAM_DOUBLE, &camera_parameters.fx_factor, 0, NULL },
		{ bumblebee_string, (char*) "fy", CARMEN_PARAM_DOUBLE, &camera_parameters.fy_factor, 0, NULL },
		{ bumblebee_string, (char*) "cu", CARMEN_PARAM_DOUBLE, &camera_parameters.cu_factor, 0, NULL },
		{ bumblebee_string, (char*) "cv", CARMEN_PARAM_DOUBLE, &camera_parameters.cv_factor, 0, NULL },
		{ bumblebee_string, (char*) "pixel_size", CARMEN_PARAM_DOUBLE, &camera_parameters.pixel_size, 0, NULL },

		{(char *) "velodyne",  (char *) "x", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.x), 0, NULL},
		{(char *) "velodyne",  (char *) "y", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.y), 0, NULL},
		{(char *) "velodyne",  (char *) "z", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.z), 0, NULL},
		{(char *) "velodyne",  (char *) "roll", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.roll), 0, NULL},
		{(char *) "velodyne",  (char *) "pitch", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.pitch), 0, NULL},
		{(char *) "velodyne",  (char *) "yaw", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.yaw), 0, NULL},

		{ camera_string, (char*) "x", CARMEN_PARAM_DOUBLE, &camera_pose.position.x, 0, NULL },
		{ camera_string, (char*) "y", CARMEN_PARAM_DOUBLE, &camera_pose.position.y, 0, NULL },
		{ camera_string, (char*) "z", CARMEN_PARAM_DOUBLE, &camera_pose.position.z, 0, NULL },
		{ camera_string, (char*) "roll", CARMEN_PARAM_DOUBLE, &camera_pose.orientation.roll, 0, NULL },
		{ camera_string, (char*) "pitch", CARMEN_PARAM_DOUBLE, &camera_pose.orientation.pitch, 0, NULL },
		{ camera_string, (char*) "yaw", CARMEN_PARAM_DOUBLE, &camera_pose.orientation.yaw, 0, NULL }
    };


    num_items = sizeof(param_list) / sizeof(param_list[0]);
    carmen_param_install_params(argc, argv, param_list, num_items);

    return 0;
}


int
main(int argc, char **argv)
{
    if ((argc != 3))
        carmen_die("%s: Wrong number of parameters. Neural_car_detector requires 2 parameter and received %d. \n Usage: %s <camera_number> <camera_side(0-left; 1-right)\n>",
                   argv[0], argc - 1, argv[0]);

    int gpu = 1;
    int device_id = 0;
    /**** DETECTNET PARAMETERS ****/
    std::string model_file = "deploy.prototxt";
    std::string trained_file = "snapshot_iter_21600.caffemodel";
    /**** YOLO PARAMETERS ****/
    std::string darknet_home = std::getenv("DARKNET_HOME"); /*< get environment variable pointing path of darknet*/
    if (darknet_home.empty())
        printf("Cannot find darknet path. Check if you have correctly set DARKNET_HOME environment variable.\n");
    std::string cfg_filename = darknet_home + "/cfg/yolo_voc_lane.cfg";
    std::string weight_filename = darknet_home + "/yolo_lane.weights";
    std::string voc_names = darknet_home + "/data/lane.names";
    obj_names = objects_names_from_file(voc_names);
    darknet = new Detector(cfg_filename, weight_filename, device_id);
    carmen_test_alloc(darknet);

#ifdef SHOW_DETECTIONS
    cv::namedWindow("Lane detector", cv::WINDOW_AUTOSIZE);
#endif

    setlocale(LC_ALL, "C");

    carmen_ipc_initialize(argc, argv);
    signal(SIGINT, shutdown_module);

    read_parameters(argc, argv);

    // define the messages
    register_ipc_messages();

    subscribe_messages();
    carmen_ipc_dispatch();

    return 0;
}
