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
#include "carmen_darknet_interface.hpp" /*< Yolo V2 */
#include "lane_detector.hpp"

#include <carmen/lane_detector_messages.h>
#include <carmen/lane_detector_interface.h>
#include <cstdio>
#include <cstdlib> /*< std::getenv */
#include <fstream>
#include <math.h>
#include <algorithm>

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
std::vector<string> classifications;
#define USE_YOLO_V2 publish	1
#define USE_DETECTNET 	0


void *darknet;
std::vector<std::string> obj_names;


// Moving objects message
carmen_moving_objects_point_clouds_message moving_objects_point_clouds_message;
carmen_point_t globalpos;

/*
 This function find the closest velodyne message with the camera message
 */

typedef struct
{
	unsigned int a;
	unsigned int b;
	unsigned int c;
}lines;

bool
function_to_order_left (carmen_velodyne_points_in_cam_with_obstacle_t i, carmen_velodyne_points_in_cam_with_obstacle_t j)
{
	if (i.velodyne_points_in_cam.ipx > j.velodyne_points_in_cam.ipx)
		return true;
	else
		return (i.velodyne_points_in_cam.ipy > j.velodyne_points_in_cam.ipy);
}

bool
function_to_order_right (carmen_velodyne_points_in_cam_with_obstacle_t i, carmen_velodyne_points_in_cam_with_obstacle_t j)
{
	if (i.velodyne_points_in_cam.ipx < j.velodyne_points_in_cam.ipx)
		return true;
	else
		return (i.velodyne_points_in_cam.ipy > j.velodyne_points_in_cam.ipy);
}

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


lines
get_line(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2)
{
	// Line Equation a = yA - yB, b = xB - xA, c = xAyB - xByA
	unsigned int a = y1 - y2;
	unsigned int b = x2 - x1;
	unsigned int c = (x1 * y2) - (x2 * y1);

	lines line_aux;
	line_aux.a = a;
	line_aux.b = b;
	line_aux.c = c;
	return line_aux;
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

lines
construct_the_line(bbox_t &predictions, bool left_or_right)
{
	lines line;
	if (left_or_right)
	{
		line = get_line(predictions.x, predictions.y + predictions.h, predictions.x + predictions.w, predictions.y) ;
	}else
	{
		line = get_line(predictions.x, predictions.y, predictions.x + predictions.w, predictions.y + predictions.h);
	}

	return line;
}

double
calculate_the_distance_point_to_the_line (lines line_aux, carmen_velodyne_points_in_cam_with_obstacle_t point_aux)
{
	double distance = (abs(line_aux.a * point_aux.velodyne_points_in_cam.ipx + line_aux.b * point_aux.velodyne_points_in_cam.ipy + line_aux.c)
			/ sqrt(line_aux.a * line_aux.a + line_aux.b * line_aux.b));
	return distance;
}

std::vector<carmen_velodyne_points_in_cam_with_obstacle_t>
locate_the_candidate_points_in_the_bounding_box(bbox_t &predictions,
		std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> &laser_points_in_camera_box_list,
		bool left_or_right)
{
	lines line_aux = construct_the_line(predictions, left_or_right);
	std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> candidate_points;
	for (int j = 0; j < laser_points_in_camera_box_list.size(); j++)
	{
		double distance_aux = calculate_the_distance_point_to_the_line(line_aux, laser_points_in_camera_box_list[j]);
		if (distance_aux <= 5)
		{
			candidate_points.push_back(laser_points_in_camera_box_list[j]);
		}
	}
	return candidate_points;
}

void
seting_part_of_the_lane_message(carmen_vector_3D_t p1, int i, int idx_pt1,
		carmen_vector_3D_t p2, int idx_pt2,
	    carmen_lane_detector_lane_message_t& message,
		std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> &candidate_points)
{
	if (candidate_points.empty())
		return;
	p1 =
			carmen_covert_sphere_to_cartesian_coord(
					candidate_points[idx_pt1].velodyne_points_in_cam.laser_polar);
	if (idx_pt2 > 0)
		p2 =
			carmen_covert_sphere_to_cartesian_coord(
					candidate_points[idx_pt2].velodyne_points_in_cam.laser_polar);
	else
		p1 = p2;
	carmen_vector_3D_t offset;
	/*std:: cout <<"i " << i << '\n';
	std:: cout <<"p1 x " << candidate_points[idx_pt1].velodyne_points_in_cam.ipx << " y " << candidate_points[idx_pt1].velodyne_points_in_cam.ipy << '\n';
	std:: cout <<"p2 x " << candidate_points[idx_pt2].velodyne_points_in_cam.ipx << " y " << candidate_points[idx_pt2].velodyne_points_in_cam.ipy << '\n';*/
	p1.y = -p1.y;
	p2.y = -p2.y;
	offset.x = 0.572;
	offset.y = 0.0;
	offset.z = 2.154;
	//printf("%lf %lf\n%lf %lf\n", p1.x, p1.y , p2.x, p2.y);

	if (p1.x == 0.0 || p1.y == 0.0 || p2.x == 0.0 || p2.y == 0.0)
		return;

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
}

std::vector < std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> >
lane_publish_messages(double _timestamp, std::vector<bbox_t> &predictions, std::vector< std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> > &laser_points_in_camera_box_list,
		std::vector<bool> &left_or_right)
{
	// stamps the message
	carmen_lane_detector_lane_message_t message;
	message.host = carmen_get_host();
	message.timestamp = _timestamp;
	message.lane_vector_size = laser_points_in_camera_box_list.size();
	//printf("globalpos %lf %lf\n", globalpos.x, globalpos.y);
	std::vector < std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> > vector_candidate_points;
	if (laser_points_in_camera_box_list.size() == 0)
			return vector_candidate_points;
	message.lane_vector = (carmen_lane_detector_lane_t*) malloc ( message.lane_vector_size * sizeof (carmen_lane_detector_lane_t) );
	for (int i = 0; i < laser_points_in_camera_box_list.size(); i++)
	{
		std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> candidate_points =
				locate_the_candidate_points_in_the_bounding_box(predictions[i], laser_points_in_camera_box_list[i], left_or_right[i]);
		carmen_vector_3D_t p1, p2;
		int idx_pt1, idx_pt2;
		if (left_or_right[i] == true)
		{
			std::sort(candidate_points.begin(), candidate_points.end(), function_to_order_right);
			idx_pt1 = 0;
			idx_pt2 = candidate_points.size() - 1;
			message.lane_vector[i].left = 1;
		}else
		{
			std::sort(candidate_points.begin(), candidate_points.end(), function_to_order_left);
			idx_pt1 = 0;
			idx_pt2 = candidate_points.size() - 1;
			message.lane_vector[i].left = 0;
		}
		seting_part_of_the_lane_message(p1, i, idx_pt1, p2, idx_pt2, message, candidate_points);
		vector_candidate_points.push_back(candidate_points);
	}
	// publish!
	carmen_lane_publish_message(&message);
	free(message.lane_vector);
	return vector_candidate_points;classifications.push_back("sem");
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
		std::vector< std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> >& vector_candidate_points,
		const cv::Mat& rgb_image, std::vector<bounding_box> bouding_boxes_list, int end_time, int start_time) {
	double fps;
	int delta_t = end_time - start_time;
	fps = 1.0 / delta_t;
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
		cv::putText(rgb_image, classifications[i].c_str(), cv::Point(bouding_boxes_list[i].pt1.x, bouding_boxes_list[i].pt1.y),
				cv::FONT_HERSHEY_PLAIN, 2, cvScalar(255, 255, 255), 2);
		for (unsigned int j = 0; j < laser_points_in_camera_box_list[i].size(); j++)
		{
			int size = vector_candidate_points[i].size() -1;
			if (size < 0)
			{
				cv::circle(rgb_image, cv::Point(laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipx,
						laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy), 1, cv::Scalar(255, 0, 0), 3, 0);
			}else
			{
				if (vector_candidate_points[i][0]. velodyne_points_in_cam.ipx == laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipx
						&& vector_candidate_points[i][0]. velodyne_points_in_cam.ipy == laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy)
				{
					cv::circle(rgb_image, cv::Point(laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipx,
					laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy), 1, cv::Scalar(0, 0, 255), 3, 0);
				}
				else
				{
					if (vector_candidate_points[i][size]. velodyne_points_in_cam.ipx == laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipx
							&& vector_candidate_points[i][size]. velodyne_points_in_cam.ipy == laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy)
					{
						cv::circle(rgb_image, cv::Point(laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipx,
						laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy), 1, cv::Scalar(0, 0, 255), 3, 0);
					}else
						cv::circle(rgb_image, cv::Point(laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipx,
								laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy), 1, cv::Scalar(255, 0, 0), 3, 0);
				}
			}
		}
	}
	classifications.clear();
	vector_candidate_points.clear();
	laser_points_in_camera_box_list.clear();
	cv::Mat resized_image(cv::Size(640, 480), CV_8UC3);
	cv::resize(rgb_image, resized_image, resized_image.size());
	cv::imshow("Lane detector", resized_image);
	cv::waitKey(1);
	resized_image.release();
}

void correcting_lane_sides(std::vector<bounding_box>& bouding_boxes_list,
		std::vector<bool>& left_or_right)
{
	std::vector<int> aux;
	for (int i = 0; i < bouding_boxes_list.size(); i++)
	{
		if (!left_or_right[i])
			aux.push_back(i);
	}
	for (int i = 0; i < bouding_boxes_list.size(); i++)
	{
		std::vector<int>::iterator it;
		it = find(aux.begin(), aux.end(), i);
		if (it == aux.end())
		{
			for (int j = 0; j < aux.size(); j++)
				if (std::abs(bouding_boxes_list[aux[j]].pt1.x
					- (bouding_boxes_list[i].pt2.x)) < 5)
				{
					left_or_right[aux[j]] = true;
				}
		}
	}
	aux.clear();
	for (int i = 0; i < bouding_boxes_list.size(); i++)
	{
		if (left_or_right[i])
			aux.push_back(i);
	}
	for (int i = 0; i < bouding_boxes_list.size(); i++)
	{
		std::vector<int>::iterator it;
		it = find(aux.begin(), aux.end(), i);
		if (it == aux.end())
		{
			for (int j = 0; j < aux.size(); j++)
				if (std::abs(bouding_boxes_list[aux[j]].pt2.x
				 - (bouding_boxes_list[i].pt1.x)) < 5)
				{
					left_or_right[aux[j]] = false;
				}
		}
	}
}

std::vector<bbox_t>
detection_of_the_lanes(std::vector<bounding_box> &bouding_boxes_list, cv::Mat& rgb_image, std::vector<bool> &left_or_right) {
	// detect the objects in image
	char ** classes_names = get_classes_names((char*) "../../sharedlib/darknet/data/coco.names");
	std::vector<bbox_t> predictions = run_YOLO(rgb_image.data, rgb_image.cols, rgb_image.rows, darknet ,  classes_names, 0.1);	//find x min and x max for orientation of the lines
	unsigned int x_min = 10000, x_max = 0;
	for (int i = 0; i < predictions.size(); i++)
	{
		/*switch (predictions[i].obj_id)
		{
			case 0:
				classifications.push_back("sem");
				break;
			case 1:
				classifications.push_back("branca cont.");
				break;
			case 2:
				classifications.push_back("branca trace.");
				break;
			case 3:
				classifications.push_back("amarela cont.");
				break;
			case 4:
				classifications.push_back("amarela trace.");
				break;
			case 5:
				classifications.push_back("amarela dupla cont.");
				break;
			case 6:
				classifications.push_back("amarela cont trace");
				break;
			case 7:
				classifications.push_back("amarela 2");
				break;
			default:
				classifications.push_back("erro");
		}*/
		switch (predictions[i].obj_id)
		{
			case 0:
				classifications.push_back("sem esq");
				break;
			case 1:
				classifications.push_back("sem dir");
				break;
			case 2:
				classifications.push_back("branca cont. esq");
				break;
			case 3:
				classifications.push_back("branca cont. dir");
				break;
			case 4:
				classifications.push_back("branca trace. esq");
				break;
			case 5:
				classifications.push_back("branca trace. dir");
				break;
			case 6:
				classifications.push_back("amarela cont. esq");
				break;
			case 7:
				classifications.push_back("amarela cont. dir");
				break;
			case 8:
				classifications.push_back("amarela trace. esq");
				break;
			case 9:
				classifications.push_back("amarela trace. dir");
				break;
			case 10:
				classifications.push_back("amarela dupla cont. esq");
				break;
			case 11:
				classifications.push_back("amarela dupla cont. dir");
				break;
			case 12:
				classifications.push_back("amarela cont trace esq");
				break;
			case 13:
				classifications.push_back("amarela cont trace dir");
				break;
			case 14:
				classifications.push_back("amarela 2 esq");
				break;
			case 15:
				classifications.push_back("amarela 2 dir");
				break;
			default:
				classifications.push_back("erro");
		}

		printf("%d\n",predictions[i].obj_id);
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
		if (box.x  > x_min + (x_max - x_min) / 2)
		{
			bbox.pt1.y = box.y;
			bbox.pt2.y = box.y + box.h;
			left_or_right.push_back(false);
		} else
		{
			bbox.pt2.y = box.y;
			bbox.pt1.y = box.y + box.h;
			left_or_right.push_back(true);
		}
		bouding_boxes_list.push_back(bbox);
	}

	correcting_lane_sides(bouding_boxes_list, left_or_right);
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

	double start_time, end_time;

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
	std::vector<bbox_t> predictions = detection_of_the_lanes(bouding_boxes_list, src_image, left_or_right);
    std::vector< std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> > laser_points_in_camera_box_list = velodyne_points_in_lane_bboxes(bouding_boxes_list,
    		&velodyne_sync_with_cam, camera_parameters, velodyne_pose, camera_pose, image_msg->width, image_msg->height, &left_or_right);
    end_time = carmen_get_time();
	std::vector < std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> > vector_candidate_points =
			lane_publish_messages(image_msg-> timestamp, predictions, laser_points_in_camera_box_list, left_or_right);
	put_the_lane_dectections_in_image(laser_points_in_camera_box_list, vector_candidate_points, rgb_image, bouding_boxes_list, end_time, start_time);
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
    carmen_param_install_params(argc, argv, param_list, num_items);using namespace cv;


    return 0;
}


int
main(int argc, char **argv)
{
    if ((argc != 3))
        carmen_die("%s: Wrong number of parameters. Neural_car_detector requires 2 parameter and received %d. \n Usage: %s <camera_number> <camera_side(0-left; 1-right)\n>",
                   argv[0], argc - 1, argv[0]);

    /**** DETECTNET PARAMETERS ****/
    std::string model_file = "deploy.prototxt";
    std::string trained_file = "snapshot_iter_21600.caffemodel";
    /**** YOLO PARAMETERS ****/
    std::string darknet_home = std::getenv("DARKNET_HOME"); /*< get environment variable pointing path of darknet*/
    if (darknet_home.empty())
        printf("Cannot find darknet path. Check if you have correctly set DARKNET_HOME environment variable.\n");
    std::string cfg_filename =  "/home/vinicius/yolov3-voc_lane.cfg";
    std::string weight_filename = darknet_home + "2/backup/yolov3-voc_lane_15000.weights";
    std::string voc_names = darknet_home + "/data/lane.names";
    obj_names = objects_names_from_file(voc_names);
    darknet = initialize_YOLO((char*) cfg_filename.c_str(),(char*) weight_filename.c_str());
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
