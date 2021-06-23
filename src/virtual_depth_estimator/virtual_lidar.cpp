#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <carmen/carmen.h>
#include <prob_measurement_model.h>
#include <prob_map.h>
#include <prob_interface.h>
#include <prob_measurement_model.h>
#include <prob_transforms.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/velodyne_camera_calibration.h>
#include <carmen/laser_ldmrs_interface.h>
#include <carmen/laser_ldmrs_utils.h>
#include <carmen/rotation_geometry.h>
#include <carmen/mapper_interface.h>
#include <carmen/stereo_velodyne.h>
#include <carmen/stereo_velodyne_interface.h>
#include <carmen/grid_mapping.h>
#include <carmen/stereo_mapping_interface.h>
#include <carmen/map_server_interface.h>
#include <carmen/rddf_messages.h>
#include <carmen/rddf_interface.h>
#include <carmen/rddf_util.h>
#include <carmen/ultrasonic_filter_interface.h>
#include <carmen/parking_assistant_interface.h>
//#include <carmen/librange.h>
#include <carmen/carmen_darknet4_interface.hpp>
#include <omp.h>
#include <carmen/mapper.h>
#include <sys/stat.h>

// #include "message_interpolation.cpp"
#include "dbscan.h"
#include "movable_object.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream> //Para salvar arquivos
#include <fstream> //Para salvar arquivos
#include <list> 
#include <iterator> 

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION


// //YOLO global variables
char **classes_names;
void *network_struct;
//-------
char* camera_model;    // Camera model in case of using camera_message
int message_number = -1;   // Number of the camera message
double resize_factor = 1.0; // Factor of resize
int crop_x = 0;       // Crop starting point
int crop_y = 0;
int crop_w  = -1;     // Width of crop
int crop_h = -1;     // Heigth of crop
int original_img_width = -1;
int original_img_height = -1;

carmen_localize_ackerman_globalpos_message *globalpos_msg = NULL;
carmen_velodyne_partial_scan_message *velodyne_msg = NULL;
carmen_laser_ldmrs_new_message* sick_laser_message = NULL;
carmen_rddf_annotation_message* rddf_annotation_message = NULL;


carmen_pose_3D_t velodyne_pose;
carmen_pose_3D_t board_pose;
carmen_pose_3D_t bullbar_pose;
carmen_pose_3D_t sick_pose;
tf::Transformer transformer_sick;

vector<carmen_velodyne_partial_scan_message> velodyne_vector; //used to correct camera delay
vector<carmen_laser_ldmrs_new_message> sick_vector; //used to correct camera delay

#define CAM_DELAY 0.25
#define MAX_POSITIONS 10

#define IMAGE_HEIGHT_CROP 0.82

#define MAX_CAMERA_INDEX	9
#define NUM_CAMERA_IMAGES	5
int camera_alive[MAX_CAMERA_INDEX + 1];
carmen_camera_parameters camera_params[MAX_CAMERA_INDEX + 1];
carmen_pose_3D_t camera_pose[MAX_CAMERA_INDEX + 1];
int active_cameras;

using namespace std;
using namespace cv;

vector<movable_object> movable_object_tracks;

const static double sorted_vertical_angles[32] =
{
	-30.67, -29.33, -28.0, -26.67, -25.33, -24.0, -22.67, -21.33, -20.0,
	-18.67, -17.33, -16.0, -14.67, -13.33, -12.0, -10.67, -9.3299999, -8.0,
	-6.6700001, -5.3299999, -4.0, -2.6700001, -1.33, 0.0, 1.33, 2.6700001, 4.0,
	5.3299999, 6.6700001, 8.0, 9.3299999, 10.67
};

carmen_velodyne_partial_scan_message
find_velodyne_most_sync_with_cam(double bumblebee_timestamp)  // TODO is this necessary?
{
	bumblebee_timestamp -= CAM_DELAY;
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

carmen_laser_ldmrs_new_message
find_sick_most_sync_with_cam(double bumblebee_timestamp)  // TODO is this necessary?
{
	bumblebee_timestamp -= CAM_DELAY;
	carmen_laser_ldmrs_new_message sick;
    double minTimestampDiff = DBL_MAX;
    int minTimestampIndex = -1;

    for (unsigned int i = 0; i < sick_vector.size(); i++)
    {
        if (fabs(sick_vector[i].timestamp - bumblebee_timestamp) < minTimestampDiff)
        {
            minTimestampIndex = i;
            minTimestampDiff = fabs(sick_vector[i].timestamp - bumblebee_timestamp);
        }
    }

    sick = sick_vector[minTimestampIndex];
    return (sick);
}

void
clean_movable_objects(double max_time)
{
	double timestamp = carmen_get_time();

	for (vector<movable_object>::iterator it=movable_object_tracks.begin(); it != movable_object_tracks.end();)
	{
		if(timestamp - it->last_timestamp > max_time)
		{
			it = movable_object_tracks.erase(it);
		}
		else{
			it++;
		}
	}
}

void
carmen_translte_2d(double *x, double *y, double offset_x, double offset_y)
{
	*x += offset_x;
	*y += offset_y;
}

/** The goal is change this function from get point to generate points inside bounding boxes*/
vector<vector<image_cartesian>>
get_points_inside_bounding_boxes(vector<movable_object> &predictions, vector<image_cartesian> &velodyne_points_vector)
{
	vector<vector<image_cartesian>> laser_list_inside_each_bounding_box; //each_bounding_box_laser_list

	for (unsigned int i = 0; i < predictions.size(); i++)
	{
		vector<image_cartesian> lasers_points_inside_bounding_box;
		laser_list_inside_each_bounding_box.push_back(lasers_points_inside_bounding_box);
	}

	for (unsigned int j = 0; j < velodyne_points_vector.size(); j++)
	{
		double min_dist = 0.0;
		int min_dist_index = -1;
		for (unsigned int i = 0; i < predictions.size(); i++)
		{
			if (predictions[i].active && (	(unsigned int) velodyne_points_vector[j].image_x >  predictions[i].x &&
											(unsigned int) velodyne_points_vector[j].image_x < (predictions[i].x + predictions[i].w) &&
											(unsigned int) velodyne_points_vector[j].image_y >  predictions[i].y &&
											(unsigned int) velodyne_points_vector[j].image_y < (predictions[i].y + predictions[i].h)))
			{
				double delta_x =  get_movable_object_x(predictions[i]);
				double delta_y =  get_movable_object_y(predictions[i]);
				double actual_dist = sqrt(delta_x*delta_x+delta_y*delta_y);
				if ( actual_dist < min_dist || min_dist_index==-1)
				{
					min_dist = actual_dist;
					min_dist_index = i;
				}
				//laser_list_inside_each_bounding_box[i].push_back(velodyne_points_vector[j]);
			}
		}
		if (min_dist_index > -1)
		{
			laser_list_inside_each_bounding_box[min_dist_index].push_back(velodyne_points_vector[j]);
		}
	}
	return laser_list_inside_each_bounding_box;
}

vector<image_cartesian>
compute_detected_objects_poses(vector<vector<image_cartesian>> filtered_points)
{
	vector<image_cartesian> objects_positions;
	unsigned int i, j;

	for(i = 0; i < filtered_points.size(); i++)
	{
		image_cartesian point;

		if (filtered_points[i].size() == 0)
		{
			point.cartesian_x = -999.0;    // This error code is set, probably the object is out of the LiDAR's range
			point.cartesian_y = -999.0;
			point.cartesian_z = -999.0;
			//printf("Empty Bbox\n");
		}
		else
		{
			point.cartesian_x = 0.0;
			point.cartesian_y = 0.0;
			point.cartesian_z = 0.0;

			for(j = 0; j < filtered_points[i].size(); j++)
			{
				point.cartesian_x += filtered_points[i][j].cartesian_x;
				point.cartesian_y += filtered_points[i][j].cartesian_y;
				point.cartesian_z += filtered_points[i][j].cartesian_z;
			}
			point.cartesian_x = point.cartesian_x / j;
			point.cartesian_y = point.cartesian_y / j;
			point.cartesian_z = point.cartesian_z / j;
		}
		objects_positions.push_back(point);
	}
	return (objects_positions);
}


int
compute_num_measured_objects(vector<movable_object> objects_poses)
{
	int num_objects = 0;

	for (unsigned int i = 0; i < objects_poses.size(); i++)
	{
		if ((get_movable_object_x(objects_poses[i]) > 0.0 || get_movable_object_y(objects_poses[i]) > 0.0) && objects_poses[i].active)
			num_objects++;
	}
	return (num_objects);
}


void
show_LIDAR_points(Mat &image, vector<image_cartesian> all_points)
{
	for (unsigned int i = 0; i < all_points.size(); i++)
		circle(image, Point(all_points[i].image_x, all_points[i].image_y), 1, cvScalar(0, 0, 255), 1, 8, 0);
}


void
show_LIDAR(Mat &image, vector<vector<image_cartesian>> points_lists, int r, int g, int b)
{
	for (unsigned int i = 0; i < points_lists.size(); i++)
	{
		for (unsigned int j = 0; j < points_lists[i].size(); j++)
			circle(image, Point(points_lists[i][j].image_x, points_lists[i][j].image_y), 1, cvScalar(b, g, r), 1, 8, 0);
	}
}

void
show_LIDAR_deepth(Mat &image, vector<vector<image_cartesian>> points_lists, int r, int g, int b, int rt, int gt, int bt, double max_range)
{
	for (unsigned int i = 0; i < points_lists.size(); i++)
	{
		for (unsigned int j = 0; j < points_lists[i].size(); j++){
			double distance = sqrt(points_lists[i][j].cartesian_x*points_lists[i][j].cartesian_x+points_lists[i][j].cartesian_y*points_lists[i][j].cartesian_y);
			int rp = rt + double((max_range-distance)/max_range)*double(r-rt);
			int gp = gt + double((max_range-distance)/max_range)*double(g-gt);
			int bp = bt + double((max_range-distance)/max_range)*double(b-bt);
			circle(image, Point(points_lists[i][j].image_x, points_lists[i][j].image_y), 1, cvScalar(bp, gp, rp), 1, 8, 0);
		}
	}
}



void
show_all_points(Mat &image, unsigned int image_width, unsigned int image_height, unsigned int crop_x, unsigned int crop_y, unsigned int crop_width, unsigned int crop_height, int camera_index)
{
	vector<carmen_velodyne_points_in_cam_t> all_points = carmen_velodyne_camera_calibration_lasers_points_in_camera(
					velodyne_msg, camera_params[camera_index], velodyne_pose, camera_pose[camera_index], image_width, image_height);

	int max_x = crop_x + crop_width, max_y = crop_y + crop_height;

	for (unsigned int i = 0; i < all_points.size(); i++)
		if ((unsigned int) all_points[i].ipx >= crop_x && 
			all_points[i].ipx <= max_x && 
			(unsigned int) all_points[i].ipy >= crop_y && 
			all_points[i].ipy <= max_y)
			circle(image, Point(all_points[i].ipx - crop_x, all_points[i].ipy - crop_y), 1, cvScalar(0, 0, 255), 1, 8, 0);
}


void
display_lidar(Mat &image, vector<image_cartesian> points, int r, int g, int b)
{
	for (unsigned int i = 0; i < points.size(); i++)
		circle(image, Point(points[i].image_x, points[i].image_y), 1, cvScalar(b, g, r), 1, 8, 0);
}


double
distance_error (double measured, double gt)
{
	double dif = abs(gt - measured);
	if (gt != 0.0){
		double acc = 100.0 * (dif/gt);
		return acc;
	}else{
		return 0.0;
	}
}

double
estimate_distance(double image_height, double object_height, int camera_index)
{
	double focal_length = camera_params[camera_index].focal_length; //f(mm) 6.0
	double sensor_height = 4.927868852; // sensorheight(mm)
	double real_height = 1498.0; //car_height(mm) GIVES AN ERROR
	double distance_to_object = focal_length * real_height * image_height / (object_height * sensor_height); // mm
	distance_to_object = distance_to_object / 1000.0; // mm to meters.
	return distance_to_object;
}

void
show_detections(Mat image, vector<movable_object> movable_object,vector<bbox_t> predictions, vector<image_cartesian> points, vector<vector<image_cartesian>> points_inside_bbox,
		vector<vector<image_cartesian>> filtered_points, double fps, unsigned int image_width, unsigned int image_height, unsigned int crop_x, unsigned int crop_y, 
		unsigned int crop_width, unsigned int crop_height, double dist_to_movable_object_track, int camera_index)
{
	char info[128];
	double distance_to_object;
	double sensor_distance;
	double lidar_distance = 0.0;
	double sick_distance = 0.0;


    cvtColor(image, image, COLOR_RGB2BGR);

	sprintf(info, "%dx%d", image.cols, image.rows);
    putText(image, info, Point(10, 15), FONT_HERSHEY_PLAIN, 1, cvScalar(0, 255, 0), 1);

    sprintf(info, "FPS %.2f", fps);
    putText(image, info, Point(10, 30), FONT_HERSHEY_PLAIN, 1, cvScalar(0, 255, 0), 1);

	if (dist_to_movable_object_track < 800)
	{
		sprintf(info, "DIST %.2f", dist_to_movable_object_track);
		putText(image, info, Point(10, 45), FONT_HERSHEY_PLAIN, 1, cvScalar(0, 255, 0), 1);
	}
	
    /*for (unsigned int i = 0; i < predictions.size(); i++)
	{
		distance_to_object = estimate_distance (image.rows, predictions[i].h, camera_index);
		// sprintf(info, "%s %.2f track_id %d", classes_names[predictions[i].obj_id], distance_to_object, predictions[i].track_id);
		// putText(image, info, Point(predictions[i].x, predictions[i].y - 4), FONT_HERSHEY_PLAIN, 1, cvScalar(0, 255, 0), 1);
		//rectangle(image, Point(predictions[i].x, predictions[i].y), Point((predictions[i].x + predictions[i].w), (predictions[i].y + predictions[i].h)),
		//		Scalar(255, 0, 255), 4);
	}*/

    for (unsigned int i = 0; i < movable_object.size(); i++)
    {
		if (movable_object[i].active)
    	{
			distance_to_object = estimate_distance (image.rows, movable_object[i].h, camera_index);
			sprintf(info, "%.2f %d %d %.2f", movable_object[i].velocity, movable_object[i].track_id, movable_object[i].obj_id, distance_to_object);

			rectangle(image, Point(movable_object[i].x, movable_object[i].y), Point((movable_object[i].x + movable_object[i].w), (movable_object[i].y + movable_object[i].h)),
							Scalar(255, 255, 0), 4);

			putText(image, info, Point(movable_object[i].x + 1, movable_object[i].y - 3), FONT_HERSHEY_PLAIN, 1, cvScalar(255, 255, 0), 1);
    	}
	}

	// show_all_points(image, image_width, image_height, crop_x, crop_y, crop_width, crop_height, camera_index);
	// vector<vector<image_cartesian>> lidar_points;
	// lidar_points.push_back(points);
	// show_LIDAR(image, lidar_points, 255, 0, 0);
	show_LIDAR(image, points_inside_bbox,    0, 0, 255);				// Blue points are all points inside the bbox
    show_LIDAR(image, filtered_points, 0, 255, 0); 						// Green points are filtered points

	// display_lidar(image, points, 0, 255, 0);

	resize(image, image, Size(image.cols * resize_factor, image.rows * resize_factor));
    imshow("Virtual LiDAR Object Detector", image);
    //imwrite("Image.jpg", image);
    waitKey(1);
}

char *
model_name_from_yolo(int obj_id)
{
	switch(obj_id)
	{
		case 0:
			return (char *) "pedestrian";
		case 1:
			return (char *) "bicycle";
		case 2:
			return (char *) "car";
		case 3:
			return (char *) "motorbike";
		case 5:
			return (char *) "bus";
		case 6:
			return (char *) "train";
		case 7:
			return (char *) "truck";
		default:
			return (char *) "movable_object";
	}
}

carmen_moving_objects_point_clouds_message
build_detected_objects_message(vector<movable_object> predictions, vector<vector<image_cartesian>> points_lists)
{
	carmen_moving_objects_point_clouds_message msg;

	vector<movable_object> tmp_predictions = predictions;

	int num_objects = compute_num_measured_objects(tmp_predictions);

	//printf ("Predictions %d Poses %d, Points %d\n", (int) predictions.size(), (int) objects_poses.size(), (int) points_lists.size());

	msg.num_point_clouds = num_objects;
	msg.point_clouds = (t_point_cloud_struct *) malloc (num_objects * sizeof(t_point_cloud_struct));

	for (unsigned int i = 0, l = 0; i < tmp_predictions.size(); i++)
	{                                                                                                               // The error code of -999.0 is set on compute_detected_objects_poses,
		if ((get_movable_object_x(tmp_predictions[i]) != -999.0 || get_movable_object_y(tmp_predictions[i]) != -999.0) && tmp_predictions[i].active)                       // probably the object is out of the LiDAR's range
		{
//			carmen_translte_2d(&tmp_predictions[i].x_world[tmp_predictions[i].circular_idx], &tmp_predictions[i].y_world[tmp_predictions[i].circular_idx], board_pose.position.x, board_pose.position.y);
//			carmen_rotate_2d  (&tmp_predictions[i].x_world[tmp_predictions[i].circular_idx], &tmp_predictions[i].y_world[tmp_predictions[i].circular_idx], carmen_normalize_theta(globalpos.theta));
//			carmen_translte_2d(&tmp_predictions[i].x_world[tmp_predictions[i].circular_idx], &tmp_predictions[i].y_world[tmp_predictions[i].circular_idx], globalpos.x, globalpos.y);

			msg.point_clouds[l].r = 1.0;
			msg.point_clouds[l].g = 1.0;
			msg.point_clouds[l].b = 0.0;

			// printf("V %lf\n", tmp_predictions[i].velocity);

			msg.point_clouds[l].linear_velocity = tmp_predictions[i].velocity;
			msg.point_clouds[l].orientation = tmp_predictions[i].orientation;

			msg.point_clouds[l].width  = 1.0;
			msg.point_clouds[l].length = 1.0;
			msg.point_clouds[l].height = 2.0;

			msg.point_clouds[l].object_pose.x = get_movable_object_x(tmp_predictions[i]);
			msg.point_clouds[l].object_pose.y = get_movable_object_y(tmp_predictions[i]);
			msg.point_clouds[l].object_pose.z = 0.0;


			msg.point_clouds[l].geometric_model = 0;
			msg.point_clouds[l].model_features.geometry.width  = 1.0;
			msg.point_clouds[l].model_features.geometry.length = 1.0;
			msg.point_clouds[l].model_features.geometry.height = 2.0;
			msg.point_clouds[l].model_features.red = 1.0;
			msg.point_clouds[l].model_features.green = 1.0;
			msg.point_clouds[l].model_features.blue = 0.8;
			
			msg.point_clouds[l].model_features.model_name = model_name_from_yolo(tmp_predictions[i].obj_id);

			msg.point_clouds[l].num_associated = tmp_predictions[i].track_id;

			msg.point_clouds[l].point_size = points_lists[i].size();

			msg.point_clouds[l].points = (carmen_vector_3D_t *) malloc (msg.point_clouds[l].point_size * sizeof(carmen_vector_3D_t));

			for (unsigned int j = 0; j < points_lists[i].size(); j++)
			{
				carmen_vector_3D_t p;

				p.x = points_lists[i][j].cartesian_x;
				p.y = points_lists[i][j].cartesian_y;
				p.z = points_lists[i][j].cartesian_z;

				carmen_translte_2d(&p.x, &p.y, board_pose.position.x, board_pose.position.y);
				carmen_rotate_2d  (&p.x, &p.y, globalpos_msg->globalpos.theta);
				carmen_translte_2d(&p.x, &p.y, globalpos_msg->globalpos.x, globalpos_msg->globalpos.y);

				msg.point_clouds[l].points[j] = p;
			}
			l++;
		}
	}
	return (msg);
}


vector<bbox_t>
filter_predictions_of_interest(vector<bbox_t> &predictions)
{
	vector<bbox_t> filtered_predictions;

	for (unsigned int i = 0; i < predictions.size(); i++)
	{
		if (predictions[i].obj_id == 0 ||  // person
			predictions[i].obj_id == 1 ||  // bicycle
			predictions[i].obj_id == 2 ||  // car
			predictions[i].obj_id == 3 ||  // motorbike
			predictions[i].obj_id == 5 ||  // bus
			predictions[i].obj_id == 6 ||  // train
			predictions[i].obj_id == 7) //||  // truck
			//predictions[i].obj_id == 9)    // traffic light
		{
			filtered_predictions.push_back(predictions[i]);
		}
	}
	return (filtered_predictions);
}



void
carmen_translte_3d(double *x, double *y, double *z, double offset_x, double offset_y, double offset_z)
{
	*x += offset_x;
	*y += offset_y;
	*z += offset_z;
}


float
intersectionOverUnion(bbox_t box1, bbox_t box2)
{
	// https://github.com/lukaswals/cpp-iout/blob/master/cppIOUT/IOUT.cpp
	float minx1 = box1.x;
	float maxx1 = box1.x + box1.w;
	float miny1 = box1.y;
	float maxy1 = box1.y+ box1.h;

	float minx2 = box2.x;
	float maxx2 = box2.x + box2.w;
	float miny2 = box2.y;
	float maxy2 = box2.y + box2.h;

	if (minx1 > maxx2 || maxx1 < minx2 || miny1 > maxy2 || maxy1 < miny2)
		return (0.0);
	else
	{
		float dx = std::min(maxx2, maxx1) - std::max(minx2, minx1);
		float dy = std::min(maxy2, maxy1) - std::max(miny2, miny1);
		float area1 = (maxx1 - minx1) * (maxy1 - miny1);
		float area2 = (maxx2 - minx2) * (maxy2 - miny2);
		float inter = dx * dy; // Intersection
		float uni = area1 + area2 - inter; // Union
		
		float IoU = inter / uni;

		return (IoU);
	}
}

void
publish_moving_objects_message(carmen_moving_objects_point_clouds_message *msg);

void
insert_missing_movable_objects_in_the_track(vector<bbox_t> predictions)
{
	unsigned int i = 0, predictions_size = predictions.size(), j = 0, movable_object_tracks_size = movable_object_tracks.size();

	for (i = 0; i < predictions_size; i++)
	{
		for (j = 0; j < movable_object_tracks_size; j++)
		{
			if (movable_object_tracks[i].active == false)
				continue;

			bbox_t p;
			p.x = movable_object_tracks[j].x;
			p.y = movable_object_tracks[j].y;
			p.w = movable_object_tracks[j].w;
			p.h = movable_object_tracks[j].h;
			p.obj_id = movable_object_tracks[j].obj_id;
			p.track_id = movable_object_tracks[j].track_id;

			if (intersectionOverUnion(predictions[i], p) > 0.1)
				break;
		}
		if (j == movable_object_tracks_size)
		{
			movable_object new_p = create_movable_object(predictions[i].track_id);
			new_p.x = predictions[i].x;
			new_p.y = predictions[i].y;
			new_p.w = predictions[i].w;
			new_p.h = predictions[i].h;
			new_p.obj_id = predictions[i].obj_id;
			new_p.last_timestamp = carmen_get_time();
			movable_object_tracks.push_back(new_p);
		}
	}
}


// double
// distance_to_movable_object_track_annotaion()
// {
// 	if (rddf_annotation_message == NULL)
// 		return (DBL_MAX);
	
// 	double min_dist_to_movable_object_track = DBL_MAX, dist_to_movable_object_track = DBL_MAX;

// 	for (int i = 0, size = rddf_annotation_message->num_annotations; i < size; i++)
// 	{
// 		if (rddf_annotation_message->annotations[i].annotation_type == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK)
// 			dist_to_movable_object_track = DIST2D(globalpos_msg->globalpos, rddf_annotation_message->annotations[i].annotation_point);
		
// 		if (dist_to_movable_object_track < min_dist_to_movable_object_track)
// 			min_dist_to_movable_object_track = dist_to_movable_object_track;
// 	}
// 	return (min_dist_to_movable_object_track);
// }

void
display_lidar_matrix(Mat &image, vector<vector<image_cartesian>> points, int r, int g, int b)
{
	for (unsigned int i = 0; i < points.size(); i++)
	{
		display_lidar(image, points[i], r, g, b);
	}
}


// void
// show_detections(Mat image, vector<bbox_t> predictions, 
// 				vector<image_cartesian> points, 
// 				vector<image_cartesian> points_sick, 
// 				vector<vector<image_cartesian>> clustered_points, 
// 				vector<vector<image_cartesian>> clustered_points_sick, 
// 				int camera_index)
// {
// 	char info[25];
// 	char sick_string[25];
// 	char distance_string[25];
// 	char lidar_string[25];

// 	static double start_time = 0.0;
// 	unsigned int name_size = 0;
// 	double distance_to_object;
// 	double sensor_distance;
// 	double lidar_distance = 0.0;
// 	double sick_distance = 0.0;

	
// 	if (image.empty())
// 		return;

//     cvtColor(image, image, COLOR_RGB2BGR);

//     sprintf(info, "%dx%d", image.cols, image.rows);
//     putText(image, info, Point(10, 25), FONT_HERSHEY_PLAIN, 2, cvScalar(50, 255, 50), 2);

//     sprintf(info, "FPS = %.2f",  (1.0 / (carmen_get_time() - start_time)));
//     putText(image, info, Point(10, 55), FONT_HERSHEY_PLAIN, 2, cvScalar(50, 255, 50), 2);
// 	start_time = carmen_get_time();
	
//     for (unsigned int i = 0; i < predictions.size(); i++)
// 	{
// 		name_size = strlen(classes_names[predictions[i].obj_id]) * 10;
// 		rectangle(image, Point(predictions[i].x, predictions[i].y - 4), Point(predictions[i].x + name_size, predictions[i].y - 15), Scalar(0, 0, 0), -1, 8, 0);

// 		rectangle(image, Point(predictions[i].x, predictions[i].y), Point((predictions[i].x + predictions[i].w), (predictions[i].y + predictions[i].h)),
// 				Scalar(255, 0, 255), 4);
// 		distance_to_object = estimate_distance (image.rows, camera_index, predictions[i].h);
// 		sprintf(info, "%s %.2f", classes_names[predictions[i].obj_id], distance_to_object);
// 		putText(image, info, Point(predictions[i].x, predictions[i].y - 4), FONT_HERSHEY_PLAIN, 1, cvScalar(0, 255, 0), 1);
// 		// Distância Sensor
// 		lidar_distance = 0.0;
// 		sensor_distance = 0.0;
// 		bool first = true;
// 		for (unsigned int k = 0; k < clustered_points.size(); k++)
// 		{
// 			unsigned int cluster_size = clustered_points[k].size();
// 			for (unsigned int j = 0; j < cluster_size; j++)
// 			{
// 				if ((unsigned int) clustered_points[k][j].image_x >=  predictions[i].x &&
// 					(unsigned int) clustered_points[k][j].image_x <= (predictions[i].x + predictions[i].w) &&
// 					(unsigned int) clustered_points[k][j].image_y >=  predictions[i].y &&
// 					(unsigned int) clustered_points[k][j].image_y <= (predictions[i].y + predictions[i].h))
// 				{
// 					sensor_distance = clustered_points[k][j].length;
// 					if (sensor_distance > 0.0 && first)
// 					{
// 						lidar_distance = sensor_distance;
// 						first = false;
// 					}
// 					if (sensor_distance < lidar_distance && sensor_distance > 0.0)
// 						lidar_distance = sensor_distance;
// 				}
// 			}
// 		}
// 		sick_distance = 0.0;
// 		sensor_distance = 0.0;
// 		first = true;
// 		for (unsigned int k = 0; k < clustered_points_sick.size(); k++)
// 		{
// 			unsigned int cluster_size = clustered_points_sick[k].size();
// 			for (unsigned int j = 0; j < cluster_size; j++)
// 			{
// 				if ((unsigned int) clustered_points_sick[k][j].image_x >=  predictions[i].x &&
// 					(unsigned int) clustered_points_sick[k][j].image_x <= (predictions[i].x + predictions[i].w) &&
// 					(unsigned int) clustered_points_sick[k][j].image_y >=  predictions[i].y &&
// 					(unsigned int) clustered_points_sick[k][j].image_y <= (predictions[i].y + predictions[i].h))
// 				{
// 					sensor_distance = clustered_points_sick[k][j].length;
// 					if (sensor_distance > 0.0 && first)
// 					{
// 						sick_distance = sensor_distance;
// 						first = false;
// 					}
// 					if (sensor_distance < sick_distance && sensor_distance > 0.0)
// 						sick_distance = sensor_distance;
// 				}
// 			}
// 		}
// 		double accuracy_lidar;
// 		double accuracy_sick;
// 		cout << "\t" << distance_to_object << "\t";
// 		sprintf(distance_string, "%.2f", distance_to_object);
// 		if (lidar_distance != 0.0){
// 			accuracy_lidar = distance_error(distance_to_object, lidar_distance);
// 			sprintf(lidar_string, "%.2f", lidar_distance);
// 			cout << lidar_distance << "\t" << accuracy_lidar << "\t";
// 		}else{
// 			sprintf(lidar_string, "NL");
// 			cout << "NL" << "\tNLA\t" ;
// 		}
// 		if (sick_distance != 0.0){
// 			accuracy_sick = distance_error(distance_to_object, sick_distance);
// 			sprintf(sick_string, "%.2f", sick_distance);
// 			cout << sick_distance << "\t" << accuracy_sick << "\t";
// 		}else{
// 			sprintf(sick_string, "NS");
// 			cout << "NS" << "\tNSA\t";
// 		}
// 		cout << endl;
			
// 	}

// 	//printf("Points Size %d\n", points.size());
// 	display_lidar(image, points, 0, 0, 200);                       // Blue points are all points that hit an obstacle
// 	display_lidar(image, points_sick, 204, 204, 200);                       // Blue points are all points that hit an obstacle
	
// 	display_lidar_matrix(image, clustered_points, 0, 255, 0);      // Green points are the clustered and classified as moving objects points
	
// 	display_lidar_matrix(image, clustered_points_sick, 255, 255, 0);      // Yellow points are the clustered and classified as moving objects points

//     if (image.cols > 640)
// 	{
// 		resize(image, image, Size(640, image.rows * (640.0 / image.cols)));
// 	}
	
// 	sprintf(info, "Camera %d Detections", camera_index);
//     //imwrite("Image.jpg", image);
//     imshow(info, image);
// 	waitKey(1);
// }



// ===================================================================================================================================================================================================================================
// LEMBRAR DE EXPLICAR NO ARTIGO QUE PODEMOS SEMPRE CONSIDERAR A POSIÇÃO DO OBJETO EM RELAÇÃO AO ROBO E NAO A POSICAO DO OBJETO NO MUNDO, POR QUE EM RELACAO AO ROBO OS OBJETOS SE MOVEM MAIS LENTAMENTE
// Por isso a funcao compute_mean_point_of_each_object nao precisa transladar os objetos para suas respectivas poses no mundo
// ===================================================================================================================================================================================================================================

vector<carmen_vector_2D_t>
compute_mean_point_of_each_cluster(vector<vector<image_cartesian>> clusters_vector)
{
	vector<carmen_vector_2D_t> clusters_poses_vector;
	carmen_vector_2D_t point;
	double x_mean = 0.0;
	double y_mean = 0.0;
	unsigned int j = 0, c_size = 0;


	for (unsigned int i = 0, size = clusters_vector.size(); i < size; i++)
	{
		for (j = 0, c_size = clusters_vector[i].size(); j < c_size; j++)
		{
			// if (size < 5)    // Empty cluster
			// 	break;
			
			x_mean += clusters_vector[i][j].cartesian_x;
			y_mean += clusters_vector[i][j].cartesian_y;
		}
		if (j > 0)
		{
			point.x = x_mean / double (j - 1);
			point.y = y_mean / double (j - 1);

			clusters_poses_vector.push_back(point);
		}
		x_mean = 0.0;
		y_mean = 0.0;
	}
	return (clusters_poses_vector);
}

// ===================================================================================================================================================================================================================================
// Parametro a ser movido para o local adequado de definição de parametros
// ===================================================================================================================================================================================================================================
#define MAX_CLUSTER_DIST 1.0      // Distance in meters between actual mean pose of a cluster of points and and previous mean pose of a cluster of points, to be considered the same cluster. Used to track clusters and recover a cluster from missing detection

vector<vector<image_cartesian>>
find_missing_movable_objects(vector<carmen_vector_2D_t> previous_filtered_clusters_mean_pose_vector, vector<carmen_vector_2D_t> filtered_clusters_mean_pose_vector,
	vector<carmen_vector_2D_t> clustered_points_mean_pose_vector, vector<vector<image_cartesian>> clustered_points, vector<carmen_vector_2D_t> &recovered_clusters_mean_pose_vector)
{
	vector<vector<image_cartesian>> recovered_clusters;
	bool cluster_found = false;
	unsigned int i, j, k, max_i, max_j, max_k;

	for (i = 0, max_i = previous_filtered_clusters_mean_pose_vector.size(); i < max_i; i++)
	{
		for (j = 0, max_j = filtered_clusters_mean_pose_vector.size(); j < max_j; j++)
		{
			if (DIST2D(previous_filtered_clusters_mean_pose_vector[i], filtered_clusters_mean_pose_vector[j]) < MAX_CLUSTER_DIST)
			{
				cluster_found = true;
				break;
			}
		}
		if (!cluster_found)
		{
			for (k = 0, max_k = clustered_points_mean_pose_vector.size(); k < max_k; k++)
			{
				if (DIST2D(previous_filtered_clusters_mean_pose_vector[i], clustered_points_mean_pose_vector[k]) < MAX_CLUSTER_DIST)
				{
					recovered_clusters.push_back(clustered_points[k]);

					recovered_clusters_mean_pose_vector.push_back(clustered_points_mean_pose_vector[k]);
					break;
				}
			}
		}
		cluster_found = false;
	}
	return (recovered_clusters);
}

vector<image_cartesian>
velodyne_camera_calibration_fuse_virtual_lidar(carmen_velodyne_partial_scan_message *velodyne_message, carmen_camera_parameters camera_parameters,
                                              carmen_pose_3D_t velodyne_pose, carmen_pose_3D_t camera_pose, unsigned int image_width,
											  unsigned int image_height, unsigned int crop_x, unsigned int crop_y, unsigned int crop_width, unsigned int crop_height,
											  vector<movable_object> &predictions, int camera_index)
{
	std::vector<image_cartesian> points;
	double horizontal_angle = 0.0, vertical_angle = 0.0, previous_vertical_angle = 0.0, range = 0.0, previous_range = 0.0;
	unsigned int image_x = 0, image_y = 0;

	unsigned int max_x = crop_x + crop_width;
	unsigned int max_y = crop_y + crop_height;

	if (velodyne_message == NULL)
		return (points);

	double fx_meters = camera_parameters.fx_factor * camera_parameters.pixel_size * image_width;
	double fy_meters = camera_parameters.fy_factor * camera_parameters.pixel_size * image_height;

	double cu = camera_parameters.cu_factor * (double) image_width;
	double cv = camera_parameters.cv_factor * (double) image_height;

	for (int j = 0; j < velodyne_message->number_of_32_laser_shots; j++)
	{
		horizontal_angle = carmen_normalize_theta(carmen_degrees_to_radians(velodyne_message->partial_scan[j].angle));

		if (fabs(carmen_normalize_theta(horizontal_angle - camera_pose.orientation.yaw)) > M_PI_2) // Disregard laser shots out of the camera's field of view
			continue;

		previous_range = (((double) velodyne_message->partial_scan[j].distance[0]) / 500.0);
		previous_vertical_angle = carmen_normalize_theta(carmen_degrees_to_radians(sorted_vertical_angles[0]));

		for (int i = 1; i < 32; i++)
		{
			for (int obj_index = 0; obj_index < predictions.size(); obj_index++)
			{
				// range = (((double) velodyne_message->partial_scan[j].distance[i]) / 500.0);
				range = estimate_distance (image_height, predictions[obj_index].h, camera_index);

				/*if (range <= MIN_RANGE || range >= MAX_RANGE)
					continue;
				*/
				vertical_angle = carmen_normalize_theta(carmen_degrees_to_radians(sorted_vertical_angles[i]));

				tf::Point p3d_velodyne_reference = spherical_to_cartesian(horizontal_angle, vertical_angle, range);
				/*
				tf::Point p3d_velodyne_reference_1 = spherical_to_cartesian(horizontal_angle, previous_vertical_angle, previous_range);
				*/
				/*
				// Jose's method check if a point is obstacle
				double delta_x = abs(p3d_velodyne_reference.x() - p3d_velodyne_reference_1.x());
				double delta_z = abs(p3d_velodyne_reference.z() - p3d_velodyne_reference_1.z());
				double line_angle = carmen_radians_to_degrees(fabs(atan2(delta_z, delta_x)));
				
				if (!(line_angle > MIN_ANGLE_OBSTACLE) && (line_angle < MAX_ANGLE_OBSTACLE))
					continue;
				*/
				tf::Point p3d_camera_reference = move_to_camera_reference(p3d_velodyne_reference, velodyne_pose, camera_pose);
				image_x = (unsigned int) (fx_meters * (p3d_camera_reference.y() / p3d_camera_reference.x()) / camera_parameters.pixel_size + cu);
				image_y = (unsigned int) (fy_meters * (-p3d_camera_reference.z() / p3d_camera_reference.x()) / camera_parameters.pixel_size + cv);
				
				if (image_x >= crop_x && image_x <= max_x && image_y >= crop_y && image_y <= max_y)
				{
					image_cartesian point;
					point.shot_number = j;
					point.ray_number  = i;
					point.image_x     = image_x - crop_x;
					point.image_y     = image_y - crop_y;
					point.cartesian_x = p3d_velodyne_reference.x();
					point.cartesian_y = -p3d_velodyne_reference.y();             // Must be inverted because Velodyne angle is reversed with CARMEN angles
					point.cartesian_z = p3d_velodyne_reference.z();
					points.push_back(point);
				}
				previous_range = range;
				previous_vertical_angle = vertical_angle;
			}
		}
	}
	return points;
}

int 
virtual_lidar(Mat open_cv_image, double timestamp, int camera_index)
{
	if (globalpos_msg == NULL)
		return 1;

	static bool first_time = true;
	double fps;
	static double start_time = 0.0;
	double dist_to_movable_object_track = DBL_MAX;

	vector<bbox_t> predictions;
	vector<image_cartesian> points;
	vector<vector<image_cartesian>> points_inside_bbox;
	vector<vector<image_cartesian>> filtered_points;

	if (first_time)
	{
		// init_python(open_cv_image.cols, open_cv_image.rows);

		original_img_width = open_cv_image.cols;
		original_img_height = open_cv_image.rows;

		if (crop_w == -1)
			crop_w = open_cv_image.cols;

		if (crop_h == -1)
			crop_h = open_cv_image.rows;
		
		first_time = false;
	}

	// if (velodyne_vector.size() > 0)
	// 	velodyne_sync_with_cam = find_velodyne_most_sync_with_cam(image_msg->timestamp);
	// else
	// 	return;

	Rect myROI(crop_x, crop_y, crop_w, crop_h);     // TODO put this in the .ini file
	open_cv_image = open_cv_image(myROI);
	/* YOLO with track_id*/
	predictions = run_YOLO(open_cv_image.data, open_cv_image.cols, open_cv_image.rows, 0.45);
	/* YOLO */
	predictions = filter_predictions_of_interest(predictions);

	insert_missing_movable_objects_in_the_track(predictions);
	/* Piumbini: Generate these points with virtual lidar, using estimated distance */
	points = velodyne_camera_calibration_fuse_virtual_lidar(velodyne_msg, camera_params[camera_index], velodyne_pose, camera_pose[camera_index],
				original_img_width, original_img_height, crop_x, crop_y, crop_w, crop_h, movable_object_tracks, camera_index);
	
	// points = velodyne_camera_calibration_fuse_camera_lidar(velodyne_msg, camera_params[camera_index], velodyne_pose, camera_pose[camera_index],
	// 			original_img_width, original_img_height, crop_x, crop_y, crop_w, crop_h);
	// //	vector<image_cartesian> points = sick_camera_calibration_fuse_camera_lidar(&sick_sync_with_cam, camera_params, &transformer_sick,
	//			image_msg->width, image_msg->height, crop_x, crop_y, crop_w, crop_h);
	
	points_inside_bbox = get_points_inside_bounding_boxes(movable_object_tracks, points); 
	
	filtered_points = filter_object_points_using_dbscan(points_inside_bbox);
	
	vector<image_cartesian> positions = compute_detected_objects_poses(filtered_points);
	for (unsigned int i = 0; i < positions.size(); i++)
	{
		if (!(positions[i].cartesian_x == -999.0 && positions[i].cartesian_y == -999.0))
		{
			carmen_translte_2d(&positions[i].cartesian_x, &positions[i].cartesian_y, board_pose.position.x, board_pose.position.y);
			//			carmen_translte_2d(&positions[i].cartesian_x, &positions[i].cartesian_y, bullbar_pose.position.x, bullbar_pose.position.y); //bullbar if the points are from sick, board if the points are from velodyne
			carmen_rotate_2d  (&positions[i].cartesian_x, &positions[i].cartesian_y, carmen_normalize_theta(globalpos_msg->globalpos.theta));
			carmen_translte_2d(&positions[i].cartesian_x, &positions[i].cartesian_y, globalpos_msg->globalpos.x, globalpos_msg->globalpos.y);
			update_world_position(&movable_object_tracks[i], positions[i].cartesian_x,positions[i].cartesian_y, timestamp);
			printf("[%03d] Velocity: %2.2f  - Orientation(absolute | car): %.3f | %.3f \n",
					movable_object_tracks[i].track_id, movable_object_tracks[i].velocity,movable_object_tracks[i].orientation,abs(movable_object_tracks[i].orientation - globalpos_msg->globalpos.theta));
		}
	}
	clean_movable_objects(1.0);

	carmen_moving_objects_point_clouds_message msg = build_detected_objects_message(movable_object_tracks, filtered_points);

	publish_moving_objects_message(&msg);

	fps = 1.0 / (carmen_get_time() - start_time);
	start_time = carmen_get_time();
	show_detections(open_cv_image, movable_object_tracks, predictions, points, points_inside_bbox, filtered_points, fps, original_img_width, original_img_height, crop_x, crop_y, crop_w, crop_h, dist_to_movable_object_track, camera_index);
	
	return 0;
}

vector<carmen_vector_2D_t> moving_objecst_cells_vector;

/*
 * Authors: Marcos Thiago Piumbini
 * Source Code for Virtual LiDAR using only cameras
 * It is based on neural_object_detector_tracker.cpp
 * */

/** End of codes with rangenet_lib, libsqueeze_seg_v2 and salsanet*/

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
publish_moving_objects_message(carmen_moving_objects_point_clouds_message *msg)
{
	msg->timestamp = carmen_get_time();
	msg->host = carmen_get_host();

    carmen_moving_objects_point_clouds_publish_message_generic(0, msg);
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
	velodyne_msg = velodyne_message;

	carmen_velodyne_camera_calibration_arrange_velodyne_vertical_angles_to_true_position(velodyne_msg);
}

void
carmen_laser_ldmrs_new_message_handler(carmen_laser_ldmrs_new_message* laser_message)
{
	sick_laser_message = laser_message;

	carmen_laser_ldmrs_new_message sick_copy;
	sick_copy.host = sick_laser_message->host;
	sick_copy.angle_ticks_per_rotation = sick_laser_message->angle_ticks_per_rotation;
	sick_copy.end_angle = sick_laser_message->end_angle;
	sick_copy.flags = sick_laser_message->flags;
	sick_copy.scan_end_time = sick_laser_message->scan_end_time;
	sick_copy.scan_number = sick_laser_message->scan_number;
	sick_copy.scan_points = sick_laser_message->scan_points;
	sick_copy.scan_start_time = sick_laser_message->scan_start_time;
	sick_copy.scanner_status = sick_laser_message->scanner_status;
	sick_copy.start_angle = sick_laser_message->start_angle;
	sick_copy.sync_phase_offset = sick_laser_message->sync_phase_offset;
	sick_copy.timestamp = sick_laser_message->timestamp;

	sick_copy.arraypoints = (carmen_laser_ldmrs_new_point *) malloc(
			sizeof(carmen_laser_ldmrs_new_point) * sick_laser_message->scan_points);

	memcpy(sick_copy.arraypoints, sick_laser_message->arraypoints,
			sizeof(carmen_laser_ldmrs_new_point) * sick_laser_message->scan_points);

	sick_vector.push_back(sick_copy);

	if (sick_vector.size() > MAX_POSITIONS)
	{
		free(sick_vector.begin()->arraypoints);
		sick_vector.erase(sick_vector.begin());
	}
}


void
localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	globalpos_msg = msg;
}


void
rddf_annotation_message_handler(carmen_rddf_annotation_message *msg)
{
	rddf_annotation_message = msg;
}

void
bumblebee_basic_image_handler(int camera, carmen_bumblebee_basic_stereoimage_message *msg)
{
	unsigned char *img;
	
	if (camera_alive[camera] == 1)
		img = msg->raw_right;
	else
		img = msg->raw_left;

	Mat open_cv_image = Mat(msg->height, msg->width, CV_8UC3, img, 0);              // CV_32FC3 float 32 bit 3 channels (to char image use CV_8UC3)
	
	virtual_lidar(open_cv_image, msg->timestamp, camera);
}


void
bumblebee_basic1_image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	bumblebee_basic_image_handler(1, image_msg);
}


void
bumblebee_basic2_image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	bumblebee_basic_image_handler(2, image_msg);
}


void
bumblebee_basic3_image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	bumblebee_basic_image_handler(3, image_msg);
}


void
bumblebee_basic4_image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	bumblebee_basic_image_handler(4, image_msg);
}


void
bumblebee_basic5_image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	bumblebee_basic_image_handler(5, image_msg);
}


void
bumblebee_basic6_image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	bumblebee_basic_image_handler(6, image_msg);
}


void
bumblebee_basic7_image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	bumblebee_basic_image_handler(7, image_msg);
}


void
bumblebee_basic8_image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	bumblebee_basic_image_handler(8, image_msg);
}


void
bumblebee_basic9_image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	bumblebee_basic_image_handler(9, image_msg);
}

void
bumblebee_basic10_image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	bumblebee_basic_image_handler(10, image_msg);
}

void (*image_handler[]) (carmen_bumblebee_basic_stereoimage_message *) =
{
		NULL,
		bumblebee_basic1_image_handler,
		bumblebee_basic2_image_handler,
		bumblebee_basic3_image_handler,
		bumblebee_basic4_image_handler,
		bumblebee_basic5_image_handler,
		bumblebee_basic6_image_handler,
		bumblebee_basic7_image_handler,
		bumblebee_basic8_image_handler,
		bumblebee_basic9_image_handler,
		bumblebee_basic10_image_handler,
};

static void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		fprintf(stderr, "Virtual LiDAR: disconnected.\n");

		exit(0);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Initializations                                                                           //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

static void
get_camera_param(int argc, char **argv, int camera)
{
	char bumblebee_name[256];
	char camera_name[256];

	if (camera_alive[camera] >= 0)
	{
		sprintf(bumblebee_name, "bumblebee_basic%d", camera);
		sprintf(camera_name, "camera%d", camera);

		carmen_param_t param_list[] =
		{
			{bumblebee_name, (char*) "fx",         CARMEN_PARAM_DOUBLE, &camera_params[camera].fx_factor,       0, NULL },
			{bumblebee_name, (char*) "fy",         CARMEN_PARAM_DOUBLE, &camera_params[camera].fy_factor,       0, NULL },
			{bumblebee_name, (char*) "cu",         CARMEN_PARAM_DOUBLE, &camera_params[camera].cu_factor,       0, NULL },
			{bumblebee_name, (char*) "cv",         CARMEN_PARAM_DOUBLE, &camera_params[camera].cv_factor,       0, NULL },
			{bumblebee_name, (char*) "pixel_size", CARMEN_PARAM_DOUBLE, &camera_params[camera].pixel_size,      0, NULL },
			{bumblebee_name, (char*) "fov",        CARMEN_PARAM_DOUBLE, &camera_params[camera].fov,             0, NULL },
			{bumblebee_name, (char*) "tlight_dist_correction", CARMEN_PARAM_DOUBLE, &camera_params[camera].focal_length, 0, NULL },

			{camera_name,    (char*) "x",          CARMEN_PARAM_DOUBLE, &camera_pose[camera].position.x,        0, NULL },
			{camera_name,    (char*) "y",          CARMEN_PARAM_DOUBLE, &camera_pose[camera].position.y,        0, NULL },
			{camera_name,    (char*) "z",          CARMEN_PARAM_DOUBLE, &camera_pose[camera].position.z,        0, NULL },
			{camera_name,    (char*) "roll",       CARMEN_PARAM_DOUBLE, &camera_pose[camera].orientation.roll,  0, NULL },
			{camera_name,    (char*) "pitch",      CARMEN_PARAM_DOUBLE, &camera_pose[camera].orientation.pitch, 0, NULL },
			{camera_name,    (char*) "yaw",        CARMEN_PARAM_DOUBLE, &camera_pose[camera].orientation.yaw,   0, NULL },
		};

		carmen_param_allow_unfound_variables(0);
		carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

		// camera_msg_count[camera] = 0;
		// camera_filter_count[camera] = 0;
		// camera_datmo_count[camera] = 0;
	}
}


static void
read_camera_parameters(int argc, char **argv)
{
	char *camera_side[MAX_CAMERA_INDEX + 1] = {NULL};

	carmen_param_t camera_param_list[] =
	{
		{(char *) "commandline", (char *) "camera1", CARMEN_PARAM_STRING, &camera_side[1], 0, NULL},
		{(char *) "commandline", (char *) "camera2", CARMEN_PARAM_STRING, &camera_side[2], 0, NULL},
		{(char *) "commandline", (char *) "camera3", CARMEN_PARAM_STRING, &camera_side[3], 0, NULL},
		{(char *) "commandline", (char *) "camera4", CARMEN_PARAM_STRING, &camera_side[4], 0, NULL},
		{(char *) "commandline", (char *) "camera5", CARMEN_PARAM_STRING, &camera_side[5], 0, NULL},
		{(char *) "commandline", (char *) "camera6", CARMEN_PARAM_STRING, &camera_side[6], 0, NULL},
		{(char *) "commandline", (char *) "camera7", CARMEN_PARAM_STRING, &camera_side[7], 0, NULL},
		{(char *) "commandline", (char *) "camera8", CARMEN_PARAM_STRING, &camera_side[8], 0, NULL},
		{(char *) "commandline", (char *) "camera9", CARMEN_PARAM_STRING, &camera_side[9], 0, NULL},
	};

	carmen_param_allow_unfound_variables(1);
	carmen_param_install_params(argc, argv, camera_param_list, sizeof(camera_param_list) / sizeof(camera_param_list[0]));

	active_cameras = 0;
	for (int i = 1; i <= MAX_CAMERA_INDEX; i++)
	{
		camera_alive[i] = -1;
		if (camera_side[i] == NULL)
			continue;

		if (strcmp(camera_side[i], "left") == 0 || strcmp(camera_side[i], "0") == 0)
			camera_alive[i] = 0;
		else if (strcmp(camera_side[i], "right") == 0 || strcmp(camera_side[i], "1") == 0)
			camera_alive[i] = 1;
		else
			carmen_die("-camera%d %s: Wrong camera side option. Must be either left or right\n", i, camera_side[i]);

		active_cameras++;
		get_camera_param(argc, argv, i);
	}
	if (active_cameras == 0)
		fprintf(stderr, "No cameras active for virtual_lidar\n\n");
	
}

void
read_parameters(int argc, char **argv)
{
	if ((argc < 3))
		carmen_die("%s: Wrong number of parameters. virtual_lidar requires 2 parameter and received %d. \n Usage: %s <camera_model> <message_number>\n", argv[0], argc - 1, argv[0]);

	read_camera_parameters(argc, argv);

	carmen_param_t param_list[] =
    {
		{(char *) "velodyne", (char *) "x",     CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.x), 0, NULL},
		{(char *) "velodyne", (char *) "y",     CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.y), 0, NULL},
		{(char *) "velodyne", (char *) "z",     CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.z), 0, NULL},
		{(char *) "velodyne", (char *) "roll",  CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.roll), 0, NULL},
		{(char *) "velodyne", (char *) "pitch", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.pitch), 0, NULL},
		{(char *) "velodyne", (char *) "yaw",   CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.yaw), 0, NULL},

		{(char *) "sensor_board_1", (char*) "x",     CARMEN_PARAM_DOUBLE, &board_pose.position.x, 0, NULL },
		{(char *) "sensor_board_1", (char*) "y",     CARMEN_PARAM_DOUBLE, &board_pose.position.y, 0, NULL },
		{(char *) "sensor_board_1", (char*) "z",     CARMEN_PARAM_DOUBLE, &board_pose.position.z, 0, NULL },
		{(char *) "sensor_board_1", (char*) "roll",  CARMEN_PARAM_DOUBLE, &board_pose.orientation.roll, 0, NULL},
		{(char *) "sensor_board_1", (char*) "pitch", CARMEN_PARAM_DOUBLE, &board_pose.orientation.pitch, 0, NULL},
		{(char *) "sensor_board_1", (char*) "yaw",   CARMEN_PARAM_DOUBLE, &board_pose.orientation.yaw, 0, NULL},
    };
    carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

 	carmen_param_allow_unfound_variables(1);
   	carmen_param_t optional_commandline_param_list[] =
   	{
		{(char *) "commandline", (char *) "resize", CARMEN_PARAM_DOUBLE, &resize_factor, 0, NULL},
   		{(char *) "commandline", (char *) "cropx",  CARMEN_PARAM_INT, &crop_x, 0, NULL},
   		{(char *) "commandline", (char *) "cropy",  CARMEN_PARAM_INT, &crop_y, 0, NULL},
   		{(char *) "commandline", (char *) "cropw",  CARMEN_PARAM_INT, &crop_w, 0, NULL},
   		{(char *) "commandline", (char *) "croph", CARMEN_PARAM_INT, &crop_h, 0, NULL},
   	};
   	carmen_param_install_params(argc, argv, optional_commandline_param_list, sizeof(optional_commandline_param_list) / sizeof(optional_commandline_param_list[0]));

	// printf ("%s %d Iid%d %d %d %d %d\n", camera_model, message_number, resize_factor, crop_x, crop_y, crop_h, crop_w);
}


void
subscribe_messages()
{
	carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t) velodyne_partial_scan_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_laser_subscribe_ldmrs_new_message(NULL, (carmen_handler_t) carmen_laser_ldmrs_new_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_rddf_subscribe_annotation_message(NULL, (carmen_handler_t) rddf_annotation_message_handler, CARMEN_SUBSCRIBE_LATEST);

	for (int camera = 1; camera <= MAX_CAMERA_INDEX; camera++)
	{
		if (camera_alive[camera] >= 0)
			carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) image_handler[camera], CARMEN_SUBSCRIBE_LATEST);
	}
}

void
initializer_YOLO()
{
	char* carmen_home = getenv("CARMEN_HOME");
	char classes_names_path[1024];
	char yolo_cfg_path[1024];
	char yolo_weights_path[1024];

	sprintf(classes_names_path, "%s/sharedlib/darknet4/data/coco.names", carmen_home);
	sprintf(yolo_cfg_path, "%s/sharedlib/darknet4/cfg/yolov4.cfg", carmen_home);
	sprintf(yolo_weights_path, "%s/sharedlib/darknet4/yolov4.weights", carmen_home);

	classes_names = get_classes_names(classes_names_path);
	network_struct = load_yolo_network(yolo_cfg_path, yolo_weights_path, 0);
}
//////////////////////////////////////////////////////////////////////////////////////////////////


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);

	read_parameters(argc, argv);

	subscribe_messages();

	carmen_moving_objects_point_clouds_define_messages_generic(0);

	signal(SIGINT, shutdown_module);

	initializer_YOLO();

	setlocale(LC_ALL, "C");

	carmen_ipc_dispatch();

	return (0);
}
