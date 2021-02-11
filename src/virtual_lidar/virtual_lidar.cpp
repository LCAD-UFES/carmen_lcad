#include "neural_object_detector2.hpp"
#include <carmen/tf.h>
#include <sys/stat.h>

#define SHOW_DETECTIONS

//bool rectangles_intersects(cv::Point l1, cv::Point r1, cv::Point l2, cv::Point r2);
bool rectangles_intersects(cv::Rect A, cv::Rect B);

using namespace std;

//yolov3//
char **classes_names;
void *network_struct;
//*********//

cv::Mat or_image;
int camera;
int camera_side;
carmen_camera_parameters camera_parameters;
double focal_length;
carmen_pose_3D_t velodyne_pose;
carmen_pose_3D_t bullbar_pose;
carmen_pose_3D_t sick_pose;
carmen_pose_3D_t camera_pose;
carmen_pose_3D_t board_pose;
tf::Transformer transformer;
tf::Transformer transformer_sick;
carmen_laser_ldmrs_new_message sick_sync_with_cam;

const unsigned int maxPositions = 50;
carmen_velodyne_partial_scan_message *velodyne_message_arrange;
vector<carmen_velodyne_partial_scan_message> velodyne_vector;

carmen_laser_ldmrs_new_message* sick_laser_message;
carmen_velodyne_partial_scan_message sick_message_arrange;
vector<carmen_laser_ldmrs_new_message> sick_vector;

vector<string> obj_names;

carmen_moving_objects_point_clouds_message moving_objects_point_clouds_message;
carmen_point_t globalpos;
carmen_pose_3D_t pose;

#define CAM_DELAY 0.2
#define MAX_POSITIONS 10
// This function find the closest velodyne message with the camera message
carmen_velodyne_partial_scan_message
find_velodyne_most_sync_with_cam(double bumblebee_timestamp)  // TODO is this necessary?
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
build_moving_objects_message(vector<carmen_tracked_cluster_t> clusters)
{

    moving_objects_point_clouds_message.num_point_clouds = clusters.size();
    moving_objects_point_clouds_message.point_clouds = (t_point_cloud_struct *) (malloc(
            moving_objects_point_clouds_message.num_point_clouds * sizeof(t_point_cloud_struct)));


    for (int i = 0; i < moving_objects_point_clouds_message.num_point_clouds; i++) {
        carmen_vector_3D_t box_centroid = compute_centroid(clusters[i].points);
        carmen_vector_3D_t offset;

        // TODO ler do carmen ini
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


vector<string>
objects_names_from_file(string const class_names_file)
{
    ifstream file(class_names_file);
    vector<string> file_lines;

    if (!file.is_open())
    	return file_lines;

    for (string line; getline(file, line);)
    	file_lines.push_back(line);

    cout << "object names loaded \n";

    return file_lines;
}
//Piumbini
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

//Piumbini

std::vector< std::vector<carmen_velodyne_points_in_cam_t> >
sick_points_in_boxes(std::vector<bounding_box> bouding_boxes_list,
                         carmen_camera_parameters camera_parameters,
						 unsigned int width, unsigned int height)
{
	int crop_x = 0;
	int crop_y = 0;
	int crop_w = width;// 1280;
	int crop_h = height;
	vector<carmen_velodyne_points_in_cam_t> sick_points = carmen_sick_camera_calibration_lasers_points_in_camera(sick_laser_message,
			camera_parameters,
			&transformer_sick,
			width, height);
	vector<image_cartesian> points = sick_camera_calibration_fuse_camera_lidar(&sick_sync_with_cam, camera_parameters, &transformer_sick,
			width, height, crop_x, crop_y, crop_w, crop_h);

	std::vector< std::vector<carmen_velodyne_points_in_cam_t> > laser_points_in_camera_box_list;

	
	for (unsigned int i = 0; i < bouding_boxes_list.size(); i++)
	{
		std::vector<carmen_velodyne_points_in_cam_t> laser_points_in_camera_box;
		for (unsigned int j = 0; j < sick_points.size(); j++)
		{
			if (sick_points[j].ipx > bouding_boxes_list[i].pt1.x
					&& sick_points[j].ipx < bouding_boxes_list[i].pt2.x
					&& sick_points[j].ipy > bouding_boxes_list[i].pt1.y
					&& sick_points[j].ipy < bouding_boxes_list[i].pt2.y)
			{
				laser_points_in_camera_box.push_back(sick_points[j]);
			}
		}

		laser_points_in_camera_box_list.push_back(laser_points_in_camera_box);
	}

	return laser_points_in_camera_box_list;
}

void
show_detections(cv::Mat *rgb_image, vector<vector<carmen_velodyne_points_in_cam_with_obstacle_t>> laser_points_in_camera_box_list,
		vector<vector<carmen_velodyne_points_in_cam_t>> sick_points_in_camera_box_list,
		vector<bbox_t> predictions, double hood_removal_percentage, double fps,
                vector<carmen_position_t> rddf_points, string window_name)
{
    char confianca[25];
    char frame_rate[25];
	char sick_string[25];
	char distance_string[25];
	char lidar_string[25];

    sprintf(frame_rate, "FPS = %.2f", fps);
	double image_height = (double) rgb_image->rows; // 768 pixels
	// double focal_length = 6.0; //f(mm)
	//double sensor_height = camera_pose.position.z * 10000.0; // sensorheight(mm)
	double focal_length = 6.0; //f(mm) 3.8
	double sensor_height = 4.927868852;

    //cv::putText(*rgb_image, frame_rate, cv::Point(10, 25), cv::FONT_HERSHEY_PLAIN, 2, cvScalar(0, 255, 0), 2);

    for (unsigned int i = 0; i < predictions.size(); i++)
    {
		double lidar_distance = 0.0;
		double sick_distance = 0.0;
		bool first = true;
		bool first_sick = true;
		double real_height = 1500.0; //car_height(mm)
		for (unsigned int j = 0; j < laser_points_in_camera_box_list[i].size(); j++)
		{
			cv::circle(*rgb_image, cv::Point(laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipx,
					laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy), 1, cv::Scalar(0, 0, 255), 1);
			if (laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.laser_polar.length != 0.0)
			{
				if(first)
				{
					lidar_distance = laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.laser_polar.length;
					first = false;
				} else {
					// Takes the middle point of bbox
					if ( laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipx > (predictions[i].x + predictions[i].w)/2 - 5 &&
					 laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipx < (predictions[i].x + predictions[i].w)/2 + 5 &&
					 laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy > (predictions[i].y + predictions[i].h)/2 - 5 &&
					 laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy < (predictions[i].y + predictions[i].h)/2 + 5)
					 	lidar_distance = laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.laser_polar.length;
						
				}
				// real_height = lidar_distance * 500.0 * (double) predictions[i].h * sensor_height / (focal_length * image_height); // mm
			}
		}
		for (unsigned int j = 0; j < sick_points_in_camera_box_list[i].size(); j++)
		{
			cv::circle(*rgb_image, cv::Point(sick_points_in_camera_box_list[i][j].ipx,
					sick_points_in_camera_box_list[i][j].ipy), 1, cv::Scalar(0, 255, 255), 1);
			if (sick_points_in_camera_box_list[i][j].laser_polar.length != 0.0 && sick_points_in_camera_box_list[i][j].laser_polar.length < 150.0)
			{
				if(first_sick)
				{
					sick_distance = sick_points_in_camera_box_list[i][j].laser_polar.length;
					first_sick = false;
				} else {
					if ( sick_points_in_camera_box_list[i][j].ipx > (predictions[i].x + predictions[i].w)/2 - 5 &&
					 sick_points_in_camera_box_list[i][j].ipx < (predictions[i].x + predictions[i].w)/2 + 5 &&
					 sick_points_in_camera_box_list[i][j].ipy > (predictions[i].y + predictions[i].h)/2 - 5 &&
					 sick_points_in_camera_box_list[i][j].ipy < (predictions[i].y + predictions[i].h)/2 + 5)
						sick_distance = sick_points_in_camera_box_list[i][j].laser_polar.length;
				}
			}
		}
        cv::Scalar object_color;

        // sprintf(confianca, "%d  %.3f", predictions.at(i).obj_id, predictions.at(i).prob);

        int obj_id = predictions.at(i).obj_id;

        string obj_name;
        if (obj_names.size() > obj_id)
            obj_name = obj_names[obj_id];

        if (obj_name.compare("car") == 0)
        {
        	object_color = cv::Scalar(0, 255, 0);
        	cv::rectangle(*rgb_image,
        			cv::Point(predictions[i].x, predictions[i].y),
					cv::Point(predictions[i].x + predictions[i].w, predictions[i].y + predictions[i].h),
					object_color, 1);

			double distance_to_object = focal_length * real_height * image_height / (predictions[i].h * sensor_height); // mm
			distance_to_object = distance_to_object / 1000.0; // mm to meters.
			double accuracy_lidar;
			double accuracy_sick;
			cout << "\t" << distance_to_object << "\t";
			sprintf(distance_string, "%.2f", distance_to_object);
			if (lidar_distance != 0.0){
				accuracy_lidar = distance_error(distance_to_object, lidar_distance);
				sprintf(lidar_string, "%.2f", lidar_distance);
				cout << lidar_distance << "\t" << accuracy_lidar << "\t";
			}else{
				sprintf(lidar_string, "NL");
				cout << "NL" << "\tNLA\t" ;
			}
			if (sick_distance != 0.0){
				accuracy_sick = distance_error(distance_to_object, sick_distance);
				sprintf(sick_string, "%.2f", sick_distance);
				cout << sick_distance << "\t" << accuracy_sick << "\t";
			}else{
				sprintf(sick_string, "NS");
				cout << "NS" << "\tNSA\t";
			}
			cout << endl;


        	cv::putText(*rgb_image, lidar_string,
        			cv::Point(predictions[i].x + 1, predictions[i].y - 3),
					cv::FONT_HERSHEY_PLAIN, 1, cvScalar(0, 0, 255), 1);
			cv::putText(*rgb_image, sick_string,
        			cv::Point(predictions[i].x + 1, predictions[i].y - 15),
					cv::FONT_HERSHEY_PLAIN, 1, cvScalar(0, 255, 255), 1);
			cv::putText(*rgb_image, distance_string,
        			cv::Point(predictions[i].x + 1, predictions[i].y - 28),
					cv::FONT_HERSHEY_PLAIN, 1, cvScalar(0, 255, 0), 1);
        }


    }

    cv::imshow(window_name, *rgb_image);
    cv::waitKey(1);
}

void
detections(vector<bbox_t> predictions, carmen_bumblebee_basic_stereoimage_message *image_msg, carmen_velodyne_partial_scan_message velodyne_sync_with_cam,
		   cv::Mat src_image, cv::Mat *rgb_image, double start_time, double fps, vector<carmen_position_t> rddf_points,
		   string window_name)
{
	vector <bounding_box> bouding_boxes_list;
	double hood_removal_percentage = 0.2;
	vector<carmen_tracked_cluster_t> clusters;

	//predictions = darknet->tracking(predictions); // Coment this line if object tracking is not necessary

	for (const auto &box : predictions) // Covert Darknet bounding box to neural_object_deddtector bounding box
	{
		bounding_box bbox;

		bbox.pt1.x = box.x;
		bbox.pt1.y = box.y;
		bbox.pt2.x = box.x + box.w;
		bbox.pt2.y = box.y + box.h;

		bouding_boxes_list.push_back(bbox);
	}

	// Removes the ground, Removes points outside cameras field of view and Returns the points that are obstacles and are inside bboxes
	vector<vector<carmen_velodyne_points_in_cam_with_obstacle_t>> laser_points_in_camera_box_list = velodyne_points_in_boxes(bouding_boxes_list,
			&velodyne_sync_with_cam, camera_parameters, velodyne_pose, camera_pose, image_msg->width, image_msg->height);
	
	//SICK
	vector<vector<carmen_velodyne_points_in_cam_t>> sick_points_in_camera_box_list = sick_points_in_boxes(bouding_boxes_list,
			camera_parameters,
			image_msg->width, image_msg->height);

	// ONLY Convert from sferical to cartesian cordinates
	vector< vector<carmen_vector_3D_t>> cluster_list = get_cluster_list(laser_points_in_camera_box_list);

	// Cluster points and get biggest
	filter_points_in_clusters(&cluster_list);

	for (int i = 0; i < cluster_list.size(); i++)
	{
		carmen_moving_object_type tp = find_cluster_type_by_obj_id(obj_names, predictions.at(i).obj_id);

		int cluster_id = predictions.at(i).track_id;

		carmen_tracked_cluster_t clust;

		clust.points = cluster_list.at(i);

		clust.orientation = globalpos.theta;  //TODO: Calcular velocidade e orientacao corretas (provavelmente usando um tracker)
		clust.linear_velocity = 0.0;
		clust.track_id = cluster_id;
		clust.last_detection_timestamp = image_msg->timestamp;
		clust.cluster_type = tp;

		clusters.push_back(clust);
	}

	build_moving_objects_message(clusters);

	fps = 1.0 / (carmen_get_time() - start_time);
	start_time = carmen_get_time();

#ifdef SHOW_DETECTIONS
	show_detections(rgb_image, laser_points_in_camera_box_list, sick_points_in_camera_box_list, predictions,
			hood_removal_percentage, fps, rddf_points, window_name);
#endif
}


bool
check_rect_inside_image (cv::Rect rec, cv::Mat img)
{
	if(0 <= rec.x
		&& 0 <= rec.width
		&& rec.x + rec.width <= img.cols
		&& 0 <= rec.y
		&& 0 <= rec.height
		&& rec.y + rec.height <= img.rows)
	{
		return true;
	}

	return false;
}



bool valueInRange(int value, int min, int max)
{ return (value >= min) && (value <= max); }

bool rectangles_intersects(cv::Rect A, cv::Rect B)
{
    bool xOverlap = valueInRange(A.x, B.x, B.x + B.width) ||
                    valueInRange(B.x, A.x, A.x + A.width);

    bool yOverlap = valueInRange(A.y, B.y, B.y + B.height) ||
                    valueInRange(B.y, A.y, A.y + A.height);

    return xOverlap && yOverlap;
}


double
euclidean_distance (double x1, double x2, double y1, double y2)
{
	return ( sqrt(pow(x2-x1,2) + pow(y2-y1,2)) );
}


vector<cv::Scalar>
get_slice_colors (unsigned int slices_size)
{
	vector<cv::Scalar> colors;
	cv::Scalar color;
	if (slices_size <= 1)
	{
		color = cv::Scalar (0, 0, 255);
		colors.push_back(color);
	}
	if (slices_size <= 2)
	{
		color = cv::Scalar (0, 255, 0);
		colors.push_back(color);
	}
	if (slices_size <= 3)
	{
		color = cv::Scalar (255, 0, 0);
		colors.push_back(color);
	}
	if (slices_size <= 4)
	{
		color = cv::Scalar (255, 255, 0);
		colors.push_back(color);
	}
	if (slices_size <= 5)
	{
		color = cv::Scalar (255, 0, 255);
		colors.push_back(color);
	}
	if (slices_size <= 6)
	{
		color = cv::Scalar (0, 255, 255);
		colors.push_back(color);
	}
	if (slices_size <= 7)
	{
		color = cv::Scalar (0, 0, 0);
		colors.push_back(color);
	}
	if (slices_size <= 8)
	{
		color = cv::Scalar (255, 255, 255);
		colors.push_back(color);
	}
	return (colors);
}

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
publish_moving_objects_message(double timestamp)
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

#define crop_x 0.0
#define crop_y 1.0

void
image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	char janela[25];
	char arr[50];
	vector<carmen_position_t> rddf_points_in_image_filtered;
	vector<carmen_position_t> rddf_points_in_image_full;
	vector<double> distances_of_rddf_from_car;
	double hood_removal_percentage = 0.2;
	carmen_velodyne_partial_scan_message velodyne_sync_with_cam;
	cv::Size size(320, 320);
	
    cv::Mat src_image = cv::Mat(cv::Size(image_msg->width, image_msg->height - image_msg->height * hood_removal_percentage), CV_8UC3);
    cv::Mat rgb_image = cv::Mat(cv::Size(image_msg->width, image_msg->height - image_msg->height * hood_removal_percentage), CV_8UC3);

    static double start_time = 0.0;
	double fps;

    if (camera_side == 0)
        memcpy(src_image.data, image_msg->raw_left, image_msg->image_size * sizeof(char) - image_msg->image_size * hood_removal_percentage * sizeof(char));
    else
        memcpy(src_image.data, image_msg->raw_right, image_msg->image_size * sizeof(char) - image_msg->image_size * hood_removal_percentage * sizeof(char));

    if (velodyne_vector.size() > 0)
        velodyne_sync_with_cam = find_velodyne_most_sync_with_cam(image_msg->timestamp); // TODO não faz sentido! Tem que sempre pegar a ultima msg do velodyne
    else
        return;
	
	if (sick_vector.size() > 0)
		sick_sync_with_cam = find_sick_most_sync_with_cam(image_msg->timestamp);
	else
		return;

	cv::Mat src_image_copy = src_image.clone();
	
    cv::Mat pRoi = src_image_copy(cv::Rect(src_image_copy.cols * crop_x / 2.0, 0,
    		src_image_copy.cols - src_image_copy.cols * crop_x, src_image_copy.rows));
    src_image = pRoi;
    src_image_copy = src_image.clone();
	
    cv::cvtColor(src_image, rgb_image, cv::COLOR_RGB2BGR);
	
    cv::Mat rgb_image_copy = rgb_image.clone();
    or_image = rgb_image.clone();
	
    vector<bbox_t> bounding_boxes_of_slices_in_original_image;
    vector<cv::Scalar> colors;
    vector<cv::Mat> scene_slices;
    vector<t_transform_factor> transform_factor_of_slice_to_original_frame;
    
	bounding_boxes_of_slices_in_original_image = run_YOLO(src_image.data, 3, src_image.cols, src_image.rows, network_struct, classes_names, 0.5, 0.5);
	if (bounding_boxes_of_slices_in_original_image.size()>0)
	{
		sprintf(arr,"%lf", image_msg->timestamp);
		cout << arr;
	}
    detections(bounding_boxes_of_slices_in_original_image, image_msg, velodyne_sync_with_cam, src_image, &rgb_image, start_time, fps, rddf_points_in_image_filtered, "Original Detection");
    colors = get_slice_colors (1);
   
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

    if (velodyne_vector.size() > maxPositions)
    {
        free(velodyne_vector.begin()->partial_scan);
        velodyne_vector.erase(velodyne_vector.begin());
    }
}


void
playback_command_handler(carmen_playback_command_message *command)
{
	//command_of_playback = command;
	cout<<command->cmd<<" "<<command->message<<" "<<command->offset<<" "<<command->speed<<endl;
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
localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *globalpos_message)
{
	pose = globalpos_message->pose;
    globalpos.theta = globalpos_message->globalpos.theta;
    globalpos.x = globalpos_message->globalpos.x;
    globalpos.y = globalpos_message->globalpos.y;

    //printf("Global pos: %lf X %lf Theta: %lf\n", globalpos.x, globalpos.y, globalpos.theta);
    //printf("Global pose: %lf X %lf X lf Theta: %lf Roll: %lf Pitch: %lf\n", pose.position.x, pose.position.y, pose.position.y, pose.orientation.yaw, pose.orientation.roll, pose.orientation.pitch);
}


void
shutdown_module(int signo)
{
    if (signo == SIGINT) {
        carmen_ipc_disconnect();
        cvDestroyAllWindows();

        printf("Virtual LiDAR: disconnected.\n");
        exit(0);
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////

void
subscribe_messages()
{
    carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) image_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t) velodyne_partial_scan_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_laser_subscribe_ldmrs_new_message(NULL, (carmen_handler_t) carmen_laser_ldmrs_new_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);

}


int
read_parameters(int argc, char **argv)
{
    camera = atoi(argv[1]);             // Define the camera to be used
    camera_side = atoi(argv[2]);        // 0 For left image 1 for right image
    
    int num_items;

    char bumblebee_string[256];
    char camera_string[256];
    char bullbar_string[256];
    char sick_string[256];
    char velodyne_string[256];
    char sensor_board_string[256];



    sprintf(bumblebee_string, "%s%d", "bumblebee_basic", camera); // Geather the camera ID
    sprintf(camera_string, "%s%d", "camera", camera);
    sprintf(bullbar_string, "%s", "front_bullbar");
    sprintf(sick_string, "%s", "laser_ldmrs");
    sprintf(velodyne_string, "%s", "velodyne");
    sprintf(sensor_board_string, "%s", "sensor_board_1");

    carmen_param_t param_list[] =
    {
		{bumblebee_string, (char*) "fx", CARMEN_PARAM_DOUBLE, &camera_parameters.fx_factor, 0, NULL },
		{bumblebee_string, (char*) "fy", CARMEN_PARAM_DOUBLE, &camera_parameters.fy_factor, 0, NULL },
		{bumblebee_string, (char*) "cu", CARMEN_PARAM_DOUBLE, &camera_parameters.cu_factor, 0, NULL },
		{bumblebee_string, (char*) "cv", CARMEN_PARAM_DOUBLE, &camera_parameters.cv_factor, 0, NULL },
		{bumblebee_string, (char*) "baseline", CARMEN_PARAM_DOUBLE, &camera_parameters.baseline, 0, NULL },
		{bumblebee_string, (char*) "fov", CARMEN_PARAM_DOUBLE, &camera_parameters.fov, 0, NULL },
		{bumblebee_string, (char*) "pixel_size", CARMEN_PARAM_DOUBLE, &camera_parameters.pixel_size, 0, NULL },
		{bumblebee_string, (char*) "tlight_dist_correction", CARMEN_PARAM_DOUBLE, &focal_length, 0, NULL },

		{sensor_board_string, (char*) "x",     CARMEN_PARAM_DOUBLE, &board_pose.position.x, 0, NULL },
		{sensor_board_string, (char*) "y",     CARMEN_PARAM_DOUBLE, &board_pose.position.y, 0, NULL },
		{sensor_board_string, (char*) "z",     CARMEN_PARAM_DOUBLE, &board_pose.position.z, 0, NULL },
		{sensor_board_string, (char*) "roll",  CARMEN_PARAM_DOUBLE, &board_pose.orientation.roll, 0, NULL },
		{sensor_board_string, (char*) "pitch", CARMEN_PARAM_DOUBLE, &board_pose.orientation.pitch, 0, NULL },
		{sensor_board_string, (char*) "yaw",   CARMEN_PARAM_DOUBLE, &board_pose.orientation.yaw, 0, NULL },

		{camera_string, (char*) "x",     CARMEN_PARAM_DOUBLE, &camera_pose.position.x, 0, NULL },
		{camera_string, (char*) "y",     CARMEN_PARAM_DOUBLE, &camera_pose.position.y, 0, NULL },
		{camera_string, (char*) "z",     CARMEN_PARAM_DOUBLE, &camera_pose.position.z, 0, NULL },
		{camera_string, (char*) "roll",  CARMEN_PARAM_DOUBLE, &camera_pose.orientation.roll, 0, NULL },
		{camera_string, (char*) "pitch", CARMEN_PARAM_DOUBLE, &camera_pose.orientation.pitch, 0, NULL },
		{camera_string, (char*) "yaw",   CARMEN_PARAM_DOUBLE, &camera_pose.orientation.yaw, 0, NULL },

		{bullbar_string, (char*) "x",     CARMEN_PARAM_DOUBLE, &bullbar_pose.position.x, 0, NULL },
		{bullbar_string, (char*) "y",     CARMEN_PARAM_DOUBLE, &bullbar_pose.position.y, 0, NULL },
		{bullbar_string, (char*) "z",     CARMEN_PARAM_DOUBLE, &bullbar_pose.position.z, 0, NULL },
		{bullbar_string, (char*) "roll",  CARMEN_PARAM_DOUBLE, &bullbar_pose.orientation.roll, 0, NULL },
		{bullbar_string, (char*) "pitch", CARMEN_PARAM_DOUBLE, &bullbar_pose.orientation.pitch, 0, NULL },
		{bullbar_string, (char*) "yaw",   CARMEN_PARAM_DOUBLE, &bullbar_pose.orientation.yaw, 0, NULL },

		{sick_string, (char*) "x",     CARMEN_PARAM_DOUBLE, &sick_pose.position.x, 0, NULL },
		{sick_string, (char*) "y",     CARMEN_PARAM_DOUBLE, &sick_pose.position.y, 0, NULL },
		{sick_string, (char*) "z",     CARMEN_PARAM_DOUBLE, &sick_pose.position.z, 0, NULL },
		{sick_string, (char*) "roll",  CARMEN_PARAM_DOUBLE, &sick_pose.orientation.roll, 0, NULL },
		{sick_string, (char*) "pitch", CARMEN_PARAM_DOUBLE, &sick_pose.orientation.pitch, 0, NULL },
		{sick_string, (char*) "yaw",   CARMEN_PARAM_DOUBLE, &sick_pose.orientation.yaw, 0, NULL },

		{velodyne_string, (char*) "x",     CARMEN_PARAM_DOUBLE, &velodyne_pose.position.x, 0, NULL },
		{velodyne_string, (char*) "y",     CARMEN_PARAM_DOUBLE, &velodyne_pose.position.y, 0, NULL },
		{velodyne_string, (char*) "z",     CARMEN_PARAM_DOUBLE, &velodyne_pose.position.z, 0, NULL },
		{velodyne_string, (char*) "roll",  CARMEN_PARAM_DOUBLE, &velodyne_pose.orientation.roll, 0, NULL },
		{velodyne_string, (char*) "pitch", CARMEN_PARAM_DOUBLE, &velodyne_pose.orientation.pitch, 0, NULL },
		{velodyne_string, (char*) "yaw",   CARMEN_PARAM_DOUBLE, &velodyne_pose.orientation.yaw, 0, NULL }



		//LER SICK DO CARMEN INI
    };

    num_items = sizeof(param_list) / sizeof(param_list[0]);
    carmen_param_install_params(argc, argv, param_list, num_items);

    return 0;
}

void
initializer()
{
	initialize_transformations(board_pose, camera_pose, &transformer);
	initialize_sick_transformations(board_pose, camera_pose, bullbar_pose, sick_pose, &transformer_sick);

	char* carmen_home = getenv("CARMEN_HOME");
	char classes_names_path[1024];
	char yolo_cfg_path[1024];
	char yolo_weights_path[1024];

	sprintf(classes_names_path, "%s/sharedlib/darknet3/data/coco.names", carmen_home);
	sprintf(yolo_cfg_path, "%s/sharedlib/darknet3/cfg/yolov4.cfg", carmen_home);
	sprintf(yolo_weights_path, "%s/sharedlib/darknet3/yolov4.weights", carmen_home);

	classes_names = get_classes_names(classes_names_path);
	obj_names = objects_names_from_file(classes_names_path);
	network_struct = load_yolo_network(yolo_cfg_path, yolo_weights_path, 1);

}


int
main(int argc, char **argv)
{
    if ((argc != 3))
        carmen_die("%s: Wrong number of parameters. virtual_lidar requires 2 parameter and received %d. \n Usage: %s <camera_number> <camera_side(0-left; 1-right)>"
        		" \n",
                   argv[0], argc - 1, argv[0]);

    int device_id = 0;

    setlocale(LC_ALL, "C");

    carmen_ipc_initialize(argc, argv);

    signal(SIGINT, shutdown_module);

    read_parameters(argc, argv);

    initializer();

    subscribe_messages();

    carmen_ipc_dispatch();

    return 0;
}
