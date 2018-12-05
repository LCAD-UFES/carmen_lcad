#include "neural_object_detector.hpp"


int camera;
int camera_side;
char **classes_names;
void *network_struct;
carmen_localize_ackerman_globalpos_message *globalpos_msg = NULL;
carmen_velodyne_partial_scan_message *velodyne_msg;
carmen_camera_parameters camera_parameters;
carmen_pose_3D_t velodyne_pose;
carmen_pose_3D_t camera_pose;
carmen_pose_3D_t board_pose;
tf::Transformer transformer;

vector <carmen_pose_3D_t> annotation_vector;
carmen_pose_3D_t nearest_traffic_light_pose;

#define MAX_TRAFFIC_LIGHT_DIST 50
#define MIN_TRAFFIC_LIGHT_TRESHOLD 4
#define MAX_TRAFFIC_LIGHT_TRESHOLD 4

double
compute_distance_to_the_traffic_light()
{
    double nearest_traffic_light_distance = DBL_MAX;

    if (globalpos_msg == NULL)
    	return DBL_MAX;

    for (unsigned int i = 0; i < annotation_vector.size(); i++)
    {
    		double distance = sqrt(pow(globalpos_msg->globalpos.x - annotation_vector[i].position.x, 2) +
							pow(globalpos_msg->globalpos.y - annotation_vector[i].position.y, 2));
    		if (distance < nearest_traffic_light_distance)
    		{
    			bool orientation_ok = fabs(carmen_radians_to_degrees(globalpos_msg->globalpos.theta - annotation_vector[i].orientation.yaw)) < 10.0 ? 1 : 0;

    			bool behind = fabs(carmen_normalize_theta((atan2(globalpos_msg->globalpos.y - annotation_vector[i].position.y,
    							globalpos_msg->globalpos.x - annotation_vector[i].position.x) - M_PI - globalpos_msg->globalpos.theta))) > M_PI_2;

				if (distance <= MAX_TRAFFIC_LIGHT_DISTANCE && orientation_ok && behind == false)
				{
	    			nearest_traffic_light_distance = distance;
	    			nearest_traffic_light_pose = annotation_vector[i];
				}
    		}
    }
    return (nearest_traffic_light_distance);
}


void
carmen_translte_2d(double *x, double *y, double offset_x, double offset_y)
{
	*x += offset_x;
	*y += offset_y;
}


void
show_LIDAR_points(Mat &image, vector<image_cartesian> all_points, int r, int g, int b)
{
	for (unsigned int i = 0; i < all_points.size(); i++)
		circle(image, Point(all_points[i].image_x, all_points[i].image_y), 1, cvScalar(b, g, r), 5, 8, 0);
}


void
show_LIDAR(Mat &image, vector<vector<image_cartesian>> points_lists, int r, int g, int b)
{
	for (unsigned int i = 0; i < points_lists.size(); i++)
	{
		for (unsigned int j = 0; j < points_lists[i].size(); j++)
			circle(image, Point(points_lists[i][j].image_x, points_lists[i][j].image_y), 1, cvScalar(b, g, r), 5, 8, 0);
	}
}


void
show_all_points(Mat &image, unsigned int image_width, unsigned int image_height, unsigned int crop_x, unsigned int crop_y, unsigned int crop_width, unsigned int crop_height)
{
	vector<carmen_velodyne_points_in_cam_t> all_points = carmen_velodyne_camera_calibration_lasers_points_in_camera(
					velodyne_msg, camera_parameters, velodyne_pose, camera_pose, image_width, image_height);

	int max_x = crop_x + crop_width, max_y = crop_y + crop_height;

	for (unsigned int i = 0; i < all_points.size(); i++)
		if (all_points[i].ipx >= crop_x && all_points[i].ipx <= max_x && all_points[i].ipy >= crop_y && all_points[i].ipy <= max_y)
			circle(image, Point(all_points[i].ipx - crop_x, all_points[i].ipy - crop_y), 1, cvScalar(0, 0, 255), 5, 8, 0);
}


void
display(Mat image, vector<bbox_t> predictions, double fps, unsigned int image_width, unsigned int image_height)
{
	char object_info[25];
    char frame_rate[25];

    cvtColor(image, image, COLOR_RGB2BGR);

    sprintf(frame_rate, "FPS = %.2f", fps);

    putText(image, frame_rate, Point(10, 25), FONT_HERSHEY_PLAIN, 2, cvScalar(0, 255, 0), 2);

    for (unsigned int i = 0; i < predictions.size(); i++)
    {
    	//printf("---%d \n", predictions[i].obj_id);
        sprintf(object_info, "%d %s %d", predictions[i].obj_id, classes_names[predictions[i].obj_id], (int)predictions[i].prob);

        rectangle(image, Point(predictions[i].x, predictions[i].y), Point((predictions[i].x + predictions[i].w), (predictions[i].y + predictions[i].h)),
        		Scalar(0, 0, 255), 1);

        putText(image, object_info/*(char*) "Obj"*/, Point(predictions[i].x + 1, predictions[i].y - 3), FONT_HERSHEY_PLAIN, 1, cvScalar(255, 255, 0), 1);
    }

	//show_all_points(image, image_width, image_height, crop_x, crop_y, crop_width, crop_height);
    //show_LIDAR_points(image, points, 255, 0, 0);
    //show_LIDAR(image, points_inside_bbox, 0, 0, 255);				 	// Blue points are all points inside the bbox
    //show_LIDAR(image, filtered_points, 0, 255, 0); 						// Green points are filtered points

    resize(image, image, Size(640, 480));
    imshow("Neural Object Detector", image);
    waitKey(1);
}


unsigned char *
crop_raw_image(int image_width, int image_height, unsigned char *raw_image, int displacement_x, int displacement_y, int crop_width, int crop_height)
{
	unsigned char *cropped_image = (unsigned char *) malloc (crop_width * crop_height * 3 * sizeof(unsigned char));  // Only works for 3 channels image

	displacement_x = (displacement_x - 2) * 3;
	displacement_y = (displacement_y - 2) * image_width * 3;
	crop_width     = displacement_x + ((crop_width + 1) * 3);
	crop_height    = displacement_y + ((crop_height + 1) * image_width * 3);
	image_height   = image_height * image_width * 3;
	image_width   *= 3;

	for (int line = 0, index = 0; line < image_height; line += image_width)
	{
		for (int column = 0; column < image_width; column += 3)
		{
			if (column > displacement_x && column < crop_width && line > displacement_y && line < crop_height)
			{
				cropped_image[index]     = raw_image[line + column];
				cropped_image[index + 1] = raw_image[line + column + 1];
				cropped_image[index + 2] = raw_image[line + column + 2];

				index += 3;
			}
		}
	}
	return (cropped_image);
}


vector<vector<image_cartesian>>
get_points_inside_bounding_boxes(vector<bbox_t> &predictions, vector<image_cartesian> &velodyne_points_vector)
{
	vector<vector<image_cartesian>> laser_list_inside_each_bounding_box; //each_bounding_box_laser_list

	for (unsigned int i = 0; i < predictions.size();)
	{
		vector<image_cartesian> lasers_points_inside_bounding_box;

		for (unsigned int j = 0; j < velodyne_points_vector.size(); j++)
		{
			if (velodyne_points_vector[j].image_x >  predictions[i].x &&
				velodyne_points_vector[j].image_x < (predictions[i].x + predictions[i].w) &&
				velodyne_points_vector[j].image_y >  predictions[i].y &&
				velodyne_points_vector[j].image_y < (predictions[i].y + predictions[i].h))
			{
				lasers_points_inside_bounding_box.push_back(velodyne_points_vector[j]);
			}
		}
		if (lasers_points_inside_bounding_box.size() > 0)
		{
			laser_list_inside_each_bounding_box.push_back(lasers_points_inside_bounding_box);
			i++;
		}
		else
		{
			predictions.erase(predictions.begin()+i);
		}

	}
	return laser_list_inside_each_bounding_box;
}


vector<image_cartesian>
get_biggest_cluster(vector<vector<image_cartesian>> &clusters)
{
	unsigned int max_size = 0, max_index = 0;

	for (unsigned int i = 0; i < clusters.size(); i++)
	{
		if (clusters[i].size() > max_size)
		{
			max_size = clusters[i].size();
			max_index = i;
		}
	}
	return (clusters[max_index]);
}


inline double
distance2(image_cartesian a, image_cartesian b)
{
	double dx = a.cartesian_x - b.cartesian_x;
	double dy = a.cartesian_y - b.cartesian_y;

	return (dx * dx + dy * dy);
}


vector<int>
query(double d2, int i, const vector<image_cartesian> &points, std::vector<bool> clustered)
{
	vector<int> neighbors;
	const image_cartesian &point = points[i];

	for (size_t j = 0; j < points.size(); j++)
	{
		if ((distance2(point, points[j]) < d2) && !clustered[j])
			neighbors.push_back(j);
	}

	return (neighbors);
}


vector<vector<image_cartesian>>
dbscan_compute_clusters(double d2, size_t density, const vector<image_cartesian> &points)
{
	vector<vector<image_cartesian>> clusters;
	vector<bool> clustered(points.size(), false);

	for (size_t i = 0; i < points.size(); ++i)
	{
		// Ignore already clustered points.
		if (clustered[i])
			continue;

		// Ignore points without enough neighbors.
		vector<int> neighbors = query(d2, i, points, clustered);
		if (neighbors.size() < density)
			continue;

		// Create a new cluster with the i-th point as its first element.
		vector<image_cartesian> c;
		clusters.push_back(c);
		vector<image_cartesian> &cluster = clusters.back();
		cluster.push_back(points[i]);
		clustered[i] = true;

		// Add the point's neighbors (and possibly their neighbors) to the cluster.
		for (size_t j = 0; j < neighbors.size(); ++j)
		{
			int k = neighbors[j];
			if (clustered[k])
				continue;

			cluster.push_back(points[k]);
			clustered[k] = true;

			vector<int> farther = query(d2, k, points, clustered);
			if (farther.size() >= density)
				neighbors.insert(neighbors.end(), farther.begin(), farther.end());
		}
	}
	return (clusters);
}


vector<vector<image_cartesian>>
filter_object_points_using_dbscan(vector<vector<image_cartesian>> &points_lists)
{
	vector<vector<image_cartesian>> filtered_points;

	for (unsigned int i = 0; i < points_lists.size(); i++)
	{
		vector<vector<image_cartesian>> clusters = dbscan_compute_clusters(0.5, 3, points_lists[i]);        // Compute clusters using dbscan

		if (clusters.size() == 0)                                          // If dbscan returned zero clusters
		{
			vector<image_cartesian> empty_cluster;
			filtered_points.push_back(empty_cluster);                      // An empty cluster is added to the clusters vector
		}
		else if (clusters.size() == 1)
		{
			filtered_points.push_back(clusters[0]);
		}
		else                                                               // dbscan returned more than one cluster
		{
			filtered_points.push_back(get_biggest_cluster(clusters));      // get the biggest, the biggest cluster will always better represent the object
		}
	}
	return (filtered_points);
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
compute_num_measured_objects(vector<image_cartesian> objects_poses)
{
	int num_objects = 0;

	for (int i = 0; i < objects_poses.size(); i++)
	{
		if (objects_poses[i].cartesian_x > 0.0 || objects_poses[i].cartesian_y > 0.0)
			num_objects++;
	}
	return (num_objects);
}


vector<bbox_t>
filter_predictions_traffic_light(vector<bbox_t> &predictions)
{
	vector<bbox_t> filtered_predictions;

	for (unsigned int i = 0; i < predictions.size(); i++)
	{
		if (predictions[i].obj_id == 9)    // traffic light
			filtered_predictions.push_back(predictions[i]);
	}
	return (filtered_predictions);
}


void
compute_annotation_specifications(vector<vector<image_cartesian>> traffic_light_clusters)
{
	double mean_x = 0.0, mean_y = 0.0, mean_z = 0.0;
	int i, j;

	for (i = 0; i < traffic_light_clusters.size(); i++)
	{
		for (j = 0; j < traffic_light_clusters[i].size(); j++)
		{
			mean_x += traffic_light_clusters[i][j].cartesian_x;
			mean_y += traffic_light_clusters[i][j].cartesian_y;
			mean_z += traffic_light_clusters[i][j].cartesian_z;
		}
		printf("TL %lf %lf %lf %lf Cluster Size %d\n", globalpos_msg->globalpos.theta, mean_x/j, mean_y/j, mean_z/j, (int)traffic_light_clusters[i].size());

		mean_x = 0.0;
		mean_y = 0.0;
		mean_z = 0.0;
	}
}


void
carmen_translte_3d(double *x, double *y, double *z, double offset_x, double offset_y, double offset_z)
{
	*x += offset_x;
	*y += offset_y;
	*z += offset_z;
}


void
generate_traffic_light_annotations(vector<bbox_t> predictions, vector<vector<image_cartesian>> points_inside_bbox)
{
	static vector<image_cartesian> traffic_light_points;
	static int count = 0;
	int traffic_light_found = 1;

	for (int i = 0; i < predictions.size(); i++)
	{
		for (int j = 0; j < points_inside_bbox[i].size(); j++)
		{
			carmen_translte_2d(&points_inside_bbox[i][j].cartesian_x, &points_inside_bbox[i][j].cartesian_y, board_pose.position.x, board_pose.position.y);
			carmen_rotate_2d  (&points_inside_bbox[i][j].cartesian_x, &points_inside_bbox[i][j].cartesian_y, globalpos_msg->globalpos.theta);
			carmen_translte_2d(&points_inside_bbox[i][j].cartesian_x, &points_inside_bbox[i][j].cartesian_y, globalpos_msg->globalpos.x, globalpos_msg->globalpos.y);

			points_inside_bbox[i][j].cartesian_z += board_pose.position.z + velodyne_pose.position.z;

			traffic_light_points.push_back(points_inside_bbox[i][j]);
		}
		count = 0;
		traffic_light_found = 0;
	}
	count += traffic_light_found;

	if (count > 8)                  // If stays without see a traffic light for more than N frames
	{                               // Compute traffic light positions and generate annotations
		vector<vector<image_cartesian>> traffic_light_clusters = dbscan_compute_clusters(0.5, 3, traffic_light_points);

		compute_annotation_specifications(traffic_light_clusters);
		traffic_light_points.clear();
		count = 0;
	}
	//printf("Cont %d\n", count);
}


void
compute_traffic_light_pose(vector<bbox_t> predictions, int resized_w, int resized_h, int crop_x, int crop_y, int crop_w, int crop_h)
{
	vector<image_cartesian> points = velodyne_camera_calibration_fuse_camera_lidar(velodyne_msg, camera_parameters, velodyne_pose, camera_pose,
			resized_w, resized_h, crop_x, crop_y, crop_w, crop_h);

	vector<vector<image_cartesian>> points_inside_bbox = get_points_inside_bounding_boxes(predictions, points); // TODO remover bbox que nao tenha nenhum ponto

	generate_traffic_light_annotations(predictions, points_inside_bbox);
}


double
compute_treshold()
{
	double dist = DIST2D(nearest_traffic_light_pose.position, globalpos_msg->globalpos);

	return (MIN_TRAFFIC_LIGHT_TRESHOLD + (MAX_TRAFFIC_LIGHT_TRESHOLD / dist));
}


carmen_traffic_light*
get_main_traffic_light(vector<bbox_t> predictions, carmen_position_t tf_annotation_on_image)
{
	carmen_traffic_light *main_traffic_light = (carmen_traffic_light *) malloc (1 * sizeof(carmen_traffic_light));
	bbox_t main_bbox;
	double dist = 0.0, main_dist = DBL_MAX, x_centroid = 0.0, y_centroid = 0.0;

	for (unsigned int i = 0; i < predictions.size(); i++)
	{
		x_centroid = predictions[i].x + (predictions[i].w / 2);
		y_centroid = predictions[i].y + (predictions[i].h / 2);

		x_centroid = x_centroid - tf_annotation_on_image.x;
		y_centroid = y_centroid - tf_annotation_on_image.y;

		dist = sqrt((x_centroid * x_centroid) + (y_centroid * y_centroid));

		if (dist < main_dist)
		{
			main_dist = dist;
			main_bbox = predictions[i];
		}
	}
	main_traffic_light->x1 = main_bbox.x;
	main_traffic_light->y1 = main_bbox.y;
	main_traffic_light->x2 = main_bbox.x + main_bbox.w;
	main_traffic_light->y2 = main_bbox.y + main_bbox.h;
	main_traffic_light->color = RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_OFF;                    // In case of any failure, TRAFFIC_LIGHT_OFF message is sent

	if (dist < compute_treshold())                                                         //RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_RED
	{                                                                                      //RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_GREEN
		if (main_bbox.obj_id == 0)                                                         //RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_YELLOW
			main_traffic_light->color = RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_RED;            //RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_OFF
		else if (main_bbox.obj_id == 1)                                                    //RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_TURN_RIGHT
			main_traffic_light->color = RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_GREEN;          //RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_TURN_LEFT
	}

	return (main_traffic_light);
}


carmen_traffic_light_message
build_traffic_light_message(carmen_bumblebee_basic_stereoimage_message *image_msg, vector<bbox_t> predictions, carmen_position_t tf_annotation_on_image)
{
	carmen_traffic_light_message traffic_light_message;

	traffic_light_message.num_traffic_lights = 1;
	traffic_light_message.traffic_lights = get_main_traffic_light(predictions, tf_annotation_on_image);
	traffic_light_message.traffic_light_annotation_distance = 9999.0;
	traffic_light_message.timestamp = image_msg->timestamp;
	traffic_light_message.host = carmen_get_host();

	return (traffic_light_message);
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
publish_moving_objects_message(double timestamp, carmen_moving_objects_point_clouds_message *msg)
{
	msg->timestamp = timestamp;
	msg->host = carmen_get_host();

    carmen_moving_objects_point_clouds_publish_message(msg);
}


static void
publish_traffic_lights(carmen_traffic_light_message *traffic_light_message)
{
    carmen_traffic_light_publish_message(camera, traffic_light_message);
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////



void
image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	if (image_msg == NULL)
		return;

	double fps;
	static double start_time = 0.0;
	unsigned char *img;

	if (camera_side == 0)
		img = image_msg->raw_left;
	else
		img = image_msg->raw_right;

	int resized_w = image_msg->width;
	int resized_h = image_msg->height;
	int crop_x = 0;
	int crop_y = 0;
	int crop_w = resized_w;
	int crop_h = resized_h;

//	unsigned char *cropped_img = crop_raw_image(image_msg->width, image_msg->height, img, crop_x, crop_y, crop_w, crop_h);
	Mat open_cv_image = Mat(image_msg->height, image_msg->width, CV_8UC3, img, 0);
	resize(open_cv_image, open_cv_image, Size(resized_w, resized_h));

	vector<bbox_t> predictions = run_YOLO(open_cv_image.data, open_cv_image.cols, open_cv_image.rows, network_struct, classes_names, 0.2);
		//predictions = filter_predictions_traffic_light(predictions);

	compute_traffic_light_pose(predictions, resized_w, resized_h, crop_x, crop_y, crop_w, crop_h);

	fps = 1.0 / (carmen_get_time() - start_time);
	start_time = carmen_get_time();

	double distance_to_tf = compute_distance_to_the_traffic_light();

	if (distance_to_tf < DBL_MAX)
	{
		if (predictions.size() > 0)
			printf("Distance %lf\n", distance_to_tf);
		else
			printf("No Traffic Light Detected!\n");

		tf::StampedTransform world_to_camera_pose = get_world_to_camera_transformer(&transformer, globalpos_msg->globalpos.x, globalpos_msg->globalpos.y, 0.0,
				0.0, 0.0, globalpos_msg->globalpos.theta);

		carmen_position_t tf_annotation_on_image = convert_rddf_pose_to_point_in_image(nearest_traffic_light_pose.position.x, nearest_traffic_light_pose.position.y, nearest_traffic_light_pose.position.z,
				world_to_camera_pose, camera_parameters, resized_w, resized_h);

		if (predictions.size() > 0)
		{
			carmen_traffic_light_message traffic_light_message = build_traffic_light_message(image_msg, predictions, tf_annotation_on_image);
			publish_traffic_lights(&traffic_light_message);
		}
		printf("----%lf\n", compute_treshold());
		circle(open_cv_image, Point((int)tf_annotation_on_image.x, (int)tf_annotation_on_image.y), 5.0, Scalar(255, 255, 0), -1, 8);
		circle(open_cv_image, Point((int)tf_annotation_on_image.x, (int)tf_annotation_on_image.y), compute_treshold(), Scalar(255, 255, 0), 1, 8);
	}
	display(open_cv_image, predictions, fps, resized_w, resized_h);
}


void
velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
	velodyne_msg = velodyne_message;

	carmen_velodyne_camera_calibration_arrange_velodyne_vertical_angles_to_true_position(velodyne_msg);
}


void
localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *globalpos_message)
{
	globalpos_msg = globalpos_message;
}


void
shutdown_module(int signo)
{
    if (signo == SIGINT) {
        carmen_ipc_disconnect();
        cvDestroyAllWindows();

        printf("Neural Object Detector: Disconnected.\n");
        exit(0);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////


void
subscribe_messages()
{
    carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) image_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t) velodyne_partial_scan_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


void
load_traffic_light_annotation_file(char* path)
{
	carmen_pose_3D_t pose;
	FILE *annotation_file = fopen(path,"r");

	int annotation_type, annotation_code;
	char line[1024], annotation_description[1024];

	while (fgets(line, 1023, annotation_file) != NULL)
	{
		sscanf(line, "%s %d %d %lf %lf %lf %lf\n", annotation_description, &annotation_type, &annotation_code,
						&pose.orientation.yaw, &pose.position.x, &pose.position.y, &pose.position.z);

		if (strcmp("RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT", annotation_description) == 0)
		{
			printf("%lf %lf %lf %lf\n", pose.orientation.yaw, pose.position.x, pose.position.y, pose.position.z);
			pose.orientation.roll = 0.0;
			pose.orientation.pitch = 0.0;
			annotation_vector.push_back(pose);
		}
	}

	fclose(annotation_file);
}


void
read_parameters(int argc, char **argv)
{
	if ((argc != 4))
		carmen_die("%s: Wrong number of parameters. neural_object_detector requires 3 parameter and received %d. \n Usage: %s <camera_number> <camera_side(0-left; 1-right)> <rddf_file_path>\n", argv[0], argc - 1, argv[0]);

	camera = atoi(argv[1]);             // Define the camera to be used
    camera_side = atoi(argv[2]);        // 0 For left image 1 for right image

    load_traffic_light_annotation_file(argv[3]);

    int num_items;

    char bumblebee_string[256];
    char camera_string[256];

    sprintf(bumblebee_string, "%s%d", "bumblebee_basic", camera); // Geather the cameri ID
    sprintf(camera_string, "%s%d", "camera", camera);

    carmen_param_t param_list[] =
    {
		{bumblebee_string, (char*) "fx", CARMEN_PARAM_DOUBLE, &camera_parameters.fx_factor, 0, NULL },
		{bumblebee_string, (char*) "fy", CARMEN_PARAM_DOUBLE, &camera_parameters.fy_factor, 0, NULL },
		{bumblebee_string, (char*) "cu", CARMEN_PARAM_DOUBLE, &camera_parameters.cu_factor, 0, NULL },
		{bumblebee_string, (char*) "cv", CARMEN_PARAM_DOUBLE, &camera_parameters.cv_factor, 0, NULL },
		{bumblebee_string, (char*) "pixel_size", CARMEN_PARAM_DOUBLE, &camera_parameters.pixel_size, 0, NULL },

		{camera_string, (char*) "x",     CARMEN_PARAM_DOUBLE, &camera_pose.position.x, 0, NULL },
		{camera_string, (char*) "y",     CARMEN_PARAM_DOUBLE, &camera_pose.position.y, 0, NULL },
		{camera_string, (char*) "z",     CARMEN_PARAM_DOUBLE, &camera_pose.position.z, 0, NULL },
		{camera_string, (char*) "roll",  CARMEN_PARAM_DOUBLE, &camera_pose.orientation.roll, 0, NULL },
		{camera_string, (char*) "pitch", CARMEN_PARAM_DOUBLE, &camera_pose.orientation.pitch, 0, NULL },
		{camera_string, (char*) "yaw",   CARMEN_PARAM_DOUBLE, &camera_pose.orientation.yaw, 0, NULL },

		{(char *) "velodyne", (char *) "x",     CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.x), 0, NULL},
		{(char *) "velodyne", (char *) "y",     CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.y), 0, NULL},
		{(char *) "velodyne", (char *) "z",     CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.z), 0, NULL},
		{(char *) "velodyne", (char *) "roll",  CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.roll), 0, NULL},
		{(char *) "velodyne", (char *) "pitch", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.pitch), 0, NULL},
		{(char *) "velodyne", (char *) "yaw",   CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.yaw), 0, NULL},

		{(char *) "sensor_board_1", (char*) "x",     CARMEN_PARAM_DOUBLE, &board_pose.position.x, 0, NULL },
		{(char *) "sensor_board_1", (char*) "y",     CARMEN_PARAM_DOUBLE, &board_pose.position.y, 0, NULL },
		{(char *) "sensor_board_1", (char*) "z",     CARMEN_PARAM_DOUBLE, &board_pose.position.z, 0, NULL },
		{(char *) "sensor_board_1", (char*) "roll",  CARMEN_PARAM_DOUBLE, &board_pose.orientation.roll, 0, NULL },
		{(char *) "sensor_board_1", (char*) "pitch", CARMEN_PARAM_DOUBLE, &board_pose.orientation.pitch, 0, NULL },
		{(char *) "sensor_board_1", (char*) "yaw",   CARMEN_PARAM_DOUBLE, &board_pose.orientation.yaw, 0, NULL }
    };

    num_items = sizeof(param_list) / sizeof(param_list[0]);
    carmen_param_install_params(argc, argv, param_list, num_items);
}


void
initializer()
{
    initialize_transformations(board_pose, camera_pose, &transformer);

    classes_names = get_classes_names((char*) "../../sharedlib/darknet2/data/traffic_light.names");

	network_struct = initialize_YOLO((char*) "../../sharedlib/darknet2/cfg/traffic_light.cfg", (char*) "../../sharedlib/darknet2/yolov3_traffic_light_rgo.weights");
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);

	read_parameters(argc, argv);

	subscribe_messages();

	signal(SIGINT, shutdown_module);

	initializer();

	setlocale(LC_ALL, "C");

	carmen_ipc_dispatch();

	return 0;
}
