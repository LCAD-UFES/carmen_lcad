#include "neural_object_detector.hpp"

int camera;
int camera_side;

char* camera_model;    // Camera model in case of using CAMERA_MESSAGE
int image_index = 0;   // camera_message may contain several images from same camera
int message_number = -1;   // Number of the camera message
bool is_intelbras = false;
int camera_width;
int camera_height;
double resize_factor = 1.0; // Factor of resize
int crop_x = 0;       // Crop starting point
int crop_y = 0;
int crop_w  = -1;     // Width of crop
int crop_h = -1;     // Heigth of crop
int original_img_width = -1;
int original_img_height = -1;
int calibrate_camera_lidar_aligment = 0;
int detect_always = 0;
int show_output = 1;

char **classes_names;
void *network_struct;
carmen_localize_ackerman_globalpos_message *globalpos_msg;
carmen_velodyne_partial_scan_message *velodyne_msg;
carmen_camera_parameters camera_parameters;
carmen_pose_3D_t velodyne_pose;
carmen_pose_3D_t camera_pose;
carmen_pose_3D_t board_pose;
tf::Transformer transformer;

//carmen_bumblebee_basic_stereoimage_message *image_msg = NULL;
camera_message *image_msg = NULL;


void
carmen_translte_2d(double *x, double *y, double offset_x, double offset_y)
{
	*x += offset_x;
	*y += offset_y;
}
void  Initialize ()
{ 
    Py_Initialize();
}

void Finalize ()
{
    Py_Finalize();
}

string
runPython(Mat filename)
{
	std::string my_str = "[";
	std::string ret;


    for(int i = 0; i < filename.rows; i++)
    {
		my_str += "[";

       	for(int j = 0; j < filename.cols; j++)
			my_str += "[" + to_string(filename.at<cv::Vec3b>(i,j)[0]) +
					  ", " + to_string(filename.at<cv::Vec3b>(i,j)[1]) + 
					  ", " + to_string(filename.at<cv::Vec3b>(i,j)[2]) +
					  (j + 1 < filename.cols ? "], " : "]");

		my_str += (i + 1 < filename.rows ? "],\n" : "]");
    }

	my_str += "]";
	// ofstream MyFile("aaaaaaaaaaaaaaaaaa.txt", std::ios_base::app);
	// MyFile << "oi" << endl;

	PyObject *pName, *pModule, *pFunc;
    PyObject *pArgs, *pValue;
    int i;
	static int asd;

    setenv("PYTHONPATH", "../src/neural_moving_objects_detector", 1);
	// MyFile << "passou setenv" << endl;
	// Initialize();
	if (asd == 0){
		++asd;
		Initialize();
        atexit(Finalize);
	}
	pName = PyUnicode_DecodeFSDefault("demo");
	// MyFile << "passou PyUnicode_DecodeFSDefault" << endl;
	/* Error checking of pName left out */

	pModule = PyImport_Import(pName); 
	// MyFile << "passou PyImport_Import" << endl;
	Py_DECREF(pName);
	// MyFile << "passou Py_DECREF(pName)" << endl;

	if (pModule != NULL) {
		pFunc = PyObject_GetAttrString(pModule, "main");
		// MyFile << "passou PyObject_GetAttrString" << endl;

		/* pFunc is a new reference */

		if (pFunc && PyCallable_Check(pFunc)) {
			// MyFile << "passou PyCallable_Check" << endl;
			pArgs = PyTuple_New(1);
			// MyFile << "passou PyTuple_New" << endl;
			pValue = PyUnicode_DecodeFSDefault(my_str.c_str());
			// MyFile << "passou PyUnicode_DecodeFSDefault" << endl;
			PyTuple_SetItem(pArgs, 0, pValue);
			// MyFile << "passou PyTuple_SetItem" << endl;
			pValue = PyObject_CallObject(pFunc, pArgs);// <--------------------------------------------- demora uma vida
			// MyFile << "passou PyObject_CallObject" << endl;
			Py_DECREF(pArgs);
			// MyFile << "passou Py_DECREF" << endl;
			if (pValue != NULL) {
				Py_ssize_t len;
				ret = PyUnicode_AsUTF8AndSize(pValue, &len);
				// MyFile << ret << endl;
				Py_DECREF(pValue);
				// MyFile << "passou Py_DECREF" << endl;
			}
		}
		Py_XDECREF(pFunc);
		// MyFile << "passou Py_XDECREF" << endl;
		Py_DECREF(pModule);
		// MyFile << "passou Py_DECREF" << endl;
	}
	// atexit(Finalize);
	// std::vector<Byte> vectordata(ret.begin(),ret.end());
	// cv::Mat data_mat(vectordata,true);
	// cv::Mat image(cv::imdecode(data_mat,1));
	return ret;

}

void
display(Mat image, vector<bbox_t> predictions, vector<image_cartesian> points, vector<vector<image_cartesian>> points_inside_bbox,
		vector<vector<image_cartesian>> filtered_points, double fps, unsigned int image_width, unsigned int image_height)
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
    //show_LIDAR(image, points_inside_bbox,    0, 0, 255);				// Blue points are all points inside the bbox
    //show_LIDAR(image, filtered_points, 0, 255, 0); 						// Green points are filtered points

    //resize(image, image, Size(600, 300));
    imshow("Neural Object Detector", image);
    //imwrite("Image.jpg", image);
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

	//cout << predictions.size() << endl;

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
			//cout << predictions.size() << endl;
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


carmen_moving_objects_point_clouds_message
build_detected_objects_message(vector<bbox_t> predictions, vector<image_cartesian> objects_poses, vector<vector<image_cartesian>> points_lists)
{
	carmen_moving_objects_point_clouds_message msg;
	int num_objects = compute_num_measured_objects(objects_poses);

	//printf ("Predictions %d Poses %d, Points %d\n", (int) predictions.size(), (int) objects_poses.size(), (int) points_lists.size());

	msg.num_point_clouds = num_objects;
	msg.point_clouds = (t_point_cloud_struct *) malloc (num_objects * sizeof(t_point_cloud_struct));

	for (int i = 0, l = 0; i < objects_poses.size(); i++)
	{                                                                                                               // The error code of -999.0 is set on compute_detected_objects_poses,
		if (objects_poses[i].cartesian_x != -999.0 || objects_poses[i].cartesian_y != -999.0)                       // probably the object is out of the LiDAR's range
		{
			carmen_translte_2d(&objects_poses[i].cartesian_x, &objects_poses[i].cartesian_y, board_pose.position.x, board_pose.position.y);
			carmen_rotate_2d  (&objects_poses[i].cartesian_x, &objects_poses[i].cartesian_y, carmen_normalize_theta(globalpos_msg->globalpos.theta));
			carmen_translte_2d(&objects_poses[i].cartesian_x, &objects_poses[i].cartesian_y, globalpos_msg->globalpos.x, globalpos_msg->globalpos.y);

			msg.point_clouds[l].r = 1.0;
			msg.point_clouds[l].g = 1.0;
			msg.point_clouds[l].b = 0.0;

			msg.point_clouds[l].linear_velocity = 0;
			msg.point_clouds[l].orientation = globalpos_msg->globalpos.theta;

			msg.point_clouds[l].length = 4.5;
			msg.point_clouds[l].height = 1.8;
			msg.point_clouds[l].width  = 1.6;

			msg.point_clouds[l].object_pose.x = objects_poses[i].cartesian_x;
			msg.point_clouds[l].object_pose.y = objects_poses[i].cartesian_y;
			msg.point_clouds[l].object_pose.z = 0.0;

			switch (predictions[i].obj_id)
			{
				case 0:
					msg.point_clouds[l].geometric_model = 0;
					msg.point_clouds[l].model_features.geometry.height = 1.8;
					msg.point_clouds[l].model_features.geometry.length = 1.0;
					msg.point_clouds[l].model_features.geometry.width = 1.0;
					msg.point_clouds[l].model_features.red = 1.0;
					msg.point_clouds[l].model_features.green = 1.0;
					msg.point_clouds[l].model_features.blue = 0.8;
					msg.point_clouds[l].model_features.model_name = (char *) "pedestrian";
					break;
				case 2:
					msg.point_clouds[l].geometric_model = 0;
					msg.point_clouds[l].model_features.geometry.height = 1.8;
					msg.point_clouds[l].model_features.geometry.length = 4.5;
					msg.point_clouds[l].model_features.geometry.width = 1.6;
					msg.point_clouds[l].model_features.red = 1.0;
					msg.point_clouds[l].model_features.green = 0.0;
					msg.point_clouds[l].model_features.blue = 0.8;
					msg.point_clouds[l].model_features.model_name = (char *) "car";
					break;
				default:
					msg.point_clouds[l].geometric_model = 0;
					msg.point_clouds[l].model_features.geometry.height = 1.8;
					msg.point_clouds[l].model_features.geometry.length = 4.5;
					msg.point_clouds[l].model_features.geometry.width = 1.6;
					msg.point_clouds[l].model_features.red = 1.0;
					msg.point_clouds[l].model_features.green = 1.0;
					msg.point_clouds[l].model_features.blue = 0.0;
					msg.point_clouds[l].model_features.model_name = (char *) "other";
					break;
			}
			msg.point_clouds[l].num_associated = 0;

			msg.point_clouds[l].point_size = points_lists[i].size();

			msg.point_clouds[l].points = (carmen_vector_3D_t *) malloc (msg.point_clouds[l].point_size * sizeof(carmen_vector_3D_t));

			for (int j = 0; j < points_lists[i].size(); j++)
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
			predictions[i].obj_id == 7 ||  // truck
		    predictions[i].obj_id == 9)    // traffic light
		{
			filtered_predictions.push_back(predictions[i]);
		}
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
			//printf("%lf %lf %lf\n", traffic_light_clusters[i][j].cartesian_x, traffic_light_clusters[i][j].cartesian_y, traffic_light_clusters[i][j].cartesian_z);

			mean_x += traffic_light_clusters[i][j].cartesian_x;
			mean_y += traffic_light_clusters[i][j].cartesian_y;
			mean_z += traffic_light_clusters[i][j].cartesian_z;
		}
		printf("TL %lf %lf %lf\n", mean_x/j, mean_y/j, mean_z/j);

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
		if (predictions[i].obj_id == 9)
		{
			//printf("%s\n", obj_names_vector[predictions[i].obj_id].c_str());
			for (int j = 0; j < points_inside_bbox[i].size(); j++)
			{
				//printf("%lf %lf\n", points_inside_bbox[i][j].cartesian_x, points_inside_bbox[i][j].cartesian_y);

				//carmen_translte_3d(&points_inside_bbox[i][j].cartesian_x, &points_inside_bbox[i][j].cartesian_y, &points_inside_bbox[i][j].cartesian_z, board_x, board_y, board_pose.position.z);
				carmen_translte_2d(&points_inside_bbox[i][j].cartesian_x, &points_inside_bbox[i][j].cartesian_y, board_pose.position.x, board_pose.position.y);
				carmen_rotate_2d  (&points_inside_bbox[i][j].cartesian_x, &points_inside_bbox[i][j].cartesian_y, globalpos_msg->globalpos.theta);
				carmen_translte_2d(&points_inside_bbox[i][j].cartesian_x, &points_inside_bbox[i][j].cartesian_y, globalpos_msg->globalpos.x, globalpos_msg->globalpos.y);

				traffic_light_points.push_back(points_inside_bbox[i][j]);
				//printf("%lf %lf\n", points_inside_bbox[i][j].cartesian_x, points_inside_bbox[i][j].cartesian_y);
			}
			count = 0;
			traffic_light_found = 0;
		}
	}
	count += traffic_light_found;

	if (count >= 20)                // If stays without see a traffic light for more than 20 frames
	{                               // Compute traffic light positions and generate annotations
		vector<vector<image_cartesian>> traffic_light_clusters = dbscan_compute_clusters(0.5, 3, traffic_light_points);
		//printf("--- %d\n", (int)traffic_light_clusters.size());
		compute_annotation_specifications(traffic_light_clusters);
		traffic_light_points.clear();
		count = 0;
	}
	//printf("Cont %d\n", count);
}


void
call_neural_network()
{
	if (image_msg == NULL)
		return;

	int crop_w = image_msg->images[0].width;// 1280;
	int crop_h = image_msg->images[0].height;//400; // 500;
	double timestamp = image_msg->timestamp;
	double fps;
	static double start_time = 0.0;
	unsigned char *img = (unsigned char *) malloc(3 * crop_w * crop_h);

	memcpy(img, image_msg->images[0].raw_data, 3 * crop_w * crop_h);

	Mat image = Mat(crop_h, crop_w, CV_8UC3, img, 0);
    cvtColor(image, image, COLOR_RGB2BGR);

	fps = 1.0 / (carmen_get_time() - start_time);
	start_time = carmen_get_time();
	printf("FPS= %.2f\n", fps);
	//cv::imwrite("../src/neural_moving_objects_detector/YOLOPv2/data/example.jpg", image);
	ofstream MyFile("aaaaaaaaaaaaaaaaaa.txt", std::ios_base::app);
	runPython(image);
	// MyFile << "------------------------FIM DO RUN PYTHON---------------------------" << endl;
	image = cv::imread("../src/neural_moving_objects_detector/YOLOPv2/runs/detect/exp/example.jpg", 1);

	//	vector<bbox_t> predictions = run_YOLO(img, crop_w, crop_h, network_struct, classes_names, 0.5);
//	predictions = filter_predictions_of_interest(predictions);
//
//	vector<image_cartesian> points = velodyne_camera_calibration_fuse_camera_lidar(velodyne_msg, camera_parameters, velodyne_pose, camera_pose,
//			crop_w, crop_h, crop_x, crop_y, crop_w, crop_h);
//
//	vector<vector<image_cartesian>> points_inside_bbox = get_points_inside_bounding_boxes(predictions, points); // TODO remover bbox que nao tenha nenhum ponto
//
//	vector<vector<image_cartesian>> filtered_points = filter_object_points_using_dbscan(points_inside_bbox);
//
//	vector<image_cartesian> positions = compute_detected_objects_poses(filtered_points);
//
//	carmen_moving_objects_point_clouds_message msg = build_detected_objects_message(predictions, positions, filtered_points);
//
//	publish_moving_objects_message(timestamp, &msg);
//
//	if (globalpos_msg == NULL)
//		return;
//
//	tf::StampedTransform world_to_camera_pose = get_world_to_camera_transformer(&transformer, globalpos_msg->globalpos.x, globalpos_msg->globalpos.y, 0.0,
//			globalpos_msg->pose.orientation.roll, globalpos_msg->pose.orientation.pitch, globalpos_msg->pose.orientation.yaw);
//
//	carmen_position_t object_on_image = convert_rddf_pose_to_point_in_image(7757493.704663, -364151.945918, 5.428209,
//			world_to_camera_pose, camera_parameters, crop_w, crop_h);
//
//	circle(image, Point((int)object_on_image.x, (int)object_on_image.y), 5.0, Scalar(255, 255, 0), -1, 8);

//	display(image, predictions, points, points_inside_bbox, filtered_points, fps, crop_w, crop_h);
    if(image.data != NULL){
    	imshow( "Display window", image );
    	waitKey(200);
	}

    free(img);
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


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
camera_image_handler(camera_message *msg)
{
	image_msg = msg;
	call_neural_network();
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
    if (signo == SIGINT)
    {
        carmen_ipc_disconnect();
        cvDestroyAllWindows();

        printf("Neural Object Detector: Disconnected.\n");
        exit(0);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////


//void
//subscribe_messages_old()
//{
////    carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) image_handler, CARMEN_SUBSCRIBE_LATEST);
//
//	camera_drivers_subscribe_message(camera, NULL, (carmen_handler_t) camera_image_handler, CARMEN_SUBSCRIBE_LATEST);
//
//	carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t) velodyne_partial_scan_message_handler, CARMEN_SUBSCRIBE_LATEST);
//
//    carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);
//}
//
//
//void
//read_parameters_old(int argc, char **argv)
//{
//	if ((argc != 2))
//		carmen_die("%s: Wrong number of parameters. neural_moving_objects_detector requires 1 parameter and received %d. \n Usage: %s <camera_number>\n>", argv[0], argc - 1, argv[0]);
//
//	camera = atoi(argv[1]);             // Define the camera to be used
//
//    int num_items;
//
//    char camera_string[256];
//
//    sprintf(camera_string, "%s%d", "camera", camera);
//
//    carmen_param_t param_list[] =
//    {
//		{bumblebee_string, (char*) "fx", CARMEN_PARAM_DOUBLE, &camera_parameters.fx_factor, 0, NULL },
//		{bumblebee_string, (char*) "fy", CARMEN_PARAM_DOUBLE, &camera_parameters.fy_factor, 0, NULL },
//		{bumblebee_string, (char*) "cu", CARMEN_PARAM_DOUBLE, &camera_parameters.cu_factor, 0, NULL },
//		{bumblebee_string, (char*) "cv", CARMEN_PARAM_DOUBLE, &camera_parameters.cv_factor, 0, NULL },
//		{bumblebee_string, (char*) "pixel_size", CARMEN_PARAM_DOUBLE, &camera_parameters.pixel_size, 0, NULL },
//
//		{camera_string, (char*) "x",     CARMEN_PARAM_DOUBLE, &camera_pose.position.x, 1, NULL },
//		{camera_string, (char*) "y",     CARMEN_PARAM_DOUBLE, &camera_pose.position.y, 1, NULL },
//		{camera_string, (char*) "z",     CARMEN_PARAM_DOUBLE, &camera_pose.position.z, 1, NULL },
//		{camera_string, (char*) "roll",  CARMEN_PARAM_DOUBLE, &camera_pose.orientation.roll, 1, NULL },
//		{camera_string, (char*) "pitch", CARMEN_PARAM_DOUBLE, &camera_pose.orientation.pitch, 1, NULL },
//		{camera_string, (char*) "yaw",   CARMEN_PARAM_DOUBLE, &camera_pose.orientation.yaw, 1, NULL },
//
//		{(char *) "velodyne", (char *) "x",     CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.x), 0, NULL},
//		{(char *) "velodyne", (char *) "y",     CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.y), 0, NULL},
//		{(char *) "velodyne", (char *) "z",     CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.z), 0, NULL},
//		{(char *) "velodyne", (char *) "roll",  CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.roll), 0, NULL},
//		{(char *) "velodyne", (char *) "pitch", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.pitch), 0, NULL},
//		{(char *) "velodyne", (char *) "yaw",   CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.yaw), 0, NULL},
//
//		{(char *) "sensor_board_1", (char*) "x",     CARMEN_PARAM_DOUBLE, &board_pose.position.x, 0, NULL },
//		{(char *) "sensor_board_1", (char*) "y",     CARMEN_PARAM_DOUBLE, &board_pose.position.y, 0, NULL },
//		{(char *) "sensor_board_1", (char*) "z",     CARMEN_PARAM_DOUBLE, &board_pose.position.z, 0, NULL },
//		{(char *) "sensor_board_1", (char*) "roll",  CARMEN_PARAM_DOUBLE, &board_pose.orientation.roll, 0, NULL },
//		{(char *) "sensor_board_1", (char*) "pitch", CARMEN_PARAM_DOUBLE, &board_pose.orientation.pitch, 0, NULL },
//		{(char *) "sensor_board_1", (char*) "yaw",   CARMEN_PARAM_DOUBLE, &board_pose.orientation.yaw, 0, NULL }
//    };
//
//    num_items = sizeof(param_list) / sizeof(param_list[0]);
//    carmen_param_install_params(argc, argv, param_list, num_items);
//}


void
read_parameters(int argc, char **argv)
{
	int parameter_number = 2;
	if ((argc < parameter_number))
		carmen_die("%s: Wrong number of parameters. %s requires at least %d parameters and received %d.\n"
				"   Usage: %s <camera_model> <message_number> -calibrate_camera <0 or 1>(optional)\n>",
				argv[0], argv[0], parameter_number, argc - 1, argv[0]);

	camera_model = argv[1];
	message_number = atoi(argv[2]);
	string camera_model_string(camera_model);

	if (camera_model_string.find("intelbras") != string::npos)
	{
		is_intelbras = true;
		carmen_param_t param_list[] =
		{
			{camera_model, (char *) "width_1",  CARMEN_PARAM_INT, &camera_width, 0, NULL},
			{camera_model, (char *) "height_1", CARMEN_PARAM_INT, &camera_height, 0, NULL},
			{camera_model, (char *) "fx", CARMEN_PARAM_DOUBLE, &camera_parameters.fx_factor, 0, NULL},
			{camera_model, (char *) "fy", CARMEN_PARAM_DOUBLE, &camera_parameters.fy_factor, 0, NULL},
			{camera_model, (char *) "cu", CARMEN_PARAM_DOUBLE, &camera_parameters.cu_factor, 0, NULL},
			{camera_model, (char *) "cv", CARMEN_PARAM_DOUBLE, &camera_parameters.cv_factor, 0, NULL},
			{camera_model, (char *) "pixel_size", CARMEN_PARAM_DOUBLE, &camera_parameters.pixel_size, 0, NULL},
			{camera_model, (char *) "x",     CARMEN_PARAM_DOUBLE, &camera_pose.position.x, 1, NULL},
			{camera_model, (char *) "y",     CARMEN_PARAM_DOUBLE, &camera_pose.position.y, 1, NULL},
			{camera_model, (char *) "z",     CARMEN_PARAM_DOUBLE, &camera_pose.position.z, 1, NULL},
			{camera_model, (char *) "roll",  CARMEN_PARAM_DOUBLE, &camera_pose.orientation.roll, 1, NULL},
			{camera_model, (char *) "pitch", CARMEN_PARAM_DOUBLE, &camera_pose.orientation.pitch, 1, NULL},
			{camera_model, (char *) "yaw",   CARMEN_PARAM_DOUBLE, &camera_pose.orientation.yaw, 1, NULL},

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
	}
	else
	{
		camera = atoi(argv[1]);             // Define the camera to be used
   		camera_side = atoi(argv[2]);        // 0 For left image 1 for right image
		char bumblebee_string[256];
		char camera_string[256];
		char bullbar_string[256];
		char sick_string[256];

		sprintf(bumblebee_string, "%s%d", "bumblebee_basic", camera); // Geather the cameri ID
		sprintf(camera_string, "%s%d", "camera", camera);
		sprintf(bullbar_string, "%s", "front_bullbar");
		sprintf(sick_string, "%s", "laser_ldmrs");

		carmen_param_t param_list[] =
		{
			{bumblebee_string, (char*) "width",  CARMEN_PARAM_INT, &camera_width, 0, NULL},
			{bumblebee_string, (char*) "height", CARMEN_PARAM_INT, &camera_height, 0, NULL},
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
			{(char *) "sensor_board_1", (char*) "yaw",   CARMEN_PARAM_DOUBLE, &board_pose.orientation.yaw, 0, NULL },

//			{bullbar_string, (char*) "x",     CARMEN_PARAM_DOUBLE, &bullbar_pose.position.x, 0, NULL },
//			{bullbar_string, (char*) "y",     CARMEN_PARAM_DOUBLE, &bullbar_pose.position.y, 0, NULL },
//			{bullbar_string, (char*) "z",     CARMEN_PARAM_DOUBLE, &bullbar_pose.position.z, 0, NULL },
//			{bullbar_string, (char*) "roll",  CARMEN_PARAM_DOUBLE, &bullbar_pose.orientation.roll, 0, NULL },
//			{bullbar_string, (char*) "pitch", CARMEN_PARAM_DOUBLE, &bullbar_pose.orientation.pitch, 0, NULL },
//			{bullbar_string, (char*) "yaw",   CARMEN_PARAM_DOUBLE, &bullbar_pose.orientation.yaw, 0, NULL },
		};
		carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
		cout<<"width "<<camera_width<<" height "<<camera_height<<endl;

	}

 	carmen_param_allow_unfound_variables(1);
   	carmen_param_t optional_commandline_param_list[] =
   	{
   		{(char *) "commandline", (char *) "image", CARMEN_PARAM_INT, &image_index, 0, NULL},
		{(char *) "commandline", (char *) "resize", CARMEN_PARAM_DOUBLE, &resize_factor, 0, NULL},
   		{(char *) "commandline", (char *) "cropx",  CARMEN_PARAM_INT, &crop_x, 0, NULL},
   		{(char *) "commandline", (char *) "cropy",  CARMEN_PARAM_INT, &crop_y, 0, NULL},
   		{(char *) "commandline", (char *) "cropw",  CARMEN_PARAM_INT, &crop_w, 0, NULL},
   		{(char *) "commandline", (char *) "croph", CARMEN_PARAM_INT, &crop_h, 0, NULL},
		{(char *) "commandline", (char *) "calibrate_camera", CARMEN_PARAM_INT, &calibrate_camera_lidar_aligment, 0, NULL},
		{(char *) "commandline", (char *) "detect_always", CARMEN_PARAM_INT, &detect_always, 0, NULL},
		{(char *) "commandline", (char *) "show", CARMEN_PARAM_ONOFF, &show_output, 0, NULL},
   	};
	carmen_param_install_params(argc, argv, optional_commandline_param_list, sizeof(optional_commandline_param_list) / sizeof(optional_commandline_param_list[0]));
	// printf("crops: %d %d %d %d %d\n", crop_x, crop_y, crop_w, crop_h, __LINE__);
//	printf("detect_always: %d\n", detect_always);

//   	for (int i = 0; i < argc; i++) // De acordo com o que pensei, i deveria iniciar igual a 3. Mas se o valor não for =0, o primeiro elemento do lidars_alive não ativa. Não entendi a razão.
//	{
//		if (strcmp(argv[i], "-lidar") == 0 && i < argc - 1 && argv[i + 1][0] != '-')
//		{
//			if (atoi(argv[i + 1]) < MAX_NUMBER_OF_LIDARS_NEURAL)
//				lidars_alive[atoi(argv[i + 1])] = true;
//		}
//
//	   if (strcmp(argv[i], "-ouster64") == 0)
//	   {
//			lidars_alive[0] = true;
//			lidars_alive[1] = true;
//			lidars_alive[2] = true;
//			lidars_alive[3] = true;
//	   }
//
//	   if (strcmp(argv[i], "-velodyne") == 0)
//		   lidars_alive[16] = true;
//
//	}
//
//   int at_least_one_lidar_alive = 0;
//   carmen_lidar_config *p;
//   for(int i = 0; i < MAX_NUMBER_OF_LIDARS_NEURAL; i++)
//   {
//	   if (lidars_alive[i])  // Lidars start from 10 in the sensors_params vector
//	   {
//		   at_least_one_lidar_alive = 1;
//		   if (i == 16)
//		   {
//			   lidar_config[i].pose = velodyne_pose;
//			   lidar_config[i].vertical_angles = sorted_vertical_angles;
//			   lidar_config[i].range_division_factor = 500.0;
//			   continue;
//		   }
//		   p = &lidar_config[i];
//		   load_lidar_config(argc, argv, i, &p);
//	   }
//   }
//
//   if (at_least_one_lidar_alive == 0)
//	   carmen_die("Nenhum lidar classificado como 'alive', verifique se seus argumentos estão corretos!\nExemplos de argumentos:\n ./neural_object_detector_tracker 3 1 -lidar 1 -lidar 3 -lidar 16\n ./neural_object_detector_tracker intelbras1 1 -lidar 0 -lidar 1 -lidar 2 -lidar 3\n ./neural_object_detector_tracker intelbras1 1 -velodyne\n ./neural_object_detector_tracker intelbras1 1 -ouster64\n");

   	carmen_param_install_params(argc, argv, optional_commandline_param_list, sizeof(optional_commandline_param_list) / sizeof(optional_commandline_param_list[0]));

//	printf ("%s %d image_index %d %lf %d %d %d %d\n", camera_model, message_number, image_index, resize_factor, crop_x, crop_y, crop_w, crop_w);
}


void
subscribe_messages()
{
//	if (is_intelbras == false)
//    	carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) image_handler, CARMEN_SUBSCRIBE_LATEST);
//	else
	camera_drivers_subscribe_message(message_number, NULL, (carmen_handler_t) camera_image_handler, CARMEN_SUBSCRIBE_LATEST);
/*
	carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler0, CARMEN_SUBSCRIBE_LATEST, 0);

    carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler1, CARMEN_SUBSCRIBE_LATEST, 1);

    carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler2, CARMEN_SUBSCRIBE_LATEST, 2);

    carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler3, CARMEN_SUBSCRIBE_LATEST, 3);
*/

//	if (lidars_alive[0])
//		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler0, CARMEN_SUBSCRIBE_LATEST, 0);
//
//	if (lidars_alive[1])
//		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler1, CARMEN_SUBSCRIBE_LATEST, 1);
//
//	if (lidars_alive[2])
//		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler2, CARMEN_SUBSCRIBE_LATEST, 2);
//
//	if (lidars_alive[3])
//		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler3, CARMEN_SUBSCRIBE_LATEST, 3);
//
//	if (lidars_alive[4])
//		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler4, CARMEN_SUBSCRIBE_LATEST, 4);
//
//	if (lidars_alive[5])
//		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler5, CARMEN_SUBSCRIBE_LATEST, 5);
//
//	if (lidars_alive[6])
//		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler6, CARMEN_SUBSCRIBE_LATEST, 6);
//
//	if (lidars_alive[7])
//		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler7, CARMEN_SUBSCRIBE_LATEST, 7);
//
//	if (lidars_alive[8])
//		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler8, CARMEN_SUBSCRIBE_LATEST, 8);
//
//	if (lidars_alive[9])
//		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler9, CARMEN_SUBSCRIBE_LATEST, 9);
//
//	if (lidars_alive[10])
//		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler10, CARMEN_SUBSCRIBE_LATEST, 10);
//
//	if (lidars_alive[11])
//		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler11, CARMEN_SUBSCRIBE_LATEST, 11);
//
//	if (lidars_alive[12])
//		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler12, CARMEN_SUBSCRIBE_LATEST, 12);
//
//	if (lidars_alive[13])
//		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler13, CARMEN_SUBSCRIBE_LATEST, 13);
//
//	if (lidars_alive[14])
//		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler14, CARMEN_SUBSCRIBE_LATEST, 14);
//
//	if (lidars_alive[15])
//		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) variable_scan_message_handler15, CARMEN_SUBSCRIBE_LATEST, 15);
//
//	if (lidars_alive[16])
//		carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t) velodyne_partial_scan_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t) velodyne_partial_scan_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);

//	carmen_rddf_subscribe_annotation_message(NULL, (carmen_handler_t) rddf_annotation_message_handler, CARMEN_SUBSCRIBE_LATEST);

#ifdef NEW_SCAN_MESSAGE
	if (lidars_alive[0])
		carmen_mapper_subscribe_probability_of_each_ray_of_lidar_hit_obstacle_message(NULL,	(carmen_handler_t) probability_of_each_ray_message_handler_0, CARMEN_SUBSCRIBE_LATEST, 0);

	if (lidars_alive[1])
		carmen_mapper_subscribe_probability_of_each_ray_of_lidar_hit_obstacle_message(NULL,	(carmen_handler_t) probability_of_each_ray_message_handler_1, CARMEN_SUBSCRIBE_LATEST, 1);

	if (lidars_alive[2])
		carmen_mapper_subscribe_probability_of_each_ray_of_lidar_hit_obstacle_message(NULL,	(carmen_handler_t) probability_of_each_ray_message_handler_2, CARMEN_SUBSCRIBE_LATEST, 2);

	if (lidars_alive[3])
		carmen_mapper_subscribe_probability_of_each_ray_of_lidar_hit_obstacle_message(NULL,	(carmen_handler_t) probability_of_each_ray_message_handler_3, CARMEN_SUBSCRIBE_LATEST, 3);

	if (lidars_alive[4])
		carmen_mapper_subscribe_probability_of_each_ray_of_lidar_hit_obstacle_message(NULL,	(carmen_handler_t) probability_of_each_ray_message_handler_4, CARMEN_SUBSCRIBE_LATEST, 4);

	if (lidars_alive[5])
		carmen_mapper_subscribe_probability_of_each_ray_of_lidar_hit_obstacle_message(NULL,	(carmen_handler_t) probability_of_each_ray_message_handler_5, CARMEN_SUBSCRIBE_LATEST, 5);

	if (lidars_alive[6])
		carmen_mapper_subscribe_probability_of_each_ray_of_lidar_hit_obstacle_message(NULL,	(carmen_handler_t) probability_of_each_ray_message_handler_6, CARMEN_SUBSCRIBE_LATEST, 6);

	if (lidars_alive[7])
		carmen_mapper_subscribe_probability_of_each_ray_of_lidar_hit_obstacle_message(NULL,	(carmen_handler_t) probability_of_each_ray_message_handler_7, CARMEN_SUBSCRIBE_LATEST, 7);

	if (lidars_alive[8])
		carmen_mapper_subscribe_probability_of_each_ray_of_lidar_hit_obstacle_message(NULL,	(carmen_handler_t) probability_of_each_ray_message_handler_8, CARMEN_SUBSCRIBE_LATEST, 8);

	if (lidars_alive[9])
		carmen_mapper_subscribe_probability_of_each_ray_of_lidar_hit_obstacle_message(NULL,	(carmen_handler_t) probability_of_each_ray_message_handler_9, CARMEN_SUBSCRIBE_LATEST, 9);

	if (lidars_alive[10])
		carmen_mapper_subscribe_probability_of_each_ray_of_lidar_hit_obstacle_message(NULL,	(carmen_handler_t) probability_of_each_ray_message_handler_10, CARMEN_SUBSCRIBE_LATEST, 10);

	if (lidars_alive[11])
		carmen_mapper_subscribe_probability_of_each_ray_of_lidar_hit_obstacle_message(NULL,	(carmen_handler_t) probability_of_each_ray_message_handler_11, CARMEN_SUBSCRIBE_LATEST, 11);

	if (lidars_alive[12])
		carmen_mapper_subscribe_probability_of_each_ray_of_lidar_hit_obstacle_message(NULL,	(carmen_handler_t) probability_of_each_ray_message_handler_12, CARMEN_SUBSCRIBE_LATEST, 12);

	if (lidars_alive[13])
		carmen_mapper_subscribe_probability_of_each_ray_of_lidar_hit_obstacle_message(NULL,	(carmen_handler_t) probability_of_each_ray_message_handler_13, CARMEN_SUBSCRIBE_LATEST, 13);

	if (lidars_alive[14])
		carmen_mapper_subscribe_probability_of_each_ray_of_lidar_hit_obstacle_message(NULL,	(carmen_handler_t) probability_of_each_ray_message_handler_14, CARMEN_SUBSCRIBE_LATEST, 14);

	if (lidars_alive[15])
		carmen_mapper_subscribe_probability_of_each_ray_of_lidar_hit_obstacle_message(NULL,	(carmen_handler_t) probability_of_each_ray_message_handler_15, CARMEN_SUBSCRIBE_LATEST, 15);

	if (lidars_alive[16])
		carmen_mapper_subscribe_probability_of_each_ray_of_lidar_hit_obstacle_message(NULL,	(carmen_handler_t) probability_of_each_ray_message_handler_16, CARMEN_SUBSCRIBE_LATEST, 16);

	#endif
}


void
initializer()
{
	initialize_transformations(board_pose, camera_pose, &transformer);
	char* carmen_home = getenv("CARMEN_HOME");
	char str_classes_names[1024];
	char yolocfg[1024];
	char yoloweights[1024];
	sprintf(str_classes_names, "%s/sharedlib/darknet2/data/coco.names", carmen_home);
	sprintf(yolocfg, "%s/sharedlib/darknet2/cfg/yolov3.cfg", carmen_home);
	sprintf(yoloweights, "%s/sharedlib/darknet2/yolov3.weights", carmen_home);

//	classes_names = get_classes_names((char *) str_classes_names);
//	network_struct = initialize_YOLO((char *) yolocfg, (char *) yoloweights);
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

	return (0);
}
