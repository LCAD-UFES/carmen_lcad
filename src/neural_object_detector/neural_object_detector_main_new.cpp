#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/velodyne_camera_calibration.h>
#include <carmen/camera_boxes_to_world.h>
#include <carmen/moving_objects_messages.h>
#include <carmen/moving_objects_interface.h>
#include <string>
#include <cstdlib>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include "Darknet.hpp"
#include <carmen/dbscan.h>

using namespace std;
using namespace cv;

int camera;
int camera_side;

carmen_camera_parameters camera_parameters;
carmen_pose_3D_t velodyne_pose;
carmen_pose_3D_t camera_pose;

const unsigned int maxPositions = 50;
carmen_velodyne_partial_scan_message *velodyne_msg;
vector<carmen_velodyne_partial_scan_message> velodyne_vector;


Detector *darknet;
vector<string> obj_names_vector;
vector<Scalar> obj_colours_vector;

carmen_moving_objects_point_clouds_message moving_objects_point_clouds_message;
carmen_point_t globalpos;


carmen_vector_3D_t
compute_objects_3D_position(vector<carmen_vector_3D_t> points_inside_box)
{
	carmen_vector_3D_t centroid;
	centroid.x = 0.0;
	centroid.y = 0.0;
	centroid.z = 0.0;

	for (unsigned int i = 0; i < points_inside_box.size(); i++)
	{
		centroid.x += points_inside_box[i].x;
		centroid.y += points_inside_box[i].y;
		centroid.z += points_inside_box[i].z;
	}

	centroid.x = centroid.x  / (double) points_inside_box.size();
	centroid.y = centroid.y  / (double) points_inside_box.size();
	centroid.z = centroid.z  / (double) points_inside_box.size();

	return (centroid);
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

    cout << "Object names loaded!\n\n";

    return file_lines;
}


Scalar
set_object_vector_color()
{
	for (unsigned int i = 0; i < obj_names_vector.size(); i++)
	{
		if (i == 2 || i == 3 || i == 5 || i == 6 || i == 7)
			obj_colours_vector.push_back(Scalar(0, 0, 255));
		else if (i == 0 || i == 1)
			obj_colours_vector.push_back(Scalar(255, 0, 0));
		else
			obj_colours_vector.push_back(Scalar(230, 230, 230));
	}
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
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


vector< vector<velodyne_camera_points> >
velodyne_points_inside_bounding_boxes(vector<bbox_t> &bouding_box_vector, vector<velodyne_camera_points> &velodyne_points_vector)
{
	vector< vector<velodyne_camera_points> > laser_list_inside_each_bounding_box; //each_bounding_box_laser_list

	for (unsigned int i = 0; i < bouding_box_vector.size(); i++)
	{
		vector<velodyne_camera_points> lasers_points_inside_bounding_box;

		for (unsigned int j = 0; j < velodyne_points_vector.size(); j++)
		{
			if (velodyne_points_vector[j].image_x >  bouding_box_vector[i].x &&
				velodyne_points_vector[j].image_x < (bouding_box_vector[i].x + bouding_box_vector[i].w) &&
				velodyne_points_vector[j].image_y >  bouding_box_vector[i].y &&
				velodyne_points_vector[j].image_y < (bouding_box_vector[i].y + bouding_box_vector[i].h))
			{
				lasers_points_inside_bounding_box.push_back(velodyne_points_vector[j]);
			}
		}
		laser_list_inside_each_bounding_box.push_back(lasers_points_inside_bounding_box);
	}

	return laser_list_inside_each_bounding_box;
}


dbscan::Cluster
convert_to_dbscan_type(vector<velodyne_camera_points> points_list)
{
	dbscan::Cluster initial_cluster;
	carmen_point_t aux;

	for (unsigned int i = 0; i < points_list.size(); i++)
	{
		aux.x = points_list[i].cartesian.x;
		aux.y = points_list[i].cartesian.y;
		aux.theta = 0.0;                // The theta value is not used for clustering, although dbscan Cluster type has a theta field
		initial_cluster.push_back(aux);
	}
	return (initial_cluster);
}


dbscan::Cluster
get_biggest_cluster(dbscan::Clusters clusters)
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


dbscan::Clusters
filter_object_points_using_dbscan(vector<vector<velodyne_camera_points>> &points_lists)
{
	dbscan::Clusters filtered_points;

	for (unsigned int i = 0; i < points_lists.size(); i++)
	{
		if (points_lists[i].size() > 0)
		{
			dbscan::Cluster points = convert_to_dbscan_type(points_lists[i]);  // Create vector in dbscan point type

			dbscan::Clusters clusters = dbscan::dbscan(0.5, 5, points);       // Compute clusters using dbscan

			if (clusters.size() > 1)                                           // If dbscan returned more then one cluster
			{
				filtered_points.push_back(get_biggest_cluster(clusters));      // get the biggest, the biggest cluster will always better represent the object
			}
		}
	}
	return (filtered_points);
}


void
show_object_detections(Mat rgb_image, vector<bbox_t> predictions, double fps)
{
	char object_info[25];
    char frame_rate[25];

    cvtColor(rgb_image, rgb_image, COLOR_RGB2BGR);

    sprintf(frame_rate, "FPS = %.2f", fps);

    putText(rgb_image, frame_rate, Point(10, 25), FONT_HERSHEY_PLAIN, 2, cvScalar(0, 255, 0), 2);

    for (unsigned int i = 0; i < predictions.size(); i++)
    {
        sprintf(object_info, "%d %s %.2f", predictions[i].obj_id, obj_names_vector[predictions[i].obj_id].c_str(), predictions[i].prob);

        rectangle(rgb_image, Point(predictions[i].x, predictions[i].y), Point((predictions[i].x + predictions[i].w), (predictions[i].y + predictions[i].h)),
                      obj_colours_vector[predictions[i].obj_id], 1);

        putText(rgb_image, object_info, Point(predictions[i].x + 1, predictions[i].y - 3), FONT_HERSHEY_PLAIN, 1, cvScalar(255, 255, 0), 1);
    }

    imshow("Neural Object Detector", rgb_image);
    waitKey(1);
}


vector<carmen_point_t>
compute_detected_objects_poses(dbscan::Clusters filtered_points)
{
	vector<carmen_point_t> objects_positions;
	carmen_point_t point;
	point.x = 0.0;
	point.y = 0.0;
	point.theta = 0.0;    // The theta value is not used

	for(unsigned int i = 0; i < filtered_points.size(); i++)
	{
		for(unsigned int j = 0; j < filtered_points[i].size(); j++)
		{
			point.x = filtered_points[i][j].x;
			point.y = filtered_points[i][j].y;
		}
		objects_positions.push_back(point);
	}
	return (objects_positions);
}

void
carmen_translte_2d(double *x, double *y, double *offset_x, double *offset_y)
{
	*x += *offset_x;
	*y += *offset_y;
}


carmen_moving_objects_point_clouds_message
build_detected_objects_message(vector<bbox_t> predictions, vector<carmen_point_t> objects_poses)
{
	carmen_moving_objects_point_clouds_message msg;

	printf ("Predictions %d Poses %d\n", (int) predictions.size(), (int) objects_poses.size());

	msg.num_point_clouds = objects_poses.size();
	msg.point_clouds = (t_point_cloud_struct *) malloc (msg.num_point_clouds * sizeof(t_point_cloud_struct));

	for (int i = 0; i < msg.num_point_clouds; i++)
	{
		//carmen_translte_2d()
		//carmen_rotate_2d(x, y, the)

	}

	return (msg);
}


void
image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	unsigned char *img;
	double fps, start_time = carmen_get_time();

	if (camera_side == 0)
		img = image_msg->raw_left;
	else
		img = image_msg->raw_right;

	Mat open_cv_image = Mat(image_msg->height, image_msg->width, CV_8UC3, img, 0);              // CV_32FC3 float 32 bit 3 channels (to char image use CV_8UC3)

	Rect myROI(280, 70, 720, 480);     // TODO put this in the .ini file
	Mat cropped_image = open_cv_image(myROI);

	vector<bbox_t> predictions = darknet->detect(cropped_image, 0.2);  // Arguments (image, threshold)

	// Removes the ground, Removes points outside cameras field of view and Returns the points that reach obstacles  TODO TODO TODO TODO TODO TODO TODO Modificar essa funcao
	vector<velodyne_camera_points> velodyne_points_vector = velodyne_camera_calibration_remove_points_out_of_FOV_and_that_hit_ground(velodyne_msg,
			camera_parameters, velodyne_pose, camera_pose, image_msg->width, image_msg->height);

	vector<vector<velodyne_camera_points>> points_lists = velodyne_points_inside_bounding_boxes(predictions, velodyne_points_vector);

	dbscan::Clusters filtered_points_lists = filter_object_points_using_dbscan(points_lists);

	vector<carmen_point_t> objects_poses = compute_detected_objects_poses(filtered_points_lists);



	fps = 1.0 / (carmen_get_time() - start_time);

	show_object_detections(cropped_image, predictions, fps);
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


int
read_parameters(int argc, char **argv)
{
    camera = atoi(argv[1]);             // Define the camera to be used
    camera_side = atoi(argv[2]);        // 0 For left image 1 for right image

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

		{(char *) "velodyne", (char *) "x",     CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.x), 0, NULL},
		{(char *) "velodyne", (char *) "y",     CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.y), 0, NULL},
		{(char *) "velodyne", (char *) "z",     CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.z), 0, NULL},
		{(char *) "velodyne", (char *) "roll",  CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.roll), 0, NULL},
		{(char *) "velodyne", (char *) "pitch", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.pitch), 0, NULL},
		{(char *) "velodyne", (char *) "yaw",   CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.yaw), 0, NULL},

		{camera_string, (char*) "x",     CARMEN_PARAM_DOUBLE, &camera_pose.position.x, 0, NULL },
		{camera_string, (char*) "y",     CARMEN_PARAM_DOUBLE, &camera_pose.position.y, 0, NULL },
		{camera_string, (char*) "z",     CARMEN_PARAM_DOUBLE, &camera_pose.position.z, 0, NULL },
		{camera_string, (char*) "roll",  CARMEN_PARAM_DOUBLE, &camera_pose.orientation.roll, 0, NULL },
		{camera_string, (char*) "pitch", CARMEN_PARAM_DOUBLE, &camera_pose.orientation.pitch, 0, NULL },
		{camera_string, (char*) "yaw",   CARMEN_PARAM_DOUBLE, &camera_pose.orientation.yaw, 0, NULL }
    };


    num_items = sizeof(param_list) / sizeof(param_list[0]);
    carmen_param_install_params(argc, argv, param_list, num_items);

    return 0;
}


int
main(int argc, char **argv)
{
    if ((argc != 3))
        carmen_die("%s: Wrong number of parameters. neural_object_detector requires 2 parameter and received %d. \n Usage: %s <camera_number> <camera_side(0-left; 1-right)\n>",
                   argv[0], argc - 1, argv[0]);

    int gpu = 1;
    int device_id = 0;

    string darknet_home = getenv("DARKNET_HOME");  // Get environment variable pointing path of darknet

    if (darknet_home.empty())
        printf("Cannot find darknet path. Check if you have correctly set DARKNET_HOME environment variable.\n");

    string cfg_filename = darknet_home + "/cfg/neural_object_detector_yolo.cfg";
    string weight_filename = darknet_home + "/yolo.weights";
    string class_names_file = darknet_home + "/data/coco.names";
    obj_names_vector = objects_names_from_file(class_names_file);
    set_object_vector_color();

    darknet = new Detector(cfg_filename, weight_filename, device_id);
    carmen_test_alloc(darknet);

    setlocale(LC_ALL, "C");

    carmen_ipc_initialize(argc, argv);

    signal(SIGINT, shutdown_module);

    read_parameters(argc, argv);

    subscribe_messages();

    carmen_ipc_dispatch();

    return 0;
}
