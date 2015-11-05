#include "laslam_server.h"
#include "laslam_messages.h"
#include "laslam_interface.h"
#include "laslam_features.h"
#include "laslam_interpolation.cpp"

int image_width, image_height;

LandmarkSlamServer *laslam_server;
stereo_util camera_parameters;
carmen_vector_3D_t robot_size_g;
tf::Transformer transformer;

carmen_pose_3D_t car_pose_g, camera_pose_g, board_pose_g;

int read_robot_base_pose_g = 1;
carmen_pose_3D_t robot_base_pose_g;

carmen_laslam_landmark_message landmarks_message;

LandmarkSlamMessagesInterpolation<carmen_visual_odometry_pose6d_message, carmen_simple_stereo_disparity_message> odometry_interpolator(10);
int init_visual_odometry_and_stereo_interpolation_counter_g = 0;

LandmarkSlamMessagesInterpolation<carmen_simple_stereo_disparity_message, carmen_laslam_landmark_message> stereo_interpolator(10);
int init_visual_odometry_and_landmark_interpolation_counter_g = 0;

void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("laslam: disconnected.\n");

		exit(0);
	}
}

void
carmen_laslam_initialize_landmark_message(int width, int height)
{
	landmarks_message.host = carmen_get_host();
	landmarks_message.image_height = height;
	landmarks_message.image_width = width;
	landmarks_message.image_size = 3 * width * height;
	landmarks_message.reference_image = (uchar*)malloc(landmarks_message.image_size * sizeof(uchar));
}

int
carmen_laslam_read_parameters(int argc, char **argv)
{
	int num_items;
	char bumblebee_string[256];
	char camera_string[256];

	if (argc < 2)
		printf("usage: ./laslam <camera>\n");

	sprintf(bumblebee_string, "%s%d", "bumblebee_basic", atoi(argv[1]));
	sprintf(camera_string, "%s%d", "camera", atoi(argv[1]));

	carmen_param_t param_list[] =
	{
			{(char *) "carmodel", (char *) "size_x", CARMEN_PARAM_DOUBLE, &(robot_size_g.x), 0, NULL},
			{(char *) "carmodel", (char *) "size_y", CARMEN_PARAM_DOUBLE, &(robot_size_g.y), 0, NULL},
			{(char *) "carmodel", (char *) "size_z", CARMEN_PARAM_DOUBLE, &(robot_size_g.z), 0, NULL},
			{(char *) bumblebee_string, (char *) "width", CARMEN_PARAM_INT, &(image_width), 0, NULL},
			{(char *) bumblebee_string, (char *) "height", CARMEN_PARAM_INT, &(image_height), 0, NULL},

			{(char *) "car", (char *) "x", CARMEN_PARAM_DOUBLE, &(car_pose_g.position.x), 0, NULL},
			{(char *) "car", (char *) "y", CARMEN_PARAM_DOUBLE, &(car_pose_g.position.y), 0, NULL},
			{(char *) "car", (char *) "z", CARMEN_PARAM_DOUBLE, &(car_pose_g.position.z), 0, NULL},
			{(char *) "car", (char *) "yaw", CARMEN_PARAM_DOUBLE, &(car_pose_g.orientation.yaw), 0, NULL},
			{(char *) "car", (char *) "pitch", CARMEN_PARAM_DOUBLE, &(car_pose_g.orientation.pitch), 0, NULL},
			{(char *) "car", (char *) "roll", CARMEN_PARAM_DOUBLE, &(car_pose_g.orientation.roll), 0, NULL},

			{(char *) "sensor_board_1", (char *) "x", CARMEN_PARAM_DOUBLE, &(board_pose_g.position.x), 0, NULL},
			{(char *) "sensor_board_1", (char *) "y", CARMEN_PARAM_DOUBLE, &(board_pose_g.position.y), 0, NULL},
			{(char *) "sensor_board_1", (char *) "z", CARMEN_PARAM_DOUBLE, &(board_pose_g.position.z), 0, NULL},
			{(char *) "sensor_board_1", (char *) "yaw", CARMEN_PARAM_DOUBLE, &(board_pose_g.orientation.yaw), 0, NULL},
			{(char *) "sensor_board_1", (char *) "pitch", CARMEN_PARAM_DOUBLE, &(board_pose_g.orientation.pitch), 0, NULL},
			{(char *) "sensor_board_1", (char *) "roll", CARMEN_PARAM_DOUBLE, &(board_pose_g.orientation.roll), 0, NULL},

			{(char *) camera_string, (char *) "x", CARMEN_PARAM_DOUBLE, &(camera_pose_g.position.x), 0, NULL},
			{(char *) camera_string, (char *) "y", CARMEN_PARAM_DOUBLE, &(camera_pose_g.position.y), 0, NULL},
			{(char *) camera_string, (char *) "z", CARMEN_PARAM_DOUBLE, &(camera_pose_g.position.z), 0, NULL},
			{(char *) camera_string, (char *) "yaw", CARMEN_PARAM_DOUBLE, &(camera_pose_g.orientation.yaw), 0, NULL},
			{(char *) camera_string, (char *) "pitch", CARMEN_PARAM_DOUBLE, &(camera_pose_g.orientation.pitch), 0, NULL},
			{(char *) camera_string, (char *) "roll", CARMEN_PARAM_DOUBLE, &(camera_pose_g.orientation.roll), 0, NULL},
	};


	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	camera_parameters = get_stereo_instance(atoi(argv[1]), image_width, image_height);

	return 0;
}

static gint
updateIPC(gpointer *data __attribute__ ((unused)))
{
	carmen_ipc_sleep(0.01);
	carmen_graphics_update_ipc_callbacks((GdkInputFunction)updateIPC);
	return 1;
}

void
update_robot_pose_in_transformation_tree(carmen_pose_3D_t robot_pose)
{
	tf::Transform world_to_car_pose;

	// initial car pose with respect to the world
	world_to_car_pose.setOrigin(tf::Vector3(car_pose_g.position.x + robot_pose.position.x, car_pose_g.position.y + robot_pose.position.y, car_pose_g.position.z + robot_pose.position.z));
	world_to_car_pose.setRotation(tf::Quaternion(car_pose_g.orientation.yaw + robot_pose.orientation.yaw, car_pose_g.orientation.pitch + robot_pose.orientation.pitch, car_pose_g.orientation.roll + robot_pose.orientation.roll));

	tf::StampedTransform world_to_car_transform(world_to_car_pose, tf::Time(0), "/world", "/car");
	transformer.setTransform(world_to_car_transform, "world_to_car_transform");
}

unsigned char*
crop_landmark_region_from_image(unsigned char* src_data, int src_width, int src_height, carmen_position_t landmark_point)
{
	IplImage* src = NULL;
	IplImage* dst = NULL;

	src = cvCreateImage(cvSize(src_width, src_height), IPL_DEPTH_8U, 3);
	src->imageData = (char *) src_data;

	int xmin = landmark_point.x - 100;
	int ymin = landmark_point.y - 100;

	xmin = (xmin >= 100) ? xmin : 100;
	ymin = (ymin >= 100) ? ymin : 100;

	xmin = (xmin < (src_width - 100)) ? xmin : src_width - 100;
	ymin = (ymin < (src_height - 100)) ? ymin : src_height - 100;

	cvSetImageROI(src, cvRect(xmin, ymin, 200, 200));
	dst = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 3);

	cvCopy(src, dst, NULL);
	cvReleaseImage(&src);

	return ((unsigned char *) dst->imageData);
}

void
carmen_laslam_landmark_handler(carmen_laslam_landmark_message* message)
{
	carmen_pose_3D_t robot_pose;
	carmen_vector_3D_t vodom_pose;
	carmen_position_t landmark_point;
	carmen_vector_3D_p landmark_pose_reference_camera;
	carmen_vector_3D_t landmark_pose_reference_world;
	tf::Transform landmark_transform;
	tf::Transform landmark_transform_reference_world;
	tf::StampedTransform world_to_landmark_transform;
	unsigned char* cropped_landmark;
	carmen_visual_odometry_pose6d_message odometry_message;
	carmen_simple_stereo_disparity_message stereo_message;

	if (stereo_interpolator.Initialized() == 0 || odometry_interpolator.Initialized() == 0)
		return;

	stereo_message = stereo_interpolator.InterpolateMessages(message);
	odometry_message = odometry_interpolator.InterpolateMessages(&stereo_message);

	vodom_pose.x = robot_pose.position.x = odometry_message.pose_6d.x - robot_base_pose_g.position.x;
	vodom_pose.y = robot_pose.position.y = odometry_message.pose_6d.y - robot_base_pose_g.position.y;
	vodom_pose.z = robot_pose.position.z = 0.0;
	robot_pose.orientation.yaw = odometry_message.pose_6d.yaw - robot_base_pose_g.orientation.yaw;
	robot_pose.orientation.pitch = 0.0;
	robot_pose.orientation.roll  = 0.0;

	update_robot_pose_in_transformation_tree(robot_pose);
	transformer.lookupTransform("/world", "/landmark", tf::Time(0), world_to_landmark_transform);

	for(int i = 0; i < message->landmark_list_size; i++)
	{
		landmark_point.x = message->landmark_list[i].x;
		landmark_point.y = message->landmark_list[i].y;

		cropped_landmark = crop_landmark_region_from_image(message->reference_image, message->image_width, message->image_height, landmark_point);

		landmark_pose_reference_camera = reproject_single_point_to_3D_with_depth(camera_parameters, landmark_point,
				stereo_message.disparity[(int)landmark_point.y * image_width + (int)landmark_point.x]);

		landmark_transform.setOrigin(tf::Vector3(landmark_pose_reference_camera->x, landmark_pose_reference_camera->y, landmark_pose_reference_camera->z));
		landmark_transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0));

		landmark_transform_reference_world = world_to_landmark_transform * landmark_transform;

		landmark_pose_reference_world.x = landmark_transform_reference_world.getOrigin().x();
		landmark_pose_reference_world.y = landmark_transform_reference_world.getOrigin().y();
		landmark_pose_reference_world.z = landmark_transform_reference_world.getOrigin().z();

		laslam_server->gui_->addLandmarkPoseToLandmarksList(landmark_pose_reference_world, cropped_landmark);
	}
	laslam_server->gui_->setRobotPose(robot_pose);
	laslam_server->gui_->addVisualOdometryPose(vodom_pose);
}

//TODO move this handler to another process like find_object
void
carmen_laslam_bumblebee_handler(carmen_bumblebee_basic_stereoimage_message *message)
{
	int width = message->width;
	int height = message->height;

	IplImage *left_image = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	IplImage *right_image = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);

	for(int i = 0; i < (height * width); i++)
	{
		/**
		 * A imagem da bumblebee usa o formato rgb-rgb-rgb, enquanto
		 * a imagem da opencv usa o formato bgr-bgr-bgr. As linhas
		 * abaixo fazem essa conversao.
		 */
		left_image->imageData[3 * i] = message->raw_left[3 * i + 2];
		left_image->imageData[3 * i + 1] = message->raw_left[3 * i + 1];
		left_image->imageData[3 * i + 2] = message->raw_left[3 * i];
		right_image->imageData[3 * i] = message->raw_right[3 * i + 2];
		right_image->imageData[3 * i + 1] = message->raw_right[3 * i + 1];
		right_image->imageData[3 * i + 2] = message->raw_right[3 * i];
	}

//	std::vector<cv::KeyPoint> features = extract_matched_features(left_image, right_image);
	std::vector<cv::KeyPoint> features = extract_features(right_image);
	int features_size = 10;
	features_size = std::min((int)features.size(), features_size);

	if (features_size > 0)
	{
		landmarks_message.timestamp = message->timestamp;
		landmarks_message.landmark_list_size = features_size;
		landmarks_message.landmark_list = (carmen_landmark_t *) calloc (landmarks_message.landmark_list_size, sizeof(carmen_landmark_t));
		for(int i = 0; i < features_size; i++)
		{
			landmarks_message.landmark_list[i].x = (int)features.at(i).pt.x;
			landmarks_message.landmark_list[i].y = (int)features.at(i).pt.y;
		}
	}

	carmen_laslam_publish_landmark_message(&landmarks_message);

	cvReleaseImage(&left_image);
	cvReleaseImage(&right_image);
}

void
carmen_laslam_stereo_disparity_handler(carmen_simple_stereo_disparity_message* message)
{
	stereo_interpolator.AddMessageToInterpolationList(message);
}

void
carmen_laslam_visual_odometry_handler(carmen_visual_odometry_pose6d_message* message)
{
	odometry_interpolator.AddMessageToInterpolationList(message);
}

void
carmen_laslam_transformation_tree()
{
	tf::Transform camera_to_landmark_pose;
	tf::Transform board_to_camera_pose;
	tf::Transform car_to_board_pose;
	tf::Transform world_to_car_pose;

	tf::Time::init();

	// initial car pose with respect to the world
	world_to_car_pose.setOrigin(tf::Vector3(car_pose_g.position.x, car_pose_g.position.y, car_pose_g.position.z));
	world_to_car_pose.setRotation(tf::Quaternion(car_pose_g.orientation.yaw, car_pose_g.orientation.pitch, car_pose_g.orientation.roll));

	tf::StampedTransform world_to_car_transform(world_to_car_pose, tf::Time(0), "/world", "/car");
	transformer.setTransform(world_to_car_transform, "world_to_car_transform");

	// board pose with respect to the car
	car_to_board_pose.setOrigin(tf::Vector3(board_pose_g.position.x, board_pose_g.position.y, board_pose_g.position.z));
	car_to_board_pose.setRotation(tf::Quaternion(board_pose_g.orientation.yaw, board_pose_g.orientation.pitch, board_pose_g.orientation.roll)); 				// yaw, pitch, roll

	tf::StampedTransform car_to_board_transform(car_to_board_pose, tf::Time(0), "/car", "/board");
	transformer.setTransform(car_to_board_transform, "car_to_board_transform");

	// camera pose with respect to the board
	board_to_camera_pose.setOrigin(tf::Vector3(camera_pose_g.position.x, camera_pose_g.position.y, camera_pose_g.position.z));
	board_to_camera_pose.setRotation(tf::Quaternion(camera_pose_g.orientation.yaw, camera_pose_g.orientation.pitch, camera_pose_g.orientation.roll)); 				// yaw, pitch, roll

	tf::StampedTransform board_to_camera_transform(car_to_board_pose, tf::Time(0), "/board", "/camera");
	transformer.setTransform(board_to_camera_transform, "board_to_camera_transform");

	// landmark pose with respect to the camera
	camera_to_landmark_pose.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	camera_to_landmark_pose.setRotation(tf::Quaternion(-M_PI / 2.0, 0.0, -M_PI / 2.0)); 				// yaw, pitch, roll

	tf::StampedTransform camera_to_landmark_transform(camera_to_landmark_pose, tf::Time(0), "/camera", "/landmark");
	transformer.setTransform(camera_to_landmark_transform, "camera_to_landmark_transform");
}

int
main(int argc, char **argv)
{
	/* connect to ipc server */
	carmen_ipc_initialize(argc, argv);

	/* check the param server version */
	carmen_param_check_version(argv[0]);

	/* read parameters from .ini */
	carmen_laslam_read_parameters(argc,argv);

	/* initialize transformation tree */
	carmen_laslam_transformation_tree();
	carmen_laslam_initialize_landmark_message(image_width, image_height);

	/* register a gdk's update callback */
	carmen_graphics_update_ipc_callbacks((GdkInputFunction)updateIPC);

	laslam_server = new LandmarkSlamServer();

	/* initialize the graphical interface */
	laslam_server->InitializeGui(robot_size_g);

	//subscribe messages here
	carmen_bumblebee_basic_subscribe_stereoimage(atoi(argv[1]), NULL, (carmen_handler_t) carmen_laslam_bumblebee_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_visual_odometry_subscribe_pose6d_message(NULL, (carmen_handler_t) carmen_laslam_visual_odometry_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_stereo_subscribe(atoi(argv[1]), NULL, (carmen_handler_t) carmen_laslam_stereo_disparity_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_laslam_subscribe_landmark_message(NULL, (carmen_handler_t) carmen_laslam_landmark_handler, CARMEN_SUBSCRIBE_LATEST);

	/* loop forever in gdk */
	laslam_server->SpinOnce();

	return 0;
}
