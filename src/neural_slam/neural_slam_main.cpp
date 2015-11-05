#include "neural_slam_server.h"
#include "message_interpolation.cpp"
#include <carmen/fused_odometry_interface.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/ekf_odometry_interface.h>

int image_width, image_height;

NeuralSlamServer *neural_slam_server;
carmen_vector_3D_t robot_size_g;
tf::Transformer transformer;
//MessageInterpolation<carmen_fused_odometry_message, carmen_bumblebee_basic_stereoimage_message> interpolator(3);
MessageInterpolation<carmen_localize_ackerman_globalpos_message, carmen_bumblebee_basic_stereoimage_message> interpolator(1);

carmen_pose_3D_t car_pose_g, camera_pose_g, board_pose_g;

int read_robot_base_pose_g = 1;
carmen_pose_3D_t robot_base_pose_g;

void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("neural_slam: disconnected.\n");

		exit(0);
	}
}

int read_parameters(int argc, char **argv)
{
	int num_items;
	char bumblebee_string[256];
	char camera_string[256];

	if (argc < 2)
		printf("usage: ./neural_slam <camera>\n");

	sprintf(bumblebee_string, "%s%d", "bumblebee_basic", atoi(argv[1]));
	sprintf(camera_string, "%s%d", "camera", atoi(argv[1]));

	carmen_param_t param_list[] =
	{
			{(char *) "carmodel", (char *) "size_x", CARMEN_PARAM_DOUBLE, &(robot_size_g.x), 0, NULL},
			{(char *) "carmodel", (char *) "size_y", CARMEN_PARAM_DOUBLE, &(robot_size_g.y), 0, NULL},
			{(char *) "carmodel", (char *) "size_z", CARMEN_PARAM_DOUBLE, &(robot_size_g.z), 0, NULL},
			{(char *) bumblebee_string, (char *) "width", CARMEN_PARAM_DOUBLE, &(image_width), 0, NULL},
			{(char *) bumblebee_string, (char *) "height", CARMEN_PARAM_DOUBLE, &(image_height), 0, NULL},

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

	return 0;
}

static gint
updateIPC(gpointer *data __attribute__ ((unused)))
{
	carmen_ipc_sleep(0.01);
	carmen_graphics_update_ipc_callbacks((GdkInputFunction)updateIPC);
	return 1;
}


static int first_update_robot_pose = 0;
static tf::Transform first_car_pose;
static tf::Transform first_neural_pose;
void
update_robot_pose_in_transformation_tree(carmen_pose_3D_t robot_pose)
{
	tf::Transform world_to_car_pose;

	if(!first_update_robot_pose && (robot_pose.position.x != 0 && robot_pose.position.x != 0))
	{
		first_update_robot_pose = 1;
		first_car_pose.setOrigin(tf::Vector3(robot_pose.position.x, robot_pose.position.y, robot_pose.position.z));
		first_car_pose.setRotation(tf::Quaternion(robot_pose.orientation.yaw, robot_pose.orientation.pitch, robot_pose.orientation.roll));
	}

	world_to_car_pose.setOrigin(tf::Vector3(robot_pose.position.x, robot_pose.position.y, robot_pose.position.z + car_pose_g.position.z));
	world_to_car_pose.setRotation(tf::Quaternion(robot_pose.orientation.yaw, robot_pose.orientation.pitch, robot_pose.orientation.roll));

	world_to_car_pose = first_car_pose.inverse() * world_to_car_pose;

	tf::StampedTransform world_to_car_transform(world_to_car_pose, tf::Time(0), "/world", "/car");
	transformer.setTransform(world_to_car_transform, "world_to_car_transform");
}

int
writePpm( char* 	szFilename,
		unsigned char* pucBuffer,
		int		width,
		int		height )
{
	FILE* stream;
	stream = fopen( szFilename, "wb" );
	if( stream == NULL)
	{
		perror( "Can't open image file" );
		return 1;
	}

	fprintf( stream, "P6\n%u %u 255\n", width, height );
	fwrite( pucBuffer, 3*width, height, stream );
	fclose( stream );
	return 0;
}

int file_counter[100];

void
save_image_and_robot_pose(int n, char* directory, char* filename, carmen_bumblebee_basic_stereoimage_message* message, carmen_pose_3D_t robot_pose)
{
	FILE* fd;
	char image[1024];
	char filename_path[1024];

	sprintf(filename_path, "%s%s", directory, filename);

	sprintf(image, "%s%03dr.ppm", directory, file_counter[n]);
	writePpm(image, message->raw_right, message->width, message->height);
	sprintf(image, "%s%03dl.ppm", directory, file_counter[n]);
	writePpm(image, message->raw_left, message->width, message->height);

	file_counter[n]++;

	fd = fopen(filename_path, "a+");

	fprintf(fd, "%s ", image);
	fprintf(fd, "%f %f %f %f %f %f ", robot_pose.position.x, robot_pose.position.y, robot_pose.position.z,
			robot_pose.orientation.yaw, robot_pose.orientation.pitch, robot_pose.orientation.roll);
	fprintf(fd, "\n");
	fclose(fd);

}

void capture_images_in_meters(int n, char* directory, carmen_pose_3D_t pose, carmen_bumblebee_basic_stereoimage_message* message)
{
	static carmen_world_point_t previous_pose[100];
	carmen_world_point_t current_pose[100];

	current_pose[n].pose.x = pose.position.x;
	current_pose[n].pose.y = pose.position.y;
	current_pose[n].pose.theta = pose.orientation.yaw;

	double distance = carmen_distance_world(&current_pose[n], &previous_pose[n]);

	printf("distance: %f\n", distance);
	if(fabs(distance - (double) n) < 0.05 || distance >= (double) n)
	{
		printf("distance_chosen: %f\n", distance);

		previous_pose[n] = current_pose[n];
		save_image_and_robot_pose(n, (char *)directory, (char *)"data.txt", message, pose);
	}

	printf("\n");
}

void getErrorEllipse(double chisquare_val, cv::Point2f mean, cv::Mat covmat, double *angle, double* major_axis, double *minor_axis){

	//Get the eigenvalues and eigenvectors
	cv::Mat eigenvalues, eigenvectors;
	cv::eigen(covmat, true, eigenvalues, eigenvectors);

	//Calculate the angle between the largest eigenvector and the x-axis
	*angle = atan2(eigenvectors.at<double>(1,0), eigenvectors.at<double>(0,0));

	//Conver to degrees instead of radians
	*angle = carmen_radians_to_degrees(*angle);

	//Calculate the size of the minor and major axes
	*major_axis = chisquare_val*sqrt(eigenvalues.at<double>(0));
	*minor_axis = chisquare_val*sqrt(eigenvalues.at<double>(1));
}

void carmen_bumblebee_basic_stereoimage_message_handler(carmen_bumblebee_basic_stereoimage_message* message)
{
//	carmen_fused_odometry_message nearest_message;
	carmen_localize_ackerman_globalpos_message nearest_message;
	nearest_message = interpolator.InterpolateMessages(message);

	//capture_images_in_meters(1, (char *)"/media/OS/Users/Lauro/Downloads/Log/datasets2/2012_1.0/", nearest_message.pose, message);
	//capture_images_in_meters(5, (char *)"/media/OS/Users/Lauro/Downloads/Log/datasets2/2012_5.0/", nearest_message.pose, message);
	//capture_images_in_meters(10, (char *)"/media/OS/Users/Lauro/Downloads/Log/datasets2/2012_10.0/", nearest_message.pose, message);
	//capture_images_in_meters(15, (char *)"/media/OS/Users/Lauro/Downloads/Log/datasets2/2012_15.0/", nearest_message.pose, message);
}

//void carmen_fused_odometry_odometry_message_handler(carmen_fused_odometry_message* message)
//{
//	interpolator.AddMessageToInterpolationList(message);
//	update_robot_pose_in_transformation_tree(message->pose);
//	neural_slam_server->gui_->setRobotPose(message->pose);
//}


int localize_calls = 0;

void carmen_localizer_ackerman_globalpos_handler(carmen_localize_ackerman_globalpos_message* message)
{
	tf::Transform world_to_localize_pose;
	carmen_pose_3D_t updated_pose;

	if(localize_calls < 1)
	{
		localize_calls++;
		return;
	}

	world_to_localize_pose.setOrigin(tf::Vector3(message->pose.position.x, message->pose.position.y, message->pose.position.z + car_pose_g.position.z));
	world_to_localize_pose.setRotation(tf::Quaternion(message->pose.orientation.yaw, message->pose.orientation.pitch, message->pose.orientation.roll));

	world_to_localize_pose = first_car_pose.inverse() * world_to_localize_pose;

	updated_pose.position.x = world_to_localize_pose.getOrigin().x();
	updated_pose.position.y = world_to_localize_pose.getOrigin().y();
	updated_pose.position.z = world_to_localize_pose.getOrigin().z();
	tf::Matrix3x3(world_to_localize_pose.getRotation()).getEulerYPR(updated_pose.orientation.yaw, updated_pose.orientation.pitch, updated_pose.orientation.roll);

	neural_slam_server->gui_->setLocalizeAckermanPose(updated_pose);
}

void carmen_ekf_odometry_handler(carmen_ekf_odometry_odometry_message* message)
{
	if(localize_calls < 1)
	{
		return;
	}

	tf::StampedTransform updated_robot_pose;
	tf::StampedTransform updated_neural_pose;
	carmen_pose_3D_t updated_pose, updated_camera_pose;

	tf::StampedTransform world_to_camera_transform;
	tf::StampedTransform car_to_camera_transform;

	transformer.lookupTransform("/car", "/camera", tf::Time(0), car_to_camera_transform);
	updated_camera_pose.position.x = car_to_camera_transform.getOrigin().x();
	updated_camera_pose.position.y = car_to_camera_transform.getOrigin().y();
	updated_camera_pose.position.z = car_to_camera_transform.getOrigin().z();

	tf::Matrix3x3(car_to_camera_transform.getRotation()).getEulerYPR(updated_camera_pose.orientation.yaw, updated_camera_pose.orientation.pitch, updated_camera_pose.orientation.roll);
	neural_slam_server->gui_->setCameraPose(updated_camera_pose);

	update_robot_pose_in_transformation_tree(message->neural_global_pose);
	transformer.lookupTransform("/world", "/car", tf::Time(0), updated_neural_pose);
	updated_pose.position.x = updated_neural_pose.getOrigin().x();
	updated_pose.position.y = updated_neural_pose.getOrigin().y();
	updated_pose.position.z = updated_neural_pose.getOrigin().z();
	tf::Matrix3x3(updated_neural_pose.getRotation()).getEulerYPR(updated_pose.orientation.yaw, updated_pose.orientation.pitch, updated_pose.orientation.roll);
	neural_slam_server->gui_->setNeuralPose(updated_pose);

	for(int i = 0; i < 5; i++)
	{
		tf::Transform car_to_measurement_pose;

		car_to_measurement_pose.setOrigin(tf::Vector3(message->saliencies[i].x, message->saliencies[i].y, 0.0));
		car_to_measurement_pose.setRotation(tf::Quaternion(0.0, 0.0, 0.0));

		transformer.lookupTransform("/world", "/camera", tf::Time(0), world_to_camera_transform);

		car_to_measurement_pose = world_to_camera_transform * car_to_measurement_pose;

		carmen_vector_3D_t saliency_pose;
		saliency_pose.x = car_to_measurement_pose.getOrigin().x();
		saliency_pose.y = car_to_measurement_pose.getOrigin().y();

		neural_slam_server->gui_->setNeuralSaliency(saliency_pose);
	}

	update_robot_pose_in_transformation_tree(message->estimated_pose);
	transformer.lookupTransform("/world", "/car", tf::Time(0), updated_robot_pose);

	updated_pose.position.x = updated_robot_pose.getOrigin().x();
	updated_pose.position.y = updated_robot_pose.getOrigin().y();
	updated_pose.position.z = updated_robot_pose.getOrigin().z();
	tf::Matrix3x3(updated_robot_pose.getRotation()).getEulerYPR(updated_pose.orientation.yaw, updated_pose.orientation.pitch, updated_pose.orientation.roll);
	neural_slam_server->gui_->setRobotPose(updated_pose);

	double angle, major_axis, minor_axis;
	cv::Mat covmat = (cv::Mat_<double>(2,2) << message->covariance[0][0], message->covariance[0][1], message->covariance[1][0], message->covariance[1][1]);
	cv::Point2f mean(updated_pose.position.x, updated_pose.position.y);
	getErrorEllipse(2.4477, mean, covmat, &angle, &major_axis, &minor_axis);

	neural_slam_server->gui_->setRobotCovarianceElipse(angle, major_axis, minor_axis);
	printf("angle: %f, major: %f, minor: %f\n", angle, major_axis, minor_axis);

	for(int i = 0; i < 5; i++)
	{
		tf::Transform car_to_observation_pose;

		car_to_observation_pose.setOrigin(tf::Vector3(message->correspondences[i].x, message->correspondences[i].y, 0.0));
		car_to_observation_pose.setRotation(tf::Quaternion(0.0, 0.0, 0.0));

		transformer.lookupTransform("/world", "/camera", tf::Time(0), world_to_camera_transform);

		car_to_observation_pose = world_to_camera_transform * car_to_observation_pose;

		carmen_vector_3D_t correspondence_pose;

		correspondence_pose.x = car_to_observation_pose.getOrigin().x();
		correspondence_pose.y = car_to_observation_pose.getOrigin().y();

		neural_slam_server->gui_->setNeuralCorrespondence(correspondence_pose);
	}
}

void
neural_slam_transformation_tree()
{
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

	tf::StampedTransform board_to_camera_transform(board_to_camera_pose, tf::Time(0), "/board", "/camera");
	transformer.setTransform(board_to_camera_transform, "board_to_camera_transform");
}

int
main(int argc, char **argv)
{
	/* connect to ipc server */
	carmen_ipc_initialize(argc, argv);

	/* check the param server version */
	carmen_param_check_version(argv[0]);

	/* read parameters from .ini */
	read_parameters(argc,argv);

	/* initialize transformation tree */
	neural_slam_transformation_tree();

	/* register a gdk's update callback */
	carmen_graphics_update_ipc_callbacks((GdkInputFunction)updateIPC);

	neural_slam_server = new NeuralSlamServer();

	/* initialize the graphical interface */
	neural_slam_server->InitializeGui(robot_size_g);

	for(int i = 0; i < 100; i++)
		file_counter[i] = 0;

	//carmen_fused_odometry_subscribe_fused_odometry_message(NULL, (carmen_handler_t) carmen_fused_odometry_odometry_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) carmen_localizer_ackerman_globalpos_handler, CARMEN_SUBSCRIBE_LATEST);

	//carmen_bumblebee_basic_subscribe_stereoimage(atoi(argv[1]), NULL, (carmen_handler_t) carmen_bumblebee_basic_stereoimage_message_handler, CARMEN_SUBSCRIBE_ALL);

	carmen_ekf_odometry_subscribe_odometry_message(NULL, (carmen_handler_t) carmen_ekf_odometry_handler, CARMEN_SUBSCRIBE_LATEST);

	/* loop forever in gdk */
	neural_slam_server->SpinOnce();

	return 0;
}
