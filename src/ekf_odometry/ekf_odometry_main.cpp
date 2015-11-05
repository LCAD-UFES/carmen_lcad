#include <carmen/carmen.h>
#include <carmen/visual_odometry_interface.h>
#include <carmen/neural_global_localizer_interface.h>
#include <carmen/visual_search_thin_interface.h>
#include <carmen/stereo_util.h>

#include "ekf_odometry_util.h"
#include "ekf_odometry_main.h"

/*sensor parameters*/
int ekf_odometry_camera_g;
double ekf_odometry_sensor_timeout_g;
double ekf_odometry_frequency_g;
int ekf_odometry_vodom_used_g;
int ekf_odometry_neural_globalpos_used_g;
int ekf_odometry_use_landmark_correction_g;
double ekf_odometry_prediction_covariance_g;
double ekf_odometry_max_distance_outlier_g;
int ekf_odometry_visual_search_image_width_g;
int ekf_odometry_visual_search_image_height_g;
int ekf_odometry_visual_search_num_landmarks_g;
double ekf_odometry_visual_search_landmark_coeff_g;
double ekf_odometry_neural_globalpos_coeff_g;

carmen_ekf_odometry_odometry_message 		message_to_publish;
carmen_localize_ackerman_globalpos_message 	localize_globalpos_message;
carmen_visual_correspondence_t 				visual_search_correspondences[15];
carmen_visual_search_thin_train_message		visual_search_training_message;
carmen_visual_search_thin_test_message		visual_search_testing_message;
stereo_util 								camera_instance;
FILE *fd_result;

/*********************************************************
		   --- Publishers ---
 **********************************************************/


/*********************************************************
		   --- Handlers ---
 **********************************************************/

void
ekf_odometry_localizer_ackerman_globapos_handler(carmen_localize_ackerman_globalpos_message* message)
{
	localize_globalpos_message = *message;
}

carmen_visual_odometry_pose6d_message vo_last_message;
void ekf_odometry_visual_odometry_handler (carmen_visual_odometry_pose6d_message* message)
{
	tf::Transform world_to_vodom_transform;
	tf::StampedTransform vodom_to_world;

	carmen_verbose("VODOM\n");
	vo_callback_counter_++;

	vo_covariance_(1, 1) = vo_covariance_(2, 2) = vo_covariance_(3, 3) = vo_covariance_(4, 4) = vo_covariance_(5, 5) = vo_covariance_(6, 6) = pow(ekf_odometry_prediction_covariance_g, 2);

	// receive data
	vo_meas_ = tf::Transform(tf::Quaternion(message->pose_6d.yaw, 0.0, 0.0), tf::Vector3(message->pose_6d.x, message->pose_6d.y, 0.0));
	vo_stamp_ = tf::Time(message->timestamp);
	vo_time_  = tf::Time::now();

	my_filter_.addMeasurement(tf::StampedTransform(vo_meas_.inverse(), vo_stamp_, "/base_footprint", "/vo"), vo_covariance_);

	// activate vo
	if (!vo_active_) {
		if (!vo_initializing_){
			vo_initializing_ = true;
			vo_init_stamp_ = vo_stamp_;
			printf("Initializing Vo sensor\n");
		}
		if (filter_stamp_ >= vo_init_stamp_){
			vo_active_ = true;
			vo_initializing_ = false;
			printf("Vo sensor activated\n");
		}
		else printf("Waiting to activate VO, because VO measurements are still %f sec in the future.",
				(vo_init_stamp_ - filter_stamp_).toSec());
	}
}

void compute_visual_search_correspondences(carmen_neural_global_localizer_globalpos_message *previous_message, carmen_neural_global_localizer_globalpos_message *message, carmen_visual_correspondence_t* correspondences, double distance_between_poses)
{
	carmen_position_t right_image_point;
	carmen_position_t left_image_point;
	char observation_frame[1024];
	char measurement_frame[1024];

	visual_search_training_message.reference_points_size = 1;
	visual_search_training_message.reference_image_size = ekf_odometry_visual_search_image_width_g * ekf_odometry_visual_search_image_height_g * 3;
	resize_image(visual_search_training_message.reference_image, previous_message->output_image, camera_instance.width, camera_instance.height, 2);
	visual_search_training_message.timestamp = carmen_get_time();
	visual_search_training_message.host = carmen_get_host();

	visual_search_testing_message.associated_points_size = 0;
	visual_search_testing_message.associated_image_size = ekf_odometry_visual_search_image_width_g * ekf_odometry_visual_search_image_height_g * 3;
	resize_image(visual_search_testing_message.associated_image, message->test_image, camera_instance.width, camera_instance.height, 2);
	visual_search_testing_message.timestamp = carmen_get_time();
	visual_search_testing_message.host = carmen_get_host();
	visual_search_testing_message.scale = 1.0;

	landmark_measurement_stamp_ = landmark_observation_stamp_ = tf::Time(message->timestamp);

	for (int i = 0; i < ekf_odometry_visual_search_num_landmarks_g; i++)
	{
		double x, y;

		x = previous_message->saliencies[i].coordinates.x / 2;
		y = previous_message->saliencies[i].coordinates.y / 2;

		right_image_point.x = x;
		right_image_point.y = ekf_odometry_visual_search_image_height_g - y;
		visual_search_training_message.reference_points = &right_image_point;

		carmen_visual_search_thin_output_training_message * output_training =
				carmen_visual_search_thin_query_training_message(&visual_search_training_message, 10.0);

		if (output_training)
		{
			carmen_visual_search_thin_output_message * output_testing =
					carmen_visual_search_thin_query_output_message(&visual_search_testing_message, 10.0);

			if (output_testing)
			{

				left_image_point.x = output_testing->saccade_point.x;
				left_image_point.y = output_testing->saccade_point.y;

				if (left_image_point.x > 0.0 && left_image_point.y > 0.0)
				{
					double disparity = 0.0;
					carmen_position_t stereo_position;
					carmen_vector_3D_p point3D;

					x = left_image_point.x * 2;
					y = (ekf_odometry_visual_search_image_height_g - left_image_point.y) * 2;

					stereo_position.x = correspondences[i].coordinate.x = x;
					stereo_position.y = correspondences[i].coordinate.y = y;
					disparity = (double) message->test_disparity[correspondences[i].coordinate.y * camera_instance.width + correspondences[i].coordinate.x];

					point3D = reproject_single_point_to_3D(&camera_instance, stereo_position, disparity);

					if(point3D != NULL)
					{
						correspondences[i].pose.x = point3D->x;
						correspondences[i].pose.y = point3D->y;
						correspondences[i].pose.z = point3D->z;

						free(point3D);

						tf::StampedTransform measurement_map_pose;
						tf::StampedTransform observation_map_pose;
						tf::Transform world_to_car_pose;

						world_to_car_pose.setOrigin(tf::Vector3(previous_message->pose.position.x, previous_message->pose.position.y, previous_message->pose.position.z + car_pose_g.position.z));
						world_to_car_pose.setRotation(tf::Quaternion(previous_message->pose.orientation.yaw, previous_message->pose.orientation.pitch, previous_message->pose.orientation.roll));

						tf::StampedTransform world_to_car_transform1(world_to_car_pose, tf::Time(0), "/world", "/car");
						transformer.setTransform(world_to_car_transform1, "world_to_car_transform");

						transformer.lookupTransform("/world", "/camera", tf::Time(0), measurement_map_pose);

						world_to_car_pose.setOrigin(tf::Vector3(0.0, 0.0, car_pose_g.position.z));
						world_to_car_pose.setRotation(tf::Quaternion(0.0, 0.0, 0.0));

						tf::StampedTransform world_to_car_transform2(world_to_car_pose, tf::Time(0), "/world", "/car");
						transformer.setTransform(world_to_car_transform2, "world_to_car_transform");

						transformer.lookupTransform("/world", "/camera", tf::Time(0), observation_map_pose);

						//TODO: comparar a distancia com cada pose em seu frame de referencia
						tf::Transform measurement_transform = tf::Transform(tf::Quaternion(0.0, 0.0, 0.0), tf::Vector3(previous_message->saliencies[i].pose.x, previous_message->saliencies[i].pose.y, 0.0));
						tf::Transform observation_transform = tf::Transform(tf::Quaternion(0.0, 0.0, 0.0), tf::Vector3(correspondences[i].pose.x, correspondences[i].pose.y, 0.0));

						double distance_between_correspondences = sqrt((measurement_transform.getOrigin().x() - observation_transform.getOrigin().x()) * (measurement_transform.getOrigin().x() - observation_transform.getOrigin().x()) + (measurement_transform.getOrigin().y() - observation_transform.getOrigin().y()) * (measurement_transform.getOrigin().y() - observation_transform.getOrigin().y()));

						message_to_publish.saliencies[i].x = measurement_transform.getOrigin().x();
						message_to_publish.saliencies[i].y = measurement_transform.getOrigin().y();
						message_to_publish.saliencies[i].z = measurement_transform.getOrigin().z();

						message_to_publish.correspondences[i].x = observation_transform.getOrigin().x();
						message_to_publish.correspondences[i].y = observation_transform.getOrigin().y();
						message_to_publish.correspondences[i].z = observation_transform.getOrigin().z();

						correspondences[i].is_valid = 1;
						measurement_transform = measurement_transform * measurement_map_pose;
						observation_transform = observation_transform * observation_map_pose;

						sprintf(measurement_frame, "/landmark_measurement%d", i);
						sprintf(observation_frame, "/landmark_observation%d", i);

						if(distance_between_correspondences < (2.0 * distance_between_poses))
						{
							landmark_covariance_(1, 1) = ekf_odometry_visual_search_landmark_coeff_g * pow(2.5 + distance_between_poses, 2.0);
							landmark_covariance_(2, 2) = 10.0 * ekf_odometry_visual_search_landmark_coeff_g * pow(carmen_degrees_to_radians(25.0), 2.0);
							my_filter_.addMeasurement(tf::StampedTransform(measurement_transform.inverse(), landmark_measurement_stamp_, "/base_footprint", measurement_frame), landmark_covariance_);

							landmark_covariance_(1, 1) = ekf_odometry_visual_search_landmark_coeff_g * pow(2.5 + distance_between_poses, 2.0);
							landmark_covariance_(2, 2) = 10.0 * ekf_odometry_visual_search_landmark_coeff_g * pow(carmen_degrees_to_radians(25.0), 2.0);
							my_filter_.addMeasurement(tf::StampedTransform(observation_transform.inverse(), landmark_observation_stamp_, "/base_footprint", observation_frame), landmark_covariance_);
						}
					}
					else
					{
						correspondences[i].is_valid = 0;

						message_to_publish.correspondences[i].x = 0.0;
						message_to_publish.correspondences[i].y = 0.0;
						message_to_publish.correspondences[i].z = 0.0;
					}
				}
				free(output_testing);
			}
			free(output_training);
		}
	}

	//save_correspondence_images(previous_message->output_image, previous_message->saliencies, message->test_image, correspondences);
}


void set_neural_global_covariances(double covariance_value)
{
	neural_globalpos_covariance_(1, 1) = neural_globalpos_covariance_(2, 2) = neural_globalpos_covariance_(3, 3) = neural_globalpos_covariance_(4, 4) = neural_globalpos_covariance_(5, 5) = neural_globalpos_covariance_(6, 6) = covariance_value;
}

carmen_neural_global_localizer_globalpos_message previous_message;
int neural_globalpos_first_time = 1;

void ekf_odometry_neural_globalpos_handler (carmen_neural_global_localizer_globalpos_message* message)
{
	tf::StampedTransform neural_globalpos_to_world;
	double distance_outlier, distance_variance, variance_value;

	carmen_verbose("NGL\n");
	neural_globapos_callback_counter_++;

	carmen_point_t previous, current, estimated;
	previous.x = previous_message.pose.position.x;
	previous.y = previous_message.pose.position.y;

	current.x = message->pose.position.x;
	current.y = message->pose.position.y;

	estimated.x = message_to_publish.estimated_pose.position.x;
	estimated.y = message_to_publish.estimated_pose.position.y;

	distance_outlier = carmen_distance(&current, &estimated);
	distance_variance = carmen_distance(&previous, &estimated);

	variance_value = pow(2.5 + (2.0 * distance_variance), 2.0);
	set_neural_global_covariances(ekf_odometry_neural_globalpos_coeff_g * variance_value);

	if(neural_globalpos_first_time)
	{
		neural_globalpos_first_time = 0;
		variance_value = pow(0.01, 2);
	}

	if((distance_outlier < ekf_odometry_max_distance_outlier_g) || !my_filter_.isInitialized())
	{
		neural_globalpos_meas_ = tf::Transform(tf::Quaternion(message->pose.orientation.yaw, 0.0, 0.0),
				tf::Vector3(message->pose.position.x, message->pose.position.y, 0.0));
		previous_message = *message;

		if(ekf_odometry_use_landmark_correction_g && distance_variance < (ekf_odometry_max_distance_outlier_g / 2.0))
			compute_visual_search_correspondences(&previous_message, message, visual_search_correspondences, distance_variance);
	}
	else
		set_neural_global_covariances(100.0 * variance_value);

	message_to_publish.neural_global_pose.position.x = message->pose.position.x;
	message_to_publish.neural_global_pose.position.y = message->pose.position.y;
	message_to_publish.neural_global_pose.position.z = 0.0;
	message_to_publish.neural_global_pose.orientation.yaw = message->pose.orientation.yaw;
	message_to_publish.neural_global_pose.orientation.pitch = 0.0;
	message_to_publish.neural_global_pose.orientation.roll = 0.0;
	message_to_publish.timestamp = message->timestamp;

	neural_globalpos_stamp_ = tf::Time(message->timestamp);
	neural_globalpos_time_  = tf::Time::now();
	my_filter_.addMeasurement(tf::StampedTransform(neural_globalpos_meas_.inverse(), neural_globalpos_stamp_, "/base_footprint", "/neural_globalpos"), neural_globalpos_covariance_);

	// activate
	if (!neural_globalpos_active_) {
		if (!neural_globalpos_initializing_){
			neural_globalpos_initializing_ = true;
			neural_globalpos_init_stamp_ = neural_globalpos_stamp_;
			printf("Initializing Neural Globalpos sensor\n");
		}
		if (filter_stamp_ >= neural_globalpos_init_stamp_){
			neural_globalpos_active_ = true;
			neural_globalpos_initializing_ = false;
			printf("Neural Globalpos sensor activated\n");
		}
		else printf("Waiting to activate Neural Globalpos, because Neural Globalpos measurements are still %f sec in the future.\n",
				(neural_globalpos_init_stamp_ - filter_stamp_).toSec());
	}
}

static void
spin(void *clientData __attribute__ ((unused)), unsigned long currentTime __attribute__ ((unused)), unsigned long scheduledTime __attribute__ ((unused)))
{
	IPC_RETURN_TYPE err;

	// initial value for filter stamp; keep this stamp when no sensors are active
	filter_stamp_ = tf::Time::now();
	carmen_verbose("Spin function at time %f\n", filter_stamp_.toSec());

	if ((vo_active_ || vo_initializing_) &&
			(tf::Time::now() - vo_time_).toSec() > ekf_odometry_sensor_timeout_g){
		vo_active_ = false;  vo_initializing_ = false;
		printf("VO sensor not active any more\n");
	}

	if ((neural_globalpos_active_ || neural_globalpos_initializing_) &&
			(tf::Time::now() - neural_globalpos_time_).toSec() > ekf_odometry_sensor_timeout_g){
		neural_globalpos_active_ = false;  neural_globalpos_initializing_ = false;
		printf("Neural Globalpos sensor not active any more\n");
	}

	// only update filter when one of the sensors is active
	if (vo_active_ || neural_globalpos_active_){

		// update filter at time where all sensor measurements are available
		if (neural_globalpos_active_)
			filter_stamp_ = std::min(filter_stamp_, neural_globalpos_stamp_);
		if (vo_active_)
			filter_stamp_ = std::min(filter_stamp_, vo_stamp_);

		// update filter
		if ( my_filter_.isInitialized() )  {
			if (my_filter_.update(neural_globalpos_active_, vo_active_, ekf_odometry_use_landmark_correction_g, filter_stamp_)){

				// output most recent estimate and relative covariance
				my_filter_.getEstimate(&message_to_publish);

				err = IPC_publishData(CARMEN_EKF_ODOMETRY_ODOMETRY_MESSAGE_NAME, &message_to_publish);
				carmen_test_ipc_exit(err, "Could not publish", CARMEN_EKF_ODOMETRY_ODOMETRY_MESSAGE_NAME);

//				localize_globalpos_message.pose = message_to_publish.estimated_pose;
//				localize_globalpos_message.globalpos.x = message_to_publish.estimated_pose.position.x;
//				localize_globalpos_message.globalpos.y = message_to_publish.estimated_pose.position.y;
//				localize_globalpos_message.globalpos.theta = message_to_publish.estimated_pose.orientation.yaw;
//				localize_globalpos_message.timestamp = message_to_publish.timestamp;
//
//				err = IPC_publishData(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, &localize_globalpos_message);
//				carmen_test_ipc_exit(err, "Could not publish", CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME);

				//double vo_yaw, neural_globalpos_yaw, p, r;
				//tf::Matrix3x3(vo_meas_.getRotation()).getRPY(r, p, vo_yaw);
				//tf::Matrix3x3(neural_globalpos_meas_.getRotation()).getRPY(r, p, neural_globalpos_yaw);


				//fd_result = fopen("results.txt", "a");

				//fprintf(fd_result, "vo: %8.5f %8.5f %8.5f; ng: %8.5f %8.5f %8.5f; ekf: %8.5f %8.5f %8.5f %lf; gpos: %8.5f %8.5f %8.5f %lf\n", vo_meas_.getOrigin().x(), vo_meas_.getOrigin().y(), vo_yaw, neural_globalpos_meas_.getOrigin().x(), neural_globalpos_meas_.getOrigin().y(), neural_globalpos_yaw,
				//		message_to_publish.estimated_pose.position.x, message_to_publish.estimated_pose.position.y, message_to_publish.estimated_pose.orientation.yaw, message_to_publish.timestamp, localize_globalpos_message.globalpos.x, localize_globalpos_message.globalpos.y, localize_globalpos_message.globalpos.theta, localize_globalpos_message.timestamp);

				//fclose(fd_result);
			}
		}

		if (neural_globalpos_active_ && !my_filter_.isInitialized()){
			my_filter_.initialize(neural_globalpos_meas_, neural_globalpos_stamp_);
			printf("Kalman filter initialized with neural_global measurement\n");
		}
		else if ( vo_active_ && !ekf_odometry_neural_globalpos_used_g && !my_filter_.isInitialized()){
			my_filter_.initialize(vo_meas_, vo_stamp_);
			printf("Kalman filter initialized with vo measurement\n");
		}
	}
}


void 
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("ekf_odometry: disconnected.\n");

		exit(0);
	}
}

static int read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] =
	{
			{(char *) "ekf_odometry",    (char *) "camera", 	CARMEN_PARAM_INT, &ekf_odometry_camera_g, 0, NULL},
			{(char *) "ekf_odometry",    (char *) "sensor_timeout", 	CARMEN_PARAM_DOUBLE, &ekf_odometry_sensor_timeout_g, 0, NULL},
			{(char *) "ekf_odometry",    (char *) "frequency", 	CARMEN_PARAM_DOUBLE, &ekf_odometry_frequency_g, 0, NULL},
			{(char *) "ekf_odometry",    (char *) "vodom_used", 	CARMEN_PARAM_ONOFF, &ekf_odometry_vodom_used_g, 0, NULL},
			{(char *) "ekf_odometry",    (char *) "neural_globalpos_used", 	CARMEN_PARAM_ONOFF, &ekf_odometry_neural_globalpos_used_g, 0, NULL},
			{(char *) "ekf_odometry",    (char *) "use_landmark_correction", 	CARMEN_PARAM_ONOFF, &ekf_odometry_use_landmark_correction_g, 0, NULL},

			{(char *) "ekf_odometry",    (char *) "prediction_covariance", 	CARMEN_PARAM_DOUBLE, &ekf_odometry_prediction_covariance_g, 0, NULL},
			{(char *) "ekf_odometry",    (char *) "max_distance_outlier", 	CARMEN_PARAM_DOUBLE, &ekf_odometry_max_distance_outlier_g, 0, NULL},

			{(char *) "ekf_odometry",    (char *) "visual_search_image_width", 	CARMEN_PARAM_INT, &ekf_odometry_visual_search_image_width_g, 0, NULL},
			{(char *) "ekf_odometry",    (char *) "visual_search_image_height", 	CARMEN_PARAM_INT, &ekf_odometry_visual_search_image_height_g, 0, NULL},
			{(char *) "ekf_odometry",    (char *) "visual_search_num_landmarks", 	CARMEN_PARAM_INT, &ekf_odometry_visual_search_num_landmarks_g, 0, NULL},
			{(char *) "ekf_odometry",    (char *) "visual_search_landmark_coeff", 	CARMEN_PARAM_DOUBLE, &ekf_odometry_visual_search_landmark_coeff_g, 0, NULL},
			{(char *) "ekf_odometry",    (char *) "neural_globalpos_coeff", 	CARMEN_PARAM_DOUBLE, &ekf_odometry_neural_globalpos_coeff_g, 0, NULL},

			{(char *) "car", 			 (char *) "x", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.position.x), 0, NULL},
			{(char *) "car", 			 (char *) "y", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.position.y), 0, NULL},
			{(char *) "car", 			 (char *) "z", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.position.z), 0, NULL},
			{(char *) "car", 			 (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.orientation.yaw), 0, NULL},
			{(char *) "car", 			 (char *) "pitch", CARMEN_PARAM_DOUBLE, &(car_pose_g.orientation.pitch), 0, NULL},
			{(char *) "car", 			 (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.orientation.roll), 0, NULL},

			{(char *) "sensor_board_1",  (char *) "x", 	CARMEN_PARAM_DOUBLE, &(board_pose_g.position.x), 0, NULL},
			{(char *) "sensor_board_1",  (char *) "y", 	CARMEN_PARAM_DOUBLE, &(board_pose_g.position.y), 0, NULL},
			{(char *) "sensor_board_1",  (char *) "z", 	CARMEN_PARAM_DOUBLE, &(board_pose_g.position.z), 0, NULL},
			{(char *) "sensor_board_1",  (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(board_pose_g.orientation.yaw), 0, NULL},
			{(char *) "sensor_board_1",  (char *) "pitch", CARMEN_PARAM_DOUBLE, &(board_pose_g.orientation.pitch), 0, NULL},
			{(char *) "sensor_board_1",  (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(board_pose_g.orientation.roll), 0, NULL}
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	camera_instance = get_stereo_instance(ekf_odometry_camera_g, -1, -1);

	return 0;
}

void initialize_transformations_tree()
{
	tf::Transform board_to_camera_pose;
	tf::Transform car_to_board_pose;
	tf::Transform world_to_car_pose;

	tf::Time::init();
	filter_stamp_ = tf::Time::now();

	// initial car pose with respect to the world
	world_to_car_pose.setOrigin(tf::Vector3(car_pose_g.position.x, car_pose_g.position.y, car_pose_g.position.z));
	world_to_car_pose.setRotation(tf::Quaternion(car_pose_g.orientation.yaw, car_pose_g.orientation.pitch, car_pose_g.orientation.roll));
	tf::Transform rotaciona;

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

void ekf_odometry_define_messages()
{
	carmen_ekf_odometry_define_messages();

	carmen_visual_search_thin_define_message_query_test();
	carmen_visual_search_thin_define_message_query_train();
}

void ekf_odometry_initialize()
{
	initialize_transformations_tree();
}

int 
main(int argc, char **argv) 
{
	/* Connect to IPC Server */
	carmen_ipc_initialize(argc, argv);

	/* Check the param server version */
	carmen_param_check_version(argv[0]);

	/* read parameters from .ini file */
	read_parameters(argc, argv);

	/* define messages */
	ekf_odometry_define_messages();
	carmen_localize_ackerman_define_globalpos_messages();

	/* initialize system variables */
	ekf_odometry_initialize();

	/* ekf filter loop */
	carmen_ipc_addPeriodicTimer(1.0 / std::max(1.0, ekf_odometry_frequency_g), spin, NULL);

	// subscribe to vo messages
	if (ekf_odometry_vodom_used_g){
		printf("VO sensor can be used\n");
		carmen_visual_odometry_subscribe_pose6d_message(NULL, (carmen_handler_t) ekf_odometry_visual_odometry_handler, CARMEN_SUBSCRIBE_LATEST);
	}
	else printf("VO sensor will NOT be used\n");

	if (ekf_odometry_neural_globalpos_used_g){
		printf("Neural Globalpos sensor can be used\n");
		carmen_neural_global_localizer_subscribe_globalpos_message(NULL, (carmen_handler_t) ekf_odometry_neural_globalpos_handler, CARMEN_SUBSCRIBE_LATEST);
	}
	else printf("Neural Globalpos will NOT be used\n");

	if (ekf_odometry_use_landmark_correction_g)
		printf("Landmark Correction can be used\n");
	else
		printf("Landmark Correction will NOT be used\n");

	carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) ekf_odometry_localizer_ackerman_globapos_handler, CARMEN_SUBSCRIBE_LATEST);

	visual_search_training_message.reference_image = (unsigned char*) calloc (320 * 182 * 3, sizeof(unsigned char));
	visual_search_testing_message.associated_image = (unsigned char*) calloc (320 * 182 * 3, sizeof(unsigned char));

	/* Register shutdown cleaner handler */
	signal(SIGINT, shutdown_module);

	/* Loop forever waiting for messages */
	carmen_ipc_dispatch();

	return (0);
}
