#include <carmen/carmen.h>
#include <carmen/stereo_util.h>
#include <carmen/visual_odometry_interface.h>
#include <carmen/visual_odometry_messages.h>

#include <viso_stereo.h>
#include <vector>

#include <carmen/rotation_geometry.h>
#include <tf.h>


static int camera;
static int bumblebee_basic_width;
static int bumblebee_basic_height;
static int robot_publish_odometry;
static int visual_odometry_is_global_pos;

static int visual_odometry_publish_odometry;

Matrix visual_odometry_pose_6d_transformation_matrix;
VisualOdometryStereo *viso = NULL;
VisualOdometryStereo::parameters viso_param;
std::vector<Matcher::p_match> viso_matches;
std::vector<int> viso_inliers_indices;

carmen_visual_odometry_pose6d_message odometry_msg;
carmen_visual_odometry_image_message image_msg;
stereo_util stereo_instance;

tf::Transformer transformer(false);
tf::StampedTransform g_carmen_to_visual_odometry_transform;

unsigned char* left_image;
unsigned char* right_image;


static carmen_pose_3D_t camera_pose;
static carmen_pose_3D_t sensor_board_pose;

static double distance_between_front_and_rear_axles;

static carmen_pose_3D_t previous_pose = {{0.0,0.0,0.0},{0.0,0.0,0.0}};

static char* car_tf_name = (char*)"/car";
static char* camera_tf_name = (char*)"/camera";
static char* board_tf_name = (char*)"/board";

static double previous_timestamp = 0.0;

static double maximum_acceleration_forward;

static double visual_odometry_phi_multiplier;
static double visual_odometry_phi_bias;
static double visual_odometry_v_multiplier;
//static double visual_odometry_publish;

static carmen_base_ackerman_odometry_message last_base_ackerman_odometry_message;


void 
convert_image_rgb_to_gray(unsigned char *lsrc, unsigned char *ldst, unsigned char *rsrc, unsigned char *rdst, int width, int height, int cutline)
{
	int i;
	int n = width * height;
	int step;

	for (i = 0; i < n; i++)
	{
		if (i > cutline * width)
		{
			step = 3 * i;

			ldst[i]= 0.30 * lsrc[step] + 0.59 * lsrc[step + 1] + 0.11 * lsrc[step + 2];
			rdst[i]= 0.30 * rsrc[step] + 0.59 * rsrc[step + 1] + 0.11 * rsrc[step + 2];
		}
		else
		{
			ldst[i] = 0;
			rdst[i] = 0;
		}
	}
}


void 
visual_odometry_copy_inliers(int* vector_inliers, std::vector<Matcher::p_match> _matches, std::vector<int> _inliers)
{
	for (unsigned int i=0; i<_inliers.size(); i++)
	{
		vector_inliers[4*i] = _matches[_inliers[i]].u1p;
		vector_inliers[4*i+1] = _matches[_inliers[i]].v1p;
		vector_inliers[4*i+2]= _matches[_inliers[i]].u1c;
		vector_inliers[4*i+3]= _matches[_inliers[i]].v1c;
	}
}


tf::Transform 
convert_matrix_to_tf_transform(Matrix transformation_matrix)
{
	tf::Transform transformation;

	double vo_x = transformation_matrix.val[0][3];
	double vo_y = transformation_matrix.val[1][3];
	double vo_z = transformation_matrix.val[2][3];
	
	transformation.setOrigin(tf::Vector3(vo_x, vo_y, vo_z));		// x, y, z;

	double xx = transformation_matrix.val[0][0];
	double xy = transformation_matrix.val[0][1];
	double xz = transformation_matrix.val[0][2];
	double yx = transformation_matrix.val[1][0];
	double yy = transformation_matrix.val[1][1];
	double yz = transformation_matrix.val[1][2];
	double zx = transformation_matrix.val[2][0];
	double zy = transformation_matrix.val[2][1];
	double zz = transformation_matrix.val[2][2];

	btMatrix3x3 bt_matrix(xx, xy, xz,  yx, yy, yz,  zx, zy, zz);
	btQuaternion quaternion;
	bt_matrix.getRotation(quaternion);
	transformation.setRotation(quaternion);

	return transformation;
}


carmen_6d_point 
get_carmen_pose6D_from_matrix(Matrix visual_odometry_transformation_matrix)
{
	// See: http://www.ros.org/wiki/tf
	tf::Transform visual_odometry_pose;
	tf::Transform carmen_pose;
	carmen_6d_point carmen_pose_6d;

	// convert the visual odometry output matrix to the tf::Transform type.
	visual_odometry_pose = convert_matrix_to_tf_transform(visual_odometry_transformation_matrix);

	// compute the current visual odometry pose with respect to the carmen coordinate system
	carmen_pose = g_carmen_to_visual_odometry_transform * visual_odometry_pose * g_carmen_to_visual_odometry_transform.inverse();

	double yaw, pitch, roll;
	tf::Matrix3x3(carmen_pose.getRotation()).getRPY(roll, pitch, yaw);

	carmen_pose_6d.x = carmen_pose.getOrigin().x();
	carmen_pose_6d.y = carmen_pose.getOrigin().y();
	carmen_pose_6d.z = carmen_pose.getOrigin().z();

	carmen_pose_6d.yaw = yaw;
	carmen_pose_6d.pitch = pitch;
	carmen_pose_6d.roll = roll;
	
/*	tf::Matrix3x3(visual_odometry_pose.getRotation()).getRPY(roll, pitch, yaw);
	printf("x: % 6.2lf,   y: % 6.2lf,   z: % 6.2lf,   yaw: % 6.2lf,   pitch: % 6.2lf,   roll: % 6.2lf  -  Visual O. Original\n",
				visual_odometry_pose.getOrigin().x(), visual_odometry_pose.getOrigin().y(), visual_odometry_pose.getOrigin().z(),
				carmen_radians_to_degrees(yaw),
				carmen_radians_to_degrees(pitch),
				carmen_radians_to_degrees(roll));

*/	return carmen_pose_6d;
}


static tf::Vector3 
carmen_vector3_to_tf_vector3(carmen_vector_3D_t carmen_vector)
{
	tf::Vector3 tf_vector(carmen_vector.x, carmen_vector.y, carmen_vector.z);

	return tf_vector;
}


static tf::Quaternion 
carmen_rotation_to_tf_quaternion(carmen_orientation_3D_t carmen_orientation)
{
	tf::Quaternion tf_quat(carmen_orientation.yaw, carmen_orientation.pitch, carmen_orientation.roll);

	return tf_quat;
}


static carmen_vector_3D_t 
tf_vector3_to_carmen_vector3(tf::Vector3 tf_vector)
{
	carmen_vector_3D_t carmen_vector;
	carmen_vector.x = tf_vector.x();
	carmen_vector.y = tf_vector.y();
	carmen_vector.z = tf_vector.z();
	
	return carmen_vector;
}


static carmen_orientation_3D_t 
get_carmen_orientation_from_tf_transform(tf::Transform transform)
{
	carmen_orientation_3D_t orientation;
	tf::Matrix3x3(transform.getRotation()).getEulerYPR(orientation.yaw, orientation.pitch, orientation.roll);
	
	return orientation;
}


static carmen_pose_3D_t 
tf_transform_to_carmen_pose_3D(tf::Transform transform)
{
	carmen_pose_3D_t pose;

	pose.position = tf_vector3_to_carmen_vector3(transform.getOrigin());
	pose.orientation = get_carmen_orientation_from_tf_transform(transform);
	
	return pose;
}


static tf::Transform 
carmen_pose_3D_to_tf_transform(carmen_pose_3D_t carmen_pose_3D)
{
	tf::Transform transformation;

	transformation.setOrigin(carmen_vector3_to_tf_vector3(carmen_pose_3D.position));
	transformation.setRotation(carmen_rotation_to_tf_quaternion(carmen_pose_3D.orientation));

	return transformation;
}


static carmen_pose_3D_t 
calculate_delta_pose(carmen_pose_3D_t p1, carmen_pose_3D_t p2)
{
	carmen_pose_3D_t delta_pose;

	delta_pose.position.x = p1.position.x - p2.position.x;
	delta_pose.position.y = p1.position.y - p2.position.y;
	delta_pose.position.z = p1.position.z - p2.position.z;
	delta_pose.orientation.roll = carmen_normalize_theta(p1.orientation.roll - p2.orientation.roll);
	delta_pose.orientation.pitch = carmen_normalize_theta(p1.orientation.pitch - p2.orientation.pitch);
	delta_pose.orientation.yaw = carmen_normalize_theta(p1.orientation.yaw - p2.orientation.yaw);

	return delta_pose;
}


static carmen_pose_3D_t 
get_carmen_pose_from_visual_odometry_message(carmen_visual_odometry_pose6d_message* visual_odometry_message)
{
	carmen_pose_3D_t pose;

	pose.position.x = visual_odometry_message->pose_6d.x;
	pose.position.y = visual_odometry_message->pose_6d.y;
	pose.position.z = visual_odometry_message->pose_6d.z;
	pose.orientation.roll = carmen_normalize_theta(visual_odometry_message->pose_6d.roll);
	pose.orientation.pitch = carmen_normalize_theta(visual_odometry_message->pose_6d.pitch);
	pose.orientation.yaw = carmen_normalize_theta(visual_odometry_message->pose_6d.yaw);

	return pose;
}


static void 
compute_v_and_phi(carmen_visual_odometry_pose6d_message *visual_odometry_message)
{	
	if (previous_timestamp == 0.0)
		previous_timestamp = visual_odometry_message->timestamp;

	double delta_t = visual_odometry_message->timestamp - previous_timestamp;
	
	carmen_pose_3D_t current_pose = get_carmen_pose_from_visual_odometry_message(visual_odometry_message);
	carmen_pose_3D_t delta_pose = calculate_delta_pose(current_pose, previous_pose);

	double yaw2 = atan2(delta_pose.position.y, delta_pose.position.x);

	static double yaw_velocity = 0.0;

	if (fabs(delta_t) > 3.0)
	{
		visual_odometry_message->v = 0.0;
		visual_odometry_message->phi = 0.0;
	}
	else if (delta_t > 0.0)
	{
		visual_odometry_message->v += 0.5 * ((sqrt(delta_pose.position.x * delta_pose.position.x + delta_pose.position.y * delta_pose.position.y) / delta_t) - visual_odometry_message->v);
				
		if (fabs(carmen_normalize_theta(current_pose.orientation.yaw - yaw2)) > M_PI/4.0)
		{
			visual_odometry_message->v = -visual_odometry_message->v;	//@@@ Isso parou de funcionar com o visual odometry novo. Mas acho que consertei.. Checar.
		}

		double yaw_velocity_raw = delta_pose.orientation.yaw / delta_t;
//		yaw_velocity += 0.25 * (yaw_velocity_raw - yaw_velocity);
		yaw_velocity = yaw_velocity_raw;

		double L = distance_between_front_and_rear_axles;
		if (fabs(visual_odometry_message->v) > 0.1)
		{
			visual_odometry_message->phi = atan2(L * yaw_velocity, fabs(visual_odometry_message->v));
		}
		else
		{
			visual_odometry_message->phi = 0.0;
		}
	}	
	else
	{
		visual_odometry_message->v = 0.0;
		visual_odometry_message->phi = 0.0;
	}
	
	previous_pose = current_pose;
	previous_timestamp = visual_odometry_message->timestamp;
}


carmen_pose_3D_t
get_carmen_pose_3D_from_carmen_6d_point(carmen_6d_point pose_6d)
{
	carmen_pose_3D_t pose;
	
	pose.position.x = pose_6d.x;
	pose.position.y = pose_6d.y;
	pose.position.z = pose_6d.z;
	pose.orientation.roll = pose_6d.roll;
	pose.orientation.pitch = pose_6d.pitch;
	pose.orientation.yaw = pose_6d.yaw;
	
	return pose;
}


carmen_6d_point
get_carmen_6d_point_from_carmen_pose_3D(carmen_pose_3D_t pose)
{
	carmen_6d_point pose_6d;
	
	pose_6d.x     = pose.position.x;
	pose_6d.y     = pose.position.y;
	pose_6d.z     = pose.position.z;
	pose_6d.roll  = pose.orientation.roll;
	pose_6d.pitch = pose.orientation.pitch;
	pose_6d.yaw   = pose.orientation.yaw;
	
	return pose_6d;
}


static carmen_pose_3D_t 
get_car_pose_from_visual_odometry_pose(carmen_pose_3D_t visual_odometry_pose_from_message)
{
	tf::StampedTransform camera_to_car;
	transformer.lookupTransform(camera_tf_name, car_tf_name, tf::Time(0), camera_to_car);

 	tf::Transform visual_odometry_pose = carmen_pose_3D_to_tf_transform(visual_odometry_pose_from_message);
	
	tf::Transform car_tf_pose = visual_odometry_pose * camera_to_car;
	
	carmen_pose_3D_t car_pose = tf_transform_to_carmen_pose_3D(car_tf_pose);

	return car_pose;
}


void 
visual_odometry_assembly_odometry_message(carmen_6d_point pose_6d, double timestamp, char *host)
{
	odometry_msg.pose_6d = get_carmen_6d_point_from_carmen_pose_3D(get_car_pose_from_visual_odometry_pose(get_carmen_pose_3D_from_carmen_6d_point(pose_6d)));
	odometry_msg.timestamp = timestamp;
	compute_v_and_phi(&odometry_msg); 
	odometry_msg.host = host;
}


void 
visual_odometry_assembly_image_message(carmen_6d_point pose_6d, unsigned char *image, int height, int width, double timestamp, char *host)
{
	image_msg.pose_6d = pose_6d;

	image_msg.image_size = height * width;
	image_msg.left_image = image;

	image_msg.inliers_size = viso->getNumberOfInliers() * 4;
	image_msg.inliers = (int*) malloc (image_msg.inliers_size * sizeof(int));

	image_msg.timestamp = timestamp;
	image_msg.host = host;
}


static void
initialize_transformations()
{
	tf::Time::init();

	tf::Transform camera_position_on_board;
	camera_position_on_board.setOrigin(carmen_vector3_to_tf_vector3(camera_pose.position));
	camera_position_on_board.setRotation(carmen_rotation_to_tf_quaternion(camera_pose.orientation));
	tf::StampedTransform board_to_camera_transform(camera_position_on_board, tf::Time(0), board_tf_name, camera_tf_name);
	transformer.setTransform(board_to_camera_transform, "board_to_camera_transform");

	tf::Transform board_position_on_car;
	board_position_on_car.setOrigin(carmen_vector3_to_tf_vector3(sensor_board_pose.position));
	board_position_on_car.setRotation(carmen_rotation_to_tf_quaternion(sensor_board_pose.orientation));
	tf::StampedTransform car_to_board_transform(board_position_on_car, tf::Time(0), car_tf_name, board_tf_name);
	transformer.setTransform(car_to_board_transform, "car_to_board_transform");
}


void
initialize_pose_6d_transformation_matrix()
{
	visual_odometry_pose_6d_transformation_matrix = Matrix::eye(4);

	// See: http://www.ros.org/wiki/tf
	tf::Transform camera_to_visual_odometry_pose;
	tf::Transform world_to_camera_pose;
	tf::Transform world_to_carmen_pose;

	tf::Time::init();

	// camera pose with respect to the world
	world_to_camera_pose.setOrigin(tf::Vector3(0.0, 0.0, 0.0));					// x, y, z;
	//world_to_camera_pose.setRotation(tf::Quaternion(0.0, atan2(4.30, 42.25), 0.0)); 		// yaw, pitch, roll
	world_to_camera_pose.setRotation(tf::Quaternion(0.0, 0.0, 0.0)); 				// yaw, pitch, roll

	// create a time stamped transformation that defines the camera position with respect to the world
	tf::StampedTransform world_to_camera_transform(world_to_camera_pose, tf::Time(0), "/world", "/camera");
	// add a link to the tf tree with the camera stamped transformation
	transformer.setTransform(world_to_camera_transform, "world_to_camera_transform");


	// visual odometry pose with respect to the camera
	// (the rotation comes from parent coordinate system (camera) to the child coordinate system (visual_odometry))
	// following yaw, pitch, roll angles order).
	camera_to_visual_odometry_pose.setOrigin(tf::Vector3(0.0, 0.0, 0.0));				// x, y, z;
	camera_to_visual_odometry_pose.setRotation(tf::Quaternion(-M_PI / 2.0, 0.0, -M_PI / 2.0));	// yaw, pitch, roll

	// create a time stamped transformation that defines the visual odometry position with respect to the camera
	tf::StampedTransform camera_to_visual_odometry_transform(camera_to_visual_odometry_pose, tf::Time(0), "/camera", "/visual_odometry");
	// add a link to the tf tree with the camera_to_visual_odometry_transform stamped transformation
	transformer.setTransform(camera_to_visual_odometry_transform, "camera_to_visual_odometry_transform");


	// carmen pose transformation with respect to the world (defined at world origin)
	world_to_carmen_pose.setOrigin(tf::Vector3(0.0, 0.0, 0.0));					// x, y, z;
	world_to_carmen_pose.setRotation(tf::Quaternion(0.0, 0.0, 0.0));  				// yaw, pitch, roll;

	// create a time stamped transformation that defines the carmen position with respect to the world
	tf::StampedTransform world_to_carmen_transform(world_to_carmen_pose, tf::Time(0), "/world", "/carmen");
	// add a link to the tf tree with the carmen stamped transformation
	transformer.setTransform(world_to_carmen_transform, "world_to_carmen_transform");


	// get the transformation between the visual odometry coordinate system with respect to the carmen coordinate system.
	transformer.lookupTransform("/carmen", "/visual_odometry", tf::Time(0), g_carmen_to_visual_odometry_transform);
}
///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
publish_base_ackerman_odometry_message(carmen_visual_odometry_pose6d_message *visual_odometry_message)
{
//	IPC_RETURN_TYPE err = IPC_OK;
	static carmen_base_ackerman_odometry_message odometry;
	static int first = 1;
	static double first_timestamp;
	static FILE *graf;

	if (first)
	{
		odometry.host = visual_odometry_message->host;
		odometry.x = 0;
		odometry.y = 0;
		odometry.theta = 0;

		odometry.v = odometry.phi = 0;
		first_timestamp = visual_odometry_message->timestamp;

		graf = fopen("visual_odometry_graph.txt", "w");
		first = 0;
	}

	odometry.x = visual_odometry_message->pose_6d.x;
	odometry.y = visual_odometry_message->pose_6d.y;
	odometry.theta = visual_odometry_message->pose_6d.yaw;
	odometry.v = visual_odometry_message->v;
	odometry.phi = visual_odometry_message->phi;
	odometry.timestamp = visual_odometry_message->timestamp;

	fprintf(graf, "%lf %lf %lf %lf %lf\n",
			odometry.v, odometry.phi,
			last_base_ackerman_odometry_message.v, last_base_ackerman_odometry_message.phi,
			odometry.timestamp - first_timestamp);
	
//	err = IPC_publishData(CARMEN_BASE_ACKERMAN_ODOMETRY_NAME, &odometry);
//	carmen_test_ipc(err, "Could not publish base_odometry_message", CARMEN_BASE_ACKERMAN_ODOMETRY_NAME);
}


static void
publish_robot_ackerman_velocity_message(carmen_visual_odometry_pose6d_message *visual_odometry_message)
{
	IPC_RETURN_TYPE err = IPC_OK;
//	static carmen_base_ackerman_odometry_message odometry;
	carmen_robot_ackerman_velocity_message robot_ackerman_velocity_message;
//	static int first = 1;
//	static double first_timestamp;

//	if (first)
//	{
//		odometry.host = visual_odometry_message->host;
//		odometry.x = 0;
//		odometry.y = 0;
//		odometry.theta = 0;
//
//		odometry.v = odometry.phi = 0;
//		first_timestamp = visual_odometry_message->timestamp;
//		first = 0;
//	}

//	odometry.x = visual_odometry_message->pose_6d.x;
//	odometry.y = visual_odometry_message->pose_6d.y;
//	odometry.theta = visual_odometry_message->pose_6d.yaw;
	robot_ackerman_velocity_message.v = visual_odometry_message->v;
	robot_ackerman_velocity_message.phi = visual_odometry_message->phi;
	robot_ackerman_velocity_message.timestamp = visual_odometry_message->timestamp;
	robot_ackerman_velocity_message.host = carmen_get_host();
//	printf("v_phi_time %lf %lf\n", robot_ackerman_velocity_message.v, robot_ackerman_velocity_message.phi);
//	printf("v_phi_time %lf %lf %lf\n", odometry.v, -odometry.phi, odometry.timestamp - first_timestamp); // @@@ Alberto: O phi esta negativado porque o carro inicialmente publicava a odometria ao contrario

//	err = IPC_publishData(CARMEN_BASE_ACKERMAN_ODOMETRY_NAME, &odometry);
//	carmen_test_ipc(err, "Could not publish base_odometry_message",
//			CARMEN_BASE_ACKERMAN_ODOMETRY_NAME);
	err = IPC_publishData(CARMEN_ROBOT_ACKERMAN_VELOCITY_NAME, &robot_ackerman_velocity_message);
	carmen_test_ipc(err, "Could not publish base_ackerman_velocity_message", CARMEN_ROBOT_ACKERMAN_VELOCITY_NAME);
}


static void
publish_visual_odometry_as_global_pos(carmen_visual_odometry_pose6d_message *visual_odometry_message)
{
	IPC_RETURN_TYPE err = IPC_OK;
	static carmen_localize_ackerman_globalpos_message global_pos;

	global_pos.globalpos.x = global_pos.pose.position.x = visual_odometry_message->pose_6d.x;
	global_pos.globalpos.y = global_pos.pose.position.y = visual_odometry_message->pose_6d.y;
	global_pos.globalpos.theta = global_pos.pose.orientation.yaw = visual_odometry_message->pose_6d.yaw;
	global_pos.pose.position.z = visual_odometry_message->pose_6d.z;
	global_pos.pose.orientation.roll = visual_odometry_message->pose_6d.roll;
	global_pos.pose.orientation.pitch = visual_odometry_message->pose_6d.pitch;

	global_pos.odometrypos.x = global_pos.pose.position.x;
	global_pos.odometrypos.y = global_pos.pose.position.y;
	global_pos.odometrypos.theta = global_pos.pose.orientation.yaw;

	global_pos.v = visual_odometry_message->v;
	global_pos.phi = visual_odometry_message->phi;
	global_pos.timestamp = visual_odometry_message->timestamp;

	err = IPC_publishData(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, &global_pos);
	carmen_test_ipc(err, "Could not publish global_pos_message", CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME);
}
///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
bumblebee_stereo_message_handler(carmen_bumblebee_basic_stereoimage_message *message)
{
	IPC_RETURN_TYPE err;
	carmen_6d_point pose_6d;
	Matrix carmen_pose_6d_transformation_matrix;
	static double previous_timestamp = 0.0;

	if (previous_timestamp == 0.0)
		previous_timestamp = message->timestamp;
		
	convert_image_rgb_to_gray(message->raw_left, left_image, message->raw_right, right_image, bumblebee_basic_width, bumblebee_basic_height, 0);

	int32_t dims[] = {bumblebee_basic_width, bumblebee_basic_height, bumblebee_basic_width};

	if (fabs(message->timestamp - previous_timestamp) > 3.0)
	{
		viso->process(left_image, right_image, dims);

		initialize_pose_6d_transformation_matrix();
		initialize_transformations();
	}

	if (viso->process(left_image, right_image, dims))
	{
		// update visual odometry pose transformation with delta transformation from getMotion()
		// The reference camera is the left camera (origim).
		visual_odometry_pose_6d_transformation_matrix = visual_odometry_pose_6d_transformation_matrix * Matrix::inv(viso->getMotion());

		// get visual odometry pose with respect to carmen
		pose_6d = get_carmen_pose6D_from_matrix(visual_odometry_pose_6d_transformation_matrix);
		
		// if number of features matches is greater than 0 then publish visual odometry messages
		if (viso->getNumberOfMatches() != 0)
		{
			visual_odometry_assembly_odometry_message(pose_6d, message->timestamp, carmen_get_host());

//			printf("VO\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", pose_6d.x, pose_6d.y, pose_6d.z, carmen_radians_to_degrees(pose_6d.yaw), carmen_radians_to_degrees(pose_6d.pitch), carmen_radians_to_degrees(pose_6d.roll));

			// copy the inlier indices to visual odometry image messages (viewer)
			// viso_inliers_indices = viso->getInlierIndices();
			// viso_matches = viso->getMatches();
			// visual_odometry_copy_inliers(image_msg.inliers, viso_matches, viso_inliers_indices);

//			err = IPC_publishData(CARMEN_VISUAL_ODOMETRY_IMAGE_MESSAGE_NAME, &image_msg);
//			carmen_test_ipc_exit(err, "Could not publish", CARMEN_VISUAL_ODOMETRY_IMAGE_MESSAGE_NAME);

			carmen_add_bias_and_multiplier_to_v_and_phi(&(odometry_msg.v), &(odometry_msg.phi), odometry_msg.v, odometry_msg.phi,
					0.0, visual_odometry_v_multiplier, visual_odometry_phi_bias, visual_odometry_phi_multiplier);

			err = IPC_publishData(CARMEN_VISUAL_ODOMETRY_POSE6D_MESSAGE_NAME, &odometry_msg);
			carmen_test_ipc_exit(err, "Could not publish", CARMEN_VISUAL_ODOMETRY_POSE6D_MESSAGE_NAME);
			
//			if (!robot_publish_odometry)
				publish_base_ackerman_odometry_message(&odometry_msg);

			if (visual_odometry_publish_odometry)
				publish_robot_ackerman_velocity_message(&odometry_msg);

			if (visual_odometry_is_global_pos)
				publish_visual_odometry_as_global_pos(&odometry_msg);
		}

		//free(image_msg.inliers);
		//viso_matches.clear();
		//viso_inliers_indices.clear();
	}
	else
		printf("failed!\n");
	
	previous_timestamp = message->timestamp;
}


static void
base_ackerman_odometry_message_handler(carmen_base_ackerman_odometry_message *msg)
{
	last_base_ackerman_odometry_message = *msg;
}


void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("visual_odometry: disconnected.\n");
		exit(0);
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
subscribe_to_relevant_messages(int camera)
{
	carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) bumblebee_stereo_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_base_ackerman_subscribe_odometry_message(NULL, (carmen_handler_t) base_ackerman_odometry_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


void
read_parameters(int argc, char **argv)
{
	int num_items;
	char bumblebee_string[256];
	char camera_string[256];

	if(argc == 2)
		camera = atoi(argv[1]);
	else
	{
		printf("usage: %s %s", argv[0], argv[1]);
		exit(0);
	}

	sprintf(camera_string, "%s%d", "camera", camera);
	sprintf(bumblebee_string, "%s%d", "bumblebee_basic", camera);

	carmen_param_t param_list[] = 
	{
		{bumblebee_string, (char *) "width", CARMEN_PARAM_INT, &bumblebee_basic_width, 0, NULL},
		{bumblebee_string, (char *) "height", CARMEN_PARAM_INT, &bumblebee_basic_height, 0, NULL},
		{(char *) "robot", (char *) "maximum_acceleration_forward", CARMEN_PARAM_DOUBLE, &maximum_acceleration_forward, 0, NULL},
		{(char *) "robot", (char *) "publish_odometry", CARMEN_PARAM_ONOFF, &robot_publish_odometry, 0, NULL},
		{(char *) camera_string, (char *) "x", CARMEN_PARAM_DOUBLE, &camera_pose.position.x, 0, NULL},
		{(char *) camera_string, (char *) "y", CARMEN_PARAM_DOUBLE, &camera_pose.position.y, 0, NULL},
		{(char *) camera_string, (char *) "z", CARMEN_PARAM_DOUBLE, &camera_pose.position.z, 0, NULL},
		{(char *) camera_string, (char *) "roll", CARMEN_PARAM_DOUBLE, &camera_pose.orientation.roll, 0, NULL},
		{(char *) camera_string, (char *) "pitch", CARMEN_PARAM_DOUBLE, &camera_pose.orientation.pitch, 0, NULL},
		{(char *) camera_string, (char *) "yaw", CARMEN_PARAM_DOUBLE, &camera_pose.orientation.yaw, 0, NULL},
		{(char *) "robot", (char *) "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &distance_between_front_and_rear_axles, 1,NULL},
		{(char *) "sensor_board_1", (char *) "x", CARMEN_PARAM_DOUBLE, &sensor_board_pose.position.x,	0, NULL},
		{(char *) "sensor_board_1", (char *) "y", CARMEN_PARAM_DOUBLE, &sensor_board_pose.position.y,	0, NULL},
		{(char *) "sensor_board_1", (char *) "z", CARMEN_PARAM_DOUBLE, &sensor_board_pose.position.z,	0, NULL},
		{(char *) "sensor_board_1", (char *) "roll", CARMEN_PARAM_DOUBLE, &sensor_board_pose.orientation.roll,0, NULL},
		{(char *) "sensor_board_1", (char *) "pitch", CARMEN_PARAM_DOUBLE, &sensor_board_pose.orientation.pitch,0, NULL},
		{(char *) "sensor_board_1", (char *) "yaw", CARMEN_PARAM_DOUBLE, &sensor_board_pose.orientation.yaw,	0, NULL},
		{(char *) "visual_odometry", (char *) "publish", CARMEN_PARAM_ONOFF, &visual_odometry_publish_odometry, 0, NULL},
		{(char *) "visual_odometry", (char *) "is_global_pos", CARMEN_PARAM_ONOFF, &visual_odometry_is_global_pos, 0, NULL},
		{(char *) "visual_odometry", (char *) "phi_multiplier", CARMEN_PARAM_DOUBLE, &visual_odometry_phi_multiplier, 0, NULL},
		{(char *) "visual_odometry", (char *) "phi_bias", CARMEN_PARAM_DOUBLE, &visual_odometry_phi_bias, 0, NULL},
		{(char *) "visual_odometry", (char *) "v_multiplier", CARMEN_PARAM_DOUBLE, &visual_odometry_v_multiplier, 0, NULL}
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	stereo_instance = get_stereo_instance(camera, bumblebee_basic_width, bumblebee_basic_height);
}


bool 
visual_odometry_initialize(float focal_length, float principal_point_x, float principal_point_y, float baseline)
{
	// TODO Isso nao deveria vir dos parametros da bumblebee
	viso_param.calib.f = focal_length;
	viso_param.calib.cu = principal_point_x;
	viso_param.calib.cv = principal_point_y;
	viso_param.base = baseline;

	viso_param.bucket.max_features = 100;
	viso_param.bucket.bucket_width = 64;
	viso_param.bucket.bucket_height = 48;

	viso_param.match.nms_n                  = 10;  // non-max-suppression: min. distance between maxima (in pixels)
	viso_param.match.nms_tau                = 30;  // non-max-suppression: interest point peakiness threshold
	viso_param.match.match_binsize          = 25;  // matching bin width/height (affects efficiency only)
	viso_param.match.match_radius           = 100; // matching radius (du/dv in pixels)
	viso_param.match.match_disp_tolerance   = 10;  // dv tolerance for stereo matches (in pixels)
	viso_param.match.outlier_disp_tolerance = 20;  // outlier removal: disparity tolerance (in pixels)
	viso_param.match.outlier_flow_tolerance = 20;  // outlier removal: flow tolerance (in pixels)
	viso_param.match.multi_stage            = 1;   // 0=disabled,1=multistage matching (denser and faster)
	viso_param.match.half_resolution        = 1;   // 0=disabled,1=match at half resolution, refine at full resolution
	viso_param.match.refinement             = 1;   // refinement (0=none,1=pixel,2=subpixel)

	if (viso == NULL)
	{
		viso = new VisualOdometryStereo(viso_param);
		return true;
	}
	else
		return false;
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	signal(SIGINT, shutdown_module);

	if (argc < 2)
	{
		printf("Usage: ./visual_odometry2 camera_number");
		return 1;
	}

	read_parameters(argc, argv);
	carmen_visual_odometry_define_messages();

	// initialize visual odometry with read parameters
	if (visual_odometry_initialize(stereo_instance.fx, stereo_instance.xc, stereo_instance.yc, stereo_instance.baseline))
	{
		// initialize visual odometry pose transformation with identity matrix
		initialize_pose_6d_transformation_matrix();

		left_image = (unsigned char *) malloc(bumblebee_basic_height * bumblebee_basic_width * sizeof(unsigned char));
		right_image = (unsigned char *) malloc(bumblebee_basic_height * bumblebee_basic_width * sizeof(unsigned char));

		initialize_transformations();

		subscribe_to_relevant_messages(camera);
		carmen_ipc_dispatch();
	}
	else
	{
		printf("Could not start visual_odometry2 module!\n");
		return (1);
	}
}
