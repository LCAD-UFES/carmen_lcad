#include <carmen/carmen.h>
#include <carmen/voslam_interface.h>
#include <carmen/stereo_interface.h>
#include <carmen/visual_odometry_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/stereo_util.h>

#include <Eigen/Core>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/angles.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/ply_io.h>

#include "voslam_main.h"
#include "voslam_messages_interpolation.cpp"
#include "voslam_keyframes.h"
#include "voslam_generalized_icp.h"
#include "voslam_loop_detector.h"

int 	init_visual_odometry_and_stereo_interpolation_counter_g = 0;
bool	global_initialization_g = false;
bool 	has_new_frame_g = false;

static int carmen_stereo_width_g;
static int carmen_stereo_height_g;
static int witch_camera_g;
static carmen_pose_3D_t car_pose_g;
static carmen_pose_3D_t camera_pose_g;
static carmen_pose_3D_t velodyne_pose_g;
static carmen_pose_3D_t sensor_board_pose_g;
static int stereo_vertical_ROI_ini_g;
static int stereo_vertical_ROI_end_g;

stereo_util stereo_reprojection_params_g;
carmen_voslam_pointcloud_t target_voslam_pointcloud_g;
carmen_fused_odometry_message initial_fused_odometry_g;

VoslamOpenGLThread opengl;
VoslamMessagesInterpolation<carmen_visual_odometry_pose6d_message, carmen_simple_stereo_disparity_message> interpolator(20);
VoslamMessagesInterpolation<carmen_fused_odometry_message, carmen_simple_stereo_disparity_message> fused_odometry_interpolator(20);
VoslamKeyframes keyframes(3.0, 20.0);
VoslamGeneralizedICP generalized_icp(0.4, 20, 0.2, 1e-5);
VoslamLoopDetector loop_detector(640, 480, 4.0, 100);

tf::Transformer transformer(false);

carmen_pose_3D_t globalpos_g;
carmen_pose_3D_t current_odom_g, last_odom_g;
double current_v = 0.0, current_phi = 0.0;

struct timeval start, end;

void
start_count_time()
{
	gettimeofday(&start, NULL);
}


double
get_time_ellapsed()
{
	gettimeofday(&end, NULL);
	double ellapsed = (end.tv_sec - start.tv_sec) * 1000000 + (end.tv_usec - start.tv_usec);
	return (ellapsed / 1000000.0);
}

void
initialize_position_and_rotation(tf::Vector3 position, tf::Matrix3x3 rotation)
{
	position[0] = 0.0;
	position[1] = 0.0;
	position[2] = 0.0;

	rotation[0][0] = 1.0;
	rotation[0][1] = 0.0;
	rotation[0][2] = 0.0;
	rotation[1][0] = 0.0;
	rotation[1][1] = 1.0;
	rotation[1][2] = 0.0;
	rotation[2][0] = 0.0;
	rotation[2][1] = 0.0;
	rotation[2][2] = 1.0;
}

void
initialize_voslam_pointcloud_struct()
{
	target_voslam_pointcloud_g.pointcloud_size = carmen_stereo_height_g * carmen_stereo_width_g;
	initialize_position_and_rotation(target_voslam_pointcloud_g.pose.position, target_voslam_pointcloud_g.pose.rotation);
	target_voslam_pointcloud_g.pointcloud = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>);

	target_voslam_pointcloud_g.image = (unsigned char *) calloc (carmen_stereo_height_g * carmen_stereo_width_g * 3, sizeof(unsigned char));
	carmen_test_alloc(target_voslam_pointcloud_g.image);

	target_voslam_pointcloud_g.depth = (unsigned short* ) calloc (carmen_stereo_height_g * carmen_stereo_width_g, sizeof(unsigned short));
	carmen_test_alloc(target_voslam_pointcloud_g.depth);

	target_voslam_pointcloud_g.timestamp = 0.0;
}

/**
 * TODO: Criar uma forma de escolher modo carro e modo free-hand. Em condicao de uso "free-hand",
 * somente a transformada da camera para o mundo e da pointcloud para a camera sao necessarias.
 */

void initialize_transforms()
{
	tf::Transform world_to_opengl_transform;
	tf::Transform camera_to_pointcloud_pose;
	tf::Transform board_to_camera_pose;
	tf::Transform board_to_velodyne_pose;
	tf::Transform car_to_board_pose;
	tf::Transform world_to_car_pose;

	tf::Time::init();

	// initial car pose with respect to the world
	world_to_car_pose.setOrigin(tf::Vector3(car_pose_g.position.x, car_pose_g.position.y, car_pose_g.position.z));
	world_to_car_pose.setRotation(tf::Quaternion(car_pose_g.orientation.yaw, car_pose_g.orientation.pitch, car_pose_g.orientation.roll));
	tf::StampedTransform world_to_car_transform(world_to_car_pose, tf::Time(0), "/world", "/car");
	transformer.setTransform(world_to_car_transform, "world_to_car_transform");

	// board pose with respect to the car
	car_to_board_pose.setOrigin(tf::Vector3(sensor_board_pose_g.position.x, sensor_board_pose_g.position.y, sensor_board_pose_g.position.z));
	car_to_board_pose.setRotation(tf::Quaternion(sensor_board_pose_g.orientation.yaw, sensor_board_pose_g.orientation.pitch, sensor_board_pose_g.orientation.roll)); 				// yaw, pitch, roll
	tf::StampedTransform car_to_board_transform(car_to_board_pose, tf::Time(0), "/car", "/board");
	transformer.setTransform(car_to_board_transform, "car_to_board_transform");

	// camera pose with respect to the board
	board_to_camera_pose.setOrigin(tf::Vector3(camera_pose_g.position.x, camera_pose_g.position.y, camera_pose_g.position.z));
	board_to_camera_pose.setRotation(tf::Quaternion(camera_pose_g.orientation.yaw, camera_pose_g.orientation.pitch, camera_pose_g.orientation.roll)); 				// yaw, pitch, roll
	tf::StampedTransform board_to_camera_transform(board_to_camera_pose, tf::Time(0), "/board", "/camera");
	transformer.setTransform(board_to_camera_transform, "board_to_camera_transform");

	// pointcloud pose with respect to the camera
	camera_to_pointcloud_pose.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	camera_to_pointcloud_pose.setRotation(tf::Quaternion(0.0, 0.0, 0.0)); 				// yaw, pitch, roll
	tf::StampedTransform camera_to_pointcloud_transform(camera_to_pointcloud_pose, tf::Time(0), "/camera", "/pointcloud");
	transformer.setTransform(camera_to_pointcloud_transform, "camera_to_pointcloud_transform");

	world_to_opengl_transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));										// x, y, z;
	world_to_opengl_transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0)); 				// yaw, pitch, roll
	tf::StampedTransform world_to_opengl_stamped_transform(world_to_opengl_transform, tf::Time(0), "/opengl", "/world");
	transformer.setTransform(world_to_opengl_stamped_transform, "carmen_to_world_stamped_transform");
}

void
initialize_localize()
{
	// Envia a mensagem de inicializacao do localize
	carmen_point_t mean;
	carmen_point_t std;

	memset(&mean, 0, sizeof(mean));
	memset(&std, 0, sizeof(std));

	carmen_localize_ackerman_initialize_gaussian_time_command(mean, std, carmen_get_time());
}

void
carmen_voslam_initialize()
{
	global_initialization_g = true;
	initialize_voslam_pointcloud_struct();
	initialize_transforms();
	initialize_localize();
	stereo_reprojection_params_g = get_stereo_instance(witch_camera_g, carmen_stereo_width_g, carmen_stereo_height_g);

	memset(&globalpos_g, 0, sizeof(globalpos_g));
}

void get_pointcloud_from_depthmap(unsigned short* depth_map, unsigned char* image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud, stereo_util instance, int top_cut, int bottom_cut)
{
	int x, y;
	pcl::PointXYZRGB p3D;
	double z, dfx_inv, dfy_inv, range;
	double d_xc, d_yc;
	int pos;

	dfx_inv = 1.0 / instance.fx;
	dfy_inv = 1.0 / instance.fy;

	d_xc = instance.width / 2.0;
	d_yc = instance.height / 2.0;

	pointcloud->clear();

	for(y = top_cut; y < bottom_cut; y++)
	{
		for(x = 0; x < instance.width; x++)
		{
			pos = y * instance.width + x;
			z = depth_map[pos] / 1000.0;

			/****************************************************
			 * TODO: Eu mudei esse trecho para que a nuvem seja
			 * reconstruida de acordo com o referencia do carmen.
			 * Fazer isso por meio de uma transformada da TF
			 ****************************************************/

			p3D.y = -(z * (x - d_xc) * dfx_inv);
			p3D.z = -(z * (y - d_yc) * dfy_inv);
			p3D.x = z;

			p3D.r = image[3 * pos];
			p3D.g = image[3 * pos + 1];
			p3D.b = image[3 * pos + 2];

			range = sqrt(DOT3D(p3D, p3D));

			if(range >= 1.5 && range <= STEREO_RANGE_MAX)
				pointcloud->push_back(p3D);
		}
	}
}


void
publish_globalpos(carmen_voslam_pointcloud_t *pointcloud, double timestamp)
{
	carmen_localize_ackerman_globalpos_message globalpos;
	double roll, pitch, yaw;

	pointcloud->pose.rotation.getRPY(roll, pitch, yaw);

	globalpos.timestamp = timestamp;
	globalpos.host = carmen_get_host();
	globalpos.v = pointcloud->v;
	globalpos.phi = pointcloud->phi;

	globalpos.pose.position.x = pointcloud->pose.position.x();
	globalpos.pose.position.y = pointcloud->pose.position.y();
	globalpos.pose.position.z = pointcloud->pose.position.z();

	globalpos.pose.orientation.pitch = pitch;
	globalpos.pose.orientation.roll = roll;
	globalpos.pose.orientation.yaw = yaw;

	globalpos.velocity.x = pointcloud->v;
	globalpos.velocity.y = 0.0;
	globalpos.velocity.z = 0.0;
	globalpos.globalpos.x = pointcloud->pose.position.x();
	globalpos.globalpos.y = pointcloud->pose.position.y();
	globalpos.globalpos.theta = yaw;

//	printf("GLOBALPOS: \tx %lf \ty %lf \tz %lf \tpitch %lf \troll %lf \tyaw %lf\n",
//			globalpos.pose.position.x,
//			globalpos.pose.position.y,
//			globalpos.pose.position.z,
//			globalpos.pose.orientation.pitch,
//			globalpos.pose.orientation.roll,
//			globalpos.pose.orientation.yaw
//	);

	IPC_RETURN_TYPE err;
	err = IPC_publishData(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, &globalpos);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME);
}

tf::Transform
tf_transform_from_carmen_pose(carmen_pose_3D_t pose)
{
	tf::Vector3 position(
		pose.position.x,
		pose.position.y,
		pose.position.z
	);

	tf::Quaternion orientation(
		pose.orientation.yaw,
		pose.orientation.pitch,
		pose.orientation.roll
	);

	tf::Transform transform(orientation, position);
	return transform;
}

carmen_pose_3D_t
carmen_pose_from_tf_transform(tf::Transform tf_pose)
{
	carmen_pose_3D_t pose;
	double roll, pitch, yaw;

	pose.position.x = tf_pose.getOrigin().x();
	pose.position.y = tf_pose.getOrigin().y();
	pose.position.z = tf_pose.getOrigin().z();

	tf::Matrix3x3(tf_pose.getRotation()).getRPY(roll, pitch, yaw);

	pose.orientation.yaw = yaw;
	pose.orientation.pitch = pitch;
	pose.orientation.roll = roll;

	return pose;
}

void
correct_globalpos(double x, double y, double z, double roll, double pitch, double yaw)
{
	globalpos_g.position.x = x;
	globalpos_g.position.y = y;
	globalpos_g.position.z = 0.0; //z;

	globalpos_g.orientation.yaw = yaw;
	globalpos_g.orientation.roll = 0.0; //roll;
	globalpos_g.orientation.pitch = 0.0; //pitch;

	//printf("ICP: %lf %lf %lf %lf %lf %lf\n", x, y, z, roll, pitch, yaw);
}

void
update_odometry_command(double x, double y, double z, double roll, double pitch, double yaw)
{
	last_odom_g = current_odom_g;

	current_odom_g.position.x = x;
	current_odom_g.position.y = y;
	current_odom_g.position.z = 0.0; //z;

	current_odom_g.orientation.yaw = yaw;
	current_odom_g.orientation.roll = 0.0; // roll;
	current_odom_g.orientation.pitch = 0.0; // pitch;
}

void
predict_globalpos()
{
	tf::Vector3 last_position(
		last_odom_g.position.x,
		last_odom_g.position.y,
		0.0 // last_odom_g.position.z
	);

	tf::Quaternion last_orientation(
		last_odom_g.orientation.yaw,
		0.0, // last_odom_g.orientation.pitch,
		0.0  // last_odom_g.orientation.roll
	);

	tf::Vector3 current_position(
		current_odom_g.position.x,
		current_odom_g.position.y,
		0.0 // current_odom_g.position.z
	);

	tf::Quaternion current_orientation(
		current_odom_g.orientation.yaw,
		0.0, // current_odom_g.orientation.pitch,
		0.0 //current_odom_g.orientation.roll
	);

	tf::Transform current_odometry_transform(current_orientation, current_position);
	tf::Transform last_odometry_transform(last_orientation, last_position);
	tf::Transform tf_globalpos = tf_transform_from_carmen_pose(globalpos_g);

	globalpos_g = carmen_pose_from_tf_transform((current_odometry_transform * last_odometry_transform.inverse()) * tf_globalpos);
}

void
interpolate_visual_odometry_and_stereo_messages(carmen_simple_stereo_disparity_message *message, carmen_voslam_pointcloud_t *interpolated_message)
{
	double yaw, pitch, roll;
//	carmen_visual_odometry_pose6d_message odometry_message;

	tf::Transform carmen_to_world_transform;
	Eigen::Matrix<float, 4, 4> pointcloud_pose;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;

//	odometry_message = interpolator.InterpolateMessages(message);
//
	for (int i = 0; i < (carmen_stereo_height_g * carmen_stereo_width_g * 3); i++)
		interpolated_message->image[i] = message->reference_image[i];

	get_depth_map_from_disparity_map(message->disparity, interpolated_message->depth, stereo_reprojection_params_g, STEREO_RANGE_MAX);
	//apply_bilateral_filter(interpolated_message->depth, interpolated_message->depth, stereo_reprojection_params_g);
	get_pointcloud_from_depthmap(interpolated_message->depth, interpolated_message->image, interpolated_message->pointcloud, stereo_reprojection_params_g, stereo_vertical_ROI_ini_g + 20, stereo_vertical_ROI_end_g - 10);

//	carmen_to_world_transform.setOrigin(tf::Vector3(odometry_message.pose_6d.x, odometry_message.pose_6d.y, car_pose_g.position.z));
//	carmen_to_world_transform.setRotation(tf::Quaternion(odometry_message.pose_6d.yaw, 0.0, 0.0));
//	tf::StampedTransform carmen_to_world_stamped_transform(carmen_to_world_transform, tf::Time(0), "/world", "/car");
//	transformer.setTransform(carmen_to_world_stamped_transform, "carmen_to_world_stamped_transform");

	carmen_to_world_transform.setOrigin(tf::Vector3(globalpos_g.position.x, globalpos_g.position.y, globalpos_g.position.z));
	carmen_to_world_transform.setRotation(tf::Quaternion(globalpos_g.orientation.yaw, globalpos_g.orientation.pitch, globalpos_g.orientation.roll));
	tf::StampedTransform carmen_to_world_stamped_transform(carmen_to_world_transform, tf::Time(0), "/world", "/car");
	transformer.setTransform(carmen_to_world_stamped_transform, "carmen_to_world_stamped_transform");

	/**
	 * TODO: Aqui eu estou dizendo que a pose do stereo eh a pose do carro. Ao meu ver, o correto seria
	 * colocar a pose da camera. Para fazer isso, eu teria que dar um lookup na tf para pegar a pose
	 * da camera do mundo. ENTRETANTO, se eu fizer isso, na hora de publicar a globalpos, eu teria que
	 * converter a pose das nuvens da camera para a pose do carro.
	 */
	interpolated_message->pose.position = carmen_to_world_stamped_transform.getOrigin();
	interpolated_message->pose.rotation = tf::Matrix3x3(carmen_to_world_stamped_transform.getRotation());

	interpolated_message->pose.rotation.getRPY(roll, pitch, yaw);
	interpolated_message->pose.orientation = tf::Vector3(roll, pitch, yaw);

	/*****************************************
	 ************** FILIPE *******************
	 ****************************************/

//	interpolated_message->pose.position = tf::Vector3(globalpos_g.position.x, globalpos_g.position.y, globalpos_g.position.z);
//	interpolated_message->pose.rotation = tf::Matrix3x3(tf::Quaternion(globalpos_g.orientation.yaw, globalpos_g.orientation.pitch, globalpos_g.orientation.roll));
//	interpolated_message->pose.orientation = tf::Vector3(globalpos_g.orientation.roll, globalpos_g.orientation.pitch, globalpos_g.orientation.yaw);

	/*****************************************
	 ************** \FILIPE ******************
	 ****************************************/

	interpolated_message->v = current_v; // odometry_message.v;
	interpolated_message->phi = current_phi; // odometry_message.phi;

	pointcloud_pose = voslam_pointcloud_pose_to_eigen_transform(interpolated_message->pose);
	pcl::transformPointCloud(*interpolated_message->pointcloud, *interpolated_message->pointcloud, pointcloud_pose);

//	sor.setInputCloud (interpolated_message->pointcloud);
//	sor.setMeanK (30);
//	sor.setStddevMulThresh (1.0);
//	sor.filter (*cloud_filtered);

//	*interpolated_message->pointcloud = *cloud_filtered;
}

void upload_last_frame_in_chain(int ms)
{
	if(has_new_frame_g)
	{
#ifndef NO_CUDA
		opengl.UploadChainCuda(*(keyframes.getKeyframesList()), carmen_stereo_width_g, carmen_stereo_height_g, stereo_reprojection_params_g.fx, stereo_reprojection_params_g.fy, stereo_reprojection_params_g.fx, stereo_reprojection_params_g.fy, stereo_reprojection_params_g.xc, stereo_reprojection_params_g.yc);
#else
		opengl.UploadChain(keyframes, carmen_stereo_width_g, carmen_stereo_height_g);
#endif

		has_new_frame_g = false;
	}
}

/*********************************************************
		   --- Publishers ---
 **********************************************************/

void
publish_voslam_pointcloud_message(carmen_voslam_pointcloud_message* message)
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData(CARMEN_VOSLAM_POINTCLOUD_MESSAGE_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_VOSLAM_POINTCLOUD_MESSAGE_NAME);
}

/*********************************************************
		   --- Handlers ---
 **********************************************************/

void
carmen_visual_odometry_pose6d_handler(carmen_visual_odometry_pose6d_message *message)
{
//	printf("VO: %lf %lf %lf %lf %lf %lf\n",
//		message->pose_6d.x,
//		message->pose_6d.y,
//		message->pose_6d.z,
//		message->pose_6d.roll,
//		message->pose_6d.pitch,
//		message->pose_6d.yaw
//	);

	update_odometry_command(
		message->pose_6d.x,
		message->pose_6d.y,
		message->pose_6d.z,
		message->pose_6d.roll,
		message->pose_6d.pitch,
		message->pose_6d.yaw
	);

	predict_globalpos();

	carmen_voslam_pointcloud_t pointcloud;
	pointcloud.pose.position = tf::Vector3(globalpos_g.position.x, globalpos_g.position.y, globalpos_g.position.z);
	pointcloud.pose.rotation = tf::Matrix3x3(tf::Quaternion(globalpos_g.orientation.yaw, globalpos_g.orientation.pitch, globalpos_g.orientation.roll));
	pointcloud.pose.orientation = tf::Vector3(globalpos_g.orientation.roll, globalpos_g.orientation.pitch, globalpos_g.orientation.yaw);

	publish_globalpos(&pointcloud, message->timestamp);

	// interpolator.AddMessageToInterpolationList(message);

	current_v = message->v;
	current_phi = message->phi;
}


void
carmen_fused_odometry_pose6d_handler(carmen_fused_odometry_message *message)
{
	printf("VO: %lf %lf %lf %lf %lf %lf\n",
		message->pose.position.x,
		message->pose.position.y,
		message->pose.position.z,
		message->pose.orientation.roll,
		message->pose.orientation.pitch,
		message->pose.orientation.yaw
	);

	update_odometry_command(
		message->pose.position.x,
		message->pose.position.y,
		message->pose.position.z,
		message->pose.orientation.roll,
		message->pose.orientation.pitch,
		message->pose.orientation.yaw
	);

	predict_globalpos();

	carmen_voslam_pointcloud_t pointcloud;
	pointcloud.pose.position = tf::Vector3(globalpos_g.position.x, globalpos_g.position.y, globalpos_g.position.z);
	pointcloud.pose.rotation = tf::Matrix3x3(tf::Quaternion(globalpos_g.orientation.yaw, globalpos_g.orientation.pitch, globalpos_g.orientation.roll));
	pointcloud.pose.orientation = tf::Vector3(globalpos_g.orientation.roll, globalpos_g.orientation.pitch, globalpos_g.orientation.yaw);

	publish_globalpos(&pointcloud, message->timestamp);

	// interpolator.AddMessageToInterpolationList(message);

	current_v = message->velocity.x;
	current_phi = message->phi;
}

double fitness_total = 0;
double time_total = 0;

void
carmen_stereo_disparity_handler(carmen_simple_stereo_disparity_message* message)
{
	if (init_visual_odometry_and_stereo_interpolation_counter_g >= interpolator.GetListSize())
	{
		interpolate_visual_odometry_and_stereo_messages(message, &target_voslam_pointcloud_g);

		if (global_initialization_g)
		{
			keyframes.addKeyframe(&target_voslam_pointcloud_g, stereo_reprojection_params_g);
			global_initialization_g = false;
			return;
		}

		if(keyframes.isKeyframe(target_voslam_pointcloud_g.pose))
		{
			keyframes.addKeyframe(&target_voslam_pointcloud_g, stereo_reprojection_params_g);
			// loop_detector.CheckLoopDetection(&keyframes, &generalized_icp);

			start_count_time();
			double fitness = generalized_icp.runGeneralizedICP(keyframes.getSourceKeyframe(), keyframes.getTargetKeyframe(), NULL);
			double time_ellapsed = get_time_ellapsed();
			time_total += time_ellapsed;

			if (fitness != -1)
			{
				fitness_total += fitness;

				has_new_frame_g = true;

				double roll, pitch, yaw;
				keyframes.getTargetKeyframe()->pose.rotation.getRPY(roll, pitch, yaw);

				correct_globalpos(
					keyframes.getTargetKeyframe()->pose.position.x(),
					keyframes.getTargetKeyframe()->pose.position.y(),
					keyframes.getTargetKeyframe()->pose.position.z(),
					roll,
					pitch,
					yaw
				);

			}
//			else
//				printf("failed!\n");

			publish_globalpos(keyframes.getTargetKeyframe(), message->timestamp);
//			printf("fitness: %lf time_total: %lf\n", fitness_total, time_total);
		}
	}
	else
	{
		init_visual_odometry_and_stereo_interpolation_counter_g++;
	}
}

void 
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		// print_voslam_graph(*keyframes.getKeyframesList());
		// save_keyframe_pointclouds(*keyframes.getKeyframesList());
		opengl.Detach();
		carmen_ipc_disconnect();
		printf("slam6d: disconnected.\n");
		exit(0);
	}
}

static int read_parameters(int argc, char **argv)
{
	int num_items;
	char stereo_string[256];
	char camera_string[256];

	sprintf(camera_string, "%s%d", "camera", witch_camera_g);
	sprintf(stereo_string, "%s%d", "stereo", witch_camera_g);

	carmen_param_t param_list[] =
	{
			{stereo_string, 			  (char *)"width",  CARMEN_PARAM_INT, 	 &carmen_stereo_width_g, 0, NULL},
			{stereo_string, 			  (char *)"height",	CARMEN_PARAM_INT, 	 &carmen_stereo_height_g, 0, NULL},
			{stereo_string, 			  (char *)"vertical_ROI_ini",  CARMEN_PARAM_INT, 	 &stereo_vertical_ROI_ini_g, 0, NULL},
			{stereo_string, 			  (char *)"vertical_ROI_end",	CARMEN_PARAM_INT, 	 &stereo_vertical_ROI_end_g, 0, NULL},

			{(char *) "car", 			  (char *) "x", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.position.x), 0, NULL},
			{(char *) "car", 			  (char *) "y", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.position.y), 0, NULL},
			{(char *) "car", 			  (char *) "z", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.position.z), 0, NULL},
			{(char *) "car", 			  (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.orientation.yaw), 0, NULL},
			{(char *) "car", 			  (char *) "pitch", CARMEN_PARAM_DOUBLE, &(car_pose_g.orientation.pitch), 0, NULL},
			{(char *) "car", 			  (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.orientation.roll), 0, NULL},

			{(char *) "sensor_board_1", (char *) "x", 	CARMEN_PARAM_DOUBLE, &(sensor_board_pose_g.position.x), 0, NULL},
			{(char *) "sensor_board_1", (char *) "y", 	CARMEN_PARAM_DOUBLE, &(sensor_board_pose_g.position.y), 0, NULL},
			{(char *) "sensor_board_1", (char *) "z", 	CARMEN_PARAM_DOUBLE, &(sensor_board_pose_g.position.z), 0, NULL},
			{(char *) "sensor_board_1", (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(sensor_board_pose_g.orientation.yaw), 0, NULL},
			{(char *) "sensor_board_1", (char *) "pitch", CARMEN_PARAM_DOUBLE, &(sensor_board_pose_g.orientation.pitch), 0, NULL},
			{(char *) "sensor_board_1", (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(sensor_board_pose_g.orientation.roll), 0, NULL},

			{(char *) camera_string, 	  (char *) "x", 	CARMEN_PARAM_DOUBLE, &(camera_pose_g.position.x), 0, NULL},
			{(char *) camera_string,    (char *) "y", 	CARMEN_PARAM_DOUBLE, &(camera_pose_g.position.y), 0, NULL},
			{(char *) camera_string,    (char *) "z", 	CARMEN_PARAM_DOUBLE, &(camera_pose_g.position.z), 0, NULL},
			{(char *) camera_string,    (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(camera_pose_g.orientation.yaw), 0, NULL},
			{(char *) camera_string,    (char *) "pitch", CARMEN_PARAM_DOUBLE, &(camera_pose_g.orientation.pitch), 0, NULL},
			{(char *) camera_string,    (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(camera_pose_g.orientation.roll), 0, NULL},

			{(char *) "velodyne",    (char *) "x", 	CARMEN_PARAM_DOUBLE, &(velodyne_pose_g.position.x), 0, NULL},
			{(char *) "velodyne",    (char *) "y", 	CARMEN_PARAM_DOUBLE, &(velodyne_pose_g.position.y), 0, NULL},
			{(char *) "velodyne",    (char *) "z", 	CARMEN_PARAM_DOUBLE, &(velodyne_pose_g.position.z), 0, NULL},
			{(char *) "velodyne",    (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(velodyne_pose_g.orientation.yaw), 0, NULL},
			{(char *) "velodyne",    (char *) "pitch", CARMEN_PARAM_DOUBLE, &(velodyne_pose_g.orientation.pitch), 0, NULL},
			{(char *) "velodyne",    (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(velodyne_pose_g.orientation.roll), 0, NULL}

	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}

int 
main(int argc, char **argv) 
{
	/* Connect to IPC Server */
	carmen_ipc_initialize(argc, argv);

	/* Check the param server version */
	carmen_param_check_version(argv[0]);

	/* Register shutdown cleaner handler */
	signal(SIGINT, shutdown_module);

	/* Read carmen.ini dependent parameters */
	if(argc > 1)
		witch_camera_g = atoi(argv[1]);

	read_parameters(argc, argv);

	/* Initialize data structures */
	carmen_voslam_initialize();

	/* Define messages that your module publishes */
	carmen_voslam_define_messages();

	/* Subscribe to sensor messages */
	carmen_fused_odometry_subscribe_fused_odometry_message(NULL, (carmen_handler_t) carmen_fused_odometry_pose6d_handler,	CARMEN_SUBSCRIBE_LATEST);
	//carmen_visual_odometry_subscribe_pose6d_message(NULL, (carmen_handler_t) carmen_visual_odometry_pose6d_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_stereo_subscribe(witch_camera_g, NULL, (carmen_handler_t) carmen_stereo_disparity_handler, CARMEN_SUBSCRIBE_LATEST);

	/* Starts the graphical user interface */
	// opengl.Start(argc, argv, 150000, 3, 60, upload_last_frame_in_chain, 0, 0);
	// opengl.Start(argc, argv, 1280 * 960, 3, 10, upload_last_frame_in_chain, 1, 0);
	// opengl.Start(argc, argv, 640 * 480, 3, 10, upload_last_frame_in_chain, 1, 0);
	// opengl.Start(argc, argv, 320 * 240, 3, 10, upload_last_frame_in_chain, 1, 0);

	/* Run IPC forever */
	carmen_ipc_dispatch();
}

