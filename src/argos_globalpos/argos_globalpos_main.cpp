#include <memory>
#include <iostream>
#include <unistd.h>
#include <cmath>  

#include <carmen/carmen.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/fused_odometry_interface.h>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


static double start_x, start_y, start_theta, timestamp;
int num_messages_per_second;
double vel_lin = 0.0, vel_ang = 0.0,curvature = 0.0,phi = 0.0;

class GlobalPoseConverter;
void odom_and_global_pos_handler(GlobalPoseConverter* global_pose_converter, const nav_msgs::msg::Odometry::SharedPtr msg);

class GlobalPoseConverter : public rclcpp::Node
{
public:
    GlobalPoseConverter()
    : Node("global_pose_converter"), 
	tf_buffer_(this->get_clock()), 
	tf_listener_(tf_buffer_)
    {
		odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, 
																				[this](const nav_msgs::msg::Odometry::SharedPtr msg) 
																				{ odom_and_global_pos_handler(this, msg); } );
    }

	tf2_ros::Buffer tf_buffer_;
private:
	tf2_ros::TransformListener tf_listener_;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

};

geometry_msgs::msg::TransformStamped compose_transforms(const geometry_msgs::msg::TransformStamped &transform1,
														const geometry_msgs::msg::TransformStamped &transform2) 
{	
	tf2::Transform tf1, tf2_result;
	tf2::fromMsg(transform1.transform, tf1);
	tf2::Transform tf2;
	tf2::fromMsg(transform2.transform, tf2);

	tf2_result = tf1 * tf2;
	
	geometry_msgs::msg::TransformStamped transform_result;
	transform_result.header.stamp = transform1.header.stamp;
	transform_result.header.frame_id = transform1.header.frame_id;
	transform_result.child_frame_id = transform2.child_frame_id;
	transform_result.transform = tf2::toMsg(tf2_result);

	return transform_result;
}

//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//																								//
// Publishers																					//
//																								//
//////////////////////////////////////////////////////////////////////////////////////////////////

void
publish_fused(double x, double y, double theta, double timestamp)
{
	static int first = 1;
	static carmen_fused_odometry_particle_message odometry_message;

	IPC_RETURN_TYPE err;

	if (first)
	{
		odometry_message.particle_pos = (carmen_vector_3D_t *) malloc (1 * sizeof(carmen_vector_3D_t));
		odometry_message.weights = (double *) malloc (1 * sizeof(double));

		first = 0;
	}

	odometry_message.angular_velocity.pitch = 0.0;
	odometry_message.angular_velocity.roll = 0.0;
	odometry_message.angular_velocity.yaw = 0.0;

	odometry_message.gps_position_at_turn_on.x = x;
	odometry_message.gps_position_at_turn_on.y = y;
	odometry_message.gps_position_at_turn_on.z = 0.0;

	odometry_message.host = carmen_get_host();
	odometry_message.num_particles = 1;

	odometry_message.particle_pos[0].x = x;
	odometry_message.particle_pos[0].y = y;
	odometry_message.particle_pos[0].z = 0.0;

	odometry_message.phi = 0.0;
	odometry_message.pose.position.x = x;
	odometry_message.pose.position.y = y;
	odometry_message.pose.position.z = 0.0;
	odometry_message.pose.orientation.yaw = theta;
	odometry_message.pose.orientation.roll = 0.0;
	odometry_message.pose.orientation.pitch = 0.0;

	odometry_message.timestamp = timestamp;

	odometry_message.velocity.x = 1.0;
	odometry_message.velocity.y = 2.0;
	odometry_message.velocity.z = 3.0;

	odometry_message.weight_type = 0;
	odometry_message.weights[0] = 1.0;
	odometry_message.xsens_yaw_bias = 0.0;

	err = IPC_publishData(CARMEN_FUSED_ODOMETRY_PARTICLE_NAME, &odometry_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_FUSED_ODOMETRY_PARTICLE_NAME);
}

void
publish_globalpos(double x, double y, double theta, double timestamp)
{
	IPC_RETURN_TYPE err;
	carmen_localize_ackerman_globalpos_message globalpos;

	globalpos.globalpos.x = x;
	globalpos.globalpos.y = y;
	globalpos.globalpos.theta = theta;

	globalpos.pose.position.x = globalpos.globalpos.x;
	globalpos.pose.position.y = globalpos.globalpos.y;
	globalpos.pose.position.z = 0.0;
	globalpos.pose.orientation.yaw = globalpos.globalpos.theta;
	globalpos.pose.orientation.pitch = globalpos.pose.orientation.roll = 0.0;

	globalpos.v = vel_lin;
	globalpos.phi = phi;

	globalpos.velocity.x = globalpos.velocity.y = globalpos.velocity.z = 0.0;

	globalpos.globalpos_std.x = 0.001;
	globalpos.globalpos_std.y = 0.001;
	globalpos.globalpos_std.theta = 0.001;
	globalpos.globalpos_xy_cov = 0.001;

	globalpos.odometrypos.x = x;
	globalpos.odometrypos.y = y;
	globalpos.odometrypos.theta = theta;

	globalpos.converged = 1;

	globalpos.semi_trailer_type = 0;
	globalpos.semi_trailer_engaged = 0;

	globalpos.timestamp = timestamp;
	globalpos.host = carmen_get_host();

	err = IPC_publishData(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, &globalpos);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME);
}

//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//																								//
// Handlers																						//
//																								//
//////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Função para lidar com a mensagem de odometria, e usando a transformada entre mapa e base_link, publicar global_pos e fused_odom
 */
void 
odom_and_global_pos_handler(GlobalPoseConverter* global_pose_converter, const nav_msgs::msg::Odometry::SharedPtr msg)
{
	timestamp = carmen_get_time();

	// Essa parte calcula o phi e a curvatura do carro
	vel_lin = msg->twist.twist.linear.x;
	vel_ang = msg->twist.twist.angular.z;
	if (vel_lin <= 0.1)	curvature = 0.0;
	else curvature = vel_ang/vel_lin;
	phi = -atan(curvature);

	// Essa parte lê a transformada entre mapa e base_link
	if (!global_pose_converter->tf_buffer_.canTransform("map", "odom", tf2::TimePointZero)) 
	{
		printf("map to odom Transform not available yet");
    	return;
	}
	if (!global_pose_converter->tf_buffer_.canTransform("odom", "base_link", tf2::TimePointZero)) 
	{
		printf("odom to base_link Transform not available yet");
    	return;
	}
	auto transform_map_to_odom = global_pose_converter->tf_buffer_.lookupTransform("map", "odom", tf2::TimePointZero);
	auto transform_odom_to_base_link = global_pose_converter->tf_buffer_.lookupTransform("odom", "base_link", tf2::TimePointZero);
	auto transform_map_to_base_link = compose_transforms(transform_map_to_odom, transform_odom_to_base_link);

	// Calcula yaw
	tf2::Quaternion q(
		transform_map_to_base_link.transform.rotation.x,
		transform_map_to_base_link.transform.rotation.y,
		transform_map_to_base_link.transform.rotation.z,
		transform_map_to_base_link.transform.rotation.w
	);
	double roll, pitch, yaw;
	tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

	// Publica a global_pos e a fused_odom
	publish_fused(transform_map_to_base_link.transform.translation.x + start_x,
					transform_map_to_base_link.transform.translation.y + start_y,
					yaw + start_theta,
					timestamp);

	publish_globalpos(transform_map_to_base_link.transform.translation.x + start_x,
						transform_map_to_base_link.transform.translation.y + start_y,
						yaw + start_theta,
						timestamp);
}

//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//																								//
// Inicializations																				//
//																								//
//////////////////////////////////////////////////////////////////////////////////////////////////

static void 
shutdown_module(int sig)
{
	(void) sig;

	static int done = 0;

	if (!done)
	{
		carmen_ipc_disconnect();
		printf("argos_global_pose disconnected from IPC.\n");
		fflush(stdout);

		rclcpp::shutdown();
		printf("argos_global_pose disconnected from ROS.\n");
		fflush(stdout);
	}

	exit(0);
}


static void
define_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME);

	err = IPC_defineMsg(CARMEN_FUSED_ODOMETRY_PARTICLE_NAME, IPC_VARIABLE_LENGTH, CARMEN_FUSED_ODOMETRY_PARTICLE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_FUSED_ODOMETRY_PARTICLE_NAME);
}


static void 
read_parameters(int argc, char *argv[])
{
	if (argc < 5)
	{
		printf("Use %s <num messages per second> <x> <y> <theta>\n", argv[0]);
		exit(-1);
	}

	num_messages_per_second = atoi(argv[1]);
	start_x = atof(argv[2]);
	start_y = atof(argv[3]);
	start_theta = atof(argv[4]);
}

int 
main(int argc, char **argv)
{
    
	signal(SIGINT, shutdown_module);

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	read_parameters(argc, argv);
    define_messages();

	// Init ROS node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalPoseConverter>());
    rclcpp::shutdown();

    return 0;
}
