#include <memory>
#include <iostream>
#include <unistd.h>

#include <carmen/carmen.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/fused_odometry_interface.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include <cmath>  

static double start_x, start_y, start_theta, timestamp;
int num_messages_per_second;

// void
// publish_fused(double x, double y, double theta, double timestamp)
// {
// 	static int first = 1;
// 	static carmen_fused_odometry_particle_message odometry_message;

// 	IPC_RETURN_TYPE err;

// 	if (first)
// 	{
// 		odometry_message.particle_pos = (carmen_vector_3D_t *) malloc (1 * sizeof(carmen_vector_3D_t));
// 		odometry_message.weights = (double *) malloc (1 * sizeof(double));

// 		first = 0;
// 	}

// 	odometry_message.angular_velocity.pitch = 0.0;
// 	odometry_message.angular_velocity.roll = 0.0;
// 	odometry_message.angular_velocity.yaw = 0.0;

// 	odometry_message.gps_position_at_turn_on.x = x;
// 	odometry_message.gps_position_at_turn_on.y = y;
// 	odometry_message.gps_position_at_turn_on.z = 0.0;

// 	odometry_message.host = carmen_get_host();
// 	odometry_message.num_particles = 1;

// 	odometry_message.particle_pos[0].x = x;
// 	odometry_message.particle_pos[0].y = y;
// 	odometry_message.particle_pos[0].z = 0.0;

// 	odometry_message.phi = 0.0;
// 	odometry_message.pose.position.x = x;
// 	odometry_message.pose.position.y = y;
// 	odometry_message.pose.position.z = 0.0;
// 	odometry_message.pose.orientation.yaw = theta;
// 	odometry_message.pose.orientation.roll = 0.0;
// 	odometry_message.pose.orientation.pitch = 0.0;

// 	odometry_message.timestamp = timestamp;

// 	odometry_message.velocity.x = 0.0;
// 	odometry_message.velocity.y = 0.0;
// 	odometry_message.velocity.z = 0.0;

// 	odometry_message.weight_type = 0;
// 	odometry_message.weights[0] = 1.0;
// 	odometry_message.xsens_yaw_bias = 0.0;

// 	err = IPC_publishData(CARMEN_FUSED_ODOMETRY_PARTICLE_NAME, &odometry_message);
// 	carmen_test_ipc_exit(err, "Could not publish", CARMEN_FUSED_ODOMETRY_PARTICLE_NAME);
// }


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

	globalpos.v = 0.0;
	globalpos.phi = 0.0;

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



class GlobalPoseConverter : public rclcpp::Node
{
public:
    GlobalPoseConverter()
    : Node("global_pose_converter")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/global_pos", 10, 
            std::bind(&GlobalPoseConverter::topic_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription_;
    
	void topic_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
		// O tópico /global_pos contém a posição no sistema ROS2, que deslocamos em start_x, start_y e start_theta para seguir o padrão do carmen

		timestamp = carmen_get_time();

		publish_globalpos(msg->x + start_x,
                      	  msg->y + start_y,
                          msg->z + start_theta,
                          timestamp);

		printf("Published globalpos(%lf, %lf, %lf) at timestamp %lf\n\n\n",msg->x + start_x,
                                                                          msg->y + start_y,
                                                                          msg->z + start_theta,
                                                                          timestamp);
    }

};


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


void
define_messages()
{
	IPC_RETURN_TYPE err;

	carmen_localize_ackerman_define_globalpos_messages();

	err = IPC_defineMsg(CARMEN_FUSED_ODOMETRY_PARTICLE_NAME, IPC_VARIABLE_LENGTH, CARMEN_FUSED_ODOMETRY_PARTICLE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_FUSED_ODOMETRY_PARTICLE_NAME);
}

// static void 
// initialize_ipc(void)
// {
// 	IPC_RETURN_TYPE err;

// 	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME,
// 			IPC_VARIABLE_LENGTH,
// 			CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME);
// 	carmen_test_ipc_exit(err, "Could not define message",
// 			CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME);

// 	return;
// }

int 
main(int argc, char **argv)
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
    
	signal(SIGINT, shutdown_module);

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
    define_messages();

	//initialize_ipc();

	// Init ROS node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalPoseConverter>());
    rclcpp::shutdown();

    return 0;
}
