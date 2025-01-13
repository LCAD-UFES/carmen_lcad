#include <memory>
#include <iostream>
#include <unistd.h>

#include <carmen/carmen.h>
#include <carmen/velodyne_messages.h>
#include <carmen/velodyne_interface.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include <cmath>  

double x_origin = 0;
double y_origin = 0;

class GlobalPoseConverter : public rclcpp::Node
{
public:
    GlobalPoseConverter()
    : Node("global_pose_converter") // Initialize the Node with a name
    {
        // Create a subscription to the /baselink_to_map_tf topic
        subscription_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
            "/baselink_to_map_tf", 10, 
            std::bind(&GlobalPoseConverter::topic_callback, this, std::placeholders::_1));
    }

private:
    // Callback to handle incoming messages
    void topic_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
    {
        double roll = 0;
        double pitch = 0;
        double yaw = 0;
        
        printf("Received transform:\n");
        printf("Frame ID Transform from: %s\n", msg->header.frame_id.c_str());
        printf("Frame ID Transform to: %s\n", msg->child_frame_id.c_str());

        printf("Translacao: x: %.2f, y:%.2f, z:%.2f\n", 
                    msg->transform.translation.x, 
                    msg->transform.translation.y, 
                    msg->transform.translation.z);

        printf("Rotacao: roll: %.2f, pitch: %.2f, yaw: %.2f, w: %.2f\n", 
                    msg->transform.rotation.x, 
                    msg->transform.rotation.y, 
                    msg->transform.rotation.z, 
                    msg->transform.rotation.w);
        
          // Convert quaternion to Euler angles (roll, pitch, yaw)
        tf2::Quaternion quat(msg->transform.rotation.x, 
                             msg->transform.rotation.y,
                             msg->transform.rotation.z,
                             msg->transform.rotation.w);

        tf2::Matrix3x3 mat(quat);
        mat.getRPY(roll, pitch, yaw); 

        double theta_deg = yaw * 180.0 / M_PI;

        printf("Posicao Recebida -> X: %f, Y: %f, Z: %f\n", msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z);
        printf("Theta (Yaw) (Z rotacao em radianos): %f\n", yaw);
        printf("Theta (Yaw) (Z rotacao em degraus): %f\n", theta_deg);
        printf("\n\n\n\n");
    }

    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr subscription_;
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

int 
main(int argc, char **argv)
{
	signal(SIGINT, shutdown_module);

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
    carmen_velodyne_define_messages();

	// Init ROS node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalPoseConverter>());
    rclcpp::shutdown();

    return 0;
}
