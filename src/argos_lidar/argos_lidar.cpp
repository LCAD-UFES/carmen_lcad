#include <memory>
#include <carmen/carmen.h>
#include <carmen/velodyne_messages.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
using std::placeholders::_1;


class LidarSubscriber : public rclcpp::Node
{
  public:
    LidarSubscriber()
    : Node("lidar_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
		"/utlidar/cloud", 10, std::bind(&LidarSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
		carmen_velodyne_variable_scan_message carmen_msg;
		carmen_msg.host = carmen_get_host();
		carmen_msg.timestamp = carmen_get_time();
		carmen_msg.number_of_shots = msg->width * msg->height;
		for (uint32_t i = 0; i < msg->height; i++)
		{
			for (uint32_t j = 0; j < msg->width; j++)
			{
				carmen_velodyne_shot shot;
				_Float32 x, y, z, intensity;
				memcpy(&x, &msg->data[0 + msg->point_step * i], 4);
				memcpy(&y, &msg->data[4 + msg->point_step * i], 4);
				memcpy(&z, &msg->data[8 + msg->point_step * i], 4);
				memcpy(&intensity, &msg->data[16], 4);
				// shot.distance = sqrt(x*x + y*y + z*z);
				printf("x: %f, y: %f, z: %f intensity: %f\n", x, y, z, intensity);
			}
			// sleep(20);
		}
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

// static void 
// shutdown_module(int sig)
// {
// 	(void) sig;

// 	static int done = 0;

// 	if (!done)
// 	{
// 		carmen_ipc_disconnect();
// 		printf("argos_lidar disconnected from IPC.\n");
// 		fflush(stdout);

// 		rclcpp::shutdown();
// 		printf("argos_lidar disconnected from ROS.\n");
// 		fflush(stdout);
// 	}

// 	exit(0);
// }

int 
main(int argc, char **argv)
{
	// signal(SIGINT, shutdown_module);

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	// define_relevant_messages();
	// subscribe_to_relevant_messages();
	// initialize_structures();

	// Init ROS node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarSubscriber>());
    rclcpp::shutdown();
    // return 0;
}