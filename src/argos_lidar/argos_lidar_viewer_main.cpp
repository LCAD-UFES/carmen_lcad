#include <memory>

#include <carmen/carmen.h>
#include <carmen/velodyne_messages.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#define lidar_range_division_factor 1000

using std::placeholders::_1;

void
plot_point_cloud(std::vector<std::tuple<_Float32,_Float32,_Float32>> points)
{
	static bool first_time = true;
	static FILE *gnuplot_pipe;

	if (first_time)
	{
		first_time = false;

		gnuplot_pipe = popen("gnuplot", "w"); // -persist to keep last plot after program closes
		fprintf(gnuplot_pipe, "set xrange [-5:5]\n");
		fprintf(gnuplot_pipe, "set yrange [-5:5]\n");
		fprintf(gnuplot_pipe, "set zrange [0:1]\n");
		fprintf(gnuplot_pipe, "set xlabel 'x'\n");
		fprintf(gnuplot_pipe, "set ylabel 'y'\n");
		fprintf(gnuplot_pipe, "set zlabel 'z'\n");
	}

	FILE *points_file = fopen("mpp.txt", "w");

	int index = 0;
	for (auto point : points)
	{
		_Float32 x, y, z;
		std::tie(x, y, z) = point;
		fprintf(points_file, "%f %f %f\n", x, y, z);

		_Float32 distance_float = sqrt(x * x + y * y + z * z);
		unsigned int distance = distance_float * lidar_range_division_factor;

		_Float32 angle_horizontal = atan2(y, x);
		angle_horizontal = angle_horizontal * 180.0 / M_PI;

		_Float32 angle_vertical = atan2(z, sqrt(x * x + y * y));
		angle_vertical = angle_vertical * 180.0 / M_PI;

		printf("N: %d, x: %f, y: %f, z: %f, Distance: %d, Horizontal Angle: %f, Vertical Angle: %f\n",
				index, x, y, z, distance, angle_horizontal, angle_vertical);
		index++;
	}

	fclose(points_file);
	fprintf(gnuplot_pipe, "splot "
			"'./mpp.txt' using 1:2:3 with points pointtype 7 pointsize 1 title \"Pontos x, y, z\"");

	fprintf(gnuplot_pipe, "\n");

	fflush(gnuplot_pipe);
}



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
		std::vector<std::tuple<_Float32,_Float32,_Float32>> points;
		points.reserve(msg->width);
		for (uint32_t i = 0; i < msg->width; i++)
		{
			_Float32 x, y, z, intensity;
			memcpy(&x, &msg->data[0 + msg->point_step * i], 4);
			memcpy(&y, &msg->data[4 + msg->point_step * i], 4);
			memcpy(&z, &msg->data[8 + msg->point_step * i], 4);
			memcpy(&intensity, &msg->data[16 + msg->point_step * i], 4);
			points.emplace_back(std::make_tuple(x, y, z));
			// shot.distance = sqrt(x*x + y*y + z*z);
			printf("x: %f, y: %f, z: %f intensity: %f\n", x, y, z, intensity);
		}
		plot_point_cloud(points);

    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

static void 
shutdown_module(int sig)
{
	(void) sig;

	static int done = 0;

	if (!done)
	{
		carmen_ipc_disconnect();
		printf("argos_lidar disconnected from IPC.\n");
		fflush(stdout);

		rclcpp::shutdown();
		printf("argos_lidar disconnected from ROS.\n");
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

	// define_relevant_messages();
	// subscribe_to_relevant_messages();
	// initialize_structures();

	// Init ROS node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarSubscriber>());
    rclcpp::shutdown();
    // return 0;
}
