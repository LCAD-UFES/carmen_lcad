#include <memory>

#include <carmen/carmen.h>
#include <carmen/velodyne_messages.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#define lidar_range_division_factor 1000

using std::placeholders::_1;

void
plot_point_cloud(std::vector<std::tuple<int,_Float32,_Float32>> points)
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

	for (auto point : points)
	{
		_Float32 x, y, z;
		std::tie(x, y, z) = point;
		fprintf(points_file, "%f %f %f\n", x, y, z);
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
		carmen_velodyne_variable_scan_message carmen_msg;
		carmen_msg.host = carmen_get_host();
		carmen_msg.timestamp = carmen_get_time();
		// carmen_msg.number_of_shots = msg->width;
		// std::vector<std::tuple<_Float32,_Float32,_Float32>> points;
		// points.reserve(msg->width);
		carmen_msg.number_of_shots = std::min(msg->width, 50u);
		std::vector<std::tuple<int, _Float32, _Float32>> points; // A distância agora será um valor inteiro
		points.reserve(carmen_msg.number_of_shots); // Reservar espaço para o número limitado de pontos

		uint32_t processed_points = 0;

		for (uint32_t i = 0; i < msg->width && processed_points < 50; i++) 
		{
			//carmen_velodyne_shot shot;
			_Float32 x, y, z, intensity;
			memcpy(&x, &msg->data[0 + msg->point_step * i], 4);
			memcpy(&y, &msg->data[4 + msg->point_step * i], 4);
			memcpy(&z, &msg->data[8 + msg->point_step * i], 4);
			memcpy(&intensity, &msg->data[16 + msg->point_step * i], 4);

			// Calcular a distância
			_Float32 distance_float = sqrt(x * x + y * y + z * z);

			// distancia em unsigend_int
			unsigned int distance = distance_float * lidar_range_division_factor;

			// Compute horizontal angle (azimuth) in radians
			_Float32 angle_horizontal = atan2(y, x);
			
			// Converte p/ graus
			angle_horizontal = angle_horizontal * 180 / M_PI;

			// Compute vertical angle (elevation) in radians
			_Float32 angle_vertical = atan2(z, sqrt(x * x + y * y));

			// Converte p/ graus
			angle_vertical = angle_vertical * 180 / M_PI;

			printf("Nº: %d, Distance: %d, Horizontal Angle: %f, Vertical Angle: %f, Intensity: %f\n", 
					processed_points, distance, angle_horizontal, angle_vertical, intensity);

			// // Processar apenas se o ângulo vertical estiver entre 3 e 90 graus
			// if (angle_vertical > 3 && angle_vertical < 90)
			// {
			// 	// Armazenar as coordenadas polares
			// 	// points.emplace_back(std::make_tuple(distance, angle_horizontal, angle_vertical));

			// 	// Exibir informações
			// 	printf("Nº: %d, Distance: %d, Horizontal Angle: %f, Vertical Angle: %f, Intensity: %f\n", 
			// 		processed_points, distance, angle_horizontal, angle_vertical, intensity);

			// 	processed_points++;
			// }
		}
		plot_point_cloud(points);
		sleep(30);
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