#include <memory>

#include <carmen/carmen.h>
#include <carmen/velodyne_messages.h>
#include <carmen/velodyne_interface.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#define lidar_range_division_factor 1000

using std::placeholders::_1;

struct RectangularPoint
{
    _Float32 x;
    _Float32 y;
    _Float32 z;
};

struct SphericalLidarPoint
{
    unsigned int distance;
    double horizontal_angle;
    double vertical_angle;
    unsigned short intensity;

    SphericalLidarPoint(unsigned int distance, double horizontal_angle, double vertical_angle, unsigned short intensity) :
    distance(distance), horizontal_angle(horizontal_angle), vertical_angle(vertical_angle), intensity(intensity) {}
};


const uint32_t shot_size = 51;
double lidar_angles[51] = {
    0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0,
    11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0,
    21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 30.0,
    31.0, 32.0, 33.0, 34.0, 35.0, 36.0, 37.0, 38.0, 39.0, 40.0,
    41.0, 42.0, 43.0, 44.0, 45.0, 46.0, 47.0, 48.0, 49.0, 50.0
};


void
plot_point_cloud(std::vector<std::tuple<_Float32,_Float32,_Float32,_Float32>> points)
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
		_Float32 x, y, z, intensity;
		std::tie(x, y, z, intensity) = point;
		fprintf(points_file, "%f %f %f\n", x, y, z);

//		_Float32 distance_float = sqrt(x * x + y * y + z * z);
//		unsigned int distance = distance_float * lidar_range_division_factor;
//
//		_Float32 angle_horizontal = atan2(y, x);
//		angle_horizontal = angle_horizontal * 180.0 / M_PI;
//
//		_Float32 angle_vertical = atan2(z, sqrt(x * x + y * y));
//		angle_vertical = angle_vertical * 180.0 / M_PI;
//
//		printf("N: %d, x: %f, y: %f, z: %f, Distance: %d, Horizontal Angle: %f, Vertical Angle: %f\n",
//				index, x, y, z, distance, angle_horizontal, angle_vertical);
		index++;
	}

	fclose(points_file);
	fprintf(gnuplot_pipe, "splot "
			"'./mpp.txt' using 1:2:3 with points pointtype 7 pointsize 1 title \"Pontos x, y, z\"");

	fprintf(gnuplot_pipe, "\n");

	fflush(gnuplot_pipe);
}


double
interpolate(double x0, double y0, double x1, double y1, double x)
{
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}


void
process_points(std::vector<std::tuple<_Float32,_Float32,_Float32,_Float32>> points_xyz_vector)
{
	std::vector<SphericalLidarPoint> points;
	points.reserve(points_xyz_vector.size());
	//plot_point_cloud(points_xyz_vector);

    // Funcao para converter pontos cartesianos para esfericos
	for (auto point : points_xyz_vector)
	{
		_Float32 x, y, z, intensity;
		std::tie(x, y, z, intensity) = point;

		_Float32 distance_float = sqrt(x * x + y * y + z * z);
		unsigned int distance = distance_float * lidar_range_division_factor;

		_Float32 angle_horizontal = atan2(y, x);
		angle_horizontal = angle_horizontal * 180.0 / M_PI;

		_Float32 angle_vertical = atan2(z, sqrt(x * x + y * y));
		angle_vertical = angle_vertical * 180.0 / M_PI;

		points.emplace_back(distance, angle_horizontal, angle_vertical, intensity);
		//printf("Distance: %d, Horizontal Angle: %f, Vertical Angle: %f, Intensity: %f\n",
		//		 distance, angle_horizontal, angle_vertical, intensity);
	}

    // std::vector<std::tuple<_Float32,_Float32,_Float32,_Float32>> xyz_points;
    // xyz_points.reserve(points_xyz_vector.size());

    // for (auto point : points)
	// {
    //     _Float32 x, y, z;
    //     _Float32 distance, angle_horizontal, angle_vertical, intensity;
    //     distance = (point.distance/1000.0);
    //     angle_horizontal = point.horizontal_angle;
    //     angle_vertical = point.vertical_angle;
    //     intensity = point.intensity;

    //     x = (distance)*cos(angle_horizontal)*sin(angle_vertical);
    //     y = (distance)*sin(angle_horizontal)*sin(angle_vertical);
    //     z = (distance)*cos(angle_vertical);

    //     xyz_points.emplace_back(std::make_tuple(x, y, z, intensity));
	// 		//printf("shot: %d, point: %d, x: %f, y: %f, z: %f, dist: %d,intensity %f \n", l, m, x, y, z, shot.distance[m], intensity);
    // }
	// plot_point_cloud(xyz_points);


    carmen_velodyne_variable_scan_message carmen_msg;
    carmen_msg.number_of_shots = 0;
    carmen_msg.host = carmen_get_host();

    _Float32 previous_vertical_angle = 0;
    bool vertical_angle_rising = true;
    std::vector<uint32_t> turning_point_indexes;

    // Find the local maxima and minima
    turning_point_indexes.push_back(0);
    for (uint32_t i = 0; i < points.size(); i++)
    {
        if (points[i].vertical_angle > previous_vertical_angle && !vertical_angle_rising)
        {
            vertical_angle_rising = true;
            turning_point_indexes.push_back(i);
        }
        else if (points[i].vertical_angle < previous_vertical_angle && vertical_angle_rising)
        {
            vertical_angle_rising = false;
            turning_point_indexes.push_back(i);
        }
        previous_vertical_angle = points[i].vertical_angle;
    }
    turning_point_indexes.push_back(points.size() - 1);

    std::vector<carmen_velodyne_shot> shots;

    // Each rise of vertical angle from 0 to 90 degrees become a velodyne_shot
    // Same for dawn from 90 to 0
    for (uint32_t i = 1; i < turning_point_indexes.size(); i++) // Itera para cada "meia senoide"
    {
        // Insufficient data to consider it a shot
        uint32_t number_of_points = turning_point_indexes[i] - turning_point_indexes[i-1];
        if (number_of_points < shot_size)
        {
            continue;
        }

        std::vector<SphericalLidarPoint>::iterator slice_begin = points.begin() + turning_point_indexes[i-1];
        std::vector<SphericalLidarPoint>::iterator slice_end = points.begin() + turning_point_indexes[i];

        std::array<unsigned int, shot_size> distances;
        std::array<unsigned short, shot_size> intensity;
        for (uint32_t j = 0; j < shot_size; j++)
        {
            auto it = std::lower_bound(slice_begin, slice_end, lidar_angles[j], [](const SphericalLidarPoint &p, double angle) {
                return p.vertical_angle < angle;
            });

            if (it == slice_begin)
            {
                distances[j] = slice_begin->distance;
                intensity[j] = slice_begin->intensity;
            }
            else if (it == slice_end)
            {
                distances[j] = slice_end->distance;
                intensity[j] = slice_end->intensity;
            }
            else
            {
                distances[j] = interpolate((it-1)->vertical_angle, (it-1)->distance, it->vertical_angle, it->distance, lidar_angles[j]);
                intensity[j] = interpolate((it-1)->vertical_angle, (it-1)->intensity, it->vertical_angle, it->intensity, lidar_angles[j]);
            }
        }
        double angle = std::accumulate(
            slice_begin,
            slice_end,
            0,
            [](double sum, const SphericalLidarPoint& point) {
                return sum + point.horizontal_angle; // Accumulate horizontal_angle
            }
        ) / number_of_points;

        carmen_velodyne_shot shot;
        shot.angle = angle;
        shot.distance = &distances.front();
        shot.intensity = &intensity.front();
        shot.shot_size = shot_size;

        shots.push_back(shot);

        carmen_msg.number_of_shots++;
    }
    carmen_msg.timestamp = carmen_get_time();
    carmen_msg.partial_scan = &shots.front();
    if (carmen_velodyne_publish_variable_scan_message(&carmen_msg, 3) == IPC_Error)
    {
        fprintf(stderr, "Failed to publish variable scan message\n");
    }

    // Plotando a msg do carmen no gnuplot

    // Calculando numero de pontos
    std::vector<std::tuple<_Float32, _Float32, _Float32>> xyz_points;

	// Reconstrução dos pontos a partir dos dados do "carmen"
	for (int i = 0; i < carmen_msg.number_of_shots; ++i)
	{
		const carmen_velodyne_shot &shot = carmen_msg.partial_scan[i];
		double horizontal_angle_deg = shot.angle;
		double horizontal_angle_rad = horizontal_angle_deg * M_PI / 180.0;

		for (int j = 0; j < shot.shot_size; ++j)
		{
			double vertical_angle_deg = lidar_angles[j];
			double vertical_angle_rad = vertical_angle_deg * M_PI / 180.0;

			double distance = static_cast<double>(shot.distance[j]) / lidar_range_division_factor;

			double x = distance * cos(vertical_angle_rad) * cos(horizontal_angle_rad);
			double y = distance * cos(vertical_angle_rad) * sin(horizontal_angle_rad);
			double z = distance * sin(vertical_angle_rad);

			xyz_points.emplace_back(static_cast<_Float32>(x), static_cast<_Float32>(y), static_cast<_Float32>(z));
		}
	}

	// Plotagem dos pontos usando gnuplot (função integrada)
	static bool first_time = true;
	static FILE *gnuplot_pipe;

	if (first_time)
	{
		first_time = false;

		gnuplot_pipe = popen("gnuplot -persist", "w"); // '-persist' mantém a janela aberta após o término do programa
		if (gnuplot_pipe == nullptr)
		{
			fprintf(stderr, "Erro ao abrir o pipe para o gnuplot.\n");
			return;
		}
		fprintf(gnuplot_pipe, "set xrange [-5:5]\n");
		fprintf(gnuplot_pipe, "set yrange [-5:5]\n");
		fprintf(gnuplot_pipe, "set zrange [0:5]\n");
		fprintf(gnuplot_pipe, "set xlabel 'x'\n");
		fprintf(gnuplot_pipe, "set ylabel 'y'\n");
		fprintf(gnuplot_pipe, "set zlabel 'z'\n");
	}

	// Escreve os pontos em um arquivo temporário
	FILE *points_file = fopen("mpp.txt", "w");
	if (points_file == nullptr)
	{
		fprintf(stderr, "Erro ao abrir o arquivo 'mpp.txt' para escrita.\n");
		return;
	}

	int index = 0;
	for (const auto& point : xyz_points)
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

	// Envia o comando para plotar os pontos no gnuplot
	fprintf(gnuplot_pipe, "splot './mpp.txt' using 1:2:3 with points pointtype 7 pointsize 1 title \"Pontos x, y, z\"\n");
	fflush(gnuplot_pipe);

//    int num_pontos = 0;
//    for (int k = 0; k < carmen_msg.number_of_shots; k++)
//	{
//    	num_pontos += carmen_msg.partial_scan[k].shot_size;
//	}
//
//    std::vector<std::tuple<_Float32,_Float32,_Float32,_Float32>> xyz_points;
//    xyz_points.reserve(num_pontos);
//    float vertical_angle = 0.0;
//    float theta = 0.0;
//    float phi = 0.0;
//    float dist = 0.0;
//
//    for (int l = 0; l < carmen_msg.number_of_shots; l++)
//	{
//    	const carmen_velodyne_shot &shot = carmen_msg.partial_scan[l];
//    	vertical_angle = 0.0;
//    	for(int m = 0; m < shot.shot_size; m++)
//    	{
//			_Float32 x, y, z, intensity;
//			intensity = shot.intensity[m];
//			dist = (shot.distance[m]) / 1000.0; // lidar_range_division_factor
//			theta = shot.angle;
//			phi = lidar_angles[m];
//			vertical_angle += 1.0;
//
//			double theta_rad = theta * 180.0 / M_PI;
//			double phi_rad = phi * 180.0 / M_PI;
//
//			x = (dist)*cos(theta_rad)*sin(phi_rad);
//			y = (dist)*sin(theta_rad)*sin(phi_rad);
//			z = (dist)*cos(phi_rad);
//
//			xyz_points.emplace_back(std::make_tuple(x, y, z, intensity));
//			printf("shot: %d, point: %d, x: %f, y: %f, z: %f, dist: %d,intensity %f \n", l, m, x, y, z, shot.distance[m], intensity);
//    	}
//    }
//	plot_point_cloud(xyz_points);

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
//        std::vector<SphericalLidarPoint> points;
		std::vector<std::tuple<_Float32,_Float32,_Float32, _Float32>> points;
        points.reserve(msg->width);

        for (uint32_t i = 0; i < msg->width; i++)
        {
            _Float32 x, y, z, intensity;
			memcpy(&x, &msg->data[0 + msg->point_step * i], 4);
			memcpy(&y, &msg->data[4 + msg->point_step * i], 4);
			memcpy(&z, &msg->data[8 + msg->point_step * i], 4);
			memcpy(&intensity, &msg->data[16 + msg->point_step * i], 4);
			points.emplace_back(std::make_tuple(x, y, z, intensity));
        }
        process_points(points);
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

    carmen_velodyne_define_messages();

	// Init ROS node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarSubscriber>());
    rclcpp::shutdown();

    return 0;
}
