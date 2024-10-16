#include <memory>
#include <iostream>
#include <unistd.h>

#include <carmen/carmen.h>
#include <carmen/velodyne_messages.h>
#include <carmen/velodyne_interface.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#define lidar_range_division_factor 1000
#define max_points 50
#define msg_accumulated_to_carmen 15

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
    1.50 , 2.56 , 3.81 , 4.05 , 6.32 , 7.59 , 8.87 , 10.15, 11.44, 12.72, 
    13.99, 15.29, 16.58, 17.87, 19.16, 19.95, 21.74, 23.04, 24.34, 25.63, 
    27.06, 28.06, 29.68, 31.45, 33.27, 35.10, 36.71, 38.05, 39.39, 40.88,
    42.24, 44.26, 45.26, 47.28, 48.28, 49.66, 51.31, 52.13, 54.04, 56.00, 
    57.51, 58.74, 60.45, 62.16, 63.62, 65.16, 67.27, 69.36, 70.36, 71.62, 73.01  
};

int debug_option = 0;

void
argos_lidar_read_parameters(int argc, char **argv)
{
    // Lendo argumento de debug
    for (int i = 1; i < argc; i++)
    {
        if (!strcmp(argv[i], "-debug"))
        {
            // Essa função "i+1<argc"
            if (i + 1 < argc)
            {
                debug_option = atoi(argv[i + 1]);
            }
        }
    }
}

void
plot_point_cloud(std::vector<std::tuple<_Float32,_Float32,_Float32,_Float32>> points_gnuplot)
{
    // Plota um vetor de pontos xyz no gnuplot
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
	for (auto point : points_gnuplot)
	{
		_Float32 x, y, z, intensity;
		std::tie(x, y, z, intensity) = point;
		fprintf(points_file, "%f %f %f\n", x, y, z);

		// _Float32 distance_float = sqrt(x * x + y * y + z * z);
		// unsigned int distance = distance_float * lidar_range_division_factor;

		// _Float32 angle_horizontal = atan2(y, x);
		// angle_horizontal = angle_horizontal * 180.0 / M_PI;

		// _Float32 angle_vertical = acos(z / distance_float);
		// angle_vertical = angle_vertical * 180.0 / M_PI;

		// printf("N: %d, x: %f, y: %f, z: %f, Distance: %d, Horizontal Angle: %f, Vertical Angle: %f\n",
		// 		index, x, y, z, distance, angle_horizontal, angle_vertical);
		index++;
	}

	fclose(points_file);

        fprintf(gnuplot_pipe, "splot ""'./mpp.txt' using 1:2:3 with points pointtype 7 pointsize 1 title \"Pontos x, y, z\"");

	fprintf(gnuplot_pipe, "\n");

	fflush(gnuplot_pipe);
}

void
plot_accumulated_points(std::vector<std::vector<std::tuple<_Float32,_Float32,_Float32, _Float32>>> vector_accumulator)
{
    // recebe um vetor de vetor de pontos xyz, acumular em um só vetor e plota no gnuplot
    std::vector<std::tuple<_Float32,_Float32,_Float32, _Float32>> accumulator;
    for (__uint32_t i = 0; i < max_points; i++)
    {
        accumulator.reserve(vector_accumulator[i].size());
        for (__uint32_t j = 0; j < vector_accumulator[i].size(); j++)
        {
            _Float32 x, y, z, intensity;
            std::tie(x, y, z, intensity) = vector_accumulator[i][j];
            accumulator.emplace_back(std::make_tuple(x, y, z, intensity));
        }
    }
    plot_point_cloud(accumulator);
}

double
interpolate(double x0, double y0, double x1, double y1, double x)
{
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}

void
process_points(std::vector<std::tuple<_Float32,_Float32,_Float32,_Float32>> points_xyz, bool debug_on = false)
{
	std::vector<SphericalLidarPoint> points_spherical;
	points_spherical.reserve(points_xyz.size());
    int index = 0;

    if(debug_on)
    {
        printf("----------------------------------------------------------------\
                ----------------------------------------------------------------------------\n");
        printf("                                                        Pontos Recebidos\n");
    }

    // Funcao para converter pontos cartesianos para esfericos
    _Float32 last_vertical_angle = 0;
    _Float32 delta_vertical_angle = 0;
	for (auto point : points_xyz)
	{
		_Float32 x, y, z, intensity;
		std::tie(x, y, z, intensity) = point;

		_Float32 distance_float = sqrt(x * x + y * y + z * z);
		unsigned int distance = distance_float * lidar_range_division_factor;

		_Float32 angle_horizontal = atan2(y, x);
		angle_horizontal = angle_horizontal * 180.0 / M_PI;

		_Float32 angle_vertical = acos(z / distance_float);
		angle_vertical = angle_vertical * 180.0 / M_PI;

		points_spherical.emplace_back(distance, angle_horizontal, angle_vertical, intensity);

        if(debug_on)
        {
            delta_vertical_angle = angle_vertical - last_vertical_angle;
            printf("N: %d, x: %f, y: %f, z: %f, Distance: %d, Horizontal Angle: %f, Vertical Angle: %f, delta vertical angle: %f, Intensity: %f\n",
                index, x, y, z, distance, angle_horizontal, angle_vertical, delta_vertical_angle, intensity);
            index++;
            last_vertical_angle = angle_vertical;
        }
	}

    /*
    // Testando se a conversão está correta
    if(debug_on)
    {
        printf("----------------------------------------------------------------\
            ----------------------------------------------------------------------------\n");
        printf("                                                        Pontos Recebidos\n");
        std::vector<std::tuple<_Float32,_Float32,_Float32, _Float32>> points_test;
        points_test.reserve(points_xyz.size());
        for (auto point : points_spherical)
        {
            _Float32 r, theta, phi, intensity;
            r = point.distance / (float) lidar_range_division_factor;
            theta = point.horizontal_angle * M_PI / 180.0;
            phi = point.vertical_angle * M_PI / 180.0;
            intensity = point.intensity;

            _Float32 x = r*sin(phi)*cos(theta);
            _Float32 y = r*sin(phi)*sin(theta);
            _Float32 z = r*cos(phi);

            points_test.emplace_back(x, y, z, intensity);
            //printf("Distance: %d, Horizontal Angle: %f, Vertical Angle: %f, Intensity: %f\n",
            //		 distance, angle_horizontal, angle_vertical, intensity);
        }
        plot_point_cloud(points_test);
    }
    */

    carmen_velodyne_variable_scan_message carmen_msg;
    carmen_msg.number_of_shots = 0;
    carmen_msg.host = carmen_get_host();

    _Float32 previous_vertical_angle = 0;
    bool vertical_angle_rising = true;
    std::vector<std::tuple<__uint32_t,bool>> turning_point_indexes;
    std::tuple<__uint32_t,bool> turning_point;
    std::tuple<__uint32_t,bool> last_turning_point;

    // Find the local maxima and minima
    turning_point_indexes.emplace_back(std::make_tuple(0, true));
    for (uint32_t i = 0; i < points_spherical.size(); i++)
    {
        if (points_spherical[i].vertical_angle > previous_vertical_angle && !vertical_angle_rising)
        {
            vertical_angle_rising = true;
            turning_point_indexes.emplace_back(std::make_tuple(i, vertical_angle_rising));
        }
        else if (points_spherical[i].vertical_angle < previous_vertical_angle && vertical_angle_rising)
        {
            vertical_angle_rising = false;
            turning_point_indexes.emplace_back(std::make_tuple(i, vertical_angle_rising));
        }
        previous_vertical_angle = points_spherical[i].vertical_angle;
    }
    turning_point_indexes.emplace_back(std::make_tuple(points_spherical.size() - 1, !vertical_angle_rising));


    if(debug_on)
    {
        // Printando turning points
        printf("----------------------------------------------------------------\
                ----------------------------------------------------------------------------\n");
        printf("                                                        Turning Points\n");
        for (uint32_t i = 0; i < turning_point_indexes.size(); i++)
        {
            turning_point = turning_point_indexes[i];
            if(i > 0)
            {
                last_turning_point = turning_point_indexes[i-1];
                printf("number of points in the shot: %d\n", (std::get<0>(turning_point) - std::get<0>(last_turning_point)));
            }
            printf("Turning Point %d: %d\n", i, std::get<0>(turning_point));
        }
    }

    std::vector<carmen_velodyne_shot> shots;
    shots.reserve(turning_point_indexes.size());

    // Each rise of vertical angle from 0 to 90 degrees become a velodyne_shot
    // Same for dawn from 90 to 0
    for (uint32_t i = 1; i < turning_point_indexes.size(); i++) // Itera para cada "meia senoide"
    {
        // Insufficient data to consider it a shot
        turning_point = turning_point_indexes[i];
        last_turning_point = turning_point_indexes[i-1];
        uint32_t number_of_points = std::get<0>(turning_point) - std::get<0>(last_turning_point);
        if (number_of_points < shot_size)
        {
            continue;
        }

        std::vector<SphericalLidarPoint>::iterator slice_begin = points_spherical.begin() + std::get<0>(last_turning_point);
        std::vector<SphericalLidarPoint>::iterator slice_end = points_spherical.begin() + std::get<0>(turning_point);

        std::array<unsigned int, shot_size> distances;
        std::array<unsigned short, shot_size> intensity;
        for (uint32_t j = 0; j < shot_size; j++)
        {
            std::vector<SphericalLidarPoint>::iterator it;
            if(std::get<1>(turning_point))
            {
                it = std::lower_bound(slice_begin, slice_end, (90 - lidar_angles[j]), [](const SphericalLidarPoint &p, double angle) {
                    return p.vertical_angle > angle;
                });
            }
            else
            {
                it = std::lower_bound(slice_begin, slice_end, (90 - lidar_angles[j]), [](const SphericalLidarPoint &p, double angle) {
                    return p.vertical_angle < angle;
                });
            }

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
                distances[j] = interpolate((it-1)->vertical_angle, (it-1)->distance, it->vertical_angle, it->distance, (90 - lidar_angles[j]));
                intensity[j] = interpolate((it-1)->vertical_angle, (it-1)->intensity, it->vertical_angle, it->intensity, (90 - lidar_angles[j]));
            }
        }
        double angle = 0.0;
        for(__uint32_t j = std::get<0>(last_turning_point); j < (std::get<0>(last_turning_point) + shot_size); j++)
        {
            angle += points_spherical[j].horizontal_angle;
        }
        angle = angle / shot_size;

        // double angle = std::accumulate(
        //     slice_begin,
        //     slice_end,
        //     0,
        //     [](double sum, const SphericalLidarPoint& point) {
        //         return sum + point.horizontal_angle; // Accumulate horizontal_angle
        //     }
        // ) / number_of_points;


        carmen_velodyne_shot shot;
        shot.distance = new unsigned int[shot_size];
        shot.intensity = new unsigned short[shot_size];

        shot.angle = angle;
        shot.shot_size = shot_size;
        for(__uint32_t i = 0; i < shot_size; i++)
        {
            shot.distance[i] = distances[i];
            shot.intensity[i] = intensity[i];
        }

        shots.push_back(shot);

        carmen_msg.number_of_shots++;
    }
    carmen_msg.timestamp = carmen_get_time();
    carmen_msg.partial_scan = &shots.front();
    if (carmen_velodyne_publish_variable_scan_message(&carmen_msg, 3) == IPC_Error)
    {
        fprintf(stderr, "Failed to publish variable scan message\n");
    }
    // Printando mensagem do carmen
    if(debug_on)
    {
        printf("----------------------------------------------------------------\
                ----------------------------------------------------------------------------\n");
        printf("                                                        Carmen msg\n");
        printf("Numero de shots: %d\n", carmen_msg.number_of_shots);
        printf("Shots:\n");
        for(int i = 0; i < carmen_msg.number_of_shots; i++)
        {
            printf("--> Shot: %d, Angle: %lf\n", i, carmen_msg.partial_scan[i].angle);
            for (int j = 0; j < carmen_msg.partial_scan[i].shot_size; j++)
            {
                printf("Point: %d, Distance: %d, Intensity: %d\n", j, carmen_msg.partial_scan[i].distance[j], carmen_msg.partial_scan[i].intensity[j]);
                /* code */
            }
            
        }

    }

    if(debug_on)
    {
        // Testando se a conversão está correta
        std::vector<std::tuple<_Float32,_Float32,_Float32, _Float32>> points_test;
        points_test.reserve(shot_size*carmen_msg.number_of_shots);
        for (int i = 0; i < carmen_msg.number_of_shots; i++)
        {   
            for(uint32_t j = 0; j < shot_size; j++)
            {
                _Float32 r, theta, phi, intensity;
                r = carmen_msg.partial_scan[i].distance[j] / (float) lidar_range_division_factor;
                theta = carmen_msg.partial_scan[i].angle * M_PI / 180.0;
                phi = (90 - lidar_angles[j])* M_PI / 180.0;
                intensity = 0.0;

                _Float32 x = r*sin(phi)*cos(theta);
                _Float32 y = r*sin(phi)*sin(theta);
                _Float32 z = r*cos(phi);

                points_test.emplace_back(x, y, z, intensity);
            }
        }
        plot_point_cloud(points_test);
    }


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
        // Opcao passada como argumento para debugar
		// opcao 0 = recebe, ajeita e manda pro carmen (padrão),
        // opcao 1 = como opcao 0, mas com print de debug,
		// opcao 2 = plota em tempo real, 
		// opcao 3 = plota mensagens acumuladas, 
		// opcao 4 = plota uma msg por vez e pode mandar msg no terminal para plotar a próxima
        // opcao 5 = como opcao 0, mas acumula antes de mandar para o carmen
		if (debug_option == 0)
		{
			std::vector<std::tuple<_Float32,_Float32,_Float32, _Float32>> points_callback;
            points_callback.reserve(msg->width);

            for (uint32_t i = 0; i < msg->width; i++)
            {
                _Float32 x, y, z, intensity;
                memcpy(&x, &msg->data[0 + msg->point_step * i], 4);
                memcpy(&y, &msg->data[4 + msg->point_step * i], 4);
                memcpy(&z, &msg->data[8 + msg->point_step * i], 4);
                memcpy(&intensity, &msg->data[16 + msg->point_step * i], 4);
                points_callback.emplace_back(std::make_tuple(x, y, z, intensity));
            }
            process_points(points_callback);
		}
        else if (debug_option == 1)
        {
            static bool first_time = true;
            if(first_time)
            {
                std::vector<std::tuple<_Float32,_Float32,_Float32, _Float32>> points_callback;
                points_callback.reserve(msg->width);

                for (uint32_t i = 0; i < msg->width; i++)
                {
                    _Float32 x, y, z, intensity;
                    memcpy(&x, &msg->data[0 + msg->point_step * i], 4);
                    memcpy(&y, &msg->data[4 + msg->point_step * i], 4);
                    memcpy(&z, &msg->data[8 + msg->point_step * i], 4);
                    memcpy(&intensity, &msg->data[16 + msg->point_step * i], 4);
                    points_callback.emplace_back(std::make_tuple(x, y, z, intensity));
                }
                process_points(points_callback, debug_option=true);
                first_time = false;
            }
        }
		else if (debug_option == 2)
		{
			std::vector<std::tuple<_Float32,_Float32,_Float32,_Float32>> points;
			points.reserve(msg->width);
			for (uint32_t i = 0; i < msg->width; i++)
			{
				_Float32 x, y, z, intensity;
				memcpy(&x, &msg->data[0 + msg->point_step * i], 4);
				memcpy(&y, &msg->data[4 + msg->point_step * i], 4);
				memcpy(&z, &msg->data[8 + msg->point_step * i], 4);
				memcpy(&intensity, &msg->data[16 + msg->point_step * i], 4);
				points.emplace_back(std::make_tuple(x, y, z, intensity));
				// shot.distance = sqrt(x*x + y*y + z*z);
				printf("x: %f, y: %f, z: %f intensity: %f\n", x, y, z, intensity);
			}
			plot_point_cloud(points);
		}
		else if (debug_option == 3)
		{
			using Tuple4f = std::tuple<_Float32,_Float32,_Float32, _Float32>;
			static std::vector<std::vector<Tuple4f>> vector_accumulator;
			static int counted_msg = 0;
			if (counted_msg == 0)
			{
				vector_accumulator.reserve(msg->width*max_points);
			}
			
			if (counted_msg < max_points)
			{
				std::vector<std::tuple<_Float32,_Float32,_Float32, _Float32>> points_callback;
				points_callback.reserve(msg->width);

				for (uint32_t i = 0; i < msg->width; i++)
				{
					_Float32 x, y, z, intensity;
					memcpy(&x, &msg->data[0 + msg->point_step * i], 4);
					memcpy(&y, &msg->data[4 + msg->point_step * i], 4);
					memcpy(&z, &msg->data[8 + msg->point_step * i], 4);
					memcpy(&intensity, &msg->data[16 + msg->point_step * i], 4);
					points_callback.emplace_back(std::make_tuple(x, y, z, intensity));
				}
				vector_accumulator.push_back(points_callback);
				counted_msg += 1;
			}
			else if (counted_msg == max_points)
			{
				printf("acumulado\n");
				
				plot_accumulated_points(vector_accumulator);
				counted_msg += 1;
			}
		}
		else if (debug_option == 4)
		{
			using Tuple4f = std::tuple<_Float32,_Float32,_Float32, _Float32>;
			static std::vector<std::vector<Tuple4f>> vector_accumulator;
			static int counted_msg = 0;
			static int msg_showing = 0;
			if (counted_msg == 0)
			{
				vector_accumulator.reserve(msg->width*max_points);
			}
			
			if (counted_msg < max_points)
			{
				std::vector<std::tuple<_Float32,_Float32,_Float32, _Float32>> points_callback;
				points_callback.reserve(msg->width);

				for (uint32_t i = 0; i < msg->width; i++)
				{
					_Float32 x, y, z, intensity;
					memcpy(&x, &msg->data[0 + msg->point_step * i], 4);
					memcpy(&y, &msg->data[4 + msg->point_step * i], 4);
					memcpy(&z, &msg->data[8 + msg->point_step * i], 4);
					memcpy(&intensity, &msg->data[16 + msg->point_step * i], 4);
					points_callback.emplace_back(std::make_tuple(x, y, z, intensity));
				}
				vector_accumulator.push_back(points_callback);
				counted_msg += 1;
			}
			else if (counted_msg == max_points)
			{
				printf("acumulado\n");
				char s[100];
				plot_point_cloud(vector_accumulator[msg_showing]);
				scanf("%s", s);
				if (msg_showing < max_points - 1)
				{
					msg_showing += 1;
				}
			}
		}
        else if (debug_option == 5)
        {
			static std::vector<std::tuple<_Float32,_Float32,_Float32, _Float32>> vector_accumulator;
			static int counted_msg = 0;
            static int reserved_points = 0;
            printf("opcao 5: %d\n", counted_msg);
            reserved_points += msg->width;
			vector_accumulator.reserve(reserved_points);
            for (uint32_t i = 0; i < msg->width; i++)
            {
                _Float32 x, y, z, intensity;
                memcpy(&x, &msg->data[0 + msg->point_step * i], 4);
                memcpy(&y, &msg->data[4 + msg->point_step * i], 4);
                y = -y;
                memcpy(&z, &msg->data[8 + msg->point_step * i], 4);
                memcpy(&intensity, &msg->data[16 + msg->point_step * i], 4);
                vector_accumulator.emplace_back(std::make_tuple(x, y, z, intensity));
            }
			if (counted_msg < (msg_accumulated_to_carmen - 1))
			{
				counted_msg += 1;
			}
			else if (counted_msg == (msg_accumulated_to_carmen - 1))
			{
				printf("acumulado\n");
				
				process_points(vector_accumulator);
                vector_accumulator.clear();
                reserved_points = 0;
				counted_msg = 0;
			}
        }
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

    argos_lidar_read_parameters(argc, argv);

	// Init ROS node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarSubscriber>());
    rclcpp::shutdown();

    return 0;
}
