#include <memory>
#include <iostream>
#include <unistd.h>

#include <carmen/carmen.h>
#include <carmen/grid_mapping.h>
#include <carmen/mapper_interface.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include <cmath>  
#include <nav_msgs/msg/occupancy_grid.hpp>


carmen_map_t occupancy_map;

void argos_map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

void
convert_ros_map_to_carmen_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg, carmen_map_t *map)
{
	// Notar que o mapa do ROS pode ser um tamanho diferente do mapa do CARMEN, e o do carmen se mantém no seu tamanho

	if((map->config.resolution - msg->info.resolution) < 0.0001)
	{
		for(unsigned int i = 0; ((i < msg->info.height) && (i < (unsigned int)map->config.y_size)); i++)
		{
			for(unsigned int j = 0; ((j < msg->info.width) && (j < (unsigned int)map->config.x_size)); j++)
			{
				if(msg->data[j + i * msg->info.width] != -1)
				{
					map->complete_map[i + j * map->config.x_size] = (((double)msg->data[j + i * msg->info.width]) / 100.0);
				}
			}
		}

	}
	else
	{
		printf("Map resolution mismatch.\n");
		printf("ROS map resolution: %f\n", msg->info.resolution);
		printf("CARMEN map resolution: %f\n", map->config.resolution);
		exit(1);
		// i e j são as coordenadas da célula do mapa do carmen
		// o código abaixo não está pronto para funcionar
		// i * map->config.resolution / msg->info.resolution é a coordenada da célula do mapa do ROS
		// for(unsigned int i = 0; ((i < msg->info.width) && ((i * map->config.resolution / msg->info.resolution) < (map->config.x_size))); i++)
		// {
		// 	for(unsigned int j = 0; ((j < msg->info.height) && ((j * map->config.resolution / msg->info.resolution) < map->config.y_size)); j++)
		// 	{
		// 		int ros_i = i * map->config.resolution / msg->info.resolution;
		// 		int ros_j = j * map->config.resolution / msg->info.resolution;
		// 		if(msg->data[ros_j + ros_i * msg->info.width] != -1)
		// 		{
		// 			map->map[i][j] = (((double)msg->data[ros_j + ros_i * msg->info.width]) / 100.0);
		// 		}
		// 	}
		// }
	}
}

class ArgosROSMapper : public rclcpp::Node
{
public:
	ArgosROSMapper()
    : Node("argos_ros_mapper")
    {
        argos_map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/global_costmap/costmap", 10, argos_map_callback);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr argos_map_subscription_;
};

//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//																								//
// Publishers																					//
//																								//
//////////////////////////////////////////////////////////////////////////////////////////////////


static void
publish_map(double timestamp)
{
	carmen_mapper_publish_map_message(&occupancy_map, timestamp);
}
//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//																								//
// Handlers																						//
//																								//
//////////////////////////////////////////////////////////////////////////////////////////////////


void
argos_map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
	carmen_grid_mapping_set_unknown_value(&occupancy_map, 'm'); // reseta o mapa
	convert_ros_map_to_carmen_map(msg, &occupancy_map); // converte
	publish_map(carmen_get_time()); // publica
}


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
//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//																								//
// Inicializations																				//
//																								//
//////////////////////////////////////////////////////////////////////////////////////////////////


void
argos_ros_mapper_read_parameters(int argc, char **argv)
{
	double map_resolution, map_width, map_height;
	carmen_param_t param_list[] =
	{
		{(char *) "mapper",  (char *) "map_grid_res", CARMEN_PARAM_DOUBLE, &map_resolution, 0, NULL},
		{(char *) "mapper",  (char *) "map_width", CARMEN_PARAM_DOUBLE, &map_width, 0, NULL},
		{(char *) "mapper",  (char *) "map_height", CARMEN_PARAM_DOUBLE, &map_height, 0, NULL},
	};
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	occupancy_map.config.x_size = round(map_width / map_resolution);
	occupancy_map.config.y_size = round(map_height / map_resolution);
	occupancy_map.config.resolution = map_resolution;
	carmen_grid_mapping_create_new_map(&occupancy_map, occupancy_map.config.x_size, occupancy_map.config.y_size, occupancy_map.config.resolution, 'm');
	occupancy_map.config.x_origin = 0.0;
	occupancy_map.config.y_origin = 0.0;
	occupancy_map.config.map_name = (char *) "argos_ros_map";
	strcpy(occupancy_map.config.origin, "argos");
}


void
define_messages()
{
	carmen_mapper_define_map_message();
}


int 
main(int argc, char **argv)
{
	signal(SIGINT, shutdown_module);

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	argos_ros_mapper_read_parameters(argc, argv);
    define_messages();

	//initialize_ipc();

	// Init ROS node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArgosROSMapper>());
    rclcpp::shutdown();

    return (0);
}
