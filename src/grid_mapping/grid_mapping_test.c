#include <carmen/carmen.h>
#include <carmen/grid_mapping.h>
#include <carmen/grid_mapping_interface.h>

/*
 * grid_mapping_test.c
 *
 *  Created on: 11/09/2012
 *      Author: romulo
 */
int
main(int argc, char **argv)
{
	int x, y;
	carmen_ipc_initialize(argc, argv);

	carmen_point_t global_pos;
	carmen_map_t map1, map2;

	global_pos.x = 55;
	global_pos.y = 55;

	carmen_grid_mapping_get_map_by_origin("../../data/grid_mapping_test", global_pos, NULL, &map1);

	double val = 0;




	for(int i = 0; i < 3; i++)
	{
		for(x = i * map1.config.x_size/3; x < ((i + 1) * map1.config.x_size/3); x++)
		{
			val = i * 0.5;
			for(int j = 0; j < 3; j++)
			{
				for(y = j * map1.config.y_size/3; y < ((j + 1) * map1.config.y_size/3); y++)
				{
					map1.complete_map[x * map1.config.y_size + y] = val;
				}

				val += 0.5;

				if(val > 1)
					val = 0;
			}

		}
	}

	map1.config.map_name = malloc(sizeof(char) * 100);
	sprintf(map1.config.map_name, "origin:%d,%d", (int)map1.config.x_origin, (int)map1.config.y_origin);

	carmen_grid_mapping_publish_message(&map1, carmen_get_time());

	global_pos.x = 20;
	global_pos.y = 139;

	carmen_grid_mapping_get_map_by_origin("../../data/grid_mapping_test", global_pos, &map1, &map2);

	carmen_grid_mapping_publish_message(&map2, carmen_get_time());



	carmen_ipc_disconnect();

	return 0;
}
