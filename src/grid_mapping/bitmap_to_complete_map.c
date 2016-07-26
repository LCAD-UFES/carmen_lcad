/*
 * bitmap_to_complete_map.c
 *
 *  Created on: 22/07/2016
 *      Author: Luan
 */



#include <dirent.h>
#include <stdio.h>
#include <carmen/carmen.h>
#include "grid_mapping.h"

static char *map_path = ".";

static void
read_parameters(int argc, char **argv)
{
	carmen_param_t param_list[] = {
			{(char *)"command_line",	 (char *)"map_path",	CARMEN_PARAM_STRING, &map_path,	 1, NULL},
	};
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
}

int
main(int argc, char **argv)
{
	carmen_map_t complete_map;

	carmen_ipc_initialize(argc, argv);
	read_parameters(argc, argv);

	//carmen_grid_mapping_init_parameters(map_resolution, block_size_in_meters);

	carmen_grid_mapping_read_complete_map(map_path, &complete_map);

	// headers do bitmap
	FILE *f;

	char bitmap_file_name[1024];
	char new_complete_map_name[1024];

	sprintf(bitmap_file_name,"%s/complete_bitmap.bmp",map_path);

	f = fopen(bitmap_file_name,"rb");	// abre o arquivo bitmap

	unsigned char info[54];
	fread(info, sizeof(unsigned char), 54, f); // lê o cabeçalho do arquivo

	int width = *(int*)&info[18];
	int height = *(int*)&info[22];

	if(width != complete_map.config.x_size || height != complete_map.config.y_size)
	{
		printf("O bitmap não corresponde ao complete_map\n");
		return 0;
	}

	unsigned char *img = NULL;

	int i;
	unsigned char r,g,b;
	double mult = 1.0/255.0;
	double prob;

	int size = 3*width*height;		// tamanho da imagem

	if(img)
	    free(img);

	img = (unsigned char *)malloc(3*width*height);
	fread(img, sizeof(unsigned char), size, f);
	fclose(f);

	for (int x = 0; x < width; x++)
	{
		for (int y = 0; y < height; y++)
		{

			r = img[(x+(y)*width)*3+2];
			g = img[(x+(y)*width)*3+1];
			b = img[(x+(y)*width)*3+0];

			if ((r == g) && (b == g))
			{
				prob = 1.0 - (double) (r * mult);
				complete_map.map[x][y] = prob;
			}
			else{
				complete_map.map[x][y] = -1;
			}

		}
	}

	sprintf(new_complete_map_name, "%s/complete_map2.map", map_path);
	carmen_grid_mapping_save_map(new_complete_map_name, &complete_map);

	printf("Completo!\n%d %d\n", complete_map.config.x_size, complete_map.config.y_size);

	return 1;
}

