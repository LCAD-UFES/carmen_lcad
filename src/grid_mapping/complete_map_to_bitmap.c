/*
 * complete_map_to_bitmap.c
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

	unsigned char *img = NULL;

	int width = complete_map.config.x_size;
	int height = complete_map.config.y_size;
	int i;
	unsigned char r,g,b,prob;

	int filesize = 54 + 3*width*height;

	if(img)
	    free(img);

	img = (unsigned char *)calloc(3 * width * height, sizeof(unsigned char));


	for (int x = 0; x < width; x++)
	{
		for (int y = 0; y < height; y++)
		{
			if(complete_map.map[x][y] == -1)
			{
				r = 0;
				g = 0;
				b = 255;
			} else {
				prob = (unsigned char) ((1 - complete_map.map[x][y]) * 255.0); // converte a probabilidade para [0-255]
				r = prob;
				g = prob;
				b = prob;
			}
			img[(x+(height-1-y)*width)*3+2] = (unsigned char)(r);	// height -1 -y pois no mapa a origem é
			img[(x+(height-1-y)*width)*3+1] = (unsigned char)(g);	// no canto inferior esquerdo e no
			img[(x+(height-1-y)*width)*3+0] = (unsigned char)(b);	// bitmap é no canto superior esquerdo
		}
	}

	// preenche os headers do bitmap
	unsigned char bmpfileheader[14] = {'B','M', 0,0,0,0, 0,0, 0,0, 54,0,0,0};
	unsigned char bmpinfoheader[40] = {40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0, 24,0};
	unsigned char bmppad[3] = {0,0,0};

	bmpfileheader[ 2] = (unsigned char)(filesize    );
	bmpfileheader[ 3] = (unsigned char)(filesize>> 8);
	bmpfileheader[ 4] = (unsigned char)(filesize>>16);
	bmpfileheader[ 5] = (unsigned char)(filesize>>24);

	bmpinfoheader[ 4] = (unsigned char)(width    );
	bmpinfoheader[ 5] = (unsigned char)(width>> 8);
	bmpinfoheader[ 6] = (unsigned char)(width>>16);
	bmpinfoheader[ 7] = (unsigned char)(width>>24);
	bmpinfoheader[ 8] = (unsigned char)(height    );
	bmpinfoheader[ 9] = (unsigned char)(height>> 8);
	bmpinfoheader[10] = (unsigned char)(height>>16);
	bmpinfoheader[11] = (unsigned char)(height>>24);

	char bitmap_file_name[1024];

	sprintf(bitmap_file_name,"%s/complete_bitmap.bmp",map_path);

	f = fopen(bitmap_file_name,"wb");
	fwrite(bmpfileheader,1,14,f);
	fwrite(bmpinfoheader,1,40,f);
	for(i=0; i<height; i++)
	{
	    fwrite(img+(width*(height-i-1)*3),3,width,f);
	    fwrite(bmppad,1,(4-(width*3)%4)%4,f);			// a largura deve ser multipla de 4 no bitmap
	}
	fclose(f);
	printf("Completo!\n%d %d\n", complete_map.config.x_size, complete_map.config.y_size);

	return 1;
}

