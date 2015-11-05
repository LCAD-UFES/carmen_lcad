/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#include <carmen/carmen.h>
#include "map_io.h"

void
usage(char *fmt, ...)
{
  va_list args;
  
  va_start(args, fmt);
  vfprintf(stderr, fmt, args);
  va_end(args);

  fprintf(stderr, "Usage: generate_blank -w <width> -h <height> -r <resolution> <filename> \n");
  exit(-1);
}

int
main(int argc, char **argv)
{
  int width = 0, height = 0;
  double resolution = 0.0;
  char *filename = NULL;
  carmen_map_t map;
  int index;
  carmen_FILE *out_file;

  carmen_read_commandline_parameters(argc, argv);

  if (argc > 1)
    {
      if (strcmp(argv[argc-1], "-w") != 0 && 
	  strcmp(argv[argc-1], "-h") != 0 &&
	  strcmp(argv[argc-2], "-w") != 0 && 
	  strcmp(argv[argc-2], "-h") != 0)
	filename = argv[argc-1];	  
    }

  carmen_process_param_int("w", usage, &width);
  carmen_process_param_int("h", usage, &height);

  carmen_process_param_double("r", usage, &resolution);

  if (width <= 0)
    {
      printf("Map width: ");
      scanf("%d", &width);
    }

  if (height <= 0)
    {
      printf("Map height: ");
      scanf("%d", &height);
    }

  if (resolution <= 0)
    {
      printf("Map resolution: ");
      scanf("%lf", &resolution);
    }

  if (filename == NULL)
    {
      printf("Filename: ");
      filename = (char *)calloc(1024, sizeof(char));
      carmen_test_alloc(filename);
      scanf("%s", filename);
    }

  printf("Creating map %d x %d, resolution %.2f, storing in %s\n", 
	 width, height, resolution, filename);

  map.config.x_size = width;
  map.config.y_size = height;
  map.config.resolution = resolution;
  map.config.map_name = filename;

  map.complete_map = (double *)calloc(width*height, sizeof(double));
  carmen_test_alloc(map.complete_map);
  for (index = 0; index < width*height; index++)
    map.complete_map[index] = -1;

  map.map = (double **)calloc(width, sizeof(double *));
  carmen_test_alloc(map.map);

  for (index = 0; index < width; index++)    
    map.map[index] = map.complete_map+index*height;

  out_file = carmen_fopen(filename, "w");
  if (out_file == NULL)
      carmen_die("Couldn't open %s for writing map: %s\n", filename, 
		 strerror(errno));
  carmen_map_write_id(out_file);
  carmen_map_write_creator_chunk(out_file, "generate_blank", "blank map");
  carmen_map_write_gridmap_chunk(out_file, map.map, width, height, resolution);
  carmen_fclose(out_file);
      
  return 0;
}
