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
#include <ctype.h>
#include "map_io.h"

carmen_map_p
read_beesoft_map(char *filename)
{
  FILE *in_fp;
  carmen_map_p map;
  int x_index, y_index;
  double *map_ptr;
  char buf[1024];

  map = (carmen_map_p)calloc(1, sizeof(carmen_map_t));
  carmen_test_alloc(map);

  in_fp = fopen(filename, "r");
  if (in_fp == NULL)
    carmen_die_syserror("Couldn't open %s for reading", filename);
  
  while (fgets(buf, 1024, in_fp) != NULL && 
	 strncmp("global_map[0]", buf, 13) != 0)
    {
      if (strncmp("robot_specifications->resolution", buf, 32) == 0) 
	{
	  sscanf(buf+32, "%lf", &(map->config.resolution));
	  map->config.resolution /= 100.0;
	}
    }

  sscanf(buf, "%*s %d %d", &(map->config.y_size), &(map->config.x_size));

  map->complete_map = (double *)
    calloc(map->config.x_size*map->config.y_size, sizeof(double));
  carmen_test_alloc(map->complete_map);

  map->map = (double **)calloc(map->config.x_size, sizeof(double *));
  carmen_test_alloc(map->map);
  for (x_index = 0; x_index < map->config.x_size; x_index++) 
    map->map[x_index] = map->complete_map+x_index*map->config.y_size;

  map_ptr = map->complete_map;
  for (x_index = 0; x_index < map->config.x_size; x_index++) 
    for (y_index = 0; y_index < map->config.y_size; y_index++) 
      {
	fscanf(in_fp, "%f", map_ptr);
	if (*map_ptr >= 0)
	  *map_ptr = 1- *map_ptr;
	map_ptr++;
      }
  fclose(in_fp);

  return map;
}

int
main(int argc, char **argv) 
{
  char *in_filename, *out_filename;
  carmen_FILE *out_fp;
  char buf[1024];
  carmen_map_p map;

  if (argc != 3)
    carmen_die("Usage: %s <bee_filename> <carmen_filename>\n", argv[0]);
  
  in_filename = argv[1];
  out_filename = argv[2];

  if (carmen_file_exists(out_filename))
    {
      printf("Overwrite %s? ", out_filename);
      scanf("%s", buf);
      if (tolower(buf[0]) != 'y')
	exit(0);
    }

  map = read_beesoft_map(in_filename);
  
  out_fp = carmen_fopen(out_filename, "w");
  if (out_fp == NULL)
    carmen_die("Couldn't open %s for writing : %s\n", out_filename, 
	       strerror(errno));

  if (carmen_map_write_id(out_fp) < 0)
    carmen_die_syserror("Couldn't write map id to %s", out_filename);

  sprintf(buf, "Created from %s", out_filename);
  if (carmen_map_write_creator_chunk(out_fp, "bee2carmen", buf) < 0)
    carmen_die_syserror("Couldn't write creator chunk to %s", out_filename);

  if (carmen_map_write_gridmap_chunk(out_fp, map->map, map->config.x_size,
				     map->config.y_size, 
				     map->config.resolution) < 0)
    carmen_die_syserror("Couldn't write gridmap chunk to %s", out_filename);
  
  carmen_fclose(out_fp);

  return 0;
}
