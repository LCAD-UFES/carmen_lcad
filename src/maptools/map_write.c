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

void write_fixed_string(carmen_FILE *fp, char *str, int n)
{
  int l, i;

  if (str == NULL)
    l = 0;
  else
    l = strlen(str);
  
  if(l > n)
    l = n;
  for(i = 0; i < l; i++)
    carmen_fprintf(fp, "%c", str[i]);
  if(l < n)
    for(i = l; i < n; i++)
      carmen_fprintf(fp, " ");
}

/*
 * vchunk_size expects different input for different chunk types!
 *
 * GRIDMAP:      int size_x, int size_y
 * OFFLIMITS:    carmen_offlimits_p offlimits_list, int num_items
 * PLACES:       carmen_place_p places, int num_places
 * EXPECTED:     (not implemented)
 * LASERSCANS:   carmen_laser_scan_p scan_list, int num_scans
 * CREATOR:      (none)
 * HMAP:         carmen_hmap_p hmap
 *
 */
static int vchunk_size(unsigned int chunk_type, va_list ap)
{
  int size, num, i;
  carmen_offlimits_p offlimits_list;
  carmen_place_p places;
  carmen_laser_scan_p scan_list;
  carmen_hmap_p hmap;

  size = 10;  // chunk description

  switch(chunk_type) {

  case CARMEN_MAP_GRIDMAP_CHUNK:
    size += 12 + va_arg(ap, int) * va_arg(ap, int) * 4;
    break;

  case CARMEN_MAP_OFFLIMITS_CHUNK:
    size += 4 + 4 + 4;
    offlimits_list = va_arg(ap, carmen_offlimits_p);
    num = va_arg(ap, int);
    for(i = 0; i < num; i++)
      if(offlimits_list[i].type == CARMEN_OFFLIMITS_POINT_ID)
	size += 8;
      else if(offlimits_list[i].type == CARMEN_OFFLIMITS_LINE_ID)
        size += 16;
      else if(offlimits_list[i].type == CARMEN_OFFLIMITS_RECT_ID)
	size += 16;
    break;

  case CARMEN_MAP_GLOBAL_OFFSET_CHUNK:
    size += sizeof(carmen_global_offset_t);
    break;

  case CARMEN_MAP_PLACES_CHUNK:
    places = va_arg(ap, carmen_place_p);
    num = va_arg(ap, int);
    size += 4 + 24 * num;
    for(i = 0; i < num; i++)
      if(places[i].type == CARMEN_NAMED_POSITION_TYPE)
	size += 8;
      else if(places[i].type == CARMEN_NAMED_POSE_TYPE)
	size += 12;
      else if(places[i].type == CARMEN_LOCALIZATION_INIT_TYPE)
	size += 24;
    break;

  case CARMEN_MAP_EXPECTED_CHUNK:
    // not implemented
    break;

  case CARMEN_MAP_LASERSCANS_CHUNK:
    scan_list = va_arg(ap, carmen_laser_scan_p);
    num = va_arg(ap, int);
    size += 4 + 16 * num;
    for(i = 0; i < num; i++)
      size += 4 * scan_list[i].num_readings;
    break;

  case CARMEN_MAP_CREATOR_CHUNK:
    size += 10 + sizeof(time_t) + 80 + 80;
    break;

  case CARMEN_MAP_HMAP_CHUNK:
    hmap = va_arg(ap, carmen_hmap_p);
    size += 4;
    for (i = 0; i < hmap->num_zones; i++)
      size += strlen(hmap->zone_names[i]) + 1;
    size += 4;
    for (i = 0; i < hmap->num_links; i++) {
      size += 4 + 4 + 4*hmap->links[i].degree;
      num = 3 * 4 * hmap->links[i].degree;
      switch (hmap->links[i].type) {
      case CARMEN_HMAP_LINK_DOOR:       num *= 2; break;
      case CARMEN_HMAP_LINK_ELEVATOR:   num *= 1; break;
      }
      size += num;
    }
  }

  return size;
}

static int chunk_size(unsigned int chunk_type, ...)
{
  int size;
  va_list ap;

  va_start(ap, chunk_type);
  size = vchunk_size(chunk_type, ap);
  va_end(ap);

  return size;
}

static int named_chunk_size(unsigned int chunk_type, char *name, ...)
{
  int size;
  va_list ap;

  va_start(ap, name);
  size = vchunk_size(chunk_type, ap) + strlen(name) + 1;
  va_end(ap);

  return size;  
}

int carmen_map_write_all(carmen_FILE *fp, double **prob,
			 int size_x, int size_y, double resolution,
			 char *comment_origin, char *comment_description,
			 char *creator_origin, char *creator_description,
			 carmen_place_p places, int num_places,
			 carmen_offlimits_p offlimits_list, int offlimits_num_items,
			 carmen_laser_scan_p scan_list __attribute__ ((unused)),
			 int num_scans __attribute__ ((unused)))
{
  if (fp == NULL || prob == NULL)
    return -1;

  if (comment_origin == NULL)
    comment_origin = "";
  if (comment_description == NULL)
    comment_description = "";
  if (carmen_map_write_comment_chunk(fp, size_x, size_y, resolution, comment_origin,
				     comment_description) < 0)
    return -1;

  if (carmen_map_write_id(fp) < 0)
    return -1;

  if (creator_origin == NULL)
    creator_origin = "";
  if (creator_description == NULL)
    creator_description = "";
  if (carmen_map_write_creator_chunk(fp, creator_origin, creator_description) < 0)
    return -1;

  if (carmen_map_write_gridmap_chunk(fp, prob, size_x, size_y, resolution) < 0)
    return -1;

  if (places != NULL && num_places > 0)
    if (carmen_map_write_places_chunk(fp, places, num_places) < 0)
      return -1;

  if (offlimits_list != NULL && offlimits_num_items > 0)
    if (carmen_map_write_offlimits_chunk(fp, offlimits_list, offlimits_num_items) < 0)
      return -1;

  if (scan_list != NULL && num_scans > 0)
    if (carmen_map_write_laserscans_chunk(fp, scan_list, num_scans) < 0)
      return -1;

  return 0;
}

int carmen_map_write_comment_chunk(carmen_FILE *fp, int size_x, int size_y, 
				 double resolution, char *origin,
				 char *description)
{
  time_t t = time(NULL);

  carmen_fprintf(fp, "################################################"
		     "#####\n");
  carmen_fprintf(fp, "#\n");
  carmen_fprintf(fp, "# Carnegie Mellon Robot Toolkit (CARMEN) "
		     "map file\n");
  carmen_fprintf(fp, "#\n");
  if (getlogin() == NULL)
    carmen_fprintf(fp, "# Map author    : UNKNOWN\n");
  else
    carmen_fprintf(fp, "# Map author    : %s\n", getlogin());
  carmen_fprintf(fp, "# Creation date : %s", asctime(localtime(&t)));
  carmen_fprintf(fp, "# Map size      : %d x %d\n", size_x, size_y);
  carmen_fprintf(fp, "# Resolution    : %.1f\n", resolution);
  carmen_fprintf(fp, "# Origin        : %s\n", origin);
  carmen_fprintf(fp, "# Description   : %s", description);
  carmen_fprintf(fp, "#\n");
  carmen_fprintf(fp, "###############################################"
		     "######\n");
  return 0;
}

int carmen_map_write_id(carmen_FILE *fp)
{
  carmen_fprintf(fp, "%s%s", CARMEN_MAP_LABEL, CARMEN_MAP_VERSION);
  return 0;
}

static int carmen_map_write_creator_chunk_data(carmen_FILE *fp, char *origin, 
					       char *description)
{
  time_t t;

  if (getlogin() == NULL)
    write_fixed_string(fp, "UNKNOWN", 10);
  else
    write_fixed_string(fp, getlogin(), 10);
  t = time(NULL);
  carmen_fwrite(&t, sizeof(time_t), 1, fp);
  write_fixed_string(fp, origin, 80);
  write_fixed_string(fp, description, 80);

  return 0;
}

int carmen_map_write_creator_chunk(carmen_FILE *fp, char *origin, 
				   char *description)
{
  int size;

  carmen_fputc(CARMEN_MAP_CREATOR_CHUNK, fp);
  size = chunk_size(CARMEN_MAP_CREATOR_CHUNK);
  carmen_fwrite(&size, sizeof(int), 1, fp);
  carmen_fprintf(fp, "CREATOR   ");

  return carmen_map_write_creator_chunk_data(fp, origin, description);
}

int carmen_map_write_named_creator_chunk(carmen_FILE *fp, char *name,
					 char *origin, char *description)
{
  int size;

  carmen_fputc(CARMEN_MAP_CREATOR_CHUNK | CARMEN_MAP_NAMED_CHUNK_FLAG, fp);
  size = named_chunk_size(CARMEN_MAP_CREATOR_CHUNK, name);
  carmen_fwrite(&size, sizeof(int), 1, fp);
  carmen_fprintf(fp, "CREATOR   ");
  carmen_fprintf(fp, "%s", name);
  carmen_fputc('\0', fp);

  return carmen_map_write_creator_chunk_data(fp, origin, description);
}

static int carmen_map_write_gridmap_chunk_data(carmen_FILE *fp, double **prob,
					       int size_x, int size_y, double resolution)
{
  int x;
  double local_resolution = resolution;

  carmen_fwrite(&size_x, sizeof(int), 1, fp);
  carmen_fwrite(&size_y, sizeof(int), 1, fp);
  carmen_fwrite(&local_resolution, sizeof(double), 1, fp);
  for(x = 0; x < size_x; x++)
    carmen_fwrite(prob[x], sizeof(double) * size_y, 1, fp);

  return 0;
}

int carmen_map_write_gridmap_chunk(carmen_FILE *fp, double **prob,
				   int size_x, int size_y, double resolution)
{
  int size;

  carmen_fputc(CARMEN_MAP_GRIDMAP_CHUNK, fp);
  size = chunk_size(CARMEN_MAP_GRIDMAP_CHUNK, size_x, size_y);
  carmen_fwrite(&size, sizeof(int), 1, fp);
  carmen_fprintf(fp, "GRIDMAP   ");

  return carmen_map_write_gridmap_chunk_data(fp, prob, size_x, size_y, resolution);
}

int carmen_map_write_named_gridmap_chunk(carmen_FILE *fp, char *name, double **prob,
					 int size_x, int size_y, double resolution)
{
  int size;

  carmen_fputc(CARMEN_MAP_GRIDMAP_CHUNK | CARMEN_MAP_NAMED_CHUNK_FLAG, fp);
  size = named_chunk_size(CARMEN_MAP_GRIDMAP_CHUNK, name, size_x, size_y);
  carmen_fwrite(&size, sizeof(int), 1, fp);
  carmen_fprintf(fp, "GRIDMAP   ");
  carmen_fprintf(fp, "%s", name);
  carmen_fputc('\0', fp);

  return carmen_map_write_gridmap_chunk_data(fp, prob, size_x, size_y, resolution);
}

static int carmen_map_write_places_chunk_data(carmen_FILE *fp, carmen_place_p places, 
					      int num_places)
{
  int i;
  double double_var;

  carmen_fwrite(&num_places, sizeof(int), 1, fp);
  for(i = 0; i < num_places; i++) {
    carmen_fwrite(&(places[i].type), sizeof(int), 1, fp);
    write_fixed_string(fp, places[i].name, 20);
    if(places[i].type == CARMEN_NAMED_POSITION_TYPE) {
      double_var = places[i].x;
      carmen_fwrite(&double_var, sizeof(double), 1, fp);
      double_var = places[i].y;
      carmen_fwrite(&double_var, sizeof(double), 1, fp);
    }
    else if(places[i].type == CARMEN_NAMED_POSE_TYPE) {
      double_var = places[i].x;
      carmen_fwrite(&double_var, sizeof(double), 1, fp);
      double_var = places[i].y;
      carmen_fwrite(&double_var, sizeof(double), 1, fp);
      double_var = places[i].theta;
      carmen_fwrite(&double_var, sizeof(double), 1, fp);
    }
    else if(places[i].type == CARMEN_LOCALIZATION_INIT_TYPE) {
      double_var = places[i].x;
      carmen_fwrite(&double_var, sizeof(double), 1, fp);
      double_var = places[i].y;
      carmen_fwrite(&double_var, sizeof(double), 1, fp);
      double_var = places[i].theta;
      carmen_fwrite(&double_var, sizeof(double), 1, fp);
      double_var = places[i].x_std;
      carmen_fwrite(&double_var, sizeof(double), 1, fp);
      double_var = places[i].y_std;
      carmen_fwrite(&double_var, sizeof(double), 1, fp);
      double_var = places[i].theta_std;
      carmen_fwrite(&double_var, sizeof(double), 1, fp);
    }
  }

  return 0;
}

int carmen_map_write_places_chunk(carmen_FILE *fp, carmen_place_p places, 
				  int num_places)
{
  int size;

  carmen_fputc(CARMEN_MAP_PLACES_CHUNK, fp);
  size = chunk_size(CARMEN_MAP_PLACES_CHUNK, places, num_places);
  carmen_fwrite(&size, sizeof(int), 1, fp);
  carmen_fprintf(fp, "PLACES    ");

  return carmen_map_write_places_chunk_data(fp, places, num_places);
}

int carmen_map_write_named_places_chunk(carmen_FILE *fp, char *name,
					carmen_place_p places, int num_places)
{
  int size;

  carmen_fputc(CARMEN_MAP_PLACES_CHUNK | CARMEN_MAP_NAMED_CHUNK_FLAG, fp);
  size = named_chunk_size(CARMEN_MAP_PLACES_CHUNK, name, places, num_places);
  carmen_fwrite(&size, sizeof(int), 1, fp);
  carmen_fprintf(fp, "PLACES    ");
  carmen_fprintf(fp, "%s", name);
  carmen_fputc('\0', fp);

  return carmen_map_write_places_chunk_data(fp, places, num_places);
}

static int carmen_map_write_offlimits_chunk_data(carmen_FILE *fp, 
						 carmen_offlimits_p offlimits_list,
						 int num_items)
{
  int i;
  int num_points = 0, num_lines = 0, num_rects = 0;

  for(i = 0; i < num_items; i++)
    if(offlimits_list[i].type == CARMEN_OFFLIMITS_POINT_ID)
      num_points++;
    else if(offlimits_list[i].type == CARMEN_OFFLIMITS_LINE_ID)
      num_lines++;
    else if(offlimits_list[i].type == CARMEN_OFFLIMITS_RECT_ID)
      num_rects++;

  carmen_fwrite(&num_points, sizeof(int), 1, fp);
  for(i = 0; i < num_items; i++)
    if(offlimits_list[i].type == CARMEN_OFFLIMITS_POINT_ID) {
      carmen_fwrite(&offlimits_list[i].x1, sizeof(int), 1, fp);
      carmen_fwrite(&offlimits_list[i].y1, sizeof(int), 1, fp);
    }
  carmen_fwrite(&num_lines, sizeof(int), 1, fp);
  for(i = 0; i < num_lines; i++)
    if(offlimits_list[i].type == CARMEN_OFFLIMITS_LINE_ID) {
      carmen_fwrite(&offlimits_list[i].x1, sizeof(int), 1, fp);
      carmen_fwrite(&offlimits_list[i].y1, sizeof(int), 1, fp);
      carmen_fwrite(&offlimits_list[i].x2, sizeof(int), 1, fp);
      carmen_fwrite(&offlimits_list[i].y2, sizeof(int), 1, fp);
    }
  carmen_fwrite(&num_rects, sizeof(int), 1, fp);
  for(i = 0; i < num_items; i++)
    if(offlimits_list[i].type == CARMEN_OFFLIMITS_RECT_ID) {
      carmen_fwrite(&offlimits_list[i].x1, sizeof(int), 1, fp);
      carmen_fwrite(&offlimits_list[i].y1, sizeof(int), 1, fp);
      carmen_fwrite(&offlimits_list[i].x2, sizeof(int), 1, fp);
      carmen_fwrite(&offlimits_list[i].y2, sizeof(int), 1, fp);
    }

  return 0;
}

int carmen_map_write_offlimits_chunk(carmen_FILE *fp, 
				     carmen_offlimits_p offlimits_list,
				     int num_items)
{
  int size;

  carmen_fputc(CARMEN_MAP_OFFLIMITS_CHUNK, fp);
  size = chunk_size(CARMEN_MAP_OFFLIMITS_CHUNK, offlimits_list, num_items);
  carmen_fwrite(&size, sizeof(int), 1, fp);
  carmen_fprintf(fp, "OFFLIMITS ");

  return carmen_map_write_offlimits_chunk_data(fp, offlimits_list, num_items);
}

int carmen_map_write_named_offlimits_chunk(carmen_FILE *fp, char *name,
					   carmen_offlimits_p offlimits_list,
					   int num_items)
{
  int size;

  carmen_fputc(CARMEN_MAP_OFFLIMITS_CHUNK | CARMEN_MAP_NAMED_CHUNK_FLAG, fp);
  size = chunk_size(CARMEN_MAP_OFFLIMITS_CHUNK, name, offlimits_list, num_items);
  carmen_fwrite(&size, sizeof(int), 1, fp);
  carmen_fprintf(fp, "OFFLIMITS ");
  carmen_fprintf(fp, "%s", name);
  carmen_fputc('\0', fp);

  return carmen_map_write_offlimits_chunk_data(fp, offlimits_list, num_items);
}

int carmen_map_write_global_offset_chunk(carmen_FILE *fp, 
					 carmen_global_offset_t *global_offset)
{
  int size;

  carmen_fputc(CARMEN_MAP_GLOBAL_OFFSET_CHUNK, fp);
  size = chunk_size(CARMEN_MAP_GLOBAL_OFFSET_CHUNK);
  carmen_fwrite(&size, sizeof(int), 1, fp);
  carmen_fprintf(fp, "OFFSET    ");
  carmen_fwrite(global_offset, sizeof(carmen_global_offset_t), 1, fp);

  return 0;
}

int carmen_map_write_named_global_offset_chunk(carmen_FILE *fp, char *name,
					       carmen_global_offset_t 
					       *global_offset)
{
  int size;

  carmen_fputc(CARMEN_MAP_GLOBAL_OFFSET_CHUNK | 
	       CARMEN_MAP_NAMED_CHUNK_FLAG, fp);
  size = chunk_size(CARMEN_MAP_GLOBAL_OFFSET_CHUNK, name);
  carmen_fwrite(&size, sizeof(int), 1, fp);
  carmen_fprintf(fp, "OFFSET    ");
  carmen_fprintf(fp, "%s", name);
  carmen_fputc('\0', fp);

  carmen_fwrite(global_offset, sizeof(carmen_global_offset_t), 1, fp);

  return 0;
}

static int carmen_map_write_laserscans_chunk_data(carmen_FILE *fp,
						  carmen_laser_scan_p scan_list,
						  int num_scans)
{
  int i, j;
  double double_var;

  carmen_fwrite(&num_scans, sizeof(int), 1, fp);
  for(i = 0; i < num_scans; i++) {
    double_var = scan_list[i].x;
    carmen_fwrite(&double_var, sizeof(double), 1, fp);
    double_var = scan_list[i].y;
    carmen_fwrite(&double_var, sizeof(double), 1, fp);
    double_var = scan_list[i].theta;
    carmen_fwrite(&double_var, sizeof(double), 1, fp);
    carmen_fwrite(&(scan_list[i].num_readings), sizeof(int), 1, fp);
    for (j = 0; j < scan_list[i].num_readings; j++) {
      double_var = scan_list[i].range[j];
      carmen_fwrite(&double_var, sizeof(double), 1, fp);
    }
  }

  return 0;
}

int carmen_map_write_laserscans_chunk(carmen_FILE *fp,
				      carmen_laser_scan_p scan_list,
				      int num_scans)
{
  int size;

  carmen_fputc(CARMEN_MAP_LASERSCANS_CHUNK, fp);
  size = chunk_size(CARMEN_MAP_LASERSCANS_CHUNK, scan_list, num_scans);
  carmen_fwrite(&size, sizeof(int), 1, fp);
  carmen_fprintf(fp, "LASERSCANS");

  return carmen_map_write_laserscans_chunk_data(fp, scan_list, num_scans);
}

int carmen_map_write_named_laserscans_chunk(carmen_FILE *fp, char *name,
					    carmen_laser_scan_p scan_list,
					    int num_scans)
{
  int size;

  carmen_fputc(CARMEN_MAP_LASERSCANS_CHUNK | CARMEN_MAP_NAMED_CHUNK_FLAG, fp);
  size = named_chunk_size(CARMEN_MAP_LASERSCANS_CHUNK, name, scan_list, num_scans);
  carmen_fwrite(&size, sizeof(int), 1, fp);
  carmen_fprintf(fp, "LASERSCANS");
  carmen_fprintf(fp, "%s", name);
  carmen_fputc('\0', fp);

  return carmen_map_write_laserscans_chunk_data(fp, scan_list, num_scans);
}

static int carmen_map_write_hmap_chunk_data(carmen_FILE *fp, carmen_hmap_p hmap)
{
  int i, size, j, k, n = 0;
  double x, y, theta;

  size = 0;

  carmen_fwrite(&hmap->num_zones, sizeof(int), 1, fp);
  size += sizeof(int);
  for (i = 0; i < hmap->num_zones; i++) {
    carmen_fprintf(fp, "%s", hmap->zone_names[i]);
    carmen_fputc('\0', fp);
    size += strlen(hmap->zone_names[i]) + 1;
  }
  carmen_fwrite(&hmap->num_links, sizeof(int), 1, fp);
  size += sizeof(int);
  for (i = 0; i < hmap->num_links; i++) {
    carmen_fwrite(&hmap->links[i].type, sizeof(int), 1, fp);
    size += sizeof(int);
    carmen_fwrite(&hmap->links[i].degree, sizeof(int), 1, fp);
    size += sizeof(int);
    carmen_fwrite(hmap->links[i].keys, sizeof(int), hmap->links[i].degree, fp);
    size += sizeof(int) * hmap->links[i].degree;
    switch (hmap->links[i].type) {
    case CARMEN_HMAP_LINK_DOOR:       n = 2; break;
    case CARMEN_HMAP_LINK_ELEVATOR:   n = 1; break;
    }
    for (j = 0; j < hmap->links[i].degree; j++) {
      for (k = 0; k < n; k++) {
	x = (double) hmap->links[i].points[j*n+k].x;
	carmen_fwrite(&x, sizeof(double), 1, fp);
	size += sizeof(double);
	y = (double) hmap->links[i].points[j*n+k].y;
	carmen_fwrite(&y, sizeof(double), 1, fp);
	size += sizeof(double);
	theta = (double) hmap->links[i].points[j*n+k].theta;
	carmen_fwrite(&theta, sizeof(double), 1, fp);
	size += sizeof(double);
      }
    }
  }

  return 0;
}

int carmen_map_write_hmap_chunk(carmen_FILE *fp, carmen_hmap_p hmap)
{
  int size;

  carmen_fputc(CARMEN_MAP_HMAP_CHUNK, fp);
  size = chunk_size(CARMEN_MAP_HMAP_CHUNK, hmap);
  carmen_fwrite(&size, sizeof(int), 1, fp);
  carmen_fprintf(fp, "HMAP      ");

  return carmen_map_write_hmap_chunk_data(fp, hmap);
}

int carmen_map_write_to_ppm(carmen_map_p map, char *output_filename)
{
  int x, y;
  char c;
  FILE *fp;

  fp = fopen(output_filename, "w");
  if (fp == NULL)
    return -1;

  fprintf(fp, "P6\n%d %d\n255\n", map->config.x_size, map->config.y_size);
  for(y = map->config.y_size - 1; y >= 0; y--)
    for(x = 0; x < map->config.x_size; x++) 
      if(map->map[x][y] == -1) {
	fputc(0, fp); fputc(0, fp); fputc(255, fp);
      }
      else if(map->map[x][y] >= 2.0) {
	c = 255 - 255 * (map->map[x][y] - 2.0);
	fputc(255, fp); fputc(c, fp); fputc(c, fp);
      }
      else {
	c = 255 - 255 * map->map[x][y];
	fputc(c, fp); fputc(c, fp); fputc(c, fp);
      }
  fclose(fp);

  return 0;
}

