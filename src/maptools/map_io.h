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

#ifndef CARMEN_MAP_IO_H
#define CARMEN_MAP_IO_H

#include <carmen/carmen_stdio.h>
#include <carmen/map.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CARMEN_MAP_LABEL "CARMENMAPFILE"
#define CARMEN_MAP_VERSION "v020"

#define CARMEN_MAP_NO_CHUNK          0
#define CARMEN_MAP_GRIDMAP_CHUNK     1
#define CARMEN_MAP_OFFLIMITS_CHUNK   2
#define CARMEN_MAP_PLACES_CHUNK      4
#define CARMEN_MAP_EXPECTED_CHUNK    8
#define CARMEN_MAP_LASERSCANS_CHUNK  16
#define CARMEN_MAP_CREATOR_CHUNK     32
#define CARMEN_MAP_GLOBAL_OFFSET_CHUNK     64
#define CARMEN_MAP_HMAP_CHUNK        3

#define CARMEN_MAP_NAMED_CHUNK_FLAG (1 << 7)
#define CARMEN_MAP_CHUNK_IS_NAMED(type) ((type) & CARMEN_MAP_NAMED_CHUNK_FLAG)

int carmen_map_write_all(carmen_FILE *fp, double **prob,
			 int size_x, int size_y, double resolution,
			 char *comment_origin, char *comment_description,
			 char *creator_origin, char *creator_description,
			 carmen_place_p places, int num_places,
			 carmen_offlimits_p offlimits_list, int offlimits_num_items,
			 carmen_laser_scan_p scan_list, int num_scans);
int carmen_map_write_comment_chunk(carmen_FILE *fp, int size_x, 
				   int size_y, double resolution, char *origin,
				   char *description);
int carmen_map_write_id(carmen_FILE *fp);
int carmen_map_copy_comments(carmen_FILE *fp_in, 
			     carmen_FILE *fp_out);
int carmen_map_strip(carmen_FILE *fp_in, carmen_FILE *fp_out, int chunk);
int carmen_map_vstrip(carmen_FILE *fp_in, carmen_FILE *fp_out, 
		      int num_chunks, ...);

int carmen_map_name_chunk(char *in_file, char *out_file, int chunk_type, 
			  char *chunk_name);

int carmen_map_write_creator_chunk(carmen_FILE *fp, char *origin, 
				   char *description);
int carmen_map_write_named_creator_chunk(carmen_FILE *fp, char *name,
					 char *origin, char *description);
int carmen_map_write_gridmap_chunk(carmen_FILE *fp, double **prob,
				   int size_x, int size_y, double resolution);
int carmen_map_write_named_gridmap_chunk(carmen_FILE *fp, char *name, 
					 double **prob, int size_x, int size_y,
					 double resolution);
int carmen_map_write_places_chunk(carmen_FILE *fp, carmen_place_p places, 
				  int num_places);
int carmen_map_write_named_places_chunk(carmen_FILE *fp, char *name,
					carmen_place_p places, int num_places);
int carmen_map_write_offlimits_chunk(carmen_FILE *fp, 
				     carmen_offlimits_p offlimits_list,
				     int num_items);
int carmen_map_write_named_offlimits_chunk(carmen_FILE *fp, char *name,
					   carmen_offlimits_p offlimits_list,
					   int num_items);
int carmen_map_write_laserscans_chunk(carmen_FILE *fp,
				      carmen_laser_scan_p scan_list,
				      int num_scans);
int carmen_map_write_named_laserscans_chunk(carmen_FILE *fp, char *name,
					    carmen_laser_scan_p scan_list,
					    int num_scans);

int carmen_map_write_global_offset_chunk(carmen_FILE *fp,
					 carmen_global_offset_t *offset);
int carmen_map_write_named_global_offset_chunk(carmen_FILE *fp, char *name,
					       carmen_global_offset_t *offset);

int carmen_map_write_hmap_chunk(carmen_FILE *fp, carmen_hmap_p hmap);

int carmen_map_write_to_ppm(carmen_map_p map, char *output_filename);

int carmen_map_chunk_exists(char *filename, int specific_chunk);
int carmen_map_named_chunk_exists(char *filename, int specific_chunk, 
				  char *name);
int carmen_map_advance_to_chunk(carmen_FILE *fp, int specific_chunk);
int carmen_map_advance_to_named_chunk(carmen_FILE *fp, int specific_chunk, 
				      char *name);
int carmen_map_read_creator_chunk(char *filename, time_t *creation_time, 
				  char *username, char *origin, 
				  char *description);
int carmen_map_read_named_creator_chunk(char *filename, char *chunk_name, 
					time_t *creation_time, char *username, 
					char *origin, char *description);
int carmen_map_read_gridmap_config(char *filename, carmen_map_config_p config);
int carmen_map_read_named_gridmap_config(char *filename, char *chunk_name, 
					 carmen_map_config_p config);
int carmen_map_read_gridmap_chunk(char *filename, carmen_map_p map);
int carmen_map_read_gridmap_chunk_verbose(char *filename, carmen_map_p map, int verbose);
int carmen_map_read_named_gridmap_chunk(char *filename, char *chunk_name, 
					carmen_map_p map);
int carmen_map_read_offlimits_chunk(char *filename, 
				    carmen_offlimits_p *offlimits_list,
				    int *list_length);
int carmen_map_read_named_offlimits_chunk(char *filename, char *chunk_name,
					  carmen_offlimits_p *offlimits_list,
					  int *list_length);
int carmen_map_read_offlimits_chunk_into_map(char *filename, carmen_map_p map);
int carmen_map_read_places_chunk(char *filename, 
				 carmen_map_placelist_p places);
int carmen_map_read_named_places_chunk(char *filename, char *chunk_name,
				       carmen_map_placelist_p places);
int carmen_map_read_laserscans_chunk(char *filename,
				     carmen_laser_scan_p *scan_list,
				     int *num_scans);
int carmen_map_read_named_laserscans_chunk(char *filename, char *chunk_name,
					   carmen_laser_scan_p *scan_list,
					   int *num_scans);

int carmen_map_read_global_offset_chunk(char *filename, 
					carmen_global_offset_t *global_offset);
int carmen_map_read_named_global_offset_chunk(char *filename, char *chunk_name,
					      carmen_global_offset_t 
					      *global_offset);

int carmen_map_read_hmap_chunk(char *filename, carmen_hmap_p hmap);

int carmen_map_file(char *filename);
int carmen_map_initialize_ipc(void);
void carmen_map_set_filename(char *new_filename);
void carmen_map_publish_update(void);

#ifdef __cplusplus
}
#endif

#endif
