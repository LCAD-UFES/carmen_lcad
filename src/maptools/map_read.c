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
#include <assert.h>

static int file_warnings = 1;

/*
 * if size < 0 then ignore buf and advance fp to one byte after the next '\0'
 */
static int read_string(char *buf, int size, carmen_FILE *fp)
{
  int i;
  int result;
  char c;

  for (i = 0; i < size || size < 0; i++) {
    result = carmen_fgetc(fp);
    c = (char) result;
    if ( result == EOF)
      return -1;
    if (c == '\0') {
      if (size >= 0)
	buf[i] = '\0';
      return i;
    }
    if (size >= 0)
      buf[i] = c;
  }

  buf[size-1] = '\0';
  return -1;
}

int carmen_map_read_comment_chunk(carmen_FILE *fp)
{
  char comment_str[100], c;
  char *err, id[1024];
  int result;

  do {
    result = carmen_fgetc(fp);
    c = (char) result;
    if(result == EOF)
      return -1;
    else if(c == '#')
      {
	err = carmen_fgets(comment_str, 100, fp);
	if(err == NULL)
	  return -1;
      }
    else
      carmen_fseek(fp, -1, SEEK_CUR);
  } while(c == '#');

  assert (strlen(CARMEN_MAP_LABEL)+strlen(CARMEN_MAP_VERSION) < 1024);

  if(carmen_fread(&id, strlen(CARMEN_MAP_LABEL) +
		  strlen(CARMEN_MAP_VERSION), 1, fp) == 0)
    return -1;
  if(strncmp(id, CARMEN_MAP_LABEL, strlen(CARMEN_MAP_LABEL)) != 0)
    return 1;
  if(strncmp(id + strlen(CARMEN_MAP_LABEL), CARMEN_MAP_VERSION,
	     strlen(CARMEN_MAP_VERSION)) != 0)
    return 1;

  return 0;
}

int carmen_map_file(char *filename)
{
  carmen_FILE *fp;
  int comment_return;

  if(!carmen_file_exists(filename))
    return 0;
  fp = carmen_fopen(filename, "r");
  if(fp == NULL)
    return 0;

  comment_return = carmen_map_read_comment_chunk(fp);
  carmen_fclose(fp);

  if(!comment_return)
    return 1;
  return 0;
}

int carmen_map_copy_comments(carmen_FILE *fp_in, carmen_FILE *fp_out)
{
  char *err, comment_str[100], id[1024];
  int c;

  do {
    c = carmen_fgetc(fp_in);
    if(c == EOF)
      return -1;
    else if(c == '#') {
      err = carmen_fgets(comment_str, 100, fp_in);
      if(err == NULL)
	return -1;
      else {
	carmen_fputc(c, fp_out);
	carmen_fprintf(fp_out, "%s", comment_str);
      }
    }
    else
      carmen_fseek(fp_in, -1, SEEK_CUR);
  } while(c == '#');
  if(carmen_fread(id, strlen(CARMEN_MAP_LABEL) + strlen(CARMEN_MAP_VERSION),
		  1, fp_in) < 1)
    return -1;
  carmen_fwrite(id, strlen(CARMEN_MAP_LABEL) + strlen(CARMEN_MAP_VERSION),
		1, fp_out);
  return 0;
}

int carmen_map_vstrip(carmen_FILE *fp_in, carmen_FILE *fp_out,
		      int num_chunks, ...)
{
  static int strip[128];
  int chunk_type, chunk_size, i, done = 0;
  char *buffer;
  va_list chunks;

  memset(strip, 0, 128 * sizeof(int));

  va_start(chunks, num_chunks);
  for (i = 0; i < num_chunks; i++){
    chunk_type = va_arg(chunks, int);
    if (chunk_type < 0 || chunk_type >= 128) {
      carmen_warn("Error: Invalid chunk type: %d\n", chunk_type);
      va_end(chunks);
      return -1;
    }
    strip[chunk_type] = 1;
  }

  va_end(chunks);

  if(carmen_map_copy_comments(fp_in, fp_out) < 0)
    return -1;

  do {
    chunk_type = carmen_fgetc(fp_in);
    if(chunk_type == EOF)
      done = 1;

    if(!done)
      if(carmen_fread(&chunk_size, sizeof(int), 1, fp_in) < 1)
        done = 1;

    if(!done && (!strip[chunk_type])) {
      carmen_fputc(chunk_type, fp_out);
      carmen_fwrite(&chunk_size, sizeof(int), 1, fp_out);
      buffer = (char *)calloc(chunk_size, sizeof(char));
      carmen_test_alloc(buffer);
      if(carmen_fread(buffer, chunk_size, 1, fp_in) < 1) {
	fprintf(stderr, "Error: error writing new file: %s\n",
		strerror(errno));
	return -1;
      }
      carmen_fwrite(buffer, chunk_size, 1, fp_out);
      free(buffer);
    }
    else if(!done)
      if(carmen_fseek(fp_in, chunk_size, SEEK_CUR) < 0) {
	fprintf(stderr, "Error: error copying map.\n");
	return -1;
      }

    if(!done && carmen_feof(fp_in))
      done = 1;
  } while(!done);

  return 0;
}

int carmen_map_strip(carmen_FILE *fp_in, carmen_FILE *fp_out, int chunk_id)
{
  return carmen_map_vstrip(fp_in, fp_out, 1, chunk_id);
}

int carmen_map_name_chunk(char *in_file, char *out_file, int chunk_type,
			  char *chunk_name)
{
  unsigned char *buf;
  int size, type, retval;
  carmen_FILE *in_fp, *out_fp;

  in_fp = carmen_fopen(in_file, "r");

  if(in_fp == NULL) {
    carmen_warn("Error: Can't open file %s for reading", in_file);
    return -1;
  }

  if(carmen_map_advance_to_chunk(in_fp, chunk_type) < 0) {
    carmen_warn("Error: Can't advance to chunk %d", chunk_type);
    carmen_fclose(in_fp);
    return -1;
  }

  type = carmen_fgetc(in_fp);
  if(type == EOF) {
    carmen_warn("Error: Unexpected EOF");
    carmen_fclose(in_fp);
    return -1;
  }

  if(CARMEN_MAP_CHUNK_IS_NAMED(type)) {
    carmen_warn("Error: Chunk is already named");
    carmen_fclose(in_fp);
    return -1;
  }

  if(carmen_fread(&size, sizeof(int), 1, in_fp) < 1) {
    carmen_fclose(in_fp);
    return -1;
  }

  buf = (unsigned char *)calloc(size, sizeof(char));
  carmen_test_alloc(buf);

  carmen_fread(buf, 1, size, in_fp);

  carmen_fclose(in_fp);

  retval = carmen_map_named_chunk_exists(out_file, chunk_type, chunk_name);
  if(retval < 0)
    return -1;
  if(retval > 0) {
    carmen_warn("Error: Map chunk of type %d named %s already exists in file %s", chunk_type, chunk_name, out_file);
    return -1;
  }

  out_fp = carmen_fopen(out_file, "a");
  if(out_fp == NULL) {
    carmen_warn("Error: Can't open file %s for appending", out_file);
    return -1;
  }

  carmen_fputc(chunk_type | CARMEN_MAP_NAMED_CHUNK_FLAG, out_fp);
  size += strlen(chunk_name) + 1;
  carmen_fwrite(&size, sizeof(int), 1, out_fp);
  carmen_fwrite(buf, sizeof(char), 10, out_fp);
  carmen_fprintf(out_fp, "%s", chunk_name);
  carmen_fputc('\0', out_fp);
  size -= 10 + strlen(chunk_name) + 1;
  carmen_fwrite(buf+10, sizeof(char), size, out_fp);

  carmen_fclose(out_fp);
  return 0;
}

void strip_trailing_spaces(char *str, int len)
{
  int i;

  i = len - 1;
  while(i >= 0 && str[i] == ' ')
    i--;
  i++;
  if(i == len)
    i--;
  str[i] = '\0';
}

int carmen_map_advance_to_chunk(carmen_FILE *fp, int specific_chunk)
{
  int chunk_type, chunk_size, done = 0;

  carmen_fseek(fp, 0, SEEK_SET);
  if(carmen_map_read_comment_chunk(fp) < 0) {
    fprintf(stderr, "Error: Could not read comment chunk.\n");
    return -1;
  }

  specific_chunk &= ~(unsigned char)CARMEN_MAP_NAMED_CHUNK_FLAG;

  do {
    chunk_type = carmen_fgetc(fp);
    if(chunk_type == EOF)
      done = 1;
    chunk_type &= ~(unsigned char)CARMEN_MAP_NAMED_CHUNK_FLAG;
    if(chunk_type == specific_chunk) {
      carmen_fseek(fp, -1, SEEK_CUR);
      return 0;
    }
    if(!done)
      if(carmen_fread(&chunk_size, sizeof(int), 1, fp) < 1)
	done = 1;
    if(!done)
      if(carmen_fseek(fp, chunk_size, SEEK_CUR) < 0)
	done = 1;
    if(!done && carmen_feof(fp))
      done = 1;
  } while(!done);
  return -1;
}

int carmen_map_advance_to_named_chunk(carmen_FILE *fp, int specific_chunk,
				      char *name)
{
  int chunk_type, named, chunk_size, len, done = 0;
  char buf[128];

  carmen_fseek(fp, 0, SEEK_SET);
  if(carmen_map_read_comment_chunk(fp) < 0) {
    fprintf(stderr, "Error: Could not read comment chunk.\n");
    return -1;
  }

  specific_chunk &= ~CARMEN_MAP_NAMED_CHUNK_FLAG;

  do {
    chunk_type = carmen_fgetc(fp);
    if(chunk_type == EOF)
      done = 1;
    named = CARMEN_MAP_CHUNK_IS_NAMED(chunk_type);
    chunk_type &= ~CARMEN_MAP_NAMED_CHUNK_FLAG;
    if(chunk_type == specific_chunk && named) {
      carmen_fseek(fp, 14, SEEK_CUR);
      len = read_string(buf, 128, fp);
      if (len < 0)
	return -1;
      if (!strcmp(buf, name)) {
	carmen_fseek(fp, -15-len-1 , SEEK_CUR);
	return 0;
      }
      else
	carmen_fseek(fp, -14-len-1 , SEEK_CUR);
    }
    if(!done)
      if(carmen_fread(&chunk_size, sizeof(int), 1, fp) < 1)
	done = 1;
    if(!done)
      if(carmen_fseek(fp, chunk_size, SEEK_CUR) < 0)
	done = 1;
    if(!done && carmen_feof(fp))
      done = 1;
  } while(!done);
  return -1;
}

int carmen_map_chunk_exists(char *filename, int specific_chunk)
{
  carmen_FILE *fp;
  int chunk_size;

  fp = carmen_fopen(filename, "r");
  if(fp == NULL) {
    fprintf(stderr, "Error: could not open file %s for reading.\n",
	    filename);
    return -1;
  }
  if(carmen_map_advance_to_chunk(fp, specific_chunk) < 0) {
    carmen_fclose(fp);
    return 0;
  }
  else {
    carmen_fgetc(fp);
    carmen_fread(&chunk_size, sizeof(int), 1, fp);
    carmen_fclose(fp);
    return chunk_size;
  }
}

int carmen_map_named_chunk_exists(char *filename, int specific_chunk,
				  char *name)
{
  carmen_FILE *fp;
  int chunk_size;

  fp = carmen_fopen(filename, "r");
  if(fp == NULL) {
    fprintf(stderr, "Error: could not open file %s for reading.\n",
	    filename);
    return -1;
  }
  if(carmen_map_advance_to_named_chunk(fp, specific_chunk, name) < 0) {
    carmen_fclose(fp);
    return 0;
  }
  else {
    carmen_fgetc(fp);
    carmen_fread(&chunk_size, sizeof(int), 1, fp);
    carmen_fclose(fp);
    return chunk_size;
  }
}

static int carmen_map_read_creator_chunk_data(carmen_FILE *fp,
					      time_t *creation_time,
					      char *username, char *origin,
					      char *description)
{
  carmen_fread(username, 10, 1, fp);
  strip_trailing_spaces(username, 10);
  carmen_fread(creation_time, sizeof(time_t), 1, fp);
  carmen_fread(origin, 80, 1, fp);
  strip_trailing_spaces(origin, 80);
  carmen_fread(description, 80, 1, fp);
  strip_trailing_spaces(description, 80);
  carmen_fclose(fp);
  return 0;
}

int carmen_map_read_creator_chunk(char *filename, time_t *creation_time,
				  char *username, char *origin,
				  char *description)
{
  carmen_FILE *fp;
  int chunk_type, chunk_size;
  char chunk_description[12];

  fp = carmen_fopen(filename, "r");
  if(fp == NULL) {
    fprintf(stderr, "Error: could not open file %s for reading.\n",
	    filename);
    return -1;
  }
  if(carmen_map_advance_to_chunk(fp, CARMEN_MAP_CREATOR_CHUNK) < 0)
    {
      if (carmen_map_advance_to_chunk(fp, 0) < 0)
	{
	  carmen_warn("You have an old-style map. The creator chunk id "
		      "is wrong.\n\nFIX IT!\n\n");
	}
      else {
	carmen_warn("Error: Could not find a creator chunk.\n");
	carmen_fclose(fp);
	return -1;
      }
    }
  chunk_type = carmen_fgetc(fp);
  carmen_fread(&chunk_size, sizeof(int), 1, fp);
  carmen_fread(chunk_description, 10, 1, fp);

  if (CARMEN_MAP_CHUNK_IS_NAMED(chunk_type)) {
    if (read_string(NULL, -1, fp) < 0) {
      carmen_warn("Error: Unexpected EOF.\n");
      carmen_fclose(fp);
      return -1;
    }
  }
  return carmen_map_read_creator_chunk_data(fp, creation_time, username,
					    origin, description);
}

int carmen_map_read_named_creator_chunk(char *filename, char *chunk_name,
					time_t *creation_time,
					char *username, char *origin,
					char *description)
{
  carmen_FILE *fp;
  int chunk_type, chunk_size;
  char chunk_description[12];

  fp = carmen_fopen(filename, "r");
  if(fp == NULL) {
    fprintf(stderr, "Error: could not open file %s for reading.\n",
	    filename);
    return -1;
  }
  if(carmen_map_advance_to_named_chunk(fp, CARMEN_MAP_CREATOR_CHUNK,
				       chunk_name) < 0) {
    if(carmen_map_advance_to_chunk(fp, 0) < 0) {
      carmen_warn("You have an old-style map. The creator chunk id "
		  "is wrong.\n\nFIX IT!\n\n");
    }
    else {
      carmen_warn("Error: Could not find a creator chunk named \"%s\"\n",
		  chunk_name);
      carmen_fclose(fp);
      return -1;
    }
  }
  chunk_type = carmen_fgetc(fp);
  carmen_fread(&chunk_size, sizeof(int), 1, fp);
  carmen_fread(chunk_description, 10, 1, fp);

  if(CARMEN_MAP_CHUNK_IS_NAMED(chunk_type)) {
    if(read_string(NULL, -1, fp) < 0) {
      carmen_warn("Error: Unexpected EOF.\n");
      carmen_fclose(fp);
      return -1;
    }
  }

  return carmen_map_read_creator_chunk_data(fp, creation_time, username,
					    origin, description);
}

static int carmen_map_read_gridmap_config_data(carmen_FILE *fp,
					       carmen_map_config_p config)
{
  int size_x, size_y;
  double resolution;

  carmen_fread(&size_x, sizeof(int), 1, fp);
  carmen_fread(&size_y, sizeof(int), 1, fp);
  carmen_fread(&resolution, sizeof(double), 1, fp);

  config->x_size = size_x;
  config->y_size = size_y;
  config->resolution = resolution;

  carmen_fclose(fp);
  return 0;
}

int carmen_map_read_gridmap_config(char *filename, carmen_map_config_p config)
{
  carmen_FILE *fp;
  int chunk_type, chunk_size;
  char chunk_description[12];

  if(config == NULL) {
    fprintf(stderr, "Error: config argument is NULL in %s.\n",
	    __FUNCTION__);
    return -1;
  }

  fp = carmen_fopen(filename, "r");
  if(fp == NULL) {
    fprintf(stderr, "Error: could not open file %s for reading.\n",
	    filename);
    return -1;
  }

  if(carmen_map_advance_to_chunk(fp, CARMEN_MAP_GRIDMAP_CHUNK) < 0) {
    fprintf(stderr, "Error: Could not find a gridmap chunk.\n");
    fprintf(stderr, "       This file is probably not a map file.\n");
    carmen_fclose(fp);
    return -1;
  }
  chunk_type = carmen_fgetc(fp);
  carmen_fread(&chunk_size, sizeof(int), 1, fp);
  carmen_fread(chunk_description, 10, 1, fp);

  if(CARMEN_MAP_CHUNK_IS_NAMED(chunk_type)) {
    if(read_string(NULL, -1, fp) < 0) {
      carmen_warn("Error: Unexpected EOF.\n");
      carmen_fclose(fp);
      return -1;
    }
  }

  config->map_name = (char *)calloc(strlen(filename) + 1, sizeof(char));
  carmen_test_alloc(config->map_name);
  strcpy(config->map_name, filename);

  return carmen_map_read_gridmap_config_data(fp, config);
}

int carmen_map_read_named_gridmap_config(char *filename, char *chunk_name,
					 carmen_map_config_p config)
{
  carmen_FILE *fp;
  int chunk_type, chunk_size;
  char chunk_description[12];

  if(config == NULL) {
    fprintf(stderr, "Error: config argument is NULL in %s.\n",
	    __FUNCTION__);
    return -1;
  }

  fp = carmen_fopen(filename, "r");
  if(fp == NULL) {
    fprintf(stderr, "Error: could not open file %s for reading.\n",
	    filename);
    return -1;
  }

  if(carmen_map_advance_to_named_chunk(fp, CARMEN_MAP_GRIDMAP_CHUNK,
				       chunk_name) < 0) {
    fprintf(stderr, "Error: Could not find a gridmap chunk named \"%s\"\n",
	    chunk_name);
    fprintf(stderr, "       This file is probably not a map file.\n");
    carmen_fclose(fp);
    return -1;
  }
  chunk_type = carmen_fgetc(fp);
  carmen_fread(&chunk_size, sizeof(int), 1, fp);
  carmen_fread(chunk_description, 10, 1, fp);

  if(CARMEN_MAP_CHUNK_IS_NAMED(chunk_type)) {
    if(read_string(NULL, -1, fp) < 0) {
      carmen_warn("Error: Unexpected EOF.\n");
      carmen_fclose(fp);
      return -1;
    }
  }

  config->map_name = (char *)calloc(strlen(chunk_name) + 1, sizeof(char));
  carmen_test_alloc(config->map_name);
  strcpy(config->map_name, chunk_name);

  return carmen_map_read_gridmap_config_data(fp, config);
}

static int carmen_map_read_gridmap_chunk_data(carmen_FILE *fp,
					      carmen_map_p map)
{
  int size_x, size_y;
  double resolution;
  int n;

  carmen_fread(&size_x, sizeof(int), 1, fp);
  carmen_fread(&size_y, sizeof(int), 1, fp);
  carmen_fread(&resolution, sizeof(double), 1, fp);

  map->config.x_size = size_x;
  map->config.y_size = size_y;
  map->config.resolution = resolution;

  map->complete_map = (double *)calloc(map->config.x_size * map->config.y_size, sizeof(double));
  carmen_test_alloc(map->complete_map);
  map->map = (double **)calloc(map->config.x_size, sizeof(double *));
  carmen_test_alloc(map->map);

  for(n = 0; n < map->config.x_size; n++)
    map->map[n] = map->complete_map + n * map->config.y_size;

  carmen_fread(map->complete_map, sizeof(double) * size_x * size_y, 1, fp);

  carmen_fclose(fp);
  return 0;
}

int
carmen_map_read_gridmap_chunk(char *filename, carmen_map_p map)
{
  carmen_FILE *fp;
  int chunk_type, chunk_size;
  char chunk_description[12];

  if(filename == NULL)
    return -1;

  fp = carmen_fopen(filename, "r");
  if (fp == NULL) {
//	if (file_warnings)
//      fprintf(stderr, "Error: could not open file %s for reading.\n",
//	    filename);
    return -1;
  }
  if(carmen_map_advance_to_chunk(fp, CARMEN_MAP_GRIDMAP_CHUNK) < 0) {
    fprintf(stderr, "Error: Could not find a gridmap chunk.\n");
    fprintf(stderr, "       This file is probably not a map file.\n");
    carmen_fclose(fp);
    return -1;
  }
  chunk_type = carmen_fgetc(fp);
  carmen_fread(&chunk_size, sizeof(int), 1, fp);
  carmen_fread(chunk_description, 10, 1, fp);

  chunk_description[10] = '\0';

  if(CARMEN_MAP_CHUNK_IS_NAMED(chunk_type)) {
    if(read_string(NULL, -1, fp) < 0) {
      carmen_warn("Error: Unexpected EOF.\n");
      carmen_fclose(fp);
      return -1;
    }
  }

  map->config.map_name = (char *)calloc(strlen(filename)+1, sizeof(char));
  carmen_test_alloc(map->config.map_name);
  strcpy(map->config.map_name, filename);

  return carmen_map_read_gridmap_chunk_data(fp, map);
}


int
carmen_map_read_gridmap_chunk_verbose(char *filename, carmen_map_p map, int verbose)
{
	int result;
	int previous_file_warnings_option = file_warnings;

	file_warnings = verbose;
	result = carmen_map_read_gridmap_chunk(filename, map);
	file_warnings = previous_file_warnings_option;

	return result;
}


void
carmen_map_free_gridmap(carmen_map_p map)
{
	if (map->config.map_name != NULL)
	{
		free(map->config.map_name);
		map->config.map_name = NULL;
	}
	if (map->complete_map != NULL)
	{
		free(map->complete_map);
		map->complete_map = NULL;
	}
	if (map->map != NULL)
	{
		free(map->map);
		map->map = NULL;
	}
}


int
carmen_map_read_named_gridmap_chunk(char *filename, char *chunk_name,
					carmen_map_p map)
{
  carmen_FILE *fp;
  int chunk_type, chunk_size;
  char chunk_description[12];

  if(filename == NULL)
    return -1;

  fp = carmen_fopen(filename, "r");
  if(fp == NULL) {
    fprintf(stderr, "Error: could not open file %s for reading.\n",
	    filename);
    return -1;
  }
  if(carmen_map_advance_to_named_chunk(fp, CARMEN_MAP_GRIDMAP_CHUNK,
				       chunk_name) < 0) {
    fprintf(stderr, "Error: Could not find a gridmap chunk named \"%s\"\n",
	    chunk_name);
    carmen_fclose(fp);
    return -1;
  }
  chunk_type = carmen_fgetc(fp);
  carmen_fread(&chunk_size, sizeof(int), 1, fp);
  carmen_fread(chunk_description, 10, 1, fp);

  if(CARMEN_MAP_CHUNK_IS_NAMED(chunk_type)) {
    if(read_string(NULL, -1, fp) < 0) {
      carmen_warn("Error: Unexpected EOF.\n");
      carmen_fclose(fp);
      return -1;
    }
  }

  map->config.map_name = (char *)calloc(strlen(chunk_name)+1, sizeof(char));
  carmen_test_alloc(map->config.map_name);
  strcpy(map->config.map_name, chunk_name);

  return carmen_map_read_gridmap_chunk_data(fp, map);
}

static int carmen_map_read_offlimits_chunk_data(carmen_FILE *fp,
						carmen_offlimits_p
						*offlimits_list,
						int *list_length)
{
  int num_points, num_lines, num_rects, i;
  carmen_offlimits_t cur_seg;

  if(offlimits_list == NULL || list_length == NULL) {
    carmen_fclose(fp);
    return -1;
  }

  *list_length = 0;

  carmen_fread(&num_points, sizeof(int), 1, fp);

  if (num_points > 0) {
    *list_length = num_points;
    *offlimits_list = (carmen_offlimits_p)calloc
      (num_points, sizeof(carmen_offlimits_t));
    carmen_test_alloc(*offlimits_list);
    for(i = 0; i < num_points; i++) {
      cur_seg.type = CARMEN_OFFLIMITS_POINT_ID;
      carmen_fread(&(cur_seg.x1), sizeof(int), 1, fp);
      carmen_fread(&(cur_seg.y1), sizeof(int), 1, fp);
      cur_seg.x2 = 0;
      cur_seg.y2 = 0;
      (*offlimits_list)[i] = cur_seg;
    }
  }
  carmen_fread(&num_lines, sizeof(int), 1, fp);

  if (num_lines > 0)
    {
      if (*list_length == 0)
	{
	  *offlimits_list = (carmen_offlimits_p)calloc
	    (num_lines, sizeof(carmen_offlimits_t));
	  carmen_test_alloc(*offlimits_list);
	  *list_length = num_lines;
	}
      else {
	*list_length += num_lines;
	*offlimits_list = (carmen_offlimits_p)realloc
	  (*offlimits_list, (*list_length)*
	   sizeof(carmen_offlimits_t));
	carmen_test_alloc(*offlimits_list);
      }

      for(i = 0; i < num_lines; i++) {
	cur_seg.type = CARMEN_OFFLIMITS_LINE_ID;
	carmen_fread(&(cur_seg.x1), sizeof(int), 1, fp);
	carmen_fread(&(cur_seg.y1), sizeof(int), 1, fp);
	carmen_fread(&(cur_seg.x2), sizeof(int), 1, fp);
	carmen_fread(&(cur_seg.y2), sizeof(int), 1, fp);
	(*offlimits_list)[i+num_points] = cur_seg;
      }
    }
  carmen_fread(&num_rects, sizeof(int), 1, fp);

  if (num_rects > 0)
    {
      if (*list_length == 0)
	{
	  *list_length = num_rects;
	  *offlimits_list = (carmen_offlimits_p)calloc
	    (num_rects, sizeof(carmen_offlimits_t));
	  carmen_test_alloc(*offlimits_list);
	}
      else
	{
	  *list_length += num_rects;
	  *offlimits_list = (carmen_offlimits_p)realloc
	    (*offlimits_list, (*list_length)*
	     sizeof(carmen_offlimits_t));
	  carmen_test_alloc(*offlimits_list);
	}
      for(i = 0; i < num_rects; i++) {
	cur_seg.type = CARMEN_OFFLIMITS_RECT_ID;
	carmen_fread(&(cur_seg.x1), sizeof(int), 1, fp);
	carmen_fread(&(cur_seg.y1), sizeof(int), 1, fp);
	carmen_fread(&(cur_seg.x2), sizeof(int), 1, fp);
	carmen_fread(&(cur_seg.y2), sizeof(int), 1, fp);
	(*offlimits_list)[i+num_points+num_lines] = cur_seg;
      }
    }

  carmen_fclose(fp);
  return 0;
}

int carmen_map_read_offlimits_chunk(char *filename,
				    carmen_offlimits_p *offlimits_list,
				    int *list_length)
{
  carmen_FILE *fp;
  char chunk_type, chunk_description[12];
  int chunk_size;

  *offlimits_list = NULL;
  *list_length = 0;

  fp = carmen_fopen(filename, "r");
  if(fp == NULL) {
    fprintf(stderr, "Error: could not open file %s for reading.\n",
	    filename);
    return -1;
  }
  if(carmen_map_advance_to_chunk(fp, CARMEN_MAP_OFFLIMITS_CHUNK) < 0) {
    carmen_fclose(fp);
    return -1;
  }
  chunk_type = carmen_fgetc(fp);
  carmen_fread(&chunk_size, sizeof(int), 1, fp);
  carmen_fread(chunk_description, 10, 1, fp);

  if (CARMEN_MAP_CHUNK_IS_NAMED(chunk_type)) {
    if (read_string(NULL, -1, fp) < 0) {
      carmen_warn("Error: Unexpected EOF.\n");
      carmen_fclose(fp);
      return -1;
    }
  }

  return carmen_map_read_offlimits_chunk_data(fp, offlimits_list, list_length);
}

int carmen_map_read_named_offlimits_chunk(char *filename, char *chunk_name,
					  carmen_offlimits_p *offlimits_list,
					  int *list_length)
{
  carmen_FILE *fp;
  char chunk_type, chunk_description[12];
  int chunk_size;

  *offlimits_list = NULL;
  *list_length = 0;

  fp = carmen_fopen(filename, "r");
  if(fp == NULL) {
    fprintf(stderr, "Error: could not open file %s for reading.\n",
	    filename);
    return -1;
  }
  if(carmen_map_advance_to_named_chunk(fp, CARMEN_MAP_OFFLIMITS_CHUNK, chunk_name) < 0) {
    fprintf(stderr, "Error: Could not find an offlimits chunk named \"%s\"\n", chunk_name);
    carmen_fclose(fp);
    return -1;
  }
  chunk_type = carmen_fgetc(fp);
  carmen_fread(&chunk_size, sizeof(int), 1, fp);
  carmen_fread(chunk_description, 10, 1, fp);

  if (CARMEN_MAP_CHUNK_IS_NAMED(chunk_type)) {
    if (read_string(NULL, -1, fp) < 0) {
      carmen_warn("Error: Unexpected EOF.\n");
      carmen_fclose(fp);
      return -1;
    }
  }

  return carmen_map_read_offlimits_chunk_data(fp, offlimits_list, list_length);
}

int carmen_map_read_global_offset_chunk(char *filename,
					carmen_global_offset_t *global_offset)
{
  carmen_FILE *fp;
  char chunk_type, chunk_description[12];
  int chunk_size;

  fp = carmen_fopen(filename, "r");
  if(fp == NULL) {
    fprintf(stderr, "Error: could not open file %s for reading.\n",
	    filename);
    return -1;
  }
  if(carmen_map_advance_to_chunk(fp, CARMEN_MAP_GLOBAL_OFFSET_CHUNK) < 0) {
    carmen_fclose(fp);
    return -1;
  }
  chunk_type = carmen_fgetc(fp);
  carmen_fread(&chunk_size, sizeof(int), 1, fp);
  carmen_fread(chunk_description, 10, 1, fp);

  if (CARMEN_MAP_CHUNK_IS_NAMED(chunk_type)) {
    if (read_string(NULL, -1, fp) < 0) {
      carmen_warn("Error: Unexpected EOF.\n");
      carmen_fclose(fp);
      return -1;
    }
  }

  carmen_fread(global_offset, sizeof(carmen_global_offset_t), 1, fp);
  carmen_fclose(fp);

  return 0;
}

int carmen_map_read_named_global_offset_chunk(char *filename, char *chunk_name,
					      carmen_global_offset_t
					      *global_offset)
{
  carmen_FILE *fp;
  char chunk_type, chunk_description[12];
  int chunk_size;

  fp = carmen_fopen(filename, "r");
  if(fp == NULL) {
    fprintf(stderr, "Error: could not open file %s for reading.\n",
	    filename);
    return -1;
  }
  if(carmen_map_advance_to_named_chunk
     (fp, CARMEN_MAP_GLOBAL_OFFSET_CHUNK, chunk_name) < 0) {
    fprintf(stderr, "Error: Could not find a global offset chunk "
	    "named \"%s\"\n", chunk_name);
    carmen_fclose(fp);
    return -1;
  }
  chunk_type = carmen_fgetc(fp);
  carmen_fread(&chunk_size, sizeof(int), 1, fp);
  carmen_fread(chunk_description, 10, 1, fp);

  if (CARMEN_MAP_CHUNK_IS_NAMED(chunk_type)) {
    if (read_string(NULL, -1, fp) < 0) {
      carmen_warn("Error: Unexpected EOF.\n");
      carmen_fclose(fp);
      return -1;
    }
  }

  carmen_fread(global_offset, sizeof(carmen_global_offset_t), 1, fp);
  carmen_fclose(fp);

  return 0;
}

int carmen_map_read_offlimits_chunk_into_map(char *filename, carmen_map_p map)
{
  carmen_offlimits_p offlimits_list;
  int list_length;

  if (carmen_map_read_offlimits_chunk
      (filename, &offlimits_list, &list_length) < 0)
    return -1;

  if (carmen_map_apply_offlimits_chunk_to_map
      (offlimits_list, list_length, map) < 0)
    return -1;

  free(offlimits_list);
  return 0;
}

static int carmen_map_read_places_chunk_data(carmen_FILE *fp, carmen_map_placelist_p places)
{
  int i;
  int place_type;
  double double_var;

  carmen_fread(&(places->num_places), sizeof(int), 1, fp);
  places->places = (carmen_place_p)calloc(places->num_places,
					  sizeof(carmen_place_t));
  carmen_test_alloc(places->places);
  for(i = 0; i < places->num_places; i++) {
    carmen_fread(&place_type, sizeof(int), 1, fp);
    carmen_fread(places->places[i].name, 20, 1, fp);
    strip_trailing_spaces(places->places[i].name, 20);
    if(place_type == CARMEN_NAMED_POSITION_TYPE) {
      carmen_fread(&double_var, sizeof(double), 1, fp);
      places->places[i].x = double_var;
      carmen_fread(&double_var, sizeof(double), 1, fp);
      places->places[i].y = double_var;
    }
    else if(place_type == CARMEN_NAMED_POSE_TYPE) {
      carmen_fread(&double_var, sizeof(double), 1, fp);
      places->places[i].x = double_var;
      carmen_fread(&double_var, sizeof(double), 1, fp);
      places->places[i].y = double_var;
      carmen_fread(&double_var, sizeof(double), 1, fp);
      places->places[i].theta = double_var;
    }
    else if(place_type == CARMEN_LOCALIZATION_INIT_TYPE) {
      carmen_fread(&double_var, sizeof(double), 1, fp);
      places->places[i].x = double_var;
      carmen_fread(&double_var, sizeof(double), 1, fp);
      places->places[i].y = double_var;
      carmen_fread(&double_var, sizeof(double), 1, fp);
      places->places[i].theta = double_var;

      carmen_fread(&double_var, sizeof(double), 1, fp);
      places->places[i].x_std = double_var;
      carmen_fread(&double_var, sizeof(double), 1, fp);
      places->places[i].y_std = double_var;
      carmen_fread(&double_var, sizeof(double), 1, fp);
      places->places[i].theta_std = double_var;
    }
    places->places[i].type = place_type;
  }

  carmen_fclose(fp);
  return 0;
}

int carmen_map_read_places_chunk(char *filename, carmen_map_placelist_p places)
{
  carmen_FILE *fp;
  int chunk_type, chunk_size;
  char chunk_description[12];

  fp = carmen_fopen(filename, "r");
  if(fp == NULL) {
    fprintf(stderr, "Error: could not open file %s for reading.\n",
	    filename);
    return -1;
  }
  if(carmen_map_advance_to_chunk(fp, CARMEN_MAP_PLACES_CHUNK) < 0) {
    carmen_fclose(fp);
    return -1;
  }
  chunk_type = carmen_fgetc(fp);
  carmen_fread(&chunk_size, sizeof(int), 1, fp);
  carmen_fread(chunk_description, 10, 1, fp);

  if (CARMEN_MAP_CHUNK_IS_NAMED(chunk_type)) {
    if (read_string(NULL, -1, fp) < 0) {
      carmen_warn("Error: Unexpected EOF.\n");
      carmen_fclose(fp);
      return -1;
    }
  }

  return carmen_map_read_places_chunk_data(fp, places);
}

int carmen_map_read_named_places_chunk(char *filename, char *chunk_name,
				       carmen_map_placelist_p places)
{
  carmen_FILE *fp;
  int chunk_type, chunk_size;
  char chunk_description[12];

  fp = carmen_fopen(filename, "r");
  if(fp == NULL) {
    fprintf(stderr, "Error: could not open file %s for reading.\n",
	    filename);
    return -1;
  }
  if(carmen_map_advance_to_named_chunk(fp, CARMEN_MAP_PLACES_CHUNK, chunk_name) < 0) {
    fprintf(stderr, "Error: Could not find a places chunk named \"%s\"\n", chunk_name);
    carmen_fclose(fp);
    return -1;
  }
  chunk_type = carmen_fgetc(fp);
  carmen_fread(&chunk_size, sizeof(int), 1, fp);
  carmen_fread(chunk_description, 10, 1, fp);

  if (CARMEN_MAP_CHUNK_IS_NAMED(chunk_type)) {
    if (read_string(NULL, -1, fp) < 0) {
      carmen_warn("Error: Unexpected EOF.\n");
      carmen_fclose(fp);
      return -1;
    }
  }

  return carmen_map_read_places_chunk_data(fp, places);
}

static int carmen_map_read_laserscans_chunk_data(carmen_FILE *fp,
						 carmen_laser_scan_p *scan_list,
						 int *num_scans)
{
  int i, j;
  double double_var;

  carmen_fread(num_scans, sizeof(int), 1, fp);
  *scan_list =
    (carmen_laser_scan_p)calloc(*num_scans, sizeof(carmen_laser_scan_t));
  carmen_test_alloc(*scan_list);
  for(i = 0; i < *num_scans; i++) {
    carmen_fread(&double_var, sizeof(double), 1, fp);
    (*scan_list)[i].x = double_var;
    carmen_fread(&double_var, sizeof(double), 1, fp);
    (*scan_list)[i].y = double_var;
    carmen_fread(&double_var, sizeof(double), 1, fp);
    (*scan_list)[i].theta = double_var;

    carmen_fread(&((*scan_list)[i].num_readings), sizeof(int), 1, fp);

    (*scan_list)[i].range = (double *)calloc((*scan_list)[i].num_readings,
					    sizeof(double));
    carmen_test_alloc((*scan_list)[i].range);
    for (j = 0; j < (*scan_list)[i].num_readings; j++) {
      carmen_fread(&double_var, sizeof(double), 1, fp);
      (*scan_list)[i].range[j] = double_var;
    }
  }

  carmen_fclose(fp);
  return 0;
}

int carmen_map_read_laserscans_chunk(char *filename,
				     carmen_laser_scan_p *scan_list,
				     int *num_scans)
{
  carmen_FILE *fp;
  int chunk_type, chunk_size;
  char chunk_description[12];

  fp = carmen_fopen(filename, "r");
  if(fp == NULL) {
    fprintf(stderr, "Error: could not open file %s for reading.\n",
	    filename);
    return -1;
  }
  if(carmen_map_advance_to_chunk(fp, CARMEN_MAP_LASERSCANS_CHUNK) < 0) {
    fprintf(stderr, "Error: Could not find a laserscans chunk.\n");
    carmen_fclose(fp);
    return -1;
  }
  chunk_type = carmen_fgetc(fp);
  carmen_fread(&chunk_size, sizeof(int), 1, fp);
  carmen_fread(chunk_description, 10, 1, fp);

  if (CARMEN_MAP_CHUNK_IS_NAMED(chunk_type)) {
    if (read_string(NULL, -1, fp) < 0) {
      carmen_warn("Error: Unexpected EOF.\n");
      carmen_fclose(fp);
      return -1;
    }
  }

  return carmen_map_read_laserscans_chunk_data(fp, scan_list, num_scans);
}

int carmen_map_read_named_laserscans_chunk(char *filename, char *chunk_name,
					   carmen_laser_scan_p *scan_list,
					   int *num_scans)
{
  carmen_FILE *fp;
  int chunk_type, chunk_size;
  char chunk_description[12];

  fp = carmen_fopen(filename, "r");
  if(fp == NULL) {
    fprintf(stderr, "Error: could not open file %s for reading.\n",
	    filename);
    return -1;
  }
  if(carmen_map_advance_to_named_chunk(fp, CARMEN_MAP_LASERSCANS_CHUNK, chunk_name) < 0) {
    fprintf(stderr, "Error: Could not find a laserscans chunk named \"%s\"\n", chunk_name);
    carmen_fclose(fp);
    return -1;
  }
  chunk_type = carmen_fgetc(fp);
  carmen_fread(&chunk_size, sizeof(int), 1, fp);
  carmen_fread(chunk_description, 10, 1, fp);

  if (CARMEN_MAP_CHUNK_IS_NAMED(chunk_type)) {
    if (read_string(NULL, -1, fp) < 0) {
      carmen_warn("Error: Unexpected EOF.\n");
      carmen_fclose(fp);
      return -1;
    }
  }

  return carmen_map_read_laserscans_chunk_data(fp, scan_list, num_scans);
}

static int carmen_map_read_hmap_chunk_data(carmen_FILE *fp, carmen_hmap_p hmap)
{
  int i, j, k, n = 0;
  double x, y, theta;
  char buf[128];

  carmen_fread(&hmap->num_zones, sizeof(int), 1, fp);
  hmap->zone_names = (char **) calloc(hmap->num_zones, sizeof(char *));
  carmen_test_alloc(hmap->zone_names);
  for (i = 0; i < hmap->num_zones; i++) {
    n = read_string(buf, 128, fp);
    if (n < 0) {
      free(hmap->zone_names);
      hmap->zone_names = NULL;
      return -1;
    }
    hmap->zone_names[i] = (char *) calloc(n+1, sizeof(char));
    carmen_test_alloc(hmap->zone_names[i]);
    strncpy(hmap->zone_names[i], buf, n+1);
  }

  carmen_fread(&hmap->num_links, sizeof(int), 1, fp);
  hmap->links = (carmen_hmap_link_p) calloc(hmap->num_links, sizeof(carmen_hmap_link_t));
  carmen_test_alloc(hmap->links);
  for (i = 0; i < hmap->num_links; i++) {
    carmen_fread(&hmap->links[i].type, sizeof(int), 1, fp);
    carmen_fread(&hmap->links[i].degree, sizeof(int), 1, fp);
    hmap->links[i].keys = (int *) calloc(hmap->links[i].degree, sizeof(int));
    carmen_test_alloc(hmap->links[i].keys);
    carmen_fread(hmap->links[i].keys, sizeof(int), hmap->links[i].degree, fp);
    switch (hmap->links[i].type) {
    case CARMEN_HMAP_LINK_DOOR:       n = 2; break;
    case CARMEN_HMAP_LINK_ELEVATOR:   n = 1; break;
    }
    hmap->links[i].num_points = n * hmap->links[i].degree;
    hmap->links[i].points = (carmen_point_p) calloc(hmap->links[i].num_points, sizeof(carmen_point_t));
    carmen_test_alloc(hmap->links[i].points);
    for (j = 0; j < hmap->links[i].degree; j++) {
      for (k = 0; k < n; k++) {
	carmen_fread(&x, sizeof(double), 1, fp);
	hmap->links[i].points[j*n+k].x = (double) x;
	carmen_fread(&y, sizeof(double), 1, fp);
	hmap->links[i].points[j*n+k].y = (double) y;
	carmen_fread(&theta, sizeof(double), 1, fp);
	hmap->links[i].points[j*n+k].theta = (double) theta;
      }
    }
  }

  return 0;
}

int carmen_map_read_hmap_chunk(char *filename, carmen_hmap_p hmap)
{
  carmen_FILE *fp;
  int chunk_type, chunk_size;
  char chunk_description[12];

  fp = carmen_fopen(filename, "r");
  if(fp == NULL) {
    fprintf(stderr, "Error: could not open file %s for reading.\n",
	    filename);
    return -1;
  }
  if(carmen_map_advance_to_chunk(fp, CARMEN_MAP_HMAP_CHUNK) < 0) {
    carmen_fclose(fp);
    return -1;
  }
  chunk_type = carmen_fgetc(fp);
  carmen_fread(&chunk_size, sizeof(int), 1, fp);
  carmen_fread(chunk_description, 10, 1, fp);

  if (CARMEN_MAP_CHUNK_IS_NAMED(chunk_type)) {
    if (read_string(NULL, -1, fp) < 0) {
      carmen_warn("Error: Unexpected EOF.\n");
      carmen_fclose(fp);
      return -1;
    }
  }

  return carmen_map_read_hmap_chunk_data(fp, hmap);
}




