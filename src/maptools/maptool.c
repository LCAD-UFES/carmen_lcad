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

#ifndef NO_GRAPHICS
#include <carmen/carmen_graphics.h>
#else
#include <carmen/carmen.h>
#endif

#include <getopt.h>
#include "map_io.h"
#include "map_util.h"

static void main_usage(char *prog_name)
{
  carmen_die("\nUsage: %s <action> <...>\n\n"
	     "<action> is one of: toppm, "
#ifndef NO_GRAPHICS
	     "tomap, "
#endif
	     "rotate, minimize, add_place, add_offset, \n"
	     "strip, info.\n"
	     "Run %s help <action> to get help on using each action.\n\n",
	     prog_name, prog_name);  
}

static int handle_options(int argc,  char *argv[], int *force)
{
  static struct option long_options[] = {
    {"force", 0, NULL, 'f'},
    {"help", 0, NULL, 'h'},
    {0, 0, 0, 0}
  };

  int option_index = 0;
  int c;

  *force = 0;
  opterr = 0;
  while (1) {
    c = getopt_long (argc, argv, "fh?", long_options, &option_index);
    if (c == -1)
      break;
    switch (c) {
    case 'f':
      *force = 1;
      break;
    case 'h':
      main_usage(argv[0]);
    case '?':
    default:
      carmen_warn("\nUnknown option character %c", optopt);
      if (isdigit(optopt) || optopt == '.')
	carmen_die("\n\nThis looks like a negative numeric value. "
		   "If you want to pass a negative \nvalue, you need "
		   "to suppress option parsing by using -- after the "
		   "command.\n Any options before -- will be parsed, but "
		   "nothing after. For example:\n\n"
		   "%% maptool add_offset -- map.cmf -1 -1 0\n\n"
		   "works where \n\n"
		   "%% maptool add_offset map.cmf -1 -1 0\n\n"
		   "does not.\n");
      else
	main_usage(argv[0]);
      break;
    }
  }

  return optind;
}

static void help(int argc, char **argv)
{
  char *action;

  if (argc < 3 || argc > 3) 
    main_usage(argv[0]);

  action = argv[2];
  if (carmen_strcasecmp(action, "info") == 0)
    carmen_die("\nUsage: %s info <map filename>\n\n", 
	       argv[0]); 
  else if (carmen_strcasecmp(action, "toppm") == 0)
    carmen_die("\nUsage: %s toppm <map filename> <ppm filename>\n\n", 
	       argv[0]); 
  else if (carmen_strcasecmp(action, "tomap") == 0)
    carmen_die("\nUsage: %s tomap <resolution> <image file> "
	       "<map filename>\n\nThe image file can be in most standard "
	       "formats such as gif, png, jpeg, etc.\n\n", argv[0]); 
  else if (carmen_strcasecmp(action, "rotate") == 0)
    carmen_die("\nUsage: %s rotate <rotation in degrees> <in map filename> "
	       "<out map filename>\n\nThe rotation angle must be in multiples "
	       "of 90degrees -- arbitrary rotations are not\nyet supported."
	       "\n\n", argv[0]);
  else if (carmen_strcasecmp(action, "minimize") == 0)
    carmen_die("\nUsage: %s minimize <in map filename> <out map filename>\n\n",
	       argv[0]);
  else if (carmen_strcasecmp(action, "add_place") == 0) {
    fprintf(stderr, "\nUsage: %s <mapfilename> <placename> <place params>\n",
	    argv[0]);
    fprintf(stderr, "       2, 3, or 6 place parameters can be given.\n");
    fprintf(stderr, "       2 parameters - named position (x, y)\n");
    fprintf(stderr, "       3 parameters - named pose (x, y, theta)\n");
    fprintf(stderr, "       6 parameters - initialization position\n");
    fprintf(stderr, "                      (x, y, theta, std_x, std_y, "
	    "std_theta)\n");
    fprintf(stderr, "\nRemember: The x/y/std_x/std_y co-ordinates are in "
	    "%sMETRES%s, not in\n", carmen_red_code, carmen_normal_code);
    fprintf(stderr, "grid cells. \n");
    fprintf(stderr, "\nAlso: theta is in %sDEGREES%s. I will print out the "
	    "conversion to radians\n", carmen_red_code, carmen_normal_code);
    fprintf(stderr, "automatically for you.\n\n");
    fprintf(stderr, "Note that unlike rotate, minimize, etc., add_place "
	    "performs an in-place edit\nto the map, and creates a backup "
	    "copy.\n\n");
    exit(0);
  } else if (carmen_strcasecmp(action, "add_offset") == 0) {
    carmen_warn("\nUsage: %s <mapfilename> <x> <y> <theta>\n",
		argv[0]);
    carmen_warn("       <theta> should be given in degrees.\n");
    exit(0);
  } else if (carmen_strcasecmp(action, "strip") == 0) {
    fprintf(stderr, "\nUsage: %s strip <in map filename> <in map filename> "
	    "<chunk type>\n\n", argv[0]);
    fprintf(stderr, "If the map file contains a data chunk of this type, "
	    "it will\nbe removed on output. All other chunks will be "
	    "passed through untouched.\n");
    fprintf(stderr, "<chunk type> can be one of \"laserscans\", "
	    "\"places\", \"gridmap\", \"offlimits\", ");
    fprintf(stderr, "                                \"expected\".\n\n");

    exit(0);
  }

  carmen_warn("\nUnrecognized command %s\n", action);
  main_usage(argv[0]);
}


static char *check_mapfile(char *filename)
{
   if(!carmen_file_exists(filename))
     carmen_die("Map file %s does not exist.\n", filename);

   if (!carmen_map_file(filename)) 
     carmen_die("File %s is not a recognizable map file.\n", filename);
   
   return filename;
}

static char *check_output(char *filename, int force)
{
  char key;
  
  if (!force && carmen_file_exists(filename)) {
    fprintf(stderr, "Overwrite %s? ", filename);
    scanf("%c", &key);
    if (toupper(key) != 'Y')
      exit(-1);
  }
  
  return filename;
}

static void toppm(int argc, char *argv[]) 
{
  int err;
  int next_arg;
  int force;
  char *input_filename, *output_filename;
  carmen_map_t map;

  next_arg = handle_options(argc, argv, &force);

  if(argc - next_arg != 3) {
    carmen_warn("\nError: wrong number of parameters.\n");    
    carmen_die("\nUsage: %s toppm <map filename> <ppm filename>\n\n", 
	       argv[0]); 
  }

  input_filename = check_mapfile(argv[next_arg+1]);
  output_filename = check_output(argv[next_arg+2], force);

  if(carmen_map_read_gridmap_chunk(input_filename, &map) < 0) 
    carmen_die("Error: %s did not contain a gridmap.\n", input_filename);
  carmen_map_read_offlimits_chunk_into_map(input_filename, &map);

  err = carmen_map_write_to_ppm(&map, output_filename);
  if (err < 0)
    carmen_die_syserror("Could not write ppm to %s", output_filename);
}


#ifndef NO_GRAPHICS
static void tomap(int argc, char *argv[]) 
{
  int next_arg;
  int force;
  double resolution;
  char *input_filename, *output_filename;
  carmen_map_t *map;
  carmen_FILE *out_fp;
  char buf[1024];

  gtk_init (&argc, &argv);

  next_arg = handle_options(argc, argv, &force);

  if(argc - next_arg != 4) {
    carmen_warn("\nError: wrong number of parameters.\n");    
    carmen_die("\nUsage: %s tomap <resolution> <ppm filename> "
	       "<map filename>\n\n", argv[0]); 
  }

  resolution = (double)atof(argv[next_arg+1]);

  if (resolution == 0) 
    carmen_die("%s translated to a resolution of 0.\n"
		"A positive, non-zero resolution is required.\n", 
		argv[next_arg+1]);
  
  input_filename = argv[next_arg+2];
  if(!carmen_file_exists(input_filename)) 
     carmen_die("Image file %s does not exist.\n", input_filename);

  output_filename = check_output(argv[next_arg+3], force);

  map = carmen_map_imagefile_to_map(input_filename, resolution);

  out_fp = carmen_fopen(output_filename, "w");
  if (carmen_map_write_id(out_fp) < 0)
    carmen_die_syserror("Couldn't write map id to %s", output_filename);

  sprintf(buf, "Created from %s", input_filename);
  if (carmen_map_write_creator_chunk(out_fp, "img_to_map", buf) < 0)
    carmen_die_syserror("Couldn't write creator chunk to %s", output_filename);

  if (carmen_map_write_gridmap_chunk(out_fp, map->map, map->config.x_size,
				     map->config.y_size, 
				     map->config.resolution) < 0)
    carmen_die_syserror("Couldn't write gridmap chunk to %s", output_filename);

  carmen_fclose(out_fp);
}
#endif

static void rotate(int argc, char *argv[])
{
  int force;
  char *input_filename, *output_filename;
  int next_arg;

  carmen_FILE *in_fp, *out_fp;
  int ret_val;
  carmen_map_t map;
  int rotation = 0;
  double remain;
  int degrees_angle;
  carmen_offlimits_list_t offlimits_list;
  carmen_map_placelist_t places_list;

  next_arg = handle_options(argc, argv, &force);
  next_arg++;

  if(argc - next_arg != 4) {
    carmen_warn("\nError: wrong number of parameters.\n");    
    carmen_die("\nUsage: %s rotate <rotation in degrees> <in map filename> "
	       "<out map filename>\n\n", argv[0]);
  }

  degrees_angle = (int)(atof(argv[next_arg]) / 90);
  remain = fabs(degrees_angle*90 - atof(argv[next_arg]));
  if (carmen_radians_to_degrees(remain) > 2)
    carmen_die("Rotations only supported in increments of 90 degrees.\n");
  else
    rotation = (int)atof(argv[next_arg]) / 90;

  input_filename = check_mapfile(argv[next_arg+1]);
  output_filename = check_output(argv[next_arg+2], force);

  carmen_warn("Rotating by %d degrees\n", rotation*90);

  /*
   * Read the gridmap, places and offlimits chunks and rotate them
   */ 

  ret_val = carmen_map_chunk_exists(input_filename, CARMEN_MAP_GRIDMAP_CHUNK);
  if (ret_val < 0)    
    carmen_die_syserror("Couldn't check existence of GRIDMAP_CHUNK in %s", 
			input_filename);
  
  if (carmen_map_read_gridmap_chunk(input_filename, &map) < 0)
    carmen_die_syserror("Couldn't read GRIDMAP_CHUNK from %s", input_filename);

  carmen_rotate_gridmap(&map, rotation);

  ret_val = carmen_map_chunk_exists(input_filename, 
				    CARMEN_MAP_OFFLIMITS_CHUNK);
  if (ret_val > 0) {
    ret_val = carmen_map_read_offlimits_chunk
      (input_filename, &(offlimits_list.offlimits), 
       &(offlimits_list.list_length));

    if (ret_val < 0)
      carmen_die_syserror("Couldn't read OFFLIMITS_CHUNK in %s",
			  input_filename);

    carmen_rotate_offlimits(map.config, &offlimits_list, rotation);
  } else
    offlimits_list.list_length = 0;

  ret_val = carmen_map_chunk_exists(input_filename, CARMEN_MAP_PLACES_CHUNK);
  if (ret_val > 0) {
    ret_val = carmen_map_read_places_chunk(input_filename, &places_list);
    if (ret_val < 0)
      carmen_die_syserror("Couldn't read PLACES_CHUNK in %s", input_filename);

    carmen_rotate_places(map.config, &places_list, rotation);
  } else
    places_list.num_places = 0;


  /*
   * Pass everything else through untouched, and then write the rotated
   * chunks at the end.
   */

  in_fp = carmen_fopen(input_filename, "r");
  if (in_fp == NULL)
    carmen_die_syserror("Couldn't open %s for reading", input_filename);

  out_fp = carmen_fopen(output_filename, "w");
  if (out_fp == NULL)
    carmen_die_syserror("Couldn't open %s for writing", output_filename);
  
  if (carmen_map_vstrip(in_fp, out_fp, 3, CARMEN_MAP_GRIDMAP_CHUNK,
			CARMEN_MAP_OFFLIMITS_CHUNK, 
			CARMEN_MAP_PLACES_CHUNK) < 0) 
    carmen_die_syserror("Couldn't strip map to %s", output_filename);

  if (carmen_map_write_gridmap_chunk(out_fp, map.map, map.config.x_size, 
				     map.config.y_size, 
				     map.config.resolution) < 0)
    carmen_die_syserror("Couldn't write gridmap to %s", output_filename);

  if (offlimits_list.list_length > 0) {
    if (carmen_map_write_offlimits_chunk(out_fp, offlimits_list.offlimits,
					 offlimits_list.list_length) < 0)
      carmen_die_syserror("Couldn't write offlimits list to %s", 
			  output_filename);
  }

  if (places_list.num_places > 0) {
    if (carmen_map_write_places_chunk(out_fp, places_list.places, 
				      places_list.num_places) < 0)
      carmen_die_syserror("Couldn't write places list to %s", output_filename);
  }

  carmen_fclose(in_fp);
  carmen_fclose(out_fp);
}

static void minimize(int argc, char *argv[])
{
  char *input_filename, *output_filename;
  int next_arg;
  carmen_FILE *in_fp, *out_fp;

  int ret_val;
  carmen_map_t map;
  int x_offset, y_offset;
  carmen_offlimits_list_t offlimits_list;
  carmen_map_placelist_t places;
  int force;
  int previous_num_places;

  next_arg = handle_options(argc, argv, &force);
  next_arg++;

  if(argc - next_arg != 2) {
    carmen_warn("\nError: wrong number of parameters.\n");    
    carmen_die("\nUsage: %s minimize <in map filename> <out map filename>\n\n",
	       argv[0]);
  }

  input_filename = check_mapfile(argv[next_arg]);
  output_filename = check_output(argv[next_arg+1], force);

  /*
   * Read the gridmap, places and offlimits chunks and minimize them
   */ 

  if (carmen_map_read_gridmap_chunk(input_filename, &map) < 0) {
    carmen_die_syserror("Couldn't read GRIDMAP_CHUNK from %s", input_filename);
  }

  carmen_warn("Map size was %d x %d, ",map.config.x_size, map.config.y_size);

  carmen_minimize_gridmap(&map, &x_offset, &y_offset);

  carmen_warn("is now %d x %d (offset %d, %d)\n", map.config.x_size, 
	      map.config.y_size, x_offset, y_offset);


  ret_val = carmen_map_chunk_exists(input_filename, 
				    CARMEN_MAP_OFFLIMITS_CHUNK);
  if (ret_val > 0) {
    ret_val = carmen_map_read_offlimits_chunk
      (input_filename, &(offlimits_list.offlimits), 
       &(offlimits_list.list_length));
    if (ret_val < 0)
      carmen_die_syserror("Couldn't read OFFLIMITS_CHUNK in %s",
			  input_filename);

    carmen_minimize_offlimits(&offlimits_list, x_offset*map.config.resolution, 
			      y_offset*map.config.resolution);
  } else
    offlimits_list.list_length = 0;

  ret_val = carmen_map_chunk_exists(input_filename, CARMEN_MAP_PLACES_CHUNK);
  if (ret_val > 0) {
    ret_val = carmen_map_read_places_chunk(input_filename, &places);
    if (ret_val < 0)
      carmen_die_syserror("Couldn't read PLACES_CHUNK in %s", input_filename);

    previous_num_places = places.num_places;
    carmen_minimize_places(&places, x_offset*map.config.resolution, 
			   y_offset*map.config.resolution, 
			   map.config.x_size*map.config.resolution,
			   map.config.y_size*map.config.resolution);
    if (places.num_places < previous_num_places)
      carmen_warn("%d place locations were dropped from the map after "
		  "minimization.\n", previous_num_places-places.num_places);
  } else
    places.num_places = 0;

  /*
   * Pass everything else through untouched, and then write the rotated
   * chunks at the end.
   */


  in_fp = carmen_fopen(input_filename, "r");
  if (in_fp == NULL)
    carmen_die_syserror("Couldn't open %s for reading", input_filename);

  out_fp = carmen_fopen(output_filename, "w");
  if (out_fp == NULL)
    carmen_die_syserror("Couldn't open %s for writing", output_filename);
  
  if (carmen_map_vstrip(in_fp, out_fp, 3, CARMEN_MAP_GRIDMAP_CHUNK,
			CARMEN_MAP_OFFLIMITS_CHUNK, 
			CARMEN_MAP_PLACES_CHUNK) < 0) 
    carmen_die_syserror("Couldn't strip map to %s", output_filename);
  

  if (carmen_map_write_gridmap_chunk(out_fp, map.map, map.config.x_size, 
				     map.config.y_size, 
				     map.config.resolution) < 0)
    carmen_die_syserror("Couldn't write gridmap to %s", output_filename);

  if (offlimits_list.list_length > 0) {
    if (carmen_map_write_offlimits_chunk(out_fp, offlimits_list.offlimits,
					 offlimits_list.list_length) < 0)
      carmen_die_syserror("Couldn't write offlimits list to %s", 
			  output_filename);
  }

  if (places.num_places > 0) {
    if (carmen_map_write_places_chunk(out_fp, places.places,
				      places.num_places) < 0)
      carmen_die_syserror("Couldn't write places list to %s", output_filename);
  }

  carmen_fclose(in_fp);
  carmen_fclose(out_fp);
}

static void add_place(int argc, char *argv[])
{
  char *input_filename;
  int next_arg;
  int force;
  char cmd[1024];
  char tmp_filename[1024];
  carmen_FILE *fp_in, *fp_out;
  carmen_map_placelist_t place_list;
  carmen_place_p places;
  int system_err;
  int num_args;

  next_arg = handle_options(argc, argv, &force);
  next_arg++;

  num_args = argc - next_arg;

  if(num_args != 4 && num_args != 5 && num_args != 8) {
    carmen_warn("\nError: wrong number of parameters.\n");    
    carmen_warn("\nUsage: %s <mapfilename> <placename> <place params>\n",
		argv[0]);
    carmen_warn("       2, 3, or 6 place parameters can be given.\n");
  }
     
  input_filename = check_mapfile(argv[next_arg]);
  next_arg++;

  if(!carmen_map_file(input_filename))
    carmen_die("Error: %s does not appear to be a valid carmen map file;\n" 
	       "if it is gzipped, make sure it has a \".gz\" extension.\n",
	       input_filename);

  if(!carmen_map_chunk_exists(input_filename, CARMEN_MAP_PLACES_CHUNK)) {
    place_list.num_places = 1;
    place_list.places = (carmen_place_p)calloc(1, sizeof(carmen_place_t)); 
    carmen_test_alloc(place_list.places);
  } else {
    carmen_map_read_places_chunk(input_filename, &place_list);
    place_list.num_places++;
    place_list.places = (carmen_place_p)realloc
      (place_list.places, sizeof(carmen_place_t) * place_list.num_places); 
    /* check_alloc checked */
  }
  carmen_test_alloc(place_list.places);

  places = place_list.places;

  if(num_args == 4) {
    strcpy(places[place_list.num_places - 1].name, argv[next_arg]);
    places[place_list.num_places - 1].type = CARMEN_NAMED_POSITION_TYPE;
    places[place_list.num_places - 1].x = atof(argv[next_arg+1]);
    places[place_list.num_places - 1].y = atof(argv[next_arg+2]);
  }
  else if(num_args == 5) {
    strcpy(places[place_list.num_places - 1].name, argv[next_arg]);
    places[place_list.num_places - 1].type = CARMEN_NAMED_POSE_TYPE;
    places[place_list.num_places - 1].x = atof(argv[next_arg+1]);
    places[place_list.num_places - 1].y = atof(argv[next_arg+2]);
    places[place_list.num_places - 1].theta = 
      carmen_degrees_to_radians(atof(argv[next_arg+3]));
    fprintf(stderr, "Set (%s %s %s) to (%f.2m, %f.2m, %f.2 rad)\n", argv[4],
	    argv[5], argv[6], places[place_list.num_places - 1].x, 
	    places[place_list.num_places - 1].y,
	    places[place_list.num_places - 1].theta);
  }
  else {
    strcpy(places[place_list.num_places - 1].name, argv[next_arg]);
    places[place_list.num_places - 1].type = CARMEN_LOCALIZATION_INIT_TYPE;
    places[place_list.num_places - 1].x = atof(argv[next_arg+1]);
    places[place_list.num_places - 1].y = atof(argv[next_arg+2]);
    places[place_list.num_places - 1].theta = 
      carmen_degrees_to_radians(atof(argv[next_arg+3]));
    places[place_list.num_places - 1].x_std = atof(argv[next_arg+4]);
    places[place_list.num_places - 1].y_std = atof(argv[next_arg+5]);
    places[place_list.num_places - 1].theta_std = 
      carmen_degrees_to_radians(atof(argv[next_arg+6]));

    fprintf(stderr, "Set (%s %s %s %s %s %s) to:\n(%f.2m +/- %f.2m, %f.2m "
	    "+/- %f.2m, %f.2 +/- %f.2 rad)\n", 
	    argv[next_arg+1], argv[next_arg+2], argv[next_arg+3], 
	    argv[next_arg+4], argv[next_arg+5], argv[next_arg+6], 
	    places[place_list.num_places - 1].x, 
	    places[place_list.num_places - 1].x_std, 
	    places[place_list.num_places - 1].y, 
	    places[place_list.num_places - 1].y_std,
	    places[place_list.num_places - 1].theta, 
	    places[place_list.num_places - 1].theta_std);    
  }
  
  fp_in = carmen_fopen(input_filename, "r");
  if(fp_in == NULL)
    carmen_die_syserror("Error: file %s could not be opened for reading", 
			input_filename);
  
  strcpy(tmp_filename, "/tmp/newmapXXXXXX");
  system_err = mkstemp(tmp_filename);
  if (system_err == -1) 
    carmen_die_syserror("I need to create a temporary file in /tmp, but I "
			"can't for the\n following reason\n");
  
  fp_out = carmen_fopen(tmp_filename, "w");
  if(fp_out == NULL)
    carmen_die_syserror("Error: file could not be opened for writing");

  if(carmen_map_strip(fp_in, fp_out, CARMEN_MAP_PLACES_CHUNK) < 0)
    carmen_die("Error: could not strip places from map file.\n");
  if(carmen_map_write_places_chunk(fp_out, place_list.places, 
				   place_list.num_places) < 0)
    carmen_die("Error: could not write places chunk.\n");

  carmen_fclose(fp_in);
  carmen_fclose(fp_out);

  sprintf(cmd, "mv -f %s %s", tmp_filename, input_filename);
  system_err = system(cmd);
  if (system_err != 0) 
    carmen_die("I created a temporary file contained the map with the new "
	       "place name\nat %s. I tried to copy the new file onto your "
	       "old file, but the copy\nfailed for the following reason: %s\n"
	       "\n"
	       "You will have to copy the new file over yourself.\n", 
	       tmp_filename, strerror(errno));
}

static void add_offset(int argc, char *argv[])
{
  char *input_filename;
  int next_arg;
  int force;
  char cmd[1024];
  char tmp_filename[1024];
  carmen_FILE *fp_in, *fp_out;
  carmen_global_offset_t offset;
  int system_err;
  int num_args;
  int response;

  next_arg = handle_options(argc, argv, &force);
  next_arg++;

  num_args = argc - next_arg;

  if(num_args != 4) {
    carmen_warn("\nError: wrong number of parameters.\n");    
    carmen_warn("\nUsage: %s <mapfilename> <x> <y> <theta>\n",
		argv[0]);
    carmen_warn("       <theta> should be given in degrees.\n");
  }
     
  input_filename = check_mapfile(argv[next_arg]);
  next_arg++;

  if(!carmen_map_file(input_filename))
    carmen_die("Error: %s does not appear to be a valid carmen map file;\n" 
	       "if it is gzipped, make sure it has a \".gz\" extension.\n",
	       input_filename);

  if(carmen_map_chunk_exists(input_filename, CARMEN_MAP_GLOBAL_OFFSET_CHUNK)) {
    carmen_warn("Offset chunk exists already. Replace? [y/n] ");
    response = getchar();
    if (tolower(response) != 'y')
      exit(0);
    carmen_warn("Replacing...\n");
  }

  offset.x = atof(argv[next_arg]);
  offset.y = atof(argv[next_arg+1]);
  offset.theta = carmen_degrees_to_radians(atof(argv[next_arg+2]));

  fprintf(stderr, "Set (%s %s %s) to (%.2f m, %.2f m, %.2f rad)\n", 
	  argv[next_arg], argv[next_arg+1], argv[next_arg+2], 
	  offset.x, offset.y, offset.theta);
  
  fp_in = carmen_fopen(input_filename, "r");
  if(fp_in == NULL)
    carmen_die_syserror("Error: file %s could not be opened for reading", 
			input_filename);
  
  strcpy(tmp_filename, "/tmp/newmapXXXXXX");
  system_err = mkstemp(tmp_filename);
  if (system_err == -1) 
    carmen_die_syserror("I need to create a temporary file in /tmp, but I "
			"can't for the\n following reason\n");
  
  fp_out = carmen_fopen(tmp_filename, "w");
  if(fp_out == NULL)
    carmen_die_syserror("Error: file could not be opened for writing");

  if(carmen_map_strip(fp_in, fp_out, CARMEN_MAP_GLOBAL_OFFSET_CHUNK) < 0)
     carmen_die("Error: could not strip global offset from map file.\n");
  if(carmen_map_write_global_offset_chunk(fp_out, &offset) < 0)
    carmen_die("Error: could not write offset chunk.\n");

  carmen_fclose(fp_in);
  carmen_fclose(fp_out);

  sprintf(cmd, "mv -f %s %s", tmp_filename, input_filename);
  system_err = system(cmd);
  if (system_err != 0) 
    carmen_die("I created a temporary file contained the map with the new "
	       "place name\nat %s. I tried to copy the new file onto your "
	       "old file, but the copy\nfailed for the following reason: %s\n"
	       "\n"
	       "You will have to copy the new file over yourself.\n", 
	       tmp_filename, strerror(errno));
}

static void strip(int argc, char *argv[])
{
  char *input_filename, *output_filename, *chunk_type;
  carmen_FILE *fp_in, *fp_out;
  int next_arg;
  int force;

  next_arg = handle_options(argc, argv, &force);
  next_arg++;

  if(argc - next_arg != 4) {
    carmen_warn("\nError: wrong number of parameters.\n");    
    carmen_die("\nUsage: %s strip <in map filename> "
	       "<out map filename> <chunk type>\n\n", argv[0]);
  }

  input_filename = check_mapfile(argv[next_arg]);
  output_filename = check_output(argv[next_arg+1], force);
  chunk_type = argv[next_arg+2];

  fp_in = carmen_fopen(input_filename, "r");
  if(fp_in == NULL) 
    carmen_die_syserror("Could not open %s for reading", input_filename);

  fp_out = carmen_fopen(output_filename, "w");
  if(fp_out == NULL) 
    carmen_die_syserror("Could not open %s for writing", output_filename);
  
  if (carmen_strcasecmp(chunk_type, "laserscans") == 0) {
    if (carmen_map_strip(fp_in, fp_out, CARMEN_MAP_LASERSCANS_CHUNK) < 0) 
      carmen_die_syserror("Error: could not strip file");
  }
  else if (carmen_strcasecmp(chunk_type, "gridmap") == 0) {
    if (carmen_map_strip(fp_in, fp_out, CARMEN_MAP_GRIDMAP_CHUNK) < 0) 
      carmen_die_syserror("Error: could not strip file");
  }
  else if (carmen_strcasecmp(chunk_type, "offlimits") == 0) {
    if (carmen_map_strip(fp_in, fp_out, CARMEN_MAP_OFFLIMITS_CHUNK) < 0) 
      carmen_die_syserror("Error: could not strip file");
  }
  else if (carmen_strcasecmp(chunk_type, "expected") == 0) {
    if (carmen_map_strip(fp_in, fp_out, CARMEN_MAP_EXPECTED_CHUNK) < 0) 
      carmen_die_syserror("Error: could not strip file");
  }
  else if (carmen_strcasecmp(chunk_type, "places") == 0) {
    if (carmen_map_strip(fp_in, fp_out, CARMEN_MAP_PLACES_CHUNK) < 0) 
      carmen_die_syserror("Error: could not strip file");
  }

  carmen_fclose(fp_in);
  carmen_fclose(fp_out); 
}

static void info(int argc, char *argv[])
{
  time_t creation_time;
  char *filename;
  char username[100], origin[100], description[100];
  int chunk_size, i;
  carmen_map_config_t config;
  carmen_map_placelist_t place_list;
  carmen_offlimits_t *offlimits_list;
  int list_length;
  carmen_global_offset_t offset;

  /* Check for the appropriate command line argument */
  if(argc != 3) 
    carmen_die("Error in info: wrong number of parameters.\n");

  filename = argv[2];

  /* Make sure the file exists */
  if (!carmen_map_file(filename))
    carmen_die("Error: %s does not appear to be a valid carmen map file;\n"
	       "if it is gzipped, make sure it has a \".gz\" extension.\n" 
	       "Do you have an incompatible file?\n"
	       "Current map version: %s\n", filename, CARMEN_MAP_VERSION);
  
  /* Print the CREATOR information if the chunk exists */
  if(carmen_map_read_creator_chunk(filename, &creation_time,
				 username, origin, description) == 0) {
    printf("\nMap CREATOR information:\n");
    printf("------------------------\n");
    printf("Map author    : %s\n", username);
    printf("Creation date : %s", asctime(localtime(&creation_time)));
    printf("Origin        : %s\n", origin);
    printf("Description   : %s", description);
    printf("\n");
  }

  /* Print the list of chunks in the file */
  printf("Map contains the following CHUNK types:\n");
  printf("---------------------------------------\n");
  chunk_size = carmen_map_chunk_exists(filename, CARMEN_MAP_CREATOR_CHUNK);
  if(chunk_size > 0)
    printf("CREATOR       : yes (%d bytes)\n", chunk_size);
  else
    printf("CREATOR       : no\n");
  chunk_size = carmen_map_chunk_exists(filename, CARMEN_MAP_GRIDMAP_CHUNK);
  if(chunk_size > 0)
    {
      carmen_map_read_gridmap_config(filename, &config);
      printf("GRIDMAP       : yes  size : %4d x %4d, resolution : %.2f m/cell (%d bytes)\n", 
	     config.x_size, config.y_size, config.resolution, chunk_size);
    }
  else
    printf("GRIDMAP       : no\n");
  chunk_size = carmen_map_chunk_exists(filename, CARMEN_MAP_OFFLIMITS_CHUNK);
  if(chunk_size > 0) {
    printf("OFFLIMITS     : yes (%d bytes)\n", chunk_size);
    carmen_map_read_offlimits_chunk(filename, &offlimits_list, &list_length);
    for (i = 0; i < list_length; i++) {
      if (offlimits_list[i].type == CARMEN_OFFLIMITS_POINT_ID)
	carmen_warn("\tOfflimits %d: point: %d %d\n", i, offlimits_list[i].x1,
		    offlimits_list[i].y1);
      else if (offlimits_list[i].type == CARMEN_OFFLIMITS_LINE_ID)
	carmen_warn("\tOfflimits %d: line: %d %d %d %d\n", i, offlimits_list[i].x1,
		    offlimits_list[i].y1, offlimits_list[i].x2, offlimits_list[i].y2);
      else if (offlimits_list[i].type == CARMEN_OFFLIMITS_RECT_ID)
	carmen_warn("\tOfflimits %d: rect: %d %d %d %d\n", i, offlimits_list[i].x1,
		    offlimits_list[i].y1, offlimits_list[i].x2, offlimits_list[i].y2);
    }
  }
  else
    printf("OFFLIMITS     : no\n");
  chunk_size = carmen_map_chunk_exists(filename, CARMEN_MAP_PLACES_CHUNK);
  if(chunk_size > 0)
    printf("PLACES        : yes (%d bytes)\n", chunk_size);
  else
    printf("PLACES        : no\n");
  chunk_size = carmen_map_chunk_exists(filename, CARMEN_MAP_EXPECTED_CHUNK);
  if(chunk_size > 0)
    printf("EXPECTED      : yes (%d bytes)\n", chunk_size);
  else
    printf("EXPECTED      : no\n");
  chunk_size = carmen_map_chunk_exists(filename, CARMEN_MAP_LASERSCANS_CHUNK);
  if(chunk_size > 0)
    printf("LASERSCANS    : yes (%d bytes)\n", chunk_size);
  else
    printf("LASERSCANS    : no\n");

  if(carmen_map_chunk_exists(filename, CARMEN_MAP_PLACES_CHUNK)) {
    printf("\nMap contains the following places:\n");
    printf("----------------------------------\n");
    carmen_map_read_places_chunk(filename, &place_list);
    for(i = 0; i < place_list.num_places; i++)
      if(place_list.places[i].type == CARMEN_NAMED_POSITION_TYPE)
	printf("%-20s (%.1f m, %.1f m)\n", place_list.places[i].name, 
	       place_list.places[i].x, place_list.places[i].y);
      else if(place_list.places[i].type == CARMEN_NAMED_POSE_TYPE)
	printf("%-20s (%.1f m, %.1f m, %.1f deg)\n", 
	       place_list.places[i].name, place_list.places[i].x, 
	       place_list.places[i].y, 
	       carmen_radians_to_degrees(place_list.places[i].theta));
      else if(place_list.places[i].type == CARMEN_LOCALIZATION_INIT_TYPE)
	printf("%-20s (%.1f m, %.1f m, %.1f deg) std = (%.2f m, %.2f m, "
	       "%.2f deg)\n", place_list.places[i].name, 
	       place_list.places[i].x, place_list.places[i].y, 
	       carmen_radians_to_degrees(place_list.places[i].theta), 
	       place_list.places[i].x_std, place_list.places[i].y_std,
	       carmen_radians_to_degrees(place_list.places[i].theta_std));
  }

  chunk_size = carmen_map_chunk_exists(filename, 
				       CARMEN_MAP_GLOBAL_OFFSET_CHUNK);
  if (chunk_size > 0) {
    carmen_map_read_global_offset_chunk(filename, &offset);
    printf("OFFSET        : %10.3f %10.3f %10.3f rads\n", offset.x, 
	   offset.y, offset.theta);
  } else
    printf("OFFSET        : no\n");
}

int main(int argc, char **argv)
{
  char *action;

  if (argc < 2) 
    main_usage(argv[0]);
  
  action = argv[1];
  
  if (carmen_strcasecmp(action, "-h") == 0 || 
      carmen_strcasecmp(action, "--help") == 0) 
    main_usage(argv[0]);
  if (carmen_strcasecmp(action, "help") == 0) {
    help(argc, argv);
    return 0;
  }

  if (carmen_strcasecmp(action, "toppm") == 0)
    toppm(argc, argv);
#ifndef NO_GRAPHICS
  else if (carmen_strcasecmp(action, "tomap") == 0)
    tomap(argc, argv);
#endif
  else if (carmen_strcasecmp(action, "rotate") == 0)
    rotate(argc, argv);
  else if (carmen_strcasecmp(action, "minimize") == 0)
    minimize(argc, argv);
  else if (carmen_strcasecmp(action, "add_place") == 0)
    add_place(argc, argv);
  else if (carmen_strcasecmp(action, "add_offset") == 0)
    add_offset(argc, argv);
  else if (carmen_strcasecmp(action, "strip") == 0)
    strip(argc, argv);
  else if (carmen_strcasecmp(action, "info") == 0)
    info(argc, argv);
  else {
    carmen_warn("\nUnrecognized action %s\n", argv[1]);
    main_usage(argv[0]);
  }

  return 0;
}
