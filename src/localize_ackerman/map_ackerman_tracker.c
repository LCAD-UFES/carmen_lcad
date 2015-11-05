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

#include <carmen/carmen_graphics.h>
#include "localize_ackerman_core.h"

localize_map_t map;
carmen_map_t mod_map;

int received_sensor = 0;
carmen_localize_ackerman_sensor_message sensor;
float expected_laser[MAX_SCANNS_PER_SCANN];

double front_laser_offset, occupied_prob, lmap_std, global_lmap_std;

GtkWidget *drawing_area;
GdkGC *gc = NULL;
unsigned char *map_memory;
GdkPixmap *map_pixmap = NULL, *back_pixmap = NULL;
GdkColor spectrum[256];

#define PRIOR       0
#define EMPTY_DIFF -2
#define FILL_DIFF   2

void draw_line(int x_1, int y_1, int x_2, int y_2)
{
  int dx = x_2 - x_1;
  int dy = y_2 - y_1;
  int x = x_1, y = y_1;
  int eps = 0;

  if(dx > 0) {
    if(dy >= 0) {
      if(dx > dy) {
	/* first octant */
	for(x = x_1; x <= x_2; x++) {
	  gdk_draw_point(back_pixmap, gc, x, y);
	  eps += dy;
	  if((eps << 1) >= dx) {
	    y++;
	    eps -= dx;
	  }
	}
      }
      else {
	/* second octant */
	for(y = y_1; y <= y_2; y++) {
	  gdk_draw_point(back_pixmap, gc, x, y);
	  eps += dx;
	  if((eps << 1) >= dy) {
	    x++;
	    eps -= dy;
	  }
	}
      }
    }
    else {
      if(dx > -dy) {
	/* eigth octant */
	for(x = x_1; x <= x_2; x++) {
	  gdk_draw_point(back_pixmap, gc, x, y);
	  eps += dy;
	  if((eps << 1) <= -dx) {
	    y--;
	    eps += dx;
	  }
	}
      }
      else {
	/* seventh octant */
	for(y = y_1; y >= y_2; y--) {
	  gdk_draw_point(back_pixmap, gc, x, y);
	  eps += dx;
	  if((eps << 1) >= -dy) {
	    x++;
	    eps += dy;
	  }
	}
      }      
    }
  }
  else {
    if(dy >= 0) {
      if(-dx > dy) {
	/* forth octant */
	for(x = x_1; x >= x_2; x--) {
	  gdk_draw_point(back_pixmap, gc, x, y);
	  eps += dy;
	  if((eps << 1) >= -dx) {
	    y++;
	    eps += dx;
	  }
	}
      }
      else {
	/* third octant */
	for(y = y_1; y <= y_2; y++) {
	  gdk_draw_point(back_pixmap, gc, x, y);
	  eps += dx;
	  if((eps << 1) <= -dy) {
	    x--;
	    eps += dy;
	  }
	}
      }
    }
    else {
      if(-dx > -dy) {
	/* fifth octant */
	for(x = x_1; x >= x_2; x--) {
	  gdk_draw_point(back_pixmap, gc, x, y);
	  eps -= dy;
	  if((eps << 1) > -dx) {
	    y--;
	    eps += dx;
	  }
	}
      }
      else {
	/* sixth octant */
	for(y = y_1; y >= y_2; y--) {
	  gdk_draw_point(back_pixmap, gc, x, y);
	  eps -= dx;
	  if((eps << 1) > -dy) {
	    x--;
	    eps += dy;
	  }
	}
      }
    }
  }
}

carmen_inline void ray_trace(carmen_map_p map, int x_1, int y_1, int x_2, int y_2,
		      int *hit_x, int *hit_y, float occupied_limit)
{
  int dx = x_2 - x_1;
  int dy = y_2 - y_1;
  int x = x_1, y = y_1;
  int eps = 0;

  if(dx > 0) {
    if(dy >= 0) {
      if(dx > dy) {
	/* first octant */
	for(x = x_1; x <= x_2; x++) {
	  if(map->map[x][y] == -1 ||
	     map->map[x][y] > occupied_limit) {
	    *hit_x = x;
	    *hit_y = y;
	    return;
	  }
	  eps += dy;
	  if((eps << 1) >= dx) {
	    y++;
	    eps -= dx;
	  }
	}
      }
      else {
	/* second octant */
	for(y = y_1; y <= y_2; y++) {
	  if(map->map[x][y] == -1 ||
	     map->map[x][y] > occupied_limit) {
	    *hit_x = x;
	    *hit_y = y;
	    return;
	  }
	  eps += dx;
	  if((eps << 1) >= dy) {
	    x++;
	    eps -= dy;
	  }
	}
      }
    }
    else {
      if(dx > -dy) {
	/* eigth octant */
	for(x = x_1; x <= x_2; x++) {
	  if(map->map[x][y] == -1 ||
	     map->map[x][y] > occupied_limit) {
	    *hit_x = x;
	    *hit_y = y;
	    return;
	  }
	  eps += dy;
	  if((eps << 1) <= -dx) {
	    y--;
	    eps += dx;
	  }
	}
      }
      else {
	/* seventh octant */
	for(y = y_1; y >= y_2; y--) {
	  if(map->map[x][y] == -1 ||
	     map->map[x][y] > occupied_limit) {
	    *hit_x = x;
	    *hit_y = y;
	    return;
	  }
	  eps += dx;
	  if((eps << 1) >= -dy) {
	    x++;
	    eps += dy;
	  }
	}
      }      
    }
  }
  else {
    if(dy >= 0) {
      if(-dx > dy) {
	/* forth octant */
	for(x = x_1; x >= x_2; x--) {
	  if(map->map[x][y] == -1 ||
	     map->map[x][y] > occupied_limit) {
	    *hit_x = x;
	    *hit_y = y;
	    return;
	  }
	  eps += dy;
	  if((eps << 1) >= -dx) {
	    y++;
	    eps += dx;
	  }
	}
      }
      else {
	/* third octant */
	for(y = y_1; y <= y_2; y++) {
	  if(map->map[x][y] == -1 ||
	     map->map[x][y] > occupied_limit) {
	    *hit_x = x;
	    *hit_y = y;
	    return;
	  }
	  eps += dx;
	  if((eps << 1) <= -dy) {
	    x--;
	    eps += dy;
	  }
	}
      }
    }
    else {
      if(-dx > -dy) {
	/* fifth octant */
	for(x = x_1; x >= x_2; x--) {
	  if(map->map[x][y] == -1 ||
	     map->map[x][y] > occupied_limit) {
	    *hit_x = x;
	    *hit_y = y;
	    return;
	  }
	  eps -= dy;
	  if((eps << 1) > -dx) {
	    y--;
	    eps += dx;
	  }
	}
      }
      else {
	/* sixth octant */
	for(y = y_1; y >= y_2; y--) {
	  if(map->map[x][y] == -1 ||
	     map->map[x][y] > occupied_limit) {
	    *hit_x = x;
	    *hit_y = y;
	    return;
	  }
	  eps -= dx;
	  if((eps << 1) > -dy) {
	    x--;
	    eps += dy;
	  }
	}
      }
    }
  }
}

#define observe_empty(current_x, current_y, dmap) \
{ \
  if(dmap->map[current_x][current_y] == -1e6) \
    dmap->map[current_x][current_y] = PRIOR; \
  else { \
    dmap->map[current_x][current_y] += EMPTY_DIFF; \
    if(dmap->map[current_x][current_y] < -100.0) \
      dmap->map[current_x][current_y] = -100.0; \
  } \
}

#define observe_filled(current_x, current_y, dmap) \
{ \
  if(dmap->map[current_x][current_y] == -1e6) \
    dmap->map[current_x][current_y] = PRIOR; \
  else { \
    dmap->map[current_x][current_y] += FILL_DIFF; \
    if(dmap->map[current_x][current_y] > 100.0) \
      dmap->map[current_x][current_y] = 100.0; \
  } \
}

carmen_inline void ray_empty(carmen_map_p new_map, int x_1, int y_1, int x_2, int y_2)
{
  int dx = x_2 - x_1;
  int dy = y_2 - y_1;
  int x = x_1, y = y_1;
  int eps = 0;

  if(dx > 0) {
    if(dy >= 0) {
      if(dx > dy) {
	/* first octant */
	for(x = x_1; x <= x_2; x++) {
	  observe_empty(x, y, new_map);
	  eps += dy;
	  if((eps << 1) >= dx) {
	    y++;
	    eps -= dx;
	  }
	}
      }
      else {
	/* second octant */
	for(y = y_1; y <= y_2; y++) {
	  observe_empty(x, y, new_map);
	  eps += dx;
	  if((eps << 1) >= dy) {
	    x++;
	    eps -= dy;
	  }
	}
      }
    }
    else {
      if(dx > -dy) {
	/* eigth octant */
	for(x = x_1; x <= x_2; x++) {
	  observe_empty(x, y, new_map);
	  eps += dy;
	  if((eps << 1) <= -dx) {
	    y--;
	    eps += dx;
	  }
	}
      }
      else {
	/* seventh octant */
	for(y = y_1; y >= y_2; y--) {
	  observe_empty(x, y, new_map);
	  eps += dx;
	  if((eps << 1) >= -dy) {
	    x++;
	    eps += dy;
	  }
	}
      }      
    }
  }
  else {
    if(dy >= 0) {
      if(-dx > dy) {
	/* forth octant */
	for(x = x_1; x >= x_2; x--) {
	  observe_empty(x, y, new_map);
	  eps += dy;
	  if((eps << 1) >= -dx) {
	    y++;
	    eps += dx;
	  }
	}
      }
      else {
	/* third octant */
	for(y = y_1; y <= y_2; y++) {
	  observe_empty(x, y, new_map);
	  eps += dx;
	  if((eps << 1) <= -dy) {
	    x--;
	    eps += dy;
	  }
	}
      }
    }
    else {
      if(-dx > -dy) {
	/* fifth octant */
	for(x = x_1; x >= x_2; x--) {
	  observe_empty(x, y, new_map);
	  eps -= dy;
	  if((eps << 1) > -dx) {
	    y--;
	    eps += dx;
	  }
	}
      }
      else {
	/* sixth octant */
	for(y = y_1; y >= y_2; y--) {
	  observe_empty(x, y, new_map);
	  eps -= dx;
	  if((eps << 1) > -dy) {
	    x--;
	    eps += dy;
	  }
	}
      }
    }
  }
}

carmen_inline void ray_filled(carmen_map_p new_map, int x_1, int y_1, int x_2, int y_2)
{
  int dx = x_2 - x_1;
  int dy = y_2 - y_1;
  int x = x_1, y = y_1;
  int eps = 0;

  if(dx > 0) {
    if(dy >= 0) {
      if(dx > dy) {
	/* first octant */
	for(x = x_1; x <= x_2; x++) {
	  observe_filled(x, y, new_map);
	  eps += dy;
	  if((eps << 1) >= dx) {
	    y++;
	    eps -= dx;
	  }
	}
      }
      else {
	/* second octant */
	for(y = y_1; y <= y_2; y++) {
	  observe_filled(x, y, new_map);
	  eps += dx;
	  if((eps << 1) >= dy) {
	    x++;
	    eps -= dy;
	  }
	}
      }
    }
    else {
      if(dx > -dy) {
	/* eigth octant */
	for(x = x_1; x <= x_2; x++) {
	  observe_filled(x, y, new_map);
	  eps += dy;
	  if((eps << 1) <= -dx) {
	    y--;
	    eps += dx;
	  }
	}
      }
      else {
	/* seventh octant */
	for(y = y_1; y >= y_2; y--) {
	  observe_filled(x, y, new_map);
	  eps += dx;
	  if((eps << 1) >= -dy) {
	    x++;
	    eps += dy;
	  }
	}
      }      
    }
  }
  else {
    if(dy >= 0) {
      if(-dx > dy) {
	/* forth octant */
	for(x = x_1; x >= x_2; x--) {
	  observe_filled(x, y, new_map);
	  eps += dy;
	  if((eps << 1) >= -dx) {
	    y++;
	    eps += dx;
	  }
	}
      }
      else {
	/* third octant */
	for(y = y_1; y <= y_2; y++) {
	  observe_filled(x, y, new_map);
	  eps += dx;
	  if((eps << 1) <= -dy) {
	    x--;
	    eps += dy;
	  }
	}
      }
    }
    else {
      if(-dx > -dy) {
	/* fifth octant */
	for(x = x_1; x >= x_2; x--) {
	  observe_filled(x, y, new_map);
	  eps -= dy;
	  if((eps << 1) > -dx) {
	    y--;
	    eps += dx;
	  }
	}
      }
      else {
	/* sixth octant */
	for(y = y_1; y >= y_2; y--) {
	  observe_filled(x, y, new_map);
	  eps -= dx;
	  if((eps << 1) > -dy) {
	    x--;
	    eps += dy;
	  }
	}
      }
    }
  }
}

void compute_expected_scan(carmen_map_p map, double x, double y, double theta,
			   float *range, int num_readings,
			   double angular_resolution,
			   double first_beam_angle)
{
  float angle, angle_diff;
  int hit_x, hit_y;
  int i;

  angle = theta + first_beam_angle;
  for(i = 0; i < num_readings; i++) {
    ray_trace(map, 
	      (x - map.config.x_origin) / map->config.resolution,
	      (y - map.config.y_origin) / map->config.resolution,
	      ((x - map.config.x_origin) + 500.0 * cos(angle)) / map->config.resolution,
	      ((y - map.config.y_origin) + 500.0 * sin(angle)) / map->config.resolution,
	      &hit_x, &hit_y, 0.25);
    range[i] = hypot((hit_x * map->config.resolution + map.config.x_origin) - x,
		     (hit_y * map->config.resolution + map.config.y_origin) - y);
    angle += angular_resolution;
  }
}

static void redraw(void)
{
  int x, y, c;
  //  GdkColor color;
  int height = drawing_area->allocation.height;
  int i, width = drawing_area->allocation.width;
  float angle, p;
  int disagree;
  
  if(back_pixmap == NULL)
    back_pixmap = gdk_pixmap_new(drawing_area->window, width, height, -1);
  if(back_pixmap == NULL)
    return;

  gdk_draw_pixmap(back_pixmap, gc, map_pixmap, 0, 0, 0, 0, 
		  map.config.x_size, map.config.y_size);

  for(x = 0; x < map.config.x_size; x++)
    for(y = 0; y < map.config.y_size; y++) 
      if(mod_map.map[x][y] != -1e6) {
	disagree = map.carmen_map.map[x][y] == -1 ||
	  (map.carmen_map.map[x][y] > 0.5 && mod_map.map[x][y] < 0) ||
	  (map.carmen_map.map[x][y] < 0.5 && mod_map.map[x][y] > 0);
	if(disagree) {
	  p = mod_map.map[x][y];
	  c = (p + 100) / 200.0 * 255;
	  if(c > 255)
	    c = 255;
	  else if (c < 0)
	    c = 0;
	  gdk_gc_set_foreground(gc, &spectrum[c]);
	  gdk_draw_point(back_pixmap, gc, x, map.config.y_size - 1 - y);
	}
      }

  /* draw the mean scan */
  for(i = 0; i < sensor.num_readings; i++) {
    if(sensor.range[i] - expected_laser[i] > 0.2)
      gdk_gc_set_foreground(gc, &carmen_red);
    else
      gdk_gc_set_foreground(gc, &carmen_blue);
      
    angle = sensor.pose.theta + sensor.config.start_angle + 
      i * sensor.config.angular_resolution;
    x = (sensor.pose.x + sensor.range[i] * cos(angle)) / 
      map.config.resolution;
    y = height - 1 - (sensor.pose.y + sensor.range[i] * sin(angle)) / 
      map.config.resolution;
    gdk_draw_point(back_pixmap, gc, x, y);
  }

  /* draw the robot */
  x = (sensor.pose.x - front_laser_offset * cos(sensor.pose.theta)) /
    map.config.resolution;
  y = height - (sensor.pose.y - front_laser_offset * 
		sin(sensor.pose.theta)) / map.config.resolution;
  gdk_gc_set_foreground(gc, &carmen_red);
  gdk_draw_arc(back_pixmap, gc, TRUE, x - 3, y - 3, 6, 6, 0, 360 * 64);
  gdk_gc_set_foreground(gc, &carmen_black);
  gdk_draw_arc(back_pixmap, gc, FALSE, x - 3, y - 3, 6, 6, 0, 360 * 64);
  gdk_draw_line(back_pixmap, gc, x, y, 
		x + 5 * cos(sensor.pose.theta),
		y - 5 * sin(sensor.pose.theta));

  gdk_draw_pixmap(drawing_area->window, gc, back_pixmap, 0, 0, 0, 0,
		  width, height);
}

gint expose_event(GtkWidget *widget __attribute__ ((unused)), 
		  GdkEventExpose *event __attribute__ ((unused))) 
{
  redraw();
  return 1;
}

void incorporate_laser_difference(float *actual, float *expected, 
				  int num_readings, 
				  double angular_resolution,
				  double first_beam_angle,
				  float x, float y, float theta, 
				  carmen_map_p diff_map)
{
  int i, x1, y1, x2, y2, x3, y3;
  float angle, cangle, sangle;
  float wall_thickness = 0.2;

  for(i = 0; i < num_readings; i++)
    if(expected[i] < 5.0 && actual[i] < 20.0 && 
       actual[i] - expected[i] > 0.5) {
      angle = theta + first_beam_angle + i * angular_resolution; 
      //i / ((float)num_readings - 1.0) * M_PI;
      cangle = cos(angle); sangle = sin(angle);
      
      x1 = (x + expected[i] * cangle) / diff_map->config.resolution;
      y1 = (y + expected[i] * sangle) / diff_map->config.resolution;

      x2 = (x + actual[i] * cangle) / diff_map->config.resolution;
      y2 = (y + actual[i] * sangle) / diff_map->config.resolution;

      x3 = (x + (actual[i] + wall_thickness) * cangle) / 
	diff_map->config.resolution;
      y3 = (y + (actual[i] + wall_thickness) * sangle) / 
	diff_map->config.resolution;

      ray_empty(diff_map, x1, y1, x2, y2);
      ray_filled(diff_map, x2, y2, x3, y3);
      //      observe_filled(x2, y2, diff_map);
    }
}

void localize_sensor_handler(void)
{
  carmen_traj_point_t pose;
  //  int i;
  static int count = 0;

  received_sensor = 1;
  fprintf(stderr, "S");

  pose.x = sensor.pose.x;
  pose.y = sensor.pose.y;
  pose.theta = sensor.pose.theta;

  compute_expected_scan(&map.carmen_map, pose.x, pose.y, pose.theta,
			expected_laser, sensor.num_readings);

  incorporate_laser_difference(sensor.range, expected_laser, 
			       sensor.num_readings, pose.x, pose.y, 
			       pose.theta, &mod_map);

  if(count % 5 == 0)
    redraw();
  count++;
}

gint updateIPC(gpointer *data __attribute__ ((unused))) 
{
  dgc_ipc_sleep(0.01);
  carmen_graphics_update_ipc_callbacks((GdkInputFunction)updateIPC);
  return 1;
}

void graphics_init(int argc, char **argv)
{
  GtkWidget *main_window;
  int i;

  gtk_init(&argc, &argv);
  gdk_rgb_init();

  main_window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title(GTK_WINDOW(main_window), "Map Tracker");
  
  drawing_area = gtk_drawing_area_new();
  gtk_widget_set_usize(drawing_area, map.config.x_size, map.config.y_size);

  gtk_container_add(GTK_CONTAINER(main_window), drawing_area);
  gtk_signal_connect(GTK_OBJECT(drawing_area), "expose_event",
                     (GtkSignalFunc)expose_event, NULL);
  gtk_widget_add_events(drawing_area, GDK_EXPOSURE_MASK);
  
  carmen_graphics_update_ipc_callbacks((GdkInputFunction)updateIPC);
  gtk_widget_show(drawing_area);
  gtk_widget_show(main_window);

  map_memory = carmen_graphics_convert_to_image(&map.carmen_map, 0);
  map_pixmap = carmen_graphics_generate_pixmap(drawing_area, map_memory,
					       &map.config, 1.0);
  gc = gdk_gc_new(drawing_area->window);

  carmen_graphics_setup_colors();
  
  for(i = 0; i < 255; i++)
    spectrum[i] = carmen_graphics_add_color_rgb(255, 255 - i, 0);
}

void read_parameters(int argc, char **argv)
{
  occupied_prob = 0.5;
  lmap_std = 0.3;
  global_lmap_std = 0.6;

  carmen_param_t param_list[] = {
    {"robot", "frontlaser_offset", CARMEN_PARAM_DOUBLE,
     &front_laser_offset, 0, NULL},
    {"localize", "occupied_prob", CARMEN_PARAM_DOUBLE, 
     &occupied_prob, 0, NULL},
    {"localize", "lmap_std", CARMEN_PARAM_DOUBLE, &lmap_std, 0, NULL},
    {"localize", "global_lmap_std", CARMEN_PARAM_DOUBLE, 
     &global_lmap_std, 0, NULL},
  };
  carmen_param_install_params(argc, argv, param_list, 
                              sizeof(param_list) / sizeof(param_list[0]));
}

void create_localize_ackerman_map(localize_map_t *map)
{
  carmen_map_t raw_map;
  int i, j;

  /* Request map, and wait for it to arrive */
  carmen_warn("Requesting a map from the map server... ");
  if(carmen_map_get_gridmap(&raw_map) < 0)
    carmen_die("Could not get a map from the map server.\n");
  else
    carmen_warn("done.\n");

  /* create a localize map */
  carmen_warn("Creating likelihood maps... ");
  carmen_to_localize_ackerman_map(&raw_map, map, occupied_prob,
			 lmap_std, global_lmap_std);
  carmen_warn("done.\n");

  /* create an evidence grid */
  mod_map.config = raw_map.config;
  mod_map.complete_map = (float *)calloc(mod_map.config.x_size *
					 mod_map.config.y_size, sizeof(float));
  carmen_test_alloc(mod_map.complete_map);
  mod_map.map = (float **)calloc(mod_map.config.x_size, sizeof(float *));
  carmen_test_alloc(mod_map.map);
  for(i = 0; i < mod_map.config.x_size; i++) {
    mod_map.map[i] = mod_map.complete_map + i * mod_map.config.y_size;
    for(j = 0; j < mod_map.config.y_size; j++)
      mod_map.map[i][j] = -1e6;
  }
}

int main(int argc, char **argv)
{
  /* initialize carmen */
  carmen_randomize();
  carmen_ipc_initialize(argc, argv);
  carmen_param_check_version(argv[0]);

  /* get parameters and create a localize likelihood map */
  read_parameters(argc, argv);
  create_localize_ackerman_map(&map);

  carmen_localize_ackerman_subscribe_sensor_message(&sensor, (carmen_handler_t)
					   localize_sensor_handler,
					   CARMEN_SUBSCRIBE_LATEST);

  graphics_init(argc, argv);
  gtk_main();
  return 0;
}
