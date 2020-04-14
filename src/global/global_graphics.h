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

#ifndef GLOBAL_GRAPHICS_H
#define GLOBAL_GRAPHICS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <gtk/gtk.h>
#include <carmen/global.h>
#include <carmen/map.h>
#include <carmen/road_mapper.h>

#define CARMEN_GRAPHICS_INVERT          	1
#define CARMEN_GRAPHICS_RESCALE         	2
#define CARMEN_GRAPHICS_ROTATE          	4
#define CARMEN_GRAPHICS_BLACK_AND_WHITE 	8
#define CARMEN_GRAPHICS_ENHANCE_CONTRAST	16
#define CARMEN_GRAPHICS_GRAYSCALE			32
#define CARMEN_GRAPHICS_REMOVE_MINUS_ONE	64
#define CARMEN_GRAPHICS_LOG_ODDS			128
#define CARMEN_GRAPHICS_ROAD_CONTRAST		256

  typedef struct {
    int fd;
    int callback_id;
    int ok;
  } carmen_graphics_callback;

  extern GdkColor carmen_red, carmen_blue, carmen_white, carmen_yellow, 
    carmen_green, carmen_light_blue, carmen_black, carmen_orange, 
    carmen_grey, carmen_light_grey, carmen_purple, carmen_light_green;

  enum
  {
	  CARMEN_RED = 0,
	  CARMEN_BLUE = 1,
	  CARMEN_WHITE = 2,
	  CARMEN_YELLOW = 3,
	  CARMEN_GREEN = 4,
	  CARMEN_LIGHT_BLUE = 5,
	  CARMEN_BLACK = 6,
	  CARMEN_ORANGE = 7,
	  CARMEN_GREY = 8,
	  CARMEN_LIGHT_GREY = 9,
	  CARMEN_PURPLE = 10,
	  CARMEN_LIGHT_GREEN = 11,
  };

  extern GdkColor carmen_colors[12];


  void carmen_graphics_update_ipc_callbacks(GdkInputFunction callback_Func);

  GdkColor carmen_graphics_add_color(char *name);
  void carmen_graphics_setup_colors(void);
  GdkColor carmen_graphics_add_color_rgb(int r, int g, int b);

  void carmen_graphics_write_pixmap_as_png(GdkPixmap *pixmap, 
					   char *user_filename,
					   int x, int y, int w, int h);
  void carmen_graphics_write_data_as_png(unsigned char *data, 
					 char *user_filename, int w, int h);

  void carmen_graphics_write_pixmap_as_png(GdkPixmap *pixmap, 
					   char *user_filename, 
					   int x, int y, int w, int h);
  void carmen_graphics_write_data_as_png(unsigned char *data, 
					 char *user_filename, int w, int h);
  int carmen_map_image_to_map_color_unknown(unsigned char r, unsigned char g, 
					    unsigned char b);

  #ifndef COMPILE_WITHOUT_MAP_SUPPORT

  #ifndef NO_JPEG
  void carmen_graphics_write_map_as_jpeg(char *filename, carmen_map_p map, 
					 int flags);
  #endif

  unsigned char *carmen_graphics_convert_to_image(carmen_map_p map, int flags);

  carmen_map_p carmen_pixbuf_to_map(GdkPixbuf* pixbuf, double resolution);
  carmen_map_p carmen_map_imagefile_to_map(char *filename, double resolution);
  GdkPixmap *carmen_graphics_generate_pixmap(GtkWidget* drawing_area, 
					     unsigned char* data,
					     carmen_map_config_p config, 
					     double zoom);
  
  #endif

#ifdef __cplusplus
}
#endif

#endif
