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

#ifndef CARMEN_MAP_EDITOR_H
#define CARMEN_MAP_EDITOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <gtk/gtk.h>

extern char map_filename[255];

typedef enum {CARMEN_MAP_BRUSH, CARMEN_MAP_RECTANGLE, CARMEN_MAP_CROP, 
	      CARMEN_MAP_LINE, CARMEN_MAP_FILL,
	      CARMEN_MAP_FUZZY_FILL, CARMEN_MAP_EYEDROPPER,
	      CARMEN_MAP_ZOOM, CARMEN_MAP_MOVER} carmen_utensil_t;

extern carmen_map_p map;

extern int num_offlimits_segments;
extern int offlimits_capacity;
extern carmen_offlimits_p offlimits_array;
extern carmen_map_placelist_p place_list;
extern int places_capacity;
extern int drawing_offlimits;
extern int show_offlimits;
extern int show_place_names;
extern int deleting_placename;
extern int adding_placename;
extern int adding_door;
extern int current_door_num;

extern int screen_width;
extern int screen_height;
extern int xstart, ystart;
extern int xend, yend;
extern double mult;

extern unsigned char * image_data;
extern GtkWidget *window;
extern GdkPixmap *map_pixmap;
extern GdkPixmap *tmp_pixmap;
extern GtkWidget *drawing_area;
extern GtkWidget *scrolled_window;
extern GdkGC     *drawing_gc;
extern GdkColor   color;
extern GdkColor   yellow, purple, blue, red;
extern GtkWidget *save_file;
extern GtkWidget *entry_x_size;
extern GtkWidget *entry_y_size;
extern GtkWidget *entry_resolution;
extern GtkWidget *cur_label_label;
extern GtkWidget *tool_label;
extern GtkWidget *coords_label;
extern GtkWidget *ink_label;
extern GtkWidget *fuzzyness_label;
extern GtkWidget *brush_label;
extern GtkWidget *line_label;
extern GtkWidget *fill_label;
extern GtkWidget *hrule, *vrule;
extern GtkWidget *ink_scale;
extern GtkWidget *fuzzy_scale;
extern GtkWidget *hscroll;
extern GtkWidget *vscroll;
extern GdkCursor *ccross, *cfill, *cpoint, *cselect;

extern double     *backup;

extern int modified;
extern int saveas_binary;
extern int no_quit;
extern int get_name;
extern int save_cancel;

extern carmen_utensil_t utensil;
extern double ink;
extern double brush_size;
extern double line_size;
extern int filled;
extern int move_enable;

extern double fuzzyness;

#ifdef __cplusplus
}
#endif

#endif
