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

/***********************************
 * map_editor_menus implements the *
 * menu items in the map_editor    *
 ***********************************/
#ifndef CARMEN_MAP_EDITOR_MENUS_H
#define CARMEN_MAP_EDITOR_MENUS_H

#ifdef __cplusplus
extern "C" {
#endif

/* file */

void new_map_menu(GtkAction *action, gpointer user_data __attribute__ ((unused)));
void open_map_menu(GtkAction *action, gpointer user_data __attribute__ ((unused)));
void save_map_menu(GtkAction *action, gpointer user_data __attribute__ ((unused)));
void save_map_as_menu(GtkAction *action, gpointer user_data __attribute__ ((unused)));
void reload_map_menu(GtkAction *action, gpointer user_data __attribute__ ((unused)));
void import_from_bmp_menu(GtkAction *action, gpointer user_data __attribute__ ((unused)));
void quit_menu(GtkAction *action, gpointer user_data __attribute__ ((unused)));
void undo_menu(GtkAction *action, gpointer user_data __attribute__ ((unused)));
void add_placename(GtkAction *action, gpointer user_data __attribute__ ((unused)));
void delete_placename(GtkAction *action, gpointer user_data __attribute__ ((unused)));
void add_door(GtkAction *action, gpointer user_data __attribute__ ((unused)));
void toggle_view(GtkAction *action, gpointer user_data __attribute__ ((unused)));
void help_menu(GtkAction *action, gpointer user_data __attribute__ ((unused)));

int map_open(char *filename, int have_graphics);
void do_delete_placename(int i);
void start_add_placename(double x, double y);
void start_add_door(double x, double y);
void finish_add_door(double x, double y);

#ifdef __cplusplus
}
#endif

#endif
