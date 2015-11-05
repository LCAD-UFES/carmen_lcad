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

/*************************************
 * program to test the map server.   *
 * loads a map and displays it       *
 *************************************/

#include <carmen/global_graphics.h>
#include <carmen/ipc_wrapper.h>
#include <carmen/param_interface.h>
#include <signal.h>
#include "map_graphics.h"
#include "map_interface.h"

carmen_map_t map;

/* handler for C^c */
void 
shutdown_module(int x)
{
  if(x == SIGINT) {
    carmen_ipc_disconnect();
    exit(1);
  }
}

void post_func(GtkMapViewer *map_view) {
  carmen_world_point_t world_point;

  world_point.pose.x = 3010;
  world_point.pose.y = 700;
  world_point.map = &map;

  carmen_map_graphics_draw_arc(map_view, &carmen_red, TRUE, &world_point, 5, 0,
			       360*64);
  carmen_map_graphics_draw_arc(map_view, &carmen_black, FALSE, &world_point, 
			       5, 0, 360*64);
}

/* main
   read arg(s)
   initiallize ipc
   subscribe to map messages,
   request a map message
   wait */
int 
main(int argc, char** argv)
{
  GtkWidget *window;
  GtkMapViewer *map_view;
  carmen_offlimits_p offlimits;
  int list_length;

  gtk_init (&argc, &argv);
  carmen_graphics_setup_colors();

  carmen_ipc_initialize(argc, argv);
  carmen_param_check_version(argv[0]);

  signal(SIGINT, shutdown_module);

//  carmen_map_get_gridmap(&map); // @@@ Alberto: esta funcao nao deve mais ser usada. Este modulo precisa ser revisto em funcao disso. Mas agora sao apenas publicados pelo map_server.
  carmen_map_get_offlimits(&offlimits, &list_length);
  carmen_map_apply_offlimits_chunk_to_map(offlimits, list_length, &map);

  window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  
  map_view = carmen_map_graphics_new_viewer(400, 400, 40.0);
  gtk_container_add (GTK_CONTAINER (window), map_view->map_box);
  gtk_widget_show_all(window);

  carmen_map_graphics_add_drawing_func
    (map_view, (carmen_graphics_mapview_drawing_func_t)post_func);
  carmen_map_graphics_add_map(map_view, &map, 0);
  gtk_main();

  return 0;
}
