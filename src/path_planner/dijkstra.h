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

#ifndef DIJKSTRA_H_
#define DIJKSTRA_H_

typedef struct {
    int x;
    int y;
    double score;
}carmen_dijkstra_node_t, *carmen_dijkstra_node_p;

void alloc_dijkstra_map();
void add_point(carmen_dijkstra_node_p point);
void open_dijkstra_node(carmen_dijkstra_node_p node);
void dijkstra(carmen_ackerman_traj_point_t goal);
void init_dijkstra_map();
void dijkstra_map_to_utility();



#endif /* DIJKSTRA_H_ */
