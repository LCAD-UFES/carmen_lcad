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

#ifndef NAVIGATOR_PRECOMPUTED_COST_ACKERMAN_H
#define NAVIGATOR_PRECOMPUTED_COST_ACKERMAN_H

#define ORIENTATION_LENGHT 3
#define DIRECTION_LENGHT 2

enum NODE_STATUS {OPEN, CLOSE};

/** The data structure **/

#include "FH/fh.hpp"

typedef struct {
    double f_score;
    double g_score;
    double h_score;
    int range;
    int status;
    int astar_call_cont;
    double direction;
    FH_NODE* fh_node;
    carmen_robot_and_trailer_traj_point_t point;
    carmen_robot_and_trailer_traj_point_t prev_point;
}carmen_astar_node_t, *carmen_astar_node_p;

void carmen_precomputed_cost_ackerman_dijkstra();


#endif
