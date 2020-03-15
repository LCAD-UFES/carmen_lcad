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

#ifndef CARMEN_PIONEER_PARAMS_H
#define CARMEN_PIONEER_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

const char *carmen_pioneer_models[] = {"amigo", "p2at8", "p2at8+", "p2at", "p2ce", 
				 "p2d8", "p2d8+", "p2de", "p2df", "p2dx", 
				 "p2it", "p2pb", "p2pp", "powerbot-sh", "perfpb",
                                 "perfpb+", "pion1m", "pion1x", "pionat", "powerbot", 
				 "psos1m", "psos1x", "p3at-sh", "p3at", "p3atiw-sh",
                                 "p3atiw", "p3dx-sh", "p3dx", "peoplebot-sh",
                                 0};
  
double carmen_pioneer_params[][6] = 
  {
    // AngleConvFactor DistConvFactor VelConvFactor RangeConfFactor 
    // DiffConvFactor Vel2Divisor

    // Amigo params
    {0.001534, 0.508300, 0.615400, 1.000000, 0.011000, 20.000000},
    // p2at8
    {0.001534, 1.320000, 1.000000, 1.000000, 0.003400, 20.000000},
    // p2at8+
    {0.001534, 0.465000, 1.000000, 1.000000, 0.003400, 20.000000},
    // pt2at
    {0.001534, 1.320000, 1.000000, 0.268000, 0.003400, 20.000000},
    // p2ce
    {0.001534, 0.826000, 1.000000, 0.268000, 0.005700, 20.000000},
    // p2d8
    {0.001534, 0.485000, 1.000000, 1.000000, 0.005600, 20.000000},
    // p2d8+
    {0.001534, 0.485000, 1.000000, 1.000000, 0.005600, 20.000000},
    // p2de
    {0.001534, 0.969000, 1.000000, 0.268000, 0.005600, 20.000000},
    // p2df
    {0.001534, 0.485000, 1.000000, 0.268000, 0.006000, 20.000000},
    // p2dx
    {0.001534, 0.840000, 1.000000, 0.268000, 0.005600, 20.000000},
    // p2it
    {0.001534, 1.136000, 1.000000, 0.268000, 0.003200, 20.000000},
    // p2pb
    {0.001534, 0.424000, 1.000000, 0.268000, 0.005600, 20.000000},
    // p2pp
    {0.001534, 0.485000, 1.000000, 0.268000, 0.006000, 20.000000},
    // powerbot-sh
    {0.001534, 1.000000, 1.000000, 1.000000, 0.003730, 20.000000},
    // perfp
    {0.001534, 0.485000, 1.000000, 1.000000, 0.006000, 20.000000},
    // perfpb+
    {0.001534, 0.485000, 1.000000, 1.000000, 0.006000, 20.000000},
    // pion1m
    {0.006136, 0.050660, 2.533200, 0.173400, 0.003300, 4.000000},
    // pion1x
    {0.006136, 0.050660, 0.079790, 0.173400, 0.003333, 4.000000},
    // pionat
    {0.006136, 0.070000, 2.533200, 0.173400, 0.003333, 4.000000},
    // powerbot
    {0.001534, 0.581300, 1.000000, 1.000000, 0.003730, 20.000000},
    // psos1m
    {0.006136, 0.050660, 2.533200, 0.173400, 0.003300, 4.000000},
    // psos1x
    {0.006136, 0.050660, 0.079790, 0.173400, 0.003333, 4.000000},
    // p3at-sh
    {0.001534, 1.000000, 1.000000, 1.000000, 0.003400, 20.000000 },
    // p3at
    {0.001534, 0.465000, 1.000000, 1.000000, 0.003400, 20.000000 },
    // p3atiw-sh
    {0.001534, 1.000000, 1.000000, 1.000000, 0.003400, 20.000000 },
    // p3atiw
    {0.001534, 0.376800, 1.000000, 1.000000, 0.003400, 20.000000 },
    // p3dx-sh
    {0.001534, 1.000000, 1.000000, 1.000000, 0.005600, 2.0000000 },
    // p3dx
    {0.001534, 0.485000, 1.000000, 1.000000, 0.005600, 20.000000 },
    // peoplebot-sh
    {0.001534, 1.000000, 1.000000, 1.000000, 0.006000, 20.000000 },
  };

#ifdef __cplusplus
}
#endif

#endif 
