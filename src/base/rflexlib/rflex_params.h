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

#ifndef CARMEN_RFLEX_PARAMS_H
#define CARMEN_RFLEX_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

char *carmen_rflex_models[] = {"atrv", "atrv2", "atrvjr", "atrvmini", "b14r", 
			       "b21r+", "magellanpro", "magellan", "pioneer", 
			       "transit", 0};

double carmen_rflex_params[][2] =
  {
    // distance_conversion angle_conversion
    // ATRV params
    {146685.14, 72640.107},
    // ATRV2
    {161353.50, 79904.12},
    // ATRVJR
    {87462.73, 36277.67},
    // ATRVMINI
    {139654.7,  49784.5},
    // B14R
    {84157.1, 75055.859},
    // B21R
    {101230.0, 35343.888},
    // MAGELLAN-PRO
    {31271.8, 4752.6},
    // MAGELLAN
    {44509.7, 5818.0204},
    // PIONEER
    {0.01, 0.017453293},
    // TRANSIT
    {31502.000, 6641.313},
    {0, 0}};

#ifdef __cplusplus
}
#endif

#endif 
