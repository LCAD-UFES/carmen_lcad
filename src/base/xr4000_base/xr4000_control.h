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

#ifndef XR4000_CONTROL_H
#define XR4000_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

void carmen_xr4000_usage(char *progname, char *fmt, ...);

void carmen_xr4000_start(void);

void carmen_xr4000_set_velocity(double xv, double yv, double rv);

void carmen_xr4000_run(void);

void carmen_xr4000_shutdown(int sig);

void carmen_xr4000_emergency_crash(int sig __attribute ((unused)));

#ifdef __cplusplus
}
#endif

#endif
