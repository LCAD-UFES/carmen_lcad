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

/* Standard include file */

#ifndef CARMEN_H
#define CARMEN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#ifndef va_copy
#define va_copy __va_copy
#endif

#include <unistd.h>
#include <ctype.h>
#ifdef __USE_BSD
#undef __USE_BSD
#include <string.h>
#define __USE_BSD
#else
#include <string.h>
#endif
#include <signal.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#ifdef CYGWIN
#include <sys/socket.h>
#endif
#include <errno.h>
#include <limits.h>
#include <float.h>
#ifndef MAXDOUBLE
#define MAXDOUBLE DBL_MAX
#endif
#ifndef MAXFLOAT
#define MAXFLOAT FLT_MAX
#endif

#ifdef EXTERNAL_IPC
#include <ipc.h>
#else
#include <carmen/ipc.h>
#endif


#include <carmen/global.h>
#include <carmen/ipc_wrapper.h>
#include <carmen/carmenserial.h>

#include <carmen/pswrap.h>

#include <carmen/param_interface.h>

#ifndef COMPILE_WITHOUT_LASER_SUPPORT
#include <carmen/laser_messages.h>
#include <carmen/laser_interface.h>
#include <carmen/laser_ldmrs_messages.h>
#include <carmen/laser_ldmrs_interface.h>
#endif

#ifndef COMPILE_WITHOUT_LASER_SUPPORT
#include <carmen/keyctrl.h>
#endif

#ifndef COMPILE_WITHOUT_MAP_SUPPORT
#include <carmen/map.h>
#include <carmen/map_io.h>
#include <carmen/map_messages.h>
#include <carmen/map_interface.h>
#include <carmen/map_util.h>
#endif

#include <carmen/geometry.h>
#include <carmen/movement.h>

#include <carmen/gps_nmea_messages.h>
#include <carmen/gps_nmea_interface.h>

#include <carmen/gps_xyz_messages.h>
#include <carmen/gps_xyz_interface.h>

#include <carmen/pantilt_messages.h>
#include <carmen/pantilt_interface.h>

#include <carmen/imu_messages.h>
#include <carmen/imu_interface.h>

#include <carmen/velodyne_messages.h>
#include <carmen/velodyne_interface.h>

#include <carmen/bumblebee_basic_interface.h>
#include <carmen/bumblebee_basic_messages.h>

#include <carmen/stereo_interface.h>
#include <carmen/stereo_messages.h>

#include <carmen/kinect_messages.h>
#include <carmen/kinect_interface.h>
#include <carmen/kinect_util.h>

#include <carmen/xsens_messages.h>
#include <carmen/xsens_interface.h>

#include <carmen/xsens_mtig_interface.h>

#include <carmen/logger_messages.h>
#include <carmen/logger.h>
#include <carmen/logger_interface.h>

#include <carmen/writelog.h>
#include <carmen/readlog.h>

#include <carmen/playback_interface.h>
#include <carmen/playback_messages.h>

#include <carmen/rddf_messages.h>
#include <carmen/rddf_interface.h>

#include <carmen/road_mapper_interface.h>
#include <carmen/road_mapper_messages.h>
#include <carmen/road_mapper.h>

#include <carmen/robot_ackerman_messages.h>
#include <carmen/robot_ackerman_interface.h>

#include <carmen/simulator_ackerman_messages.h>
#include <carmen/simulator_ackerman_interface.h>

#include <carmen/localize_ackerman_messages.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/localize_ackerman_motion.h>

#include <carmen/navigator_ackerman_messages.h>
#include <carmen/navigator_ackerman_interface.h>
#include <carmen/navigator_ackerman.h>

#include <carmen/navigator_gui_messages.h>
#include <carmen/navigator_gui_interface.h>

#include <carmen/velodyne_messages.h>
#include <carmen/velodyne_interface.h>

#include <carmen/map_server_messages.h>
#include <carmen/map_server_interface.h>

#include <carmen/rotation_geometry.h>

#include <carmen/base_ackerman_messages.h>
#include <carmen/base_ackerman_interface.h>

#include <carmen/ford_escape_hybrid_messages.h>
#include <carmen/ford_escape_hybrid_interface.h>

#include <carmen/can_dump_messages.h>
#include <carmen/can_dump_interface.h>

#include <carmen/route_planner_messages.h>
#include <carmen/route_planner_interface.h>

#include <carmen/offroad_planner_messages.h>
#include <carmen/offroad_planner_interface.h>


#ifdef CARMEN_MEMORY_DEBUG
#include <dmalloc.h>
#endif

#ifdef CARMEN_ULTRASONIC_SENSOR
#include <carmen/ultrasonic_filter_messages.h>
#include <carmen/ultrasonic_filter_interface.h>
#endif

#include <carmen/localize_neural_messages.h>
#include <carmen/localize_neural_interface.h>
#ifdef __cplusplus
}
#endif

#endif
