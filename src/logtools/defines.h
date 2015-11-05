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

#ifndef UXTOOLS_DEFINES_H
#define UXTOOLS_H

#define ANGLE_0_25_DEG_IN_RAD 0.00436332312998582394

#define READ_MODE_SILENT    0x00
#define READ_MODE_VERBOSE   0x01
#define READ_MODE_DONT_STOP 0x02

#define FILE_SCRIPT_EXT              ".script"
#define FILE_CARMEN_EXT              ".clf"
#define FILE_REC_EXT                 ".rec"
#define FILE_MOOS_EXT                ".alog"
#define FILE_PLAYER_EXT              ".plf"
#define FILE_SAPHIRA_EXT             ".2d"
#define FILE_PLACELAB_EXT            ".plab"

#define GSM_ALLOC_STEP                 1000
#define MAX_LINE_LENGTH               65536
#define MAX_CMD_LENGTH                 2048
#define MAX_NUM_LENGTH                   80
#define MAX_NAME_LENGTH                 256
#define MAX_NUM_GSM_NEIGHBORS            40

#define MAX_NUM_SWEEPS                  100
#define MAX_POLYGON_SIZE              10000

#define MAX_NUM_LASER_BEAMS             401

#define DEFAULT_AMTEC_PANTILT_AXIS_DIST        15.0
#define DEFAULT_AMTEC_PANTILT_AXIS_FORWARD     60.0
#define DEFAULT_AMTEC_PANTILT_AXIS_SIDEWARD    0.0
#define DEFAULT_AMTEC_PANTILT_AXIS_UPWARD      70.0

#define DEFAULT_EPSILON                   0.00001
#define NO_TAG                            "all"

#define SWISSRANGER_HORIZONTAL_FOV          43
#define SWISSRANGER_VERTICAL_FOV            46
#define SWISSRANGER_HORIZONTAL_NUM_PIXELS  160
#define SWISSRANGER_VERTICAL_NUM_PIXELS    124

#ifndef MAXFLOAT
#define MAXFLOAT      3.40282347e+38F
#endif

#endif

