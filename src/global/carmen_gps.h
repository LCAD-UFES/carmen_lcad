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
#ifndef CARMEN_GPS_H
#define CARMEN_GPS_H
 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <stdio.h>



#include <carmen/carmen.h>
#include <carmen/Utm_To_Gdc_Converter.h>
#include <carmen/Gdc_To_Utm_Converter.h>

#include <carmen/Utm_Coord_3d.h>
#include <carmen/Gdc_Coord_3d.h>



//Transforma em Coordenadas Utm dado uma coordenada gdc
Utm_Coord_3d carmen_Gdc_Utm(Gdc_Coord_3d gdc);

//Transforma em Coordenadas Utm dados latitude, orientacao da latitude, logitude, orientacao da longitude e altitude
Utm_Coord_3d carmen_Gdc_Utm2(double latitude, double lat_orient, double longitude, double long_orient, double altitude);

//Transforma em Coordenadas Gdc dada uma coordenada Utm
Gdc_Coord_3d carmen_Utm_Gdc(Utm_Coord_3d utm);

//Transforma em Coordenadas Gdc dada uma coordenada Utm
Gdc_Coord_3d carmen_Utm_Gdc2(double X, double Y, double Z, double Zone, int hemi_n);

#endif
