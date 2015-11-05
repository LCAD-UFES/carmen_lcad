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
#ifndef CARMEN_GPS_WRAPPER_H__
#define CARMEN_GPS_WRAPPER_H__

#include <carmen/carmen.h>

#ifdef __cplusplus
extern "C" {
#endif

	//Transforma em Coordenadas Gdc dada uma coordenada Utm (C wrapper)
	void carmen_Utm_Gdc3(
			double X, double Y, double Z,
			double Zone, int hemi_n,
			double *latitude, double *longitude,
			double *elevation);

	//Transforma em Coordenadas Utm dada uma coordenada Gdc
	void carmen_Gdc3_Utm(
			double *X, double *Y, double *Z,
			double *Zone, int *hemi_n,
			double latitude,
			double longitude,
			double elevation);

#ifdef __cplusplus
}
#endif
#endif
