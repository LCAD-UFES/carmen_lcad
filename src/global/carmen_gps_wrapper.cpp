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

#include <carmen/carmen_gps.h>
#include <carmen/carmen_gps_wrapper.h>
#include <carmen/carmen.h>

//Transforma em Coordenadas Gdc dada uma coordenada Utm
void carmen_Utm_Gdc3(double X, double Y, double Z, double Zone, int hemi_n, double *latitude, double *longitude, double *elevation)
{
	Utm_Coord_3d utm;
	utm.x = X;
	utm.y = Y;
	utm.z = Z;
	utm.zone = Zone;
	if (hemi_n == 1) utm.hemisphere_north = true;
	else utm.hemisphere_north = false;

	Gdc_Coord_3d gdc;
	Utm_To_Gdc_Converter::Init();
	Utm_To_Gdc_Converter::Convert(utm,gdc);

	*latitude = gdc.latitude;
	*longitude = gdc.longitude;
	*elevation = gdc.elevation;
}

//Transforma em Coordenadas Utm dada uma coordenada Gdc
void carmen_Gdc3_Utm(double *X, double *Y, double *Z, double *Zone, int *hemi_n, double latitude, double longitude, double elevation)
{
	Utm_Coord_3d utm;
	Gdc_Coord_3d gdc;

	gdc.latitude = latitude;
	gdc.longitude = longitude;
	gdc.elevation = elevation;

	Gdc_To_Utm_Converter::Init();
	Gdc_To_Utm_Converter::Convert(gdc, utm);

	*X = utm.x;
	*Y = utm.y;
	*Z = utm.z;
	*Zone = utm.zone;

	if (utm.hemisphere_north)
		*hemi_n = 1;
	else
		*hemi_n = 0;

}

