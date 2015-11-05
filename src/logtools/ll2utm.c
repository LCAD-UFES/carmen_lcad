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

/* logtools_ll2utm.c - convert latitude & longitude into Universal Transverse Mercator
**
** Partially based on code by Chuck Gantz <chuck.gantz@globalstar.com>.
**
** Copyright © 2001 by Jef Poskanzer <jef@acme.com>.
** All rights reserved.
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions
** are met:
** 1. Redistributions of source code must retain the above copyright
**    notice, this list of conditions and the following disclaimer.
** 2. Redistributions in binary form must reproduce the above copyright
**    notice, this list of conditions and the following disclaimer in the
**    documentation and/or other materials provided with the distribution.
**
** THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
** ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
** ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
** FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
** DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
** OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
** HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
** LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
** OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
** SUCH DAMAGE.
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#include <carmen/logtools.h>
#include "coords.h"

static char utm_letter( double latitude );

logtools_utm_coord_t
logtools_ll2utm( logtools_ll_coord_t ll )
{
  double lat_rad, long_rad;
  double long_origin, long_origin_rad;
  double eccPrimeSquared;
  double N, T, C, A, M;
  int zone;
  logtools_utm_coord_t utm;
  
  /* We want the longitude within -180..180. */
  if ( ll.longitude < -180.0 )
    ll.longitude += 360.0;
  if ( ll.longitude > 180.0 )
    ll.longitude -= 360.0;
  
  /* Now convert. */
  lat_rad = ll.latitude * M_PI / 180.0;
  long_rad = ll.longitude * M_PI / 180.0;
  zone = (int) ( ( ll.longitude + 180 ) / 6 ) + 1;
  if ( ll.latitude >= 56.0 && ll.latitude < 64.0 &&
       ll.longitude >= 3.0 && ll.longitude < 12.0 )
    zone = 32;
  /* Special zones for Svalbard. */
  if ( ll.latitude >= 72.0 && ll.latitude < 84.0 )
    {
      if      ( ll.longitude >= 0.0  && ll.longitude <  9.0 ) zone = 31;
      else if ( ll.longitude >= 9.0  && ll.longitude < 21.0 ) zone = 33;
      else if ( ll.longitude >= 21.0 && ll.longitude < 33.0 ) zone = 35;
      else if ( ll.longitude >= 33.0 && ll.longitude < 42.0 ) zone = 37;
    }
  long_origin = ( zone - 1 ) * 6 - 180 + 3;	/* +3 puts origin in middle of zone */
  long_origin_rad = long_origin * M_PI / 180.0;
  eccPrimeSquared = EccentricitySquared / ( 1.0 - EccentricitySquared );
  N = EquatorialRadius / sqrt( 1.0 - EccentricitySquared * sin( lat_rad ) * sin( lat_rad ) );
  T = tan( lat_rad ) * tan( lat_rad );
  C = eccPrimeSquared * cos( lat_rad ) * cos( lat_rad );
  A = cos( lat_rad ) * ( long_rad - long_origin_rad );
  M = EquatorialRadius *
    ( ( 1.0 - EccentricitySquared / 4 - 3 *
	EccentricitySquared * EccentricitySquared / 64 - 5 *
	EccentricitySquared * EccentricitySquared *
	EccentricitySquared / 256 ) *
      lat_rad - ( 3 * EccentricitySquared / 8 +
		  3 * EccentricitySquared * EccentricitySquared / 32 +
		  45 * EccentricitySquared * EccentricitySquared *
		  EccentricitySquared / 1024 ) *
      sin( 2 * lat_rad ) +
      ( 15 * EccentricitySquared * EccentricitySquared / 256 +
	45 * EccentricitySquared * EccentricitySquared *
	EccentricitySquared / 1024 ) *
      sin( 4 * lat_rad ) -
      ( 35 * EccentricitySquared * EccentricitySquared *
	EccentricitySquared / 3072 ) *
      sin( 6 * lat_rad ) );
  utm.easting =
    K0 * N * ( A + ( 1 - T + C ) * A * A * A / 6 +
	       ( 5 - 18 * T + T * T + 72 * C - 58 * eccPrimeSquared ) *
	       A * A * A * A * A / 120 ) + 500000.0;
  utm.northing =
    K0 * ( M + N * tan( lat_rad ) * ( A * A / 2 +
				      ( 5 - T + 9 * C + 4 * C * C ) *
				      A * A * A * A / 24 +
				      ( 61 - 58 * T + T * T + 600 * C -
					330 * eccPrimeSquared ) *
				      A * A * A * A * A * A / 720 ) );
  if ( ll.latitude < 0.0 )
    utm.northing += 10000000.0;  /* 1e7 meter offset for southern hemisphere */

  utm.zone = zone;
  utm.letter = utm_letter( ll.latitude );
  
  return(utm);
}


static char
utm_letter( double latitude )
{
  /* This routine determines the correct UTM letter designator for the
  ** given latitude.  It returns 'Z' if the latitude is outside the UTM
  ** limits of 84N to 80S.
  */
  if ( latitude <= 84.0 && latitude >= 72.0 ) return 'X';
  else if ( latitude < 72.0 && latitude >= 64.0 ) return 'W';
  else if ( latitude < 64.0 && latitude >= 56.0 ) return 'V';
  else if ( latitude < 56.0 && latitude >= 48.0 ) return 'U';
  else if ( latitude < 48.0 && latitude >= 40.0 ) return 'T';
  else if ( latitude < 40.0 && latitude >= 32.0 ) return 'S';
  else if ( latitude < 32.0 && latitude >= 24.0 ) return 'R';
  else if ( latitude < 24.0 && latitude >= 16.0 ) return 'Q';
  else if ( latitude < 16.0 && latitude >= 8.0 ) return 'P';
  else if ( latitude <  8.0 && latitude >= 0.0 ) return 'N';
  else if ( latitude <  0.0 && latitude >= -8.0 ) return 'M';
  else if ( latitude < -8.0 && latitude >= -16.0 ) return 'L';
  else if ( latitude < -16.0 && latitude >= -24.0 ) return 'K';
  else if ( latitude < -24.0 && latitude >= -32.0 ) return 'J';
  else if ( latitude < -32.0 && latitude >= -40.0 ) return 'H';
  else if ( latitude < -40.0 && latitude >= -48.0 ) return 'G';
  else if ( latitude < -48.0 && latitude >= -56.0 ) return 'F';
  else if ( latitude < -56.0 && latitude >= -64.0 ) return 'E';
  else if ( latitude < -64.0 && latitude >= -72.0 ) return 'D';
  else if ( latitude < -72.0 && latitude >= -80.0 ) return 'C';
  else return 'Z';
}
