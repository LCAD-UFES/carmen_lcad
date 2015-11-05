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

/* logtools_utm2ll.c - convert Universal Transverse Mercator into latitude & longitude
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


#define ABS(x) ((x)>=0?(x):(-x))

logtools_ll_coord_t
logtools_utm2ll( logtools_utm_coord_t utm )
{
  logtools_ll_coord_t ll;
  double x, y;
  double eccPrimeSquared;
  double e1;
  double N1, T1, C1, R1, D, M;
  double long_origin;
  double mu, phi1_rad;
  int    northernHemisphere;	/* 1 for northern hemisphere, 0 for southern */
  
  /* Now convert. */
  x = utm.easting - 500000.0;	/* remove 500000 meter offset */
  y = utm.northing;
  if ( ( utm.letter - 'N' ) >= 0 )
    northernHemisphere = 1;	/* northern hemisphere */
  else {
    northernHemisphere = 0;	/* southern hemisphere */
    y -= 10000000.0;	        /* remove 1e7 meter offset */
  }
  long_origin = ( utm.zone - 1 ) * 6 - 180 + 3;	/* +3 puts origin in middle of zone */
  eccPrimeSquared =
    EccentricitySquared / ( 1.0 - EccentricitySquared );
  e1 =
    ( 1.0 - sqrt( 1.0 - EccentricitySquared ) ) /
    ( 1.0 + sqrt( 1.0 - EccentricitySquared ) );
  M = y / K0;
  mu = M / ( EquatorialRadius *
	     ( 1.0 - EccentricitySquared / 4 -
	       3 * EccentricitySquared * EccentricitySquared / 64 -
	       5 * EccentricitySquared * EccentricitySquared *
	       EccentricitySquared / 256 ) );
  phi1_rad =
    mu + ( 3 * e1 / 2 - 27 * e1 * e1 * e1 / 32 ) * sin( 2 * mu ) +
    ( 21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32 ) * sin( 4 * mu ) +
    ( 151 * e1 * e1 * e1 / 96 ) * sin( 6 *mu );
  N1 = EquatorialRadius /
    sqrt( 1.0 - EccentricitySquared * sin( phi1_rad ) * sin( phi1_rad ) );
  T1 = tan( phi1_rad ) * tan( phi1_rad );
  C1 = eccPrimeSquared * cos( phi1_rad ) * cos( phi1_rad );
  R1 = EquatorialRadius * ( 1.0 - EccentricitySquared ) /
    pow( 1.0 - EccentricitySquared * sin( phi1_rad ) * sin( phi1_rad ), 1.5 );
  D = x / ( N1 * K0 );
  ll.latitude = phi1_rad -
    ( N1 * tan( phi1_rad ) / R1 ) *
    ( D * D / 2 -
      ( 5 + 3 * T1 + 10 * C1 - 4 * C1 * C1 - 9 * eccPrimeSquared ) *
      D * D * D * D / 24 +
      ( 61 + 90 * T1 + 298 * C1 + 45 * T1 * T1 -
	252 * eccPrimeSquared - 3 * C1 * C1 ) *
      D * D * D * D * D * D / 720 );
  ll.latitude = ll.latitude * 180.0 / M_PI;
  ll.longitude = ( D - ( 1 + 2 * T1 + C1 ) * D * D * D / 6 +
		   ( 5 - 2 * C1 + 28 * T1 -
		     3 * C1 * C1 + 8 * eccPrimeSquared + 24 * T1 * T1 ) *
		   D * D * D * D * D / 120 ) / cos( phi1_rad );
  ll.longitude = long_origin + ll.longitude * 180.0 / M_PI;
  return(ll);
}

