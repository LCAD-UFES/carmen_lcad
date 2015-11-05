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

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/time.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <ctype.h>

#include <carmen/logtools.h>

void 
compute_forward_correction( logtools_rpos2_t pos, logtools_rpos2_t corr, logtools_rpos2_t *cpos )
{
  cpos->o = pos.o - corr.o;
  for (; cpos->o <= -M_PI;) cpos->o += 2*M_PI;
  for (; cpos->o  >  M_PI;) cpos->o -= 2*M_PI;

  cpos->x = ((pos.x - corr.x) * cos(corr.o))
    + ((pos.y - corr.y) * sin(corr.o));

  cpos->y = ((pos.y - corr.y) * cos(corr.o))
    - ((pos.x - corr.x) * sin(corr.o));
}

void 
compute_backward_correction( logtools_rpos2_t cpos, logtools_rpos2_t corr, logtools_rpos2_t *pos )
{
  pos->o = corr.o + cpos.o;
  for (; pos->o <= -M_PI;) pos->o += 2*M_PI;
  for (; pos->o  >  M_PI;) pos->o -= 2*M_PI;

  pos->x = corr.x + (cpos.x * cos(corr.o))
    - (cpos.y * sin(corr.o));
  
  pos->y = corr.y + (corr.y * cos(corr.o))
    + (cpos.x * sin(corr.o));
}  

void 
compute_correction_parameters( logtools_rpos2_t pos, logtools_rpos2_t cpos, logtools_rpos2_t *corr )
{
  corr->o = pos.o - cpos.o;
  for (; corr->o <= -M_PI;) corr->o += 2*M_PI;
  for (; corr->o  >  M_PI;) corr->o -= 2*M_PI;
  
  corr->x = pos.x - (cpos.x * cos(corr->o))
    + (cpos.y * sin(corr->o));

  corr->y = pos.y - (cpos.y * cos(corr->o))
    - (cpos.x * sin(corr->o));
}

void 
update_correction_parameters( logtools_rpos2_t cpos, logtools_rpos2_t delta, logtools_rpos2_t *corr )
{
  logtools_rpos2_t pos, cpos2;

  compute_backward_correction( cpos, *corr, &pos );
  
  cpos2.x = cpos.x + delta.x;
  cpos2.y = cpos.y + delta.y;
  cpos2.o = cpos.o + delta.o;

  compute_correction_parameters( pos, cpos2, corr );
}

void
robot2map( logtools_rpos2_t pos, logtools_rpos2_t corr, logtools_rpos2_t *map )
{
  logtools_rpos2_t rpos = pos;
  rpos.o = deg2rad(pos.o);
  compute_forward_correction( rpos, corr, map );
  /*  map->o = deg2rad( map->o ); */
}


void
map2robot( logtools_rpos2_t map, logtools_rpos2_t corr, logtools_rpos2_t *pos )
{
  logtools_rpos2_t map2 = map;
  /*  map2.o = rad2deg( map.o ); */
  compute_backward_correction( map2, corr, pos );
  pos->o = 90.0 - rad2deg(pos->o);
}


void
computeCorr( logtools_rpos2_t pos, logtools_rpos2_t map, logtools_rpos2_t *corr )
{
  logtools_rpos2_t map2, pos2;
  
  map2   = map;
  map2.o = rad2deg( map.o );

  pos2   = pos;
  pos2.o = 90.0 - pos.o;

  compute_correction_parameters( pos2, map2, corr );
}









