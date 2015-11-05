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

#include <carmen/carmen.h>
#include "pantilt.h"
#include "pantilt_messages.h"

void
PantiltInit( int argc, char **argv )
{
  pDevice.port = (char *) malloc( 100 * sizeof(char *) );
  strncpy( pDevice.port, "/dev/ttyR1", 100 );
  pDevice.baud   = 9600;
  pDevice.bits   = 8;
  pDevice.parity = 'N';
  pDevice.swf    = FALSE;
  pDevice.hwf    = FALSE;
  
  PantiltParams( argc, argv );
  PantiltInitializeDevice( &pDevice );
  PantiltGetInfo();
}

void
PantiltGetInfo( void )
{
   char      buffer[BUFFER_SIZE];
 
  fprintf( stderr, "PT2: -----------------------------------\n" );

  /* disable echo */
  writeCMD( pDevice, "ED", buffer );
  
  /* print version */
  writeCMD( pDevice, "V", buffer );
  fprintf( stderr, "PT2: %s\n", buffer );
  
  /* enable limits */
  writeCMD( pDevice, "LE", buffer );
  
  /* ticks 2 angle */
  writeCMD( pDevice, "PR", buffer );
  fprintf( stderr, "PT2: sec-arc per pan pos           -> " ); 
  if (!sscanf( buffer, "%f seconds arc per Pan position",
	       &pSettings.res_pan )) {
    fprintf( stderr, "unknown\n" );
  } else {
    fprintf( stderr, "%f seconds arc (%.5f degree)\n",
	     pSettings.res_pan, ( pSettings.res_pan/3600.0 ) );
  }
  
  fprintf( stderr, "PT2: sec-arc per tilt pos          -> " ); 
  writeCMD( pDevice, "TR", buffer );
  if (!sscanf( buffer, "%f seconds arc per Tilt position",
	       &pSettings.res_tilt )) {
    fprintf( stderr, "unknown\n" );
  } else {
    fprintf( stderr, "%f seconds arc (%.5f degree)\n", 
	     pSettings.res_tilt, ( pSettings.res_tilt/3600.0 ) );
  }
  
  pSettings.toff_pan = ((pSettings.doff_pan*3600.0)/(pSettings.res_pan+0.0));
  fprintf( stderr, "PT2: offset pan                    -> %d degrees (%d ticks)\n", 
	   pSettings.doff_pan, pSettings.toff_pan );
  
  pSettings.toff_tilt = ((pSettings.doff_tilt*3600.0)/(pSettings.res_tilt+0.0));
  fprintf( stderr, "PT2: offset tilt                   -> %d degrees (%d ticks)\n",
	   pSettings.doff_tilt, pSettings.toff_tilt );
  
  /* request limits */
  fprintf( stderr, "PT2: pan limit min                 -> " );
  writeCMD( pDevice, "PX", buffer );
  if (!sscanf( buffer, "Maximum Pan position is %d", &pSettings.max_pan )) {
    fprintf( stderr, "unknown\n" );
  } else {
    pSettings.max_pan_deg = pticks2deg(pSettings.max_pan);
    fprintf( stderr, "%d ticks (%d degrees)\n",
	     pSettings.max_pan, pSettings.max_pan_deg );
    if (pLimits.upmin && (pLimits.pmin>pSettings.max_pan_deg)) {
      pSettings.max_pan_deg = pLimits.pmin;
      pSettings.max_pan = pdeg2ticks(pLimits.pmin);
      fprintf( stderr, "PT2:  -> argument set limit to %d degrees (%d ticks)\n",
	       pSettings.max_pan_deg, pSettings.max_pan );
    }
  }
  
  fprintf( stderr, "PT2: pan limit max                 -> " );
  writeCMD( pDevice, "PN", buffer );
  if (!sscanf( buffer, "Minimum Pan position is %d", &pSettings.min_pan )) {
    fprintf( stderr, "unknown\n" );
  } else {
    pSettings.min_pan_deg = pticks2deg(pSettings.min_pan);
    fprintf( stderr, "%d ticks (%d degrees)\n",
	     pSettings.min_pan, pSettings.min_pan_deg );
    if (pLimits.upmax && (pLimits.pmax<pSettings.min_pan_deg)) {
      pSettings.min_pan_deg = pLimits.pmax;
      pSettings.min_pan = pdeg2ticks(pLimits.pmax);
      fprintf( stderr, "PT2:  -> argument set limit to %d degrees (%d ticks)\n",
	       pSettings.min_pan_deg, pSettings.min_pan );
    }
  }
  
  fprintf( stderr, "PT2: tilt limit min                -> " );
  writeCMD( pDevice, "TN", buffer );
  if (!sscanf( buffer, "Minimum Tilt position is %d",
	       &pSettings.min_tilt )) {
    fprintf( stderr, "unknown\n" );
  } else {
    pSettings.min_tilt_deg = tticks2deg(pSettings.min_tilt);
    fprintf( stderr, "%d ticks (%d degrees)\n",
	     pSettings.min_tilt, pSettings.min_tilt_deg );
    if (pLimits.utmin && (pLimits.tmin>pSettings.min_tilt_deg)) {
      pSettings.min_tilt_deg = pLimits.tmin;
      pSettings.min_tilt = tdeg2ticks(pLimits.tmin);
      fprintf( stderr, "PT2:  -> argument set limit to %d degrees (%d ticks)\n",
	       pSettings.min_tilt_deg, pSettings.min_tilt );
    }
  }
  
  fprintf( stderr, "PT2: tilt limit max                -> " );
  writeCMD( pDevice, "TX", buffer );
  if (!sscanf( buffer, "Maximum Tilt position is %d",
	       &pSettings.max_tilt )) {
    fprintf( stderr, "unknown\n" );
  } else {
    pSettings.max_tilt_deg = tticks2deg(pSettings.max_tilt);
    fprintf( stderr, "%d ticks (%d degrees)\n", 
	     pSettings.max_tilt, pSettings.max_tilt_deg );
    if (pLimits.utmax && (pLimits.tmax<pSettings.max_tilt_deg)) {
      pSettings.max_tilt_deg = pLimits.tmax;
      pSettings.max_tilt = tdeg2ticks(pLimits.tmax);
      fprintf( stderr, "PT2:  -> argument set limit to %d degrees (%d ticks)\n",
	       pSettings.max_tilt_deg, pSettings.max_tilt );
    }
  }
  
  /* acceleration */
  fprintf( stderr, "PT2: Pan acceleration              -> " );
  writeCMD( pDevice, "PA", buffer );
  if (!sscanf( buffer, "Pan acceleration is %d positions/sec^2",
	       &pSettings.acc_pan )) {
    fprintf( stderr, "unknown\n" );
  } else {
    fprintf( stderr, "%d positions/sec^2\n", pSettings.acc_pan );
  }
  
  fprintf( stderr, "PT2: Tilt acceleration             -> " );
  writeCMD( pDevice, "TA", buffer );
  if (!sscanf( buffer, "Tilt acceleration is %d positions/sec^2",
	       &pSettings.acc_tilt )) {
    fprintf( stderr, "unknown\n" );
  } else {
    fprintf( stderr, "%d positions/sec^2\n", pSettings.acc_tilt );
  }
  
  /* target speed */
  fprintf( stderr, "PT2: Pan target speed              -> " );
  writeCMD( pDevice, "PS", buffer );
  if (!sscanf( buffer, "Target Pan speed is %d positions/sec",
	       &pSettings.tspd_pan )) {
    fprintf( stderr, "unknown\n" );
  } else {
    fprintf( stderr, "%d positions/sec\n", pSettings.tspd_pan );
  }
  
  fprintf( stderr, "PT2: Tilt target speed             -> " );
  writeCMD( pDevice, "TS", buffer );
  if (!sscanf( buffer, "Target Tilt speed is %d positions/sec",
	       &pSettings.tspd_tilt )) {
    fprintf( stderr, "unknown\n" );
  } else {
    fprintf( stderr, "%d positions/sec\n", pSettings.tspd_tilt );
  }
    
  /* base speed */
  fprintf( stderr, "PT2: Pan base speed                -> " );
  writeCMD( pDevice, "PB", buffer );
  if (!sscanf(buffer,"Current Pan base speed is %d positions/sec",
	      &pSettings.bspd_pan)) {
    fprintf( stderr, "unknown\n" );
  } else {
    fprintf( stderr, "%d positions/sec\n", pSettings.bspd_pan );
  }
  
  fprintf( stderr, "PT2: Tilt base speed               -> " );
  writeCMD( pDevice, "TB", buffer );
  if (!sscanf(buffer,"Current Tilt base speed is %d positions/sec",
	      &pSettings.bspd_tilt)) {
    fprintf( stderr, "unknown\n" );
  } else {
    fprintf( stderr, "%d positions/sec\n", pSettings.bspd_tilt );
  }
  
  fprintf( stderr, "PT2: current pan position          -> " );
  writeCMD( pDevice, "PP", buffer );
  if (!sscanf(buffer,"Current Pan position is %d",&pPos.pan)) {
    fprintf( stderr, "unknown\n" );
  } else {
    fprintf( stderr, "%d ticks (%d degrees)\n", pPos.pan,
	       (int) pticks2deg(pPos.pan) );
  }
  
  fprintf( stderr, "PT2: current tilt position         -> " );
  writeCMD( pDevice, "TP", buffer );
  if (!sscanf(buffer,"Current Tilt position is %d",&pPos.tilt)) {
      fprintf( stderr, "unknown\n" );
  } else {
    fprintf( stderr, "%d ticks (%d degrees)\n", pPos.tilt,
	     (int) tticks2deg(pPos.tilt) );
  }
  
  fprintf( stderr, "PT2: -----------------------------------\n" );
}
