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
#include <fnmatch.h>

#include <carmen/logtools.h>

#include "defines.h"
#include "internal.h"

int
logtools_read_logfile( logtools_log_data_t * rec, char * filename )
{
  enum logtools_file_t     inp_type = UNKOWN;
  char               fname[MAX_NAME_LENGTH];

    fprintf( stderr, "#####################################################################\n" );
  if ( !fnmatch( "script:*", filename, 0) ) {
    fprintf( stderr, "# INFO: use script-file-type!\n" );
    strncpy( fname, &(filename[7]), MAX_NAME_LENGTH );
    inp_type = SCRIPT;
  } else if ( !fnmatch( "rec:*", filename, 0) ) {
    fprintf( stderr, "# INFO: read rec-file-type!\n" );
    strncpy( fname, &(filename[4]), MAX_NAME_LENGTH );
    inp_type = REC;
  } else if ( !fnmatch( "carmen:*", filename, 0) ) {
    fprintf( stderr, "# INFO: read carmen-file-type!\n" );
    strncpy( fname, &(filename[7]), MAX_NAME_LENGTH );
    inp_type = CARMEN;
  } else if ( !fnmatch( "moos:*", filename, 0) ) {
    fprintf( stderr, "# INFO: read moos-file-type!\n" );
    strncpy( fname, &(filename[5]), MAX_NAME_LENGTH );
    inp_type = MOOS;
  } else if ( !fnmatch( "player:*", filename, 0) ) {
    fprintf( stderr, "# INFO: read player-file-type!\n" );
    strncpy( fname, &(filename[7]), MAX_NAME_LENGTH );
    inp_type = PLAYER;
  } else if ( !fnmatch( "placelab:*", filename, 0) ) {
    fprintf( stderr, "# INFO: read placelab-file-type!\n" );
    strncpy( fname, &(filename[9]), MAX_NAME_LENGTH );
    inp_type = PLACELAB;
  } else if ( !fnmatch( "plab:*", filename, 0) ) {
    fprintf( stderr, "# INFO: read placelab-file-type!\n" );
    strncpy( fname, &(filename[5]), MAX_NAME_LENGTH );
    inp_type = PLACELAB;
  } else if ( !fnmatch( "*" FILE_SCRIPT_EXT, filename, 0) ) {
    fprintf( stderr, "# INFO: use script-file-type!\n" );
    strncpy( fname, filename, MAX_NAME_LENGTH );
    inp_type = SCRIPT;
  } else if ( !fnmatch( "*" FILE_REC_EXT, filename, 0) ) {
    fprintf( stderr, "# INFO: read rec-file-type!\n" );
    strncpy( fname, filename, MAX_NAME_LENGTH );
    inp_type = REC;
  } else if ( !fnmatch( "*"FILE_CARMEN_EXT, filename, 0) ) {
    fprintf( stderr, "# INFO: read carmen-file-type!\n" );
    strncpy( fname, filename, MAX_NAME_LENGTH );
    inp_type = CARMEN;
  } else if ( !fnmatch( "*" FILE_MOOS_EXT, filename, 0) ) {
    fprintf( stderr, "# INFO: read moos-file-type!\n" );
    strncpy( fname, filename, MAX_NAME_LENGTH );
    inp_type = MOOS;
  } else if ( !fnmatch( "*" FILE_PLAYER_EXT, filename, 0) ) {
    fprintf( stderr, "# INFO: read player-file-type!\n" );
    strncpy( fname, filename, MAX_NAME_LENGTH );
    inp_type = PLAYER;
  } else if ( !fnmatch( "*" FILE_PLACELAB_EXT, filename, 0) ) {
    fprintf( stderr, "# INFO: read placelab-file-type!\n" );
    strncpy( fname, filename, MAX_NAME_LENGTH );
    inp_type = PLACELAB;
  } else if ( !fnmatch( "*" FILE_SAPHIRA_EXT, filename, 0) ) {
    fprintf( stderr, "# INFO: read saphira-file-type!\n" );
    strncpy( fname, filename, MAX_NAME_LENGTH );
    inp_type = SAPHIRA;
  } else {
    fprintf( stderr, "# INFO: read carmen-file-type!\n" );
    strncpy( fname, filename, MAX_NAME_LENGTH );
    inp_type = CARMEN;
  }

  switch (inp_type) {
  case SCRIPT:
    if (read_script( fname, rec, 1 ) !=0 )
      return(FALSE);
    break;
  case REC:
    if (load_rec2d_file( fname, rec, REC, READ_MODE_DONT_STOP ) !=0 )
      return(FALSE);
    break;
  case CARMEN:
    if (load_rec2d_file( fname, rec, CARMEN, READ_MODE_DONT_STOP ) !=0 )
      return(FALSE);
    break;
  case MOOS:
    if (load_rec2d_file( fname, rec, MOOS, READ_MODE_DONT_STOP ) !=0 )
      return(FALSE);
    break;
  case PLAYER:
    if (load_rec2d_file( fname, rec, PLAYER, READ_MODE_DONT_STOP ) !=0 )
      return(FALSE);
    break;
  case SAPHIRA:
    if (load_rec2d_file( fname, rec, SAPHIRA, READ_MODE_DONT_STOP ) !=0 )
      return(FALSE);
    break;
  case PLACELAB:
    if (load_rec2d_file( fname, rec, PLACELAB, READ_MODE_DONT_STOP ) !=0 )
      return(FALSE);
    break;
  default:
    fprintf( stderr, "ERROR: unknown file-type!\n" );
    return(FALSE);
  }

  return(TRUE);
}

/***********************************************************************/
/*                                                                     */
/*                                                                     */
/*                                                                     */
/***********************************************************************/

int
read_rec2d_file( char *filename, logtools_log_data_t *rec,  int mode )
{
  return(load_rec2d_file( filename, rec, REC, mode ));
}


int
load_rec2d_file( char *filename, logtools_log_data_t *rec,
		 enum logtools_file_t type, int mode )
{

  char      line[MAX_LINE_LENGTH];
  int       FEnd, numPos, numScan;
  char      dummy[MAX_CMD_LENGTH];
  char      command[MAX_CMD_LENGTH];
  char    * sptr, * iline, * lptr;
  FILE    * iop;
  int       linectr = 0;
  int       posctr = 0;
  int       laserctr = 0;
  int       gpsctr = 0;
  int       numEntries = 0;
  int       markerctr = 0;
  int       wifictr = 0;

  fprintf( stderr, "# read file %s ...\n", filename );
  if ((iop = fopen( filename, "r")) == 0){
    fprintf(stderr, "# WARNING: no file %s\n", filename );
    return(-1);
  }

  FEnd = 0;

  rec->info.system = type;

  switch (rec->info.system) {
    
  case REC:
    do{
      linectr++;
      if (fscanf(iop, "%s", command) == EOF)
	FEnd=1;
      else{
	if (!strcmp( command, "POS" )) {
	  posctr++;
	} else if ( (!strcmp( command, "LASER" ))       ||
		    (!strcmp( command, "LASER-RANGE" )) ||
		    (!strcmp( command, "LASER-SCAN" ))  ||
		    (!strcmp( command, "CARMEN-LASER" )) ) {
	  laserctr++;
	} else if (!strcmp( command, "GPS" )) {
	  gpsctr++;
	} else if (!strcmp( command, "NMEA-GGA" )) {
	  gpsctr++;
	} else if ( (!strcmp( command, "MARK-POS")) ||
		    (!strcmp( command, "MARKER")) ) {
	  markerctr++;
	} else if ( !strcmp( command, "WIFI") ||
		    !strcmp( command, "WIFI-DIST") ) {
	  wifictr++;
	  fgets(command,sizeof(command),iop);
	}
      }
    } while (!FEnd);
    break;
    
  case CARMEN:
    do{
      linectr++;
      if (fscanf(iop, "%s", command) == EOF)
	FEnd=1;
      else{
	if (!strcmp( command, "ODOM")) {
	  posctr++;
	} else if (!strcmp( command, "FLASER") ){
	  laserctr++;	
	} else if (!strcmp( command, "RAWLASER1") ){
	  laserctr++;
	} else if (!strcmp( command, "ROBOTLASER1") ){
	  laserctr++;
	} else if (!strcmp( command, "ROBOTLASER2") ){
	  laserctr++;
	} else if (!strcmp( command, "RLASER") ){
	  laserctr++;
	} else if (!strcmp( command, "MARKER")) {
	  markerctr++;
	}
	fgets(command,sizeof(command),iop);
      }
    } while (!FEnd);
    break;
    
  case MOOS:
    do{
      linectr++;
      if (fscanf( iop, "%s %s", dummy, command ) == EOF) {
	FEnd=1;
      } else{
	if (!strcmp( command, "ROBOT_POSITION")) {
	  posctr++;
	} else if (!strcmp( command, "LASER_RANGE") ){
	  laserctr++;
	}
	fgets(command,sizeof(command),iop);
      }
    } while (!FEnd);
    break;

  case PLAYER:
    do{
      linectr++;
      if (fgets(line,MAX_LINE_LENGTH,iop) == NULL) {
	FEnd=1;
      } else {
	if (strstr( line, "position" ) != NULL) {
	  posctr++;
	} else if (strstr( line, "laser" ) != NULL) {
	  laserctr++;
	}
      }
    } while (!FEnd);
    break;

  case SAPHIRA:
    do{
      linectr++;
      if (fscanf( iop, "%s", command ) == EOF) {
	FEnd=1;
      } else{
	if (!strcmp( command, "robot:")) {
	  posctr++;
	} else if (!strcmp( command, "sick1:") ){
	  laserctr++;
	}
	fgets(command,sizeof(command),iop);
      }
    } while (!FEnd);
    break;

  case PLACELAB:
    iline = (char *) malloc( MAX_LINE_LENGTH * sizeof(char) );
    while (fgets(iline,MAX_LINE_LENGTH,iop) != NULL) {
      lptr = iline; linectr++;
      do {
	sptr = strsep( &lptr, "|" );
	if (sptr != NULL) {
	  sscanf( sptr, "%[^=]", command );
	  if (!strncasecmp( "type", command, MAX_LINE_LENGTH)) {
	    sscanf( sptr, "%[^=]=%[^|]", dummy, command );
	    if (!strncasecmp( "gps", command, MAX_LINE_LENGTH)) {
	      gpsctr++;
	      break;
	    } else if (!strncasecmp( "marker", command, MAX_LINE_LENGTH)) {
	      markerctr++;
	      break;
	    }
	  }
	}
      } while (sptr != NULL); 
    }
    break;
  default:
    fprintf( stderr, "ERROR: unknown file-type!\n" );
    return(FALSE);

    }

    
  if (mode && READ_MODE_VERBOSE) {
    fprintf( stderr, "#####################################################################\n" );
    if (posctr>0)
      fprintf( stderr, "# found %d positions\n", posctr );
    if (laserctr>0)
      fprintf( stderr, "# found %d laserscans\n", laserctr );
    if (gpsctr>0)
      fprintf( stderr, "# found %d gps pos\n", gpsctr );
    if (markerctr>0)
      fprintf( stderr, "# found %d marker\n", markerctr );
    if (wifictr>0)
      fprintf( stderr, "# found %d wifi\n", wifictr );
  }
  
  numEntries =
    posctr +
    laserctr +
    gpsctr +
    markerctr +
    wifictr + 1;

  rec->numentries = 0;

  rec->entry   =
    (logtools_entry_position_t *) malloc( numEntries * sizeof(logtools_entry_position_t) );

  rewind(iop);

  if (posctr>0) {
    rec->psens =
      (logtools_possens2_data_t *) malloc( posctr * sizeof(logtools_possens2_data_t) );
  } else
    rec->psens = NULL;

  if (laserctr>0)
    rec->lsens =
      (logtools_lasersens2_data_t *) malloc( laserctr * sizeof(logtools_lasersens2_data_t) ); 
  else
    rec->lsens = NULL;

  if (gpsctr>0)
    rec->gps =
      (logtools_gps_data_t *) malloc( gpsctr * sizeof(logtools_gps_data_t) );
  else
    rec->gps = NULL;

  if (markerctr>0)
    rec->marker =
      (logtools_marker_data_t *) malloc( markerctr * sizeof(logtools_marker_data_t) );
  else 
    rec->marker = NULL;
  
  if (wifictr>0)
    rec->wifi =
      (logtools_wifi_data_t *) malloc( wifictr * sizeof(logtools_wifi_data_t) );
  else 
    rec->wifi = NULL;
  
  numScan    = 0;
  numPos     = 0;

  rec->numpositions    = 0;
  rec->numlaserscans   = 0;
  rec->numgps          = 0;
  rec->nummarkers      = 0;
  rec->numwifi         = 0;
  
  FEnd    = 0;
  linectr = 0;


  switch (rec->info.system) {
    
  case REC:
    do{
      if (fgets(line,MAX_LINE_LENGTH,iop) == NULL) {
	break;
      }
      rec->entry[rec->numentries].linenr = linectr++;
    } while (rec2_parse_line( line, rec, TRUE, TRUE ));
    break;

  case CARMEN:
    do{
      if (fgets(line,MAX_LINE_LENGTH,iop) == NULL) {
	break;
      }
    } while (carmen_parse_line( line, rec, TRUE, TRUE ));
    break;

  case MOOS:
    do{
      if (fgets(line,MAX_LINE_LENGTH,iop) == NULL) {
	break;
      }
    } while (moos_parse_line( line, rec, TRUE, TRUE ));
    break;

  case PLAYER:
    do{
      if (fgets(line,MAX_LINE_LENGTH,iop) == NULL) {
	break;
      }
    } while (player_parse_line( line, rec, TRUE, TRUE ));
    break;

  case SAPHIRA:
    do{
      if (fgets(line,MAX_LINE_LENGTH,iop) == NULL) {
	break;
      }
    } while (saphira_parse_line( line, rec, TRUE, TRUE ));
    break;

  case PLACELAB:
    {
      do{
	if (fgets(line,MAX_LINE_LENGTH,iop) == NULL) {
	  break;
	}
      } while (placelab_parse_line( line, rec, TRUE, TRUE ));
    }
    break;

  default:
    fprintf( stderr, "ERROR: unknown file-type!\n" );
    return(FALSE);

  }
  
  if (0 && mode && READ_MODE_VERBOSE) {
    fprintf( stderr, "#####################################################################\n" );
    fprintf( stderr, "# num positions     = %d\n",
	     rec->numpositions );
    fprintf( stderr, "# num laserscans    = %d\n",
	     rec->numlaserscans );
    fprintf( stderr, "#####################################################################\n" );
  }

  fclose(iop);
  return(0);
}

