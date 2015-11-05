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
#include <time.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <ctype.h>

#include <carmen/logtools.h>

#define MAX_LINE_LENGTH 40000

int
read_script( char *filename, logtools_log_data_t *script, int verbose ) {

  FILE   *fp;

  logtools_rpos2_t  rpos, npos = {0.0, 0.0, 0.0};
  logtools_rvel2_t  rvel = {0.0, 0.0};
  
  char inp0[80], inp1[80], inp2[80], sstr[80], secstr[80];

  char line[MAX_LINE_LENGTH];
  
  char command[80];

  struct timeval curTime;
  struct tm      dayTime;
  
  int i, cnt;

  int numValues1, numValues2;

  int day, month, year, hour, minute, sec;

  double angleDiff;

  logtools_laser_prop2_t  lprop[2];
  
  int numPositions      = 0;
  int numSonarScans     = 0;
  int numLaserScans     = 0;
  int numRLaserScans    = 0;
  int numTimes          = 0;
  int numEntries        = 0;

  /* this is one of the most awful things in beeSoft */
  int rotation_90_minus = FALSE;

  int lastFScan         = -1;
  int lastRScan         = -1;

  int FileEnd = FALSE;

  lprop[0].fov.start     =   -M_PI_2;
  lprop[0].fov.end       =    M_PI_2;
  lprop[0].fov.delta     =      M_PI;
  lprop[0].offset.forward  =      11.5;
  lprop[0].offset.forward  =       0.0;
  lprop[0].offset.sideward =       0.0;
  lprop[0].offset.rotation =       0.0;

  lprop[1].fov.start     =    M_PI_2;
  lprop[1].fov.end       =  3*M_PI_2;
  lprop[1].fov.delta     =      M_PI;
  lprop[1].offset.forward  =     -11.5;
  lprop[1].offset.sideward =       0.0;
  lprop[1].offset.rotation =       0.0;

  if (verbose) {
    fprintf( stderr, "#####################################################################\n" );
  }
  if ((fp = fopen( filename, "r")) == 0){
    if (verbose) {
      fprintf(stderr, "# ERROR: can't read script file %s\n",
	      filename );
    }
    return(-1);
  } else {
    if (verbose) {
      fprintf( stderr, "# read script file %s ...\n", filename );
    }
  }

  do{
    if (fscanf(fp, "%s", command) == EOF)
      FileEnd = TRUE;
    else{
      if (!strcmp( command, "#ROBOT") ){
	numPositions++;
	if (fscanf(fp, "%s %s %s", inp0, inp1, inp2 ) == EOF)
	  FileEnd = TRUE;
      } else if (!strcmp( command, "@SENS") ){
	numTimes++;
	if (fscanf(fp, "%d-%d-%d %d:%d:%s",
		   &day, &month, &year, &hour, &minute, inp0 ) == EOF)
	  FileEnd = TRUE;
      } else if (!strcmp( command, "#SONAR") ){
	numSonarScans++;
	if (fscanf(fp, "%d:", &numValues1) == EOF)
	  FileEnd = TRUE;
	else {
	  fgets( line, MAX_LINE_LENGTH,fp );
	}
      } else if (!strcmp( command, "#LASER") ){
	numLaserScans++;
	if (fscanf(fp, "%d %d:", &numValues1, &numValues2) == EOF)
	    FileEnd = TRUE;
	else {
	  fgets( line, MAX_LINE_LENGTH,fp );
	}
	if (numValues1>0)
	  numRLaserScans++;
	if (numValues2>0)
	  numRLaserScans++;
      } else {
	fgets(command,sizeof(command),fp);
      }
    }
  } while (!FileEnd);

  FileEnd = FALSE;

  rewind( fp );

  numEntries = numPositions + numRLaserScans;
  
  script->entry   =
    (logtools_entry_position_t *) malloc( numEntries * sizeof(logtools_entry_position_t) );
  script->psens  =
    (logtools_possens2_data_t *) malloc( numPositions * sizeof(logtools_possens2_data_t) );
  script->lsens  =
    (logtools_lasersens2_data_t *) malloc( numRLaserScans * sizeof(logtools_lasersens2_data_t) );

  script->numentries = numEntries;
  
  if (verbose) {
    fprintf( stderr, "#####################################################################\n" );
    fprintf( stderr, "# found %d times\n", numTimes );    
    fprintf( stderr, "# found %d positions\n", numPositions );    
    fprintf( stderr, "# found %d sonar scans\n", numSonarScans );    
    fprintf( stderr, "# found %d laser scans\n", numLaserScans );
    fprintf( stderr, "# found %d real laser scans\n", numRLaserScans );
    fprintf( stderr, "#####################################################################\n" );
  }

  cnt               = 0;
  numPositions      = 0;
  numRLaserScans    = 0;
  numSonarScans     = 0;
  numLaserScans     = 0;

  curTime.tv_sec = 0;
  curTime.tv_usec = 0;


  do{
    
    if (fscanf(fp, "%s", command) == EOF)
      FileEnd = TRUE;
    else{
      
      /* ****************************************************************
         ****************************************************************
         **                                                            **
         **                                                            **
	 **                ROBOT POSITON                               **
	 **                                                            **
	 **                                                            **
         ****************************************************************
	 **************************************************************** */
	 
      if (!strcmp( command, "#ROBOT") ){

	script->entry[cnt].type  = POSITION;
	script->entry[cnt].index = numPositions;
	if (fscanf(fp, "%s %s %s", inp0, inp1, inp2 ) == EOF)
	  FileEnd = TRUE;
	else {
	  /*
	    rpos.y = atof( inp0 );
	    rpos.x = -atof( inp1 );
	    rpos.o = deg2rad(atof( inp2 ));
	  */
	  rpos.x = atof( inp0 );
	  rpos.y = atof( inp1 );
	  if (rotation_90_minus) {
	    rpos.o = deg2rad(90.0-atof(inp2));
	  } else {
	    rpos.o = deg2rad(atof(inp2));
	  }
	  script->psens[numPositions].time = curTime;
	  script->psens[numPositions].rpos = rpos;
	  script->psens[numPositions].rvel = rvel;
	}
	numPositions++;
	cnt++;
	
      } else if (!strcmp( command, "#ROTATION_90_MINUS") ){
	rotation_90_minus = TRUE;
	if (verbose) {
	  fprintf( stderr, "# INFO: using ROTATION_90_MINUS mode\n" );
	}
	
      } else if (!strcmp( command, "#NO_ROTATION_90_MINUS") ){
	rotation_90_minus = FALSE;
	if (verbose) {
	  fprintf( stderr, "# INFO: using NO_ROTATION_90_MINUS mode\n" );
	}
	
      } else if (!strcmp( command, "@SENS") ){
	
        /* **************************************************************
           **************************************************************
           **                                                          **
	   **                                                          **
	   **                TIME                                      **
	   **                                                          **
	   **                                                          **
	   **************************************************************
	   ************************************************************** */

	  if (fscanf(fp, "%d-%d-%d %d:%d:%d.%s",
		     &day, &month, &year, &hour, &minute, &sec, inp0 ) == EOF)
	    FileEnd = TRUE;
	  else {
	    
	    if (year<1900) {
	      year+=100;
	    } else {
	      year-=1900;
	    }
	    dayTime.tm_year = year;
	    dayTime.tm_mon  = month;
	    dayTime.tm_mday = day;
	    dayTime.tm_hour = hour;
	    dayTime.tm_min  = minute;
	    dayTime.tm_sec  = sec;
	    snprintf( sstr, 80, "%s000000", inp0 );
	    strncpy( secstr, sstr, 6 );
	    curTime.tv_usec = (long) atoi(secstr);
	    curTime.tv_sec  = (long) mktime( &dayTime );
	  }

      } else if (!strcmp( command, "#LASER") ){
	
        /* ***************************************************************
           ***************************************************************
	   **                                                           **
	   **                                                           **
	   **                 LASER VALUES                              **
	   **                                                           **
	   **                                                           **
	   ***************************************************************
	   *************************************************************** */

	if (fscanf(fp, "%d %d:", &numValues1, &numValues2) == EOF)
	  FileEnd = TRUE;
	else {

	  if ( numValues1>0 ) {

	    script->entry[cnt].type  = LASER_VALUES;
	    script->entry[cnt].index = numRLaserScans;

	    if (numPositions>0) {
	      script->lsens[numRLaserScans].estpos =
		script->psens[numPositions-1].rpos;
	    } else {
	      script->lsens[numRLaserScans].estpos = npos;
	    }

	    script->lsens[numRLaserScans].coord            = NULL;

	    script->lsens[numRLaserScans].laser.time       = curTime;
	    script->lsens[numRLaserScans].id               = 0;
	    script->lsens[numRLaserScans].laser.numvalues  = numValues1;
	    script->lsens[numRLaserScans].laser.val        =
	      (float *) malloc( numValues1 * sizeof(float) );
	    script->lsens[numRLaserScans].laser.angle      =
	      (float *) malloc( numValues1 * sizeof(float) );
	    
	    angleDiff = lprop[0].fov.delta / (double) (numValues1-1);
	    script->lsens[numRLaserScans].laser.fov    = lprop[0].fov.delta;
	    script->lsens[numRLaserScans].laser.offset = lprop[0].offset;
	    for ( i=0; i<numValues1 ;i++) {
	      if (fscanf(fp, "%s", inp0) == EOF) {
		FileEnd = TRUE;
		break;
	      } else {
		script->lsens[numRLaserScans].laser.val[i]   = atof(inp0);
		script->lsens[numRLaserScans].laser.angle[i] =
		  lprop[0].fov.start+(i*angleDiff);
	      }
	    }
	    
	    lastFScan = numRLaserScans;
	    numRLaserScans++;
	    cnt++;

	  }
	  
	  if ( numValues2>0 ) {
	    
	    script->entry[cnt].type  = LASER_VALUES;
	    script->entry[cnt].index = numRLaserScans;

	    if (numPositions>0) {
	      script->lsens[numRLaserScans].estpos =
		script->psens[numPositions-1].rpos;
	    } else {
	      script->lsens[numRLaserScans].estpos = npos;
	    }
	  
	    script->lsens[numRLaserScans].coord            = NULL;

	    script->lsens[numRLaserScans].laser.time       = curTime;
	    script->lsens[numRLaserScans].id               = 1;
	    script->lsens[numRLaserScans].laser.numvalues  = numValues2;
	    script->lsens[numRLaserScans].laser.val        =
	      (float *) malloc( numValues2 * sizeof(float) );
	    script->lsens[numRLaserScans].laser.angle      =
	      (float *) malloc( numValues2 * sizeof(float) );
	    angleDiff = lprop[1].fov.delta / (double) (numValues2-1);
	    script->lsens[numRLaserScans].laser.fov    = lprop[0].fov.delta;
	    script->lsens[numRLaserScans].laser.offset = lprop[1].offset;
	    for ( i=0; i<numValues2 ;i++) {
	      if (fscanf(fp, "%s", inp0) == EOF) {
		FileEnd = TRUE;
		break;
	      } else {
		script->lsens[numRLaserScans].laser.val[i] =
		  atof(inp0);
		script->lsens[numRLaserScans].laser.angle[i] =
		  lprop[1].fov.start+(i*angleDiff);
	      }
	    }
	    lastRScan = numRLaserScans;
	    numRLaserScans++;
	    cnt++;
	  }
	}
	
      } else {
	if (!(command[0]=='%')){
	  /*
	    fprintf( stderr, "%s: unknown keyword %s\n",
	    prgname, command );
	    fclose(fp);
	    exit(0);
	  */
	} else {
	  fgets(command,sizeof(command),fp);
	}
      }
    }
  } while (!FileEnd);

  script->numentries    = cnt;
  script->numpositions  = numPositions;
  script->numlaserscans = numRLaserScans;
  script->nummarkers    = 0;
  
  return( 0 );
}
