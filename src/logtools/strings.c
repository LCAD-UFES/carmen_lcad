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
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>

#include <carmen/logtools.h>
#include "defines.h"

char *
logtools_str_get_valstr( char *str )
{
  char   * ptr;
  int      n, i = 0;
  
  n = strlen(str); i = 0;
  while (i<n) {
    if(str[i]=='{') {
      ptr = rindex(str,'}');
      if (ptr==NULL) {
	return(NULL);
      } else {
	ptr[0]='\0';
	return(logtools_str_get_valstr(&(str[i+1])));
      }
    } else if(str[i]=='(') {
      ptr = rindex(str,')');
      if (ptr==NULL) {
	return(NULL);
      } else {
	ptr[0]='\0';
	return(logtools_str_get_valstr(&(str[i+1])));
      }
    } else if(str[i]=='[') {
      ptr = rindex(str,']');
      if (ptr==NULL) {
	return(NULL);
      } else {
	ptr[0]='\0';
	return(logtools_str_get_valstr(&(str[i+1])));
      }
    } else if(str[i]==' '||str[i]=='\t') {
      i++;
    } else {
      n = strlen(&(str[i])); 
      while (n>=0) {
	if(str[i+n-1]==' ' || str[i+n-1]=='\t') {
	  n--;
	} else {
	  str[i+n]='\0';
	  return(&(str[i]));
	}
      }
      str[i+n]='\0';
      return(&(str[i]));
    }
  } 
  return(NULL);
}

char *
logtools_str_get_str( char *str )
{
  char   * ptr;
  int      n, i = 0;
  
  n = strlen(str); i = 0;
  while (i<n) {
    if(str[i]=='{') {
      ptr = rindex(str,'}');
      if (ptr==NULL) {
	return(NULL);
      } else {
	ptr[0]='\0';
	return(logtools_str_get_str(&(str[i+1])));
      }
    } else if(str[i]=='(') {
      ptr = rindex(str,')');
      if (ptr==NULL) {
	return(NULL);
      } else {
	ptr[0]='\0';
	return(logtools_str_get_str(&(str[i+1])));
      }
    } else if(str[i]=='[') {
      ptr = rindex(str,']');
      if (ptr==NULL) {
	return(NULL);
      } else {
	ptr[0]='\0';
	return(logtools_str_get_str(&(str[i+1])));
      }
    } else {
      return(&(str[i]));
    }
  } 
  return(NULL);
}


int
logtools_str_get_numbers( char *str, int num, ... )
{
  int          n, i = 0;
  va_list      argp;
  char         numstr[MAX_CMD_LENGTH];
  char         dummy[MAX_CMD_LENGTH];
  char         args[MAX_CMD_LENGTH];
  double     * ptr;

  if (strlen(str)>0 && num>0) {

    if (sscanf( str, "%[^0-9-]", dummy )==0) {
      /* it starts with numbers */
      strncpy( args, str, MAX_CMD_LENGTH );
    } else {
      /* cut off head */
      sscanf( str, "%[^0-9-]%[^:]", dummy, args );
    }
    
    va_start( argp, num );

    i=0;
    do {
      n = sscanf( args, "%[^,],%[^:]", numstr, dummy );
      ptr = va_arg(argp, double *);
      *ptr = atof(numstr);
      strncpy( args, dummy, MAX_CMD_LENGTH );
      i++;
    } while (i<num && n>1);
    if (i<num) {
      for (n=i; n<num; n++) {
        ptr = va_arg(argp, double *);
        *ptr = 0.0;
      }
    }
  }
  return(i);
}
