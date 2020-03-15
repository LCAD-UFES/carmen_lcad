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

#include <sys/time.h>
#include <time.h>
#include <stdio.h>

void timespec_now(struct timespec *ts)
{
  struct timeval  tv;

  // get the current time
  gettimeofday(&tv, NULL);
  ts->tv_sec=tv.tv_sec;
  ts->tv_nsec=tv.tv_usec*1000;  
}

void timespec_addms(struct timespec *ts, long ms)
{
  int sec=ms/1000;
  ms=ms-sec*1000;

  // perform the addition
  ts->tv_nsec+=ms*1000000;

  // adjust the time
  ts->tv_sec+=ts->tv_nsec/1000000000 + sec;
  ts->tv_nsec=ts->tv_nsec%1000000000;
}

int timespec_compare(struct timespec *a, struct timespec *b)
{
  if (a->tv_sec!=b->tv_sec)
    return a->tv_sec-b->tv_sec;

  return a->tv_nsec-b->tv_nsec;
}

void timespec_print(struct timespec *a)
{
  printf("%li.%09li\n",a->tv_sec, a->tv_nsec);
}
