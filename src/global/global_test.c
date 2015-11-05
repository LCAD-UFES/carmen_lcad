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

#include "global.h"

void test_carmen_int_random(void)
{
  int index;
  int bins[100];
  int rand;

  memset(bins, 0, 100*sizeof(int));

  for (index = 0; index < 10000; index++) {
    rand = carmen_int_random(100);
    bins[rand]++;
  }

  for (index = 0; index < 100; index++) {
    if (fabs(bins[index] - 100) > 30)
      carmen_warn("Failed: %d %d\n", index, bins[index]);
  }
    
}

int main(int argc __attribute__ ((unused)), 
	 char *argv[] __attribute__ ((unused))) 
{
  test_carmen_int_random();
  
  return 0;
}
