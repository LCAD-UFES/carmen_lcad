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

#ifndef LLC1_H
#define LLC1_H

#ifdef __cplusplus
extern "C" {
#endif

struct LLC_MSG
{
  unsigned char event;
  unsigned char dstation;
  unsigned char ssap;
  unsigned char dsap;
  unsigned char group;
  unsigned char control;
  unsigned char msbcount;
  unsigned char lsbcount;
  unsigned char *msgptr;
};

unsigned char llc1_service();
unsigned char llc1_request(unsigned char lssap, unsigned char ldsap, 
			   unsigned char event, struct LLC_MSG *request);
unsigned char llc1_indication(unsigned char lsap);

#ifdef __cplusplus
}
#endif

#endif
