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

#include "arcnet.h"
#include "llc1.h"
#include "pc_d51.h"

unsigned char llc1_rbuf[512]; 
unsigned char llc1_sap_indication[16]; 
unsigned char llc1_xbuf[512]; 
unsigned char llc1_sap_state[16]; 
unsigned char llc1_gsap_state[16]; 
unsigned char llc1_gsap_indication[16]; 
unsigned char llc1_loop_back; 
struct LLC_MSG *llc1_stsap_ptr; 
unsigned char llc1_station_id; 
unsigned char llc1_stsap_state; 
unsigned char llc1_sbuf[512]; 
unsigned char llc1_station_state; 
unsigned char llc1_stsap_indication; 
struct LLC_MSG *llc1_sap_ptr[16]; 
struct LLC_MSG *llc1_gsap_ptr[16]; 

unsigned char llc1_send(struct LLC_MSG *request, unsigned char cmdresp);
void llc1_service_packet(unsigned char *ptr);

unsigned char llc1_request(unsigned char lssap, unsigned char ldsap, 
			   unsigned char event, struct LLC_MSG *request)
{
  unsigned char status = 0; 
  unsigned char i; 
  unsigned char temp; 
  unsigned char *bufptr; 

  if (llc1_station_state != 0x1e) {
    if ((event != 0xb) && (event != 0xc)) {
      return 0x1f;
    } 
  } else { 
    status = 0;
  } 

  request->ssap = lssap << 2;

  if (ldsap != 255) {
    request->dsap = ldsap << 2;
  } else { 
    request->dsap = 0xff;
  } 
  
  request->event = event;
  if (request->group) {
    request->dsap |= 1;
  }
  
  switch (event - 1) {
  case 0:
    if ((lssap != 0) && (lssap < 0x11)) {
      if (request->group != 0) {
	llc1_gsap_state[lssap] = 0x1e;
	llc1_gsap_ptr[lssap] = request;
      } else {
	llc1_sap_state[lssap] = 0x1e;
	llc1_sap_ptr[lssap] = request;
      } 
    } else { 
      status = 0x20;
    } 
    break;
    
  case 1: 
    if ((lssap != 0) && (lssap < 0x11)) {
      if (request->group != 0) {
	llc1_gsap_state[lssap] = 0x1f;
      } else {
	llc1_sap_state[lssap] = 0x1f;
      } 
    } 
    break;
    
  case 2: 
    request->control = 0xbf;
    bufptr = request->msgptr;
    *bufptr++ = 0x81;
    *bufptr++ = 0x01;
    *bufptr = 0x02;
    request->msbcount = 0;
    request->lsbcount = 3;
    status = llc1_send(request, 0);
    break;
    
  case 3: 
    request->control = 0xf3;
    status = llc1_send(request, 0);
    break;
    
  case 20: 
    request->msbcount = 0;
    request->lsbcount = 0;
    request->control = 0x83;
    status = llc1_send(request, 0);
    break;
    
  case 21: 
    if (request->group != 0) {
      llc1_gsap_indication[ldsap] = 4;
    } else { 
      llc1_sap_indication[ldsap] = 4;
    } 
    temp = request->dsap;
    request->dsap = request->ssap;
    request->ssap = temp & 0xfe;
    request->msbcount = 0;
    request->lsbcount = 0;
    request->control = 0x83;
    status = llc1_send(request, 1);
    llc1_station_state = 0x1f;
    d51_exit();
    break;
    
  case 22: 
    if (request->group != 0) {
      llc1_gsap_indication[ldsap] = 5;
    } else { 
      llc1_sap_indication[ldsap] = 5;
    } 
    break;
    
  case 4: 
    if (request->group != 0) {
      llc1_gsap_indication[ldsap] = 3;
    } else { 
      llc1_sap_indication[ldsap] = 3;
    }
    break;
    
  case 5: 
    request->control = 3;
    status = llc1_send(request, 0);
    break;
    
  case 6: 
    temp = request->dsap;
    request->dsap = request->ssap;
    request->ssap = temp & 0xfe;
    bufptr = request->msgptr;
    *bufptr++ = 0x81;
    *bufptr++ = 0x01;
    *bufptr = 0x02;
    request->msbcount = 0;
    request->lsbcount = 3;
    request->group = 0;
    request->control ^= 0x10;
    status = llc1_send(request, 1);    
    break;
    
  case 7: 
    if (request->group != 0) {
      llc1_gsap_indication[ldsap] = 1;
    } else { 
      llc1_sap_indication[ldsap] = 1;
    } 
    break;
    
  case 8: 
    temp = request->dsap;
    request->dsap = request->ssap;
    request->ssap = temp & 0xfe;
    request->control ^= 0x10;
    status = llc1_send(request, 1);
    break;
    
  case 9: 
    if (request->group != 0) {
      llc1_gsap_indication[ldsap] = 2;
    } else { 
      llc1_sap_indication[ldsap] = 2;
    } 
    break;
    
  case 10: 
  case 11:
    request->msgptr = llc1_sbuf;
    for (i = 0; i < 16; i++) {
      llc1_sap_state[i] = 0x1f;
      llc1_gsap_state[i] = 0x1f;
    } 
    llc1_stsap_state = 0x1e;
    llc1_stsap_ptr = request;
    llc1_station_id = d51_params[16];
    request->dstation = llc1_station_id;
    if (llc1_station_id != 0) {
      llc1_station_state = 0x1e;
      status = 0;
    } else { 
      llc1_station_state = 0x1f;
      status = 0x1f;
    } 
    break;
    
  case 14: 
    request->dsap = request->ssap;
    request->ssap = 0;
    bufptr = request->msgptr;
    *bufptr++ = 0x81;
    *bufptr++ = 0x01;
    *bufptr = 0x02;
    request->msbcount = 0;
    request->lsbcount = 3;
    status = llc1_send(request, 1);
    break;
    
  case 17: 
    request->dsap = request->ssap;
    request->ssap = 0;
    status = llc1_send(request, 1);    
    break;
    
  case 18: 
    llc1_station_state = 0x1f;
    d51_exit();
    break;
    
  case 19: 
    if (request->ssap == 0) {
      status = llc1_station_state;
    } else { 
      if (request->group != 0) {
	status = llc1_gsap_state[lssap];
      } else { 
	status = llc1_sap_state[lssap];
      } 
    } 
    break;
    
  case 12:
  case 13:
  case 15:
  case 16:
  default: 
    status = 5;
    break;

  } 
  
  return status;
}

unsigned char llc1_indication(unsigned char lsap)
{
  
  unsigned char status; 

  if (llc1_station_state != 0x1e)
    return 0;
  
  status = llc1_sap_indication[lsap];
  if (status != 0) {
    llc1_sap_indication[lsap] = 0;
  }
  
  return status;
}

unsigned char llc1_group_indication(unsigned char lsap)
{
  
  unsigned char status; 

  if (llc1_station_state != 0x1e)
    return 0;

  status = llc1_gsap_indication[lsap];
  if (status != 0) 
    llc1_gsap_indication[lsap] = 0;
  
  return status;
}

unsigned char llc1_service()
{
  unsigned char status; 
  unsigned int count; 

  if (llc1_station_state != 0x1e)
    return 0x1f;

  if (llc1_loop_back == 1) {
    count = llc1_xbuf[2];
    count <<= 8;
    count += llc1_xbuf[3];
    count += 8;
    while (count != 0) {
      llc1_rbuf[count] = llc1_xbuf[count];
      count--;
    } 
    
    status = 0;
    llc1_loop_back = 0;
  } else { 
    status = d51_read_packet(0, llc1_rbuf);
  } 

  if (status == 0) {
    if (llc1_rbuf[4] == 0) 
      llc1_service_packet(llc1_rbuf);
    else
      status = 6;
  } 
  
  return status;
}


void llc1_service_packet(unsigned char *ptr)
{
  unsigned char ldsap; 
  unsigned char lssap; 
  unsigned char control; 
  unsigned char event; 
  unsigned char dsap; 
  unsigned char ssap; 
  unsigned char num_sap; 
  unsigned char sap_index = 0; 
  unsigned int count; 
  unsigned int offset; 
  struct LLC_MSG *local_sap = 0; 

  dsap = ptr[5];
  ldsap = dsap >> 2;
  ssap = ptr[6];
  lssap = ssap >> 2;
  control = ptr[7];
  
  if (dsap != 0xff) {
    if ((dsap & 3) == 0) {
      if (llc1_sap_state[ldsap] == 0x1f) {
	return ;
      } else { 
	local_sap = llc1_sap_ptr[ldsap];
      } 
    } else { 
      if ((dsap & 3) == 1) {
	if (llc1_gsap_state[ldsap] == 0x1f) {
	  return ;
	} else { 
	  local_sap = llc1_gsap_ptr[ldsap];
	} 
      } 
    } 

    num_sap = 1;
  } else { 
    num_sap = 32;
    sap_index = 0; 
    while (num_sap != 0) {
      if (num_sap >= 16) {
	if (llc1_gsap_state[sap_index] == 0x1e) {
	  local_sap = llc1_gsap_ptr[sap_index];
	  ldsap = sap_index;
	  dsap = ldsap << 2;
	  break;
	} 
      } else {
	if (llc1_sap_state[sap_index] == 0x1e) {
	  local_sap = llc1_sap_ptr[sap_index];
	  ldsap = sap_index;
	  dsap = ldsap << 2;
	  break;
	} 
      } 
      num_sap--;
      if (num_sap == 0xf) {
	sap_index = 0;
      } else { 
	sap_index++;
      } 
    } 
  } 
  
  while (num_sap != 0) {
    if ((control == 0xaf) || (control == 0xbf)) {
      if (dsap == 0) {
	local_sap->event = 0xf;
      } else { 
	if ((ssap & 1) == 0) {
	  local_sap->event = 7;
	} else { 
	  local_sap->event = 8;
	} 
      } 
    } else { 
      if ((control == 0xe3) || (control == 0xf3)) {
	if (dsap == 0) {
	  local_sap->event = 0x12;
	} else { 
	  if ((ssap & 1) == 0) {
	    local_sap->event = 9;
	  } else { 
	    local_sap->event = 10;
	  } 
	} 
	count = ptr[2];
	count <<= 8;
	count += ptr[3];
	if (count > 3)
	  count -= 4;
	else
	  count = 0;
	
	local_sap->msbcount = count >> 8;
	local_sap->lsbcount = count & 0xff;
	offset = (int) local_sap->msgptr;
	do {
	  count--;
	  *(((unsigned char *) offset) + count) = ptr[count + 8];
	} while (count != 0);
	
      } else { 
	if (control == 0x83) {
	  if ((ssap & 1) == 0) {
	    local_sap->event = 0x16;
	  } else { 
	    local_sap->event = 0x17;
	  } 
	} else { 
	  if ((control == 0x3) || (control == 0x13)) {
	    local_sap->event = 5;
	    count = ptr[2];
	    count <<= 8;
	    count += ptr[3];
	    if (count > 3)
	      count -= 4;
	    else
	      count = 0;
	    
	    local_sap->msbcount = count >> 8;
	    local_sap->lsbcount = count & 0xff;
	    offset = (int) local_sap->msgptr;
	    do {
	      count--;
	      *(((unsigned char *) offset) + count) = ptr[count + 8];
	    } while (count != 0);
	    
	  } else { 
	    return ;
	  }
	} 
      } 
    } 

    local_sap->dsap = dsap;
    local_sap->ssap = ssap;
    local_sap->control = control;
    local_sap->dstation = ptr[0];
    event = local_sap->event;
    llc1_request(lssap, ldsap, event, local_sap);
    num_sap--;
    while (num_sap != 0) {
      if (num_sap >= 16) {
	if (llc1_gsap_state[sap_index] == 0x1e) {
	  local_sap = llc1_gsap_ptr[sap_index];
	  ldsap = sap_index;
	  dsap = ldsap << 2;
	  break;
	} 
      } else { 
	if (llc1_sap_state[sap_index] == 0x1e) {
	  local_sap = llc1_sap_ptr[sap_index];
	  ldsap = sap_index;
	  dsap = ldsap << 2;
	  break;
	} 
      } 
      num_sap--;
      if (num_sap == 0xf) {
	sap_index = 0;
      } else { 
	sap_index++;
      } 
    } 
  } 
}

unsigned char llc1_send(struct LLC_MSG *request, unsigned char cmdresp)
{
  
  unsigned int count; 
  unsigned int ncount; 
  unsigned int i; 
  unsigned int offset; 
  unsigned char status; 

  llc1_xbuf[0] = llc1_station_id;
  if (request->group != 0) {
    llc1_xbuf[1] = 0;
  } else { 
    llc1_xbuf[1] = request->dstation;
  } 

  count = request->msbcount;
  count <<= 8;
  count += request->lsbcount;
  ncount = count + 4;
  
  llc1_xbuf[2] = ncount >> 8;
  llc1_xbuf[3] = ncount & 0xff;
  llc1_xbuf[4] = 0;
  llc1_xbuf[5] = request->dsap;
  llc1_xbuf[6] = request->ssap;
  
  if (cmdresp == 1) {
    llc1_xbuf[6] |= 1;
  } else { 
    llc1_xbuf[6] &= 0xfe;
  } 
  
  llc1_xbuf[7] = request->control;
  offset = (int) request->msgptr;
  for (i = 0; i < count; i++) {
    llc1_xbuf[i + 8] = ((unsigned char *) offset)[i];
  } 
  
  if (llc1_xbuf[0] == llc1_xbuf[1]) {
    llc1_loop_back = 1;
    status = 0;
  } else { 
    llc1_loop_back = 0;
    status = d51_write_packet(llc1_xbuf);
  } 
  
  return status;
}

