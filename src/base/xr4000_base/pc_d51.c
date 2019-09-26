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
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>

unsigned char d51_status = 0; 
unsigned char d51_diag = 0; 
unsigned char d51_nextid = 0; 
unsigned char d51_cmd = 0; 
unsigned char d51_config = 0; 
unsigned char d51_tentid = 0; 
unsigned char d51_setup = 0; 
unsigned char d51_in_page = 0; 
unsigned char d51_out_page = 0; 
unsigned char d51_broadcast = 0; 
unsigned char d51_rx_page = 0; 
unsigned char d51_write_stat = -1; 
unsigned char d51_write_pending = 0; 


unsigned char inbuf[4096]; 
unsigned char d51_params[26]; 
unsigned char j; 
unsigned char d51_net_map[32]; 
unsigned int d51_diag_cnt[13]; 
unsigned int inbuf_in; 
unsigned char i; 
unsigned char buffer[16]; 
unsigned int inbuf_out; 
unsigned char ta_bit; 
unsigned char dummy; 
unsigned char ri_bit; 

void d51_clear_diag();
unsigned char d51_free_node();
void d51_tokens(unsigned char ntokens);
void d51_check_int();
void d51_get_qentry(unsigned char *ptr);
void read_data(unsigned char page, unsigned char offset, unsigned char count, 
	       unsigned char shortlong, unsigned char *user_buffer);
void write_data(unsigned char page, unsigned char offset, unsigned char count,
		unsigned char shortlong, unsigned char *user_buffer);
unsigned char check_network_status();

void DELAY10US(unsigned int us)
{
  struct timeval tv; 
  long unsigned int sec; 
  long unsigned int usec; 
  long unsigned int norm_usec; 

  gettimeofday(&tv, NULL);
  sec = tv.tv_sec;
  usec = tv.tv_usec;

  do {
    gettimeofday(&tv, NULL);
    norm_usec = (tv.tv_sec - sec) * 1000000 + (tv.tv_usec - usec);
  } while ((10 * us) > norm_usec);
}

void DELAYMS(unsigned int ms)
{
  struct timeval tv; 
  long unsigned int sec; 
  long unsigned int usec; 
  long unsigned int norm_usec; 

  gettimeofday(&tv, NULL);
  sec = tv.tv_sec;
  usec = tv.tv_usec;

  do {
    gettimeofday(&tv, NULL);
    norm_usec = (tv.tv_sec - sec) * 1000000 + (tv.tv_usec - usec);
  } while ((1000 * ms) > norm_usec);
}

int smc_fd = 0;

void smc_out(unsigned char reg, unsigned char val)
{
  
  

  lseek(smc_fd, reg + 0x2e0, 0);
  write(smc_fd, &val, 1);
  
}

unsigned char smc_in(unsigned char reg)
{
  
  unsigned char val; 

  lseek(smc_fd, reg + 0x2e0, 0);
  read(smc_fd, &val, 1);
  return val;
}

unsigned char d51_init()
{
  unsigned char return_code = 0; 
  unsigned int click; 


  smc_fd = open("/dev/port", 2);
  if (smc_fd < 0) {
    return -1;
  } 
  
  inbuf_in = 0;
  inbuf_out = 0;
  if (d51_params[13] == 0) {
    ri_bit = 0x80;
    ta_bit = 0x01;
  } else { 
    ri_bit = 0x40;
    ta_bit = 0x20;
  } 
  
  d51_clear_diag();
  smc_out(6, smc_in(6) & 0xfc);
  smc_out(6, smc_in(6) | 0x02);

  smc_out(7, smc_in(7) | d51_params[17]);
  smc_out(7, smc_in(7) | d51_params[18]);
  smc_out(7, smc_in(7) | d51_params[19]);
  smc_out(7, smc_in(7) | d51_params[20]);
  smc_out(7, smc_in(7) | d51_params[21]);
  smc_out(7, smc_in(7) | d51_params[22]);

  smc_out(6, smc_in(6) | d51_params[13]);
  smc_out(6, smc_in(6) | d51_params[14]);
  smc_out(6, smc_in(6) | d51_params[15]);
  smc_out(6, smc_in(6) & 0xfc);
  smc_out(6, smc_in(6) | 0x01);
  smc_out(7, 0xfe);
  smc_out(6, smc_in(6) | 0x80);
  DELAYMS(10);
  smc_out(6, smc_in(6) & 0x7f);
  
  click = 0;
  buffer[0] = 0;
  buffer[1] = 0;
  while ((buffer[0] != 0xd1) || (buffer[1] != 0xfe)) {
    read_data(0, 0, 2, 0, buffer);
    DELAYMS(10);
    click++;
    if (click > 1000) {
      return 2;
    } 
  } 
  
  smc_out(1, 0x1e);
  
  if (d51_params[0] == 0) {
    smc_out(6, smc_in(6) | 0x20);
    dummy = d51_free_node();
    smc_out(6, smc_in(6) & 0xdf);
    if (dummy == 0) {
      return 0x15;
    } else { 
      smc_out(6, smc_in(6) & 0xfc);
      smc_out(6, smc_in(6) | 0x01);
      smc_out(7, dummy);
    } 
    
    d51_tokens(1);
    if ((smc_in(1) & 0x40) != 0) {
      return_code = 0x0f;
    } else { 
      return_code = 0;
    }    
  } else { 
    if (d51_params[0] == 1) {
      switch (check_network_status()) {
      case 0x00: 
	smc_out(6, smc_in(6) & 0xfc);
	smc_out(6, smc_in(6) | 0x01);
	smc_out(7, d51_params[16]);
	d51_tokens(1);
	return_code = 0;
	break;
	
      case 0x08: 
      case 0x0e: 
      case 0x14: 
	smc_out(6, smc_in(6) & 0xfc);
	smc_out(6, smc_in(6) | 0x01);
	smc_out(7, d51_params[16]);
	return_code = 0;
	break;

      default: 
	return -1;
      } 
    } 
  } 

  smc_out(1, 0x1e);
  
  if (d51_params[12] == 0) {
    smc_out(1, 0x05);
  } else { 
    smc_out(1, 0x0d);
  } 

  d51_in_page = 0;
  d51_rx_page = 0;
 
  d51_out_page = d51_params[9];
  d51_write_pending = 0;
  smc_out(6, smc_in(6) | 0x20);
  smc_out(1, (d51_in_page << 3) | 0x04 | d51_params[11]);
  if (d51_params[13] == 0x40) {
    smc_out(1, ((d51_in_page + 1) << 3) | 0x04 | d51_params[11]);
  } 

  return return_code;
}

unsigned char d51_read_packet(unsigned char wait_flag, unsigned char *data_ptr)
{
  
  unsigned char status; 

  if (wait_flag == 1) {
    do {
      d51_check_int();
    } while ((d51_status & ri_bit) == 0);
  } else { 
    if (wait_flag == 0) {
      d51_check_int();
      if ((d51_status & ri_bit) == 0)
	status = 7;
    } else { 
      return -1;
    } 
  } 

  if (inbuf_in != inbuf_out) {
    status = 0;
    d51_get_qentry(data_ptr);
  } else { 
    status = 7;
  } 
  
  return status;
}

unsigned char d51_write_packet(unsigned char *data_ptr)
{
  unsigned char count; 
  unsigned char offset; 
  unsigned char status; 
  unsigned char counts[2]; 

  write_data(d51_out_page, 0, 2, 0, data_ptr);
  data_ptr++;
  if (*data_ptr != 0) {
    d51_broadcast = 0;
  } else { 
    d51_broadcast = 1;
  } 
  
  data_ptr++;
  if (*data_ptr != 0) {
    if (d51_params[12] != 1) {
      return 0x12;
    } 
    
    count = -data_ptr[1];
    data_ptr++;
    if (count == 0) {
      return 9;
    } else { 
      offset = ~(*data_ptr) + 1;
      counts[0] = 0;
      counts[1] = offset;
      write_data(d51_out_page, 2, 2, 0, counts);
      data_ptr++;
      write_data(d51_out_page, offset, *(data_ptr - 1), 1, data_ptr);
    } 
  } else { 
    data_ptr++;
    count = *data_ptr;
    if ((count == 0) || (count > 0xfd)) {
      return 9;
    } else { 
      offset = ~count + 1;
      counts[0] = offset;
      write_data(d51_out_page, 2, 1, 0, counts);
      data_ptr++;
      write_data(d51_out_page, offset, count, 0, data_ptr);
    } 
  } 
  
  d51_diag_cnt[7] = d51_params[5];
  if (d51_params[8] == 1) {
    while ((smc_in(0) & ta_bit) == 0)
      ;
  } else { 
    if ((smc_in(0) & ta_bit) == 0) {
      return 0x0a;
    } 
  } 
  
  smc_out(1, (d51_out_page << 3) | 3);
  d51_out_page++;
  if (d51_out_page > 1) {    
    d51_out_page = d51_params[9];
  } 
  
  d51_write_pending = 1;
  d51_diag_cnt[4]++;
  if (d51_params[7] == 1) {
    d51_write_stat = -1;
    do {
      d51_check_int();
    } while (!((d51_write_stat == 0x13) || (d51_write_stat == 0)));
    return d51_write_stat;
  } 
  
  status = 0;
  return status;
}

void d51_get_qentry(unsigned char *ptr)
{
  unsigned int index; 
  unsigned int count; 
  unsigned int i2; 

  index = inbuf_out;
  i2 = 0;
  count = 2; 
  while (count != 0) {
    ptr[i2] = inbuf[index];
    index++;
    i2++;
    count--;
    if (index > 4095) {
      index = 0;
    } 
    if (i2 == 2) {
      count = inbuf[index];
      count <<= 8;
      if (index + 1 > 4095) {
	count |= inbuf[0];
      } else { 
	count |= inbuf[index + 1];
      }
      count += 2;
    } 
  } 
  
  inbuf_out = index;
}

unsigned char d51_network_map()
{
  unsigned char recon_cnt; 
  unsigned char aux_reg; 
  unsigned char mask; 
  unsigned char status; 
  int aux; 
  unsigned char a; 
  unsigned char b; 
  unsigned char c;
  unsigned char d; 
  
  for (aux = 0; aux < 32; aux++) {
    d51_net_map[aux] = 0;
  } 
  status = check_network_status();
  smc_out(6, smc_in(6) & 0xfc);
  smc_out(6, smc_in(6) | 0x01);
  mask = 1 << (smc_in(7) & 0x07);
  d = (smc_in(7) >> 3);
  d51_net_map[d] |= mask;
  
  if (status != 0) {
    return status;
  } 
  
  smc_out(6, smc_in(6) & 0xfc);
  smc_out(6, smc_in(6) | 0x03);
  mask = 1 << (smc_in(7) & 0x07);
  a = smc_in(7) >> 3;
  d51_net_map[a] |= mask;
  
  smc_out(6, smc_in(6) & 0xfc);
  smc_out(6, smc_in(6));
  
  if ((smc_in(6) & 0x20) == 0) {
    smc_out(7, 1);
    while (smc_in(7) != 0) {
      dummy = smc_in(1);
      DELAY10US(10);
      aux_reg = smc_in(1);
      
      if (((aux_reg & 0x10) == 0) || ((aux_reg & 0x20) == 0)) {
	return 0x16;
      } 
      dummy = smc_in(1);
      smc_out(1, 0x1e);
      DELAYMS(10);
      aux_reg = smc_in(1);
      if (((aux_reg & 0x10) != 0) && ((aux_reg & 0x20) != 0) && 
	  ((aux_reg & 0x04) != 0)) {
	mask = 1 << (smc_in(7) & 0x07); 
	b = smc_in(7) >> 3;
	d51_net_map[b] |= mask;
      } 
      smc_out(7, smc_in(7) + 1);
    } 
  } else { 
    smc_out(7, 1);
    while (smc_in(7) != 0) {
      dummy = smc_in(1);
      aux_reg = smc_in(1);
      while ((aux_reg & 0x40) == 0) {	
	aux_reg = smc_in(1);
      } 
      dummy = smc_in(1);      
      smc_out(1, 0x1e);
      recon_cnt = 0;
      aux_reg = smc_in(1);
      while (!(((aux_reg & 0x40) != 0) || (recon_cnt > 3) || 
	       ((aux_reg & 0x80) != 0))) {
	if ((smc_in(0) & 0x04) != 0) {
	  recon_cnt++;
	  smc_out(1, 0x1e);
	} 
	if ((aux_reg & 0x04) != 0) {
	  mask = 1 << (smc_in(7) & 0x07);
	  c = smc_in(7) >> 3;
	  d51_net_map[c] |= mask;
	  break;
	} 
	  
	aux_reg = smc_in(1);
      } 
      smc_out(7, smc_in(7) + 1);
    } 
  } 
  
  return status;
}

unsigned char d51_free_node()
{
  unsigned char mask; 
  unsigned char i2; 

  if (d51_network_map() == 0) {
    i2 = 1; 
    while (i2 != 0) {
      mask = 1 << (i2 & 0x07);
      d51_net_map[smc_in(7) >> 3] & (mask == 0) ? 0x01 : 0xff;
      /* WARNING - This appears broken */
      return i2;

      i2++;
    } 
  } 
  
  return 0;
}

void d51_clear_diag()
{
  int i2; 

  for (i2 = 0; i2 < 13; i2++) {
    d51_diag_cnt[i2] = 0;
  } 
  d51_diag_cnt[7] = d51_params[5];
}

void d51_tokens(unsigned char ntokens)
{
  
  unsigned char recon_cnt; 
  unsigned char aux_reg; 

  dummy = smc_in(1);
  smc_out(1, 0x1e);
  while (ntokens--) {
    recon_cnt = 0;
    aux_reg = smc_in(1);
    while (!(((aux_reg & 0x40) != 0) || (recon_cnt > 3) || 
	     ((aux_reg & 0x80) != 0))) {
      if ((smc_in(0) & 0x04) != 0) {
	recon_cnt++;
	smc_out(1, 0x1e);
      } 
      aux_reg = smc_in(1);
    } 
  } 
}

void d51_exit()
{
  smc_out(1, 1);
  smc_out(1, 2);
  smc_out(6, smc_in(6) & 0xdf);
  if (d51_params[2] == 1) {
    while (1) 
      ;
  }  
} 

void d51_check_int()
{
  unsigned char count[2]; 
  unsigned char offset; 
  unsigned int nbytes; 
  unsigned char good_count; 
  unsigned int address; 
  unsigned int index; 
  unsigned int old_index; 
  unsigned int i2; 
  unsigned int cnt; 
  unsigned int temp; 

  good_count = 0;
  d51_status = smc_in(0);
  d51_diag = smc_in(1);
  
  if ((d51_status & ri_bit) != 0) {
    if ((d51_params[4] & 0x80) != 0) {
      if (d51_params[13] == 0x40) {
	smc_out(1, 8);
      } 
      offset = 0;
      address = (d51_rx_page << 9) + offset;
      smc_out(2, (address >> 8) | 0xc0);
      smc_out(3, address);
      index = inbuf_in;
      old_index = inbuf_in;
      for (i2 = 0; i2 <= 1; i2++) {
	inbuf[index] = smc_in(4);
	index++;
	if (index > 4095) {
	  index = 0;
	} 
      } 
      count[0] = smc_in(4);
      count[1] = smc_in(4);
      if (count[0] == 0) {
	offset = count[1];
	nbytes = 0x200 - offset;
	cnt = nbytes + 4;
	temp = index + 1;
	for (i2 = 1; i2 <= cnt; i2++) {
	  if (temp != inbuf_out) {
	    temp++;
	    if (temp > 4095) {
	      temp = 0;
	    } 
	    good_count = 1;
	  } else { 
	    good_count = 0;
	    break;
	  } 
	} 
	if (good_count == 1) {
	  inbuf[index] = 1;
	  index++;
	  if (index > 4095) {
	    index = 0;
	  } 
	  inbuf[index] = nbytes;
	  index++;
	  if (index > 4095) {
	    index = 0;
	  } 
	  address = (d51_rx_page << 9) + offset;
	  smc_out(2, (address >> 8) | 0xc0);
	  smc_out(3, address);
	  while (nbytes != 0) {
	    inbuf[index] = smc_in(4);
	    index++;
	    nbytes--;
	    if (index > 4095) {
	      index = 0;
	    } 
	    if (index == inbuf_out)
	      break;
	  } 
	} else { 
	  d51_diag_cnt[8]++;
	} 
      } else { 
	offset = count[0];
	nbytes = (unsigned char) (((unsigned char) 
				   ~((unsigned char) count[0])) + 1);
	cnt = nbytes + 4;
	temp = index + 1;
	for (i2 = 0; i2 <= cnt; i2++) {
	  if (temp != inbuf_out) {
	    temp++;
	    if (temp > 4095) {
	      temp = 0;
	    } 
	    good_count = 1;
	  } else { 
	    good_count = 0;
	    break;
	  } 
	} 
	
	if (good_count == 1) {
	  inbuf[index] = 0;
	  index++;
	  if (index > 4095) {
	    index = 0;
	  } 
	  inbuf[index] = nbytes;
	  index++;
	  if (index > 4095) {
	    index = 0;
	  } 
	  address = (d51_rx_page << 9) + offset;
	  smc_out(2, (address >> 8) | 0xc0);
	  smc_out(3, address);
	  while (nbytes != 0) {
	    inbuf[index] = smc_in(4);
	    index++;
	    nbytes--;
	    if (index > 4095) {
	      index = 0;
	    } 
	    if (index == inbuf_out)
	      break;
	  } 
	} else { 
	  d51_diag_cnt[8]++;
	} 
      } 
      if (good_count != 1) {
	inbuf_in = old_index;
      } else { 
	if (index != inbuf_out) {
	  inbuf_in = index;
	} 
      } 
      d51_rx_page = d51_in_page;
      d51_in_page++;
      if (d51_in_page >= d51_params[9])
	d51_in_page = 0;
      if (d51_params[13] == 0) {
	d51_rx_page = d51_in_page;
      } 
      smc_out(1, (d51_in_page << 3) | 0x04 | d51_params[11]);
      d51_diag_cnt[0]++;
    } 
  } 
  
  if (((d51_status & ta_bit) != 0) && ((d51_params[4] & 0x01) != 0) &&
      (d51_write_pending == 1)) {
    if (d51_params[13] == 0x40) {
      smc_out(1, 0);
    } 
    if (d51_broadcast != 0) {
      d51_write_stat = 0;
      d51_diag_cnt[9]++;
    } else { 
      if ((d51_status & 0x02) != 0) {
	d51_write_stat = 0;
	d51_diag_cnt[9]++;
      } else { 
	d51_write_stat = 0x13;
	d51_diag_cnt[10]++;
      } 
    } 
    d51_write_pending = 0;
  } 

  if (((d51_status & 0x04) != 0) && ((d51_params[4] & 0x04) != 0)) {
    smc_out(1, 0x16);
    d51_diag_cnt[2]++;
    if ((d51_diag & 0x80) != 0) 
      d51_diag_cnt[6]++;
  } 
  
  if (((d51_diag & 0x08) != 0) && ((d51_params[4] & 0x08) != 0) && 
      (d51_write_pending == 1)) {
    smc_out(1, 0x0e);
    
    if (d51_diag_cnt[7] == 0) {
      if (d51_params[6] != 0) {
	smc_out(1, 1);
	d51_write_stat = 0x0c;
	d51_diag_cnt[10]++;
	d51_diag_cnt[7] = d51_params[5];
      } else { 
	d51_write_stat = 0x11;
      } 
    } else { 
      d51_diag_cnt[7]--;
    } 
    d51_diag_cnt[1]++;
  } 
  
  if (((d51_diag & 0x02) != 0) && ((d51_params[4] & 0x02) != 0)) {
    smc_out(6, smc_in(6) & 0xfc);
    smc_out(6, smc_in(6) | 0x03);
    d51_nextid = smc_in(7);
    dummy = smc_in(1);
    d51_diag_cnt[3]++;
  } 
} 

void read_data(unsigned char page, unsigned char offset, unsigned char count, 
	       unsigned char shortlong, unsigned char *user_buffer)
{
  
  
  
  
  unsigned int address; 

  address = (page << 9) + offset;
  smc_out(2, (address >> 8) | 0xc0);
  smc_out(3, address);
  
  while (count != 0) {
    *user_buffer = smc_in(4);
    user_buffer++;
    count--;
  } 
  
  if (shortlong == 1) {
    count = 0;
    do {
      *user_buffer = smc_in(4);
      user_buffer++;
      count--;
    } while (count != 0);
  } 
} 

void write_data(unsigned char page, unsigned char offset, unsigned char count,
		unsigned char shortlong, unsigned char *user_buffer)
{
  
  
  
  
  unsigned int address; 

  address = (page << 9) + offset;
  smc_out(2, ((address >> 8) | 0x40) & 0x7f);
  smc_out(3, address);
  while (count-- != 0) {
    smc_out(4, *user_buffer++);
  } 

  if (shortlong == 1) {
    count = 0;
    do {
      smc_out(4, *user_buffer);
      user_buffer++;
      count--;
    } while (count != 0);
  } 
} 

unsigned char check_network_status()
{
  unsigned char status; 
  unsigned char aux; 
  unsigned char consec_recon; 
  unsigned int cnt; 

  status = 0;
  consec_recon = 0;
  cnt = 0;
  dummy = smc_in(1);
  smc_out(1, 0x1e);
  DELAYMS(200);
  DELAYMS(200);
  DELAYMS(200);
  DELAYMS(240);
  aux = smc_in(1);
  if ((smc_in(6) & 0x20) == 0) {
    if (((aux & 0x40) == 0) && ((aux & 0x20) != 0) && ((aux & 0x10) != 0)) {
      status = 0;
    } else { 
      if ((aux & 0x40) != 0) {
	status = 0x0d;
      } else { 
	if (((aux & 0x10) == 0) && ((aux & 0x20) != 0)) {
	  status = 0x08;
	} else { 
	  if (((aux & 0x10) == 0) && ((aux & 0x20) == 0)) {
	    status = 0x14;
	  } else { 
	    status = -1;
	  } 
	} 
      } 
    } 
  } else { 
    if (((aux & 0x20) != 0) && ((aux & 0x10) != 0)) {
      status = 0;
    } else { 
      if (((aux & 0x10) == 0) && ((aux & 0x20) != 0)) {
	status = 8;
      } else { 
	if (((aux & 0x10) == 0) && ((aux & 0x20) == 0)) {
	  status = 0x14;
	} else {
	  status = -1;
	}
      } 
    } 
  } 

  return status;
} 

