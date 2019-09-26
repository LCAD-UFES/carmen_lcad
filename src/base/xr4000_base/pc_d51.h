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

#ifndef PC_D51_H
#define PC_D51_H

#ifdef __cplusplus
extern "C" {
#endif

unsigned char d51_read_packet(unsigned char wait_flag, 
			      unsigned char *data_ptr);
unsigned char d51_write_packet(unsigned char *data_ptr);
unsigned char d51_init();
void d51_exit();
void d51_check_int();

void DELAYMS(unsigned int ms);

void DELAY10US(unsigned int us);

#ifdef __cplusplus
extern "C" {
#endif

#endif
