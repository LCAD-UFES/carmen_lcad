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

#ifndef _PACKET_H
#define _PACKET_H

#include <string.h>
#include <stdint.h>

#include "orcconstants.h"

uint8_t packet_compute_checksum(uint8_t *p);
void packet_fill_checksum(uint8_t *p);
uint8_t packet_test_checksum(uint8_t *p);

void packet_init(uint8_t *p, uint8_t id);
void packet_fill_1(uint8_t *p, uint8_t val);
void packet_fill_2(uint8_t *p, uint16_t val);
void packet_fill_4(uint8_t *p, uint32_t val);

int packet_16u(uint8_t *p, int offset);
int packet_8u(uint8_t *p, int offset);

#endif

