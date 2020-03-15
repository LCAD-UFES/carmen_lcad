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

#ifndef __INC_buffer_h
#define __INC_buffer_h

#ifdef __cplusplus
extern "C" {
#endif

void BUF_AddString(char *s);
char *BUF_Buffer();
void BUF_Reset(char *Buf);
long int BUF_Index();
void BUF_AddChar(char c);
void BUF_AddSpace();
void BUF_AddBoolean(unsigned char b);
void BUF_AddLong(long l);
void BUF_AddDouble(double d);
void BUF_AddPrecisionDouble(double d);

#ifdef __cplusplus
}
#endif

#endif /* __INC_buffer_h */
