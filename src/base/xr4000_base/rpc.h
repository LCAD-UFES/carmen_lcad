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

#ifndef RPC_H
#define RPC_H

#ifdef __cplusplus
extern "C" {
#endif

char RPC_SetInfo(unsigned char index1, char *string);


void RPC_SetTimeout(unsigned long nodePresent, unsigned long nodeResponding);

ArgContent RPC_NextFree(unsigned char argNum, ArgBlock *args);

unsigned char RPC_Init(unsigned char addr);

ArgBlock *RPC_Receive(unsigned char service, unsigned char *type,
		      unsigned char *sourceAddr, unsigned char *procIndex);

ArgBlock *RPC_Send(unsigned char destAddr, unsigned char service, 
		   unsigned char type, unsigned char procIndex, 
		   ArgBlock *args);

void RPC_PutArg(short unsigned int argType, short unsigned int dataLength, 
		char *data, ArgBlock *args);

void *RPC_GetArg(unsigned char *error, short unsigned int expContent, 
		 ArgContent *content, 
		 unsigned char argNum, ArgBlock *args);

void RPC_PutINT8(char c, ArgBlock *args);

void RPC_PutINT16(short int i, ArgBlock *args);

void RPC_PutINT32(long int i, ArgBlock *args);

#ifdef __cplusplus
}
#endif

#endif
