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

#ifndef __INC_arcnet_h
#define __INC_arcnet_h

#ifdef __cplusplus
extern "C" {
#endif

typedef short unsigned int ArgContent;

typedef struct  
{
  unsigned short length;
  unsigned char argNumber;
  unsigned char argNumberRef;
  unsigned char *argData;    
} ArgBlock;

typedef struct {
  ArgBlock *(*procedure)();
  char *description;
} TableEntry;

extern TableEntry RPC_Table[];
extern TableEntry IRPC_Table[];
extern unsigned char d51_params[26];
extern unsigned char d51_net_map[32];

void *RPC_GetArg(unsigned char *error, short unsigned int expContent, 
		 ArgContent *content, 
		 unsigned char argNum, ArgBlock *args);

void RPC_PutArg(short unsigned int argType, short unsigned int dataLength, 
		char *data, ArgBlock *args);


unsigned char MAC_NoResponse();

unsigned char smc_in(unsigned char reg);

int ANET_Initialize();
void ANET_Poll();
void ANET_ConfSonar();

#ifdef __cplusplus
}
#endif

#endif /* __INC_arcnet_h */
