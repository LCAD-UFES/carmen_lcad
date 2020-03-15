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

#include <stdio.h>
#include <stdlib.h>
#include <netinet/in.h>
#include "arcnet.h"
#include "pc_d51.h"
#include "mac.h"

char *RPC_Info[20] = {
  NULL, NULL, NULL, NULL, NULL,
  NULL, NULL, NULL, NULL, NULL,
  NULL, NULL, NULL, NULL, NULL,
  NULL, NULL, NULL, NULL, NULL
}; 

long unsigned int nodePresentTimeout = 20; 
long unsigned int nodeRespondingTimeout = 30000; 
static unsigned char rpc_addr; 
ArgBlock block; 
unsigned char rpcTableSize; 
unsigned char inputBuf[508]; 

ArgBlock *RPC_Send(unsigned char destAddr, unsigned char service, 
		   unsigned char type, unsigned char procIndex, 
		   ArgBlock *args);
ArgBlock *IRPC_Print(ArgBlock *args);

TableEntry IRPC_Table[] = {
  {IRPC_Print, "IRPC_Print"},
  {NULL, NULL}
}; 

ArgBlock *IRPC_Print(ArgBlock *args)
{
  char *line; 
  line = RPC_GetArg(0, 0, 0, 0xff, args);
  printf("%s\n", line);
  args->length = 0;
  args->argNumber = 0;
  args->argNumberRef = 0;
  args->argData += 5;

  return args;
}

char RPC_SetInfo(unsigned char index1, char *string)
{
  
  RPC_Info[index1] = string;
  return 0;
}

void RPC_SetTimeout(unsigned long nodePresent, unsigned long nodeResponding)
{
  nodePresentTimeout = nodePresent;
  nodeRespondingTimeout = nodeResponding;
}

ArgContent RPC_NextFree(unsigned char argNum, ArgBlock *args)
{
  
  ArgContent *content; 
  ArgContent offset; 

  if (argNum > args->argNumber) {
    argNum = args->argNumber;
  } 
  
  offset = 0;
  while (argNum != 0) {
    content = (ArgContent *) (args->argData + offset);
    offset += (htons(*content) & 0x1fff);
    argNum--;
  } 
  
  return offset;
}

unsigned char RPC_Init(unsigned char addr)
{
  
  unsigned char status; 

  rpc_addr = addr;
  status = MAC_Init(addr);
  if (status != 0)
    return status;
  rpcTableSize = 0;

  while (RPC_Table[rpcTableSize].procedure != NULL)
    rpcTableSize++;
  DELAYMS(200);
  DELAYMS(200);
  DELAYMS(200);
  DELAYMS(200);
  DELAYMS(200);

  return 0;
}

ArgBlock *RPC_Receive(unsigned char service, unsigned char *type,
		      unsigned char *sourceAddr, unsigned char *procIndex)
{
  
  ArgContent length; 
  ArgContent tmp; 
  unsigned long count; 
  unsigned char originAddr; 
  unsigned char inType; 
  unsigned char inProcIndex; 
  ArgBlock *resp; 
  ArgBlock *(*proc)(); 

  count = 0;
  do {
    MAC_Receive(inputBuf, &length, &originAddr);
    if (length != 0) {
      inType = inputBuf[0];
      inProcIndex = inputBuf[1];
      tmp = *((ArgContent *) (inputBuf + 2));
      block.length = htons(tmp);
      block.argNumber = inputBuf[4];
      block.argNumberRef = 0;
      block.argData = inputBuf + 5;
      
      
      if (inType & 1) {
	if (service & 1) {
	  if (((type == NULL) || (inType == *type)) && 
	      ((procIndex == NULL) || (inProcIndex == *procIndex)) &&
	      ((sourceAddr == NULL) || (originAddr == *sourceAddr))) {
	    return &block;
	  }
	} 
	if (service & 8) {
	  if (type != NULL)
	    *type = inType;
	  
	  if (procIndex != NULL)
	    *procIndex = inProcIndex;
	  
	  if (sourceAddr != NULL)
	    *sourceAddr = originAddr;
	  
	  return &block;
	}      
      } 
      
      if ((inType == 0) || (inType == 0x10))  {
	if (inType == 0) {
	  proc = RPC_Table[inProcIndex].procedure;
	} else { 
	  proc = IRPC_Table[inProcIndex].procedure;
	} 
	if ((inProcIndex < rpcTableSize) || (inType == 0x10)) {
	  resp = (*proc)(&block);
	  RPC_Send(originAddr, 0, inType | 1, inProcIndex, resp);
	} 
      } 
    } else {
      if (service & 2) {
	if ((count > nodePresentTimeout) && (MAC_NoResponse())) {
	  printf("Not present\n");
	  return NULL;
	} 
	
	if (count > nodeRespondingTimeout) {
	  printf("Not responding\n");
	  return NULL;
	}
	DELAY10US(10);
	count++;
      } 
    }

  } while ((service & 2) && (service & 1));

  return NULL;
}

ArgBlock *RPC_Send(unsigned char destAddr, unsigned char service, 
		   unsigned char type, unsigned char procIndex, 
		   ArgBlock *args)
{
  ArgBlock *ret; 
  unsigned char *message; 
  ArgContent tmp; 

  if (destAddr == rpc_addr) {
    args->argNumberRef = 0;
    ret = (*(RPC_Table[procIndex].procedure))(args);
    ret->argNumberRef = 0;
    return ret;
  } 

  message = args->argData - 5;
  message[0] = type;
  message[1] = procIndex;
  tmp = htons(args->length);
  message[2] = *((char *) &tmp);
  message[3] = *(((char *) &tmp) + 1);
  message[4] = args->argNumber;
  
  MAC_Send(destAddr, message, args->length + 5);

  if (service & 1) {
    type |= 1;
    return RPC_Receive(service, &type, &destAddr, &procIndex);
  } else { 
    return NULL;
  }
}

void RPC_PutArg(short unsigned int argType, short unsigned int dataLength, 
		char *data, ArgBlock *args)
{
  
  
  ArgContent *content; 
  ArgContent offset; 
  ArgContent i; 
  unsigned char *ptemp; 

  switch (argType) {
  case 0x4000: 
    for (i = 0; i < dataLength; i += 2) {
      ptemp = data + i;
      *((unsigned short *) ptemp) = htons(*((unsigned short *) ptemp));
    } 
    break;

  case 0x6000: 
    for (i = 0; i < dataLength; i += 4) {
      ptemp = &(data[i]);
      *((unsigned int *) ptemp) = htonl(*((unsigned int *) ptemp));
    } 
    break;

  } 

  offset = RPC_NextFree(0xff, args);
  content = (ArgContent *) (args->argData + offset);
  *content = htons(((dataLength & 0x1fff) + 2) | argType);
  offset += 2;
  for (i = 0; i < dataLength; i++) {
    (args->argData)[offset + i] = data[i];
  }

  args->length += dataLength + 2;
  args->argNumber ++;
}

void *RPC_GetArg(unsigned char *error, short unsigned int expContent, 
		 ArgContent *content, 
		 unsigned char argNum, ArgBlock *args)
{
  
  
  ArgContent offset; 
  ArgContent i; 
  unsigned char result; 
  unsigned char *ptemp; 
  ArgContent lcontent; 

  result = 0;
  if (argNum > args->argNumber) {
    argNum = args->argNumberRef;
    args->argNumberRef++;
  } 
  
  offset = RPC_NextFree(argNum, args);

  lcontent = htons(*((unsigned int *) (args->argData + offset)));
  if (content == NULL) {
    if (expContent + 2 != lcontent)
      result = 0xff;
  } else { 
    content[0] = lcontent - 2;
  } 

  if (error != NULL) {
    *error = result;
  } 

  switch (lcontent & 0xe000) {
  case 0x4000: 
    for (i = 0; i < (unsigned int) ((lcontent & 0x1fff) - 2); i += 2) {
      ptemp = &(args->argData[offset + 2 + i]);
      *((unsigned short *) ptemp) = htons(*((unsigned short *) ptemp));
    } 
    break;

  case 0x6000: 
    for (i = 0; i < (unsigned int) ((lcontent & 0x1fff) - 2); i += 4) {
      ptemp = &(args->argData[offset + 2 + i]);
      *((unsigned int *) ptemp) = htonl(*((unsigned int *) ptemp));
    } 
    break;
  } 

  return &(args->argData[offset + 2]);
}

void RPC_PutINT8(char c, ArgBlock *args)
{
  
  char cTmp; 
  cTmp = c;
  RPC_PutArg(0x2000, 1, &cTmp, args);
}

void RPC_PutINT16(short int i, ArgBlock *args)
{
  
  short int iTmp; 

  iTmp = i;
  RPC_PutArg(0x4000, 2, (char *) &iTmp, args);
}

void RPC_PutINT32(long int i, ArgBlock *args)
{
  long int iTmp; 

  iTmp = i;
  RPC_PutArg(0x6000, 4, (char *) &iTmp, args);
}

