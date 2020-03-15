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

#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "arcnet.h"
#include "rpc.h"
#include "robot_client.h"
#include "setup.h"
#include "Nclient.h"
#include "i200m.h"

enum anet_node_states { ANET_SEND, ANET_RECEIVE };

static ArgBlock *RCLNT_QueryDiagnosticsCall(ArgBlock *args);
static ArgBlock *RCLNT_PowerRequestCall(ArgBlock *args);

TableEntry RPC_Table[2] = {
  {RCLNT_QueryDiagnosticsCall, "QueryDiagnostics"},
  {RCLNT_PowerRequestCall, "PowerRequest"}
}; 

static enum anet_node_states anet_door_state[3] = {ANET_SEND,
						   ANET_SEND, 
						   ANET_SEND};
static enum {ANET_CSN_NO,
	     ANET_CSN_CS0, 
	     ANET_CSN_CS1,
	     ANET_CSN_CFD,
	     ANET_CSN_RESTART} anet_conf_sn_req[3] = {ANET_CSN_NO, 
						      ANET_CSN_NO, 
						      ANET_CSN_NO};
static unsigned char anet_ppc_channel = 0;
static unsigned long anet_ppc_send_time = 0;
static enum anet_node_states anet_ppc_state = 0;

static char rclnt_arcnet_buffer[512];
static long int anet_door_timer_offsets[3];
static float anet_door_timer_errors[3];
static long unsigned int anet_door_send_time[3];
static unsigned char rclnt_power_state;

ArgBlock *RPRT_Init(ArgBlock *args, unsigned char type)
{
  args->length = 0;
  args->argNumber = 0;
  args->argNumberRef = 0;
  args->argData += 5;
  
  RPC_PutINT8((char)type, args);

  if(type == 0xff)
    RPC_PutINT8(0, args);
  return args;
}

ArgBlock *RPRT_Error(ArgBlock *args, unsigned char type)
{
  args->length = 0;
  args->argNumber = 0;
  args->argNumberRef = 0;
  args->argData += 5;

  RPC_PutINT8((char)type, args);
  RPC_PutINT16(-1, args);
  return args;
}

ArgBlock *RPRT_Line(ArgBlock *args, short unsigned int line_no, 
		    unsigned char type, char *text)
{
  args->length = 0;
  args->argNumber = 0;
  args->argNumberRef = 0;
  args->argData += 5;

  RPC_PutINT8((char) type, args);
  RPC_PutINT16((short) line_no, args);
  RPC_PutINT8(0, args);

  if (type == 0) {
    RPC_PutINT8(0, args);
    RPC_PutINT16(0, args);
  }
  RPC_PutArg(0x2000, (unsigned short) (strlen(text) + 1), text, args);
  return args;
}

static ArgBlock *RCLNT_QueryDiagnosticsCall(ArgBlock *args)
{
  unsigned char type; 
  unsigned short line; 
  unsigned short screen; 

  type = *((char *) RPC_GetArg(0, 0x2001, 0, 0xff, args));
  screen = *((unsigned short *) RPC_GetArg(0, 0x4002, 0, 0xff, args));

  switch (type) {
  case 0:
    switch ((short) screen) {
    case 0:
      return RPRT_Init(args, 1);
    default:
      return RPRT_Init(args, 0xff);
    }
  case 1:
    line = *((unsigned short *) RPC_GetArg(0, 0x4002, 0, 0xff, args));
    switch ((short) screen) {
    case 0: 
      switch (line) {
      case 0:
	return RPRT_Line(args, line, 0, "PC is operational");
      default:
	return RPRT_Error(args, 0);	
      }
    default: 
      return RPRT_Init(args, 0xff);
    }
  }
  return args;
}

static ArgBlock *RCLNT_PowerRequestCall(ArgBlock *args)
{
  rclnt_power_state = *((char *) RPC_GetArg(0, 0x2001, 0, 0xff, args));
  
  args->length = 0;
  args->argNumber = 0;
  args->argNumberRef = 0;
  args->argData += 5;

  return args;
}

static void FillDoorStateStructFromArgBlock(int door, ArgBlock *argsp)
{
  int i; 
  int j; 
  int set; 
  unsigned short *sonars; 
  unsigned char sw0; 
  unsigned char sw1; 
  unsigned char *bumper_sw0; 
  unsigned char *bumper_sw1; 
  unsigned char *infrareds; 
  unsigned long *sonar_ts; 
  unsigned long *infrared_ts; 
  unsigned long *bumper_sw0_ts; 
  unsigned long *bumper_sw1_ts; 
  
  sonars = (unsigned short *) RPC_GetArg(0, 0, 0, 0xff, argsp);
  bumper_sw0 = (unsigned char *) RPC_GetArg(0, 0, 0, 0xff, argsp);
  bumper_sw1 = (unsigned char *) RPC_GetArg(0, 0, 0, 0xff, argsp);
  infrareds = (unsigned char *) RPC_GetArg(0, 0, 0, 0xff, argsp);
  sonar_ts = (unsigned long *) RPC_GetArg(0, 0, 0, 0xff, argsp);
  bumper_sw0_ts = (unsigned long *) RPC_GetArg(0, 0, 0, 0xff, argsp);
  bumper_sw1_ts = (unsigned long *) RPC_GetArg(0, 0, 0, 0xff, argsp);
  infrared_ts = (unsigned long *) RPC_GetArg(0, 0, 0, 0xff, argsp);

  i = 0;
  j = 0;
  for (set = 2 * door; set < 2 * (door + 1); set++) {
    for (i = 0; i < 8; i++) {
      rclnt_rs.SonarController.SonarSet[set].Sonar[i].Reading = 
	(sonars[j] != 300) ? sonars[j] * 25.4 : 0x7fffffff;
      rclnt_rs.SonarController.SonarSet[set].Sonar[i].TimeStamp =  
      	sonar_ts[j] + anet_door_timer_offsets[door];
      j++;
    } 
  } 

  j = 0;
  set = 2 * door;
  while (set < 2 * (door + 1)) {
    sw0 = bumper_sw0[set & 1];
    sw1 = bumper_sw1[set & 1];
    for (i = 0; i < 8; i++) {
      rclnt_rs.BumperController.BumperSet[set].Bumper[i].Reading =
	(!(sw1 & 1)) ? sw0 & 1 : 2;
      rclnt_rs.BumperController.BumperSet[set].Bumper[i].TimeStamp =
	(sw1 & 1) ? bumper_sw1_ts[j] + anet_door_timer_offsets[door] : 
	bumper_sw0_ts[j] + anet_door_timer_offsets[door];
      sw0 >>= 1;
      sw1 >>= 1;
      j++;
    } 
    set++;
  } 

  sw0 = bumper_sw0[2];
  while (i < 12) {
    rclnt_rs.BumperController.BumperSet[set].Bumper[i].Reading =
      sw0 & 1;
    rclnt_rs.BumperController.BumperSet[set].Bumper[i].TimeStamp =
      bumper_sw0_ts[j] + anet_door_timer_offsets[door];
    j++;
    i++;
  } 
  
  j = 0;
  set = door * 2;
  while (set < 2 * (door + 1)) {
    for (i = 0; i < 8; i++) {
      rclnt_rs.InfraredController.InfraredSet[set].Infrared[i].Reading = 
	infrareds[j];
      rclnt_rs.InfraredController.InfraredSet[set].Infrared[i].TimeStamp = 
      	infrared_ts[j] + anet_door_timer_offsets[door];
      j++;
    } 
    set++;
  } 
  return ;
}

static void HandleDoorResponse(int door, ArgBlock *argsp, 
			       unsigned char proc_index, 
			       unsigned long rxtime __attribute__ ((unused)))
{
  long int tr; 

  if (anet_door_state[door] != ANET_RECEIVE) {
    printf("anet: response to door %d while not if receive state\n",
	   door);
    return ;
  } 
  
  switch (proc_index) {
  case 1: 
    FillDoorStateStructFromArgBlock(door, argsp);
    break;
  case 2: 
    switch (anet_conf_sn_req[door]) {
    case ANET_CSN_NO: 
      printf("anet: conf_sn response while door %d is not waiting\n",
	     door);
      return ;
    case ANET_CSN_CS0: 
    case ANET_CSN_CS1:
      anet_conf_sn_req[door]++;
      break;
    case ANET_CSN_CFD: 
      anet_conf_sn_req[door] = ANET_CSN_NO;
      break;
    default: 
      break;
    }
    break;
  case 0: 
    anet_door_timer_errors[door] = (rxtime - anet_door_send_time[door]) / 2.0;
    tr = *((long int *) RPC_GetArg(0, 0x6004, 0, 0xff, argsp));
    anet_door_timer_offsets[door] = anet_door_send_time[door] + 
      anet_door_timer_errors[door] - tr;
    break;
  default: 
    printf("anet: got bad proc from door %d: %d\n", door, proc_index);
    return ;
  }
  anet_door_state[door] = ANET_SEND;
  return ;
}

static void PollDoors(unsigned long current_time)
{
  unsigned char fo[9]; 
  unsigned char *cp; 
  int i, j, idx; 
  float delta_err; 
  ArgBlock args; 
  static long unsigned int last_poll_time = 0;
  
  delta_err = 8e-05 * (current_time - last_poll_time);
  last_poll_time = current_time;
  
  for (i = 0; i <= 2; i++) {
    anet_door_timer_errors[i] = anet_door_timer_errors[i] + delta_err;
    switch (anet_door_state[i]) {
    case ANET_SEND: 
      if (anet_door_timer_errors[i] > 15.0) {
	args.length = 0;
	args.argNumber = 0;
	args.argNumberRef = 0;
	args.argData = rclnt_arcnet_buffer + 5;
	
	RPC_Send((unsigned char) (i + 16), 0, 0, 0, &args);
      } else {
	if (anet_conf_sn_req[i]) {
	  switch (anet_conf_sn_req[i]) {
	    
	  case ANET_CSN_RESTART: 
	    anet_conf_sn_req[i] = ANET_CSN_CS0;
	    /* WARNING - break here ? */

	  case 1: 
	  case 2:
	    idx = 2 * i + anet_conf_sn_req[i] - 1;	    
	    memset(fo, 0xff, 9);
	    cp = fo;
	    j = 0; 
		
	    do {
	      *cp = rclnt_rs.SonarController.SonarSet[idx].FiringOrder[j];
	    } while ((*cp++ != 255) && (++j < 8));
	    args.length = 0;
	    args.argNumber = 0;
	    args.argNumberRef = 0;
	    args.argData = rclnt_arcnet_buffer + 5;
	    RPC_PutINT8(((unsigned char) (anet_conf_sn_req[i] - 1)), &args);
	    RPC_PutINT8(0, &args);
	    RPC_PutArg(0x2000, 9, fo, &args);
	    RPC_Send((unsigned char) (i + 16), 0, 0, 2, &args);
	    break ;
	    
	  case 3: 
	    args.length = 0;
	    args.argNumber = 0;
	    args.argNumberRef = 0;
	    args.argData = rclnt_arcnet_buffer + 5;
	    RPC_PutINT8(0, &args);
	    RPC_PutINT8(2, &args);
	    RPC_PutINT8((char)
			rclnt_rs.SonarController.SonarSet[i * 2].FiringDelay, 
			&args);
	    RPC_Send((unsigned char)(i + 16), 0, 0, 2, &args);
	    break;
	  default: 
	    break;
	  } 
	} else {
	  if (I200M_MSecSinceBoot() - anet_door_send_time[i] < 5) {
	    break;
	  } else { 
	    args.length = 0;
	    args.argNumber = 0;
	    args.argNumberRef = 0;
	    args.argData = rclnt_arcnet_buffer + 5;
	    RPC_PutINT8(15, &args);
	    RPC_Send((unsigned char)(i + 16), 0, 0, 1, &args);
	  } 
	} 
      } 
      anet_door_send_time[i] = current_time;
      anet_door_state[i] = ANET_CSN_CS0;
      break;
    case ANET_RECEIVE: 
      if (current_time - anet_door_send_time[i] > 5000) {
	printf("anet: door %d receive timeout\n", i);
	anet_door_state[i] = ANET_CSN_NO;
      }
      break;
    } 
  } 
  return ;
}

static void HandlePPCResponse(ArgBlock *argsp, unsigned char procIndex, 
			      unsigned long rxtime __attribute__ ((unused)))
{
  long int value; 

  if (anet_ppc_state != ANET_RECEIVE) {
    printf("anet: response to ppc while not if receive state\n");
    return ;
  }
  if (procIndex == 2) {
    value = *((long int *) RPC_GetArg(0, 0x6004, 0, 0xff, argsp));
    rclnt_rs.BatterySet.Battery[anet_ppc_channel].Voltage = value;
    anet_ppc_channel++;
    anet_ppc_channel &= 3;
    anet_ppc_send_time = I200M_MSecSinceBoot();
    anet_ppc_state = ANET_SEND;
  } 
  return ;
}

static void PollPPC(unsigned long current_time)
{
  ArgBlock args; 

  switch (anet_ppc_state) {
  case ANET_SEND: 
    if (current_time - anet_ppc_send_time < 10000)
      break;
    args.length = 0;
    args.argNumber = 0;
    args.argNumberRef = 0;
    args.argData = rclnt_arcnet_buffer + 5;
    RPC_PutINT8(anet_ppc_channel, &args);
    RPC_Send(32, 0, 0, 2, &args);
    anet_ppc_send_time = current_time;
    anet_ppc_state = ANET_RECEIVE;
    break;
  case ANET_RECEIVE: 
    if (current_time - anet_ppc_send_time > 5000) {
      printf("PPC timeout\n");
      anet_ppc_state = ANET_SEND;
    }
    break;
  } 
  return;
}

static void RunShutdown()
{
  int result; 
  char *valuep; 
  char *endp; 
  unsigned long delay; 
  int pid; 
  ArgBlock args; 

  pid = fork();
  if(pid == -1) {
    fprintf(stderr, "Nrobot: error in fork() while responding to "
	    "shutdown request: ");
    perror(NULL);
    return ;
  } 
  if(pid != 0)
    return;
  if((valuep = SETUP_GetValue("[shutdown]delay")) == NULL)
    delay = 25000;
  else { 
    delay = strtol(valuep, &endp, 0);
    if (*endp != '\0') {
      fprintf(stderr, "Nrobot_client (warning): invalid value for %s;\n"
	      "  attempting to use \"%u\"\n", "[shutdown]delay", 25000);
      delay = 25000;
    }
  }
  if((valuep = SETUP_GetValue("[shutdown]command")) == NULL)
    valuep = "/sbin/shutdown -h -t 0 now";
  result = system(valuep);
  if (result == -1) {
    fprintf(stderr, "Nrobot: system(\"%s\") failed (shutdown aborted): ",
	    valuep);
    perror(NULL);
    return;
  }
  else if (result == 127) { 
    fprintf(stderr, "Nrobot: exec of /bin/sh failed while running system()."
	    "  Shutdown aborted.\n");
    return;
  } 
  else if (result != 0) {
    fprintf(stderr, "Nrobot: \"%s\" returned %d.  Shutdown aborted.\n",
	    valuep, result);
    return;
  } 
  printf("Nrobot: shutting down power.\n");
  args.length = 0;
  args.argNumber = 0;
  args.argNumberRef = 0;
  args.argData = rclnt_arcnet_buffer + 5;
  RPC_PutINT8(rclnt_power_state, &args);
  RPC_PutINT32(delay, &args);
  RPC_Send(32, 3, 0, 3, &args);
  exit(0);
}

int ANET_Initialize()
{
  ArgBlock args, *argsp; 
  int i, j; 

  RPC_Init(1);
  for (j = 0; j <= 2; j++) {
    anet_door_timer_errors[j] = 15.0;
    args.length = 0;
    args.argNumber = 0;
    args.argNumberRef = 0;
    args.argData = rclnt_arcnet_buffer + 5;
    RPC_PutINT8(0, &args);
    RPC_PutINT8(2, &args);
    argsp = RPC_Send((unsigned char) (j + 16), 3, 0, 4, &args);
    if(argsp != NULL) {
      rclnt_rs.SonarController.SonarSet[2 * j].FiringDelay =
	rclnt_rs.SonarController.SonarSet[2 * j+1].FiringDelay =
	*((char *) RPC_GetArg(0, 0x2001, 0, 0xff, argsp));
    }
    for (i = 0; i <= 1; i++) {
      memset(rclnt_rs.SonarController.SonarSet[2 * j + i].FiringOrder, 255, 9);
      args.length = 0;
      args.argNumber = 0;
      args.argNumberRef = 0;
      args.argData = rclnt_arcnet_buffer + 5;
      RPC_PutINT8((char) i, &args);
      RPC_PutINT8(0, &args);
      argsp = RPC_Send((unsigned char) (j + 16), 3, 0, 4, &args);
      if (argsp != NULL)
	memcpy(rclnt_rs.SonarController.SonarSet[2 * j + i].FiringOrder,
	       RPC_GetArg(0, 0, 0, 0xff, argsp), 9);
    }
  } 
  rclnt_power_state = 0;
  return 0;
}

void ANET_ConfSonar()
{
  int i; 

  for (i = 0; i <= 2; i++)
    anet_conf_sn_req[i] = ANET_CSN_RESTART;
  return;
}

void ANET_Poll()
{
  unsigned char type; 
  unsigned char sourceAddr; 
  unsigned char procIndex; 
  unsigned long current_time; 
  ArgBlock *argsp; 

  current_time = I200M_MSecSinceBoot();
  while (1) {
    if(((argsp = RPC_Receive(12, &type, &sourceAddr, &procIndex)) == NULL) ||
       (type == 0))
      break;
      
    if ((sourceAddr > 15) && (sourceAddr < 19))
      HandleDoorResponse((unsigned char) sourceAddr - 16, 
			 argsp, procIndex, current_time);
    else if (sourceAddr == 32)
      HandlePPCResponse(argsp, procIndex, current_time);
  } 
  
  PollDoors(current_time);
  PollPPC(current_time);
  if (rclnt_power_state) {
    RunShutdown();
    rclnt_power_state = 0;
  }
  return;
}

