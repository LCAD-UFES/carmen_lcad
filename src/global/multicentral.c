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

#include "global.h"
#include <pthread.h>
#include "ipc_wrapper.h"
#include <carmen/param_interface.h>
#include "multicentral.h"

void x_ipcRegisterExitProc(void (*proc)(void));

carmen_centrallist_p centrallist_copy = NULL;
char module_name[256];
void (*client_exit_handler)(void);

int carmen_allow_no_centrals = 0;

int carmen_multicentral_ipc_connect(char *ipc_module_name, char *central_name)
{
  IPC_RETURN_TYPE err;

  /* connect to the central server */
  err = IPC_connectModule(ipc_module_name, central_name);
  if(err == IPC_Error)
    return -1;
  /* Set local message queue capacity */
  err = IPC_setCapacity(4);
  carmen_test_ipc_exit(err,
		       "I had problems setting the IPC capacity. This is a "
		       "very strange error and should never happen.\n",
		       "IPC_setCapacity");
  return 0;
}

void carmen_multicentral_ipc_exit_handler(void)
{
  IPC_CONTEXT_PTR current_context;
  int i;
  
  current_context = IPC_getContext();
  for(i = 0; i < centrallist_copy->num_centrals; i++)
    if(centrallist_copy->central[i].connected &&
       current_context == centrallist_copy->central[i].context) {
      fprintf(stderr, "MULTICENTRAL: central %s disconnected.\n", 
	      centrallist_copy->central[i].host);
      centrallist_copy->central[i].connected = 0;
      centrallist_copy->central[i].ready_for_reconnect = 0;
    }
  if(client_exit_handler != NULL)
    client_exit_handler();
}

void carmen_multicentral_allow_zero_centrals(int allow)
{
  carmen_allow_no_centrals = allow;
}

carmen_centrallist_p carmen_multicentral_initialize(int argc, char **argv,
						    void (*exit_handler)(void))
{
  FILE *fp = NULL;
  carmen_centrallist_p centrallist;
  int err2, valid, i, one_central;
  char filename[256], *centralhost, *err, line[256];

  centrallist = (carmen_centrallist_p)calloc(1, sizeof(carmen_centrallist_t));
  carmen_test_alloc(centrallist);

  centrallist_copy = centrallist;

  valid = 0;
  if(argc >= 3) 
    for(i = 1; i < argc - 1; i++)
      if(strcmp(argv[i], "-central") == 0) {
	strcpy(filename, argv[i + 1]);
	valid = 1;
	break;
      }
  
  if(valid)
    fp = fopen(filename, "r");

  if(valid && fp != NULL) {
    /* read from the file */
    centrallist->num_centrals = 0;
    centrallist->central = NULL;
    do {
      err = fgets(line, 256, fp);
      if(err != NULL) {
	valid = 0;
	for(i = 0; i < (int)strlen(line); i++)
	  if(isalpha(line[i]))
	    valid = 1;
	if(valid) {
	  (centrallist->num_centrals)++;
	  centrallist->central = 
	    (carmen_central_p)realloc(centrallist->central,
				      centrallist->num_centrals *
				      sizeof(carmen_central_t));
	  carmen_test_alloc(centrallist->central);
	  sscanf(line, "%s", 
		 centrallist->central[centrallist->num_centrals - 1].host);
	  centrallist->central[centrallist->num_centrals - 1].connected = 0;
	  centrallist->central[centrallist->num_centrals - 1].ready_for_reconnect = 0;
	}
      }
    } while(err != NULL);
    fclose(fp);
  }
  else {
    /* if we can't open the list of centrals, default to connecting to
       localhost */
    if(valid)
      carmen_warn("Error: could not open central list %s for reading.\n", 
	       filename);
    centrallist->num_centrals = 1;
    
    centrallist->central = (carmen_central_p)
      calloc(1, sizeof(carmen_central_t));
    carmen_test_alloc(centrallist->central);
    
    centralhost = getenv("CENTRALHOST");
    if(centralhost == NULL)
      strcpy(centrallist->central[0].host, "localhost");
    else
      strcpy(centrallist->central[0].host, centralhost);
    centrallist->central[0].connected = 0;
    centrallist->central[0].ready_for_reconnect = 0;
  }

  fprintf(stderr, "CENTRAL List:\n");
  fprintf(stderr, "-------------\n");
  for(i = 0; i < centrallist->num_centrals; i++)
    fprintf(stderr, "%d : %s\n", i, centrallist->central[i].host);
  fprintf(stderr, "\n");

  /* set verbosity level */
  IPC_setVerbosity(IPC_Silent);

  /* construct unique IPC module name */
  snprintf(module_name, 200, "%s-%d", 
	   carmen_extract_filename(argv[0]), getpid());

  /* attempt to connect to each central */
  one_central = 0;
  for(i = 0; i < centrallist->num_centrals; i++) {
    err2 = carmen_multicentral_ipc_connect(module_name, 
					   centrallist->central[i].host);
    /* if successful, get IPC context */
    if(err2 >= 0) {
      fprintf(stderr, "MULTICENTRAL: central %s connected.\n", 
	      centrallist->central[i].host);
      centrallist->central[i].connected = 1;
      centrallist->central[i].context = IPC_getContext();
      x_ipcRegisterExitProc(carmen_multicentral_ipc_exit_handler);
      one_central = 1;
    }
    else
      centrallist->central[i].context = NULL;
  }
  if(!one_central && carmen_allow_no_centrals)
    carmen_warn("Error: could not connect to any central.\n");
  else  if(!one_central)
    carmen_die("Error: could not connect to any central.\n");
  client_exit_handler = exit_handler;
  return centrallist;
}

void carmen_multicentral_get_params(carmen_centrallist_p centrallist, 
				    int argc, char **argv,
				    void (*param_func)(int, char **))
{
  int i;

  /* get parameters from first valid central */
  for(i = 0; i < centrallist->num_centrals; i++)
    if(centrallist->central[i].connected) {
      IPC_setContext(centrallist->central[i].context);
      carmen_param_check_version(argv[0]);
      param_func(argc, argv);
      break;
    }
}

void carmen_multicentral_register_messages(carmen_centrallist_p centrallist,
					   void (*register_func)(void))
{
  int i;

  for(i = 0; i < centrallist->num_centrals; i++)
    if(centrallist->central[i].connected) {
      IPC_setContext(centrallist->central[i].context);
      register_func();
    }
}

void carmen_multicentral_subscribe_messages(carmen_centrallist_p centrallist,
					    void (*subscribe_func)(void))
{
  int i;

  for(i = 0; i < centrallist->num_centrals; i++)
    if(centrallist->central[i].connected) {
      IPC_setContext(centrallist->central[i].context);
      subscribe_func();
    }
}

void *test_central_thread(void *ptr)
{
  carmen_central_p central;
  int err;

  central = (carmen_central_p)ptr;
  do {
    if(central->connected || central->ready_for_reconnect) 
      usleep(250000);
    else {
      err = carmen_multicentral_ipc_connect(module_name, central->host);
      if(err >= 0) {
	IPC_disconnect();
	central->ready_for_reconnect = 1;
      }
    }
  } while(1);
  return NULL;
}

void carmen_multicentral_start_central_check(carmen_centrallist_p centrallist)
{
  pthread_t thread;
  int i;

  for(i = 0; i < centrallist->num_centrals; i++)
    pthread_create(&thread, NULL, test_central_thread,
		   &centrallist->central[i]);
}

void carmen_multicentral_reconnect_centrals(carmen_centrallist_p centrallist,
					    void (*register_func)(void),
					    void (*subscribe_func)(void))
{
  int i, err;

  /* try to reconnect to broken centrals */
  for(i = 0; i < centrallist->num_centrals; i++)
    if(!centrallist->central[i].connected && 
       centrallist->central[i].ready_for_reconnect) {
      fprintf(stderr, "MULTICENTRAL: central %s reconnected.\n", 
	      centrallist->central[i].host);
      err = carmen_multicentral_ipc_connect(module_name, 
					    centrallist->central[i].host);
      /* if successful, get IPC context */
      if(err >= 0) {
	centrallist->central[i].connected = 1;
	centrallist->central[i].ready_for_reconnect = 0;
	centrallist->central[i].context = IPC_getContext();

	x_ipcRegisterExitProc(carmen_multicentral_ipc_exit_handler);
	if(register_func != NULL)
	  register_func();
	if(subscribe_func != NULL)
	  subscribe_func();
      }
    }
}

void carmen_multicentral_ipc_sleep(carmen_centrallist_p centrallist, 
				   double sleep_time)
{
  int i, count = 0;

  for(i = 0; i < centrallist->num_centrals; i++) 
    if(centrallist->central[i].connected) 
      count++;
  
  if(count == 0)
    usleep((int)(sleep_time * 1e6));
  else {
    /* handle IPC messages for each central */
    for(i = 0; i < centrallist->num_centrals; i++) 
      if(centrallist->central[i].connected) {
	IPC_setContext(centrallist->central[i].context);
	carmen_ipc_sleep(sleep_time / count);
      }
  }
}
