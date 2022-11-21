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

#include <carmen/carmen.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#define FREE_RESPONSE_STRING(msg) \
  free(msg->module_name); \
  free(msg->variable_name); \
  free(msg->value); \
  free(msg->host); \
  free(msg);

#define FREE_RESPONSE_DOUBLE(msg) \
  free(msg->module_name); \
  free(msg->variable_name); \
  free(msg->host); \
  free(msg);

#define FREE_RESPONSE_INT(msg) FREE_RESPONSE_DOUBLE(msg)
#define FREE_RESPONSE_ONOFF(msg) FREE_RESPONSE_DOUBLE(msg)

/*
typedef struct {
  char *module_name;
  char *variable_name;
  void *variable_address;
  void (*handler)(void);
} carmen_param_subscription_t, *carmen_param_subscription_p; 
*/

static char *module_name = NULL;
static unsigned int timeout = 5000;
static int allow_not_found_parameters = 0;

static carmen_param_t *installed_parameters = NULL;
static int installed_list_length = 0;
static int installed_list_capacity = 0;

static char error_buffer[2048];

static char *usage_line = NULL;

static void install_parameter(char *module, char *variable, 
			      void *variable_address, 
			      carmen_param_type_t type, int subscribe, 
			      carmen_param_change_handler_t handler);
static void change_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
			   void *clientData __attribute__ ((unused)));

static int 
param_check_commandline_int(const char *lvalue, int *return_value) 
{
  char *endptr;
  char *arg;

  if(carmen_find_param(lvalue) == 0) 
    return 0;
    
  if (carmen_find_param_pair(lvalue) == 0) {
    sprintf(error_buffer, "Option '%s' requires argument: should be an "
	    "integer.", lvalue);
    return -1;
  }
  
  arg = carmen_param_pair_and_remove(lvalue);
  *return_value = strtol(arg, &endptr, 0);

  if (endptr == arg) {
    sprintf(error_buffer, "Bad argument to %s: %s, should be an integer.", 
	    lvalue, arg);
    return -1;
  }
  
  return 1;
}

static int 
param_check_commandline_double(const char *lvalue, double *return_value) 
{
  char *endptr;
  char *arg;

  if(carmen_find_param(lvalue) <= 0) 
    return 0;

  if (carmen_find_param_pair(lvalue) == 0) {
    sprintf(error_buffer, "Option '%s' requires argument: should be an "
	    "double.", lvalue);
    return -1;
  }
  arg = carmen_param_pair_and_remove(lvalue);
  *return_value = strtod(arg, &endptr);
  if (endptr == arg) {
    sprintf(error_buffer, "Bad argument to %s: %s, should be a double.", 
	    lvalue, arg);
    return -1;
  }
  
  return 1;
}

static int 
param_check_commandline_onoff(const char *lvalue, int *return_value) 
{
  char *arg;

  if(carmen_find_param(lvalue) <= 0) 
    return 0;

  if (carmen_find_param_pair(lvalue) == 0) {
    sprintf(error_buffer, "Option '%s' requires argument: should be "
	    "on/off.", lvalue);
    return -1;
  }
  arg = carmen_param_pair_and_remove(lvalue);
  if (strcmp(arg, "on") == 0)
    *return_value = 1;
  else if (strcmp(arg, "off") == 0)
    *return_value = 0;
  else {
    sprintf(error_buffer, "Bad argument to %s: %s, should be on/off.", 
	    lvalue, arg);
    return -1;
  }
  
  return 1;
}

static int
param_check_commandline_string(const char *lvalue, char **string) 
{
  char *arg;

  if(carmen_find_param(lvalue) <= 0) 
    return 0;

  if (carmen_find_param_pair(lvalue) == 0) {
    sprintf(error_buffer, "Option '%s' requires argument: should be a "
	    "string.", lvalue);
    return -1;
  }
  arg = carmen_param_pair_and_remove(lvalue);
  *string = arg;

  return 1;
}

static int 
param_check_commandline_filename(const char *lvalue, char **filename) 
{
  char *arg;
  struct stat buf;

  if(carmen_find_param(lvalue) <= 0) 
    return 0;

  if (carmen_find_param_pair(lvalue) == 0) {
    sprintf(error_buffer, "Option '%s' requires argument: should be a "
	    "filename.", lvalue);
    return -1;
  }
  arg = carmen_param_pair_and_remove(lvalue);
  if (stat(arg, &buf) < 0) {
    sprintf(error_buffer, "Option '%s', file %s not available: %s", lvalue, 
	    arg, 
	    strerror(errno));
    return -1;
  }
  if (!S_ISREG(buf.st_mode)) {
    sprintf(error_buffer, "Option '%s' requires regular file. %s is "
	    "not a regular file.", lvalue, arg);
    return -1;
  }
  *filename = arg;

  return 1;
}

static int
param_check_commandline_directory(char *lvalue, char **dirname) 
{
  char *arg;
  struct stat buf;

  if(carmen_find_param(lvalue) <= 0) 
    return 0;

  if (carmen_find_param_pair(lvalue) == 0) {
    sprintf(error_buffer, "Option '%s' requires argument: should be "
	    "a directory name.", lvalue);
    return -1;
  }
  arg = carmen_param_pair_and_remove(lvalue);
  if (stat(arg, &buf) < 0) {
    sprintf(error_buffer, "Option '%s', directory %s not available: %s", 
	    lvalue, arg, 
	    strerror(errno));
    return -1;
  }
  if (!S_ISDIR(buf.st_mode)) {
    sprintf(error_buffer, "Option '%s' requires a directory name. %s is "
	    "not a directory.", lvalue, arg);
    return -1;
  }
  *dirname = arg;

  return 1;
}

char *
carmen_param_get_robot(void)
{
  IPC_RETURN_TYPE err;
  carmen_param_query_message query;
  carmen_param_response_robot_message *response;
  char *robot_name = NULL;
  static int initialized = 0;

  if (!initialized) {
    err = IPC_defineMsg(CARMEN_PARAM_QUERY_ROBOT_NAME,
			IPC_VARIABLE_LENGTH, 
			CARMEN_PARAM_QUERY_FMT);
    carmen_test_ipc_exit(err, "Could not define message", 
			 CARMEN_PARAM_QUERY_ROBOT_NAME);
    initialized = 1;
  }

  query.timestamp = carmen_get_time();
  query.host = carmen_get_host();
  query.module_name = "paramServer";
  query.variable_name = "robot";

  err = IPC_queryResponseData(CARMEN_PARAM_QUERY_ROBOT_NAME, &query,
			      (void **) &response, timeout);
  carmen_test_ipc(err, "Could not query parameter",
		  CARMEN_PARAM_QUERY_ROBOT_NAME);
  if (err == IPC_Error || err == IPC_Timeout) {
    sprintf(error_buffer, "Did you remember to start the parameter server?\n"
	    "Remember, this program loads its parameters from the param_daemon."
	    "\n");
    return NULL;
  }

  if (response->status == CARMEN_PARAM_OK) {
    robot_name = (char *) calloc(strlen(response->robot) + 1, sizeof(char));
    carmen_test_alloc(robot_name);
    strcpy(robot_name, response->robot);
  }

  free(response->host);
  free(response->robot);
  free(response);

  return robot_name;
}

int
carmen_param_get_modules(char ***modules, int *num_modules)
{
  IPC_RETURN_TYPE err;
  carmen_param_query_message query;
  carmen_param_response_modules_message *response;
  int m;
  static int initialized = 0;
  int status;

  if (!initialized) {
    err = IPC_defineMsg(CARMEN_PARAM_QUERY_MODULES_NAME,
			IPC_VARIABLE_LENGTH, 
			CARMEN_PARAM_QUERY_FMT);
    carmen_test_ipc_exit(err, "Could not define message", 
			 CARMEN_PARAM_QUERY_MODULES_NAME);
    initialized = 1;
  }


  if (modules == NULL || num_modules == NULL)
    return -1;

  query.timestamp = carmen_get_time();
  query.host = carmen_get_host();

  query.module_name = "paramServer";
  query.variable_name = "modules";
  
  response = NULL;
  err = IPC_queryResponseData(CARMEN_PARAM_QUERY_MODULES_NAME, &query,
			      (void **) &response, timeout);
  carmen_test_ipc(err, "Could not query parameter",
		  CARMEN_PARAM_QUERY_MODULES_NAME);
  if (err == IPC_Error || err == IPC_Timeout) {
    sprintf(error_buffer, "Did you remember to start the parameter server?\n"
	    "Remember, this program loads its parameters from the paramServer."
	    "\n");
    return -1;
  }

  *num_modules = response->num_modules;
  status = response->status;

  if (status == CARMEN_PARAM_OK) {
    *modules = (char **) calloc(*num_modules, sizeof(char *));
    carmen_test_alloc(*modules);
    for (m = 0; m < *num_modules; m++) {
      (*modules)[m] = (char *) calloc(strlen(response->modules[m]) + 1,
				      sizeof(char));
      carmen_test_alloc((*modules)[m]);
      strcpy((*modules)[m], response->modules[m]);
    }
  }
  
  for (m = 0; m < *num_modules; m++)
    free(response->modules[m]);
  free(response->host);
  free(response->modules);
  free(response);

  if (status == CARMEN_PARAM_OK)
    return 0;

  return -1;
}

int
carmen_param_get_all(const char *module, char ***variables, char ***values,
		     int **expert, int *list_length)
{
  IPC_RETURN_TYPE err;

  carmen_param_query_message query;
  carmen_param_response_all_message *response;
  int status;
  int index;
  size_t length;
  static int initialized = 0;

  if (!initialized) {
    err = IPC_defineMsg(CARMEN_PARAM_QUERY_ALL_NAME,
			IPC_VARIABLE_LENGTH, 
			CARMEN_PARAM_QUERY_FMT);
    carmen_test_ipc_exit(err, "Could not define message", 
			 CARMEN_PARAM_QUERY_ALL_NAME);
    initialized = 1;
  }


  if (module == NULL || module[0] == '\0' || list_length == NULL)
    return -1;
  
  query.timestamp = carmen_get_time();
  query.host = carmen_get_host();
  query.module_name = (char*)module;
  query.variable_name = "*";
  
  err = IPC_queryResponseData(CARMEN_PARAM_QUERY_ALL_NAME, &query, 
			      (void **)&response, timeout);
  carmen_test_ipc(err, "Could not query all parameters", 
		  CARMEN_PARAM_QUERY_ALL_NAME);
  if (err == IPC_Error || err == IPC_Timeout) {
    sprintf(error_buffer, "Did you remember to start the parameter server?\n"
	    "Remember, this program loads its parameters from the "
	    "param_daemon.\n");
    return -1;
  }
  
  *list_length = 0;
  status = response->status;
  if (status == CARMEN_PARAM_OK) {
    *list_length = response->list_length;
    if (variables) {
      *variables = (char **)calloc(*list_length, sizeof(char *));
      carmen_test_alloc(*variables);
    }
    if (values) {
      *values = (char **)calloc(*list_length, sizeof(char *));
      carmen_test_alloc(*values);
    }
    if (expert) {
      *expert = (int *)calloc(*list_length, sizeof(int));
      carmen_test_alloc(*expert);
    }
    for (index = 0; index < response->list_length; index++) {
      if (variables) {
	length = strlen(response->variables[index])+1;
	(*variables)[index] = (char *)calloc(length, sizeof(char));
	carmen_test_alloc((*variables)[index]);
	strcpy((*variables)[index], response->variables[index]);
      }
      if (values) {
	length = strlen(response->values[index])+1;
	(*values)[index] = (char *)calloc(length, sizeof(char));
	carmen_test_alloc((*values)[index]);
	strcpy((*values)[index], response->values[index]);
      }
      if (expert)
	(*expert)[index] = response->expert[index];
      
    } /* End of for (index = 0; index < response->list_length ...*/
  }
  
  for (index = 0; index < response->list_length; index++) {
    free(response->variables[index]);
    free(response->values[index]);
  }
  free(response->variables);
  free(response->values);
  free(response->expert);
  free(response);

  if (status == CARMEN_PARAM_OK)
    return 0;

  return -1;
}


int
carmen_param_get_int(const char *variable, int *return_value, int *expert)
{
  IPC_RETURN_TYPE err;
  int commandline_return;
  char buffer[1024];

  carmen_param_query_message query;
  carmen_param_response_int_message *response;
  static int initialized = 0;

  if (!initialized) {
    err = IPC_defineMsg(CARMEN_PARAM_QUERY_INT_NAME,
			IPC_VARIABLE_LENGTH, 
			CARMEN_PARAM_QUERY_FMT);
    carmen_test_ipc_exit(err, "Could not define message", 
			 CARMEN_PARAM_QUERY_INT_NAME);
    initialized = 1;
  }


  if (variable == NULL || variable[0] == '\0' || return_value == NULL) {
    sprintf(error_buffer, "Bad argument passed to %s", __FUNCTION__);
    return -1;
  }
  
  if (module_name) {
    sprintf(buffer, "%s_%s", module_name, variable);
    commandline_return = 
      param_check_commandline_int(buffer, return_value);
    if (commandline_return != 0)
      return commandline_return;
  }

  commandline_return = param_check_commandline_int(variable, return_value);
  if (commandline_return != 0)
    return commandline_return;

  query.timestamp = carmen_get_time();
  query.host = carmen_get_host();
  query.module_name = module_name;
  query.variable_name = (char*)variable;
  
  err = IPC_queryResponseData(CARMEN_PARAM_QUERY_INT_NAME, &query, 
			      (void **)&response, timeout);
  carmen_test_ipc(err, "Could not query int parameter", 
		  CARMEN_PARAM_QUERY_INT_NAME);
  if (err == IPC_Error || err == IPC_Timeout) {
    sprintf(error_buffer, "Did you remember to start the parameter server?\n"
	    "Remember, this program loads its parameters from the "
	    "param_daemon.\n");
    return -1;
  }
  
  if (response->status == CARMEN_PARAM_NOT_FOUND && 
      !allow_not_found_parameters) {
    sprintf(error_buffer, "The parameter server contains no definition "
	    "for %s_%s,\nrequested by this program. You may have started "
	    "the param_daemon with\nan out-of-date carmen.ini file. Or, this "
	    "may be a bug in this program\n(but probably not the parameter "
	    "server). \n", (!module_name ? "" : module_name), variable);
    return -1;
  }
  
  if (response->status == CARMEN_PARAM_OK) {
    *return_value = response->value;
    if (expert)
      *expert = response->expert;
    FREE_RESPONSE_INT(response);
    return 1;
  }
  
  FREE_RESPONSE_INT(response);
  return 0;
}

int
carmen_param_get_double(const char *variable, double *return_value, int *expert)
{
  IPC_RETURN_TYPE err;
  int commandline_return;
  char buffer[1024];
  carmen_param_query_message query;
  carmen_param_response_double_message *response;
  static int initialized = 0;

  if (!initialized) {
    err = IPC_defineMsg(CARMEN_PARAM_QUERY_DOUBLE_NAME,
			IPC_VARIABLE_LENGTH, 
			CARMEN_PARAM_QUERY_FMT);
    carmen_test_ipc_exit(err, "Could not define message", 
			 CARMEN_PARAM_QUERY_DOUBLE_NAME);
    initialized = 1;
  }
  
  if (variable == NULL || variable[0] == '\0' || return_value == NULL) {
    sprintf(error_buffer, "Bad argument passed to %s", __FUNCTION__);
    return -1;
  }
  
  if (module_name) {
    sprintf(buffer, "%s_%s", module_name, variable);
    commandline_return = 
      param_check_commandline_double(buffer, return_value);
    if (commandline_return != 0)
      return commandline_return;
  }

  commandline_return = param_check_commandline_double(variable, return_value);
  if (commandline_return != 0)
    return commandline_return;

  query.timestamp = carmen_get_time();
  query.host = carmen_get_host();
  query.module_name = module_name;
  query.variable_name = (char*)variable;
  
  err = IPC_queryResponseData(CARMEN_PARAM_QUERY_DOUBLE_NAME, &query, 
			      (void **)&response, timeout);
  carmen_test_ipc(err, "Could not query double parameter", 
		  CARMEN_PARAM_QUERY_DOUBLE_NAME);
  if (err == IPC_Error || err == IPC_Timeout) {
    sprintf(error_buffer, "Did you remember to start the parameter server?\n"
	    "Remember, this program loads its parameters from the "
	    "param_daemon.\n");
    return -1;
  }
  
  if (response->status == CARMEN_PARAM_NOT_FOUND && 
      !allow_not_found_parameters) {
    sprintf(error_buffer, "The parameter server contains no definition "
	    "for %s_%s,\nrequested by this program. You may have started "
	    "the param_daemon with\nan out-of-date carmen.ini file. Or, this "
	    "may be a bug in this program\n(but probably not the parameter "
	    "server). \n", (!module_name ? "" : module_name), variable);
    return -1;
  }

  if (response->status == CARMEN_PARAM_OK) {
    *return_value = response->value;
    if (expert)
      *expert = response->expert;
    FREE_RESPONSE_DOUBLE(response);
    return 1;
  }
  
  FREE_RESPONSE_DOUBLE(response);
  return 0;
}

int
carmen_param_get_onoff(const char *variable, int *return_value, int *expert)
{
  IPC_RETURN_TYPE err;
  int commandline_return;
  char buffer[1024];
  carmen_param_query_message query;
  carmen_param_response_onoff_message *response;
  static int initialized = 0;

  if (!initialized) {
    err = IPC_defineMsg(CARMEN_PARAM_QUERY_ONOFF_NAME,
			IPC_VARIABLE_LENGTH, 
			CARMEN_PARAM_QUERY_FMT);
    carmen_test_ipc_exit(err, "Could not define message", 
			 CARMEN_PARAM_QUERY_ONOFF_NAME);
    initialized = 1;
  }
  
  
  if (variable == NULL || variable[0] == '\0' || return_value == NULL) {
    sprintf(error_buffer, "Bad argument passed to %s", __FUNCTION__);
    return -1;
  }
  
  if (module_name) {
    sprintf(buffer, "%s_%s", module_name, variable);
    commandline_return = 
      param_check_commandline_onoff(buffer, return_value);
    if (commandline_return != 0)
      return commandline_return;
  }

  commandline_return = param_check_commandline_onoff(variable, return_value);
  if (commandline_return != 0)
    return commandline_return;

  query.timestamp = carmen_get_time();
  query.host = carmen_get_host();
  query.module_name = module_name;
  query.variable_name = (char*)variable;
  
  err = IPC_queryResponseData(CARMEN_PARAM_QUERY_ONOFF_NAME, &query, 
			      (void **)&response, timeout);
  carmen_test_ipc(err, "Could not query onoff parameter", 
		  CARMEN_PARAM_QUERY_ONOFF_NAME);
  if (err == IPC_Error || err == IPC_Timeout)
    {
      sprintf(error_buffer, "Did you remember to start the parameter server?\n"
	      "Remember, this program loads its parameters from the "
	      "param_daemon.\n");
      return -1;
    }

  if (response->status == CARMEN_PARAM_NOT_FOUND && 
      !allow_not_found_parameters) {
    sprintf(error_buffer, "The parameter server contains no definition "
	    "for %s_%s,\nrequested by this program. You may have started "
	    "the param_daemon with\nan out-of-date carmen.ini file. Or, this "
	    "may be a bug in this program\n(but probably not the parameter "
	    "server). \n", (!module_name ? "" : module_name), variable);
    return -1;
  }
  
  if (response->status == CARMEN_PARAM_OK) {
    *return_value = response->value;  
    if (expert)
      *expert = response->expert;
    FREE_RESPONSE_ONOFF(response);
    return 1;
  }
  
  FREE_RESPONSE_ONOFF(response);
  return 0;
}

int
carmen_param_get_string(const char *variable, char **return_value, int *expert)
{
  IPC_RETURN_TYPE err;
  int commandline_return;
  char buffer[1024];
  carmen_param_query_message query;
  carmen_param_response_string_message *response;
  static int initialized = 0;

  if (!initialized) {
    err = IPC_defineMsg(CARMEN_PARAM_QUERY_STRING_NAME,
			IPC_VARIABLE_LENGTH, 
			CARMEN_PARAM_QUERY_FMT);
    carmen_test_ipc_exit(err, "Could not define message", 
			 CARMEN_PARAM_QUERY_STRING_NAME);
    initialized = 1;
  }
  
  
  if (variable == NULL || variable[0] == '\0' || return_value == NULL) {
    sprintf(error_buffer, "Bad argument passed to %s", __FUNCTION__);
    return -1;
  }

  if (module_name) {
    sprintf(buffer, "%s_%s", module_name, variable);
    commandline_return = param_check_commandline_string(buffer, return_value);
    if (commandline_return != 0)
      return commandline_return;
  }

  commandline_return = param_check_commandline_string(variable, return_value);
  if (commandline_return != 0)
    return commandline_return;
  
  query.timestamp = carmen_get_time();
  query.host = carmen_get_host();
  query.module_name = module_name;
  query.variable_name = (char*)variable;
  
  err = IPC_queryResponseData(CARMEN_PARAM_QUERY_STRING_NAME, &query, 
			      (void **)&response, timeout);
  carmen_test_ipc(err, "Could not query string parameter", 
		  CARMEN_PARAM_QUERY_STRING_NAME);
  if (err == IPC_Error || err == IPC_Timeout) {
    sprintf(error_buffer, "Did you remember to start the parameter server?\n"
	    "Remember, this program loads its parameters from the "
	    "param_daemon.\n");
    return -1;
  }
  
  if (response->status == CARMEN_PARAM_NOT_FOUND && 
      !allow_not_found_parameters) {
    sprintf(error_buffer, "The parameter server contains no definition "
	    "for %s_%s,\nrequested by this program. You may have started "
	    "the param_daemon with\nan out-of-date carmen.ini file. Or, this "
	    "may be a bug in this program\n(but probably not the parameter "
	    "server). \n", (!module_name ? "" : module_name), variable);
    return -1;
  }
  
  if (response->status == CARMEN_PARAM_OK) {
    *return_value = (char *)calloc(strlen(response->value)+1, sizeof(char));
    carmen_test_alloc(*return_value);
    strcpy(*return_value, response->value);
    if (expert)
      *expert = response->expert;
    FREE_RESPONSE_STRING(response);
    return 1;
  }
  
  FREE_RESPONSE_STRING(response);

  return 0;
}

int
carmen_param_get_filename(const char *variable, char **return_value, int *expert)
{
  IPC_RETURN_TYPE err;
  int commandline_return;
  carmen_param_query_message query;
  carmen_param_response_string_message *response;
  char buffer[1024];
  static int initialized = 0;

  if (!initialized) {
    err = IPC_defineMsg(CARMEN_PARAM_QUERY_STRING_NAME,
			IPC_VARIABLE_LENGTH, 
			CARMEN_PARAM_QUERY_FMT);
    carmen_test_ipc_exit(err, "Could not define message", 
			 CARMEN_PARAM_QUERY_STRING_NAME);
    initialized = 1;
  }
  
  
  if (variable == NULL || variable[0] == '\0' || return_value == NULL) {
    sprintf(error_buffer, "Bad argument passed to %s", __FUNCTION__);
    return -1;
  }
  
  if (module_name) {
    sprintf(buffer, "%s_%s", module_name, variable);
    commandline_return = 
      param_check_commandline_filename(buffer, return_value);
    if (commandline_return != 0)
      return commandline_return;
  }

  commandline_return = 
    param_check_commandline_filename(variable, return_value);
  if (commandline_return != 0)
    return commandline_return;
  
  query.timestamp = carmen_get_time();
  query.host = carmen_get_host();
  query.module_name = module_name;
  query.variable_name = (char*)variable;
  
  err = IPC_queryResponseData(CARMEN_PARAM_QUERY_STRING_NAME, &query, 
			      (void **)&response, timeout);
  carmen_test_ipc(err, "Could not query filename parameter", 
		  CARMEN_PARAM_QUERY_STRING_NAME);
  if (err == IPC_Error || err == IPC_Timeout) {
    sprintf(error_buffer, "Did you remember to start the parameter server?\n"
	    "Remember, this program loads its parameters from the "
	    "param_daemon.\n");
    return -1;
  }
  
  if (response->status == CARMEN_PARAM_NOT_FOUND && 
      !allow_not_found_parameters)
    {
      sprintf(error_buffer, "The parameter server contains no definition "
	      "for %s_%s,\nrequested by this program. You may have started "
	      "the param_daemon with\nan out-of-date carmen.ini file. Or, this "
	      "may be a bug in this program\n(but probably not the parameter "
	      "server). \n", (!module_name ? "" : module_name), variable);
      return -1;
    }

  if (response->status == CARMEN_PARAM_OK) {
    *return_value = (char *)calloc(strlen(response->value)+1, sizeof(char));
    carmen_test_alloc(*return_value);
    strcpy(*return_value, response->value);
    if (expert)
      *expert = response->expert;
    FREE_RESPONSE_STRING(response);
    return 1;
  }
  
  FREE_RESPONSE_STRING(response);
  return 0;
}

int
carmen_param_get_directory(char *variable, char **return_value, int *expert)
{
  IPC_RETURN_TYPE err;
  int commandline_return;
  carmen_param_query_message query;
  carmen_param_response_string_message *response;
  char buffer[1024];
  static int initialized = 0;

  if (!initialized) {
    err = IPC_defineMsg(CARMEN_PARAM_QUERY_STRING_NAME,
			IPC_VARIABLE_LENGTH, 
			CARMEN_PARAM_QUERY_FMT);
    carmen_test_ipc_exit(err, "Could not define message", 
			 CARMEN_PARAM_QUERY_STRING_NAME);
    initialized = 1;
  }
  
  
  if (variable == NULL || variable[0] == '\0' || return_value == NULL) {
    sprintf(error_buffer, "Bad argument passed to %s", __FUNCTION__);
    return -1;
  }
  
  if (module_name) {
    sprintf(buffer, "%s_%s", module_name, variable);
    commandline_return = 
      param_check_commandline_directory(buffer, return_value);
    if (commandline_return != 0)
      return commandline_return;
  }

  commandline_return = 
    param_check_commandline_directory(variable, return_value);
  if (commandline_return != 0)
    return commandline_return;
  
  query.timestamp = carmen_get_time();
  query.host = carmen_get_host();
  query.module_name = module_name;
  query.variable_name = variable;
  
  err = IPC_queryResponseData(CARMEN_PARAM_QUERY_STRING_NAME, &query, 
			      (void **)&response, timeout);
  carmen_test_ipc(err, "Could not query filename parameter", 
		  CARMEN_PARAM_QUERY_STRING_NAME);
  if (err == IPC_Error || err == IPC_Timeout)
    {
      sprintf(error_buffer, "Did you remember to start the parameter server?\n"
	      "Remember, this program loads its parameters from the "
	      "param_daemon.\n");
      return -1;
    }

  if (response->status == CARMEN_PARAM_NOT_FOUND && 
      !allow_not_found_parameters) {
    sprintf(error_buffer, "The parameter server contains no definition "
	    "for %s_%s,\nrequested by this program. You may have started "
	    "the param_daemon with\nan out-of-date carmen.ini file. Or, this "
	    "may be a bug in this program\n(but probably not the parameter "
	    "server). \n", (!module_name ? "" : module_name), variable);
    return -1;
  }
  
  if (response->status == CARMEN_PARAM_OK) {
    *return_value = (char *)calloc(strlen(response->value)+1, sizeof(char));
    carmen_test_alloc(*return_value);
    strcpy(*return_value, response->value);
    if (expert)
      *expert = response->expert;
    FREE_RESPONSE_STRING(response);
    return 1;
  }
  
  FREE_RESPONSE_STRING(response);
  return 0;
}

void
carmen_param_set_module(const char *new_module_name)
{
  if (module_name) {
    free(module_name);
    module_name = NULL;
  }
  
  if (new_module_name == NULL || new_module_name[0] == '\0')
    return;

  module_name = (char *)calloc(strlen(new_module_name)+1, sizeof(char));
  carmen_test_alloc(module_name);
  strcpy(module_name, new_module_name);
}

char *
carmen_param_get_module(void)
{
  return module_name;
}

void
carmen_param_allow_unfound_variables(int new_value)
{
  allow_not_found_parameters = new_value;
}

int
carmen_param_are_unfound_variables_allowed(void)
{
  return allow_not_found_parameters;
}

int
carmen_param_set_variable(const char *variable, const char *new_value, char **return_value)
{
  IPC_RETURN_TYPE err;

  carmen_param_set_message query;
  carmen_param_response_string_message *response;
  static int initialized = 0;

  if (!initialized) {
    err = IPC_defineMsg(CARMEN_PARAM_SET_NAME,
			IPC_VARIABLE_LENGTH, 
			CARMEN_PARAM_SET_FMT);
    carmen_test_ipc_exit(err, "Could not define message", 
			 CARMEN_PARAM_SET_NAME);
    initialized = 1;
  }  
  
  if (variable == NULL || variable[0] == '\0' || new_value == NULL || 
      new_value[0] == '\0')
    return -1;
  
  query.timestamp = carmen_get_time();
  query.host = carmen_get_host();
  query.module_name = module_name;
  query.variable_name = (char*)variable;
  query.value = (char*)new_value;
  
  err = IPC_queryResponseData(CARMEN_PARAM_SET_NAME, &query, 
			      (void **)&response, timeout);
  carmen_test_ipc(err, "Could not set variable", CARMEN_PARAM_SET_NAME);
  if (err == IPC_Error || err == IPC_Timeout) {
    sprintf(error_buffer, "Did you remember to start the parameter server?\n"
	    "Remember, this program loads its parameters from the "
	    "param_daemon.\n");
    return -1;
  }
  
  if (response->status == CARMEN_PARAM_OK) {
    if (return_value) {
      *return_value = (char *)calloc(strlen(response->value)+1, 
				     sizeof(char));
      carmen_test_alloc(*return_value);
      strcpy(*return_value, response->value);
    }
    FREE_RESPONSE_STRING(response);
    return 0;
  }
  else
    carmen_warn("Error: %d\n", response->status);
  
  FREE_RESPONSE_STRING(response);
  return -1;
}

char *
carmen_param_get_error(void) 
{
  return error_buffer;
}

void
carmen_param_set_usage_line(const char *fmt, ...)
{
  va_list args;

  if (usage_line == NULL)
    free(usage_line);
  usage_line = (char *)calloc(1024, sizeof(char));
  carmen_test_alloc(usage_line);

  va_start(args, fmt);
  /* This may be a bug -- should we be checking at every 
     point how many chars have been written?*/
  vsnprintf(usage_line, 1024, fmt, args);
  va_end(args);
}

void 
carmen_param_usage(char *progname, carmen_param_p param_list, int num_items, 
		   char *fmt, ...) 
{
  va_list args;
  int index;

  if (fmt != NULL) {
    fprintf(stderr, "\n%s", carmen_red_code);
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);
    fprintf(stderr, "%s\n\n", carmen_normal_code);
  } else {
    if (strrchr(progname, '/') != NULL) {
      progname = strrchr(progname, '/');
      progname++;
    }
      
    if (param_list == NULL) {
      param_list = installed_parameters;
      num_items = installed_list_length;
    }
    
      if (usage_line == NULL)
	fprintf(stderr, "Usage: %s\n", progname);
      else
	fprintf(stderr, "%s\n", usage_line);
      
      for (index = 0; index < num_items; index++) {
	if (!param_list[index].variable)
	  continue;
	fprintf(stderr, "\t-%s ", param_list[index].variable);
	switch (param_list[index].type) {
	case CARMEN_PARAM_INT:
	  fprintf(stderr, "%%d");
	  break;
	case CARMEN_PARAM_DOUBLE:
	  fprintf(stderr, "%%f");
	  break;
	case CARMEN_PARAM_ONOFF:
	  fprintf(stderr, "{on|off}");
	  break;
	case CARMEN_PARAM_STRING:
	  fprintf(stderr, "%%s");
	  break;
	case CARMEN_PARAM_FILE:
	  fprintf(stderr, "<filename>");
	  break;
	case CARMEN_PARAM_DIR:
	  fprintf(stderr, "<directory>");
	  break;
	}
	
	fprintf(stderr, "[50G Autoupdate : %s\n", 
		(param_list[index].subscribe ? "on" : "off"));
      }
  }
  exit(-1);
}

int
carmen_param_install_params(int argc, char *argv[], carmen_param_p param_list, 
			    int num_items) 
{
  int index;
  int err = 0;
  int last_command_line_arg;
  //char *prog_name;
  int expert;

  last_command_line_arg = carmen_read_commandline_parameters(argc, argv);

  if (carmen_find_param("h") || carmen_find_param("help"))
    carmen_param_usage(argv[0], param_list, num_items, NULL);

  if (carmen_find_param("robot") || carmen_find_param("pearl") || 
      carmen_find_param("dorothy") || carmen_find_param("flo") || 
      carmen_find_param("robin"))
    carmen_param_usage(argv[0], param_list, num_items, 
		       "%s no longer takes a robot name as an argument.\n"
		       "It loads parameter settings from the param_daemon.", 
		       argv[0]);

  for (index = 0; index < num_items; index++) {
    carmen_param_set_module(param_list[index].module);
    
  //  prog_name = carmen_extract_filename(argv[0]);
    
    switch (param_list[index].type) {
    case CARMEN_PARAM_INT:
      err = carmen_param_get_int(param_list[index].variable, 
				 param_list[index].user_variable, &expert);
      carmen_verbose("%s_%s %s: %d\n", param_list[index].module,
		     param_list[index].variable, expert ? "[expert]" : "",
		     *((int *)(param_list[index].user_variable)));
      break;
    case CARMEN_PARAM_DOUBLE:
      err = carmen_param_get_double(param_list[index].variable, 
				    param_list[index].user_variable, &expert);
      carmen_verbose("%s_%s %s: %f\n", param_list[index].module,
		     param_list[index].variable, expert ? "[expert]" : "", 
		     *((double *)(param_list[index].user_variable)));
      break;
    case CARMEN_PARAM_ONOFF:
      err = carmen_param_get_onoff(param_list[index].variable,
				   param_list[index].user_variable, &expert);
      carmen_verbose("%s_%s %s: %s\n", param_list[index].module,
		     param_list[index].variable, expert ? "[expert]" : "",
		     (*((int *)(param_list[index].user_variable)) == 0 ? 
		      "off" : "on"));
      break;
    case CARMEN_PARAM_STRING:
      err = carmen_param_get_string(param_list[index].variable, 
				    param_list[index].user_variable, &expert);
      carmen_verbose("%s_%s %s: %s\n", param_list[index].module,
		     param_list[index].variable, expert ? "[expert]" : "",
		     (*(char **)(param_list[index].user_variable)));
      break;
    case CARMEN_PARAM_FILE:
      err = carmen_param_get_filename(param_list[index].variable, 
				      param_list[index].user_variable, &expert);
      carmen_verbose("%s_%s %s: %s\n", param_list[index].module,
		     param_list[index].variable, expert ? "[expert]" : "",
		     (*(char **)(param_list[index].user_variable)));
      break;
    case CARMEN_PARAM_DIR:
      err = carmen_param_get_directory(param_list[index].variable, 
				       param_list[index].user_variable, &expert);
      carmen_verbose("%s_%s %s: %s\n", param_list[index].module,
		     param_list[index].variable, expert ? "[expert]" : "",
		     (*(char **)(param_list[index].user_variable)));
      break;
    } /* switch (param_list[index].type) */
    if (err < 0)
      carmen_param_usage((char *)argv[0], param_list, num_items,
    		  "%s", carmen_param_get_error());
    install_parameter(param_list[index].module, param_list[index].variable, 
		      param_list[index].user_variable, 
		      param_list[index].type, param_list[index].subscribe, 
		      param_list[index].handler);
  }
  
  return last_command_line_arg;
}


void 
carmen_param_check_unhandled_commandline_args(int argc __attribute__ 
					      ((unused)), char *argv[])
{
  int num_params;

  num_params = carmen_num_params();
  if (num_params < 1)
    return;

  carmen_param_usage(argv[0], installed_parameters, installed_list_length, 
		     "Unknown command line argument '%s'", 
		     carmen_get_param_by_num(0));
}


static carmen_param_p
get_table_entry(char *module, char *variable)
{
  int index;
  carmen_param_t *param;

  if (!module && !variable)
    return NULL;

  for (index = 0; index < installed_list_length; index++) {
    param = installed_parameters+index;
    if (strcmp(param->variable, variable) == 0) {
      if ((!module && !(param->module)) ||
	  (strcmp(module, param->module) == 0))
	return param;
    }
  }
  
  return NULL;
}

static void
check_table_capacity(void)
{
  carmen_param_t *new_table;

  if (installed_parameters == NULL) {
    installed_list_capacity = 100;
    installed_list_length = 0;
    installed_parameters = (carmen_param_t *)
      calloc(installed_list_capacity, sizeof(carmen_param_t));
    carmen_test_alloc(installed_parameters);
  }
  else if (installed_list_capacity == installed_list_length) {
    new_table = (carmen_param_t *)
      realloc(installed_parameters, 
	      installed_list_capacity*2*sizeof(carmen_param_t));
    carmen_test_alloc(new_table);
    installed_parameters = new_table;
    installed_list_capacity *= 2;
  }
}

static void
start_change_subscription()
{
  static int started = 0;
  int err;

  if (started) 
    return;

  err = IPC_subscribe(CARMEN_PARAM_VARIABLE_CHANGE_NAME, change_handler, NULL);
  carmen_test_ipc(err, "Could not subscribe", 
		  CARMEN_PARAM_VARIABLE_CHANGE_NAME);      
  IPC_setMsgQueueLength(CARMEN_PARAM_VARIABLE_CHANGE_NAME, 100);

  started = 1;
}
  
static void
install_parameter(char *module, char *variable, void *variable_address, 
		  carmen_param_type_t type, int subscribe, 
		  carmen_param_change_handler_t handler)
{
  int num_entries = installed_list_length;

  check_table_capacity();

  if (!variable)
    return;

  if (module) 
    {
      installed_parameters[num_entries].module = (char *)
	calloc(strlen(module)+1, sizeof(char));
      carmen_test_alloc(installed_parameters[num_entries].module);
      strcpy(installed_parameters[num_entries].module, module);
    }
  else
    installed_parameters[num_entries].module = NULL;

  installed_parameters[num_entries].variable = (char *)
    calloc(strlen(variable)+1, sizeof(char));
  carmen_test_alloc(installed_parameters[num_entries].variable);
  strcpy(installed_parameters[num_entries].variable, variable);

  installed_parameters[num_entries].type = type;
  installed_parameters[num_entries].user_variable = variable_address;

  installed_parameters[num_entries].subscribe = subscribe;  

  if (subscribe)
    start_change_subscription();

  installed_parameters[num_entries].handler = handler;  

  installed_list_length++;

  carmen_verbose("Installed %s_%s, got %d parameters\n",
		 module, variable, installed_list_length);
}

static void 
change_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
	       void *clientData __attribute__ ((unused)))
{
  FORMATTER_PTR formatter;
  IPC_RETURN_TYPE err = IPC_OK;
  carmen_param_variable_change_message msg;
  carmen_param_t *table_entry;
  carmen_param_type_t table_type;
  char *endptr;

  int new_int;
  int *int_address, *onoff_address;
  double new_double;
  double *double_address;
  char **string_address;

  carmen_param_change_handler_t handler;

  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &msg, 
			   sizeof(carmen_param_variable_change_message));
  IPC_freeByteArray(callData);
  carmen_test_ipc(err, "Could not unmarshall", IPC_msgInstanceName(msgRef));

  carmen_verbose("Received parameter update : %s_%s = %s\n",
		 msg.module_name, msg.variable_name, msg.value);

  table_entry = get_table_entry(msg.module_name, msg.variable_name);
  if (table_entry != NULL && table_entry->subscribe == 1) {
    table_type = table_entry->type;
    switch(table_type) {
    case CARMEN_PARAM_INT:
      if (strlen(msg.value) > 254) {
	carmen_warn("Received variable change notice : bad variable\n"
		    "value %s for variable %s_%s\n", msg.value,
		    msg.module_name, msg.variable_name);
	return;
      }
      new_int = strtol(msg.value, &endptr, 0);
      if (endptr == msg.value) {
	carmen_warn("Received variable change notice : could not\n"
		    "convert %s to type int for variable %s_%s\n", 
		    msg.value, msg.module_name, msg.variable_name);
	return;
      }
      int_address = table_entry->user_variable;
      if (int_address)
	*int_address = new_int;
      break;
    case CARMEN_PARAM_DOUBLE:
      if (strlen(msg.value) > 254) {
	carmen_warn("Received variable change notice : bad variable\n"
		    "value %s for variable %s_%s\n", msg.value,
		    msg.module_name, msg.variable_name);
	return;
      }
      new_double = strtod(msg.value, &endptr);
      if (endptr == msg.value) {
	carmen_warn("Received variable change notice : could not\n"
		    "convert %s to type double for variable %s_%s\n", 
		    msg.value, msg.module_name, msg.variable_name);
	return;
      }
      double_address = table_entry->user_variable;
      if (double_address) 
	*double_address = new_double;
      break;
    case CARMEN_PARAM_ONOFF:
      if (strlen(msg.value) > 254) {
	carmen_warn("Received variable change notice : bad variable\n"
		    "value %s for variable %s_%s\n", msg.value,
		    msg.module_name, msg.variable_name);
	return;
      }
      if (carmen_strncasecmp(msg.value, "ON", 2) == 0)
	new_int = 1;
      else if (carmen_strncasecmp(msg.value, "OFF", 3) == 0)
	new_int = 0;
      else {
	carmen_warn("Received variable change notice : could not\n"
		    "convert %s to type on/off for variable %s_%s\n", 
		    msg.value, msg.module_name, msg.variable_name);
	return;
      }
      onoff_address = table_entry->user_variable;
      if (onoff_address) 
	*onoff_address = new_int;
      break;
    case CARMEN_PARAM_STRING:
    case CARMEN_PARAM_FILE:
    case CARMEN_PARAM_DIR:
      string_address = table_entry->user_variable;
      if (string_address) {
	if (*string_address)
	  free(*string_address);
	*string_address = 
	  (char *)calloc(strlen(msg.value)+1, sizeof(char));
	carmen_test_alloc(*string_address);
	strcpy(*string_address, msg.value);
      }
      break;
    } /* switch (table_type) { */
    handler = table_entry->handler;
    if (handler)
      (*handler)(msg.module_name, msg.variable_name, msg.value);
  }

  IPC_freeDataElements(formatter, &msg);
}

void
carmen_param_subscribe_int(char *module, char *variable, 
			   int *variable_address, 
			   carmen_param_change_handler_t handler)
{
  carmen_param_p table_entry;

  table_entry = get_table_entry(module, variable);
  if (table_entry == NULL)
    install_parameter(module, variable, (int *)variable_address, 
		      CARMEN_PARAM_INT, 1, handler);
  else 
    {
      table_entry->subscribe = 1;
      start_change_subscription();
    }
}

void
carmen_param_subscribe_double(char *module, char *variable, 
			      double *variable_address, 
			      carmen_param_change_handler_t handler)
{
  carmen_param_p table_entry;

  table_entry = get_table_entry(module, variable);
  if (table_entry == NULL)
    install_parameter(module, variable, (int *)variable_address, 
		      CARMEN_PARAM_DOUBLE, 1, handler);
  else {
    table_entry->subscribe = 1;
    start_change_subscription();
  }
}

void
carmen_param_subscribe_onoff(char *module, char *variable, 
			     int *variable_address, 
			     carmen_param_change_handler_t handler)
{
  carmen_param_p table_entry;

  table_entry = get_table_entry(module, variable);
  if (table_entry == NULL)
    install_parameter(module, variable, (int *)variable_address, 
		      CARMEN_PARAM_ONOFF, 1, handler);
  else {
    table_entry->subscribe = 1;
    start_change_subscription();
  }
}

void
carmen_param_subscribe_string(char *module, char *variable, 
			      char **variable_address, 
			      carmen_param_change_handler_t handler)
{
  carmen_param_p table_entry;

  table_entry = get_table_entry(module, variable);
  if (table_entry == NULL)
    install_parameter(module, variable, (int *)variable_address, 
		      CARMEN_PARAM_STRING, 1, handler);
  else {
    table_entry->subscribe = 1;
    start_change_subscription();
  }
}

void
carmen_param_subscribe_file(char *module, char *variable, 
			    char **variable_address, 
			    carmen_param_change_handler_t handler)
{
  carmen_param_p table_entry;

  table_entry = get_table_entry(module, variable);
  if (table_entry == NULL)
    install_parameter(module, variable, (int *)variable_address, 
		      CARMEN_PARAM_FILE, 1, handler);
  else {
    table_entry->subscribe = 1;
    start_change_subscription();
  }
}

void
carmen_param_subscribe_dir(char *module, char *variable, 
			   char **variable_address, carmen_param_change_handler_t handler)
{
  carmen_param_p table_entry;

  table_entry = get_table_entry(module, variable);
  if (table_entry == NULL)
    install_parameter(module, variable, (int *)variable_address, 
		      CARMEN_PARAM_DIR, 1, handler);
  else {
    table_entry->subscribe = 1;
    start_change_subscription();
  }
}

int
carmen_param_check_version(char *prog_name)
{
  IPC_RETURN_TYPE err;

  carmen_param_query_version_message *query;
  carmen_param_version_message *response;
  static int initialized = 0;

  if (!initialized) {
    err = IPC_defineMsg(CARMEN_PARAM_VERSION_QUERY_NAME,
			IPC_VARIABLE_LENGTH, 
			CARMEN_DEFAULT_MESSAGE_FMT);
    carmen_test_ipc_exit(err, "Could not define message", 
			 CARMEN_PARAM_VERSION_QUERY_NAME);
    initialized = 1;
  }  

  query = carmen_default_message_create();
  err = IPC_queryResponseData(CARMEN_PARAM_VERSION_QUERY_NAME, query, 
			      (void **)&response, timeout);
  carmen_test_ipc(err, "Could not check Carmen version", 
		  CARMEN_PARAM_VERSION_QUERY_NAME);
  if (err == IPC_Error || err == IPC_Timeout) {
    sprintf(error_buffer, "Did you remember to start the parameter server?\n"
	    "Remember, this program loads its parameters from the "
	    "param_daemon.\n");
    return -1;
  }
  
  if (response->major != CARMEN_MAJOR_VERSION || 
      response->minor != CARMEN_MINOR_VERSION)
    carmen_die("Version mismatch: %s is Carmen version %d.%d, but \n"
	       "param_daemon is Carmen version %d.%d\n", prog_name, 
	       CARMEN_MAJOR_VERSION, CARMEN_MINOR_VERSION, response->major, 
	       response->minor);

  free(response->host);
  free(response);
  
  return 0;
}

int 
carmen_param_set_int(const char *variable, int new_value, int *return_value)
{
  char buffer[255];
  char *return_string = NULL;
  int err;

  sprintf(buffer, "%d", new_value);
  if (return_value) {
    err = carmen_param_set_variable(variable, buffer, &return_string);
    if (return_string) {
      sscanf(return_string, "%d", return_value);
      free(return_string);
    }
  }
  else
    err = carmen_param_set_variable(variable, buffer, NULL);

  return err;
}

int 
carmen_param_set_double(const char *variable, double new_value, double *return_value)
{
  char buffer[255];
  char *return_string = NULL;
  int err;

  sprintf(buffer, "%f", new_value);
  if (return_value) {
    err = carmen_param_set_variable(variable, buffer, &return_string);
    if (return_string) {
      sscanf(return_string, "%lf", return_value);
      free(return_string);
    }
  }
  else
    err = carmen_param_set_variable(variable, buffer, NULL);

  return err;
}

int 
carmen_param_set_onoff(const char *variable, int new_value, int *return_value)
{
  char buffer[255];
  char *return_string = NULL;
  int err;

  sprintf(buffer, "%s", (new_value ? "on" : "off"));
  if (return_value) {
    err = carmen_param_set_variable(variable, buffer, &return_string);
    if (return_string) {
      sscanf(return_string, "%d", return_value);
      free(return_string);
    }
  }
  else
    err = carmen_param_set_variable(variable, buffer, NULL);

  return err;

}

int carmen_param_set_string(const char *variable, const char *new_value, 
			    char **return_value)
{
  return carmen_param_set_variable(variable, new_value, return_value);
}

int 
carmen_param_set_filename(const char *variable, const char *new_value, char **return_value)
{
  return carmen_param_set_variable(variable, new_value, return_value);
}

int 
carmen_param_get_paramserver_host(char **hostname)
{
  IPC_RETURN_TYPE err;

  carmen_param_query_version_message *query;
  carmen_param_version_message *response;
  static int initialized = 0;

  if (!initialized) {
    err = IPC_defineMsg(CARMEN_PARAM_VERSION_QUERY_NAME,
			IPC_VARIABLE_LENGTH, 
			CARMEN_DEFAULT_MESSAGE_FMT);
    carmen_test_ipc_exit(err, "Could not define message", 
			 CARMEN_PARAM_VERSION_QUERY_NAME);
    initialized = 1;
  }

  query = carmen_default_message_create();
  err = IPC_queryResponseData(CARMEN_PARAM_VERSION_QUERY_NAME, query, 
			      (void **)&response, timeout);
  carmen_test_ipc_return_int(err, "Could not check Carmen version", 
			     CARMEN_PARAM_VERSION_QUERY_NAME);
  *hostname = (char *)calloc(strlen(response->host)+1, sizeof(char));
  carmen_test_alloc(*hostname);
  strcpy(*hostname, response->host);

  return 0;
}

void
carmen_param_load_paramfile(const char *filename, const char *param_set)
{
  FILE *fp;
  char line[2000], *err, param[1024], value[1024];
  int count = 0;
  int param_err;
  int skipping = 0;
  int line_num = 0;
  char *mark, *token, *dummy;
  int token_num;
  int line_length;

  if (!carmen_file_exists(filename))
    carmen_warn("Error loading parameters: %s does not exist.\n", filename);
    
  fp = fopen(filename, "r");
  if (fp == NULL) 
    carmen_die_syserror("Error loading parameters: cannot open %s for reading.\n", filename);

  err = fgets(line, 2000, fp);
  for (; err != NULL; err = fgets(line, 2000, fp)) {
    line_num++;
    line[1999] = '\0';
    if (strlen(line) == 1999) 
      carmen_die("Error loading parameters: line %d of file %s is too long.\n"
		 "Maximum line length is %d. Please correct this line.\n\n"
		 "It is also possible that this file has become corrupted.\n"
		 "Make sure you have an up-to-date version of carmen, and\n"
		 "consult the param_server documentation to make sure the\n"
		 "file format is valid.\n", line_num, filename, 2000);
    
    if (feof(fp))
      break;
    mark = strchr(line, '#');    /* strip comments and trailing returns */
    if (mark != NULL)
      mark[0] = '\0';
    mark = strchr(line, '\n');
    if (mark != NULL)
      mark[0] = '\0';
    
    // Trim off trailing white space 
    
    line_length = strlen(line) - 1;
    while (line_length >= 0 && 
	   (line[line_length] == ' ' || line[line_length] == '\t' ))
      line[line_length--] = '\0';
  
    line_length++;
    
    if (line_length == 0)
      continue;
    
    // Skip over initial blank space
    
    mark = line + strspn(line, " \t");
    if (strlen(mark) == 0) 
      carmen_die("You have encountered a bug in carmen. Please report it\n"
		 "to the carmen maintainers. \n"
		 "Line %d, function %s, file %s\n", __LINE__, __FUNCTION__,
		 __FILE__);
    
    token_num = 0;
    
    /* tokenize line */
    token = mark;
    
    // Move mark to the first whitespace character.
    mark = strpbrk(mark, " \t");
    // If we found a whitespace character, then turn it into a NULL
    // and move mark to the next non-whitespace.
    if (mark) {
      mark[0] = '\0';
      mark++;
      mark += strspn(mark, " \t");
    }
    
    if (strlen(token) > 254) {
      carmen_warn("Error loading parameters: bad file format of %s on line %d.\n"
		  "The parameter name %s is too long (%d characters).\n"
		  "A parameter name can be no longer than 254 "
		  "characters.\nSkipping this line.\n", filename, 
		  count, token, (int) strlen(token));
      continue;
    }
    
    strcpy(param, token);
    token_num++;
    
    // If mark points to a non-whitespace character, then we have a
    // two-token line
    if (mark) {
      if (strlen(mark) > 1999) {
	carmen_warn("Error loading parameters: bad file format of %s on line %d.\n"
		    "The parameter value %s is too long (%d "
		    "characters).\nA parameter value can be no longer "
		    "than %d characters.\nSkipping this line.\n", 
		    filename, count, mark, (int) strlen(mark),
		    1999);
	continue;
      }
      strcpy(value, mark);
      token_num++;
    }
    
    if (param[0] == '[') {
      if (param[1] == '*')
	skipping = 0;
      else if (strcspn(param+1,"]") == 6 && !carmen_strncasecmp(param+1, "expert", 6))
	skipping = 0;
      else if (param_set != NULL && 
	       carmen_strncasecmp(param+1, param_set, strlen(param_set)) == 0)
	skipping = 0;	    
      else
	skipping = 1;
      continue;
    }
    else if(token_num == 2 && !skipping) {
      param_err = carmen_param_get_string(param, &dummy, NULL);
      if (param_err >= 0)
	carmen_warn("Overwriting parameter %s from %s: new value = %s.\n",
		    param, carmen_extract_filename((char*)filename), value);	  
      param_err = carmen_param_set_variable(param, value, NULL);
      if (param_err < 0)
	carmen_warn("Couldn't set parameter %s\n", param);
      else {
	count++;
      }
    }
  } 
  
  carmen_warn("Set %d parameters\n", count);
}

void carmen_param_send_reread(void)
{
  IPC_RETURN_TYPE err = IPC_OK;
  carmen_param_reread_command_message msg;
  static int initialized = 0;

  if (!initialized) {
    err = IPC_defineMsg(CARMEN_PARAM_REREAD_COMMAND_NAME,
			IPC_VARIABLE_LENGTH, 
			CARMEN_DEFAULT_MESSAGE_FMT);
    carmen_test_ipc_exit(err, "Could not define message", 
			 CARMEN_PARAM_REREAD_COMMAND_NAME);
    initialized = 1;
  }

  msg.timestamp = carmen_get_time();
  msg.host = carmen_get_host();
  
  err = IPC_publishData(CARMEN_PARAM_REREAD_COMMAND_NAME, &msg);
  carmen_test_ipc(err, "Could not publish", CARMEN_PARAM_REREAD_COMMAND_NAME);

}

void carmen_param_subscribe_started_message(carmen_param_started_message* msg,
    carmen_handler_t handler,
    carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_PARAM_STARTED_NAME,
      CARMEN_PARAM_STARTED_FMT,
      msg, sizeof(carmen_param_started_message), handler,
      subscribe_how);
}
