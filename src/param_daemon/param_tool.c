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
#include <getopt.h>

static void main_usage(char *prog_name)
{
  carmen_die("\nUsage: %s <action> <...>\n\n"
	     "<action> is one of: set, get, install, reread.\n"
	     "Run %s help <action> to get help on using each action.\n\n",
	     prog_name, prog_name);  
}

static int handle_options(int argc,  char *argv[])
{
  static struct option long_options[] = {
    {"help", 0, NULL, 'h'},
    {0, 0, 0, 0}
  };

  int option_index = 0;
  int c;

  opterr = 0;
  while (1) {
    c = getopt_long (argc, argv, "h", long_options, &option_index);
    if (c == -1)
      break;
    switch (c) {
    case 'h':
    case '?':
      main_usage(argv[0]);
    default:
      carmen_warn("Unknown option character %c", optopt);
      main_usage(argv[0]);
      break;
    }
  }

  return optind;
}

static void print_all(char *module_name)
{
  char **variables, **values;
  int max_variable_width;
  int list_length;
  int index;
  char buffer[255];

  if (carmen_param_get_all(module_name, &variables, &values, NULL, &list_length) < 0) {
    IPC_perror("Error retrieving all variables of module");
    exit(-1);
  }
  max_variable_width = 0;
  for (index = 0; index < list_length; index++)
    max_variable_width = carmen_fmax(max_variable_width, 
				     strlen(variables[index]));
  
  if (max_variable_width > 60)
    max_variable_width = 60;
  
  max_variable_width += 5;
  printf("\nVariable list for module [31;1m%s[0m\n", module_name);
  memset(buffer, '-', max_variable_width+15);
  buffer[max_variable_width+15] = '\0';
  printf("%s\n", buffer);
  for (index = 0; index < list_length; index++) {
    printf("%s[%dC%s\n", variables[index],
	   max_variable_width - (int)strlen(variables[index]),
	   values[index]);
    free(variables[index]);
    free(values[index]);
  }
  free(variables);
  free(values);
  printf("\n");
}

static int get(int argc, char** argv)
{
  int param_error;
  char **modules;
  int num_modules;
  int next_arg;
  char *return_string = NULL;
  int i;

  next_arg = handle_options(argc, argv);
  next_arg++;
  if (argc - next_arg != 1) {
    carmen_warn("\nError: wrong number of parameters.\n");    
    carmen_die("\nUsage: %s get <module name | parameter name>\n\n", 
	       argv[0]);
  }

  param_error = carmen_param_get_modules(&modules, &num_modules);
  if (param_error < 0) 
    carmen_die("%s\n", carmen_param_get_error());

  for (i = 0; i < num_modules; i++) {
    if (carmen_strcasecmp(argv[next_arg], modules[i]) == 0) {
      print_all(modules[i]);
      return 0;
    }
  }

  param_error = carmen_param_get_string(argv[next_arg], &return_string, NULL);    
  if (!return_string)
    carmen_die("Could not retrieve %s from param_daemon.\n", argv[next_arg]);

  carmen_warn("%s = %s\n", argv[next_arg], return_string);
  free(return_string);

  return 0;
}


static int set(int argc, char** argv)
{
  char *return_string = NULL;
  int next_arg;

  next_arg = handle_options(argc, argv);
  next_arg++;

  if (argc - next_arg != 2) {
    carmen_warn("\nError: wrong number of parameters.\n");    
    carmen_die("\nUsage: %s set <parameter name> <parameter value>\n\n", 
	       argv[0]);
  }

  if (carmen_param_set_variable(argv[next_arg], argv[next_arg+1], &return_string) < 0) {
    IPC_perror("Error setting variable");
    exit(-1);
  }
  
  printf("%s = %s\n", argv[next_arg], return_string);
  free(return_string);
  
  return 0;
}

void publish_params_from_logfile(FILE *fp)
{
  char line[2000], *err, tag[1024], param[1024], value[1024];
  int count = 0;
  int param_err;

  err = fgets(line, 2000, fp);
  while(err != NULL) {
    if(line[0] != '#') {
      sscanf(line, "%s", tag);
        if(strcmp(tag, "PARAM") == 0) {
	  sscanf(line, "%*s %s %s", param, value);
	  param_err = carmen_param_set_variable(param, value, NULL);
	  if (param_err == -1)
	    exit(-1);
	  count++;
	}
    }
    err = fgets(line, 2000, fp);
  }

  carmen_warn("Set %d parameters\n", count);
}

void publish_params_from_paramfile(FILE *fp, char *param_label, char *filename)
{
  char line[2000], *err, param[1024], value[1024];
  int count = 0;
  int param_err;
  int skipping = 0;
  int line_num = 0;
  char *mark, *token;
  int token_num;
  int line_length;

  err = fgets(line, 2000, fp);
  for (; err != NULL; err = fgets(line, 2000, fp)) {
    line_num++;
    line[1999] = '\0';
    if (strlen(line) == 1999) 
      carmen_die("Line %d of file %s is too long.\n"
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
    if (mark) 
      {
	mark[0] = '\0';
	mark++;
	mark += strspn(mark, " \t");
      }
    
    if (strlen(token) > 254) {
      carmen_warn("Bad file format of %s on line %d.\n"
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
	carmen_warn("Bad file format of %s on line %d.\n"
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
      else if (param_label != NULL && 
	       carmen_strncasecmp(param+1, param_label, strlen(param_label)) == 0)
	skipping = 0;	    
      else
	skipping = 1;
      continue;
    } else if(token_num == 2 && !skipping) {
      param_err = carmen_param_set_variable(param, value, NULL);
      if (param_err < 0)
	carmen_warn("Couldn't set parameter %s\n", param);
      else
	count++;
    }
  } 
  
  carmen_warn("Set %d parameters\n", count);
}

static int install_params(int argc, char *argv[])
{
  char buffer[1024];
  char *param_label = NULL;
  char *param_file; 
  FILE *fp;
  int next_arg;

  next_arg = handle_options(argc, argv);
  next_arg++;

  if (argc - next_arg < 1 || argc - next_arg > 2) {
    carmen_warn("\nError: wrong number of parameters.\n");    
    carmen_die("\nUsage: %s install [parameter set] <parameter file>\n\n", 
	       argv[0]);
  }

  if (argc - next_arg == 2)
    param_label = argv[next_arg++];

  param_file = argv[next_arg];

  if (!carmen_file_exists(param_file)) {
    carmen_warn("%s does not exist.\n", param_file);
    carmen_die("\nUsage: %s install [parameter set] <parameter file>\n\n", 
	       argv[0]);
  }
    
  fp = fopen(param_file, "r");
  if (fp == NULL) 
    carmen_die_syserror("Error reading from %s\n", param_file);

  buffer[1023] = 0;
  if (fgets(buffer, 1023, fp) == NULL)
    carmen_die("Unexpected end of file on %s\n", param_file);
  rewind(fp);

  if (strncmp(buffer, CARMEN_LOGFILE_HEADER, 
	      strlen(CARMEN_LOGFILE_HEADER)) == 0) {
    if (param_label)
      carmen_warn("Ignoring paramset argument for reading from logfile.\n");
    publish_params_from_logfile(fp);    
  } else {
    publish_params_from_paramfile(fp, param_label, param_file);
  }

  return 0;
}

static int reread(int argc, char *argv[])
{
  int next_arg;

  next_arg = handle_options(argc, argv);
  next_arg++;

  if (argc - next_arg > 0) {
    carmen_warn("\nError: wrong number of parameters.\n");    
    carmen_die("\nUsage: %s reread\n\n", argv[0]);
  }
    
  carmen_param_send_reread();

  return 0;
}

static void help(int argc, char **argv)
{
  char *action;

  if (argc < 3 || argc > 3) 
    main_usage(argv[0]);

  action = argv[2];
  if (carmen_strcasecmp(action, "set") == 0)
    carmen_die("\nUsage: %s set <parameter name> <parameter value>\n\n", 
	       argv[0]);
  else if (carmen_strcasecmp(action, "get") == 0)
    carmen_die("\nUsage: %s get <module name | parameter name>\n\n"
	       "If a module name is specified, prints out all variables "
	       "of that module.\n\n", argv[0]);
  else if (carmen_strcasecmp(action, "install") == 0)
    carmen_die("\nUsage: %s install [parameter set] <parameter file>\n\n", 
	       argv[0]);
  else if (carmen_strcasecmp(action, "reread") == 0)
    carmen_die("\nUsage: %s reread\n\nCauses the param_daemon to reread "
	       "the ini file, erasing all local changes.\n\n", argv[0]);

  carmen_warn("\nUnrecognized command %s\n", action);
  main_usage(argv[0]);

}

int main(int argc, char **argv)
{
  char *action;

  if (argc < 2) 
    main_usage(argv[0]);
  
  action = argv[1];
  
  if (strcmp(action, "-h") == 0 || carmen_strcasecmp(action, "--help") == 0) 
    main_usage(argv[0]);
  if (carmen_strcasecmp(action, "help") == 0) {
    help(argc, argv);
    return 0;
  }

  carmen_ipc_initialize(argc, argv);

  carmen_param_check_version(argv[0]);

  if (carmen_strcasecmp(action, "set") == 0)
    set(argc, argv);
  else if (carmen_strcasecmp(action, "get") == 0)
    get(argc, argv);
  else if (carmen_strcasecmp(action, "install") == 0)
    install_params(argc, argv);
  else if (carmen_strcasecmp(action, "reread") == 0)
    reread(argc, argv);
  else {
    carmen_warn("\nUnrecognized action %s\n", argv[1]);
    main_usage(argv[0]);
  }

  return 0;
}
