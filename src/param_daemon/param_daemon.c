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
#ifndef COMPILE_WITHOUT_MAP_SUPPORT
#include <carmen/map_io.h>
#endif

#include <ctype.h>
#include <string.h>
#include <getopt.h>
#include <sys/stat.h>

#define MAX_VARIABLE_LENGTH 2048
#define MAX_NUM_MODULES 128
#define MAX_ROBOTS 100

#define MAX_ROBOT_NAME_LENGTH 50

#define PARAM_LEVEL_BASIC     0
#define PARAM_LEVEL_EXPERT    1
#define PARAM_LEVEL_NOCHANGE  2

typedef struct
{
	char *module_name;
	char *variable_name;
	char *lvalue;
	char *rvalue;
	int expert;
} carmen_ini_param_t, *carmen_ini_param_p;

static int connected = 0;

static char *modules[MAX_NUM_MODULES];
static int num_modules = 0;

static char *default_list[] = { "carmen.ini", "../carmen.ini", "../src/carmen.ini", 0 };

static carmen_ini_param_p param_list = NULL;
static int num_params = 0;
static int param_table_capacity = 0;
#ifndef COMPILE_WITHOUT_MAP_SUPPORT
static char *map_filename = NULL;
#endif
static char *selected_robot = NULL;
static char *param_filename = NULL;

static int alphabetize = 0;
static int generate_from_log = 0;

static void publish_new_param(int index);
static int read_parameters_from_file(void);

void carmen_param_get_param_list(carmen_ini_param_p *list, int *list_size)
{
	if (list == NULL || list_size == NULL)
		return;
	*list = param_list;
	*list_size = num_params;
}

static void shutdown_param_server(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		exit(1);
	}
}

static int lookup_name(char *full_name)
{
	int index;

	for (index = 0; index < num_params; index++)
	{
		if (carmen_strcasecmp(param_list[index].lvalue, full_name) == 0)
			return index;
	}

	return -1;
}

static int lookup_parameter(char *module_name, char *parameter_name)
{
	char buffer[1024];

	if (module_name != NULL)
	{
		sprintf(buffer, "%s_%s", module_name, parameter_name);

		return lookup_name(buffer);
	}
	else
		return lookup_name(parameter_name);
}

static int lookup_module(char *module_name)
{
	int i;

	for (i = 0; i < num_modules; i++)
		if (!strcmp(modules[i], module_name))
			return i;

	return -1;
}

static void add_module(char *module_name)
{
	if (num_modules == MAX_NUM_MODULES)
		carmen_die("You have added %d modules already, and you just tried to \n"
				"add one more. %d is the maximum number of modules carmen\n"
				"currently supports.\n", MAX_NUM_MODULES, MAX_NUM_MODULES);
	modules[num_modules] = (char*) calloc(strlen(module_name) + 1, sizeof(char));
	carmen_test_alloc(modules[num_modules]);
	strcpy(modules[num_modules], module_name);
	num_modules++;
}

static int query_num_params(char *module_name)
{
	int count;
	int index;

	count = 0;
	for (index = 0; index < num_params; index++)
		if (carmen_strcasecmp(param_list[index].module_name, module_name) == 0)
			count++;

	return count;
}

static void check_param_space()
{
	if (param_list == NULL)
	{
		param_table_capacity = 30;
		param_list = (carmen_ini_param_p) calloc(param_table_capacity, sizeof(carmen_ini_param_t));
		carmen_test_alloc(param_list);
	}

	if (num_params == param_table_capacity)
	{
		param_table_capacity *= 2;
		param_list = realloc(param_list, param_table_capacity * sizeof(carmen_ini_param_t));
		carmen_test_alloc(param_list);
	}
}

static void set_param(char *lvalue, char *rvalue, int param_level)
{
	int param_index;
	char module[255], variable[255];
	int num_items;

	param_index = lookup_name(lvalue);
	if (param_index == -1)
	{
		num_items = sscanf(lvalue, "%[^_]_%s", module, variable);
		if (num_items != 2)
		{
			carmen_warn("Ill-formed parameter name %s%s%s. Could not find "
					"module and variable name.\n"
					"Not setting this parameter.\n", carmen_red_code, lvalue, carmen_normal_code);
			return;
		}

		check_param_space();
		param_index = num_params;
		num_params++;
		param_list[param_index].lvalue = (char*) calloc(strlen(lvalue) + 1, sizeof(char));
		carmen_test_alloc(param_list[param_index].lvalue);
		strcpy(param_list[param_index].lvalue, lvalue);

		param_list[param_index].module_name = (char*) calloc(strlen(module) + 1, sizeof(char));
		carmen_test_alloc(param_list[param_index].module_name);
		strcpy(param_list[param_index].module_name, module);

		if (lookup_module(module) == -1)
			add_module(module);

		param_list[param_index].variable_name = (char*) calloc(strlen(variable) + 1, sizeof(char));
		carmen_test_alloc(param_list[param_index].variable_name);
		strcpy(param_list[param_index].variable_name, variable);

		if (param_level == PARAM_LEVEL_NOCHANGE)
			param_level = PARAM_LEVEL_BASIC;
	}
	else
	{
		free(param_list[param_index].rvalue);
	}

	param_list[param_index].rvalue = (char*) calloc(strlen(rvalue) + 1, sizeof(char));
	carmen_test_alloc(param_list[param_index].rvalue);

	strcpy(param_list[param_index].rvalue, rvalue);

	if (param_level != PARAM_LEVEL_NOCHANGE)
		param_list[param_index].expert = param_level;

	carmen_verbose("Added %s %s%s: %s = %s \n", param_list[param_index].module_name, param_list[param_index].variable_name,
			param_list[param_index].expert ? " (expert)" : "", param_list[param_index].lvalue, param_list[param_index].rvalue);

	publish_new_param(param_index);
}

static int find_valid_robots(char **robot_names, int *num_robots, int max_robots)
{
	FILE *fp = NULL;
	char *err, *line, *mark;
	char *left, *right;
	int count = 0;

	fp = fopen(param_filename, "r");
	if (fp == NULL)
		return -1;

	line = (char*) calloc(MAX_VARIABLE_LENGTH, sizeof(char));
	carmen_test_alloc(line);

	do
	{
		err = fgets(line, MAX_VARIABLE_LENGTH - 1, fp);
		line[MAX_VARIABLE_LENGTH - 1] = '\0';
		if (strlen(line) == MAX_VARIABLE_LENGTH - 1)
			carmen_die("Line %d of file %s is too long.\n"
					"Maximum line length is %d. Please correct this line.\n\n"
					"It is also possible that this file has become corrupted.\n"
					"Make sure you have an up-to-date version of carmen, and\n"
					"consult the param_server documentation to make sure the\n"
					"file format is valid.\n", count, param_filename,
			MAX_VARIABLE_LENGTH - 1);
		count++;
		if (err != NULL)
		{
			mark = strchr(line, '#'); /* strip comments and trailing returns */
			if (mark != NULL)
				mark[0] = '\0';
			mark = strchr(line, '\n');
			if (mark != NULL)
				mark[0] = '\0';

			left = strchr(line, '[');
			right = strchr(line, ']');
			if (left != NULL && right != NULL && left < right && left[1] != '*' && carmen_strncasecmp(left + 1, "expert", 6) != 0)
			{
				(*num_robots)++;

				if (*num_robots > max_robots)
					carmen_die("Error: exceeded maximum number of robots in parameter file (%d).\n", max_robots);

				robot_names[*num_robots - 1] = (char*) calloc(right - left, 1);
				carmen_test_alloc(robot_names[*num_robots - 1]);
				strncpy(robot_names[*num_robots - 1], left + 1, right - left - 1);
				robot_names[*num_robots - 1][right - left - 1] = '\0';
			}
		}
	} while (err != NULL);
	free(line);
	return 0;
}


void generate_parameters_file_from_log()
{
	char _param_filename[1048];
	sprintf(_param_filename, "%s_parameters", param_filename);
	printf("writing on [31;1m%s[0m\n", _param_filename);

	FILE *fp_log = fopen(param_filename, "r");
	FILE *fp = fopen(_param_filename, "w");
	if (!fp_log || !fp)
	{
		carmen_die("Could not open %s %s\n", param_filename, _param_filename);
		exit(1);
	}

	fprintf(fp, "[*]\n\n");

	char *pch, *line = NULL, **param_name, **param_value, _param_name[256], _param_value[1024];
	param_name = (char**) malloc(8192*sizeof(char*));
	param_value = (char**) malloc(8192*sizeof(char*));
	int n_parameters = 0;
	size_t len = 0;
	ssize_t read;
	while ((read = getline (&line, &len, fp_log)) != -1) {
		if (!strncmp (line, "PARAM", 5))
		{
			sscanf(line, "PARAM %s %[^\t\n]\n", _param_name, _param_value);
			pch = strrchr (_param_value, ' ');
			if (pch)
			{
				pch[0] = '\0';
				pch = strrchr (_param_value, ' ');
				if (pch)
				{
					pch[0] = '\0';
					pch = strrchr (_param_value, ' ');
					if (pch)
						pch[0] = '\0';
				}
			}
			param_name[n_parameters] = (char*) malloc((strlen(_param_name)+1)*sizeof(char*));
			param_value[n_parameters] = (char*) malloc((strlen(_param_value)+1)*sizeof(char*));
			strcpy(param_name[n_parameters], _param_name);
			strcpy(param_value[n_parameters], _param_value);
			n_parameters++;
		}
	}

	// sort
    for (int i = 0; i < n_parameters - 1; i++)
	{
		for (int j = 0; j < n_parameters - i - 1; j++)
		{
			if (strcmp(param_name[j], param_name[j+1]) > 0)
			{
				pch = param_name[j];
				param_name[j] = param_name[j+1];
				param_name[j+1] = pch;

				pch = param_value[j];
				param_value[j] = param_value[j+1];
				param_value[j+1] = pch;
			}
		}
	}

	pch = param_name[0];
	for (int i = 0; i < n_parameters; i++)
	{
		if (strncmp(pch, param_name[i], 5))
			fprintf(fp, "\n");
		fprintf(fp, "%s %s\n", param_name[i], param_value[i]);
		pch = param_name[i];
	}

	fclose(fp);
	fclose(fp_log);

	strcpy(param_filename, _param_filename);
	// "PARAM %s_%s %s %f %s %f\n", module, variable, value, ipc_time, hostname, timestamp);
}


static int read_parameters_from_file(void)
{
	FILE *fp = NULL;
	char *line;
	char *mark, *token;
	int token_num;
	char lvalue[255], rvalue[MAX_VARIABLE_LENGTH];
	int found_matching_robot = 0;
	int found_desired_robot = 0;
	int expert = 0;
	int line_length;
	int count;

	fp = fopen(param_filename, "r");
	if (fp == NULL)
		return -1;

	line = (char*) calloc(MAX_VARIABLE_LENGTH, sizeof(char));
	carmen_test_alloc(line);

	count = 0;
	while (!feof(fp))
	{
		fgets(line, MAX_VARIABLE_LENGTH - 1, fp);
		line[MAX_VARIABLE_LENGTH - 1] = '\0';
		if (strlen(line) == MAX_VARIABLE_LENGTH - 1)
			carmen_die("Line %d of file %s is too long.\n"
					"Maximum line length is %d. Please correct this line.\n\n"
					"It is also possible that this file has become corrupted.\n"
					"Make sure you have an up-to-date version of carmen, and\n"
					"consult the param_server documentation to make sure the\n"
					"file format is valid.\n", count, param_filename,
			MAX_VARIABLE_LENGTH - 1);
		count++;
		if (feof(fp))
			break;
		mark = strchr(line, '#'); /* strip comments and trailing returns */
		if (mark != NULL)
			mark[0] = '\0';
		mark = strchr(line, '\n');
		if (mark != NULL)
			mark[0] = '\0';

		// Trim off trailing white space

		line_length = strlen(line) - 1;
		while (line_length >= 0 && (line[line_length] == ' ' || line[line_length] == '\t'))
		{
			line[line_length--] = '\0';
		}
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

		if (strlen(token) > 254)
		{
			carmen_warn("Bad file format of %s on line %d.\n"
					"The parameter name %s is too long (%d characters).\n"
					"A parameter name can be no longer than 254 "
					"characters.\nSkipping this line.\n", param_filename, count, token, (int) strlen(token));
			continue;
		}

		strcpy(lvalue, token);
		token_num++;

		// If mark points to a non-whitespace character, then we have a
		// two-token line
		if (mark)
		{
			if (strlen(mark) > MAX_VARIABLE_LENGTH - 1)
			{
				carmen_warn("Bad file format of %s on line %d.\n"
						"The parameter value %s is too long (%d "
						"characters).\nA parameter value can be no longer "
						"than %u characters.\nSkipping this line.\n", param_filename, count, mark, (int) strlen(mark),
				MAX_VARIABLE_LENGTH - 1);
				continue;
			}
			strcpy(rvalue, mark);
			token_num++;
		}

		if (lvalue[0] == '[')
		{
			if (strcspn(lvalue + 1, "]") == 6 && !carmen_strncasecmp(lvalue + 1, "expert", 6))
			{
				found_matching_robot = 1;
				expert = 1;
			}
			else
			{
				expert = 0;
				if (lvalue[1] == '*')
					found_matching_robot = 1;
				else if (selected_robot)
				{
					if (strlen(lvalue) < strlen(selected_robot) + 2)
						found_matching_robot = 0;
					else if (lvalue[strlen(selected_robot) + 1] != ']')
						found_matching_robot = 0;
					else if (carmen_strncasecmp(lvalue + 1, selected_robot, strlen(selected_robot)) == 0)
					{
						found_matching_robot = 1;
						found_desired_robot = 1;
					}
					else
						found_matching_robot = 0;
				}
			}
		}
		else if (token_num == 2 && found_matching_robot == 1)
			set_param(lvalue, rvalue, expert);
	} /* End of while (!feof(fp)) */

	fclose(fp);

	if (selected_robot && !found_desired_robot)
	{
		carmen_die("Did not find a match for robot %s. Do you have the right\n"
				"init file? (Reading parameters from %s)\n", selected_robot, param_filename);
	}

	return 0;
}

#define MAX_LINE_LENGTH 4096

static int contains_binary_chars(char *filename)
{
	FILE *fp;
	int c;

	fp = fopen(filename, "r");
	if (fp == NULL)
		return -1;
	while ((c = fgetc(fp)) != EOF)
	{
		if (!isascii(c))
		{
			fclose(fp);
			return 1;
		}
	}
	fclose(fp);
	return 0;
}

static void usage(char *progname, char *fmt, ...)
{
	va_list args;

	if (fmt != NULL)
	{
		fprintf(stderr, "\n[31;1m");
		va_start(args, fmt);
		vfprintf(stderr, fmt, args);
		va_end(args);
		fprintf(stderr, "[0m\n\n");
	}
	else
	{
		fprintf(stderr, "\n");
	}

	if (strrchr(progname, '/') != NULL)
	{
		progname = strrchr(progname, '/');
		progname++;
	}

#ifndef COMPILE_WITHOUT_MAP_SUPPORT
	fprintf(stderr, "usage: %s [-ahr] [map_filename] [ini filename] \n\n", progname);
#else
  fprintf(stderr, "usage: %s [-ahr] [ini filename] \n\n", 
	  progname);
#endif

	exit(-1);
}

static void help(char *progname)
{
	if (strrchr(progname, '/') != NULL)
	{
		progname = strrchr(progname, '/');
		progname++;
	}

	fprintf(stderr, "usage: %s "
	//"[-ahr] "
#ifndef COMPILE_WITHOUT_MAP_SUPPORT
					"[map_filename] [ini filename] \n\n",
#else
	  "[ini filename] \n\n", 
#endif
			progname);
	fprintf(stderr, " --alphabetize\talphabetize parameters when listing\n"
			" --robot=ROBOT\tuse parameters for ROBOT\n"
			"\n\n"
#ifndef COMPILE_WITHOUT_MAP_SUPPORT
			"[map_filename] and [ini filename] are both optional arguments. \n"
			"If you do not provide [map_filename], then no map will be served by \n"
			"the %s. If you do not provide an [ini filename], then \n"
#else
     "If you do not provide an [ini filename], then \n"
#endif
			"%s will look for carmen.ini in the current directory, \n"
			"and then look for ../carmen.ini, and then ../src/carmen.ini. \n"
			"If you provide no ini filename, and %s cannot find\n"
			"the carmen.ini, then you will get this error message. \n"
			"\n"
#ifndef COMPILE_WITHOUT_MAP_SUPPORT
			"If you only provide one file name, then %s will try to infer\n"
			"whether it is a map or not and process it accordingly. \n"
#endif
			"\n"
			"The ini file must provide a section of parameters that matches the \n"
			"robot name.  \n\n",
#ifndef COMPILE_WITHOUT_MAP_SUPPORT
			progname, progname, progname, progname);
#else
     progname, progname);
#endif

	exit(-1);
}

static int read_commandline(int argc, char **argv)
{
	int index, i;
	int cur_arg;
	char **robot_names = NULL;
	int num_robots = 0;
	int binary;

	//  extern char *optarg;
	//  extern int optind;
	//  extern int optopt;

	static struct option long_options[] = { { "help", 0, 0, 0 }, { "robot", 1, NULL, 0 }, { "alphabetize", 0, &alphabetize, 1 }, { 0, 0, 0, 0 } };

	int option_index = 0;
	int c;
	char arg_buffer[255];

	opterr = 0;
	while (1)
	{
		c = getopt_long(argc, argv, "ahrl:", long_options, &option_index);
		if (c == -1)
			break;

		if (c == 0)
		{
			sprintf(arg_buffer, "--%s", long_options[option_index].name);
			if (carmen_strcasecmp(long_options[option_index].name, "help") == 0)
				c = 'h';
			else if (carmen_strcasecmp(long_options[option_index].name, "robot") == 0)
				c = 'r';
		}
		else
		{
			sprintf(arg_buffer, "-%c", c);
		}

		switch (c)
		{
		case 'r':
			if (optarg == NULL)
				usage(argv[0], "%s requires an argument", arg_buffer);
			if (strlen(optarg) > MAX_ROBOT_NAME_LENGTH)
				usage(argv[0], "argument to %s: %s is invalid (too long).", arg_buffer, optarg);
			selected_robot = (char*) calloc(strlen(optarg) + 1, sizeof(char));
			carmen_test_alloc(selected_robot);
			strcpy(selected_robot, optarg);
			break;
		case 'l':
			generate_from_log = 1;
			break;
		case 'a':
			alphabetize = 1;
			break;
		case 'h':
			help(argv[0]);
			break;
		case ':':
			usage(argv[0], "-%c requires an argument", optopt);
			break;
		case '?':
			//      usage(argv[0], "unknown option character %c", optopt);
			break;
		}
	}

#ifndef COMPILE_WITHOUT_MAP_SUPPORT
	/* look for a map file */
	map_filename = NULL;
	for (cur_arg = optind; cur_arg < argc; cur_arg++)
	{
		if (carmen_map_file(argv[cur_arg]) && map_filename)
			usage(argv[0], "too many map files given: %s and %s", map_filename, argv[cur_arg]);
		if (carmen_map_file(argv[cur_arg]))
		{
			map_filename = (char*) calloc(strlen(argv[cur_arg]) + 1, sizeof(char));
			carmen_test_alloc(map_filename);
			strcpy(map_filename, argv[cur_arg]);
		}
	}
#endif

	param_filename = NULL;
	for (cur_arg = optind; cur_arg < argc; cur_arg++)
	{
#ifndef COMPILE_WITHOUT_MAP_SUPPORT
		if (map_filename && strcmp(map_filename, argv[cur_arg]) == 0)
			continue;
#endif
		if (param_filename && carmen_file_exists(argv[cur_arg]))
		{
			usage(argv[0], "Too many ini files given: %s and %s", param_filename, argv[cur_arg]);
		}
		if (!carmen_file_exists(argv[cur_arg]))
			usage(argv[0], "No such file: %s", argv[cur_arg]);

		binary = contains_binary_chars(argv[cur_arg]);
		if (binary < 0)
			usage(argv[0], "Couldn't read from %s: %s", argv[cur_arg], strerror(errno));
		if (binary > 0)
			usage(argv[0], "Invalid ini file %s: (not a valid map file either)", argv[cur_arg]);
		param_filename = (char*) calloc(strlen(argv[cur_arg]) + 1, sizeof(char));
		carmen_test_alloc(param_filename);
		strcpy(param_filename, argv[cur_arg]);
	}

	if (generate_from_log)
	{
		if (argc >= 2 && carmen_file_exists(argv[2]))
		{
			param_filename = (char*) calloc(strlen(argv[2]) + 1, sizeof(char));
			carmen_test_alloc(param_filename);
			strcpy(param_filename, argv[2]);
		}
	}

	if (!param_filename)
	{
		for (index = 0; default_list[index]; index++)
		{
			if (carmen_file_exists(default_list[index]))
			{
				param_filename = default_list[index];
				break;
			}
		}
	}

	robot_names = (char**) calloc(MAX_ROBOTS, sizeof(char*));
	carmen_test_alloc(robot_names);
	find_valid_robots(robot_names, &num_robots, MAX_ROBOTS);

	if (generate_from_log)
	{
		carmen_warn("Loading parameters for default robot using log file "
				"[31;1m%s[0m\n", param_filename);
		return 0;
	}
	if (num_robots == 0)
	{
		carmen_warn("Loading parameters for default robot using param file "
				"[31;1m%s[0m\n", param_filename);
		return 0;
	}

	robot_names = (char**) realloc(robot_names, num_robots * sizeof(char*));
	carmen_test_alloc(robot_names);

	if (selected_robot == NULL && num_robots > 1)
	{
		for (cur_arg = 1; cur_arg < optind; cur_arg++)
		{
			for (i = 0; i < num_robots; i++)
				if (strcmp(robot_names[i], argv[cur_arg] + 1) == 0)
				{
					selected_robot = argv[cur_arg] + 1;
					break;
				}
		}
	}

	if (selected_robot == NULL && num_robots > 1)
		usage(argv[0], "The ini_file %s contains %d robot definitions.\n"
				"You must specify a robot name on the command line using --robot.", param_filename, num_robots);
	if (selected_robot == NULL)
		selected_robot = carmen_new_string("%s", robot_names[0]);

	for (i = 0; i < num_robots; i++)
		free(robot_names[i]);
	free(robot_names);

	carmen_warn("Loading parameters for robot [31;1m%s[0m using param file "
			"[31;1m%s[0m\n", selected_robot, param_filename);

	return 0;
}

static
int lookup_ipc_query(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData __attribute__ ((unused)), carmen_param_query_message *query)
{
	FORMATTER_PTR formatter;
	IPC_RETURN_TYPE err = IPC_OK;

	formatter = IPC_msgInstanceFormatter(msgRef);
	err = IPC_unmarshallData(formatter, callData, query, sizeof(carmen_param_query_message));
	IPC_freeByteArray(callData);

	carmen_test_ipc_return_int(err, "Could not unmarshall", IPC_msgInstanceName(msgRef));

	return lookup_parameter(query->module_name, query->variable_name);
}

static void get_robot(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData __attribute__ ((unused)))
{
	FORMATTER_PTR formatter;
	IPC_RETURN_TYPE err = IPC_OK;
	carmen_param_query_message query;
	carmen_param_response_robot_message response;

	formatter = IPC_msgInstanceFormatter(msgRef);
	err = IPC_unmarshallData(formatter, callData, &query, sizeof(carmen_param_query_message));
	IPC_freeByteArray(callData);

	carmen_test_ipc_return(err, "Could not unmarshall", IPC_msgInstanceName(msgRef));

	response.timestamp = carmen_get_time();
	response.host = carmen_get_host();

	if (selected_robot)
	{
		response.robot = (char*) calloc(strlen(selected_robot) + 1, sizeof(char));
		carmen_test_alloc(response.robot);
		strcpy(response.robot, selected_robot);
	}
	else
	{
		response.robot = (char*) calloc(8, sizeof(char));
		carmen_test_alloc(response.robot);
		strcpy(response.robot, "default");
	}
	response.status = CARMEN_PARAM_OK;

	err = IPC_respondData(msgRef, CARMEN_PARAM_RESPONSE_ROBOT_NAME, &response);
	carmen_test_ipc(err, "Could not respond", CARMEN_PARAM_RESPONSE_ROBOT_NAME);

	IPC_freeDataElements(formatter, &query);
	free(response.robot);
}

static void get_modules(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData __attribute__ ((unused)))
{
	FORMATTER_PTR formatter;
	IPC_RETURN_TYPE err = IPC_OK;
	carmen_param_query_message query;
	carmen_param_response_modules_message response;
	int m;

	formatter = IPC_msgInstanceFormatter(msgRef);
	err = IPC_unmarshallData(formatter, callData, &query, sizeof(carmen_param_query_message));
	IPC_freeByteArray(callData);

	carmen_test_ipc_return(err, "Could not unmarshall", IPC_msgInstanceName(msgRef));

	response.timestamp = carmen_get_time();
	response.host = carmen_get_host();

	response.modules = (char**) calloc(num_modules, sizeof(char*));
	carmen_test_alloc(response.modules);
	response.num_modules = num_modules;
	for (m = 0; m < num_modules; m++)
	{
		response.modules[m] = (char*) calloc(strlen(modules[m]) + 1, sizeof(char));
		carmen_test_alloc(response.modules[m]);
		strcpy(response.modules[m], modules[m]);
	}

	response.status = CARMEN_PARAM_OK;

	err = IPC_respondData(msgRef, CARMEN_PARAM_RESPONSE_MODULES_NAME, &response);
	carmen_test_ipc(err, "Could not respond", CARMEN_PARAM_RESPONSE_MODULES_NAME);

	for (m = 0; m < num_modules; m++)
		free(response.modules[m]);
	free(response.modules);
	IPC_freeDataElements(formatter, &query);
}

static int strqcmp(const void *A, const void *B)
{
	return strcmp(*((char**) A), *((char**) B));
}

static void get_param_all(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData __attribute__ ((unused)))
{
	FORMATTER_PTR formatter;
	IPC_RETURN_TYPE err = IPC_OK;
	carmen_param_query_message query;
	carmen_param_response_all_message response;
	int param_index;
	int num_variables;
	int variable_count;
	size_t length;

	formatter = IPC_msgInstanceFormatter(msgRef);
	err = IPC_unmarshallData(formatter, callData, &query, sizeof(carmen_param_query_message));
	IPC_freeByteArray(callData);

	carmen_test_ipc_return(err, "Could not unmarshall", IPC_msgInstanceName(msgRef));

	response.timestamp = carmen_get_time();
	response.host = carmen_get_host();

	response.module_name = query.module_name;
	response.status = CARMEN_PARAM_OK;

	num_variables = query_num_params(query.module_name);
	response.list_length = num_variables;
	variable_count = 0;
	if (num_variables > 0)
	{
		response.variables = calloc(num_variables, sizeof(char*));
		carmen_test_alloc(response.variables);
		response.values = (char**) calloc(num_variables, sizeof(char*));
		carmen_test_alloc(response.values);
		response.expert = (int*) calloc(num_variables, sizeof(int));
		carmen_test_alloc(response.expert);
		for (param_index = 0; param_index < num_params; param_index++)
		{
			if (carmen_strcasecmp(param_list[param_index].module_name, query.module_name) == 0)
			{
				length = strlen(param_list[param_index].variable_name) + 1;
				response.variables[variable_count] = (char*) calloc(length, sizeof(char));
				carmen_test_alloc(response.variables[variable_count]);
				strcpy(response.variables[variable_count], param_list[param_index].variable_name);
				variable_count++;
				if (variable_count == num_variables)
					break;
			} /* End of if (carmen_strcasecmp
			 (param_list[param_index].module_name...) */
		} /* End of for (param_index = 0; param_index < num_params ...) */

		if (alphabetize)
			qsort(response.variables, num_variables, sizeof(char*), strqcmp);
		for (variable_count = 0; variable_count < num_variables; variable_count++)
		{
			param_index = lookup_parameter(query.module_name, response.variables[variable_count]);
			length = strlen(param_list[param_index].rvalue) + 1;
			response.values[variable_count] = (char*) calloc(length, sizeof(char));
			carmen_test_alloc(response.values[variable_count]);
			strcpy(response.values[variable_count], param_list[param_index].rvalue);
			response.expert[variable_count] = param_list[param_index].expert;
		}
	} /* End of if (num_variables < 0) */

	err = IPC_respondData(msgRef, CARMEN_PARAM_RESPONSE_ALL_NAME, &response);
	carmen_test_ipc(err, "Could not respond", CARMEN_PARAM_RESPONSE_ALL_NAME);

	// clean up
	for (variable_count = 0; variable_count < num_variables; variable_count++)
	{
		free(response.variables[variable_count]);
		free(response.values[variable_count]);
	}
	free(response.variables);
	free(response.values);
	free(response.expert);
	IPC_freeDataElements(formatter, &query);
}

static void get_param_int(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	IPC_RETURN_TYPE err;

	carmen_param_query_message query;
	carmen_param_response_int_message response;

	int param_index;
	char *endptr;

	param_index = lookup_ipc_query(msgRef, callData, clientData, &query);

	response.timestamp = carmen_get_time();
	response.host = carmen_get_host();

	response.module_name = query.module_name;
	response.variable_name = query.variable_name;
	response.status = CARMEN_PARAM_OK;

	if (param_index < 0)
		response.status = CARMEN_PARAM_NOT_FOUND;
	else
	{
		response.value = strtol(param_list[param_index].rvalue, &endptr, 0);
		if (endptr == param_list[param_index].rvalue)
			response.status = CARMEN_PARAM_NOT_INT;
		response.expert = param_list[param_index].expert;
	}

	err = IPC_respondData(msgRef, CARMEN_PARAM_RESPONSE_INT_NAME, &response);
	carmen_test_ipc(err, "Could not respond", CARMEN_PARAM_RESPONSE_INT_NAME);

	free(query.host);
	free(query.module_name);
	free(query.variable_name);
}

static void get_param_double(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	IPC_RETURN_TYPE err;

	carmen_param_query_message query;
	carmen_param_response_double_message response;

	int param_index;
	char *endptr;

	param_index = lookup_ipc_query(msgRef, callData, clientData, &query);

	response.timestamp = carmen_get_time();
	response.host = carmen_get_host();

	response.module_name = query.module_name;
	response.variable_name = query.variable_name;
	response.status = CARMEN_PARAM_OK;

	if (param_index < 0)
		response.status = CARMEN_PARAM_NOT_FOUND;
	else
	{
		response.value = (double) strtod(param_list[param_index].rvalue, &endptr);
		if (endptr == param_list[param_index].rvalue)
			response.status = CARMEN_PARAM_NOT_DOUBLE;
		response.expert = param_list[param_index].expert;
	}

	err = IPC_respondData(msgRef, CARMEN_PARAM_RESPONSE_DOUBLE_NAME, &response);
	carmen_test_ipc(err, "Could not respond", CARMEN_PARAM_RESPONSE_DOUBLE_NAME);

	free(query.host);
	free(query.module_name);
	free(query.variable_name);
}

static void get_param_onoff(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	IPC_RETURN_TYPE err;
	char buffer[255];

	carmen_param_query_message query;
	carmen_param_response_onoff_message response;

	int param_index;

	param_index = lookup_ipc_query(msgRef, callData, clientData, &query);

	response.timestamp = carmen_get_time();
	response.host = carmen_get_host();

	response.module_name = query.module_name;
	response.variable_name = query.variable_name;
	response.status = CARMEN_PARAM_OK;

	if (param_index < 0)
		response.status = CARMEN_PARAM_NOT_FOUND;
	else
	{
		if (strlen(param_list[param_index].rvalue) > 254)
			response.status = CARMEN_PARAM_NOT_ONOFF;
		else
		{
			strcpy(buffer, param_list[param_index].rvalue);
			if (carmen_strncasecmp(buffer, "ON", 2) == 0)
				response.value = 1;
			else if (carmen_strncasecmp(buffer, "OFF", 3) == 0)
				response.value = 0;
			else
				response.status = CARMEN_PARAM_NOT_ONOFF;
			response.expert = param_list[param_index].expert;
		}
	}

	err = IPC_respondData(msgRef, CARMEN_PARAM_RESPONSE_ONOFF_NAME, &response);
	carmen_test_ipc(err, "Could not respond", CARMEN_PARAM_RESPONSE_ONOFF_NAME);

	free(query.host);
	free(query.module_name);
	free(query.variable_name);
}

static void get_param_string(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	IPC_RETURN_TYPE err;

	carmen_param_query_message query;
	carmen_param_response_string_message response;

	int param_index;

	param_index = lookup_ipc_query(msgRef, callData, clientData, &query);

	response.timestamp = carmen_get_time();
	response.host = carmen_get_host();

	response.module_name = query.module_name;
	response.variable_name = query.variable_name;
	response.status = CARMEN_PARAM_OK;

	if (param_index < 0)
	{
		response.status = CARMEN_PARAM_NOT_FOUND;
		response.value = NULL;
	}
	else
	{
		response.value = param_list[param_index].rvalue;
		response.expert = param_list[param_index].expert;
	}

	err = IPC_respondData(msgRef, CARMEN_PARAM_RESPONSE_STRING_NAME, &response);
	carmen_test_ipc(err, "Could not respond", CARMEN_PARAM_RESPONSE_STRING_NAME);

	free(query.host);
	free(query.module_name);
	free(query.variable_name);
}

static void set_param_ipc(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData __attribute__ ((unused)))
{
	FORMATTER_PTR formatter;
	IPC_RETURN_TYPE err = IPC_OK;
	carmen_param_set_message query;
	char buffer[1024];
	carmen_param_response_string_message response;
	int param_index = -1;

	formatter = IPC_msgInstanceFormatter(msgRef);
	err = IPC_unmarshallData(formatter, callData, &query, sizeof(carmen_param_set_message));
	IPC_freeByteArray(callData);

	carmen_test_ipc_return(err, "Could not unmarshall", IPC_msgInstanceName(msgRef));

	if (query.module_name != NULL && query.module_name[0] != '\0')
	{
		sprintf(buffer, "%s_%s", query.module_name, query.variable_name);
		if (query.value != NULL && query.value[0] != '\0')
			set_param(buffer, query.value, PARAM_LEVEL_NOCHANGE);
		param_index = lookup_name(buffer);
	}
	else
	{
		set_param(query.variable_name, query.value, PARAM_LEVEL_NOCHANGE);
		param_index = lookup_name(query.variable_name);
	}
	/* Respond with the new value */

	response.timestamp = carmen_get_time();
	response.host = carmen_get_host();

	response.module_name = query.module_name;
	response.variable_name = query.variable_name;
	response.status = CARMEN_PARAM_OK;

	if (param_index < 0)
	{
		response.status = CARMEN_PARAM_NOT_FOUND;
		carmen_die("Major error: inside set_param_ipc, tried to recover value "
				"of parameter\nthat was just set, and failed.\n");
	}
	else
		response.value = param_list[param_index].rvalue;

	err = IPC_respondData(msgRef, CARMEN_PARAM_RESPONSE_STRING_NAME, &response);
	carmen_test_ipc(err, "Could not respond", CARMEN_PARAM_RESPONSE_STRING_NAME);

	IPC_freeDataElements(formatter, &query);
}

static void publish_new_param(int index)
{
	IPC_RETURN_TYPE err;

	carmen_param_response_string_message response;

	if (!connected)
		return;

	if (index < 0)
		return;

	response.timestamp = carmen_get_time();
	response.host = carmen_get_host();

	response.module_name = param_list[index].module_name;
	response.variable_name = param_list[index].variable_name;
	response.value = param_list[index].rvalue;
	response.status = CARMEN_PARAM_OK;

	err = IPC_publishData(CARMEN_PARAM_VARIABLE_CHANGE_NAME, &response);
	carmen_test_ipc(err, "Could not respond", CARMEN_PARAM_VARIABLE_CHANGE_NAME);
}

static void get_version(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData __attribute__ ((unused)))
{
	IPC_RETURN_TYPE err;

	IPC_freeByteArray(callData);
	carmen_param_version_message response;

	response.major = CARMEN_MAJOR_VERSION;
	response.minor = CARMEN_MINOR_VERSION;
	response.revision = CARMEN_REVISION;

	response.timestamp = carmen_get_time();
	response.host = carmen_get_host();

	err = IPC_respondData(msgRef, CARMEN_PARAM_VERSION_NAME, &response);
	carmen_test_ipc(err, "Could not respond", CARMEN_PARAM_VERSION_NAME);
}

static void publish_started_message()
{
	IPC_RETURN_TYPE err;

	carmen_param_started_message msg;
	msg.timestamp = carmen_get_time();
	msg.host = carmen_get_host();
	err = IPC_publishData(CARMEN_PARAM_STARTED_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_PARAM_STARTED_NAME);
}

static void reread_command(MSG_INSTANCE msgRef __attribute__ ((unused)), BYTE_ARRAY callData, void *clientData __attribute__ ((unused)))
{
	IPC_freeByteArray(callData);
	read_parameters_from_file();
}

static int initialize_param_ipc(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_PARAM_QUERY_ROBOT_NAME, IPC_VARIABLE_LENGTH,
	CARMEN_PARAM_QUERY_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_PARAM_QUERY_ROBOT_NAME);
	err = IPC_defineMsg(CARMEN_PARAM_RESPONSE_ROBOT_NAME, IPC_VARIABLE_LENGTH,
	CARMEN_PARAM_RESPONSE_ROBOT_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_PARAM_RESPONSE_ROBOT_NAME);

	err = IPC_defineMsg(CARMEN_PARAM_QUERY_MODULES_NAME, IPC_VARIABLE_LENGTH,
	CARMEN_PARAM_QUERY_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_PARAM_QUERY_MODULES_NAME);
	err = IPC_defineMsg(CARMEN_PARAM_RESPONSE_MODULES_NAME, IPC_VARIABLE_LENGTH,
	CARMEN_PARAM_RESPONSE_MODULES_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_PARAM_RESPONSE_MODULES_NAME);

	err = IPC_defineMsg(CARMEN_PARAM_QUERY_ALL_NAME, IPC_VARIABLE_LENGTH,
	CARMEN_PARAM_QUERY_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_PARAM_QUERY_ALL_NAME);

	err = IPC_defineMsg(CARMEN_PARAM_RESPONSE_ALL_NAME, IPC_VARIABLE_LENGTH,
	CARMEN_PARAM_RESPONSE_ALL_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_PARAM_RESPONSE_ALL_NAME);

	err = IPC_defineMsg(CARMEN_PARAM_QUERY_INT_NAME, IPC_VARIABLE_LENGTH,
	CARMEN_PARAM_QUERY_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_PARAM_QUERY_INT_NAME);

	err = IPC_defineMsg(CARMEN_PARAM_RESPONSE_INT_NAME, IPC_VARIABLE_LENGTH,
	CARMEN_PARAM_RESPONSE_INT_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_PARAM_RESPONSE_INT_NAME);

	err = IPC_defineMsg(CARMEN_PARAM_QUERY_DOUBLE_NAME, IPC_VARIABLE_LENGTH,
	CARMEN_PARAM_QUERY_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_PARAM_QUERY_DOUBLE_NAME);

	err = IPC_defineMsg(CARMEN_PARAM_RESPONSE_DOUBLE_NAME, IPC_VARIABLE_LENGTH,
	CARMEN_PARAM_RESPONSE_DOUBLE_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_PARAM_RESPONSE_DOUBLE_NAME);

	err = IPC_defineMsg(CARMEN_PARAM_QUERY_ONOFF_NAME, IPC_VARIABLE_LENGTH,
	CARMEN_PARAM_QUERY_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_PARAM_QUERY_ONOFF_NAME);

	err = IPC_defineMsg(CARMEN_PARAM_RESPONSE_ONOFF_NAME, IPC_VARIABLE_LENGTH,
	CARMEN_PARAM_RESPONSE_ONOFF_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_PARAM_RESPONSE_ONOFF_NAME);

	err = IPC_defineMsg(CARMEN_PARAM_QUERY_STRING_NAME, IPC_VARIABLE_LENGTH,
	CARMEN_PARAM_QUERY_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_PARAM_QUERY_STRING_NAME);

	err = IPC_defineMsg(CARMEN_PARAM_RESPONSE_STRING_NAME, IPC_VARIABLE_LENGTH,
	CARMEN_PARAM_RESPONSE_STRING_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_PARAM_RESPONSE_STRING_NAME);

	err = IPC_defineMsg(CARMEN_PARAM_SET_NAME, IPC_VARIABLE_LENGTH,
	CARMEN_PARAM_SET_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_PARAM_SET_NAME);

	err = IPC_defineMsg(CARMEN_PARAM_VARIABLE_CHANGE_NAME, IPC_VARIABLE_LENGTH,
	CARMEN_PARAM_VARIABLE_CHANGE_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_PARAM_VARIABLE_CHANGE_NAME);

	err = IPC_defineMsg(CARMEN_PARAM_VERSION_QUERY_NAME, IPC_VARIABLE_LENGTH,
	CARMEN_DEFAULT_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_PARAM_VERSION_QUERY_NAME);

	err = IPC_defineMsg(CARMEN_PARAM_VERSION_NAME, IPC_VARIABLE_LENGTH,
	CARMEN_PARAM_VERSION_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_PARAM_VERSION_NAME);

	err = IPC_defineMsg(CARMEN_PARAM_REREAD_COMMAND_NAME, IPC_VARIABLE_LENGTH,
	CARMEN_DEFAULT_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_PARAM_REREAD_COMMAND_NAME);

	err = IPC_subscribe(CARMEN_PARAM_QUERY_ROBOT_NAME, get_robot, NULL);
	carmen_test_ipc_exit(err, "Could not subscribe to", CARMEN_PARAM_QUERY_ROBOT_NAME);
	IPC_setMsgQueueLength(CARMEN_PARAM_QUERY_ROBOT_NAME, 100);

	err = IPC_subscribe(CARMEN_PARAM_QUERY_MODULES_NAME, get_modules, NULL);
	carmen_test_ipc_exit(err, "Could not subscribe to", CARMEN_PARAM_QUERY_MODULES_NAME);
	IPC_setMsgQueueLength(CARMEN_PARAM_QUERY_MODULES_NAME, 100);

	err = IPC_subscribe(CARMEN_PARAM_QUERY_ALL_NAME, get_param_all, NULL);
	carmen_test_ipc_exit(err, "Could not subscribe to", CARMEN_PARAM_QUERY_ALL_NAME);
	IPC_setMsgQueueLength(CARMEN_PARAM_QUERY_ALL_NAME, 100);

	err = IPC_subscribe(CARMEN_PARAM_QUERY_INT_NAME, get_param_int, NULL);
	carmen_test_ipc_exit(err, "Could not subscribe to", CARMEN_PARAM_QUERY_INT_NAME);
	IPC_setMsgQueueLength(CARMEN_PARAM_QUERY_INT_NAME, 100);

	err = IPC_subscribe(CARMEN_PARAM_QUERY_DOUBLE_NAME, get_param_double, NULL);
	carmen_test_ipc_exit(err, "Could not subscribe to", CARMEN_PARAM_QUERY_DOUBLE_NAME);

	err = IPC_subscribe(CARMEN_PARAM_QUERY_ONOFF_NAME, get_param_onoff, NULL);
	carmen_test_ipc_exit(err, "Could not subscribe to", CARMEN_PARAM_QUERY_ONOFF_NAME);
	IPC_setMsgQueueLength(CARMEN_PARAM_QUERY_ONOFF_NAME, 100);

	err = IPC_subscribe(CARMEN_PARAM_QUERY_STRING_NAME, get_param_string, NULL);
	carmen_test_ipc_exit(err, "Could not subscribe to", CARMEN_PARAM_QUERY_STRING_NAME);
	IPC_setMsgQueueLength(CARMEN_PARAM_QUERY_STRING_NAME, 100);

	err = IPC_subscribe(CARMEN_PARAM_VERSION_QUERY_NAME, get_version, NULL);
	carmen_test_ipc_exit(err, "Could not subscribe to", CARMEN_PARAM_VERSION_QUERY_NAME);
	IPC_setMsgQueueLength(CARMEN_PARAM_VERSION_QUERY_NAME, 100);

	err = IPC_subscribe(CARMEN_PARAM_REREAD_COMMAND_NAME, reread_command, NULL);
	carmen_test_ipc_exit(err, "Could not subscribe to", CARMEN_PARAM_REREAD_COMMAND_NAME);
	IPC_setMsgQueueLength(CARMEN_PARAM_REREAD_COMMAND_NAME, 1);

	err = IPC_subscribe(CARMEN_PARAM_SET_NAME, set_param_ipc, NULL);
	carmen_test_ipc_exit(err, "Could not subscribe to", CARMEN_PARAM_SET_NAME);
	IPC_setMsgQueueLength(CARMEN_PARAM_SET_NAME, 100);

	err = IPC_defineMsg(CARMEN_PARAM_STARTED_NAME, IPC_VARIABLE_LENGTH,
	CARMEN_PARAM_STARTED_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_PARAM_STARTED_NAME);

	return 0;
}

int main(int argc, char **argv)
{
	signal(SIGINT, shutdown_param_server);

	if (read_commandline(argc, argv) < 0)
		carmen_die_syserror("Could not read from ini file");

	carmen_verbose("Read %d parameters\n", num_params);

	if (generate_from_log)
		generate_parameters_file_from_log();
	read_parameters_from_file();

	carmen_ipc_initialize(argc, argv);

	connected = 1;

	if (initialize_param_ipc() < 0)
		carmen_die("Error: could not connect to IPC Server\n");

#ifndef COMPILE_WITHOUT_MAP_SUPPORT
	if (map_filename)
	{
		if (carmen_map_initialize_ipc() < 0)
			carmen_die("Error: could initialize map IPC calls.\n");
		carmen_map_set_filename(map_filename);
	}
#endif
	publish_started_message();
	IPC_dispatch();

	return 0;
}

