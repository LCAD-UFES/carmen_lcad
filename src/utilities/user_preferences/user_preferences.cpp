#include <carmen/carmen.h>
#include "user_preferences.h"
#include <carmen/param_interface.h>


void
get_user_param(const char *module, user_param_t param, carmen_FILE *user_pref, const char *filename)
{
	static char line[10000];
	static char line_module[1000], line_variable[1000], line_value[8000];
	int pos = 0;

	carmen_fseek(user_pref, 0L, SEEK_SET);

	while (carmen_fgets(line, 9999, user_pref) != NULL)
	{
		if ((line[0] == '#') ||
			(sscanf(line, " %s %s %s ", line_module, line_variable, line_value) != 3) ||
			(strcmp(line_module, module) != 0 || strcmp(line_variable, param.variable) != 0))
			continue;

		switch(param.type)
		{
			case USER_PARAM_TYPE_INT:
				if (sscanf(line_value, "%d%n", (int *) param.value, &pos) != 1 || line_value[pos] != '\0')
					carmen_warn("WARNING: User preference parameter is not of type %s:\nfile: %s\ntext: %s", "INT", filename, line);
				return;

			case USER_PARAM_TYPE_DOUBLE:
				if (sscanf(line_value, "%lf%n", (double *) param.value, &pos) != 1 || line_value[pos] != '\0')
					carmen_warn("WARNING: User preference parameter is not of type %s:\nfile: %s\ntext: %s", "DOUBLE", filename, line);
				return;

			case USER_PARAM_TYPE_ONOFF:
				if (strcmp(line_value, "ON") == 0 || strcmp(line_value, "on") == 0)
					*(int *) param.value = 1;
				else if (strcmp(line_value, "OFF") == 0 || strcmp(line_value, "off") == 0)
					*(int *) param.value = 0;
				else
					carmen_warn("WARNING: User preference parameter is not of type %s:\nfile: %s\ntext: %s", "ONOFF", filename, line);
				return;

			case USER_PARAM_TYPE_STRING:
			case USER_PARAM_TYPE_FILE:
			case USER_PARAM_TYPE_DIR:
			default:
				param.value = malloc(strlen(line_value) + 1);
				strcpy((char *) param.value, line_value);
				return;
		}
	}
}


int
find_variable_in_param_list(const char *variable, user_param_t *param_list, int num_items)
{
	int index;

	for (index = num_items - 1; index >= 0; index--)
		if (strcmp(variable, param_list[index].variable) == 0)
			break;

	return index;
}


int
compare_values(const char *value, void *param_value, char param_type)
{
	int int_value, onoff_value;
	double double_value;
	char *endptr;

	switch(param_type)
	{
		case USER_PARAM_TYPE_INT:
			int_value = strtol(value, &endptr, 0);
			return ((endptr != value) && (*endptr == '\0') && (int_value == *(int *) param_value)) ? 0 : 1;

		case USER_PARAM_TYPE_DOUBLE:
			double_value = strtod(value, &endptr);
			return ((endptr != value) && (*endptr == '\0') && (double_value == *(double *) param_value)) ? 0 : 1;

		case USER_PARAM_TYPE_ONOFF:
			if (strcmp(value, "ON") == 0 || strcmp(value, "on") == 0)
				onoff_value = 1;
			else if (strcmp(value, "OFF") == 0 || strcmp(value, "off") == 0)
				onoff_value = 0;
			else
				return 1;
			return (onoff_value == *(int *) param_value) ? 0 : 1;

		case USER_PARAM_TYPE_STRING:
		case USER_PARAM_TYPE_FILE:
		case USER_PARAM_TYPE_DIR:
		default:
			return strcmp(value, (char *) param_value) == 0 ? 0 : 1;
	}
	return 1;
}


void
param_value_to_str(char *value_str, void *param_value, char param_type)
{
	switch(param_type)
	{
		case USER_PARAM_TYPE_INT:
			sprintf(value_str, "%d", *(int *) param_value);
			break;

		case USER_PARAM_TYPE_DOUBLE:
			sprintf(value_str, "%lf", *(double *) param_value);
			break;

		case USER_PARAM_TYPE_ONOFF:
			sprintf(value_str, "%s", (*(int *) param_value == 0) ? "off" : "on");
			break;

		case USER_PARAM_TYPE_STRING:
		case USER_PARAM_TYPE_FILE:
		case USER_PARAM_TYPE_DIR:
		default:
			sprintf(value_str, "%s", (char *) param_value);
			break;
	}
}


void
user_preferences_read(const char *module, user_param_t *param_list, int num_items, const char *filename = USER_DEFAULT_FILENAME)
{
	if (num_items <= 0)
		return;

	if (filename == NULL)
		return;

	carmen_FILE *user_pref = carmen_fopen(filename, "r");
	if (user_pref == NULL)
		return;

	for (int i = 0; i < num_items; i++)
		get_user_param(module, param_list[i], user_pref, filename);

	carmen_fclose(user_pref);
}


void
user_preferences_read_commandline(int argc, char **argv, user_param_t *param_list, int num_items)
{
	if (num_items <= 0)
		return;

	carmen_param_t *commandline_param_list = (carmen_param_t *) calloc(num_items, sizeof(carmen_param_t));

	for (int i = 0; i < num_items; i++)
		commandline_param_list[i] = (carmen_param_t) {(char *) "commandline", (char *) param_list[i].variable, param_list[i].type,
			param_list[i].value, 0, NULL};

	carmen_param_allow_unfound_variables(1);
	carmen_param_install_params(argc, argv, commandline_param_list, num_items);

	free(commandline_param_list);
}


void
user_preferences_save(const char *module, user_param_t *param_list, int num_items, const char *filename = USER_DEFAULT_FILENAME)
{
	if (num_items <= 0)
		return;

	char new_filename[2000];
	strcpy(new_filename, filename);
	strcat(new_filename, "_new");

	carmen_FILE *user_pref = carmen_fopen(filename, "r");
	carmen_FILE *new_user_pref = carmen_fopen(new_filename, "w");
	if (new_user_pref == NULL)
	{
		carmen_warn("WARNING: Could not open user preferences file for writing: %s\n", filename);
		return;
	}

	char line[10000], line_module[1000], line_variable[1000], line_value[8000], param_value_str[8000];
	int index, pos = 0, saved_count = 0;
	int *saved_list = (int *) calloc(num_items, sizeof(int));

	while (user_pref != NULL && carmen_fgets(line, 9999, user_pref) != NULL)
	{
		if ((line[0] == '#') ||
			(sscanf(line, " %s %s %n%s ", line_module, line_variable, &pos, line_value) != 3) ||
			(strcmp(line_module, module) != 0))
		{
			carmen_fprintf(new_user_pref, "%s", line);
			continue;
		}

		index = find_variable_in_param_list(line_variable, param_list, num_items);
		if (index < 0)
		{
			carmen_fprintf(new_user_pref, "%s", line);
			continue;
		}

		if (compare_values(line_value, param_list[index].value, param_list[index].type) == 0)
			carmen_fprintf(new_user_pref, "%s", line);
		else
		{
			carmen_fwrite(line, pos, 1, new_user_pref);
			param_value_to_str(param_value_str, param_list[index].value, param_list[index].type);
			carmen_fprintf(new_user_pref, "%s   # %s", param_value_str, line + pos);
		}
		saved_list[index] = 1;
		saved_count++;
	}

	if (saved_count < num_items)
		carmen_fprintf(new_user_pref, "\n");

	for (index = 0; index < num_items; index++)
	{
		if (saved_list[index] == 1)
			continue;

		param_value_to_str(param_value_str, param_list[index].value, param_list[index].type);
		carmen_fprintf(new_user_pref, "%s   %s   %s\n", module, param_list[index].variable, param_value_str);
	}

	free(saved_list);
	if (user_pref)
	{
		carmen_fclose(user_pref);
		char bak_filename[2000];
		strcpy(bak_filename, filename);
		strcat(bak_filename, "_bak");
		remove(bak_filename);
		rename(filename, bak_filename);
	}
	carmen_fclose(new_user_pref);
	rename(new_filename, filename);
}
