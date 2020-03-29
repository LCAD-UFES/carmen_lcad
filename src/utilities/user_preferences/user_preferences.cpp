#include <carmen/carmen.h>
#include <carmen/user_preferences.h>


void
get_param_value(void *value, char type, const char *line, const char *filename = "commandline")
{
	int len = 0;

	switch(type)
	{
		case USER_PARAM_TYPE_INT:
			if (sscanf(line, "%d%n", (int *) value, &len) != 1 || line[len] != '\0')
				carmen_warn("WARNING: User preference parameter is not of type %s:\nfile: %s\ntext: %s", "INT", filename, line);
			break;

		case USER_PARAM_TYPE_DOUBLE:
			if (sscanf(line, "%lf%n", (double *) value, &len) != 1 || line[len] != '\0')
				carmen_warn("WARNING: User preference parameter is not of type %s:\nfile: %s\ntext: %s", "DOUBLE", filename, line);
			break;

		case USER_PARAM_TYPE_ONOFF:
			if (strcmp(line, "ON") == 0 || strcmp(line, "on") == 0)
				*(int *) value = 1;
			else if (strcmp(line, "OFF") == 0 || strcmp(line, "off") == 0)
				*(int *) value = 0;
			else
				carmen_warn("WARNING: User preference parameter is not of type %s:\nfile: %s\ntext: %s", "ONOFF", filename, line);
			break;

		case USER_PARAM_TYPE_STRING:
		case USER_PARAM_TYPE_FILE:
		case USER_PARAM_TYPE_DIR:
		default:
			len = strlen(line) + 1;
			value = malloc(len);
			memcpy(value, line, len);
	}
}


void
get_user_param(const char *module, user_param_t param, carmen_FILE *user_pref, const char *filename)
{
	static char line[10000], line_module[1000], line_variable[1000], line_value[8000];

	carmen_fseek(user_pref, 0L, SEEK_SET);

	while (carmen_fgets(line, 9999, user_pref) != NULL)
	{
		if ((line[0] == '#') ||
			(sscanf(line, " %s %s %s ", line_module, line_variable, line_value) != 3) ||
			(strcmp(line_module, module) != 0 || strcmp(line_variable, param.variable) != 0))
			continue;

		get_param_value(param.value, param.type, line_value, filename);
		break;
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
			sprintf(value_str, "%s", *(int *) param_value == 0 ? "off" : "on");
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
user_preferences_read_from_file(const char *module, user_param_t *param_list, int num_items, const char *filename)
{
	if (num_items <= 0 || filename == NULL || filename[0] == '\0')
		return;

	carmen_FILE *user_pref = carmen_fopen(filename, "r");
	if (user_pref == NULL)
		return;

	for (int i = 0; i < num_items; i++)
		get_user_param(module, param_list[i], user_pref, filename);

	carmen_fclose(user_pref);
}


void
user_preferences_read(const char *module, user_param_t *param_list, int num_items)
{
	user_preferences_read_from_file(module, param_list, num_items, USER_DEFAULT_FILENAME);
}


void
user_preferences_read_commandline(int argc, char **argv, user_param_t *param_list, int num_items)
{
	if (num_items <= 0)
		return;

	for (int i = 1; i < (argc - 1); i++)
	{
		if (argv[i][0] != '-')
			continue;

		int index = find_variable_in_param_list(&argv[i][1], param_list, num_items);
		if (index < 0)
			continue;

		i++;
		get_param_value(param_list[index].value, param_list[index].type, argv[i]);
	}
}


void
user_preferences_save_to_file(const char *module, user_param_t *param_list, int num_items, const char *filename)
{
	if (num_items <= 0)
		return;

	if (filename == NULL || filename[0] == '\0')
	{
		carmen_warn("WARNING: No filename provided for saving user preferences\n");
		return;
	}

	char new_filename[2005];
	strncpy(new_filename, filename, 2000);
	strcat(new_filename, "_new");

	carmen_FILE *user_pref = carmen_fopen(filename, "r");
	carmen_FILE *new_user_pref = carmen_fopen(new_filename, "w");
	if (new_user_pref == NULL)
	{
		carmen_warn("WARNING: Could not open user preferences file for writing: %s  (errno: %d)\n", new_filename, errno);
		return;
	}

	char line[10000], line_module[1000], line_variable[1000], line_value[8000], param_value_str[8000];
	int index, len = 0, saved_count = 0;
	int *saved_list = (int *) calloc(num_items, sizeof(int));

	while (user_pref != NULL && carmen_fgets(line, 9999, user_pref) != NULL)
	{
		if ((line[0] == '#') ||
			(sscanf(line, " %s %s %n%s ", line_module, line_variable, &len, line_value) != 3) ||
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
			carmen_fwrite(line, len, 1, new_user_pref);
			param_value_to_str(param_value_str, param_list[index].value, param_list[index].type);
			carmen_fprintf(new_user_pref, "%s   # %s", param_value_str, line + len);
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
		char bak_filename[2005];
		strncpy(bak_filename, filename, 2000);
		strcat(bak_filename, "_bak");
		remove(bak_filename);
		rename(filename, bak_filename);
	}
	carmen_fclose(new_user_pref);
	rename(new_filename, filename);
}


void
user_preferences_save(const char *module, user_param_t *param_list, int num_items)
{
	user_preferences_save_to_file(module, param_list, num_items, USER_DEFAULT_FILENAME);
}
