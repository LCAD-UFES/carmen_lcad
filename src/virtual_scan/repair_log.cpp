#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

char **
str_split(char *a_str, const char a_delim, int &size)
{
	char **result = 0;
	size_t count = 0;
	char *tmp = a_str;
	char *last_comma = 0;
	char delim[2];
	delim[0] = a_delim;
	delim[1] = 0;

	/* Count how many elements will be extracted. */
	while (*tmp != '\n')
	{
		if (a_delim == *tmp)
		{
			count++;
			last_comma = tmp;
		}
		tmp++;
	}
	size = count + 1;

	/* Add space for trailing token. */
	count += last_comma < (a_str + strlen(a_str) - 1);

	/* Add space for terminating null string so caller
	 knows where the list of returned strings ends. */
	count++;

	result = (char **) malloc(sizeof(char *) * count);

	if (result)
	{
		size_t idx = 0;
		char *token = strtok(a_str, delim);

		while (token)
		{
			assert(idx < count);
			*(result + idx++) = strdup(token);
			token = strtok(0, delim);
		}
		assert(idx == count - 1);
		*(result + idx) = 0;
	}

	return (result);
}


int
main(int argc, char **argv)
{
	if (argc != 3)
	{
		printf("Error: Wrong number of parameters.\n Usage: repair_log input_log.txt output_log.txt\n");
		exit (1);
	}

	FILE *input_log = fopen(argv[1], "r");
	if (input_log == NULL)
	{
		printf("Error: Could not open input_log.txt %s\n", argv[1]);
		exit (1);
	}

	FILE *output_log = fopen(argv[2], "w");
	if (output_log == NULL)
	{
		printf("Error: Could not open output_log.txt %s\n", argv[1]);
		exit (1);
	}

	bool first_time = true;
	double first_timestamp;
	char *input_line = (char *) malloc(50000000 * sizeof(char));
	while (fgets(input_line, 50000000-1, input_log))
	{
		int size;
		char **token = str_split(input_line, ' ', size);

		if (token)
		{
			bool change_timestamp = true;
			if ((strcmp(token[0], "#") == 0) ||
				(strcmp(token[0], "PARAM") == 0))
				change_timestamp = false;

			if ((strcmp(token[0], "GLOBALPOS_ACK") == 0) ||
				(strcmp(token[0], "FUSED_ODOM") == 0))
				continue;

			for (int i = 0; *(token + i); i++)
			{
				char *current_string = *(token + i);
				double timestamp;

				if (change_timestamp && (i == (size - 3)))
				{
					timestamp = atof(current_string);
					if (first_time)
					{
						first_timestamp = timestamp;
						first_time = false;
					}
				}

				if (change_timestamp && (i == size - 1))
					fprintf(output_log, "%lf\n", timestamp - first_timestamp);
				else
				{
					if (current_string[strlen(current_string) - 1] == '\n')
						fprintf(output_log, "%s", current_string);
					else
						fprintf(output_log, "%s ", current_string);
				}

				free(current_string);
			}

			free(token);
		}
	}
	free(input_line);

	return (0);
}
