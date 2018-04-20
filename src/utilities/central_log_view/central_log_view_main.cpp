#include <carmen/carmen.h>

char *g_reg_module = NULL;
bool g_reg_all = false;
int g_reg_count = 0;
char *g_logfile_name = NULL;
FILE *g_logfile = NULL;

bool
read_line(FILE *textfile, char *line, size_t *buffersize)
{
	line[0] = 0;
	size_t linesize = 0;
	long int textstart = ftell(textfile);
	char c = fgetc(textfile);

	while (!feof(textfile) &&  c != '\n')
	{
		linesize++;
		c = fgetc(textfile);
	}
	if (linesize == 0)
		return (false);

	if (linesize > *buffersize)
	{
		*buffersize = linesize;
		realloc(line, linesize + 1);
	}

	fseek(textfile, textstart, SEEK_SET);
	fgets(line, linesize + 1, textfile);
	fgetc(textfile); // discard the final '\n'

	return (true);
}


void
process_analysis()
{
	size_t buffersize = 2000;
	char *line = (char *) malloc(buffersize + 1);
	int num_scan;
	char name[2000];

	while (read_line(g_logfile, line, &buffersize))
	{
		num_scan = sscanf(line, "Registration: Message %s Found. Updating.", name);
		if (num_scan == 1)
		{
			// Registration record found
			if (g_reg_all || strcmp(g_reg_module, name) == 0)
			{
				printf("Registration: %s\n", name);
				g_reg_count++;
			}
		}
	}

	printf("\nRegistration total count: %d\n", g_reg_count);

	free(line);
}


void
usage(char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	vfprintf(stderr, fmt, args);
	va_end(args);

	fprintf(stderr, "Usage: central_log_view <logfile> [args]\n");
	fprintf(stderr, "[args]:\n");
	fprintf(stderr, "\t-r  {<module>|all}    : list registrations of a specific module or all modules\n");

	exit(-1);
}


void
read_parameters(int argc, char **argv)
{
//	carmen_param_t optional_param_list[] =
//	{
//		{(char *) "commandline", (char *) "r", CARMEN_PARAM_STRING, &(g_reg_module),   0, NULL},
//	};
//
//	carmen_param_install_params(argc, argv, optional_param_list, sizeof(optional_param_list) / sizeof(optional_param_list[0]));

	for (int i = 2; i < argc; i++)
	{
		if (strcmp(argv[i], "-r") == 0)
		{
			if (g_reg_module)
				usage((char *) "Only one -r argument is allowed.\n");
			if (i == argc - 1)
				usage((char *) "Module name expected after -r argument.\n");
			g_reg_module = argv[++i];
			g_reg_all = (strcmp(g_reg_module, "all") == 0);
			printf("List all registrations of module: %s\n", g_reg_module);
		}
		else
			usage((char *) "Invalid command line argument: %s\n", argv[i]);
	}
}


int
main(int argc, char **argv)
{
	if (argc < 2)
		usage((char *) "Error: wrong number of parameters: program requires 1 parameter and received %d parameter(s).\n"
				"Usage:\n %s <logfile>\n", argc - 1, argv[0]);

	if (strcmp(argv[1], "-h") == 0)
		usage(NULL);

    g_logfile_name = argv[1];
	g_logfile = fopen(g_logfile_name, "r");
	if (g_logfile == NULL)
		usage((char *) "Error: could not open file %s for reading.\n", g_logfile_name);
	printf("Central Log View Utility\n" "Log file: %s\n", g_logfile_name);

	read_parameters(argc, argv);

	process_analysis();

	fclose(g_logfile);

	return 0;
}
