#include <carmen/carmen.h>

char *g_reg_module = NULL;
char *g_dereg_module = NULL;
char *g_logfile_name = NULL;
carmen_FILE *g_logfile = NULL;

void
usage(char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	vfprintf(stderr, fmt, args);
	va_end(args);

	fprintf(stderr, "Usage: central_log_view <logfile> [args]\n");
	fprintf(stderr, "[args]: -r {<module>|all} -d {<module>|all}\n");
	exit(-1);
}


static void
read_parameters(int argc, char **argv)
{
	carmen_param_t optional_param_list[] =
	{
		{(char *) "commandline", (char *) "r", CARMEN_PARAM_STRING, &(g_reg_module),   0, NULL},
		{(char *) "commandline", (char *) "d", CARMEN_PARAM_STRING, &(g_dereg_module), 0, NULL},
	};

	carmen_param_allow_unfound_variables(true);
	carmen_param_install_params(argc, argv, optional_param_list, sizeof(optional_param_list) / sizeof(optional_param_list[0]));
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
	g_logfile = carmen_fopen(g_logfile_name, "r");
	if (g_logfile == NULL)
		usage((char *) "Error: could not open file %s for reading.\n", g_logfile_name);

	read_parameters(argc, argv);

	return 0;
}
