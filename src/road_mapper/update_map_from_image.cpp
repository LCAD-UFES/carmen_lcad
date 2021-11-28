#include "road_mapper_utils.h"
#include <wordexp.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>


char *g_input_dir = (char *) ".";
char *g_out_dir = (char *) ".";
char *g_image_file;
cv::Mat g_image;
double g_x_origin;
double g_y_origin;
int g_x_size = 350;
int g_y_size = 350;
double g_resolution = 0.2;


void
prog_usage(char *prog_name, const char *error_msg = NULL, const char *error_msg2 = NULL)
{
	if (error_msg)
		fprintf(stderr, "\n%s", error_msg);
	if (error_msg2)
		fprintf(stderr, "%s", error_msg2);

	fprintf(stderr, "\n\nUsage:    %s   <image_file>  -input_dir <dir>  -out_dir <dir>  -resolution <meters>  -x_size <num>  -y_size <num>\n", prog_name);
	fprintf(stderr,    "defaults: %*c                 -input_dir .      -out_dir .      -resolution 0.2       -x_size 350    -y_size 350\n\n", int(strlen(prog_name)), ' ');

	exit(-1);
}


static int
get_map_origin_by_filename(char *full_path, double *x_origin, double *y_origin, char map_type = 'm')
{
	char *filename, *file_extension;
	double x, y;
	int pos;

	*x_origin = *y_origin = 0.0;

	filename = strrchr(full_path, '/');
	if (filename == NULL)
		filename = full_path;
	else
		filename++;

	if (strncmp(filename, "complete_", 9) == 0)
		filename += 9;

	file_extension = strrchr(filename, '.');
	if (file_extension == NULL)
		file_extension = filename + strlen(filename);
	else
		file_extension++;

	if (filename[0] == map_type && sscanf(filename, "%*c%lf_%lf%n", &x, &y, &pos) == 2 && (filename + pos) == file_extension)
	{
		*x_origin = x;
		*y_origin = y;
		return 0;
	}
	return -1;
}


void
update_map()
{

}


void
get_param_string(char **param_string, int argc, char **argv, int i)
{
	if (i + 1 >= argc)
		prog_usage(argv[0], "string argument expected following: ", argv[i]);
	*param_string = argv[i + 1];
}


void
get_param_int(int *param_int, int argc, char **argv, int i)
{
	if (i + 1 >= argc)
		prog_usage(argv[0], "integer argument expected following ", argv[i]);
	char *endptr;
	*param_int = (int) strtol(argv[i + 1], &endptr, 0);
	if (*endptr != '\0' || *param_int <= 0)
		prog_usage(argv[0], "invalid positive integer number: ", argv[i + 1]);
}


void
get_param_double(double *param_double, int argc, char **argv, int i)
{
	if (i + 1 >= argc)
		prog_usage(argv[0], "floating point argument expected following ", argv[i]);
	char *endptr;
	*param_double = strtod(argv[i + 1], &endptr);
	if (*endptr != '\0' || *param_double <= 0.0)
		prog_usage(argv[0], "invalid positive floating point number: ", argv[i + 1]);
}


static void
read_parameters(int argc, char **argv)
{
	char *input_dir = NULL;
	char *out_dir = NULL;

	g_image_file = argv[1];
	g_image = cv::imread(g_image_file);
	if (g_image.empty())
		prog_usage(argv[0], "invalid image file: ", g_image_file);
	if (get_map_origin_by_filename(g_image_file, &g_x_origin, &g_y_origin, 'm') != 0)
		prog_usage(argv[0], "invalid image file complete_m<x>_<y> or m<x>_<y>: ", g_image_file);

	for (int i = 2; i < argc; i += 2)
	{
		if (strcmp(argv[i], "-input_dir") == 0)
			get_param_string(&input_dir, argc, argv, i);
		else if (strcmp(argv[i], "-out_dir") == 0)
			get_param_string(&out_dir, argc, argv, i);
		else if (strcmp(argv[i], "-resolution") == 0)
			get_param_double(&g_resolution, argc, argv, i);
		else if (strcmp(argv[i], "-x_size") == 0)
			get_param_int(&g_x_size, argc, argv, i);
		else if (strcmp(argv[i], "-y_size") == 0)
			get_param_int(&g_y_size, argc, argv, i);
		else
			prog_usage(argv[0], "invalid option: ", argv[i]);
	};

	if (input_dir)
	{
		// expand environment variables on path to full path
		wordexp_t we_input_dir;
		wordexp(input_dir, &we_input_dir, 0);
		g_input_dir = realpath(*we_input_dir.we_wordv, NULL);
		wordfree(&we_input_dir);

		struct stat st_input_dir;
		int st = stat(g_input_dir, &st_input_dir);
		if (st != 0 || !S_ISDIR(st_input_dir.st_mode))
			prog_usage(argv[0], "invalid -input_dir: ", input_dir);
	}

	if (out_dir)
	{
		// expand environment variables on path to full path
		wordexp_t we_out_dir;
		wordexp(out_dir, &we_out_dir, 0);
		g_out_dir = realpath(*we_out_dir.we_wordv, NULL);
		wordfree(&we_out_dir);

		struct stat st_out_dir;
		int st = stat(g_out_dir, &st_out_dir);
		if (st != 0 || !S_ISDIR(st_out_dir.st_mode))
			prog_usage(argv[0], "invalid -out_dir: ", out_dir);
	}
}


void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		fprintf(stderr,"\nInterrupt signal received\n\n");
		exit(0);
	}
}


int
main(int argc, char **argv)
{
	if (argc < 2 || strcmp(argv[1], "-h") == 0)
		prog_usage(argv[0]);

	signal(SIGINT, shutdown_module);
	read_parameters(argc, argv);

	update_map();

	return 0;
}
