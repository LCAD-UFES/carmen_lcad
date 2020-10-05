#include <carmen/carmen.h>
#include <carmen/rddf_interface.h>
#include <carmen/rddf_util.h>


char *prog_name = NULL;
char action_char = ' ';
crud_t action = (crud_t) -1;
carmen_annotation_t annotation1 = {{0,0,0}, 0, NULL, 0, 0};
carmen_annotation_t annotation2 = {{0,0,0}, 0, NULL, 0, 0};


//void
//timer_handler()
//{
//
//}


static void
shutdown_module(int sig)
{
	(void) sig;

	carmen_ipc_disconnect();
	fprintf(stderr, "\nModule exited by soft shutdown\n\n");
	exit(0);
}


//void
//subscribe_messages()
//{
//
//}


void
module_usage(const char *err_msg = NULL)
{
	if (err_msg)
		fprintf(stderr, "\n%s\n", err_msg);

    fprintf(stderr, "\nUsage:\t %s  <action>  -x <val>  -y <val>  -z <val>  -theta <val>  -descr <name>  -type <int>  -code <int>  "
    				"-x2 <val>  -y2 <val>  -z2 <val>  -theta2 <val>  -descr2 <name>  -type2 <int>  -code2 <int>\n", prog_name);
    fprintf(stderr, "\taction: c (create) | u (update) | d (delete)\n");
    exit(1);
}


void
process_parameters()
{
	if (action == CREATE_ACTION)
	{
//		if (annotation1.annotation_point.x == 0.0 || annotation1.annotation_point.y == 0.0)
//			module_usage("required arguments: x, y");
		carmen_rddf_publish_update_annotation_message(CREATE_ACTION, annotation1, annotation1);
		printf("\nPublished carmen_rddf_update_annotation_message:\n");
		printf("action = %d (%c)  x = %lf  y = %lf  z = %lf  theta = %lf  description = %s  type = %d  code = %d\n", action, action_char,
				annotation1.annotation_point.x, annotation1.annotation_point.y, annotation1.annotation_point.z,
				annotation1.annotation_orientation, annotation1.annotation_description, annotation1.annotation_type, annotation1.annotation_code);
	}
	else if (action == UPDATE_ACTION)
	{
//		if (annotation1.annotation_point.x == 0.0 || annotation1.annotation_point.y == 0.0 || annotation2.annotation_point.x == 0.0 || annotation2.annotation_point.y == 0.0)
//			module_usage("required arguments: x, y, x2, y2");
		carmen_rddf_publish_update_annotation_message(UPDATE_ACTION, annotation1, annotation2);
		printf("\nPublished carmen_rddf_update_annotation_message:\n");
		printf("action = %d (%c)  x  = %lf  y  = %lf  z  = %lf  theta  = %lf  description  = %s  type  = %d  code  = %d\n", action, action_char,
				annotation1.annotation_point.x, annotation1.annotation_point.y, annotation1.annotation_point.z,
				annotation1.annotation_orientation, annotation1.annotation_description, annotation1.annotation_type, annotation1.annotation_code);
		printf("                x2 = %lf  y2 = %lf  z2 = %lf  theta2 = %lf  description2 = %s  type2 = %d  code2 = %d\n",
				annotation2.annotation_point.x, annotation2.annotation_point.y, annotation2.annotation_point.z,
				annotation2.annotation_orientation, annotation2.annotation_description, annotation2.annotation_type, annotation2.annotation_code);
	}
	else if (action == DELETE_ACTION)
	{
//		if (annotation1.annotation_point.x == 0.0 || annotation1.annotation_point.y == 0.0)
//			module_usage("required arguments: x, y");
		carmen_rddf_publish_update_annotation_message(DELETE_ACTION, annotation1, annotation1);
		printf("\nPublished carmen_rddf_update_annotation_message:\n");
		printf("action = %d (%c)  x = %lf  y = %lf  z = %lf  theta = %lf  description = %s  type = %d  code = %d\n", action, action_char,
				annotation1.annotation_point.x, annotation1.annotation_point.y, annotation1.annotation_point.z,
				annotation1.annotation_orientation, annotation1.annotation_description, annotation1.annotation_type, annotation1.annotation_code);
	}
}


void
read_parameters(int argc, char *argv[])
{
	action = (strcmp(argv[1], "c") == 0) ? CREATE_ACTION :
			 (strcmp(argv[1], "u") == 0) ? UPDATE_ACTION :
			 (strcmp(argv[1], "d") == 0) ? DELETE_ACTION : (crud_t) -1;

	if (action < 0)
		module_usage("A single action must be used: c, u, d");

	action_char = argv[1][0];

	carmen_param_t param_list[] =
	{
		{(char *) "commandline", (char *) "x",		CARMEN_PARAM_DOUBLE, &annotation1.annotation_point.x, 0, NULL},
		{(char *) "commandline", (char *) "y",		CARMEN_PARAM_DOUBLE, &annotation1.annotation_point.y, 0, NULL},
		{(char *) "commandline", (char *) "z",		CARMEN_PARAM_DOUBLE, &annotation1.annotation_point.z, 0, NULL},
		{(char *) "commandline", (char *) "theta",  CARMEN_PARAM_DOUBLE, &annotation1.annotation_orientation, 0, NULL},
		{(char *) "commandline", (char *) "descr",	CARMEN_PARAM_STRING, &annotation1.annotation_description, 0, NULL},
		{(char *) "commandline", (char *) "type",	CARMEN_PARAM_INT,	 &annotation1.annotation_type, 0, NULL},
		{(char *) "commandline", (char *) "code",	CARMEN_PARAM_INT,	 &annotation1.annotation_code, 0, NULL},
		{(char *) "commandline", (char *) "x2",		CARMEN_PARAM_DOUBLE, &annotation2.annotation_point.x, 0, NULL},
		{(char *) "commandline", (char *) "y2",		CARMEN_PARAM_DOUBLE, &annotation2.annotation_point.y, 0, NULL},
		{(char *) "commandline", (char *) "z2",		CARMEN_PARAM_DOUBLE, &annotation2.annotation_point.z, 0, NULL},
		{(char *) "commandline", (char *) "theta2", CARMEN_PARAM_DOUBLE, &annotation2.annotation_orientation, 0, NULL},
		{(char *) "commandline", (char *) "descr2",	CARMEN_PARAM_STRING, &annotation2.annotation_description, 0, NULL},
		{(char *) "commandline", (char *) "type2",	CARMEN_PARAM_INT,	 &annotation2.annotation_type, 0, NULL},
		{(char *) "commandline", (char *) "code2",	CARMEN_PARAM_INT,	 &annotation2.annotation_code, 0, NULL},
	};

	int num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_allow_unfound_variables(1);
	carmen_param_install_params(argc, argv, param_list, num_items);

	process_parameters();
}


int
main(int argc, char *argv[])
{
	prog_name = argv[0];
	if (argc < 2 || strcmp(argv[1], "-h") == 0)
		module_usage();

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	signal(SIGINT, shutdown_module);
//	subscribe_messages();
	read_parameters(argc, argv);
//  carmen_ipc_addPeriodicTimer(0.5, (TIMER_HANDLER_TYPE) timer_handler, NULL);
	carmen_ipc_dispatch();
	return 0;
}
