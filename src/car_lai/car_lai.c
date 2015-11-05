#include <stdio.h>
#include <carmen/carmen.h>


int 
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	//carmen_param_check_version(argv[0]);
	
	while(1)
	  printf ("%lf\n", carmen_get_time());
	
	carmen_ipc_dispatch();

	return 0;
}