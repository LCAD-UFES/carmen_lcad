
#include <cstdio>
#include <carmen/segmap_command_line.h>


int
main(int argc, char **argv)
{
	CommandLineArguments args;

	args.add<int>("size,s", "Size of something", 10);
	args.add<double>("multiplier,m", "Some multiplier", 2.53);
	args.save_config_file("config.txt");
	args.parse(argc, argv);

	printf("size: %d\n", args.get<int>("size"));
	printf("multiplier: %lf\n", args.get<double>("multiplier"));

	return 1;
}
