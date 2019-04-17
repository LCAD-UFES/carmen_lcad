
#include <vector>
#include <string>
#include <iostream>
#include <carmen/util_time.h>
#include <carmen/command_line.h>
using namespace std;


int
main(int argc, char **argv)
{
	Timer timer;
	vector<string> paths;
	CommandLineArguments args;

	// Mandatory arguments
	args.add_positional<vector<string>>("path", "Paths for a set of files.");
	args.add<double>("number,n", "A number to be printed on the screen");
	// Arguments with default values are optional. Only they are saved in config files.
	args.add<int>("print-time,t", "Flag for printing or not the duration of the program", 0);
	// Dump the arguments to a config file.
	args.save_config_file("config.cfg");
	// Parse arguments from argc, and argv
	args.parse(argc, argv);

	timer.start();

	paths = args.get<vector<string>>("path");

	for (int i = 0; i < paths.size(); i++)
		cout << "path[" << i << "]: " << paths[i] << endl;

	cout << "number: " << args.get<double>("number") << endl;

	if (args.get<int>("print-time"))
		cout << "Duration (s): " << timer.ellapsed() << endl;

	return 0;
}

