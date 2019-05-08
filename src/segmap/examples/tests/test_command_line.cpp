
#include <vector>
#include <string>
#include <iostream>
#include <carmen/util_time.h>
#include <carmen/command_line.h>
using namespace std;


void
declare_and_parse_args(int argc, char **argv, CommandLineArguments *args)
{
	args->add_positional<string>("log_path", "Path to a log");
	args->add_positional<string>("carmen_ini", "Path to a file containing system parameters");
	args->add_positional<string>("output_calibration", "Path to an output calibration file");
	args->add_positional<string>("output_poses", "Path to a file in which poses will be saved for debug");
	args->add_positional<string>("poses_opt", "Path to a file in which the poses will be saved in graphslam format");
	args->add<int>("gps_to_use", "Id of the gps that will be used for the calibration", 1);
	args->add<int>("board_to_use", "Id of the sensor board that will be used for the calibration", 1);
	args->add<int>("use_non_linear_phi", "0 - linear phi; 1 - use a spline to map phi to a new phi", 0);
	args->add<int>("n_particles,n", "Number of particles", 500);
	args->add<int>("n_iterations,i", "Number of iterations", 300);
	args->add<int>("initial_log_line,l", "Number of lines to skip in the beggining of the log file", 1);
	args->add<int>("max_log_lines,m", "Maximum number of lines to read from the log file", -1);
	args->save_config_file("odom_calib_config.txt");
	args->parse(argc, argv);
}


int
main(int argc, char **argv)
{
	Timer timer;
	string paths;

	CommandLineArguments args;
	declare_and_parse_args(argc, argv, &args);

	return 0;

	timer.start();

	cout << "path: "<< args.get<string>("path") << endl;
	cout << "bola: " << args.get<string>("bola") << endl;	
	cout << "number: " << args.get<double>("number") << endl;

	if (args.get<int>("print-time"))
		cout << "Duration (s): " << timer.ellapsed() << endl;

	return 0;
}

