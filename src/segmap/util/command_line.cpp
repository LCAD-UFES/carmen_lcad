
#include <carmen/command_line.h>

#include <vector>
#include <string>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <boost/program_options.hpp>


namespace po = boost::program_options;
using namespace std;


CommandLineArguments::CommandLineArguments()
{
	_positional_args = new po::options_description("Positional Arguments");
	_non_positional_args = new po::options_description("Additional Arguments");
	_positional_args_for_parsing = new po::positional_options_description();

	_non_positional_args->add_options()("help,h", "produce help message");
	_non_positional_args->add_options()("config,c", po::value<string>(&_config_path)->default_value(""), "config file");
}


CommandLineArguments::~CommandLineArguments()
{
	delete(_positional_args_for_parsing);
	delete(_positional_args);
	delete(_non_positional_args);
}


std::string
CommandLineArguments::_filter_argument_name(std::string &pattern)
{
	size_t p;
	std::string name;

	p = pattern.find(",");

	// if comma is absent, the pattern only contains the name.
	// else, the name is the substring from start until the comma's position.
	if (p == std::string::npos)
		name = std::string(pattern);
	else
		name = pattern.substr(0, p);

	return name;
}


void
CommandLineArguments::save_config_file(std::string path)
{
	FILE *f = fopen(path.c_str(), "w");

	if (f == NULL)
		exit(printf("Error: unable to save config file '%s'\n", path.c_str()));

	for (int i = 0; i < _non_positional_args_and_default_values.size(); i++)
	{
		fprintf(f, "# %s\n%s = %s\n",
						_non_positional_args_and_default_values[i].description.c_str(),
						_non_positional_args_and_default_values[i].name.c_str(),
						_non_positional_args_and_default_values[i].default_value.c_str());
	}

	fclose(f);
}


std::string
CommandLineArguments::help_message(const char *program_name)
{
	std::string msg;
	std::string positional_args_as_str = "";

	// check if positional arguments are missing
	for (int i = 0; i < _all_positional_args.size(); i++)
		positional_args_as_str += " " + _all_positional_args[i];

	msg += "\nUsage: ";
	msg += program_name;
	msg += " [additional args] " + positional_args_as_str + "\n\n";
	msg += to_string(*_non_positional_args);
	msg += "\n";

	return msg;
}


vector<string>
CommandLineArguments::_find_missing_mandatory_arguments(po::variables_map &vmap)
{
	vector<string> missing_arguments;

	// check if positional arguments are missing
	for (int i = 0; i < _all_positional_args.size(); i++)
		if (vmap.count(_all_positional_args[i]) == 0)
			missing_arguments.push_back(_all_positional_args[i]);

	// check if non-positional, but mandatory arguments are missing
	for (int i = 0; i < _non_positional_args_without_default_values.size(); i++)
		if (vmap.count(_non_positional_args_without_default_values[i]) == 0)
			missing_arguments.push_back(_non_positional_args_without_default_values[i]);

	return missing_arguments;
}


void
CommandLineArguments::_read_args_from_config_file(po::options_description &config_file_options,
																									po::variables_map *vmap)
{
	if (_config_path.size() > 0)
	{
		ifstream ifs(_config_path.c_str());

		if (ifs)
		{
			store(parse_config_file(ifs, config_file_options), *vmap);
			notify(*vmap);
		}
	}
}


void
CommandLineArguments::parse(int argc, char **argv)
{
	vector<string> missing_arguments;
	po::options_description all_options;
	all_options.add(*_positional_args).add(*_non_positional_args);

	// parse arguments from command line
	store(po::command_line_parser(argc, argv).options(all_options).positional(*_positional_args_for_parsing).run(), _args);
	notify(_args);

	_read_args_from_config_file(all_options, &_args);
	missing_arguments = _find_missing_mandatory_arguments(_args);

	if (_args.count("help") || missing_arguments.size() > 0)
	{
		for (string arg : missing_arguments)
			cerr << "Error: Missing mandatory argument '" << arg << "'" << endl;

		cerr << help_message(argv[0]) << endl;
		exit(-1);
	}

}

