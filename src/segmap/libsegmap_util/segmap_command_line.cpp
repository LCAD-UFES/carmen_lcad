
#include <vector>
#include <string>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <boost/program_options.hpp>
#include "segmap_command_line.h"

namespace po = boost::program_options;
using namespace std;


CommandLineArguments::CommandLineArguments()
{
	_positional_args = new po::options_description("Positional Arguments");
	_additional_args = new po::options_description("Additional Arguments");
	_positional_args_for_parsing = new po::positional_options_description();

	_additional_args->add_options()("help,h", "produce help message");
	_additional_args->add_options()("config,c", po::value<string>(&_config_path)->default_value(""), "config file");
}


CommandLineArguments::~CommandLineArguments()
{
	delete(_positional_args_for_parsing);
	delete(_positional_args);
	delete(_additional_args);
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

	for (int i = 0; i < _additional_args_and_default_values.size(); i++)
	{
		fprintf(f, "%s = %s\n",
						_additional_args_and_default_values[i].first.c_str(),
						_additional_args_and_default_values[i].second.c_str());
	}

	fclose(f);
}


void
CommandLineArguments::_show_help_message_if_necessary(po::variables_map vmap, char *program_name)
{
	int required_args_are_missing = 0;
	std::string all_args_str = "";

	// check if positional arguments are missing
	for (int i = 0; i < _all_positional_args.size(); i++)
	{
		if (vmap.count(_all_positional_args[i]) == 0)
		{
			required_args_are_missing = 1;
			all_args_str = all_args_str + " " + _all_positional_args[i];
		}
	}

	if (vmap.count("help") || required_args_are_missing)
	{
		std::cerr << std::endl << "Usage: " << program_name << " [additional args] " << all_args_str << std::endl;
		std::cerr << std::endl << *_additional_args << std::endl;
		exit(-1);
	}
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


po::variables_map
CommandLineArguments::parse(int argc, char **argv)
{
	po::variables_map vmap;
	po::options_description all_options;

	all_options.add(*_positional_args).add(*_additional_args);

	// parse arguments from command line
	store(po::command_line_parser(argc, argv).options(all_options).positional(*_positional_args_for_parsing).run(), vmap);
	notify(vmap);

	_read_args_from_config_file(all_options, &vmap);
	_show_help_message_if_necessary(vmap, argv[0]);

	return vmap;
}


