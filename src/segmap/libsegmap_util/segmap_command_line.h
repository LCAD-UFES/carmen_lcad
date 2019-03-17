
#ifndef __SEGMAP_COMMAND_LINE_H__
#define __SEGMAP_COMMAND_LINE_H__

#include <vector>
#include <string>
#include <sstream>
#include <boost/program_options.hpp>

namespace po = boost::program_options;


class CommandLineArguments
{
public:

	CommandLineArguments();
	~CommandLineArguments();

	template<class T>
	void add(std::string pattern, std::string description, T default_value);

	template<class T>
	void add(std::string pattern, std::string description);

	template<class T>
	void add_positional(std::string name, std::string description, int nmax = -1);

	void save_config_file(std::string path);

	po::variables_map parse(int argc, char **argv);

protected:

	po::positional_options_description *_positional_args_for_parsing;
	po::options_description *_positional_args;
	po::options_description *_additional_args;

	std::string _config_path;
	std::vector<std::string> _all_positional_args;

	std::vector<std::pair<std::string, std::string>> _additional_args_and_default_values;

	static std::string _filter_argument_name(std::string &pattern);

	void _show_help_message_if_necessary(po::variables_map vmap, char *program_name);
	void _read_args_from_config_file(po::options_description &config_file_options, po::variables_map *vmap);
};


template<class T> void
CommandLineArguments::add(std::string pattern, std::string description, T default_value)
{
	_additional_args->add_options()(pattern.c_str(),
			po::value<T>()->default_value(default_value),
			description.c_str());

	// store argument name, and default value as a string in case
	// the user wants to save a config file.
	std::string name, value;

	name = _filter_argument_name(pattern);

	// convert default value to string
	std::stringstream serializer;
	serializer << default_value;
	value = serializer.str();

	std::pair<std::string, std::string> name_and_val(name, value);
	_additional_args_and_default_values.push_back(name_and_val);
}


template<class T> void
CommandLineArguments::add(std::string pattern, std::string description)
{
	_additional_args->add_options()(pattern.c_str(), po::value<T>(), description.c_str());
}


template<class T> void
CommandLineArguments::add_positional(std::string name, std::string description, int nmax)
{
	_positional_args->add_options()(name.c_str(), po::value<T>(), description.c_str());
	_positional_args_for_parsing->add(name.c_str(), nmax);
	_all_positional_args.push_back(name);
}


#endif
