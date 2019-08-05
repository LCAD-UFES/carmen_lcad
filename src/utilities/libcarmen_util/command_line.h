
#ifndef __SEGMAP_COMMAND_LINE_H__
#define __SEGMAP_COMMAND_LINE_H__

#include <vector>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <carmen/util_strings.h>
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

	// use nmax <= 0 for assigning an unlimited number of values for the argument.
	template<class T>
	void add_positional(std::string name, std::string description, int nmax = 1);

	void save_config_file(std::string path);
	std::string help_message(const char *program_name);

	void parse(int argc, char **argv);

	template<class T>
	T get(const std::string &name) const;

protected:

	class ArgumentWithDefaultValue
	{
	public:
		std::string name;
		std::string description;
		std::string default_value;

		ArgumentWithDefaultValue(std::string arg_name,
														 std::string arg_description,
														 std::string arg_default_value)
		{
			name = arg_name;
			description = arg_description;
			default_value = arg_default_value;
		}
	};


	po::variables_map _args;
	po::positional_options_description *_positional_args_for_parsing;
	po::options_description *_positional_args;
	po::options_description *_non_positional_args;

	std::string _save_path;
	std::string _config_path;
	std::vector<std::string> _all_positional_args;
	std::vector<std::string> _non_positional_args_without_default_values;
	std::vector<ArgumentWithDefaultValue> _non_positional_args_and_default_values;

	static std::string _filter_argument_name(std::string &pattern);
	std::vector<std::string> _find_missing_mandatory_arguments(po::variables_map &vmap);
	void _read_args_from_config_file(po::options_description &config_file_options, po::variables_map *vmap);
};


template<class T> void
CommandLineArguments::add(std::string pattern, std::string description, T default_value)
{
	_non_positional_args->add_options()(pattern.c_str(),
			po::value<T>()->default_value(default_value),
			description.c_str());

	// store argument name, and default value as a string in case
	// the user wants to save a config file.
	std::string name, value;

	name = _filter_argument_name(pattern);
	value = to_string<T>(default_value);

	po::option_description desc = _non_positional_args->find(name, 0);
	_non_positional_args_and_default_values.push_back(ArgumentWithDefaultValue(name, description, value));
}


template<class T> void
CommandLineArguments::add(std::string pattern, std::string description)
{
	_non_positional_args_without_default_values.push_back(_filter_argument_name(pattern));
	_non_positional_args->add_options()(pattern.c_str(), po::value<T>(), description.c_str());
}


template<class T> void
CommandLineArguments::add_positional(std::string name, std::string description, int nmax)
{
	_positional_args->add_options()(name.c_str(), po::value<T>(), description.c_str());
	_positional_args_for_parsing->add(name.c_str(), nmax);
	_all_positional_args.push_back(name);
}


template<class T> T
CommandLineArguments::get(const std::string &name) const
{
	if (_args.count(name) <= 0)
		exit(printf("Error: argument '%s' not found.\n", name.c_str()));

	return _args[name.c_str()].as<T>();
}


#endif
