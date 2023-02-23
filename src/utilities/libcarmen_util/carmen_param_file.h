
#ifndef __SEGMAP_CARMEN_PARAM_FILE_H__
#define __SEGMAP_CARMEN_PARAM_FILE_H__

#include <map>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <typeinfo>
#include <boost/lexical_cast.hpp>
#include <assert.h>
#include <typeinfo>


class CarmenParamFile
{
public:
	CarmenParamFile();
	CarmenParamFile(const char *path);
	template<typename T> T get(std::string name);
	template<typename T> void set(std::string name, T value);
	void print();
	int rewrite(const char *path);

protected:
	std::map<std::string, std::string> _params;
};


template<typename T> T
CarmenParamFile::get(std::string name)
{
	//printf("name: %s\n", name.c_str());
	//printf("_params.size(): %ld\n", _params.size());
	//printf("_params.count(%s): %ld\n", name.c_str(), _params.count(name));

	if (_params.count(name) == 0)
		exit(printf("Error: parameter '%s' not found.\n", name.c_str()));

	std::string value;

	value = _params[name];

	// convert booleans to int
	if ((_params[name].compare("on") == 0) ||
			(_params[name].compare("ON") == 0))
		value = "1";
	else if ((_params[name].compare("off") == 0) ||
			(_params[name].compare("OFF") == 0))
		value = "0";

	return boost::lexical_cast<T>(value);
}


template<typename T> void
CarmenParamFile::set(std::string name, T value)
{
	// if (_params.count(name) == 0)
	// 	exit(printf("Error: parameter '%s' not found.\n", name.c_str()));

	assert(typeid(value)==typeid(boost::lexical_cast<T>(_params[name])));

	_params[name] = boost::lexical_cast<std::string>(value);
}

#endif
