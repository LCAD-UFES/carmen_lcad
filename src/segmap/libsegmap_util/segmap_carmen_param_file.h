
#ifndef __SEGMAP_CARMEN_PARAM_FILE_H__
#define __SEGMAP_CARMEN_PARAM_FILE_H__

#include <map>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <typeinfo>
#include <boost/lexical_cast.hpp>


class CarmenParamFile
{
public:
	CarmenParamFile(const char *path);
	template<typename T> T get(std::string name);

protected:
	std::map<std::string, std::string> _params;
};


template<typename T> T
CarmenParamFile::get(std::string name)
{
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


#endif
