
#ifndef __SEGMAP_UTIL_STRINGS_H__
#define __SEGMAP_UTIL_STRINGS_H__

#include <vector>
#include <string>
#include <sstream>

std::vector<std::string> string_split(std::string s, std::string pattern);


template<class T>
std::string to_string(T value)
{
	std::stringstream serializer;
	serializer << value;
	return (serializer.str());
}


template<class T>
T from_string(const std::string &value)
{
	T output;
	std::stringstream deserializer(value);
	deserializer >> output;
	return output;
}


#endif
