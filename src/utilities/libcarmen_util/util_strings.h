
#ifndef __SEGMAP_UTIL_STRINGS_H__
#define __SEGMAP_UTIL_STRINGS_H__

#include <vector>
#include <string>
#include <sstream>

std::string trim(std::string s);
std::vector<std::string> string_split(std::string s, std::string pattern);
std::vector<std::string> string_split_once(std::string s, std::string pattern);
std::string replace(const std::string &base, const std::string &from, const std::string &to);

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
