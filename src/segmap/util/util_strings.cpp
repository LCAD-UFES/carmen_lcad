
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>
#include <carmen/util_strings.h>

using namespace std;


vector<string>
string_split(string s, string pattern)
{
	vector<string> splitted, splitted_without_empties;

	boost::split(splitted, s, boost::is_any_of(pattern));

	for (int i = 0; i < splitted.size(); i++)
	{
		if (splitted[i].size() > 0)
			splitted_without_empties.push_back(splitted[i]);
	}

	return splitted_without_empties;
}


