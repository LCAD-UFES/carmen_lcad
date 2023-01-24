
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>
#include <carmen/util_strings.h>

using namespace std;


string
trim(string s) { boost::trim(s); return s; }

vector<string>
string_split_once(string s, string pattern)
{
	vector<string> splitted;

    size_t equals_idx = s.find_first_of(pattern);
    if (std::string::npos != equals_idx)
    {
		splitted.push_back(s.substr(0, equals_idx));
        splitted.push_back(s.substr(equals_idx + 1));
    }
    else
	{
		splitted.push_back(s);
	}
	
	return splitted;
}


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


std::string 
replace(const std::string &base, const std::string &from, const std::string &to)
{
	std::string output;
	int found, substr_start;
	
	found = base.find(from);
	substr_start = 0;
	
	while (found != std::string::npos)
	{
		output += base.substr(substr_start, found - substr_start);
		output += to;
		
		// aguabolabolaagua
		substr_start = found + from.size();
		found = base.find(from, found + from.size());
	}

	if (substr_start < base.size())
		output += base.substr(substr_start, base.size());

	return output;
}


