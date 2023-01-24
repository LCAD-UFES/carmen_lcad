
#include <iostream>
#include <cstdlib>
#include <vector>
#include <carmen/carmen_param_file.h>
#include <carmen/util_strings.h>
#include <carmen/util_io.h>


CarmenParamFile::CarmenParamFile(const char *path)
{
	char line[4096];
	std::vector<std::string> parts;

	FILE *fptr = safe_fopen(path, "r");

	std::cout << std::endl;
	while (!feof(fptr))
	{
		fscanf(fptr, "\n%[^\n]\n", line);
		parts = string_split_once(line, " \t");

		if (parts.size() < 2) continue;
		if (parts[0][0] == '#') continue;

		if (_params.count(parts[0]))
			std::cout << parts[0] << ": " << _params[parts[0]] << " -> " << std::string(trim(string_split_once(parts[1], "#")[0])) << std::endl;

		_params.insert(std::pair<std::string, std::string>(std::string(parts[0]), std::string(trim(string_split_once(parts[1], "#")[0]))));
	}
	std::cout << std::endl;

	fclose(fptr);
}


void
CarmenParamFile::print()
{
	std::map<std::string, std::string>::iterator it;

	for (it = _params.begin(); it != _params.end(); it++)
		printf("%s -> %s\n", it->first.c_str(), it->second.c_str());
}


std::string
get_parameter_name(std::string parameter)
{
	return parameter.substr(0, parameter.find("_"));
}


int
CarmenParamFile::rewrite(const char *path)
{
	std::map<std::string, std::string>::iterator it;
	FILE *fp = fopen(path, "w");
	if (!fp)
		return 1;

	std::string parameter_name, last_parameter = get_parameter_name(_params.begin()->first);

	fprintf(fp, "[*]\n");
	fprintf(fp, "\n");
	fprintf(fp, "# parameters for all robots\n");
	fprintf(fp, "\n");
	for (it = _params.begin(); it != _params.end(); it++)
	{
		parameter_name = get_parameter_name(it->first);
		if (last_parameter != parameter_name)
			fprintf(fp, "\n");
		fprintf(fp, "%s\t%s\n", it->first.c_str(), it->second.c_str());

		last_parameter = parameter_name;
	}
	
	fclose(fp);

	return 0;
}
