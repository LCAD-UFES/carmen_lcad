
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <carmen/carmen_param_file.h>
#include <carmen/util_strings.h>
#include <carmen/util_io.h>


CarmenParamFile::CarmenParamFile(const char *path)
{
	static char line[1024];

	std::vector<std::string> parts;
	FILE *fptr = safe_fopen(path, "r");

	while (!feof(fptr))
	{
		fscanf(fptr, "\n%[^\n]\n", line);
		parts = string_split(line, " \t");

		if (parts.size() < 2) continue;
		if (parts[0][0] == '#') continue;

		_params[parts[0]] = parts[1];
	}

	fclose(fptr);
}
