
#include "segmap_carmen_param_file.h"
#include <carmen/segmap_util.h>
#include <vector>

using namespace std;

static char line[1024];


CarmenParamFile::CarmenParamFile(const char *path)
{
	vector<string> parts;
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

