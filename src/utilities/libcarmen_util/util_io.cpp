
#include <string>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <carmen/util_io.h>
#include <carmen/util_strings.h>

using namespace std;


FILE*
safe_fopen(const char *path, const char *mode)
{
	FILE *f = fopen(path, mode);

	if (f == NULL)
		exit(printf("fopen failed with path: '%s', and mode '%s'\n", path, mode));

	return f;
}


vector<double>
read_vector(char *name)
{
	double d;
	vector<double> data;

	FILE *f = fopen(name, "r");

	if (f == NULL)
		exit(printf("File '%s' not found.", name));

	while (!feof(f))
	{
		fscanf(f, "%lf\n", &d);
		data.push_back(d);
	}

	fclose(f);
	return data;
}


std::string
file_name_from_path(const char *path)
{
	vector<string> splitted = string_split(path, "/");
	return std::string(splitted[splitted.size() - 1]);
}


std::string
file_name_from_path(std::string &path)
{
	return file_name_from_path(path.c_str());
}


std::string 
read_line(FILE *fptr)
{
	char c;
	std::string line;
	
	while ((!feof(fptr)) && ((c = fgetc(fptr)) != '\n'))
		line.push_back(c);
		
	return line;
}


