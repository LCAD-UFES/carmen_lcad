
#include <string>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <carmen/util_io.h>
#include <carmen/util_strings.h>
#include <boost/algorithm/string.hpp>

using namespace std;


FILE*
safe_fopen(const char *path, const char *mode)
{
	FILE *f = fopen(path, mode);

	if (f == NULL)
		exit(printf("fopen failed with path: '%s', and mode '%s'\n", path, mode));

	return (f);
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
	return (data);
}


std::string
file_name_from_path(const char *path)
{
	vector<string> splitted = string_split(path, "/");
	return (std::string(splitted[splitted.size() - 1]));
}


std::string
file_name_from_path(std::string &path)
{
	return (file_name_from_path(path.c_str()));
}


std::string 
read_line(FILE *fptr)
{
	char c;
	std::string line;
	
	while ((!feof(fptr)) && ((c = fgetc(fptr)) != '\n'))
		line.push_back(c);
		
	return (line);
}


vector<string>
string_split(string s, string pattern)
{
	vector<string> splitted, splitted_without_empties;

	boost::split(splitted, s, boost::is_any_of(pattern), boost::token_compress_on);
	for (int i = 0; i < splitted.size(); i++)
	{
		if (splitted[i].size() > 0)
			splitted_without_empties.push_back(splitted[i]);
	}

	return (splitted_without_empties);
}


carmen_line_content
create_carmen_line_content(std::string current_string, char* token)
{
	carmen_line_content return_carmen_line_content;
	std::vector<std::string> splitted_string = string_split(current_string, token);
	return_carmen_line_content.size  = splitted_string.size();
	return_carmen_line_content.splitted_string = splitted_string;

	return (return_carmen_line_content);
}


std::string
get_string_from_carmen_line_content(carmen_line_content content, int index)
{
	int real_index = index;
	if (index < 0)
		real_index = content.size + index; // Isso é para simular o uso de index negativos no python;; index[-1] é o último elemento da lista

	if (real_index >= content.size)
		real_index = content.size - 1; // Se o index for maior que o tamanho de campos, retornar o último

	if (real_index < 0)
		real_index = 0; // Se o index passado por parametro for um index negativo maior que o size do vetor, obtêm-se o primeiro elemento do vetor

	return (content.splitted_string[real_index]);
}


void
carmen_print_line_content(carmen_line_content content, int print_mode)
{
	if (print_mode == 0)
		return;
	// Printa a linha inteira, mas separada por content, para verificar se a associação está correta
	for (size_t i = 0; i < content.size; ++i)
		printf("%ld:%s ", i, get_string_from_carmen_line_content(content, i).c_str());
	printf("\n");
}
