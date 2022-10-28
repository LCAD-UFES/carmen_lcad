
#ifndef __SEGMAP_UTIL_IO_H__
#define __SEGMAP_UTIL_IO_H__

#include <cstdio>
#include <string>
#include <vector>

typedef struct
{
	int size;
	std::vector<std::string> splitted_string;
} carmen_line_content;

FILE* safe_fopen(const char *path, const char *mode);

std::vector<double> read_vector(char *name);
std::string file_name_from_path(const char *path);
std::string file_name_from_path(std::string &path);

std::string read_line(FILE *fptr);
std::vector<std::string> string_split(std::string s, std::string pattern);
carmen_line_content create_carmen_line_content(std::string current_string, char* token = (char*) "\t ");
std::string get_string_from_carmen_line_content(carmen_line_content content, int index);
void carmen_print_line_content(carmen_line_content content, int print_mode = 1);

#endif
