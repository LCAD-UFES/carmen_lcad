#include <stdio.h>
#include "carmen/ipc.h"
#include "carmen/global.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <dirent.h>  
#include <sys/stat.h> 
#include <string.h>
#include <cstring>    
#include <regex>
#include <sstream>

#define RECORD_LIMIT 20
bool  raw_message        = false;

std::vector<std::string> 
get_struct_field_names(const std::string& file_path, const char* msg_name) 
{
    std::ifstream file(file_path);
    std::vector<std::string> fields;

    if (!file.is_open()) 
    {
        std::cerr << "Cannot open file: " << file_path << std::endl;
        return fields;
    }

    // Read all lines of the file
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(file, line)) 
    {
        lines.push_back(line);
    }

    // Find the line with the struct alias
    int msg_line_index = -1;
    std::string quoted_msg = msg_name;
    for (int i = 0; i < (int)lines.size(); ++i) 
    {
        if (lines[i].find(quoted_msg) != std::string::npos) 
        {
            msg_line_index = i;
            break;
        }
    }
    if (msg_line_index == -1) 
    {
        std::cerr << "Message name not found: " << msg_name << std::endl;
        return fields;
    }

    // Go backwards to find typedef
    int struct_start_index = -1;
    for (int i = msg_line_index; i >= 0; --i) 
    {
        if (lines[i].find("typedef") != std::string::npos) 
        {
            struct_start_index = i;
            break;
        }
    }
    if (struct_start_index == -1) 
    {
        std::cerr << "typedef not found for message: " << msg_name << std::endl;
        return fields;
    }

    // Parse struct body
    bool struct_started = false;
    for (int i = struct_start_index; i < (int)lines.size(); ++i) 
    {
        std::string l = lines[i];

        // Remove single-line comments
        size_t comment_pos = l.find("//");
        if (comment_pos != std::string::npos) 
        {
            l = l.substr(0, comment_pos);
        }

        // Remove block comments (basic)
        comment_pos = l.find("/*");
        if (comment_pos != std::string::npos) 
        {
            l = l.substr(0, comment_pos);
        }

        // Detect struct opening
        if (!struct_started) 
        {
            if (l.find("{") != std::string::npos) 
            {
                struct_started = true;
            }
            continue;
        }

        // Detect struct closing
        if (l.find("}") != std::string::npos) 
        {
            break;
        }

        // Collapse extra spaces
        std::string trimmed;
        std::istringstream iss(l);
        std::string token;
        while (iss >> token) 
        {
            trimmed += token + " ";
        }
        if (trimmed.empty()) 
        {
            continue;
        }

        // Skip invalid lines
        size_t first_space = trimmed.find(' ');
        if (first_space == std::string::npos) 
        {
            continue;
        }

        // Extract type (everything before first space)
        std::string type = trimmed.substr(0, first_space);

        // Extract variable part (everything after the type)
        std::string vars_part = trimmed.substr(first_space + 1);

        // Split by comma
        std::istringstream vars(vars_part);
        std::string var;
        while (std::getline(vars, var, ',')) 
        {
            // Trim spaces
            std::string clean;
            std::istringstream vs(var);
            vs >> clean;

            // Remove initialization (everything after "=")
            size_t eq_pos = clean.find('=');
            if (eq_pos != std::string::npos) 
            {
                clean = clean.substr(0, eq_pos);
            }

            // Remove trailing semicolon
            while (!clean.empty() && clean.back() == ';') 
            {
                clean.pop_back();
            }

            // Keep array brackets [] intact
            if (!clean.empty()) 
            {
                fields.push_back(type + " " + clean);
            }
        }
    }

    return fields;
}


bool 
file_contains_string(const std::string& file_path, const char* str) 
{
    std::ifstream file(file_path);
    if (!file.is_open()) return false;

    std::string line;
    while (std::getline(file, line)) 
    {
        if (line.find(str) != std::string::npos) 
        {
            return true;
        }
    }
    return false;
}

std::vector<std::string> 
find_files_with_quoted_string(const char* msg_name) {
    std::vector<std::string> result;
    std::string str_include_path = std::string(getenv("CARMEN_HOME")) + std::string("/include/carmen");
    const char* include_path = str_include_path.c_str();

    DIR* dir = opendir(include_path);
    if (!dir) 
    {
        std::cerr << "Failed to open directory: " << include_path << std::endl;
        return result;
    }

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) 
    {
        // Skip "." and ".."
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
        {
            continue;
        }

        std::string full_path = std::string(include_path) + "/" + entry->d_name;

        struct stat path_stat;
        stat(full_path.c_str(), &path_stat);

        if (S_ISREG(path_stat.st_mode)) 
        {
            // Only files ending with .h
            if (full_path.size() >= 2 && full_path.substr(full_path.size() - 2) == ".h") 
            {
                if (file_contains_string(full_path, msg_name)) 
                {
                    result.push_back(full_path);
                }
            }
        }
    }

    closedir(dir);
    return result;
}

double
calc_freq(double timestamp)
{
    static double record_timestamp[RECORD_LIMIT];
    static int circular_idx = 0;

    double frequency = RECORD_LIMIT/(timestamp - record_timestamp[circular_idx]);
    record_timestamp[circular_idx] = timestamp;
    circular_idx = (circular_idx + 1) % RECORD_LIMIT;
    return frequency;
}

char* 
remove_newlines_and_double_spaces(const char* buffer) 
{
    size_t len = strlen(buffer);
    char* result = (char*) malloc(len + 1);
    if (!result) 
    {
        return NULL;
    }

    size_t j = 0;
    int last_was_space = 0;

    for (size_t i = 0; i < len; i++) 
    {
        char c = buffer[i];

        if (c == '\n') 
        {
            continue; 
        }

        if (c == ' ') 
        {
            if (!last_was_space) 
            {
                result[j++] = c;  
                last_was_space = 1;
            }
        } 
        else 
        {
            result[j++] = c;
            last_was_space = 0;
        }
    }

    result[j] = '\0';
    return result;
}

void 
split_msg_into_fields(std::vector<char*>& msg_fields, char* buffer)
{
    int num_scopes = 0;
    int last_pos = 0;
    int i = 0;
    char* content = remove_newlines_and_double_spaces(buffer);
    if (content != NULL)
    {
        for (i = 0; content[i] != '\0'; i++) 
        {
            if ((content[i] == '[') || (content[i] == '{') || (content[i] == '<')) 
            {
                num_scopes += 1;
            } 
            else if ((content[i] == ']') || (content[i] == '}') || (content[i] == '>')) 
            {
                num_scopes -= 1;
            } 
            else if (num_scopes == 1)
            {
                if (content[i] == ',') 
                {
                    int size = i - last_pos;
                    msg_fields.push_back((char*) malloc(size*sizeof(char)));
                    msg_fields[msg_fields.size()-1][size-1] = '\0';
                    strncpy(msg_fields[msg_fields.size()-1], content + last_pos + 1, size - 1);

                    last_pos = i;
                }
            }
        }
        int size = i - last_pos - 1;
        msg_fields.push_back((char*) malloc(size*sizeof(char)));
        msg_fields[msg_fields.size()-1][size-1] = '\0';
        strncpy(msg_fields[msg_fields.size()-1], content + last_pos + 1, size - 1);
        
        free(content);  
    }
}

void
get_message_as_string(MSG_INSTANCE msg_instance, void *call_data, char** buffer, const char** msg_name)
{
    *msg_name = IPC_msgInstanceName(msg_instance);
    FORMATTER_PTR format = IPC_msgInstanceFormatter(msg_instance);
    if (format == NULL) 
    {
        printf(" Not captured format \n");
        return;
    }

    size_t size_buffer = 0;
    // Saves the description of the message on a buffer
    FILE *memstream = open_memstream(buffer, &size_buffer);
    if (!memstream) 
    {
        perror("open_memstream");
        return;
    }
    IPC_printData(format, memstream, call_data);
    fclose(memstream);
}

void 
generic_handler(MSG_INSTANCE msg_instance, void *call_data, void *client_data) 
{
    double timestamp = carmen_get_time();
    char* buffer = NULL;
    const char* msg_name = NULL;
    get_message_as_string(msg_instance, call_data, &buffer, &msg_name);

    if (raw_message)
    {
        printf("\n-----------------------------------------\n");
        printf("Message: '%s'; Frequency: %lf\n", msg_name, calc_freq(timestamp));
        printf("%s\n", buffer);
        return;
    }
    

    // Separate message into fields (message contains double, string...)
    std::vector<char*> msg_fields;
    split_msg_into_fields(msg_fields, buffer);

    // Find the name of each field of the message (such as host and timestamp)
    std::string msg_name_quoted = "\"" + std::string(msg_name) + "\"";
    std::vector<std::string> files = find_files_with_quoted_string(msg_name_quoted.c_str());

    if (files.size() != 1)
    {
        for(int i = 0; i < msg_fields.size(); i++)
        {
            std::cout << msg_fields[i] << "\n";
        }
        return;
    }

    std::vector<std::string> struct_fields = get_struct_field_names(files[0], msg_name_quoted.c_str());

    printf("\n-----------------------------------------\n");
    printf("Message: '%s'; Frequency: %lf\n", msg_name, calc_freq(timestamp));
    if (msg_fields.size() == msg_fields.size()) 
    {
        for(int i = 0; i < struct_fields.size(); i++)
        {
            std::cout << struct_fields[i] << " : " << msg_fields[i] << "\n";
        }
    }
    else
    {
        for(int i = 0; i < msg_fields.size(); i++)
        {
            std::cout << msg_fields[i] << "\n";
        }
    }
    free(buffer); 
    for (auto msg_field : msg_fields) 
    {
        free(msg_field);
    }
}

int
read_parameters(int argc, char **argv, char** msg_name)
{
    if (argc < 2) 
    {
        return 1;
    }
    *msg_name = argv[1];
    
    if (argc > 2) 
    {
        if (strcmp("-raw_message", argv[2]) == 0)
        {
            raw_message = true;
        }
        else 
        {
            return 1;
        }
    }
    return 0;
}

int 
main(int argc, char **argv) 
{
    char* msg_name = NULL;
    if(read_parameters(argc, argv, &msg_name))
    {
        printf("Use any of the below: \n %s <msg name> \n %s <msg name> -raw_message \n", argv[0], argv[0]);
        return -1;
    }

    if (IPC_connect("print_ipc_message") != IPC_OK) 
    {
        IPC_perror("Error connecting");
        return -1;
    }

    if (IPC_subscribeData(msg_name, generic_handler, NULL) != IPC_OK) 
    {
        IPC_perror("Error subscribing");
        return -1;
    }

    printf("Subscribing on message '%s'...\n", msg_name);

    while (1) 
    {
        IPC_listen(1000); // 1000 ms
    }

    IPC_disconnect();
    return 0;
}
