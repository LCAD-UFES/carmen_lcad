#include <memory>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <unordered_map>
#include <sstream>
#include <regex>
#include <string>
#include <vector>
#include <algorithm>
#include <cctype>
#include <stack>
#include <cmath>  
#include <stdarg.h>
#include <stdio.h>

#include <carmen/carmen.h>
#include <carmen/ipc.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/fused_odometry_interface.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include <nav_msgs/msg/odometry.hpp>

size_t generate_struct_size_from_format(std::string msg_format);

// Mapeia as mensagens para os seus publisher/subscribers e formato

struct msg_register
{
    std::string format;
    bool subscribed;
    int size;
    std::vector<std::string> publishers;
    std::vector<std::string> subscribers;
    msg_register()
    {}
    msg_register(char* format) 
    : format(format)
    {
        subscribed = false;
        size = generate_struct_size_from_format(std::string(format));
    }
};

std::unordered_map<std::string, msg_register> messages_defined; 

using namespace std;

struct TypeInfo {
    size_t size;
    size_t alignment;
    string signature;
};

unordered_map<string, TypeInfo> type_info_hash = {
    {"string",   {8, 8, ""}},
    {"int",      {4, 4, ""}},
    {"double",   {8, 8, ""}},
    {"short",    {2, 2, ""}},
    {"ushort",   {2, 2, ""}},
    {"float",    {4, 4, ""}},
    {"char",     {1, 1, ""}},
    {"byte",     {1, 1, ""}},
};

unordered_map<string, TypeInfo> struct_cache;

vector<string> split_dims(const string& s) {
    vector<string> parts;
    stringstream ss(s);
    string part;
    while (getline(ss, part, ',')) {
        part.erase(remove_if(part.begin(), part.end(), ::isspace), part.end());
        if (!part.empty()) parts.push_back(part);
    }
    return parts;
}

vector<string> split_top_level(const string& s) {
    vector<string> types;
    stack<char> brackets;
    string token;
    
    for (char c : s) {
        if (c == '{' || c == '[' || c == '<') {
            brackets.push(c);
        } else if (c == '}' || c == ']' || c == '>') {
            if (!brackets.empty()) brackets.pop();
        }
        
        if (c == ',' && brackets.empty()) {
            if (!token.empty()) {
                types.push_back(token);
                token.clear();
            }
        } else {
            token += c;
        }
    }
    if (!token.empty()) types.push_back(token);
    return types;
}

TypeInfo parse_type(const string& type_str);

TypeInfo parse_struct(const string& struct_str) {
    if (struct_cache.count(struct_str)) {
        return struct_cache[struct_str];
    }

    vector<string> members = split_top_level(struct_str);
    size_t current_offset = 0;
    size_t max_alignment = 0;

    for (const string& member : members) {
        TypeInfo info = parse_type(member);
        size_t padding = (info.alignment - (current_offset % info.alignment)) % info.alignment;
        current_offset += padding + info.size;
        max_alignment = max(max_alignment, info.alignment);
    }

    size_t final_padding = (max_alignment - (current_offset % max_alignment)) % max_alignment;
    current_offset += final_padding;

    TypeInfo struct_info = {current_offset, max_alignment, struct_str};
    struct_cache[struct_str] = struct_info;
    return struct_info;
}

size_t find_valid_colon(const string& content) {
    stack<char> brackets;
    for (size_t i = 0; i < content.size(); ++i) {
        char c = content[i];
        if (c == '{' || c == '[' || c == '<') {
            brackets.push(c);
        } else if (c == '}' || c == ']' || c == '>') {
            if (!brackets.empty()) brackets.pop();
        } else if (c == ':' && brackets.empty()) {
            return i;
        }
    }
    return string::npos;
}

TypeInfo parse_type(const string& type_str) {
    if (type_str.empty()) return {0, 0, ""};

    // Ponteiros
    if (type_str.front() == '<' && type_str.back() == '>') {
        return {8, 8, ""};
    }

    // Arrays
    if (type_str.front() == '[' && type_str.back() == ']') {
        string content = type_str.substr(1, type_str.size()-2);
        size_t colon_pos = find_valid_colon(content);
        if (colon_pos == string::npos) {
            throw runtime_error("Formato de array inválido: " + type_str);
        }

        string base_type = content.substr(0, colon_pos);
        vector<string> dims = split_dims(content.substr(colon_pos + 1));
        
        TypeInfo base_info = parse_type(base_type);
        size_t total_elements = 1;
        for (const string& dim : dims) total_elements *= stoi(dim);

        return {base_info.size * total_elements, base_info.alignment, ""};
    }

    // Structs aninhadas
    if (type_str.front() == '{' && type_str.back() == '}') {
        string inner_content = type_str.substr(1, type_str.size()-2);
        return parse_struct(inner_content);
    }

    // Tipos primitivos
    auto it = type_info_hash.find(type_str);
    if (it != type_info_hash.end()) return it->second;

    throw runtime_error("Tipo desconhecido: " + type_str);
}

size_t generate_struct_size_from_format(std::string msg_format)
{
    // Remove apenas o primeiro '{' e o último '}' da struct principal
    size_t first_brace = msg_format.find('{');
    size_t last_brace = msg_format.rfind('}');
    if (first_brace != string::npos && last_brace != string::npos) {
        msg_format = msg_format.substr(first_brace + 1, last_brace - first_brace - 1);
    }
    msg_format.erase(remove(msg_format.begin(), msg_format.end(), ' '), msg_format.end());

    vector<string> types = split_top_level(msg_format);

	
    size_t current_offset =  0;
    size_t max_alignment = 0;

    for (const string& t : types) {
        TypeInfo info;
        try {
            info = parse_type(t);
        } catch (const exception& e) {
            cerr << "Erro: " << e.what() << endl;
            return NULL;
        }

        size_t padding = (info.alignment - (current_offset % info.alignment)) % info.alignment;
        current_offset += padding + info.size;
        max_alignment = max(max_alignment, info.alignment);
    }

    size_t final_padding = (max_alignment - (current_offset % max_alignment)) % max_alignment;
    current_offset += final_padding;

    return current_offset;
}

static
void record_log()
{
	std::string path = getenv("HOME") + std::string("/carmen_lcad/src/ipc_watcher/log.txt");
	std::ofstream arquivo(path, std::ios::app);

	if (arquivo.is_open()) 
    {
        arquivo << "------------------------------------------------\n";
        arquivo << "                   SUBSCRIÇÕES                  \n";
        for (const auto& msg : messages_defined)
        {
            for(const auto& subscriber : msg.second.subscribers)
                arquivo << "S_" << subscriber << " --> M_" << msg.first << '\n';
        }
        arquivo << "------------------------------------------------\n";

        arquivo << "\n------------------------------------------------\n";
        arquivo << "                   Publicações                  \n";
        for (const auto& msg : messages_defined)
        {
            for(const auto& publisher : msg.second.publishers)
                arquivo << "P_" << publisher << " --> M_" << msg.first << '\n';
        }
        arquivo << "------------------------------------------------\n";
        
        arquivo << "\n------------------------------------------------\n";
        arquivo << "                MENSAGENS DEFINIDAS              \n";
        for (const auto& msg : messages_defined)
        {
            arquivo << "M: " << msg.first << '\n';

        }
        arquivo << "------------------------------------------------\n";

    } else {
        std::cerr << "Erro ao abrir o arquivo!" << std::endl;
    }
}

static 
void new_subscribe_handler(carmen_ipc_watcher_subscribe_message *msg)
{
	// printf("New subscribe: %s to %s\n", msg->host, msg->msg_name);
    if(messages_defined.find(msg->msg_name) != messages_defined.end())
	{
        auto it = std::find(messages_defined[msg->msg_name].subscribers.begin(), messages_defined[msg->msg_name].subscribers.end(), std::string(msg->host));
        if(it == messages_defined[msg->msg_name].subscribers.end())
		{
            messages_defined[msg->msg_name].subscribers.emplace_back(msg->host);
        }
	}
}

static
void debug_generic_handler(void* msg, ...)
{
    // va_list args;
    // va_start(args, msg);
    // int size = va_arg(args, int);
    // char* msg_name = va_arg(args, char*);

    // char* ptr = (char*) msg;
    // ptr+= size - sizeof(char*);
    // char* host = *((char**) ptr);

    // auto it = std::find(messages_defined[msg_name].publishers.begin(), messages_defined[msg_name].publishers.end(), std::string(host));
    // if(it == messages_defined[msg_name].publishers.end())
    // {
    //     messages_defined[msg_name].publishers.emplace_back(host);
    // }
}

static 
void new_message_handler(carmen_ipc_watcher_new_message *msg)
{
	if(messages_defined.find(msg->msg_name) == messages_defined.end())
	{
		messages_defined[msg->msg_name] = msg_register(msg->formatString);
	}
}

void
define_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_IPC_WATCHER_NEW_MESSAGE_NAME,       IPC_VARIABLE_LENGTH, CARMEN_IPC_WATCHER_NEW_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_IPC_WATCHER_NEW_MESSAGE_NAME);

	err = IPC_defineMsg(CARMEN_IPC_WATCHER_SUBSCRIBE_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_IPC_WATCHER_SUBSCRIBE_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_IPC_WATCHER_NEW_MESSAGE_NAME);
}

static 
void subscribe_to_messages()
{

	carmen_subscribe_message((char*) CARMEN_IPC_WATCHER_NEW_MESSAGE_NAME,
			(char*) CARMEN_IPC_WATCHER_NEW_MESSAGE_FMT,
			NULL, sizeof(carmen_ipc_watcher_new_message),
			(carmen_handler_t) new_message_handler, CARMEN_SUBSCRIBE_ALL);

	carmen_subscribe_message((char*) CARMEN_IPC_WATCHER_SUBSCRIBE_MESSAGE_NAME,
			(char*) CARMEN_IPC_WATCHER_SUBSCRIBE_MESSAGE_FMT,
			NULL, sizeof(carmen_ipc_watcher_subscribe_message),
			(carmen_handler_t) new_subscribe_handler, CARMEN_SUBSCRIBE_ALL);
}

static 
void shutdown_module(int sig)
{
	(void) sig;

	static int done = 0;

	if (!done)
	{
		carmen_ipc_disconnect();
        record_log();
		printf("ipc_watcher disconnected from IPC.\n");
		fflush(stdout);
	}

	exit(0);
}

int 
main(int argc, char **argv)
{
    
	signal(SIGINT, shutdown_module);

	carmen_ipc_initialize(argc, argv);
	
    define_messages();
	subscribe_to_messages();

	carmen_ipc_dispatch();
    // while (IPC_listen(1000) != IPC_Error) 
    // {
    //     for(auto& msg : messages_defined)
    //     {
    //         if(!msg.second.subscribed)
    //         {
    //             carmen_subscribe_message((char*) msg.first.c_str(),
    //                                     (char*) msg.second.format.c_str(),
    //                                     NULL, msg.second.size,
    //                                     (carmen_handler_t) debug_generic_handler, CARMEN_SUBSCRIBE_ALL);
    //             msg.second.subscribed = true;
    //         }
    //     }
    // };
    return 0;
}

