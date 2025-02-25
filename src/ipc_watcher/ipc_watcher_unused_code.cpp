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


// static
// void debug_generic_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
// {
// 	// // msgRef é a descrição da msg, callData é a msg em si

// 	// IPC_RETURN_TYPE err = IPC_OK;
// 	// FORMATTER_PTR formatter;
// 	// const char* nome_msg = IPC_msgInstanceName(msgRef);
// 	// unsigned int message_total_size = IPC_dataLength(msgRef);

// 	// size_t message_struct_size = messages_defined[string(nome_msg)];
// 	// // printf("Message received: %s, tam: %lu\n", nome_msg, message_struct_size);

// 	// // coloca a msg em data
// 	// void* data = malloc(message_total_size);
// 	// formatter = IPC_msgInstanceFormatter(msgRef);
// 	// err = IPC_unmarshall(formatter, callData, &data);
// 	// carmen_test_ipc_exit(err, "Could not unmarshall message", nome_msg);

// 	// char* ptr = (char*) data; // casta para char* para manipular
// 	// ptr += message_struct_size - sizeof(char*); // vai até o final da struct e volta para ir no local do host
// 	// char* host = *((char**) ptr); // pega o host
	

// 	// // printf("%s publicou %s\n", host, nome_msg);

// 	// free(data);
// 	// // printf("Message processed\n");
// 	// // printf("-----------------------------------\n");
// }