#include <iostream>
#include <fstream>
#include <string>
#include <set>
#include <regex>
#include <thread>
#include <chrono>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <dirent.h>  
#include <queue>
#include <gtkmm.h>

#include <carmen/carmen.h>
#include "carmen/ipc.h"
#include "carmen/global.h"

#define MESSAGES_ON_SCAN_TICK 20
#define SCAN_PERIOD 3 // Tempo que ele espera até dar unsubscribe no scan anterior, e subscrever em novas msg

IPC_RETURN_TYPE FAKE_IPC_printData(FORMATTER_PTR formatter, FILE *stream, void *dataptr);

typedef struct {
    std::string message_name; // nome da mensagem para o IPC (não necessariamente é igual ao nome da struct!)
    std::vector<std::string> struct_fields;
    std::vector<std::string> msg_fields; // Constantemente atualizado
    std::vector<std::string> publishers;
    std::vector<std::string> subscribers;
    // Size information
    double average_message_size = 0;
    int num_of_records_msg_size = 0;
    // Frequency information
    double frequency = 0;
    double start_record_timestamp = 0;
    int num_of_records = 0;
} Msg_info;

static std::unordered_map<std::string, Msg_info> messages_descriptions;
static std::unordered_set<std::string> potential_messages;
static std::unordered_map<std::string, bool> unsubscribe_queue;
static std::queue<std::string> messages_to_scan_queue;
static std::queue<std::string> messages_subscribed_to_scan_queue;

bool scan_messages_state = false;                  // Ativado quando usuário clica em "scan_messages"

// Modelo de colunas da tabela
class MsgColumns : public Gtk::TreeModel::ColumnRecord {
public:
    MsgColumns() {
        add(col_name);
        add(col_publishers);
        add(col_frequency);
        add(col_msg_size);
        add(col_fmt);
    }

    Gtk::TreeModelColumn<Glib::ustring> col_name;
    Gtk::TreeModelColumn<Glib::ustring> col_publishers;
    Gtk::TreeModelColumn<Glib::ustring> col_frequency;
    Gtk::TreeModelColumn<Glib::ustring> col_msg_size;
    Gtk::TreeModelColumn<Glib::ustring> col_fmt;
};

// Janela principal
class MsgWindow : public Gtk::Window {
public:
    void reorder_messages_handler();
    void update_window_handler();
    void update_box_handler(std::string& msg_name);
    void on_row_selected_handler();
    void scan_messages_handler();
    void put_text_on_box(std::string& text);
    MsgWindow()
    : label_msg_page("Mensagens registradas"),
      hbox_main_(false, 5) // false = horizontal
    {
        set_title("Monitor de Mensagens IPC");
        set_default_size(1200, 400);

        // Adiciona notebook na esquerda
        hbox_main_.pack_start(notebook_, true, true, 5);

        // Cria modelo
        list_store_ = Gtk::ListStore::create(msg_page_columns);
        tree_view_.set_model(list_store_);

        // Colunas
        tree_view_.append_column("Mensagem", msg_page_columns.col_name);
        tree_view_.append_column("Publishers", msg_page_columns.col_publishers);
        tree_view_.append_column("Frequência (Hz)", msg_page_columns.col_frequency);
        tree_view_.append_column("Tamanho médio", msg_page_columns.col_msg_size);
        tree_view_.append_column("Formatos", msg_page_columns.col_fmt);

        // Cria layout vertical da página
        Gtk::VBox* vbox_msg_page = Gtk::manage(new Gtk::VBox(false, 5));

        // Cria o botão "Scan messages"
        Gtk::Button* scan_button = Gtk::manage(new Gtk::Button("Scanear mensagens"));
        scan_button->signal_clicked().connect(
            sigc::mem_fun(*this, &MsgWindow::scan_messages_handler)
        );

        // Cria o botão "Reordenar"
        Gtk::Button* reorder_button = Gtk::manage(new Gtk::Button("Reordenar"));
        reorder_button->signal_clicked().connect(
            sigc::mem_fun(*this, &MsgWindow::reorder_messages_handler)
        );

        // Ordena os botões
        Gtk::HBox* button_box = Gtk::manage(new Gtk::HBox(false, 5));
        button_box->pack_start(*scan_button, Gtk::PACK_SHRINK);
        button_box->pack_start(*reorder_button, Gtk::PACK_SHRINK);

        // Cria o ScrolledWindow para conter a TreeView
        Gtk::ScrolledWindow* scroll_window = Gtk::manage(new Gtk::ScrolledWindow());
        scroll_window->set_policy(Gtk::POLICY_AUTOMATIC, Gtk::POLICY_AUTOMATIC);
        scroll_window->add(tree_view_);

        // Adiciona o botão e a área de scroll ao layout
        vbox_msg_page->pack_start(*button_box, Gtk::PACK_SHRINK);
        vbox_msg_page->pack_start(*scroll_window, Gtk::PACK_EXPAND_WIDGET);

        // Adiciona o VBox completo como a página do notebook
        notebook_.append_page(*vbox_msg_page, label_msg_page);

        // TextView à direita
        text_view_buffer_ = Gtk::TextBuffer::create();
        text_view_.set_buffer(text_view_buffer_);
        text_view_.set_editable(false);
        text_view_.set_wrap_mode(Gtk::WRAP_WORD);

        text_scrolled_window_.add(text_view_);
        text_scrolled_window_.set_policy(Gtk::POLICY_AUTOMATIC, Gtk::POLICY_AUTOMATIC);
        text_scrolled_window_.set_size_request(800, -1);

        hbox_main_.pack_start(text_scrolled_window_, false, false, 5);
        add(hbox_main_);

        // Conecta seleção
        Glib::RefPtr<Gtk::TreeSelection> selection = tree_view_.get_selection();
        selection->signal_changed().connect(sigc::mem_fun(*this, &MsgWindow::on_row_selected_handler));


        show_all();
    }

    bool messages_dirty = true;                        // indica se messages_descriptions mudou

private:
    /* Variaveis da aba que exibe as msg recebidas (fica a direita) */
    Gtk::ScrolledWindow text_scrolled_window_;

    /* Notebook que guarda as paginas (fica na esquerda) */
    Gtk::Notebook notebook_;

    /* Variaveis da pagina de msg (a que tem nome, frequencia, publishers e fica dentro do notebook na esquerda) */
    Gtk::Label label_msg_page;    // Nome da pagina
    Gtk::ScrolledWindow msg_page_; // Janela que contem a descricao das mensagens
    MsgColumns msg_page_columns;  // Guarda as colunas
    std::unordered_map<std::string, Gtk::TreeModel::RowReference> msg_page_rows; // Guarda as linhas

    /* Outros */
    Gtk::HBox hbox_main_;
    Gtk::TextView text_view_;                           // área de exibição de mensagem
    Glib::RefPtr<Gtk::TextBuffer> text_view_buffer_;    // buffer para o TextView
    Gtk::TreeView tree_view_;
    Glib::RefPtr<Gtk::ListStore> list_store_;

    std::string last_subscribed_message = ""; // Necessario para dar unsubscribe antes de subscrever em uma nova msg
    std::vector<std::string> cached_sorted_names;      // nomes já ordenados
};

std::unique_ptr<MsgWindow> g_window;

void
MsgWindow::put_text_on_box(std::string& text)
{
    text_view_buffer_->set_text(text);
}

static std::string extract_publisher_from_host(const std::string& field)
{
    std::string s = field;

    // Remove aspas
    s.erase(std::remove(s.begin(), s.end(), '"'), s.end());

    // Remove tudo depois do '@'
    size_t at = s.find('@');
    if (at != std::string::npos)
        s = s.substr(0, at);

    // Remove TODOS os espaços e tabs
    s.erase(
        std::remove_if(
            s.begin(),
            s.end(),
            [](unsigned char c) {
                return std::isspace(c);
            }
        ),
        s.end()
    );

    return s;
}

// Para arredondar sem adicionar lib
auto format_fixed = [](double value, int decimals)
{
    double factor = 1.0;
    for (int i = 0; i < decimals; ++i)
        factor *= 10.0;

    // arredondamento correto
    value = std::round(value * factor) / factor;

    std::string s = std::to_string(value);

    auto pos = s.find('.');
    if (pos == std::string::npos)
        return s;

    return s.substr(0, pos + 1 + decimals);
};


std::string split_by_braces(std::string input)
{
    std::vector<std::string> result;

    // 1) Checa se tem '<'
    if (input.find('<') == std::string::npos)
        return input; // ou retorne {input}, se preferir

    // 2) Remove '<' e '>'
    input.erase(
        std::remove(input.begin(), input.end(), '<'),
        input.end()
    );
    input.erase(
        std::remove(input.begin(), input.end(), '>'),
        input.end()
    );

    std::string current;
    bool inside_braces = false;
    bool found_info = false;
    // 3) Parse
    for (size_t i = 0; i < input.size(); ++i)
    {
        char c = input[i];

        if (c == '{')
        {
            if (!current.empty())
            {
                result.push_back(current);
                current.clear();
            }
            current += c;
            inside_braces = true;
            found_info = false;
        }
        else if (c == '}')
        {
            current += c;
            result.push_back(current);
            current.clear();
            inside_braces = false;
        }
        else
        {
            if(inside_braces)
            {
                current += c;
                continue;
            }
            if((c != ' ') && (c != '\n') && (c != ','))
                found_info = true;
            if(found_info)
                current += c;
        }
    }

    // 4) Sobrou algo (inclui '{' sem '}')
    if (!current.empty())
        result.push_back(current);

    std::string final_result = "\n";
    bool first_element = true;

    // for (auto& element : result)
    for (size_t i = 0; i < result.size(); i++)
    {
        if(first_element)
        {
            first_element = false;
            if (i == (result.size() - 1))
                final_result += "--  <" + result[i] + ">\n";
            else 
                final_result += "--  <" + result[i] + "\n";
        }
        else if (i == (result.size() - 1))
        {
            final_result += "--  " + result[i] + ">\n";
        }
        else 
        {
            final_result += "--  " + result[i] + "\n";
        }
    }

    return final_result;
}



/**
 * @brief Removes newline characters and extra spaces from a string buffer.
 *
 * This function processes a given string buffer, removing all newline ('\n')
 * characters and collapsing multiple consecutive spaces into a single one.
 * The result is a clean, single-line string.
 *
 * @param buffer The input string to process.
 * @return A dynamically allocated string with newlines and extra spaces removed.
 *         The caller is responsible for freeing the returned string.
 */
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

/**
 * @brief Splits a serialized IPC message into its individual fields.
 *
 * This function parses a string representation of a message (in the format
 * "{field1, field2, ...}") and extracts each field into a vector of strings.
 * It ignores nested structures and properly handles brackets and braces.
 *
 * @param msg_fields Reference to a vector that will receive the parsed fields.
 * @param buffer The message string to parse.
 */
void 
split_msg_into_fields(std::vector<std::string>& msg_fields, char* buffer)
{
    int num_scopes = 0;
    int last_pos = 0;
    int i = 0;
    char* content = remove_newlines_and_double_spaces(buffer);
    // printf("Mensagem: %s\n", content);
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
                    char* buffer = (char*) malloc(size*sizeof(char));
                    buffer[size-1] = '\0';
                    strncpy(buffer, content + last_pos + 1, size - 1);
                    msg_fields.push_back(buffer);
                    free(buffer);

                    last_pos = i;
                }
            }
        }
        int size = i - last_pos - 1;
        char* buffer = (char*) malloc(size*sizeof(char));
        buffer[size-1] = '\0';
        strncpy(buffer, content + last_pos + 1, size - 1);
        msg_fields.push_back(buffer);
        free(buffer);
        
        free(content);  
    }
}

/**
 * @brief Converts an IPC message instance into a human-readable string.
 *
 * Retrieves the message formatter from a given message instance and uses it
 * to serialize the message into a string buffer.
 *
 * @param msg_instance The IPC message instance.
 * @param call_data Pointer to the message data.
 * @param buffer Pointer to a string buffer that will receive the serialized message.
 */
void
get_message_as_string(MSG_INSTANCE msg_instance, void *call_data, char** buffer)
{
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
    FAKE_IPC_printData(format, memstream, call_data);
    fclose(memstream);
}

/**
 * @brief Checks if a given file contains a specific substring.
 *
 * Opens a text file and scans its content line by line for a given string.
 * Used for finding which header file contais the struct definition of the message
 *
 * @param file_path Path to the file to be searched.
 * @param str The string to look for.
 * @return True if the string is found in the file, false otherwise.
 */
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

/**
 * @brief Searches header files in the CARMEN include directory for a specific message name.
 *
 * This function scans all `.h` files under `$CARMEN_HOME/include/carmen`
 * and returns a list of files that contain the given message name.
 *
 * @param msg_name The message name to search for.
 * @return A vector containing the paths of files where the message name was found.
 */
std::vector<std::string> 
find_files_with_string(const char* msg_name) {
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

/**
 * @brief Extracts field names from a struct definition corresponding to a given message.
 *
 * Searches a header file for the typedef or struct that defines a message,
 * then parses the struct fields, returning them as a list of strings.
 *
 * @param file The input file stream for the header file.
 * @param msg_name The message name (quoted) to search for.
 * @return A vector containing the field names and types of the struct.
 */
std::vector<std::string> 
get_struct_field_names(std::ifstream& file, std::string msg_name) 
{
    std::vector<std::string> fields;

    if (!file.is_open()) 
    {
        std::cerr << "File not open! " << std::endl;
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

    // Go backwards to find struct
    bool found_struct_start = false;
    size_t struct_start_index = 0;
    for (int i = msg_line_index; i >= 0; --i) 
    {
        if (lines[i].find("typedef") != std::string::npos) 
        {
            found_struct_start = true;
            struct_start_index = i;
            break;
        }
    }
    if (!found_struct_start) 
    {
        std::cerr << "struct not found for message: " << msg_name << std::endl;
        return fields;
    }

    // Normalmente, encontra a declaracao da struct. Contudo, podem ocorrer casos como o abaixo:
    // typedef carmen_mapper_map_message carmen_map_server_feature_map_message;
    // Por esses casos, checamos se realmente tem a palavra struct nessa linha ou na próxima
    // Caso tenha, signfica que achou a struct, e ignora, mas se nao achou, tenta achar o formato da msg real,
    // nesse caso seriam os campos de "carmen_mapper_map_message"
    if((lines[struct_start_index].find("struct") == std::string::npos) && 
       (lines.size() > (struct_start_index+1)) && (lines[struct_start_index+1].find("struct") == std::string::npos))
    {
        // Caso seja o formato acima, a primeira palavra sera typedef, e a ultima a propria msg, testa:
        std::regex pattern(R"(\b(\w+)\s+(\w+)\s+(\w+);)");
        std::smatch match;
        if (std::regex_match(lines[struct_start_index], match, pattern)) 
        {
            std::string first  = match[1].str();
            std::string second = match[2].str() + ';'; // Nome "Real" da mensagem
            // std::cout << "msg: " << first << ", " << second << '\n';

            if (first == "typedef") 
            {
                std::vector<std::string> files_found_with_real_name = find_files_with_string(second.c_str());
                // se encontrou exatamente um arquivo, extrai campos
                if (files_found_with_real_name.size() == 1) 
                {
                    std::ifstream file_with_real_name(files_found_with_real_name[0]);
                    std::cout << "found struct for: " << second << '\n';
                    return get_struct_field_names(file_with_real_name, second);
                }
            } 
        }
        std::cout << "not found struct real description for: " << msg_name << '\n';
        return fields;
    }


    bool in_block_comment = false;
    // Parse struct body
    bool struct_started = false;
    for (int i = struct_start_index; i < (int)lines.size(); ++i) 
    {
        std::string l = lines[i];

        // Remove comments (single-line and multi-line)
        std::string cleaned;
        for (size_t j = 0; j < l.size(); ) 
        {
            // Inside block comment
            if (in_block_comment) 
            {
                size_t end = l.find("*/", j);
                if (end == std::string::npos) 
                {
                    // Entire line is inside comment
                    j = l.size();
                } 
                else 
                {
                    in_block_comment = false;
                    j = end + 2;
                }
            }
            // Start of block comment
            else if (l.compare(j, 2, "/*") == 0) 
            {
                in_block_comment = true;
                j += 2;
            }
            // Single-line comment
            else if (l.compare(j, 2, "//") == 0) 
            {
                break; // ignore rest of line
            }
            else 
            {
                cleaned += l[j++];
            }
        }

        l = cleaned;


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


///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


/**
 * @brief Updates the GTK message list window with the current message data.
 *
 * This handler refreshes the GUI by listing all known IPC messages,
 * updating their publishers, frequency, and format fields.
 * Old entries no longer present in the system are removed.
 */
void 
MsgWindow::update_window_handler() 
{
    auto model = list_store_;

    // Seta visitados
    std::unordered_set<std::string> visited;

    // Atualiza cache se necessário
    if (messages_dirty) {
        model->clear();
        msg_page_rows.clear();
        cached_sorted_names.clear();
        cached_sorted_names.reserve(messages_descriptions.size());

        for (const auto& pair : messages_descriptions)
            cached_sorted_names.push_back(pair.first);

        std::sort(
            cached_sorted_names.begin(),
            cached_sorted_names.end(),
            [&](const std::string& a, const std::string& b)
            {
                const auto& msg_a = messages_descriptions.at(a);
                const auto& msg_b = messages_descriptions.at(b);

                bool a_has_pub = !msg_a.publishers.empty();
                bool b_has_pub = !msg_b.publishers.empty();

                // 1️⃣ Quem tem publisher vem primeiro
                if (a_has_pub != b_has_pub)
                    return a_has_pub > b_has_pub;

                // 2️⃣ Ambos têm publisher → usa o MENOR publisher (ordem alfabética)
                if (a_has_pub && b_has_pub)
                {
                    auto min_pub_a = *std::min_element(
                        msg_a.publishers.begin(),
                        msg_a.publishers.end()
                    );

                    auto min_pub_b = *std::min_element(
                        msg_b.publishers.begin(),
                        msg_b.publishers.end()
                    );

                    if (min_pub_a != min_pub_b)
                        return min_pub_a < min_pub_b;
                }

                // 3️⃣ Desempate final: nome da mensagem
                return msg_a.message_name < msg_b.message_name;
            }
        );

        messages_dirty = false;
    }

    for (const auto& name : cached_sorted_names) {
        const auto& msg = messages_descriptions.at(name);
        visited.insert(msg.message_name);

        Gtk::TreeModel::iterator iter;
        auto it_row = msg_page_rows.find(msg.message_name);

        if (it_row == msg_page_rows.end()) {
            iter = model->append();
            msg_page_rows[msg.message_name] = Gtk::TreeModel::RowReference(model, model->get_path(iter));
        } else {
            iter = model->get_iter(it_row->second.get_path());
            if (!iter) continue;
        }

        Gtk::TreeModel::Row row = *iter;

        // Atualiza apenas se mudou, evitando redraws desnecessários
        if (row[msg_page_columns.col_name] != msg.message_name)
            row[msg_page_columns.col_name] = msg.message_name;

        // Concatenação rápida com std::string
        std::string pub_str;
        if (!msg.publishers.empty()) {
            pub_str = msg.publishers[0];
            for (size_t i = 1; i < msg.publishers.size(); ++i) {
                pub_str += ", " + msg.publishers[i];
            }
        }
        if (row[msg_page_columns.col_publishers] != pub_str)
            row[msg_page_columns.col_publishers] = pub_str;

        std::string fmt_str;
        if (!msg.struct_fields.empty()) {
            fmt_str = msg.struct_fields[0];
            for (size_t i = 1; i < msg.struct_fields.size(); ++i)
                fmt_str += ", " + msg.struct_fields[i];
        }
        if (row[msg_page_columns.col_fmt] != fmt_str)
            row[msg_page_columns.col_fmt] = fmt_str;

        std::string freq_str = format_fixed(msg.frequency, 2);
        if (row[msg_page_columns.col_frequency] != freq_str)
            row[msg_page_columns.col_frequency] = freq_str;
        
        std::string msg_size_str = format_fixed(msg.average_message_size, 1);
        if (row[msg_page_columns.col_msg_size] != msg_size_str)
            row[msg_page_columns.col_msg_size] = msg_size_str;
    }

    // Remove linhas obsoletas
    for (auto it = msg_page_rows.begin(); it != msg_page_rows.end(); ) {
        if (visited.find(it->first) == visited.end()) {
            if (auto iter = model->get_iter(it->second.get_path()))
                model->erase(iter);
            it = msg_page_rows.erase(it);
        } else {
            ++it;
        }
    }
}

/**
 * @brief Updates the right-side message information box in the GUI.
 *
 * Displays detailed information about a selected message, including its
 * publishers, frequency, and data fields.
 *
 * @param msg_name The name of the message to display.
 */
void 
MsgWindow::update_box_handler(std::string& msg_name)
{
    auto it = messages_descriptions.find(msg_name);
    if (it != messages_descriptions.end()) 
    {
        const Msg_info& msg = it->second;

        // Gestão de cores (Ainda nao foi feita)

        // Preenche o TextView com o conteúdo da mensagem
        std::ostringstream box_buffer;
        box_buffer << "Mensagem: " << msg.message_name << "\n";
        box_buffer << "Publishers: ";
        for (size_t i = 0; i < msg.publishers.size(); ++i) 
        {
            box_buffer << msg.publishers[i];
            if (i < msg.publishers.size() - 1) box_buffer << ", ";
        }
        box_buffer << "\nFrequência: " << msg.frequency << " Hz\n";
        box_buffer << "\n-----------------------------------------\n";

        // Se tem o mesmo tamanho, supoe-se que a msg tenha sido separada de forma correta, e a struct esteja correta
        if (msg.msg_fields.size() == msg.struct_fields.size()) 
        {        
            for(size_t i = 0; i < msg.struct_fields.size(); i++)
            {
                box_buffer << msg.struct_fields[i] << " : " << split_by_braces(msg.msg_fields[i]) << "\n";
            }
        }
        else
        {
            for(size_t i = 0; i < msg.msg_fields.size(); i++)
            {
                box_buffer << msg.msg_fields[i] << "\n";
            }
        }

        // Limpa buffer e coloca texto
        text_view_buffer_->set_text(box_buffer.str());
    }
}

void
MsgWindow::reorder_messages_handler()
{
    // Força reordenação
    messages_dirty = true;

    // Atualiza imediatamente a tabela
    update_window_handler();
}


/**
 * @brief Callback executed when a subscribed IPC message is received.
 *
 * Updates message statistics (frequency, publishers, and data fields),
 * then updates the GUI window to display the latest received message content.
 *
 * @param msg_instance The IPC message instance received.
 * @param call_data Pointer to the message data.
 * @param client_data Unused parameter.
 */
void
print_message_on_window_and_update_info_handler(MSG_INSTANCE msg_instance, void *call_data, void *client_data)
{
    (void) client_data;
    double timestamp = carmen_get_time();
    const char* msg_name = IPC_msgInstanceName(msg_instance);
    Msg_info& msg_info = messages_descriptions[msg_name];

    // Calcula frequencia da mensagem
    if(msg_info.start_record_timestamp > 0)
        msg_info.frequency = msg_info.num_of_records / (timestamp - msg_info.start_record_timestamp);
    else
        msg_info.start_record_timestamp = timestamp;
    msg_info.num_of_records += 1;

    // Calcula tamanho medio da msg
    unsigned int msg_size = IPC_dataLength(msg_instance);
    msg_info.num_of_records_msg_size += 1;
    // Ao inves de manter o valor da soma de todos os anteriores, que poderia acumular para um número MUITO grande,
    // Faz um cálculo com o valor direto da média anterior
    msg_info.average_message_size = msg_info.average_message_size + (msg_size - msg_info.average_message_size)/msg_info.num_of_records_msg_size;

    // Captura a messagem como uma string
    char* buffer = NULL;
    get_message_as_string(msg_instance, call_data, &buffer);

    // Limpa o que tinha antes, e tenta separar a msg em campos
    std::vector<std::string>& msg_fields = msg_info.msg_fields;
    msg_fields.clear();
    split_msg_into_fields(msg_fields, buffer);
    
    // If publishers is not yet added to list of publisher, add it
    for(size_t i = 0; i < msg_fields.size(); i++)
    {
        if (msg_info.struct_fields.size() != msg_fields.size())
            continue;
        if(msg_info.struct_fields[i] != "char *host" && msg_info.struct_fields[i] != "char *hostname")
            continue;

        // Parse the publisher string (transform "localize_ackerman@lume-Nitro..." into localize_ackerman)
        size_t pos = msg_fields[i].find('@');
        std::string msg_publisher;
        if (pos != std::string::npos) 
            msg_publisher = extract_publisher_from_host(msg_fields[i]);
        else 
            msg_publisher = msg_fields[i]; // Se não tiver '@', mantém o texto original

        bool publisher_already_registered = false;
        for(auto& publisher : msg_info.publishers)
        {
            if(publisher == msg_publisher)
            {
                publisher_already_registered = true;
                break;
            }   
        }
        if(!publisher_already_registered)
            msg_info.publishers.emplace_back(msg_publisher);
    }

    // Atualiza a janela (a parte lateral direita)
    g_window->update_box_handler(msg_info.message_name);
}


/**
 * @brief Handles row selection in the GTK message list.
 *
 * When a message is selected in the GUI, this function subscribes to it
 * using IPC and unsubscribes from any previously selected message.
 * It also resets message frequency counters.
 */
void 
MsgWindow::on_row_selected_handler() 
{
    if(scan_messages_state) return;
    auto selection = tree_view_.get_selection();
    if (!selection) return;
    auto iter = selection->get_selected();
    if (!iter) return;

    Glib::ustring msg_name_ustr = (*iter)[msg_page_columns.col_name];
    std::string msg_name = msg_name_ustr.raw();
    update_box_handler(msg_name); // Atualiza com os dados que tiver 

    // Da unsubscribe caso ja tenha dado subscribe  
    if(last_subscribed_message != "")
    {
        std::cout << "unsubscribed window_handler to: " << last_subscribed_message << '\n';
        IPC_unsubscribe(last_subscribed_message.c_str(), print_message_on_window_and_update_info_handler);
    }
    last_subscribed_message = msg_name;

    // Subscreve na nova msg, e reseta alguma variaveis
    std::cout << "subscribed window_handler to: " << msg_name << '\n';
    IPC_subscribeData(msg_name.c_str(), print_message_on_window_and_update_info_handler, NULL);
    messages_descriptions[msg_name].frequency = 0;
    messages_descriptions[msg_name].start_record_timestamp = -1;
    messages_descriptions[msg_name].num_of_records = 0;
    messages_descriptions[msg_name].average_message_size = 0;
    messages_descriptions[msg_name].num_of_records_msg_size = 0;
}

void
scan_update_info_handler(MSG_INSTANCE msg_instance, void *call_data, void *client_data)
{
    (void) client_data;
    double timestamp = carmen_get_time();
    const char* msg_name = IPC_msgInstanceName(msg_instance);
    Msg_info& msg_info = messages_descriptions[msg_name];

    // Calcula frequencia da mensagem
    if(msg_info.start_record_timestamp > 0)
        msg_info.frequency = msg_info.num_of_records / (timestamp - msg_info.start_record_timestamp);
    else
        msg_info.start_record_timestamp = timestamp;
    msg_info.num_of_records += 1;

    // Calcula tamanho medio da msg
    unsigned int msg_size = IPC_dataLength(msg_instance);
    msg_info.num_of_records_msg_size += 1;
    // Ao inves de manter o valor da soma de todos os anteriores, que poderia acumular para um número MUITO grande,
    // Faz um cálculo com o valor direto da média anterior
    msg_info.average_message_size = msg_info.average_message_size + (msg_size - msg_info.average_message_size)/msg_info.num_of_records_msg_size;

    // Captura a messagem como uma string
    char* buffer = NULL;
    get_message_as_string(msg_instance, call_data, &buffer);

    // Limpa o que tinha antes, e tenta separar a msg em campos
    std::vector<std::string>& msg_fields = msg_info.msg_fields;
    msg_fields.clear();
    split_msg_into_fields(msg_fields, buffer);
    
    // If publishers is not yet added to list of publisher, add it
    for(size_t i = 0; i < msg_fields.size(); i++)
    {
        if (msg_info.struct_fields.size() != msg_fields.size())
            continue;
        if(msg_info.struct_fields[i] != "char *host" && msg_info.struct_fields[i] != "char *hostname")
            continue;

        // Parse the publisher string (transform "localize_ackerman@lume-Nitro..." into localize_ackerman)
        size_t pos = msg_fields[i].find('@');
        std::string msg_publisher;
        if (pos != std::string::npos) 
            msg_publisher = extract_publisher_from_host(msg_fields[i]); // Pega tudo antes do '@' 
        else 
            msg_publisher = msg_fields[i]; // Se não tiver '@', mantém o texto original

        bool publisher_already_registered = false;
        for(auto& publisher : msg_info.publishers)
        {
            if(publisher == msg_publisher)
            {
                publisher_already_registered = true;
                break;
            }   
        }
        if(!publisher_already_registered)
            msg_info.publishers.emplace_back(msg_publisher);
    }
}

/**
 * @brief Starts the scanning process
 *
 * This handler turns on the scan_messages_state, which starts to scan every messages of the system slowly,
 * registering publishers, frequency and size.
 */
void 
MsgWindow::scan_messages_handler()
{
    // Nao repete até terminar
    if(scan_messages_state) return;

    scan_messages_state = true;
    // Da unsubscribe caso ja tenha dado subscribe  
    if(last_subscribed_message != "")
        IPC_unsubscribe(last_subscribed_message.c_str(), print_message_on_window_and_update_info_handler);
    last_subscribed_message = "";

    // Empilha todas as mensagens para subscrever futuramente
    messages_to_scan_queue = std::queue<std::string>(); // Substitui por fila limpa (caso tenha alguma msg sobrando)
    for(const auto& pair : messages_descriptions)
        messages_to_scan_queue.push(pair.first);
}


void 
scan_messages_update_handler()
{
    // Espera ate apertarem o botao e mudar de estado
    if(!scan_messages_state) return;

    std::ostringstream scan_box_buffer;
    // Da unsubscribe na leva anterior
    scan_box_buffer << "----------------------------------------\n";
    scan_box_buffer << "unsubscribing to: ";
    while(!messages_subscribed_to_scan_queue.empty())
    {
        std::string msg_name = messages_subscribed_to_scan_queue.front();
        messages_subscribed_to_scan_queue.pop();
        scan_box_buffer << "\n- " << msg_name;
        IPC_unsubscribe(msg_name.c_str(), scan_update_info_handler);
    }

    // Se acabou o scan, reseta o estado
    if(messages_to_scan_queue.empty())
    {
        scan_messages_state = false;
        g_window->messages_dirty = true;
        return;
    }

    // Da subscribe em uma nova leva
    scan_box_buffer << "\n\nsubscribing to: ";
    for(int i = 0; i < MESSAGES_ON_SCAN_TICK; i++)
    {
        if(messages_to_scan_queue.empty()) break;

        std::string msg_name = messages_to_scan_queue.front();
        messages_to_scan_queue.pop();
        messages_subscribed_to_scan_queue.push(msg_name);

        scan_box_buffer << "\n- " << msg_name;
        IPC_subscribeData(msg_name.c_str(), scan_update_info_handler, NULL);
        messages_descriptions[msg_name].frequency = 0;
        messages_descriptions[msg_name].start_record_timestamp = -1;
        messages_descriptions[msg_name].num_of_records = 0;
        messages_descriptions[msg_name].average_message_size = 0;
        messages_descriptions[msg_name].num_of_records_msg_size = 0;
    }   

    scan_box_buffer << "\n\nListening for " << SCAN_PERIOD << " seconds...\n";
    std::string text = scan_box_buffer.str();
    g_window->put_text_on_box(text);
}

/**
 * @brief Periodically updates the GTK window and processes GUI events.
 *
 * This function is registered as a periodic timer handler that refreshes
 * the message table and processes pending GTK events to keep the interface responsive.
 */
void
update_window_and_read_events()
{
    // Atualiza GUI
    g_window->update_window_handler();

    // Processa eventos da janela (para não travar a interface)
    while (Gtk::Main::events_pending())
        Gtk::Main::iteration();
}


///////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


/**
 * @brief Searches for potential IPC messages defined in header files.
 *
 * Scans all files under `$CARMEN_HOME/include/carmen` looking for lines
 * containing the `_NAME` pattern, which indicate a possible IPC message definition.
 * All found message names are added to the `potential_messages` set.
 */
void
find_potential_messages()
{
    // Procura em cada linha de todos os arquivos em carmen/include/carmen (onde ficam os headers) pela string "_NAME ", 
    // se encontra considera que é um nome de msg e adiciona nas mensagens potenciais
    std::string str_include_path = std::string(getenv("CARMEN_HOME")) + std::string("/include/carmen");
    DIR* dir = opendir(str_include_path.c_str());
    if (!dir) {
        std::cerr << "Erro ao abrir diretório: " << str_include_path << std::endl;
        return;
    }

    struct dirent* entry;
    std::regex name_line_regex(R"(\b[A-Z0-9_]+_NAME\s*\"[^\"]+\")");
    std::regex quoted_regex("\"([^\"]+)\""); // captura o conteúdo entre aspas

    while ((entry = readdir(dir)) != nullptr) {
        std::string filename = entry->d_name;

        // Ignorar "." e ".."
        if (filename == "." || filename == "..")
            continue;

        std::string full_path = str_include_path + "/" + filename;

        // Abre o arquivo
        std::ifstream file(full_path.c_str());
        if (!file.is_open()) {
            std::cerr << "Não foi possível abrir: " << full_path << std::endl;
            continue;
        }

        std::string line;
        while (std::getline(file, line)) {
            if (std::regex_search(line, name_line_regex)) {
                std::smatch match;
                if (std::regex_search(line, match, quoted_regex)) {
                    potential_messages.insert(match[1]);
                }
            }
        }

        file.close();
    }
    closedir(dir);
}

/**
 * @brief Identifies which potential IPC messages are currently defined.
 *
 * Iterates through all potential messages and verifies their definition using `IPC_isMsgDefined`.
 * For each valid message, it retrieves the corresponding struct fields and stores them
 * in the `messages_descriptions` map.
 */
void
identify_active_messages()
{
    // iterador manual para poder apagar enquanto itera de forma segura
    for (auto it = potential_messages.begin(); it != potential_messages.end(); ) {
        const std::string message_name = *it;

        // se a mensagem não estiver definida, apenas avance
        if (!IPC_isMsgDefined(message_name.c_str())) {
            ++it;
            continue;
        }

        auto &msg_info = messages_descriptions[message_name];
        msg_info.message_name = message_name;

        // procura arquivos que contenham o nome entre aspas
        std::string msg_name_quoted = "\"" + message_name + "\"";
        std::vector<std::string> files = find_files_with_string(msg_name_quoted.c_str());

        // se encontrou exatamente um arquivo, extrai campos
        if (files.size() > 0) 
        {
            std::ifstream file(files[0]);
            msg_info.struct_fields = get_struct_field_names(file, msg_name_quoted);
        }
        else
        {
            std::cout << "WARNING: found " << files.size() << " files with struct for: " << msg_name_quoted << '\n';
        }

        // remove do conjunto de potenciais mensagens — erase(it) retorna o próximo iterador
        it = potential_messages.erase(it);
        g_window->messages_dirty = true;
    }
}

void
shutdown_module(int signo)
{
    if (signo == SIGINT)
    {
        carmen_ipc_disconnect();
    
        printf("list_ipc_message: disconnected.\n");    
        exit(0);
    } 
}

int 
main(int argc, char **argv) 
{
    gtk_disable_setlocale(); // Tem que executar se não ele substitui '.' por ',' e quebra a separação de campos da msg
    Gtk::Main kit(argc, argv);
    g_window = std::unique_ptr<MsgWindow>(new MsgWindow());
    g_window->show();


    carmen_ipc_initialize(argc, argv);
    signal(SIGINT, shutdown_module);

    find_potential_messages();

    carmen_ipc_addPeriodicTimer(1.0 / 1.0, (TIMER_HANDLER_TYPE) identify_active_messages, NULL);
    carmen_ipc_addPeriodicTimer(1.0 / 20.0, (TIMER_HANDLER_TYPE) update_window_and_read_events, NULL);
    carmen_ipc_addPeriodicTimer(SCAN_PERIOD, (TIMER_HANDLER_TYPE) scan_messages_update_handler, NULL);

    std::cout << "\n-------------------------------------------\n";
    std::cout << "Listening...\n";
    carmen_ipc_dispatch();

    return 0;
}
