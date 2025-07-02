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
#include <chrono>
#include <string.h>
#include <gtkmm.h>
#include <string>
#include <thread>
#include <mutex>
#include <stack>

#include <carmen/carmen.h>
#include <carmen/ipc.h>
#include "ipc_watcher.h"
#include "ipc_watcher_usertime_logger.h"

// Mapeia as mensagens para os seus publisher/subscribers
struct msg_register
{
	std::vector<std::string> publishers;
    std::vector<std::string> subscribers;
    msg_register() {}
};

struct unlisted_pair
{
	std::string msg;
	std::string module;
	unlisted_pair() {}
};

// Janela
class MyWindow : public Gtk::Window {
public:
    MyWindow();

    // Atualiza o conteúdo da janela com base nas variáveis globais
    void update_window();

private:
    Gtk::Notebook m_notebook;
    Gtk::ScrolledWindow m_scroll;
    Gtk::Box m_box;
	std::vector<Gtk::ScrolledWindow> m_scroll_timers;
	std::unordered_map<std::string, Gtk::Box*> m_box_timers;
};

// mapeia mensagens definidas para seus publisher/subscribers
std::unordered_map<std::string, msg_register> messages_defined; 
std::stack<std::string> unlisted_messages;// Mensagens que ainda n foram mostradas na tela
// std::stack<unlisted_pair> unlisted_message_publisher;// Mensagens que ainda n foram mostradas na tela
// std::stack<unlisted_pair> unlisted_message_subscriber;// Mensagens que ainda n foram mostradas na tela

// mapeia um timer pai (mapper, por exemplo) para os nodos filhos (funções de callback, por exemplo)
std::unordered_map<std::string, std::unordered_map<std::string, double>> log_usetime_register; 
std::stack<std::string> unlisted_parents;

MyWindow* g_window_ptr = nullptr;

// Thread que fica responsável pela janela
void gtk_thread_main(int argc, char* argv[]) {
    auto app = Gtk::Application::create(argc, argv, "org.exemplo.variaveis.globais");
    MyWindow janela;
    g_window_ptr = &janela;
    app->run(janela);
}

//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//																								//
// Handlers																						//
//																								//
//////////////////////////////////////////////////////////////////////////////////////////////////

static 
void new_message_handler(carmen_ipc_watcher_new_message *msg)
{
	if(messages_defined.find(msg->msg_name) == messages_defined.end())
	{
		messages_defined[msg->msg_name] = msg_register();
		unlisted_messages.push(msg->msg_name);

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
			// unlisted_message_subscriber.push(msg->msg_name,msg->host)
        }
	}
}

static 
void log_usetime_handler(carmen_ipc_watcher_log_usetime* msg)
{
	if(log_usetime_register.find(msg->parent_name) == log_usetime_register.end())
	{
		log_usetime_register[msg->parent_name] = std::unordered_map<std::string, double>();
		unlisted_parents.push(std::string(msg->parent_name));
	}	
	if(log_usetime_register[msg->parent_name].find(msg->record_name) == log_usetime_register[msg->parent_name].end())
	{
		log_usetime_register[msg->parent_name][msg->record_name] = 0.0;
	}
	log_usetime_register[msg->parent_name][msg->record_name] += msg->time_spent;	
}

static
void update_window_handler()
{
	Glib::signal_idle().connect_once([] {
		if (g_window_ptr) {
			g_window_ptr->update_window();
		}
	});
}

void 
MyWindow::update_window() {
	// Adicionar abas de timers
	while (!unlisted_parents.empty()) {
        std::string parent_name = unlisted_parents.top();
        unlisted_parents.pop();

		m_scroll_timers.emplace_back();
		auto* box = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_VERTICAL, 5);
		m_box_timers[parent_name] = box;

		Gtk::ScrolledWindow& scroll = m_scroll_timers.back();

        scroll.set_policy(Gtk::POLICY_AUTOMATIC, Gtk::POLICY_AUTOMATIC);
		scroll.add(*box);
		box->set_orientation(Gtk::ORIENTATION_VERTICAL);
		box->set_spacing(5);
		box->set_border_width(10);
		m_notebook.append_page(scroll, parent_name);
    }
	show_all_children();

	// Update nas abas de timer
	for (auto& [parent_name, timers] : log_usetime_register) {
		// Limpa a aba
		Gtk::Box* box = m_box_timers[parent_name];
    	auto linhas = box->get_children();
    	for (auto* linha : linhas) {
        	box->remove(*linha);
   		}

		// Re-adiciona os valores
		for(auto& [record_name, time_spent] : timers) {
			// Cria uma linha horizontal
			auto* row = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_HORIZONTAL, 10);

			// Label com a string
			auto* label_texto = Gtk::make_managed<Gtk::Label>(record_name);
			label_texto->set_xalign(0);

			// // Label com o valor formatado (exemplo com 2 casas decimais)
			// auto valor_str = std::to_string(valor);
			// Opcional: formatar para 2 casas decimais (mais elegante)
			char buf[50];
			snprintf(buf, sizeof(buf), "%.2f", time_spent);
			auto* label_valor = Gtk::make_managed<Gtk::Label>(buf);
			label_valor->set_xalign(0);

			// Adiciona os labels na linha
			row->pack_start(*label_texto, Gtk::PACK_SHRINK);
			row->pack_start(*label_valor, Gtk::PACK_EXPAND_WIDGET);

			// Adiciona a linha na box principal
			box->pack_start(*row, Gtk::PACK_SHRINK);
		}
		box->show_all_children();
	}
	
	// Adicionar mensagens
	while (!unlisted_messages.empty()) {
		std::string& msg_name = unlisted_messages.top();

		auto* row = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_HORIZONTAL, 10);
    	auto* label1 = Gtk::make_managed<Gtk::Label>(msg_name);
    	auto* label2 = Gtk::make_managed<Gtk::Label>("");
		label1->set_xalign(0);
		label2->set_xalign(0);
		row->pack_start(*label1, Gtk::PACK_SHRINK);
		row->pack_start(*label2, Gtk::PACK_EXPAND_WIDGET);
		m_box.pack_start(*row, Gtk::PACK_SHRINK);

		unlisted_messages.pop();
	}

	m_box.show_all_children();
}

//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//																								//
// Inicializations																				//
//																								//
//////////////////////////////////////////////////////////////////////////////////////////////////

MyWindow::MyWindow() {
    set_title("IPC Watcher");
    set_default_size(400, 300);
    m_notebook.set_border_width(10);
    add(m_notebook);

    m_scroll.set_policy(Gtk::POLICY_AUTOMATIC, Gtk::POLICY_AUTOMATIC);
    m_scroll.add(m_box);
    m_box.set_orientation(Gtk::ORIENTATION_VERTICAL);
    m_box.set_spacing(5);
    m_box.set_border_width(10);
    m_notebook.append_page(m_scroll, "Mensagens");

    show_all_children();
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
        
        arquivo << "------------------------------------------------\n";
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
void print_usetime_register()
{
	std::cout << "\nRegistro: \n";
	for (const auto& parent : log_usetime_register)
	{
		std::cout << "---- Parent: " << parent.first << std::endl;
		for (const auto& child : parent.second)
		{
			std::cout << "-- Nome: " << child.first << ", Tempo: " << child.second << std::endl; 
		}
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

	err = IPC_defineMsg(CARMEN_IPC_WATCHER_LOG_USETIME_NAME, IPC_VARIABLE_LENGTH, CARMEN_IPC_WATCHER_LOG_USETIME_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_IPC_WATCHER_LOG_USETIME_NAME);
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

	carmen_subscribe_message((char*) CARMEN_IPC_WATCHER_LOG_USETIME_NAME,
			(char*) CARMEN_IPC_WATCHER_LOG_USETIME_FMT,
			NULL, sizeof(carmen_ipc_watcher_log_usetime),
			(carmen_handler_t) log_usetime_handler, CARMEN_SUBSCRIBE_ALL);
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
		print_usetime_register();
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

    std::thread gui_thread(gtk_thread_main, argc, argv);
	while (!g_window_ptr) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

	carmen_ipc_addPeriodicTimer(1.0 / 40.0, (TIMER_HANDLER_TYPE) update_window_handler, NULL);
	carmen_ipc_dispatch();
    return 0;
}

