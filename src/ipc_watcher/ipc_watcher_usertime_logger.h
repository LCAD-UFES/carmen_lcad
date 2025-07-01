#ifndef IPC_WATCHER_USERTIME_LOGGER_H
#define IPC_WATCHER_USERTIME_LOGGER_H

// Essa mensagem serve para monitorar uso de tempo, por exemplo, se quer monitorar o tempo gasto em uma função
#define		CARMEN_IPC_WATCHER_LOG_USETIME_NAME		"carmen_ipc_watcher_log_usetime"
#define		CARMEN_IPC_WATCHER_LOG_USETIME_FMT		"{string, string, double, double, string}"
typedef struct
{
	const char *record_name;
	const char *parent_name;
	double time_spent;
	double timestamp;
	char *host;
} carmen_ipc_watcher_log_usetime;

// Um contador de tempo de execução
class ipc_watcher_log_timer
{
private:
	double m_start;
	carmen_ipc_watcher_log_usetime m_msg;
public:
	ipc_watcher_log_timer(const char* parent_name, const char* record_name);
	~ipc_watcher_log_timer();
	void update_timer();
};

void ipc_watcher_report_usetime(const char* parent_name, const char* record_name, double time_spent);

#endif