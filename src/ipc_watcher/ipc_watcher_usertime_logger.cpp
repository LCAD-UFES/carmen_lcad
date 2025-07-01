#include <unordered_map>
#include <iostream>
#include <chrono>

#include <carmen/carmen.h>
#include "ipc_watcher_usertime_logger.h"

// Reportar um tempo arbitrário passado de um módulo
void
ipc_watcher_report_usetime(const char* parent_name, const char* record_name, double time_spent)
{
	carmen_ipc_watcher_log_usetime msg;
	msg.parent_name = parent_name;
	msg.record_name = record_name;
	msg.time_spent = time_spent;
	msg.host = carmen_get_host();

	IPC_RETURN_TYPE err;
	err = IPC_publishData(CARMEN_IPC_WATCHER_LOG_USETIME_NAME, &msg);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_IPC_WATCHER_LOG_USETIME_NAME);
}

// Monitorar tempo gasto dentro de uma função
ipc_watcher_log_timer::ipc_watcher_log_timer(const char* parent_name, const char* record_name)
{
	auto now = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(now.time_since_epoch());
	m_start = duration.count();
	m_msg.record_name = record_name;
	m_msg.parent_name = parent_name;
}

ipc_watcher_log_timer::~ipc_watcher_log_timer()
{
	// Pega o tempo
	auto now = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(now.time_since_epoch());
	double d_t = duration.count() - m_start;

	if(IPC_isMsgDefined(CARMEN_IPC_WATCHER_LOG_USETIME_NAME))
	{
		m_msg.time_spent = d_t;
		m_msg.host = carmen_get_host();

		// Envia
		IPC_RETURN_TYPE err;
		err = IPC_publishData(CARMEN_IPC_WATCHER_LOG_USETIME_NAME, &m_msg);
		carmen_test_ipc_exit(err, "Could not publish", CARMEN_IPC_WATCHER_LOG_USETIME_NAME);
	}
}

void 
ipc_watcher_log_timer::update_timer()
{
	// Pega o tempo
	auto now = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(now.time_since_epoch());
	double d_t = duration.count() - m_start;
	if(IPC_isMsgDefined(CARMEN_IPC_WATCHER_LOG_USETIME_NAME))
	{
		m_start = duration.count();
		m_msg.time_spent = d_t;
		m_msg.host = carmen_get_host();

		// Envia
		IPC_RETURN_TYPE err;
		err = IPC_publishData(CARMEN_IPC_WATCHER_LOG_USETIME_NAME, &m_msg);
		carmen_test_ipc_exit(err, "Could not publish", CARMEN_IPC_WATCHER_LOG_USETIME_NAME);
	}
}