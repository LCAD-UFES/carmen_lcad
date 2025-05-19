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

#include <carmen/carmen.h>
#include <carmen/ipc.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/fused_odometry_interface.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include <nav_msgs/msg/odometry.hpp>

// Mapeia as mensagens para os seus publisher/subscribers

struct msg_register
{
    std::vector<std::string> publishers;
    std::vector<std::string> subscribers;
    msg_register() {}
};

std::unordered_map<std::string, msg_register> messages_defined; 

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
void new_message_handler(carmen_ipc_watcher_new_message *msg)
{
	if(messages_defined.find(msg->msg_name) == messages_defined.end())
	{
		messages_defined[msg->msg_name] = msg_register();
		printf("Mensagem definida! \n");
		
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
    return 0;
}

