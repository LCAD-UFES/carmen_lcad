#include "carmen/carmen.h"
#include "carmen/ipc.h"
#include "carmen/global.h"
#include "carmen/task_manager_messages.h"
#include <unordered_map>
#include <iostream>
#include "string.h"
#include <boost/bind.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros1_noetic_ros_drive/carmen_ros_drive_globalpos_message.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>


typedef struct {
std::string IPC_msg_name;
std::vector<std::string> ROS_topics; // Uma mesma mensagem carmen pode publicar em vários tópicos ROS
std::vector<std::string> ROS_msg_types;
// 1 Publisher para cada tópico da mensage,
std::vector<ros::Publisher> ROS_publishers;
} carmen_to_ros_msg_info;

typedef struct {
std::string IPC_msg_name;
std::vector<std::string> ROS_topics; // Varios tópicos ROS podem publicar uma mesma mensagem carmen (não sei em que momento isso seria útil, mas é possível)
std::vector<std::string> ROS_msg_types;
// 1 Subscriber para cada tópico da mensage,
std::vector<ros::Subscriber> ROS_subscribers;
} ros_to_carmen_msg_info;

char* raw_carmen_to_ros_info;
int carmen_to_ros_info_quantity;
std::unordered_map<std::string, carmen_to_ros_msg_info> carmen_to_ros_info;  // Mapeia nome da msg Astro para informações

char* raw_ros_to_carmen_info;
int ros_to_carmen_info_quantity;
std::unordered_map<std::string, ros_to_carmen_msg_info> ros_to_carmen_info; // Mapeia nome da msg Astro para informações

ros::NodeHandle* nh;  

// Retirada de readlog.cpp 
int 
first_wordlength(char *str)
{
	char* c_enter = strchr(str, '\n'); // check also for newline
	char* c = strchr(str, ' ');

	if (c_enter == NULL && c == NULL) // it is the last word in the string
		return strlen(str);

	if (c_enter != NULL && c == NULL) // there is no space but a newline
		return c_enter - str;

	if (c_enter == NULL && c != NULL) // there is a space but no newline
		return c - str;

	if (c_enter < c )    // use whatever comes first
		return c_enter - str;
	else
		return c - str;
}

// Retirada de readlog.cpp 
void 
CLF_READ_STRING(char *dst, char **string)
{
	int l;

	/* advance past spaces */
	while(*string[0] == ' ')
		*string += 1;

	l = first_wordlength(*string);
	strncpy(dst, *string, l);
	dst[l] = '\0';
	*string += l;
}

void
read_carmen_to_ros_info()
{
	for(int i = 0; i < (carmen_to_ros_info_quantity*3); i++)
	{
		static std::string last_msg_name;
		char tmp[250];
		CLF_READ_STRING(tmp, &raw_carmen_to_ros_info);
		switch (i % 3)
		{
		case 0:
			last_msg_name = tmp;
			carmen_to_ros_info[last_msg_name].IPC_msg_name = std::string(tmp);
			break;
		case 1:
			carmen_to_ros_info[last_msg_name].ROS_topics.emplace_back(tmp);
			break;
		case 2:
			carmen_to_ros_info[last_msg_name].ROS_msg_types.emplace_back(tmp);
			break;
		default:
			break;
		}
	}
	// for(const auto& pair : carmen_to_ros_info)
	// {
	// 	std::cout << pair.second.IPC_msg_name << ", " <<  pair.second.ROS_topics.size() << ", " << pair.second.ROS_topics[0] << std::endl;
	// }
}

void
read_ros_to_carmen_info()
{
	for(int i = 0; i < (ros_to_carmen_info_quantity*3); i++)
	{
		static std::string last_msg_name;
		char tmp[250];
		CLF_READ_STRING(tmp, &raw_ros_to_carmen_info);
		switch (i % 3)
		{
		case 0:
			last_msg_name = tmp;
			ros_to_carmen_info[last_msg_name].IPC_msg_name = std::string(tmp);
			break;
		case 1:
			ros_to_carmen_info[last_msg_name].ROS_topics.emplace_back(tmp);
			break;
		case 2:
			ros_to_carmen_info[last_msg_name].ROS_msg_types.emplace_back(tmp);
			break;
		default:
			break;
		}
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
carmen_localize_ackerman_globalpos_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	std::string msg_name = CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME;
	carmen_to_ros_msg_info& msg_info = carmen_to_ros_info[msg_name];
	int num_of_topics = msg_info.ROS_topics.size();
	for(int i = 0; i < num_of_topics; i++)
	{
		if(msg_info.ROS_msg_types[i] == "carmen_ros_drive_globalpos_message")
		{
			ros1_noetic_ros_drive::carmen_ros_drive_globalpos_message ros_msg;
			ros_msg.globalpos.x          = msg->globalpos.x;
			ros_msg.globalpos.y          = msg->globalpos.y;
			ros_msg.globalpos.theta      = msg->globalpos.theta;
			ros_msg.globalpos_std.x      = msg->globalpos_std.x;
			ros_msg.globalpos_std.y      = msg->globalpos_std.y;
			ros_msg.globalpos_std.theta  = msg->globalpos_std.theta;
			ros_msg.odometrypos.x        = msg->odometrypos.x;
			ros_msg.odometrypos.y        = msg->odometrypos.y;
			ros_msg.odometrypos.theta    = msg->odometrypos.theta;
			ros_msg.v                    = msg->v;
			ros_msg.phi                  = msg->phi;
			ros_msg.globalpos_xy_cov     = msg->globalpos_xy_cov;
			ros_msg.converged            = msg->converged;
			ros_msg.semi_trailer_engaged = msg->semi_trailer_engaged;
			ros_msg.semi_trailer_type    = msg->semi_trailer_type;
			ros_msg.num_trailers         = msg->num_trailers;
			ros_msg.trailer_theta[0]	 = msg->trailer_theta[0];
			ros_msg.trailer_theta[1]	 = msg->trailer_theta[1];
			ros_msg.trailer_theta[2]	 = msg->trailer_theta[2];
			ros_msg.trailer_theta[3]	 = msg->trailer_theta[3];
			ros_msg.trailer_theta[4]	 = msg->trailer_theta[4];

			ros_msg.timestamp            = msg->timestamp;
			ros_msg.host                 = msg->host; // Transforma char* para std::string
			msg_info.ROS_publishers[i].publish(ros_msg);
		}
		else
		{
			printf("msg_type not recognized to carmen_msg localize_ackerman_globalpos\n");
		}
	}
}


static void
carmen_task_manager_string_handler(carmen_task_manager_string_message *msg)
{
	std::string msg_name = CARMEN_TASK_MANAGER_STRING_FROM_TASK_MANAGER_MESSAGE_NAME;
	carmen_to_ros_msg_info& msg_info = carmen_to_ros_info[msg_name];
	int num_of_topics = msg_info.ROS_topics.size();
	for(int i = 0; i < num_of_topics; i++)
	{
		if(msg_info.ROS_msg_types[i] == "string")
		{
			std_msgs::String ros_msg;
			ros_msg.data = msg->data;
			msg_info.ROS_publishers[i].publish(ros_msg);
		}
		else
		{
			printf("msg_type not recognized to carmen_msg carmen_task_manager_string_message\n");
		}
	}
}



static void
ros_string_handler(const std_msgs::String::ConstPtr& ros_msg, const std::string& IPC_msg_name)
{
	if(IPC_msg_name == "carmen_heartbeat")
	{
		carmen_publish_heartbeat((char*) ros_msg->data.c_str());
	}
	else if(IPC_msg_name == "carmen_task_manager_string_to_task_manager_message")
	{
		IPC_RETURN_TYPE err;
		static int first_time = 1;
		if (first_time)
		{
			err = IPC_defineMsg(CARMEN_TASK_MANAGER_STRING_TO_TASK_MANAGER_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_TASK_MANAGER_STRING_MESSAGE_FMT);
			carmen_test_ipc_exit(err, "Could not define", CARMEN_TASK_MANAGER_STRING_TO_TASK_MANAGER_MESSAGE_NAME);
			first_time = 0;
		}

		static carmen_task_manager_string_message carmen_msg;
		carmen_msg.data = (char*) ros_msg->data.c_str();
		carmen_msg.host = carmen_get_host();
		carmen_msg.timestamp = carmen_get_time();

		err = IPC_publishData(CARMEN_TASK_MANAGER_STRING_TO_TASK_MANAGER_MESSAGE_NAME, &carmen_msg);
		carmen_test_ipc_exit(err, "Could not publish", CARMEN_TASK_MANAGER_STRING_TO_TASK_MANAGER_MESSAGE_NAME);
	}
	else
	{
		printf("carmen msg_name not recognized to ros msg_type string\n");
	}
}

static void
ros_point_handler(const geometry_msgs::Point::ConstPtr& ros_msg, const std::string& IPC_msg_name)
{
	if(IPC_msg_name == "carmen_task_manager_point_message")
	{
		IPC_RETURN_TYPE err;
		static int first_time = 1;
		if (first_time)
		{
			err = IPC_defineMsg(CARMEN_TASK_MANAGER_POINT_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_TASK_MANAGER_POINT_MESSAGE_FMT);
			carmen_test_ipc_exit(err, "Could not define", CARMEN_TASK_MANAGER_POINT_MESSAGE_NAME);
			first_time = 0;
		}

		static carmen_task_manager_point_message carmen_msg;
		carmen_msg.x = ros_msg->x;
		carmen_msg.y = ros_msg->y;
		carmen_msg.z = ros_msg->z;
		carmen_msg.host = carmen_get_host();
		carmen_msg.timestamp = carmen_get_time();

		err = IPC_publishData(CARMEN_TASK_MANAGER_POINT_MESSAGE_NAME, &carmen_msg);
		carmen_test_ipc_exit(err, "Could not publish", CARMEN_TASK_MANAGER_POINT_MESSAGE_NAME);
	}
	else 
	{
		printf("carmen msg_name not recognized to ros msg_type pount\n");
	}
}

static void
ros_marker_handler(const visualization_msgs::Marker::ConstPtr& ros_msg, const std::string& IPC_msg_name)
{
	if(IPC_msg_name == "carmen_task_manager_pose_quaternion_message")
	{
		IPC_RETURN_TYPE err;
		static int first_time = 1;
		if (first_time)
		{
			err = IPC_defineMsg(CARMEN_TASK_MANAGER_POSE_QUATERNION_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_TASK_MANAGER_POSE_QUATERNION_MESSAGE_FMT);
			carmen_test_ipc_exit(err, "Could not define", CARMEN_TASK_MANAGER_POSE_QUATERNION_MESSAGE_NAME);
			first_time = 0;
		}

		static carmen_task_manager_pose_quaternion_message carmen_msg;
		carmen_msg.x = ros_msg->pose.position.x;
		carmen_msg.y = ros_msg->pose.position.y;
		carmen_msg.z = ros_msg->pose.position.z;
		carmen_msg.quat_x = ros_msg->pose.orientation.x;
		carmen_msg.quat_y = ros_msg->pose.orientation.y;
		carmen_msg.quat_z = ros_msg->pose.orientation.z;
		carmen_msg.quat_w = ros_msg->pose.orientation.w;		
		carmen_msg.host = carmen_get_host();
		carmen_msg.timestamp = carmen_get_time();

		err = IPC_publishData(CARMEN_TASK_MANAGER_POSE_QUATERNION_MESSAGE_NAME, &carmen_msg);
		carmen_test_ipc_exit(err, "Could not publish", CARMEN_TASK_MANAGER_POSE_QUATERNION_MESSAGE_NAME);
	}
	else 
	{
		printf("carmen msg_name not recognized to ros msg_type pount\n");
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////

static void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		delete nh;

        printf("ros1_noetic_ros_drive: disconnected.\n");    
		exit(0);
	}
}

void
carmen_read_parameters(int argc, char **argv)
{

	carmen_param_t param_list[] =
	{
		{(char *) "ros_drive", (char *) "carmen_to_ros_msg_details", CARMEN_PARAM_STRING, &raw_carmen_to_ros_info, 0, NULL},
		{(char *) "ros_drive", (char *) "carmen_to_ros_msg_number", CARMEN_PARAM_INT, &carmen_to_ros_info_quantity, 0, NULL},
		{(char *) "ros_drive", (char *) "ros_to_carmen_msg_details", CARMEN_PARAM_STRING, &raw_ros_to_carmen_info, 0, NULL},
		{(char *) "ros_drive", (char *) "ros_to_carmen_msg_number", CARMEN_PARAM_INT, &ros_to_carmen_info_quantity, 0, NULL},
	};
	carmen_param_allow_unfound_variables(0);
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	read_carmen_to_ros_info();
	read_ros_to_carmen_info();
}


static void
subscribe_to_ipc_messages()
{
	for(auto& pair : carmen_to_ros_info)
	{
		carmen_to_ros_msg_info& msg_info = pair.second;

		// Subscreve para as mensagens carmen
		std::string& IPC_msg_name = msg_info.IPC_msg_name;
		if(IPC_msg_name == CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME)
		{
			carmen_subscribe_message((char*)CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME,
			(char*) CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_FMT,
			NULL, sizeof(carmen_localize_ackerman_globalpos_message),
			(carmen_handler_t) carmen_localize_ackerman_globalpos_handler, CARMEN_SUBSCRIBE_LATEST);
		}
		else if(IPC_msg_name == CARMEN_TASK_MANAGER_STRING_FROM_TASK_MANAGER_MESSAGE_NAME)
		{
			carmen_subscribe_message((char*) CARMEN_TASK_MANAGER_STRING_FROM_TASK_MANAGER_MESSAGE_NAME,
			(char*) CARMEN_TASK_MANAGER_STRING_MESSAGE_FMT,
			NULL, sizeof(carmen_task_manager_string_message),
			(carmen_handler_t) carmen_task_manager_string_handler, CARMEN_SUBSCRIBE_LATEST);
		}
		else
		{
			printf("Carmen message %s not implemented yet...\n", IPC_msg_name.c_str());
		}

		// Cria os publishers ROS
		for(int i = 0; i < msg_info.ROS_topics.size(); i++)
		{
			std::string ROS_msg_type = msg_info.ROS_msg_types[i];
			std::string ROS_topic = msg_info.ROS_topics[i];

			if(ROS_msg_type == "carmen_ros_drive_globalpos_message")
			{
				msg_info.ROS_publishers.push_back(nh->advertise<ros1_noetic_ros_drive::carmen_ros_drive_globalpos_message>(ROS_topic, 10));
			}
			else if(ROS_msg_type == "string")
			{
				msg_info.ROS_publishers.push_back(nh->advertise<std_msgs::String>(ROS_topic, 10));
			}
			else
			{
				printf("ROS message %s not implemented yet to publish...\n", ROS_msg_type.c_str());
			}
		}
	}
}


static void
subscribe_to_ros_messages()
{
	for(auto& pair : ros_to_carmen_info)
	{
		ros_to_carmen_msg_info& msg_info = pair.second;

		for(int i = 0; i < msg_info.ROS_msg_types.size(); i++)
		{
			std::string& ROS_msg_type = msg_info.ROS_msg_types[i];
			std::string& ROS_topic = msg_info.ROS_topics[i];
			if(ROS_msg_type == "string")
			{
				msg_info.ROS_subscribers.push_back(nh->subscribe<std_msgs::String>(ROS_topic, 10, boost::bind(ros_string_handler, _1, msg_info.IPC_msg_name)));
			}
			else if(ROS_msg_type == "point")
			{
				msg_info.ROS_subscribers.push_back(nh->subscribe<geometry_msgs::Point>(ROS_topic, 10, boost::bind(ros_point_handler, _1, msg_info.IPC_msg_name)));
			}
			else if(ROS_msg_type == "marker")
			{
				msg_info.ROS_subscribers.push_back(nh->subscribe<visualization_msgs::Marker>(ROS_topic, 10, boost::bind(ros_marker_handler, _1, msg_info.IPC_msg_name)));
			}
			else
			{
				printf("ROS message %s not implemented yet to subscribe...\n", ROS_msg_type.c_str());
			}
		}
	}
}


static void
define_messages()
{
}

int
main(int argc, char **argv)
{
	/* Connect to IPC Server */
	carmen_ipc_initialize(argc, argv);

	/* Init ros node*/
	ros::init(argc, argv, "ros_drive");
	nh = new ros::NodeHandle();

	/* Check the param server version */
	carmen_param_check_version(argv[0]);

	/* Register shutdown cleaner handler */
	signal(SIGINT, shutdown_module);

	/* Initialize all the relevant parameters */
	carmen_read_parameters(argc, argv);
	
	/* Register messages */
	define_messages();

	/* Subscribe to relevant messages */
	subscribe_to_ipc_messages();
	subscribe_to_ros_messages();

	while (true)
	{
		IPC_listen(10);
		ros::spinOnce();
	}
	return (0);
}