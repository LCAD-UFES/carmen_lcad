#include <iostream>
#include <vector>

#include <carmen/carmen.h>
#include <carmen/traffic_light_interface.h>
#include <carmen/traffic_light_messages.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/localize_ackerman_messages.h>
#include <carmen/rddf_interface.h>
#include <carmen/rddf_messages.h>

using namespace std;

//Parameters
static int new_message_localize = 0;
//static int infinite = 9999.0;


carmen_vector_3D_t vector_3D;
std::vector<carmen_rddf_annotation_message> positions;

//Messages
carmen_rddf_annotation_message annotation_message;
carmen_localize_ackerman_globalpos_message localize_message;
carmen_mapping_traffic_light_message mapping_traffic_light_message;
int count = 0;


void
calculate_distance()
{
    double distance = 0.0;
    double orientation = 0.0;
    double behind = 0.0;

    for (std::vector<carmen_rddf_annotation_message>::iterator it = positions.begin(); it != positions.end(); ++it)
    {
        distance = sqrt(pow(localize_message.globalpos.x - it.base()->annotation_point.x, 2) +
                        pow(localize_message.globalpos.y - it.base()->annotation_point.y, 2));
        orientation = fabs(carmen_radians_to_degrees(localize_message.globalpos.theta - it.base()->annotation_orientation)) < 10 ? 1 : 0;
        behind = fabs(carmen_normalize_theta((atan2(localize_message.globalpos.y - it.base()->annotation_point.y,
                                                    localize_message.globalpos.x - it.base()->annotation_point.x) - M_PI
                                             - localize_message.globalpos.theta))) > M_PI_2;

        if (distance <= MAX_TRAFFIC_LIGHT_DISTANCE && orientation == 1 && behind == 0)
        {

            mapping_traffic_light_message.distance = distance;
            mapping_traffic_light_message.has_signals = 1;
            mapping_traffic_light_message.timestamp = localize_message.timestamp;
            carmen_mapping_traffic_light_publish_message(&mapping_traffic_light_message);

            return;
        }
    }

    //    mapping_traffic_light_message.distance = infinite;
    //    mapping_traffic_light_message.has_signals = 0;
    //    mapping_traffic_light_message.timestamp = localize_message.timestamp;
    //Para procurar sempre
    mapping_traffic_light_message.distance = -1;
    mapping_traffic_light_message.has_signals = 1;
    mapping_traffic_light_message.timestamp = localize_message.timestamp;
    carmen_mapping_traffic_light_publish_message(&mapping_traffic_light_message);
}


void
compute_mapping_traffic_light()
{
    if (new_message_localize == 1)
        calculate_distance();
}


void
carmen_rddf_annotation_message_handler(carmen_rddf_annotation_message *msg)
{
    carmen_rddf_add_annotation_message message = *msg;
    positions.push_back(message);
}


void
carmen_localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *msg)
{
    localize_message = *msg;
    new_message_localize = 1;
    compute_mapping_traffic_light();
}


void
subscribe_messages()
{
    carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) carmen_localize_ackerman_globalpos_message_handler,
    		CARMEN_SUBSCRIBE_LATEST);
    carmen_rddf_subscribe_annotation_message(NULL, (carmen_handler_t) carmen_rddf_annotation_message_handler, CARMEN_SUBSCRIBE_ALL);
}


void
carmen_mapping_traffic_light_algorithm_initialization()
{
    mapping_traffic_light_message.position.x = 0.0;
    mapping_traffic_light_message.position.y = 0.0;
    mapping_traffic_light_message.position.z = 0.0;

    mapping_traffic_light_message.has_signals = 0;
    mapping_traffic_light_message.distance = 0.0;

    mapping_traffic_light_message.host = carmen_get_host();
    mapping_traffic_light_message.timestamp = carmen_get_time();
}


static int
read_parameters(int argc, char **argv)
{
    int num_items;

    if (argc != 1)
    {
        printf("Usage: %s", argv[0]);
        exit(0);
    }

    carmen_param_t param_list[] = {};

    num_items = sizeof (param_list) / sizeof (param_list[0]);
    carmen_param_install_params(argc, argv, param_list, num_items);

    return 0;
}


int
main(int argc, char **argv)
{
    carmen_ipc_initialize(argc, argv);
    carmen_param_check_version(argv[0]);

    if ((argc != 1))
        carmen_die(
                   "%s: Wrong number of parameters. Mapping Traffic Light requires 0 parameters and received %d parameter(s). \nUsage:\n %s\n",
                   argv[0], argc - 1, argv[0]);

    read_parameters(argc, argv);

    carmen_mapping_traffic_light_define_messages();
    subscribe_messages();

    carmen_mapping_traffic_light_algorithm_initialization();

    carmen_ipc_dispatch();

    return (EXIT_SUCCESS);
}
