/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#include <carmen/carmen.h>
#include <carmen/traffic_light_messages.h>
#include <carmen/traffic_light_interface.h>

char *
carmen_traffic_light_message_name(int camera)
{
    char *message_name = (char*) malloc(128 * sizeof (char));
    sprintf(message_name, "%s%d", CARMEN_TRAFFIC_LIGHT_NAME, camera);
    return message_name;
}

void
carmen_traffic_light_subscribe(int camera,
        carmen_traffic_light_message *traffic_light_message,
        carmen_handler_t handler,
        carmen_subscribe_t subscribe_how)
{
    char *message_name = carmen_traffic_light_message_name(camera);
    carmen_subscribe_message((char *) message_name, (char *) CARMEN_TRAFFIC_LIGHT_FMT,
            traffic_light_message, sizeof (carmen_traffic_light_message),
            handler, subscribe_how);
    free(message_name);
    printf("\nSubscribed to Traffic Light Messages (From Camera %d)!\n", camera);
}

void
carmen_traffic_light_unsubscribe(int camera, carmen_handler_t handler)
{
    char *message_name = carmen_traffic_light_message_name(camera);
    carmen_unsubscribe_message(message_name, handler);
    free(message_name);
}

IPC_RETURN_TYPE
carmen_traffic_light_publish_message(int camera, carmen_traffic_light_message *message)
{
    IPC_RETURN_TYPE err;

    char *message_name = carmen_traffic_light_message_name(camera);

    err = IPC_publishData(message_name, message);
    carmen_test_ipc_exit(err, "Could not publish", message_name);
    free(message_name);

    return err;
}

IPC_RETURN_TYPE
carmen_traffic_light_define_messages(int camera)
{
    IPC_RETURN_TYPE err;

    char *message_name = carmen_traffic_light_message_name(camera);
    err = IPC_defineMsg(message_name, IPC_VARIABLE_LENGTH, CARMEN_TRAFFIC_LIGHT_FMT);
    carmen_test_ipc_exit(err, "Could not define", message_name);
    free(message_name);
    return err;
}

IPC_RETURN_TYPE
carmen_mapping_traffic_light_define_messages()
{
    IPC_RETURN_TYPE err;

    char *message_name = carmen_mapping_traffic_light_message_name();
    err = IPC_defineMsg(message_name, IPC_VARIABLE_LENGTH, CARMEN_MAPPING_TRAFFIC_LIGHT_FMT);
    carmen_test_ipc_exit(err, "Could not define", message_name);
    free(message_name);
    return err;
}

char *
carmen_mapping_traffic_light_message_name()
{
    char *message_name = (char*) malloc(128 * sizeof (char));
    sprintf(message_name, "%s", CARMEN_MAPPING_TRAFFIC_LIGHT_NAME);
    return message_name;
}

void
carmen_mapping_traffic_light_subscribe(
        carmen_mapping_traffic_light_message *mapping_traffic_light_message,
        carmen_handler_t handler,
        carmen_subscribe_t subscribe_how)
{
    char *message_name = carmen_mapping_traffic_light_message_name();
    carmen_subscribe_message((char *) message_name, (char *) CARMEN_MAPPING_TRAFFIC_LIGHT_FMT,
            mapping_traffic_light_message, sizeof (carmen_mapping_traffic_light_message),
            handler, subscribe_how);
    free(message_name);
    printf("\nSubscribed to Mapping Traffic Light Messages)!\n");
}

void
carmen_mapping_traffic_light_unsubscribe(carmen_handler_t handler)
{
    char *message_name = carmen_mapping_traffic_light_message_name();
    carmen_unsubscribe_message(message_name, handler);
    free(message_name);
}

IPC_RETURN_TYPE
carmen_mapping_traffic_light_publish_message(carmen_mapping_traffic_light_message *message)
{
    IPC_RETURN_TYPE err;

    char *message_name = carmen_mapping_traffic_light_message_name();

    err = IPC_publishData(message_name, message);
    carmen_test_ipc_exit(err, "Could not publish", message_name);
    free(message_name);

    return err;
}
