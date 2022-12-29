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
#include <locale.h>


void
carmen_subscribe_playback_info_message (carmen_playback_info_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message (CARMEN_PLAYBACK_INFO_MESSAGE_NAME, CARMEN_PLAYBACK_INFO_MESSAGE_FMT, message, sizeof (carmen_playback_info_message), handler, subscribe_how);
}


void
carmen_unsubscribe_playback_info_message (carmen_handler_t handler)
{
	carmen_unsubscribe_message (CARMEN_PLAYBACK_INFO_MESSAGE_NAME, handler);
}


void
carmen_playback_define_messages (void)
{
	IPC_RETURN_TYPE err = IPC_defineMsg (CARMEN_PLAYBACK_INFO_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_PLAYBACK_INFO_MESSAGE_FMT);
	carmen_test_ipc_exit (err, "Could not define", CARMEN_PLAYBACK_INFO_MESSAGE_NAME);
}


void
carmen_playback_command (int cmd, char *message, int offset, float speed)
{
	IPC_RETURN_TYPE err;
	carmen_playback_command_message playback_msg;

	static int initialized = 0;

	if (!initialized)
	{
		err = IPC_defineMsg (CARMEN_PLAYBACK_COMMAND_NAME, IPC_VARIABLE_LENGTH, CARMEN_PLAYBACK_COMMAND_FMT);
		carmen_test_ipc_exit (err, "Could not define message", CARMEN_PLAYBACK_COMMAND_NAME);

		initialized = 1;
	}

	playback_msg.cmd = cmd;
	playback_msg.message = message;
	playback_msg.offset = offset;
	playback_msg.speed = speed;

	err = IPC_publishData (CARMEN_PLAYBACK_COMMAND_NAME, &playback_msg);
	carmen_test_ipc (err, "Could not publish", CARMEN_PLAYBACK_COMMAND_NAME);
}


int
carmen_playback_is_valid_speed(char *value, double *speed)
{
    setlocale(LC_ALL, "C");

    char *comma = strchr(value, ','); // para o caso de alguem digitar virgula em vez de ponto decimal
    if (comma != NULL)
        (*comma) = '.';

    int ok = FALSE, pos = 0;

    if (sscanf(value, " %lf %n", speed, &pos) == 1 && value[pos] == 0)
    	ok = ((*speed) > 0.0);

    return (ok);
}


int
carmen_playback_is_valid_message(char *message, int *start_msg, int *stop_msg, double *start_ts, double *stop_ts,
		double *recur_start_ts, double *recur_stop_ts, double *start_x, double *start_y, double *stop_x, double *stop_y, double *radius)
{
    setlocale(LC_ALL, "C");

	char *comma;

    for (comma = message; (*comma) != 0; comma++)
    {
    	if ((*comma) == ',') // para o caso de alguem digitar virgulas em vez de pontos decimais
    		(*comma) = '.';
    }

    int ok = FALSE, pos = 0;

    if (sscanf(message, " %d %n", start_msg, &pos) == 1 && message[pos] == 0)
    	ok = ((*start_msg) >= 0);
    else if (sscanf(message, " : %d %n", stop_msg, &pos) == 1 && message[pos] == 0)
    	ok = ((*stop_msg) >= 0);
    else if (sscanf(message, " %d : %d %n", start_msg, stop_msg, &pos) == 2 && message[pos] == 0)
    	ok = ((*start_msg) >= 0 && (*stop_msg) >= (*start_msg));
    else if (sscanf(message, " t %lf %n", start_ts, &pos) == 1 && message[pos] == 0)
    	ok = ((*start_ts) >= 0.0);
    else if (sscanf(message, " t : %lf %n", stop_ts, &pos) == 1 && message[pos] == 0)
    	ok = ((*stop_ts) >= 0.0);
    else if (sscanf(message, " t %lf : %lf %n", start_ts, stop_ts, &pos) == 2 && message[pos] == 0)
    	ok = ((*start_ts) >= 0.0 && (*stop_ts) >= (*start_ts));
    else if (sscanf(message, " r %lf %n", recur_start_ts, &pos) == 1 && message[pos] == 0)
    	ok = ((*recur_start_ts) >= 0.0);
    else if (sscanf(message, " r : %lf %n", recur_stop_ts, &pos) == 1 && message[pos] == 0)
    	ok = ((*recur_stop_ts) >= 0.0);
    else if (sscanf(message, " r %lf : %lf %n", recur_start_ts, recur_stop_ts, &pos) == 2 && message[pos] == 0)
    	ok = ((*recur_start_ts) >= 0.0 && (*recur_stop_ts) >= (*recur_start_ts));
    else if (sscanf(message, " p %lf %lf %n", start_x, start_y, &pos) == 2 && message[pos] == 0)
    	ok = TRUE;
    else if (sscanf(message, " p : %lf %lf %n", stop_x, stop_y, &pos) == 2 && message[pos] == 0)
    	ok = TRUE;
    else if (sscanf(message, " p %lf %lf : %lf %lf %n", start_x, start_y, stop_x, stop_y, &pos) == 4 && message[pos] == 0)
    	ok = TRUE;
    else if (sscanf(message, " p : : %lf %n", radius, &pos) == 1 && message[pos] == 0)
    	ok = ((*radius) >= 0.0);
    else if (sscanf(message, " p %lf %lf : : %lf %n", start_x, start_y, radius, &pos) == 3 && message[pos] == 0)
    	ok = ((*radius) >= 0.0);
    else if (sscanf(message, " p : %lf %lf : %lf %n", stop_x, stop_y, radius, &pos) == 3 && message[pos] == 0)
    	ok = ((*radius) >= 0.0);
    else if (sscanf(message, " p %lf %lf : %lf %lf : %lf %n", start_x, start_y, stop_x, stop_y, radius, &pos) == 5 && message[pos] == 0)
    	ok = ((*radius) >= 0.0);

    return (ok);
}
