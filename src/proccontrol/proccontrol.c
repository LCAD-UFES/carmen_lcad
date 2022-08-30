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
#include <sys/wait.h>
#include "proccontrol_messages.h"
#include "proccontrol_ipc.h"
#include "proccontrol.h"

process_info_t process[MAX_PROCESSES];
int num_processes = 0;
int my_pid;
char *my_hostname = NULL;
char warning_msg[2000];

void kill_process(process_info_p process)
{
	int status;

	snprintf(warning_msg, 2000, "PROCCONTROL (%d): Killing %s (%d)\n",
			my_pid, process->command_line, process->pid);
	fprintf(stderr, "%s", warning_msg);
	proccontrol_publish_output(process->pid, process->module_name, warning_msg);
	kill(process->pid, SIGINT);
	usleep(0.05 * 1e6);
	kill(process->pid, SIGKILL);
	process->state = 0;

	waitpid(process->pid, &status, 0);

	close(process->pipefd[0]);
	close(process->pipefd[1]);
}

void start_process(process_info_p process)
{
	char *arg[4];
	int spawned_pid;

	/* First argument will be "sh" plus one byte for the terminating null. */
	arg[0] = (char *)malloc((strlen("sh") + 1) * sizeof(char));
	carmen_test_alloc(arg[0]);
	strcpy(arg[0], "sh");

	/* Second argument will be "-c" plus one byte for the terminating null. */
	arg[1] = (char *)malloc((strlen("-c") + 1) * sizeof(char));
	carmen_test_alloc(arg[1]);
	strcpy(arg[1], "-c");

	/* Third argument needs to be a single string with the actual command
     to run and its arguments. */
	arg[2] = (char *)malloc((strlen(process->command_line) + 1) * sizeof(char));
	carmen_test_alloc(arg[2]);
	strcpy(arg[2], process->command_line);

	/* Fourth argument will be NULL as required by the function. */
	arg[3] = NULL;

	pipe(process->pipefd);

	/* fork! */
	if((spawned_pid = fork()) == 0) {
		/* I am the child */

		/* redirect child's stdout and stderr to pipe to parent */
		dup2(process->pipefd[1], fileno(stdout));
		dup2(process->pipefd[1], fileno(stderr));

		//antigo: /bin/sh
		execv("/bin/bash", arg);
		/* execv() only returns if there's an error.  If there's an error,
       we just exit uncleanly and the parent will restart the
       process again. */
		exit(-1);
	}

	snprintf(warning_msg, 2000, "PROCCONTROL (%d): Spawned %s (%d)\n", my_pid,
			process->command_line, spawned_pid);
	fprintf(stderr, "%s", warning_msg);
	proccontrol_publish_output(spawned_pid, process->module_name, warning_msg);
	process->pid = spawned_pid;
	process->state = 1;
	process->last_heartbeat = carmen_get_time();
	process->start_time = carmen_get_time();
}

void pc_handle_signal(int sig __attribute__ ((unused)))
{
	int i;
	for(i = 0; i < num_processes; i++)
		if(process[i].state)
			kill_process(&process[i]);
	exit(0);
}

void clear_signal_handlers(void)
{
	int this_signal;

	for(this_signal = 0; this_signal < 128; this_signal++)
		signal(this_signal, SIG_DFL);
}

void start_signal_handlers(void)
{
	int this_signal;

	for(this_signal = 0; this_signal < 128; this_signal++)
//		if(this_signal != SIGCHLD && // this_signal != SIGCLD &&
//				this_signal != SIGCONT)
		if(this_signal == SIGINT || this_signal == SIGKILL || this_signal == SIGTERM || this_signal == SIGSEGV || this_signal == SIGBUS)
			signal(this_signal, pc_handle_signal);
}

void process_timer(void *clientdata __attribute__ ((unused)),
		unsigned long currenttime __attribute__ ((unused)),
		unsigned long scheduledTime __attribute__ ((unused)))
{
	int i, err, status, process_killed, n, nread, update_pidtable = 0;
	unsigned char buffer[10000];
	double current_time;
	static double last_pidtable = 0;

	do {
		process_killed = 0;

		/* loop through process table */
		for(i = 0; i < num_processes; i++) {
			if(process[i].state != process[i].requested_state) {
				if(process[i].requested_state == 1) {
					/* if the program hasn't been started, start it */
					//carmen_ipc_sleep(2.0);
					clear_signal_handlers();
					start_process(&process[i]);
					start_signal_handlers();
				}
				else
					kill_process(&process[i]);
				update_pidtable = 1;
			}
			else if(process[i].state == 1) {
				/* check for last heartbeat */
				current_time = carmen_get_time();
				if(process[i].watch_heartbeats &&
						current_time - process[i].last_heartbeat > 0.5)
				{
					snprintf(warning_msg, 2000, "PROCCONTROL (%d): %s (%d) lost heartbeats\n",
							my_pid, process[i].command_line, process[i].pid);
					fprintf(stderr, "%s", warning_msg);
					proccontrol_publish_output(process[i].pid, process[i].module_name, warning_msg);

					kill_process(&process[i]);
					clear_signal_handlers();
					start_process(&process[i]);
					start_signal_handlers();
				}

				/* otherwise check to make sure it is still running */
				err = waitpid(process[i].pid, &status, WNOHANG);

				if(err == 0) {
					/* read stdout or stderr from pipe */
					n = carmen_serial_numChars(process[i].pipefd[0]);
					if(n > 0) {
						nread = carmen_serial_readn(process[i].pipefd[0], buffer, n);
						if(nread >= 0)
							buffer[nread] = '\0';
						proccontrol_publish_output(process[i].pid, process[i].module_name, (char *)buffer);
					}
				}
				else {
					/* Check if the child process terminated due to uncaught signal.
	     If so, mark it to be restarted */
					if(WIFSIGNALED(status))
					{
						snprintf(warning_msg, 2000, "PROCCONTROL (%d): %s (%d) exited due to SIGNAL (code = %d).\n",
								my_pid, process[i].command_line, process[i].pid, WTERMSIG(status));
						fprintf(stderr, "%s", warning_msg);
						proccontrol_publish_output(process[i].pid, process[i].module_name, warning_msg);
						usleep(0.05 * 1e6);

						/* Clean up file descriptors. */
						close(process[i].pipefd[0]);
						close(process[i].pipefd[1]);

						/* mark process as inactive */
						process[i].state = 0;
						process_killed = 1;
						continue;
					}

					/* Check if the child process was stopped or suspended.  If so, kill
	     it and mark it to be restarted */
					if(WIFSTOPPED(status))
					{
						snprintf(warning_msg, 2000, "PROCCONTROL (%d): %s (%d) was STOPPED "
								"(code = %d).  Killing process.\n", my_pid,
								process[i].command_line, process[i].pid, WSTOPSIG(status));
						fprintf(stderr, "%s", warning_msg);
						proccontrol_publish_output(process[i].pid, process[i].module_name, warning_msg);

						/* Kill the child process. */
						kill(process[i].pid, SIGINT);
						usleep(0.05 * 1e6);
						kill(process[i].pid, SIGKILL);
						process[i].state = 0;
						process_killed = 1;

						/* Clean up file descriptors. */
						close(process[i].pipefd[0]);
						close(process[i].pipefd[1]);
						continue;
					}

					/* Check if the child process exited (ie: return; exit(int); etc.).
	     If so and the code != 0, restart it. */
					if(WIFEXITED(status))
					{
						snprintf(warning_msg, 2000, "PROCCONTROL (%d): %s (%d) exited UNCLEANLY (code = %d).\n",
								my_pid, process[i].command_line, process[i].pid, WEXITSTATUS(status));
						fprintf(stderr, "%s", warning_msg);
						proccontrol_publish_output(process[i].pid, process[i].module_name, warning_msg);
						usleep(0.05 * 1e6);

						/* Clean up file descriptors. */
						close(process[i].pipefd[0]);
						close(process[i].pipefd[1]);

						/* Go to top of loop and start over. */
						process[i].state = 0;
						process_killed = 1;
						continue;
					}
				}
			}
		}
		if(process_killed)
			update_pidtable = 1;
	} while(process_killed);

	/* periodically publish PID table */
	current_time = carmen_get_time();
	if(update_pidtable || current_time - last_pidtable > 2.0) {
		proccontrol_publish_pidtable(num_processes, process);
		last_pidtable = current_time;
	}
}

void moduleset_handler(carmen_proccontrol_moduleset_message *query)
{
	int i;

	for(i = 0; i < num_processes; i++)
		if(strcmp(query->module_name, process[i].module_name) == 0)
			process[i].requested_state = query->requested_state;
}

void groupset_handler(carmen_proccontrol_groupset_message *query)
{
	int i;

	for(i = 0; i < num_processes; i++)
		if(strcmp(query->group_name, process[i].group_name) == 0)
			process[i].requested_state = query->requested_state;
}

void heartbeat_handler(carmen_heartbeat_message *heartbeat)
{
	int i;

	for(i = 0; i < num_processes; i++)
		if(strcmp(process[i].module_name, heartbeat->module_name) == 0)
			process[i].last_heartbeat = carmen_get_time();
}

void read_process_ini(char *filename)
{
	FILE *fp;
	char *err, *mark, line[1000];
	int i, l;

	num_processes = 0;

	fp = fopen(filename, "r");
	if(fp == NULL)
		carmen_die("Error: could not read ini file %s\n", filename);

	do {
		err = fgets(line, 1000, fp);
		if(err != NULL) {
			l = strlen(line);
			/* strip out comments and newlines */
			for(i = 0; i < l; i++)
				if(line[i] == '#' || line[i] == '\n')
					line[i] = '\0';

			/* advance to the first non-whitespace character */
			mark = line;
			while(mark[0] != '\0' && isspace(mark[0]))
				mark++;

			/* read the line */
			if(mark[0] != '\0') {
				process[num_processes].module_name = (char *)calloc(256, 1);
				carmen_test_alloc(process[num_processes].module_name);
				sscanf(mark, "%s", process[num_processes].module_name);
				mark = carmen_next_word(mark);
				process[num_processes].group_name = (char *)calloc(256, 1);
				carmen_test_alloc(process[num_processes].group_name);
				sscanf(mark, "%s", process[num_processes].group_name);
				mark = carmen_next_word(mark);
				process[num_processes].requested_state = atoi(mark);
				mark = carmen_next_word(mark);
				process[num_processes].watch_heartbeats = atoi(mark);
				mark = carmen_next_word(mark);
				strcpy(process[num_processes].command_line, mark);

				fprintf(stderr, "%d: %s %s %d %d %s\n", num_processes,
						process[num_processes].module_name,
						process[num_processes].group_name,
						process[num_processes].requested_state,
						process[num_processes].watch_heartbeats,
						process[num_processes].command_line);
				process[num_processes].state = 0;
				num_processes++;
			}
		}
	} while(err != NULL);
	fclose(fp);
}

void x_ipcRegisterExitProc(void (*proc)(void));

char module_name[256];

int my_ipc_connect(char *ipc_module_name)
{
	IPC_RETURN_TYPE err;

	/* connect to the central server */
	err = IPC_connect(ipc_module_name);
	if(err == IPC_Error)
		return -1;

	/* Set local message queue capacity */
//	err = IPC_setCapacity(4);
//	carmen_test_ipc_exit(err, "I had problems setting the IPC capacity. This is a "
//			"very strange error and should never happen.\n",
//			"IPC_setCapacity");
	return 0;
}

static void reconnect_central(void)
{
	int err;

	do {
		if(IPC_isConnected())
			IPC_disconnect();
		err = my_ipc_connect(module_name);
		if(err == 0) {
			carmen_warn("Reconnected to IPC.\n");
			proccontrol_register_ipc_messages();
			x_ipcRegisterExitProc(reconnect_central);
			process_timer(NULL, 0, 0);
			carmen_ipc_addPeriodicTimer(1.0 / 5.0, process_timer, NULL);

			/* subscribe to requests to change process and group state */
			carmen_subscribe_message(CARMEN_PROCCONTROL_MODULESET_NAME,
					CARMEN_PROCCONTROL_MODULESET_FMT,
					NULL, sizeof(carmen_proccontrol_moduleset_message),
					(carmen_handler_t)moduleset_handler,
					CARMEN_SUBSCRIBE_ALL);

			carmen_subscribe_message(CARMEN_PROCCONTROL_GROUPSET_NAME,
					CARMEN_PROCCONTROL_GROUPSET_FMT,
					NULL, sizeof(carmen_proccontrol_groupset_message),
					(carmen_handler_t)groupset_handler,
					CARMEN_SUBSCRIBE_ALL);

			carmen_subscribe_heartbeat_message(NULL, (carmen_handler_t)
					heartbeat_handler,
					CARMEN_SUBSCRIBE_ALL);
		}
		else
			usleep(0.1 * 1e6);
	} while(err == -1);
}

int main(int argc, char **argv)
{
	char filename[256];

	my_hostname = carmen_get_host();
	if(argc >= 2)
		strcpy(filename, argv[1]);
	else {
		strcpy(filename, "process.ini");
		if(!carmen_file_exists(filename))
			strcpy(filename, "../race/src/process.ini");
	}

	if(!carmen_file_exists(filename))
		carmen_die("Error: could not open process file %s\n", filename);

	/* construct unique IPC module name */
	snprintf(module_name, 200, "%s-%d", carmen_extract_filename(argv[0]), getpid());

	/* connect to the IPC server, regsiter messages */
	carmen_ipc_initialize(argc, argv);
	x_ipcRegisterExitProc(reconnect_central);
	proccontrol_register_ipc_messages();

	/* get my process ID */
	my_pid = getpid();

	read_process_ini(filename);

	/* subscribe to requests to change process and group state */
	carmen_subscribe_message(CARMEN_PROCCONTROL_MODULESET_NAME,
			CARMEN_PROCCONTROL_MODULESET_FMT,
			NULL, sizeof(carmen_proccontrol_moduleset_message),
			(carmen_handler_t)moduleset_handler,
			CARMEN_SUBSCRIBE_ALL);

	carmen_subscribe_message(CARMEN_PROCCONTROL_GROUPSET_NAME,
			CARMEN_PROCCONTROL_GROUPSET_FMT,
			NULL, sizeof(carmen_proccontrol_groupset_message),
			(carmen_handler_t)groupset_handler,
			CARMEN_SUBSCRIBE_ALL);

	carmen_subscribe_heartbeat_message(NULL, (carmen_handler_t)heartbeat_handler,
			CARMEN_SUBSCRIBE_ALL);

	/* add 5 Hz timer function */
	carmen_ipc_addPeriodicTimer(1.0 / 5.0, process_timer, NULL);

	/* loop forever */
	carmen_ipc_dispatch();
	return 0;
}
