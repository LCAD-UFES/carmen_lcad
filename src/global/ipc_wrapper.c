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

#include "global.h"
#include "carmen_stdio.h"
#include "ipc_wrapper.h"

/* internal list of IPC callback functions */

typedef struct {
  IPC_CONTEXT_PTR context;
  carmen_handler_t handler;
  void *data;
  int first, message_size;
} carmen_callback_t, *carmen_callback_p;

typedef struct carmen_message_list {
  char *message_name;
  int num_callbacks;
  carmen_callback_p callback;
  struct carmen_message_list *next;
} carmen_message_list_t, *carmen_message_list_p;

/* internal list of IPC file descriptor callback functions */

static carmen_message_list_p message_list = NULL;

typedef struct {
  IPC_CONTEXT_PTR context;
  carmen_handler_t handler;
} carmen_fd_callback_t, *carmen_fd_callback_p;

typedef struct carmen_fd_list {
  int fd;
  int num_callbacks;
  carmen_fd_callback_p callback;
  struct carmen_fd_list *next;
} carmen_fd_list_t, *carmen_fd_list_p;

static carmen_fd_list_p fd_list = NULL;

/* logging variables */

static carmen_FILE *outfile = NULL;
static double start_time = 0;

static carmen_FILE *infile = NULL;

static int blogfile_fast = 0;
static int carmen_use_handlers = 1;

typedef struct {
  char *message_name;
  FORMATTER_PTR formatter;
} carmen_blogfile_index_t, *carmen_blogfile_index_p;

static carmen_blogfile_index_p message_index = NULL;
static int index_length = 0;

#define     CARMEN_BLOG_FORMAT_MSG_ID       0
#define     CARMEN_BLOG_DATA_MSG_ID         1

MSG_INSTANCE current_msgRef;

char carmen_module_name[1024];


int
carmen_ipc_connect_locked(char *module_name)
{
  IPC_RETURN_TYPE err;
  int ierr;
  char ipc_name[200];

  module_name = carmen_extract_filename(module_name);
  carmen_print_version();

  /* set verbosity level */
  IPC_setVerbosity(IPC_Silent);

  /* connect with unique ID */
  snprintf(ipc_name, 200, "%s-%d", module_name, getpid());
  err = IPC_connect(ipc_name);
  if(err == IPC_Error) {
    carmen_warn("Could not connect to central.\n\n");
    carmen_warn("Did you remember to start central?\n");
    carmen_warn("Did you remember to set your CENTRALHOST variable?");
    carmen_warn(" It is currently ");
    if(getenv("CENTRALHOST"))
      carmen_warn("\nset to %s.\n", getenv("CENTRALHOST"));
    else
      carmen_warn("not set.\n");
    return -1;
  }

  /* check to see if non-unique module name is already connected */
  ierr = IPC_isModuleConnected(module_name);
  if(ierr == 1)
    carmen_die("Error: module %s already connected to IPC network.\n",
	    module_name);

  /* disconnect and reconnect with non-unique name */
  IPC_disconnect();
  err = IPC_connect(module_name);
  if(err == IPC_Error) {
    carmen_warn("Could not connect to central.\n\n");
    carmen_warn("Did you remember to start central?\n");
    carmen_warn("Did you remember to set your CENTRALHOST variable?");
    carmen_warn(" It is currently ");
    if(getenv("CENTRALHOST"))
      carmen_warn("\nset to %s.\n", getenv("CENTRALHOST"));
    else
      carmen_warn("not set.\n");
    return -1;
  }

  /* Set local message queue capacity */
  err = IPC_setCapacity(1);
  carmen_test_ipc_exit(err, "I had problems setting the IPC capacity. This is a "
		    "very strange error and should never happen.\n",
		    "IPC_setCapacity");
  return 0;
}

int
carmen_ipc_connect(char *module_name)
{
  IPC_RETURN_TYPE err;
  char ipc_name[200];

  module_name = carmen_extract_filename(module_name);
  strcpy(carmen_module_name, module_name);

  carmen_print_version();
  snprintf(ipc_name, 200, "%s-%d", module_name, getpid());

  /* set verbosity level */
  IPC_setVerbosity(IPC_Silent);

  /* connect to the central server */
  err = IPC_connect(ipc_name);
  if(err == IPC_Error) {
    carmen_warn("Could not connect to central.\n\n");
    carmen_warn("Did you remember to start central?\n");
    carmen_warn("Did you remember to set your CENTRALHOST variable?");
    carmen_warn(" It is currently ");
    if(getenv("CENTRALHOST"))
      carmen_warn("\nset to %s.\n", getenv("CENTRALHOST"));
    else
      carmen_warn("not set.\n");
    return -1;
  }

  /* Set local message queue capacity */
  err = IPC_setCapacity(1);
  carmen_test_ipc_exit(err, "I had problems setting the IPC capacity. This is a "
		    "very strange error and should never happen.\n",
		    "IPC_setCapacity");
  return 0;
}

int
carmen_ipc_connect_long(char *module_name, char *host, int port)
{
  IPC_RETURN_TYPE err;
  char ipc_name[200], central_name[200];

  module_name = carmen_extract_filename(module_name);
  carmen_print_version();
  snprintf(ipc_name, 200, "%s-%d", module_name, getpid());
  snprintf(central_name, 200, "%s:%d", host, port);

  /* set verbosity level */
  IPC_setVerbosity(IPC_Silent);

  /* connect to the central server */
  err = IPC_connectModule(ipc_name, central_name);
  if(err == IPC_Error) {
    carmen_warn("Could not connect to central %s.\n\n", central_name);
    return -1;
  }

  /* Set local message queue capacity */
  err = IPC_setCapacity(1);
  carmen_test_ipc_exit(err, "I had problems setting the IPC capacity. This is a "
		    "very strange error and should never happen.\n",
		    "IPC_setCapacity");
  return 0;
}

void carmen_write_formatter_message(char *message_name, char *message_fmt,
				 int message_size)
{
  int temp;

  message_index =
    (carmen_blogfile_index_p)realloc(message_index,
				  sizeof(carmen_blogfile_index_t) *
				  (index_length + 1));
  message_index[index_length].message_name =
    (char *)calloc(strlen(message_name) + 1, 1);
  carmen_test_alloc(message_index[index_length].message_name);
  strcpy(message_index[index_length].message_name, message_name);
  index_length++;

  /* write the message ID */
  temp = CARMEN_BLOG_FORMAT_MSG_ID;
  carmen_fwrite(&temp, sizeof(int), 1, outfile);
  /* write the message length */
  temp = strlen(message_name) + strlen(message_fmt) + 3 * sizeof(int);
  carmen_fwrite(&temp, sizeof(int), 1, outfile);
  /* write the message data */
  temp = strlen(message_name);
  carmen_fwrite(&temp, sizeof(int), 1, outfile);
  carmen_fwrite(message_name, strlen(message_name), 1, outfile);
  temp = strlen(message_fmt);
  carmen_fwrite(&temp, sizeof(int), 1, outfile);
  carmen_fwrite(message_fmt, strlen(message_fmt), 1, outfile);
  temp = message_size;
  carmen_fwrite(&temp, sizeof(int), 1, outfile);
}

void carmen_write_data_message(BYTE_ARRAY callData, int data_length,
			    char *message_name)
{
  int i, temp;
  double timestamp;

  /* write the message ID */
  temp = CARMEN_BLOG_DATA_MSG_ID;
  carmen_fwrite(&temp, sizeof(int), 1, outfile);
  /* write the message length */
  temp = sizeof(int) + sizeof(int) + data_length + sizeof(double);
  carmen_fwrite(&temp, sizeof(int), 1, outfile);
  /* write the message id */
  i = 0;
  while(i < index_length &&
	strcmp(message_name, message_index[i].message_name))
    i++;
  if(i == index_length)
    carmen_die("Error: tried to write an unindexed message."
	    " This should never happen.\n");
  carmen_fwrite(&i, sizeof(int), 1, outfile);
  /* write the data length */
  carmen_fwrite(&data_length, sizeof(int), 1, outfile);
  /* write the message data */
  carmen_fwrite((void *)callData, data_length, 1, outfile);
  /* write the timestamp */
  timestamp = carmen_get_time() - start_time;
  carmen_fwrite(&timestamp, sizeof(double), 1, outfile);
}

static void
carmen_generic_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
		       void *clientData)
{
  IPC_RETURN_TYPE err = IPC_OK;
  IPC_CONTEXT_PTR context;
  FORMATTER_PTR formatter;
  carmen_message_list_p mark = (carmen_message_list_p)clientData;
  int i, n;

  current_msgRef = msgRef;
  /* NEW LOGGER BEGIN */
  if(outfile != NULL)
    carmen_write_data_message(callData, IPC_dataLength(msgRef),
			      mark->message_name);
  /* NEW LOGGER END */

  i = 0;
  context = IPC_getContext();
  while(i < mark->num_callbacks) {
    if(mark->callback[i].context == context) {
      if(mark->callback[i].data) {
	formatter = IPC_msgInstanceFormatter(msgRef);
	if(!mark->callback[i].first)
	  IPC_freeDataElements(formatter, mark->callback[i].data);
	err = IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData,
				 mark->callback[i].data,
				 mark->callback[i].message_size);

	mark->callback[i].first = 0;
      }
      n = mark->num_callbacks;
      if(mark->callback[i].handler && carmen_use_handlers)
	mark->callback[i].handler(mark->callback[i].data);
      if(mark->num_callbacks >= n)
	i++;
    }
    else
      i++;
  }
  IPC_freeByteArray(callData);
  carmen_test_ipc_return(err, "Could not unmarshall",
		      IPC_msgInstanceName(msgRef));
}

/* NEW LOGGER BEGIN */
void
carmen_set_output_blogfile(char *filename)
{
  fprintf(stderr, "Writing IPC input to logfile %s.\n", filename);
  if(infile != NULL)
    carmen_die("Error: logfiles cannot be used as both input and output.\n");
  outfile = carmen_fopen(filename, "w");
  if(outfile == NULL)
    carmen_die("Error: could not open blog file %s.\n", filename);
  start_time = carmen_get_time();
}

void
carmen_set_input_blogfile(char *filename)
{
  fprintf(stderr, "Taking IPC input from logfile %s.\n", filename);
  if(outfile != NULL)
    carmen_die("Error: logfiles cannot be used as both input and output.\n");
  infile = carmen_fopen(filename, "r");
  if(infile == NULL)
    carmen_die("Error: could not open blog file %s\n", filename);
  start_time = carmen_get_time();
}

void
carmen_close_output_blogfile(void)
{
  if(outfile != NULL)
    carmen_fclose(outfile);
}

void
carmen_set_blogfile_fast(void)
{
  blogfile_fast = 1;
}

void
carmen_close_input_blogfile(void)
{
  if(infile != NULL)
    carmen_fclose(infile);
}

void
carmen_disable_handlers(void)
{
  carmen_use_handlers = 0;
}

/* NEW LOGGER END */

void
carmen_subscribe_message(char *message_name, char *message_fmt,
			 void *message_mem, int message_size,
			 carmen_handler_t handler,
			 carmen_subscribe_t subscribe_how)
{
  IPC_RETURN_TYPE err = IPC_OK;
  IPC_CONTEXT_PTR context;
  carmen_message_list_p mark;
  int i;

  /* define the message, just in case */
  err = IPC_defineMsg(message_name, IPC_VARIABLE_LENGTH, message_fmt);
  carmen_test_ipc_exit(err, "Could not define message", message_name);

  /* look for a matching message name */
  mark = message_list;
  while(mark != NULL && strcmp(message_name, mark->message_name))
    mark = mark->next;

  /* if no match, add message name to the list */
  if(mark == NULL) {
    mark = (carmen_message_list_p)calloc(1, sizeof(carmen_message_list_t));
    carmen_test_alloc(mark);
    mark->message_name = (char *)calloc(1, strlen(message_name) + 1);
    carmen_test_alloc(mark->message_name);
    strcpy(mark->message_name, message_name);
    mark->num_callbacks = 0;
    mark->callback = NULL;
    mark->next = message_list;
    message_list = mark;
    /* NEW LOGGER BEGIN */
    if(outfile != NULL)
      carmen_write_formatter_message(message_name, message_fmt, message_size);
    /* NEW LOGGER END */
  }

  /* make sure that the callback isn't already in there */
  context = IPC_getContext();
  i = 0;
  while(i < mark->num_callbacks && (mark->callback[i].handler != handler ||
				    mark->callback[i].context != context))
    i++;
  if(i == mark->num_callbacks) {
    mark->num_callbacks++;
    mark->callback = (carmen_callback_p)realloc(mark->callback,
					     mark->num_callbacks *
					     sizeof(carmen_callback_t));
    carmen_test_alloc(mark->callback);

    /* allocate message memory if necessary */
    if(message_mem) {
      mark->callback[mark->num_callbacks - 1].data = message_mem;
      memset(message_mem, 0, message_size);
    }
    else {
      mark->callback[mark->num_callbacks - 1].data = calloc(1, message_size);
      carmen_test_alloc(mark->callback[mark->num_callbacks - 1].data);
    }

    mark->callback[mark->num_callbacks - 1].context = context;
    mark->callback[mark->num_callbacks - 1].handler = handler;
    mark->callback[mark->num_callbacks - 1].first = 1;
    mark->callback[mark->num_callbacks - 1].message_size = message_size;
  }

  if(infile == NULL) {
    err = IPC_subscribe(message_name, carmen_generic_handler, mark);
    if(subscribe_how == CARMEN_SUBSCRIBE_LATEST)
      IPC_setMsgQueueLength(message_name, 1);
    else
      IPC_setMsgQueueLength(message_name, 1000);
    carmen_test_ipc(err, "Could not subscribe", message_name);
  }
}

void
carmen_unsubscribe_message(char *message_name, carmen_handler_t handler)
{
  IPC_CONTEXT_PTR context;
  carmen_message_list_p mark;
  int i;

  /* look for a matching message name */
  mark = message_list;
  while(mark != NULL && strcmp(message_name, mark->message_name))
    mark = mark->next;

  if(mark == NULL) {
    carmen_warn("carmen_unsubscribe_message: Not subscribed to %s."
	     "Taking no action.\n", message_name);
    return;
  }

  /* look for the appropriate handler */
  context = IPC_getContext();
  i = 0;
  while(i < mark->num_callbacks && (mark->callback[i].handler != handler ||
				    mark->callback[i].context != context))
    i++;
  if(i != mark->num_callbacks) {
    memmove(mark->callback + i, mark->callback + i + 1,
	   (mark->num_callbacks - i - 1) * sizeof(carmen_callback_t));
    mark->num_callbacks--;
    if(mark->num_callbacks == 0)
      if(infile == NULL)
	IPC_unsubscribe(message_name, carmen_generic_handler);
  }
  else
    carmen_warn("carmen_unsubscribe_message: Could not find"
	     " matching callback for %s\n", message_name);
}

char *carmen_get_host(void)
{
  // The return value of getenv() is a pointer to a static buffer, whose contents
  // may be changed by later calls. Therefore it's safer to create a copy of
  // the returned string.
  //
  // See: http://www.cplusplus.com/reference/cstdlib/getenv/
  //
  // "The pointer returned [by a getenv() call] points to an internal memory block,
  // whose content or validity may be altered by further calls to getenv
  // (but not by other library functions)."
  static char hostname[1026];
  FILE *bin_host;

  if (getenv("HOST") == NULL) {
    if (getenv("HOSTNAME") != NULL)
      setenv("HOST", getenv("HOSTNAME"), 1);
    else if (getenv("host") != NULL)
      setenv("HOST", getenv("host"), 1);
    else if (getenv("hostname") != NULL)
      setenv("HOST", getenv("hostname"), 1);
    else {
      bin_host = popen("/bin/hostname", "r");
      if (bin_host == NULL)
	carmen_die("\n\tCan't get machine name from $HOST, $host, $hostname or /bin/hostname.\n"
		   "\tPlease set one of these environment variables properly.\n\n");
      fscanf(bin_host, "%s", hostname);
      setenv("HOST", hostname, 1);
      pclose(bin_host);
    }
  }

  sprintf(hostname, "%s@%s", carmen_module_name, getenv("HOST"));
//  strcpy(hostname, getenv("HOST"));
  return hostname;
}

static double initialize_time = 0;

void carmen_ipc_initialize(int argc, char **argv)
{
  int err, i;

  err = carmen_ipc_connect(argv[0]);
  if(err < 0)
    exit(1);

  for(i = 1; i < argc; i++) {
    if(strcmp(argv[i], "-fast") == 0)
      carmen_set_blogfile_fast();
    if(strcmp(argv[i], "-input_log") == 0 && i < argc - 1 &&
       argv[i + 1][0] != '-')
      carmen_set_input_blogfile(argv[i + 1]);
    else if(strcmp(argv[i], "-output_log") == 0 && i < argc - 1 &&
       argv[i + 1][0] != '-')
      carmen_set_output_blogfile(argv[i + 1]);
    else if(strcmp(argv[i], "-no_handlers") == 0)
      carmen_disable_handlers();
  }

  initialize_time = carmen_get_time();
}

double
carmen_ipc_initialize_time(void)
{
	return initialize_time;
}

void carmen_ipc_initialize_locked(int argc, char **argv)
{
  int err, i;

  err = carmen_ipc_connect_locked(argv[0]);
  if(err < 0)
    exit(1);
  for(i = 1; i < argc; i++) {
    if(strcmp(argv[i], "-fast") == 0)
      carmen_set_blogfile_fast();
    if(strcmp(argv[i], "-input_log") == 0 && i < argc - 1 &&
       argv[i + 1][0] != '-')
      carmen_set_input_blogfile(argv[i + 1]);
    else if(strcmp(argv[i], "-output_log") == 0 && i < argc - 1 &&
       argv[i + 1][0] != '-')
      carmen_set_output_blogfile(argv[i + 1]);
    else if(strcmp(argv[i], "-no_handlers") == 0)
      carmen_disable_handlers();
  }
}

void carmen_ipc_initialize_locked_with_name(int argc, char **argv, char *name)
{
  int err, i;

  err = carmen_ipc_connect_locked(name);
  if(err < 0)
    exit(1);
  for(i = 1; i < argc; i++) {
    if(strcmp(argv[i], "-fast") == 0)
      carmen_set_blogfile_fast();
    if(strcmp(argv[i], "-input_log") == 0 && i < argc - 1 &&
       argv[i + 1][0] != '-')
      carmen_set_input_blogfile(argv[i + 1]);
    else if(strcmp(argv[i], "-output_log") == 0 && i < argc - 1 &&
       argv[i + 1][0] != '-')
      carmen_set_output_blogfile(argv[i + 1]);
    else if(strcmp(argv[i], "-no_handlers") == 0)
      carmen_disable_handlers();
  }
}

double carmen_blogfile_handle_one_message(void)
{
  int err, i, n, message_length, message_id, message_name_length, data_length;
  unsigned char buffer[10000], message_name[256];
  double current_time, timestamp;
  carmen_message_list_p mark;

  err = carmen_fread(&message_id, sizeof(int), 1, infile);
  if(err != 1)
    return -1;
  err = carmen_fread(&message_length, sizeof(int), 1, infile);
  if(err != 1)
    return -1;
  err = carmen_fread(&buffer, message_length, 1, infile);
  if(err != 1)
    return -1;
  if(message_id == CARMEN_BLOG_FORMAT_MSG_ID) {
    message_name_length = *((int *)buffer);
    strncpy((char *)message_name, (const char *)(buffer + sizeof(int)),
	    message_name_length);
    message_name[message_name_length] = '\0';
    message_index =
      (carmen_blogfile_index_p)realloc(message_index,
				   sizeof(carmen_blogfile_index_t) *
				   (index_length + 1));
    message_index[index_length].message_name =
      (char *)calloc(message_name_length + 1, 1);
    carmen_test_alloc(message_index[index_length].message_name);
    strcpy(message_index[index_length].message_name,
	   (const char *)message_name);
    message_index[index_length].formatter =
      IPC_msgFormatter((const char *)message_name);
    index_length++;
    carmen_test_alloc(message_index);
    return -1;
  }
  else if(message_id == CARMEN_BLOG_DATA_MSG_ID) {
    message_id = *((int *)buffer);
    strcpy((char *)message_name, message_index[message_id].message_name);
    data_length = *((int *)(buffer + sizeof(int)));
    timestamp = *((double *)(buffer + sizeof(int) +
			     sizeof(int) + data_length));

    if(!blogfile_fast) {
      current_time = carmen_get_time() - start_time;
      if(timestamp > current_time)
	usleep((timestamp - current_time) * 1e6);
    }

    mark = message_list;
    while(mark != NULL && strcmp((char *)message_name, mark->message_name))
      mark = mark->next;

    if(mark != NULL) {
      i = 0;
      while(i < mark->num_callbacks) {
	if(mark->callback[i].data) {
	  if(!mark->callback[i].first)
	    IPC_freeDataElements(message_index[message_id].formatter,
				 mark->callback[i].data);
	  IPC_unmarshallData(message_index[message_id].formatter,
			     buffer + sizeof(int) + sizeof(int),
			     mark->callback[i].data,
			     mark->callback[i].message_size);
	  mark->callback[i].first = 0;
	}
	n = mark->num_callbacks;
	if(mark->callback[i].handler)
	  mark->callback[i].handler(mark->callback[i].data);
	if(mark->num_callbacks >= n)
	  i++;
      }
    }
    return timestamp;
  }
  else
    carmen_die("Error: could not parse logfile.\n");
  return -1;
}

void carmen_ipc_dispatch(void)
{
  if(infile == NULL)
    IPC_dispatch();
  else {
    do {
      carmen_blogfile_handle_one_message();
    } while(!carmen_feof(infile));
    fprintf(stderr, "Logfile complete.\n");
  }
}

void carmen_ipc_dispatch_nonblocking(void)
{
  if (IPC_listen(1) == IPC_Error) // wait 1 micro second for a message
  {
    /* If control reaches here, IPC_listen returned IPC_Error */
    printf("Error while receiving IPC messages!\n");
  }
}

void carmen_ipc_sleep(double timeout)
{
  double current_time, timestamp;

  current_time = carmen_get_time();
  if(infile == NULL)
    IPC_listenWait(timeout * 1000);
  else {
    do {
      timestamp = carmen_blogfile_handle_one_message();
    } while(!carmen_feof(infile) && timestamp != -1 &&
	    timestamp - current_time < timeout);
  }
}

void carmen_ipc_disconnect(void)
{
  carmen_close_output_blogfile();
  carmen_close_input_blogfile();
  IPC_disconnect();
}

void carmen_ipc_addPeriodicTimer(double interval, TIMER_HANDLER_TYPE handler,
			      void *clientData)
{
  if(infile == NULL)
    IPC_addPeriodicTimer(interval * 1000.0, handler, clientData);
  else
    carmen_die("Error: carmen_ipc_addPeriodicTimer not supported from logfiles.\n");
}

void carmen_test_ipc(IPC_RETURN_TYPE err, const char *err_msg,
		     const char *ipc_msg)
{
  if(err == IPC_OK)
    return;
  if(err == IPC_Timeout) {
    fprintf(stderr, "IPC_ERROR : Message timed out : message %s\n", ipc_msg);
    return;
  }
  if(err == IPC_Error) {
    fprintf(stderr, "IPC_ERROR : %s : message %s\n", err_msg, ipc_msg);
    IPC_perror("          ");
    return;
  }
  fprintf(stderr, "Bad value encountered in err value passed to %s\n",
	  __FUNCTION__);
  fprintf(stderr, "This is very definitely a bug, and most likely in IPC.\n");
  fprintf(stderr, "Please file a bug report.\n");
}

void carmen_generic_fd_handler(int fd, void *clientData)
{
  IPC_CONTEXT_PTR context;
  carmen_fd_list_p mark = (carmen_fd_list_p)clientData;
  int i, n;

  i = 0;
  context = IPC_getContext();
  while(i < mark->num_callbacks) {
    if(mark->callback[i].context == context) {
      n = mark->num_callbacks;
      mark->callback[i].handler(&fd);
      if(mark->num_callbacks >= n)
	i++;
    }
    else
      i++;
  }
}

void carmen_ipc_subscribe_fd(int fd, carmen_handler_t handler)
{
  IPC_RETURN_TYPE err = IPC_OK;
  IPC_CONTEXT_PTR context;
  carmen_fd_list_p mark;
  int i;

  /* look for a matching message name */
  mark = fd_list;
  while(mark != NULL && fd != mark->fd)
    mark = mark->next;

  /* if no match, add message name to the list */
  if(mark == NULL) {
    mark = (carmen_fd_list_p)calloc(1, sizeof(carmen_fd_list_t));
    carmen_test_alloc(mark);
    mark->fd = fd;
    mark->num_callbacks = 0;
    mark->callback = NULL;
    mark->next = fd_list;
    fd_list = mark;
  }

  /* make sure that the callback isn't already in there */
  context = IPC_getContext();
  i = 0;
  while(i < mark->num_callbacks && (mark->callback[i].handler != handler ||
				    mark->callback[i].context != context))
    i++;
  if(i == mark->num_callbacks) {
    mark->num_callbacks++;
    mark->callback = (carmen_fd_callback_p)realloc(mark->callback,
						mark->num_callbacks *
						sizeof(carmen_fd_callback_t));
    carmen_test_alloc(mark->callback);

    mark->callback[mark->num_callbacks - 1].context = context;
    mark->callback[mark->num_callbacks - 1].handler = handler;
  }

  err = IPC_subscribeFD(fd, carmen_generic_fd_handler, mark);
  if(err != IPC_OK)
    carmen_warn("Error: could not subscribe to fd = %d\n", fd);
}

void carmen_ipc_unsubscribe_fd(int fd, carmen_handler_t handler)
{
  IPC_CONTEXT_PTR context;
  carmen_fd_list_p mark;
  int i;

  /* look for a matching message name */
  mark = fd_list;
  while(mark != NULL && fd != mark->fd)
    mark = mark->next;

  if(mark == NULL) {
    carmen_warn("carmen_unsubscribe_fd: Not subscribed to fd = %d\n", fd);
    return;
  }

  /* look for the appropriate handler */
  context = IPC_getContext();
  i = 0;
  while(i < mark->num_callbacks && (mark->callback[i].handler != handler ||
				    mark->callback[i].context != context))
    i++;
  if(i != mark->num_callbacks) {
    memmove(mark->callback + i, mark->callback + i + 1,
	   (mark->num_callbacks - i - 1) * sizeof(carmen_fd_callback_t));
    mark->num_callbacks--;
    if(mark->num_callbacks == 0)
      if(infile == NULL)
	IPC_unsubscribeFD(fd, carmen_generic_fd_handler);
  }
  else
    carmen_warn("carmen_unsubscribe_fd: Could not find"
		" matching callback for %d\n", fd);
}


void
carmen_publish_heartbeat(char *module_name)
{
	carmen_heartbeat_message msg;
	IPC_RETURN_TYPE err;

	msg.module_name = carmen_new_string(module_name);
	msg.pid = getpid();
	msg.hostname = carmen_get_host();
	msg.timestamp = carmen_get_time();

	static int first_time = 1;
	if (first_time)
	{
		err = IPC_defineMsg(CARMEN_HEARTBEAT_NAME, IPC_VARIABLE_LENGTH, CARMEN_HEARTBEAT_FMT);
		carmen_test_ipc_exit(err, "Could not define", CARMEN_HEARTBEAT_NAME);
		first_time = 0;
	}

	err = IPC_publishData(CARMEN_HEARTBEAT_NAME, &msg);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_HEARTBEAT_NAME);

	free(msg.module_name);
}


void
carmen_subscribe_heartbeat_message(carmen_heartbeat_message *heartbeat,
				   carmen_handler_t handler,
				   carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_HEARTBEAT_NAME, CARMEN_HEARTBEAT_FMT,
                           heartbeat, sizeof(carmen_heartbeat_message),
			   handler, subscribe_how);
}

void x_ipcRegisterExitProc(void (*proc)(void));
void carmen_ipc_registerExitProc(void (*proc)(void)) {
       x_ipcRegisterExitProc(proc);
}
