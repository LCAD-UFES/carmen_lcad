#include <carmen/carmen.h>
#include <carmen/global.h>
#include <carmen/carmen_stdio.h>
#include <carmen/ipc_wrapper.h>

/* internal list of IPC callback functions */

typedef struct
{
	IPC_CONTEXT_PTR context;
	carmen_handler_t handler;
	void *data;
	int first, message_size;
} carmen_callback_t, *carmen_callback_p;

typedef struct carmen_message_list
{
	char *message_name;
	int num_callbacks;
	carmen_callback_p callback;
	struct carmen_message_list *next;
} carmen_message_list_t, *carmen_message_list_p;

/* internal list of IPC file descriptor callback functions */

//static carmen_message_list_p message_list = NULL;

typedef struct
{
	IPC_CONTEXT_PTR context;
	carmen_handler_t handler;
} carmen_fd_callback_t, *carmen_fd_callback_p;

typedef struct carmen_fd_list
{
	int fd;
	int num_callbacks;
	carmen_fd_callback_p callback;
	struct carmen_fd_list *next;
} carmen_fd_list_t, *carmen_fd_list_p;


/* logging variables */

//static double start_time = 0;

static carmen_FILE *infile = NULL;

//static int blogfile_fast = 0;

typedef struct
{
	char *message_name;
	FORMATTER_PTR formatter;
} carmen_blogfile_index_t, *carmen_blogfile_index_p;

//static carmen_blogfile_index_p message_index = NULL;
//static int index_length = 0;

#define     CARMEN_BLOG_FORMAT_MSG_ID       0
#define     CARMEN_BLOG_DATA_MSG_ID         1

MSG_INSTANCE current_msgRef;


//double
//carmen_blogfile_handle_one_message(void)
//{
//	int err, i, n, message_length, message_id, message_name_length, data_length;
//	unsigned char buffer[10000], message_name[256];
//	double current_time, timestamp;
//	carmen_message_list_p mark;
//
//	err = carmen_fread(&message_id, sizeof(int), 1, infile);
//	if (err != 1)
//		return -1;
//	err = carmen_fread(&message_length, sizeof(int), 1, infile);
//	if (err != 1)
//		return -1;
//	err = carmen_fread(&buffer, message_length, 1, infile);
//	if (err != 1)
//		return -1;
//	if (message_id == CARMEN_BLOG_FORMAT_MSG_ID)
//	{
//		printf("assa\n");
//		message_name_length = *((int *) buffer);
//		strncpy((char *) message_name, (const char *) (buffer + sizeof(int)), message_name_length);
//		message_name[message_name_length] = '\0';
//		message_index = (carmen_blogfile_index_p) realloc(message_index, sizeof(carmen_blogfile_index_t) * (index_length + 1));
//		message_index[index_length].message_name = (char *) calloc(message_name_length + 1, 1);
//		carmen_test_alloc(message_index[index_length].message_name);
//		strcpy(message_index[index_length].message_name, (const char *) message_name);
//		message_index[index_length].formatter = IPC_msgFormatter((const char *) message_name);
//		index_length++;
//		carmen_test_alloc(message_index);
//		return -1;
//	}
//	else if (message_id == CARMEN_BLOG_DATA_MSG_ID)
//	{
//		printf("xxx\n");
//
//		message_id = *((int *) buffer);
//		strcpy((char *) message_name, message_index[message_id].message_name);
//		data_length = *((int *) (buffer + sizeof(int)));
//		timestamp = *((double *) (buffer + sizeof(int) + sizeof(int) + data_length));
//
//		if (!blogfile_fast)
//		{
//			current_time = carmen_get_time() - start_time;
//			if (timestamp > current_time)
//				usleep((timestamp - current_time) * 1e6);
//		}
//
//		mark = message_list;
//		while (mark != NULL && strcmp((char *) message_name, mark->message_name))
//			mark = mark->next;
//
//		if (mark != NULL)
//		{
//			i = 0;
//			while (i < mark->num_callbacks)
//			{
//				if (mark->callback[i].data)
//				{
//					if (!mark->callback[i].first)
//						IPC_freeDataElements(message_index[message_id].formatter, mark->callback[i].data);
//					IPC_unmarshallData(message_index[message_id].formatter, buffer + sizeof(int) + sizeof(int), mark->callback[i].data,
//							mark->callback[i].message_size);
//					mark->callback[i].first = 0;
//				}
//				n = mark->num_callbacks;
//				if (mark->callback[i].handler)
//					mark->callback[i].handler(mark->callback[i].data);
//				if (mark->num_callbacks >= n)
//					i++;
//			}
//		}
//		return timestamp;
//	}
//	else
//		carmen_die("Error: could not parse logfile.\n");
//	return -1;
//}


int
main(int argc, char **argv)
{
	if (argc != 2)
		return(printf("Usage: print_binary_log <binary_log_file_name>\n"));

	carmen_ipc_initialize(argc, argv);

	infile = carmen_fopen(argv[1], "r");

//	do
//	{
//		carmen_blogfile_handle_one_message();
//	} while (!carmen_feof(infile));

	printf("\n\nLogfile complete.\n");

	return (0);
}
