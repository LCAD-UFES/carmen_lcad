#include <locale.h>
#include <signal.h>
#include <vector>
#include <carmen/carmen.h>
#include <carmen/rddf_interface.h>

using namespace std;

static char *annotation_filename;
vector<carmen_rddf_add_annotation_message> annotation_queue;

void
carmen_annotation_manager_shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		fprintf(stderr, "\nAnnotation Manager Disconnecting...\n");
		exit(0);
	}
}


void
publish_new_annotation(carmen_rddf_add_annotation_message new_annotation)
{
	// to interface updates
	carmen_rddf_publish_annotation_message(new_annotation.annotation_point, new_annotation.annotation_orientation, new_annotation.annotation_description, new_annotation.annotation_type, new_annotation.annotation_code);
}


int
is_a_valid_point(carmen_rddf_add_annotation_message a)
{
	// Tiago disse que quando o ponto clicado esta no infinito, ele retorna 0.0
	if ((a.annotation_point.x == 0.0) && (a.annotation_point.y == 0.0) && (a.annotation_point.z == 0.0))
		return 0;

	return 1;
}


int
is_the_same_point(carmen_rddf_add_annotation_message a, carmen_rddf_add_annotation_message b)
{
	double delta_x, delta_y, delta_z, dist;

	delta_x = a.annotation_point.x - b.annotation_point.x;
	delta_y = a.annotation_point.y - b.annotation_point.y;
	delta_z = a.annotation_point.z - b.annotation_point.z;
	dist = sqrt(pow(delta_x, 2) + pow(delta_y, 2) + pow(delta_z, 2));

	if (dist < 1.0)
		return 1;

	return 0;
}


int
annotation_already_added(carmen_rddf_add_annotation_message new_annotation)
{
	uint i;

	for (i = 0; i < annotation_queue.size(); i++)
		if (is_the_same_point(new_annotation, annotation_queue[i]))
			return 1;

	return 0;
}


void
carmen_rddf_add_annotation_handler(carmen_rddf_add_annotation_message *message)
{
	carmen_rddf_add_annotation_message new_annotation;

	new_annotation.annotation_type = message->annotation_type;
	new_annotation.annotation_orientation = message->annotation_orientation;
	new_annotation.annotation_code = message->annotation_code;
	new_annotation.annotation_point = message->annotation_point;
	new_annotation.timestamp = message->timestamp;

	new_annotation.host = (char *) calloc(strlen(message->host), sizeof (char));
	new_annotation.annotation_description = (char *) calloc(strlen(message->annotation_description), sizeof (char));

	strcpy(new_annotation.host, message->host);
	strcpy(new_annotation.annotation_description, message->annotation_description);

	if (!annotation_already_added(new_annotation) && is_a_valid_point(new_annotation))
	{
		annotation_queue.push_back(new_annotation);
		publish_new_annotation(new_annotation);
	}
	else
	{
		free(new_annotation.host);
		free(new_annotation.annotation_description);
	}
}


void
carmen_annotation_manager_timer_handler(char *annotation_filename)
{
	uint i;
	FILE *f;
    IPC_RETURN_TYPE err;

	f = fopen(annotation_filename, "w");

	if (f == NULL)
		exit(printf("Unable to open the file '%s'\n", annotation_filename));

	//printf("annotation_queue.size(): %ld\n", annotation_queue.size());

	for (i = 0; i < annotation_queue.size(); i++)
	{
		char *description = rddf_get_annotation_description_by_type(annotation_queue[i].annotation_type);

		if (strlen(description) <= 0)
			continue;

		fprintf(f, "%s\t%d\t%d\t%lf\t%lf\t%lf\t%lf\n",
				description,
				annotation_queue[i].annotation_type,
				annotation_queue[i].annotation_code,
				annotation_queue[i].annotation_orientation,
				annotation_queue[i].annotation_point.x,
				annotation_queue[i].annotation_point.y,
				annotation_queue[i].annotation_point.z
		);

		annotation_queue[i].timestamp = carmen_get_time() + i;
		err = IPC_publishData(CARMEN_RDDF_ANNOTATION_MESSAGE_NAME, &(annotation_queue[i]));
		carmen_test_ipc_exit(err, "Could not publish", CARMEN_RDDF_ANNOTATION_MESSAGE_FMT);
	}

	fclose(f);
}


void
carmen_annotation_manager_subscribe_messages()
{
	carmen_rddf_subscribe_add_annotation_message(NULL,
			(carmen_handler_t) carmen_rddf_add_annotation_handler,
			CARMEN_SUBSCRIBE_ALL);
}


void
carmen_annotation_manager_initialize()
{
	// save the annotation file every second to avoid data loss
	carmen_ipc_addPeriodicTimer(1.0, (TIMER_HANDLER_TYPE) carmen_annotation_manager_timer_handler,
			annotation_filename);
}


void
carmen_annotation_manager_load_annotations()
{
	FILE *f = fopen(annotation_filename, "r");

	/* file already exists */
	if (f != NULL)
	{
		while (!feof(f))
		{
			carmen_rddf_add_annotation_message message;
			message.annotation_description = (char *) calloc(64, sizeof (char));

			fscanf(f, "%s\t%d\t%d\t%lf\t%lf\t%lf\t%lf\n",
					message.annotation_description,
					&message.annotation_type,
					&message.annotation_code,
					&message.annotation_orientation,
					&message.annotation_point.x,
					&message.annotation_point.y,
					&message.annotation_point.z
			);

			message.host = carmen_get_host();
			message.timestamp = carmen_get_time();
			annotation_queue.push_back(message);
			//publish_new_annotation(message);
		}

		fclose(f);
	}

	printf("Num annotations load: %ld\n", annotation_queue.size());
}


int
main(int argc, char **argv)
{
	if (argc < 2)
		exit(printf("Use %s <filename>\n", argv[0]));

	annotation_filename = argv[1];

	setlocale(LC_ALL, "C");
	signal(SIGINT, carmen_annotation_manager_shutdown_module);

	carmen_ipc_initialize(argc, argv);
	carmen_rddf_define_messages();
	carmen_annotation_manager_load_annotations();
	carmen_annotation_manager_initialize();
	carmen_annotation_manager_subscribe_messages();

	carmen_ipc_dispatch();
	return (0);
}

