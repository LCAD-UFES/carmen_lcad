#include <locale.h>
#include <signal.h>
#include <vector>
#include <carmen/carmen.h>
#include <carmen/rddf_interface.h>

using namespace std;


static char *annotation_filename;
vector<carmen_annotation_t> annotation_read_from_file;
carmen_rddf_annotation_message annotation_queue_message;


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
publish_annotation_queue(vector<carmen_annotation_t> &annotations_to_publish)
{
	IPC_RETURN_TYPE err;

	if (annotation_queue_message.annotations == NULL || annotation_queue_message.num_annotations == 0)
		annotation_queue_message.annotations = (carmen_annotation_t*) calloc (annotations_to_publish.size(), sizeof(carmen_annotation_t));
	else if (annotation_queue_message.num_annotations != (int) annotations_to_publish.size())
		annotation_queue_message.annotations = (carmen_annotation_t*) realloc (annotation_queue_message.annotations, annotations_to_publish.size() * sizeof(carmen_annotation_t));

	annotation_queue_message.num_annotations = annotations_to_publish.size();

	for (size_t i = 0; i < annotations_to_publish.size(); i++)
	{
		annotation_queue_message.annotations[i].annotation_code = annotations_to_publish[i].annotation_code;
		// it's ok copy the pointer here
		//if (annotation_queue_message.annotations[i].annotation_description == NULL)
			//annotation_queue_message.annotations[i].annotation_description = (char *) calloc (strlen(annotations_to_publish[i].annotation_description) + 1, sizeof(char));
		//strcpy(annotation_queue_message.annotations[i].annotation_description, annotations_to_publish[i].annotation_description);
		annotation_queue_message.annotations[i].annotation_description = annotations_to_publish[i].annotation_description;
		annotation_queue_message.annotations[i].annotation_orientation = annotations_to_publish[i].annotation_orientation;
		annotation_queue_message.annotations[i].annotation_point = annotations_to_publish[i].annotation_point;
		annotation_queue_message.annotations[i].annotation_type = annotations_to_publish[i].annotation_type;
	}

	annotation_queue_message.host = carmen_get_host();
	annotation_queue_message.timestamp = carmen_get_time();

	err = IPC_publishData(CARMEN_RDDF_ANNOTATION_MESSAGE_NAME, &annotation_queue_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_RDDF_ANNOTATION_MESSAGE_FMT);

}


int
is_a_valid_point(carmen_annotation_t a)
{
	// Tiago disse que quando o ponto clicado esta no infinito, ele retorna 0.0
	if ((a.annotation_point.x == 0.0) && (a.annotation_point.y == 0.0) && (a.annotation_point.z == 0.0))
		return 0;

	return 1;
}


int
is_the_same_point(carmen_annotation_t a, carmen_annotation_t b)
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
annotation_already_added(carmen_annotation_t new_annotation)
{
	uint i;

	for (i = 0; i < annotation_read_from_file.size(); i++)
		if (is_the_same_point(new_annotation, annotation_read_from_file[i]))
			return 1;

	return 0;
}


void
carmen_rddf_add_annotation_handler(carmen_rddf_add_annotation_message *message)
{
	carmen_annotation_t new_annotation;

	new_annotation.annotation_type = message->annotation_type;
	new_annotation.annotation_orientation = message->annotation_orientation;
	new_annotation.annotation_code = message->annotation_code;
	new_annotation.annotation_point = message->annotation_point;
	new_annotation.annotation_description = (char *) calloc (strlen(message->annotation_description) + 1, sizeof(char));
	strcpy(new_annotation.annotation_description, message->annotation_description);

	if (!annotation_already_added(new_annotation) && is_a_valid_point(new_annotation))
	{
		annotation_read_from_file.push_back(new_annotation);
		publish_annotation_queue(annotation_read_from_file);
	}
	else
	{
		free(new_annotation.annotation_description);
	}
}


void
carmen_annotation_manager_timer_handler(char *annotation_filename)
{
	uint i;
	FILE *f;

	f = fopen(annotation_filename, "w");

	if (f == NULL)
		exit(printf("Unable to open the file '%s'\n", annotation_filename));

	printf("annotation_queue.size(): %ld\n", annotation_read_from_file.size());

	for (i = 0; i < annotation_read_from_file.size(); i++)
	{
		char *description = rddf_get_annotation_description_by_type(annotation_read_from_file[i].annotation_type);

		if (strlen(description) <= 0)
			continue;

		fprintf(f, "%s\t%d\t%d\t%lf\t%lf\t%lf\t%lf\n",
				description,
				annotation_read_from_file[i].annotation_type,
				annotation_read_from_file[i].annotation_code,
				annotation_read_from_file[i].annotation_orientation,
				annotation_read_from_file[i].annotation_point.x,
				annotation_read_from_file[i].annotation_point.y,
				annotation_read_from_file[i].annotation_point.z
		);
	}

	publish_annotation_queue(annotation_read_from_file);
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
	memset(&annotation_queue_message, 0, sizeof(annotation_queue_message));

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
			carmen_annotation_t message;
			message.annotation_description = (char *) calloc(128, sizeof (char));

			int n = fscanf(f, "%s\t%d\t%d\t%lf\t%lf\t%lf\t%lf\n",
					message.annotation_description,
					&message.annotation_type,
					&message.annotation_code,
					&message.annotation_orientation,
					&message.annotation_point.x,
					&message.annotation_point.y,
					&message.annotation_point.z
			);

			if (n != 7)
				break;

			annotation_read_from_file.push_back(message);
		}

		fclose(f);
	}

	printf("Num annotations load: %ld\n", annotation_read_from_file.size());
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

