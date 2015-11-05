/*********************************************************
	---   Skeleton Module Application ---
 **********************************************************/

#include <carmen/carmen.h>
#include <carmen/facial_greeting_interface.h>
#include <carmen/facial_greeting_interface.h>
#include <carmen/base_ackerman_interface.h>
#include <string.h>
#include <espeak/speak_lib.h>


/*********************************************************
		   --- Publishers ---
 **********************************************************/


/*********************************************************
		   --- Handlers ---
 **********************************************************/

int abriu = 0;
int fechou = 1;

espeak_AUDIO_OUTPUT output;
int Buflength = 500;
char *path = NULL;
int Options = 0;
char Voice[] = {"pt+f3"};

void
speak_to_user(char * message)
{
	void* user_data = NULL; 
	unsigned int Size, position = 0, end_position = 0, flags = espeakCHARS_AUTO, *unique_identifier = 0;

	espeak_POSITION_TYPE position_type = POS_CHARACTER;

	Size = strlen(message) + 1;

	output = AUDIO_OUTPUT_PLAYBACK;

	espeak_Synth(message, Size, position, position_type, end_position, flags, unique_identifier, user_data);
}


char last_name[64];
double last_time = 0.0;

void
face_recog_handler(carmen_face_recog_message *message)
{
	char what_to_say[512];


	sprintf(what_to_say, "Seja bem vindo ao IARA, %s\n", message->name);

	//printf("%s\n", what_to_say);

	if (message)
	{	
		if((strcmp(message->name, last_name) != 0) && ((message->timestamp - last_time) > 10.0))
		{		
			last_time = message->timestamp;
			strcpy(last_name, message->name);
		//if(abriu && fechou)
		{
			speak_to_user(what_to_say);
			abriu = 0;
		}
		}
	}
}

static char getBitValue(unsigned short data, int bit_number)
{
	char bit;

	bit = (data & (0x01 << (bit_number - 1))) != 0x0;

	return bit;
}

void ford_escape_status_handler(carmen_ford_escape_status_message *message)
{
	int is_door_open = getBitValue(message->g_XGV_component_status, 8);

	if(is_door_open)
	{
		fechou  = 0;
		abriu 	= 1;
	}

	if(!is_door_open)
	{
		fechou = 1;
	}
}

void 
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("facial_greeting: disconnected.\n");

		exit(0);
	}
}

int 
main(int argc, char **argv) 
{
	/* Connect to IPC Server */
	carmen_ipc_initialize(argc, argv);

	/* Check the param server version */
	//carmen_param_check_version(argv[0]);

	/* Register shutdown cleaner handler */
	signal(SIGINT, shutdown_module);

	/* Define messages that your module publishes */
	carmen_facial_greeting_define_face_recog_message();

	/* Subscribe to sensor messages */

	carmen_ford_escape_subscribe_status_message(NULL, (carmen_handler_t) ford_escape_status_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_facial_greeting_subscribe_face_recog_message(NULL, (carmen_handler_t) face_recog_handler, CARMEN_SUBSCRIBE_LATEST);

	espeak_Initialize(output, Buflength, path, Options);
	espeak_SetVoiceByName(Voice);

	/* Loop forever waiting for messages */
	carmen_ipc_dispatch();

	return (0);
}
