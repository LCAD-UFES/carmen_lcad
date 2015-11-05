#include <stdio.h>
#include <stdlib.h>
#include <locale.h>
#include <sys/types.h>
#include <dirent.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "neural_localizer_user_functions.h"
#include "neural_localizer_utils.h"

#include <carmen/carmen.h>
#include <carmen/neural_localizer_interface.h>
#include <carmen/localize_interface.h>

#ifndef PI
#define PI 3.1415926535
#endif

#define TOP			0
#define MIDDLE			1
#define BOTTON			2

#define DIRECTION_FORWARD	1
#define DIRECTION_REWIND	-1
#define NO_DIRECTION		0


//############### Global Variables ###############
int g_nStatus;
int g_time_shift = 0;

void *g_current_font_type = GLUT_STROKE_MONO_ROMAN;
DIR *g_data_set_dir = NULL;
DIR *g_image_set_dir = NULL;
DATA_SET *g_data_set = NULL;
IMAGE_SET *g_image_set = NULL;
int g_num_data_set_inputs = 0;
int g_num_image_set_inputs = 0;
int g_image_set_index = 0;
int LAST_SAMPLE = 1;

int go_stop_flag = 0;
int input_initialized = 0;
INPUT_DESC* input_desc = NULL;
carmen_map_t* input_map = NULL;
carmen_point_t robot_pose;

//############### User Functions ###############

void shift_input_to_right (INPUT_DESC *input, int num_cols)
{
	int i, j, w, h, org_col, dst_col;

	w = input->neuron_layer->dimentions.x;
	h = input->neuron_layer->dimentions.y;

	for(j = 0; j < (w - num_cols); j++)
	{
		dst_col = w - j - 1; // width - column - 1
		org_col = dst_col - num_cols;

		for(i = 0; i < h; i++)
		{
			input->neuron_layer->neuron_vector[dst_col + i * w].output.fval =
				input->neuron_layer->neuron_vector[org_col + i * w].output.fval;
		}
	}

	// clean the first num_cols columns (set to zero)
	for(i = 0; i < h; i++) {
		for(j = 0; j < num_cols; j++) {
			input->neuron_layer->neuron_vector[i * w + j].output.fval = 0;
		}
	}
}

//***********************************************************
//* Function: MakeInputImage
//* Description:
//***********************************************************
void
MakeInputImage (INPUT_DESC *input, int w, int h)
{
	char message[256];

	//texture frame width and height, must be powers of 2
	input->tfw = nearest_power_of_2 (w);
	input->tfh = nearest_power_of_2 (h);

	//window width and height
	input->ww = w;
	input->wh = h;

	switch(TYPE_SHOW)
	{
		case SHOW_FRAME:
			//visible part (of the window) width and height
			input->vpw = input->neuron_layer->dimentions.x;
			input->vph = input->neuron_layer->dimentions.y;
			break;
		case SHOW_WINDOW:
			input->vph = h;
			input->vpw = w;
			break;
		default:
			sprintf(message,"%d. It can be SHOW_FRAME or SHOW_WINDOW.",TYPE_SHOW);
			Erro ("Invalid Type Show ", message, " Error in update_input_image.");
			return;
	}
	//origin x and y of the input window
	input->vpxo = 0;
	input->vpyo = h - input->vph;

	if (input->image == NULL)
	{
		input->image = (GLubyte *) alloc_mem (input->tfw * input->tfh * 3 * sizeof (GLubyte));
	}
}

int
get_current_input_number(char *input_name, char *caller_function_name)
{
	int current_input;

	current_input = 0;
	while ((strcmp(g_data_set[current_input].input_name, input_name) != 0) && (current_input < g_num_data_set_inputs))
		current_input++;

	if (current_input == g_num_data_set_inputs)
	{
		Erro("Could not find input named ", input_name, caller_function_name);
		return (0);
	}
	return (current_input);
}



/*
***********************************************************
* Function: get_next_sentence
* Description:
* Inputs: nDirection
***********************************************************
*/

int
get_next_sentence(INPUT_DESC *input, int nDirection)
{
	int current_input;
	int forward = 0;

	current_input = get_current_input_number(input->name, " in get_next_sentence()");

	if (nDirection != NO_DIRECTION)
	{
		forward = (nDirection == DIRECTION_FORWARD)? 1:0;

		if (g_data_set[current_input].input_index >= (g_data_set[current_input].num_input_sentences - 1))
		{
			if (forward)
				g_data_set[current_input].input_index = 0;
			else
				g_data_set[current_input].input_index -= 1;
		}
		else if (g_data_set[current_input].input_index == 0)
		{
			if (forward)
				g_data_set[current_input].input_index += 1;
			else
				g_data_set[current_input].input_index = g_data_set[current_input].num_input_sentences - 1;
		}
		else
		{
			if (forward)
				g_data_set[current_input].input_index += 1;
			else
				g_data_set[current_input].input_index -= 1;
		}
	}
	return (g_data_set[current_input].input_index);
}



FILE *
get_file(char* directory, char *base_file_name, char *file_type)
{
	FILE *data_set_file;
	char file_name[1000];

	strcpy(file_name, directory);
	strcat(file_name, base_file_name);
	strcat(file_name, file_type);
	if ((data_set_file = fopen(file_name, "r")) == NULL)
	{
		Erro("Could not open file: ", file_name, "");
		return (NULL);
	}
	return (data_set_file);
}



int
is_not_empty_line(char *line)
{
	unsigned int i;

	for (i = 0; i < strlen(line); i++)
		if (isgraph(line[i]))
			return (1);
	return (0);
}



char *
get_valid_line(char *line, FILE *data_set_file)
{
	while (fgets(line, 900, data_set_file) != NULL)
	{
		if (line[0] == '$')
			return (NULL);
		if (line[0] == '#')
			continue;
		if (is_not_empty_line(line))
		{
			if (line[strlen(line)-1] == '\n')
				line[strlen(line)-1] = '\0';
			return (line);
		}
	}
	return (NULL);
}

int
count_image_sets(FILE *data_set_file)
{
	int num_data_sets;
	char line[1000];

	num_data_sets = 0;
	while (fgets(line, 900, data_set_file) != NULL)
		num_data_sets++;

	rewind(data_set_file);

	return (num_data_sets);
}

void
init_g_image_set(int num_inputs)
{
	int i;

	for (i = 0; i < num_inputs; i++)
	{
		g_image_set[i].image_index = 0;
	}
}

int
count_data_sets(FILE *data_set_file)
{
	int num_data_sets;
	char line[1000];

	num_data_sets = 0;
	while (fgets(line, 900, data_set_file) != NULL)
	{
		if (line[0] == '$')
			num_data_sets++;
	}

	rewind(data_set_file);

	return (num_data_sets);
}

int
count_input_sentences(FILE *data_set_file, char *input_name)
{
	int num_sentences;
	char line[1000];

	while (fgets(line, 900, data_set_file) != NULL)
	{
		if ((line[0] == '$') && (strncmp(line+1, input_name, strlen(input_name)) == 0))
			break;
	}

	if (line[0] != '$')
		return (0);

	num_sentences = 0;
	while (get_valid_line(line, data_set_file) != NULL)
		num_sentences++;

	rewind(data_set_file);

	return (num_sentences);
}


void
read_input_sentences(FILE *data_set_file, DATA_SET *data_set)
{
	int current_sentence;
	char line[1000];

	while (fgets(line, 900, data_set_file) != NULL)
	{
		if ((line[0] == '$') && (strncmp(line+1, data_set->input_name, strlen(data_set->input_name)) == 0))
			break;
	}

	if (line[0] != '$')
	{
		Erro("Could not read data_set from input named: ", data_set->input_name, " in read_input_sentences().");
		return;
	}

	data_set->input_sentences = (char **) alloc_mem(data_set->num_input_sentences * sizeof(char *));
	current_sentence = 0;
	while (get_valid_line(line, data_set_file) != NULL)
	{
		data_set->input_sentences[current_sentence] = (char *) alloc_mem((strlen(line) + 1) * sizeof(char));
		strcpy(data_set->input_sentences[current_sentence], line);
#ifdef	VERBOSE
		printf("input: %s, sentence number: %d, sentence: %s\n", data_set->input_name, current_sentence, data_set->input_sentences[current_sentence]);
#endif
		current_sentence++;
	}
}

void
init_g_data_set(int num_inputs)
{
	int i;

	for (i = 0; i < num_inputs; i++)
	{
		g_data_set[i].input_name = NULL;
		g_data_set[i].input_sentences = NULL;
		g_data_set[i].input_index = 0;
		g_data_set[i].num_input_sentences = 0;
	}
}


int
read_train_data_set_data(char *base_file_name, INPUT_DESC *input)
{
	FILE *data_set_file;
	int current_input;

	char directory[1024];
	char* carmen_home = getenv("CARMEN_HOME");
	sprintf(directory, "%s/src/neural_localizer/%s", carmen_home, DATA_PATH);

	data_set_file = get_file(directory, base_file_name, ".txt");

	if (g_data_set == NULL)
	{
		g_num_data_set_inputs = count_data_sets(data_set_file);
		g_data_set = (DATA_SET *) alloc_mem(g_num_data_set_inputs * sizeof(DATA_SET));
		init_g_data_set(g_num_data_set_inputs);
	}

	current_input = 0;
	while (g_data_set[current_input].input_name != NULL)
		current_input++;

	g_data_set[current_input].input_name = input->name;
	g_data_set[current_input].num_input_sentences = count_input_sentences(data_set_file, input->name);
	read_input_sentences(data_set_file, &(g_data_set[current_input]));

	fclose(data_set_file);

	return (g_data_set[current_input].num_input_sentences);
}


char *
get_next_train_data_set(INPUT_DESC *input)
{
	struct dirent *train_data_set_file_entry;
	char base_file_name[1000];
	char *aux;
	int num_samples;

	do
	{
		train_data_set_file_entry = readdir(g_data_set_dir);
		if (train_data_set_file_entry == NULL)
			return (NULL);
		aux = strrchr(train_data_set_file_entry->d_name, '.');
	} while (strcmp(aux, ".txt") != 0);

	strcpy(base_file_name, train_data_set_file_entry->d_name);

	aux = strrchr(base_file_name, '.');
	aux[0] = '\0';

	LAST_SAMPLE = num_samples = read_train_data_set_data(base_file_name, input);
	printf("# Data set: %s, Input: %s, num_samples: %d\n", train_data_set_file_entry->d_name, input->name, num_samples);

	return (train_data_set_file_entry->d_name);
}


void
init_input_data_set(INPUT_DESC *input)
{
	char *data_set_name;

	char directory[1024];
	char* carmen_home = getenv("CARMEN_HOME");
	sprintf(directory, "%s/src/neural_localizer/%s", carmen_home, DATA_PATH);

	if ((g_data_set_dir = opendir(directory)) == NULL)
	{
		show_message("Could not open data set directory named: ", directory, "");
		return;
	}

	if ((data_set_name = get_next_train_data_set(input)) == NULL)
	{
		show_message("Could not initialize data set from directory:", directory, "");
		exit(1);
	}
}



/*
***********************************************************
* Function: make_input_image_nc
* Description:
* Inputs:
* Output: none
***********************************************************
*/

void
make_input_image_nc(INPUT_DESC *input, int w, int h)
{
	char message[256];
	int i;

	input->tfw = nearest_power_of_2(w);
	input->tfh = nearest_power_of_2(h);

	input->ww = w;
	input->wh = h;

	switch(TYPE_SHOW)
	{
		case SHOW_FRAME:
			input->vpw = input->neuron_layer->dimentions.x;
			input->vph = input->neuron_layer->dimentions.y;
			break;
		case SHOW_WINDOW:
			input->vph = h;
			input->vpw = w;
			break;
		default:
			sprintf(message,"%d. It can be SHOW_FRAME or SHOW_WINDOW.",TYPE_SHOW);
			Erro("Invalid Type Show ", message, " Error in make_input_image_nc().");
			return;
	}

	input->vpxo = 0;
	input->vpyo = h - input->vph;

	if(input->image == NULL)
	{
		input->image = (GLubyte *) alloc_mem(input->tfw * input->tfh * 3 * sizeof(GLubyte));
		for (i = 0; i < input->tfw * input->tfh * 3; i++)
			input->image[i] = 0;
	}
}


int
set_network_status_interface (int net_status)
{
	switch (net_status)
	{
		case TRAINING_PHASE:
			break;
		case RECALL_PHASE:
			break;
		default:
			printf ("Error: invalid Net Status '%d' (SetNetworkStatus).\n", g_nStatus);
			return (-1);
	}

	g_nStatus = net_status;
	return (0);
}

void
shutdown_module(int signo)
{
  if (signo == SIGINT)
  {
    carmen_ipc_disconnect();
    printf("neural_localizer: disconnected.\n");

    exit(0);
  }
}

void Timer(int value)
{
    IPC_handleMessage(0);
    glutTimerFunc(10,Timer, 1);
}

void publish_neural_place_command(OUTPUT_DESC* out_nl_place_command)
{
  IPC_RETURN_TYPE err;

	carmen_neural_localizer_place_command_message message;
	message.image = (unsigned char*) malloc (3 * NL_WIDTH * NL_HEIGHT/2 * sizeof(unsigned char));

	message.heigth = NL_HEIGHT/2;
	message.width = NL_WIDTH;
	message.image_size = message.heigth * message.width * 3;

	int i, j, y;

	for(i = 0; i < NL_WIDTH; i++)
	{
		for(j=0, y = NL_HEIGHT/2 - 1; j < NL_HEIGHT/2; j++, y--)
		{
			message.image[3 * (j * NL_WIDTH + i) + 0] = (unsigned char) out_nl_place_command->image[3 * (y * NL_WIDTH + i) + 0];
			message.image[3 * (j * NL_WIDTH + i) + 1] = (unsigned char) out_nl_place_command->image[3 * (y * NL_WIDTH + i) + 1];
			message.image[3 * (j * NL_WIDTH + i) + 2] = (unsigned char) out_nl_place_command->image[3 * (y * NL_WIDTH + i) + 2];
		}
	}

  err = IPC_publishData(CARMEN_NEURAL_LOCALIZER_PLACE_COMMAND_NAME, &message);
  carmen_test_ipc_exit(err, "Could not publish", CARMEN_NEURAL_LOCALIZER_PLACE_COMMAND_FMT);
}

void crop_carmen_input_map_to_input_layer(carmen_map_t* input_map, INPUT_DESC *input, carmen_point_t pose)
{
	int r, g, b;
	int init_crop_x, init_crop_y;
	int i, j, x, y;
	float value;

	carmen_position_t pose_position_in_pixels;

	IplImage* image_map = cvCreateImage(cvSize(INPUT_WIDTH/2, INPUT_HEIGHT/2), IPL_DEPTH_8U, 3);
	IplImage* image_map_resized = cvCreateImage(cvSize(INPUT_WIDTH, INPUT_HEIGHT), IPL_DEPTH_8U, 3);

	pose_position_in_pixels.x = pose.x / (input_map->config.resolution);
	pose_position_in_pixels.y = pose.y / (input_map->config.resolution);

	init_crop_x = pose_position_in_pixels.x - INPUT_WIDTH/4;
	init_crop_y = pose_position_in_pixels.y - INPUT_HEIGHT/4;

	if(init_crop_x < 0)
		init_crop_x = 0;
	if(init_crop_x > input_map->config.x_size - 1)
		init_crop_x = input_map->config.x_size -1;

	if(init_crop_y < 0)
			init_crop_y = 0;
	if(init_crop_y > input_map->config.y_size - 1)
		init_crop_y = input_map->config.y_size -1;

	for(i = init_crop_x, x=0; i < (init_crop_x + INPUT_WIDTH/2); i++, x++)
	{
		for(j = init_crop_y, y=0; j < (init_crop_y + INPUT_HEIGHT/2); j++, y++)
		{

			if(input_map != NULL)
			{
				value = input_map->map[i][j];

				if(value == 0)
					r = g = b = 255;
				else if(value == -1)
				{
					r = g = 0;
					b = 255;
				}
				else
				{
					r = 255 - (value * 255);
					g = 255 - (value * 255);
					b = 255 - (value * 255);
				}

				image_map->imageData[3 * (y * (INPUT_WIDTH/2) + x) + 0] = r;
				image_map->imageData[3 * (y * (INPUT_WIDTH/2) + x) + 1] = g;
				image_map->imageData[3 * (y * (INPUT_WIDTH/2) + x) + 2] = b;
			}
		}
	}

	cvResize(image_map, image_map_resized, CV_INTER_LINEAR);

	for(i = 0; i < image_map_resized->width; i++)
	{
		for(j=0; j < image_map_resized->height; j++)
		{
			input->image[3 * (j * input->tfw + i) + 0] = (unsigned char) image_map_resized->imageData[3 * (j * input->tfw + i) + 0];
			input->image[3 * (j * input->tfw + i) + 1] = (unsigned char) image_map_resized->imageData[3 * (j * input->tfw + i) + 1];
			input->image[3 * (j * input->tfw + i) + 2] = (unsigned char) image_map_resized->imageData[3 * (j * input->tfw + i) + 2];
		}
	}

	check_input_bounds (input, input->wx + input->ww/2, input->wy + input->wh/2);
	input->up2date = 0;
	update_input_neurons (input);
	update_input_image (input);

}

static void navigator_stop_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
				   void *clientData __attribute__ ((unused)))
{
	go_stop_flag = 0;
}


static void navigator_go_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
				 void *clientData __attribute__ ((unused)))
{
	go_stop_flag = 1;
}


void carmen_localizer_handler(carmen_localize_globalpos_message* message)
{
	robot_pose.x =  message->globalpos.x;
	robot_pose.y = message->globalpos.y;
	robot_pose.theta =	message->globalpos.theta;

	if(input_initialized)
	{
		crop_carmen_input_map_to_input_layer(input_map, input_desc, robot_pose);

		all_filters_update();
		all_outputs_update();

		if(go_stop_flag)
		{
			all_dendrites_update();
			all_neurons_update();

			publish_neural_place_command(&out_nl_place_command);
		}

		printf("robot pos - x: %6.2f, y: %6.2f, theta: %6.2f\n", message->globalpos.x, message->globalpos.y, message->globalpos.theta);
	}
}


void
init_neural_localizer (INPUT_DESC *input)
{
#ifndef NO_INTERFACE
	int x, y;
#endif

  IPC_RETURN_TYPE err;

	/* Connect to IPC Server */
	carmen_ipc_initialize(global_argc, global_argv);

	/* Check the param server version */
	carmen_param_check_version(global_argv[0]);

	/* Register shutdown cleaner handler */
	signal(SIGINT, shutdown_module);

	if(input_map == NULL)
		input_map = (carmen_map_p)calloc(1, sizeof(carmen_map_t));
  carmen_test_alloc(input_map);

  /* initial map */
  carmen_map_get_gridmap(input_map);

	/* Define messages that your module publishes */
  carmen_neural_localizer_define_messages();

	/* Subscribe to sensor messages */
  carmen_localize_subscribe_globalpos_message(NULL, (carmen_handler_t) carmen_localizer_handler, CARMEN_SUBSCRIBE_LATEST);

  err = IPC_subscribe(CARMEN_NAVIGATOR_GO_NAME, navigator_go_handler, NULL);
  carmen_test_ipc_exit(err, "Could not subcribe message",
		       CARMEN_NAVIGATOR_GO_NAME);
  IPC_setMsgQueueLength(CARMEN_NAVIGATOR_GO_NAME, 1);

  err = IPC_subscribe(CARMEN_NAVIGATOR_STOP_NAME, navigator_stop_handler,
		      NULL);
  carmen_test_ipc_exit(err, "Could not subcribe message",
		       CARMEN_NAVIGATOR_STOP_NAME);
  IPC_setMsgQueueLength(CARMEN_NAVIGATOR_STOP_NAME, 1);

	g_nStatus = TRAINING_PHASE;
	set_network_status_interface(g_nStatus);

	MakeInputImage (input, INPUT_WIDTH, INPUT_HEIGHT);

	input->up2date = 0;
	update_input_neurons (input);

#ifndef NO_INTERFACE
	glutInitWindowSize (input->ww, input->wh);

	if (read_window_position (input->name, &x, &y))
	{
		glutInitWindowPosition (x, y);
	}
	else
	{
		glutInitWindowPosition (-1, -1);
	}
	input->win = glutCreateWindow (input->name);

	glGenTextures (1, (GLuint *)(&(input->tex)));
	input_init (input);
	glutReshapeFunc (input_reshape);
	glutDisplayFunc (input_display);
	glutKeyboardFunc (keyboard);
  glutTimerFunc(10,Timer, 1);
	glutPassiveMotionFunc (input_passive_motion);
	glutMouseFunc (input_mouse);
#endif
}


void
get_text_lines_content(char *text_line1, char *text_line2, char *text_line3, char *input_string)
{
	int i, j;

	j = i = 0;
	while (input_string[i] != '|')
	{
		text_line1[j] = input_string[i];
		j++; i++;
	}
	text_line1[j] = '\0'; i++;

	j = 0;
	while (input_string[i] != '|')
	{
		text_line2[j] = input_string[i];
		j++; i++;
	}
	text_line2[j] = '\0'; i++;

	j = 0;
	while ((input_string[i] != '\0') && (input_string[i] != '#'))
	{
		text_line3[j] = input_string[i];
		j++; i++;
	}
	text_line3[j] = '\0';
}


void
nc_input_reshape (int w, int h)
{
	INPUT_DESC *input;
	double winput, hinput, d, ang;

	input = get_input_by_win (glutGetWindow ());
	winput = (GLdouble) input->vpw;
	hinput = (GLdouble) input->vph;
	d = sqrt(winput*winput + hinput*hinput);
	ang = 2.5 * ((atan2 (hinput/2.0, d) * 180.0)/PI);
	glViewport (0, 0, w, h);
	glMatrixMode (GL_PROJECTION);
	glLoadIdentity ();
	gluPerspective (ang, (GLfloat)w/(GLfloat)h, 100.0*d, d);
}


void
copy_input_image_to_input_neurons (INPUT_DESC *input)
{

	int p_nViewport[4];
	int nWidth, nHeight;
	static int nPreviusWidth = 0, nPreviusHeight = 0;
	int xi, yi, wi, hi, xo, yo, wo, ho, r, g, b;
	static GLubyte *pScreenPixels = (GLubyte *) NULL;
	NEURON *neuron_vector;
	float k;

	glutSetWindow(input->win);

	glGetIntegerv (GL_VIEWPORT, p_nViewport);
	nWidth = p_nViewport[2];
	nHeight = p_nViewport[3];

	if ((nWidth != nPreviusWidth) || (nHeight != nPreviusHeight))
	{
		free (pScreenPixels);
		if ((pScreenPixels = (GLubyte *) malloc (3 * nWidth * nHeight * sizeof (GLubyte))) == (GLubyte *) NULL)
		{
			Erro ("Cannot allocate more memory", "", "");
			return;
		}
		nPreviusWidth = nWidth;
		nPreviusHeight = nHeight;
	}

	glReadBuffer (GL_BACK);
	glEnable(GL_READ_BUFFER);
	glReadPixels(0, 0, nWidth, nHeight, GL_RGB, GL_UNSIGNED_BYTE, pScreenPixels);
	glDisable(GL_READ_BUFFER);

	wi = nWidth;
	hi = nHeight;

	neuron_vector = input->neuron_layer->neuron_vector;
	wo = input->neuron_layer->dimentions.x;
	ho = input->neuron_layer->dimentions.y;

	k = (float) wi / (float) wo;

	for (yo = 0; yo < ho; yo++)
	{
		yi = (int) (k * (float) yo + .5f);
		for (xo = 0; xo < wo; xo++)
		{
			xi = (int) (k * (float) xo + .5f);
			if ((xi >= 0) && (xi < wi) && (yi >= 0) && (yi < hi))
			{
				r = pScreenPixels[3 * (yi * wi + xi) + 0];
				g = pScreenPixels[3 * (yi * wi + xi) + 1];
				b = pScreenPixels[3 * (yi * wi + xi) + 2];
			}
			else
				r = g = b = 0;

			switch (input->neuron_layer->output_type)
			{
			case COLOR:
				neuron_vector[yo * wo + xo].output.ival = PIXEL(r, g, b);
				break;
			case GREYSCALE:
				neuron_vector[yo * wo + xo].output.ival = (r + g + b) / 3;
				break;
			case BLACK_WHITE:
				neuron_vector[yo * wo + xo].output.ival = r > 50? NUM_COLORS - 1: 0;
				break;
			case GREYSCALE_FLOAT:
				neuron_vector[yo * wo + xo].output.fval = (float) (r + g + b) / 3.0;
				break;
			}
		}
	}
}

void
draw_text_into_input(char *text_line, int position, GLdouble w, GLdouble h, GLdouble d)
{
	unsigned int i;
	float x, y, z;
	
	x = y = z = 0.0f;

	x = -((GLdouble) strlen (text_line)) * 104.76/2.3;

	switch (position)
	{
		case TOP: 		y = +3.9 * (119.05/3.0) + 10.0; break;
		case MIDDLE: 	y =       -(119.05/3.0) + 20.0; break;
		case BOTTON: 	y = -5.5 * (119.05/3.0) + 10.0; break;
	}

	z = -8 * d;

	glPushMatrix();

	glScalef (w / (0.47*(104.76 / (119.05 + 25.0)) * ((GLdouble) strlen (text_line)) * h), 0.8, 1.0);

	glTranslatef(x, y, z);

	glRasterPos3f(x, y, z);

	for (i = 0; i < strlen (text_line); i++)
		glutStrokeCharacter (g_current_font_type, text_line[i]);

	glPopMatrix();
}


void
draw_attention_control_into_input(char *text_line, int position, GLfloat w, GLfloat h)
{
	float red = 0.0, blue = 0.0;
	GLfloat x0, x1, y0, y1;
	
	x0 = x1 = y0 = y1 = 0.0f;

	if (strchr(text_line, 'u') != NULL)
		red = 1.0;
	if (strchr(text_line, 'd') != NULL)
		blue = 1.0;
		
	x0 = 0;
	x1 = w;
	
	switch (position)
	{
		case TOP:
			y0 = 2.0 * h / 3.0;
			y1 = h;
			break;
		case MIDDLE:
			y0 = 1.0 * h / 3.0;
			y1 = 2.0 * h / 3.0;
			break;
		case BOTTON:
			y0 = 0.0;
			y1 = 1.0 * h / 3.0;
			break;
	}


	glEnable(GL_BLEND);
	glBlendFunc(GL_ONE_MINUS_DST_COLOR, GL_DST_ALPHA);
	// Draw a filled square.
	glColor3f (red, 0.0, blue);
	glBegin(GL_POLYGON);
	glVertex2f(x0, y0);
	glVertex2f(x1, y0);
	glVertex2f(x1, y1);
	glVertex2f(x0, y1);
	glEnd();
}


void
draw_text(INPUT_DESC *input, GLdouble w, GLdouble h, GLdouble d)
{
	char text_line1[200];
	char text_line2[200];
	char text_line3[200];
	char *attention_control;

	attention_control = strchr(input->string, '$');
	attention_control[0] = '\0';
	get_text_lines_content(text_line1, text_line2, text_line3, input->string);
	attention_control[0] = '$';
	attention_control++;

	draw_text_into_input(text_line1, TOP, w, h, d);
	draw_text_into_input(text_line2, MIDDLE, w, h, d);
	draw_text_into_input(text_line3, BOTTON, w, h, d);

	get_text_lines_content(text_line1, text_line2, text_line3, attention_control);
	draw_attention_control_into_input(text_line1, TOP,    (GLfloat) input->neuron_layer->dimentions.x, (GLfloat) input->neuron_layer->dimentions.y);
	draw_attention_control_into_input(text_line2, MIDDLE, (GLfloat) input->neuron_layer->dimentions.x, (GLfloat) input->neuron_layer->dimentions.y);
	draw_attention_control_into_input(text_line3, BOTTON, (GLfloat) input->neuron_layer->dimentions.x, (GLfloat) input->neuron_layer->dimentions.y);
}


void
nc_input_display (void)
{
	INPUT_DESC *input;
	GLdouble w, h, d;

	input = get_input_by_win (glutGetWindow ());

	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity ();
	w = (GLdouble) input->vpw;
	h = (GLdouble) input->vph;
	d = sqrt(w*w + h*h);
	gluLookAt (0.0, 0.0, 0.0,
		   0.0, 0.0, -d,
		   0.0, 1.0, 0.0);
	glScalef (sqrt(1.6), sqrt(1.6), 1.0);
	glTranslatef (-w/2.0, -h/2.0, -d);

	glClear (GL_COLOR_BUFFER_BIT);
	glColor3f (0.0, 1.0, 0.0);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_LINE_SMOOTH);
	glLineWidth(d/5.0);

	draw_text(input, w, h, d);
	copy_input_image_to_input_neurons(input);

	glutSwapBuffers ();

	input->waiting_redisplay = 0;
}



/*
***********************************************************
* Function: read_input_sentence
* Description: Writes a set of returns into the MAE input
* Inputs: input - input image
* Output: 0 if OK, -1 otherwise
***********************************************************
*/

void
read_input_sentence(INPUT_DESC *input, int sentence_number)
{
	int current_input;

	current_input = get_current_input_number(input->name, " in read_input_sentence()");
	input->string = g_data_set[current_input].input_sentences[sentence_number];
	glutSetWindow(input->win);
	nc_input_display();
}

void
initialize_data_set(INPUT_DESC *input)
{
	int sentence_number;

	init_input_data_set(input);

	sentence_number = get_next_sentence(input, NO_DIRECTION);
	read_input_sentence(input, sentence_number);
}


/*
***********************************************************
* Function: init_nc_input
* Description:
* Inputs:input - Neuron Layer de input
* Output: none
***********************************************************
*/

void
init_nc_input(INPUT_DESC *input)
{
#ifndef NO_INTERFACE
	int x, y;
#endif

	make_input_image_nc(input, 3*NL_WIDTH, 3*NL_HEIGHT/2);

	input->up2date = 0;

#ifndef NO_INTERFACE
	glutInitWindowSize(input->ww, input->wh);
	if (read_window_position(input->name, &x, &y))
		glutInitWindowPosition(x, y);
	else
		glutInitWindowPosition(-1, -1);
	input->win = glutCreateWindow(input->name);

	glGenTextures(1, (GLuint *)(&(input->tex)));
	input_init(input);
	glutReshapeFunc(nc_input_reshape);
	glutDisplayFunc(nc_input_display);
	glutKeyboardFunc(keyboard);
	glutPassiveMotionFunc(input_passive_motion);
	glutMouseFunc(input_mouse);
#endif
}


//***********************************************************
//* Function: init_user_functions
//* Description:
//***********************************************************
int
init_user_functions (void)
{
	char strCommand[128];

	sprintf (strCommand, "toggle move_active;");
	interpreter (strCommand);

	sprintf (strCommand, "toggle draw_active;");
	interpreter (strCommand);

	return (0);
}

int
ReadFrameInput (INPUT_DESC *input, char *strFileName)
{
	int y, x, h, w;
	int r, g, b;
	uchar *image_pixel;
	IplImage *img = NULL;

	/* Clear image */
	for (y = 0; y <= input->tfw * input->tfh * 3; y++)
		input->image[y] = (GLubyte) 0;

	if (input->neuron_layer->output_type == COLOR)
		img = cvLoadImage(strFileName, CV_LOAD_IMAGE_COLOR);
	else if (input->neuron_layer->output_type == GREYSCALE)
		img = cvLoadImage(strFileName, CV_LOAD_IMAGE_GRAYSCALE);
	else
		img = cvLoadImage(strFileName, CV_LOAD_IMAGE_UNCHANGED);

	h = img->height;
	w = img->width;
	int g_img_x1 = input->vpw/2-w/2;
	int g_img_y1 = input->vph/2-h/2;

	for (y = h-1; y > 0; y--)
	{
		for (x = 0; x < w; x++)
		{
			image_pixel = (uchar*) (img->imageData + (h-1-y) * img->widthStep);
			switch (img->nChannels)
			{
			case 3:
				r = (int) image_pixel[3*x+2];
				g = (int) image_pixel[3*x+1];
				b = (int) image_pixel[3*x+0];
			break;
			default:
			case 1:
				r = (int) image_pixel[3*x+0];
				g = (int) image_pixel[3*x+0];
				b = (int) image_pixel[3*x+0];
			break;
			}

			input->image[3 * ((g_img_y1+y) * input->tfw + (g_img_x1+x)) + 0] = (GLubyte) r;
			input->image[3 * ((g_img_y1+y) * input->tfw + (g_img_x1+x)) + 1] = (GLubyte) g;
			input->image[3 * ((g_img_y1+y) * input->tfw + (g_img_x1+x)) + 2] = (GLubyte) b;
		}
	}

	cvReleaseImage(&img);

	return (0);
}

//***********************************************************
//* Function: input_generator
//* Description:
//***********************************************************
void
input_generator (INPUT_DESC *input, int status)
{
	// Inicializacao executada apenas uma vez por janela
	if (input->win == 0)
	{
		init_neural_localizer(input);
		input_initialized = 1;
		input_desc = input;
#ifdef NO_INTERFACE
		input->win = 1;
#endif
	}
	else
	{
		if (status == MOVE)
		{
//			crop_carmen_input_map_to_input_layer(input_map, input, robot_pose);
//
//			all_filters_update();
//			all_dendrites_update();
//			all_neurons_update();
//			all_outputs_update();

#ifndef NO_INTERFACE
			glutSetWindow(input->win);
			input_display();
#endif
		}
	}
}


/*
***********************************************************
* Function: get_new_sentence
* Description:
* Inputs: input -
*	  nDirection -
* Output: 0 if OK, -1 otherwise
***********************************************************
*/

void
get_new_sentence(INPUT_DESC *input, int nDirection)
{
	int sentence_number;

	sentence_number = get_next_sentence(input, nDirection);
	read_input_sentence(input, sentence_number);
}


/*
***********************************************************
* Function: input_generator
* Description:
* Inputs: input -
*	  status -
* Output: None
***********************************************************
*/

void
input_generator1(INPUT_DESC *input, int status)
{
	NEURON_LAYER *target_neuron_layer;
	OUTPUT_DESC *associated_output;

	// Inicializacao executada apenas uma vez por janela
	if (input->win == 0)
	{
		init_nc_input(input);
		input->win = glutGetWindow();
		initialize_data_set(input);
	}
	else
	{
		if (status == MOVE)
		{
			if (input->wxd < NL_WIDTH/5)
				get_new_sentence(input, DIRECTION_REWIND);
			else if (input->wxd >= NL_WIDTH-NL_WIDTH/5)
				get_new_sentence(input, DIRECTION_FORWARD);

			target_neuron_layer = (NEURON_LAYER *) input->input_generator_params->next->param.pval;
			copy_neuron_outputs(target_neuron_layer, input->neuron_layer);
			associated_output = get_output_by_neural_layer(target_neuron_layer);
			output_update(associated_output);

			glutSetWindow(input->win);

			all_filters_update();
		}
		else if (status == FORWARD)
		{
			target_neuron_layer = (NEURON_LAYER *) input->input_generator_params->next->param.pval;
			copy_neuron_outputs(target_neuron_layer, input->neuron_layer);
			associated_output = get_output_by_neural_layer(target_neuron_layer);
			output_update(associated_output);

			glutSetWindow(input->win);

			all_filters_update();
		}
	}
}



//***********************************************************
//* Function: draw_output
//* Description:
//***********************************************************
void
draw_output (char *strOutputName, char *strInputName)
{
	OUTPUT_DESC *output;
	output = get_output_by_name (strOutputName);
	set_neurons (output->neuron_layer->neuron_vector, 0, output->wh * output->ww, 23132);

	update_output_image (output);
#ifndef NO_INTERFACE
	glutSetWindow(output->win);
	glutPostWindowRedisplay (output->win);
#endif
}


//***********************************************************
//* Function: input_controller
//* Description:
//***********************************************************
void
input_controler (INPUT_DESC *input, int status)
{
	char strCommand[128];

	if ((input->mouse_button == GLUT_RIGHT_BUTTON) &&
	    (input->mouse_state == GLUT_DOWN) &&
	    (draw_active == 1))
	{
		sprintf (strCommand, "draw out_nl_voice_command based on spectogram move;");
		interpreter (strCommand);
	}

	if ((move_active == 1) &&
	    (input->mouse_button == GLUT_LEFT_BUTTON) &&
	    (input->mouse_state == GLUT_DOWN))
	{
		sprintf (strCommand, "move %s to %d, %d;", input->name, input->wxd, input->wyd);
		interpreter (strCommand);
		interpreter("forward network;");
	}

	input->mouse_button = -1;
}


/**
 * Description: handles the mouse input window events
 * -When mouse left button is pressed then a move command is triggered passing x, y coordinates.
 * @param input
 * @param status
 */
void input_controler1(INPUT_DESC *input, int status)
{
	char strCommand[128];

	if ((move_active == 1) &&
	    (input->mouse_button == GLUT_LEFT_BUTTON) &&
	    (input->mouse_state == GLUT_DOWN))
	{
		sprintf(strCommand, "move %s to %d, %d;", input->name, input->wxd, input->wyd);
		interpreter(strCommand);
	}

	input->mouse_button = -1;
}


//***********************************************************
//* Function: EvaluateOutput
//* Description: evaluates the output value
//* Inputs: output
//* Output: person ID
//***********************************************************
int EvaluateOutput(OUTPUT_DESC *output, float *confidence)
{
	return 0;
}


//***********************************************************
//* Function: output_handler
//* Description:
//***********************************************************
void output_handler(OUTPUT_DESC *output, int type_call, int mouse_button, int mouse_state)
{
#ifndef NO_INTERFACE
	glutSetWindow (output->win);
	output_display (output);
#endif
}

NEURON_OUTPUT
SetNetworkStatus (PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;

	g_nStatus = pParamList->next->param.ival;

	output.ival = set_network_status_interface(g_nStatus);

	return (output);
}

//***********************************************************
//* Function: f_keyboard
//* Description: Called whenever a key is pressed
//***********************************************************
void f_keyboard (char *key_value)
{
	char key;

	key = key_value[0];
	switch (key)
	{
		case 't': //set training status
			set_network_status_interface(TRAINING_PHASE);
			break;

		case 'r': //set recall status
			set_network_status_interface(RECALL_PHASE);
			break;
			
		case 'w': //up shift
			printf("up_shift\n");
			break;
			
		case 's': //down shift
		   printf("down_shift\n");
			break;
		
		case 'a': //left shift
		   printf("left_shift\n");
			break;
		
		case 'd': //right shift
		   printf("right_shift\n");
		   break;
			
		default:
			break;
	}
}
