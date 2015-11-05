#include <locale.h>
#include "neural_global_localizer_user_functions.h"
#include "../neural_global_localizer_interface.h"
#include <carmen/ekf_odometry_interface.h>


neural_global_localizer_config_t neural_global_localizer_config;
carmen_simple_stereo_disparity_message last_test_message;
carmen_neural_global_localizer_globalpos_message publish_message;

carmen_ekf_odometry_odometry_message last_ekf_odometry_estimate;
int has_ekf_odometry_estimate = 0;

PPMImage* last_output_ppm;
unsigned char* last_output_image = NULL;
int g_outputFrameID[NUMBER_OF_TRAINING_FRAMES];

static void *readPPM(const char *filename)
{
         char buff[1024];
         FILE *fp;
         int c, rgb_comp_color;
         //open PPM file for reading
         fp = fopen(filename, "rb");
         if (!fp) {
              fprintf(stderr, "Unable to open file '%s'\n", filename);
              exit(1);
         }

         //read image format
         if (!fgets(buff, sizeof(buff), fp)) {
              perror(filename);
              exit(1);
         }

    //check the image format
    if (buff[0] != 'P' || buff[1] != '6') {
         fprintf(stderr, "Invalid image format (must be 'P6')\n");
         exit(1);
    }

    //check for comments
    c = getc(fp);
    while (c == '#') {
    while (getc(fp) != '\n') ;
         c = getc(fp);
    }

    ungetc(c, fp);
    //read image size information
    if (fscanf(fp, "%d %d", &last_output_ppm->x, &last_output_ppm->y) != 2) {
         fprintf(stderr, "Invalid image size (error loading '%s')\n", filename);
         exit(1);
    }

    //read rgb component
    if (fscanf(fp, "%d", &rgb_comp_color) != 1) {
         fprintf(stderr, "Invalid rgb component (error loading '%s')\n", filename);
         exit(1);
    }

    //check rgb component depth
    if (rgb_comp_color!= 255) {
         fprintf(stderr, "'%s' does not have 8-bits components\n", filename);
         exit(1);
    }

    while (fgetc(fp) != '\n') ;

    if (!last_output_ppm) {
         fprintf(stderr, "Unable to allocate memory\n");
         exit(1);
    }

    //read pixel data from file
    if (fread(last_output_ppm->data, 3 * last_output_ppm->x, last_output_ppm->y, fp) != last_output_ppm->y) {
         fprintf(stderr, "Error loading image '%s'\n", filename);
         exit(1);
    }

    fclose(fp);
}

void
copy_raw_image_into_input(const unsigned char* raw_image, INPUT_DESC *input)
{
	int i, j;
	unsigned char red, green, blue;
	int intensity;
	int im_size = 3 * input->ww * input->wh ;

	switch(input->neuron_layer->output_type)
	{
		case BLACK_WHITE:
		case GREYSCALE:
			for (i = 0; i < input->wh ; i++)
			{
				for (j = 0; j < input->ww; j++)
				{
					red = raw_image[im_size - 3 * (i * input->ww + (input->ww- j) ) + 0];
					green = raw_image[im_size - 3 * (i * input->ww + (input->ww- j) ) + 1];
					blue = raw_image[im_size - 3 * (i * input->ww + (input->ww- j) ) + 2];

					// Intensity as pixel averaged intensity. ( RGB to greyscale )
					intensity = (int)(red + green + blue)/3;
					intensity = (intensity > 255) ? 255: intensity;

					input->image[3 * (i * input->ww + j) + 0] = (GLubyte) intensity;
					input->image[3 * (i * input->ww + j) + 1] = (GLubyte) intensity;
					input->image[3 * (i * input->ww + j) + 2] = (GLubyte) intensity;
				}
			}
			break;
		case COLOR:
		default:// case color
			for (i = 0; i < input->vph; i++)
			{
				for (j = 0; j < input->vpw; j++)
				{
					red = raw_image[im_size - 3 * (i * input->ww + (input->ww- j) ) + 0];
					green = raw_image[im_size - 3 * (i * input->ww + (input->ww- j) ) + 1];
					blue = raw_image[im_size - 3 * (i * input->ww + (input->ww- j) ) + 2];

					input->image[3 * (i * input->tfw + j) + 0] = (GLubyte) red;
					input->image[3 * (i * input->tfw + j) + 1] = (GLubyte) green;
					input->image[3 * (i * input->tfw + j) + 2] = (GLubyte) blue;
				}
			}
			break;
	}
}

unsigned char* test(unsigned char* image)
{
	char strCommand[1024];

	copy_raw_image_into_input(image, &in_pattern);

	in_pattern.up2date = 0;
	update_input_neurons (&in_pattern);
	update_input_image (&in_pattern);

	move_input_window(in_pattern.name, 320, 182);

	g_networkStatus = RECALL_PHASE;

	all_dendrites_update();
	all_neurons_update();
	all_outputs_update();
}

double last_timestamp = 0.0;

void
carmen_simple_stereo_disparity_handler(carmen_simple_stereo_disparity_message* message)
{
	g_networkStatus = MOVING_PHASE;
	last_timestamp = message->timestamp;
	test(message->reference_image);
}

void
carmen_ekf_odometry_odometry_handler(carmen_ekf_odometry_odometry_message* message)
{
	static int ekf_odometry_count = 0;

	if(!has_ekf_odometry_estimate && ekf_odometry_count > 10)
		has_ekf_odometry_estimate = 1;  //must be one!
	else
		ekf_odometry_count++;

	last_ekf_odometry_estimate = *message;
}

static int read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] =
	{
			{(char *) "neural_global_localizer", (char *) "camera",  CARMEN_PARAM_INT, &neural_global_localizer_config.camera, 0, NULL},
			{(char *) "neural_global_localizer", (char *) "training",  CARMEN_PARAM_ONOFF, &neural_global_localizer_config.training, 0, NULL},
			{(char *) "neural_global_localizer", (char *) "save_train",  CARMEN_PARAM_ONOFF, &neural_global_localizer_config.save_train, 0, NULL}
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}

void
initialize_ipc(int argc, char** argv)
{
	/* connect to IPC server */
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	read_parameters(argc, argv);

	carmen_neural_global_localizer_define_messages();

	carmen_stereo_subscribe(neural_global_localizer_config.camera, &last_test_message, (carmen_handler_t) carmen_simple_stereo_disparity_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_ekf_odometry_subscribe_odometry_message(NULL, (carmen_handler_t) carmen_ekf_odometry_odometry_handler, CARMEN_SUBSCRIBE_LATEST);
}

void
ipc_timer_function(int value)
{
	IPC_listenWait((unsigned int)100);
	glutTimerFunc(100, ipc_timer_function, value);
}

/*** ############################# ***/

int read_frame_input(INPUT_DESC *input, char *strFileName)
{
	load_input_image (input, strFileName);
	return (0);
}

int get_frame_by_id(INPUT_DESC *input, char* directory, int id)
{
	char strFileName[1024];
	sprintf (strFileName, "%s%03dl.ppm", directory, id);
	printf("%s\n", strFileName);

	if (read_frame_input(input, strFileName))
		return (-1);

	check_input_bounds (input, input->wx + input->ww/2, input->wy + input->wh/2);

	input->up2date = 0;
	update_input_image(input);

	return (0);
}

int read_dataset_file(char* filename, char* directory, int number_of_frames, int number_of_saliencies)
{
	int i;
	FILE* fd;
	char path[1024];
	char image_name[1024];

	sprintf(path, "%s%s", directory, filename);

	if((fd = fopen(path, "r")) == NULL)
		return -1;

	for(i = 0; i < number_of_frames; i++)
	{
		fscanf(fd, "\n%s", image_name);
		g_training_frame_list[i].image_id = i;

		fscanf(fd, "%lf", &g_training_frame_list[i].pose.position.x);
		fscanf(fd, "%lf", &g_training_frame_list[i].pose.position.y);
		fscanf(fd, "%lf", &g_training_frame_list[i].pose.position.z);

		fscanf(fd, "%lf", &g_training_frame_list[i].pose.orientation.yaw);
		fscanf(fd, "%lf", &g_training_frame_list[i].pose.orientation.pitch);
		fscanf(fd, "%lf", &g_training_frame_list[i].pose.orientation.roll);

		for(int j = 0; j < number_of_saliencies; j++)
		{
			fscanf(fd, "%lf", &g_training_frame_list[i].saliencies[j].coordinates.x);
			fscanf(fd, "%lf", &g_training_frame_list[i].saliencies[j].coordinates.y);
			fscanf(fd, "%lf", &g_training_frame_list[i].saliencies[j].pose.x);
			fscanf(fd, "%lf", &g_training_frame_list[i].saliencies[j].pose.y);
			fscanf(fd, "%lf", &g_training_frame_list[i].saliencies[j].pose.z);
		}
	}

	fclose(fd);
	return 0;
}

void train_dataset(int sample)
{
	char strCommand[1024];

	for(int i = 0; i < NUMBER_OF_TRAINING_FRAMES; i+=sample)
	{
		g_testedFrameID = g_frameID = i;

		get_frame_by_id(&in_pattern, (char *) INPUT_TRAINING_PATH, i);

		sprintf (strCommand, "move %s to %d, %d;", in_pattern.name, 320, 182);
		interpreter (strCommand);

		sprintf (strCommand, "draw %s based on %s move;", out_nl_v1_pattern.name, in_pattern.name);
		interpreter (strCommand);

		all_filters_update();
		all_outputs_update();

		train_network();
	}
}

int init_user_functions ()
{
	char strCommand[1024];
	char *locale_string;

	initialize_ipc(global_argc, global_argv);

	locale_string = setlocale (LC_ALL, "C");
	if (locale_string == NULL)
	{
	        fprintf (stderr, "Could not set locale.\n");
	        exit (1);
	}
	else
        	printf ("Locale set to %s.\n", locale_string);

	in_pattern.wxd = in_pattern.ww / 2;
	in_pattern.wyd = in_pattern.wh / 2;

	sprintf (strCommand, "move %s to %d, %d;", in_pattern.name, in_pattern.wxd, in_pattern.wyd);
	interpreter (strCommand);

	sprintf (strCommand, "toggle move_active;");
	interpreter (strCommand);

	sprintf (strCommand, "toggle draw_active;");
	interpreter (strCommand);

	update_input_neurons (&in_pattern);
	all_filters_update();
	all_outputs_update ();

	if(neural_global_localizer_config.training)
	{
		g_networkStatus = TRAINING_PHASE;

		printf("training...\n");
		train_dataset(1);

		if(neural_global_localizer_config.save_train)
		{
			printf("saving network...\n");
			unload_network((char *) "ngl1-2014.mem");
			printf("network saved...\n");
		}
	}
	else
	{
		destroy_network();
		printf("loading network...\n");
		reload_network((char *) "ngl1-2014.mem");
		printf("network loaded!\n");
	}

	return (0);
}

void make_input_image_neural_global_localizer(INPUT_DESC *input, int w, int h)
{
	char message[1024];

	input->tfw = nearest_power_of_2 (w);
	input->tfh = nearest_power_of_2 (h);

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
			sprintf(message, (char *) "%d. It can be SHOW_FRAME or SHOW_WINDOW.",TYPE_SHOW);
			Erro ((char *) "Invalid Type Show ", message, (char *) "Error in update_input_image.");
			return;
	}

	input->vpxo = 0;
	input->vpyo = h - input->vph;

	if(input->image == NULL)
		input->image = (GLubyte *) alloc_mem (input->tfw * input->tfh * 3 * sizeof (GLubyte));
}

void init_neural_global_localizer (INPUT_DESC *input)
{
	int x, y, i;

	g_training_frame_list = (frame_t *) malloc (NUMBER_OF_TRAINING_FRAMES * sizeof(frame_t));
	last_output_image = (unsigned char *) calloc (IMAGE_WIDTH * IMAGE_HEIGHT * 3, sizeof(unsigned char));

	read_dataset_file((char *) DATA_SET_FILE, (char *) INPUT_TRAINING_PATH, NUMBER_OF_TRAINING_FRAMES, NUMBER_OF_SALIENCIES);
	make_input_image_neural_global_localizer(input, IMAGE_WIDTH, IMAGE_HEIGHT);

	last_output_ppm = (PPMImage*)malloc(sizeof(PPMImage));
    last_output_ppm->data = (PPMPixel*)malloc(IMAGE_WIDTH * IMAGE_HEIGHT * sizeof(PPMPixel));

	if(publish_message.test_image == NULL)
			publish_message.test_image = (unsigned char *) calloc (IMAGE_HEIGHT * IMAGE_WIDTH * 3, sizeof(unsigned char));

	if(publish_message.test_disparity == NULL)
			publish_message.test_disparity = (float *) calloc (IMAGE_HEIGHT * IMAGE_WIDTH, sizeof(float));

	if(publish_message.output_image == NULL)
		publish_message.output_image = (unsigned char *) calloc (IMAGE_HEIGHT * IMAGE_WIDTH * 3, sizeof(unsigned char));


	glutInitWindowSize (input->ww, input->wh);
	if (read_window_position (input->name, &x, &y))
		glutInitWindowPosition (x, y);
	else
		glutInitWindowPosition (-1, -1);
	input->win = glutCreateWindow (input->name);

	glGenTextures (1, (GLuint *)(&(input->tex)));
	input_init (input);
	glutReshapeFunc (input_reshape);
	glutDisplayFunc (input_display);
	glutKeyboardFunc (keyboard);
	glutPassiveMotionFunc (input_passive_motion);
	glutMouseFunc (input_mouse);
	glutTimerFunc(100, ipc_timer_function, 1);

}

void input_generator (INPUT_DESC *input, int status)
{
	if (input->win == 0)
		init_neural_global_localizer (input);
	else
	{
		if (status == MOVE)
		{
			if (input->wxd < 0)
			{
				g_frameID--;
				get_frame_by_id(input, INPUT_TRAINING_PATH, g_frameID);
			}
			else if (input->wxd >= IMAGE_WIDTH)
			{
				g_frameID++;
				get_frame_by_id(input, INPUT_TRAINING_PATH, g_frameID);
			}
			else
			{
				translation_filter_deltaX = (float) input->wxd;
				translation_filter_deltaY = (float) input->wyd;
			}

			check_input_bounds (input, input->wxd, input->wxd);
			glutSetWindow (input->win);

			all_filters_update ();
			all_outputs_update ();
		}
	}
}

void input_controler (INPUT_DESC *input, int status)
{
	char strCommand[1024];

	if ((move_active == 1) &&
	    (input->mouse_button == GLUT_LEFT_BUTTON) &&
	    (input->mouse_state == GLUT_DOWN))
	{
		// Move the input window
		sprintf (strCommand, "move %s to %d, %d;", input->name, input->wxd, input->wyd);
		interpreter (strCommand);
	}
	input->mouse_button = -1;

	return;
}

int evaluate_output(OUTPUT_DESC *output, float *confidence)
{
	int i;
	int nMax1 = 0;
	int nMax2 = 0;
	NEURON *neuron_vector;

	neuron_vector = output->neuron_layer->neuron_vector;

	for (i = 0; i < NUMBER_OF_TRAINING_FRAMES; i++)
		g_outputFrameID[i] = 0;

	for (i = 0; i < (output->wh * output->ww); i++)
	{
		if ((neuron_vector[i].output.ival >= 0) && (neuron_vector[i].output.ival < NUMBER_OF_TRAINING_FRAMES))
			g_outputFrameID[neuron_vector[i].output.ival]++;
	}

	for (i = 0; i < NUMBER_OF_TRAINING_FRAMES; i++)
	{
		if (g_outputFrameID[i] > nMax1)
		{
			nMax1 = g_outputFrameID[i];
			g_frameID = i;
		}
	}

	g_outputFrameID[g_frameID] = 0;

	for (i = 0; i < NUMBER_OF_TRAINING_FRAMES; i++)
	{
		if (g_outputFrameID[i] > nMax2)
			nMax2 = g_outputFrameID[i];
	}

	*confidence = ((float) (nMax1 - nMax2)) / ((float) nMax1);
	return (g_frameID);
}

void evaluate_n_last_outputs(OUTPUT_DESC *output, int *outputs, int number_to_evaluate)
{
	int i, k;
	int nMax1 = 0;
	NEURON *neuron_vector;

	neuron_vector = output->neuron_layer->neuron_vector;

	for (i = 0; i < NUMBER_OF_TRAINING_FRAMES; i++)
		g_outputFrameID[i] = 0;

	for (i = 0; i < (output->wh * output->ww); i++)
	{
		if ((neuron_vector[i].output.ival >= 0) && (neuron_vector[i].output.ival < NUMBER_OF_TRAINING_FRAMES))
			g_outputFrameID[neuron_vector[i].output.ival]++;
	}

	for(k = 0; k < number_to_evaluate; k++)
	{
		nMax1 = 0;

		for (i = 0; i < NUMBER_OF_TRAINING_FRAMES; i++)
		{
			if (g_outputFrameID[i] > nMax1)
			{
				nMax1 = g_outputFrameID[i];
				g_frameID = i;
			}
		}

		g_outputFrameID[g_frameID] = 0;
		outputs[k] = g_frameID;
	}
}

void publish_neural_global_localizer_globalpos(int frame, double confidence)
{
	IPC_RETURN_TYPE err;

	publish_message.pose = g_training_frame_list[frame].pose;
	publish_message.confidence = confidence;
	publish_message.image_size = IMAGE_HEIGHT * IMAGE_WIDTH * 3;
	publish_message.disparity_size = IMAGE_HEIGHT * IMAGE_WIDTH;

	memcpy(publish_message.test_image, last_test_message.reference_image, publish_message.image_size);
	memcpy(publish_message.output_image,  last_output_image, publish_message.image_size);
	memcpy(publish_message.test_disparity, last_test_message.disparity, publish_message.disparity_size * 4);

	for(int i = 0; i < NUMBER_OF_SALIENCIES; i++)
	{
		publish_message.saliencies[i].coordinates.x = g_training_frame_list[frame].saliencies[i].coordinates.x;
		publish_message.saliencies[i].coordinates.y = g_training_frame_list[frame].saliencies[i].coordinates.y;
		publish_message.saliencies[i].pose = g_training_frame_list[frame].saliencies[i].pose;
	}

	publish_message.timestamp = last_timestamp;
	publish_message.host = carmen_get_host();

	err = IPC_publishData(CARMEN_NEURAL_GLOBAL_LOCALIZER_GLOBALPOS_MESSAGE_NAME, &publish_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_NEURAL_GLOBAL_LOCALIZER_GLOBALPOS_MESSAGE_NAME);
}

double distance_between_poses(carmen_pose_3D_t p1, carmen_pose_3D_t p2)
{
	return sqrt((p1.position.x - p2.position.x) * (p1.position.x - p2.position.x) + (p1.position.y - p2.position.y) * (p1.position.y - p2.position.y));
}

int get_most_likelihood_output(carmen_ekf_odometry_odometry_message last_ekf_odometry_estimate, int *possible_outputs, int n_evaluate)
{
	double min_distance = 999999.0;
	double distance;
	int min_distance_index = 0;

	for(int i = 0; i < n_evaluate; i++)
	{
		distance = distance_between_poses(last_ekf_odometry_estimate.estimated_pose, g_training_frame_list[possible_outputs[i]].pose);

		if(distance < min_distance)
		{
			min_distance = distance;
			min_distance_index = i;
		}
	}

	return possible_outputs[min_distance_index];
}


void output_handler (OUTPUT_DESC *output, int type_call, int mouse_button, int mouse_state)
{
	int frame = 0;
	float confidence = 0.0;
	char recall_filename[1024];
	int possible_outputs[2];

	if (g_networkStatus == RECALL_PHASE)
	{

		if (strcmp (output->name, out_nl_v1_pattern.name) == 0)
		{
			if(has_ekf_odometry_estimate)
			{
				evaluate_n_last_outputs(output, possible_outputs, 2);
				frame = get_most_likelihood_output(last_ekf_odometry_estimate, possible_outputs, 2);
			}
			else
				frame = evaluate_output(output, &confidence);

			sprintf(recall_filename, "%s%03dl.ppm", INPUT_TRAINING_PATH, frame);
			readPPM(recall_filename);
			load_image_to_object(output->output_handler_params->next->param.sval, recall_filename);

			for(int i = 0; i < last_output_ppm->x * last_output_ppm->y; i++)
			{
				last_output_image[3 * i] = last_output_ppm->data[i].red;
				last_output_image[3 * i + 1] = last_output_ppm->data[i].green;
				last_output_image[3 * i + 2] = last_output_ppm->data[i].blue;
			}

			publish_neural_global_localizer_globalpos(frame, confidence);
		}
	}

	#ifndef NO_INTERFACE
		glutSetWindow (output->win);
		output_display ();
	#endif
}

void draw_output (char *output_name, char *input_name)
{
	OUTPUT_DESC* output;

	output = get_output_by_name (output_name);

	set_neurons (output->neuron_layer->neuron_vector, 0, output->wh * output->ww, g_frameID);

	output_update(output);

	return;
}

void f_keyboard (char *key_value)
{
	char key;

	switch (key = key_value[0])
	{
		// Train network
		case 'T':
		case 't':
			g_networkStatus = TRAINING_PHASE;
			break;

		case 'R':
		case 'r':
			g_networkStatus = RECALL_PHASE;
			break;
			break;

	}

	return;
}
