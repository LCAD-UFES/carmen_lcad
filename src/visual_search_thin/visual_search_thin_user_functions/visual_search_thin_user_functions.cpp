#include <map>
#include <algorithm>
#include <locale.h>
#include "visual_search_thin_user_functions.h"
#include "visual_search_thin_filters.h"
#include "../visual_search_thin_handlers.h"

float *gaussian_filtered_training_pattern = NULL; 		//training image (also used in filter module)
float gaussian_filtered_training_pattern_sum = 0.0f;	//total filtered training pattern accumulated weight

// this function is required by the MAE core modules
void
draw_output (char *output_name __attribute__ ((unused)),
		char *input_name __attribute__ ((unused)) ) {}

int
conditional_filter_update(FILTER_DESC	*filter)
{
	if(filter)
	{
		filter_update(filter);
		return(0);
	}
	else
	{
		return(-1);
	}
}

void
ipc_timer_function(int value)
{
	IPC_listenWait((unsigned int)10);
	glutTimerFunc(10, ipc_timer_function, value);
}

void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("visual_search: disconnected.\n");

		exit(0);
	}
}

int
init_user_functions ()
{
	char *locale_string;

	/* connect to IPC server */
	carmen_ipc_initialize(global_argc, global_argv);
	carmen_param_check_version(global_argv[0]);

	carmen_visual_search_thin_define_message_output_train();

	carmen_visual_search_thin_define_message_output();

	carmen_visual_search_thin_subscribe_train(NULL, (carmen_handler_t)training_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_visual_search_thin_subscribe_test(NULL, (carmen_handler_t)test_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_visual_search_thin_subscribe_query_test(query_test_message_handler);

	carmen_visual_search_thin_subscribe_query_train(query_training_message_handler);

	signal(SIGINT, shutdown_module);

	locale_string = setlocale (LC_ALL, "C");
	if (locale_string == NULL)
	{
		fprintf (stderr, "Could not set locale.\n");
		exit (1);
	}
	else
		printf ("Locale set to %s.\n", locale_string);

	interpreter ((char*)"toggle move_active;");

	interpreter ((char*)"toggle draw_active;");

	return (0);
}


void
make_input_image_visual_search_thin (INPUT_DESC *input, int w, int h)
{
	char message[256];

	input->tfw = w;
	input->tfh = h;

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
			Erro ((char*)"Invalid Type Show ", message, (char*)" Error in update_input_image.");
			return;
	}
	
	input->vpxo = 0;
	input->vpyo = h - input->vph;

	if(input->image == NULL)
		input->image = (GLubyte *) alloc_mem (input->tfw * input->tfh * 3 * sizeof (GLubyte));
}

void
init_visual_search_thin (INPUT_DESC *input)
{
	int x,y;

	make_input_image_visual_search_thin (input, IMAGE_WIDTH, IMAGE_HEIGHT);

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
	glutTimerFunc(10, ipc_timer_function, 1);
}

int
update_input_filters()
{
	// Try to Update All filters and check if it was a successful update
	if(	!(
		conditional_filter_update(get_filter_by_name((char*)"in_saccade_translated_filter")) /*&
		conditional_filter_update(get_filter_by_name((char*)"in_saccade_translated_gaussian_filter"))*/
		))
		return 0;

	return 1;
}

void
input_generator (INPUT_DESC *input, int status)
{

	if (input->win == 0)
	{
		init_visual_search_thin (input);
	}
	else
	{
		if (status == MOVE)
		{
			check_input_bounds (input, input->wxd, input->wxd);
			glutSetWindow (input->win);
			input_display ();
			update_input_filters();
			all_dendrites_update (); 
			all_neurons_update ();
			filter_update(get_filter_by_name((char*)"nl_v1_activation_map_f_filter"));	//this filter is obligatory
			all_outputs_update ();
		}
	}
}

void
input_generator2 (INPUT_DESC *input,
		int status __attribute__ ((unused)))
{
	if (input->win == 0)
	{
		init_visual_search_thin (input);
	}
	else
	{
		if (status == MOVE)
		{
			check_input_bounds (input, input->wxd, input->wxd);
			glutSetWindow (input->win);
			input_display ();
			update_input_filters();
		}
	}
}

void
output_handler_max_value_position (OUTPUT_DESC *output,
		int type_call __attribute__ ((unused)) ,
		int mouse_button __attribute__ ((unused)) ,
		int mouse_state __attribute__ ((unused)) )
{
	char *nl_target_coordinates_name = NULL;
	int u, v, u_max, v_max, w, h, xi, yi; //, step;
	float current_value, max_value = FLT_MIN, log_factor;
	NEURON_LAYER *nl_target_coordinates = NULL;
	
	// Gets the output handler parameters
	nl_target_coordinates_name = output->output_handler_params->next->param.sval;
	log_factor = output->output_handler_params->next->next->param.fval;
	
	// Gets the target coordinates neuron layer
	nl_target_coordinates = get_neuron_layer_by_name (nl_target_coordinates_name);
	
	// Gets the Neuron Layer Dimentions
	w = output->neuron_layer->dimentions.x;
	h = output->neuron_layer->dimentions.y;
	
	// Finds the max value position
	for (v = 0, u_max = v_max = 0; v < h; v++)
	{
		for (u = 0; u < w; u++)
		{
			current_value = output->neuron_layer->neuron_vector[v * w + u].output.fval;
			
			if (max_value < current_value)
			{
				max_value = current_value;
				u_max = u;
				v_max = v;
			}
		}
	}
	
	// Map the max value coordinates to image
	map_v1_to_image (&xi, &yi, IMAGE_WIDTH, IMAGE_HEIGHT, u_max, v_max, w, h, 0, 0, (double) h / (double) (h - 1), log_factor);
	
	// Saves the max value position
	nl_target_coordinates->neuron_vector[0].output.fval = (1.0/dynamic_scale_factor) * ( (float) 2*xi);
	nl_target_coordinates->neuron_vector[1].output.fval = (1.0/dynamic_scale_factor) * ( (float) 2*yi);
}

void
visual_search_thin_saccade (INPUT_DESC *input,
		int max_number_of_saccades, float min_threshold_of_saccade,
		int *x_saccade_vector_out, int *y_saccade_vector_out)
{
	float x, y;
	int count = 0;
	int x_saccade_vector = 0;
	int y_saccade_vector = 0;
	
	// Saccade until reach the target
	x = nl_target_coordinates.neuron_vector[0].output.fval;
	y = nl_target_coordinates.neuron_vector[1].output.fval;

	do
	{
		// Rounding adjustments
		x = (x > 0.0)? x + 0.5: x - 0.5;
		y = (y > 0.0)? y + 0.5: y - 0.5;
		
		// Saccade movement must saturate on neuron layer limits 
		
		// Width boundary check
		if((input->wxd + (int) x) > IMAGE_WIDTH)
			input->wxd = IMAGE_WIDTH;
		else if((input->wxd + (int) x) < 0)
			input->wxd = 0;
		else
		{
			input->wxd += (int) x;
			x_saccade_vector += (int) x;
		}
		 
		// Height boundary check
		if((input->wyd + (int) y) > IMAGE_HEIGHT)
			input->wyd = IMAGE_HEIGHT;
		else if((input->wyd + (int) y) < 0)
			input->wyd = 0;
		else
		{
			input->wyd += (int) y;
			y_saccade_vector += (int) y;
		}
		
		set_input_layer_translation(input, input->wxd, input->wyd);
				
		//Get next delta coordinates from the last layer after saccadic movement
		x = nl_target_coordinates.neuron_vector[0].output.fval;
		y = nl_target_coordinates.neuron_vector[1].output.fval;
		
		count++;
	} 
	while ( (count < max_number_of_saccades) &&
			((fabs(x) > min_threshold_of_saccade) ||
			(fabs(y) > min_threshold_of_saccade))
		  );

	if (x_saccade_vector_out)
		*x_saccade_vector_out = x_saccade_vector;
	if (y_saccade_vector_out)
		*y_saccade_vector_out = y_saccade_vector;

}

void
input_controler (INPUT_DESC *input, int status __attribute__ ((unused)) )
{
	if ((move_active == 1) &&
	    (input->mouse_button == GLUT_LEFT_BUTTON) &&
	    (input->mouse_state == GLUT_DOWN))
	{
		// Translate the input image & Move the input center cursor
		set_input_layer_translation(input, input->wxd, input->wyd);
	}
	input->mouse_button = -1;
}

void
input_controler2 (INPUT_DESC *input, int status __attribute__ ((unused)) )
{
	input->mouse_button = -1;
}

/* Sets the internal neuron layer band with the high activation output value in a gaussian pattern */
void
set_neuron_layer_band_gaussian (NEURON_LAYER *neuron_layer, float max_val, float std_dev, int width)
{
	float	activation_value;
	int u, v, w, h;
	
	w = neuron_layer->dimentions.x;
	h = neuron_layer->dimentions.y;
	
	for(u = 0; u < w; u++)
	{
		if (u < w/2)
		{
			if((w-1)/2 - u > width/2)
				activation_value = 0.0;
			else
				activation_value = max_val*exp(-((w-1)/2 - u)*((w-1)/2 - u)/(2*std_dev*std_dev));
		}
		else
		{
			if(u - w/2 > width/2)
				activation_value = 0.0;
			else
				activation_value = max_val*exp(-(u - w/2)*(u - w/2)/(2*std_dev*std_dev));
		}
		
		
		for(v = 0;v < h; v++)
			neuron_layer->neuron_vector[u + w * v].output.fval = activation_value;
	}
}

float
visual_search_thin_confidence(NEURON_LAYER *neuron_layer, int band_width)
{
	int neuron_layer_width, neuron_layer_height;
	int hit_count = 0, miss_count = 0;

	neuron_layer_width = neuron_layer->dimentions.x;
	neuron_layer_height = neuron_layer->dimentions.y;

	for(int x = 0; x < neuron_layer_width; x++)
	{
		if ( (x > (neuron_layer_width/2 - band_width/2)) &&	(x < (neuron_layer_width/2 + band_width/2))	)
		{
			for(int y = 0;y < neuron_layer_height; y++)
			{
				float training_value = gaussian_filtered_training_pattern[x + neuron_layer_width * y];
				float activation_value = neuron_layer->neuron_vector[x + neuron_layer_width * y].output.fval;
				if (activation_value && training_value)
				{
					hit_count++;
				}
				else
				{
					miss_count++;
				}
			}
		}
	}

	if (hit_count > miss_count)
		return (float)(hit_count - miss_count)/(float)hit_count;
	else
		return 0.0;
}

float
visual_search_thin_scale(NEURON_LAYER *neuron_layer)
{
	int x, y, w, h;
	float accumulated_active_weight;
	float ratio;
	
	w = neuron_layer->dimentions.x;
	h = neuron_layer->dimentions.y;
	
	accumulated_active_weight = 0.0;

	for (y = 0; y < h; y++)
		for (x = 0; x < w; x++)
			accumulated_active_weight += neuron_layer->neuron_vector[x + w * y].output.fval;
	
	ratio = accumulated_active_weight / gaussian_filtered_training_pattern_sum;

	return(ratio);
}

// Saves the gaussian filtered image from the neuron layer (the training pattern)
void
save_gaussian_filtered_training_pattern(NEURON_LAYER *neuron_layer)
{
	int w, h, i;
	
	w = neuron_layer->dimentions.x;
	h = neuron_layer->dimentions.y;
	
	if(gaussian_filtered_training_pattern)
	{
		free(gaussian_filtered_training_pattern);
		gaussian_filtered_training_pattern = (float *)malloc(w * h * sizeof(float));
	}
	else
	{
		gaussian_filtered_training_pattern = (float *)malloc(w * h * sizeof(float));
	}
	gaussian_filtered_training_pattern_sum = 0.0;	//accumulated training image weight
		
	for (i = 0; i < w*h; i++)
	{
		gaussian_filtered_training_pattern[i] = neuron_layer->neuron_vector[i].output.fval;
		gaussian_filtered_training_pattern_sum += neuron_layer->neuron_vector[i].output.fval;
	}

}

// Keyboard actions are still acceptable
void f_keyboard (char *key_value)
{
	char key;
	
	switch (key = key_value[0])
	{
		// Train network
		case 'T':	
		case 't':
			visual_search_thin_train();
			break;
		// Saccade until reach the target
		case 'S':
		case 's':
			visual_search_thin_saccade(&in_saccade_current, MAX_NUMBER_OF_SACCADE, MIN_THRESHOLD_OF_SACCADE, NULL, NULL);
			break;	
		// Force the filters in_saccade_filtered_filter & in_saccade_filtered_translated_filter to be updated
		case 'U':
		case 'u':
			update_input_filters();
			break;
		// Forgets last training
		case 'F':
		case 'f':
			clear_neural_layers_memory((char*)"nl_v1_activation_map");
			break;
	}
	
	return;
}

void
set_input_layer_translation(INPUT_DESC *input, int x, int y)
{
	// The coordinate system must be adjusted to the MAE internal coordinates
	input->wxd = x;
	input->wyd = y;
	translation_filter_deltaX = (float)(-IMAGE_WIDTH  + input->wxd);
	translation_filter_deltaY = (float)(-IMAGE_HEIGHT + input->wyd);
	move_input_window (input->name, input->wxd, input->wyd);
}

void
input_filters_and_outputs_update(void)
{
	conditional_filter_update(get_filter_by_name((char*)"in_saccade_translated_filter"));
	
	if(get_output_by_name((char*)"in_saccade_current_filtered_out"))
		output_update(get_output_by_name((char*)"in_saccade_current_filtered_out"));
		
	conditional_filter_update(get_filter_by_name((char*)"in_saccade_translated_scaled_filter"));

	if(get_output_by_name((char*)"in_saccade_translated_scaled_out"))
		output_update(get_output_by_name((char*)"in_saccade_translated_scaled_out"));

	conditional_filter_update(get_filter_by_name((char*)"in_saccade_translated_scaled_gaussian_filter"));
	
	if(get_output_by_name((char*)"in_saccade_translated_scaled_gaussian_out"))
		output_update(get_output_by_name((char*)"in_saccade_translated_scaled_gaussian_out"));

	conditional_filter_update(get_filter_by_name((char*)"in_saccade_trained_log_polar_filter"));
	
	if(get_output_by_name((char*)"in_saccade_trained_log_polar_out"))
		output_update(get_output_by_name((char*)"in_saccade_trained_log_polar_out"));

}

// Update the input layer neurons and Image
void
update_input_layer_neurons_and_image(INPUT_DESC *input)
{
	check_input_bounds (input, input->wx + input->ww/2, input->wy + input->wh/2);
	input->up2date = 0;
	update_input_neurons (input);
	update_input_image (input);
	
	input_filters_and_outputs_update();
}

void
update_input_layer_neurons_and_image_light(INPUT_DESC *input)
{
	check_input_bounds (input, input->wx + input->ww/2, input->wy + input->wh/2);
	input->up2date = 0;
	update_input_neurons (input);
	update_input_image (input);
}
			
// Trains the visual search module
void
visual_search_thin_train(void)
{	
	set_neuron_layer_band_gaussian(&nl_v1_activation_map,HIGHEST_OUTPUT,(float)BAND_WIDTH/2.0,BAND_WIDTH);

	filter_update(get_filter_by_name((char*)"nl_v1_activation_map_f_filter"));

	train_neuron_layer((char*)"nl_v1_activation_map");

	save_gaussian_filtered_training_pattern(&nl_v1_activation_map);

	all_filters_update();
	all_outputs_update();
}

NEURON_OUTPUT
run_train(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;

	visual_search_thin_train();

	return (output);
}

NEURON_OUTPUT
run_test(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;

	visual_search_thin_saccade(&in_saccade_current, MAX_NUMBER_OF_SACCADE, MIN_THRESHOLD_OF_SACCADE, NULL, NULL);

	return (output);
}
