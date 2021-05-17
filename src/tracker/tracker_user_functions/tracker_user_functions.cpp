#include <map>
#include <algorithm>
#include <locale.h>
#include "tracker_user_functions.h"
#include "tracker_filters.h"
#include "../tracker_handlers.h"


#define ACEPT_IN_THE_IMAGE 50
#define CONFIDENCE_LEVEL 3
#define MIN_CONFIANCE_TO_ACCEPT_RETRAIN 50
#define MIN_CONFIANCE_TO_RETRAIN 50
#define NUM_IMAGES_BEFORE_TO_RETRAIN 3
#define NUM_PIXELS 3
#define	sigma 10.0
#define NUM_PIXELS_WEIGHT 4
#define COLOR_BAND_WIDTH_WEIGHT 2.0/6.0
#define BAND_ZOOM 75

float *gaussian_filtered_training_pattern = NULL; 	//training image (also used in filter module)
float gaussian_filtered_training_pattern_sum = 0.0f;	//total filtered training pattern accumulated weight
double g_confidence;
double g_color_band_width;
double scale_before = -1.0;
float g_confidence_zoom_init = -1.0;
int x_before;
int y_before;
	

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
filters_update()
{
	filter_update(get_filter_by_name((char*)"in_saccade_translated_filter"));
	filter_update(get_filter_by_name((char*)"in_pattern_translated_filter"));
       	filter_update(get_filter_by_name((char*)"in_pattern_filtered_translated_filter"));
       	filter_update(get_filter_by_name((char*)"in_pattern_filtered_translated_red_filter"));
       	filter_update(get_filter_by_name((char*)"in_pattern_filtered_translated_green_filter"));
       	filter_update(get_filter_by_name((char*)"in_pattern_filtered_translated_blue_filter"));
	filter_update(get_filter_by_name((char*)"nl_v1_pattern_filter"));	//this filter is obligatory
	filter_update(get_filter_by_name((char*)"table_v1_filter"));	//this filter is obligatory
	filter_update(get_filter_by_name((char*)"nl_v1_activation_map_neuron_weight_filter"));	//this filter is obligatory
	filter_update(get_filter_by_name((char*)"nl_activation_map_hough_filter"));	//this filter is obligatory
	filter_update(get_filter_by_name((char*)"nl_activation_map_hough_gaussian_filter"));	//this filter is obligatory
	filter_update(get_filter_by_name((char*)"nl_activation_map_hough_v1_filter"));	//this filter is obligatory
	filter_update(get_filter_by_name((char*)"nl_activation_map_hough_zoom_filter"));	//this filter is obligatory
	filter_update(get_filter_by_name((char*)"nl_activation_map_hough_zoom_gaussian_filter"));	//this filter is obligatory

	all_dendrites_update(); 
	all_neurons_update();

	forward_objects((char*)"nl_v1_activation_map_neuron_weight_filter");
	forward_objects((char*)"nl_activation_map_hough_filter");
	forward_objects((char*)"nl_activation_map_hough_gaussian_filter");
	forward_objects((char*)"nl_activation_map_hough_v1_filter");
	forward_objects((char*)"nl_activation_map_hough_zoom_filter");
	forward_objects((char*)"nl_activation_map_hough_zoom_gaussian_filter");

}

NEURON_OUTPUT
ForwardVisualSearchNetwork(PARAM_LIST *pParamList __attribute__ ((unused)))
{
	NEURON_OUTPUT output;
	forward_objects((char*)"in_pattern_translated_filter");
	forward_objects((char*)"in_pattern_filtered_translated_filter");
	forward_objects((char*)"in_pattern_filtered_translated_red_filter");
	forward_objects((char*)"in_pattern_filtered_translated_green_filter");
	forward_objects((char*)"in_pattern_filtered_translated_blue_filter");
	forward_objects((char*)"nl_v1_pattern_filter");
	forward_objects((char*)"nl_v1_activation_map");
	forward_objects((char*)"table_v1_filter");
	forward_objects((char*)"nl_v1_activation_map_neuron_weight_filter");
	forward_objects((char*)"nl_activation_map_hough_filter");
	forward_objects((char*)"nl_activation_map_hough_gaussian_filter");
	forward_objects((char*)"nl_activation_map_hough_v1_filter");
	forward_objects((char*)"nl_activation_map_hough_zoom_filter");
	forward_objects((char*)"nl_activation_map_hough_zoom_gaussian_filter");
	all_outputs_update();

	output.ival = 0;

	return (output);
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

	carmen_tracker_define_message_output_train();

	carmen_tracker_define_message_output();

	carmen_tracker_subscribe_train(NULL, (carmen_handler_t)training_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_tracker_subscribe_test(NULL, (carmen_handler_t)test_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_tracker_subscribe_query_test(query_test_message_handler);

	carmen_tracker_subscribe_query_train(query_training_message_handler);

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

	all_filters_update();
	all_outputs_update ();

	g_color_band_width = table.dimentions.x * COLOR_BAND_WIDTH_WEIGHT;
	if ((int)g_color_band_width % 2 == 0) g_color_band_width = (double)((int)g_color_band_width) + 1.0;

	return (0);
}


void
make_input_image_tracker (INPUT_DESC *input, int w, int h)
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
init_tracker (INPUT_DESC *input)
{
	int x,y;

	make_input_image_tracker (input, IMAGE_WIDTH, IMAGE_HEIGHT);

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
		init_tracker (input);
	}
	else
	{
		if (status == MOVE)
		{
			check_input_bounds (input, input->wxd, input->wxd);
			glutSetWindow (input->win);
			input_display ();
			update_input_filters();
			filters_update();
			//all_dendrites_update (); 
			//all_neurons_update ();
	
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
		init_tracker (input);
	}
	else
	{
		if (status == MOVE)
		{
			check_input_bounds (input, input->wxd, input->wxd);
			glutSetWindow (input->win);
			input_display ();
			update_input_filters();
			filters_update();
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
output_handler_resize(OUTPUT_DESC *output, int type_call __attribute__ ((unused)), int mouse_button __attribute__ ((unused)), int mouse_state __attribute__ ((unused)))
{
	static int first_time = 1;
	
	if (first_time <= 2)
	{
		glutSetWindow(output->win);
		glutReshapeWindow(1500, 5);
		glutPostRedisplay();
		first_time = first_time + 1;
	}
	
}


void
tracker_saccade (INPUT_DESC *input,
		int max_number_of_saccades, float min_threshold_of_saccade,
		int *x_saccade_vector_out, int *y_saccade_vector_out)
{
	float x, y;
	int count = 0;
	int x_saccade_vector = 0;
	int y_saccade_vector = 0;
	double d_delta_x, d_delta_y;
	int delta_x, delta_y;
	
	// Saccade until reach the target
	x = nl_target_coordinates.neuron_vector[0].output.fval;
	y = nl_target_coordinates.neuron_vector[1].output.fval;

	do
	{
		// Rounding adjustments		
		d_delta_x = x / dynamic_scale_factor;

		if (d_delta_x > 0.0)
			delta_x = (int) (d_delta_x + 0.5);
		else if (d_delta_x < 0.0)
			delta_x = (int) (d_delta_x - 0.5);
		else
			delta_x = 0;
		
		if(input->wxd >= IMAGE_WIDTH)
			input->wxd = IMAGE_WIDTH;
		else if(input->wxd < 0)
			input->wxd = 0;
		else
		{
			input->wxd += delta_x;
			x_saccade_vector += delta_x;
		}

		d_delta_y = y / dynamic_scale_factor;

		if (d_delta_y > 0.0)
			delta_y = (int) (d_delta_y + 0.5);
		else if (d_delta_y < 0.0)
			delta_y = (int) (d_delta_y - 0.5);
		else
			delta_y = 0;

		if (input->wyd >= IMAGE_HEIGHT)
			input->wyd = IMAGE_HEIGHT;
		else if (input->wyd < 0)
			input->wyd = 0;
		else
		{
			input->wyd +=  delta_y;
			y_saccade_vector +=  delta_y;
		}
		
		set_input_layer_translation(input, input->wxd, input->wyd);
filters_update();
				
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
/*void
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
*/


float
tracker_confidence()
{
	return g_confidence;
}


float
tracker_scale()
{	
	return(dynamic_scale_factor);
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
			tracker_train();
			break;
		// Saccade until reach the target
		case 'S':
		case 's':
			tracker_saccade(&in_saccade_current, MAX_NUMBER_OF_SACCADE, MIN_THRESHOLD_OF_SACCADE, NULL, NULL);
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
	ForwardVisualSearchNetwork(NULL);
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

	update_input_filters();
	filters_update();
	//all_dendrites_update (); 
	//all_neurons_update ();

}


void
reset_gaussian_filter_parameters()
{
	g_sigma = sqrt(dynamic_scale_factor * 0.8 * 128.0 / 35.0);
	g_kernel_size = (int) (6.0 * g_sigma);
	g_kernel_size = ((g_kernel_size % 2) == 0)? g_kernel_size + 1: g_kernel_size;

	if (g_sigma < 1.0)
	{
		g_sigma = 1.0;
		g_kernel_size = 5.0;
	}
}

// Update the input layer neurons and Image
void
update_input_layer_neurons_and_image(INPUT_DESC *input)
{
	check_input_bounds (input, input->wx + input->ww/2, input->wy + input->wh/2);
	input->up2date = 0;
	update_input_neurons (input);
	update_input_image (input);
	reset_gaussian_filter_parameters();
	input_filters_and_outputs_update();
}


void
update_input_layer_neurons_and_image_light(INPUT_DESC *input)
{
	check_input_bounds (input, input->wx + input->ww/2, input->wy + input->wh/2);
	input->up2date = 0;
	update_input_neurons (input);
	update_input_image (input);
	update_input_filters();
	reset_gaussian_filter_parameters();
	filters_update();
	//all_dendrites_update (); 
	//all_neurons_update ();
}

/*
int
gt_color_from_x(int x)
{
	return (int)( ( (double)(x)) * 254.0/IMAGE_WIDTH_RESIZED + 0.5);
}


int
gt_color_from_y(int y)
{
	return (int)( ( (double)(y)) * 254.0/IMAGE_HEIGHT_RESIZED + 0.5);
}


int
gt_x_displecement_from_fovea(int green)
{
	return (int)( (double) green * IMAGE_WIDTH_RESIZED/254.0 - IMAGE_WIDTH_RESIZED/2 + 0.5);
}


int
gt_y_displecement_from_fovea(int red)
{
	return (int)( (double) red * IMAGE_HEIGHT_RESIZED/254.0 - IMAGE_HEIGHT_RESIZED/2 + 0.5);
}
*/

int
gt_color_from_x(int x)
{
	return (int)( ( (double)(x - (table.dimentions.x-g_color_band_width)/2)) * 254.0/g_color_band_width + 0.5);
}


int
gt_color_from_y(int y)
{
	return (int)( ( (double)(y - (table.dimentions.y-g_color_band_width)/2)) * 254.0/g_color_band_width + 0.5);
}


int
gt_x_displecement_from_fovea(int green)
{
	return (int)( (double) green * g_color_band_width/254.0 - g_color_band_width/2 + 0.5);
}


int
gt_y_displecement_from_fovea(int red)
{
	return (int)( (double) red * g_color_band_width/254.0 - g_color_band_width/2 + 0.5);
}


int
compute_lim_inf_u(int u_neuron)
{
	if (u_neuron >= NUM_PIXELS_WEIGHT) 
		return (u_neuron - NUM_PIXELS_WEIGHT);
	else 
		return 0;
}


int
compute_lim_inf_v(int v_neuron)
{
	if (v_neuron >= NUM_PIXELS_WEIGHT) 
		return (v_neuron - NUM_PIXELS_WEIGHT);
	else 
		return 0;
}


int
compute_lim_sup_u(int u_neuron, int w)
{
	if (u_neuron < w - NUM_PIXELS_WEIGHT - 1) 
		return (u_neuron + NUM_PIXELS_WEIGHT);
	else 
		return w - 1;
}


int
compute_lim_sup_v(int v_neuron, int h)
{
	if (v_neuron < h - NUM_PIXELS_WEIGHT - 1) 
		return (v_neuron + NUM_PIXELS_WEIGHT);
	else 
		return h - 1;
}


int
compute_lim_inf_u_borda(int u_neuron, int w)
{
	int u_neighbor_neuron;

	u_neighbor_neuron = (w - 1) - u_neuron; 
	return compute_lim_inf_u(u_neighbor_neuron);
}


int
compute_lim_sup_u_borda(int u_neuron, int w)
{
	int u_neighbor_neuron;

	u_neighbor_neuron = (w - 1) - u_neuron; 
	return compute_lim_sup_u(u_neighbor_neuron, w);
}


// deslocamento em x do centro para o neuronio principal
double
compute_desloc_x(int neuron_output, int w, int h, int u_neuron, int v_neuron, double log_factor)
{
	int xi, xi_neuron, yi;
	double dist_x_neuron;

//	xi = gt_x_displecement_from_fovea(GREEN(table_v1.neuron_vector[v_neuron * w + u_neuron].output.ival));
	map_v1_to_image(&xi, &yi, IMAGE_WIDTH_RESIZED, IMAGE_HEIGHT_RESIZED, u_neuron, v_neuron, w, h, 0, 0, 0, log_factor);
	xi_neuron = gt_x_displecement_from_fovea(GREEN(neuron_output));
	dist_x_neuron = (double)(xi - xi_neuron);
	
	return dist_x_neuron;
}


// deslocamento em y do centro para o neuronio principal
double
compute_desloc_y(int neuron_output, int w, int h, int u_neuron, int v_neuron, double log_factor)
{
	int yi, yi_neuron,xi;
	double dist_y_neuron;

//	yi = gt_y_displecement_from_fovea(RED(table_v1.neuron_vector[v_neuron * w + u_neuron].output.ival));
	map_v1_to_image(&xi, &yi, IMAGE_WIDTH_RESIZED, IMAGE_HEIGHT_RESIZED, u_neuron, v_neuron, w, h, 0, 0, 0, log_factor);
	yi_neuron = gt_y_displecement_from_fovea(RED(neuron_output));
	dist_y_neuron = (double)(yi - yi_neuron);

	return dist_y_neuron;
	
}


//diferenca de deslocamentos
double
compute_dif_desloc(double dist_x, double dist_x_neuron, double dist_y, double dist_y_neuron)
{
	double dif_x, dif_y, dist;

	dif_x = (dist_x - dist_x_neuron)*(dist_x - dist_x_neuron);
	dif_y = (dist_y - dist_y_neuron)*(dist_y - dist_y_neuron);
	dist = exp(- (1.0 / 2.0) * ((dif_x + dif_y) / (sigma * sigma)));

	return dist;
}


double
compute_weight (int lim_inf_u_neighbor, int lim_sup_u_neighbor, int lim_inf_v_neighbor, int lim_sup_v_neighbor, int u_neuron, int v_neuron, int w, int h, double dist_x_neuron, double dist_y_neuron, double log_factor)
{			

	int u_neighbor_neuron, v_neighbor_neuron;
	int neighbor_neuron_output;
	double partial_weight = 0.0;
	double dist_x, dist_y;

	for (v_neighbor_neuron = lim_inf_v_neighbor; v_neighbor_neuron <= lim_sup_v_neighbor; v_neighbor_neuron++)
	{   
		for (u_neighbor_neuron = lim_inf_u_neighbor; u_neighbor_neuron <= lim_sup_u_neighbor; u_neighbor_neuron++)
		{
			if ((u_neighbor_neuron != u_neuron) || (v_neighbor_neuron != v_neuron))
			{
				neighbor_neuron_output = nl_v1_activation_map.neuron_vector[v_neighbor_neuron * w + u_neighbor_neuron].output.ival;
				if (BLUE(neighbor_neuron_output) > 0)
				{
					dist_x = compute_desloc_x(neighbor_neuron_output, w, h, u_neighbor_neuron, v_neighbor_neuron, log_factor);
					dist_y = compute_desloc_y(neighbor_neuron_output, w, h, u_neighbor_neuron, v_neighbor_neuron, log_factor);
					partial_weight += compute_dif_desloc(dist_x, dist_x_neuron, dist_y, dist_y_neuron);
				}
			}
		}
	}

	return partial_weight;	
}


double
compute_weigheted_neighborhood(NEURON *n __attribute__ ((unused)), int w, int h, int u_neuron, int v_neuron, double log_factor)
{
	double partial_weight, partial_weight_center;
	int u_neighbor_neuron, v_neighbor_neuron, v_neighbor_neuron_aux;
	int lim_inf_u_neighbor, lim_inf_v_neighbor, lim_sup_u_neighbor, lim_sup_v_neighbor;
	int x_neuron, y_neuron;
	double dist, dist_x, dist_y;
	int neighbor_neuron_output, neuron_output, b_neuron_color;
	double dist_x_neuron;
	double dist_y_neuron;
	int value;

	partial_weight = 0.0;
	partial_weight_center = 0.0;

	neuron_output = nl_v1_activation_map.neuron_vector[v_neuron * w + u_neuron].output.ival;
	x_neuron = IMAGE_WIDTH_RESIZED/2.0 + gt_x_displecement_from_fovea(GREEN(neuron_output));   
	y_neuron = IMAGE_HEIGHT_RESIZED/2.0 + gt_y_displecement_from_fovea(RED(neuron_output));
	value = table.neuron_vector[y_neuron * IMAGE_WIDTH_RESIZED + x_neuron].output.ival;

	if (neuron_output != 0 && value != 0){
		b_neuron_color = BLUE(value);

		dist_x_neuron = compute_desloc_x(neuron_output, w, h, u_neuron, v_neuron, log_factor);
		dist_y_neuron = compute_desloc_y(neuron_output, w, h, u_neuron, v_neuron, log_factor);

		if (b_neuron_color > 0)
		{   
			if ( (v_neuron > NUM_PIXELS_WEIGHT - 1) && (v_neuron < h - NUM_PIXELS_WEIGHT))
			{
				lim_inf_v_neighbor = compute_lim_inf_v(v_neuron);
				lim_inf_u_neighbor = compute_lim_inf_u(u_neuron);
				lim_sup_v_neighbor = compute_lim_sup_v(v_neuron, h);
				lim_sup_u_neighbor = compute_lim_sup_u(u_neuron, w);

				partial_weight += compute_weight(lim_inf_u_neighbor, lim_sup_u_neighbor, lim_inf_v_neighbor, lim_sup_v_neighbor, u_neuron, v_neuron, w, h, dist_x_neuron, dist_y_neuron, log_factor);
			}

			// Borda inferior
			else if ( (v_neuron <= NUM_PIXELS_WEIGHT) || (v_neuron >= h - NUM_PIXELS_WEIGHT))
			{
				lim_inf_v_neighbor = v_neuron - NUM_PIXELS_WEIGHT;
				lim_sup_v_neighbor = v_neuron + NUM_PIXELS_WEIGHT;

				for (v_neighbor_neuron_aux = lim_inf_v_neighbor; v_neighbor_neuron_aux <= lim_sup_v_neighbor; v_neighbor_neuron_aux++)
				{   
					if ( v_neighbor_neuron_aux < 0 )
					{
						v_neighbor_neuron = -v_neighbor_neuron_aux - 1;
						lim_inf_u_neighbor = compute_lim_inf_u_borda(u_neuron, w);
						lim_sup_u_neighbor = compute_lim_sup_u_borda(u_neuron, w);
					}
	
					else if ( v_neighbor_neuron_aux >= h )
					{
						v_neighbor_neuron = h - 1 - (v_neighbor_neuron_aux-h);
						lim_inf_u_neighbor = compute_lim_inf_u_borda(u_neuron, w);
						lim_sup_u_neighbor = compute_lim_sup_u_borda(u_neuron, w);
					}
					else
					{
						v_neighbor_neuron = v_neighbor_neuron_aux;
						lim_inf_u_neighbor = compute_lim_inf_u(u_neuron);
						lim_sup_u_neighbor = compute_lim_sup_u(u_neuron, w);
					}

					for (u_neighbor_neuron = lim_inf_u_neighbor; u_neighbor_neuron <= lim_sup_u_neighbor; u_neighbor_neuron++)
					{
						neighbor_neuron_output = nl_v1_activation_map.neuron_vector[v_neighbor_neuron * w + u_neighbor_neuron].output.ival;
						if (BLUE(neighbor_neuron_output) > 0)
						{
							dist_x = compute_desloc_x(neighbor_neuron_output, w, h, u_neighbor_neuron, v_neighbor_neuron, log_factor);
							dist_y = compute_desloc_y(neighbor_neuron_output, w, h, u_neighbor_neuron, v_neighbor_neuron, log_factor);
							partial_weight += compute_dif_desloc(dist_x, dist_x_neuron, dist_y, dist_y_neuron);

							if ((u_neighbor_neuron != u_neuron) || (v_neighbor_neuron != v_neuron))
							{
								partial_weight_center = dist;
							}
						}
					}
				}
				partial_weight -= partial_weight_center;
			}
		}
	}

	return partial_weight / ((2*NUM_PIXELS_WEIGHT + 1)*(2*NUM_PIXELS_WEIGHT + 1)-1);
}



double
compute_weigheted_max_neighbor_linear(double *weigheted_xi, double *weigheted_yi, NEURON *n, int w, int h, double log_factor __attribute__ ((unused)))
{
	int u, v;
	double xi_max = 0.0, yi_max = 0.0;
	double value, value_max = -1, count = 1.0;

	for (v = 0; v < h; v++)
	{
		for (u = 0; u < w; u++)
		{
			value = (float)n[v * w + u].output.fval;
			if (value > value_max)
			{
				value_max = value;
				xi_max = u;
				yi_max = v;
				count = 1.0;
			}
			else if (value == value_max)
			{
				xi_max += u;
				yi_max += v;
				count += 1.0;
			}
		}
	}
	g_confidence = value_max;
	*weigheted_xi = xi_max/count - w/2;
	*weigheted_yi = yi_max/count - h/2;

	return (g_confidence);
}


void 
output_handler_weighted_average_value_position(OUTPUT_DESC *output, int type_call __attribute__ ((unused)), int mouse_button __attribute__ ((unused)), int mouse_state __attribute__ ((unused)))
{
	char *nl_target_coordinates_name = NULL;
	int w, h;
	double xi, yi;
	double log_factor;
	NEURON_LAYER *nl_target_coordinates = NULL;
		
	// Gets the output handler parameters
	nl_target_coordinates_name = output->output_handler_params->next->param.sval;
	log_factor = output->output_handler_params->next->next->param.fval;
	
	// Gets the target coordinates neuron layer
	nl_target_coordinates = get_neuron_layer_by_name(nl_target_coordinates_name);
	
	// Gets the Neuron Layer Dimentions
	w = output->neuron_layer->dimentions.x;
	h = output->neuron_layer->dimentions.y;
	
	compute_weigheted_max_neighbor_linear(&xi, &yi, output->neuron_layer->neuron_vector, w, h, log_factor);

	// Saves the max value position
	nl_target_coordinates->neuron_vector[0].output.fval = xi;
	nl_target_coordinates->neuron_vector[1].output.fval = yi;

}

void
generate_color_linear_map(NEURON_LAYER *nl_landmark_eval_mask)
{
	int w, h;
	int x, y;
	double dist_2_center;
	NEURON *neuron_vector;
	int r, g, b;

	w = nl_landmark_eval_mask->dimentions.x;
	h = nl_landmark_eval_mask->dimentions.y;
	neuron_vector = nl_landmark_eval_mask->neuron_vector;

	for (y = 0; y < h; y++)
	{
		for (x = 0; x < w; x++)
		{
			if ( sqrt( (x - w/2)*(x - w/2) + (y - h/2)*(y - h/2) ) < (g_color_band_width/2) )
			{
				g = gt_color_from_x(x);
				r = gt_color_from_y(y);
				dist_2_center = sqrt( (double)( (x-w/2)*(x-w/2) + (y-h/2)*(y-h/2) ) ); 
				b = 255 - (int)( dist_2_center * 255.0/(g_color_band_width/2.0) );
			}
			else
			{
				r = g = b = 0;
			}

			neuron_vector[y * w + x].output.ival = PIXEL (r, g, b);
		}
	}
}



//CUIDADO QDO FOR FAZER RETREINO
void
mask_output_color(NEURON_LAYER *nl_color, NEURON_LAYER *nl_masked, double log_factor)
{
	int w, h;
	int x, y, u, v;
	double ground_truth_w;
	double ground_truth_h;
	double leftCol_gt;
	double rightCol_gt;
	double bottomRow_gt;
	double topRow_gt;

//	ground_truth_w = width_in_train * dynamic_scale_factor;
//	ground_truth_h = height_in_train * dynamic_scale_factor;


	if (dynamic_scale_factor_init == -1.0)
	{
		ground_truth_w = width_in_train * dynamic_scale_factor;
		ground_truth_h = height_in_train * dynamic_scale_factor;
	}
	else
	{
		ground_truth_w = (int) ((dynamic_scale_factor_init / dynamic_scale_factor) * width_in_train + 0.5);
		ground_truth_h = (int) ((dynamic_scale_factor_init / dynamic_scale_factor) * height_in_train + 0.5);
	}


	w = nl_masked->dimentions.x;
	h = nl_masked->dimentions.y;

	leftCol_gt = ((double)(w/2.0)) - ((double)((ground_truth_w / 2.0) + 0.5));
	rightCol_gt = ((double)(w/2.0)) + ((double)((ground_truth_w / 2.0) - 0.5));
	bottomRow_gt = ((double)(h/2.0)) - ((double)((ground_truth_h / 2.0) + 0.5));
	topRow_gt = ((double)(h/2.0)) + ((double)((ground_truth_h / 2.0) - 0.5));

	for (v = 0; v < h; v++)
	{
		for (u = 0; u < w; u++)
		{
			map_v1_to_image(&x, &y, IMAGE_WIDTH_RESIZED, IMAGE_HEIGHT_RESIZED, u, v, w, h, (w/2), (h/2), 0, log_factor);
			if (((double) x >= leftCol_gt) &&
			    ((double) x <= rightCol_gt) &&
			    ((double) y >= bottomRow_gt) &&
			    ((double) y <= topRow_gt))
			{
				nl_masked->neuron_vector[v * w + u].output = nl_color->neuron_vector[v * w + u].output;
			}
			else
				nl_masked->neuron_vector[v * w + u].output.ival = 0.0;
		}
	}
}


			
// Trains the tracker module
void
tracker_train(void)
{
	long num_neurons;
	int i;

	if (dynamic_scale_factor_init == -1.0)
	{
		dynamic_scale_factor_init = dynamic_scale_factor;
		generate_color_linear_map(&table);
		scale_before = dynamic_scale_factor;
	}

   //filters_update();
	forward_objects((char *)"table_v1_filter");
	mask_output_color(&table_v1, &nl_v1_activation_map, LOG_FACTOR);

	//filters_update();
	//all_outputs_update();


	num_neurons = get_num_neurons (nl_v1_activation_map.dimentions);
	for (i = 0; i < num_neurons; i++)
		train_neuron(&nl_v1_activation_map, i);

	save_gaussian_filtered_training_pattern(&nl_v1_activation_map);
	filters_update();
	all_outputs_update();

}


NEURON_OUTPUT
run_train(PARAM_LIST *pParamList __attribute__ ((unused)))
{
	NEURON_OUTPUT output;
	output.ival = 0;

	int i;
	for (i = 0; i<32; i++){
	tracker_train();
	}
	
	return (output);
}


NEURON_OUTPUT
run_test(PARAM_LIST *pParamList __attribute__ ((unused)))
{
	NEURON_OUTPUT output;
	output.ival = 0;

	tracker_saccade(&in_saccade_current, MAX_NUMBER_OF_SACCADE, MIN_THRESHOLD_OF_SACCADE, NULL, NULL);

	return (output);
}


void
set_scale_before()
{
	dynamic_scale_factor = scale_before;
	reset_gaussian_filter_parameters();
	ForwardVisualSearchNetwork(NULL);
}


void
set_scale(float scale)
{
	dynamic_scale_factor = scale;
	reset_gaussian_filter_parameters();
	ForwardVisualSearchNetwork(NULL);

}


float
GetScaleFactorZoom()
{
	double x;
	int i;
	double count;
	float maior, z;
	float multiplicador;
	int zoom = 0;

	x = 0.0;
	count = 0.0;
	maior = 0.0;

	for (i = 0; i < TAM_NL_ZOOM; i++)
	{
		z = nl_activation_map_hough_zoom_gaussian.neuron_vector[i].output.fval;
		if ( z != 0)
		{
			x += (double)i * z;
			count += z;
			if (z > maior)
			{
				maior = z;
			}
		}
	}

	if (count != 0.0)
	{
		if (x/count == 0.0)
			zoom = 0;
		else
		{
			if ((x/count >= (float)(TAM_NL_ZOOM/2 - BAND_ZOOM)) && (x/count <= (float)(TAM_NL_ZOOM/2 + BAND_ZOOM)))
				zoom = 0;
			else
			{
				if ((int)(x/count) > (TAM_NL_ZOOM/2))
					zoom = -1;
				else
					zoom = 1;
			}
		}
	}
	else
		zoom = 0;

	if ((g_confidence_zoom_init - maior) > 0)
	{
		multiplicador = (g_confidence_zoom_init - maior) / g_confidence_zoom_init; 
	}
	else
		multiplicador = 1.0;


	//printf("multiplicador = %f, zoom = %d, maior = %f\n", multiplicador, zoom, maior);
	if (zoom == 0)
		dynamic_scale_factor = dynamic_scale_factor;
	else if (zoom > 0)
		dynamic_scale_factor = dynamic_scale_factor / (1 + (0.1 * multiplicador));
	else
		dynamic_scale_factor = dynamic_scale_factor / (1 - (0.1 * multiplicador));


	return dynamic_scale_factor;
}


int 
save_image(INPUT_DESC *input)
{
	int w, h;
	char file_name[256];
	FILE *image_file = (FILE *) NULL;
	GLubyte *image;
	int i, j, tfw;
	int r, g, b;
	
	strcpy (file_name, "last_image");
	strcat (file_name, ".pnm");
        if ((image_file = fopen (file_name, "w")) == (FILE *) NULL)
       		Erro ((char *)"Cannot create file: ", file_name, (char *)"");

	// Write the image format and comment
	fprintf (image_file, "P3\n# CREATOR: MAE save_image ()\n");	

	h = input->neuron_layer->dimentions.y;
	w = input->neuron_layer->dimentions.x;

	// Write the image dimentions and range
	fprintf (image_file, "%d %d\n%d\n", w, h, 255);
	image = input->image;
	tfw = input->tfw;
		
	for (i = h - 1; i >= 0; i--)  
	{
		for (j = 0; j < w; j++) 
		{
			r = image[3 * (i * tfw + j) + 0];
			g = image[3 * (i * tfw + j) + 1];
			b = image[3 * (i * tfw + j) + 2];
			fprintf (image_file, "%d\n%d\n%d\n", r, g, b);
		}
	}
	
	// Closes image file
	fclose (image_file);

	return (0);
}


void
MoveToPoint(INPUT_DESC *input, int x, int y)
{
	input->wxd = x;
	input->wyd = y;

	set_input_layer_translation(input, input->wxd, input->wyd);
}


void
visual_search(INPUT_DESC *input __attribute__ ((unused)))
{

	int i;
	float z;
	float maior = 0.0;
	float scale;
	int retrain = 0;
	int x_sacade, y_sacade;
	static int count = 1;

	//sacada para ajuste de zoom
	//zoom anterior
	set_scale_before(); 
	tracker_saccade(&in_saccade_current, MAX_NUMBER_OF_SACCADE, MIN_THRESHOLD_OF_SACCADE, NULL, NULL);

	//confianca na 1a sacada
	if (g_confidence_zoom_init == -1.0)
	{
		for (i = 0; i < TAM_NL_ZOOM; i++)
		{
			z = nl_activation_map_hough_zoom_gaussian.neuron_vector[i].output.fval;
			if ( z != 0)
			{
				if (z > maior)
				{
					maior = z;
				}	
			}
		}
		g_confidence_zoom_init = maior;
	}

	//ajuste do zoom atual
	scale = GetScaleFactorZoom();
	printf("dynamic_scale_factor = %f\n", dynamic_scale_factor);
	set_scale(scale);

	//sacada
	tracker_saccade(&in_saccade_current, MAX_NUMBER_OF_SACCADE, MIN_THRESHOLD_OF_SACCADE, NULL, NULL);
	x_sacade = in_saccade_current.wxd; 
	y_sacade = in_saccade_current.wyd;

	printf("g_confidence = %f\n", g_confidence);
	//retreino é necessário ?
	if ((int)g_confidence < MIN_CONFIANCE_TO_RETRAIN)
		retrain = 1;

	printf("x = %d, y = %d, x_before = %d, y_before = %d\n", in_saccade_current.wxd, in_saccade_current.wyd, x_before, y_before);
	//salvando a imagem anterior para ser utilizada no retreino
	save_image(input);

	//se o retreino é necessario
	if (retrain == 1)
	{
		if ((count%10 == 0))
		{
			load_input_image(input, (char *)"last_image.pnm");
			update_input_layer_neurons_and_image_light(input);
			set_scale_before(); 
			printf("TREINANDO!!!!!!, g_confidence = %f\n", g_confidence);
			MoveToPoint(input, x_before, y_before);
			tracker_train();
			//ForwardVisualSearchNetwork(NULL);
		}
		retrain = 0;
	}

	x_before = x_sacade; 
	y_before = y_sacade;
	scale_before = scale;

	//contador
	if (count < 10)
		count = count + 1; 
	else
		count = 1;
}



