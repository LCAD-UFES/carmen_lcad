#include <locale.h>
#include <time.h>	// For time measuraments
#include "visual_search_user_functions.h"
#include "visual_search_filters.h"

#define RAMDOM_FACES_TEST	"random_faces_t.txt"
#define RAMDOM_FACES_RECALL	"random_faces_r.txt"

// For coordinating message transfers from CARMEN modules
carmen_visual_search_state_change_message	MAE_visual_search_state_change_message;
carmen_visual_search_message			MAE_visual_search_training_message;
carmen_visual_search_test_message		MAE_visual_search_test_message;
carmen_visual_search_output_message		MAE_visual_search_output_message;

// CARMEN message flags
int	new_VS_state_change_message = FALSE;
int	new_VS_training_message = FALSE;
int	new_VS_test_message = FALSE;

// Visual Search state flag
VISUAL_SEARCH_STATE	visual_search_state;

// Masking point list
MASK_POINT_LIST		*visual_search_mask_point_list = NULL;

// Saccade counters
int		last_count_number;
float		saccade_moving_average = MOVING_AVERAGE_INTIAL_VALUE;

float		*gaussian_filtered_training_pattern = NULL; 		//training image (also used in filter module)
float		gaussian_filtered_training_pattern_total_weight = 0.0f;	//total filtered training pattern acumulated weight
float		training_pattern_max_val = 0.0f;		//training image max value
float		training_pattern_min_val = 0.0f;		//training image min value

int		g_nNetworkStatus;
double		network_certainty;

// For hashing the wisard neuron output values
float	wisard_neuron_output_hash[NL_WIDTH];

// Output log file descriptor
FILE		*output_log_file = NULL;
// Comparison file descriptor
FILE		*comparison_log_file = NULL;
// Global hit rate comparison
float		global_hit_rate;
// Total Number of Saccades (for performance measuraments)
int		total_number_of_saccades;
// Timekeeping structs
struct timespec		start,end;

// For Time Interval Measuraments
unsigned long int timespecDiff(struct timespec *timeA_p, struct timespec *timeB_p)
{
	return (timeA_p->tv_sec - timeB_p->tv_sec);
}

void	get_output_message(carmen_visual_search_output_message *message)
{
	if (message)
	{
		*message = MAE_visual_search_output_message;
	}
}

/*
********************************************************
* Function: draw_output                                *
* Description:                                         *
* Inputs:                                              *
* Output: none                                         *
********************************************************
*/

/* this function is required by the MAE core modules */
void draw_output (char *output_name, char *input_name)
{
	output_name = output_name; 	/* For Keeping The compiler Happy */
	input_name = input_name;	/* For Keeping The compiler Happy */

        return;
}

/*
********************************************************
* Function: conditional_filter_update 		       *
* Description:  				       *
* Inputs: filter  				       *
* Output:					       *
********************************************************
*/

int	conditional_filter_update(FILTER_DESC	*filter)
{
	if(filter)
	{
		filter_update(filter);
		return(0);
	}
	else
		return(-1);
}

void ipc_timer_function(int value)
{
	IPC_listenWait((unsigned int)10);
	glutTimerFunc(10, ipc_timer_function, value);
}
/*
********************************************************
* Function: init_user_functions 		       *
* Description:  				       *
* Inputs: none  				       *
* Output:					       *
********************************************************
*/


int init_user_functions ()
{
	char *locale_string;

	/* Neural network not started yet */
	visual_search_state = NON_STARTED;
	
	/* connect to IPC server */
	carmen_ipc_initialize(global_argc, global_argv);
	carmen_param_check_version(global_argv[0]);

	/* register output message */
	carmen_visual_search_define_message_output();
		
	/* subscribe to training message */
	carmen_visual_search_subscribe_train(NULL, (carmen_handler_t)training_message_handler, CARMEN_SUBSCRIBE_LATEST);
	
	/* subscribe to state change message */
	carmen_visual_search_subscribe_state_change(NULL, (carmen_handler_t)state_change_message_handler, CARMEN_SUBSCRIBE_LATEST);
	
	/* subscribe to query output message */
	carmen_visual_search_subscribe_query(query_message_handler);

	/* subscribes to testing messages (if directly using CML based scripts) */
	#ifndef CML_SCRIPT
	carmen_visual_search_subscribe_test(NULL, (carmen_handler_t)test_message_handler, CARMEN_SUBSCRIBE_LATEST);
	#endif

	/* subscribes to bumblebee camera message (if directly using CML based scripts) */
	#ifdef CML_SCRIPT
	carmen_bumblebee_basic_subscribe_stereoimage(CAMERA, NULL, (carmen_handler_t)bumblebee_image_handler, CARMEN_SUBSCRIBE_ALL);
	#endif
	
	/* Set the total number of saccades towards zero */
	total_number_of_saccades = 0;
	
	locale_string = setlocale (LC_ALL, "C");
	if (locale_string == NULL)
	{
	        fprintf (stderr, "Could not set locale.\n");
	        exit (1);
	}
	else
        	printf ("Locale set to %s.\n", locale_string);
	
	interpreter ("toggle move_active;");
	
	interpreter ("toggle draw_active;");
	
	//Now the neural network is Wating for Training
	visual_search_state = WAITING_FOR_TRAINNING;

	return (0);
}


void make_input_image_visual_search (INPUT_DESC *input, int w, int h)
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
			Erro ("Invalid Type Show ", message, " Error in update_input_image.");
			return;
	}
	
	input->vpxo = 0;
	input->vpyo = h - input->vph;

	if(input->image == NULL)
		input->image = (GLubyte *) alloc_mem (input->tfw * input->tfh * 3 * sizeof (GLubyte));
}


/*
********************************************************
* Function: init_visual_search			       *
* Description: initialize variables, structures and    *
*	       program procedures		       *
* Inputs: input layer				       *
* Output: none  				       *
********************************************************
*/

void init_visual_search (INPUT_DESC *input)
{
	int x,y;

	g_nStatus = MOVING_PHASE;
	 
	make_input_image_visual_search (input, IMAGE_WIDTH, IMAGE_HEIGHT);

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
#ifndef CML_SCRIPT
	glutTimerFunc(10, ipc_timer_function, 1);
#endif
}



/*
********************************************************
* Function: input_generator			       *
* Description: pattern generator		       *
* Inputs: input layer, status			       *
* Output: none  				       *
********************************************************
*/

void input_generator (INPUT_DESC *input, int status)
{

	if (input->win == 0)
		init_visual_search (input);
	else
	{
		if (status == MOVE)
		{
			check_input_bounds (input, input->wxd, input->wxd);
			glutSetWindow (input->win);
			input_display ();
			filter_update(get_filter_by_name("in_pattern_translated_filter"));
			filter_update(get_filter_by_name("in_pattern_translated_scaled_filter"));
			filter_update(get_filter_by_name("in_pattern_translated_scaled_filtered_filter"));
			filter_update(get_filter_by_name("in_pattern_translated_scaled_filtered_inhibited_filter"));
			all_dendrites_update (); 
			all_neurons_update ();
			//conditional_filter_update(get_filter_by_name("nl_v1_activation_map_hamming_inhibited_filter"));
			filter_update(get_filter_by_name("nl_v1_activation_map_f_filter"));	//this filter is obbligatory
			conditional_filter_update(get_filter_by_name("hamming_distance_layer_filter"));
			all_outputs_update ();
		}
	}
}

/*
***********************************************************************************
* Function: output_handler_port_and_wurtz_weighted_vector_average_position        *
* Description:                                         				  *
* Inputs:                                              				  *
* Output:                                              				  *
***********************************************************************************
*/

void output_handler_port_and_wurtz_weighted_vector_average_position (OUTPUT_DESC *output, int type_call, int mouse_button, int mouse_state)
{
	char *nl_target_coordinates_name = NULL;
	int u, v, w, h, xi, yi; //, step;
	float log_factor;
	float highest_neuron_layer_output;
	int	number_of_active_neurons;
	NEURON_LAYER *nl_target_coordinates = NULL;

	type_call = type_call;		/* For Keeping The compiler Happy */
	mouse_button = mouse_button;	/* For Keeping The compiler Happy */
	mouse_state = mouse_state;	/* For Keeping The compiler Happy */
	
	// Vector weight accumulators
	float   x_accumulator;
	float   y_accumulator;
		
	// Gets the output handler parameters
	nl_target_coordinates_name = output->output_handler_params->next->param.sval;
	log_factor = output->output_handler_params->next->next->param.fval;

	// Gets the Neuron Layer Dimentions
        w = output->neuron_layer->dimentions.x;
	h = output->neuron_layer->dimentions.y;

	// Gets the target coordinates neuron layer
	nl_target_coordinates = get_neuron_layer_by_name (nl_target_coordinates_name);

	// Highest output activation value for neuron layer
	highest_neuron_layer_output = output->neuron_layer->max_neuron_output.fval;
	
	// Accumulator values initialized
	x_accumulator = 0.0;
       	y_accumulator = 0.0;
       	number_of_active_neurons = 0;
       	
	// Makes the weighted vector summation
	for (v = 0 ; v < h; v++)
        {
		for (u = 0; u < w; u++)
		{
			map_v1_to_image (&xi, &yi, IMAGE_WIDTH, IMAGE_HEIGHT, u, v, w , h, 0, 0, (double) h / (double) (h - 1), log_factor);
			if(output->neuron_layer->neuron_vector[v * w + u].output.fval > 0.0)	// if non-zero activation value
			{
				x_accumulator += ((float) xi)*(output->neuron_layer->neuron_vector[v * w + u].output.fval)/highest_neuron_layer_output;	//accumulates the normalized output for x direction
				y_accumulator += ((float) yi)*(output->neuron_layer->neuron_vector[v * w + u].output.fval)/highest_neuron_layer_output;	//accumulates the normalized output for y direction
				number_of_active_neurons++;
			}
		}
	}

	// Averaged on the number of active neurons
	x_accumulator /= ((float) number_of_active_neurons);
	y_accumulator /= ((float) number_of_active_neurons);

        // Saves the centroid position
	nl_target_coordinates->neuron_vector[0].output.fval = (float)( ((1.0/SCALE_FACTOR) * x_accumulator));
	nl_target_coordinates->neuron_vector[1].output.fval = (float)( ((1.0/SCALE_FACTOR) * y_accumulator));
}

/*
********************************************************
* Function: output_handler_max_value_position	       *
* Description:  				       *
* Inputs:					       *
* Output:					       *
********************************************************
*/

void output_handler_max_value_position (OUTPUT_DESC *output, int type_call, int mouse_button, int mouse_state)
{
	char *nl_target_coordinates_name = NULL;
	int u, v, u_max, v_max, w, h, xi, yi; //, step;
	float current_value, max_value = FLT_MIN, log_factor;
	NEURON_LAYER *nl_target_coordinates = NULL;
	
	type_call = type_call;		/* For Keeping The compiler Happy */
	mouse_button = mouse_button;	/* For Keeping The compiler Happy */
	mouse_state = mouse_state;	/* For Keeping The compiler Happy */
	
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
			//current_value = (float) output->neuron_layer->neuron_vector[v * w + u].output.ival;
			current_value = output->neuron_layer->neuron_vector[v * w + u].output.fval;
			
			if (max_value < current_value)
			{
				max_value = current_value;
				u_max = u;
				v_max = v;
			}
		}
	}
	
	// Saves the max value
	global_max_value = max_value;
	
	// Map the max value coordinates to image
	map_v1_to_image (&xi, &yi, IMAGE_WIDTH, IMAGE_HEIGHT, u_max, v_max, w, h, 0, 0, (double) h / (double) (h - 1), log_factor);
	
	// Saves the max value position
	nl_target_coordinates->neuron_vector[0].output.fval = (1.0/SCALE_FACTOR) * ( (float) xi);
	nl_target_coordinates->neuron_vector[1].output.fval = (1.0/SCALE_FACTOR) * ( (float) yi);
}


/*
*********************************************************************************
* Function: output_handler_max_value_position_outside_activation_band 	       	*
* Description:  				       				*
* Inputs:					       				*
* Output:					       				*
*********************************************************************************
*/

void output_handler_max_value_position_outside_activation_band (OUTPUT_DESC *output, int type_call, int mouse_button, int mouse_state)
{
	char *nl_target_coordinates_name = NULL;
	int u, v, u_max, v_max, w, h, xi, yi; //, step;
	float current_value, max_value = FLT_MIN, log_factor;
	NEURON_LAYER *nl_target_coordinates = NULL;
	
	type_call = type_call;		/* For Keeping The compiler Happy */
	mouse_button = mouse_button;	/* For Keeping The compiler Happy */
	mouse_state = mouse_state;	/* For Keeping The compiler Happy */
	
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
			// Searches only outside the activation bandwith region
			if( u < (w - ACT_BAND_WIDTH)/2 ||  u > (w + ACT_BAND_WIDTH)/2 )
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
	}
	
	// Saves the max value
	global_max_value = max_value;
	
	// Map the max value coordinates to image
	map_v1_to_image (&xi, &yi, IMAGE_WIDTH, IMAGE_HEIGHT, u_max, v_max, w, h, 0, 0, (double) h / (double) (h - 1), log_factor);
	
	// Saves the max value position
	nl_target_coordinates->neuron_vector[0].output.fval = (1.0/SCALE_FACTOR) * ( (float) xi);
	nl_target_coordinates->neuron_vector[1].output.fval = (1.0/SCALE_FACTOR) * ( (float) yi);
}

/*
*****************************************************************
* Function: output_handler_max_value_position_thresholded	*
* Description:  				       		*
* Inputs:					       		*
* Output:					       		*
*****************************************************************
*/

void output_handler_max_value_position_thresholded (OUTPUT_DESC *output, int type_call, int mouse_button, int mouse_state)
{
	if(saccade_moving_average < MOVING_AVERAGE_THRESHOLD)
		output_handler_max_value_position_outside_activation_band (output,type_call,mouse_button,mouse_state);
	else
		output_handler_max_value_position (output,type_call,mouse_button,mouse_state);
}

/*
********************************************************
* Function: output_handler_min_value_position	       *
* Description:  				       *
* Inputs:					       *
* Output:					       *
********************************************************
*/

void output_handler_min_value_position (OUTPUT_DESC *output, int type_call, int mouse_button, int mouse_state)
{
	char *nl_target_coordinates_name = NULL;
	int u, v, u_min, v_min, w, h, xi, yi; //, step;
	float current_value, min_value = FLT_MAX, log_factor;
	NEURON_LAYER *nl_target_coordinates = NULL;
	
	type_call = type_call;		/* For Keeping The compiler Happy */
	mouse_button = mouse_button;	/* For Keeping The compiler Happy */
	mouse_state = mouse_state;	/* For Keeping The compiler Happy */
		
	// Gets the output handler parameters
	nl_target_coordinates_name = output->output_handler_params->next->param.sval;
	log_factor = output->output_handler_params->next->next->param.fval;
	
	// Gets the target coordinates neuron layer
	nl_target_coordinates = get_neuron_layer_by_name (nl_target_coordinates_name);
	
	// Gets the Neuron Layer Dimentions
	w = output->neuron_layer->dimentions.x;
	h = output->neuron_layer->dimentions.y;
	
	// Finds the min value position
	for (v = 0, u_min = v_min = 0; v < h; v++)
	{
		for (u = 0; u < w; u++)
		{	
			current_value = output->neuron_layer->neuron_vector[v * w + u].output.fval;

			if (min_value > current_value)
			{
				min_value = current_value;
				u_min = u;
				v_min = v;
			}
		}	
	}
	
	// Map the max value coordinates to image
	map_v1_to_image (&xi, &yi, IMAGE_WIDTH, IMAGE_HEIGHT, u_min, v_min, w, h, 0, 0, (double) h / (double) (h - 1), log_factor);

	// Correct the output coordinates according to the log-polar connection scale factor
	//xi = IMAGE_WIDTH/2 + (xi - IMAGE_WIDTH/2)*(1.0/SCALE_FACTOR);
	//yi = IMAGE_HEIGHT/2 + (yi - IMAGE_HEIGHT/2)*(1.0/SCALE_FACTOR);
	
	// Saves the max value position
	nl_target_coordinates->neuron_vector[0].output.fval = (1.0/SCALE_FACTOR) * ( (float) xi);
	nl_target_coordinates->neuron_vector[1].output.fval = (1.0/SCALE_FACTOR) * ( (float) yi);
}


float confidence;

/*
*********************************************************************************
* Function: output_handler_mean_position					*
* Description:                                        				*
* Inputs:									*
* Output:									*
*********************************************************************************
*/

void output_handler_mean_position (OUTPUT_DESC *output, int type_call, int mouse_button, int mouse_state)
{
	char *nl_target_coordinates_name = NULL;
	int u, v, /* u_max, v_max,*/ w, h, xi, yi;
	float x_mean, y_mean, weight, accumulator, log_factor, cut_point, /*current_value, max_value,*/ band_width, confidence_count;
	NEURON_LAYER *nl_target_coordinates = NULL;
	
	type_call = type_call;		/* For Keeping The compiler Happy */
	mouse_button = mouse_button;	/* For Keeping The compiler Happy */
	mouse_state = mouse_state;	/* For Keeping The compiler Happy */
	
	// Gets the output handler parameters
	nl_target_coordinates_name = output->output_handler_params->next->param.sval;
	log_factor = output->output_handler_params->next->next->param.fval;
	cut_point = output->output_handler_params->next->next->next->param.fval;
	band_width = output->output_handler_params->next->next->next->next->param.fval;
	
	// Gets the target coordinates neuron layer
	nl_target_coordinates = get_neuron_layer_by_name (nl_target_coordinates_name);
	
	// Gets the neuron layer dimentions
	w = output->neuron_layer->dimentions.x;
	h = output->neuron_layer->dimentions.y;
	
	// Initialize the mean coordinates 
	x_mean = 0.0;
 	y_mean = 0.0;

	cut_point = cut_point * (output->neuron_layer->max_neuron_output.fval - output->neuron_layer->min_neuron_output.fval) + output->neuron_layer->min_neuron_output.fval;
	
	confidence_count = confidence = accumulator = 0.0;
	for (v = 0; v < h; v++)
	{
		for (u = 0; u < w; u++)
		{
			weight = output->neuron_layer->neuron_vector[v * w + u].output.fval;

			if ((u >= (int) ((0.5 - 0.5 * band_width) * (float) w)) && (u < (int) ((0.5 + 0.5 * band_width) * (float) w)))
			{
				confidence += weight;
				confidence_count += 1.0;
			}
			
			if (weight < cut_point)
				continue;
				
			// Map the Mean Coordinates to Image
			map_v1_to_image (&xi, &yi, IMAGE_WIDTH, IMAGE_HEIGHT, u, v, w, h, 0, 0, (double) h / (double) (h - 1), log_factor);

			x_mean += (double) xi * weight;
			y_mean += (double) yi * weight;
			accumulator += weight;
		}
	}
	
	// Normalize the mean coordinates 
	x_mean /= accumulator;
	y_mean /= accumulator;

	// Saves the mean coordinates
	nl_target_coordinates->neuron_vector[0].output.fval = (1.0/SCALE_FACTOR) * x_mean;
	nl_target_coordinates->neuron_vector[1].output.fval = (1.0/SCALE_FACTOR) * y_mean;	
	
	confidence = confidence / confidence_count;
	printf ("confidence = %f\n", confidence);
}



/*
********************************************************
* Function: jump				       *
* Description:   				       *
* Inputs: input			 		       *
* Output: none  				       *
********************************************************
*/

void jump (INPUT_DESC *input)
{
	int x, y;
	
	// Jump to the target
	x = (int) nl_target_coordinates.neuron_vector[0].output.fval;
	y = (int) nl_target_coordinates.neuron_vector[1].output.fval;
		
	input->wxd += x;
	input->wyd += y;

	translation_filter_deltaX = (float)(-IMAGE_WIDTH/2.0 + input->wxd);
	translation_filter_deltaY = (float)(-IMAGE_HEIGHT/2.0 + input->wyd);
	move_input_window (input->name, input->wxd, input->wyd);
	
	return;
}



/*
********************************************************
* Function: saccadic_movement                          *
* Description:                                         *
* Inputs: input                                        *
* Output: none                                         *
********************************************************
*/

void	saccadic_movement (INPUT_DESC *input)
{
	float x, y;

	x = nl_target_coordinates.neuron_vector[0].output.fval;
	y = nl_target_coordinates.neuron_vector[1].output.fval;
	x = (x > 0.0)? x + 0.5: x - 0.5;
	y = (y > 0.0)? y + 0.5: y - 0.5;
	input->wxd += (int) x;
	input->wyd += (int) y;
	translation_filter_deltaX = (float)(-IMAGE_WIDTH/2.0 + input->wxd);
	translation_filter_deltaY = (float)(-IMAGE_HEIGHT/2.0 + input->wyd);
	move_input_window (input->name, input->wxd, input->wyd);

	return;
}

/*
********************************************************
* Function: saccade				       *
* Description:   				       *
* Inputs: input			 		       *
* Output: none  				       *
********************************************************
*/

void saccade (INPUT_DESC *input)
{
	float x, y;
	int count = 0;
	
	// Saccade until reach the target
	x = nl_target_coordinates.neuron_vector[0].output.fval;
	y = nl_target_coordinates.neuron_vector[1].output.fval;

	do
	{
		x = (x > 0.0)? x + 0.5: x - 0.5;
		y = (y > 0.0)? y + 0.5: y - 0.5;
		input->wxd += (int) x;
		input->wyd += (int) y;
		translation_filter_deltaX = (float)(-IMAGE_WIDTH/2.0 + input->wxd);
		translation_filter_deltaY = (float)(-IMAGE_HEIGHT/2.0 + input->wyd);

		move_input_window (input->name, input->wxd, input->wyd);

		count++;
		
		x = nl_target_coordinates.neuron_vector[0].output.fval;
		y = nl_target_coordinates.neuron_vector[1].output.fval;
	} 
	while (((fabs(x) > 0.5) || (fabs(y) > 0.5)) && (count < 4));

	return;
}


/*
********************************************************
* Function: saccade				       *
* Description:   				       *
* Inputs: input, number_of_saccades		       *
* Output: none  				       *
********************************************************
*/

void repetitive_saccade (INPUT_DESC *input,int max_number_of_saccades)
{
	float x, y, delta;
	int count = 0;
	int x_saccade_vector = 0;
	int y_saccade_vector = 0;
	
	last_count_number = 0;
	
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
		
		// Filter translation adjustment
		translation_filter_deltaX = (float)(-IMAGE_WIDTH/2.0 + input->wxd);
		translation_filter_deltaY = (float)(-IMAGE_HEIGHT/2.0 + input->wyd);
		
		move_input_window (input->name, input->wxd, input->wyd);
		conditional_filter_update(get_filter_by_name("in_pattern_translated_scaled_filtered_overlayed_filter"));
		all_outputs_update();
				
		count++;
		total_number_of_saccades++;
		
		//Updates the moving average according to the saccade delta
		delta = sqrt((float)((int)(x)*(int)(x) + (int)(y)*(int)(y)));
		saccade_moving_average = MOVING_AVERAGE_WEIGHT*delta + (1.0 - MOVING_AVERAGE_WEIGHT)*saccade_moving_average;
		
		//Updates the last layer output for next saccadic movement
		x = nl_target_coordinates.neuron_vector[0].output.fval;
		y = nl_target_coordinates.neuron_vector[1].output.fval;
		
	} 
	while (((fabs(x) > 0.5) || (fabs(y) > 0.5)) && (count < max_number_of_saccades));

	last_count_number = count;

	// Updates the saccade vector of the output message
	MAE_visual_search_output_message.x_saccade_vector = x_saccade_vector;
	MAE_visual_search_output_message.y_saccade_vector = y_saccade_vector;

	return;
}



/*
********************************************************
* Function: input_controler			       *
* Description: input events handler		       *
* Inputs: input layer, input status		       *
* Output: none  				       *
********************************************************
*/

void input_controler (INPUT_DESC *input, int status)
{
	char strCommand[128];
	
	status = status;	/* For Keeping The compiler Happy */

	if ((move_active == 1) &&
	    (input->mouse_button == GLUT_LEFT_BUTTON) &&
	    (input->mouse_state == GLUT_DOWN))
	{
		// Translate the input image & Move the input center cursor
		translation_filter_deltaX = (float)(-IMAGE_WIDTH/2.0 + input->wxd);
		translation_filter_deltaY = (float)(-IMAGE_HEIGHT/2.0 + input->wyd);
		sprintf (strCommand, "move %s to %d, %d;", input->name, input->wxd, input->wyd);		
		interpreter (strCommand);
	}
	input->mouse_button = -1;
	
	return;
}

inline double	hamming_squared_cosine(int hamming_dist, double ham_limit)
{
	return ( 1.0 - (double)hamming_dist/ ham_limit);
}

inline double	hamming_squared_cosine_int(int hamming_dist, int ham_limit)
{
	return ( 1.0 - (double)hamming_dist/ ham_limit);
}

inline double	hamming_squared_cosine_threshold(int hamming_dist, int ham_limit)
{
	if(!ham_limit)
		return ( 1.0 );
	else
		return ( 1.0 - (double)hamming_dist/ (double)ham_limit);
}

inline double	hamming_squared_sine(int hamming_dist, double ham_limit)
{
	return ( (double)hamming_dist/ ham_limit);
}

inline double	elliptical_interpolation_radius(double A_axis,double B_axis, double squared_cosine)
{
	return ( A_axis*B_axis / sqrt(A_axis*A_axis*squared_cosine + B_axis*B_axis*(1.0 - squared_cosine)) );
}

inline double	angular_similarity(double cosine)
{
	return ( 1.0 - acos(cosine)/M_PI_2 );
}

inline double	harmonic_mean (int a, int b)
{
	return ( (double) (2*a*b) / (double) (a+b) );
}

inline double	normalized_output_error(float a, float b,float max, float min)
{
	return ( (double) fabs(a-b)/ (double) (max - min) );
}

inline	int	hamming_threshold(float threshold,int	max_ham)
{
	return ( (int) ((1.0 - threshold) * max_ham) );
}

inline	double	unitary_gaussian_func(double x,double u)
{
	return ( exp( -(x-u)*(x-u)*M_PI*2.0 ) )	;
}

int	hamming_based_neuron_layer_retrain(NEURON_LAYER *neuron_layer, float threshold)
{
	int i, w, h;
	NEURON	*neuron_vector;
	int max_val, avg_val;
	double	squared_cosine;
	
	w = neuron_layer->dimentions.x;
	h = neuron_layer->dimentions.y;
	neuron_vector = neuron_layer->neuron_vector;
	
	// Calculates Average and Maximum Hamming Distances
	max_val = 0;
	avg_val = 0;
	for (i = 0; i < w*h; i++)
	{
		avg_val += neuron_vector[i].last_hamming_distance;
		if(neuron_vector[i].last_hamming_distance > max_val)
			max_val = neuron_vector[i].last_hamming_distance;
	}
	avg_val /= w*h;
	
	// If the max value is under the threshold value, must return
	//max_val = (int) harmonic_mean (max_val,INPUTS_PER_NEURON);	//Harmonic mean Max Val - Number of Synapses
	printf("Max val:%d\nAvg Val: %d\nThreshold: %d\n",max_val,avg_val,hamming_threshold(threshold,INPUTS_PER_NEURON));
	if( max_val < hamming_threshold(threshold,INPUTS_PER_NEURON) )
		return 0;
		
	
	// Now for each neuron above the average value, we perform a retraining
	for (i = 0; i < w*h; i++)
	{
		if(neuron_vector[i].last_hamming_distance > avg_val)
		{
			// Calculates the squared 
			squared_cosine = hamming_squared_cosine(neuron_vector[i].last_hamming_distance, harmonic_mean (max_val,INPUTS_PER_NEURON) );
			// Sets the output value (x_axis -> output y_axis -> training )
			neuron_vector[i].output.fval = (float) elliptical_interpolation_radius(
				(double) neuron_vector[i].output.fval,
				(double) gaussian_filtered_training_pattern[i], 
				squared_cosine);
			// Perform the retraining of the individual neuron
			//train_neuron(neuron_layer,i);
		}
	}
	train_neuron_layer("nl_v1_activation_map");
	
	return 1;
}

double 	neuron_layer_hamming_output_composed_certainty(NEURON_LAYER *neuron_layer)
{
	int i, w, h;
	NEURON	*neuron_vector;
	int max_val;
	double	hamming_limit;
	double	network_certainty;
	
	w = neuron_layer->dimentions.x;
	h = neuron_layer->dimentions.y;
	neuron_vector = neuron_layer->neuron_vector;

	//obtains the maximum hamming distance value
	max_val = 0;	//zero maximum value
	for (i = 0; i < w*h; i++)
		if(neuron_vector[i].last_hamming_distance > max_val)
			max_val = neuron_vector[i].last_hamming_distance;

	hamming_limit = harmonic_mean (max_val,INPUTS_PER_NEURON); //calculates the harmonic mean limit value
	
	//Now calculates the network certainty
	network_certainty = 0.0;
	//network_certainty = 1.0;
	for (i = 0; i < w*h; i++)
		network_certainty +=	unitary_gaussian_func(neuron_vector[i].output.fval, gaussian_filtered_training_pattern[i])*
				 	/*(1.0 - normalized_output_error(neuron_vector[i].output.fval, gaussian_filtered_training_pattern[i],
					training_pattern_max_val,training_pattern_min_val))**/
					angular_similarity(sqrt(hamming_squared_cosine(neuron_vector[i].last_hamming_distance,hamming_limit)));
					//angular_similarity(sqrt(hamming_squared_cosine_threshold(neuron_vector[i].last_hamming_distance,max_val)));
	
	network_certainty /= (double) (w*h);
	//network_certainty = pow(network_certainty,1/(double) (w*h)); 
	
	return(network_certainty);
}

double	neuron_layer_simetrized_kullback_leibler_divergence (NEURON_LAYER *neuron_layer)
{
	int	neurons,i;
	double 	divergence;
	
	divergence = 0.0;
	
	neurons = (neuron_layer->dimentions.x) * (neuron_layer->dimentions.y);
	for(i = 0;i < neurons;i++)
		divergence += 	(neuron_layer->neuron_vector[i].output.fval - gaussian_filtered_training_pattern[i]) * 
				log(neuron_layer->neuron_vector[i].output.fval / gaussian_filtered_training_pattern[i]);
		
	return(divergence);	
}

double	neuron_layer_jeffrey_divergence (NEURON_LAYER *neuron_layer)
{
	int	neurons,i;
	double 	divergence,mean_div;

	divergence = 0.0;
	
	neurons = (neuron_layer->dimentions.x) * (neuron_layer->dimentions.y);
	for(i = 0;i < neurons;i++)
	{
		mean_div = (neuron_layer->neuron_vector[i].output.fval + gaussian_filtered_training_pattern[i])/2.0;
		divergence += neuron_layer->neuron_vector[i].output.fval * log (neuron_layer->neuron_vector[i].output.fval/mean_div);
		divergence += gaussian_filtered_training_pattern[i] * log (gaussian_filtered_training_pattern[i]/mean_div);
	}
	
	return(divergence);
}

double	neuron_layer_battacharayya_coefficient(NEURON_LAYER *neuron_layer)
{
	int	w, h, i;
	float	coefficient;
	float	x_norm;
	float	y_norm;
	
	if(!gaussian_filtered_training_pattern)
		return(0.0);
	
	w = neuron_layer->dimentions.x;
	h = neuron_layer->dimentions.y;
	
	coefficient = 0.0;
	x_norm = 0.0;
	y_norm = 0.0;
	
	for(i = 0 ; i < w*h ; i++)
	{
		coefficient += sqrt(gaussian_filtered_training_pattern[i]) * sqrt(neuron_layer->neuron_vector[i].output.fval);
		x_norm += gaussian_filtered_training_pattern[i];
		y_norm += neuron_layer->neuron_vector[i].output.fval;
	}
	
	x_norm = sqrt(x_norm);
	y_norm = sqrt(y_norm);
	coefficient /= x_norm*y_norm;
	
	return(coefficient);
}

/* Sets the internal neuron layer band with the high activation output value */
void set_neuron_layer_band (NEURON_LAYER *neuron_layer, int x1, int x2, int y1,int y2, float value)
{
	int i, x, y, w, h;
	
	w = neuron_layer->dimentions.x;
	h = neuron_layer->dimentions.y;
	
	for (i = 0; i < w*h; i++)
		neuron_layer->neuron_vector[i].output.fval = 0.0f;

	for (y = y1; y < y2; y++)
		for (x = x1; x < x2; x++)
			if ((x >= 0) && (x < w) && (y >= 0) && (y < h))
				neuron_layer->neuron_vector[x + w * y].output.fval = value;
}

/* Sets the internal neuron layer band with integer tags representing output values */
void set_neuron_layer_band_integer_tags (NEURON_LAYER *neuron_layer)
{
	int activation_value;
	int u, v, w, h;
	
	w = neuron_layer->dimentions.x;
	h = neuron_layer->dimentions.y;
	
	for(u = 0; u < w; u++)
	{
		if (u < w/2)
			activation_value = w - ((w-1)/2 - u) ; 
		
		else
			activation_value = w - (u - w/2) ;
		
		
		for(v = 0;v < h; v++)
			neuron_layer->neuron_vector[u + w * v].output.ival = activation_value;
	}
}

/* A simpler version of the function above (for using with hashing type filters) */
void set_neuron_layer_band_integer_tags_simple (NEURON_LAYER *neuron_layer)
{
	int u, v, w, h;
	
	w = neuron_layer->dimentions.x;
	h = neuron_layer->dimentions.y;
	
	for(u = 0; u < w; u++)
		for(v = 0;v < h; v++)
			neuron_layer->neuron_vector[u + w * v].output.ival = u;
}

/* Sets the internal neuron layer band with the high activation output value in a gaussian pattern */
void set_neuron_layer_band_gaussian (NEURON_LAYER *neuron_layer, float max_val, float std_dev, int width)
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

/* Sets the internal neuron layer band with the high activation output value in a triangular pattern */
void set_neuron_layer_band_triangular (NEURON_LAYER *neuron_layer, float max_val, int width)
{
	float	activation_value;
	int u, v, w;
	
	w = neuron_layer->dimentions.x;
	
	for(u = 0; u < w; u++)
	{
		if (u < w/2)
		{
			if((w-1)/2 - u > width/2)
				activation_value = 0.0;
			else
				activation_value = max_val - ((w-1)/2 - u)*max_val/width;
		}
		else
		{
			if(u - w/2 > width/2)
				activation_value = 0.0;
			else
				activation_value = max_val - (u - w/2)*max_val/width;
		}
		
		
		for(v = 0;v < w; v++)
			neuron_layer->neuron_vector[u + w * v].output.fval = activation_value;
	}
}

/* Sets the neuron layer with the masked output according to the training pattern */
void set_masked_neuron_layer (NEURON_LAYER *neuron_layer)
{
	int i, w, h;
	
	w = neuron_layer->dimentions.x;
	h = neuron_layer->dimentions.y;
	
	// for each neuron, mask it value with the previous training patten
	for (i = 0; i < w*h; i++)
		neuron_layer->neuron_vector[i].output.fval *= gaussian_filtered_training_pattern[i];

}

/* Performs the conditional */
void conditional_masked_neuron_layer_training (NEURON_LAYER *neuron_layer)
{
	int i, w, h;
	
	w = neuron_layer->dimentions.x;
	h = neuron_layer->dimentions.y;
	
	// for each neuron, if the activation value is different from trained value, 
	for (i = 0; i < w*h; i++)
		if(neuron_layer->neuron_vector[i].output.fval != gaussian_filtered_training_pattern[i])
		{
			neuron_layer->neuron_vector[i].output.fval = gaussian_filtered_training_pattern[i];
			train_neuron(neuron_layer,i);	//train individual neurons
		}
}

/* Calculates the certainty of the output neuron layer according to the number of different pixels */
double	output_different_outputs_certainty (NEURON_LAYER *neuron_layer)
{
	int i, w, h, diff_neurons;
	double certainty;
	
	w = neuron_layer->dimentions.x;
	h = neuron_layer->dimentions.y;
	diff_neurons = 0;

	// for each neuron, if the activation value is different from trained value ...
	for (i = 0; i < w*h; i++)
		if(neuron_layer->neuron_vector[i].output.fval != gaussian_filtered_training_pattern[i])
			diff_neurons++;
	
	certainty = 1.0 - ((double) (diff_neurons) / (double) (w*h));
	return(certainty);
}

/* Calculates the total output weight of the neuron layer */
double	output_total_weight (NEURON_LAYER *neuron_layer)
{
	double total_weight;
	int x, y, w, h;
	
	w = neuron_layer->dimentions.x;
	h = neuron_layer->dimentions.y;
	
	total_weight = 0.0;
	
	for (y = 0; y < h; y++)
		for (x = 0; x < w; x++)
			total_weight += neuron_layer->neuron_vector[x + w * y].output.fval;
			
	return(total_weight);
}

/* Calculates the neuron layer activation centroid */
void	output_centroid_position (NEURON_LAYER *neuron_layer,int *u,int *v)
{
	double total_weight;
	double x_centroid,y_centroid;

	int x, y, w, h;
		
	w = neuron_layer->dimentions.x;
	h = neuron_layer->dimentions.y;
	
	total_weight = output_total_weight(neuron_layer);
	
	x_centroid = 0.0;
	y_centroid = 0.0;
	
	for (y = 0; y < h; y++)
	{
		for (x = 0; x < w; x++)
		{
			x_centroid += x*neuron_layer->neuron_vector[x + w * y].output.fval;
			y_centroid += y*neuron_layer->neuron_vector[x + w * y].output.fval;
		}
	}
	
	x_centroid /= total_weight;
	y_centroid /= total_weight;
	*u = (int) x_centroid;
	*v = (int) y_centroid;
}

/* Calculates the standard deviation measuraments in the desired neuron layer */
void	output_std_dev_measure (NEURON_LAYER *neuron_layer,double *u_std_dev,double *v_std_dev)
{
	double total_weight;
	double x_std_dev,y_std_dev;
	
	int x_centroid,y_centroid;

	int x, y, w, h;
		
	w = neuron_layer->dimentions.x;
	h = neuron_layer->dimentions.y;
	
	total_weight = output_total_weight(neuron_layer);
	output_centroid_position (neuron_layer,&x_centroid,&y_centroid);
	
	x_std_dev = 0.0;
	y_std_dev = 0.0;
	
	for (y = 0; y < h; y++)
	{
		for (x = 0; x < w; x++)
		{
			x_std_dev += (x-x_centroid)*(x-x_centroid)*neuron_layer->neuron_vector[x + w * y].output.fval;
			y_std_dev += (y-y_centroid)*(y-y_centroid)*neuron_layer->neuron_vector[x + w * y].output.fval;
		}
	}
	
	x_std_dev /= total_weight;
	y_std_dev /= total_weight;
	x_std_dev = sqrt(x_std_dev);
	y_std_dev = sqrt(y_std_dev);
	*u_std_dev = x_std_dev;
	*v_std_dev = y_std_dev;
}

double	visual_search_certainty_simple (NEURON_LAYER *neuron_layer, int x1, int x2, int y1,int y2, float value)
{
	int x, y, w, h;
	int nl_band_neurons, activated_nl_band_neurons;
	
	w = neuron_layer->dimentions.x;
	h = neuron_layer->dimentions.y;
	
	nl_band_neurons = 0;
	activated_nl_band_neurons = 0;


	for (y = y1; y < y2; y++)
		for (x = x1; x < x2; x++)
			if ((x >= 0) && (x < w) && (y >= 0) && (y < h))
			{
				nl_band_neurons++;
				if(neuron_layer->neuron_vector[x + w * y].output.fval == value)
					activated_nl_band_neurons++;
			}

	return((double) activated_nl_band_neurons / (double) nl_band_neurons );
}

double	obtain_measured_scale_factor(NEURON_LAYER *neuron_layer)
{
	int x, y, w, h;
	float accumulated_active_weight;
	double ratio;
	
	w = neuron_layer->dimentions.x;
	h = neuron_layer->dimentions.y;
	
	accumulated_active_weight = 0.0;

	for (y = 0; y < h; y++)
		for (x = 0; x < w; x++)
			accumulated_active_weight += neuron_layer->neuron_vector[x + w * y].output.fval;
	
	// Calculates the ratio between the current active weigth and the total active weight in the trained pattern
	ratio = (double) accumulated_active_weight / (double) gaussian_filtered_training_pattern_total_weight;

	return(ratio);
}

//In this particular case, the filtered output must be considered in order to compute the certainty
double	visual_search_certainty_percentage_of_active_neurons_versus_trained_bar_float(NEURON_LAYER *neuron_layer)
{
	int x, y, w, h;
	float accumulated_active_weight;
	double ratio;
	
	w = neuron_layer->dimentions.x;
	h = neuron_layer->dimentions.y;
	
	accumulated_active_weight = 0.0;

	for (y = 0; y < h; y++)
		for (x = 0; x < w; x++)
			accumulated_active_weight += neuron_layer->neuron_vector[x + w * y].output.fval;
	
	// Calculates the ratio between the current active weigth and the total active weight in the trained pattern
	ratio = (double) accumulated_active_weight / (double) gaussian_filtered_training_pattern_total_weight;
	
	if(ratio > 1.0)
		return(1.0);
	else
		return(ratio);
}

double	visual_search_certainty_percentage_of_active_neurons_versus_trained_bar(NEURON_LAYER *neuron_layer, int x1, int x2, int y1,int y2, float value)
{
	int x, y, w, h;
	int nl_band_neurons, activated_neurons;
	double ratio;
	
	w = neuron_layer->dimentions.x;
	h = neuron_layer->dimentions.y;
	
	nl_band_neurons = 0;
	activated_neurons = 0;

	for (y = 0; y < h; y++)
	{
		for (x = 0; x < w; x++)
		{
			//Count this neuron if its active
			if(neuron_layer->neuron_vector[x + w * y].output.fval == value)
				activated_neurons++;
			
			//Count this neuron if it is inside the activation band
			if ((x >= x1) && (x < x2) && (y >= y1) && (y < y2))
				nl_band_neurons++;
		}
	}
	
	ratio = (double) activated_neurons / (double) nl_band_neurons;
	
	if(ratio > 1.0)
		return(1.0);
	else
		return(ratio);
}

//Counts only the nerons in the central activation patten bar
double	visual_search_certainty_width (NEURON_LAYER *neuron_layer, int x1, int x2, int y1,int y2, float value)
{
	int x, y, w, h;
	int complete_height;
	int nl_band_neurons, activated_nl_band_neurons;
	
	w = neuron_layer->dimentions.x;
	h = neuron_layer->dimentions.y;
	
	//Neuron layer height limits
	if(y2 > h)
		y2 = h;

	nl_band_neurons = (x2-x1)*(y2-y1);	//inside band neurons, limits guarded
	activated_nl_band_neurons = 0;
	
	//First neuron layer half search
	complete_height = 1;
	for (x = w/2; x >= x1; x--)
	{
		if(!complete_height)
			break;
			
		for (y = y1; y < y2; y++)
		{
			if(neuron_layer->neuron_vector[x + w * y].output.fval == value)
				activated_nl_band_neurons++;
			else
				complete_height = 0;
		}
	}
	
	//Second neuron layer half search
	complete_height = 1;
	for (x = w/2 + 1; x < x2; x++)
	{
		if(!complete_height)
			break;
			
		for (y = y1; y < y2; y++)
		{
			if(neuron_layer->neuron_vector[x + w * y].output.fval == value)
				activated_nl_band_neurons++;
			else
				complete_height = 0;
		}
	}

	return((double) activated_nl_band_neurons / (double) nl_band_neurons );
}

//Counts only the nerons in the central activation patten bar
double	visual_search_certainty_width_weighted (NEURON_LAYER *neuron_layer, int x1, int x2, int y1,int y2, float value)
{
	int x, y, w;//, h;
/*
	int complete_height;
*/
	//int nl_band_neurons, activated_nl_band_neurons;
	double accumulator, total_weight;
	
	w = neuron_layer->dimentions.x;
//	h = neuron_layer->dimentions.y;
	
	accumulator = 0.0;
	total_weight = 0.0;
	
	//First neuron layer half search
	//complete_height = 1;
	for (x = w/2; x >= x1; x--)
	{
		for (y = y1; y < y2; y++)
			total_weight += exp(-abs(x-w/2));
		
		//if(!complete_height)
		//	break;
			
		for (y = y1; y < y2; y++)
		{
			if(neuron_layer->neuron_vector[x + w * y].output.fval == value)
				accumulator += exp(-abs(x-w/2));
			//else
			//	complete_height = 0;
		}
	}
	
	//Second neuron layer half search
	//complete_height = 1;
	for (x = w/2 + 1; x < x2; x++)
	{
		for (y = y1; y < y2; y++)
			total_weight += exp(-abs(x - (w/2 + 1)));

		//if(!complete_height)
		//	break;
			
		for (y = y1; y < y2; y++)
		{
			if(neuron_layer->neuron_vector[x + w * y].output.fval == value)
				accumulator += exp(-abs(x - (w/2 + 1)));
			//else
			//	complete_height = 0;
		}
	}

	return((double) accumulator / (double) total_weight );
}


double	visual_search_certainty_weighted (NEURON_LAYER *neuron_layer, int x1, int x2, int y1,int y2, float value)
{
	int x, y, w, h;
	double accumulator, total_weight;

	w = neuron_layer->dimentions.x;
	h = neuron_layer->dimentions.y;

	accumulator = 0.0;
	total_weight = 0.0;
	
	for (y = y1; y < y2; y++)
		for (x = x1; x < x2; x++)
			if ((x >= 0) && (x < w) && (y >= 0) && (y < h))
			{
				total_weight += exp(-abs(x-w/2));
				if(neuron_layer->neuron_vector[x + w * y].output.fval == value)
					accumulator += exp(-abs(x-w/2));
			}

	return((double) accumulator / (double) total_weight );
}

// Gets the gaussian filtered image from the neuron layer (the training pattenr)
void	new_gaussian_filtered_training_pattern(NEURON_LAYER *neuron_layer)
{
	int w, h, i;
	
	w = neuron_layer->dimentions.x;
	h = neuron_layer->dimentions.y;
	
	if(gaussian_filtered_training_pattern)
	{
		// New pattern alloc'ed
		free(gaussian_filtered_training_pattern);
		gaussian_filtered_training_pattern = (float *)malloc(w * h * sizeof(float));
	}
	else
		gaussian_filtered_training_pattern = (float *)malloc(w * h * sizeof(float));
	
	gaussian_filtered_training_pattern_total_weight = 0.0;	//acumulated training image weight
		
	for (i = 0; i < w*h; i++)
	{
		gaussian_filtered_training_pattern[i] = neuron_layer->neuron_vector[i].output.fval;
		gaussian_filtered_training_pattern_total_weight += neuron_layer->neuron_vector[i].output.fval;
	}
	
	//Now we have to obtain the maximum and minimum output values
	training_pattern_min_val = FLT_MAX;
	training_pattern_max_val = FLT_MIN;

	for (i = 0; i < w*h; i++)
	{
		if(gaussian_filtered_training_pattern[i] > training_pattern_max_val)
			training_pattern_max_val = gaussian_filtered_training_pattern[i];
			
		if(gaussian_filtered_training_pattern[i] < training_pattern_min_val)
			training_pattern_min_val = gaussian_filtered_training_pattern[i];
	}
}

// Calculates the correlation between the gaussian filtered and the output activation layer
double	calculate_correlation_between_gaussian_filtered_outputs(NEURON_LAYER *neuron_layer)
{
	int	w, h, i;
	float	dot_product;
	float	x_norm,y_norm;
	float	normalized_dot_product;	
	
	if(!gaussian_filtered_training_pattern)
		return(0.0);
	
	w = neuron_layer->dimentions.x;
	h = neuron_layer->dimentions.y;
	
	dot_product = 0.0;
	x_norm = 0.0;
	y_norm = 0.0;
	
	for(i = 0 ; i < w*h ; i++)
	{
		dot_product += gaussian_filtered_training_pattern[i] * neuron_layer->neuron_vector[i].output.fval;
		x_norm += gaussian_filtered_training_pattern[i] * gaussian_filtered_training_pattern[i];
		y_norm += neuron_layer->neuron_vector[i].output.fval * neuron_layer->neuron_vector[i].output.fval;
	}
	
	x_norm = sqrt(x_norm);
	y_norm = sqrt(y_norm);
	
	normalized_dot_product = dot_product/(x_norm*y_norm);
	
	return(normalized_dot_product);
}

// Calculates the Angular Similarity between the gayssian filtered outputs
double	calculate_angular_similarity_between_gaussian_filtered_outputs(NEURON_LAYER *neuron_layer)
{
	// Positive interval values
	return( (1.0 - 2.0*acos(calculate_correlation_between_gaussian_filtered_outputs(neuron_layer))/M_PI) );
}

/*
********************************************************
* Function: f_keyboard  			       *
* Description: keyboard events handler  	       *
* Inputs: key_value (pointer to pressed character)     *
* Output: none  				       *
********************************************************
*/

// Keyboard actions are still acceptable
void f_keyboard (char *key_value)
{
	char key;
	
	switch (key = key_value[0])
	{
		// Train network
		case 'T':	
		case 't':
			g_nNetworkStatus = TRAINNING;
			//set_neuron_layer_band (&nl_v1_activation_map, 27, 36, 0, 48, 1.0f);
			set_neuron_layer_band (&nl_v1_activation_map, (NL_WIDTH - ACT_BAND_WIDTH)/2 , (NL_WIDTH + ACT_BAND_WIDTH)/2, 0, NL_HEIGHT, HIGHEST_OUTPUT);
			filter_update(get_filter_by_name("nl_v1_activation_map_f_filter"));
			conditional_filter_update(get_filter_by_name("nl_v1_activation_map_f_thresholded_filter"));
			train_neuron_layer("nl_v1_activation_map");
			new_gaussian_filtered_training_pattern(&nl_v1_activation_map_f);

			all_outputs_update ();
			g_nNetworkStatus = RUNNING;
			break;
		// Saccade until reach the target
		case 'S':
		case 's':
			saccade (&in_pattern);
			break;	
		// Force the filters in_pattern_filtered_filter & in_pattern_filtered_translated_filter to be updated
		case 'U':
		case 'u':
			update_input_filters(NULL);
			break;
	}
	
	return;
}

		
/*
********************************************************
* Function: set_input_layer_translation 	       *
* Description:  				       *
* Inputs: input  				       *
* Output:					       *
********************************************************
*/

void	set_input_layer_translation(INPUT_DESC *input,int x, int y)
{
	char strCommand[128];
	
	// The coordinate system must be adjusted to the MAE internal coordinates
	input->wxd = x;
	input->wyd = y;
	translation_filter_deltaX = (float)(-IMAGE_WIDTH/2.0  + input->wxd);
	translation_filter_deltaY = (float)(-IMAGE_HEIGHT/2.0 + input->wyd);
	sprintf (strCommand, "move %s to %d, %d;", input->name, input->wxd, input->wyd);		
	interpreter (strCommand);
}

void	input_filters_and_outputs_update(void)
{
	conditional_filter_update(get_filter_by_name("in_pattern_translated_filter"));
	
	if(get_output_by_name("in_pattern_filtered_out"))
		output_update(get_output_by_name("in_pattern_filtered_out"));
		
	conditional_filter_update(get_filter_by_name("in_pattern_translated_scaled_filter"));	
	
	if(get_output_by_name("in_pattern_translated_scaled_out"))
		output_update(get_output_by_name("in_pattern_translated_scaled_out"));
	
	conditional_filter_update(get_filter_by_name("in_pattern_translated_scaled_filtered_filter"));	
	
	if(get_output_by_name("in_pattern_translated_scaled_filtered_out"))
		output_update(get_output_by_name("in_pattern_translated_scaled_filtered_out"));

	conditional_filter_update(get_filter_by_name("in_pattern_translated_scaled_filtered_inhibited_filter"));	
	
	if(get_output_by_name("in_pattern_translated_scaled_filtered_inhibited_out"))
		output_update(get_output_by_name("in_pattern_translated_scaled_filtered_inhibited_out"));

	conditional_filter_update(get_filter_by_name("in_pattern_filtered_translated_log_polar_filter"));
	conditional_filter_update(get_filter_by_name("in_pattern_translated_scaled_filtered_overlayed_filter"));	
}

/*
********************************************************
* Function: update_input_layer_neurons_and_image       *
* Description:  				       *
* Inputs: input  				       *
* Output:					       *
********************************************************
*/

void	update_input_layer_neurons_and_image(INPUT_DESC *input)
{
	// Update the input layer neurons and Image
	check_input_bounds (input, input->wx + input->ww/2, input->wy + input->wh/2);
	input->up2date = 0;
	update_input_neurons (input);
	update_input_image (input);
	
	input_filters_and_outputs_update();
}
			
/*
********************************************************
* Function: train_visual_search_app	               *
* Description:  				       *
* Inputs: none  				       *
* Output:					       *
********************************************************
*/

void	train_visual_search_app(void)
{	
	// Trains the visual search module
	//set_neuron_layer_band_integer_tags(&nl_v1_activation_map);
	set_neuron_layer_band_gaussian (&nl_v1_activation_map,1.0,NL_WIDTH/4.0,NL_WIDTH);	
	//set_neuron_layer_band_triangular (&nl_v1_activation_map,1.0,NL_WIDTH);
	//set_neuron_layer_band (&nl_v1_activation_map, (NL_WIDTH - ACT_BAND_WIDTH)/2 , (NL_WIDTH + ACT_BAND_WIDTH)/2, 0, NL_HEIGHT, HIGHEST_OUTPUT);
	filter_update(get_filter_by_name("nl_v1_activation_map_f_filter"));
	conditional_filter_update(get_filter_by_name("nl_v1_activation_map_f_thresholded_filter"));
	train_neuron_layer("nl_v1_activation_map");
	new_gaussian_filtered_training_pattern(&nl_v1_activation_map);
	all_outputs_update ();
	
	//check_neuron_layer_hash_collisions ("nl_v1_activation_map");
}

/*
********************************************************
* Function: masked_train_visual_search_app	       *
* Description:  				       *
* Inputs: none  				       *
* Output:					       *
********************************************************
*/

void	masked_train_visual_search_app(void)
{	
	// Trains the visual search module
	set_masked_neuron_layer (&nl_v1_activation_map);	// writes the masked pattern into the neuron layer and trains it with this pattern
	train_neuron_layer("nl_v1_activation_map");
	all_outputs_update ();
}

/*
********************************************************
* Function: conditional_masked_train_visual_search_app *
* Description:  				       *
* Inputs: none  				       *
* Output:					       *
********************************************************
*/

void	conditional_masked_train_visual_search_app(void)
{
	conditional_masked_neuron_layer_training(&nl_v1_activation_map);
	all_outputs_update ();		// update all outputs
}

/*
********************************************************
* Function: sparse_train_visual_search_app	       *
* Description:  				       *
* Inputs: none  				       *
* Output:					       *
********************************************************
*/

void	sparse_train_visual_search_app(float percentage)
{	
	// Trains the visual search module
	set_neuron_layer_band (&nl_v1_activation_map, (NL_WIDTH - ACT_BAND_WIDTH)/2 , (NL_WIDTH + ACT_BAND_WIDTH)/2, 0, NL_HEIGHT, HIGHEST_OUTPUT);
	filter_update(get_filter_by_name("nl_v1_activation_map_f_filter"));
	conditional_filter_update(get_filter_by_name("nl_v1_activation_map_f_thresholded_filter"));
	sparse_train_neuron_layer("nl_v1_activation_map",percentage);
	new_gaussian_filtered_training_pattern(&nl_v1_activation_map_f);
	all_outputs_update ();
}

/*
*****************************************************************
* Function: finalize_building_output_message_and_publish_it	*
* Description:  				         	*
* Inputs: input  				          	*
* Output:					           	*
*****************************************************************
*/

void publish_output_message(void)
{
	IPC_RETURN_TYPE err = IPC_OK;
	
	err = IPC_publishData(CARMEN_VISUAL_SEARCH_OUTPUT_MESSAGE_NAME, &MAE_visual_search_output_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_VISUAL_SEARCH_OUTPUT_MESSAGE_NAME);
}

/*
************************************************************
* Function: simple_raw_image_copy  			   *
* Description:  				           *
* Inputs: input  				           *
* Output:					           *
************************************************************
*/

inline void simple_raw_image_copy(unsigned char* src,unsigned char *dst,int width, int height)
{	
	memcpy((void *)dst,(void *)src,3*width*height);	//3 channel image
}

/*
********************************************************
* Function: copy_raw_image_into_input	               *
* Description:  				       *
* Inputs: input, raw_image  			       *
* Output:					       *
********************************************************
*/

void	copy_raw_image_into_input(INPUT_DESC *input,unsigned char *raw_image)
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
					//intensity = (int) (0.2989 * ((float)red) + 0.5870 * ((float)green) + 0.1140 * ((float)blue) ); 
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
			for (i = 0; i < input->wh ; i++)  
			{
				for (j = 0; j < input->ww; j++) 
				{	// 3 channel image
					input->image[3 * (i * input->ww + j) + 0] = (GLubyte) raw_image[im_size - 3 * (i * input->ww + (input->ww- j) ) + 0];
					input->image[3 * (i * input->ww + j) + 1] = (GLubyte) raw_image[im_size - 3 * (i * input->ww + (input->ww- j) ) + 1];
					input->image[3 * (i * input->ww + j) + 2] = (GLubyte) raw_image[im_size - 3 * (i * input->ww + (input->ww- j) ) + 2];
				}
			}
			break;
	}
}

/*
********************************************************
* Function: forget_last_training              *
* Description:  				       *
* Inputs: none  				       *
* Output:					       *
********************************************************
*/

void forget_last_training()
{
	visual_search_state = WAITING_FOR_TRAINNING;
	clear_neural_layers_memory ("nl_v1_activation_map");
}
/*
********************************************************
* Function: test_message_handler	      	       *
* Description:  				       *
* Inputs: none  				       *
* Output:					       *
********************************************************
*/

void	test_message_handler(carmen_visual_search_test_message *message)
{
	if(visual_search_state == RUNNING_NETWORK)
	{
		MAE_visual_search_test_message = *message;

#ifndef CML_SCRIPT
		dynamic_scale_factor = message->scale;
#endif
		// Visual Search Output Message
		MAE_visual_search_output_message.x_point = -1;
		MAE_visual_search_output_message.y_point = -1;
		MAE_visual_search_output_message.x_saccade_vector = 0;
		MAE_visual_search_output_message.y_saccade_vector = 0;
		MAE_visual_search_output_message.measured_scale_factor = 1.0;
		MAE_visual_search_output_message.timestamp = message->timestamp;	//same timestamp as input message
		MAE_visual_search_output_message.host = carmen_get_host();		//new hostname for utput

#ifdef CML_SCRIPT
		new_VS_test_message = TRUE;
#else
		PARAM_LIST p_list;
		MAE_copy_current_test_message_into_input(&p_list);
		MAE_perform_network_test(&p_list);
		MAE_publish_OK(&p_list);
#endif

	}
}

/*
********************************************************
* Function: bumblebee_image_handler	       *
* Description:  				       *
* Inputs: none  				       *
* Output:					       *
********************************************************
*/

void	bumblebee_image_handler(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	if(visual_search_state == RUNNING_NETWORK)
	{
		carmen_visual_search_test_message message;

		message.reference_image_size = stereo_image->image_size;
		
		//Switches bewteen the selected camera in the NADL .con file
		if(LENS == LEFT_CAMERA)
			message.reference_image = stereo_image->raw_left;
		else if(LENS == RIGHT_CAMERA)
			message.reference_image = stereo_image->raw_right;
		else
		{
			printf("Unknown camera\n");
			return;//(-1); 	//error condition
		}
		
		message.timestamp = stereo_image->timestamp;
		message.host = stereo_image->host;		//new hostname

		test_message_handler(&message);
	}
}

/*
********************************************************
* Function: state_change_message_handler	       *
* Description:  				       *
* Inputs: none  				       *
* Output:					       *
********************************************************
*/

void	state_change_message_handler(carmen_visual_search_state_change_message *visual_search_state_change_message)
{
	MAE_visual_search_state_change_message.timestamp = visual_search_state_change_message->timestamp;  
	MAE_visual_search_state_change_message.state = visual_search_state_change_message->state;
	new_VS_state_change_message = TRUE;
	//Hostname not coppied (plain uncessary here)
	
	#ifndef CML_SCRIPT
	forget_last_training();
	#endif
}

void query_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
	      void *clientData __attribute__ ((unused)) )
{
	carmen_visual_search_test_message request;
	FORMATTER_PTR formatter;
	IPC_RETURN_TYPE err;

	formatter = IPC_msgInstanceFormatter(msgRef);
	err = IPC_unmarshallData(formatter, callData, &request, sizeof(carmen_visual_search_test_message));
	IPC_freeByteArray(callData);

	carmen_test_ipc_return(err, "Could not unmarshall data", IPC_msgInstanceName(msgRef));

	MAE_visual_search_test_message = request;

	dynamic_scale_factor = request.scale;

	// Visual Search Output Message
	MAE_visual_search_output_message.x_point = -1;
	MAE_visual_search_output_message.y_point = -1;
	MAE_visual_search_output_message.x_saccade_vector = 0;
	MAE_visual_search_output_message.y_saccade_vector = 0;
	MAE_visual_search_output_message.measured_scale_factor = 1.0;
	MAE_visual_search_output_message.timestamp = request.timestamp;	//same timestamp as input message
	MAE_visual_search_output_message.host = carmen_get_host();		//new hostname for output

	MAE_copy_current_test_message_into_input(0);
	MAE_perform_network_test(0);

	MAE_visual_search_output_message.x_point = in_pattern.wxd;
	MAE_visual_search_output_message.y_point = IMAGE_HEIGHT - in_pattern.wyd;
	MAE_visual_search_output_message.measured_scale_factor = obtain_measured_scale_factor(&nl_v1_activation_map);

	err = IPC_respondData(msgRef, CARMEN_VISUAL_SEARCH_OUTPUT_MESSAGE_NAME, &MAE_visual_search_output_message);
	carmen_test_ipc(err, "Could not respond", CARMEN_VISUAL_SEARCH_OUTPUT_MESSAGE_NAME);

}

/*
********************************************************
* Function: training_message_handler		       *
* Description:  				       *
* Inputs: none  				       *
* Output:					       *
********************************************************
*/

void training_message_handler(carmen_visual_search_message *visual_search_message)
{
	MAE_visual_search_training_message.x_point = visual_search_message->x_point;
	MAE_visual_search_training_message.y_point = IMAGE_HEIGHT - visual_search_message->y_point;
	MAE_visual_search_training_message.reference_image_size = visual_search_message->reference_image_size;
	MAE_visual_search_training_message.reference_image = visual_search_message->reference_image;
	MAE_visual_search_training_message.timestamp = visual_search_message->timestamp;
#ifdef CML_SCRIPT
	new_VS_training_message = TRUE;
#else
	if (visual_search_message->forget_last_training == 1)
		forget_last_training();
	MAE_perform_network_training(NULL);
#endif
}

	
/*
********************************************************
* Function: calculate_random_position		       *
* Description:  				       *
* Inputs:					       *
* Output:					       *
********************************************************
*/

NEURON_OUTPUT calculate_random_position (PARAM_LIST *param_list)
{
	NEURON_OUTPUT output;
	int *xi, *yi;
	int wi, hi;
	
	// Get the Address of the Image Coordenates
	xi = (int *) param_list->next->param.pval;
	yi = (int *) param_list->next->next->param.pval;
	
	// Get the Image Dimentions
	wi = param_list->next->next->next->param.ival;
	hi = param_list->next->next->next->next->param.ival;
	
	*xi = random () / wi;
	*yi = random () / hi;
	
	// Zero return on default
	output.ival = 0;
	return (output);
}


/*
***********************************************************
* Function: SetNetworkStatus
* Description:
* Inputs:
* Output:
***********************************************************
*/

NEURON_OUTPUT SetNetworkStatus (PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;

	g_nStatus = pParamList->next->param.ival;

	switch (g_nStatus)
	{
		case TRAINING_PHASE:
			strcpy (g_strRandomFacesFileName, RAMDOM_FACES_TEST);
			break;
		case RECALL_PHASE:
			strcpy (g_strRandomFacesFileName, RAMDOM_FACES_RECALL);
			break;
	}
	
	output.ival = 0;
	return (output);
}

// Carmen interface defined user functions

NEURON_OUTPUT update_input_filters(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	// Try to Update All filters and check if it was a sucessfull update
	if(	!(conditional_filter_update(get_filter_by_name("in_pattern_translated_filter")) &
		conditional_filter_update(get_filter_by_name("in_pattern_translated_scaled_filter")) &
		conditional_filter_update(get_filter_by_name("in_pattern_translated_scaled_filtered_filter")) &
		conditional_filter_update(get_filter_by_name("in_pattern_translated_scaled_filtered_inhibited_filter")) &
		conditional_filter_update(get_filter_by_name("hamming_distance_layer_filter")) ))
		output.ival = 0;
	else
		output.ival = -1;
	
	return output;
}

NEURON_OUTPUT visual_search_converged(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	if((fabs(nl_target_coordinates.neuron_vector[0].output.fval) > 0.5) || (fabs(nl_target_coordinates.neuron_vector[1].output.fval) > 0.5))
		output.ival = 0;	//if does not converge - return 0
	else if(last_count_number > 1)	//more than a single saccade
		output.ival = 1;	//if converges - return 1
	else
		output.ival = 0;	//count as non convergent, should not retrain
		
	return(output);
}

NEURON_OUTPUT visual_search_get_last_count_number(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	output.ival = last_count_number;
		
	return(output);
}

NEURON_OUTPUT visual_search_get_saccade_moving_average(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	output.fval = saccade_moving_average;

	return(output);
}

NEURON_OUTPUT visual_search_print_saccade_moving_average(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	output.ival = 1;
	
	printf("Saccade Moving Average value: %f\n",saccade_moving_average);
		
	return(output);
}

NEURON_OUTPUT execute_IPC_listen(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	
	//IPC the IPC listen return is obtained from function
	output.ival = IPC_listen((unsigned int) pParamList->next->param.ival);

	return(output);
}

NEURON_OUTPUT MAE_sleep(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;

	output.ival = sleep((unsigned int) pParamList->next->param.ival);

	return(output);
}

NEURON_OUTPUT get_new_VS_state_change_message_value(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;

	pParamList = pParamList;	/* For Keeping The compiler Happy */
	output.ival = new_VS_state_change_message;
	return(output);
}

NEURON_OUTPUT get_new_VS_training_message_value(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	output.ival = new_VS_training_message;
	return(output);
}

NEURON_OUTPUT get_new_VS_test_message_value(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	output.ival = new_VS_test_message;
	return(output);
}

NEURON_OUTPUT get_visual_search_state(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	output.ival = visual_search_state;
	return(output);
}

NEURON_OUTPUT set_visual_search_state(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	
	if(!pParamList->next)
	{
		printf("set_visual_search_state requires 1 int type parameter\n");
		output.ival = -1;	//Error return
		return(output);
	}
	
	visual_search_state = pParamList->next->param.ival;
	
	output.ival = 0;
	return(output);
}

NEURON_OUTPUT get_visual_search_state_message(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	output.ival =  MAE_visual_search_state_change_message.state;
	return(output);
}

	
NEURON_OUTPUT reset_new_VS_state_change_message_value(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	new_VS_state_change_message = FALSE;
	output.ival = 0;
	return(output);
}

NEURON_OUTPUT reset_new_VS_training_message_value(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;

	pParamList = pParamList;	/* For Keeping The compiler Happy */
	new_VS_training_message = FALSE;
	output.ival = 0;
	return(output);
}

NEURON_OUTPUT reset_new_VS_test_message_value(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;

	pParamList = pParamList;	/* For Keeping The compiler Happy */
	new_VS_test_message = FALSE;
	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_copy_current_test_message_into_input(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
		
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	copy_raw_image_into_input(&in_pattern,MAE_visual_search_test_message.reference_image);
	
	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_perform_state_change(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	forget_last_training();

	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_perform_network_training(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output; 
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	visual_search_state = TRAINNING_NETWORK;
			
	// Copies the message image into the neuron layer and train it with the desired activation pattern
	copy_raw_image_into_input(&in_pattern,MAE_visual_search_training_message.reference_image);
	update_input_layer_neurons_and_image(&in_pattern);		//Must update the input layer
	set_input_layer_translation(&in_pattern,MAE_visual_search_training_message.x_point,MAE_visual_search_training_message.y_point);	//move the window
	train_visual_search_app();
	//sparse_train_visual_search_app(PERCENTAGE_OF_TRAINING_NEURONS);
	visual_search_state = RUNNING_NETWORK;
	
	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_perform_network_training_inverted_height(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output; 
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	visual_search_state = TRAINNING_NETWORK;
			
	// Copies the message image into the neuron layer and train it with the desired activation pattern
	copy_raw_image_into_input(&in_pattern,MAE_visual_search_training_message.reference_image);
	update_input_layer_neurons_and_image(&in_pattern);		//Must update the input layer
	set_input_layer_translation(&in_pattern,MAE_visual_search_training_message.x_point,IMAGE_HEIGHT - MAE_visual_search_training_message.y_point);	//move the window
	train_visual_search_app();
	//sparse_train_visual_search_app(PERCENTAGE_OF_TRAINING_NEURONS);
	visual_search_state = RUNNING_NETWORK;
	
	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_perform_sparse_network_training(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output; 
	
	if(!pParamList->next)
	{
		printf("MAE_perform_sparse_network_retraining requires 1 float type parameter\n");
		output.ival = -1;	//Error return
		return(output);
	}
	
	visual_search_state = TRAINNING_NETWORK;
			
	// Copies the message image into the neuron layer and train it with the desired activation pattern
	copy_raw_image_into_input(&in_pattern,MAE_visual_search_training_message.reference_image);
	update_input_layer_neurons_and_image(&in_pattern);		//Must update the input layer
	set_input_layer_translation(&in_pattern,MAE_visual_search_training_message.x_point,MAE_visual_search_training_message.y_point);	//Move the window
	sparse_train_visual_search_app(pParamList->next->param.fval);	//Performs the sparse retraining
	visual_search_state = RUNNING_NETWORK;
	
	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_perform_network_retraining(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output; 
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	visual_search_state = TRAINNING_NETWORK;
	train_visual_search_app();
	visual_search_state = RUNNING_NETWORK;
	
	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_perform_network_masked_retraining(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output; 
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	visual_search_state = TRAINNING_NETWORK;
	masked_train_visual_search_app();
	visual_search_state = RUNNING_NETWORK;
	
	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_perform_network_conditional_masked_retraining(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output; 
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	visual_search_state = TRAINNING_NETWORK;
	conditional_masked_train_visual_search_app();
	visual_search_state = RUNNING_NETWORK;
	
	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_perform_sparse_network_retraining(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output; 
	
	if(!pParamList->next)
	{
		printf("MAE_perform_sparse_network_retraining requires 1 float type parameter\n");
		output.ival = -1;	//Error return
		return(output);
	}
	
	// Switches the network into the trainning state
	visual_search_state = TRAINNING_NETWORK;
	sparse_train_visual_search_app(pParamList->next->param.fval);	//Performs the sparse retraining
	visual_search_state = RUNNING_NETWORK;
	
	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_perform_network_test(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	// Copies the message image into the neuron layer and performs the saccadic movement
	update_input_layer_neurons_and_image(&in_pattern);		//Must update the input layer
	repetitive_saccade(&in_pattern,MAX_NUMBER_OF_SACCADE);		//repetitive saccades
	
	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_print_network_certainty_simple(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	network_certainty = visual_search_certainty_simple (&nl_v1_activation_map, (NL_WIDTH - ACT_BAND_WIDTH)/2 , (NL_WIDTH + ACT_BAND_WIDTH)/2, 0, NL_HEIGHT, HIGHEST_OUTPUT);
	printf("Simple Certainty : %f\n", network_certainty);

	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_print_network_certainty_percentage_of_active_neurons_versus_trained_bar(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	network_certainty = visual_search_certainty_percentage_of_active_neurons_versus_trained_bar (&nl_v1_activation_map, (NL_WIDTH - ACT_BAND_WIDTH)/2 , (NL_WIDTH + ACT_BAND_WIDTH)/2, 0, NL_HEIGHT, HIGHEST_OUTPUT);
	printf("Active Neuron Certainty : %f\n", network_certainty);

	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_print_network_certainty_percentage_of_active_neurons_versus_trained_bar_float(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	network_certainty =  visual_search_certainty_percentage_of_active_neurons_versus_trained_bar_float(&nl_v1_activation_map);
	printf("Active Neuron Weight Certainty : %f\n", network_certainty);

	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_calculate_network_certainty_percentage_of_active_neurons_versus_trained_bar_float(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	/* Sets the global network certainty value also */
	network_certainty = (float) visual_search_certainty_percentage_of_active_neurons_versus_trained_bar_float(&nl_v1_activation_map);
	output.fval =  network_certainty;
	printf("Active Neuron Weight Certainty : %f\n", output.fval);
	
	return(output);
}

NEURON_OUTPUT MAE_obtain_saccade_resulting_scale_factor(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	/* Sets the global network certainty value also */
	dynamic_scale_factor /= (float) obtain_measured_scale_factor(&nl_v1_activation_map);
	
	//if(dynamic_scale_factor > 5.0f)
	//	dynamic_scale_factor = 5.0f;
	//else if(dynamic_scale_factor < 0.1f)
	//	dynamic_scale_factor = 0.1f;
	
	output.fval =  dynamic_scale_factor;
	printf("Scale Factor : %f\n", output.fval);
	
	return(output);
}

NEURON_OUTPUT MAE_print_network_certainty_correlate(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	network_certainty = calculate_correlation_between_gaussian_filtered_outputs(&nl_v1_activation_map);
	printf("Correlate Certainty : %f\n", network_certainty);

	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_calculate_network_certainty_correlate(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	network_certainty = calculate_correlation_between_gaussian_filtered_outputs(&nl_v1_activation_map);
	output.fval =  network_certainty;
	
	return(output);
}

NEURON_OUTPUT MAE_print_network_angular_similarity(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	network_certainty = calculate_angular_similarity_between_gaussian_filtered_outputs(&nl_v1_activation_map);
	printf("Angular Sim. Certainty : %f\n", network_certainty);

	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_perform_network_flush_and_retraining(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output; 
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	clear_neural_layers_memory ("nl_v1_activation_map");	//clear neural layer memory
	update_input_layer_neurons_and_image(&in_pattern);
	train_visual_search_app();
	//sparse_train_visual_search_app(PERCENTAGE_OF_TRAINING_NEURONS);
	visual_search_state = RUNNING_NETWORK;
	
	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_is_saccade_certainty_above_threshold(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	float	threshold;
	
	if(!pParamList->next)
	{
		printf("MAE_is_saccade_certainty_above_threshold requires 1 float type parameter\n");
		output.ival = -1;	//Error return
		return(output);
	}
	
	threshold = pParamList->next->param.fval;
	
	if( network_certainty > threshold )
		output.ival = 1;
	else
		output.ival = 0;

	return(output);
}

NEURON_OUTPUT MAE_is_saccade_certainty_below_threshold(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	float	threshold;
	
	if(!pParamList->next)
	{
		printf("MAE_is_saccade_certainty_below_threshold requires 1 float type parameter\n");
		output.ival = -1;	//Error return
		return(output);
	}
	
	threshold = pParamList->next->param.fval;
	
	if( network_certainty < threshold )
		output.ival = 1;
	else
		output.ival = 0;

	return(output);
}

// A direct command from the interpreter could be used to do this
NEURON_OUTPUT MAE_reset_input_layer_xy_to_central_position(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	set_input_layer_translation(&in_pattern,IMAGE_WIDTH/2,IMAGE_HEIGHT/2);
	
	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_set_input_layer_xy_to_desired_position(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	
	// check for valid number of parameters
	if(!pParamList->next)
	{
		printf("MAE_set_input_layer_xy_to_desired_position requires 1 float type parameter\n");
		output.ival = -1;	//Error return
		return(output);
	}
	
	if(!pParamList->next->next)
	{
		printf("MAE_set_input_layer_xy_to_desired_position requires 1 float type parameter\n");
		output.ival = -1;	//Error return
		return(output);
	}
	
	
	set_input_layer_translation(&in_pattern,pParamList->next->param.ival,pParamList->next->next->param.ival);
	
	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_set_input_layer_xy_to_random_position(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	int	random_x;
	int	random_y;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	random_x = (int)( ((double) rand() / (double) LRAND48_MAX) * ((double) IMAGE_WIDTH) );
	random_y = (int)( ((double) rand() / (double) LRAND48_MAX) * ((double) IMAGE_HEIGHT) );
	
	set_input_layer_translation(&in_pattern,random_x,random_y);
	
	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_printf(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	char *string;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	string = pParamList->next->param.sval;

	printf("%s\n",string);
	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_printfInt(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;

	printf("%d\n",pParamList->next->param.ival);

	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_publish_OK(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;

	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	MAE_visual_search_output_message.x_point = in_pattern.wxd;
	MAE_visual_search_output_message.y_point = IMAGE_HEIGHT - in_pattern.wyd;
	MAE_visual_search_output_message.measured_scale_factor = obtain_measured_scale_factor(&nl_v1_activation_map);

	publish_output_message();	//publishes the output message

	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_publish_FAIL(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	publish_output_message();	//simply publishes the non initialized output message

	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_update_input_layer_neurons_and_image(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	update_input_layer_neurons_and_image(&in_pattern);
	
	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_forced_sleep(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	int param;

	if(!pParamList->next)
	{
		output.ival = -1;
		return(output);
	}
	
	param = pParamList->next->param.ival;
	sleep(param);

	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_print_frame_number_and_x_y_input_position(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	int bumblebee_frame_counter;
	
	if(!pParamList->next)
	{
		printf("MAE_print_frame_number_and_x_y_input_position requires 1 int type parameter\n");
		output.ival = -1;	//Error return
		return(output);
	}
	
	bumblebee_frame_counter = pParamList->next->param.ival;
	
	//Adjusted to convetional coordinate system
	printf("%d; %d; %d\n",bumblebee_frame_counter,in_pattern.wxd,IMAGE_HEIGHT - in_pattern.wyd);

	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_create_log_file(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	output_log_file = fopen(TEST_FILE_NAME,"w");
	
	if(!output_log_file)
		output.ival = -1;	//error condition
	else
		output.ival = 0;
	
	return(output);
}

NEURON_OUTPUT MAE_print_frame_number_and_x_y_input_position_to_output_file(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	int bumblebee_frame_counter;
	
	if(!pParamList->next)
	{
		printf("MAE_print_frame_number_and_x_y_input_position_to_output_file requires 1 int type parameter\n");
		output.ival = -1;	//Error return
		return(output);
	}
	
	bumblebee_frame_counter = pParamList->next->param.ival;
	
	//Adjusted to convetional coordinate system
	if(output_log_file)
		fprintf(output_log_file,"%d; %d; %d\n",bumblebee_frame_counter,in_pattern.wxd,IMAGE_HEIGHT - in_pattern.wyd);

	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_print_x_y_input_position_to_output_file_based_on_network_certainty(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	float	certainty,threshold;
	
	//Checks for the two parameters
	if(!pParamList->next)
	{
		if(!pParamList->next->next)
		{
			printf("MAE_print_x_y_input_position_to_output_file_based_on_network_certainty requires 2 float type parameters <certainty>, <threshold>\n");
			output.ival = -1;	//Error return
			return(output);
		}
		output.ival = -1;	//Error return
		return(output);
	}
	
	certainty = pParamList->next->param.fval;			//1st parameter as newtwork certainty
	threshold = pParamList->next->next->param.fval;	//2nd parameter as the desired threshold
	
	if(output_log_file)
	{
		if(certainty > threshold)									//If the certainty is above the desired threshold
			fprintf(output_log_file,"%d;%d\n",in_pattern.wxd,IMAGE_HEIGHT - in_pattern.wyd);	//Adjusted to convetional coordinate system
		else
			fprintf(output_log_file,"NaN;NaN\n");							//Print Not-a-number if not
	}

	output.ival = 0;	//Success return
	return(output);
}

NEURON_OUTPUT MAE_print_network_settings_to_log_file(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	if(output_log_file)
	{
		output.ival = 0;
		fprintf(output_log_file,"NEURAL VISUAL SEARCH EXECUTION LOG\n\n");
		fprintf(output_log_file,"Image Dimentions: %d x %d \n",IMAGE_WIDTH,IMAGE_HEIGHT);
		fprintf(output_log_file,"Neuron Layer Dimentions: %d x %d \n",NL_WIDTH,NL_HEIGHT);
		fprintf(output_log_file,"Synapses Per Neuron: %d \n",INPUTS_PER_NEURON);
		fprintf(output_log_file,"Log-Polar Gaussian Radius: %f \n", GAUSSIAN_RADIUS);
		fprintf(output_log_file,"Log-Polar Log-Factor: %f \n", LOG_FACTOR);			//Will be kept constant
		fprintf(output_log_file,"Log-Polar Scale Factor: %f \n", SCALE_FACTOR);
		fprintf(output_log_file,"Activation Bandwidth Size: %d \n", ACT_BAND_WIDTH);		//Will be kept proportional to the neuron layer width
		fprintf(output_log_file,"Maximum Number of Saccadic Moviments per Saccade: %d \n\n", MAX_NUMBER_OF_SACCADE);
	}
	else
		output.ival = -1; //Error condition
		
	return(output);	
}

NEURON_OUTPUT MAE_print_hit_rate_to_log_file(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;

	int	total_number_of_frames;
	int	total_number_of_frames_valid;
	int	percentage_of_hit_frames;
	float precision;
	float recall;
	float f;
	
	output.ival = 0;
	
	if((!pParamList->next)||(!pParamList->next->next)||(!pParamList->next->next->next))
	{
		printf("MAE_print_hit_rate_to_log_file requires 3 int type parameter\n");
		output.ival = -1;	//Error return
		return(output);
	}
	total_number_of_frames = pParamList->next->param.ival;			//1st parameter as total number of frames
	total_number_of_frames_valid = pParamList->next->next->param.ival;	//2st parameter as total valid number of frames
	percentage_of_hit_frames = pParamList->next->next->next->param.ival;	//3nd parameter as number of hit frames
	
	precision = (float)percentage_of_hit_frames / (float) total_number_of_frames;
	recall = (float)percentage_of_hit_frames / (float) total_number_of_frames_valid;
	f = (2 * precision * recall) / (precision + recall);

	if(output_log_file)
	{
		fprintf(output_log_file,"PRECISION: %f\n",precision);
		fprintf(output_log_file,"RECALL: %f\n",recall);
		fprintf(output_log_file,"F: %f\n",f);

		printf("PRECISION: %f\n",precision);
		printf("RECALL: %f\n",recall);
		printf("F: %f\n",f);

		printf("total_number_of_frames: %f\n",(float) total_number_of_frames);
		printf("total_number_of_frames_valid: %f\n",(float) total_number_of_frames_valid);
		printf("percentage_of_hit_frames: %f\n",(float)percentage_of_hit_frames);

	}
	else
	{
		output.ival = -1; //Error condition
		return(output);
	}		
	
	return(output);
}

NEURON_OUTPUT MAE_print_hit_rate_to_terminal(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;

	int	total_number_of_frames;
	int	total_number_of_frames_valid;
	int	percentage_of_hit_frames;
	float precision;
	float recall;
	float f;
	
	output.ival = 0;
	
	if((!pParamList->next)||(!pParamList->next->next)||(!pParamList->next->next->next))
	{
		printf("MAE_print_hit_rate_to_log_file requires 3 int type parameter\n");
		output.ival = -1;	//Error return
		return(output);
	}
	total_number_of_frames = pParamList->next->param.ival;			//1st parameter as total number of frames
	total_number_of_frames_valid = pParamList->next->next->param.ival;	//2st parameter as total valid number of frames
	percentage_of_hit_frames = pParamList->next->next->next->param.ival;	//3nd parameter as number of hit frames
	
	precision = (float)percentage_of_hit_frames / (float) total_number_of_frames;
	recall = (float)percentage_of_hit_frames / (float) total_number_of_frames_valid;
	f = (2 * precision * recall) / (precision + recall);

	
	if(output_log_file)
	{
		fprintf(output_log_file,"PRECISION: %f\n",precision);
		fprintf(output_log_file,"RECALL: %f\n",recall);
		fprintf(output_log_file,"F: %f\n",f);

		printf("PRECISION: %f\n",precision);
		printf("RECALL: %f\n",recall);
		printf("F: %f\n",f);

		printf("total_number_of_frames: %f\n",(float) total_number_of_frames);
		printf("total_number_of_frames_valid: %f\n",(float) total_number_of_frames_valid);
		printf("percentage_of_hit_frames: %f\n",(float)percentage_of_hit_frames);

	}
	else
	{
		output.ival = -1; //Error condition
		return(output);
	}		
	
	return(output);
}

NEURON_OUTPUT MAE_start_timer(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;

	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	clock_gettime(CLOCK_MONOTONIC, &start);
	//start = time(NULL);
	
	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_stop_timer(PARAM_LIST *pParamList)
{	
	NEURON_OUTPUT output;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	clock_gettime(CLOCK_MONOTONIC, &end);
	
	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_print_saccade_and_time_statistics_to_output_file(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	long int time_seconds;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	time_seconds = timespecDiff(&end,&start);

	if(output_log_file)
	{
		output.ival = 0;
		fprintf(output_log_file,"Total Elapsed Time(s): %ld\n",time_seconds);
		fprintf(output_log_file,"Total Number of Saccades: %d\n", total_number_of_saccades);
		fprintf(output_log_file,"Saccades per Second: %f\n", (float) total_number_of_saccades/ (float) time_seconds);
	}
	else
		output.ival = -1;
		
	return(output);
}

NEURON_OUTPUT MAE_print_saccade_time_and_frame_statistics_to_output_file(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	int total_nember_of_frames;
	long int time_seconds;
	
	//Check for number of frames parameter
	if(!pParamList->next)
	{
		printf("MAE_print_saccade_time_and_frame_statistics_to_output_file requires 1 int type parameter\n");
		output.ival = -1;	//Error return
		return(output);
	}
	
	time_seconds = timespecDiff(&end,&start);
	total_nember_of_frames = pParamList->next->param.ival;

	if(output_log_file)
	{
		output.ival = 0;
		fprintf(output_log_file,"\nTotal Elapsed Time(s): %ld\n",time_seconds);
		fprintf(output_log_file,"Total Number of Saccades: %d\n", total_number_of_saccades);
		fprintf(output_log_file,"Saccades per Second: %f\n", (float) total_number_of_saccades/ (float) time_seconds);
		fprintf(output_log_file,"Total Number of Frames: %d\n",total_nember_of_frames);
		fprintf(output_log_file,"Frames per Second: %f\n", (float) total_nember_of_frames/ (float) time_seconds);
	}
	else
		output.ival = -1;
		
	return(output);
}

NEURON_OUTPUT MAE_print_saccade_time_and_frame_statistics_to_terminal(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	int total_nember_of_frames;
	long int time_seconds;
	
	//Check for number of frames parameter
	if(!pParamList->next)
	{
		printf("MAE_print_saccade_time_and_frame_statistics_to_output_file requires 1 int type parameter\n");
		output.ival = -1;	//Error return
		return(output);
	}
	
	time_seconds = timespecDiff(&end,&start);
	total_nember_of_frames = pParamList->next->param.ival;

	if(output_log_file)
	{
		output.ival = 0;
		printf("\nGround Truth File : %s\n",CAMSHIFT_TEST_FILE_NAME);	//name of the test file
		printf("Total Elapsed Time(s): %ld\n",time_seconds);
		printf("Total Number of Saccades: %d\n", total_number_of_saccades);
		printf("Saccades per Second: %f\n", (float) total_number_of_saccades/ (float) time_seconds);
		printf("Total Number of Frames: %d\n",total_nember_of_frames);
		printf("Frames per Second: %f\n", (float) total_nember_of_frames/ (float) time_seconds);
	}
	else
		output.ival = -1;
		
	return(output);
}

NEURON_OUTPUT MAE_terminate_log_file(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;

	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	/* If the output file exists the close it*/
	if(output_log_file)
		fclose(output_log_file);
	
	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_open_comparison_log_file(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	char	line1[40];
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	comparison_log_file = fopen(CAMSHIFT_TEST_FILE_NAME,"r");
	
	if(!comparison_log_file)
		output.ival = -1;	//error condition
	else
	{
		output.ival = 0;
		fgets(line1,40,comparison_log_file);
	}

	return(output);
}

// The TLD comparison log files don't have the first line header
NEURON_OUTPUT MAE_open_TLD_comparison_log_file(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
		
	comparison_log_file = fopen(CAMSHIFT_TEST_FILE_NAME,"r");
	
	if(!comparison_log_file)
		output.ival = -1;	//error condition
	else
		output.ival = 0;

	return(output);
}

NEURON_OUTPUT MAE_read_comparison_log_file_and_check_for_hit(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	int	x,y,fn,xc,yc,w,h;
	char	comma;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	// only if the comparison log file is open
	if(comparison_log_file)
	{
		fscanf(comparison_log_file,"%d %c %d %c %d %c %d %c %d",&fn,&comma,&xc,&comma,&yc,&comma,&w,&comma,&h);
		x = in_pattern.wxd;
		y = IMAGE_HEIGHT - in_pattern.wyd;
		
		// boundary check
		if( x <= (xc+w/2) && x >= (xc-w/2) && y <= (yc+h/2) && y >= (yc-h/2) )
			output.ival = 1;
		else
			output.ival = 0;
	}
	else
		output.ival = -1;	//error condition

	return(output);
}

NEURON_OUTPUT MAE_is_inhibition_point_list_empty(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;

	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	if(!visual_search_mask_point_list)
		output.ival = 1;
	else
		output.ival = 0;
		
	return(output);
}

NEURON_OUTPUT MAE_calculate_global_hit_rate_from_comparison_log_file(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;

	int	total_number_of_frames;
	int	percentage_of_hit_frames;
	
	if(!pParamList->next)
	{
		if(!pParamList->next->next)
		{
			printf("MAE_calculate_global_hit_rate_from_comparison_log_file requires 2 int type parameter\n");
			output.ival = -1;	//Error return
			return(output);
		}
	}
	
	total_number_of_frames = pParamList->next->param.ival;		//1st parameter as total number of frames
	percentage_of_hit_frames = pParamList->next->next->param.ival;	//2nd parameter as number of hit frames
	
	output.fval = (float)percentage_of_hit_frames / (float) total_number_of_frames;
	global_hit_rate = output.fval;
	
	printf("Hit Rate: %f\n",output.fval);	//just prints out the hit percentage
	
	return(output);
}


int
max(int a, int b)
{
	return (a < b) ? b : a;
}


int
min(int a, int b)
{
	return (b < a) ? b : a;
}


double
getJaccardCoefficient(int leftCol, int topRow, int rightCol, int bottomRow, int gtLeftCol, int gtTopRow, int gtRightCol, int gtBottomRow)
{
	double jaccCoeff = 0.;

	if (!(leftCol > gtRightCol || rightCol < gtLeftCol || topRow > gtBottomRow || bottomRow < gtTopRow))
	{
		int interLeftCol = max(leftCol, gtLeftCol);
		int interTopRow = max(topRow, gtTopRow);
		int interRightCol = min(rightCol, gtRightCol);
		int interBottomRow = min(bottomRow, gtBottomRow);

		const double areaIntersection = (abs(interRightCol - interLeftCol) + 1) * (abs(interBottomRow - interTopRow) + 1);
		const double lhRoiSize = (abs(rightCol - leftCol) + 1) * (abs(bottomRow - topRow) + 1);
		const double rhRoiSize = (abs(gtRightCol - gtLeftCol) + 1) * (abs(gtBottomRow - gtTopRow) + 1);

		jaccCoeff = areaIntersection / (lhRoiSize + rhRoiSize - areaIntersection);
	}

	return jaccCoeff;
};


/*
 // verifica se o ponto dado pela busca visual esta dentro do bound box
NEURON_OUTPUT MAE_read_comparison_log_file_and_check_for_hit_in_TLD_dataset(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	float 	x1,x2,y1,y2;
	char	comma;
	int	x1_i,x2_i,y1_i,y2_i;
	int	x,y;

	pParamList = pParamList;
	// only if the comparison log file is open
	if(comparison_log_file)
	{
		fscanf(comparison_log_file,"%f %c %f %c %f %c %f",&x1,&comma,&y1,&comma,&x2,&comma,&y2);
		x = in_pattern.wxd;
		y = IMAGE_HEIGHT - in_pattern.wyd;
		x1_i = x1 + 0.5;
		x2_i = x2 + 0.5;
		y1_i = y1 + 0.5;
		y2_i = y2 + 0.5;

		// boundary check
		if (isnan(x1) || isnan(x2) || isnan(y1) || isnan(y2)){
			output.ival = 2;	// object out of scene
			//printf("FORA DE CENA\n");
		}
		else if( x <= x2_i && x >= x1_i && y <= y2_i && y >= y1_i ){
			output.ival = 1;	// a hit has happened
			//printf("ENCONTRADO\n");
		}
		else {
			output.ival = 0;	// out of box bound
			//printf("PERDIDO\n");
		}
	}
	else
		output.ival = -1;	//error condition

	return(output);
}
*/

NEURON_OUTPUT MAE_read_comparison_log_file_and_check_for_hit_in_TLD_dataset(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	float x1,x2,y1,y2;
	char comma;
	int x1_i,x2_i,y1_i,y2_i;
	int x3,x4,y3,y4;
	int x,y;
	float area, area_interseccao;
	float jaccard;
	
	pParamList = pParamList;
	
	// only if the comparison log file is open
	if(comparison_log_file)
	{
		fscanf(comparison_log_file,"%f %c %f %c %f %c %f",&x1,&comma,&y1,&comma,&x2,&comma,&y2);
		x = in_pattern.wxd;
		y = IMAGE_HEIGHT - in_pattern.wyd;
		x1_i = x1 + 0.5;
		x2_i = x2 + 0.5;
		y1_i = y1 + 0.5;
		y2_i = y2 + 0.5;

		x3 = x-((x2_i - x1_i)/2);
		y3 = y-((y2_i - y1_i)/2);
		x4 = x+((x2_i - x1_i)/2);
		y4 = y+((y2_i - y1_i)/2);

//		area = (float)((x2_i - x1_i) * (y2_i - y1_i));

		jaccard = getJaccardCoefficient(x3, y3, x4, y4, x1_i, y1_i, x2_i, y2_i);

		// boundary check
		if (isnan(x1) || isnan(x2) || isnan(y1) || isnan(y2))
		{
			output.ival = 2;	// object out of scene
			printf("FORA DE CENA\n");
		}
		else {
			if(jaccard >= 0.5){
				output.ival = 1;	// a hit has happened
				printf("ENCONTRADO\n");
			}
			else {
				output.ival = 0;	// out of box bound
				printf("PERDIDO\n");
			}
		}
	}
	else
		output.ival = -1;	//error condition

	return(output);
}


NEURON_OUTPUT MAE_terminate_comparison_log_file(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;

	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	if(comparison_log_file)
	{
		fflush(comparison_log_file);	/* Forces the buffer flush into the disk */
		fclose(comparison_log_file);
	}
	
	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_output_divergence_measures(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	double kullback_leibler_divergence;
	double jeffrey_divergence;
	double battacharayya_coefficient;

	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	kullback_leibler_divergence = neuron_layer_simetrized_kullback_leibler_divergence (&nl_v1_activation_map);
	jeffrey_divergence = neuron_layer_jeffrey_divergence (&nl_v1_activation_map);
	battacharayya_coefficient = neuron_layer_battacharayya_coefficient (&nl_v1_activation_map);

	printf("Simetralized KL-Divergence: %f\n",kullback_leibler_divergence);
	printf("Jeffrey Divergence: %f\n",jeffrey_divergence);	
	printf("Battacharayya Coefficient: %f\n",battacharayya_coefficient);
	
	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_different_pixel_certainty(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	float 	certainty;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */

	certainty = (float) output_different_outputs_certainty(&nl_v1_activation_map);
	printf("Different Output Certainty : %f\n",certainty);
	output.fval = certainty; 
	return(output);
}

NEURON_OUTPUT MAE_print_retrain_counter(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	
	pParamList = pParamList;	/* For Keeping The compiler Happy */
	
	printf("Number of Retrains: %d\n",pParamList->next->param.ival);
	
	output.ival = 0;
	return(output);
}

NEURON_OUTPUT MAE_hamming_distance_based_retrain(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;
	float threshold;
	
	threshold = pParamList->next->param.fval;	
	output.ival = hamming_based_neuron_layer_retrain(&nl_v1_activation_map,threshold);
	
	return(output);
}

NEURON_OUTPUT MAE_hamming_distance_output_certainty(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;

	pParamList = pParamList;	/* For Keeping The compiler Happy */
		
	output.fval = (float) neuron_layer_hamming_output_composed_certainty(&nl_v1_activation_map);
	printf("Hamming + Output Composed Certainty: %f\n",output.fval);
	
	return(output);	
}
