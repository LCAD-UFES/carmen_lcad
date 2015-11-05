#include <map>
#include <algorithm>
#include <locale.h>
#include "vergence_user_functions.h"
#include "vergence_filters.h"
#include "../vergence_handlers.h"
#include <carmen/stereo_util.h>

float *gaussian_filtered_training_pattern = NULL; 		//training image (also used in filter module)
float gaussian_filtered_training_pattern_sum = 0.0f;	//total filtered training pattern accumulated weight

int	refined_saccade = 0;		//Refined saccade flag
int	trained_layer_y_position = 0;

static stereo_util camera_params;
static carmen_position_t right_image_point;
static carmen_position_t left_image_point;
static carmen_vector_3D_t line_points[2] = {{0.0,0.0,0.0},{0.0,0.0,0.0}};
static int line_point_index = 0;

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

int
init_user_functions ()
{
	char *locale_string;

	/* connect to IPC server */
	carmen_ipc_initialize(global_argc, global_argv);

	carmen_param_check_version(global_argv[0]);

	carmen_vergence_define_message_output_train();

	carmen_vergence_define_message_output_test();

	carmen_vergence_subscribe_query_train(query_vergence_train_message_handler);

	carmen_vergence_subscribe_query_test(query_vergence_test_message_handler);

//	FIXME o número da camera ta hardcode, colocar pra ler da linha de comando
	camera_params = get_stereo_instance(6, -1, -1);

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
make_input_image_vergence (INPUT_DESC *input, int w, int h)
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
init_vergence (INPUT_DESC *input)
{
	int x,y;

	make_input_image_vergence (input, IMAGE_WIDTH, IMAGE_HEIGHT);

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
		conditional_filter_update(get_filter_by_name((char*)"in_vergence_translated_filter")) &
		conditional_filter_update(get_filter_by_name((char*)"in_vergence_translated_gaussian_filter"))
		))
		return 0;

	return 1;
}

void
input_generator (INPUT_DESC *input, int status)
{

	if (input->win == 0)
	{
		init_vergence (input);
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
		init_vergence (input);
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
output_handler_average_value (OUTPUT_DESC *output,
		int type_call __attribute__ ((unused)) ,
		int mouse_button __attribute__ ((unused)) ,
		int mouse_state __attribute__ ((unused)) )
{
	int u, v, w, h;
	int upper_x_lim,lower_x_lim;
	float average_value;
	char *nl_target_coordinates_name = NULL;
	NEURON_LAYER *nl_target_coordinates = NULL;
	
	// Gets the output handler parameters (just the coordinates neuron layer name)
	nl_target_coordinates_name = output->output_handler_params->next->param.sval;
	
	// Gets the target coordinates neuron layer
	nl_target_coordinates = get_neuron_layer_by_name (nl_target_coordinates_name);
	
	// Gets the Neuron Layer Dimentions
	w = output->neuron_layer->dimentions.x;
	h = output->neuron_layer->dimentions.y;
	
	if(!refined_saccade)
	{
		lower_x_lim = 0;
		upper_x_lim = w;
	}
	else
	{
		lower_x_lim = (w/8)*3;
		upper_x_lim = w - (w/8)*3;
	}
		
	// Accumulates average value position for the whole neuron layer.
	average_value = 0.0;
	for (v = 0; v < h; v++)
		for (u = lower_x_lim; u < upper_x_lim; u++)
		{
			average_value += (float) output->neuron_layer->neuron_vector[v * w + u].output.ival;
		}
	
	average_value /= (float) (upper_x_lim - lower_x_lim)*h;
	printf("Average Disparity Value: %f\n",average_value);
	
	// Saves the average value position
	nl_target_coordinates->neuron_vector[0].output.fval = average_value;
	
}

void
vergence_saccade (INPUT_DESC *input,
		int max_number_of_saccades)
{
	float x;
	int count = 0;
	
	count = 0;	// Saccade count set as zero
	
	// Saccade until reach the target
	x = nl_target_coordinates.neuron_vector[0].output.fval;
	do
	{
		input->wxd += (int) (x + 0.5);
				
		// input->wyd will be kept due to horizontal vergence saccades only
		set_input_layer_translation(input, input->wxd, input->wyd);

		//Get next delta coordinates from the last layer after saccadic movement
		x = nl_target_coordinates.neuron_vector[0].output.fval;
		
		count++;
	} 
	while ((count < max_number_of_saccades) && x > 0.5 );
	
}

// Performs a single refined saccade using the internal neuron layer band
void
vergence_saccade_refined (INPUT_DESC *input)
{
	float x;
	
	refined_saccade = 1;
	set_input_layer_translation(input, input->wxd, input->wyd);
	x = nl_target_coordinates.neuron_vector[0].output.fval;
	refined_saccade = 0;
	input->wxd += (int) (x + 0.5);
	set_input_layer_translation(input, input->wxd, input->wyd);
	x = nl_target_coordinates.neuron_vector[0].output.fval;

}
void
input_controler (INPUT_DESC *input, int status __attribute__ ((unused)) )
{
	if ((move_active == 1) &&
	    (input->mouse_button == GLUT_LEFT_BUTTON) &&
	    (input->mouse_state == GLUT_DOWN))
	{
		// Translate the input image & Move the input center cursor
		printf("Move to x:%d y:%d\n",input->wxd, input->wyd);
		set_input_layer_translation(input, input->wxd, input->wyd);
	}
	input->mouse_button = -1;
}

void
input_controler2 (INPUT_DESC *input, int status __attribute__ ((unused)) )
{
	input->mouse_button = -1;
}

/* Sets the neuron layer band with an integer constant value */
void
set_neuron_layer_band_constant_value (NEURON_LAYER *neuron_layer, int activation_value)
{
	int u, v, w, h;
	
	w = neuron_layer->dimentions.x;
	h = neuron_layer->dimentions.y;
	
	for(u = 0; u < w; u++)
		for(v = 0;v < h; v++)
			neuron_layer->neuron_vector[u + w * v].output.ival = activation_value;
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
vergence_confidence(NEURON_LAYER *neuron_layer, int band_width)
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
vergence_scale(NEURON_LAYER *neuron_layer)
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

void
set_input_layer_translation(INPUT_DESC *input, int x, int y)
{
	input->wxd = x;
	input->wyd = y;
	translation_filter_deltaX = (float)(-IMAGE_WIDTH/2  + input->wxd);
	translation_filter_deltaY = (float)(-IMAGE_HEIGHT/2 + input->wyd);
	move_input_window (input->name, input->wxd, input->wyd);
}

void
input_filters_and_outputs_update(void)
{
	conditional_filter_update(get_filter_by_name((char*)"in_vergence_translated_filter"));
	
	if(get_output_by_name((char*)"in_vergence_current_filtered_out"))
		output_update(get_output_by_name((char*)"in_vergence_current_filtered_out"));

	conditional_filter_update(get_filter_by_name((char*)"in_vergence_translated_gaussian_filter"));

	if(get_output_by_name((char*)"in_vergence_translated_gaussian_out"))
		output_update(get_output_by_name((char*)"in_vergence_translated_gaussian_out"));
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
			
// Trains the vergence module (positive and negative values)
void
vergence_train(void)
{
	int i;
	translation_filter_deltaX -= MAX_DISP_NUM;
	for(i = -MAX_DISP_NUM; i < MAX_DISP_NUM ; i++)
	{
		translation_filter_deltaX += 1.0; //1 pixel left shift
		
		set_neuron_layer_band_constant_value(&nl_v1_activation_map,(-i));
		update_input_layer_neurons_and_image(&in_vergence_current);
		all_outputs_update();
		train_neuron_layer((char*)"nl_v1_activation_map");
	
		trained_layer_y_position = in_vergence_current.wyd;
	}
	all_outputs_update();
}

carmen_vector_3D_p
get_point_3D(carmen_position_p point_right, carmen_position_p point_left)
{
	int disparity;

	disparity = point_left->x - point_right->x;

	return reproject_single_point_to_3D(&camera_params, *point_right, disparity);
}

void
calc_distance(const carmen_vector_3D_p point_3D)
{
	if (point_3D)
	{
		line_point_index = (line_point_index+1)%2;
		line_points[line_point_index] = *point_3D;
		carmen_warn("x = %fm\n", point_3D->x);
		carmen_warn("y = %fm\n", point_3D->y);
		carmen_warn("z = %fm\n", point_3D->z);
	}

	double distance = sqrt(
			pow(line_points[0].x-line_points[1].x, 2.0) +
			pow(line_points[0].y-line_points[1].y, 2.0) +
			pow(line_points[0].z-line_points[1].z, 2.0));

	carmen_warn("d = %fm\n", distance);
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
			in_vergence_trained.wxd = in_vergence_current.wxd;
			in_vergence_trained.wyd = in_vergence_current.wyd;
			vergence_train();
			break;
		// Vergence until reach the target
		case 'V':
		case 'v':
			vergence_saccade(&in_vergence_current, MAX_NUMBER_OF_SACCADE);
			break;
		// Performs a single refined saccade over the image
		case 'R':
		case 'r':
			vergence_saccade_refined(&in_vergence_current);
			break;
		// Force the filters in_vergence_filtered_filter & in_vergence_filtered_translated_filter to be updated
		case 'U':
		case 'u':
			update_input_filters();
			break;
		// Forgets last training
		case 'F':
		case 'f':
			clear_neural_layers_memory((char*)"nl_v1_activation_map");
			break;
		// Restores the horizontal training position
		case 'Y':
		case 'y': 
			set_input_layer_translation(&in_vergence_current, in_vergence_current.wxd, trained_layer_y_position);
			break;
		// Moves a single pixel to the right
		case 'M':
		case 'm':
			set_input_layer_translation(&in_vergence_current, in_vergence_current.wxd + 1, in_vergence_current.wyd);
			break;
		// Moves a single pixel to the left
		case 'N':
		case 'n':
			set_input_layer_translation(&in_vergence_current, in_vergence_current.wxd - 1, in_vergence_current.wyd);
			break;
		case 'C':
		case 'c':
			right_image_point.x = in_vergence_trained.wxd;
			right_image_point.y = in_vergence_trained.wyd;
			left_image_point.x = in_vergence_current.wxd;
			left_image_point.y = in_vergence_current.wyd;
			carmen_vector_3D_p point_3D = get_point_3D(&right_image_point, &left_image_point);
			calc_distance(point_3D);
			break;
	}
	
	return;
}

NEURON_OUTPUT
run_train(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;

	vergence_train();

	return (output);
}

NEURON_OUTPUT
run_test(PARAM_LIST *pParamList)
{
	NEURON_OUTPUT output;

	vergence_saccade(&in_vergence_current, MAX_NUMBER_OF_SACCADE);
	vergence_saccade_refined(&in_vergence_current);

	return (output);
}
