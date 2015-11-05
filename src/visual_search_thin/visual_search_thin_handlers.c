#include "visual_search_thin_handlers.h"
#include "visual_search_thin_user_functions/visual_search_thin_user_functions.h"
#include <mae.h>

void
copy_raw_image_into_input(const unsigned char* raw_image,/* int step,*/ INPUT_DESC *input)
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
//					red = raw_image[im_size - 3 * (i * input->ww + (input->ww- j) ) + 0];
//					green = raw_image[im_size - 3 * (i * input->ww + (input->ww- j) ) + 1];
//					blue = raw_image[im_size - 3 * (i * input->ww + (input->ww- j) ) + 2];

					red = raw_image[sizeof(unsigned int)*(i * input->ww + j) + 0];
					green = raw_image[sizeof(unsigned int)*(i * input->ww + j) + 1];
					blue = raw_image[sizeof(unsigned int)*(i * input->ww + j) + 2];

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
//			for (i = 0; i < input->wh ; i++)
//			{
//				for (j = 0; j < input->ww ; j++)
//				{	// 3 channel image
//					input->image[3 * (i * input->ww + j) + 0] = (GLubyte) raw_image[3*(i * input->ww + j) + 0];
//					input->image[3 * (i * input->ww + j) + 1] = (GLubyte) raw_image[3*(i * input->ww + j) + 1];
//					input->image[3 * (i * input->ww + j) + 2] = (GLubyte) raw_image[3*(i * input->ww + j) + 2];
//				}
//			}

			for (i = 0; i < input->wh; i++)
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

void
clear_input_points(INPUT_DESC *input)
{
	if (input->cross_list != NULL)
	{
		free(input->cross_list);
		input->cross_list = NULL;
		input->cross_list_size = 0;
	}
}

void
draw_input_points(INPUT_DESC *input, const carmen_position_t *points, int size)
{
	int i;

	if ((input->cross_list == NULL) || (input->cross_list_size < size))
	{
		input->cross_list = (XY_PAIR*)realloc(input->cross_list, size * sizeof(XY_PAIR));
		input->cross_list_size = size;
	}

	for(i = 0; i < size; i++)
	{
		input->cross_list[i].x = (int)points[i].x;
		input->cross_list[i].y = (int)points[i].y;
	}

	input->cross_list_colored = 1;
}

void
update_network_training(INPUT_DESC *input, const unsigned char *image, int x, int y)
{
	copy_raw_image_into_input(image, input);

	update_input_layer_neurons_and_image_light(input);

	set_input_layer_translation(input, x, y);
}

// Copies the message image into the neuron layer and train it with the desired activation pattern
void
perform_network_training(const unsigned char *image, int x, int y)
{
	update_network_training(&in_saccade_trained, image, x, y);

	update_network_training(&in_saccade_current, image, x, y);

	visual_search_thin_train();
}

// Copies the message image into the neuron layer and performs the saccadic movement
void
perform_network_test(const unsigned char *image, int *x_saccade_vector, int *y_saccade_vector)
{
	copy_raw_image_into_input(image, &in_saccade_current);

	update_input_layer_neurons_and_image_light(&in_saccade_current);		//Must update the input layer

	visual_search_thin_saccade(&in_saccade_current,
			MAX_NUMBER_OF_SACCADE, MIN_THRESHOLD_OF_SACCADE,
			x_saccade_vector, y_saccade_vector);
}

/*!
*********************************************************************************
* Function: compute_scale_factor
* Description: Computes scale factor based object of interest width in input image (planar)
* 				and in neurons activation band considering log polar transformation
* Inputs:
* 			l -> activation band width of the neuron layer
* 			c -> original width of the region of interest
* 			u -> target width (resized) of the region of interest
* 			m -> width of the neuron layer
* 			a -> log factor of the neuron's distribution
* Output: scale factor multiplier (values greater than 1.0 scale up and values less than 1.0 scale down)
*********************************************************************************
FIXME usar a função da MAE
*/
float
get_scale_factor(float l, float c, float u, float m, float a)
{
	if ( (l <= 0.0) ||(c <= 0.0) || (u <= 0.0) || (m <= 0.0) || (a < 2.0) )
		return 0.0;

	return (u * (pow(a, (l / m)) - 1.0)) / (c * (a - 1.0));
}

void
set_scale_factor_by_width(int width)
{

	if (width > 0)
	{
		dynamic_scale_factor = get_scale_factor((float)BAND_WIDTH * ((float) NL_WIDTH / 96.0f),
				(float)width, (float)IMAGE_WIDTH*2.0, (float)nl_v1_pattern.dimentions.x, (float)LOG_FACTOR);
	}
	else
	{
		dynamic_scale_factor = 1.0;
	}
}

void
train_message(carmen_visual_search_thin_train_message *training_message)
{
//	int x, y;
	int xc = -1, yc = -1;

	if(training_message->reference_points_size == 2){

		int xls = (int)training_message->reference_points[0].x;
		int yls = (int)training_message->reference_points[0].y;

		int xri = (int)training_message->reference_points[1].x;
		int yri = (int)training_message->reference_points[1].y;

		xc = (xls + xri)/2;
		yc = (yls + yri)/2;

		set_scale_factor_by_width(abs(xls - xri));
	}else if(training_message->reference_points_size < 2){

		xc = (int)training_message->reference_points[0].x;
		yc = (int)training_message->reference_points[0].y;

		set_scale_factor_by_width(0);
	}

//	x = (int)training_message->reference_points[training_message->reference_points_size-1].x;
//	y = (int)training_message->reference_points[training_message->reference_points_size-1].y;

	clear_input_points(&in_saccade_trained);

	clear_neural_layers_memory ("nl_v1_activation_map");

	draw_input_points(&in_saccade_trained, training_message->reference_points, training_message->reference_points_size);

	perform_network_training(training_message->reference_image, xc, yc);
}

void
test_message_output(const carmen_visual_search_thin_test_message *test_message,
		carmen_visual_search_thin_output_message *output_message)
{
	int x, y;

//	dynamic_scale_factor = test_message->scale;

	clear_input_points(&in_saccade_current);

	draw_input_points(&in_saccade_current, test_message->associated_points, test_message->associated_points_size);

	perform_network_test(test_message->associated_image, &x, &y);

	output_message->saccade_vector.x = (double)x;
	output_message->saccade_vector.y = (double)y;
	output_message->saccade_point.x = (double)in_saccade_current.wxd;
	output_message->saccade_point.y = (double)in_saccade_current.wyd;
	output_message->measured_scale_factor = visual_search_thin_scale(&nl_v1_activation_map);
	output_message->measured_confidence = visual_search_thin_confidence(&nl_v1_activation_map, BAND_WIDTH);
	output_message->timestamp = test_message->timestamp;
	output_message->host = carmen_get_host();
}

void
training_message_handler(carmen_visual_search_thin_train_message *training_message)
{
	train_message(training_message);
}

void
test_message_handler(carmen_visual_search_thin_test_message *test_message)
{
	carmen_visual_search_thin_output_message output_message;

	test_message_output(test_message, &output_message);

	IPC_RETURN_TYPE err = IPC_publishData(CARMEN_VISUAL_SEARCH_THIN_OUTPUT_MESSAGE_NAME, &output_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_VISUAL_SEARCH_THIN_OUTPUT_MESSAGE_NAME);
}

void
query_test_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
	      void *clientData __attribute__ ((unused)) )
{
	carmen_visual_search_thin_test_message test_message;
	carmen_visual_search_thin_output_message output_message;
	FORMATTER_PTR formatter;
	IPC_RETURN_TYPE err;

	formatter = IPC_msgInstanceFormatter(msgRef);

	err = IPC_unmarshallData(formatter, callData, &test_message, sizeof(carmen_visual_search_thin_test_message));
	carmen_test_ipc_return(err, "Could not unmarshall data", IPC_msgInstanceName(msgRef));
	IPC_freeByteArray(callData);

	test_message_output(&test_message, &output_message);

	err = IPC_respondData(msgRef, CARMEN_VISUAL_SEARCH_THIN_OUTPUT_MESSAGE_NAME, &output_message);
	carmen_test_ipc(err, "Could not respond", CARMEN_VISUAL_SEARCH_THIN_OUTPUT_MESSAGE_NAME);

	if(test_message.associated_points)
		free(test_message.associated_points);

	if(test_message.associated_image)
		free(test_message.associated_image);

}

void
query_training_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
	      void *clientData __attribute__ ((unused)) )
{
	carmen_visual_search_thin_train_message training_message;
	carmen_visual_search_thin_output_training_message output_message;
	FORMATTER_PTR formatter;
	IPC_RETURN_TYPE err;

	formatter = IPC_msgInstanceFormatter(msgRef);
	err = IPC_unmarshallData(formatter, callData, &training_message, sizeof(carmen_visual_search_thin_train_message));
	carmen_test_ipc_return(err, "Could not unmarshall data", IPC_msgInstanceName(msgRef));
	IPC_freeByteArray(callData);

	train_message(&training_message);

	err = IPC_respondData(msgRef, CARMEN_VISUAL_SEARCH_THIN_OUTPUT_TRAINING_MESSAGE_NAME, &output_message);
	carmen_test_ipc(err, "Could not respond", CARMEN_VISUAL_SEARCH_THIN_OUTPUT_TRAINING_MESSAGE_NAME);

	if(training_message.reference_points)
		free(training_message.reference_points);

	if(training_message.reference_image)
		free(training_message.reference_image);
}
