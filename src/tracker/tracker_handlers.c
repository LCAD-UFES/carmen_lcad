#include "tracker_handlers.h"
#include "tracker_user_functions/tracker_user_functions.h"
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

	tracker_train();
}

// Copies the message image into the neuron layer and performs the saccadic movement
void
perform_network_test(const unsigned char *image, int *x_saccade_vector __attribute__ ((unused)), int *y_saccade_vector __attribute__ ((unused)))
{
	copy_raw_image_into_input(image, &in_saccade_current);

	update_input_layer_neurons_and_image_light(&in_saccade_current);		//Must update the input layer

	visual_search(&in_saccade_current);
	/*tracker_saccade(&in_saccade_current,
			MAX_NUMBER_OF_SACCADE, MIN_THRESHOLD_OF_SACCADE,
			x_saccade_vector, y_saccade_vector);*/
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
	if ( (l <= 0.0f) ||(c <= 0.0f) || (u <= 0.0f) || (m <= 0.0f) || (a < 2.0f) )
		return 0.0f;

	return (u * (pow(a, (l / m)) - 1.0f)) / (c * (a - 1.0f));
}


int
truncate_value(double *value)
{
	double fractpart, intpart;

	fractpart = modf(*value, &intpart);
	fractpart = fractpart > 0.0 ? 0.5 : 0.0;
	*value = fractpart;

	return ((int) intpart);
}


void
set_scale_factor_by_width(double leftCol, double topRow, double rightCol, double bottomRow)
{
	double d_width, altura, largura;
	double halph_band_width_fp = 7.0 * ((double) NL_WIDTH / 65.0);
	int halph_band_width_int = truncate_value(&halph_band_width_fp);

	g_halph_band_width = halph_band_width_int + halph_band_width_fp;

	altura = fabs(topRow - bottomRow);
	largura = fabs(rightCol - leftCol);

	d_width = (largura + altura) / 2.0;
	dynamic_scale_factor = get_scale_factor(2.0 * g_halph_band_width, d_width, IMAGE_WIDTH_RESIZED, nl_v1_pattern.dimentions.x, LOG_FACTOR);


}


void
train_message(carmen_tracker_train_message *training_message)
{
//	int x, y;
	int xc = -1, yc = -1;
	double leftCol, topRow, rightCol, bottomRow;

	leftCol = (int)training_message->reference_points[0].x;
	bottomRow = (int)training_message->reference_points[0].y;
	rightCol = (int)training_message->reference_points[1].x;
	topRow = (int)training_message->reference_points[1].y;

	xc = (leftCol + rightCol)/2;
	yc = (bottomRow + topRow)/2;

	set_scale_factor_by_width(leftCol, topRow, rightCol, bottomRow);

	if ((height_in_train == -1.0) && (width_in_train == -1))
	{
		height_in_train = fabs(topRow - bottomRow);
		width_in_train = fabs(rightCol - leftCol);
	}

	clear_input_points(&in_saccade_trained);

	clear_neural_layers_memory ("nl_v1_activation_map");

	draw_input_points(&in_saccade_trained, training_message->reference_points, training_message->reference_points_size);

	perform_network_training(training_message->reference_image, xc, yc);
}

/*void
test_message_output(const carmen_tracker_test_message *test_message,
		carmen_tracker_output_message *output_message)
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
	output_message->measured_scale_factor = tracker_scale();
	output_message->measured_confidence = tracker_confidence();
	output_message->timestamp = test_message->timestamp;
	output_message->host = carmen_get_host();
}
*/

void
test_message_output(const carmen_tracker_test_message *test_message,
		carmen_tracker_output_message *output_message)
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
	output_message->measured_scale_factor = tracker_scale();
	output_message->measured_confidence = tracker_confidence();
	output_message->timestamp = test_message->timestamp;
	output_message->host = carmen_get_host();
}

void
training_message_handler(carmen_tracker_train_message *training_message)
{
	train_message(training_message);
}

void
test_message_handler(carmen_tracker_test_message *test_message)
{
	carmen_tracker_output_message output_message;

	test_message_output(test_message, &output_message);

	IPC_RETURN_TYPE err = IPC_publishData(CARMEN_TRACKER_OUTPUT_MESSAGE_NAME, &output_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_TRACKER_OUTPUT_MESSAGE_NAME);
}

void
query_test_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
	      void *clientData __attribute__ ((unused)) )
{
	carmen_tracker_test_message test_message;
	carmen_tracker_output_message output_message;
	FORMATTER_PTR formatter;
	IPC_RETURN_TYPE err;

	formatter = IPC_msgInstanceFormatter(msgRef);

	err = IPC_unmarshallData(formatter, callData, &test_message, sizeof(carmen_tracker_test_message));
	carmen_test_ipc_return(err, "Could not unmarshall data", IPC_msgInstanceName(msgRef));
	IPC_freeByteArray(callData);

	test_message_output(&test_message, &output_message);

	err = IPC_respondData(msgRef, CARMEN_TRACKER_OUTPUT_MESSAGE_NAME, &output_message);
	carmen_test_ipc(err, "Could not respond", CARMEN_TRACKER_OUTPUT_MESSAGE_NAME);

	if(test_message.associated_points)
		free(test_message.associated_points);

	if(test_message.associated_image)
		free(test_message.associated_image);

}

void
query_training_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
	      void *clientData __attribute__ ((unused)) )
{
	carmen_tracker_train_message training_message;
	carmen_tracker_output_training_message output_message;
	FORMATTER_PTR formatter;
	IPC_RETURN_TYPE err;

	formatter = IPC_msgInstanceFormatter(msgRef);
	err = IPC_unmarshallData(formatter, callData, &training_message, sizeof(carmen_tracker_train_message));
	carmen_test_ipc_return(err, "Could not unmarshall data", IPC_msgInstanceName(msgRef));
	IPC_freeByteArray(callData);

	train_message(&training_message);

	err = IPC_respondData(msgRef, CARMEN_TRACKER_OUTPUT_TRAINING_MESSAGE_NAME, &output_message);
	carmen_test_ipc(err, "Could not respond", CARMEN_TRACKER_OUTPUT_TRAINING_MESSAGE_NAME);

	if(training_message.reference_points)
		free(training_message.reference_points);

	if(training_message.reference_image)
		free(training_message.reference_image);
}
