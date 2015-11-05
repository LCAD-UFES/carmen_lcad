#include "vergence_handlers.h"
#include "vergence_user_functions/vergence_user_functions.h"
#include <mae.h>

void
copy_raw_image_into_input(const unsigned char *raw_image, INPUT_DESC *input)
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
//			for (i = 0; i < input->wh ; i++)
//			{
//				for (j = 0; j < input->ww; j++)
//				{	// 3 channel image
//					input->image[3 * (i * input->ww + j) + 0] = (GLubyte) raw_image[im_size - 3 * (i * input->ww + (input->ww- j) ) + 0];
//					input->image[3 * (i * input->ww + j) + 1] = (GLubyte) raw_image[im_size - 3 * (i * input->ww + (input->ww- j) ) + 1];
//					input->image[3 * (i * input->ww + j) + 2] = (GLubyte) raw_image[im_size - 3 * (i * input->ww + (input->ww- j) ) + 2];
//				}
//			}
			for (i = 0; i < input->wh ; i++)
			{
				for (j = 0; j < input->ww ; j++)
				{	// 3 channel image
					input->image[3 * (i * input->ww + j) + 0] = (GLubyte) raw_image[3 * (i * input->ww + j) + 0];
					input->image[3 * (i * input->ww + j) + 1] = (GLubyte) raw_image[3 * (i * input->ww + j) + 1];
					input->image[3 * (i * input->ww + j) + 2] = (GLubyte) raw_image[3 * (i * input->ww + j) + 2];
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

	update_input_layer_neurons_and_image(input);

	set_input_layer_translation(input, x, y);
}

// Copies the message image into the neuron layer and train it with the desired activation pattern
void
perform_network_training(const unsigned char *image, int x, int y)
{
	update_network_training(&in_vergence_trained, image, x, y);

	update_network_training(&in_vergence_current, image, x, y);

	vergence_train();
}

// Copies the message image into the neuron layer and performs the saccadic movement
void
perform_network_test(const unsigned char *image)
{
	copy_raw_image_into_input(image, &in_vergence_current);

	update_input_layer_neurons_and_image(&in_vergence_current);		//Must update the input layer

	vergence_saccade(&in_vergence_current, MAX_NUMBER_OF_SACCADE);

	vergence_saccade_refined(&in_vergence_current);
}

void
train_message(carmen_vergence_train_message *training_message)
{
	int x, y;

	x = (int)training_message->reference_points[training_message->reference_points_size-1].x;
	y = (int)training_message->reference_points[training_message->reference_points_size-1].y;

	clear_input_points(&in_vergence_trained);

	clear_neural_layers_memory ("nl_v1_activation_map");

	perform_network_training(training_message->reference_image, x, y);
}

void
test_message_output(const carmen_vergence_test_message *test_message,
		carmen_vergence_test_output_message *output_message)
{
	perform_network_test(test_message->associated_image);

	output_message->vergence_point.x = (double)in_vergence_current.wxd;
	output_message->vergence_point.y = (double)in_vergence_current.wyd;
	output_message->confidence = 0.0;
	output_message->timestamp = test_message->timestamp;
	output_message->host = carmen_get_host();
}

void
query_vergence_train_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
	      void *clientData __attribute__ ((unused)) )
{
	carmen_vergence_train_message training_message;
	carmen_vergence_train_output_message output_message;
	FORMATTER_PTR formatter;
	IPC_RETURN_TYPE err;

	formatter = IPC_msgInstanceFormatter(msgRef);
	err = IPC_unmarshallData(formatter, callData, &training_message, sizeof(carmen_vergence_train_message));
	IPC_freeByteArray(callData);

	carmen_test_ipc_return(err, "Could not unmarshall data", IPC_msgInstanceName(msgRef));

	train_message(&training_message);

	output_message.host = carmen_get_host();
	output_message.timestamp = carmen_get_time();

	err = IPC_respondData(msgRef, CARMEN_VERGENCE_TRAIN_OUTPUT_MESSAGE_NAME, &output_message);
	carmen_test_ipc(err, "Could not respond", CARMEN_VERGENCE_TRAIN_OUTPUT_MESSAGE_NAME);

}

void
query_vergence_test_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
	      void *clientData __attribute__ ((unused)) )
{
	carmen_vergence_test_message test_message;
	carmen_vergence_test_output_message output_message;
	FORMATTER_PTR formatter;
	IPC_RETURN_TYPE err;

	formatter = IPC_msgInstanceFormatter(msgRef);
	err = IPC_unmarshallData(formatter, callData, &test_message, sizeof(carmen_vergence_test_message));
	IPC_freeByteArray(callData);

	carmen_test_ipc_return(err, "Could not unmarshall data", IPC_msgInstanceName(msgRef));

	test_message_output(&test_message, &output_message);

	err = IPC_respondData(msgRef, CARMEN_VERGENCE_TEST_OUTPUT_MESSAGE_NAME, &output_message);
	carmen_test_ipc(err, "Could not respond", CARMEN_VERGENCE_TEST_OUTPUT_MESSAGE_NAME);

}

