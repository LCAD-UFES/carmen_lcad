#include "voice_recognition_utils.h"
#include <opencv/cv.h>
#include <stdlib.h>
#include <math.h>

int
mae_int_random(int max)
{
  return (int)(max*(rand()/(RAND_MAX+1.0)));
}

double
mae_double_random(double max)
{
  return max * (double)rand() / (double)RAND_MAX;
}

double
mae_uniform_random(double min, double max)
{
  return min + (rand() / (double)RAND_MAX) * (max - min);
}

double
mae_gaussian_random(double mean, double std)
{
  const double norm = 1.0 / (RAND_MAX + 1.0);
  double u = 1.0 - rand() * norm;                  /* can't let u == 0 */
  double v = rand() * norm;
  double z = sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v);
  
  return mean + std * z;
}

double
mae_normal_distribution(double std)
{
	double normal_sample = 0;
	int i;
	
	for (i = 0; i < 12; i++)
		normal_sample += std * (2.0 * (mae_double_random(1.0) - 0.5));

	return normal_sample / 2.0;
}

double
mae_radians_to_degrees(double theta)
{
  return (theta * 180.0 / M_PI);
}

double
mae_degrees_to_radians(double theta)
{
  return (theta * M_PI / 180.0);
}

void
copy_image_to_neuron_layer(NEURON_LAYER *neuron_layer, CvRect *neuron_layer_roi, IplImage *image)
{
	NEURON_OUTPUT neuron_output;
	NEURON *neuron_vector;
	uchar *image_pixel;
	int x, y, w, h, xo, yo;
	int r, g, b;

	w = image->width;
	h = image->height;

	if (neuron_layer_roi)
	{
		xo = neuron_layer_roi->x;
		yo = neuron_layer_roi->y;
	}
	else
	{
		xo = 0;
		yo = 0;
	}

	neuron_vector = neuron_layer->neuron_vector;

	for (y = h-1; y >= 0; y--)
	{
		for (x = 0; x < w; x++)
		{
			image_pixel = (uchar*) (image->imageData + (h-1-y) * image->widthStep);

			switch (image->nChannels)
			{
				case (3):
					r = (int) image_pixel[3*x+2];
					g = (int) image_pixel[3*x+1];
					b = (int) image_pixel[3*x+0];
					neuron_output.ival = PIXEL (r, g, b);
					neuron_vector[(xo+x) + (yo+y)*w].output = neuron_output;
					break;
				case (1):
					neuron_output.ival = (int) image_pixel[x];
					neuron_vector[(xo+x) + (yo+y)*w].output = neuron_output;
					break;
				default:
					fprintf (stderr, "(copy_image_to_neuron_layer) Error: invalid image channels: %d.\n", image->nChannels);
					exit (1);
					break;
			}
		}
	}
}

void
copy_neuron_layer_to_image(IplImage *image, CvRect *neuron_layer_roi, NEURON_LAYER *neuron_layer)
{
	NEURON_OUTPUT neuron_output;
	NEURON *neuron_vector;
	uchar *image_pixel;
	int x, y, w, h, xo, yo, i;
	int r, g, b;
	NEURON_OUTPUT max, min, value;

	w = image->width;
	h = image->height;

	if (neuron_layer_roi)
	{
		xo = neuron_layer_roi->x;
		yo = neuron_layer_roi->y;
	}
	else
	{
		xo = 0;
		yo = 0;
	}

	neuron_vector = neuron_layer->neuron_vector;
	if (neuron_layer->output_type == GREYSCALE_FLOAT)
	{
		max = min = neuron_vector[0].output;

		for (i = 0; i < w * h; i++)
		{
			value = neuron_vector[i].output;

			if (value.fval > max.fval)
				max = value;
			if (value.fval < min.fval)
				min = value;
		}
	}

	for (y = h-1; y >= 0; y--)
	{
		for (x = 0; x < w; x++)
		{
			neuron_output = neuron_vector[(xo+x) + (yo+y)*neuron_layer->dimentions.x].output;
			image_pixel = (uchar *) (image->imageData + (h-1-y) * image->widthStep);

			switch (neuron_layer->output_type)
			{
				case COLOR:
					r = (double) RED (neuron_output.ival);
					g = (double) GREEN (neuron_output.ival);
					b = (double) BLUE (neuron_output.ival);
					image_pixel[3*x+0] = b;
					image_pixel[3*x+1] = g;
					image_pixel[3*x+2] = r;
					break;
				case GREYSCALE:
					image_pixel[x] = neuron_output.ival;
					break;
				case GREYSCALE_FLOAT:
					image_pixel[x] = (uchar) (255.0f * (neuron_output.fval - min.fval) / (max.fval - min.fval));
					break;
				default:
					fprintf (stderr, "(copy_neuron_layer_to_image) Error: invalid output type: %d.\n", neuron_layer->output_type);
					exit (1);
					break;
			}
		}
	}
}
