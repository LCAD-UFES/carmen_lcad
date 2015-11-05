/*
 * util.h
 *
 *  Created on: 19/10/2012
 *      Author: filipe
 */

#ifndef _NN_LIB_UTIL_H_
#define _NN_LIB_UTIL_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>

namespace nn_lib
{
	namespace util
	{
		typedef struct
		{
			int x;
			int y;
		}Point2D;

		// TODO: Pensar se esse eh o melhor lugar para esse classe
		class LogPolarReceptor
		{
			int SERENO_MODEL;
			int C1;
			double pi;
			double LOG_POLAR_SCALE_FACTOR;
			float y0_val;


			double
			distance_from_image_center (int wi, int hi, int w, int h, int u, double log_factor)
			{
				double exp_val, x, y;

				if (((int) log_factor) == SERENO_MODEL)
				{
					x = 63.951256 * ((double) u /(double) (w/2));
					y = pow ((20.05 / (0.26 * (C1 - x))), (1.0 / 0.26)) - 0.08;
					y = 30.0 * (y - y0_val) / (30.0 - y0_val);
					exp_val = (double) (wi/2) * (y / 30.0);
				}
				else
				{
					x = ((double) u / (double) (w/2)) * log_factor;
					//		exp_val = (double) (wi/2) * (exp (( log (log_factor) * (x - log_factor)) / log_factor) - exp ( -log (log_factor))) * ( 1.0 / (1.0 - exp ( -log (log_factor))));
					exp_val = (double) (wi/2) * (exp (log (log_factor) * (x - log_factor) / log_factor) - (1.0/log_factor)) * (log_factor / (log_factor - 1.0));
				}

				return (exp_val);
			}


			double
			map_v1_to_image_angle (int u, int v, int w, int h)
			{
				if (u < w/2)
					return(pi * (((double) h * (3.0 / 2.0) - ((double) v )) / (double) h ) - pi / (double) (2*h));
				else
					return(pi * (((double) h * (3.0 / 2.0) + ((double) v )) / (double) h ) + pi / (double) (2*h));
			}


			void
			map_v1_to_image (int *xi, int *yi, int wi, int hi, int u, int v, int w, int h, int x_center, int y_center, double correction, double log_factor)
			{
				static int previous_u = -1;
				static double previous_d;
				double d, theta;

				correction = correction; //for keeping the compiler happy

				if (u < w/2)
				{
					if (u == previous_u)
						d = previous_d;
					else
						d = LOG_POLAR_SCALE_FACTOR * distance_from_image_center (wi, hi, w, h, (w-1)/2 - u, log_factor);

					//theta = pi * (((double) h * (3.0 / 2.0) - ((double) v * correction)) / (double) h) + LOG_POLAR_THETA_CORRECTION;
					theta = pi * (((double) h * (3.0 / 2.0) - ((double) v )) / (double) h ) - pi / (double) (2*h);
				}
				else
				{
					if (u == previous_u)
						d = previous_d;
					else
						d = LOG_POLAR_SCALE_FACTOR * distance_from_image_center (wi, hi, w, h, u - w/2, log_factor);

					//theta = pi * (((double) h * (3.0 / 2.0) + ((double) v * correction)) / (double) h) + LOG_POLAR_THETA_CORRECTION;
					theta = pi * (((double) h * (3.0 / 2.0) + ((double) v )) / (double) h ) + pi / (double) (2*h);

				}

				*xi = (int) (d * cos(theta) + 0.5) + x_center;
				*yi = (int) (d * sin(theta) + 0.5) + y_center;

				previous_u = u;
				previous_d = d;
			}



			public:

				LogPolarReceptor()
				{
					this->SERENO_MODEL = -1.0;
					this->C1 = 95.77775;
					this->pi = 3.1415926535897932384626433832795029;
					this->LOG_POLAR_SCALE_FACTOR = 1.0;
					this->y0_val = pow ((20.05/(0.26 * C1)), (1.0/0.26)) - 0.08;
				}


				~LogPolarReceptor()
				{
				}



				void
				transform(IplImage *img, IplImage *output_img, int x, int y)
				{
					int i, j, i_polar, j_polar, previous_i_polar, previous_j_polar, p, p_polar;
					unsigned char previous_output[3];

					previous_j_polar = -1;
					previous_i_polar = -1;

					previous_output[0] = 0;
					previous_output[1] = 0;
					previous_output[2] = 0;

					for(i = 0; i < img->height; i++)
					{
						for(j = 0; j < img->width; j++)
						{
							map_v1_to_image(&j_polar, &i_polar, img->width, img->height, j, i, img->width, img->height, x, y, (double) img->height / (double) img->height - 1, 2.0);

							p = 3 * (i * img->width + j);
							p_polar = 3 * (i_polar * img->width + j_polar);

							if ((j_polar == previous_j_polar) && (i_polar == previous_i_polar))
							{
								//filter_desc->output->neuron_vector[(v * w) + u].output = previous_output;

								output_img->imageData[p + 0] = previous_output[0];
								output_img->imageData[p + 1] = previous_output[1];
								output_img->imageData[p + 2] = previous_output[2];
							}
							else
							{
								if (j_polar >= img->width || j_polar < 0 || i_polar >= img->height || i_polar < 0)
								{
									// previous_output = filter_desc->output->neuron_vector[(v * w) + u].output.fval = 0;

									output_img->imageData[p + 0] = 0;
									output_img->imageData[p + 1] = 0;
									output_img->imageData[p + 2] = 0;

									previous_output[0] = 0;
									previous_output[1] = 0;
									previous_output[2] = 0;

								}
								else
								{
									// previous_output = filter_desc->output->neuron_vector[(v * w) + u].output = n_l->neuron_vector[yi * wi + xi].output;

									output_img->imageData[p + 0] = img->imageData[p_polar + 0];
									output_img->imageData[p + 1] = img->imageData[p_polar + 1];
									output_img->imageData[p + 2] = img->imageData[p_polar + 2];

									previous_output[0] = img->imageData[p_polar + 0];
									previous_output[1] = img->imageData[p_polar + 1];
									previous_output[2] = img->imageData[p_polar + 2];
								}
							}

							previous_j_polar = j_polar;
							previous_i_polar = i_polar;
						}
					}
				}
		};
	}
}

#endif /* _NN_LIB_UTIL_H_ */
