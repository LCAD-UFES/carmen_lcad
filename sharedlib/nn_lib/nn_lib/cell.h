/*
 * cell.h
 *
 *  Created on: 19/10/2012
 *      Author: _filipe
 */

#ifndef _NN_LIB_CELL_H_
#define _NN_LIB_CELL_H_

#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include "util.h"

using namespace std;
using namespace cv;
using namespace nn_lib::util;

#define LRAND48_MAX ((unsigned int) -1 >> 1)

namespace nn_lib
{
	namespace cell
	{
		// TODO: pensar em como fazer essa classe ser n-dimensional de forma que se um usuario quiser aplicar uma michinton cell em um vetor, por exemplo, seja possivel
		class MinchintonCell
		{
			int image_width, image_height;
			Point2D attention_point;
			vector<Point2D> distribution_around_attention_point;


			float
			next_random_gauss_radius()
			{
				static double V1, V2, S;
				static int phase = 0;
				double X;

				if(phase == 0)
				{
					do
					{
						float U1 = (float) rand() / (float) LRAND48_MAX;
						float U2 = (float) rand() / (float) LRAND48_MAX;

						V1 = 2 * U1 - 1;
						V2 = 2 * U2 - 1;
						S = V1 * V1 + V2 * V2;

					}while(S >= 1 || S == 0);

					X = V1 * sqrt(-2 * log(S) / S);
				}
				else
					X = V2 * sqrt(-2 * log(S) / S);

				phase = 1 - phase;
				return X;
			}


			void
			create_gaussian_distribution(int radius, int num_synapsis)
			{
				int i;

				for(i = 0; i < num_synapsis; i++)
				{
					Point2D point;

					point.x = (int) (next_random_gauss_radius() * radius + 0.5);
					point.y = (int) (next_random_gauss_radius() * radius + 0.5);

					distribution_around_attention_point.push_back(point);
				}

			}


			int
			point_is_vertically_out_of_image(int point)
			{
				if ((point < 0) || (point >= image_height))
					return 1;
				else
					return 0;
			}


			int
			point_is_horizontally_out_of_image(int point)
			{
				if ((point < 0) || (point >= image_width))
					return 1;
				else
					return 0;
			}

			public:

				MinchintonCell()
				{
				}


				~MinchintonCell()
				{
				}


				void
				set_attention_point_coordinates(int x, int y)
				{
					attention_point.x = x;
					attention_point.y = y;
				}


				void
				set_image_size (int height, int width)
				{
					image_height = height;
					image_width = width;
				}


				// TODO: fazer uma funcao dessa com raio em x e em y
				// TODO: fazer uma funcao dessa para gerar outros tipos de distribuicao (circular, quadrada, etc.)
				// TODO: pensar em como possibilitar ao usuario usar a mesma distribuicao gaussiana para todos os neuronios
				void
				generate_gaussian_distribution(int radius, int num_points)
				{
					create_gaussian_distribution(radius, num_points);
				}

				Point2D
				calculate_point_position_from_gaussian_distribution(Point2D gaussian_point)
				{
					Point2D point = gaussian_point;
					int hout, vout;

					hout = point_is_horizontally_out_of_image(attention_point.x + point.x);
					vout = point_is_vertically_out_of_image(attention_point.y + point.y);

					if (!hout)
						point.x = attention_point.x + point.x;
					else
						point.x = attention_point.x - point.x;

					if (point.x < 0)
						point.x = 0;
					if (point.x >= image_width)
						point.x = image_width - 1;

					if (!vout)
						point.y = attention_point.y + point.y;
					else
						point.y = attention_point.y - point.y;

					if (point.y < 0)
						point.y = 0;
					if (point.y >= image_height)
						point.y = image_height - 1;

					return point;
				}

				vector<float>
				filter(IplImage *img) // TODO: pensar em um nome melhor para essa funcao
				{
					Point2D point, next_point;
					vector<float> v; // TODO: pensar em um nome melhor para esse vetor
					int attention_point_image_position, distribution_point_image_position, next_distribution_point_image_position;

					if (img->nChannels > 1)
						printf("Warning: image has more than one channel! Only the first one will be used\n");

					attention_point_image_position = img->nChannels * (attention_point.y * img->width + attention_point.x);

					for(unsigned int i = 0; i < (distribution_around_attention_point.size() - 1); i++)
					{
						point = calculate_point_position_from_gaussian_distribution(distribution_around_attention_point[i]);
						next_point = calculate_point_position_from_gaussian_distribution(distribution_around_attention_point[i + 1]);

						distribution_point_image_position = img->nChannels * (point.y * img->width + point.x);
						next_distribution_point_image_position = img->nChannels * (next_point.y * img->width + next_point.x);

						if (distribution_point_image_position < 0 || distribution_point_image_position > img->imageSize)
						{
							printf("attention: %d %d gaussian: %d e %d => point %d * (%d * %d + %d) = %d height: %d\n", attention_point.x, attention_point.y, distribution_around_attention_point[i].x, distribution_around_attention_point[i].y, img->nChannels, point.y, img->width, point.x, distribution_point_image_position, img->height);
							printf("exit\n");
							exit(-1);
						}

						if (img->imageData[attention_point_image_position] < img->imageData[distribution_point_image_position])
						//if (img->imageData[distribution_point_image_position] < img->imageData[next_distribution_point_image_position])
							v.push_back(0.0);
						else
							v.push_back(1.0);
					}

					return v;
				}


				void
				draw(IplImage *img)
				{
					Point2D point;

					for(unsigned int i = 0; i < distribution_around_attention_point.size(); i++)
					{
						point = calculate_point_position_from_gaussian_distribution(distribution_around_attention_point[i]);
						cvCircle(img, cvPoint(point.x, point.y), 2, cvScalar (0, 0, 255, 0), 1, 0, 0);
					}
				}
		};
	}
}


#endif /* _NN_LIB_CELL_H_ */
