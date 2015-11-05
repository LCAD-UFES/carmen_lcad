#include <stdio.h>
#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "neuron.h"
#include "cell.h"

using namespace std;
using namespace nn_lib::neuron;
using namespace nn_lib::cell;


int max_disparity = 16;


void
create_neuron_layer(int height, int width, vector<vector<NeuronVGRAM<int> > > &neuron_layer_over_image, vector<vector<MinchintonCell> > &neuron_layer_cells)
{
	int i, j;
	
	for(i = 0; i < height; i++)
	{
		neuron_layer_over_image.push_back(vector<NeuronVGRAM<int> > ());
		neuron_layer_cells.push_back(vector<MinchintonCell> ());
		
		for(j = 0; j < width; j++)
		{
			neuron_layer_over_image[i].push_back(NeuronVGRAM<int> ());
			neuron_layer_cells[i].push_back(MinchintonCell ());
			
			neuron_layer_cells[i][j].set_attention_point_coordinates(i, j);
			neuron_layer_cells[i][j].set_image_size(height, width);
			neuron_layer_cells[i][j].generate_gaussian_distribution(10, 1024);
		}
	}
}


void
shift_image_to_right(IplImage *img)
{
	int i, j;
	
	// shift the image to right
	for(j = (img->width - 1); j > 0; j--)
	{
		for(i = 0; i < img->height; i++)
		{
			img->imageData[i * img->width + j] = img->imageData[(i - 1) * img->width + j];
		}
	}
	
	// clean the first column
	for(i = 0; i < img->height; i++)
		img->imageData[i * img->width] = 0;
	
}


void
train(IplImage *left, vector<vector<NeuronVGRAM<int> > > &neuron_layer_over_image, vector<vector<MinchintonCell> > &neuron_layer_cells)
{
	int i, j, disparity;

	for(disparity = 0; disparity < max_disparity; disparity++)
	{
		for(i = 0; i < left->height; i++)
		{
			for(j = 0; j < left->width; j++)
			{
				// fazer uma conta para saber a posicao relativa do pixels apos o deslocamento
				vector<float> synaptic_gaussian_distribution = neuron_layer_cells[i][j].filter(left);
				neuron_layer_over_image[i][j].train(synaptic_gaussian_distribution, disparity);
			}
		}
		
		shift_image_to_right(left);
	}
}


void
test(IplImage *right, IplImage *output, vector<vector<NeuronVGRAM<int> > > &neuron_layer_over_image, vector<vector<MinchintonCell> > &neuron_layer_cells)
{
	int i, j, pixel_color, disparity;
	
	for(i = 0; i < right->height; i++)
	{
		for(j = 0; j < right->width; j++)
		{
			vector<float> synaptic_gaussian_distribution = neuron_layer_cells[i][j].filter(right);
			disparity = neuron_layer_over_image[i][j].test(synaptic_gaussian_distribution);
			
			pixel_color = 255 * (disparity / max_disparity);
			output->imageData[i * output->width + j] = pixel_color;
		}
	}
}


void
perform_stereo(IplImage *left, IplImage *right, IplImage *stereo)
{
	int i, j, width, height, disparity, disparity_found, max_disparity_found = 0;
	
	width = left->width;
	height = left->height;

	for(i = 0; i < height; i++)
	{
		printf("processing line %d\n", i);
		
		for(j = 0; j < width; j++)
		{
			NeuronVGRAM<int> neuron;
			MinchintonCell cell;
			vector<float> synaptic_gaussian_distribution;

			cell.set_attention_point_coordinates(j, i);
			cell.set_image_size(height, width);
			cell.generate_gaussian_distribution(10, 500);
			
			//train the neuron with different disparities
			for(disparity = 0; disparity < max_disparity; disparity++)
			{
				if ((j + disparity) < width)
				{
					// printf("j (%d) + disparity (%d) = %d / width: %d\n", j, disparity, j + disparity, width);
					cell.set_attention_point_coordinates(j + disparity, i);
					synaptic_gaussian_distribution = cell.filter(left);
					neuron.train(synaptic_gaussian_distribution, disparity);
				}
			}
			
			cell.set_attention_point_coordinates(j, i);
			synaptic_gaussian_distribution = cell.filter(right);
			disparity_found = neuron.test(synaptic_gaussian_distribution);
			
			stereo->imageData[i * stereo->width + j] = disparity_found;
			
			if (disparity_found > max_disparity_found)
				max_disparity_found = disparity_found;
		}
	}
	
	printf("Max disparity found: %d\n", max_disparity_found);
	
	// normalize image disparities to 255
	for(i = 0; i < height; i++)
	{
		for(j = 0; j < width; j++)
		{
			stereo->imageData[i * width + j] = (int) ((double) 255 * ((double)stereo->imageData[i * width + j] / (double)max_disparity_found));
		}
	}
}


int
main(int argc, char ** argv)
{
	IplImage *left, *right, *stereo;
	vector<vector<MinchintonCell> > neuron_layer_cells;
	vector<vector<NeuronVGRAM<int> > > neuron_layer_over_image;
	
	if (argc < 4)
		exit(printf("Use %s <left-img> <right-img> <output-image>\n", argv[0]));

	printf("Loading images...\n");	
	left = cvLoadImage(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
	right = cvLoadImage(argv[2], CV_LOAD_IMAGE_GRAYSCALE);
	
	if (left == NULL || right == NULL)
		exit(printf("input images left: '%s' right: '%s' not found\n", argv[1], argv[2]));

	stereo = cvCreateImage(cvSize(left->width, left->height), left->depth, left->nChannels);

	//
	// TODO: verificar por que o codigo abaixo aloca tanta memoria
	//
	// printf("Initializing Neuron Layer...\n");
	// create_neuron_layer(left->height, left->width, neuron_layer_over_image, neuron_layer_cells);
	//
	//printf("Training Neuron Layer...\n");
	//train(left, neuron_layer_over_image, neuron_layer_cells);
	//
	//printf("Testing Neuron Layer...\n");
	//test(right, stereo, neuron_layer_over_image, neuron_layer_cells);
	//
	
	perform_stereo(left, right, stereo);

	cvSaveImage(argv[3], stereo, NULL);
	return 0;
}
