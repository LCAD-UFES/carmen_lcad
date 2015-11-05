#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <omp.h>
#include <string.h>



#ifndef VG_RAM_H
#define VG_RAM_H

#define 	RED(pixel)	((pixel & 0x000000ffL) >> 0)
#define 	GREEN(pixel)	((pixel & 0x0000ff00L) >> 8)
#define 	BLUE(pixel)	((pixel & 0x00ff0000L) >> 16)
#define 	PIXEL(r, g, b)  (((r & 0x000000ffL) << 0) | ((g & 0x000000ffL) << 8) | ((b & 0x000000ffL) << 16))

#define 	pi 3.1415926535897932384626433832795029

#define 	GAUSSIAN(x, mean, sigma) ((1.0/(sigma*sqrt(2.0*pi)))*exp(-0.5*(pow(x-mean,2)/sigma*sigma)))


#define 	LRAND48_MAX	((unsigned int) -1 >> 1)

#define 	PATTERN_UNIT_SIZE 64
#define   BLOCK_SIZE 256


typedef struct synapsesDistribution{
	float *grx;
	float *gry;
	int numSynapses;
}synapsesDistribution;

typedef struct vg_ram_wnn{

	unsigned  int *pattern;
	int patternSize;
	int numNeurons;
	int numSynapses;
	int numNeuronsLayer;

}vg_ram_wnn;

typedef struct bi_dimension_vg_ram_wnn{

	unsigned  int *pattern;
	int pattern_size;
	int number_of_layer_width;
	int number_of_layer_height;
	int number_of_synapses;
	int number_of_neurons;

}bi_dimention_vg_ram_wnn;

typedef struct DisparityMap
{
	float *disps;
	int width;
	int height;
	int numMaps;

}DisparityMap;

// typedef struct synapsesCoordinatyOnImage{
// 	int *x;
// 	int *y;
// 	int numSynapses
// }synapsesCoordinaty;

typedef struct Image{
	unsigned char *data;
	int channels;
	int width,height;
}Image;

#ifdef __cplusplus
extern "C" {
#endif

//  functions for Winner-Takes-it-All filter
float *CENTRAL(int plane, int w, int h,float *p);
float *UP(int w, float *p);
float *UP_RIGHT(int w, float *p);
float *RIGHT(int w, float *p);
float *LOW_RIGHT(int w, float *p);
float *LOW(int w, float *p);
float *LOW_LEFT(int w, float *p);
float *LEFT(int w, float *p);
float *UP_LEFT(int w, float *p);


float gaussrand();

void CalcGuassianSynapsesDistribution(synapsesDistribution neuronSynapses);

void CalcNeuronSynapsesCoordinateOnImage(int center_x, int center_y,synapsesDistribution neuronSynapses,
		unsigned short *Internal_NeuronSynapsesCoordinate_X, unsigned short *Internal_NeuronSynapsesCoordinate_Y,
						int ImageWidth, int ImageHeight);

void CalcBitPatternMinchintonCell(Image img, unsigned int *gray_image,unsigned short *Internal_NeuronSynapsesCoordinate_X,unsigned short *Internal_NeuronSynapsesCoordinate_Y, unsigned  int *bitPattern, int numSynapses, int patternSize, int center_x, int center_y);

void WNNTrain(Image Imgleft,Image ImgRight, int MAX_Diparity, vg_ram_wnn stereo_wnn, unsigned short *Internal_NeuronSynapsesCoordinate_X, unsigned short *Internal_NeuronSynapsesCoordinate_Y,int image_line);

void BackwardsWNNTrain(Image Imgleft,Image ImgRight, int MAX_Diparity, vg_ram_wnn stereo_wnn, short *Internal_NeuronSynapsesCoordinate_X, short *Internal_NeuronSynapsesCoordinate_Y,int image_line);

int bitcount(unsigned int n);

float* get_wnn_confidance(int width, int height);

void scaline_opmization(Image left, Image right, float *agreCost, float *agreCost_out, int max_disparity, float Pi1, float Pi2, int talSO);


void WNN_initialize_mult_neuron_stereo(int height, int width, int MAX_Diparity,int numMaps,int numOfThreads, int numSynapses, float gaussian_radius_param, int horizontal_ROI_ini, int horizontal_ROI_end, int number_of_neuron);

unsigned int HammingDistance(unsigned  int *bit_pattern1, unsigned  int *bit_pattern2, int patternSize);

float *FindNearestPatterns(unsigned  int *bitPattern, unsigned  int *neuronPattern,float *agreCost, float *disps, int MAX_Diparity, int patternSize, int x, int y);

float* BackwardsFindNearestPatterns(unsigned  int *bitPattern, unsigned  int *neuronPattern,float *agreCost, float *disps, int MAX_Diparity, int patternSize, int x, int y);

float SelectDisparity(DisparityMap DispMap,float *HamDist, float *disps, int MAX_Disparity, int pixels, int line);

void SelectDisparity2(DisparityMap DispMap,unsigned short *HamDist, unsigned short *disps, int MAX_Disparity, int pixels, int line);

void allocVgRamWNN(int patternSize, int numNeurons, int numSynapses, int numNeuronsLayer, vg_ram_wnn *stereo_wnn_out);

void freeVgRamWNN(vg_ram_wnn *stereo_wnn_out);

void WNNDispatyLine(Image ImgLeft,Image ImgRight, DisparityMap DispMap, int MAX_Disparity, vg_ram_wnn stereo_wnn, unsigned short *Internal_NeuronSynapsesCoordinate_X, unsigned short *Internal_NeuronSynapsesCoordinate_Y,unsigned  int *bitPattern,int image_line,float *agreCost, float *disps);

void BackwardsWNNDispatyLine(Image ImgLeft,Image ImgRight, DisparityMap DispMap, int MAX_Disparity, vg_ram_wnn stereo_wnn, short *Internal_NeuronSynapsesCoordinate_X, short *Internal_NeuronSynapsesCoordinate_Y,unsigned  int *bitPattern,int image_line,float *agreCost, unsigned short *disps);

void allocSynapsesDistribution(synapsesDistribution *neuronSynapses, int numSynapses);

void freeSynapsesDistribution(synapsesDistribution *neuronSynapses);

void SetSynapsesCoodinate(int width, int height, int numSynapses);

void set_scan_image_param(int tal1_in, int tal2_in, int L1_in, int L2_in);

//Initialization of VG-RAM WNN variables

void set_image_filtered(unsigned char *left, unsigned char *right, int width, int height, int nchannels);

void WNNDispaty(Image ImgLeft, Image ImgRight, DisparityMap DispMap, int MAX_Diparity, int numSynapses, int numberOfThreads);

void WNNTerminate();

void WinnerTakesItAll(DisparityMap disps, int iteration);
void WinnerTakesItAll2(DisparityMap disps, int iteration);

int find_nearest_color(Image img, int *scanline_out, int *scancolumn_out, int tal1, int tal2, int L1, int L2);

void mean_filter(DisparityMap disps, int iteration);

void median_filter(DisparityMap disps, int iteration);

void allocImage(Image *img,int width, int height, int numChannels);

void allocDisparityMap(DisparityMap *disp,int w, int h, int numOfDisparity );


void WNNInitialize(int height, int width, int MAX_Diparity,int numMaps,int numOfThreads, int numSynapses, float gaussian_radius_param, int horizontal_ROI_ini, int horizontal_ROI_end, double lambda);

void disparityVgRamWNN(unsigned char*image_left,unsigned char*image_right, int stereo_height, int stereo_width, int stereo_disparity, float *disparity , int numSynapses,int numOfThreads, int iteration, int inc_width, int inc_height);

void mean(DisparityMap disps, int iteration);

void vertical_color_limits(Image img, unsigned short *up_limit_out, unsigned short *down_limit_out, int tal1, int tal2, int L1, int L2);

void horizontal_color_limits(Image img, unsigned short *left_limit_out, unsigned short *right_limit_out, int tal1, int tal2, int L1, int L2);

void
sum_horizontal_vertical(float *agreCost, float *agreCost_out, int width, int height, int L, int max_disparity,
		unsigned short *up_limit, unsigned short *down_limit,
		unsigned short *left_limit, unsigned short *right_limit);

void
sum_vertical_horizontal(float *agreCost, float *agreCost_out, int width, int height, int L, int max_disparity,
		unsigned short *up_limit, unsigned short *down_limit,
		unsigned short *left_limit, unsigned short *right_limit);

void
build_synapses_cordinates(int width, int height, int numSynapses, float gaussian_radius_param);

#ifdef __cplusplus
}
#endif


#endif



