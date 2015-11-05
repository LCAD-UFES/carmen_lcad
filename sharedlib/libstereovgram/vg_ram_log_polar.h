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


#define 	pi           	3.1415926535897932384626433832795029

#define 	GAUSSIAN(x, mean, sigma) ((1/(sigma*sqrt(2*pi)))*exp(-0.5*pow(x-mean,2)/sigma*sigma))


#define 	LRAND48_MAX	((unsigned int) -1 >> 1)

#define 	PATTERN_UNIT_SIZE 32



typedef struct synapsesDistribution{
	float *grx;
	float *gry;
	int numSynapses;
}synapsesDistribution;

typedef struct vg_ram_wnn{

	unsigned int *pattern;
	int patternSize;
	int numNeurons;
	int numSynapses;
	int numNeuronsLayer;

}vg_ram_wnn;

typedef struct DisparityMap
{
	unsigned char *disps;
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

// Inline functions for Winner-Takes-it-All filter
inline unsigned char * CENTRAL(int plane, int w, int h,unsigned char* p);
inline unsigned char * UP(int w, unsigned char* p);
inline unsigned char * UP_RIGHT(int w, unsigned char* p);
inline unsigned char * RIGHT(int w, unsigned char* p);
inline unsigned char * LOW_RIGHT(int w, unsigned char* p);
inline unsigned char * LOW(int w, unsigned char* p);
inline unsigned char * LOW_LEFT(int w, unsigned char* p);
inline unsigned char * LEFT(int w, unsigned char* p);
inline unsigned char * UP_LEFT(int w, unsigned char* p);

inline double DistanceFromImageCenter(int wi, int hi, int w, int h, int u, double log_factor);

void LogPolarMapping(int *xi, int *yi, int wi, int hi, int u, int v, int w, int h, int x_center, int y_center, double correction, double log_factor);

void DisparityMapInverseLogPolar(DisparityMap disp);

float gaussrand();

inline void CalcGuassianSynapsesDistribution(synapsesDistribution neuronSynapses);

inline void CalcNeuronSynapsesCoordinateOnImage(int center_x, int center_y,synapsesDistribution neuronSynapses,
						int *Internal_NeuronSynapsesCoordinate_X, int *Internal_NeuronSynapsesCoordinate_Y,
						int ImageWidth, int ImageHeight);

void CalcBitPatternMinchintonCell(Image img, unsigned int *gray_image,int *Internal_NeuronSynapsesCoordinate_X,int *Internal_NeuronSynapsesCoordinate_Y, unsigned int *bitPattern, int numSynapses, int patternSize);

void WNNTrain(Image Imgleft,Image ImgRight, int MAX_Diparity, vg_ram_wnn stereo_wnn, int *Internal_NeuronSynapsesCoordinate_X, int *Internal_NeuronSynapsesCoordinate_Y,int image_line);

void BackwardsWNNTrain(Image Imgleft,Image ImgRight, int MAX_Diparity, vg_ram_wnn stereo_wnn, int *Internal_NeuronSynapsesCoordinate_X, int *Internal_NeuronSynapsesCoordinate_Y,int image_line);

int bitcount(unsigned int n);

unsigned int HammingDistance(unsigned int *bit_pattern1, unsigned int *bit_pattern2, int patternSize);

unsigned int *FindNearestPatterns(unsigned int *bitPattern, unsigned int *neuronPattern,unsigned int *HamDist, unsigned short *disps, int MAX_Diparity, int patternSize);

unsigned int* BackwardsFindNearestPatterns(unsigned int *bitPattern, unsigned int *neuronPattern,unsigned int *HamDist, unsigned short *disps, int MAX_Diparity, int patternSize);

void SelectDisparity(DisparityMap DispMap,unsigned int *HamDist, unsigned short *disps, int MAX_Disparity, int pixels, int line);

void SelectDisparity2(DisparityMap DispMap,unsigned int *HamDist, unsigned short *disps, int MAX_Disparity, int pixels, int line);

void allocVgRamWNN(int patternSize, int numNeurons, int numSynapses, int numNeuronsLayer, vg_ram_wnn *stereo_wnn_out);

void freeVgRamWNN(vg_ram_wnn *stereo_wnn_out);

void WNNDispatyLine(Image ImgLeft,Image ImgRight, DisparityMap DispMap, int MAX_Disparity, vg_ram_wnn stereo_wnn, int *Internal_NeuronSynapsesCoordinate_X, int *Internal_NeuronSynapsesCoordinate_Y,unsigned int *bitPattern,int image_line,unsigned int *HamDist, unsigned short *disps);

void BackwardsWNNDispatyLine(Image ImgLeft,Image ImgRight, DisparityMap DispMap, int MAX_Disparity, vg_ram_wnn stereo_wnn, int *Internal_NeuronSynapsesCoordinate_X, int *Internal_NeuronSynapsesCoordinate_Y,unsigned int *bitPattern,int image_line,unsigned int *HamDist, unsigned short *disps);

void allocSynapsesDistribution(synapsesDistribution *neuronSynapses, int numSynapses);

void freeSynapsesDistribution(synapsesDistribution *neuronSynapses);

void SetSynapsesCoodinate(int width, int height, int numSynapses);

//Initialization of VG-RAM WNN variables


void WNNDispaty(Image ImgLeft, Image ImgRight, DisparityMap DispMap, int MAX_Diparity, int numSynapses, int numberOfThreads);

void WNNTerminate();

void WinnerTakesItAll(DisparityMap disps, int iteration);

void mean_filter(DisparityMap disps, int iteration);

void median_filter(DisparityMap disps, int iteration);

void allocImage(Image *img,int width, int height, int numChannels);

void allocDisparityMap(DisparityMap *disp,int w, int h, int numOfDisparity );


void WNNInitialize(int height, int width, int MAX_Diparity,int numMaps,int numOfThreads, int numSynapses, float gaussian_radius_param, int horizontal_ROI_ini, int horizontal_ROI_end);

void disparityVgRamWNN(unsigned char*image_left,unsigned char*image_right, int stereo_height, int stereo_width, int stereo_disparity, float *disparity , int numSynapses,int numOfThreads, int iteration, int inc_width, int inc_height);

#ifdef __cplusplus
}
#endif

#endif



