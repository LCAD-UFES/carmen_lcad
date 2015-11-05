#ifndef NO_CUDA
#include "vg_ram_GPU.h"
#else
#include "vg_ram.h"
#endif


#define   ABS(x) (x < 0 ? -x : x)
#define   MAX(x,y) (x > y ? x : y)

//VG-RAM WNN Global Variables
unsigned short 	*NeuronSynapsesCoordinate_X = NULL;
unsigned short 	*NeuronSynapsesCoordinate_Y = NULL;
unsigned short 	*NeuronSynapsesCoordinate_X_GPU = NULL;
unsigned short 	*NeuronSynapsesCoordinate_Y_GPU = NULL;

double HAMMING_DISTANCE_LAMBDA;

unsigned short 	*up_limit = NULL;
unsigned short 	*down_limit = NULL;
unsigned short 	*right_limit = NULL;
unsigned short 	*left_limit = NULL;

Image ImgLeft,ImgRight;

int	patternSize;
unsigned  int	**bitPattern=NULL;
float	*agreCost = NULL;
float	*agreCost_new = NULL;

unsigned short	*HamDist_GPU = NULL;
unsigned short	*disps_GPU = NULL;

float	*disps = NULL;
float	*confidance = NULL;

unsigned short *neuron_result = NULL;
unsigned short *neurons_out = NULL;
int *h_distance = NULL;
vg_ram_wnn		*stereo_wnn = NULL;
bi_dimention_vg_ram_wnn wnn_left_neuron;
bi_dimention_vg_ram_wnn wnn_right_neuron;

bi_dimention_vg_ram_wnn wnn_left_neuron_GPU;
bi_dimention_vg_ram_wnn wnn_right_neuron_GPU;

float gaussian_radius;
DisparityMap disp;

DisparityMap disp_GPU;

int inc_stereo_height;
int inc_stereo_width;

int stereo_horizontal_ROI_ini;
int stereo_horizontal_ROI_end;

int stereo_vertical_ROI_ini;

unsigned int *gray_image_left = NULL;
int gray_size_offset;
unsigned int *gray_image_right = NULL;

int tal1 = 3;
int tal2 = 0;
int L1 = 34;
int L2 = 0;


float *pixels_targets = NULL;

Image img_left;
Image img_right;

Image img_left_filtered;
Image img_right_filtered;

unsigned int *gray_image_left_GPU = NULL;
unsigned int *gray_image_right_GPU = NULL;

synapsesDistribution neuronSynapses_GPU;

//  functions for Winner-Takes-it-All filter
float *CENTRAL(int plane, int w, int h,float *p) { return p + (w*h*plane); }
float *UP(int w, float *p) { return (p - w * inc_stereo_height); }
float *UP_RIGHT(int w, float *p) {return (p - w * inc_stereo_height + inc_stereo_width); }
float *RIGHT(int w, float *p) { return( p + inc_stereo_width); }
float *LOW_RIGHT(int w, float *p) { return( p + w * inc_stereo_height + inc_stereo_width); }
float *LOW(int w, float *p) { return( p + w * inc_stereo_height);  }
float *LOW_LEFT(int w, float *p) { return( p + w * inc_stereo_height - inc_stereo_width); }
float *LEFT(int w, float *p) { return( p - inc_stereo_width ); }
float *UP_LEFT(int w, float *p) { return( p - w * inc_stereo_height - inc_stereo_width); }


void set_image_filtered(unsigned char *left, unsigned char *right, int width, int height, int nchannels)
{
	img_left_filtered.data = left;
	img_right_filtered.data = right;

	img_right_filtered.width = width;
	img_right_filtered.height = height;
	img_right_filtered.channels = nchannels;

	img_left_filtered.width = width;
	img_left_filtered.height = height;
	img_right_filtered.channels = nchannels;
}


float
gaussrand()
{
	static double V1, V2, S;
	static int phase = 0;
	double X;

	if(phase == 0) {
		do {
			float U1 = (float) rand() / (float) LRAND48_MAX;
			float U2 = (float) rand() / (float) LRAND48_MAX;

			V1 = 2 * U1 - 1;
			V2 = 2 * U2 - 1;
			S = V1 * V1 + V2 * V2;
		}while(S >= 1 || S == 0);

		X = V1 * sqrt(-2 * log(S) / S);
	} else
		X = V2 * sqrt(-2 * log(S) / S);

	phase = 1 - phase;

	return X;
}

void CalcGuassianSynapsesDistribution(synapsesDistribution neuronSynapses)
{
	int i;
	int N=neuronSynapses.numSynapses;
	for(i=0;i<N;i++)
	{
		neuronSynapses.grx[i] = gaussrand () * gaussian_radius + 0.5;
		neuronSynapses.gry[i] = gaussrand () * gaussian_radius + 0.5;
	}

}

void CalcNeuronSynapsesCoordinateOnImage(int center_x, int center_y,synapsesDistribution neuronSynapses,
		unsigned short *Internal_NeuronSynapsesCoordinate_X, unsigned short *Internal_NeuronSynapsesCoordinate_Y,
		int ImageWidth, int ImageHeight)
{
	int synapse=0;
	int x,y;
	int N=neuronSynapses.numSynapses;
	for(synapse=0;synapse<N;synapse++)
	{
		x = (int)((float)center_x+neuronSynapses.grx[synapse]);
		y = (int)((float)center_y+neuronSynapses.gry[synapse]);
		x = x > 0 ? x : ImageWidth + x;
		y = y > 0 ? y : ImageHeight + y;
		x = x < ImageWidth  ? x : x - ImageWidth;
		y = y < ImageHeight ? y : y - ImageHeight;
		Internal_NeuronSynapsesCoordinate_X[synapse] = x;
		Internal_NeuronSynapsesCoordinate_Y[synapse] = y;
	}
}

void
rgb_to_gray(Image img, unsigned int *gray)
{
	int i, y;
	int pos;
	int gray_x;
	int gray_y;

	#pragma omp parallel for default(none) private(y,gray_y,gray_x,pos,i) shared(gray,img,stereo_vertical_ROI_ini,gray_size_offset)
	for(y = 0; y < img.height; y++)
	{
		gray_y = y;
		for(i = 0, gray_x = 0 ; i < img.width; i++, gray_x++)
		{
			pos = 3 * (y * img.width  + i);


			gray[gray_y * gray_size_offset + gray_x] = 0.333f * img.data[pos] + 0.333f * img.data[pos + 1] + 0.333f * img.data[pos + 2];
			//gray[gray_y * gray_size_offset + gray_x] = PIXEL(img.data[pos], img.data[pos + 1], img.data[pos + 2]);

		}
	}
}

float
distance(unsigned char *pixel0, unsigned char *pixel1)
{
	float tr = 0.299f * (pixel0[0] - pixel1[0]);
	float tg = 0.587f * (pixel0[1] - pixel1[1]);
	float tb = 0.114f * (pixel0[2] - pixel1[2]);

	return tr + tg + tb;
}


void CalcBitPatternMinchintonCell(
		Image img,
		unsigned int *gray_image,
		unsigned short *Internal_NeuronSynapsesCoordinate_X,
		unsigned short *Internal_NeuronSynapsesCoordinate_Y,
		unsigned  int *bitPattern,
		int numSynapses,
		int patternSize,
		int center_x,
		int center_y)
{
	int synapse;
	unsigned int pixel0=0;
	unsigned int pixelxi;
	unsigned int pixelxi1;
	int N=numSynapses;
	int shift;
	int current_bit_pattern_group;

	int x = Internal_NeuronSynapsesCoordinate_X[0];
	int y = Internal_NeuronSynapsesCoordinate_Y[0];
//	unsigned char *p = &img.data[3 *(center_y * gray_size_offset + center_x)];
//	unsigned char *p1;

	memset(bitPattern,0,sizeof(unsigned  int)*patternSize);
	pixelxi = pixel0 = gray_image[center_y * gray_size_offset + center_x];

	for(synapse=0;synapse<N;synapse++)
	{
		shift=synapse % PATTERN_UNIT_SIZE;
		current_bit_pattern_group=synapse / PATTERN_UNIT_SIZE;

		x = Internal_NeuronSynapsesCoordinate_X[synapse + 1];
		y = Internal_NeuronSynapsesCoordinate_Y[synapse + 1];

		pixelxi1  = gray_image[y * gray_size_offset + x];
	//	p1 = &img.data[3 *(y * gray_size_offset + x)];

		bitPattern[current_bit_pattern_group] |= ((pixelxi > pixelxi1) ? 1 : 0) << shift;
		//bitPattern[current_bit_pattern_group] |= (distance(p,p1) > 0 ? 1 : 0) << shift;
		pixelxi = pixelxi1;
		//p = p1;
	}

	// Minchington: Última sinapse
	shift=synapse % PATTERN_UNIT_SIZE;
	current_bit_pattern_group=synapse / PATTERN_UNIT_SIZE;

	x = Internal_NeuronSynapsesCoordinate_X[synapse];
	y = Internal_NeuronSynapsesCoordinate_Y[synapse];

	pixelxi1 = gray_image[y * gray_size_offset + x];

	bitPattern[current_bit_pattern_group] |= ((pixelxi1 > pixel0) ? 1 : 0) << shift;
}

void WNNTrainBack(Image Imgleft,Image ImgRight, int MAX_Diparity, vg_ram_wnn stereo_wnn, unsigned short *Internal_NeuronSynapsesCoordinate_X,unsigned short *Internal_NeuronSynapsesCoordinate_Y,int image_line)
{
	int  neuron;	//Neuron counter must be treated as an integer value due to loop conditions
	unsigned  int *disparityBitPattern;
	int i = stereo_vertical_ROI_ini;

	for (neuron = stereo_horizontal_ROI_end - 1; neuron > stereo_horizontal_ROI_ini; neuron--)
	{
		i =  neuron;
		disparityBitPattern = &stereo_wnn.pattern[neuron*stereo_wnn.patternSize];
		CalcBitPatternMinchintonCell(ImgLeft, gray_image_left,(Internal_NeuronSynapsesCoordinate_X + i*stereo_wnn.numSynapses) ,(Internal_NeuronSynapsesCoordinate_Y + i*stereo_wnn.numSynapses) , disparityBitPattern, stereo_wnn.numSynapses,stereo_wnn.patternSize,i,image_line);
	}
}

void WNNTrain(Image Imgleft,Image ImgRight, int MAX_Diparity, vg_ram_wnn stereo_wnn, unsigned short *Internal_NeuronSynapsesCoordinate_X,unsigned short *Internal_NeuronSynapsesCoordinate_Y,int image_line)
{
	int  neuron;	//Neuron counter must be treated as an integer value due to loop conditions
	unsigned  int *disparityBitPattern;
	int i = stereo_vertical_ROI_ini;

	for (neuron = stereo_horizontal_ROI_ini; neuron < stereo_horizontal_ROI_end; neuron++)
	{
		i = stereo_vertical_ROI_ini + neuron;
		disparityBitPattern = &stereo_wnn.pattern[neuron*stereo_wnn.patternSize];
		CalcBitPatternMinchintonCell(ImgRight, gray_image_right,(Internal_NeuronSynapsesCoordinate_X + i*stereo_wnn.numSynapses) ,(Internal_NeuronSynapsesCoordinate_Y + i*stereo_wnn.numSynapses) , disparityBitPattern, stereo_wnn.numSynapses,stereo_wnn.patternSize,i,image_line);
	}
}


int
bitcount(unsigned int n)
{
	/* works for 32-bit numbers only    */
	/* fix last line for 64-bit numbers */

	register unsigned int tmp;

	tmp = n - ((n >> 1) & 033333333333)
	- ((n >> 2) & 011111111111);
	return ((tmp + (tmp >> 3)) & 030707070707) % 63;
}

unsigned int
HammingDistance(unsigned  int *bit_pattern1, unsigned  int *bit_pattern2, int patternSize)
{
	unsigned int i;
	unsigned  int bit_difference=0;
	unsigned int hamming_distance;

	hamming_distance = 0;
	for (i = 0; i < patternSize; i++)
	{
		bit_difference = bit_pattern1[i] ^ bit_pattern2[i];

		//hamming_distance += bitcount(bit_difference);
		hamming_distance += __builtin_popcount(bit_difference);
	}
	return hamming_distance;
}

float ro_func(float cost, float lambda)
{
	return exp(-cost / lambda);
	//return 1.0 - cost / lambda;
}

float
SAD(unsigned char *pixel0, unsigned char *pixel1)
{
	float tr = 0.299f * fabs(pixel0[0] - pixel1[0]);
	float tg = 0.587f * fabs(pixel0[1] - pixel1[1]);
	float tb = 0.114f * fabs(pixel0[2] - pixel1[2]);

	return tr + tg + tb / 3.0;
}

float*
FindNearestPatterns(unsigned  int *bitPattern, unsigned int *neuronPattern,float *agreCost, float *disps, int MAX_Diparity, int patternSize, int x, int y)
{
	int disparity;

	for (disparity = 0; disparity < MAX_Diparity; disparity++)
	{
		agreCost[disparity] = ro_func(HammingDistance(&neuronPattern[disparity*patternSize], bitPattern,patternSize), HAMMING_DISTANCE_LAMBDA);
		//agreCost[disparity] = ro_func(SAD(&ImgLeft.data[3 * (y * ImgLeft.width + x + disparity)], &ImgRight.data[3 * (y * ImgLeft.width + x)]), 10.0);
		disps[disparity] = disparity;
	}
	return agreCost;
}

int max;

float*
BackwardsFindNearestPatterns(unsigned  int *bitPattern, unsigned  int *neuronPattern,float *agreCost, float *disps, int MAX_Diparity, int patternSize, int x, int y)
{
	int disparity;
	int i=0;

	for (i = MAX_Diparity-1 , disparity = 0; disparity < MAX_Diparity; disparity++, i--)
	{
		agreCost[disparity] = ro_func(HammingDistance(&neuronPattern[i * patternSize], bitPattern,patternSize), HAMMING_DISTANCE_LAMBDA);
		disps[disparity]= disparity;
		//agreCost[disparity] += ro_func(SAD(&ImgLeft.data[3 * (y * ImgLeft.width + x )], &ImgRight.data[3 * (y * ImgLeft.width + x - disparity)]), 10);

		if (agreCost[disparity] < 0)
		{
			agreCost[disparity] = 0.0;
			disps[disparity] = 0.0;
		}
	}
//	for (i = MAX_Diparity-1; disparity < max; disparity++, i--)
//	{
//		agreCost[disparity] = -1000;
//		disps[disparity]= disparity;
//	}

	return agreCost;
}

float SelectDisparity(DisparityMap DispMap,float *agreCost, float *disps, int MAX_Disparity, int pixels, int line)
{
	int i,j;
	int bestpos=0;
	float bestHamDist=-1.0;
	float best = -1.0;
	float *layerDisp, aux;
	int imgSize=DispMap.width*DispMap.height;

	for(i=0;i<DispMap.numMaps;i++)
	{
		bestpos=0;
		bestHamDist=-1.0;
		layerDisp=&DispMap.disps[i*imgSize+line*DispMap.width];
		for(j=i;j<MAX_Disparity;j++)
		{
			aux = agreCost[j];
			if(bestHamDist < aux)
			{
				bestHamDist = aux;
				bestpos=j;
			}
		}
		layerDisp[pixels] = disps[bestpos];
		disps[bestpos]=disps[i];
		agreCost[bestpos] = agreCost[i];
		if (best < bestHamDist)
			best = bestHamDist;
	}
	return best;
}


void allocVgRamWNN(int patternSize, int numNeurons, int numSynapses, int numNeuronsLayer, vg_ram_wnn *stereo_wnn_out)
{
	stereo_wnn_out->numNeurons=numNeurons;
	stereo_wnn_out->numSynapses=numSynapses;
	stereo_wnn_out->numNeuronsLayer=numNeuronsLayer;
	stereo_wnn_out->patternSize=patternSize;

	stereo_wnn_out->pattern=(unsigned  int*)calloc(patternSize*numNeurons,sizeof(unsigned  int));
}

void alloc_vg_ram_WNN_multi_neuron(
		int number_of_neurons,
		int pattern_size,
		int number_of_layer_width,
		int number_of_layer_height,
		int number_of_synapses,
		bi_dimention_vg_ram_wnn *stereo_wnn_out)
{
	stereo_wnn_out->number_of_layer_width=number_of_layer_width;
	stereo_wnn_out->number_of_layer_height=number_of_layer_height;
	stereo_wnn_out->number_of_synapses=number_of_synapses;
	stereo_wnn_out->number_of_neurons = number_of_neurons;
	stereo_wnn_out->pattern_size=pattern_size;

	stereo_wnn_out->pattern=(unsigned  int*)calloc(pattern_size * number_of_layer_height * number_of_layer_width * number_of_neurons, sizeof(unsigned  int));
}


void freeVgRamWNN(vg_ram_wnn *stereo_wnn_out)
{
	stereo_wnn_out->numNeurons=0;
	stereo_wnn_out->numSynapses=0;
	stereo_wnn_out->numNeuronsLayer=0;
	stereo_wnn_out->patternSize=0;

	free(stereo_wnn_out->pattern);

	stereo_wnn_out->pattern = NULL;
}


void WNNDispatyLineBack(Image ImgLeft,Image ImgRight, DisparityMap DispMap, int MAX_Disparity, vg_ram_wnn stereo_wnn, unsigned short *Internal_NeuronSynapsesCoordinate_X, unsigned short *Internal_NeuronSynapsesCoordinate_Y,unsigned  int *bitPattern,int image_line, float *agreCost, float *disps)
{

	int neuron;
	int MAX_size,aux;
	int i;

	max = MAX_Disparity;
	for (neuron = stereo_horizontal_ROI_end - 1; neuron >=stereo_horizontal_ROI_ini ; neuron-=inc_stereo_width)
	{
		i = neuron;

		CalcBitPatternMinchintonCell(ImgLeft, gray_image_left,(Internal_NeuronSynapsesCoordinate_X + i * stereo_wnn.numSynapses) ,(Internal_NeuronSynapsesCoordinate_Y + neuron * stereo_wnn.numSynapses) , bitPattern, stereo_wnn.numSynapses,stereo_wnn.patternSize, i, image_line);
		MAX_size = neuron >= MAX_Disparity ? MAX_Disparity : neuron;
		aux = neuron - (MAX_size - 1) < 0 ? 0 : neuron - (MAX_size - 1);
		//max = neuron >= MAX_Disparity ? 0 : MAX_Disparity - neuron;
		BackwardsFindNearestPatterns(bitPattern, stereo_wnn.pattern + aux * stereo_wnn.patternSize , agreCost + neuron * MAX_Disparity, disps + neuron * MAX_Disparity,MAX_size, stereo_wnn.patternSize, i, image_line);

	}
}



void WNNDispatyLine(Image ImgLeft,Image ImgRight, DisparityMap DispMap, int MAX_Disparity, vg_ram_wnn stereo_wnn, unsigned short *Internal_NeuronSynapsesCoordinate_X, unsigned short *Internal_NeuronSynapsesCoordinate_Y,unsigned  int *bitPattern,int image_line, float *agreCost, float *disps)
{

	int neuron;
	int MAX_size,aux;
	int i;


	for (neuron = stereo_horizontal_ROI_ini; neuron < stereo_horizontal_ROI_end; neuron+=inc_stereo_width)
	{
		i = stereo_vertical_ROI_ini + neuron;

		CalcBitPatternMinchintonCell(ImgRight, gray_image_right,(Internal_NeuronSynapsesCoordinate_X + i * stereo_wnn.numSynapses) ,(Internal_NeuronSynapsesCoordinate_Y + neuron*stereo_wnn.numSynapses) , bitPattern, stereo_wnn.numSynapses,stereo_wnn.patternSize, i, image_line);
		aux=ImgLeft.width-neuron;
		MAX_size = aux >= MAX_Disparity ? MAX_Disparity : aux;
		FindNearestPatterns(bitPattern, stereo_wnn.pattern + neuron*stereo_wnn.patternSize, agreCost + neuron * MAX_Disparity, disps + neuron * MAX_Disparity,MAX_size, stereo_wnn.patternSize, i, image_line);

	}
}



void allocSynapsesDistribution(synapsesDistribution *neuronSynapses, int numSynapses)
{
	neuronSynapses->grx = (float *)malloc(numSynapses*sizeof(float));
	neuronSynapses->gry = (float *)malloc(numSynapses*sizeof(float));
	neuronSynapses->numSynapses=numSynapses;
}



void freeSynapsesDistribution(synapsesDistribution *neuronSynapses)
{

	free(neuronSynapses->grx);
	neuronSynapses->grx = NULL;
	free(neuronSynapses->gry);
	neuronSynapses->gry = NULL;
	neuronSynapses->numSynapses = 0;

}



void
SetSynapsesCoodinate(int width, int height, int numSynapses)
{
	int x,y;
	synapsesDistribution neuronSynapses;

	unsigned short *NeuronSynapsesAux_X, *NeuronSynapsesAux_Y;

	allocSynapsesDistribution(&neuronSynapses, numSynapses);
	CalcGuassianSynapsesDistribution(neuronSynapses);
	//CalcNeuronSynapsesCoordinateOnImage(gaussian_radius + 0.5, gaussian_radius + 0.5, neuronSynapses, NeuronSynapsesAux_X2, NeuronSynapsesAux_Y2, width, height);
	for(y = 0; y < height; y++)
	{
		NeuronSynapsesAux_X = &NeuronSynapsesCoordinate_X[y * numSynapses * width];
		NeuronSynapsesAux_Y = &NeuronSynapsesCoordinate_Y[y * numSynapses * width];
		for(x = 0; x < width; x++)
				CalcNeuronSynapsesCoordinateOnImage(x, y, neuronSynapses, &NeuronSynapsesAux_X[x*numSynapses], &NeuronSynapsesAux_Y[x*numSynapses], width, height);
	}
	freeSynapsesDistribution(&neuronSynapses);
}

float *get_wnn_confidance(int width, int height)
{
	float 	*confidance_out  = NULL;
	confidance_out = (float *)calloc((width) * (height),sizeof(float));
	memcpy(confidance_out, confidance, height * width * sizeof(float));

	return confidance_out;
}

void
build_synapses_cordinates(int width, int height, int numSynapses, float gaussian_radius_param)
{
	gaussian_radius = gaussian_radius_param;
	SetSynapsesCoodinate(width, height, numSynapses);
}


//Initialization of VG-RAM WNN variables
void WNNInitialize(int height, int width, int MAX_Diparity,int numMaps,int numOfThreads, int numSynapses, float gaussian_radius_param, int horizontal_ROI_ini, int horizontal_ROI_end, double lambda)
{
	int 	i;

	patternSize = numSynapses/PATTERN_UNIT_SIZE;
	patternSize += numSynapses%PATTERN_UNIT_SIZE ? 1: 0;
	HAMMING_DISTANCE_LAMBDA = lambda;

	gaussian_radius = gaussian_radius_param;

	synapsesDistribution neuronSynapses;

	allocSynapsesDistribution(&neuronSynapses, numSynapses);
	CalcGuassianSynapsesDistribution(neuronSynapses);

	if (disp.disps != NULL)
		free(disp.disps);
	allocDisparityMap(&disp,width, height, numMaps);


	if (stereo_wnn != NULL)
	{
		for(i=0;i<numOfThreads;i++)
		{
			if (stereo_wnn[i].pattern != NULL)
					free(stereo_wnn[i].pattern);
		}
		free(stereo_wnn);
	}
	stereo_wnn = (vg_ram_wnn*)calloc(numOfThreads,sizeof(vg_ram_wnn));

	if (bitPattern != NULL)
	{
			for(i=0;i<numOfThreads;i++)
			{
				if (bitPattern[i] != NULL)
						free(bitPattern[i]);
			}
			free(bitPattern);
	}

	bitPattern = (unsigned  int **)calloc(numOfThreads,sizeof(unsigned  int*));

	if (agreCost != NULL)
		free(agreCost);
	agreCost = (float*)calloc(width * height * MAX_Diparity,sizeof(float));

	if (agreCost_new != NULL)
		free(agreCost_new);
	agreCost_new = (float*)calloc(width * height * MAX_Diparity,sizeof(float));

	if (disps != NULL)
		free(disps);
	disps = (float*)calloc(width * height * MAX_Diparity,sizeof(float));

	if (confidance != NULL)
		free(confidance);
	confidance = (float*)calloc(width * height, sizeof(float));

	stereo_horizontal_ROI_ini = horizontal_ROI_ini;
	stereo_horizontal_ROI_end = horizontal_ROI_end;

	gray_size_offset =  width;

	for(i=0;i<numOfThreads;i++)
	{
		allocVgRamWNN(patternSize, width, numSynapses, MAX_Diparity, &stereo_wnn[i]);
		bitPattern[i] = (unsigned  int *)calloc(patternSize,sizeof(unsigned  int));
	}

	stereo_vertical_ROI_ini = 0;//(int)(gaussian_radius_param + 0.5);

	if (up_limit != NULL)
		free(up_limit);
	up_limit = (unsigned short *)calloc((width) * (height), sizeof(unsigned short));

	if (down_limit != NULL)
			free(down_limit);
	down_limit = (unsigned short *)calloc((width) * (height), sizeof(unsigned short));

	if (right_limit != NULL)
		free(right_limit);
	right_limit = (unsigned short *)calloc((width) * (height), sizeof(unsigned short));

	if (left_limit != NULL)
		free(left_limit);
	left_limit = (unsigned short *)calloc((width) * (height), sizeof(unsigned short));

	if (NeuronSynapsesCoordinate_X != NULL)
		free(NeuronSynapsesCoordinate_X);
	NeuronSynapsesCoordinate_X = (unsigned short *)calloc(numSynapses * (width) * (height), sizeof(unsigned short));

	if (NeuronSynapsesCoordinate_Y != NULL)
			free(NeuronSynapsesCoordinate_Y);
	NeuronSynapsesCoordinate_Y = (unsigned short *)calloc(numSynapses * (width) * (height), sizeof(unsigned short));

	if (gray_image_left != NULL)
		free(gray_image_left);
	gray_image_left = (unsigned int *)calloc((width) * (height),sizeof(unsigned int));

	if (gray_image_right != NULL)
			free(gray_image_right);
	gray_image_right = (unsigned int *)calloc((width) * (height),sizeof(unsigned int));


	//CalcNeuronSynapsesCoordinateOnImage(gaussian_radius + 0.5, gaussian_radius + 0.5, neuronSynapses, NeuronSynapsesCoordinate_X, NeuronSynapsesCoordinate_Y, width, height);
	//CalcNeuronSynapsesCoordinateOnImage(0, 0, neuronSynapses, NeuronSynapsesCoordinate_X, NeuronSynapsesCoordinate_Y, width, height);
	SetSynapsesCoodinate(width, height, numSynapses);
	//alloc_vg_ram_WNN_multi_neuron(1, patternSize, width, height, numSynapses, &wnn_left_neuron);


#ifndef NO_CUDA

	alloc_vg_ram_WNN_multi_neuron_on_GPU(1, patternSize, width, height, numSynapses, &wnn_left_neuron_GPU);

	img_left.data = NULL;
	img_right.data = NULL;

	alloc_image_on_GPU(width, height, 3, &img_left);
	alloc_image_on_GPU(width, height, 3, &img_right);

	allocGPUData_vg((void**)&gray_image_left_GPU, (width + (int)(2*(gaussian_radius + 0.5))) * (height+ (int)(2*(gaussian_radius + 0.5))) * sizeof(unsigned int));
	allocGPUData_vg((void**)&gray_image_right_GPU,(width + (int)(2*(gaussian_radius + 0.5))) * (height+ (int)(2*(gaussian_radius + 0.5))) * sizeof(unsigned int));

	allocGPUData_vg((void**)&NeuronSynapsesCoordinate_X_GPU, numSynapses * sizeof(unsigned short));
	allocGPUData_vg((void**)&NeuronSynapsesCoordinate_Y_GPU, numSynapses * sizeof(unsigned short));

	alloc_disparity_map_on_GPU(&disp_GPU, width, height, numMaps);

	GPUcopy_vg(NeuronSynapsesCoordinate_Y_GPU, NeuronSynapsesCoordinate_Y, numSynapses  * sizeof(unsigned short), 1);
	GPUcopy_vg(NeuronSynapsesCoordinate_X_GPU, NeuronSynapsesCoordinate_X, numSynapses  * sizeof(unsigned short), 1);

#endif

	freeSynapsesDistribution(&neuronSynapses);
}

void WNNDisparity(Image ImgLeft, Image ImgRight, DisparityMap DispMap, int MAX_Diparity, int numSynapses, int numberOfThreads)
{
	int	line, line2;
	int	threadIdx;

	#pragma omp parallel default(none) private(line,threadIdx,line2) shared(stereo_vertical_ROI_ini,stereo_wnn,NeuronSynapsesCoordinate_X,NeuronSynapsesCoordinate_Y,ImgLeft, ImgRight, DispMap, MAX_Diparity, numSynapses, numberOfThreads,patternSize,disps,bitPattern,agreCost,inc_stereo_height)
	{
		threadIdx = omp_get_thread_num();
		line2 = stereo_vertical_ROI_ini;
	  #pragma omp for
		for(line = 0; line < ImgLeft.height; line += inc_stereo_height)
		{
			//Foreward train/test -> left to right
			line2 = stereo_vertical_ROI_ini + line;
			unsigned short *NeuronSynapsesCoordinateAux_X = NeuronSynapsesCoordinate_X + line2 * ImgLeft.width * numSynapses;
			unsigned short *NeuronSynapsesCoordinateAux_Y = NeuronSynapsesCoordinate_Y + line2 * ImgLeft.width * numSynapses;

			WNNTrain(ImgLeft, ImgRight, MAX_Diparity, stereo_wnn[threadIdx], NeuronSynapsesCoordinateAux_X, NeuronSynapsesCoordinateAux_Y, line2);

			WNNDispatyLineBack(ImgLeft, ImgRight, DispMap, MAX_Diparity, stereo_wnn[threadIdx], NeuronSynapsesCoordinateAux_X, NeuronSynapsesCoordinateAux_Y, bitPattern[threadIdx], line2, agreCost + line * ImgLeft.width * MAX_Diparity, disps  + line * ImgLeft.width * MAX_Diparity);
		}
	}
}


void
calc_disparity_maps(
		DisparityMap DispMap,
	    int MAX_Disparity,
	    float *agreCost,
	    float *disps,
	    int begin,
	    int end,
	    int inc_width,
	    int inc_height
	    )
{
	int y;

	#pragma omp parallel for
	for(y = 0; y < DispMap.height; y += inc_height)
	{
		float *agreCost_line = agreCost + y * DispMap.width * MAX_Disparity;
		float *disps_line = disps + y * DispMap.width * MAX_Disparity;
		int x;
		for(x = end - 1; x >= begin; x -= inc_width)
		{
			int MAX_size = x >= MAX_Disparity ? MAX_Disparity : x;
			confidance[y * end + x] = SelectDisparity(DispMap,agreCost_line + x * MAX_Disparity,disps_line + x * MAX_Disparity,MAX_size,x,y);
		}
	}

}


void
set_scan_image_param(int tal1_in, int tal2_in, int L1_in, int L2_in)
{
	tal1 = tal1_in;
	tal2 = tal2_in;
	L1 = L1_in;
	L2 = L2_in;
}


void disparityVgRamWNN(unsigned char*image_left,unsigned char*image_right, int stereo_height, int stereo_width, int stereo_disparity, float *disparity , int numSynapses,int numOfThreads, int iteration, int inc_width, int inc_height)
{
	ImgLeft.channels=ImgRight.channels=3;
	ImgLeft.width=ImgRight.width=stereo_width;
	ImgLeft.height=ImgRight.height=stereo_height;

	ImgLeft.data=image_left;
	ImgRight.data=image_right;
	inc_stereo_height = inc_height;
	inc_stereo_width = inc_width;

#ifndef NO_CUDA

	GPUcopy_vg(img_left.data, image_left, stereo_width * stereo_height * sizeof(unsigned char) * 3, 1);
	GPUcopy_vg(img_right.data, image_right, stereo_width * stereo_height * sizeof(unsigned char) * 3, 1);

	rgb_to_gray_GPU(img_left,gray_image_left_GPU, stereo_vertical_ROI_ini,gray_size_offset);
	rgb_to_gray_GPU(img_right,gray_image_right_GPU, stereo_vertical_ROI_ini,gray_size_offset);

	 	WNN_disparity_GPU2(
			gray_image_left_GPU,
			gray_image_right_GPU,
			stereo_width,
			stereo_height,
			gray_size_offset,
			stereo_disparity,
			wnn_left_neuron_GPU,
			NeuronSynapsesCoordinate_X_GPU,
			NeuronSynapsesCoordinate_Y_GPU,
			0,
			HamDist_GPU,
			disps_GPU,
			stereo_horizontal_ROI_ini,
			stereo_horizontal_ROI_end,
			stereo_vertical_ROI_ini,
			inc_stereo_width,
			inc_stereo_height,disp_GPU);



	GPUcopy_vg(disp.disps, disp_GPU.disps, disp_GPU.width * disp_GPU.height * disp_GPU.numMaps * sizeof(unsigned char),0);

#else
		rgb_to_gray(ImgLeft,gray_image_left);
		rgb_to_gray(ImgRight,gray_image_right);

		WNNDisparity(ImgLeft, ImgRight, disp, stereo_disparity, numSynapses,numOfThreads);

		if (L1 && L2)
		{
			vertical_color_limits(img_left_filtered, up_limit, down_limit, tal1, tal2, L1, L2);
			horizontal_color_limits(img_left_filtered, left_limit, right_limit, tal1, tal2, L1, L2);

			sum_horizontal_vertical(agreCost, agreCost_new, stereo_width, stereo_height, L1, stereo_disparity,
					up_limit, down_limit, left_limit, right_limit);

			sum_vertical_horizontal(agreCost_new, agreCost, stereo_width, stereo_height, L1, stereo_disparity,
					up_limit, down_limit, left_limit, right_limit);

			sum_vertical_horizontal(agreCost, agreCost_new, stereo_width, stereo_height, L1, stereo_disparity,
					up_limit, down_limit, left_limit, right_limit);

			sum_horizontal_vertical(agreCost_new, agreCost, stereo_width, stereo_height, L1, stereo_disparity,
					up_limit, down_limit, left_limit, right_limit);
		}

		//scaline_opmization(img_left_filtered, img_right_filtered, agreCost, agreCost_new, stereo_disparity, 100.0, 300.0, 0);


		calc_disparity_maps(
				disp,
				stereo_disparity,
				agreCost,
			    disps,
			    stereo_horizontal_ROI_ini,
			    stereo_horizontal_ROI_end,
			    inc_stereo_width,
			    inc_stereo_height);

#endif

	WinnerTakesItAll(disp,iteration);
	//mean(disp,1);

	memcpy(disparity,disp.disps,stereo_height * stereo_width * sizeof(float));
}


float
color_distance(unsigned char *pixel, unsigned char *pixelL)
{
//	float R = 0.299f * fabs(pixel[0] - pixelL[0]);
//	float G = 0.587f * fabs(pixel[1] - pixelL[1]);
//	float B = 0.114f * fabs(pixel[2] - pixelL[2]);

	float R = fabs(pixel[0] - pixelL[0]);
	float G = fabs(pixel[1] - pixelL[1]);
	float B = fabs(pixel[2] - pixelL[2]);
	float dc = MAX(R, G);
	return MAX(B,dc);
	//return fabs(p1 - p2);

}

void calc_P1_P2(Image left, Image right, int x, int y, int d, int o, int v, float Pi1, float Pi2, int talSO, float *P1_out, float *P2_out)
{
	float P1 = Pi1, P2 = Pi2;
	float D1, D2;

	if ((y + v) < 0 || (y + v) > left.height - 1 || (x + o) < 0 || (x + o) > left.width -1)
	{
		P1 = Pi1 / 10.0f;
		P2 = Pi2 / 10.0f;
		return;
	}

	D1 = color_distance(&left.data[3 * (y * left.width + x)], &left.data[3 * ((y + v)* left.width + x + o)]);
	if (x - d + o > 0)
		D2 = color_distance(&right.data[3 * (y * right.width + x - d)], &right.data[3 * ((y + v) * right.width + x - d + o)]);
	else
		D2 = talSO + 1;

	if (D1 < talSO && D2 < talSO)
	{
		P1 = Pi1;
		P2 = Pi2;
	}
	else if (D1 < talSO && D2 >= talSO)
	{
		P1 = Pi1 / 4.0f;
		P2 = Pi2 / 4.0f;
	}
	else if (D1 >= talSO && D2 < talSO)
	{
		P1 = Pi1 / 4.0f;
		P2 = Pi2 / 4.0f;
	}
	else if (D1 >= talSO && D2 >= talSO)
	{
		P1 = Pi1 / 10.0f;
		P2 = Pi2 / 10.0f;
	}

	*P1_out = P1;
	*P2_out = P2;
}

float calc_min_cost(float *agreCost_p, int max_disparity)
{
	int d;
	float min = 10000000000;

	for(d = 0; d < max_disparity; d++)
	{
		min = agreCost_p[d] < min ? agreCost_p[d] : min;
	}

	return min;
}

float calc_Cr(Image left, Image right, float *agreCost_p, int pos, int x, int y, int d, int o, int v, float Pi1, float Pi2, int talSO, int max_disparity)
{
	float P1 = 0.0, P2 = 0.0;
	float Cr, min, Cr_min, min2, Cp1, Cs1;

	//printf("%lf %lf\n", P1, P2);
	int MAX_size = x >= max_disparity ? max_disparity : x;

	int p = (y + v) * left.width * max_disparity + (x + o) * max_disparity;
	if ((y + v) < 0 || (y + v) > left.height -1 || (x + o) < 0 || (x + o) > left.width - 1)
	{
		return 0.0;
	}

	calc_P1_P2(left, right, x, y, d, o, v, Pi1, Pi2, talSO, &P1, &P2);

	if (d < MAX_size - 2)
		Cp1 = agreCost_p[p + d + 1] + P1;
	else
		Cp1 = 100000000;
	if (d > 0)
		Cs1 = agreCost_p[p + d - 1] + P1;
	else
		Cs1 = 100000000;

	min = calc_min_cost(&agreCost_p[p], MAX_size);

	Cr_min = min + P2;

	min2 = agreCost_p[p + d] < Cp1 ? agreCost_p[p + d] : Cp1;
	min2 = min2 < Cs1 ? min2 : Cs1;
	min2 = min2 < Cr_min ? min2 : Cr_min;
	Cr = min2 - min;

	return Cr;

}

void scaline_opmization(Image left, Image right, float *agreCost, float *agreCost_out, int max_disparity, float Pi1, float Pi2, int talSO)
{
	int x, y, d, pos;
	float C1;
	float Cr;
	int k = 1;

	for(y = 0; y < left.height - 1; y++)
	{
		for(x = left.width - 1; x >= 0; x--)
		{
			int MAX_size = x >= max_disparity ? max_disparity : x;
			for(d = 0; d < MAX_size; d++)
			{
				pos = y * left.width * max_disparity + x * max_disparity;
				C1 = agreCost[pos + d];
				//agreCost_out[pos + d] = 0;
					//printf("%f\n", C1);
				Cr = C1 + calc_Cr(left, right, agreCost, pos, x, y, d, 0, k, Pi1, Pi2, talSO, max_disparity);
				Cr += C1 + calc_Cr(left, right, agreCost, pos, x, y, d, 0, -k, Pi1, Pi2, talSO, max_disparity);
				Cr += C1 + calc_Cr(left, right, agreCost, pos, x, y, d, k, 0, Pi1, Pi2, talSO, max_disparity);
				Cr += C1 + calc_Cr(left, right, agreCost, pos, x, y, d, -k, 0, Pi1, Pi2, talSO, max_disparity);
				agreCost_out[pos + d] += Cr / (4.0);

				//printf("%f %f\n", Cr / 4.0, 4.0 * C1);

			}
		}
	}
}


void WNNTerminate()
{
	int 	i;
	int	threadNum = 1;//omp_get_max_threads();

	free(NeuronSynapsesCoordinate_Y);
	free(NeuronSynapsesCoordinate_X);

	// TODO: Frees tem que ser melhorados aqui... Ainda existem memory leaks

	for(i=0;i<threadNum;i++)
	{
		freeVgRamWNN(&stereo_wnn[i]);
	}
	free(stereo_wnn);
}

void
sum_horizontal_vertical(float *agreCost, float *agreCost_out,int width, int height, int L, int max_disparity,
		unsigned short *up_limit, unsigned short *down_limit,
		unsigned short *left_limit, unsigned short *right_limit)
{
	int x, y, i, j, k, l, d;
	float sum;

#pragma omp parallel default(none) private(x, y, i, j, k, l, d, sum) shared(agreCost, agreCost_out, width, height, L, max_disparity,\
		up_limit, down_limit, left_limit, right_limit)
	{
		float harm[L];
#pragma omp for
		for (y = 0; y < height; y++)
		{
			for (x = 0; x < width; x++)
			{
				for (d = 0; d < max_disparity; d++)
				{
					for (j = left_limit[y * width + x], k = 0; j <= right_limit[y * width + x]; j++, k++)
					{
						sum = 0.0;
						for (i = up_limit[y * width + j]; i <= down_limit[y * width + j]; i++)
						{
							sum += agreCost[i * width * max_disparity + j * max_disparity + d];
						}
						harm[k] = sum;
					}

					sum = 0.0;
					for (l = 0; l < k; l++)
					{
						sum += harm[l];
					}
					if (k == 0)
						sum = agreCost[y * width * max_disparity + x * max_disparity + d];

					agreCost_out[y * width * max_disparity + x * max_disparity + d] = sum;			}
			}
		}
	}
}


void
sum_vertical_horizontal(float *agreCost, float *agreCost_out, int width, int height, int L, int max_disparity,
		unsigned short *up_limit, unsigned short *down_limit,
		unsigned short *left_limit, unsigned short *right_limit)
{
	int x, y, i, j, k, l, d;

	float sum;

#pragma omp parallel default(none) private(x, y, i, j, k, l, d, sum) shared(agreCost, agreCost_out, width, height, L, max_disparity,\
up_limit, down_limit, left_limit, right_limit)
	{
		float harm[L];
#pragma omp for
		for (y = 0; y < height; y++)
		{
			for (x = 0; x < width; x++)
			{
				for (d = 0; d < max_disparity; d++)
				{
					for (i = up_limit[y * width + x], k = 0; i <= down_limit[y * width + x]; i++, k++)
					{
						sum = 0.0;
						for (j = left_limit[i * width + x]; j <= right_limit[i * width + x]; j++)
						{
							sum += agreCost[i * width * max_disparity + j * max_disparity + d];
						}
						harm[k] = sum;
					}

					sum = 0.0;
					for (l = 0; l < k; l++)
					{
						sum += harm[l];
					}

					if (k == 0)
						sum = agreCost[y * width * max_disparity + x * max_disparity + d];

					agreCost_out[y * width * max_disparity + x * max_disparity + d] = sum;
				}
			}
		}
	}
}


void
vertical_color_limits(Image img, unsigned short *up_limit_out, unsigned short *down_limit_out, int tal1, int tal2, int L1, int L2)
{
	int x, y, i, j, k, l;
	int dc, dc2;
	int L, tal, up, down, flag;

#pragma omp parallel for default(none) private(x, y, i, j, k, l, dc, dc2, L, tal, up, down, flag) \
	shared(img, up_limit_out, down_limit_out, tal1, tal2, L1, L2)
	for(y = 0; y < img.height; y++)
	{
		for(x = 0; x < img.width; x++)
		{
			up = 0;
			down = 0;
			L = L2;
			tal = tal1;
			flag = 0;
			down_limit_out[y * img.width + x] = y;
			up_limit_out[y * img.width + x] = y;
			for (i = y, j = y; 1;)
			{
				if (!down)
				{
					dc = color_distance(&img.data[3 * (i * img.width + x)], &img.data[3 * (y * img.width + x)]);
					if (dc > tal)
					{
						k = i + 1;

						if (k > img.height - 1)
						{
							down = 1;
							down_limit_out[y * img.width + x] = i;
						}
						else
						{

							dc2 = color_distance(&img.data[3 * (k * img.width + x)], &img.data[3 * (y * img.width + x)]);

							if (dc2 > tal && !flag)
							{
								down = 1;
								down_limit_out[y * img.width + x] = i - 1;
							} else if (flag)
							{
								down = 1;
								down_limit_out[y * img.width + x] = i - 1;
							}
						}
					}
					if (!down)
						i++;

					if (i > img.height - 1)
					{
						down = 1;
						down_limit_out[y * img.width + x] = img.height - 1;
					}
				}

				if (!up)
				{
					dc = color_distance(&img.data[3 * (j * img.width + x)], &img.data[3 * (y * img.width + x)]);

					if (dc > tal)
					{
						l = j - 1;

						if (l < 0)
						{
							up = 1;
							up_limit_out[y * img.width + x] = j;
						}
						else
						{
							dc2 = color_distance(&img.data[3 * (l * img.width + x)], &img.data[3 * (y * img.width + x)]);

							if (dc2 > tal && !flag)
							{
								up = 1;
								up_limit_out[y * img.width + x] = j + 1;
							}
							else if (flag)
							{
								up = 1;
								up_limit_out[y * img.width + x] = j + 1;
							}
						}
					}

					if (!up)
						j--;

					if (j < 0)
					{
						up = 1;
						up_limit_out[y * img.width + x] = 0;
					}

				}


				if (up && down)
					break;

				if ((i - j) >= L)
				{
					if ((i - j) >= L1)
					{
						if (!down)
							down_limit_out[y * img.width + x] = i;
						if (!up)
							up_limit_out[y * img.width + x] = j;
						break;
					}
					flag = 1;
					L = L1;
					tal = tal2;
				}
			}
		}
	}
}


void
horizontal_color_limits(Image img, unsigned short *left_limit_out, unsigned short *right_limit_out, int tal1, int tal2, int L1, int L2)
{
	int x, y, i, j, k, l;
	float dc, dc2;
	int L = L2;
	int tal = tal1;
	int left = 0;
	int right = 0;
	int flag = 0;
#pragma omp parallel for default(none) private(x, y, i, j, k, l, dc, dc2, L, tal, left, right, flag) \
	shared(img, left_limit_out, right_limit_out, tal1, tal2, L1, L2)
	for(y = 0; y < img.height; y++)
	{
		for(x = 0; x < img.width; x++)
		{
			L = L2;
			tal = tal1;
			left = 0;
			right = 0;
			flag = 0;
			right_limit_out[y * img.width + x] = x;
			left_limit_out[y * img.width + x] = x;
			for (i = x, j = x; 1;)
			{
				if (!right)
				{
					dc = color_distance(&img.data[3 * (y * img.width + i)], &img.data[3 * (y * img.width + x)]);
					k = i + 1;

					if (dc > tal)
					{
						k = i + 1;
						if (k > img.width - 1)
						{
							right = 1;
							right_limit_out[y * img.width + x] = i;
						}
						else
						{
							dc2 = color_distance(&img.data[3 * (y * img.width + k)], &img.data[3 * (y * img.width + x)]);

							if (dc2 > tal && !flag)
							{
								right = 1;
								right_limit_out[y * img.width + x] = i - 1;
							} else if (flag)
							{
								right = 1;
								right_limit_out[y * img.width + x] = i - 1;
							}
						}
					}
					if (!right)
						i++;

					if (i > img.width - 1)
					{
						right = 1;
						right_limit_out[y * img.width + x] = img.width - 1;
					}
				}

				if (!left)
				{
					dc = color_distance(&img.data[3 * (y * img.width + j)], &img.data[3 * (y * img.width + x)]);
					if (dc > tal)
					{
						l = j - 1;

						if (l < 0)
						{
							left = 1;
							left_limit_out[y * img.width + x] = j;
						}
						else
						{
							dc2 = color_distance(&img.data[3 * (y * img.width + l)], &img.data[3 * (y * img.width + x)]);

							if (dc2 > tal && !flag)
							{
								left = 1;
								left_limit_out[y * img.width + x] = j + 1;
							}
							else if (flag)
							{
								left = 1;
								left_limit_out[y * img.width + x] = j + 1;
							}
						}
					}
					if (!left)
						j--;

					if (j <= 0)
					{
						left = 1;
						left_limit_out[y * img.width + x] = 0;
					}
				}

				if (left && right)
					break;

				if ((i - j) >= L)
				{
					if ((i - j) >= L1)
					{
						if (!right)
							right_limit_out[y * img.width + x] = i;
						if (!left)
							left_limit_out[y * img.width + x] = j;
						break;
					}
					flag = 1;
					L = L1;
					tal = tal2;
				}
			}
		}
	}
}

void mean(DisparityMap disps, int iteration)
{
	int 	l,i,j;

	float*	disp_pixel;
	float 	aux;


	for(l=0; l < iteration; l++)
	{
		#pragma omp parallel for default(none) private(i,j,disp_pixel,aux) shared(disps, inc_stereo_height, inc_stereo_width,stereo_horizontal_ROI_ini,stereo_horizontal_ROI_end)
		for(i = 0; i<disps.height; i += inc_stereo_height)
		{
			for(j = stereo_horizontal_ROI_ini; j<stereo_horizontal_ROI_end; j += inc_stereo_width)
			{
				disp_pixel = disps.disps + (i * disps.width + j);
				aux = *CENTRAL(0, disps.width, disps.height, disp_pixel);
				aux += *UP(disps.width,disp_pixel);
				aux += *RIGHT(disps.width,disp_pixel);
				aux += *LEFT(disps.width,disp_pixel);
				aux += *LOW(disps.width,disp_pixel);
				*CENTRAL(0, disps.width, disps.height, disp_pixel) = (float)((unsigned char)(aux / 5.0f));
			}
		}
	}
}

void WinnerTakesItAll(DisparityMap disps, int iteration)
{
	int l,k,i,j;
	int	opinion;
	int	highest_opinion_index;

	float*	disp_pixel;
	float*	central_pixel;
	float 	aux;

	int before_op;

	for(l=0; l < iteration; l++)
	{
		int init_i = 0;
		int init_j = stereo_horizontal_ROI_ini;
		#pragma omp parallel for default(none) private(before_op,i,j,k,opinion,disp_pixel,highest_opinion_index,central_pixel,aux) shared(init_i, init_j, disps, inc_stereo_height, inc_stereo_width,stereo_horizontal_ROI_ini,stereo_horizontal_ROI_end)
		for(i = init_i; i<disps.height; i += inc_stereo_height)
		{
			for(j = init_j; j<stereo_horizontal_ROI_end; j +=  inc_stereo_width)
			{
				disp_pixel = disps.disps + (i * disps.width + j);
				highest_opinion_index = 0;
				before_op = 1;
				for(k = 0; k < disps.numMaps; k++)
				{
					opinion = 0;
					central_pixel = CENTRAL(k, disps.width, disps.height, disp_pixel);

					//UP
					if(!(i==0))
						if(*central_pixel == *UP(disps.width,disp_pixel))
							opinion++;

					//RIGHT
					if(!(j==disps.width-inc_stereo_width))
						if(*central_pixel == *RIGHT(disps.width,disp_pixel))
							opinion++;

					//LEFT
					if(!(j==0))
						if(*central_pixel == *LEFT(disps.width,disp_pixel))
							opinion++;

					//LOW
					if(!(i==disps.height-inc_stereo_height))
						if(*central_pixel == *LOW(disps.width,disp_pixel))
							opinion++;


					// obtain the new highest opinion and the new highest opinion index
					if(opinion > before_op)
					{
						highest_opinion_index = k;
						before_op = opinion;
					}
				}

				if(highest_opinion_index)
				{
					aux = *disp_pixel;
					*disp_pixel = *CENTRAL(highest_opinion_index, disps.width, disps.height, disp_pixel);
					*CENTRAL(highest_opinion_index, disps.width, disps.height, disp_pixel) = aux;
				}
			}
		}
	}
}



void allocImage(Image *img,int width, int height, int numChannels)
{
	img->channels=numChannels;
	img->width=width;
	img->height=height;

	img->data=(unsigned char*)calloc(numChannels*width*height,sizeof(unsigned char));
}

void allocDisparityMap(DisparityMap *disp,int w, int h, int numOfDisparity )
{
	disp->height=h;
	disp->width=w;
	disp->numMaps=numOfDisparity;

	disp->disps=(float *)calloc(w*h*numOfDisparity,sizeof(float));
}


