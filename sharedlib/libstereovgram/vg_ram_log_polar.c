
#include "vg_ram_log_polar.h"


//VG-RAM WNN Global Variables
int 	*NeuronSynapsesCoordinate_X;
int 	*NeuronSynapsesCoordinate_Y;
int	patternSize;
unsigned int	**bitPattern;
unsigned int	**HamDist;
unsigned short	**disps;
vg_ram_wnn		*stereo_wnn;
float gaussian_radius;
DisparityMap disp;
DisparityMap disp_out;

int inc_stereo_height;
int inc_stereo_width;

int stereo_horizontal_ROI_ini;
int stereo_horizontal_ROI_end;

unsigned int *gray_image_left;
unsigned int *gray_image_right;

float line_pixels_mean[2048];
float line_pixels_standard_deviation[2048];
float line_target[2048];
float *pixels_targets;

//  functions for Winner-Takes-it-All filter
 unsigned char * CENTRAL(int plane, int w, int h,unsigned char* p) { return p + (w*h*plane); }
 unsigned char * UP(int w, unsigned char* p) { return (p - w * inc_stereo_height); }
 unsigned char * UP_RIGHT(int w, unsigned char* p) {return (p - w * inc_stereo_height + inc_stereo_width); }
 unsigned char * RIGHT(int w, unsigned char* p) { return( p + inc_stereo_width); }
 unsigned char * LOW_RIGHT(int w, unsigned char* p) { return( p + w * inc_stereo_height + inc_stereo_width); }
 unsigned char * LOW(int w, unsigned char* p) { return( p + w * inc_stereo_height);  }
 unsigned char * LOW_LEFT(int w, unsigned char* p) { return( p + w * inc_stereo_height - inc_stereo_width); }
 unsigned char * LEFT(int w, unsigned char* p) { return( p - inc_stereo_width ); }
 unsigned char * UP_LEFT(int w, unsigned char* p) { return( p - w * inc_stereo_height - inc_stereo_width); }


 double
DistanceFromImageCenter(int wi, int hi, int w, int h, int u, double log_factor)
{
	double exp_val, x;

	x = ((double) u / (double) (w/2)) * log_factor;
	exp_val = (double) (wi/2) * (exp (log (log_factor) * (x - log_factor) / log_factor) - (1.0/log_factor)) * (log_factor / (log_factor - 1.0));
	
	return (exp_val);
}

void
LogPolarMapping(int *xi, int *yi, int wi, int hi, int u, int v, int w, int h, int x_center, int y_center, double correction, double log_factor)
{
	double LOG_POLAR_SCALE_FACTOR = 1.0;
	//double LOG_POLAR_SCALE_FACTOR = 2.0;
	double LOG_POLAR_THETA_CORRECTION = 0.0;
	static int previous_u = -1;
	static double previous_d;
	double d, theta;

	if (u < w/2)
	{
		if (u == previous_u)
			d = previous_d;
		else
			d = LOG_POLAR_SCALE_FACTOR * DistanceFromImageCenter(wi, hi, w, h, (w-1)/2 - u, log_factor);

		theta = pi * (((double) h * (3.0 / 2.0) - ((double) v * correction)) / (double) h) + LOG_POLAR_THETA_CORRECTION;

	}
	else
	{
		if (u == previous_u)
			d = previous_d;
		else
			d = LOG_POLAR_SCALE_FACTOR * DistanceFromImageCenter(wi, hi, w, h, u - w/2, log_factor);

		theta = pi * (((double) h * (3.0 / 2.0) + ((double) v * correction)) / (double) h) + LOG_POLAR_THETA_CORRECTION;

	}

	*xi = (int) (d * cos(theta) + 0.5) + x_center;
	*yi = (int) (d * sin(theta) + 0.5) + y_center;

	previous_u = u;
	previous_d = d;
}

void
DisparityMapInverseLogPolar(DisparityMap disp)
{
	int width,height,disparities;
	int x, y, i , h, w, hi, wi, xi, yi, x_center, y_center, pixel_position_dispartity_map;
	int previous_xi , previous_yi;
	
	unsigned char *disparity_maps;
	unsigned char *disparity_maps_out;
	//unsigned char disparity;
	unsigned char previous_output[10];
	
	//double log_factor = 1.0;
	double log_factor = 2.0;
	double correction;
	
	width = disp.width;
	height = disp.height;
	disparities = disp.numMaps;	// Number of disparities
	disparity_maps = disp.disps;	// Disparity maps
	disparity_maps_out=disp_out.disps;
	
	wi = width;		// Input Image Width
	hi = height;		// Input Image Height 
	w = width;		// Neuron Layer Width
	h = height;		// Neuron Layer Height
	x_center = wi/2 ;	// Neuron Layer "x" center point
	y_center = hi/2 ;	// Neuron Layer "y" center point
	correction = (double) h / (double) (h -1);	// log-polar transform correction factor
	
	previous_xi = -1;
	previous_yi = -1;

	for(y=0;y<height;y++)
	{
		for(x=0;x<width;x++)
		{
			LogPolarMapping(&xi, &yi, wi, hi, x, y, w, h, x_center, y_center, correction, log_factor); // log-polar center position
			
			// xi, yi -> log polar, x, y -> imagem
			// Image limit range (thresholding)
			//xi = xi > 0 ? xi : 0;
			//yi = yi > 0 ? yi : 0;
			//xi = xi < wi ? xi : wi-1;
			//yi = yi < hi ? yi : hi-1;

			//#if 0
			if( xi < 0 || xi >= wi || yi < 0 || yi >= hi )
			{
				for(i = 0; i < disparities; i++)
				{
					pixel_position_dispartity_map =  width*height*i;
					disparity_maps_out[ y * width + x + pixel_position_dispartity_map ] = 0;
				}
			}
			else
			{
				for(i = 0; i < disparities; i++)
				{
					pixel_position_dispartity_map = width*height*i;
					//disparity = disparity_maps[ y * width + x + pixel_position_dispartity_map ];
					disparity_maps_out[ yi * width + xi + pixel_position_dispartity_map ] = disparity_maps[ y * width + x + pixel_position_dispartity_map ];
					//disparity_maps[ yi * width + xi + pixel_position_dispartity_map ] = swap_disparity;
				}	
			}
			//#endif
				
			#if 0		
			if ((xi == previous_xi) && (yi == previous_yi))
			{
				for(i = 0; i < disparities; i++)
				{
					pixel_position_dispartity_map =  width*height*i;
					disparity_maps_out[ yi * width + xi + pixel_position_dispartity_map ] = previous_output[i];
				}
			}	
			else
			{
				if (xi >= wi || xi < 0 || yi >= hi || yi < 0)
				{
					for(i = 0; i < disparities; i++)
					{
						pixel_position_dispartity_map =  width*height*i;
						previous_output[i] = disparity_maps_out[ yi * width + xi + pixel_position_dispartity_map ] = 0;
					}
				}
				else
				{
					for(i = 0; i < disparities; i++)
					{
						pixel_position_dispartity_map =  width*height*i;
						previous_output[i] = disparity_maps_out[ yi * width + xi + pixel_position_dispartity_map ] = disparity_maps[ y * width + x + pixel_position_dispartity_map ];
					}
				}
			}
					
			previous_xi = xi;
			previous_yi = yi;
			
			#endif
		}
	}
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
						int *Internal_NeuronSynapsesCoordinate_X, int *Internal_NeuronSynapsesCoordinate_Y,
						int ImageWidth, int ImageHeight)
{
	int synapse=0;
	int x,y;
	int N=neuronSynapses.numSynapses;
	for(synapse=0;synapse<N;synapse++)
	{
		x = (int)((float)center_x+neuronSynapses.grx[synapse]);
		y = (int)((float)center_y+neuronSynapses.gry[synapse]);
		x = x > 0 ? x : 0;
		y = y > 0 ? y : 0;
		x = x < ImageWidth  ? x : ImageWidth-1;
		y = y < ImageHeight ? y : ImageHeight-1;
		Internal_NeuronSynapsesCoordinate_X[synapse] = x;
		Internal_NeuronSynapsesCoordinate_Y[synapse] = y;
	}
}
void
rgb_to_gray(Image img, unsigned int *gray)
{
	int i, y;
	int pos;
	for(y = 0; y < img.height; y++)
	{
		for(i = stereo_horizontal_ROI_ini; i < stereo_horizontal_ROI_end; i++)
		{
			pos = y * img.width  + i;
			gray[pos] = 0.299f * img.data[3 * pos] + 0.587f * img.data[3 * pos + 1] + 0.114f * img.data[3 * pos + 2];
			//gray[i] = PIXEL(img.data[j], img.data[j + 1], img.data[j + 2]);

		}
	}
}

void
bi_dimention_gausian_disparity_filter(unsigned int *gray, int width, int heigth)
{
	int i, y;
	int n = stereo_horizontal_ROI_end - stereo_horizontal_ROI_ini;
	int aux;
	float mean, sd, sum, lt;
	for(y = 0; y < heigth; y++)
	{

		sum = 0.0;
		for(i = stereo_horizontal_ROI_ini; i < stereo_horizontal_ROI_end; i++)
		{
			sum += gray[y * width  + i];
		}
		mean = line_pixels_mean[y] = sum/n;

		sum = 0.0;
		for(i = stereo_horizontal_ROI_ini; i < stereo_horizontal_ROI_end; i++)
		{
			aux =gray[y * width  + i];
			sd = aux - mean;
			sd *=sd;
			sum += sd;
		}
		sd=line_pixels_standard_deviation[y]=sqrt(sum/(n-1));
		lt=line_target[y]=GAUSSIAN(mean+sd, mean, sd);

		for(i = stereo_horizontal_ROI_ini; i < stereo_horizontal_ROI_end; i++)
		{
			aux = gray[y * width  + i];
			pixels_targets[y * width  + i] = (lt > GAUSSIAN(aux, mean, sd)) ? 1 : 0; ;
		}
	}
}

void CalcBitPatternMinchintonCell(Image img, unsigned int *gray_image ,int *Internal_NeuronSynapsesCoordinate_X,int *Internal_NeuronSynapsesCoordinate_Y, unsigned int *bitPattern, int numSynapses, int patternSize)
{
	int synapse;
	unsigned int pixel0=0;
	unsigned int pixelxi;
	unsigned int pixelxi1;
	int N=numSynapses-1;
	int shift;
	int current_bit_pattern_group;

	int x=Internal_NeuronSynapsesCoordinate_X[0];
	int y=Internal_NeuronSynapsesCoordinate_Y[0];


	memset(bitPattern,0,sizeof(int)*patternSize);
	//image_index = img.channels*(y*img.width+x);
	pixelxi = pixel0 = gray_image[y*img.width+x]; //0.299f * img.data[image_index] + 0.587f * img.data[image_index+1] + 0.114f * img.data[image_index+2];

//	pixelxi=pixel0=PIXEL(img.data[img.channels*(y*img.width+x)], img.data[img.channels*(y*img.width+x)+1], img.data[img.channels*(y*img.width+x)+2]);

	for(synapse=0;synapse<N;synapse++)
	{
		shift=synapse % PATTERN_UNIT_SIZE;
		current_bit_pattern_group=synapse / PATTERN_UNIT_SIZE;

		x=Internal_NeuronSynapsesCoordinate_X[synapse];
		y=Internal_NeuronSynapsesCoordinate_Y[synapse];

		//image_index = img.channels*(y*img.width+x);
		pixelxi1  = gray_image[y*img.width+x]; //0.299f * img.data[image_index] + 0.587f * img.data[image_index+1] + 0.114f * img.data[image_index+2];

//		pixelxi1=PIXEL(img.data[img.channels*(y*img.width+x)], img.data[img.channels*(y*img.width+x)+1], img.data[img.channels*(y*img.width+x)+2]);

		bitPattern[current_bit_pattern_group] |= ((pixelxi > pixelxi1) ? 1 : 0) << shift;
		pixelxi = pixelxi1;
	}

	// Minchington: Última sinapse
	shift=synapse % PATTERN_UNIT_SIZE;
	current_bit_pattern_group=synapse / PATTERN_UNIT_SIZE;

	x=Internal_NeuronSynapsesCoordinate_X[synapse];
	y=Internal_NeuronSynapsesCoordinate_Y[synapse];

	pixelxi = gray_image[y*img.width+x]; //0.299f * img.data[image_index] + 0.587f * img.data[image_index+1] + 0.114f * img.data[image_index+2];

//	pixelxi1=PIXEL(img.data[img.channels*(y*img.width+x)], img.data[img.channels*(y*img.width+x)+1], img.data[img.channels*(y*img.width+x)+2]);

	bitPattern[current_bit_pattern_group] |= ((pixelxi > pixel0) ? 1 : 0) << shift;
}

void WNNTrain(Image Imgleft,Image ImgRight, int MAX_Diparity, vg_ram_wnn stereo_wnn, int *Internal_NeuronSynapsesCoordinate_X, int *Internal_NeuronSynapsesCoordinate_Y,int image_line)
{
	int  neuron;	//Neuron counter must be treated as an integer value due to loop conditions
	unsigned int *disparityBitPattern;

	for (neuron = stereo_horizontal_ROI_ini; neuron < stereo_horizontal_ROI_end; neuron++)
	{
		disparityBitPattern = &stereo_wnn.pattern[neuron*stereo_wnn.patternSize];
		CalcBitPatternMinchintonCell(Imgleft, gray_image_left,(Internal_NeuronSynapsesCoordinate_X + neuron*stereo_wnn.numSynapses) ,(Internal_NeuronSynapsesCoordinate_Y + neuron*stereo_wnn.numSynapses) , disparityBitPattern, stereo_wnn.numSynapses,stereo_wnn.patternSize);
	}
}

void BackwardsWNNTrain(Image Imgleft,Image ImgRight, int MAX_Diparity, vg_ram_wnn stereo_wnn, int *Internal_NeuronSynapsesCoordinate_X, int *Internal_NeuronSynapsesCoordinate_Y,int image_line)
{
	int  neuron;	//Neuron counter must be treated as an integer value due to loop conditions
	unsigned int *disparityBitPattern;

	for (neuron = stereo_wnn.numNeurons -1   ; neuron >= 0; neuron--)
	{
		disparityBitPattern = &stereo_wnn.pattern[neuron*stereo_wnn.patternSize];
		CalcBitPatternMinchintonCell(ImgRight, gray_image_right,(Internal_NeuronSynapsesCoordinate_X + neuron*stereo_wnn.numSynapses) ,(Internal_NeuronSynapsesCoordinate_Y + neuron*stereo_wnn.numSynapses) , disparityBitPattern, stereo_wnn.numSynapses,stereo_wnn.patternSize);
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
HammingDistance(unsigned int *bit_pattern1, unsigned int *bit_pattern2, int patternSize)
{
	unsigned int i;
	unsigned int bit_difference=0;
	unsigned int hamming_distance;

	hamming_distance = 0;
	for (i = 0; i < patternSize; i++)
	{
		bit_difference = bit_pattern1[i] ^ bit_pattern2[i];

		hamming_distance += bitcount(bit_difference);
		//		hamming_distance += __builtin_popcount(bit_difference);
	}
	return hamming_distance;
}

unsigned int*
FindNearestPatterns(unsigned int *bitPattern, unsigned int *neuronPattern,unsigned int *HamDist, unsigned short *disps, int MAX_Diparity, int patternSize)
{
	int disparity;

	for (disparity = 0; disparity < MAX_Diparity; disparity++)
	{

		HamDist[disparity] = HammingDistance(&neuronPattern[disparity*patternSize], bitPattern,patternSize);
		disps[disparity]=disparity;

	}
	return HamDist;
}

unsigned int*
BackwardsFindNearestPatterns(unsigned int *bitPattern, unsigned int *neuronPattern,unsigned int *HamDist, unsigned short *disps, int MAX_Diparity, int patternSize)
{
	int disparity;
	int i=0;

	for (i = MAX_Diparity-1 , disparity = 0; disparity < MAX_Diparity; disparity++, i--)
	{

		HamDist[i] = HammingDistance(&neuronPattern[i*patternSize], bitPattern,patternSize);
		disps[i]=disparity;

	}
	return HamDist;
}

void SelectDisparity(DisparityMap DispMap,unsigned int *HamDist, unsigned short *disps, int MAX_Disparity, int pixels, int line)
{
	int i,j;
	int bestpos=0;
	int bestHamDist=3000;
	unsigned char *layerDisp;
	int imgSize=DispMap.width*DispMap.height;

	for(i=0;i<DispMap.numMaps;i++)
	{
		bestpos=0;
		bestHamDist=3000;
		layerDisp=&DispMap.disps[i*imgSize+line*DispMap.width];
		for(j=i;j<MAX_Disparity;j++)
		{
			int aux=HamDist[j];
			if(bestHamDist>aux)
			{
				bestHamDist=aux;
				bestpos=j;
			}
		}
		layerDisp[pixels] = disps[bestpos];
		disps[bestpos]=disps[i];
		HamDist[bestpos]=HamDist[i];
	}
}


void allocVgRamWNN(int patternSize, int numNeurons, int numSynapses, int numNeuronsLayer, vg_ram_wnn *stereo_wnn_out)
{
	stereo_wnn_out->numNeurons=numNeurons;
	stereo_wnn_out->numSynapses=numSynapses;
	stereo_wnn_out->numNeuronsLayer=numNeuronsLayer;
	stereo_wnn_out->patternSize=patternSize;

	stereo_wnn_out->pattern=(unsigned int*)calloc(patternSize*numNeurons,sizeof(unsigned int));
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

void WNNDispatyLine(Image ImgLeft,Image ImgRight, DisparityMap DispMap, int MAX_Disparity, vg_ram_wnn stereo_wnn, int *Internal_NeuronSynapsesCoordinate_X, int *Internal_NeuronSynapsesCoordinate_Y,unsigned int *bitPattern,int image_line,unsigned int *HamDist, unsigned short *disps)
{

	int neuron, center_y;
	center_y=image_line;
	int MAX_size,aux;

	for (neuron = stereo_horizontal_ROI_ini; neuron < stereo_horizontal_ROI_end; neuron+=inc_stereo_width)
	{
		CalcBitPatternMinchintonCell(ImgRight, gray_image_right,(Internal_NeuronSynapsesCoordinate_X + neuron*stereo_wnn.numSynapses) ,(Internal_NeuronSynapsesCoordinate_Y + neuron*stereo_wnn.numSynapses) , bitPattern, stereo_wnn.numSynapses,stereo_wnn.patternSize );
		aux=ImgLeft.width-neuron;
		MAX_size = aux > MAX_Disparity ? MAX_Disparity : aux;
		FindNearestPatterns(bitPattern, &stereo_wnn.pattern[neuron*stereo_wnn.patternSize], HamDist, disps,MAX_size, stereo_wnn.patternSize);
		SelectDisparity(DispMap,HamDist,disps,MAX_Disparity,neuron,center_y);
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
	int *NeuronSynapsesAux_X,*NeuronSynapsesAux_Y;
	
	int h, w, hi, wi, xi, yi, x_center, y_center;
	//double log_factor = 1.0;
	double log_factor = 2.0;
	double correction;
	
	allocSynapsesDistribution(&neuronSynapses, numSynapses);
	CalcGuassianSynapsesDistribution(neuronSynapses);
	
	wi = width;		// Input Image Width
	hi = height;		// Input Image Height 
	w = width;		// Neuron Layer Width
	h = height;		// Neuron Layer Height
	x_center = wi/2 ;	// Neuron Layer "x" center point
	y_center = hi/2 ;	// Neuron Layer "y" center point
	correction = (double) h / (double) (h -1);	// log-polar transform correction factor
	
	for(y=0;y<height;y++)
	{
		NeuronSynapsesAux_X = &NeuronSynapsesCoordinate_X[y * numSynapses * width];
		NeuronSynapsesAux_Y = &NeuronSynapsesCoordinate_Y[y * numSynapses * width];
		for(x=0;x<width;x++)
		{
			//LogPolarMapping(&xi, &yi, wi, hi, x, y, w, h, x_center, y_center, correction, log_factor);								// log-polar center position
			//CalcNeuronSynapsesCoordinateOnImage(xi, yi, neuronSynapses, &NeuronSynapsesAux_X[x*numSynapses], &NeuronSynapsesAux_Y[x*numSynapses], width, height);	// neuron position
			//printf("%d %d\n",xi,yi);
			CalcNeuronSynapsesCoordinateOnImage(x, y, neuronSynapses, &NeuronSynapsesAux_X[x*numSynapses], &NeuronSynapsesAux_Y[x*numSynapses], width, height);
		}
	}
	freeSynapsesDistribution(&neuronSynapses);
}

//Initialization of VG-RAM WNN variables
void WNNInitialize(int height, int width, int MAX_Diparity,int numMaps,int numOfThreads, int numSynapses, float gaussian_radius_param, int horizontal_ROI_ini, int horizontal_ROI_end)
{
	int 	i;
	int	threadNum = numOfThreads;

	patternSize = numSynapses/PATTERN_UNIT_SIZE;
	patternSize += numSynapses%PATTERN_UNIT_SIZE ? 1: 0;

	gaussian_radius = gaussian_radius_param;

	allocDisparityMap(&disp,width, height, numMaps);
	allocDisparityMap(&disp_out,width, height, numMaps);

	stereo_wnn = (vg_ram_wnn*)calloc(threadNum,sizeof(vg_ram_wnn));
	bitPattern = (unsigned int **)calloc(threadNum,sizeof(unsigned int*));
	HamDist = (unsigned int**)calloc(threadNum,sizeof(unsigned int*));
	disps = (unsigned short**)calloc(threadNum,sizeof(unsigned short*));

	stereo_horizontal_ROI_ini = horizontal_ROI_ini;
	stereo_horizontal_ROI_end = horizontal_ROI_end;

	for(i=0;i<threadNum;i++)
	{
		allocVgRamWNN(patternSize, width, numSynapses, MAX_Diparity, &stereo_wnn[i]);
		bitPattern[i] = (unsigned int *)calloc(patternSize,sizeof(unsigned int));
		HamDist[i] = (unsigned int*)calloc(MAX_Diparity,sizeof(unsigned int));
		disps[i] = (unsigned short*)calloc(MAX_Diparity,sizeof(unsigned short));
	}

	NeuronSynapsesCoordinate_X = (int *)calloc(numSynapses*width*height,sizeof(int));
	NeuronSynapsesCoordinate_Y = (int *)calloc(numSynapses*width*height,sizeof(int));
	gray_image_left = (unsigned int *)calloc(width * height,sizeof(unsigned int));
	gray_image_right = (unsigned int *)calloc(width * height,sizeof(unsigned int));
	pixels_targets = (float *) calloc(width * height,sizeof(float));

	//Set synapses coordinates
	SetSynapsesCoodinate(width,height,numSynapses);

}

void WNNDisparity(Image ImgLeft, Image ImgRight, DisparityMap DispMap, int MAX_Diparity, int numSynapses, int numberOfThreads)
{
	int	line;
	int	threadIdx;

	#pragma omp parallel num_threads(numberOfThreads)  default(none) private(line,threadIdx) shared(stereo_wnn,NeuronSynapsesCoordinate_X,NeuronSynapsesCoordinate_Y,ImgLeft, ImgRight, DispMap, MAX_Diparity, numSynapses, numberOfThreads,patternSize,disps,bitPattern,HamDist,inc_stereo_height)
	{
		threadIdx = omp_get_thread_num();

		#pragma omp for
		for(line = 0; line < ImgLeft.height; line += inc_stereo_height)
		{
			//Foreward train/test -> left to right
			WNNTrain(ImgLeft, ImgRight, MAX_Diparity, stereo_wnn[threadIdx], &NeuronSynapsesCoordinate_X [line*stereo_wnn[threadIdx].numSynapses*ImgLeft.width], &NeuronSynapsesCoordinate_Y [line*stereo_wnn[threadIdx].numSynapses*ImgLeft.width], line);
			WNNDispatyLine(ImgLeft, ImgRight, DispMap, MAX_Diparity, stereo_wnn[threadIdx], &NeuronSynapsesCoordinate_X[line*stereo_wnn[threadIdx].numSynapses*ImgLeft.width],  &NeuronSynapsesCoordinate_Y [line*stereo_wnn[threadIdx].numSynapses*ImgLeft.width], bitPattern[threadIdx], line, HamDist[threadIdx], disps[threadIdx]);
		}
	}
}

void disparityVgRamWNN(unsigned char*image_left,unsigned char*image_right, int stereo_height, int stereo_width, int stereo_disparity, unsigned char *disparity , int numSynapses,int numOfThreads, int iteration, int inc_width, int inc_height)
{
	Image ImgLeft,ImgRight;

	ImgLeft.channels=ImgRight.channels=3;
	ImgLeft.width=ImgRight.width=stereo_width;
	ImgLeft.height=ImgRight.height=stereo_height;

	ImgLeft.data=image_left;
	ImgRight.data=image_right;

	rgb_to_gray(ImgLeft,gray_image_left);
	rgb_to_gray(ImgRight,gray_image_right);

	inc_stereo_height = inc_height;
	inc_stereo_width = inc_width;

	WNNDisparity(ImgLeft, ImgRight, disp, stereo_disparity, numSynapses,numOfThreads);

	//DisparityMapInverseLogPolar(disp);
	WinnerTakesItAll(disp,iteration);

	//mean_filter(disp, 2);

	//median_filter(disp, 1);


	for (int y = 0; y < stereo_height * stereo_width; y ++)
	{
		disparity[y]=(unsigned char)(disp_out.disps[y]);
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
		free(disps[i]);
		free(HamDist[i]);
		freeVgRamWNN(&stereo_wnn[i]);
	}
	free(stereo_wnn);
}

void WinnerTakesItAll(DisparityMap disps, int iteration)
{
	int 	l,k,i,j;
	int	opinion;
	int	highest_opinion_index;

	unsigned char*	disp_pixel;
	unsigned char*	central_pixel;
	int 	aux;

	for(l=0; l<iteration; l++)
	{
		//printf("iter: %d\n",l);
 		#pragma omp parallel for default(none) private(i,j,k,opinion,disp_pixel,highest_opinion_index,central_pixel,aux) shared(disps, inc_stereo_height, inc_stereo_width,stereo_horizontal_ROI_ini,stereo_horizontal_ROI_end)
		for(i = 0; i<disps.height; i += inc_stereo_height)
		{
			for(j = stereo_horizontal_ROI_ini; j<stereo_horizontal_ROI_end; j += inc_stereo_width)
			{
				disp_pixel = disps.disps + (i * disps.width + j);
				//highest_opinion = 0;
				highest_opinion_index = 0;

				for(k = 0; k<disps.numMaps; k++)
				{
					opinion = 0;
					central_pixel = CENTRAL(k, disps.width, disps.height, disp_pixel);

					//UP
					if(!(i==0))
						if(*central_pixel == *UP(disps.width,disp_pixel))
							opinion++;

					//UP_RIGHT
					if(!(i==0)&&!(j==disps.width-inc_stereo_width))
						if(*central_pixel == *UP_RIGHT(disps.width,disp_pixel))
							opinion++;

					//RIGHT
					if(!(j==disps.width-inc_stereo_width))
						if(*central_pixel == *RIGHT(disps.width,disp_pixel))
							opinion++;

					//LOW_RIGHT
					if(!(i==disps.height-inc_stereo_height)&&!(j==disps.width-inc_stereo_width))
						if(*central_pixel == *LOW_RIGHT(disps.width,disp_pixel))
							opinion++;

					//LOW
					if(!(i==disps.height-inc_stereo_height))
						if(*central_pixel == *LOW(disps.width,disp_pixel))
							opinion++;

					//LOW_LEFT
					if(!(i==disps.height-inc_stereo_height)&&!(j==0))
						if(*central_pixel == *LOW_LEFT(disps.width,disp_pixel))
							opinion++;

					//LEFT
					if(!(j==0))
						if(*central_pixel == *LEFT(disps.width,disp_pixel))
							opinion++;

					//UP_LEFT
					if(!(i==0)&&!(j==0))
						if(*central_pixel == *UP_LEFT(disps.width,disp_pixel))
							opinion++;

					// obtain the new highest opinion and the new highest opinion index
					if(opinion > 4)
					{
						//highest_opinion = opinion;
						highest_opinion_index = k;
					}
				}

				// if highest_opinion_index != 0
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

void mean_filter(DisparityMap disps, int iteration)
{
	int 	l,i,j;
	int	opinion;
	int	highest_opinion_index;

	unsigned char*	disp_pixel;
	unsigned char 	aux;

	for(l=0; l<iteration; l++)
	{
		//printf("iter: %d\n",l);
 		#pragma omp parallel for default(none) private(i,j,opinion,disp_pixel,highest_opinion_index,aux) shared(disps, inc_stereo_height, inc_stereo_width, stereo_horizontal_ROI_ini,stereo_horizontal_ROI_end)
		for(i = 0; i<disps.height; i += inc_stereo_height)
		{
			for(j = stereo_horizontal_ROI_ini; j<stereo_horizontal_ROI_end; j += inc_stereo_width)
			{
				disp_pixel = disps.disps + (i * disps.width + j);
				//highest_opinion = 0;
				highest_opinion_index = 0;
				aux = 0;
				opinion = 1;

				aux += *CENTRAL(0, disps.width, disps.height, disp_pixel);

				//UP
				if(!(i==0))
				{
					aux += *UP(disps.width,disp_pixel);
					opinion++;
				}


				//UP_RIGHT
				if(!(i==0)&&!(j==disps.width-inc_stereo_width))
				{
					aux += *UP_RIGHT(disps.width,disp_pixel);
					opinion++;
				}

				//RIGHT
				if(!(j==disps.width-inc_stereo_width))
				{
					aux += *RIGHT(disps.width,disp_pixel);
					opinion++;
				}

				//LOW_RIGHT
				if(!(i==disps.height-inc_stereo_height)&&!(j==disps.width-inc_stereo_width))
				{
					aux += *LOW_RIGHT(disps.width,disp_pixel);
					opinion++;
				}

				//LOW
				if(!(i==disps.height-inc_stereo_height))
				{
					aux += *LOW(disps.width,disp_pixel);
					opinion++;
				}

				//LOW_LEFT
				if(!(i==disps.height-inc_stereo_height)&&!(j==0))
				{
					aux += *LOW_LEFT(disps.width,disp_pixel);
					opinion++;
				}

				//LEFT
				if(!(j==0))
				{
					aux += *LEFT(disps.width,disp_pixel);
					opinion++;
				}

				//UP_LEFT
				if(!(i==0)&&!(j==0))
				{
					aux += *UP_LEFT(disps.width,disp_pixel);
					opinion++;
				}
				*disp_pixel = aux / opinion;
			}
		}
	}
}

void median_filter(DisparityMap disps, int iteration)
{
	int 	i,j,k,l,m;
	int	opinion;


	unsigned char*	disp_pixel;

	unsigned char 	aux;

	for(m=0;m<iteration;m++)
	{
	#pragma omp parallel  default(none) private(i,j,l,k,opinion,disp_pixel,aux) shared(disps, inc_stereo_height, inc_stereo_width, stereo_horizontal_ROI_ini,stereo_horizontal_ROI_end)
	{
		unsigned char pixels[9];
		#pragma omp for
		for(i = 0; i<disps.height; i += inc_stereo_height)
		{
			for(j = stereo_horizontal_ROI_ini; j<stereo_horizontal_ROI_end; j += inc_stereo_width)
			{
				disp_pixel = disps.disps + (i * disps.width + j);

				aux = 0;
				opinion = 0;
				memset(pixels,0,9);

				pixels[opinion] = *CENTRAL(0, disps.width, disps.height, disp_pixel);
				opinion++;

				//UP
				if(!(i==0))
				{
					pixels[opinion] = *UP(disps.width,disp_pixel);
					opinion++;
				}


				//UP_RIGHT
				if(!(i==0)&&!(j==disps.width-inc_stereo_width))
				{
					pixels[opinion] = *UP_RIGHT(disps.width,disp_pixel);
					opinion++;
				}

				//RIGHT
				if(!(j==disps.width-inc_stereo_width))
				{
					pixels[opinion] = *RIGHT(disps.width,disp_pixel);
					opinion++;
				}

				//LOW_RIGHT
				if(!(i==disps.height-inc_stereo_height)&&!(j==disps.width-inc_stereo_width))
				{
					pixels[opinion] = *LOW_RIGHT(disps.width,disp_pixel);
					opinion++;
				}

				//LOW
				if(!(i==disps.height-inc_stereo_height))
				{
					pixels[opinion] = *LOW(disps.width,disp_pixel);
					opinion++;
				}

				//LOW_LEFT
				if(!(i==disps.height-inc_stereo_height)&&!(j==0))
				{
					pixels[opinion] = *LOW_LEFT(disps.width,disp_pixel);
					opinion++;
				}

				//LEFT
				if(!(j==0))
				{
					pixels[opinion] = *LEFT(disps.width,disp_pixel);
					opinion++;
				}

				//UP_LEFT
				if(!(i==0)&&!(j==0))
				{
					pixels[opinion] = *UP_LEFT(disps.width,disp_pixel);
					opinion++;
				}
				for(l=0;l<5;l++)
				{
					aux = pixels[l];
					for(k=l+1;k<9;k++)
					{
						if(aux < pixels[k])
						{
							opinion = k;
							aux = pixels[k];
						}
					}

					pixels[opinion]=pixels[l];
					pixels[l]=aux;
				}

				*disp_pixel = pixels[5];
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

	disp->disps=(unsigned char *)calloc(w*h*numOfDisparity,sizeof(unsigned char ));
}

