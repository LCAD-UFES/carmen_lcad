#include <opencv/cv.h>
#include "voice_recognition_filters.h"
#include "voice_recognition_user_functions.h"
#include "voice_recognition_utils.h"

void equalize_image(IplImage *img)
{
	IplImage *b = NULL;
	IplImage *g = NULL;
	IplImage *r = NULL;

	if (img->nChannels == 1)
	{
		cvEqualizeHist(img, img);
	}
	else if (img->nChannels == 3)
	{
		b = cvCreateImage(cvGetSize(img), 8, 1);
		g = cvCreateImage(cvGetSize(img), 8, 1);
		r = cvCreateImage(cvGetSize(img), 8, 1);

		cvSplit(img, b, g, r, NULL);

		cvEqualizeHist(b,b);
		cvEqualizeHist(g,g);
		cvEqualizeHist(r,r);

		cvMerge(b,g,r,NULL,img);

		cvReleaseImage(&b);
		cvReleaseImage(&g);
		cvReleaseImage(&r);
	}
}

void
rotate_image(IplImage *img, double angle, double scale)
{
	CvSize img_size = cvGetSize(img);
	IplImage *tmp = cvCreateImage(img_size,img->depth, img->nChannels);
	CvMat *rotate = cvCreateMat(2,3,CV_32F);
	CvPoint2D32f center = cvPoint2D32f(
			((double)img_size.width)/2.0,
			((double)img_size.height)/2.0);
	cv2DRotationMatrix(center, angle, scale, rotate);
	cvWarpAffine(img, tmp, rotate, CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS, cvScalarAll(0) );
	cvCopyImage(tmp, img);
	cvReleaseImage(&tmp);
}

void
translate_image(IplImage *img, int dx, int dy)
{
	CvRect roi = cvGetImageROI(img);
	roi.x += dx;
	roi.y += dy;
	cvResetImageROI(img);
	cvSetImageROI(img, roi);
}


void 
reshape_opencv_filter (FILTER_DESC *filter_desc)
{
	NEURON_LAYER *nl_output = NULL;
	NEURON_LAYER *nl_input = NULL;
	IplImage *image_input = NULL;
	IplImage *image_output = NULL;
	CvRect roi_input;
	int wi, hi, wo, ho;
#define	LOW_FREQUENCY_BANDS_REMOVED	5
#define	AMOUNT_OF_INPUT_ANALYSED	INPUT_WIDTH / 4

	// Gets the Input Neuron Layer
	nl_input = filter_desc->neuron_layer_list->neuron_layer;

	// Gets the Filter Output
	nl_output = filter_desc->output;

	wi = nl_input->dimentions.x;
	hi = nl_input->dimentions.y;

	wo = nl_output->dimentions.x;
	ho = nl_output->dimentions.y;

	if (g_time_shift < 0)
		g_time_shift = 0;
	
	if (g_time_shift > (wi - AMOUNT_OF_INPUT_ANALYSED))
		g_time_shift = wi - AMOUNT_OF_INPUT_ANALYSED;
		
	roi_input = cvRect (g_time_shift, LOW_FREQUENCY_BANDS_REMOVED, AMOUNT_OF_INPUT_ANALYSED, INPUT_HEIGHT);

	image_input = cvCreateImage(cvSize(wi, hi), 8, nl_input->output_type == COLOR ? 3 : 1);
	image_output = cvCreateImage(cvSize(wo, ho), 8, nl_input->output_type == COLOR ? 3 : 1);

	copy_neuron_layer_to_image(image_input, &roi_input, nl_input);

	cvSetImageROI(image_input, roi_input);

	cvResize (image_input, image_output, CV_INTER_LINEAR);

	copy_image_to_neuron_layer(nl_output, NULL, image_output);

	cvReleaseImage(&image_input);
	cvReleaseImage(&image_output);
}

/* Inlined mask functions - Based on mae.h macros */

inline	unsigned int
red_mask(unsigned int pixel)
{
	return(RED(pixel));
}

inline	unsigned int
green_mask(unsigned int pixel)
{
	return(GREEN(pixel));
}

inline	unsigned int
blue_mask(unsigned int pixel)
{
	return(BLUE(pixel));
}

/*
*********************************************************************************
* Function: red_channel_mask							*
* Description: 			                        			*
* Inputs: 				                  			*
* Output: 				                			*
*********************************************************************************
*/

void red_mask_filter (FILTER_DESC *filter_desc)
{
	PARAM_LIST *p_list = NULL;
	NEURON_LAYER_LIST *n_list = NULL;
	NEURON_LAYER *nl_output = NULL, *nl_input = NULL;
	int nl_number, p_number;
	int xi, yi, wi, hi, wo, ho;
	//int color_channel;
	//unsigned int (*mask_func)(unsigned int);

	// Checks the Neuron Layers Number
	for (nl_number = 0, n_list = filter_desc->neuron_layer_list; n_list != NULL; n_list = n_list->next, nl_number++)
            	;

	// Checks the Parameters Number
	for (p_number = 0, p_list = filter_desc->filter_params; p_list != NULL; p_list = p_list->next, p_number++)
            	;

	if (p_number != 1)
	{
		Erro ("Error: Wrong number of parameters. The red_mask_filter must have no parameters.", "", "");
		return;
	}

	// Gets the Input Neuron Layer
	nl_input = filter_desc->neuron_layer_list->neuron_layer;

	// Gets the Filter Output 
	nl_output = filter_desc->output;
	
	// Input-Output width
	wi = nl_input->dimentions.x;
	hi = nl_input->dimentions.y;

	wo = nl_output->dimentions.x;
	ho = nl_output->dimentions.y;
	
	if(wi != wo || hi != ho)
	{
		Erro ("Error: Input and output layers on red_channel_mask filter must have the same 2D size.", "", "");
		return;
	}
	
	for (yi = 0; yi < hi; yi++)
		for(xi = 0; xi < wi; xi++)
			nl_output->neuron_vector[xi + yi * wi].output.ival =  RED(nl_input->neuron_vector[xi + yi * wi].output.ival);
}

void green_mask_filter (FILTER_DESC *filter_desc)
{
	PARAM_LIST *p_list = NULL;
	NEURON_LAYER_LIST *n_list = NULL;
	NEURON_LAYER *nl_output = NULL, *nl_input = NULL;
	int nl_number, p_number;
	int xi, yi, wi, hi, wo, ho;
	//int color_channel;
	//unsigned int (*mask_func)(unsigned int);

	// Checks the Neuron Layers Number
	for (nl_number = 0, n_list = filter_desc->neuron_layer_list; n_list != NULL; n_list = n_list->next, nl_number++)
            	;

	// Checks the Parameters Number
	for (p_number = 0, p_list = filter_desc->filter_params; p_list != NULL; p_list = p_list->next, p_number++)
            	;

	if (p_number != 1)
	{
		Erro ("Error: Wrong number of parameters. The green_mask_filter must have no parameters.", "", "");
		return;
	}

	// Gets the Input Neuron Layer
	nl_input = filter_desc->neuron_layer_list->neuron_layer;

	// Gets the Filter Output 
	nl_output = filter_desc->output;
	
	// Input-Output width
	wi = nl_input->dimentions.x;
	hi = nl_input->dimentions.y;

	wo = nl_output->dimentions.x;
	ho = nl_output->dimentions.y;
	
	if(wi != wo || hi != ho)
	{
		Erro ("Error: Input and output layers on green_channel_mask filter must have the same 2D size.", "", "");
		return;
	}
	
	for (yi = 0; yi < hi; yi++)
		for(xi = 0; xi < wi; xi++)
			nl_output->neuron_vector[xi + yi * wi].output.ival =  GREEN(nl_input->neuron_vector[xi + yi * wi].output.ival);
}

void blue_mask_filter (FILTER_DESC *filter_desc)
{
	PARAM_LIST *p_list = NULL;
	NEURON_LAYER_LIST *n_list = NULL;
	NEURON_LAYER *nl_output = NULL, *nl_input = NULL;
	int nl_number, p_number;
	int xi, yi, wi, hi, wo, ho;
	//int color_channel;
	//unsigned int (*mask_func)(unsigned int);

	// Checks the Neuron Layers Number
	for (nl_number = 0, n_list = filter_desc->neuron_layer_list; n_list != NULL; n_list = n_list->next, nl_number++)
            	;

	// Checks the Parameters Number
	for (p_number = 0, p_list = filter_desc->filter_params; p_list != NULL; p_list = p_list->next, p_number++)
            	;

	if (p_number != 1)
	{
		Erro ("Error: Wrong number of parameters. The blue_mask_filter must have no parameters.", "", "");
		return;
	}

	// Gets the Input Neuron Layer
	nl_input = filter_desc->neuron_layer_list->neuron_layer;

	// Gets the Filter Output 
	nl_output = filter_desc->output;
	
	// Input-Output width
	wi = nl_input->dimentions.x;
	hi = nl_input->dimentions.y;

	wo = nl_output->dimentions.x;
	ho = nl_output->dimentions.y;
	
	if(wi != wo || hi != ho)
	{
		Erro ("Error: Input and output layers on blue_channel_mask filter must have the same 2D size.", "", "");
		return;
	}
	
	for (yi = 0; yi < hi; yi++)
		for(xi = 0; xi < wi; xi++)
			nl_output->neuron_vector[xi + yi * wi].output.ival =  BLUE(nl_input->neuron_vector[xi + yi * wi].output.ival);
}

int
max_value(int r, int g, int b)
{
	if (r > g)
	{
		if (r > b)
			return (r);
		else 
			return (b);
	}
	else
	{
		if (g > b)
			return (g);
		else 
			return (b);
	}
}

int
min_value(int r, int g, int b)
{
	if (r < g)
	{
		if (r < b)
			return (r);
		else 
			return (b);
	}
	else
	{
		if (g < b)
			return (g);
		else 
			return (b);
	}
}

void 
hsv_v_filter(FILTER_DESC *filter_desc)
{
	// http://en.wikipedia.org/wiki/HSL_and_HSV
	// http://cs.haifa.ac.il/hagit/courses/ist/Lectures/Demos/ColorApplet2/t_convert.html
	
	PARAM_LIST *p_list = NULL;
	NEURON_LAYER_LIST *n_list = NULL;
	NEURON_LAYER *nl_output = NULL, *nl_input = NULL;
	int nl_number, p_number;
	int xo, yo, wi, hi, wo, ho;
	int r, g, b;

	// Checks the Neuron Layers Number
	for (nl_number = 0, n_list = filter_desc->neuron_layer_list; n_list != NULL; n_list = n_list->next, nl_number++)
            	;

	// Checks the Parameters Number
	for (p_number = 0, p_list = filter_desc->filter_params; p_list != NULL; p_list = p_list->next, p_number++)
            	;

	if (p_number != 1)
	{
		Erro ("Error: Wrong number of parameters. The blue_mask_filter must have no parameters.", "", "");
		return;
	}

	// Gets the Input Neuron Layer
	nl_input = filter_desc->neuron_layer_list->neuron_layer;

	// Gets the Filter Output 
	nl_output = filter_desc->output;
	
	// Input-Output width
	wi = nl_input->dimentions.x;
	hi = nl_input->dimentions.y;

	wo = nl_output->dimentions.x;
	ho = nl_output->dimentions.y;
	
	if (wi != wo || hi != ho)
	{
		Erro ("Error: Input and output layers on blue_channel_mask filter must have the same 2D size.", "", "");
		return;
	}
	
	for (yo = 0; yo < ho; yo++)
	{
		for (xo = 0; xo < wo; xo++)
		{
			r = RED(nl_input->neuron_vector[xo + yo * wo].output.ival);
			g = GREEN(nl_input->neuron_vector[xo + yo * wo].output.ival);
			b = BLUE(nl_input->neuron_vector[xo + yo * wo].output.ival);
			
			nl_output->neuron_vector[xo + yo * wo].output.ival = max_value(r, g, b);
		}
	}
}

void 
hsv_s_filter(FILTER_DESC *filter_desc)
{
	// http://en.wikipedia.org/wiki/HSL_and_HSV
	// http://cs.haifa.ac.il/hagit/courses/ist/Lectures/Demos/ColorApplet2/t_convert.html
	
	PARAM_LIST *p_list = NULL;
	NEURON_LAYER_LIST *n_list = NULL;
	NEURON_LAYER *nl_output = NULL, *nl_input = NULL;
	int nl_number, p_number;
	int xo, yo, wi, hi, wo, ho;
	int r, g, b, max, min;

	// Checks the Neuron Layers Number
	for (nl_number = 0, n_list = filter_desc->neuron_layer_list; n_list != NULL; n_list = n_list->next, nl_number++)
            	;

	// Checks the Parameters Number
	for (p_number = 0, p_list = filter_desc->filter_params; p_list != NULL; p_list = p_list->next, p_number++)
            	;

	if (p_number != 1)
	{
		Erro ("Error: Wrong number of parameters. The blue_mask_filter must have no parameters.", "", "");
		return;
	}

	// Gets the Input Neuron Layer
	nl_input = filter_desc->neuron_layer_list->neuron_layer;

	// Gets the Filter Output 
	nl_output = filter_desc->output;
	
	// Input-Output width
	wi = nl_input->dimentions.x;
	hi = nl_input->dimentions.y;

	wo = nl_output->dimentions.x;
	ho = nl_output->dimentions.y;
	
	if (wi != wo || hi != ho)
	{
		Erro ("Error: Input and output layers on blue_channel_mask filter must have the same 2D size.", "", "");
		return;
	}
	
	for (yo = 0; yo < ho; yo++)
	{
		for (xo = 0; xo < wo; xo++)
		{
			r = RED(nl_input->neuron_vector[xo + yo * wo].output.ival);
			g = GREEN(nl_input->neuron_vector[xo + yo * wo].output.ival);
			b = BLUE(nl_input->neuron_vector[xo + yo * wo].output.ival);
			max = max_value(r, g, b);
			if (max != 0)
			{
				min = min_value(r, g, b);
				nl_output->neuron_vector[xo + yo * wo].output.ival = (int) (255.0 * ((double) (max - min) / (double) max));
			}
			else
				nl_output->neuron_vector[xo + yo * wo].output.ival = 0;
		}
	}
}

void 
hsv_h_filter(FILTER_DESC *filter_desc)
{
	// http://en.wikipedia.org/wiki/HSL_and_HSV
	// http://cs.haifa.ac.il/hagit/courses/ist/Lectures/Demos/ColorApplet2/t_convert.html
	
	PARAM_LIST *p_list = NULL;
	NEURON_LAYER_LIST *n_list = NULL;
	NEURON_LAYER *nl_output = NULL, *nl_input = NULL;
	int nl_number, p_number;
	int xo, yo, wi, hi, wo, ho;
	int r, g, b, max, min;
	double h, delta;

	// Checks the Neuron Layers Number
	for (nl_number = 0, n_list = filter_desc->neuron_layer_list; n_list != NULL; n_list = n_list->next, nl_number++)
            	;

	// Checks the Parameters Number
	for (p_number = 0, p_list = filter_desc->filter_params; p_list != NULL; p_list = p_list->next, p_number++)
            	;

	if (p_number != 1)
	{
		Erro ("Error: Wrong number of parameters. The blue_mask_filter must have no parameters.", "", "");
		return;
	}

	// Gets the Input Neuron Layer
	nl_input = filter_desc->neuron_layer_list->neuron_layer;

	// Gets the Filter Output 
	nl_output = filter_desc->output;
	
	// Input-Output width
	wi = nl_input->dimentions.x;
	hi = nl_input->dimentions.y;

	wo = nl_output->dimentions.x;
	ho = nl_output->dimentions.y;
	
	if (wi != wo || hi != ho)
	{
		Erro ("Error: Input and output layers on blue_channel_mask filter must have the same 2D size.", "", "");
		return;
	}
	
	for (yo = 0; yo < ho; yo++)
	{
		for (xo = 0; xo < wo; xo++)
		{
			r = RED(nl_input->neuron_vector[xo + yo * wo].output.ival);
			g = GREEN(nl_input->neuron_vector[xo + yo * wo].output.ival);
			b = BLUE(nl_input->neuron_vector[xo + yo * wo].output.ival);
			max = max_value(r, g, b);
			min = min_value(r, g, b);
			if ((max != 0) && (max != min))
			{
				delta = (double) max - (double) min;
				
				if (r == max)
					h = (double) (g - b) / delta;		// between yellow & magenta
				else if (g == max)
					h = 2.0 + (double) (b - r) / delta;	// between cyan & yellow
				else
					h = 4.0 + (double) (r - g) / delta;	// between magenta & cyan

				h *= 60.0;					// degrees
				if (h < 0.0)
					h += 360.0;

				nl_output->neuron_vector[xo + yo * wo].output.ival = (int) (255.0 * (h / 360.0) + 0.5);
			}
			else
				nl_output->neuron_vector[xo + yo * wo].output.ival = 0;
		}
	}
}

void 
Y_filter(FILTER_DESC *filter_desc)
{
	// http://www.equasys.de/colorconversion.html
	
	PARAM_LIST *p_list = NULL;
	NEURON_LAYER_LIST *n_list = NULL;
	NEURON_LAYER *nl_output = NULL, *nl_input = NULL;
	int nl_number, p_number;
	int xo, yo, wi, hi, wo, ho;
	double r, g, b;

	// Checks the Neuron Layers Number
	for (nl_number = 0, n_list = filter_desc->neuron_layer_list; n_list != NULL; n_list = n_list->next, nl_number++)
            	;

	// Checks the Parameters Number
	for (p_number = 0, p_list = filter_desc->filter_params; p_list != NULL; p_list = p_list->next, p_number++)
            	;

	if (p_number != 1)
	{
		Erro ("Error: Wrong number of parameters. The blue_mask_filter must have no parameters.", "", "");
		return;
	}

	// Gets the Input Neuron Layer
	nl_input = filter_desc->neuron_layer_list->neuron_layer;

	// Gets the Filter Output 
	nl_output = filter_desc->output;
	
	// Input-Output width
	wi = nl_input->dimentions.x;
	hi = nl_input->dimentions.y;

	wo = nl_output->dimentions.x;
	ho = nl_output->dimentions.y;
	
	if (wi != wo || hi != ho)
	{
		Erro ("Error: Input and output layers on blue_channel_mask filter must have the same 2D size.", "", "");
		return;
	}
	
	for (yo = 0; yo < ho; yo++)
	{
		for (xo = 0; xo < wo; xo++)
		{
			r = (double) RED(nl_input->neuron_vector[xo + yo * wo].output.ival);
			g = (double) GREEN(nl_input->neuron_vector[xo + yo * wo].output.ival);
			b = (double) BLUE(nl_input->neuron_vector[xo + yo * wo].output.ival);
			nl_output->neuron_vector[xo + yo * wo].output.ival = (int) (0.299 * r + 0.587 * g + 0.114 * b + 0.0);
		}
	}
}

void 
Cb_filter(FILTER_DESC *filter_desc)
{
	// http://www.equasys.de/colorconversion.html
	
	PARAM_LIST *p_list = NULL;
	NEURON_LAYER_LIST *n_list = NULL;
	NEURON_LAYER *nl_output = NULL, *nl_input = NULL;
	int nl_number, p_number;
	int xo, yo, wi, hi, wo, ho;
	double r, g, b;

	// Checks the Neuron Layers Number
	for (nl_number = 0, n_list = filter_desc->neuron_layer_list; n_list != NULL; n_list = n_list->next, nl_number++)
            	;

	// Checks the Parameters Number
	for (p_number = 0, p_list = filter_desc->filter_params; p_list != NULL; p_list = p_list->next, p_number++)
            	;

	if (p_number != 1)
	{
		Erro ("Error: Wrong number of parameters. The blue_mask_filter must have no parameters.", "", "");
		return;
	}

	// Gets the Input Neuron Layer
	nl_input = filter_desc->neuron_layer_list->neuron_layer;

	// Gets the Filter Output 
	nl_output = filter_desc->output;
	
	// Input-Output width
	wi = nl_input->dimentions.x;
	hi = nl_input->dimentions.y;

	wo = nl_output->dimentions.x;
	ho = nl_output->dimentions.y;
	
	if (wi != wo || hi != ho)
	{
		Erro ("Error: Input and output layers on blue_channel_mask filter must have the same 2D size.", "", "");
		return;
	}
	
	for (yo = 0; yo < ho; yo++)
	{
		for (xo = 0; xo < wo; xo++)
		{
			r = (double) RED(nl_input->neuron_vector[xo + yo * wo].output.ival);
			g = (double) GREEN(nl_input->neuron_vector[xo + yo * wo].output.ival);
			b = (double) BLUE(nl_input->neuron_vector[xo + yo * wo].output.ival);
			nl_output->neuron_vector[xo + yo * wo].output.ival = (int) (- 0.169 * r - 0.331 * g + 0.500 * b + 128.0);
		}
	}
}

void 
Cr_filter(FILTER_DESC *filter_desc)
{
	// http://www.equasys.de/colorconversion.html
	
	PARAM_LIST *p_list = NULL;
	NEURON_LAYER_LIST *n_list = NULL;
	NEURON_LAYER *nl_output = NULL, *nl_input = NULL;
	int nl_number, p_number;
	int xo, yo, wi, hi, wo, ho;
	double r, g, b;

	// Checks the Neuron Layers Number
	for (nl_number = 0, n_list = filter_desc->neuron_layer_list; n_list != NULL; n_list = n_list->next, nl_number++)
            	;

	// Checks the Parameters Number
	for (p_number = 0, p_list = filter_desc->filter_params; p_list != NULL; p_list = p_list->next, p_number++)
            	;

	if (p_number != 1)
	{
		Erro ("Error: Wrong number of parameters. The blue_mask_filter must have no parameters.", "", "");
		return;
	}

	// Gets the Input Neuron Layer
	nl_input = filter_desc->neuron_layer_list->neuron_layer;

	// Gets the Filter Output 
	nl_output = filter_desc->output;
	
	// Input-Output width
	wi = nl_input->dimentions.x;
	hi = nl_input->dimentions.y;

	wo = nl_output->dimentions.x;
	ho = nl_output->dimentions.y;
	
	if (wi != wo || hi != ho)
	{
		Erro ("Error: Input and output layers on blue_channel_mask filter must have the same 2D size.", "", "");
		return;
	}
	
	for (yo = 0; yo < ho; yo++)
	{
		for (xo = 0; xo < wo; xo++)
		{
			r = (double) RED(nl_input->neuron_vector[xo + yo * wo].output.ival);
			g = (double) GREEN(nl_input->neuron_vector[xo + yo * wo].output.ival);
			b = (double) BLUE(nl_input->neuron_vector[xo + yo * wo].output.ival);
			nl_output->neuron_vector[xo + yo * wo].output.ival = (int) (0.500 * r - 0.419 * g - 0.081 * b + 128.0);
		}
	}
}
