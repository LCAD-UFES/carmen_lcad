//#include <opencv/cv.h>
#include "tracker_filters.h"
#include "tracker_user_functions.h"

/*
 *********************************************************************************
 * Function: translate_nl_filter	     	 				 *
 * Description: 			                        		 *
 * Inputs: 				                  			 *
 * Output: 				                			 *
 *********************************************************************************
 */
void gaussian_nl_filter(FILTER_DESC *filter_desc) {
    PARAM_LIST *p_list = NULL;
    NEURON_LAYER_LIST *n_list = NULL;
    NEURON_LAYER *nl_output = NULL, *nl_input = NULL;
    int nl_number, p_number;
    //float x_offset, y_offset;
    //int xi, yi, wi, hi, xo, yo;
    int wi, hi;
    int i;
  //  float* input_image_r = NULL, *input_image_g = NULL, *input_image_b = NULL;
    float *output_image_r = NULL, *output_image_g = NULL, *output_image_b = NULL;
 //   int kernel_size; double sigma;

    // Checks the Neuron Layers Number
    for (nl_number = 0, n_list = filter_desc->neuron_layer_list; n_list != NULL; n_list = n_list->next, nl_number++)
        ;

    // Checks the Parameters Number
    for (p_number = 0, p_list = filter_desc->filter_params; p_list != NULL; p_list = p_list->next, p_number++)
        ;

    if (p_number != 3) {
        Erro("Error: Wrong number of parameters. The rotate_nl_filter must have two parameters <x_offset> <y_offset>.", "", "");
        return;
    }

    // Gets the Filter Parameters - The Pointers And The Values - Void pointers must be casted
   // kernel_size = filter_desc->filter_params->next->param.ival;
   // sigma = filter_desc->filter_params->next->next->param.fval;

    // Gets the Input Neuron Layer
    nl_input = filter_desc->neuron_layer_list->neuron_layer;

    // Gets the Filter Output
    nl_output = filter_desc->output;

    wi = nl_input->dimentions.x;
    hi = nl_input->dimentions.y;

    //wo = nl_output->dimentions.x;
    //ho = nl_output->dimentions.y;


//    	input_image_b[i]  = (float)(BLUE(nl_input->neuron_vector[i].output.ival) / 255.0);
//    	input_image_g[i]  = (float)(GREEN(nl_input->neuron_vector[i].output.ival) / 255.0);
//    	input_image_r[i]  = (float)(RED(nl_input->neuron_vector[i].output.ival) / 255.0);

    for(i = 0; i < wi * hi; i++)
    	nl_output->neuron_vector[i].output.ival = PIXEL((long int)(255 * output_image_r[i]), (long int)(255 * output_image_g[i]), (long int)(255 * output_image_b[i]));

}


void translate_nl_filter(FILTER_DESC *filter_desc) {
    PARAM_LIST *p_list = NULL;
    NEURON_LAYER_LIST *n_list = NULL;
    NEURON_LAYER *nl_output = NULL, *nl_input = NULL;
    int nl_number, p_number;
    float x_offset, y_offset;
    int xi, yi, wi, hi, xo, yo, wo, ho;

    // Checks the Neuron Layers Number
    for (nl_number = 0, n_list = filter_desc->neuron_layer_list; n_list != NULL; n_list = n_list->next, nl_number++)
        ;

    // Checks the Parameters Number
    for (p_number = 0, p_list = filter_desc->filter_params; p_list != NULL; p_list = p_list->next, p_number++)
        ;

    if (p_number != 3) {
        Erro("Error: Wrong number of parameters. The rotate_nl_filter must have two parameters <x_offset> <y_offset>.", "", "");
        return;
    }

    // Gets the Filter Parameters - The Pointers And The Values - Void pointers must be casted
    x_offset = *((float*) (filter_desc->filter_params->next->param.pval));
    y_offset = *((float*) (filter_desc->filter_params->next->next->param.pval));

    // Gets the Input Neuron Layer
    nl_input = filter_desc->neuron_layer_list->neuron_layer;

    // Gets the Filter Output 
    nl_output = filter_desc->output;

    wi = nl_input->dimentions.x;
    hi = nl_input->dimentions.y;

    wo = nl_output->dimentions.x;
    ho = nl_output->dimentions.y;

    // Parallel translation filter capabilities where OpenMP available
    //#ifdef	_OPENMP
    //	#pragma omp parallel for private(yo,yi,xo,xi)
    //#endif
    for (yo = 0; yo < ho; yo++) {
        yi = (int) ((float) yo + y_offset + .5f);

        for (xo = 0; xo < wo; xo++) {
            xi = (int) ((float) xo + x_offset + .5f);

            if ((xi >= 0) && (xi < wi) && (yi >= 0) && (yi < hi))
                nl_output->neuron_vector[xo + yo * wo].output = nl_input->neuron_vector[xi + yi * wi].output;
            else
                nl_output->neuron_vector[xo + yo * wo].output.ival = 0;
        }
    }
}



void 
translate_nl_filter_3(FILTER_DESC *filter_desc)
{
	PARAM_LIST *p_list = NULL;
	NEURON_LAYER_LIST *n_list = NULL;
	NEURON_LAYER *nl_output = NULL, *nl_input = NULL;
	int nl_number, p_number;
	//float xi_target_center, yi_target_center;
	float scale_factor;
	int xi, yi, wi, hi, xo, yo, wo, ho;

	// Checks the Neuron Layers Number
	for (nl_number = 0, n_list = filter_desc->neuron_layer_list; n_list != NULL; n_list = n_list->next, nl_number++)
            	;

	// Checks the Parameters Number
	for (p_number = 0, p_list = filter_desc->filter_params; p_list != NULL; p_list = p_list->next, p_number++)
            	;

	if (p_number != 3)
	{
		Erro ("Error: Wrong number of parameters. The rotate_nl_filter must have two parameters <xi_target_center> <yi_target_center>.", "", "");
		return;
	}
	
	// Gets the Filter Parameters - The Pointers And The Values - Void pointers must be casted
	//xi_target_center = *((float *) (filter_desc->filter_params->next->param.pval));
	//yi_target_center = *((float *) (filter_desc->filter_params->next->next->param.pval));

	// Gets the Input Neuron Layer
	nl_input = filter_desc->neuron_layer_list->neuron_layer;

	// Gets the Filter Output 
	nl_output = filter_desc->output;
	
	wi = nl_input->dimentions.x;
	hi = nl_input->dimentions.y;

	wo = nl_output->dimentions.x;
	ho = nl_output->dimentions.y;
	
	scale_factor = 1.0 / dynamic_scale_factor;


	// Parallel translation filter capabilities where OpenMP available
#ifdef	_OPENMP
	#pragma omp parallel for private(yo,yi,xo,xi)
#endif
	for (yo = 0; yo < ho; yo++)
	{			
		yi = (int)   (scale_factor * ((float) yo - (float) ho / 2.0) + (float) hi / 2.0 + .5f);
		
		for (xo = 0; xo < wo; xo++)
		{
			xi = (int)   (scale_factor * ((float) xo - (float) wo / 2.0) + (float) wi / 2.0 + .5f);
			
			if ((xi >= 0) && (xi < wi) && (yi >= 0) && (yi < hi))
				#ifdef	CUDA_COMPILED
					nl_output->host_neuron_vector[xo + yo * wo].output = nl_input->host_neuron_vector[xi + yi * wi].output;
				#else
				nl_output->neuron_vector[xo + yo * wo].output = nl_input->neuron_vector[xi + yi * wi].output;
				#endif
			else
				#ifdef	CUDA_COMPILED
					nl_output->host_neuron_vector[xo + yo * wo].output.ival = 0;
				#else
				nl_output->neuron_vector[xo + yo * wo].output.ival = 0;
				#endif
		}
	}
}


void 
translate_nl_filter_2(FILTER_DESC *filter_desc)
{
	PARAM_LIST *p_list = NULL;
	NEURON_LAYER_LIST *n_list = NULL;
	NEURON_LAYER *nl_output = NULL, *nl_input = NULL;
	int nl_number, p_number;
	//float xi_target_center, yi_target_center;
	//float scale_factor;
	int xi, yi, wi, hi, xo, yo, wo, ho;

	// Checks the Neuron Layers Number
	for (nl_number = 0, n_list = filter_desc->neuron_layer_list; n_list != NULL; n_list = n_list->next, nl_number++)
            	;

	// Checks the Parameters Number
	for (p_number = 0, p_list = filter_desc->filter_params; p_list != NULL; p_list = p_list->next, p_number++)
            	;

	if (p_number != 3)
	{
		Erro ("Error: Wrong number of parameters. The rotate_nl_filter must have two parameters <xi_target_center> <yi_target_center>.", "", "");
		return;
	}
	
	// Gets the Filter Parameters - The Pointers And The Values - Void pointers must be casted
	//xi_target_center = *((float *) (filter_desc->filter_params->next->param.pval));
	//yi_target_center = *((float *) (filter_desc->filter_params->next->next->param.pval));

	// Gets the Input Neuron Layer
	nl_input = filter_desc->neuron_layer_list->neuron_layer;

	// Gets the Filter Output 
	nl_output = filter_desc->output;
	
	wi = nl_input->dimentions.x;
	hi = nl_input->dimentions.y;

	wo = nl_output->dimentions.x;
	ho = nl_output->dimentions.y;
	
	// Parallel translation filter capabilities where OpenMP available
#ifdef	_OPENMP
	#pragma omp parallel for private(yo,yi,xo,xi)
#endif
	for (yo = 0; yo < ho; yo++)
	{			
		yi = (int) (((float) hi / 2.0) - ((float) ho / 2.0) + .5f + yo);
		
		for (xo = 0; xo < wo; xo++)
		{
			xi = (int) ( ((float) wi / 2.0) - ((float) wo / 2.0) + .5f + (float) xo );
			
			if ((xi >= 0) && (xi < wi) && (yi >= 0) && (yi < hi))
				#ifdef	CUDA_COMPILED
					nl_output->host_neuron_vector[xo + yo * wo].output = nl_input->host_neuron_vector[xi + yi * wi].output;
				#else
				nl_output->neuron_vector[xo + yo * wo].output = nl_input->neuron_vector[xi + yi * wi].output;
				#endif
			else
				#ifdef	CUDA_COMPILED
					nl_output->host_neuron_vector[xo + yo * wo].output.ival = 0;
				#else
				nl_output->neuron_vector[xo + yo * wo].output.ival = 0;
				#endif
		}
	}
}

/*
 *********************************************************************************
 * Function: scale_nl_filter	             	 				*
 * Description: 			                        			*
 * Inputs: 				                  			*
 * Output: 				                			*
 *********************************************************************************
 */

void scale_nl_filter(FILTER_DESC *filter_desc) {
    PARAM_LIST *p_list = NULL;
    NEURON_LAYER_LIST *n_list = NULL;
    NEURON_LAYER *nl_output = NULL, *nl_input = NULL;
    int nl_number, p_number;
    float scale_factor, k;
    int xi, yi, wi, hi, xo, yo, wo, ho;
    //int delta_xo,delta_yo,corrected_xo,corrected_yo;	//Center offset for image scale

    // Checks the Neuron Layers Number
    for (nl_number = 0, n_list = filter_desc->neuron_layer_list; n_list != NULL; n_list = n_list->next, nl_number++)
        ;

    // Checks the Parameters Number
    for (p_number = 0, p_list = filter_desc->filter_params; p_list != NULL; p_list = p_list->next, p_number++)
        ;

    if (p_number != 4) {
        Erro("Error: Wrong number of parameters. The scale_nl_filter must have only three parameter <scale_factor> <delta_xo> <delta_yo>.", "", "");
        return;
    }

    // Gets the Filter Parameters
    scale_factor = filter_desc->filter_params->next->param.fval;
    //delta_xo = filter_desc->filter_params->next->next->param.ival;
    //delta_yo = filter_desc->filter_params->next->next->next->param.ival;

    // Gets the Input Neuron Layer
    nl_input = filter_desc->neuron_layer_list->neuron_layer;

    // Gets the Filter Output 
    nl_output = filter_desc->output;

    wi = nl_input->dimentions.x;
    hi = nl_input->dimentions.y;

    wo = nl_output->dimentions.x;
    ho = nl_output->dimentions.y;

    // Calculates the scaling constant
    k = 1.0 / scale_factor;

    //#ifdef	_OPENMP
    //	#pragma omp parallel for private(yo,yi,xo,xi)
    //#endif	
    for (yo = 0; yo < ho; yo++) {
        yi = (int) (k * (float) yo + .5f); //+ (int)(scale_factor*delta_yo);

        for (xo = 0; xo < wo; xo++) {
            xi = (int) (k * (float) xo + .5f); //+ (int)(scale_factor*delta_xo);

            if ((xi >= 0) && (xi < wi) && (yi >= 0) && (yi < hi))
                nl_output->neuron_vector[xo + yo * wo].output = nl_input->neuron_vector[xi + yi * wi].output;
            else
                nl_output->neuron_vector[xo + yo * wo].output.ival = 0;
        }
    }
}

/*
 *********************************************************************************
 * Function: rotate_nl_filter	             	 				*
 * Description: 			                        			*
 * Inputs: 				                  			*
 * Output: 				                			*
 *********************************************************************************
 */

void rotate_nl_filter(FILTER_DESC *filter_desc) {
    PARAM_LIST *p_list = NULL;
    NEURON_LAYER_LIST *n_list = NULL;
    NEURON_LAYER *nl_output = NULL, *nl_input = NULL;
    int nl_number, p_number;
    float angle, cos_angle, sin_angle;
    int xi, yi, wi, hi, xo, yo, wo, ho;

    // Checks the Neuron Layers Number
    for (nl_number = 0, n_list = filter_desc->neuron_layer_list; n_list != NULL; n_list = n_list->next, nl_number++)
        ;

    // Checks the Parameters Number
    for (p_number = 0, p_list = filter_desc->filter_params; p_list != NULL; p_list = p_list->next, p_number++)
        ;

    if (p_number != 2) {
        Erro("Error: Wrong number of parameters. The rotate_nl_filter must have only one parameter <angle>.", "", "");
        return;
    }

    // Gets the Filter Parameters
    angle = filter_desc->filter_params->next->param.fval;

    // Gets the Input Neuron Layer
    nl_input = filter_desc->neuron_layer_list->neuron_layer;

    // Gets the Filter Output 
    nl_output = filter_desc->output;

    wi = nl_input->dimentions.x;
    hi = nl_input->dimentions.y;

    wo = nl_output->dimentions.x;
    ho = nl_output->dimentions.y;

    angle *= pi / 180.0f;
    cos_angle = cos(angle);
    sin_angle = sin(angle);

    //#ifdef	_OPENMP
    //	#pragma omp parallel for private(yo,yi,xo,xi)
    //#endif	
    for (yo = 0; yo < ho; yo++) {
        for (xo = 0; xo < wo; xo++) {
            xi = (int) (cos_angle * (float) xo + sin_angle * (float) yo + .5f);
            yi = (int) (-sin_angle * (float) xo + cos_angle * (float) yo + .5f);

            if ((xi >= 0) && (xi < wi) && (yi >= 0) && (yi < hi))
                nl_output->neuron_vector[xo + yo * wo].output = nl_input->neuron_vector[xi + yi * wi].output;
            else
                nl_output->neuron_vector[xo + yo * wo].output.ival = 0;
        }
    }
}

/*
 *********************************************************************************
 * Function: nl_reshape_filter	              					*
 * Description: 			                        			*
 * Inputs: 				                  			*
 * Output: 				                			*
 *********************************************************************************
 */

void nl_reshape_filter(FILTER_DESC *filter_desc) {
    PARAM_LIST *p_list = NULL;
    NEURON_LAYER_LIST *n_list = NULL;
    NEURON_LAYER *nl_output = NULL, *nl_input = NULL;
    int nl_number, p_number;
    float x_offset, y_offset;
    float scale_factor;
    int xi, yi, wi, hi, xo, yo, wo, ho;
    float xt, yt, xs, ys, k;

    // Checks the Neuron Layers Number
    for (nl_number = 0, n_list = filter_desc->neuron_layer_list; n_list != NULL; n_list = n_list->next, nl_number++)
        ;

    // Checks the Parameters Number
    for (p_number = 0, p_list = filter_desc->filter_params; p_list != NULL; p_list = p_list->next, p_number++)
        ;

    if (p_number != 4) {
        Erro("Error: Wrong number of parameters. The face_reshape_filter2 must have tree parameters <baseline_factor> <x_offset> <y_offset>", "", "");
        return;
    }

    // Gets the Filter Parameters
    //scale_factor = filter_desc->filter_params->next->param.fval;
    scale_factor = *((float*) (filter_desc->filter_params->next->param.pval));
    x_offset = (float) filter_desc->filter_params->next->next->param.ival;
    y_offset = (float) filter_desc->filter_params->next->next->next->param.ival;

    // Gets the Filter Output 
    nl_output = filter_desc->output;

    // Gets the Input Neuron Layer
    nl_input = filter_desc->neuron_layer_list->neuron_layer;

    // Gets the Input Neuron Layer
    nl_input = filter_desc->neuron_layer_list->neuron_layer;

    // Gets the Filter Output 
    nl_output = filter_desc->output;

    wi = nl_input->dimentions.x;
    hi = nl_input->dimentions.y;

    wo = nl_output->dimentions.x;
    ho = nl_output->dimentions.y;

    // Calculates the scale factor constant
    k = 1.0 / scale_factor;

    for (yo = 0; yo < ho; yo++) {
        for (xo = 0; xo < wo; xo++) {
            xt = (float) xo - (float) wo / 2.0f;
            yt = (float) yo - (float) ho / 2.0f;

            xs = k * xt;
            ys = k * yt;

            xt = xs - x_offset;
            yt = ys - y_offset;

            xi = (int) (xt + .5f);
            yi = (int) (yt + .5f);

            if ((xi >= 0) && (xi < wi) && (yi >= 0) && (yi < hi))
                nl_output->neuron_vector[xo + yo * wo].output = nl_input->neuron_vector[xi + yi * wi].output;
            else
                nl_output->neuron_vector[xo + yo * wo].output.ival = 0;
        }
    }
}

/*
 *********************************************************************************
 * Function: scaled_map_image_v1	             	 				*
 * Description: 			                        			*
 * Inputs: 				                  			*
 * Output: 				                			*
 *********************************************************************************
 */

void 
scaled_map_image_v1(FILTER_DESC *filter_desc) {
    NEURON_LAYER_LIST *n_list = NULL;
    PARAM_LIST *p_list = NULL;
    NEURON_LAYER *n_l = NULL;
    INPUT_DESC *input = NULL;
    int i, u, v, h, w, hi, wi, xi, yi, previous_xi, previous_yi;
    int x_center, y_center;
    float log_factor, scale_factor;
    NEURON_OUTPUT previous_output;

    previous_output.ival = 0;

    // Check Neuron Layers
    for (i = 0, n_list = filter_desc->neuron_layer_list; n_list != NULL; n_list = n_list->next, i++)
        ;

    if (i != 1) {
        Erro("Wrong number of neuron layers. Map Image V1 filter must be applied on only one neuron layer.", "", "");
        return;
    }

    n_l = filter_desc->neuron_layer_list->neuron_layer;

    // Check Param
    for (i = 0, p_list = filter_desc->filter_params; p_list != NULL; p_list = p_list->next, i++)
        ;

    // 4 Parameters are passed to this filter
    if (i != 4) {
        Erro("Wrong number of parameters. Map Image V1 needs three parameters: <input name>, <log_factor>, <scale_factor>.", "", "");
        return;
    }

    input = get_input_by_name(filter_desc->filter_params->next->param.sval);
    log_factor = filter_desc->filter_params->next->next->param.fval;
    scale_factor = filter_desc->filter_params->next->next->next->param.fval;

    wi = n_l->dimentions.x;
    hi = n_l->dimentions.y;
    w = filter_desc->output->dimentions.x;
    h = filter_desc->output->dimentions.y;

    previous_xi = -1;
    previous_yi = -1;

    if (input == NULL) {
        x_center = 0;
        y_center = 0;
    } else {
        if (TYPE_MOVING_FRAME == STOP) {
            x_center = input->wxd - input->wx;
            y_center = input->wyd - input->wy;
        } else {
            x_center = wi / 2;
            y_center = hi / 2;
        }
    }

    for (u = 0; u < w; u++) {
        for (v = 0; v < h; v++) {
            map_v1_to_image(&xi, &yi, wi, hi, u, v, w, h, x_center, y_center, (double) h / (double) (h - 1), log_factor);

            // Adjust "xi" and "yi" according to the scale factor
            xi = x_center + (xi - x_center)*(1.0 / scale_factor);
            yi = y_center + (yi - y_center)*(1.0 / scale_factor);

            if ((xi == previous_xi) && (yi == previous_yi))
                filter_desc->output->neuron_vector[(v * w) + u].output = previous_output;
            else
                if (xi >= wi || xi < 0 || yi >= hi || yi < 0)
                previous_output.ival = filter_desc->output->neuron_vector[(v * w) + u].output.fval = 0;
            else
                previous_output = filter_desc->output->neuron_vector[(v * w) + u].output = n_l->neuron_vector[yi * wi + xi].output;

            previous_xi = xi;
            previous_yi = yi;
        }
    }
}

void
compute_weigheted_neighboor_filter(FILTER_DESC *filter_desc)
{
	int u, v;
	int w, h;
	double partial_weight;
	double log_factor;
	NEURON_LAYER *nl_output = NULL, *nl_input = NULL;
	//double threshold, pt;
	//int i;

	// Gets the Input Neuron Layer
	nl_input = filter_desc->neuron_layer_list->neuron_layer;

	// Gets the Filter Output 
	nl_output = filter_desc->output;

	log_factor = filter_desc->filter_params->next->param.fval;

	// Input-Output width
	w = nl_input->dimentions.x;
	h = nl_input->dimentions.y;


	//threshold = 0.0; pt = 0.0;
	//i = 0;
	for (v = 0; v < h; v++)
	{
		for (u = 0; u < w; u++)
		{
			if (BLUE(nl_input->neuron_vector[u + w * v].output.ival) != 0)
				partial_weight = (double) compute_weigheted_neighborhood(nl_input->neuron_vector, w, h, u, v, log_factor);// * (double) BLUE(nl_input->neuron_vector[u + w * v].output.ival);
			else
				partial_weight = 0.0;

			nl_output->neuron_vector[u + w * v].output.fval = partial_weight;
			//threshold +=  nl_v1_activation_map_neuron_weight.neuron_vector[i].output.fval;
			//pt += ( partial_weight > 0.0 ? 2.0 : 0.0 );
			//i++;
		}
	}
/*	threshold /= (double) pt;
	for (v = 0; v < h; v++)
	{
		for (u = 0; u < w; u++)
		{
			//nl_output->neuron_vector[u + w * v].output.fval = ( nl_output->neuron_vector[u + w * v].output.fval > threshold ? nl_output->neuron_vector[u + w * v].output.fval : 0.0 );
		}
	}
*/
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


void red_mask_filter(FILTER_DESC *filter_desc)
{
	PARAM_LIST *p_list = NULL;
	NEURON_LAYER_LIST *n_list = NULL;
	NEURON_LAYER *nl_output = NULL, *nl_input = NULL;
	int nl_number, p_number;
	int xi, yi, wi, hi, wo, ho;

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


void green_mask_filter(FILTER_DESC *filter_desc)
{
	PARAM_LIST *p_list = NULL;
	NEURON_LAYER_LIST *n_list = NULL;
	NEURON_LAYER *nl_output = NULL, *nl_input = NULL;
	int nl_number, p_number;
	int xi, yi, wi, hi, wo, ho;

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


void blue_mask_filter(FILTER_DESC *filter_desc)
{
	PARAM_LIST *p_list = NULL;
	NEURON_LAYER_LIST *n_list = NULL;
	NEURON_LAYER *nl_output = NULL, *nl_input = NULL;
	int nl_number, p_number;
	int xi, yi, wi, hi, wo, ho;

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


void
generate_hough_activation_map(FILTER_DESC *filter_desc)
{
	int u, v, i;
	int b, x, y, xc, yc, xi, yi, xv, yv;
	int wU, hV, wX, hY;
	NEURON_LAYER *nl_output = NULL, *nl_input = NULL;
	int value;
	double log_factor, partial_weight;

	log_factor = filter_desc->filter_params->next->param.fval;

	// Gets the Input Neuron Layer
	nl_input = filter_desc->neuron_layer_list->neuron_layer;

	// Gets the Filter Output 
	nl_output = filter_desc->output;

	// Output width
	wU = nl_input->dimentions.x;
	hV = nl_input->dimentions.y;

	// Input-Output width
	wX = nl_output->dimentions.x;
	hY = nl_output->dimentions.y;

	//Zera a saida
	for (i = 0; i < wX*hY; i++)
		nl_output->neuron_vector[i].output.fval = 0.0;

	for (v = 0; v < hV; v++)
	{
		for (u = 0; u < wU; u++)
		{
			value = nl_input->neuron_vector[v * wU + u].output.ival;
			b = BLUE(value);

			if (b > 0){
				
				// deslocamento do centro para o neuronio principal   
				xc = gt_x_displecement_from_fovea( GREEN(value) );
				yc = gt_y_displecement_from_fovea( RED(value) );
				map_v1_to_image(&xi, &yi, wX, hY, u, v, wU, hV, wX/2, hY/2, 0, log_factor);

				// deslocamento do centro para o neuronio principal   
				/*xc = gt_x_displecement_from_fovea(GREEN(value));
				yc = gt_y_displecement_from_fovea(RED(value));
				xi = (int)(((double)wX/2) + ((double)gt_x_displecement_from_fovea(GREEN(table_v1.neuron_vector[v * wU + u].output.ival))));
				yi = (int)(((double)hY/2) + ((double)gt_y_displecement_from_fovea(RED(table_v1.neuron_vector[v * wU + u].output.ival))));*/
						
				x = (xi - xc);
				y = (yi - yc);

				//printf("x %d y %d xc %d yc %d xi %d yi %d\n", x, y, xc, yc, xi, yi);
				partial_weight = nl_v1_activation_map_neuron_weight.neuron_vector[v * wU + u].output.fval;
				//hamming_weight = nl_v1_activation_map_neuron_weight_thresholded.neuron_vector[v * wU + u].output.fval;
				for (yv = -0; yv <= 0; yv++)
					for (xv = -0; xv <= 0; xv++)
						nl_output->neuron_vector[(x + xv) + wX * (y+yv)].output.fval += partial_weight;
			}
		}
	}
}


void
generate_hough_zoom_activation_map(FILTER_DESC *filter_desc)
{
	int u, v, i;
	int xc, yc, xi, yi;
	double x, ri, rc;
	int wU, hV, wX, hY;
	NEURON_LAYER *nl_output = NULL, *nl_input = NULL;
	int value, value2;
	double partial_weight;
	//static int first_time = 1;
	//OUTPUT_DESC *output_window;
	//double log_factor;

	//log_factor = filter_desc->filter_params->next->param.fval;

	/*
	if (first_time == 1)
	{
		output_window = get_output_by_neural_layer(filter_desc->output);
		glutSetWindow(output_window->win);
		glutReshapeWindow(1500, 5);
		glutPostRedisplay();
		//printf("tentei...\n");
		first_time = 0;
	}
*/
	// Gets the Input Neuron Layer
	nl_input = filter_desc->neuron_layer_list->neuron_layer;

	// Gets the Filter Output 
	nl_output = filter_desc->output;

	// Output width
	wU = nl_input->dimentions.x;
	hV = nl_input->dimentions.y;

	// Input-Output width
	wX = nl_output->dimentions.x;
	hY = nl_output->dimentions.y;

	//Zera a saida
	for (i = 0; i < wX*hY; i++)
		nl_output->neuron_vector[i].output.fval = 0.0;

	for (v = 0; v < hV; v++)
	{
		for (u = 0; u < wU; u++)
		{
			value = nl_input->neuron_vector[v * wU + u].output.ival;
			value2 = table_v1.neuron_vector[v * wU + u].output.ival;

			if (value > 0 && value2>0){
				
				// deslocamento do centro para o neuronio principal   
				xc = gt_x_displecement_from_fovea(GREEN(value));
				yc = gt_y_displecement_from_fovea(RED(value));
				xi = gt_x_displecement_from_fovea(GREEN(value2));
				yi = gt_y_displecement_from_fovea(RED(value2));	

				ri = sqrt((xi * xi) + (yi * yi));
				rc = sqrt((xc * xc) + (yc * yc));
				
				if (ri > 5.0){

					if (ri == rc) x = 1.0;						
					else if ((ri != 0) && (rc != 0)) x = (double)rc / (double)ri;
					else x = -20.0;
//printf("x = %f ", x);

					partial_weight = nl_v1_activation_map_neuron_weight.neuron_vector[v * wU + u].output.fval;
					if ((x > 0) && (x <= 10.0))
					{
						if (x == 1.0)
							nl_output->neuron_vector[(int)((x)*800.0)].output.fval += 1.0 * partial_weight;	 
						else if (x < 1.0)
						{
							if (x >= 0.1)
								nl_output->neuron_vector[(int)(800.0 - ((1.0/x) * 80.0)+ 0.5)].output.fval += 1.0 * partial_weight;	
							else
								nl_output->neuron_vector[0].output.fval += 1.0 * partial_weight;
						}
						else
						{
							nl_output->neuron_vector[(int)((x + 10.0)*80.0 + 0.5)].output.fval += 1.0 * partial_weight;
						}
						
					}	
				}
			
			}
		}
	}

}

