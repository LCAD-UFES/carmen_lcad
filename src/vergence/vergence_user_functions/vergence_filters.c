#include "vergence_filters.h"

/*
 *********************************************************************************
 * Function: translate_nl_filter	     	 				 *
 * Description: 			                        		 *
 * Inputs: 				                  			 *
 * Output: 				                			 *
 *********************************************************************************
 */

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

    NEURON *input_neuron_vector = nl_input->neuron_vector;
    NEURON *output_neuron_vector = nl_output->neuron_vector;

    // Parallel translation filter capabilities where OpenMP available
    //#ifdef	_OPENMP
    //	#pragma omp parallel for private(yo,yi,xo,xi)
    //#endif
    for (yo = 0; yo < ho; yo++)
    {
        yi = (int) ((float) yo + y_offset + .5f);

        for (xo = 0; xo < wo; xo++)
        {
            xi = (int) ((float) xo + x_offset + .5f);

            if ((xi >= 0) && (xi < wi) && (yi >= 0) && (yi < hi))
            	output_neuron_vector[xo + yo * wo].output = input_neuron_vector[xi + yi * wi].output;
            else
            	output_neuron_vector[xo + yo * wo].output.ival = 0;
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


