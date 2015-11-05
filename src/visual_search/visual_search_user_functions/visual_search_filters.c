#include "visual_search_filters.h"

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

	#ifndef	CML_SCRIPT
		printf("scale = %.2f\n", k);
	#endif

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

/**************************** POINT LIST OPERATIONS *****************************/

// Returns the new list if not existent or the same list if already exists

MASK_POINT_LIST* add_point_to_ihibition_list(MASK_POINT_LIST *list, int x, int y, int max_num_mov) {
    MASK_POINT_LIST *new_node;

    new_node = (MASK_POINT_LIST*) malloc(sizeof (MASK_POINT_LIST));
    new_node->x = x;
    new_node->y = y;
    new_node->timer = max_num_mov;
    new_node->next = NULL;

    if (list) {
        list->next = new_node; //the list pointer shouldn be changed here
        new_node->previous = list;
    } else {
        list = new_node;
        new_node->previous = NULL;
    }

    return (list);
}

// Returns the resulting list with the deleted node from the inhibition list

MASK_POINT_LIST* delete_node_from_ihibition_list(MASK_POINT_LIST *list, MASK_POINT_LIST *node) {
    if (node == list) // if we are deleting the list head
    {
        //make sure the new 1st node is updated
        if (list->next)
            list->next->previous = NULL;

        list = list->next; //updates the list		
        free(node); //free the node
    } else {
        // Changes the previous "next"
        if (node->next) {
            node->next->previous = node->previous;
            node->previous->next = node->next;
            free(node);
            // the list head remains unchanged
        }
    }

    return (list);
}

int is_xy_point_already_present_in_inhibition_list(MASK_POINT_LIST *list, int x, int y) {
    MASK_POINT_LIST *node;

    for (node = list; node; node = node->next)
        if (x == node->x && y == node-> y)
            return (TRUE);

    return (FALSE);
}

/*
 *********************************************************************************
 * Function: draw_inhibited_regions_in_nl_filter 	     	 		 *
 * Description: 			                        		 *
 * Inputs: 				                  			 *
 * Output: 				                			 *
 *********************************************************************************
 */

void draw_inhibited_regions_in_nl_filter(FILTER_DESC *filter_desc) {
    PARAM_LIST *p_list = NULL;
    NEURON_LAYER_LIST *n_list = NULL;
    NEURON_LAYER *nl_output = NULL, *nl_input = NULL;
    INPUT_DESC *vs_input_layer = NULL;
    MASK_POINT_LIST *filter_mask_point_list;
    int nl_number, p_number;
    int xo, yo, wo, ho; // for the output layer
    int xp, yp, xc, yc; // for drawing the inhibited regions

    float scale_factor;
    int inhib_x_region_size;
    int inhib_y_region_size;

    // Checks the Neuron Layers Number
    for (nl_number = 0, n_list = filter_desc->neuron_layer_list; n_list != NULL; n_list = n_list->next, nl_number++)
        ;

    // Checks the Parameters Number
    for (p_number = 0, p_list = filter_desc->filter_params; p_list != NULL; p_list = p_list->next, p_number++)
        ;

    if (p_number != 5) {
        Erro("Error: Wrong number of parameters. The draw_inhibited_regions_in_nl_filter must have four parameters <scale_factor> <inhib_x_region_size> <inhib_y_region_size> <visual_search_input_layer>.", "", "");
        return;
    }

    // Gets the Filter Parameters
    scale_factor = filter_desc->filter_params->next->param.fval;
    inhib_x_region_size = filter_desc->filter_params->next->next->param.ival;
    inhib_y_region_size = filter_desc->filter_params->next->next->next->param.ival;
    vs_input_layer = ((INPUT_DESC*) (filter_desc->filter_params->next->next->next->next->param.pval));

    inhib_x_region_size = (int) ((float) inhib_x_region_size * scale_factor);
    inhib_y_region_size = (int) ((float) inhib_y_region_size * scale_factor);

    // Gets the Input Neuron Layer
    nl_input = filter_desc->neuron_layer_list->neuron_layer;

    // Gets the Filter Output 
    nl_output = filter_desc->output;

    // Output layer dimentions
    wo = nl_output->dimentions.x;
    ho = nl_output->dimentions.y;

    //Copies the input data into the output neuron layer
    for (yo = 0; yo < ho; yo++)
        for (xo = 0; xo < wo; xo++)
            nl_output->neuron_vector[xo + yo * wo].output = nl_input->neuron_vector[xo + yo * wo].output;

    xc = vs_input_layer->wxd;
    yc = vs_input_layer->wyd;

    //Now we should go through the point list and draw the inhibition region of the non desired objects
    for (filter_mask_point_list = visual_search_mask_point_list; filter_mask_point_list != NULL; filter_mask_point_list = filter_mask_point_list->next) 
    {
    /*	The X-Y point need to be transformed to the last layer coordinate system (transforms inverted)
        The (0,0) coordinate in this system 
        x = -(wo/2.0)*1/scale_factor 
        y = -(yo/2.0)*1/scale_factor
        The Desired Point must be translated to this coordinate system		*/

        xp = filter_mask_point_list->x;
        yp = filter_mask_point_list->y;

        xo = (int) ((float) (xp - xc)*(scale_factor) + wo / 2.0);
        yo = (int) ((float) (yp - yc)*(scale_factor) + ho / 2.0);

        /* Now draws the black square on the inhibited region */
        for (yc = yo - inhib_y_region_size / 2; yc < yo + inhib_y_region_size / 2; yc++)
            for (xc = xo - inhib_x_region_size / 2; xc < xo + inhib_x_region_size / 2; xc++)
                if (xc > 0 && yc > 0 && xc < wo && yc < ho)
                    nl_output->neuron_vector[xc + yc * wo].output.ival = 0;

        if (filter_mask_point_list->timer) //if>0 decrease 
            (filter_mask_point_list->timer)--;
        else //if 0
            visual_search_mask_point_list = delete_node_from_ihibition_list(visual_search_mask_point_list, filter_mask_point_list);
    }
}

/*
 *********************************************************************************
 * Function: calculate_error_function_from_trained_output_filter  	 	 *
 * Description: 			                        		 *
 * Inputs: 				                  			 *
 * Output: 				                			 *
 *********************************************************************************
 */

void calculate_error_function_from_trained_output_filter(FILTER_DESC *filter_desc) {
    PARAM_LIST *p_list = NULL;
    NEURON_LAYER_LIST *n_list = NULL;
    NEURON_LAYER *nl_output = NULL, *nl_input = NULL;
    int nl_number, p_number;
    int wi, hi, xo, yo, wo, ho;

    // Checks the Neuron Layers Number
    for (nl_number = 0, n_list = filter_desc->neuron_layer_list; n_list != NULL; n_list = n_list->next, nl_number++)
        ;

    // Checks the Parameters Number
    for (p_number = 0, p_list = filter_desc->filter_params; p_list != NULL; p_list = p_list->next, p_number++)
        ;

    if (p_number != 1) {
        Erro("Error: Wrong number of parameters. The calculate_error_function_from_trained_output_filter must have no parameters.", "", "");
        return;
    }

    // Gets the Input Neuron Layer
    nl_input = filter_desc->neuron_layer_list->neuron_layer;

    // Gets the Filter Output 
    nl_output = filter_desc->output;

    // Input Layer Dimensions
    wi = nl_input->dimentions.x;
    hi = nl_input->dimentions.y;

    // Output Layer Dimensions 
    wo = nl_output->dimentions.x;
    ho = nl_output->dimentions.y;

    // Layer size verification
    if (wo != wi || ho != hi) {
        Erro("Error: calculate_error_function_from_trained_output_filter input and output layers must have te Same Size.", "", "");
        return;
    }

    // Checks whether the neuron layer has been trained
    if (!gaussian_filtered_training_pattern) {
        return;
    }

    for (yo = 0; yo < ho; yo++) {
        for (xo = 0; xo < wo; xo++) {
            // Error = Reference - Output

            // Simple Difference calculate (other error functions could be also applied)
            nl_output->neuron_vector[xo + yo * wo].output.fval = gaussian_filtered_training_pattern[xo + yo * wo] - nl_input->neuron_vector[xo + yo * wo].output.fval;
            //nl_output->neuron_vector[xo + yo * wo].output = nl_input->neuron_vector[xi + yi * wi].output;
        }
    }
}

/* float to int grayscale */
inline int float_to_int_grayscale(float val, float max_val) {
    int channel;

    if (val > 1.0)
        val = 1.0;
    if (val < 0.0)
        val = 0.0;

    channel = (int) (255.0 * val / max_val);
    return (0 + channel + (channel << 8) + (channel << 16));
}

/*
 *****************************************************************************************
 * Function: activation_pattern_overlay_filter	     	 				 *
 * Description: Overlays the image neuron layer with the output activation pattern	 *
 * Inputs: 				                  				 *
 * Output: 				                				 *
 *****************************************************************************************
 */

void activation_pattern_overlay_filter(FILTER_DESC *filter_desc) {
    PARAM_LIST *p_list = NULL;
    NEURON_LAYER_LIST *n_list = NULL;
    NEURON_LAYER *nl_output = NULL, *nl_input = NULL, *nl_act_pattern = NULL;
    int nl_number, p_number;
    float log_factor;
    int xi, yi, wi, hi, xo, yo, wo, ho, wa, ha;
    int x_center, y_center;
    float max_value = FLT_MIN;

    // Checks the Neuron Layers Number
    for (nl_number = 0, n_list = filter_desc->neuron_layer_list; n_list != NULL; n_list = n_list->next, nl_number++)
        ;

    // Checks the Parameters Number
    for (p_number = 0, p_list = filter_desc->filter_params; p_list != NULL; p_list = p_list->next, p_number++)
        ;

    if (p_number != 2) {
        Erro("Error: Wrong number of parameters. The activation_pattern_overlay_filter needs one parameter: <log_factor>.", "", "");
        return;
    }

    // Gets the filter parameters
    log_factor = filter_desc->filter_params->next->param.fval;

    // Gets the Input Neuron Layer
    nl_input = filter_desc->neuron_layer_list->neuron_layer;
    // Gets the Activation Pattern Layer
    nl_act_pattern = filter_desc->neuron_layer_list->next->neuron_layer;
    // Gets the Filter Output 
    nl_output = filter_desc->output;

    // Input layer
    wi = nl_input->dimentions.x;
    hi = nl_input->dimentions.y;
    x_center = wi / 2;
    y_center = hi / 2;

    // Activation layer
    wa = nl_act_pattern->dimentions.x;
    ha = nl_act_pattern->dimentions.y;

    // Output layer
    wo = nl_output->dimentions.x;
    ho = nl_output->dimentions.y;

    // Copies the neuron layer data
    for (yo = 0; yo < ho; yo++)
        for (xo = 0; xo < wo; xo++)
            nl_output->neuron_vector[xo + yo * wo].output = nl_input->neuron_vector[xo + yo * wo].output;

    //Search for the activation layer max value
    //TODO: If not initialized, the "nl_act_pattern->neuron_vector" could cause context errors 
    for (yo = 0; yo < ha; yo++)
        for (xo = 0; xo < wa; xo++)
            if (max_value < nl_act_pattern->neuron_vector[yo * wa + xo].output.fval)
                max_value = nl_act_pattern->neuron_vector[yo * wa + xo].output.fval;

    // Now maps the activated pattern into the image
    for (yo = 0; yo < ha; yo++)
    {
        for (xo = 0; xo < wa; xo++)
        {
            map_v1_to_image(&xi, &yi, wi, hi, xo, yo, wa, ha, x_center, y_center, (double) ha / (double) (ha - 1), log_factor);

            if (!(xi >= wi || xi < 0 || yi >= hi || yi < 0))
	    {
	    	nl_output->neuron_vector[yi * wi + xi].output.ival = float_to_int_grayscale(nl_act_pattern->neuron_vector[yo * wa + xo].output.fval, max_value);
	    	
	    	//if inside neuron layer boundaries
	    	if(xo > 0 && xo < wa)
	    	{
	    		if(gaussian_filtered_training_pattern)
	    		{
		    		if( (gaussian_filtered_training_pattern[yo * wa + xo] != 0.0f && gaussian_filtered_training_pattern[yo * wa + xo - 1] == 0.0)
		    		||  (gaussian_filtered_training_pattern[yo * wa + xo] != 0.0f && gaussian_filtered_training_pattern[yo * wa + xo + 1] == 0.0) )
		    		{
		    			nl_output->neuron_vector[yi * wi + xi].output.ival = 0x000000ff;
		    		}
		    	}
	    		
	    	}
	    	
               	//previous_output = nl_output->neuron_vector[yi * wi + xi].output;
            }
        }
    }
}


/*
 *********************************************************************************
 * Function: output_layer_non_zero_activation_mean_value_threshold_filter 	 *
 * Description: 			                        		 *
 * Inputs: 				                  			 *
 * Output: 				                			 *
 *********************************************************************************
 */

void output_layer_non_zero_activation_mean_value_threshold_filter(FILTER_DESC *filter_desc) {
    PARAM_LIST *p_list = NULL;
    NEURON_LAYER_LIST *n_list = NULL;
    NEURON_LAYER *nl_output = NULL, *nl_input = NULL;
    int nl_number, p_number;
    int wi, hi, xo, yo, wo, ho;

    float mean_activation_value;
    int non_zero_activated_outputs;

    // Checks the Neuron Layers Number
    for (nl_number = 0, n_list = filter_desc->neuron_layer_list; n_list != NULL; n_list = n_list->next, nl_number++)
        ;

    // Checks the Parameters Number
    for (p_number = 0, p_list = filter_desc->filter_params; p_list != NULL; p_list = p_list->next, p_number++)
        ;

    if (p_number != 1) {
        Erro("Error: Wrong number of parameters. The output_layer_non_zero_activation_mean_value_threshold_filter must have no parameters.", "", "");
        return;
    }

    // Gets the Input Neuron Layer
    nl_input = filter_desc->neuron_layer_list->neuron_layer;

    // Gets the Filter Output 
    nl_output = filter_desc->output;

    // Input Layer Dimensions
    wi = nl_input->dimentions.x;
    hi = nl_input->dimentions.y;

    // Output Layer Dimensions 
    wo = nl_output->dimentions.x;
    ho = nl_output->dimentions.y;

    // Layer size verification
    if (wo != wi || ho != hi) {
        Erro("Error: output_layer_non_zero_activation_mean_value_threshold_filter input and output layers must have te Same Size.", "", "");
        return;
    }

    mean_activation_value = 0.0;
    non_zero_activated_outputs = 0;

    // Checks for the non-zero activated neurons and compute the mean value
    for (yo = 0; yo < ho; yo++) {
        for (xo = 0; xo < wo; xo++) {
            if (nl_input->neuron_vector[xo + yo * wo].output.fval > 0.0) //if higher than zero
            {
                mean_activation_value += nl_input->neuron_vector[xo + yo * wo].output.fval; // accumulator
                non_zero_activated_outputs++; // count as a higher than zero neuron
            }
        }
    }

    mean_activation_value = mean_activation_value / (float) non_zero_activated_outputs; // Normalize the mean value against the number of activated neurons

    // Now discard those values below the mean value threshold
    for (yo = 0; yo < ho; yo++) {
        for (xo = 0; xo < wo; xo++) {
            if (nl_input->neuron_vector[xo + yo * wo].output.fval > mean_activation_value)
                nl_output->neuron_vector[xo + yo * wo].output.fval = nl_input->neuron_vector[xo + yo * wo].output.fval;
            else
                nl_output->neuron_vector[xo + yo * wo].output.fval = 0.0;
        }
    }
}

/*
 *********************************************************************************
 * Function: output_layer_max_activation_value_percetage_threshold_filter 	 *
 * Description: 			                        		 *
 * Inputs: 				                  			 *
 * Output: 				                			 *
 *********************************************************************************
 */

void output_layer_max_activation_value_percentage_threshold_filter(FILTER_DESC *filter_desc) {
    PARAM_LIST *p_list = NULL;
    NEURON_LAYER_LIST *n_list = NULL;
    NEURON_LAYER *nl_output = NULL, *nl_input = NULL;
    int nl_number, p_number;
    int wi, hi, xo, yo, wo, ho;
    int u, v;
    float threshold, max_activation_value, min_activation_value;

    // Checks the Neuron Layers Number
    for (nl_number = 0, n_list = filter_desc->neuron_layer_list; n_list != NULL; n_list = n_list->next, nl_number++)
        ;

    // Checks the Parameters Number
    for (p_number = 0, p_list = filter_desc->filter_params; p_list != NULL; p_list = p_list->next, p_number++)
        ;

    if (p_number != 2) {
        Erro("Error: Wrong number of parameters. The output_layer_max_activation_value_percetage_threshold_filter needs one parameters: <threshold>, .", "", "");
        return;
    }

    threshold = filter_desc->filter_params->next->param.fval;

    // Gets the Input Neuron Layer
    nl_input = filter_desc->neuron_layer_list->neuron_layer;

    // Gets the Filter Output 
    nl_output = filter_desc->output;

    // Input Layer Dimensions
    wi = nl_input->dimentions.x;
    hi = nl_input->dimentions.y;

    // Finds the max value position
    max_activation_value = FLT_MIN;
    for (v = 0; v < hi; v++)
        for (u = 0; u < wi; u++)
            if (nl_output->neuron_vector[v * wi + u].output.fval > max_activation_value)
                max_activation_value = nl_input->neuron_vector[v * wi + u].output.fval;

    // The input neuron layer maximum activation value and the treshold
    //max_activation_value = nl_input->max_neuron_output.fval;
    min_activation_value = max_activation_value*threshold;

    // Output Layer Dimensions 
    wo = nl_output->dimentions.x;
    ho = nl_output->dimentions.y;

    // Layer size verification
    if (wo != wi || ho != hi) {
        Erro("Error: output_layer_max_activation_value_percetage_threshold_filter input and output layers must have the Same Size.", "", "");
        return;
    }

    // Now discard those values below the max activation value threshold
    for (yo = 0; yo < ho; yo++) {
        for (xo = 0; xo < wo; xo++) {
            if (nl_input->neuron_vector[xo + yo * wo].output.fval > min_activation_value)
                nl_output->neuron_vector[xo + yo * wo].output.fval = nl_input->neuron_vector[xo + yo * wo].output.fval;
            else
                nl_output->neuron_vector[xo + yo * wo].output.fval = 0.0;
        }
    }
}

/*
 *********************************************************************************
 * Function:	hamming_distance_filter						 *
 * Description: 			                        		 *
 * Inputs: 				                  			 *
 * Output: 				                			 *
 *********************************************************************************
 */

void hamming_distance_filter(FILTER_DESC *filter_desc)
{    
    PARAM_LIST *p_list = NULL;
    NEURON_LAYER_LIST *n_list = NULL;
    NEURON_LAYER *nl_output = NULL, *nl_input = NULL;
    int nl_number, p_number;
    int xo, yo, wo, ho;
    
    //float avg_distance;
    
    // Checks the Neuron Layers Number
    for (nl_number = 0, n_list = filter_desc->neuron_layer_list; n_list != NULL; n_list = n_list->next, nl_number++)
        ;

    // Checks the Parameters Number
    for (p_number = 0, p_list = filter_desc->filter_params; p_list != NULL; p_list = p_list->next, p_number++)
        ;

    if (p_number != 1) {
        Erro("Error: Wrong number of parameters. The hamming_distance_filter must have no parameters", "", "");
        return;
    }

    // Gets the Filter Output 
    nl_output = filter_desc->output;

    // Gets the Input Neuron Layer
    nl_input = filter_desc->neuron_layer_list->neuron_layer;

    // Gets the Input Neuron Layer
    nl_input = filter_desc->neuron_layer_list->neuron_layer;

    // Gets the Filter Output
    nl_output = filter_desc->output;

    wo = nl_output->dimentions.x;
    ho = nl_output->dimentions.y;

    // Copies the hamming distance into the output layer
    for (yo = 0; yo < ho; yo++)
    {
        for (xo = 0; xo < wo; xo++)
        {
             nl_output->neuron_vector[xo + yo * wo].output.ival = (int) nl_input->neuron_vector[xo + yo * wo].last_hamming_distance;
             //avg_distance += (float) nl_input->neuron_vector[xo + yo * wo].last_hamming_distance;
        }
    }
    
    #if 0
    avg_distance /= (float) (wo*ho);
    
    for (yo = 0; yo < ho; yo++)
        for (xo = 0; xo < wo; xo++)
        	if((float)nl_input->neuron_vector[xo + yo * wo].last_hamming_distance < avg_distance)
        		nl_output->neuron_vector[xo + yo * wo].output.ival = 0;
    #endif
    
    #if 0 
    int i,j;
    int hamming[INPUTS_PER_NEURON];
    FILE *test_file;
    test_file = fopen("hamming.csv","a");
    
    for(i=0; i < INPUTS_PER_NEURON ;i++)
    	hamming[i] = 0;
    
    for(j=0;j<wo*ho;j++)
    	hamming[nl_input->neuron_vector[j].last_hamming_distance]++;
    
    for(i=0; i < INPUTS_PER_NEURON ;i++)
    	fprintf(test_file,"%d,",hamming[i]);
    	
    fprintf(test_file,"\n");
    fclose(test_file);
    	
    #endif
}

/*
 *********************************************************************************
 * Function:	hamming_distance_inhibition_filter				 *
 * Description: 			                        		 *
 * Inputs: 				                  			 *
 * Output: 				                			 *
 *********************************************************************************
 */
 
void hamming_distance_inhibition_filter(FILTER_DESC *filter_desc)
{    
    PARAM_LIST *p_list = NULL;
    NEURON_LAYER_LIST *n_list = NULL;
    NEURON_LAYER *nl_output = NULL, *nl_input = NULL;
    int nl_number, p_number;
    int xo, yo, wo, ho;
    
    //float avg_distance;
    
    // Checks the Neuron Layers Number
    for (nl_number = 0, n_list = filter_desc->neuron_layer_list; n_list != NULL; n_list = n_list->next, nl_number++)
        ;

    // Checks the Parameters Number
    for (p_number = 0, p_list = filter_desc->filter_params; p_list != NULL; p_list = p_list->next, p_number++)
        ;

    if (p_number != 1) {
        Erro("Error: Wrong number of parameters. The hamming_distance_filter must have no parameters", "", "");
        return;
    }

    // Gets the Filter Output 
    nl_output = filter_desc->output;

    // Gets the Input Neuron Layer
    nl_input = filter_desc->neuron_layer_list->neuron_layer;

    // Gets the Input Neuron Layer
    nl_input = filter_desc->neuron_layer_list->neuron_layer;

    // Gets the Filter Output
    nl_output = filter_desc->output;

    wo = nl_output->dimentions.x;
    ho = nl_output->dimentions.y;

    // Copies the hamming distance into the output layer
    for (yo = 0; yo < ho; yo++)
        for (xo = 0; xo < wo; xo++)
             nl_output->neuron_vector[xo + yo * wo].output.fval = nl_input->neuron_vector[xo + yo * wo].output.fval * angular_similarity(sqrt(hamming_squared_cosine_int(nl_input->neuron_vector[xo + yo * wo].last_hamming_distance,INPUTS_PER_NEURON)));
}
