#include"Calibration.h" 

namespace perls
{      
    /**
     * This function performs the gsl based gradient descent search for the transformation 
     * parameters 
     */
    float
    Calibration::gsl_minimizer (ssc_pose_t x0_hl)
    {
        double line_search_tol = 0.001;
        double gradient_tol = 0.001;
        double step_size = 0.5;
        int iter = 0;
        int status = GSL_CONTINUE;
    
        const gsl_multimin_fdfminimizer_type *T;
        gsl_multimin_fdfminimizer *gsl_minimizer;
        
        gsl_multimin_function_fdf my_func;
        my_func.n = 6; //6 dimensional space
        my_func.f = gsl_f;
        my_func.df = gsl_df;
        my_func.fdf = gsl_fdf;
        my_func.params = this;
     
        /* Starting point, x*/
        gsl_vector *x;
        x = gsl_vector_alloc (6);
        for(int i = 0; i < 6; i++)
          gsl_vector_set (x, i, x0_hl[i]);
    
        //Change this to use other methods
        //Conjugate gradient descent
        T =  gsl_multimin_fdfminimizer_conjugate_fr;
        //T = gsl_multimin_fdfminimizer_vector_bfgs2;
    
        gsl_minimizer = gsl_multimin_fdfminimizer_alloc (T, 6);
     
        //Conjugate gradient
        gsl_multimin_fdfminimizer_set (gsl_minimizer, &my_func, x, step_size, line_search_tol);
    
        while (status == GSL_CONTINUE && iter < 100)
        {
           iter++;
           status = gsl_multimin_fdfminimizer_iterate (gsl_minimizer);
     
           if (status) 
           {
             printf ("Minimum found at: status = %d gsl->f = %f\n", status, gsl_minimizer->f);
             break;
           }
    
           //Conjugate gradient 
           status = gsl_multimin_test_gradient (gsl_minimizer->gradient, gradient_tol);
    
           for (int i = 0; i < 6; i++)
             printf ("%f ", gsl_vector_get (gsl_minimizer->x, i));

           printf (" cost = %f status = %d\n", gsl_minimizer->f, status);
        }
    
        for (int i = 0; i < 6; i++)
           x0_hl[i] = gsl_vector_get (gsl_minimizer->x, i);
     
        if (status == GSL_SUCCESS)
           printf ("Converged\n");
        else
          printf ("Failed\n");
    
        gsl_multimin_fdfminimizer_free (gsl_minimizer);
        gsl_vector_free (x);
     
        return 0;
    }
    
    /**
     * This function performs the gsl based Nelder-Mead simplex search for the transformation 
     * parameters 
     */
    float
    Calibration::gsl_minimizer_nmsimplex (ssc_pose_t x0_hl, double step_trans, double step_rot)
    {
        double step_size = 1e-2;
        int iter = 0;
        int status;
        double size;
    
        const gsl_multimin_fminimizer_type *T;
        gsl_multimin_fminimizer *gsl_minimizer;
     
        
        gsl_multimin_function my_func;
        my_func.n = 6; //6 dimensional space
        my_func.f = gsl_f;
        my_func.params = this;
     
        /* Starting point, x*/
        gsl_vector *x;
        x = gsl_vector_alloc (6);
        for (int i = 0; i < 6; i++)
          gsl_vector_set (x, i, x0_hl[i]);
    
        //step size for Nelder-Mead simplex method
        gsl_vector *ss;
        ss = gsl_vector_alloc (6);
        for (int i = 0; i < 6; i++)
        {
          if (i < 3)
            gsl_vector_set (ss, i, step_trans); //10 cm
          else
            gsl_vector_set (ss, i, step_rot*DTOR); //0.5 degrees
        }
         
        //Nelder-Mead Simplex method
        T = gsl_multimin_fminimizer_nmsimplex2;
    
        gsl_minimizer = gsl_multimin_fminimizer_alloc (T, 6);
     
        //Nelder-Mead Simplex
        gsl_multimin_fminimizer_set (gsl_minimizer, &my_func, x, ss);
    
        do
        {
           iter++;
           status = gsl_multimin_fminimizer_iterate (gsl_minimizer);
     
           if (status)
             break;
     
           //Simplex method
           size = gsl_multimin_fminimizer_size (gsl_minimizer);
           status = gsl_multimin_test_size (size, step_size);
     
           if (status == GSL_SUCCESS)
             printf ("Minimum found at:\n");
    
           for (int i = 0; i < 6; i++)
             printf ("%f ", gsl_vector_get (gsl_minimizer->x, i));

        }while (status == GSL_CONTINUE && iter < 100);
    
        for (int i = 0; i < 6; i++)
           x0_hl[i] = gsl_vector_get (gsl_minimizer->x, i);
     
        gsl_multimin_fminimizer_free (gsl_minimizer);
        gsl_vector_free (x);
        gsl_vector_free (ss);
        return 0;
    } 
    
    /**
     * This function performs the grid based search for the transformation 
     * parameters, by alternating between the rotation and translation parameters.
     */
    float
    Calibration::grid_search_rotation (GridParam_t* gridParam)
    {
        //Temporary variables
        ssc_pose_t x0;
        ssc_pose_t x0_max;
        //Find the cost at grid center and assign it as max cost
        float max_cost = mi_cost (gridParam->gridCenter);
        //Set the grid center to be the curr optimal pose
        ssc_pose_set (x0_max, gridParam->gridCenter);
        //Fix the translation parameters
        double x = gridParam->gridCenter[0]; double y = gridParam->gridCenter[1]; double z = gridParam->gridCenter[2];
        double gridsize = gridParam->gszRot;
        double gridstep = gridParam->gstRot;
        for (double r = gridParam->gridCenter[3] - gridsize; r <= gridParam->gridCenter[3] + gridsize; r = r + gridstep)
        {
          for (double p = gridParam->gridCenter[4] - gridsize; p <= gridParam->gridCenter[4] + gridsize; p = p + gridstep)
          {
            for (double h = gridParam->gridCenter[5] - gridsize; h <= gridParam->gridCenter[5] + gridsize; h = h + gridstep)
            {
              x0[0] = x; x0[1] = y; x0[2] = z;
              x0[3] = r; x0[4] = p; x0[5] = h; 
              float curr_cost = mi_cost (x0);
              if (curr_cost > max_cost)
              {
                max_cost = curr_cost;
                ssc_pose_set (x0_max, x0);
              } 
            }
          }
        }
        //update the grid center  
        ssc_pose_set (gridParam->gridCenter, x0_max);
        return max_cost;
    } 
    
    float
    Calibration::grid_search_translation (GridParam_t* gridParam)
    {
        //Temporary variables
        ssc_pose_t x0;
        ssc_pose_t x0_max;
        //Find the cost at grid center and assign it as max cost
        float max_cost = mi_cost (gridParam->gridCenter);
        //Set the grid center to be the curr optimal pose
        ssc_pose_set (x0_max, gridParam->gridCenter);
        //Fix the rotation parameters
        double r = gridParam->gridCenter[3]; double p = gridParam->gridCenter[4]; double h = gridParam->gridCenter[5];
        double gridsize = gridParam->gszTrans;
        double gridstep = gridParam->gstTrans;
        //Loop over the translation grid
        for (double x = gridParam->gridCenter[0] - gridsize; x <= gridParam->gridCenter[0] + gridsize; x = x + gridstep)
        {
          for (double y = gridParam->gridCenter[1] - gridsize; y <= gridParam->gridCenter[1] + gridsize; y = y + gridstep)
          {
            for (double z = gridParam->gridCenter[2] - gridsize; z <= gridParam->gridCenter[2] + gridsize; z = z + gridstep)
            {
              x0[0] = x; x0[1] = y; x0[2] = z;
              x0[3] = r; x0[4] = p; x0[5] = h; 
              float curr_cost = mi_cost (x0);
              if (curr_cost > max_cost)
              {
                max_cost = curr_cost;
                ssc_pose_set(x0_max, x0);
              } 
            }
          }
        }
        //update the grid center  
        ssc_pose_set (gridParam->gridCenter, x0_max);
        return max_cost;
    }
    
    float
    Calibration::grid_search_rot_trans (ssc_pose_t x0_hl)
    {
        int level = 0;
        //create gridParam
        GridParam_t gridParam;
        ssc_pose_set (gridParam.gridCenter, x0_hl);
        
        gridParam.gszTrans = 0.30; //+- 30 cm
        gridParam.gstTrans = 0.05; //+- 5 cm
        gridParam.gszRot = 5*DTOR; //+- 5 degree
        gridParam.gstRot = 1*DTOR; //+- 1 degree 
        //Loop for number of levels (We perform 2 level search)  
        ssc_pose_t curr_optimal_pose;
        float max_cost;
        while (level < 2)
        {
            //Level 1 rotation search about the input transformation
            max_cost = grid_search_rotation (&gridParam); // ....(A)
            //Level 1 translation parameters (The rotation parameters are set to 
            //        the value obtained from rotation search in (A).
            max_cost = grid_search_translation (&gridParam); // ....(B)
            int loop = 1;
            ssc_pose_set (curr_optimal_pose, gridParam.gridCenter);
            
            printf ("Inside Loop Pose : %lf %lf %lf %lf %lf %lf\n",curr_optimal_pose[0], curr_optimal_pose[1], curr_optimal_pose[2],
                                   curr_optimal_pose[3]*RTOD, curr_optimal_pose[4]*RTOD, curr_optimal_pose[5]*RTOD); 
            //Loop untill we get the best grid center for level "i"
            while (loop)
            { 
                //Check if the parameters obtained 
                double cost = grid_search_rotation (&gridParam);
                cost = grid_search_translation (&gridParam);
                if (cost > max_cost)
                {
                    max_cost = cost;
                    ssc_pose_set (curr_optimal_pose, gridParam.gridCenter);       
                    //Perform translation search again for this set of rotation
                    printf ("Inside Loop Pose : %lf %lf %lf %lf %lf %lf\n",curr_optimal_pose[0], curr_optimal_pose[1], curr_optimal_pose[2],
                                       curr_optimal_pose[3]*RTOD, curr_optimal_pose[4]*RTOD, curr_optimal_pose[5]*RTOD); 
                }
                else
                {
                    loop = 0;
                    level++;
                } 
            }
            printf ("Level %d Pose : %lf %lf %lf %lf %lf %lf\n",level, curr_optimal_pose[0], curr_optimal_pose[1], curr_optimal_pose[2],
                                   curr_optimal_pose[3]*RTOD, curr_optimal_pose[4]*RTOD, curr_optimal_pose[5]*RTOD); 
            //Update the grid parameters
            gridParam.gszTrans = 0.05; //+- 4 cm
            gridParam.gstTrans = 0.01; //+- 1 cm
            gridParam.gszRot = 0.5*DTOR; //+- 1 degree
            gridParam.gstRot = 0.1*DTOR; //+- 0.1 degree 
        }
        ssc_pose_set (x0_hl, curr_optimal_pose);
        return max_cost;
    }

    /**
     * GSL minimizer function handlers
     */
    double 
    Calibration::gsl_f (const gsl_vector *x, void *params)
    {
        Calibration *ptr = (Calibration*) params; 
        ssc_pose_t x0;
        for (int i = 0; i < 6; i++)
          x0[i] = gsl_vector_get (x, i);

        float cost = ptr->mi_cost (x0);
        return (-cost);
    }
    void 
    Calibration::gsl_df (const gsl_vector *x0_hl, void *params, gsl_vector *g) 
    {
        Calibration *ptr = (Calibration*) params;
        ssc_pose_t xk;
        for (int i = 0; i < 6; i++)
          xk[i] = gsl_vector_get (x0_hl, i);
    
        double deltax = 0.01, deltay = 0.01, deltaz = 0.01;
        double deltar = 0.1*DTOR, deltap = 0.1*DTOR, deltah = 0.1*DTOR;
        double f_prev = -ptr->mi_cost (xk);
    
        double _f = 0; 
        double grad[6];    
    
        double x, y, z, r, p, h;
        x = xk[0]; y = xk[1]; z = xk[2];
        r = xk[3]; p = xk[4]; h = xk[5];
    
        double delta[6];
        delta[0] = x+deltax; delta[1] = y; delta[2] = z;
        delta[3] = r; delta[4] = p; delta[5] = h; 
        _f = -ptr->mi_cost (delta);
        grad[0] = (_f - f_prev)/deltax;
        
        delta[0] = x; delta[1] = y+deltay; delta[2] = z;
        delta[3] = r; delta[4] = p; delta[5] = h; 
        _f = -ptr->mi_cost (delta);
        grad[1] = (_f - f_prev)/deltay;
        
        delta[0] = x; delta[1] = y; delta[2] = z+deltaz;
        delta[3] = r; delta[4] = p; delta[5] = h; 
        _f = -ptr->mi_cost (delta);
        grad[2] = (_f - f_prev)/deltaz;
        
        delta[0] = x; delta[1] = y; delta[2] = z;
        delta[3] = r+deltar; delta[4] = p; delta[5] = h; 
        _f = -ptr->mi_cost (delta);
        grad[3] = (_f - f_prev)/deltar;
        
        delta[0] = x; delta[1] = y; delta[2] = z;
        delta[3] = r; delta[4] = p+deltap; delta[5] = h; 
        _f = -ptr->mi_cost (delta);
        grad[4] = (_f - f_prev)/deltap;
        
        delta[0] = x; delta[1] = y; delta[2] = z;
        delta[3] = r; delta[4] = p; delta[5] = h+deltah; 
        _f = -ptr->mi_cost (delta);
        grad[5] = (_f - f_prev)/deltah;
      
        for (int i = 0; i < 6; i++)
          gsl_vector_set (g, i, grad[i]);
    }
    void
    Calibration::gsl_fdf (const gsl_vector *x0_hl, void *params, double *f, gsl_vector *g) 
    {
        Calibration *ptr = (Calibration*) params;
        ssc_pose_t xk;
        for (int i = 0; i < 6; i++)
          xk[i] = gsl_vector_get (x0_hl, i);

        *f = -ptr->mi_cost (xk);
        
        double deltax = 0.01, deltay = 0.01, deltaz = 0.01;
        double deltar = 0.1*DTOR, deltap = 0.1*DTOR, deltah = 0.1*DTOR;
        double f_prev = *f;
    
        double _f = 0; 
        double grad[6];    
    
        double x, y, z, r, p, h;
        x = xk[0]; y = xk[1]; z = xk[2];
        r = xk[3]; p = xk[4]; h = xk[5];
    
        double delta[6];
        delta[0] = x+deltax; delta[1] = y; delta[2] = z;
        delta[3] = r; delta[4] = p; delta[5] = h; 
        _f = -ptr->mi_cost (delta);
        grad[0] = (_f - f_prev)/deltax;
        
        delta[0] = x; delta[1] = y+deltay; delta[2] = z;
        delta[3] = r; delta[4] = p; delta[5] = h; 
        _f = -ptr->mi_cost (delta);
        grad[1] = (_f - f_prev)/deltay;
        
        delta[0] = x; delta[1] = y; delta[2] = z+deltaz;
        delta[3] = r; delta[4] = p; delta[5] = h; 
        _f = -ptr->mi_cost (delta);
        grad[2] = (_f - f_prev)/deltaz;
        
        delta[0] = x; delta[1] = y; delta[2] = z;
        delta[3] = r+deltar; delta[4] = p; delta[5] = h; 
        _f = -ptr->mi_cost (delta);
        grad[3] = (_f - f_prev)/deltar;
        
        delta[0] = x; delta[1] = y; delta[2] = z;
        delta[3] = r; delta[4] = p+deltap; delta[5] = h; 
        _f = -ptr->mi_cost (delta);
        grad[4] = (_f - f_prev)/deltap;
        
        delta[0] = x; delta[1] = y; delta[2] = z;
        delta[3] = r; delta[4] = p; delta[5] = h+deltah; 
        _f = -ptr->mi_cost (delta);
        grad[5] = (_f - f_prev)/deltah;
    
        for (int i = 0; i < 6; i++)
          gsl_vector_set (g, i, grad[i]);
    }
};
