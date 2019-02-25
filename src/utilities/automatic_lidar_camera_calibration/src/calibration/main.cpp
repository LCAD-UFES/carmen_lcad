#include "Calibration.h"
#include <time.h>
#include <getopt.h>
#include "timestamp.h"

void print_help(int exval, char** argv) 
{
    printf("Usage:%s [-h] [-c FILE] \n\n", argv[0]);

    printf("  -h              print this help and exit\n");
    printf("  -c FILE         set config file\n");

    exit(exval);
}

char *
parse_args(int argc, char** argv)
{
    int opt;
    char *config_file = NULL;
    while((opt = getopt(argc, argv, "hc:")) != -1) 
    {
        switch(opt) 
        {
           case 'h':
            print_help(0, argv);
            break;
           case 'c':
            //get the config file name containing the camera parameters
            config_file = (char*)optarg;    
            break;
           case ':':
            fprintf(stderr, "Error - Option `%c' needs a value\n\n", optopt);
            print_help(1, argv);
            exit(0);
            break;
           case '?':
            fprintf(stderr, "Error - No such option: `%c'\n\n", optopt);
            print_help(1, argv);
         }
     }
    
     if(config_file == NULL) 
     {
       config_file = (char*)"../config/master.cfg";
       printf ("Using default config file: %s\n", config_file);
     }
    
     /* 
     // print all remaining options
     */
     for(; optind < argc; optind++)
      printf("argument: %s\n", argv[optind]);

     return config_file;
}


int
main (int argc, char** argv)
{
    //parse arguments
    char *config_file = parse_args (argc, argv);
    
    perls::Calibration calib (config_file);

    //Cost calculation 
    float cost = 0; 
    ssc_pose_t x0;
    for (int i = 0; i < 6; i++)
        x0[i] = calib.m_X0[i];

    //BB gradient descent search
    int64_t tic = timestamp_now ();
    printf ("****************************************************************************\n"); 
    printf ("Cost | x (m) | y (m) | z (m) | roll (degree) | pitch (degree) | yaw (degree)\n");  
    printf ("****************************************************************************\n"); 
    cost = calib.gradient_descent_search (calib.m_X0);
    printf ("****************************************************************************\n"); 
    int64_t toc = timestamp_now ();
    printf ("Time taken = %f seconds\n", (toc - tic)/1e6);
    printf ("****************************************************************************\n"); 
    printf ("Calibration parameters:\n");
    printf ("x       = %lf m\ny       = %lf m\nz       = %lf m\nroll    = %lf rad\npitch   = %lf rad\nheading = %lf rad\n", 
                       calib.m_X0[0], calib.m_X0[1], calib.m_X0[2], calib.m_X0[3], calib.m_X0[4], calib.m_X0[5]);
    printf ("****************************************************************************\n"); 

    //Save calibration parameters
    FILE *fptr_out = fopen ("calib_param.txt", "w"); 
    fprintf (fptr_out, "x       = %lf m\ny        = %lf m\nz        = %lf m\nroll     = %lf rad\npitch   = %lf rad\nheading = %lf rad\n", 
                       calib.m_X0[0], calib.m_X0[1], calib.m_X0[2], calib.m_X0[3], calib.m_X0[4], calib.m_X0[5]);
    fflush (fptr_out);
    fclose (fptr_out);

    //Calculate covariance
    gsl_matrix* cov = calib.calculate_covariance_matrix (calib.m_X0);
    
    FILE *fptr_cov = fopen ("calib_cov.txt", "w"); 
    printf ("Variance of parameters:\n"); 
    for (int i = 0; i < 6; i++)
    {
      for(int j = 0; j < 6; j++)
      {
        fprintf (fptr_cov, "%f ", (gsl_matrix_get (cov, i, j)));
        if(i == j)
        {
          if(i < 3)
            printf ("Std X[%d] = %f\n", i, sqrt (gsl_matrix_get (cov, i, j)));
          else
            printf ("Std X[%d] = %f\n", i, sqrt (gsl_matrix_get (cov, i, j)));
        }
      }
      fprintf( fptr_cov, "\n");
    }

    fflush (fptr_cov);
    fclose (fptr_cov);
    return 0;
}
