#include "Calibration.h"
#define KDE_METHOD
//#define CHI_SQUARE_TEST
//#define _DEBUG_

namespace perls
{
    Calibration::Calibration (char *configFile)
    {
        FILE *fptr = NULL;
        std::cout << configFile << std::endl;
        if (configFile)
        {
            fptr = fopen (configFile, "r");
            this->m_ConfigHandle = config_parse_file (fptr, NULL);
        }  
        else
            this->m_ConfigHandle = config_parse_default ();
        
        if (this->m_ConfigHandle == NULL)
        {
            printf ("Error in opening config file\n");
            return;
        }
    
        //get initial guess
        config_get_double_array (this->m_ConfigHandle, "calibration.initial_guess.X0", this->m_X0, 6);
        printf ("initial guess:\n");
        for (int i = 3; i < 6; i++)
            this->m_X0[i] = this->m_X0[i]*DTOR;
    
        //load scans
        char *scan_folder;
        if (config_get_str (this->m_ConfigHandle, "calibration.scan.scan_folder", &scan_folder) < 0)
        {
          printf ("Error: cannot read scan folder\n");
          return;
        }
        char *scan_base_name;
        scan_base_name = config_get_str_or_default (this->m_ConfigHandle, "calibration.scan.scan_base_name", (char*)"Scan");
        char *scan_type;
        scan_type = config_get_str_or_default (this->m_ConfigHandle, "calibration.scan.scan_type", (char*)"txt");
        int total_scans;
        total_scans = config_get_int_or_default (this->m_ConfigHandle, "calibration.scan.total_scans", 1);
        this->m_NumScans = config_get_int_or_default (this->m_ConfigHandle, "calibration.scan.num_scans_used", 1);
        int scans_random;
        scans_random = config_get_int_or_default (this->m_ConfigHandle, "calibration.scan.scans_randomly_sampled", 1);
        int *use_scans = (int*) malloc (sizeof (int)*this->m_NumScans);
        if (scans_random)
            //randomly sample
            get_random_numbers (1, total_scans, use_scans, this->m_NumScans);
        else
            config_get_int_array (this->m_ConfigHandle, "calibration.scan.use_scans", use_scans, this->m_NumScans);
     
        config_get_int (this->m_ConfigHandle, "calibration.cameras.num_cameras", &this->m_NumCams);

        //Load these scans and corresponding images  
        for (int s = 0; s < this->m_NumScans; s++)
        {
          char scan_file[256];
          this->m_scanIndex = use_scans[s];
          sprintf (scan_file, "%s/%s%04d.%s", scan_folder, scan_base_name,  use_scans[s], scan_type);
          printf ("%s\n", scan_file);
        
          //load scan
          if (load_point_cloud (scan_file) < 0)
            return;
        
          //load images
          if (load_image (use_scans[s]) < 0)
            return;
        }
        
        //load Mask
        load_mask ();
    
        //load camera parameters
        if (load_camera_parameters () < 0)
          return;
  
        //Number of bins used
        this->m_binFraction = config_get_int_or_default (this->m_ConfigHandle, "calibration.bin_fraction", 1);
        this->m_numBins = MAX_BINS/this->m_binFraction;
 
        this->m_estimatorType = config_get_int_or_default (this->m_ConfigHandle, "calibration.estimator_type", 1);
        //set target distribution;
#if 1
        this->m_jointTarget = cv::Mat::eye (this->m_numBins, this->m_numBins, CV_32FC1)/(this->m_numBins);
        //this->m_jointTarget = this->m_jointTarget + 1.0/(256*256);
        //this->m_jointTarget = this->m_jointTarget/2.0;
        //cv::GaussianBlur (this->m_jointTarget, this->m_jointTarget, cv::Size (0,0), 25);  
        this->m_grayTarget = cv::Mat::ones (1, this->m_numBins, CV_32FC1)/this->m_numBins;
        this->m_refcTarget = cv::Mat::ones (1, this->m_numBins, CV_32FC1)/this->m_numBins;
#endif

#if 0
        FILE *fptr_joint = fopen ("jointProb.txt", "r");
        FILE *fptr_refc = fopen ("refcProb.txt", "r");
        FILE *fptr_gray = fopen ("grayProb.txt", "r");
   
        this->m_jointTarget = cv::Mat::zeros (this->m_numBins, this->m_numBins, CV_32FC1);
        this->m_grayTarget = cv::Mat::zeros (1, this->m_numBins, CV_32FC1); 
        this->m_refcTarget = cv::Mat::zeros (1, this->m_numBins, CV_32FC1); 
        int tempi, tempj;
        float sum = 0;
        for (int i = 0; i < this->m_numBins; i++)
        {
            //for (int j = 0; j < this->m_numBins; j++)
            //{
            //    fscanf (fptr_joint, "%d %d %f\n", &tempi, &tempj, &this->m_jointTarget.at<float>(i, j));
            //}
            fscanf (fptr_refc, "%d %f\n", &tempi, &this->m_refcTarget.at<float>(i));
            fscanf (fptr_gray, "%d %f\n", &tempi, &this->m_grayTarget.at<float>(i));
            this->m_jointTarget.at<float> (i, i) = this->m_refcTarget.at<float>(i) * this->m_grayTarget.at<float>(i);
            //sum = sum + this->m_jointTarget.at<float> (i, i);
        }
        //this->m_jointTarget = this->m_jointTarget/sum; 
        //cv::GaussianBlur (this->m_jointTarget, this->m_jointTarget, cv::Size (0,0), 10);  
        
        fclose (fptr_joint);  
        fclose (fptr_refc);  
        fclose (fptr_gray);  
#endif

        //free
        free (scan_base_name);
        free (scan_type);
        free (scan_folder);
        free (use_scans);
        return;
    }

    Calibration::~Calibration ()
    {
        //release images
        for (int i = 0; i < this->m_NumScans; i++)
            release_image (&this->m_vecImage[i]);

        release_image (&this->m_Mask);
    }
    
    /**
     * This function loads the camera parameters from the file.
     */
    int 
    Calibration::load_camera_parameters ()
    {
        fasttrig_init ();	
        //Get the number of Cameras used
        config_get_int (this->m_ConfigHandle, "calibration.cameras.num_cameras", &this->m_NumCams);
        //Allocate memory for intrinsic parameters
        this->m_Calib.K = (double**) malloc (sizeof (double*)*this->m_NumCams);
        this->m_Calib.X_hi = (ssc_pose_t*) malloc (sizeof (ssc_pose_t)*this->m_NumCams);

        for (int i = 0; i < this->m_NumCams; i++)
          this->m_Calib.K[i] = (double*) malloc (sizeof (double)*9);
        
        for (int i = 0; i < this->m_NumCams; i++)
        {
            double focal_length, camera_center_X, camera_center_Y, scale_x, scale_y;
            char str[256];
            sprintf (str, "calibration.cameras.camera_%d.focal_length", i);
            if (config_get_double (this->m_ConfigHandle, str, &focal_length) < 0)
            {
              printf ("Error: Cannot load focal_length for camera_%d\n", i);
              return -1;
            }
            sprintf (str, "calibration.cameras.camera_%d.camera_center_X", i);
            if (config_get_double (this->m_ConfigHandle, str, &camera_center_X) < 0)
            {
              printf ("Error: Cannot load camera_center_X for camera_%d\n", i);
              return -1;
            }
            sprintf (str, "calibration.cameras.camera_%d.camera_center_Y", i);
            if (config_get_double (this->m_ConfigHandle, str, &camera_center_Y) < 0)
            {
              printf ("Error: Cannot load camera_center_Y for camera_%d\n", i);
              return -1;
            }
            sprintf (str, "calibration.cameras.camera_%d.scale_x", i);
            if (config_get_double (this->m_ConfigHandle, str, &scale_x) < 0)
            {
              printf ("Error: Cannot load scale_x for camera_%d\n", i);
              return -1;
            }
            sprintf (str, "calibration.cameras.camera_%d.scale_y", i);
            if (config_get_double (this->m_ConfigHandle, str, &scale_y) < 0)
            {
              printf ("Error: Cannot load scale_y for camera_%d\n", i);
              return -1;
            }
            
            //generate K matrix
            this->m_Calib.K[i][0] = focal_length*scale_x;
            this->m_Calib.K[i][1] = 0; 
            this->m_Calib.K[i][2] = camera_center_X;
            this->m_Calib.K[i][3] = 0;
            this->m_Calib.K[i][4] = focal_length*scale_y;
            this->m_Calib.K[i][5] = camera_center_Y;
            this->m_Calib.K[i][6] = 0; this->m_Calib.K[i][7] = 0; this->m_Calib.K[i][8] = 1;
            
            sprintf (str, "calibration.cameras.camera_%d.X_hc", i);
            config_get_double_array (this->m_ConfigHandle, str, this->m_Calib.X_hi[i], 6);
            //Convert to radians
            for (int j = 3; j < 6; j++)
              this->m_Calib.X_hi[i][j] = this->m_Calib.X_hi[i][j]*DTOR;
        }
        printf ("Cam Param loaded \n");
        return 0;
    }
   
    /**
     * This function loads the Scan from the file.
     */
    int 
    Calibration::load_point_cloud (char* filename)
    {
        //open file to read
        std::cout << "Loading " << filename << std::endl;
        FILE *fptr = fopen (filename, "r");
        if (fptr == NULL)
        {
            std::cout << "Could not open '" << filename << "'." << std::endl;
            exit (0);
        }
        int numPoints = 0;
        fscanf (fptr, "%d\n", &numPoints); 

        double DIST_THRESH = 10000;
        //read all points
        PointCloud_t pointCloud; 
        while (!feof (fptr))
        {
            Point3d_t point;
            fscanf (fptr, "%f %f %f %d\n", &point.x, &point.y, &point.z, &point.refc);
            double dist = point.x*point.x + point.y*point.y + point.z*point.z;
            point.range = dist/DIST_THRESH; 
            pointCloud.points.push_back (point);
            numPoints++;
        }
        this->m_vecPointClouds.push_back (pointCloud);
        std::cout << "Num points loaded = " << pointCloud.points.size () << std::endl;
        fflush (fptr);
        fclose (fptr);
        return 0;
    }
    
    /**
     * This function loads the images
     */
    int
    Calibration::load_image (int imageIndex)
    {
        char imageName[512];
        char *image_base_name;
        image_base_name = config_get_str_or_default (this->m_ConfigHandle, "calibration.cameras.image_base_name", (char*)"image");
        char *image_type;
        image_type = config_get_str_or_default (this->m_ConfigHandle, "calibration.cameras.image_type", (char*)"ppm");
       
        Image_t currImage; 
        for (int i = 0; i < this->m_NumCams; i++)
        {
            char *image_folder;
            char str[256];
            sprintf (str, "calibration.cameras.camera_%d.folder", i);
            config_get_str (this->m_ConfigHandle, str, &image_folder);
            sprintf (imageName,"%s/%s%04d.%s", image_folder, image_base_name, imageIndex, image_type);	
            printf ("Loading file-: %s\n",imageName);
            
            IplImage* iplimage = cvLoadImage (imageName, 0); //load image as grayscale.
            if (iplimage == NULL)
            {
               printf ("Error: Cannot load Image: %s\n", imageName);
               return -1;
            }
            //gaussian smoothing of images
            IplImage* out = cvCreateImage (cvGetSize(iplimage), IPL_DEPTH_8U, 1);
            cv::Mat outMat = cv::cvarrToMat(out);
            cv::Mat imageMat = cv::cvarrToMat(iplimage);
            cv::GaussianBlur (imageMat, outMat, cv::Size (3, 3), 0.75); 
            currImage.image.push_back (out);
            cvReleaseImage (&iplimage); 
        }
        this->m_vecImage.push_back (currImage);
        return 0;
    }
    
    /**
     * This function loads the mask. Mask is the same for all images
     */ 
    void
    Calibration::load_mask ()
    {
        char *maskName;
        for (int i = 0; i < this->m_NumCams ; i++)
        {
            char str[256];
            sprintf (str, "calibration.cameras.camera_%d.mask", i);
            int ret = config_get_str (this->m_ConfigHandle, str, &maskName);
            if (ret < 0)
            {
                this->m_Mask.image.push_back (NULL);
            }
            else
            {
                printf ("Loading file-: %s\n",maskName);
                IplImage *iplmask = cvLoadImage (maskName, 0); 
                this->m_Mask.image.push_back (iplmask);
            }
        }
    }
    
    /**
     * This function releases the image memory.
     */
    int 
    Calibration::release_image (Image_t* image)
    {
        for (int i = 0; i < this->m_NumCams; i++)
            cvReleaseImage (&image->image[i]);

        return 0;
    }
    
    /**
     * This function generates random numbers
     */ 
    void 
    Calibration::get_random_numbers (int min, int max, int* unique_random_index, int num)
    {
        srand (time (NULL));
        //Randomly select NUM_SCANS
        for (int s = 0; s < num; s++)
        {
          int rand_index = rand ()%(max - min) + min;
          if (s == 0){
            unique_random_index[s] = 1; //rand_index;
            continue;
          }
          //check if its unique
          int unique = 1;
          while (1){
            for (int temp = 0; temp < s; temp++){
              if ((rand_index - unique_random_index[temp] == 0)) 
               unique = 0;
            }
            if (unique)
              break;
            unique = 1;
            rand_index = rand ()%(max - min) + min;
          }
          unique_random_index[s] = rand_index;
          printf ("%d %d\n", s, rand_index);
        }
        //printf ("Coming out of get_random_numbers ()\n");
    }
    
    /**
     * This function computes the smoothed distribution at a given transformation x
     */
    Histogram
    Calibration::get_histogram (ssc_pose_t x0)
    {
        //Get the transformation
        ssc_pose_t X_il;
        double **t = (double**) malloc (sizeof (double*)*this->m_NumCams);
        double **R = (double**) malloc (sizeof (double*)*this->m_NumCams);
        
        for (int i = 0; i < this->m_NumCams; i++)
        {
            t[i] = (double*) malloc (sizeof (double)*3);
            R[i] = (double*) malloc (sizeof (double)*9);
            ssc_pose_t X_ih;
            ssc_inverse (X_ih, NULL, this->m_Calib.X_hi[i]);
            ssc_head2tail (X_il, NULL, X_ih, x0);
            ssc_pose_get_xyz (X_il, t[i]);
            double rph[3];
            ssc_pose_get_rph (X_il, rph);
            so3_rotxyz (R[i], rph);  
        }
        
        //count the number of points that project onto the valid image region
        double temp[3];
        double camera_xyz[3]; //point in camera ref
        double laser_xyz[3]; //point in laser ref
        float gray_sum = 0;
        float refc_sum = 0;

        Histogram hist (this->m_numBins);
        
        //for every scan
        for (int scan_index = 0; scan_index < this->m_NumScans; scan_index++)
        {
            Image_t *image = &this->m_vecImage[scan_index];
            PointCloud_t scan = this->m_vecPointClouds[scan_index]; 
            //For debugging
            #ifdef _DEBUG_
              Image_t debug_dest;
              char **window_name = (char**) malloc (sizeof (char*)*this->m_NumCams);
              for (int i = 0; i < this->m_NumCams; i++)
              {
                  window_name[i] = (char*) malloc (256);
                  debug_dest.image.push_back (cvCloneImage (image->image[i])); 
              }
            #endif
            
            //Loop over all points 
            //std::cout << "Loop over " << scan.points.size () << " points" << std::endl;
            for (int i = 0; i < scan.points.size (); i++)
            {
                //printf("count = %d\n", count); 
                //Get the point
                Point3d_t point = scan.points[i]; 
                //printf ("x = %lf, y = %lf, z = %lf, refc = %d\n", point.x, point.y, point.z, point.refc);
                //project it on each camera
                for (int j = 0; j < this->m_NumCams; j++)
                {
                    //project laser point into j'th camera
                    //point_in_camera_ref = [R, t]*[scan->X[i], scan->Y[i], scan->Z[i]]';
                    laser_xyz[0] = point.x; laser_xyz[1] = point.y; laser_xyz[2] = point.z;
                    if (point.range < 1)
                    {
                        matrix_vector_multiply_3x3_3d (R[j], laser_xyz, temp);
                        vector_add_3d (t[j], temp, camera_xyz);
                        if (camera_xyz[2] > 0)
                        {
                            //this point lies infront of this camera
                            //calculate projection on image
                            double image_point[3];
                            matrix_vector_multiply_3x3_3d (this->m_Calib.K[j], camera_xyz, image_point);
                            int u = image_point[0]/image_point[2];
                            int v = image_point[1]/image_point[2];
                            //printf("u = %d, v = %d\n", u, v);
                            //if image_point is within the frame
                            if ((u > 1) && (u < image->image[j]->width) &&  (v > 1) && (v < image->image[j]->height))
                            {
                                //get the gray scale value 
                                IplImage *img = image->image[j];
                                IplImage *msk = this->m_Mask.image[j]; 
                                int gray = 0; int mask = 1; 
                                if (msk)
                                  mask = ((uchar*)(msk->imageData + v*msk->widthStep))[u];
    
                                if (mask > 0) // && edge > 0) 
                                { 
                                    //get weighted average of the grayscale
                                    gray = ((uchar*)(img->imageData + v*img->widthStep))[u]/this->m_binFraction;
                                    int refc = point.refc/this->m_binFraction;
                                    //printf("Refc = %d, Grayscale val = %d\n", refc, gray);  
                                    //Update histograms
                                    hist.grayHist.at<float>(gray) = hist.grayHist.at<float>(gray) + 1; 
                                    hist.refcHist.at<float>(refc) = hist.refcHist.at<float>(refc) + 1; 
                                    hist.jointHist.at<float>(gray, refc) = hist.jointHist.at<float>(gray, refc) + 1; 
                                    hist.count++;
                                    gray_sum = gray_sum + gray;
                                    refc_sum = refc_sum + refc; 
                                    #ifdef _DEBUG_
                                      //Plot the 3D points on the image
                                      if (point.z > -2.0)
                                      {
                                          CvPoint c0 = cvPoint (cvRound (u), cvRound (v));
                                          cvCircle (debug_dest.image[j], c0, 2, CV_RGB(refc, 0, 0), -1, 4, 0); //-1, 8, 0);
                                      }
                                    #endif
                                }
                            }
                        } //if(camera_xyz[2] > 0)
                    } //if(dist < DIST_THRESH)
                } //for(int j = 0; j < this->m_NumCams; j++)
            } //for (int i = 0; i < scan->numPoints; i++)
            #ifdef _DEBUG_
              //Display the projected points
              for (int i = 0; i < this->m_NumCams; i++)
              {
                  //Create a window
                  sprintf (window_name[i], "SCAN Image %d", i);
                  cvNamedWindow (window_name[i], 1 );
                  //Display image with projected 3D points
                  cvShowImage (window_name[i], debug_dest.image[i]);
                  //Wait for user to press a key
                  cvWaitKey (0);
                  //Release the memory used for clone image
                  cvReleaseImage (&debug_dest.image[i]);
                  //Destroy the opencv window
                  cvDestroyWindow (window_name[i]);
              }
            #endif
        } //for(int scan_index = 0; scan_index < numScans; scan_index++)
    
        //printf ("count = %d\n", hist.count); 
        //Free memory
        for (int i = 0; i < this->m_NumCams; i++)
        {
            free (t[i]);    
            free (R[i]);    
        }
        free (t);
        free (R);

        hist.gray_sum = gray_sum;
        hist.refc_sum = refc_sum;
        return hist;
    }

    Probability
    Calibration::get_probability_MLE (Histogram hist)
    {
        //Calculate sample covariance matrix
        float mu_gray = hist.gray_sum/hist.count;
        float mu_refc = hist.refc_sum/hist.count;
        //std::cout << mu_gray << " " << mu_refc << std::endl;
        //Covariances
        double sigma_gray = 0;
        double sigma_refc = 0;
        //Cross correlation
        double sigma_gr = 0;
        
        Probability probMLE (this->m_numBins);
        
        for (int i = 0; i < this->m_numBins; i++)
        {
           for (int j = 0; j < this->m_numBins; j++)
           {
             //Cross Correlation term;
             sigma_gr = sigma_gr + hist.jointHist.at<float>(i, j)*(i - mu_refc)*(j - mu_gray);
             //Normalize the histogram so that the value is between (0,1)
             probMLE.jointProb.at<float>(i, j) = hist.jointHist.at<float>(i, j)/(hist.count);
           }
    
           //calculate sample covariance 
           sigma_gray = sigma_gray + (hist.grayHist.at<float>(i)*(i - mu_gray)*(i - mu_gray));
           sigma_refc = sigma_refc + (hist.refcHist.at<float>(i)*(i - mu_refc)*(i - mu_refc));
           
           probMLE.grayProb.at<float>(i) = hist.grayHist.at<float>(i)/hist.count;
           probMLE.refcProb.at<float>(i) = hist.refcHist.at<float>(i)/hist.count;
        }
    
        sigma_gray = sigma_gray/hist.count;
        sigma_refc = sigma_refc/hist.count;
        sigma_gr = sigma_gr/hist.count;
        double corr_coeff = ((sigma_gr)/(sigma_gray*sigma_refc));
        corr_coeff = sqrt (corr_coeff*corr_coeff);
        this->m_corrCoeff = corr_coeff;
        
        //Compute the optimal bandwidth (Silverman's rule of thumb)
        sigma_gray = 1.06*sqrt (sigma_gray)/pow (hist.count, 0.2);
        sigma_refc = 1.06*sqrt (sigma_refc)/pow (hist.count, 0.2); 
        
        #ifdef _DEBUG_
          printf ("mu_gray = %f, std_gray = %lf\nmu_refc = %f, std_refc = %lf\n", mu_gray, 
                              sigma_gray, mu_refc, sigma_refc);
          FILE *fptr = fopen ("gray_prob.txt", "w");
          for (int i = 0; i < this->m_numBins; i++)
              fprintf (fptr, "%f ", probMLE.grayProb.at<float>(i));
          fflush (fptr);
          fclose (fptr);
        #endif
        cv::GaussianBlur (probMLE.grayProb, probMLE.grayProb, cv::Size(0, 0), sigma_gray);
        #ifdef _DEBUG_
          fptr = fopen ("gray_prob_smooth.txt", "w");
          for (int i = 0; i < this->m_numBins; i++)
              fprintf (fptr, "%f ", probMLE.grayProb.at<float>(i));
          fflush (fptr);
          fclose (fptr);
        #endif
        
        cv::GaussianBlur (probMLE.refcProb, probMLE.refcProb, cv::Size(0, 0), sigma_refc);
        
        cv::GaussianBlur (probMLE.jointProb, probMLE.jointProb, cv::Size(0, 0), sigma_gray, sigma_refc);
        probMLE.count = hist.count; 
        return probMLE; 
    }

    /**
     * This function calculates the JS estimate from the MLE.
     */
    Probability 
    Calibration::get_probability_JS (Probability probMLE)
    {
        //Calculate JS estimate
        //Estimate lamda from the data
        /** 
        //Reference:[1] Entropy inference and the James Stein estimator. Hausser and Strimmer.
        //Sample Variance of MLE
        //Using unbiased estimator of variance as given in [1]:
        //Var(\theta_k) = \frac{\theta_k(1 - \theta_k)}{n-1}
        //=> \lambda = \frac{1 - \sum_{k=0}^{K} (\theta_k)^2}{\sum_{k=0}^{K} (t_k - \theta_k)
        //Here t_k      = target distribution (here m_jointTarget)
        //     \theta_k = MLE estimate (here probMLE.jointProb)  
        **/
        float squareSumMLE = cv::norm (probMLE.jointProb);
        squareSumMLE = (squareSumMLE*squareSumMLE); 
        //Difference of MLE from the target 
        float squareDiffMLETarget = cv::norm (this->m_jointTarget, probMLE.jointProb);
        squareDiffMLETarget =  (squareDiffMLETarget*squareDiffMLETarget);
         
        float lambda = (1.0-squareSumMLE)/squareDiffMLETarget;
        lambda = (lambda/(probMLE.count-1));
        //lambda = squareDiffMLETarget/squareSumMLE;
        //lambda = sqrt(this->m_corrCoeff); 
        std::cout << lambda << " " << squareSumMLE << " " << squareDiffMLETarget << std::endl;
        //lambda = 1 - sqrt(squareSumMLE); 
        //lambda = 0.0; 
        if (lambda > 1)
            lambda = 1;
        if (lambda < 0)
            lambda = 0; 
        
        //Scale the target distribution by lambda
        //Scale the MLE or the histograms by (1-lambda)  
        //Get the JS estimate as a weighted combination of target and the MLE
        Probability probJS (this->m_numBins);

        probJS.jointProb = this->m_jointTarget*lambda + probMLE.jointProb*(1.0-lambda);
        //cv::GaussianBlur (jointJSEstimate, jointJSEstimate, cv::Size(5, 5), 1.2, 1.2);
        probJS.grayProb = this->m_grayTarget*lambda + probMLE.grayProb*(1.0-lambda);
        //cv::GaussianBlur (margJSEstimate1, margJSEstimate1, cv::Size(1, 5), 0, 1.2);
        probJS.refcProb = this->m_refcTarget*lambda + probMLE.refcProb*(1.0-lambda);
        //cv::GaussianBlur (margJSEstimate2, margJSEstimate2, cv::Size(1, 5), 0, 1.2);
        
        return probJS;
    }
    
    /**
     * This calculates the Bayes estimate of distribution
     */ 
    Probability 
    Calibration::get_probability_Bayes (Histogram hist)
    {
        float a = 1; //0.5 , 1/this->m_numBins, sqrt (count)/this->m_numBins etc
        float A_joint = this->m_numBins*this->m_numBins;
        float A_marg = this->m_numBins;
        Probability probBayes (this->m_numBins);
    
        //Calculate sample covariance matrix
        float mu_gray = hist.gray_sum/hist.count;
        float mu_refc = hist.refc_sum/hist.count;
        std::cout << mu_gray << " " << mu_refc << std::endl;
        //Covariances
        double sigma_gray = 0;
        double sigma_refc = 0;
        //Cross correlation
        double sigma_gr = 0;
        
        for (int i = 0; i < this->m_numBins; i++)
        {
           for (int j = 0; j < this->m_numBins; j++)
           {
             //Cross Correlation term;
             sigma_gr = sigma_gr + hist.jointHist.at<float>(i, j)*(i - mu_refc)*(j - mu_gray);
             //Normalize the histogram so that the value is between (0,1)
             probBayes.jointProb.at<float>(i, j) = (hist.jointHist.at<float>(i, j)+a)/(hist.count + A_joint);
           }
    
           //calculate sample covariance 
           sigma_gray = sigma_gray + (hist.grayHist.at<float>(i)*(i - mu_gray)*(i - mu_gray));
           sigma_refc = sigma_refc + (hist.refcHist.at<float>(i)*(i - mu_refc)*(i - mu_refc));
           
           probBayes.grayProb.at<float>(i) = (hist.grayHist.at<float>(i) + a)/(hist.count + A_marg);
           probBayes.refcProb.at<float>(i) = (hist.refcHist.at<float>(i) + a)/(hist.count + A_marg);
        }
    
        sigma_gray = sigma_gray/hist.count;
        sigma_refc = sigma_refc/hist.count;
        sigma_gr = sigma_gr/hist.count;
        double corr_coeff = ((sigma_gr)/(sigma_gray*sigma_refc));
        corr_coeff = sqrt (corr_coeff*corr_coeff);
        this->m_corrCoeff = corr_coeff;
        
        //Compute the optimal bandwidth (Silverman's rule of thumb)
        sigma_gray = 1.06*sqrt (sigma_gray)/pow (hist.count, 0.2);
        sigma_refc = 1.06*sqrt (sigma_refc)/pow (hist.count, 0.2); 
        
        cv::GaussianBlur (probBayes.grayProb, probBayes.grayProb, cv::Size(0, 0), sigma_gray);
        
        cv::GaussianBlur (probBayes.refcProb, probBayes.refcProb, cv::Size(0, 0), sigma_refc);
        
        cv::GaussianBlur (probBayes.jointProb, probBayes.jointProb, cv::Size(0, 0), sigma_gray, sigma_refc);

        return probBayes; 
    }
 
    /**
     * This function calculates the cost based on mutual information with multiple scans
     */
    float
    Calibration::mi_cost (ssc_pose_t x0)
    {
        //Get MLE of probability distribution
        Histogram hist = get_histogram (x0);
        Probability prob;
        Probability probMLE; 
        switch (this->m_estimatorType)
        {
            case 1: //MLE
                prob = get_probability_MLE (hist);
                break;
            case 2: //James-Stein type
                probMLE = get_probability_MLE (hist);
                prob = get_probability_JS (probMLE); 
                break;
            case 3: //Bayes estimator
                prob = get_probability_Bayes (hist);
                break;
        }

        //Calculate log of JS estimate
        cv::Mat jointLog = cv::Mat::zeros(this->m_numBins, this->m_numBins, CV_32FC1);
        cv::Mat grayLog = cv::Mat::zeros(1, this->m_numBins, CV_32FC1); 
        cv::Mat refcLog = cv::Mat::zeros(1, this->m_numBins, CV_32FC1);
 
        cv::log (prob.jointProb, jointLog);
        cv::log (prob.grayProb, grayLog);
        cv::log (prob.refcProb, refcLog);
        
        cv::Mat jointEntropyMat, grayEntropyMat, refcEntropyMat;
        //jointEntropyMat = jointJSEstimate*jointJSLog;
        cv::multiply (prob.jointProb, jointLog, jointEntropyMat);
        //margEntropyMat1 = margJSEstimate1*margJSLog1;
        cv::multiply (prob.grayProb, grayLog, grayEntropyMat); 
        //margEntropyMat2 = margJSEstimate2*margJSLog2;
        cv::multiply (prob.refcProb, refcLog, refcEntropyMat); 
        
        //Sum all the elements
        float Hx  = cv::norm (grayEntropyMat, cv::NORM_L1);
        float Hy  = cv::norm (refcEntropyMat, cv::NORM_L1);
        float Hxy = cv::norm (jointEntropyMat, cv::NORM_L1);
        
        float cost = Hx + Hy - Hxy;
        //float cost = Hxy;
        return cost;
    }
    
    /**
     * This function calculates the chi square cost
     */
    float 
    Calibration::chi_square_cost (ssc_pose_t x)
    {
        //Get the joint and marginal probabilities
        Histogram hist = get_histogram (x);
        Probability P = get_probability_MLE (hist);
        
        //Calculate the chi square cost
        //sum (P(x,y) - P(x)P(y))^2/(P(x).P(y))
        float cost = 0;
        for (int i = 0; i < this->m_numBins; i++)
        {
          for (int j = 0; j < this->m_numBins; j++)
          {
            if (P.refcProb.at<float>(i) > 0 && P.grayProb.at<float>(j) > 0)
              cost = cost + (P.jointProb.at<float>(i, j) - P.refcProb.at<float>(i)*P.grayProb.at<float>(j))*
                   (P.jointProb.at<float>(i, j) - P.refcProb.at<float>(i)*P.grayProb.at<float>(j))/(P.refcProb.at<float>(i)*P.grayProb.at<float>(j));
          }
        }
        //printf ("chi_square_cost = %lf\n", cost);
        return cost;
    }

    /**
     * This function performs the exhaustive grid based search for the transformation 
     * parameters
     */
    float
    Calibration::exhaustive_grid_search (ssc_pose_t x0_hl)
    {
        //create the 1st level grid around x0_hl
        //1st level grid : 
        //[x, y, z] = +-0.20m and step = 0.05m
        //[r, p, h] = +-3 degrees and step = +-1 degree
        ssc_pose_t x0;
        ssc_pose_t x0_max_l1;
        ssc_pose_t x0_max_l2;
     
        double gridsize_trans = 0.2;
        double step_trans = 0.05;
        double gridsize_rot = 3*DTOR;
        double step_rot = 1*DTOR;
        double max_cost_l1 = 0; 
        ssc_pose_set (x0_max_l1, x0_hl);
    
        float curr_cost = 0;
        for (double x = x0_hl[0] - gridsize_trans; x <= x0_hl[0] + gridsize_trans; x = x + step_trans)
        {
          for (double y = x0_hl[1] - gridsize_trans; y <= x0_hl[1] + gridsize_trans; y = y + step_trans)
          {
            for (double z = x0_hl[2] - gridsize_trans; z <= x0_hl[2] + gridsize_trans; z = z + step_trans)
            {
              for (double r = x0_hl[3] - gridsize_rot; r <= x0_hl[3] + gridsize_rot; r = r + step_rot)
              {
                for (double p = x0_hl[4] - gridsize_rot; p <= x0_hl[4] + gridsize_rot; p = p + step_rot)
                {
                  for (double h = x0_hl[5] - gridsize_rot; h <= x0_hl[5] + gridsize_rot; h = h + step_rot)
                  {
                    x0[0] = x; x0[1] = y; x0[2] = z;
                    x0[3] = r; x0[4] = p; x0[5] = h; 
                    curr_cost = this->mi_cost (x0);
    
                    if (curr_cost > max_cost_l1)
                    {
                      max_cost_l1 = curr_cost;
                      ssc_pose_set (x0_max_l1, x0);
                      //printf("%lf %lf %lf %lf %lf %lf %lf\n", max_cost_l1, x0[0], x0[1], x0[2], x0[3]*RTOD, x0[4]*RTOD, x0[5]*RTOD);
                    } 
                  }
                }
              }
            }
          }
        }
    
        printf ("Level 1 Grid search done\n"); 
        printf ("%lf %lf %lf %lf %lf %lf %lf\n", max_cost_l1, x0_max_l1[0], x0_max_l1[1], x0_max_l1[2], x0_max_l1[3]*RTOD, x0_max_l1[4]*RTOD, x0_max_l1[5]*RTOD);
        //create the second level grid around the point with max cost in level 1 search
        //2nd level grid :
        //[x, y, z] = +-0.05m and step = 0.01m
        //[r, p, h] = +-1 degree and step = 0.1 degree
        //max cost point
        ssc_pose_set (x0_hl, x0_max_l1); 
        step_trans = 0.01;
        gridsize_trans = 0.04;
        gridsize_rot = 0.5*DTOR;
        step_rot = 0.1*DTOR;
        float max_cost_l2 = this->mi_cost (x0_max_l1);
        ssc_pose_set (x0_max_l2, x0_max_l1);
     
        curr_cost = 0; 
        for (double x = x0_hl[0] - gridsize_trans; x <= x0_hl[0] + gridsize_trans; x = x + step_trans)
        {
          for (double y = x0_hl[1] - gridsize_trans; y <= x0_hl[1] + gridsize_trans; y = y + step_trans)
          {
            for (double z = x0_hl[2] - gridsize_trans; z <= x0_hl[2] + gridsize_trans; z = z + step_trans)
            {
              for (double r = x0_hl[3] - gridsize_rot; r <= x0_hl[3] + gridsize_rot; r = r + step_rot)
              {
                for (double p = x0_hl[4] - gridsize_rot; p <= x0_hl[4] + gridsize_rot; p = p + step_rot)
                {
                  for (double h = x0_hl[5] - gridsize_rot; h <= x0_hl[5] + gridsize_rot; h = h + step_rot)
                  {
                    x0[0] = x; x0[1] = y; x0[2] = z;
                    x0[3] = r; x0[4] = p; x0[5] = h; 
                    curr_cost = this->mi_cost (x0);
    
                    if (curr_cost > max_cost_l2)
                    {
                      max_cost_l2 = curr_cost;
                      ssc_pose_set(x0_max_l2, x0);
                    } 
                  }
                }
              }
            }
          }
        }
        
        //Set x0_hl to the maxima.
        ssc_pose_set (x0_hl, x0_max_l2); 
        return max_cost_l2;
    }

    /**
     * This function performs the gradient descent search for the transformation 
     * parameters
     */
    float
    Calibration::gradient_descent_search (ssc_pose_t x0_hl)
    {
        //step parameter
        double gama_trans = 0.01;
        double gama_rot = 0.001;
        double gama_trans_u = 0.1;
        double gama_trans_l = 0.001;
        double gama_rot_u = 0.05;
        double gama_rot_l = 0.0005;
    
        double deltax = 0.01, deltay = 0.01, deltaz = 0.01;
        double deltar = 0.1*DTOR, deltap = 0.1*DTOR, deltah = 0.1*DTOR;
     
        int index = 0;
        ssc_pose_t xk, xk_minus_1;
        ssc_pose_set (xk, x0_hl);
        ssc_pose_set (xk_minus_1, x0_hl);
        int MAX_ITER = 300;
        double delF_delX_ = 0, delF_delY_ = 0, delF_delZ_ = 0;
        double delF_delR_ = 0, delF_delP_ = 0, delF_delH_ = 0;
        double f_max = 0;
        while (index < MAX_ITER)
        {
            //Evaluate function value
            double f_prev;
            #ifdef CHI_SQUARE_TEST
              f_prev = chi_square_cost (xk);
            #else
              f_prev = mi_cost (xk);
            #endif
            if (f_prev > f_max)
                f_max = f_prev;
    
            double _f = 0; 
    
            double x, y, z, r, p, h;
            x = xk[0]; y = xk[1]; z = xk[2];
            r = xk[3]; p = xk[4]; h = xk[5];
    
            double delta[6];
            delta[0] = x+deltax; delta[1] = y; delta[2] = z;
            delta[3] = r; delta[4] = p; delta[5] = h;
            #ifdef CHI_SQUARE_TEST
              _f = chi_square_cost (delta);
            #else 
              _f = mi_cost (delta);
            #endif
    
            double delF_delX = (_f - f_prev)/deltax;
            
            delta[0] = x; delta[1] = y+deltay; delta[2] = z;
            delta[3] = r; delta[4] = p; delta[5] = h; 
            #ifdef CHI_SQUARE_TEST
              _f = chi_square_cost (delta);
            #else 
              _f = mi_cost (delta);
            #endif
            double delF_delY = (_f - f_prev)/deltay;
            
            delta[0] = x; delta[1] = y; delta[2] = z+deltaz;
            delta[3] = r; delta[4] = p; delta[5] = h; 
            #ifdef CHI_SQUARE_TEST
              _f = chi_square_cost (delta);
            #else 
              _f = mi_cost (delta);
            #endif
            double delF_delZ = (_f - f_prev)/deltaz;
            
            delta[0] = x; delta[1] = y; delta[2] = z;
            delta[3] = r+deltar; delta[4] = p; delta[5] = h; 
            #ifdef CHI_SQUARE_TEST
              _f = chi_square_cost (delta);
            #else 
              _f = mi_cost (delta);
            #endif
            double delF_delR = (_f - f_prev)/deltar;
            
            delta[0] = x; delta[1] = y; delta[2] = z;
            delta[3] = r; delta[4] = p+deltap; delta[5] = h; 
            #ifdef CHI_SQUARE_TEST
              _f = chi_square_cost (delta);
            #else 
              _f = mi_cost (delta);
            #endif
            double delF_delP = (_f - f_prev)/deltap;
            
            delta[0] = x; delta[1] = y; delta[2] = z;
            delta[3] = r; delta[4] = p; delta[5] = h+deltah; 
            #ifdef CHI_SQUARE_TEST
              _f = chi_square_cost (delta);
            #else 
              _f = mi_cost (delta);
            #endif
            double delF_delH = (_f - f_prev)/deltah;
    
            double norm_delF_del_trans = sqrt(delF_delX*delF_delX + delF_delY*delF_delY + delF_delZ*delF_delZ); 
            double norm_delF_del_rot   = sqrt(delF_delR*delF_delR + delF_delP*delF_delP + delF_delH*delF_delH);
    
            delF_delX = delF_delX/norm_delF_del_trans;
            delF_delY = delF_delY/norm_delF_del_trans;
            delF_delZ = delF_delZ/norm_delF_del_trans;
            delF_delR = delF_delR/norm_delF_del_rot;
            delF_delP = delF_delP/norm_delF_del_rot;
            delF_delH = delF_delH/norm_delF_del_rot;
   
            double delta_trans = ((xk[0] - xk_minus_1[0])*(xk[0] - xk_minus_1[0]) + (xk[1] - xk_minus_1[1])*(xk[1] - xk_minus_1[1])
                         + (xk[2] - xk_minus_1[2])*(xk[2] - xk_minus_1[2]));
            double delta_rot = ((xk[3] - xk_minus_1[3])*(xk[3] - xk_minus_1[3]) + (xk[4] - xk_minus_1[4])*(xk[4] - xk_minus_1[4])
                         + (xk[5] - xk_minus_1[5])*(xk[5] - xk_minus_1[5]));
            
            //get the scaling factor
            if (delta_trans > 0)
            {
               double temp_deno_trans = ((xk[0] - xk_minus_1[0])*(delF_delX - delF_delX_) +
                         (xk[1] - xk_minus_1[1])*(delF_delY - delF_delY_) + (xk[2] - xk_minus_1[2])*(delF_delZ - delF_delZ_));
               temp_deno_trans = sqrt (temp_deno_trans*temp_deno_trans);
               gama_trans = delta_trans/temp_deno_trans; 
            }
            else
               gama_trans = gama_trans_u;
    
            if (delta_rot > 0)
            {
               double temp_deno_rot = ((xk[3] - xk_minus_1[3])*(delF_delR - delF_delR_) +
                         (xk[4] - xk_minus_1[4])*(delF_delP - delF_delP_) + (xk[5] - xk_minus_1[5])*(delF_delH - delF_delH_));
               temp_deno_rot = sqrt (temp_deno_rot*temp_deno_rot);
               gama_rot = delta_rot/temp_deno_rot;  
            }
            else
               gama_rot = gama_rot_u;
            
            //printf ("Before: gama_trans = %f, gama_rot = %f\n", gama_trans, gama_rot);
            //Since we are looking at maxima.
            if (gama_trans > gama_trans_u)
                gama_trans = gama_trans_u;
            if (gama_trans < gama_trans_l)
                gama_trans = gama_trans_l;
            
            if (gama_rot > gama_rot_u)
                gama_rot = gama_rot_u;
            if (gama_rot < gama_rot_l)
                gama_rot = gama_rot_l;
    
            //printf ("After: gama_trans = %f, gama_rot = %f\n", gama_trans, gama_rot);
            xk_minus_1[0] = xk[0];
            xk_minus_1[1] = xk[1];
            xk_minus_1[2] = xk[2];
            xk_minus_1[3] = xk[3];
            xk_minus_1[4] = xk[4];
            xk_minus_1[5] = xk[5];
     
            xk[0] = xk[0] + gama_trans*delF_delX;
            xk[1] = xk[1] + gama_trans*delF_delY;
            xk[2] = xk[2] + gama_trans*delF_delZ;
            xk[3] = xk[3] + gama_rot*delF_delR;
            xk[4] = xk[4] + gama_rot*delF_delP;
            xk[5] = xk[5] + gama_rot*delF_delH;
    
            double f_curr;
            #ifdef CHI_SQUARE_TEST
              f_curr = chi_square_cost (xk);
            #else 
              f_curr = mi_cost (xk);
            #endif
    
            if (f_curr < f_prev)
            {
              xk[0] = xk[0] - gama_trans*delF_delX;
              xk[1] = xk[1] - gama_trans*delF_delY;
              xk[2] = xk[2] - gama_trans*delF_delZ;
              xk[3] = xk[3] - gama_rot*delF_delR;
              xk[4] = xk[4] - gama_rot*delF_delP;
              xk[5] = xk[5] - gama_rot*delF_delH;
              gama_rot_u = gama_rot_u/1.2;
              gama_rot_l = gama_rot_l/1.2;
              gama_trans_u = gama_trans_u/1.2;
              gama_trans_l = gama_trans_l/1.2;
              
              deltax = deltax/1.1; deltay = deltay/1.1; deltaz = deltaz/1.1;
              deltar = deltar/1.1; deltap = deltap/1.1; deltah = deltah/1.1;
              //printf ("f_curr = %lf,  f_prev = %lf, deltax = %lf\n", f_curr, f_prev, deltax);
              index++;
              if (deltax < 0.001)
                 break;
              else
                 continue; 
            }
    
            index = index + 1;
            
            delF_delX_ = delF_delX;
            delF_delY_ = delF_delY;
            delF_delZ_ = delF_delZ;
            delF_delR_ = delF_delR;
            delF_delP_ = delF_delP;
            delF_delH_ = delF_delH;

            printf ("%lf %lf %lf %lf %lf %lf %lf\n", f_curr, xk[0], xk[1], xk[2], xk[3]*RTOD, xk[4]*RTOD, xk[5]*RTOD);
    
        }
        ssc_pose_set (x0_hl, xk);
        return index;
    }

    /**
     * This function calculates the Cramer Rao lower bound of the covariance matrix
     * using the Fisher Information matrix.
     */
    gsl_matrix*
    Calibration::calculate_covariance_matrix (ssc_pose_t x)
    {
        //Calculate the Fisher Information matrix
        double F[6][6];
        for(int i = 0; i < 6; i++)
        {
            for(int j = 0; j < 6; j++)
                F[i][j] = 0;
        }

        //get P at x
        Histogram hist = get_histogram (x);
        Probability P = get_probability_MLE (hist);
        
        ssc_pose_t tempX;

        //Get dLnP_dtx
        double h_t = 0.0001;
        tempX[0] = x[0] + h_t; tempX[1] = x[1]; tempX[2] = x[2];
        tempX[3] = x[3]; tempX[4] = x[4]; tempX[5] = x[5];
        Histogram hist_plus_dtx = get_histogram (tempX);
        Probability P_plus_dtx = get_probability_MLE (hist_plus_dtx);
        tempX[0] = x[0] - h_t; tempX[1] = x[1]; tempX[2] = x[2];
        tempX[3] = x[3]; tempX[4] = x[4]; tempX[5] = x[5];
        //Probability P_minus_dtx = get_probability_MLE (tempX);

        //Get dLnP_dty
        tempX[0] = x[0]; tempX[1] = x[1] + h_t; tempX[2] = x[2];
        tempX[3] = x[3]; tempX[4] = x[4]; tempX[5] = x[5];
        Histogram hist_plus_dty = get_histogram (tempX);
        Probability P_plus_dty = get_probability_MLE (hist_plus_dty);
        tempX[0] = x[0]; tempX[1] = x[1] - h_t; tempX[2] = x[2];
        tempX[3] = x[3]; tempX[4] = x[4]; tempX[5] = x[5];
        //Probability P_minus_dty = get_probability_MLE (tempX);
        
        //Get dLnP_dtz
        tempX[0] = x[0]; tempX[1] = x[1]; tempX[2] = x[2] + h_t;
        tempX[3] = x[3]; tempX[4] = x[4]; tempX[5] = x[5];
        Histogram hist_plus_dtz = get_histogram (tempX);
        Probability P_plus_dtz = get_probability_MLE (hist_plus_dtz);
        tempX[0] = x[0]; tempX[1] = x[1]; tempX[2] = x[2] - h_t;
        tempX[3] = x[3]; tempX[4] = x[4]; tempX[5] = x[5];
        //Probability P_minus_dtz = get_probability_MLE (tempX);
        
        //Get dLnP_drx
        double h_r = 0.001*DTOR;
        tempX[0] = x[0]; tempX[1] = x[1]; tempX[2] = x[2];
        tempX[3] = x[3] + h_r; tempX[4] = x[4]; tempX[5] = x[5];
        Histogram hist_plus_dtr = get_histogram (tempX);
        Probability P_plus_drx = get_probability_MLE (hist_plus_dtr);
        tempX[0] = x[0]; tempX[1] = x[1]; tempX[2] = x[2];
        tempX[3] = x[3] - h_r; tempX[4] = x[4]; tempX[5] = x[5];
        //Probability P_minus_drx = get_probability_MLE (tempX);
        
        //Get dLnP_dry
        tempX[0] = x[0]; tempX[1] = x[1]; tempX[2] = x[2];
        tempX[3] = x[3]; tempX[4] = x[4] + h_r; tempX[5] = x[5];
        Histogram hist_plus_dtp = get_histogram (tempX);
        Probability P_plus_dry = get_probability_MLE (hist_plus_dtp);
        tempX[0] = x[0]; tempX[1] = x[1]; tempX[2] = x[2];
        tempX[3] = x[3]; tempX[4] = x[4] - h_r; tempX[5] = x[5];
        //Probability P_minus_dry = get_probability_MLE (tempX);
        
        //Get dLnP_drz
        tempX[0] = x[0]; tempX[1] = x[1]; tempX[2] = x[2];
        tempX[3] = x[3]; tempX[4] = x[4]; tempX[5] = x[5] + h_r;
        Histogram hist_plus_dth = get_histogram (tempX);
        Probability P_plus_drz = get_probability_MLE (hist_plus_dth);
        tempX[0] = x[0]; tempX[1] = x[1]; tempX[2] = x[2];
        tempX[3] = x[3]; tempX[4] = x[4]; tempX[5] = x[5] - h_r;
        //Probability P_minus_drz = get_probability_MLE (tempX);

        //Calculate derivative of log of probability distribution
        cv::Mat dLnP_dtx = cv::Mat::zeros (this->m_numBins, this->m_numBins, CV_32FC1);
        cv::Mat dLnP_dty = cv::Mat::zeros (this->m_numBins, this->m_numBins, CV_32FC1);
        cv::Mat dLnP_dtz = cv::Mat::zeros (this->m_numBins, this->m_numBins, CV_32FC1);
        cv::Mat dLnP_drx = cv::Mat::zeros (this->m_numBins, this->m_numBins, CV_32FC1);
        cv::Mat dLnP_dry = cv::Mat::zeros (this->m_numBins, this->m_numBins, CV_32FC1);
        cv::Mat dLnP_drz = cv::Mat::zeros (this->m_numBins, this->m_numBins, CV_32FC1);

#if 0
        for (int i = 0; i < this->m_numBins; i++)
        {
            for (int j = 0; j < this->m_numBins; j++)
            {
                if (P_plus_dtx.jointProb.at<float>(i, j) > 0 && P_minus_dtx.jointProb.at<float>(i, j) > 0)
                    dLnP_dtx.at<float>(i, j) = (log (P_plus_dtx.jointProb.at<float>(i, j)) - log (P_minus_dtx.jointProb.at<float>(i, j)))/(2*h_t);

                if (P_plus_dty.jointProb.at<float>(i, j) > 0 && P_minus_dty.jointProb.at<float>(i, j) > 0)
                    dLnP_dty.at<float>(i, j) = (log (P_plus_dty.jointProb.at<float>(i, j)) - log (P_minus_dty.jointProb.at<float>(i, j)))/(2*h_t);

                if (P_plus_dtz.jointProb.at<float>(i, j) > 0 && P_minus_dtz.jointProb.at<float>(i, j) > 0)
                    dLnP_dtz.at<float>(i, j) = (log (P_plus_dtz.jointProb.at<float>(i, j)) - log (P_minus_dtz.jointProb.at<float>(i, j)))/(2*h_t);

                if (P_plus_drx.jointProb.at<float>(i, j) > 0 && P_minus_drx.jointProb.at<float>(i, j) > 0)
                    dLnP_drx.at<float>(i, j) = (log (P_plus_drx.jointProb.at<float>(i, j)) - log (P_minus_drx.jointProb.at<float>(i, j)))/(2*h_r);

                if (P_plus_dry.jointProb.at<float>(i, j) > 0 && P_minus_dry.jointProb.at<float>(i, j) > 0)
                    dLnP_dry.at<float>(i, j) = (log (P_plus_dry.jointProb.at<float>(i, j)) - log (P_minus_dry.jointProb.at<float>(i, j)))/(2*h_r);

                if (P_plus_drz.jointProb.at<float>(i, j) > 0 && P_minus_drz.jointProb.at<float>(i, j) > 0)
                    dLnP_drz.at<float>(i, j) = (log (P_plus_drz.jointProb.at<float>(i, j)) - log (P_minus_drz.jointProb.at<float>(i, j)))/(2*h_r);
            }
        } 
#endif
        for (int i = 0; i < this->m_numBins; i++)
        {
            for (int j = 0; j < this->m_numBins; j++)
            {
                if (P_plus_dtx.jointProb.at<float>(i, j) > 0 && P.jointProb.at<float>(i, j) > 0)
                    dLnP_dtx.at<float>(i, j) = (log (P_plus_dtx.jointProb.at<float>(i, j)) - log (P.jointProb.at<float>(i, j)))/(h_t);

                if (P_plus_dty.jointProb.at<float>(i, j) > 0 && P.jointProb.at<float>(i, j) > 0)
                    dLnP_dty.at<float>(i, j) = (log (P_plus_dty.jointProb.at<float>(i, j)) - log (P.jointProb.at<float>(i, j)))/(h_t);

                if (P_plus_dtz.jointProb.at<float>(i, j) > 0 && P.jointProb.at<float>(i, j) > 0)
                    dLnP_dtz.at<float>(i, j) = (log (P_plus_dtz.jointProb.at<float>(i, j)) - log (P.jointProb.at<float>(i, j)))/(h_t);

                if (P_plus_drx.jointProb.at<float>(i, j) > 0 && P.jointProb.at<float>(i, j) > 0)
                    dLnP_drx.at<float>(i, j) = (log (P_plus_drx.jointProb.at<float>(i, j)) - log (P.jointProb.at<float>(i, j)))/(h_r);

                if (P_plus_dry.jointProb.at<float>(i, j) > 0 && P.jointProb.at<float>(i, j) > 0)
                    dLnP_dry.at<float>(i, j) = (log (P_plus_dry.jointProb.at<float>(i, j)) - log (P.jointProb.at<float>(i, j)))/(h_r);

                if (P_plus_drz.jointProb.at<float>(i, j) > 0 && P.jointProb.at<float>(i, j) > 0)
                    dLnP_drz.at<float>(i, j) = (log (P_plus_drz.jointProb.at<float>(i, j)) - log (P.jointProb.at<float>(i, j)))/(h_r);
            }
        } 
        
        //Calculate F
        for(int k = 0; k < this->m_numBins; k++)
        {
          for(int l = 0; l < this->m_numBins; l++)
          {
             //std::cout<< P.jointHist.at<float>(k, l) << " " << dLnP_dtx.at<float>(k, l) << std::endl;
             F[0][0] = F[0][0] + dLnP_dtx.at<float>(k, l)*dLnP_dtx.at<float>(k, l)*P.jointProb.at<float>(k, l)*hist.jointHist.at<float>(k, l);
             F[0][1] = F[0][1] + dLnP_dtx.at<float>(k, l)*dLnP_dty.at<float>(k, l)*P.jointProb.at<float>(k, l)*hist.jointHist.at<float>(k, l);
             F[0][2] = F[0][2] + dLnP_dtx.at<float>(k, l)*dLnP_dtz.at<float>(k, l)*P.jointProb.at<float>(k, l)*hist.jointHist.at<float>(k, l);
             F[0][3] = F[0][3] + dLnP_dtx.at<float>(k, l)*dLnP_drx.at<float>(k, l)*P.jointProb.at<float>(k, l)*hist.jointHist.at<float>(k, l);
             F[0][4] = F[0][4] + dLnP_dtx.at<float>(k, l)*dLnP_dry.at<float>(k, l)*P.jointProb.at<float>(k, l)*hist.jointHist.at<float>(k, l);
             F[0][5] = F[0][5] + dLnP_dtx.at<float>(k, l)*dLnP_drz.at<float>(k, l)*P.jointProb.at<float>(k, l)*hist.jointHist.at<float>(k, l);
             F[1][1] = F[1][1] + dLnP_dty.at<float>(k, l)*dLnP_dty.at<float>(k, l)*P.jointProb.at<float>(k, l)*hist.jointHist.at<float>(k, l);
             F[1][2] = F[1][2] + dLnP_dty.at<float>(k, l)*dLnP_dtz.at<float>(k, l)*P.jointProb.at<float>(k, l)*hist.jointHist.at<float>(k, l);
             F[1][3] = F[1][3] + dLnP_dty.at<float>(k, l)*dLnP_drx.at<float>(k, l)*P.jointProb.at<float>(k, l)*hist.jointHist.at<float>(k, l);
             F[1][4] = F[1][4] + dLnP_dty.at<float>(k, l)*dLnP_dry.at<float>(k, l)*P.jointProb.at<float>(k, l)*hist.jointHist.at<float>(k, l);
             F[1][5] = F[1][5] + dLnP_dty.at<float>(k, l)*dLnP_drz.at<float>(k, l)*P.jointProb.at<float>(k, l)*hist.jointHist.at<float>(k, l);
             F[2][2] = F[2][2] + dLnP_dtz.at<float>(k, l)*dLnP_dtz.at<float>(k, l)*P.jointProb.at<float>(k, l)*hist.jointHist.at<float>(k, l);
             F[2][3] = F[2][3] + dLnP_dtz.at<float>(k, l)*dLnP_drx.at<float>(k, l)*P.jointProb.at<float>(k, l)*hist.jointHist.at<float>(k, l);
             F[2][4] = F[2][4] + dLnP_dtz.at<float>(k, l)*dLnP_dry.at<float>(k, l)*P.jointProb.at<float>(k, l)*hist.jointHist.at<float>(k, l);
             F[2][5] = F[2][5] + dLnP_dtz.at<float>(k, l)*dLnP_drz.at<float>(k, l)*P.jointProb.at<float>(k, l)*hist.jointHist.at<float>(k, l);
             F[3][3] = F[3][3] + dLnP_drx.at<float>(k, l)*dLnP_drx.at<float>(k, l)*P.jointProb.at<float>(k, l)*hist.jointHist.at<float>(k, l);
             F[3][4] = F[3][4] + dLnP_drx.at<float>(k, l)*dLnP_dry.at<float>(k, l)*P.jointProb.at<float>(k, l)*hist.jointHist.at<float>(k, l);
             F[3][5] = F[3][5] + dLnP_drx.at<float>(k, l)*dLnP_drz.at<float>(k, l)*P.jointProb.at<float>(k, l)*hist.jointHist.at<float>(k, l);
             F[4][4] = F[4][4] + dLnP_dry.at<float>(k, l)*dLnP_dry.at<float>(k, l)*P.jointProb.at<float>(k, l)*hist.jointHist.at<float>(k, l);
             F[4][5] = F[4][5] + dLnP_dry.at<float>(k, l)*dLnP_drz.at<float>(k, l)*P.jointProb.at<float>(k, l)*hist.jointHist.at<float>(k, l);
             F[5][5] = F[5][5] + dLnP_drz.at<float>(k, l)*dLnP_drz.at<float>(k, l)*P.jointProb.at<float>(k, l)*hist.jointHist.at<float>(k, l);
          }
        }
        F[1][0] = F[0][1];
        F[3][0] = F[0][3];
        F[4][0] = F[0][4];
        F[5][0] = F[0][5];   
        F[2][1] = F[1][2];
        F[3][1] = F[1][3];
        F[4][1] = F[1][4];
        F[5][1] = F[1][5];
        F[3][2] = F[2][3];
        F[4][2] = F[2][4];
        F[5][2] = F[2][5];
        F[4][3] = F[3][4];
        F[5][3] = F[3][5];
        F[5][4] = F[4][5];
        
        //calculate Covariance = inv(F)
        gsl_matrix* F_gsl =  gsl_matrix_alloc (6, 6);
        for (int i = 0; i < 6; i++)
        {
          for(int j = 0; j < 6; j++)
          {
             gsl_matrix_set (F_gsl, i, j, F[i][j]);
          }
        }  
        gsl_permutation *P_gsl = gsl_permutation_calloc (6);
        int signum;
        gsl_linalg_LU_decomp (F_gsl, P_gsl, &signum);
        gsl_matrix* F_inv = gsl_matrix_alloc (6, 6);
        gsl_linalg_LU_invert (F_gsl, P_gsl, F_inv);
        
        //Free Memory
        gsl_matrix_free(F_gsl);
        gsl_permutation_free(P_gsl);  
        
        return F_inv;
    }
}
