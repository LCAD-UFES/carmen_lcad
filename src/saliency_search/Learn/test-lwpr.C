/*!@file Learn/test-lwpr.C test the lwpr algo
* LWPR: A library for incremental online learning
* Copyright (C) 2007  Stefan Klanke, Sethu Vijayakumar
* Contact: sethu.vijayakumar@ed.ac.uk
*/

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
// //////////////////////////////////////////////////////////////////// //
// This file is part of the iLab Neuromorphic Vision C++ Toolkit.       //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is free software; you can   //
// redistribute it and/or modify it under the terms of the GNU General  //
// Public License as published by the Free Software Foundation; either  //
// version 2 of the License, or (at your option) any later version.     //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  //
// that it will be useful, but WITHOUT ANY WARRANTY; without even the   //
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
// PURPOSE.  See the GNU General Public License for more details.       //
//                                                                      //
// You should have received a copy of the GNU General Public License    //
// along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   //
// to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   //
// Boston, MA 02111-1307 USA.                                           //
// //////////////////////////////////////////////////////////////////// //
//
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Learn/test-lwpr.C $
// $Id: test-lwpr.C 10794 2009-02-08 06:21:09Z itti $
//

#include "Component/ModelManager.H"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <lwpr/lwpr.h>

#define SEED_RAND()     srand48(time(NULL))
#define URAND()         drand48()

double cross(double x1,double x2) {
   double a = exp(-10*x1*x1);
   double b = exp(-50*x2*x2);
   double c = 1.25*exp(-5*(x1*x1 + x2*x2));

   if (a>b) {
      return (a>c) ? a:c;
   } else {
      return (b>c) ? b:c;
   }
}


int main(int argc, char** argv)
{
  // Instantiate a ModelManager:
  ModelManager manager("Test LWPR");

  // Parse command-line:
  if (manager.parseCommandLine((const int)argc, (const char**)argv, "", 0, 0) == false)
    return(1);

  manager.start();

  double x[2];
  double y,yp;
  double mse;

  FILE *fp;
  LWPR_Model model;
  int i,j;

  /* This allocates some memory and sets initial values
   ** Note that the model structure itself already exists (on the stack)
   */
  lwpr_init_model(&model,2,1,"2D_Cross");

  /* Set initial distance metric to 50*(identity matrix) */
  lwpr_set_init_D_spherical(&model,50);

  /* Set init_alpha to 250 in all elements */
  lwpr_set_init_alpha(&model,250);

  /* Set w_gen to 0.2 */
  model.w_gen = 0.2;

  /* See above definition, we either use srand() on Windows or srand48 everywhere else */
  SEED_RAND();

  for (j=0;j<20;j++) {
    mse = 0.0;

    for (i=0;i<1000;i++) {
      x[0] = 2.0*URAND()-1.0;
      x[1] = 2.0*URAND()-1.0;
      y = cross(x[0],x[1]) + 0.1*URAND()-0.05;

      /* Update the model with one sample
       **
       ** x points to (x[0],x[1])  (input vector)
       ** &y points to y           (output "vector")
       ** &yp points to yp         (prediction "vector")
       **
       ** If you are interested in maximum activation, call
       ** lwpr_update(&model, x, &y, &yp, &max_w);
       */
      lwpr_update(&model, x, &y, &yp, NULL);

      mse+=(y-yp)*(y-yp);
    }
    mse/=500;
    printf("#Data = %d   #RFS = %d   MSE = %f\n",model.n_data, model.sub[0].numRFS, mse);
  }

  fp = fopen("output.txt","w");

  mse = 0.0;
  i=0;

  for (x[1]=-1.0; x[1]<=1.01; x[1]+=0.05) {
    for (x[0]=-1.0; x[0]<=1.01; x[0]+=0.05) {
      y = cross(x[0],x[1]);

      /* Use the model for predicting an output
       **
       ** x points to (x[0],x[1])     (input vector)
       ** 0.001  is the cutoff value  (clip Gaussian kernel)
       ** &yp points to yp            (prediction "vector")
       **
       ** If you are interested in confidence bounds or
       ** maximum activation, call
       ** lwpr_predict(&model, x, 0.001, &yp, &conf, &max_w);
       */
      lwpr_predict(&model, x, 0.001, &yp, NULL, NULL);

      mse += (y-yp)*(y-yp);
      i++;

      fprintf(fp,"%8.5f %8.5f %8.5f\n",x[0],x[1],yp);
    }
    fprintf(fp,"\n\n");
  }
  fclose(fp);

  printf("MSE on test data (%d) = %f\n",i,mse/(double) i);

  printf("\nTo view the output, start gnuplot, and type:\n");
  printf("   splot \"output.txt\"\n\n");

  /* Free the memory that was allocated for receptive fields etc.
   ** Note again that this does not free the LWPR_Model structure
   ** itself (but it exists on the stack, so it's automatically free'd) */
  lwpr_free_model(&model);

  // stop all our ModelComponents
  manager.stop();

  // all done!
  return 0;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
