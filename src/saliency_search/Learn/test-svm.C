/*!@file Learn/test-svm.C test the svm algo
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Learn/test-svm.C $
// $Id: test-svm.C 12062 2009-11-19 15:02:09Z lior $
//

#include "Component/ModelManager.H"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "svm.h"

void read_problem(const char *filename,
    struct svm_problem &prob,
    struct svm_parameter &param)
{
  int elements, max_index, i, j;
  FILE *fp = fopen(filename,"r");

  if(fp == NULL)
  {
    fprintf(stderr,"can't open input file %s\n",filename);
    exit(1);
  }

  prob.l = 0;
  elements = 0;
  while(1)
  {
    int c = fgetc(fp);
    switch(c)
    {
      case '\n':
        ++prob.l;
        // fall through,
        // count the '-1' element
      case ':':
        ++elements;
        break;
      case EOF:
        goto out;
      default:
        ;
    }
  }
out:
  rewind(fp);

  prob.y = new double[prob.l];
  prob.x = new struct svm_node *[prob.l];
  struct svm_node *x_space = new struct svm_node[elements];

  max_index = 0;
  j=0;
  for(i=0;i<prob.l;i++)
  {
    double label;
    prob.x[i] = &x_space[j];
    fscanf(fp,"%lf",&label);
    prob.y[i] = label;

    while(1)
    {
      int c;
      do {
        c = getc(fp);
        if(c=='\n') goto out2;
      } while(isspace(c));
      ungetc(c,fp);
      if (fscanf(fp,"%d:%lf",&(x_space[j].index),&(x_space[j].value)) < 2)
      {
        fprintf(stderr,"Wrong input format at line %d\n", i+1);
        exit(1);
      }
      ++j;
    }
out2:
    if(j>=1 && x_space[j-1].index > max_index)
      max_index = x_space[j-1].index;
    x_space[j++].index = -1;
  }

  if(param.gamma == 0)
    param.gamma = 1.0/max_index;

  if(param.kernel_type == PRECOMPUTED)
    for(i=0;i<prob.l;i++)
    {
      if (prob.x[i][0].index != 0)
      {
        fprintf(stderr,"Wrong input format: first column must be 0:sample_serial_number\n");
        exit(1);
      }
      if ((int)prob.x[i][0].value <= 0 || (int)prob.x[i][0].value > max_index)
      {
        fprintf(stderr,"Wrong input format: sample_serial_number out of range\n");
        exit(1);
      }
    }

  fclose(fp);
}

void predict(const char* filename, struct svm_model *model)
{
  int correct = 0;
  int total = 0;
  double error = 0;
  double sumv = 0, sumy = 0, sumvv = 0, sumyy = 0, sumvy = 0;
  int max_nr_attr = 64;
  struct svm_node *x = new struct svm_node[max_nr_attr];

  FILE *input = fopen(filename, "r");
  if(input == NULL)
  {
    fprintf(stderr,"can't open testing file %s\n",filename);
    exit(1);
  }
  while(1)
  {
    int i = 0;
    int c;
    double target,v;

    if (fscanf(input,"%lf",&target)==EOF)
      break;

    while(1)
    {
      if(i>=max_nr_attr-1)        // need one more for index = -1
      {
        max_nr_attr *= 2;
        x = (struct svm_node *) realloc(x,max_nr_attr*sizeof(struct svm_node));
      }

      do {
        c = getc(input);
        if(c=='\n' || c==EOF) goto out2;
      } while(isspace(c));
      ungetc(c,input);
      if (fscanf(input,"%d:%lf",&x[i].index,&x[i].value) < 2)
      {
        fprintf(stderr,"Wrong input format at line %d\n", total+1);
        exit(1);
      }
      ++i;
    }

out2:
    x[i].index = -1;

    v = svm_predict(model,x);

    if(v == target)
      ++correct;
    error += (v-target)*(v-target);
    sumv += v;
    sumy += target;
    sumvv += v*v;
    sumyy += target*target;
    sumvy += v*target;
    ++total;
  }
  printf("Accuracy = %g%% (%d/%d) (classification)\n",
      (double)correct/total*100,correct,total);
}


int main(int argc, char** argv)
{
  struct svm_parameter param;                // set by parse_command_line
  struct svm_problem prob;                // set by read_problem

  // Instantiate a ModelManager:
  ModelManager manager("Test SVM");

  // Parse command-line:
  if (manager.parseCommandLine((const int)argc, (const char**)argv, "", 0, 0) == false)
    return(1);

  manager.start();

  //default paramaters
  param.svm_type = C_SVC;
  param.kernel_type = RBF;
  param.degree = 3;
  param.gamma = 0;        // 1/k
  param.coef0 = 0;
  param.nu = 0.5;
  param.cache_size = 100;
  param.C = 1;
  param.eps = 1e-3;
  param.p = 0.1;
  param.shrinking = 1;
  param.probability = 0;
  param.nr_weight = 0;
  param.weight_label = NULL;
  param.weight = NULL;


  read_problem("tests/train.1.scale", prob, param);

  struct svm_model *model = svm_train(&prob,&param);
  //if((model=svm_load_model(argv[i+1]))==0)
  //{
  //        fprintf(stderr,"can't open model file %s\n",argv[i+1]);
  //        exit(1);
  //}

  //svm_save_model(model_file_name,model);
  //svm_destroy_model(model);

  //predict

  LINFO("Predicting");
  predict("tests/test.1.scale", model);
  LINFO("Done");

  svm_destroy_param(&param);
  free(prob.y);
  free(prob.x);
  //free(x_space);


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
