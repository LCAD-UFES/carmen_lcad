/*!@file Gist/trainUtils.C training utility functions (not limited to FFN) */

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
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/trainUtils.C $
// $Id: trainUtils.C 15310 2012-06-01 02:29:24Z itti $
//

#include "Image/DrawOps.H"
#include "Image/MathOps.H"
#include "Neuro/StdBrain.H"
#include "Raster/Raster.H"
#include "Gist/trainUtils.H"

#include <cstdio>

// ######################################################################
//! Constructor
FFNtrainInfo::FFNtrainInfo(std::string fName)
{
  if(fName.length() != 0) reset(fName);
}

// ######################################################################
//! Destructor
FFNtrainInfo::~FFNtrainInfo()
{}

// ######################################################################
//! reset the training info with a new file
bool FFNtrainInfo::reset(std::string fName)
{
  FILE *fp;  char inLine[1000];  char comment[200]; char temp[200];

  // open a file that lists the training parameters
  if((fp = fopen(fName.c_str(),"rb")) == NULL)
    {
      LINFO("training file: %s not found",fName.c_str());
      return false;
    }

  // get the location of the training folder
  if (fgets(inLine, 1000, fp) == NULL) LFATAL("fgets() failed"); sscanf(inLine, "%s %s", temp, comment);
  trainFolder = std::string(temp);

  // get the location of the testing folder
  if (fgets(inLine, 1000, fp) == NULL) LFATAL("fgets() failed"); sscanf(inLine, "%s %s", temp, comment);
  testFolder = std::string(temp);

  // get the number of categories
  if (fgets(inLine, 1000, fp) == NULL) LFATAL("fgets() failed"); sscanf(inLine, "%d %s", &nOutput, comment);

  // PCA option
  if (fgets(inLine, 1000, fp) == NULL) LFATAL("fgets() failed"); sscanf(inLine, "%s %s", temp, comment);
  isPCA = (strcmp(temp,"PCA") == 0);
  if (fgets(inLine, 1000, fp) == NULL) LFATAL("fgets() failed"); sscanf(inLine, "%s %s", temp, comment);
  evecFname = std::string(temp);

  // get the original and reduced number of features
  if (fgets(inLine, 1000, fp) == NULL) LFATAL("fgets() failed"); sscanf(inLine, "%d %s", &oriFeatSize, comment);
  if (fgets(inLine, 1000, fp) == NULL) LFATAL("fgets() failed"); sscanf(inLine, "%d %s", &redFeatSize, comment);

  // get the number of nodes at the 2 hidden layer
  if (fgets(inLine, 1000, fp) == NULL) LFATAL("fgets() failed"); sscanf(inLine, "%d %s", &h1size, comment);
  if (fgets(inLine, 1000, fp) == NULL) LFATAL("fgets() failed"); sscanf(inLine, "%d %s", &h2size, comment);

  // the training learning rate
  if (fgets(inLine, 1000, fp) == NULL) LFATAL("fgets() failed"); sscanf(inLine, "%f %s", &learnRate, comment);

  // get the training samples file
  if (fgets(inLine, 1000, fp) == NULL) LFATAL("fgets() failed"); sscanf(inLine, "%s %s", temp, comment);
  trainSampleFile = std::string(temp);

  // get the testing samples file
  if (fgets(inLine, 1000, fp) == NULL) LFATAL("fgets() failed"); sscanf(inLine, "%s %s", temp, comment);
 testSampleFile = std::string(temp);

  // get file name for the weights
  if (fgets(inLine, 1000, fp) == NULL) LFATAL("fgets() failed"); sscanf(inLine, "%s %s", temp, comment);
  h1Name = trainFolder + std::string(temp);
  if (fgets(inLine, 1000, fp) == NULL) LFATAL("fgets() failed"); sscanf(inLine, "%s %s", temp, comment);
  h2Name = trainFolder + std::string(temp);
  if (fgets(inLine, 1000, fp) == NULL) LFATAL("fgets() failed"); sscanf(inLine, "%s %s", temp, comment);
  oName = trainFolder + std::string(temp);

  fclose(fp);

  // some of the classifier parameter
  LINFO("Training folder: %s", trainFolder.c_str());
  LINFO("Testing  folder: %s", testFolder.c_str());
  LINFO("PCA?: %d (%s) %d -> %d",
        isPCA, evecFname.c_str(), oriFeatSize, redFeatSize);
  LINFO("NN: %d->%d->%d->%d  LR: %f",
        redFeatSize, h1size, h2size, nOutput, learnRate);
  LINFO("train: %s", trainSampleFile.c_str());
  LINFO("test : %s", testSampleFile.c_str());
  LINFO("h1 weight file name: %s", h1Name.c_str());
  LINFO("h2 weight file name: %s", h2Name.c_str());
  LINFO("o  weight file name: %s",  oName.c_str());

  return true;
}

// ######################################################################
// functions for PCA/ICA reductions
// ######################################################################
// setup the PCA/ICA un-mixing matrix
Image<double> setupPcaIcaMatrix(std::string inW, int oriSize, int redSize)
{
  FILE *fp;
  // it's still a resSize x oriSize matrix,
  // but the image declaration is flipped
  Image<double> ret(oriSize, redSize, NO_INIT);

  // open the matrix-entries file
  if((fp = fopen(inW.c_str(),"rb")) == NULL)
    {
      LINFO("can't open pca file: %s fill with random values",
            inW.c_str());
      Image<double>::iterator aptr = ret.beginw();
      for(int i = 0; i < redSize; i++)
        {
          for(int j = 0; j < oriSize; j++)
            {
              *aptr++ = (-TUTILS_RW_RANGE/2.0) +
                (rand()/(RAND_MAX + 1.0) * TUTILS_RW_RANGE);
            }
        }
    }
  else
    {
      Image<double>::iterator aptr = ret.beginw();
      for(int i = 0; i < redSize; i++)
        {
          for(int j = 0; j < oriSize; j++)
            { double val; if (fread(&val,sizeof(double),1,fp) != 1) LFATAL("fread error"); *aptr++ = val; }
        }
    }

  LINFO("PCA/ICA un-mixing matrix is set");
  return ret;
}

// ######################################################################
// get the PCA feature vectors in image histogram
Image<float> getPcaIcaFeatImage(Image<double> res, int w, int h, int s)
{
  Image<float> img(w * s, h * s, ZEROS);

  for(int j = 0; j < h; j++)
      for(int i = 0; i < w; i++)
        drawPatch(img, Point2D<int>(i*s+s/2,j*s+s/2),s/2, float(res.getVal(j,i)));
  //j*w+i
  return img;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
