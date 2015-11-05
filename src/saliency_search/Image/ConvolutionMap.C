/*!@file Image/ConvolutionMap.C */

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
// Primary maintainer for this file: T Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/ConvolutionMap.C $
// $Id: ConvolutionMap.C 8631 2007-07-25 23:26:37Z rjpeters $
//

#include "Image/ConvolutionMap.H"

#include "rutz/trace.h"

#include <algorithm>
#include <cmath>

// ######################################################################
template <class T>
void computeConvolutionMaps(convolutionMap<T> &cMap)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  cMap.CMcheckInit1();
  cMap.CMinit2 = true;
  // Resize the static Image and attach an array
  unsigned int totalSize = cMap.CMorigImage.getWidth() *
                           cMap.CMorigImage.getHeight();

  cMap.CMimageArrayHandle = new T[totalSize];
  cMap.CMstaticImage.attach(&cMap.CMimageArrayHandle[0],
                            cMap.CMorigImage.getWidth(),
                            cMap.CMorigImage.getHeight());

  // resize convolution maps
  cMap.CMimageMap.resize(   cMap.CMstaticImage.getWidth(),
                            cMap.CMstaticImage.getHeight(),true);
  cMap.CMkernelMap.resize(  cMap.CMstaticImage.getWidth(),
                            cMap.CMstaticImage.getHeight(),true);
  cMap.CMindexMap.resize(   cMap.CMstaticImage.getWidth(),
                            cMap.CMstaticImage.getHeight(),true);
  cMap.CMkWeightNorm.resize(cMap.CMstaticImage.getWidth(),
                            cMap.CMstaticImage.getHeight(),true);

  // set up the offset from kernel for convolution
  const unsigned int imageX =
    (unsigned int)floor(((double)cMap.CMkernel.getWidth())/2);
  const unsigned int imageY =
    (unsigned int)floor(((double)cMap.CMkernel.getHeight())/2);

  // initalize memory for convolution maps
  T zero = 0;
  unsigned int initVectorSize;

  if(cMap.CMinitVecSize == 0)
    initVectorSize = (cMap.CMkernel.getWidth() * cMap.CMkernel.getHeight());
  else
    initVectorSize = cMap.CMinitVecSize;

  typename std::vector<T*>             Ifiller(initVectorSize,&zero);
  typename std::vector<T>              Tfiller(initVectorSize,0);

  // Assign default image values for each map
  for(int x = 0; x < cMap.CMstaticImage.getWidth(); x++)
  {
    for(int y = 0; y < cMap.CMstaticImage.getHeight(); y++)
    {
      cMap.CMimageMap.setVal(x,y,Ifiller);
      cMap.CMkernelMap.setVal(x,y,Tfiller);
      cMap.CMkWeightNorm.setVal(x,y,0);
    }
  }


  // For each pixel in the image and the kernel
  for(int x = 0; x < cMap.CMstaticImage.getWidth(); x++)
  {
    for(int y = 0; y < cMap.CMstaticImage.getHeight(); y++)
    {
      for(int i = 0; i < cMap.CMkernel.getWidth(); i++)
      {
        for(int j = 0; j < cMap.CMkernel.getHeight(); j++)
        {
          // weed out any kernel elements smaller than smallNumber
          // We assume a gray scale kernel image
          if(cMap.CMkernel.getVal(i,j) > cMap.CMsmallNumber)
          {
            unsigned int Pos = y * cMap.CMimageMap.getWidth() + x;

            // if the vector needs to be expanded because we need more elements
            if(cMap.CMindexMap.getVal(x,y) == cMap.CMimageMap.getVal(x,y).size())
            {
              unsigned int m1  = cMap.CMimageMap.getVal(x,y).size() +
                initVectorSize;
              unsigned int m2  = cMap.CMkernelMap.getVal(x,y).size() +
                initVectorSize;
              cMap.CMimageMap[Pos].resize(m1,0);
              cMap.CMkernelMap[Pos].resize(m2,0);
            }
            int inspX = x + (i - imageX);
            int inspY = y + (j - imageY);
            // range check on X
            if((inspX >= 0) && (inspX < cMap.CMstaticImage.getWidth()))
            {
              // range check on Y
              if((inspY >= 0) && (inspY < cMap.CMstaticImage.getHeight()))
              {
                // if Ok, map other image pixel to this one
                cMap.CMimageMap[Pos][cMap.CMindexMap.getVal(x,y)] =
                  &cMap.CMstaticImage[
                inspY*cMap.CMstaticImage.getWidth() + inspX];
                // if OK, map kernel pixel to this image pair
                cMap.CMkernelMap[Pos][cMap.CMindexMap.getVal(x,y)] =
                  cMap.CMkernel.getVal(i,j);
                // sum the kernel values for normalization
                cMap.CMkWeightNorm[Pos] += cMap.CMkernel.getVal(i,j);
                // increment counter
                cMap.CMindexMap.setVal(x,y,cMap.CMindexMap.getVal(x,y) + 1);
              }
            }
          }
        }
      }
    }
  }
}

// ######################################################################
template <class T>
Image<T> convolveWithMaps(convolutionMap<T> &cMap)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  cMap.CMcheckInit2();
  // create and init a return image, set to zero. It will hold
  //the results of the convolution
  Image<T> returnImage(cMap.CMstaticImage.getWidth(),
                       cMap.CMstaticImage.getHeight(),ZEROS);

  // set up iterators to each important thing in convolution.
  // These include
  // (A) the image map from the static image
  // (B) the image map from the kernel
  // (C) the counter for the maps
  // (D) the normalization image for each pixel
  // (E) the clean return image

  typename Image<std::vector<T*> >::iterator iimg           =
    cMap.CMimageMap.beginw();
  typename Image<std::vector<T> >::iterator  iker           =
    cMap.CMkernelMap.beginw();
  Image<unsigned int>::iterator              iindx          =
    cMap.CMindexMap.beginw();
  typename Image<T>::iterator                inorm          =
    cMap.CMkWeightNorm.beginw();
  typename Image<T>::iterator                iret           =
    returnImage.beginw();

  typename std::vector<T*>::iterator iiimg;
  typename std::vector<T>::iterator  iiker;

  // For each pixel, while not at the end of the return image
  if(cMap.CMpreciseVectors == false)
  {
    while(iret != returnImage.endw())
    {
      iiimg = iimg->begin();
      iiker = iker->begin();
      // For each item in the map at each pixel
      for(unsigned int z = 0; z < *iindx; z++, ++iiimg, ++iiker)
      {
        *iret = ((**iiimg) * (*iiker)) + (*iret);
      }
      // normalize the results
      *iret = (*iret) / (*inorm);
      // increment all the iterators to the next pixel
      ++iimg; ++iker; ++iindx; ++iret; ++inorm;
    }
  }
  else
  {
    while(iret != returnImage.endw())
    {
      iiimg = iimg->begin();
      iiker = iker->begin();
      while(iiimg != iimg->end())
      {
        *iret = ((**iiimg) * (*iiker)) + (*iret);
        ++iiimg, ++iiker;
      }
      // normalize the results
      *iret = (*iret) / (*inorm);
      // increment all the iterators to the next pixel
      ++iimg; ++iker; ++iindx; ++iret; ++inorm;
    }
  }
  return returnImage;
}

// Include the explicit instantiations
#include "inst/Image/ConvolutionMap.I"

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
