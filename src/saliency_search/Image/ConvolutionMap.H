/*!@file Image/ConvolutionMap.H */


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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/ConvolutionMap.H $
// $Id: ConvolutionMap.H 8631 2007-07-25 23:26:37Z rjpeters $
//

#ifndef CONVOLUTIONMAP_H_DEFINED
#define CONVOLUTIONMAP_H_DEFINED

#include "Image/Image.H"
#include <vector>
#include <stdlib.h>

//! Container class for convolution maps
template <class T> class convolutionMap
{
public:
  //! default constructor
  inline convolutionMap();
  //! default constructor, with initial values
  inline convolutionMap(T smallNumber,
                        unsigned int initVecSize,
                        Image<T> &origImage,
                        Image<T> &kernelImage);
  //! default destructor
  inline ~convolutionMap();
  //! check to see that we set up everything...
  inline void CMcheckInit1();
  //! check to see that we set up everything...
  inline void CMcheckInit2();
  //! copy an image into an attached image array
  inline void CMcopyImage(Image<T> &input);
  //! This is the pointer map to each pixel in the image for convolution
  Image<std::vector<T*> >    CMimageMap;
  //! This is the pointer map to each pixel in the kernel for convolution
  Image<std::vector<T> >     CMkernelMap;
  //! This stores the number of operations at each pixel
  Image<unsigned int>        CMindexMap;
  //! This stores the normalization const at each pixel
  Image<T>                   CMkWeightNorm;
  //! This is the "attached" static image to use
  Image<T>                   CMstaticImage;
  //! This is the original or template image to create a static image from
  Image<T>                   CMorigImage;
  //! This is the handle to the static information in the attached image
  T*                         CMimageArrayHandle;
  //! This is an outside holder for the array size
  unsigned int               CMimageArraySize;
  //! This is the "attached" static kernel image to use
  Image<T>                   CMkernel;
  //! This is the small number below which we weed out operations
  T                          CMsmallNumber;
  //! initial size for vectors, 0 means kernel size
  unsigned int               CMinitVecSize;
  //! set true if we input values during construction
  bool                       CMinit1;
  //! set true if we called a known init method to make maps
  bool                       CMinit2;
  //! set this to true if the vector maps are precise in size
  bool                       CMpreciseVectors;
};

// ######################################################################
// ######################################################################
// FREE FUNCTIONS
// ######################################################################
// ######################################################################

//! Computes a set of convolution maps for doing multiple convolutions
/*! When doing more than one convolution, for instance on an animated
    sequence, this builds a map set so that we precompute
    (A) border regions
    (B) very small kernel values
    These are removed and a vector of indexes is produced. These can then
    be used by iterating over the maps which contain the index.
    @param cMap This is a container for the maps and images used
*/
template <class T>
void computeConvolutionMaps(convolutionMap<T> &cMap);

//! carries out a simple mapped convolution, call computeConvolutionMaps first
/*! This will take in the convolution maps created and the current image
    and compute convolution in a line. That is, for each pixel we can
    in a straight line iterate over all the image and kernel values using
    a map. This saves us from having to check for border regions and compute
    pixel locations. Additionally, we can pre-remove small values from
    convolution.
    @param cMap This is a contaner for maps and images
    Example use:

    convolutionMap<FLOAT> cMap;
    Image<float> inputImg;

    ...

    if(frame == 1)
    {
      readMatrix rm("lowPassKernel.mat");
      rm.echoMatrix();
      cMap.CMsmallNumber = 1.1F;
      cMap.CMinitVecSize = 1;
      cMap.CMkernel = rm.returnMatrixAsImage();
      cMap.CMorigImage = inputImg;
      computeConvolutionMaps(cMap);
    }
    cMap.CMcopyImage(inputImg);
    Image<FLOAT> i2 = convolveWithMaps(cMap);
*/
template <class T>
Image<T> convolveWithMaps(convolutionMap<T> &cMap);

// ######################################################################
// ######################################################################
// INLINE MEMBER FUNCTIONS
// ######################################################################
// ######################################################################


// ######################################################################
template <class T> inline
convolutionMap<T>::convolutionMap()
{
  CMsmallNumber    = 0;
  CMinitVecSize    = 3;
  CMpreciseVectors = false;
  CMinit1          = false;

}

// ######################################################################
template <class T> inline
convolutionMap<T>::convolutionMap(T smallNumber,
                        unsigned int initVecSize,
                        Image<T> &origImage,
                        Image<T> &kernelImage)
{
  CMsmallNumber    = smallNumber;
  CMinitVecSize    = initVecSize;
  CMorigImage      = origImage;
  CMkernel         = kernelImage;
  if(initVecSize == 1)
    CMpreciseVectors = true;
  else
    CMpreciseVectors = false;
  CMinit1          = true;
}

// ######################################################################
template <class T> inline
convolutionMap<T>::~convolutionMap()
{}

// ######################################################################
template <class T> inline
void convolutionMap<T>::CMcheckInit1()
{
  if(CMinit1 == false)
  {
    if(CMorigImage.initialized() == false)
      LFATAL("CMorigImage must be init the size of the image to be convolved");
    if(CMkernel.initialized() == false)
      LFATAL("You must supply the kernel as image CMkernel");
    CMinit1 = true;
  }
}

// ######################################################################
template <class T> inline
void convolutionMap<T>::CMcheckInit2()
{
  if(CMinit2 == false)
  {
    if(CMimageMap.initialized() == false)
      LFATAL("Image map CMimageMap not initalized in container");
    if(CMkernelMap.initialized() == false)
      LFATAL("Kernel map CMkernelMap not initalized in container");
    if(CMindexMap.initialized() == false)
      LFATAL("Index map image CMindexMap not initalized in container");
    if(CMkWeightNorm.initialized() == false)
      LFATAL("Normalization image CMkWeightNorm not initalized in container");
    if(CMstaticImage.initialized() == false)
      LFATAL("Static Image CMstaticImage not initalized in container");
    CMinit2 = true;
  }
}

// ######################################################################
template <class T> inline
void convolutionMap<T>::CMcopyImage(Image<T> &input)
{
  if(input.getWidth() != CMstaticImage.getWidth())
    LFATAL("Input image and static image different width");
  if(input.getHeight() != CMstaticImage.getHeight())
    LFATAL("Input image and static image different height");
  typename Image<T>::iterator istatic = CMstaticImage.beginw();
  typename Image<T>::iterator iinput  = input.beginw();
  while(istatic != CMstaticImage.endw())
    *istatic++ = *iinput++;
}

#endif

