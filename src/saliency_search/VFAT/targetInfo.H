/*!@file VFAT/targetInfo.H  holds properaties of objects
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
// Primary maintainer for this file: T Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/targetInfo.H $
// $Id: targetInfo.H 6003 2005-11-29 17:22:45Z rjpeters $
//

// ############################################################
// ############################################################
// ##### --- VFAT ---
// ##### Vision Feature Analysis Tool:
// ##### T. Nathan Mundhenk nathan@mundhenk.com
// ##### Laurent Itti itti@pollux.usc.edu
// #####
// ############################################################
// ############################################################

#ifndef TARGETINFO_H_DEFINED
#define TARGETINFO_H_DEFINED

#include "Util/log.H"

#include <stdlib.h>
#include <string>
#include <vector>
#include <sys/types.h>

//! A general purpose container for holding guassian signatures
/*! This is a general purpose container for classes from the feature
    classifier. Note that there are no pointers and all data is copied
    this allows this object to be pulled off from the other code if
    need be which makes it portable to other machines or memory spaces
    so long as the object type is known.

    NOTE: To see how means and averages are computed along with how each
    sample is matched temporally, see the file covEstimate.C and the
    method covEstimate<T>::matchPmeanAccum(...) . This method does a
    nearest neighbor matching and computes temporal dynamics for each
    class.
*/
template <class FLOAT> class vfatTargetInfo
{
public:
  inline vfatTargetInfo(unsigned long _dim);
  inline vfatTargetInfo();
  inline ~vfatTargetInfo();
  inline void resize(unsigned long _dim);
  //! unique base ID of this object
  unsigned long baseID;
  //! Literal size of image space inspected (image width)
  unsigned short imageSizeX;
  //! Literal size of image space inspected (image height)
  unsigned short imageSizeY;
  //! image location X (for image objects)
  unsigned short posX;
  //! image location Y (for image object)
  unsigned short posY;
  //! Max and min values X and Y
  unsigned short maxX;
  //! Max and min values X and Y
  unsigned short minX;
  //! Max and min values X and Y
  unsigned short maxY;
  //! Max and min values X and Y
  unsigned short minY;
  //! what is the pixel mass of this object?
  unsigned int mass;
  //! how long has this object been alive for?
  unsigned long lifeSpan;
  //! number of diminsions in the set
  unsigned long dim;
  //! a grand mean
  FLOAT grandMean;

  //! mean (centroid) values for all degrees of freedom
  std::vector<FLOAT> mean;
  //! basic standard deviation for all degrees of freedom
  std::vector<FLOAT> STD;
  //! eigenvalue result for each diminsion
  std::vector<FLOAT> eigenVal;
  //! min value for this feature
  std::vector<FLOAT> min;
  //! max value for this feature
  std::vector<FLOAT> max;
  //! this is the first order motion (partial 1st derivative)
  std::vector<FLOAT> speed;
  //! this is the second order motion (partial 2nd derivative)
  std::vector<FLOAT> accel;
  //! Average first order motion (partial 1st derivative)
  std::vector<FLOAT> avgspeed;
  //! Average second order motion (partial 2nd derivative)
  std::vector<FLOAT> avgaccel;
  //! how far has this object traveled? (partial 1st order integral)
  std::vector<FLOAT> distance;

  //! this is a set of bias weights to be used if needed
  std::vector<FLOAT> bias;
  //! this is a variables normalization const
  std::vector<FLOAT> norm;
  //! this is a const to translate values used with norm
  std::vector<FLOAT> trans;
  //! A pointer to this features names
  std::vector<std::string> featureName;
};

// ######################################################################
template <class FLOAT>
inline vfatTargetInfo<FLOAT>::vfatTargetInfo()
{}

// ######################################################################
template <class FLOAT>
inline vfatTargetInfo<FLOAT>::vfatTargetInfo(unsigned long _dim)
{
  resize(_dim);
}

// ######################################################################
template <class FLOAT>
inline vfatTargetInfo<FLOAT>::~vfatTargetInfo()
{}

// ######################################################################
template <class FLOAT>
inline void vfatTargetInfo<FLOAT>::resize(unsigned long _dim)
{
  lifeSpan = 0;
  dim      = _dim;
  mean.resize(dim    ,0.0F);
  STD.resize(dim     ,0.0F);
  eigenVal.resize(dim,0.0F);
  bias.resize(dim    ,1.0F);
  norm.resize(dim    ,1.0F);
  trans.resize(dim   ,1.0F);
  min.resize(dim     ,0.0F);
  max.resize(dim     ,0.0F);
  speed.resize(dim   ,0.0F);
  accel.resize(dim   ,0.0F);
  avgspeed.resize(dim,0.0F);
  avgaccel.resize(dim,0.0F);
  distance.resize(dim,0.0F);
  featureName.resize(dim,"");
}

#endif
