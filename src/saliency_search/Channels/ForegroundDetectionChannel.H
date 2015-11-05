/*!@file Channels/ForegroundDetectionChannel.H object detection channel using opecv cascade detector */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
// by the University of Southern California (USC) and the iLab at USC.  //
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
// Primary maintainer for this file: Lior Elazary
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/ForegroundDetectionChannel.H $
// $Id: ForegroundDetectionChannel.H 14605 2011-03-15 02:25:06Z dparks $
//

#ifndef FOREGROUNDDETECTIONCHANNEL_H_DEFINED
#define FOREGROUNDDETECTIONCHANNEL_H_DEFINED

#include "Image/OpenCVUtil.H"


#ifdef HAVE_OPENCVAUX
#define MAX(a,b)  ((a) < (b) ? (b) : (a))
#include <opencv/cxtypes.h>
#include <opencv/cvaux.h>
#endif // HAVE_OPENCVAUX


#include "Channels/SingleChannel.H"
#include "Image/LevelSpec.H"


#ifdef HAVE_OPENCV
class CvBGStatModel;
#endif

// ######################################################################
//! A channel for separating foreground from background
class ForegroundDetectionChannel : public SingleChannel
{
public:
  //! Constructor
  ForegroundDetectionChannel(OptionManager& mgr);

  //! overload
  virtual bool outputAvailable() const;

  //! Virtual destructor.
  virtual ~ForegroundDetectionChannel();

  virtual uint numSubmaps() const;

  //! Get the foreground detection map
  virtual Image<float> getSubmap(const uint index) const;

  //! Bogus function that just returns a blank image so that
  //we don't need to deal with image pyramids
  virtual Image<float> getRawCSmap(const uint idx) const;

  virtual std::string getSubmapName(const uint index) const;

  //! special input function
  virtual void doInput(const InputFrame& inframe);

  //! special output function
  virtual Image<float> getOutput();

  virtual Dims getMapDims() const;

  virtual void getFeatures(const Point2D<int>& locn,
                           std::vector<float>& mean) const;

  virtual void getFeaturesBatch(std::vector<Point2D<int>*> *locn,
                                std::vector<std::vector<float> > *mean,
                                int *count) const;


protected:
  Image<float> itsForegroundMap;                  //!The probability map of foreground objects
  Image<float> itsMap;                            //!The rescaled output map - for now just the foreground map
  OModelParam<LevelSpec> itsLevelSpec;
  
#ifdef HAVE_OPENCV
  CvBGStatModel* itsStatModel_cv;
#endif
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif
