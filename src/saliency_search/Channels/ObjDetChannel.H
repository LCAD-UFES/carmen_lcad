/*!@file Channels/ObjDetChannel.H object detection channel using opecv cascade detector */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/ObjDetChannel.H $
// $Id: ObjDetChannel.H 12821 2010-02-11 07:15:07Z itti $
//

#ifndef OBJDETCHANNEL_H_DEFINED
#define OBJDETCHANNEL_H_DEFINED

#include "Image/OpenCVUtil.H"  // must be first to avoid conflicting defs of int64, uint64

#include "Channels/ChannelBase.H"
#include "Image/LevelSpec.H"
#include "Image/fancynorm.H" // for MaxNormType

// ######################################################################
//! A template-matching channel for detecting pedestrians.
class ObjDetChannel : public ChannelBase
{
public:
  //! Constructor
  ObjDetChannel(OptionManager& mgr, const std::string & descrName = std::string("ObjDetChannel"), const std::string& = std::string("ObjDetChannel"));

  //! overload
  virtual bool outputAvailable() const;

  //! Virtual destructor.
  virtual ~ObjDetChannel();

  virtual uint numSubmaps() const;

  virtual Image<float> getSubmap(const uint index) const;

  virtual std::string getSubmapName(const uint index) const;

  virtual std::string getSubmapNameShort(const uint index) const;

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
  Image<float> itsMap;  //!< our face map
  OModelParam<LevelSpec> itsLevelSpec;
  OModelParam<std::string> itsCascadeFile;
  OModelParam<MaxNormType> itsNormType;

  //! Set a fixed range of values for the raw output
  OModelParam<float> itsOutputRangeMin;
  OModelParam<float> itsOutputRangeMax;
  OModelParam<bool> itsUseRandom;

  NModelParam<bool> itsNormalizeOutput;

#ifdef HAVE_OPENCV
  CvMemStorage* storage;
  CvHaarClassifierCascade* cascade;
#endif
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif
