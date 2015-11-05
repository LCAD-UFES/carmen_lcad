/*!@file Channels/DepthMotionChannel.H A depth channel. */

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
// Primary maintainer for this file: Laurent Itti
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/DepthMotionChannel.H $
// $Id: DepthMotionChannel.H 14293 2010-12-02 01:57:25Z itti $
//

#ifndef DepthMotionChannel_H_DEFINED
#define DepthMotionChannel_H_DEFINED

#include "Channels/ComplexChannel.H"
#include "Image/LevelSpec.H"
#include "Image/PyramidTypes.H"
#include "rutz/shared_ptr.h"
#include "Channels/DirectionChannel.H"

class DirectionChannel;

//######################################################################
//! A Depth channel for computing saliency based on a distance/depth to objects
/*! This channel will compute a depth map using orientations and intensity
    in the usual way one would do so with a standard input image. This gives
    conspicuity to objects in an image based on how far they are from the
    camera. */
class DepthMotionChannel : public ComplexChannel
{
public:
  //! Construct with standard params
  DepthMotionChannel(OptionManager& mgr);

  //! Destructor
  virtual ~DepthMotionChannel();

  //! returns a specific DirectionChannel
  virtual DirectionChannel& dirChan(const uint idx) const;

  //! Overload so that we can reconfigure when our params get changed
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

protected:

  OModelParam<LevelSpec> itsLevelSpec;

  //! type of pyramid to use in our DirectionChannel subcomponents
  NModelParam<PyramidType> itsPyrType;

  //! number of DirectionChannel subcomponents
  OModelParam<uint> itsNumDirs;

  //! (re-)build our subchannels
  virtual void buildSubChans();

  //! DepthMotionChannel requires only depth input.
  virtual void doInput(const InputFrame& inframe);


};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif
