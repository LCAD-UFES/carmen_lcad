/*!@file Channels/DirectionChannel.C */

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
// Primary maintainer for this file:
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/DirectionChannel.C $
// $Id: DirectionChannel.C 8195 2007-03-30 04:34:07Z rjpeters $
//

#ifndef DIRECTIONCHANNEL_C_DEFINED
#define DIRECTIONCHANNEL_C_DEFINED

#include "Channels/DirectionChannel.H"

#include "Channels/ChannelOpts.H"
#include "Component/OptionManager.H"
#include "Image/ImageSetOps.H"
#include "Image/MathOps.H"
#include "Util/sformat.H"
#include "rutz/compat_cmath.h" // for M_PI
#include "rutz/trace.h"

// ######################################################################
// DirectionChannel member definitions:
// ######################################################################

// ######################################################################
DirectionChannel::DirectionChannel(OptionManager& mgr,
                                   const uint dirIndex,
                                   const double direction,
                                   const PyramidType type) :
  SingleChannel(mgr, "", "", MOTION,
                rutz::make_shared(new ReichardtPyrBuilder<float>
                           (cos(direction * M_PI / 180.0),
                            -sin(direction * M_PI / 180.0),
                            type, direction + 90.0))),
  itsIndex("DirectionChannelIndex", this, dirIndex),
  itsDirection("DirectionChannelDirection", this, direction),
  itsTakeSqrt(&OPT_DirectionChannelTakeSqrt, this),
  itsThresh(&OPT_DirectionChannelLowThresh, this)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  itsTakeAbs.setVal(true);
  itsNormalizeOutput.setVal(true);
  itsScaleNoiseToMax.setVal(true);
  itsLowThresh.setVal(itsThresh.getVal());  // kill small values in pyramid
  itsRectifyPyramid.setVal(true); // kill negative values in pyramid

  setDescriptiveName(sformat("Direction(%d)", int(direction)));
  setTagName(sformat("dir_%d", dirIndex));
}

// ######################################################################
void DirectionChannel::start2()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  itsLowThresh.setVal(itsThresh.getVal());  // kill small values in pyramid
}

// ######################################################################
DirectionChannel::~DirectionChannel()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
ImageSet<float> DirectionChannel::
computePyramid(const Image<float>& bwimg,
               const rutz::shared_ptr<PyramidCache<float> >& cache)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // get the pyramid as usual:
  ImageSet<float> py = SingleChannel::computePyramid(bwimg, cache);

  // do we want to take the sqrt() of it?
  if (itsTakeSqrt.getVal()) doSqrt(py);

  return py;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // DIRECTIONCHANNEL_C_DEFINED
