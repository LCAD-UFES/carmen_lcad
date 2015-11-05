/*!@file Channels/RGBConvolveChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/RGBConvolveChannel.C $
// $Id: RGBConvolveChannel.C 8643 2007-07-27 18:15:43Z rjpeters $
//

#ifndef RGBCONVOLVECHANNEL_C_DEFINED
#define RGBCONVOLVECHANNEL_C_DEFINED

#include "Channels/RGBConvolveChannel.H"

// ######################################################################
// RGBConvolveChannel member definitions:
// ######################################################################

RGBConvolveChannel::RGBConvolveChannel(OptionManager& mgr) :
  SingleChannel(mgr, "RGBConvolve", "rgbconvolve", RGBCONVOLVE,
                rutz::shared_ptr< PyrBuilder<float> >(NULL))
{
  itsTakeAbs.setVal(true);
  itsNormalizeOutput.setVal(true);
}

void RGBConvolveChannel::setFilters(const Image<float>& rfilter,
                                    const Image<float>& gfilter,
                                    const Image<float>& bfilter,
                                    ConvolutionBoundaryStrategy boundary)
{
  setPyramid(rutz::make_shared
             (new RGBConvolvePyrBuilder<float>
              (rfilter, gfilter, bfilter, boundary)));
}

RGBConvolveChannel::~RGBConvolveChannel() {}

void RGBConvolveChannel::doInput(const InputFrame& inframe)
{
  ASSERT(inframe.colorFloat().initialized());

  setClipPyramid(inframe.clipMask());
  storePyramid(computePyramid(inframe.colorFloat(), inframe.pyrCache()),
               inframe.time());
}

ImageSet<float> RGBConvolveChannel::
computePyramid(const Image< PixRGB<float> >& img,
               const rutz::shared_ptr<PyramidCache<float> >& cache)
{
  rutz::shared_ptr< RGBConvolvePyrBuilder<float> > pb;
  pb.dynCastFrom(itsPyrBuilder);

  // If we want to save our raw pyramid maps, then let's compute the
  // pyramid in full; otherwise, we can skip the levels below our
  // LevelSpec's levMin():
  const int minlev =
   itsSaveRawMaps.getVal() ? 0 : itsLevelSpec.getVal().levMin();

  // Compute our pyramid and return it
  return pb->build(img, minlev, itsLevelSpec.getVal().maxDepth(), cache.get());
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // RGBCONVOLVECHANNEL_C_DEFINED
