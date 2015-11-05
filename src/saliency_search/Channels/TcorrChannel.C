/*!@file Channels/TcorrChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/TcorrChannel.C $
// $Id: TcorrChannel.C 9412 2008-03-10 23:10:15Z farhan $
//

#ifndef TCORRCHANNEL_C_DEFINED
#define TCORRCHANNEL_C_DEFINED

#include "Channels/TcorrChannel.H"

#include "Channels/ChannelOpts.H"
#include "Component/OptionManager.H"
#include "Image/MathOps.H"

// ######################################################################
// Tcorr channel member definitions
// ######################################################################
TcorrChannel::TcorrChannel(OptionManager& mgr) :
  SingleChannel(mgr, "Tcorr", "tcorr", TCORR,
                rutz::shared_ptr< PyrBuilder<float> >(NULL)),
  itsFrameLag(&OPT_TcorrChannelFrameLag, this), // see Channels/ChannelOpts.{H,C}
  itsCache(), itsMap()
{ }

// ######################################################################
TcorrChannel::~TcorrChannel()
{ }

// ######################################################################
bool TcorrChannel::outputAvailable() const
{ return (itsCache.size() == itsCache.getMaxSize()); }

// ######################################################################
void TcorrChannel::doInput(const InputFrame& inframe)
{
  const LevelSpec ls = itsLevelSpec.getVal();
  ASSERT(ls.levMin() == ls.levMax());
  ASSERT(ls.delMin() == 0 && ls.delMax() == 0);
  ASSERT(ls.levMin() == ls.mapLevel());
  ASSERT(inframe.grayFloat().initialized());

  // configure our cache if not done yet. We use one more frame in the
  // cache than the given lag, to hold our current frame:
  if (itsCache.getMaxSize() == 0)
    itsCache.setMaxSize(itsFrameLag.getVal() + 1);

  // just push our incoming input. All computations are done in getOutput():
  itsCache.push_back(inframe.grayFloat());
}

// ######################################################################
Image<float> TcorrChannel::getOutput()
{
  // if cache not full yet, we should not be called since
  // outputAvailable() returns false:
  if (itsCache.size() < itsCache.getMaxSize())
    {
      LERROR("I don't have an output yet -- RETURNING EMPTY");
      return itsMap; // should be uninitialized
    }

  // figure out the tile and map sizes:
  const Image<float> cur = itsCache.front();
  const Image<float> old = itsCache.back();

  const LevelSpec ls = itsLevelSpec.getVal();
  const uint lev = ls.mapLevel();
  const int siz = 1 << lev;
  const int w = cur.getWidth();
  const int h = cur.getHeight();
  itsMap.resize(w >> lev, h >> lev);

  // let's loop over the tiles and compute correlations:
  Image<float>::iterator dest = itsMap.beginw();
  for (int j = 0; j <= h-siz; j += siz)
    {
      const int ph = std::min(j + siz, h) - j;
      for (int i = 0; i <= w-siz; i += siz)
        {
          const int pw = std::min(i + siz, w) - i;

          // use corrpatch function of Image_MathOps, which returns
          // values in [-1 .. 1]. For this channel, we want high
          // 'salience' for patches that are highly decorrelated from
          // frame to frame. So in our map we store 1-correlation,
          // i.e., some measure of decorrelation:
          const Point2D<int> topleft(i, j); const Dims patchdims(pw, ph);

          *dest++ = 1.0 - corrpatch(cur, topleft, patchdims, old, topleft);
        }
    }

  return itsMap;
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // TCORRCHANNEL_C_DEFINED
