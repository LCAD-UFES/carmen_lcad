/*!@file Channels/ScorrChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/ScorrChannel.C $
// $Id: ScorrChannel.C 11118 2009-04-15 07:34:33Z itti $
//

#ifndef SCORRCHANNEL_C_DEFINED
#define SCORRCHANNEL_C_DEFINED

#include "Channels/ScorrChannel.H"

#include "Channels/ChannelOpts.H"
#include "Component/OptionManager.H"
#include "Image/MathOps.H"
#include "Util/MathFunctions.H"

// ######################################################################
// Scorr channel member definitions
// ######################################################################
ScorrChannel::ScorrChannel(OptionManager& mgr) :
  SingleChannel(mgr, "Scorr", "scorr", SCORR,
                rutz::shared_ptr< PyrBuilder<float> >(NULL)),
  itsRadius(&OPT_ScorrChannelRadius, this), // see Channels/ChannelOpts.{H,C}
  itsMap()
{ }

// ######################################################################
ScorrChannel::~ScorrChannel()
{ }

// ######################################################################
bool ScorrChannel::outputAvailable() const
{ return itsMap.initialized(); }

// accumulate correlation from a point pp:
#define ACCUMCORR if (itsMap.coordsOk(pp) && itsMask.getVal(pp) == 0) \
{ ++n; itsMask.setVal(pp, 255); \
corr += corrpatch(ima, topleft1, patchdims, ima, Point2D<int>(pp.i<<lev, pp.j<<lev)); }

// ######################################################################
void ScorrChannel::doInput(const InputFrame& inframe)
{
  const LevelSpec ls = itsLevelSpec.getVal();
  Image<float> ima = inframe.grayFloat();
  ASSERT(ls.levMin() == ls.levMax());
  ASSERT(ls.delMin() == 0 && ls.delMax() == 0);
  ASSERT(ls.levMin() == ls.mapLevel());
  ASSERT(ima.initialized());
  ASSERT(itsRadius.getVal() >= 1);

  const uint lev = ls.mapLevel();
  const int siz = 1 << lev;
  const int mw = ima.getWidth() >> lev, mh = ima.getHeight() >> lev;
  itsMap.resize(mw, mh);
  Image<byte> itsMask(mw, mh, NO_INIT);
  const int radius = itsRadius.getVal();
  const int r2 = radius * radius;

  // FIXME: this channel will not work with images whose dims are not
  // multiple of patch dims...
  Dims patchdims(siz, siz);

  // loop over the destination and compute spatial correlations. Code
  // here is similar to that in Image::drawDisk():
  Image<float>::iterator dest = itsMap.beginw();
  for (int j = 0; j < mh; ++j)
    for (int i = 0; i < mw; ++i)
      {
        double corr = 0.0; int n = 0; itsMask.clear();

        // our center patch's top-left corner:
        const Point2D<int> topleft1(i << lev, j << lev);

        // go over a circle and get cross-correlations. First do the
        // two horizontal extremes:
        Point2D<int> pp(i - radius, j); ACCUMCORR; pp.i = i + radius; ACCUMCORR;

        // now we draw one quarter of the circle and symmetrize it.
        // NOTE: in this algo like in Image::drawCircle() there is
        // some repetition (points that get drawn twice). Have a look at
        // http://www.cs.unc.edu/~mcmillan/comp136/Lecture7/circle.html
        // for possibly better algos. Here we don't want to count
        // those correlations several times, so we just use a mask to
        // keep track of those we already have counted:
        int bound1 = radius, bound2;
        for (int y = 1; y <= radius; ++y)
          {
            bound2 = bound1;
            bound1 = int(sqrtf(float(r2 - y*y)));
            for (int x = bound1; x <= bound2; ++x)
              {
                pp.j = j - y;
                pp.i = i - x; ACCUMCORR;
                pp.i = i + x; ACCUMCORR;
                pp.j = j + y; ACCUMCORR;
                pp.i = i - x; ACCUMCORR;
              }
          }

        if (n > 0)
          {
            const double val = 1.0 - corr / double(n);
            if (val >= 0.0) *dest++ = float(val); else *dest++ = 0.0F;
          }
        else *dest++ = 0.0F;
      }
}

// ######################################################################
Image<float> ScorrChannel::getOutput()
{ return itsMap; }


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // SCORRCHANNEL_C_DEFINED
