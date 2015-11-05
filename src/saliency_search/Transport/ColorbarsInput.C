/*!@file Transport/ColorbarsInput.C FrameIstream class that generates a static "colorbars" test pattern */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/ColorbarsInput.C $
// $Id: ColorbarsInput.C 15310 2012-06-01 02:29:24Z itti $
//

#ifndef TRANSPORT_COLORBARSINPUT_C_DEFINED
#define TRANSPORT_COLORBARSINPUT_C_DEFINED

#include "Transport/ColorbarsInput.H"

#include "Image/DrawOps.H"
#include "Raster/GenericFrame.H"
#include "Util/sformat.H"
#include "rutz/fstring.h"
#include "rutz/time.h"
#include "rutz/timeformat.h"
#include <unistd.h>

namespace
{
  void fillRegion(Image<PixRGB<byte> >& img, PixRGB<byte> col,
                  const int x0, const int x1,
                  const int y0, const int y1)
  {
    for (int x = x0; x < x1; ++x)
      for (int y = y0; y < y1; ++y)
        img.setVal(x, y, col);
  }
}

// ######################################################################
ColorbarsInput::ColorbarsInput(OptionManager& mgr)
  :
  FrameIstream(mgr, "Colorbars Input", "ColorbarsInput"),
  itsImage(),
  itsFramePeriod(SimTime::ZERO()),
  itsFrameNumber(0)
{
  const int w = 320;
  const int h = 240;

  itsImage = Image<PixRGB<byte> >(w, h, ZEROS);

  const PixRGB<byte> cols[] =
    {
      PixRGB<byte>(255, 255, 255), // white
      PixRGB<byte>(255, 255, 0),   // yellow
      PixRGB<byte>(0,   255, 255), // cyan
      PixRGB<byte>(0,   255, 0),   // green
      PixRGB<byte>(255, 0,   255), // magenta
      PixRGB<byte>(255, 0,   0),   // red
      PixRGB<byte>(0,   0,   255)  // blue
    };

  int x1 = 0;
  for (int i = 0; i < 7; ++i)
    {
      const int x0 = x1+1;
      x1 = int(double(w)*(i+1)/7.0 + 0.5);
      fillRegion(itsImage, cols[i],
                 x0, x1,
                 0, int(h*2.0/3.0));
    }

  x1 = 0;
  for (int i = 0; i < 16; ++i)
    {
      const int x0 = x1;
      x1 = int(double(w)*(i+1)/16.0 + 0.5);
      const int gray = int(255.0*i/15.0 + 0.5);
      fillRegion(itsImage, PixRGB<byte>(gray, gray, gray),
                 x0, x1,
                 int(h*2.0/3.0)+1, int(h*5.0/6.0));
    }

  fillRegion(itsImage, PixRGB<byte>(255, 0, 0),
             0, w,
             int(h*5.0/6.0)+1, h);

  writeText(itsImage, Point2D<int>(1, int(h*5.0/6.0)+2),
            "iLab Neuromorphic Vision",
            PixRGB<byte>(0, 0, 0), PixRGB<byte>(255, 0, 0),
            SimpleFont::FIXED(10));
}

// ######################################################################
ColorbarsInput::~ColorbarsInput()
{}

// ######################################################################
void ColorbarsInput::setConfigInfo(const std::string& frametime)
{
  // NOTE: if you modify any behavior here, then please update the
  // corresponding documentation for the global "--in" option inside
  // the OPT_InputFrameSource definition in Media/MediaOpts.C

  if (frametime.size() == 0)
    itsFramePeriod = SimTime::ZERO();
  else
    itsFramePeriod = SimTime::fromString(frametime);
}

// ######################################################################
bool ColorbarsInput::setFrameNumber(int n)
{
  itsFrameNumber = n;

  return true;
}

// ######################################################################
GenericFrameSpec ColorbarsInput::peekFrameSpec()
{
  GenericFrameSpec result;

  result.nativeType = GenericFrame::RGB_U8;
  result.videoFormat = VIDFMT_AUTO;
  result.videoByteSwap = false;
  result.dims = itsImage.getDims();
  result.floatFlags = 0;

  return result;
}

// ######################################################################
GenericFrame ColorbarsInput::readFrame()
{
  if (itsStartTime.sec() == 0.0)
    itsStartTime = rutz::time::wall_clock_now();

  const rutz::time nexttime(itsStartTime.sec()
                            + itsFrameNumber * itsFramePeriod.secs());

  while (rutz::time::wall_clock_now() < nexttime)
    { usleep(1000); }

  Image<PixRGB<byte> > result = itsImage;
  std::string fnum = sformat("%06d", itsFrameNumber);
  writeText(result, Point2D<int>(1, 1),
            fnum.c_str(),
            PixRGB<byte>(0, 0, 0), PixRGB<byte>(255, 255, 255),
            SimpleFont::FIXED(10));

  rutz::time t = rutz::time::wall_clock_now();

  writeText(result, Point2D<int>(1, result.getHeight() - 14),
            rutz::format_time(t).c_str(),
            PixRGB<byte>(32, 32, 32), PixRGB<byte>(255, 0, 0),
            SimpleFont::FIXED(6));

  return GenericFrame(result);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_COLORBARSINPUT_C_DEFINED
