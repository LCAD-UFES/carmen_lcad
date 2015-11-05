/*!@file Channels/FlickerChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/FlickerChannel.C $
// $Id: FlickerChannel.C 14477 2011-02-04 22:52:28Z dberg $
//

#ifndef FLICKERCHANNEL_C_DEFINED
#define FLICKERCHANNEL_C_DEFINED

#include "Channels/FlickerChannel.H"

#include "Image/MathOps.H" // for absDiff()
#include "rutz/trace.h"

// ######################################################################
// FlickerChannel member definitions:
// ######################################################################

// ######################################################################
FlickerChannel::FlickerChannel(OptionManager& mgr, const std::string& descrNam, const std::string& tagNam) :
  GaussianPyrChannel(mgr, descrNam, tagNam, FLICKER),
  itsTimeConst("FlickerChannelTimeConst", this, SimTime::SECS(0.03)),
  itsPrevInput(),
  itsPrevTime(SimTime::ZERO())
{
GVX_TRACE(__PRETTY_FUNCTION__);
  itsTakeAbs.setVal(true);
  itsNormalizeOutput.setVal(true);
}

// ######################################################################
FlickerChannel::~FlickerChannel()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void FlickerChannel::doInput(const InputFrame& inframe)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(inframe.grayFloat().initialized());

  // If this is the first time the flicker channel has seen input,
  // then itsPrevInput will be uninitialized; obviously we can't
  // compute any flicker with only one frame, so we just store the
  // current input as the next iteration's previous input
  if (!itsPrevInput.initialized())
    {
      itsPrevInput = inframe.grayFloat();
      itsPrevTime = inframe.time();
    }
  else
    {
      // take simple abs difference between current and previous frame:
      Image<float> fli = absDiff(inframe.grayFloat(), itsPrevInput);

      SingleChannel::doInput(InputFrame::fromGrayFloat
                             (&fli, inframe.time(),
                              &inframe.clipMask(), inframe.pyrCache()));

      // update the previous image:
      if (inframe.time() - itsPrevTime >= itsTimeConst.getVal())
        {
          itsPrevInput = inframe.grayFloat();
          itsPrevTime = inframe.time();
        }
      else
        {
          const float fac =
            float( (inframe.time() - itsPrevTime).secs() / itsTimeConst.getVal().secs() );
          itsPrevInput = itsPrevInput * (1.0F - fac) + inframe.grayFloat() * fac;
          itsPrevTime = inframe.time();
        }
    }
}

// ######################################################################
void FlickerChannel::reset1()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // reset some stuff in FlickerChannel
  itsPrevInput.freeMem();

  // propagate to our base class:
  SingleChannel::reset1();
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // FLICKERCHANNEL_C_DEFINED
