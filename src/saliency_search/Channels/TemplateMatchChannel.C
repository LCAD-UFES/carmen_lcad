/*!@file Channels/TemplateMatchChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/TemplateMatchChannel.C $
// $Id: TemplateMatchChannel.C 9412 2008-03-10 23:10:15Z farhan $
//

#ifndef TEMPLATEMATCHCHANNEL_C_DEFINED
#define TEMPLATEMATCHCHANNEL_C_DEFINED

#include "Channels/TemplateMatchChannel.H"

#include "Image/CutPaste.H"
#include "Image/DrawOps.H"
#include "Image/MathOps.H"
#include "Image/Pixels.H"
#include "Image/ShapeOps.H"

// ######################################################################
// TemplateMatchChannel member definitions:
// ######################################################################

// ######################################################################
TemplateMatchChannel::TemplateMatchChannel(OptionManager& mgr,
                                           const Image<float>& filt) :
  SingleChannel(mgr, "Template Match", "template", TEMPLATE,
                rutz::make_shared(new TemplateMatchPyrBuilder(filt))),
  itsBestMatchPos(-1, -1),
  itsBestMatchLev(-1),
  itsBestMatchScore(-1.0e30F),
  itsFiltDims(filt.getDims())
{  }

// ######################################################################
TemplateMatchChannel::~TemplateMatchChannel() {}

// ######################################################################
void TemplateMatchChannel::start1()
{
  SingleChannel::start1();
}

// ######################################################################
void TemplateMatchChannel::findBestMatch(const Rectangle& r,
                                         Point2D<int>& matchpos,
                                         int& match_lev, float& score)
{
  itsBestMatchScore = -1.0e30F;
  for (uint i = itsLevelSpec.getVal().levMin();
       i <= itsLevelSpec.getVal().levMax(); ++i)
    {
      Image<float> zone = rescale(this->getImage(i), this->getInputDims());

      zone = crop(zone, r, true);   // search zone

      Point2D<int> p; float val;
      findMax(zone, p, val);
      if (val > itsBestMatchScore)
        {
          itsBestMatchPos.i = p.i + r.left();
          itsBestMatchPos.j = p.j + r.top();
          itsBestMatchLev = i;
          itsBestMatchScore = val;
        }
    }
  LINFO("best match: (%d, %d) at scale %d [%f]", itsBestMatchPos.i,
        itsBestMatchPos.j, itsBestMatchLev, itsBestMatchScore);

  // return our results:
  matchpos = itsBestMatchPos; match_lev = itsBestMatchLev;
  score = itsBestMatchScore;
}

// ######################################################################
void TemplateMatchChannel::drawResults(Image< PixRGB<byte> >& traj,
                                       const Rectangle& foa)
{
  // first look for the best match in the given foa
  Point2D<int> dummy1; int dummy2; float dummy3;
  this->findBestMatch(foa, dummy1, dummy2, dummy3);

  if (itsBestMatchLev < 0) return; // nothing to draw

  const int fw = itsFiltDims.w(), fh = itsFiltDims.h();

  int pfac = (1 << itsBestMatchLev) / 2;
  Rectangle pedrect =
    Rectangle::tlbrI(std::max(itsBestMatchPos.j - fh*pfac, 0),
                    std::max(itsBestMatchPos.i - fw*pfac, 0),
                    std::min(itsBestMatchPos.j + fh*pfac, traj.getHeight()-1),
                    std::min(itsBestMatchPos.i + fw*pfac, traj.getWidth()-1));

  // display bounding box around pedestrian:
  PixRGB<byte> redPix(255, 0, 0);
  drawRect(traj, pedrect, redPix, 1);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // TEMPLATEMATCHCHANNEL_C_DEFINED
