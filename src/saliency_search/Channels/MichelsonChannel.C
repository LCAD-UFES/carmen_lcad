/*!@file Channels/MichelsonChannel.C Michelson image contrast */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/MichelsonChannel.C $
// $Id: MichelsonChannel.C 12962 2010-03-06 02:13:53Z irock $
//

#include "Channels/MichelsonChannel.H"

#include "Image/MathOps.H"

// ######################################################################
// Michelson channel member definitions
// ######################################################################
MichelsonChannel::MichelsonChannel(OptionManager& mgr) :
  SingleChannel(mgr, "Michelson", "michelson", MICHELSON,
                rutz::make_shared(new GaussianPyrBuilder<float>(5))),
  itsMap()
{ }

// ######################################################################
MichelsonChannel::~MichelsonChannel()
{ }

// ######################################################################
bool MichelsonChannel::outputAvailable() const
{ return itsMap.initialized(); }

// ######################################################################
void MichelsonChannel::doInput(const InputFrame& inframe)
{
  const LevelSpec ls = itsLevelSpec.getVal();
  ASSERT(ls.levMin() == ls.levMax());
  ASSERT(ls.delMin() == 0 && ls.delMax() == 0);
  ASSERT(ls.levMin() == ls.mapLevel());
  ASSERT(inframe.grayFloat().initialized());

  // we use our pyramid to downscale the image to level lev:
  SingleChannel::doInput(inframe); // will store the new pyramid
  const Image<float> lum = getImage(ls.mapLevel());
  itsMap.resize(lum.getDims());

  // Michelson contrast in every map pixel as abs( (La-Li) / (La+Li) )
  // where La is the local patch luminance and Li the average image
  // luminance:
  float mi, ma, li; getMinMaxAvg(lum, mi, ma, li);

  Image<float>::const_iterator src = lum.begin(), stop = lum.end();
  Image<float>::iterator dest = itsMap.beginw();
  while (src != stop) {
    const float la = *src++;
    *dest++ = fabs( (la - li) / (la + li) );
  }
}

// ######################################################################
Image<float> MichelsonChannel::getOutput()
{ return itsMap; }


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
