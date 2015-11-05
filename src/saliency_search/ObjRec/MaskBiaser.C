/*!@file ObjRec/MaskBiaser.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ObjRec/MaskBiaser.C $
// $Id: MaskBiaser.C 7966 2007-02-21 22:01:07Z lior $
//

#ifndef OBJREC_MASKBIASER_C_DEFINED
#define OBJREC_MASKBIASER_C_DEFINED

#include "ObjRec/MaskBiaser.H"

#include "Channels/ComplexChannel.H"
#include "Channels/SingleChannel.H"

// ######################################################################
MaskBiaser::MaskBiaser(Image<float>& mask,
                               const bool dobias)
  :
  itsBiasMask(mask),
  itsDoBias(dobias),
  itsIndex(0)
{}

// ######################################################################
MaskBiaser::~MaskBiaser() {}

// ######################################################################
void MaskBiaser::visitChannelBase(ChannelBase& chan)
{
  LFATAL("don't know how to handle %s", chan.tagName().c_str());
}

// ######################################################################
void MaskBiaser::visitSingleChannel(SingleChannel& chan)
{
  for (uint i = 0; i < chan.numSubmaps(); ++i)
  {
    if (itsDoBias)
      chan.setBiasMask(itsBiasMask);
    else
    {
      Image<float> tmp;
      chan.setBiasMask(tmp);
    }


    ++itsIndex;
  }
}

// ######################################################################
void MaskBiaser::visitComplexChannel(ComplexChannel& chan)
{
  for (uint i = 0; i < chan.numChans(); ++i)
    chan.subChan(i)->accept(*this);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // OBJREC_MASKBIASER_C_DEFINED
