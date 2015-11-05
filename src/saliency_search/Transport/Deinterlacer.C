/*!@file Transport/Deinterlacer.C Interface for deinterlacing video frames */

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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/Deinterlacer.C $
// $Id: Deinterlacer.C 14128 2010-10-12 23:39:22Z rand $
//

#ifndef TRANSPORT_DEINTERLACER_C_DEFINED
#define TRANSPORT_DEINTERLACER_C_DEFINED

#include "Transport/Deinterlacer.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/GenericFrame.H"
#include "Transport/FrameIstreamFactory.H"

// ######################################################################
Deinterlacer::Deinterlacer(OptionManager& mgr)
  :
  FrameIstream(mgr, "Deinterlacer ", "Deinterlacer"),
  itsDelegate()
{}

// ######################################################################
Deinterlacer::~Deinterlacer()
{}

// ######################################################################
void Deinterlacer::setDelegate(const nub::soft_ref<FrameIstream>& f)
{
  itsDelegate = f;
  if (itsDelegate.is_valid())
    {
      this->setDescriptiveName("Deinterlaced "
                               + itsDelegate->descriptiveName());
      this->setTagName("Deinterlaced "
                       + itsDelegate->tagName());
    }
  else
    {
      this->setDescriptiveName("Deinterlacer");
      this->setTagName("Deinterlacer");
    }
}

// ######################################################################
FrameIstream& Deinterlacer::getDelegate() const
{
  if (itsDelegate.is_invalid())
    LFATAL("I have no FrameIstream delegate!");

  return *itsDelegate;
}

// ######################################################################
void Deinterlacer::setConfigInfo(const std::string& cfg)
{
  this->setDelegate(makeFrameIstream(cfg));
  this->addSubComponent(itsDelegate);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_DEINTERLACER_C_DEFINED
