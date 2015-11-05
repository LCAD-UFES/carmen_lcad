/*!@file Transport/FrameOfilt.C Generic base class for output frame filters */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/FrameOfilt.C $
// $Id: FrameOfilt.C 9547 2008-03-28 23:32:43Z rjpeters $
//

#ifndef TRANSPORT_FRAMEOFILT_C_DEFINED
#define TRANSPORT_FRAMEOFILT_C_DEFINED

#include "Transport/FrameOfilt.H"

#include "Transport/FrameOstreamFactory.H"

// ######################################################################
FrameOfilt::FrameOfilt(OptionManager& mgr,
                       const std::string& descrName,
                       const std::string& tagName)
  :
  FrameOstream(mgr, descrName, tagName),
  itsDest()
{}

// ######################################################################
FrameOfilt::~FrameOfilt()
{}

// ######################################################################
void FrameOfilt::setConfigInfo(const std::string& cfg)
{
  if (itsDest.is_valid())
    this->removeSubComponent(*itsDest);

  itsDest = makeFrameOstream(cfg, this->getManager());
  this->addSubComponent(itsDest);
  itsDest->exportOptions(MC_RECURSE);
}

// ######################################################################
bool FrameOfilt::setFrameNumber(int n)
{
  if (itsDest.is_valid())
    return itsDest->setFrameNumber(n);

  return true;
}

// ######################################################################
void FrameOfilt::writeFrame(const GenericFrame& frame,
                            const std::string& shortname,
                            const FrameInfo& auxinfo)
{
  if (itsDest.is_valid())
    this->filterFrame(*itsDest, frame, shortname, auxinfo);
}

// ######################################################################
bool FrameOfilt::isVoid() const
{
  return itsDest.is_invalid() || itsDest->isVoid();
}

// ######################################################################
void FrameOfilt::closeStream(const std::string& shortname)
{
  if (itsDest.is_valid())
    itsDest->closeStream(shortname);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_FRAMEOFILT_C_DEFINED
