/*!@file Transport/LowLevelEncoderMap.C A partial implementation of FrameOstream, using a map between 'shortnames' and LowLevelEncoder objects */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/LowLevelEncoderMap.C $
// $Id: LowLevelEncoderMap.C 8906 2007-10-25 23:30:51Z rjpeters $
//

#ifndef TRANSPORT_LOWLEVELENCODERMAP_C_DEFINED
#define TRANSPORT_LOWLEVELENCODERMAP_C_DEFINED

#include "Transport/LowLevelEncoderMap.H"

#include "Raster/GenericFrame.H"
#include "Util/Assert.H"

#include <map>

// ######################################################################
struct LowLevelEncoderMap::Impl
{
  typedef std::map<std::string, rutz::shared_ptr<LowLevelEncoder> > MapType;

  MapType encoders;
};

// ######################################################################
LowLevelEncoderMap::LowLevelEncoderMap(OptionManager& mgr,
                                       const std::string& descrName,
                                       const std::string& tagName)
  :
  FrameOstream(mgr, descrName, tagName),
  rep(0)
{}

// ######################################################################
LowLevelEncoderMap::~LowLevelEncoderMap()
{
  delete rep;
}

// ######################################################################
void LowLevelEncoderMap::writeFrame(const GenericFrame& frame,
                                    const std::string& shortname,
                                    const FrameInfo& auxinfo)
{

  if (shortname.length() == 0)
    LFATAL("must specify a non-empty filename");

  if (rep == 0)
    {
      rep = new Impl;
    }

  ASSERT(rep != 0);

  if (!rep->encoders[shortname].is_valid())
    rep->encoders[shortname] =
      this->makeEncoder(frame.frameSpec(), shortname, auxinfo);

  rutz::shared_ptr<LowLevelEncoder> enc = rep->encoders[shortname];

  ASSERT(enc.is_valid());

  enc->writeFrame(frame);
}

// ######################################################################
void LowLevelEncoderMap::closeStream(const std::string& shortname)
{
  if (rep != 0)
    {
      Impl::MapType::iterator itr = rep->encoders.find(shortname);

      if (itr != rep->encoders.end())
        {
          (*itr).second->close();
          rep->encoders.erase(itr);
        }
    }
}

// ######################################################################
void LowLevelEncoderMap::stop2()
{
  if (rep)
    {
      for (Impl::MapType::iterator itr = rep->encoders.begin(),
             stop = rep->encoders.end();
           itr != stop; ++itr)
        {
          itr->second->close();
          itr->second.reset(NULL);
        }

      delete rep;
      rep = 0;
    }

  FrameOstream::stop2();
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_LOWLEVELENCODERMAP_C_DEFINED
