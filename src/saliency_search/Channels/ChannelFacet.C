/*!@file Channels/ChannelFacet.C Allow user-configurable "facets" to be associated with arbitrary classes */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/ChannelFacet.C $
// $Id: ChannelFacet.C 9748 2008-05-11 19:21:54Z rjpeters $
//

#ifndef CHANNELS_CHANNELFACET_C_DEFINED
#define CHANNELS_CHANNELFACET_C_DEFINED

#include "Channels/ChannelFacet.H"

#include "Component/ParamMap.H"
#include "rutz/demangle.h"

#include <map>
#include <string>

typedef std::map<std::string, rutz::shared_ptr<ChannelFacet> > map_type;

struct ChannelFacetMap::Impl
{
  map_type facets;
};

// ######################################################################
ChannelFacet::ChannelFacet() {}

// ######################################################################
ChannelFacet::~ChannelFacet() {}

// ######################################################################
ChannelFacetMap::ChannelFacetMap()
  :
  rep(new Impl)
{}

// ######################################################################
ChannelFacetMap::~ChannelFacetMap()
{
  delete rep;
}

// ######################################################################
void ChannelFacetMap::writeFacetsTo(ParamMap& pmap) const
{
  for (map_type::const_iterator
         itr = rep->facets.begin(), stop = rep->facets.end();
       itr != stop; ++itr)
    {
      rutz::shared_ptr<ParamMap> submap(new ParamMap);
      (*itr).second->writeTo(*submap);
      pmap.putSubpmap((*itr).first, submap);
    }
}

// ######################################################################
void ChannelFacetMap::readFacetsFrom(const ParamMap& pmap)
{
  for (map_type::const_iterator
         itr = rep->facets.begin(), stop = rep->facets.end();
       itr != stop; ++itr)
    {
      const std::string name = (*itr).first;
      if (!pmap.hasParam(name))
        LFATAL("no such facet in ParamMap: '%s'", name.c_str());

      rutz::shared_ptr<ParamMap> submap = pmap.getSubpmap(name);
      (*itr).second->readFrom(*submap);
    }
}

// ######################################################################
unsigned int ChannelFacetMap::numFacets() const
{
  return rep->facets.size();
}

// ######################################################################
rutz::shared_ptr<ChannelFacet>
ChannelFacetMap::getBaseFacet(const std::type_info& tp) const
{
  map_type::const_iterator itr =
    rep->facets.find(rutz::demangled_name(tp));
  if (itr == rep->facets.end())
    return rutz::shared_ptr<ChannelFacet>();
  // else...
  return (*itr).second;
}

// ######################################################################
void ChannelFacetMap::setBaseFacet(const std::type_info& tp,
                                   rutz::shared_ptr<ChannelFacet> f)
{
  rep->facets.insert(map_type::value_type(rutz::demangled_name(tp), f));
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_CHANNELFACET_C_DEFINED
