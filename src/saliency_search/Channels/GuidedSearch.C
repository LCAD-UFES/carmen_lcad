/*!@file Channels/GuidedSearch.C Guided Search aka saliency computation */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/GuidedSearch.C $
// $Id: GuidedSearch.C 10794 2009-02-08 06:21:09Z itti $
//

#include "Channels/GuidedSearch.H"
#include "Channels/ChannelFacets.H"
#include "Channels/SingleChannel.H"
#include "Channels/ComplexChannel.H"
#include "Component/ParamMap.H"
#include "Util/sformat.H"

// ######################################################################
GuidedSearchBiaser::GuidedSearchBiaser(rutz::shared_ptr<ParamMap> pmap) :
  itsPmap(pmap)
{ }

// ######################################################################
GuidedSearchBiaser::~GuidedSearchBiaser()
{ }

// ######################################################################
void GuidedSearchBiaser::visitChannelBase(ChannelBase& chan)
{
  LFATAL("don't know how to handle %s", chan.tagName().c_str());
}

// ######################################################################
void GuidedSearchBiaser::visitSingleChannel(SingleChannel& chan)
{
  // get or install some ChannelFacet for the gains:
  rutz::shared_ptr<ChannelFacetGainSingle> gfacet;
  if (chan.hasFacet<ChannelFacetGainSingle>())
    gfacet = chan.getFacet<ChannelFacetGainSingle>();
  else
    { gfacet.reset(new ChannelFacetGainSingle(chan)); chan.setFacet(gfacet); }

  const uint num = chan.numSubmaps();
  for (uint idx = 0; idx < num; idx ++)
    {
      // get the gain for that submap:
      const float g = itsPmap->getDoubleParam(sformat("gain(%d)", idx));

      // load it up into the facet:
      gfacet->setVal(idx, g);

      uint clev = 0, slev = 0; chan.getLevelSpec().indexToCS(idx, clev, slev);
      LINFO("%s: Loaded gain(%d,%d) = %f", chan.tagName().c_str(),
            clev, slev, g);
    }
}

// ######################################################################
void GuidedSearchBiaser::visitComplexChannel(ComplexChannel& chan)
{
  // get or install some ChannelFacet for the gains:
  rutz::shared_ptr<ChannelFacetGainComplex> gfacet;
  if (chan.hasFacet<ChannelFacetGainComplex>())
    gfacet = chan.getFacet<ChannelFacetGainComplex>();
  else
    { gfacet.reset(new ChannelFacetGainComplex(chan)); chan.setFacet(gfacet); }

  // let's explicitly recurse over our subchannels:
  const uint num = chan.numChans();
  for (uint idx = 0; idx < num; idx ++)
    {
      // get the subpmap for the subchannel (will LFATAL if not found):
      rutz::shared_ptr<ParamMap> subpmap =
        itsPmap->getSubpmap(chan.subChan(idx)->tagName());

      // use the subpmap:
      itsPmap.swap(subpmap);

      // visit the subchan, it will load gains from itsPmap:
      chan.subChan(idx)->accept(*this);

      // restore our original itsPmap:
      itsPmap.swap(subpmap);

      // get the gain for that whole subchan:
      const float g = itsPmap->getDoubleParam(sformat("gain(%d)", idx));

      // load it up into our facet:
      gfacet->setVal(idx, g);

      LINFO("%s: Loaded gain(%s) = %f", chan.tagName().c_str(),
            chan.subChan(idx)->tagName().c_str(), g);
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
