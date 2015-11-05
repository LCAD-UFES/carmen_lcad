/*!@file Channels/ChannelVisitorInstallFacet.C Install some ChannelFacet */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/ChannelVisitorInstallFacet.C $
// $Id: ChannelVisitorInstallFacet.C 10794 2009-02-08 06:21:09Z itti $
//

#include "Channels/ChannelVisitorInstallFacet.H"
#include "Channels/SingleChannel.H"
#include "Channels/ComplexChannel.H"
#include "Channels/ChannelFacets.H"
#include "rutz/demangle.h"

// ######################################################################
template <class CFS, class CFC>
ChannelVisitorInstallFacet<CFS, CFC>::ChannelVisitorInstallFacet()
  :
  ChannelVisitor()
{ }

// ######################################################################
template <class CFS, class CFC>
ChannelVisitorInstallFacet<CFS, CFC>::~ChannelVisitorInstallFacet()
{ }

// ######################################################################
template <class CFS, class CFC>
void ChannelVisitorInstallFacet<CFS, CFC>::
visitChannelBase(ChannelBase& chan)
{
  LFATAL("don't know how to handle %s", chan.tagName().c_str());
}

// ######################################################################
template <class CFS, class CFC>
void ChannelVisitorInstallFacet<CFS, CFC>::
visitSingleChannel(SingleChannel& chan)
{
  if (chan.hasFacet<CFS>() == false)
    {
      LINFO("Installing %s onto: %s", rutz::demangled_name(typeid(CFS)),
            chan.descriptiveName().c_str());

      rutz::shared_ptr<CFS> f(new CFS(chan));
      chan.setFacet<CFS>(f);
    }
}

// ######################################################################
template <class CFS, class CFC>
void ChannelVisitorInstallFacet<CFS, CFC>::
visitComplexChannel(ComplexChannel& chan)
{
  if (chan.hasFacet<CFC>() == false)
    {
      LINFO("Installing %s onto: %s", rutz::demangled_name(typeid(CFC)),
            chan.descriptiveName().c_str());

      rutz::shared_ptr<CFC> f(new CFC(chan));
      chan.setFacet<CFC>(f);
    }

  // explicitly recurse over our subchannels:
  for (uint i = 0; i < chan.numChans(); ++i) chan.subChan(i)->accept(*this);
}

// ######################################################################
// instantiations:
template class ChannelVisitorInstallFacet<class ChannelFacetGainSingle,
                                          class ChannelFacetGainComplex>;

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
