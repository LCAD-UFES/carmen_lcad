/*!@file Neuro/VisualCortexBeo.C a VisualCortex with SingleChannelBeo channels */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/VisualCortexBeo.C $
// $Id: VisualCortexBeo.C 10845 2009-02-13 08:49:12Z itti $
//

#include "Neuro/VisualCortexBeo.H"

#include "Beowulf/Beowulf.H"
#include "Channels/ChannelVisitor.H"
#include "Neuro/SingleChannelBeo.H"
#include "Channels/RawVisualCortex.H"

namespace
{
  class BeoInstaller : public ChannelVisitor
  {
  public:
    BeoInstaller(nub::ref<Beowulf> beo) : itsBeo(beo)
    {}

    virtual ~BeoInstaller()
    {}

    virtual void visitChannelBase(ChannelBase& chan)
    {
      LFATAL("I don't know how to configure channel '%s' for Beowulf use", chan.descriptiveName().c_str());
    }

    virtual void visitSingleChannel(SingleChannel& chan)
    {
      chan.setInputHandler(rutz::make_shared(new SingleChannelBeo(itsBeo)));
    }

    virtual void visitComplexChannel(ComplexChannel& chan)
    {
      chan.setSubchanVisitor(rutz::make_shared(new BeoInstaller(itsBeo)));

      // now iterate over the subchannels:
      for (uint i = 0; i < chan.numChans(); ++i) chan.subChan(i)->accept(*this);
    }

  private:
    nub::ref<Beowulf> itsBeo;
  };
}

// ######################################################################
void setupVisualCortexBeo(RawVisualCortex& vcx, nub::ref<Beowulf> beo)
{
  vcx.setSubchanVisitor(rutz::make_shared(new BeoInstaller(beo)));
  vcx.sortChannelsByNumSubmaps(true);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
