/*!@file Channels/JetFiller.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/JetFiller.C $
// $Id: JetFiller.C 9412 2008-03-10 23:10:15Z farhan $
//

#ifndef CHANNELS_JETFILLER_C_DEFINED
#define CHANNELS_JETFILLER_C_DEFINED

#include "Channels/JetFiller.H"

#include "Channels/ComplexChannel.H"
#include "Channels/SingleChannel.H"
#include "Channels/VisualFeatures.H"
#include "Image/PyramidOps.H"

// ######################################################################
JetFiller::JetFiller(const Point2D<int>& loc, Jet<float>& j, bool do_interp)
  :
  itsLoc(loc),
  itsJet(j),
  itsDoInterp(do_interp),
  itsIndex()
{}

// ######################################################################
JetFiller::~JetFiller()
{}

// ######################################################################
void JetFiller::visitChannelBase(ChannelBase& chan)
{
  LFATAL("don't know how to handle %s", chan.tagName().c_str());
}

// ######################################################################
void JetFiller::visitSingleChannel(SingleChannel& chan)
{
  if (chan.numPyramids() == 0) LFATAL("I have no input pyramid yet!");

  const LevelSpec ls = chan.getLevelSpec();

  // A trivial implementation would go as follows for the RAW case
  // (modulo some checking that the jet can accept the indices we
  // have):
  /*
  for (uint i = 0; i < ls.levMax() + ls.delMax(); i++)
    {
      float pix = getPyrPixel(itsPq.front().pyr, itsLoc, i);
      itsIndex.push_back(i);
      itsJet.setVal(pix, chan.visualFeature(), RAW, itsIndex);
      itsIndex.pop_back();
    }
  */

  // The implementation below is slightly faster by minimizing the
  // number of Jet index computations, using only one getIndexV() call
  // and then raw Jet array accesses. But it is identical in spirit.
  int rmin, rmax;
  if (itsJet.getIndexRange(chan.visualFeature(), RAW,
                           itsIndex.size(), rmin, rmax))
    {
      itsIndex.push_back(rmin);  // raw features use a single index
      int idx = itsJet.getSpec()->getIndexV(chan.visualFeature(),
                                            RAW, itsIndex);
      const int ns = int(ls.levMax() + ls.delMax());
      const int first = std::max(rmin, 0);
      const int last = std::min(rmax, ns);

      // We have data for scales [0..ns]; the Jet wants data for
      // scales [rmin..rmax] and we will put zeros in the Jet for all
      // Jet scales that it has but we don't have
      idx -= rmin;  // offset to account for rmin
      for (int i = rmin; i < 0; ++i)
        itsJet.setVal(idx + i, 0.0f);

      for (int i = first; i <= last; ++i)
        {
          const float pix =
            itsDoInterp
            ? getPyrPixel(chan.pyramid(0), itsLoc, i)
            : getPyrPixelNI(chan.pyramid(0), itsLoc, i);
          itsJet.setVal(idx + i, pix);
        }
      for (int i = ns + 1; i <= rmax; i ++)
        itsJet.setVal(idx + i, 0.0f);
      itsIndex.pop_back();
    }

  //FIXME: also do it for RAW_CS and NORM_CS
}

// ######################################################################
void JetFiller::visitComplexChannel(ComplexChannel& chan)
{
  if (chan.isHomogeneous())
    {
      itsIndex.push_back(0); // add one index for the subchannels
      for (uint i = 0; i < chan.numChans(); ++i)
        {
          itsIndex.back() = int(i);
          chan.subChan(i)->accept(*this);
        }
      itsIndex.pop_back(); // restore itsIndex for parent to use again
    }
  else
    for (uint i = 0; i < chan.numChans(); ++i)
      chan.subChan(i)->accept(*this);
}

// ######################################################################
JetSpecFiller::JetSpecFiller()
  :
  itsJetSpec(new JetSpec)
{}

// ######################################################################
JetSpecFiller::~JetSpecFiller()
{}

// ######################################################################
void JetSpecFiller::visitChannelBase(ChannelBase& chan)
{
  LFATAL("don't know how to handle %s", chan.tagName().c_str());
}

// ######################################################################
void JetSpecFiller::visitSingleChannel(SingleChannel& chan)
{
  const LevelSpec ls = chan.getLevelSpec();

  itsJetSpec->addIndexRange(chan.visualFeature(), RAW, 0,
                            ls.levMax() + ls.delMax());
  /*  FIXME: FUTURE...
  itsJetSpec->addIndexRange(chan.visualFeature(), RAW_CS,
                   ls.levMin(),
                   ls.levMax());
  itsJetSpec->addIndexRange(chan.visualFeature(), RAW_CS,
                   ls.delMin(),
                   ls.delMax());
  itsJetSpec->addIndexRange(chan.visualFeature(), NORM_CS,
                   ls.levMin(),
                   ls.levMax());
  itsJetSpec->addIndexRange(chan.visualFeature(), NORM_CS,
                   ls.delMin(),
                   ls.delMax());
  */
}

// ######################################################################
void JetSpecFiller::visitComplexChannel(ComplexChannel& chan)
{
  if (chan.numChans() > 0)
    {
      // are we a homogeneous complex channel? If so, add a range for
      // number of subchannels and initialize rest from first
      // subchannel
      if (chan.isHomogeneous())
        {
          // create ranges for the number of subchannels we hold:
          itsJetSpec->addIndexRange(chan.visualFeature(), RAW, 0, chan.numChans() - 1);
          //itsJetSpec->addIndexRange(chan.visualFeature(), RAW_CS, 0, chan.numChans() - 1);
          //itsJetSpec->addIndexRange(chan.visualFeature(), NORM_CS, 0, chan.numChans() - 1);

          // use the first subchannel to add the range specs for us:
          chan.subChan(0)->accept(*this);
        }
      else
        {
          // We are heterogeneous; each subchannel holds a different
          // VisualFeature; initialize all subchannels in turn, and we
          // do not have to do anything for us proper:
          for (uint i = 0; i < chan.numChans(); ++i)
            chan.subChan(i)->accept(*this);
        }
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_JETFILLER_C_DEFINED
