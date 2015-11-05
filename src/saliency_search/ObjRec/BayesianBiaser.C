/*!@file ObjRec/BayesianBiaser.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ObjRec/BayesianBiaser.C $
// $Id: BayesianBiaser.C 10794 2009-02-08 06:21:09Z itti $
//

#ifndef OBJREC_BAYESIANBIASER_C_DEFINED
#define OBJREC_BAYESIANBIASER_C_DEFINED

#include "ObjRec/BayesianBiaser.H"

#include "Channels/ComplexChannel.H"
#include "Channels/SingleChannel.H"

// ######################################################################
BayesianBiaser::BayesianBiaser(Bayes& b,
                               const int target_class_id,
                               const int distractor_class_id,
                               const bool dobias)
  :
  itsBayesNetwork(b),
  itsClassT(target_class_id),
  itsClassD(distractor_class_id),
  itsDoBias(dobias),
  itsIndex(0)
{}

// ######################################################################
BayesianBiaser::~BayesianBiaser() {}

// ######################################################################
void BayesianBiaser::visitChannelBase(ChannelBase& chan)
{
  LFATAL("don't know how to handle %s", chan.tagName().c_str());
}

// ######################################################################
void BayesianBiaser::visitSingleChannel(SingleChannel& chan)
{

  LFATAL("This is being reworked...");
  /*
  for (uint i = 0; i < chan.numSubmaps(); ++i)
    {
      if (itsDoBias)
        {
          if (itsClassT != -1)
            {
              const double mean =
                itsBayesNetwork.getMean(itsClassT, itsIndex);
              const double stdevSq =
                itsBayesNetwork.getStdevSq(itsClassT, itsIndex);

              chan.setMean(i, mean);
              chan.setSigmaSq(i, stdevSq);
            }
          if (itsClassD != -1)
            {
              const double mean =
                itsBayesNetwork.getMean(itsClassD, itsIndex);
              const double stdevSq =
                itsBayesNetwork.getStdevSq(itsClassD, itsIndex);

              chan.setDMean(i, mean);
              chan.setDSigmaSq(i, stdevSq);
            }
        }
      else
        {
          chan.setMean(i, 0.0F);
          chan.setSigmaSq(i, 0.0F);
        }

      ++itsIndex;
    }
  */
  //Install the submapAlg
  nub::ref<SubmapAlgorithmBiased>
    algo(new SubmapAlgorithmBiased(chan.getManager()));
  chan.setSubmapAlgorithm(algo);

}

// ######################################################################
void BayesianBiaser::visitComplexChannel(ComplexChannel& chan)
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

#endif // OBJREC_BAYESIANBIASER_C_DEFINED
