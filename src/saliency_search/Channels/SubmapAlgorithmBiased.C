/*!@file Channels/SubmapAlgorithmBiased.C Compute SingleChannel submaps with Bayesian biasing */

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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/SubmapAlgorithmBiased.C $
// $Id: SubmapAlgorithmBiased.C 9714 2008-04-29 06:48:56Z itti $
//

#ifndef CHANNELS_SUBMAPALGORITHMBIASED_C_DEFINED
#define CHANNELS_SUBMAPALGORITHMBIASED_C_DEFINED

#include "Channels/SubmapAlgorithmBiased.H"

#include "Channels/ChannelVisitor.H"
#include "Channels/ComplexChannel.H"
#include "Channels/SingleChannel.H"
#include "Image/Image.H"
#include "Image/ShapeOps.H"

#include "GUI/DebugWin.H"
namespace
{
  class Installer : public ChannelVisitor
  {
    OptionManager& itsMgr;

  public:
    Installer(OptionManager& mgr) : itsMgr(mgr) {}

    virtual ~Installer() {}

    virtual void visitChannelBase(ChannelBase& chan)
    {
      // nothing to do here
    }

    virtual void visitSingleChannel(SingleChannel& chan)
    {
      nub::ref<SubmapAlgorithmBiased> algo
        (new SubmapAlgorithmBiased(itsMgr));

      chan.setSubmapAlgorithm(algo);
    }

    virtual void visitComplexChannel(ComplexChannel& chan)
    {
      for (uint i = 0; i < chan.numChans(); ++i)
        chan.subChan(i)->accept(*this);
    }
  };
}

// ######################################################################
SubmapAlgorithmBiased::SubmapAlgorithmBiased(OptionManager& mgr,
                                             const std::string& descrName,
                                             const std::string& tagName)
  :
  SubmapAlgorithm(mgr, descrName, tagName)
{}

// ######################################################################
SubmapAlgorithmBiased::~SubmapAlgorithmBiased()
{}

// ######################################################################
Image<float> SubmapAlgorithmBiased::compute(const SingleChannel& chan,
                                            const uint i)
{
  Image<float> submap = chan.getRawCSmap(i);

  // resize submap to fixed scale if necessary:
  if (submap.getWidth() > chan.getMapDims().w())
    submap = downSize(submap, chan.getMapDims());
  else if (submap.getWidth() < chan.getMapDims().w())
    submap = rescale(submap, chan.getMapDims());

  LFATAL("This is being reworked... stay tuned");

  /*
  // bias the submap if we have a mean and sigma
  LINFO("Mean %f var %f", chan.getMean(i), chan.getSigmaSq(i));
  if (chan.getMean(i) > 0 && chan.getSigmaSq(i) > 0)
  {
    double mean = chan.getMean(i);
    double var = chan.getSigmaSq(i);

    for(int y=0; y<submap.getHeight(); y++)
      for(int x=0; x<submap.getWidth(); x++)
      {
        double val = submap.getVal(x, y);
        double delta = -(val - mean) * (val - mean);
        //Calc the normal dist
        double newVal = exp(delta/(2*var))/(sqrt(2*M_PI*var));

        // submap.setVal(x, y, newVal*10000);
        submap.setVal(x, y, log(newVal)+10000);
      }
  }

  Image<float> biasMask = chan.getBiasMask();
  if (biasMask.initialized()) //bias based on a mask
  {
    //rescale the mask to the submap scale TODO: can be done more effiently
    biasMask = rescale(biasMask, submap.getDims());
    submap *= biasMask;
  }
  // now do the standard processing
  submap = chan.postProcessMap(submap, i);

*/
  return submap;
}

// ######################################################################
void setSubmapAlgorithmBiased(ChannelBase& chan)
{
  Installer inst(chan.getManager());
  chan.accept(inst);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_SUBMAPALGORITHMBIASED_C_DEFINED
