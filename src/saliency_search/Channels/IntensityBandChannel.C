/*!@file Channels/IntensityBandChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/IntensityBandChannel.C $
// $Id: IntensityBandChannel.C 8195 2007-03-30 04:34:07Z rjpeters $
//

#ifndef INTENSITYBANDCHANNEL_C_DEFINED
#define INTENSITYBANDCHANNEL_C_DEFINED

#include "Channels/IntensityBandChannel.H"

#include "Channels/ChannelOpts.H"
#include "Channels/IntensityChannel.H"
#include "Component/OptionManager.H"
#include "Image/Pixels.H"

#include <vector>

// ######################################################################
// IntensityBandChannel member definitions:
// ######################################################################

// ######################################################################
IntensityBandChannel::IntensityBandChannel(OptionManager& mgr) :
  ComplexChannel(mgr, "IntensityBand", "intensityband", INTENS),
  itsNumBands(&OPT_NumIntensityBands,this), // see Channels/ChannelOpts.{H,C}
  itsSigma(&OPT_IntensityBandWidth, this)   // see Channels/ChannelOpts.{H,C}
{
  // let's create our subchannels (may be reconfigured if itsNumBands
  // gets changed):
  buildSubChans();
}

// ######################################################################
void IntensityBandChannel::buildSubChans()
{
  // kill any subchans we may have had...
  this->removeAllSubChans();

  // let's instantiate our subchannels now that we know how many we
  // want. They will inherit the current values (typically
  // post-command-line parsing) of all their options as they are
  // constructed:
  LINFO("Using %d intensity bands", itsNumBands.getVal());
  for (uint i = 0; i < itsNumBands.getVal(); i ++)
    {
      nub::ref<IntensityChannel> chan
        (makeSharedComp(new IntensityChannel(getManager(), i)));

      this->addSubChan(chan);

      // let's export options on the newly built channel:
      chan->exportOptions(MC_RECURSE);
    }
}

// ######################################################################
void IntensityBandChannel::paramChanged(ModelParamBase* const param,
                                        const bool valueChanged,
                                        ParamClient::ChangeStatus* status)
{
  ComplexChannel::paramChanged(param, valueChanged, status);

  // if the param is our number of orientations and it has become
  // different from our number of channels, let's reconfigure:
  if (param == &itsNumBands &&
      numChans() != itsNumBands.getVal())
    buildSubChans();
}

// ######################################################################
IntensityBandChannel::~IntensityBandChannel()
{ }

// ######################################################################
IntensityChannel& IntensityBandChannel::band(const uint idx) const
{
  // Since we are dynamic_cast'ing a reference, this operation will either
  // succeed or throw an exception.
  return *(dynCast<IntensityChannel>(subChan(idx)));
}

// ######################################################################
void IntensityBandChannel::doInput(const InputFrame& inframe)
{
  ASSERT(inframe.grayFloat().initialized());
  // create bands with different ranges of intensity
  std::vector<Image<float> > sub_input(itsNumBands.getVal());
  Image<float>::iterator bptr[sub_input.size()];
  // initialize the pointers
  for (uint i = 0; i < sub_input.size(); ++i) {
    sub_input[i].resize(inframe.getDims(), true);
    bptr[i] = sub_input[i].beginw();
  }
  // setup loop parameters
  float gap = 255 / sub_input.size();
  float c1 = 10000.0f / (sqrt(6.28) * itsSigma.getVal());
  float c2 = 2.0f * itsSigma.getVal() * itsSigma.getVal();
  // get the intensity at all pixels
  Image<float>::const_iterator aptr = inframe.grayFloat().begin();
  Image<float>::const_iterator astop = inframe.grayFloat().end();
  while (aptr != astop)
    {
      float intens = *aptr;
      aptr++;
      // what is the response of each band to this intensity?
      for (uint i = 0; i < sub_input.size(); ++i) {
        /*
        // sigmoid activation function
        // assume that intensity ranges from 0..255
        *(bptr[i]) = 1 / (1 + exp(gap*i - intens));
        */
        // gaussian tuning curve
        float dist = intens - i*gap;
        *(bptr[i]) = c1 * exp(-1.0f * dist * dist / c2);
        bptr[i] =  bptr[i] + 1;
      }
    }
  for (uint i = 0; i < sub_input.size(); ++i)
    subChan(i)->input(InputFrame::fromGrayFloat
                      (&sub_input[i], inframe.time(), &inframe.clipMask(),
                       inframe.pyrCache()));

}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // INTENSITYBANDCHANNEL_C_DEFINED
