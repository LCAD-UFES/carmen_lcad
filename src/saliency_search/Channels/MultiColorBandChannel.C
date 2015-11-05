/*!@file Channels/MultiColorBandChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/MultiColorBandChannel.C $
// $Id: MultiColorBandChannel.C 8195 2007-03-30 04:34:07Z rjpeters $
//

#ifndef MULTICOLORBANDCHANNEL_C_DEFINED
#define MULTICOLORBANDCHANNEL_C_DEFINED

#include "Channels/MultiColorBandChannel.H"

#include "Channels/ChannelOpts.H"
#include "Channels/ColorBandChannel.H"
#include "Component/OptionManager.H"
#include "Image/Pixels.H"
#include "rutz/trace.h"

#include <vector>

// ######################################################################
// MultiColorBandChannel member definitions:
// ######################################################################

// ######################################################################
MultiColorBandChannel::MultiColorBandChannel(OptionManager& mgr) :
  ComplexChannel(mgr, "MultiColorBand", "multicolorband", COLBAND),
  itsNumBands(&OPT_NumColorBands, this),  // see Channels/ChannelOpts.{H,C}
  itsSatBands(&OPT_NumSatBands, this),  // see Channels/ChannelOpts.{H,C}
  itsHueSigma(&OPT_HueBandWidth, this),  // see Channels/ChannelOpts.{H,C}
  itsSatSigma(&OPT_SatBandWidth, this)  // see Channels/ChannelOpts.{H,C}
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // let's create our subchannels (may be reconfigured if itsNumBands
  // gets changed):
  buildSubChans();
}

// ######################################################################
void MultiColorBandChannel::buildSubChans()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // kill any subchans we may have had...
  this->removeAllSubChans();

  // let's instantiate our subchannels now that we know how many we
  // want. They will inherit the current values (typically
  // post-command-line parsing) of all their options as they are
  // constructed:
  LINFO("Using %d hue bands", itsNumBands.getVal());
  for (uint i = 0; i < itsNumBands.getVal(); i ++)
    {
      nub::ref<ColorBandChannel> chan
        (makeSharedComp(new ColorBandChannel(getManager(), i)));

      this->addSubChan(chan);

      // let's export options on the newly built channel:
      chan->exportOptions(MC_RECURSE);
    }

  LINFO("Using %d saturation bands", itsSatBands.getVal());
  for (uint i = 0; i < itsSatBands.getVal(); i ++)
    {
      nub::ref<ColorBandChannel> chan
        (makeSharedComp(new ColorBandChannel
                        (getManager(), itsNumBands.getVal()+i)));

      this->addSubChan(chan);

      // let's export options on the newly built channel:
      chan->exportOptions(MC_RECURSE);
    }
}

// ######################################################################
void MultiColorBandChannel::paramChanged(ModelParamBase* const param,
                                         const bool valueChanged,
                                         ParamClient::ChangeStatus* status)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ComplexChannel::paramChanged(param, valueChanged, status);

  // if the param is our number of orientations and it has become
  // different from our number of channels, let's reconfigure:
  if (param == &itsNumBands &&
      numChans() != (itsNumBands.getVal() + itsSatBands.getVal()))
    buildSubChans();

  else if (param == &itsSatBands &&
           numChans() != (itsNumBands.getVal() + itsSatBands.getVal()))
    buildSubChans();
}

// ######################################################################
MultiColorBandChannel::~MultiColorBandChannel()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
ColorBandChannel& MultiColorBandChannel::band(const uint idx) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // Since we are dynamic_cast'ing a reference, this operation will either
  // succeed or throw an exception.
  return *(dynCast<ColorBandChannel>(subChan(idx)));
}

// ######################################################################
void MultiColorBandChannel::doInput(const InputFrame& inframe)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(inframe.colorFloat().initialized());
  /*
    in the old implementation, we only supported 3 bands: r, g, b. in
    the new implementation, we allow several bands, i.e., distributed
    coding of color.  ASSERT(numChans() == 3);
  */
  // create bands with different ranges of hues and saturation
  uint numHue = itsNumBands.getVal(), numSat = itsSatBands.getVal();
  std::vector<Image<float> > sub_input(numHue + numSat);
  Image<float>::iterator bptr[sub_input.size()];
  // initialize the pointers
  for (uint i = 0; i < sub_input.size(); ++i) {
    sub_input[i].resize(inframe.getDims(), true);
    bptr[i] = sub_input[i].beginw();
  }
  // some constants for the loop
  float hue_range = 360/numHue; // range of each band
  float c1 = 10000.0f / (sqrt(6.28) * itsHueSigma.getVal());
  float c2 = 2.0f * itsHueSigma.getVal() * itsHueSigma.getVal();
  float c3 = 10000.0f / (sqrt(6.28) * itsSatSigma.getVal());
  float c4 = 2.0f * itsSatSigma.getVal() * itsSatSigma.getVal();
  // get the hue and saturation at all pixels
  Image<PixRGB<float> >::const_iterator aptr = inframe.colorFloat().begin();
  Image<PixRGB<float> >::const_iterator astop = inframe.colorFloat().end();
  while (aptr != astop)
    {
      float h, s, v;
      PixHSV<float>(*aptr).getHSV(h, s, v);
      aptr++;
      // what is the response of each band to this hue?
      for (uint i = 0; i < numHue; i++){
        if (v == 0.0f || s == 0.0f)
          *(bptr[i]) = 0.0f; // response to black and white is zero
        else {
          float mean = i * hue_range;
          float dist = h-mean;
          if (dist > 180) dist = 360 - dist;
          else if (dist < -180) dist = dist + 360;
          *(bptr[i]) = c1 * exp(-1.0f * dist * dist / c2);
        }
        bptr[i] =  bptr[i] + 1;
      }
      // what is the response of each band to this saturation?
      for (uint i = 0; i < numSat; i++){
        if (v == 0.0f)
          *(bptr[numHue+i]) = 0.0f; // response to black is zero
        else {
          float mean = (i+0.5)/numSat;
          float dist = s-mean;
          *(bptr[numHue+i]) = c3 * exp(-1.0f * dist * dist / c4);
        }
        bptr[numHue+i] =  bptr[numHue+i] + 1;
      }
    }
  for (uint i = 0; i < sub_input.size(); ++i)
    subChan(i)->input(InputFrame::fromGrayFloat
                      (&sub_input[i], inframe.time(),
                       &inframe.clipMask(), inframe.pyrCache()));
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // MULTICOLORBANDCHANNEL_C_DEFINED
