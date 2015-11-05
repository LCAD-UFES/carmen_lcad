/*!@file Channels/MultiSpectralResidualChannel.C A complex channel holding several spectral residual sub-channels */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/MultiSpectralResidualChannel.C $
// $Id: MultiSpectralResidualChannel.C 15310 2012-06-01 02:29:24Z itti $
//

#ifndef CHANNELS_MULTISPECTRALRESIDUALCHANNEL_C_DEFINED
#define CHANNELS_MULTISPECTRALRESIDUALCHANNEL_C_DEFINED

#include "Channels/MultiSpectralResidualChannel.H"

#include "Channels/ChannelOpts.H"
#include "Channels/SpectralResidualChannel.H"
#include "Component/ModelOptionDef.H"
#include "Image/LowPass.H"
#include "Image/ShapeOps.H"
#include "Util/StringUtil.H"
#include "Util/sformat.H"

#include <iterator>

static const ModelOptionDef OPT_MultiSpectralResidualSizes =
  { MODOPT_ARG_STRING, "MultiSpectralResidualSizes", &MOC_CHANNEL, OPTEXP_CORE,
    "A comma-separated list of dimensions for each of the spectral residual "
    "subchannels in a multi-spectral residual channel",
    "multi-srs-sizes", '\0', "w1xh1,w2xh2,w3xh3,...",
    "64x64,128x128,256x256,512x512" };


// ######################################################################
MultiSpectralResidualChannel::
MultiSpectralResidualChannel(OptionManager& mgr)
  :
  ComplexChannel(mgr, "Multi-Spectral Residual Channel",
                 "MultiSpectralResidualChannel", UNKNOWN),
  itsSizesString(&OPT_MultiSpectralResidualSizes, this),
  itsDownSizeFilterWidth(&OPT_SpectralResidualChannelDownSizeFilterWidth, this),
  itsDoOutputResize(&OPT_SpectralResidualChannelOutputResize, this)
{
  this->buildSubChans();
}

// ######################################################################
MultiSpectralResidualChannel::~MultiSpectralResidualChannel()
{}

// ######################################################################
void MultiSpectralResidualChannel::
paramChanged(ModelParamBase* const param,
             const bool valueChanged,
             ParamClient::ChangeStatus* status)
{
  if (param == &itsSizesString)
    {
      std::vector<std::string> sizestrings;
      split(itsSizesString.getVal(), ",", std::back_inserter(sizestrings));

      if (sizestrings.size() == 0)
        LFATAL("Expected at least one Dims value in --%s, but got \"%s\"",
               itsSizesString.getOptionDef()->longoptname,
               itsSizesString.getVal().c_str());

      itsResizeSpecs.resize(0);
      for (size_t i = 0; i < sizestrings.size(); ++i)
        itsResizeSpecs.push_back(fromStr<ResizeSpec>(sizestrings[i]));

      this->buildSubChans();
    }
}

// ######################################################################
void MultiSpectralResidualChannel::doInput(const InputFrame& inframe)
{
  ASSERT(inframe.grayFloat().initialized());

  for (uint i = 0; i < numChans(); ++i)
    subChan(i)->input(inframe);
}

// ######################################################################
void MultiSpectralResidualChannel::buildSubChans()
{
  this->removeAllSubChans();

  rutz::shared_ptr<SpectralResidualChannel::Downsizer> dx0;

  for (size_t i = 0; i < itsResizeSpecs.size(); ++i)
    {
      nub::ref<SpectralResidualChannel> chan
        (new SpectralResidualChannel
         (this->getManager(),
          sformat("Spectral Residual (%s)",
                  itsResizeSpecs[i].toString().c_str()),
          sformat("srs_%" ZU , i)));

      // make all the channels share the same downsizer
      if (i == 0)
        {
          dx0 = chan->getDownsizer();
          ASSERT(dx0.get() != 0);
        }
      else
        chan->setDownsizer(dx0);

      this->addSubChan(chan);

      chan->exportOptions(MC_RECURSE);

      chan->setResizeSpec(itsResizeSpecs[i]);

      // we need all the subchannels to have the same output-resize
      // spec; this can happen two ways: (1) the user gives
      // --srs-output-resize and --srs-output-resize-spec=<foo> on the
      // command-line, in which case we're all set; or (2) if the user
      // specifies nothing then we force all the subchannels to have
      // an output-resize spec equivalent to chan[0]'s INPUT-resize
      // spec
      if (!itsDoOutputResize.getVal())
        chan->setOutputResizeSpec(true, itsResizeSpecs[0]);

      LINFO("created %s with input resize %s, output blur %f, "
            "output resize %s",
            chan->descriptiveName().c_str(),
            chan->getResizeSpec().toString().c_str(),
            chan->getOutputBlur(),
            convertToString(chan->getOutputResizeSpec()).c_str());
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_MULTISPECTRALRESIDUALCHANNEL_C_DEFINED
