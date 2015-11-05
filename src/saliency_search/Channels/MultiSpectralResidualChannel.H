/*!@file Channels/MultiSpectralResidualChannel.H A complex channel holding several spectral residual sub-channels */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/MultiSpectralResidualChannel.H $
// $Id: MultiSpectralResidualChannel.H 9356 2008-03-01 01:25:05Z rjpeters $
//

#ifndef CHANNELS_MULTISPECTRALRESIDUALCHANNEL_H_DEFINED
#define CHANNELS_MULTISPECTRALRESIDUALCHANNEL_H_DEFINED

#include "Channels/ComplexChannel.H"
#include "Component/ModelParam.H"
#include "Image/Dims.H"
#include "Image/ResizeSpec.H"

#include <vector>

class SpectralResidualChannel;

/// A complex channel holding several spectral residual sub-channels
class MultiSpectralResidualChannel : public ComplexChannel
{
public:
  MultiSpectralResidualChannel(OptionManager& mgr);

  virtual ~MultiSpectralResidualChannel();

  //! Override so that we can reconfigure when our params get changed
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

protected:
  OModelParam<std::string> itsSizesString;
  OModelParam<int> itsDownSizeFilterWidth;
  OModelParam<bool> itsDoOutputResize;

  //! MultiSpectralResidualChannel requires only luminance input.
  virtual void doInput(const InputFrame& inframe);

private:
  void buildSubChans();

  std::vector<ResizeSpec> itsResizeSpecs;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_MULTISPECTRALRESIDUALCHANNEL_H_DEFINED
