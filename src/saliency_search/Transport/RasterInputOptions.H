/*!@file Transport/RasterInputOptions.H Helper class to expose command-line options for various low-level parameters controlling raster file input */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/RasterInputOptions.H $
// $Id: RasterInputOptions.H 9228 2008-02-07 01:10:48Z rjpeters $
//

#ifndef TRANSPORT_RASTERINPUTOPTIONS_H_DEFINED
#define TRANSPORT_RASTERINPUTOPTIONS_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Image/Dims.H"

/// Helper class to expose command-line options for various low-level parameters controlling raster file input
/** Users don't normally need to use this class directly; it is part
    of the implementation of RasterInputSeries that allows raster
    files to be read through an InputFrameSeries with the --in option.

    However, for users using the low level Raster functions (like
    Raster::ReadFrame() or Raster::ReadRGB()) directly, instantiating
    a RasterInputOptions in the main() function will allow the
    low-level raster input parameters to be exposed as command-line
    options.
*/
class RasterInputOptions : public ModelComponent
{
public:
  RasterInputOptions(OptionManager& mgr);

  virtual ~RasterInputOptions();

  //! Override the base version so that we can trap --yuv-dims, --yuv-dims-loose, etc.
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

private:
  //! dimensions for YuvParser
  OModelParam<Dims> itsYuvDims;

  //! whether to allow loose dimensions for YuvParser
  OModelParam<bool> itsYuvDimsLoose;

  OModelParam<float> itsDpxGamma;
  OModelParam<float> itsDpxSigmoidContrast;
  OModelParam<float> itsDpxSigmoidThreshold;
  OModelParam<float> itsDpxSrcClipLo;
  OModelParam<float> itsDpxSrcClipHi;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_RASTERINPUTOPTIONS_H_DEFINED
