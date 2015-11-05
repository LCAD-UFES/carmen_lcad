/*!@file Transport/RasterInputOptions.C Helper class to expose command-line options for various low-level parameters controlling raster file input */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/RasterInputOptions.C $
// $Id: RasterInputOptions.C 9228 2008-02-07 01:10:48Z rjpeters $
//

#ifndef TRANSPORT_RASTERINPUTOPTIONS_C_DEFINED
#define TRANSPORT_RASTERINPUTOPTIONS_C_DEFINED

#include "Transport/RasterInputOptions.H"

#include "Raster/DpxParser.H"
#include "Raster/YuvParser.H"
#include "Transport/TransportOpts.H"

// ######################################################################
RasterInputOptions::RasterInputOptions(OptionManager& mgr)
  :
  ModelComponent(mgr, "Raster Input Options", "RasterInputOptions"),
  itsYuvDims(&OPT_InputYuvDims, this),
  itsYuvDimsLoose(&OPT_InputYuvDimsLoose, this),
  itsDpxGamma(&OPT_InputDpxGamma, this),
  itsDpxSigmoidContrast(&OPT_InputDpxSigmoidContrast, this),
  itsDpxSigmoidThreshold(&OPT_InputDpxSigmoidThreshold, this),
  itsDpxSrcClipLo(&OPT_InputDpxSrcClipLo, this),
  itsDpxSrcClipHi(&OPT_InputDpxSrcClipHi, this)
{}

// ######################################################################
RasterInputOptions::~RasterInputOptions()
{}

// ######################################################################
void RasterInputOptions::paramChanged(ModelParamBase* const param,
                                      const bool valueChanged,
                                      ParamClient::ChangeStatus* status)
{
  if (param == &itsYuvDims)
    {
      if (itsYuvDims.getVal() != YuvParser::getDefaultDims())
        {
          LINFO("switching default yuv dims to %dx%d",
                itsYuvDims.getVal().w(), itsYuvDims.getVal().h());
          YuvParser::setDefaultDims(itsYuvDims.getVal());
        }
    }
  else if (param == &itsYuvDimsLoose)
    {
      YuvParser::setStrictDims(!itsYuvDimsLoose.getVal());
    }
  else if (param == &itsDpxGamma)
    {
      DpxParser::setDefaultGamma(itsDpxGamma.getVal());
    }
  else if (param == &itsDpxSigmoidContrast)
    {
      DpxParser::setDefaultSigmoidContrast(itsDpxSigmoidContrast.getVal());
    }
  else if (param == &itsDpxSigmoidThreshold)
    {
      DpxParser::setDefaultSigmoidThreshold(itsDpxSigmoidThreshold.getVal());
    }
  else if (param == &itsDpxSrcClipLo)
    {
      DpxParser::setDefaultSrcClipLo(itsDpxSrcClipLo.getVal());
    }
  else if (param == &itsDpxSrcClipHi)
    {
      DpxParser::setDefaultSrcClipHi(itsDpxSrcClipHi.getVal());
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_RASTERINPUTOPTIONS_C_DEFINED
