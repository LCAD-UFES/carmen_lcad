/*!@file Transport/TransportOpts.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/TransportOpts.C $
// $Id: TransportOpts.C 15290 2012-05-13 14:06:48Z kai $
//

#ifndef TRANSPORT_TRANSPORTOPTS_C_DEFINED
#define TRANSPORT_TRANSPORTOPTS_C_DEFINED

#include "Transport/TransportOpts.H"

#include "Component/ModelOptionDef.H"
#include "Image/Dims.H"
#include "Raster/RasterFileFormat.H"

const ModelOptionCateg MOC_INPUT = {
  MOC_SORTPRI_2, "Input Reading/Formatting Options" };

const ModelOptionCateg MOC_OUTPUT = {
  MOC_SORTPRI_2, "Output Writing/Formatting Options" };

const ModelOptionDef OPT_ShowInputDetails =
  { MODOPT_FLAG, "ShowInputDetails", &MOC_INPUT, OPTEXP_CORE,
    "Request that detailed information be printed about the input source "
    "(for example, if input is coming from a camera, then try to print "
    "details about the capabilities of that camera, its vendor and model, "
    ".etc)",
    "show-input-details", '\0', "", "false" };

// Used by: RasterInputSeries
const ModelOptionDef OPT_InputRasterFileFormat =
  { MODOPT_ARG(RasterFileFormat), "InputRasterFileFormat", &MOC_INPUT, OPTEXP_CORE,
    "Input file format",
    "input-format", '\0', "<PNM|PNG|PFM|YUV422|YUV420P|RAWIMAGE|JPEG|Auto>", "Auto" };

// Used by: RasterInputOptions (on behalf of YuvParser)
const ModelOptionDef OPT_InputYuvDims =
  { MODOPT_ARG(Dims), "InputYuvDims", &MOC_INPUT, OPTEXP_CORE,
    "NOTE: This option is DEPRECATED; see the documentation of "
    "--in=raster and --out=rawvideo for the preferred approach, in "
    "which image dimensions are encoded into the filename. This "
    "option is a deprecated approach to specifying the dimensions of "
    "raw video frames. The dimensions must be supplied externally as "
    "raw video frame files do not encode the dimensions in the file "
    "internally. Note that this option does not rescale the input "
    "frames at all, it just supplies the native size of the input "
    "frames (for rescaling, use --rescale-input).",
    "yuv-dims", '\0', "<w>x<h>", "640x480" };

// Used by: RasterInputOptions (on behalf of YuvParser)
const ModelOptionDef OPT_InputYuvDimsLoose =
  { MODOPT_FLAG, "InputYuvDimsLoose", &MOC_INPUT, OPTEXP_CORE,
    "Allow loose correspondence between the dimensions of raw yuv "
    "video frames and the corresponding file size. Normally it is a "
    "hard error if these values mismatch, but this option can be "
    "used to allow file sizes larger than the expected data size.",
    "yuv-dims-loose", '\0', "", "false" };

// Used by: RasterInputOptions (on behalf of DpxParser)
const ModelOptionDef OPT_InputDpxGamma =
  { MODOPT_ARG(float), "InputDpxGamma", &MOC_INPUT, OPTEXP_CORE,
    "Gamma value to use when performing color correction of dpx images.",
    "dpx-gamma", '\0', "<float>", "0.6" };

// Used by: RasterInputOptions (on behalf of DpxParser)
const ModelOptionDef OPT_InputDpxSigmoidContrast =
  { MODOPT_ARG(float), "InputDpxSigmoidContrast", &MOC_INPUT, OPTEXP_CORE,
    "Sigmoid contrast to use when performing color correction of dpx images.",
    "dpx-sigmoid-contrast", '\0', "<float>", "10.0" };

// Used by: RasterInputOptions (on behalf of DpxParser)
const ModelOptionDef OPT_InputDpxSigmoidThreshold =
  { MODOPT_ARG(float), "InputDpxSigmoidThreshold", &MOC_INPUT, OPTEXP_CORE,
    "Sigmoid threshold to use when performing color correction of dpx images.",
    "dpx-sigmoid-threshold", '\0', "<float>", "0.1" };

// Used by: RasterInputOptions (on behalf of DpxParser)
const ModelOptionDef OPT_InputDpxSrcClipLo =
  { MODOPT_ARG(float), "InputDpxSrcClipLo", &MOC_INPUT, OPTEXP_CORE,
    "Clamp smaller values to this value when performing color "
    "correction of dpx images.",
    "dpx-src-clip-lo", '\0', "<float>", "0.0" };

// Used by: RasterInputOptions (on behalf of DpxParser)
const ModelOptionDef OPT_InputDpxSrcClipHi =
  { MODOPT_ARG(float), "InputDpxSrcClipHi", &MOC_INPUT, OPTEXP_CORE,
    "Clamp larger values to this value when performing color "
    "correction of dpx images.",
    "dpx-src-clip-hi", '\0', "<float>", "5351.0" };

// Used by: RasterOutputSeries
const ModelOptionDef OPT_OutputRasterFileFormat =
  { MODOPT_ARG(RasterFileFormat), "OutputRasterFileFormat", &MOC_OUTPUT, OPTEXP_CORE,
    "Output file format",
    "output-format", '\0', "<PNM|PNG|PFM|YUV422|YUV420P|RAWIMAGE|JPEG|Auto>", "PNM" };

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_TRANSPORTOPTS_C_DEFINED
