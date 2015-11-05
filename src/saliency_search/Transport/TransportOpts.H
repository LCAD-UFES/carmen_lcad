/*!@file Transport/TransportOpts.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/TransportOpts.H $
// $Id: TransportOpts.H 9228 2008-02-07 01:10:48Z rjpeters $
//

#ifndef TRANSPORT_TRANSPORTOPTS_H_DEFINED
#define TRANSPORT_TRANSPORTOPTS_H_DEFINED

struct ModelOptionCateg;
struct ModelOptionDef;

//! Input reading/formatting options
extern const ModelOptionCateg MOC_INPUT;

//! Output writing/formatting options
extern const ModelOptionCateg MOC_OUTPUT;

//! Command-line options for FrameIstream and subclasses
//@{
extern const ModelOptionDef OPT_ShowInputDetails;
//@}

//! Command-line options for RasterInputSeries
//@{
extern const ModelOptionDef OPT_InputRasterFileFormat;
//@}

//! Command-line options for RasterInputOptions
//@{
extern const ModelOptionDef OPT_InputYuvDims;
extern const ModelOptionDef OPT_InputYuvDimsLoose;
extern const ModelOptionDef OPT_InputDpxGamma;
extern const ModelOptionDef OPT_InputDpxSigmoidContrast;
extern const ModelOptionDef OPT_InputDpxSigmoidThreshold;
extern const ModelOptionDef OPT_InputDpxSrcClipLo;
extern const ModelOptionDef OPT_InputDpxSrcClipHi;
//@}

//! Command-line options for RasterOutputSeries
//@{
extern const ModelOptionDef OPT_OutputRasterFileFormat;
//@}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_TRANSPORTOPTS_H_DEFINED
