/*!@file Util/fpu.H manipulate floating-point unit (FPU) settings */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/fpu.H $
// $Id: fpu.H 5755 2005-10-19 20:57:27Z rjpeters $
//

#ifndef UTIL_FPU_H_DEFINED
#define UTIL_FPU_H_DEFINED

#include <string>




// ######################################################################
// fpu precision

//! Represents different floating-point precision modes
enum FpuPrecision
  {
    FPU_PREC_SINGLE,  //!< fpu operates in single precision (32-bit, 24-bit significand)
    FPU_PREC_DOUBLE,  //!< fpu operates in double precision (64-bit, 53-bit significand)
    FPU_PREC_EXTENDED //!< default: fpu operates in extended precision (80-bit, 64-bit significand)
  };

//! Query the fpu's current precision
FpuPrecision getFpuPrecision();

//! Set the fpu's precision
void setFpuPrecision(FpuPrecision prec);

//! FpuPrecision -> string
std::string convertToString(const FpuPrecision& val);

//! string -> FpuPrecision
void convertFromString(const std::string& str, FpuPrecision& val);




// ######################################################################
// fpu rounding

//! Represents different floating-point rounding modes
enum FpuRoundingMode
  {
    FPU_ROUND_NEAR,  //!< fpu rounds to nearest value
    FPU_ROUND_DOWN,  //!< fpu rounds toward -inf
    FPU_ROUND_UP,    //!< fpu rounds toward +inf
    FPU_ROUND_ZERO   //!< fpu rounds toward zero
  };

//! Query the fpu's current rounding mode
FpuRoundingMode getFpuRoundingMode();

//! Set the fpu's rounding mode
void setFpuRoundingMode(FpuRoundingMode mode);

//! FpuRoundingMode -> string
std::string convertToString(const FpuRoundingMode& val);

//! string -> FpuRoundingMode
void convertFromString(const std::string& str, FpuRoundingMode& val);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // UTIL_FPU_H_DEFINED
