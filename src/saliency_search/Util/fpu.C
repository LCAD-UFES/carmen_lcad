/*!@file Util/fpu.C manipulate floating-point unit (FPU) settings */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/fpu.C $
// $Id: fpu.C 11208 2009-05-20 02:03:21Z itti $
//

#ifndef UTIL_FPU_C_DEFINED
#define UTIL_FPU_C_DEFINED

#include "Util/fpu.H"

#include "Util/StringConversions.H"
#include "Util/log.H"

#include <cstdio>

namespace
{
#ifdef INVT_CPU_IX86

  const unsigned short infmask = (1<<12);
  const unsigned short roundmask = ((1<<11)|(1<<10));
  const unsigned short precmask = ((1<<9)|(1<<8));
  const unsigned short exceptmask = 0xff;

  /* Here's a table describing the 16-bit x87 floating-point control
     word register (from
     http://www.delorie.com/djgpp/doc/libc/libc_112.html)

     ---- ---- --XX XXXX = MCW_EM - exception masks (1=handle exception
                                                     internally, 0=fault)
     ---- ---- ---- ---X = EM_INVALID - invalid operation
     ---- ---- ---- --X- = EM_DENORMAL - denormal operand
     ---- ---- ---- -X-- = EM_ZERODIVIDE - divide by zero
     ---- ---- ---- X--- = EM_OVERFLOW - overflow
     ---- ---- ---X ---- = EM_UNDERFLOW - underflow
     ---- ---- --X- ---- = EM_INEXACT - rounding was required
     ---- --XX ---- ---- = MCW_PC - precision control
     ---- --00 ---- ---- = PC_24 - single precision
     ---- --10 ---- ---- = PC_53 - double precision
     ---- --11 ---- ---- = PC_64 - extended precision
     ---- XX-- ---- ---- = MCW_RC - rounding control
     ---- 00-- ---- ---- = RC_NEAR - round to nearest
     ---- 01-- ---- ---- = RC_DOWN - round towards -Inf
     ---- 10-- ---- ---- = RC_UP - round towards +Inf
     ---- 11-- ---- ---- = RC_CHOP - round towards zero
     ---X ---- ---- ---- = MCW_IC - infinity control (obsolete,
                                                      always affine)
     ---0 ---- ---- ---- = IC_AFFINE - -Inf < +Inf
     ---1 ---- ---- ---- = IC_PROJECTIVE - -Inf == +Inf

   */

  unsigned short get_x87_control_word()
  {
    volatile unsigned short ctrl;

    asm("fstcw %0"
        :"=m"(ctrl));

    return ctrl;
  }

  void set_x87_control_word(volatile unsigned short ctrl)
  {
    asm("fldcw %0\n\t"
        "fwait"
        :
        :"m"(ctrl));
  }

  void show_x87_control_word(unsigned short ctrl)
  {
    printf("control word:               0x%04x\n", int(ctrl));
    printf("     infinity-control bits: 0x%04x\n", int(ctrl & infmask));
    printf("     rounding-control bits: 0x%04x\n", int(ctrl & roundmask));
    printf("    precision-control bits: 0x%04x\n", int(ctrl & precmask));
    printf("           interrupt masks: 0x%04x\n", int(ctrl & exceptmask));
  }
#endif
}

// ######################################################################
FpuPrecision getFpuPrecision()
{
#ifdef INVT_CPU_IX86
  const unsigned short ctrl = get_x87_control_word();
  const unsigned short precbits = (ctrl & ((1<<9)|(1<<8))) >> 8;
  switch (precbits)
    {
    case 0: return FPU_PREC_SINGLE; // 00
    case 2: return FPU_PREC_DOUBLE; // 10
    case 3: return FPU_PREC_EXTENDED; // 11
    }
  LINFO("bogus x87 precision control bits '%d'", precbits);
  return FPU_PREC_EXTENDED;
#else
  LDEBUG("warning: x87 control word not available; "
         "assuming extended precision");
  return FPU_PREC_EXTENDED;
#endif
}

// ######################################################################
void setFpuPrecision(FpuPrecision prec)
{
#ifdef INVT_CPU_IX86
  volatile unsigned short ctrl = get_x87_control_word();
  switch (prec)
    {
    case FPU_PREC_SINGLE:
      ctrl &= (~(1<<8));
      ctrl &= (~(1<<9));
      set_x87_control_word(ctrl);
      break;
    case FPU_PREC_DOUBLE:
      ctrl &= (~(1<<8));
      ctrl |=   (1<<9);
      set_x87_control_word(ctrl);
      break;
    case FPU_PREC_EXTENDED:
      ctrl |=   (1<<8);
      ctrl |=   (1<<9);
      set_x87_control_word(ctrl);
      break;
    }
#else
  if (prec != FPU_PREC_EXTENDED)
    LERROR("warning: x87 control word not available; "
           "attempt to set precision to non-default value '%s' ignored",
           convertToString(prec).c_str());
#endif
}

// ######################################################################
std::string convertToString(const FpuPrecision& val)
{
  switch (val)
    {
    case FPU_PREC_SINGLE: return "single";
    case FPU_PREC_DOUBLE: return "double";
    case FPU_PREC_EXTENDED: return "extended";
    }
  LFATAL("unknown FpuPrecision enumerant '%d'", int(val));
  /* can't happen */ return std::string();
}

// ######################################################################
void convertFromString(const std::string& str, FpuPrecision& val)
{
  if      (str.compare("single") == 0)   { val = FPU_PREC_SINGLE; }
  else if (str.compare("double") == 0)   { val = FPU_PREC_DOUBLE; }
  else if (str.compare("extended") == 0) { val = FPU_PREC_EXTENDED; }
  else
    conversion_error::raise<FpuPrecision>(str);
}

// ######################################################################
FpuRoundingMode getFpuRoundingMode()
{
#ifdef INVT_CPU_IX86
  const unsigned short ctrl = get_x87_control_word();
  const unsigned short roundbits = (ctrl & ((1<<11)|(1<<10))) >> 10;
  switch (roundbits)
    {
    case 0: return FPU_ROUND_NEAR; // 00
    case 1: return FPU_ROUND_DOWN; // 01
    case 2: return FPU_ROUND_UP;   // 10
    case 3: return FPU_ROUND_ZERO; // 11
    }
  LINFO("bogus x87 precision control bits '%d'", roundbits);
  return FPU_ROUND_NEAR;
#else
  LDEBUG("warning: x87 control word not available; "
         "assuming round-nearest");
  return FPU_ROUND_NEAR;
#endif
}

// ######################################################################
void setFpuRoundingMode(FpuRoundingMode roundmode)
{
#ifdef INVT_CPU_IX86
  volatile unsigned short ctrl = get_x87_control_word();
  switch (roundmode)
    {
    case FPU_ROUND_NEAR: // 00
      ctrl &= (~(1<<11));
      ctrl &= (~(1<<10));
      set_x87_control_word(ctrl);
      break;
    case FPU_ROUND_DOWN: // 01
      ctrl &= (~(1<<11));
      ctrl |=   (1<<10);
      set_x87_control_word(ctrl);
      break;
    case FPU_ROUND_UP:   // 10
      ctrl |=   (1<<11);
      ctrl &= (~(1<<10));
      set_x87_control_word(ctrl);
      break;
    case FPU_ROUND_ZERO: // 11
      ctrl |=   (1<<11);
      ctrl |=   (1<<10);
      set_x87_control_word(ctrl);
      break;
    }
#else
  if (roundmode != FPU_ROUND_NEAR)
    LERROR("warning: x87 control word not available; "
           "attempt to set rounding mode to non-default value '%s' ignored",
           convertToString(roundmode).c_str());
#endif
}

// ######################################################################
std::string convertToString(const FpuRoundingMode& val)
{
  switch (val)
    {
    case FPU_ROUND_NEAR: return "nearest";
    case FPU_ROUND_DOWN: return "down";
    case FPU_ROUND_UP:   return "up";
    case FPU_ROUND_ZERO: return "zero";
    }
  LFATAL("unknown FpuRoundingMode enumerant '%d'", int(val));
  /* can't happen */ return std::string();
}

// ######################################################################
void convertFromString(const std::string& str, FpuRoundingMode& val)
{
  if      (str.compare("nearest") == 0) { val = FPU_ROUND_NEAR; }
  else if (str.compare("down") == 0)    { val = FPU_ROUND_DOWN; }
  else if (str.compare("up") == 0)      { val = FPU_ROUND_UP; }
  else if (str.compare("zero") == 0)    { val = FPU_ROUND_ZERO; }
  else
    conversion_error::raise<FpuRoundingMode>(str);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // UTIL_FPU_C_DEFINED
