/*!@file Util/Types.H  Basic integer types */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/Types.H $
// $Id: Types.H 12782 2010-02-05 22:14:30Z irock $
//

#ifndef TYPES_H_DEFINED
#define TYPES_H_DEFINED

// ######################################################################
// type definitions

//! Makes a <i>type</i> a typedef for \a T only if \a B is true.
template <class T, bool B>
struct type_if {};

template <class T>
struct type_if<T, true> { typedef T type; };

//! A compile-time check that \a T has \a N bits.
template <class T, unsigned long N>
struct type_with_N_bits
{
  typedef typename type_if<T, sizeof(T)*8 == N>::type type;
};

// Now use the type_if and type_with_N_bits helper structs to make some
// fixed-size integer typedef's. If the stated bit-size does not match the
// given type, then we get a compile-time error.

// if some other include file (e.g., from opencv, xclib, etc) defines
// some of the types below, just include those other files first, and
// then apply the appropriate #define below before including any of
// our INVT includes:

#ifndef INVT_TYPEDEF_BYTE
#define INVT_TYPEDEF_BYTE
//! 8-bit unsigned integer
typedef type_with_N_bits<unsigned char, 8>::type byte;
#endif

#ifndef INVT_TYPEDEF_INT16
#define INVT_TYPEDEF_INT16
//! 16-bit signed integer
typedef type_with_N_bits<short, 16>::type int16;
#endif

#ifndef INVT_TYPEDEF_UINT16
#define INVT_TYPEDEF_UINT16
//! 16-bit unsigned integer
typedef type_with_N_bits<unsigned short, 16>::type uint16;
#endif

#ifndef INVT_TYPEDEF_INT32
#define INVT_TYPEDEF_INT32
//! 32-bit signed integer
typedef type_with_N_bits<int, 32>::type int32;
#endif

#ifndef INVT_TYPEDEF_UINT32
#define INVT_TYPEDEF_UINT32
//! 32-bit unsigned integer
typedef type_with_N_bits<unsigned int, 32>::type uint32;
#endif

#ifndef INVT_TYPEDEF_INT64
#define INVT_TYPEDEF_INT64
//! 64-bit signed integer
typedef type_with_N_bits<long long int, 64>::type int64;
#endif

#ifndef INVT_TYPEDEF_UINT64
#define INVT_TYPEDEF_UINT64
//! 64-bit unsigned integer
typedef type_with_N_bits<unsigned long long int, 64>::type uint64;
#endif

#ifndef INVT_TYPEDEF_USHORT
#define INVT_TYPEDEF_USHORT
//! Canonical unsigned short int
typedef unsigned short int ushort;
#endif

#ifndef INVT_TYPEDEF_UINT
#define INVT_TYPEDEF_UINT
//! Canonical unsigned int
typedef unsigned int uint;
#endif

#ifndef INVT_TYPEDEF_ULONG
#define INVT_TYPEDEF_ULONG
//! Canonical unsigned long int
typedef unsigned long int ulong;
#endif

#endif // !TYPES_H_DEFINED
