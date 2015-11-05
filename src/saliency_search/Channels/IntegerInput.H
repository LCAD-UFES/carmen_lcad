/*!@file Channels/IntegerInput.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/IntegerInput.H $
// $Id: IntegerInput.H 8234 2007-04-03 22:51:59Z rjpeters $
//

#ifndef CHANNELS_INTEGERINPUT_H_DEFINED
#define CHANNELS_INTEGERINPUT_H_DEFINED

#include "Channels/IntegerDecodeType.H"
#include "Image/Image.H"
#include "Image/Pixels.H"

class GenericFrame;
class VideoFrame;

class IntegerInput
{
private:
  /// Constructor is private; use fromRgb() or fromVideo() instead
  IntegerInput();

  /// Constructor is private; use fromRgb() or fromVideo() instead
  IntegerInput(const Dims& dims);

public:
  static IntegerInput fromRgb(const Image<PixRGB<byte> >& f,
                              const unsigned int inputbits);

  static IntegerInput fromVideo(const VideoFrame& f,
                                const unsigned int inputbits);

  static IntegerInput decode(const GenericFrame& f,
                             const IntegerDecodeType t,
                             const unsigned int inputbits);

  static IntegerInput fromGrayOnly(const Image<int>& bw);

  bool initialized() const { return itsDims.isNonEmpty(); }

  Dims getDims() const { return itsDims; }

  const Image<int>& grayInt() const { return itsGrayInt; }

  const Image<int>& rgInt() const;

  const Image<int>& byInt() const;

private:
  Dims itsDims;
  Image<PixRGB<byte> > itsRgbByte;
  uint itsInputBits;
  Image<int> itsGrayInt;
  mutable Image<int> itsRgInt;
  mutable Image<int> itsByInt;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_INTEGERINPUT_H_DEFINED
