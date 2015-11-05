/*!@file Transport/RandomInput.C A FrameIstream subclass for
  generating random images */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/RandomInput.C $
// $Id: RandomInput.C 8602 2007-07-20 23:10:44Z rjpeters $
//

#ifndef TRANSPORT_RANDOMINPUT_C_DEFINED
#define TRANSPORT_RANDOMINPUT_C_DEFINED

#include "Transport/RandomInput.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/GenericFrame.H"

// ######################################################################
RandomInput::RandomInput(OptionManager& mgr)
  :
  FrameIstream(mgr, "Random Input", "RandomInput"),
  itsDims(320,240), // if you change this default value, also update
                    // the documentation of OPT_InputFrameSource in
                    // Media/MediaOpts.C
  itsGenerator(0)
{}

// ######################################################################
RandomInput::~RandomInput()
{}

// ######################################################################
void RandomInput::setConfigInfo(const std::string& dimsstring)
{
  // NOTE: if you modify any behavior here, then please update the
  // corresponding documentation for the global "--in" option inside
  // the OPT_InputFrameSource definition in Media/MediaOpts.C

  if (dimsstring.size() == 0)
    return;

  Dims d; convertFromString(dimsstring, d);
  this->setDims(d);
}

// ######################################################################
GenericFrameSpec RandomInput::peekFrameSpec()
{
  GenericFrameSpec result;

  result.nativeType = GenericFrame::RGB_U8;
  result.videoFormat = VIDFMT_AUTO;
  result.videoByteSwap = false;
  result.dims = itsDims;
  result.floatFlags = 0;

  return result;
}

// ######################################################################
GenericFrame RandomInput::readFrame()
{
  Image<PixRGB<byte> > result(itsDims, NO_INIT);

  for (Image<PixRGB<byte> >::iterator
         itr = result.beginw(), stop = result.endw(); itr != stop; ++itr)
    {
      *itr = PixRGB<byte>(itsGenerator.idraw(256),
                          itsGenerator.idraw(256),
                          itsGenerator.idraw(256));
    }

  return GenericFrame(result);
}

// ######################################################################
void RandomInput::setDims(const Dims& s)
{
  itsDims = s;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_RANDOMINPUT_C_DEFINED
