/*!@file ModelNeuron/LayerDecoder.C implementaton for a LayerDecoder*/

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
// Primary maintainer for this file: David Berg <dberg@usc.edu>
// $HeadURL:svn://ilab.usc.edu/trunk/saliency/src/ModelNeuron/LayerDecoder.C$

#include "ModelNeuron/LayerDecoder.H"

// ######################################################################
//! implementation for layer decoder
// ######################################################################
nsu::LayerDecoder::LayerDecoder() : itsW(0), itsH(0), itsInitialized(false), itsI() { };

// ######################################################################
nsu::LayerDecoder::LayerDecoder(const NeuralDecoder& nd, uint w, uint h) : 
  itsW(w), itsH(h), itsInitialized(false), itsI() {setDecoder(nd,w,h); };

// ###################################################################### 
nsu::LayerDecoder::LayerDecoder (const NeuralDecoder& nd, const Dims& dims) : 
  itsW(0), itsH(0), itsInitialized(false), itsI() {setDecoder(nd,dims); };

// ######################################################################
nsu::LayerDecoder::~LayerDecoder() { };

// ######################################################################
void nsu::LayerDecoder::push(const Image<double>& data)
{
  Image<double>::const_iterator di(data.begin());
  vector::iterator i(itsI.begin()), end(itsI.end());
  while (i != end)
    (i++)->push(*di++);
}

// ######################################################################
Image<double> nsu::LayerDecoder::getOutput() const
{
  if (initialized())
    {
      Image<double> out(itsW, itsH, NO_INIT);
      Image<double>::iterator oi(out.beginw());
      
      vector::const_iterator i(itsI.begin()), end(itsI.end());
      
      while (i != end)
        (*oi++) = (i++)->getOutput();
      
      return out;
    }
  else 
    return Image<double>();
}

// ######################################################################
void nsu::LayerDecoder::setDecoder(const NeuralDecoder& nd, const uint w, const uint h)
{
  //resize to our new dimensions
  itsW = w; itsH = h;
  itsI = vector(w*h, nd);
  itsInitialized = true;
}

// ######################################################################
void nsu::LayerDecoder::setDecoder(const NeuralDecoder& nd, const Dims dims)
{
  setDecoder(nd, dims.w(), dims.h());
}

// ######################################################################
bool nsu::LayerDecoder::initialized() const
{
  return itsInitialized;
}

// ######################################################################
void nsu::LayerDecoder::reset()
{
  vector::iterator iter(itsI.begin());
  while (iter != itsI.end())
    (iter++)->reset();
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */



