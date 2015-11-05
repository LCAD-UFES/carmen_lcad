/*!@file ModelNeuron/LayerDecoder.H Class declaration for a layer of
   neural decoders */

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
// Primary maintainer for this file: David J Berg <dberg@usc.edu>
// $HeadURL:svn://ilab.usc.edu/trunk/saliency/src/ModelNeuron/LayerDecoder.H$

#ifndef MODELNEURON_LAYERDECODER_H_DEFINED
#define MODELNEURON_LAYERDECODER_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include "Util/SimTime.H"
#include "Image/Image.H"
#include "ModelNeuron/NeuralSimUtils.H"
#include "ModelNeuron/NeuralDecoder.H"

namespace nsu
{
  // ######################################################################
  //! a clas for a layer decoder
  // ######################################################################
  class LayerDecoder
  {
    public:
      //!constructor 
      LayerDecoder();
 
      //!constructor 
      LayerDecoder(const NeuralDecoder& nd, uint w, uint h);
  
      //!constructor 
      LayerDecoder(const NeuralDecoder& nd, const Dims& dims);
  
      //!destructor
      ~LayerDecoder();
  
      //!add some data to the decoder
      void push(const Image<double>& input); 
  
      //!get the latest (and possibly time delayed) output of the decoder
      Image<double> getOutput() const;

      //set the decoder for the layer
      void setDecoder(const NeuralDecoder& nd, const uint w, const uint h);

      //!set the decoder for the layer
      void setDecoder(const NeuralDecoder& nd, const Dims dims);

      //!are we initialized?
      bool initialized() const;

      //reset to state after distruction
      void reset();
  
    private:
      typedef nsu::vector<NeuralDecoder*> vector;
      uint itsW, itsH;
      bool itsInitialized;
      vector itsI;
  };
}
#endif
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */



