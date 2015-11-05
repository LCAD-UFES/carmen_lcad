/*!@file Neuro/NeuralSimModule.H interface to neural simulaiton code */

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
// Primary maintainer for this file: David J, Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu:/software/invt/trunk/saliency/src/Neuro/NeuralSimModule.H $

#ifndef NEURALSIMMODULE_H_DEFINED
#define NEURALSIMMODULE_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Image/Image.H"
#include "Image/ImageSet.H"
#include "Image/Range.H"
#include "Util/SimTime.H"

#include "ModelNeuron/StructurePlot.H"

//#include "ModelNeuron/NeuralFieldSC.H"
//#include "ModelNeuron/SCTracker.H"


// ######################################################################
//!A SimModule interface to various models created with the ModelNeuron 
// tools. The correct model is chosen from the factory by the tagName, 
// so someplace early on, like in the constructor or start1, set the 
// tag name of the component to the name of the model. 
// ######################################################################
template <class T>
class NeuralSimModule : public ModelComponent
{
public:
  //! Uninitialized constructor
  NeuralSimModule(OptionManager& mgr, 
                  const std::string& descrName = "NeuralSimModule",
                  const std::string& tagName = "NeuralSimModule");
  
  //! Destructor
  virtual ~NeuralSimModule();
  
  //! get the number of input/output layers
  uint size() const;
  
  //! set a new input, -1 for default
  void setInput(const Image<float>& in, const int layer);

  //! set a gain for the desired input
  void setInputGain(const float& weight, const int layer);
  
  //! get the output, -1 for default
  Image<float> getV(const int layer) const;
  
  //! get the output of all the layers
  ImageSet<float> getSubV() const;
  
  //! update our internals to the current time
  void update(const SimTime& time);
  
  //get the display
  Layout<PixRGB<byte> > getDisplay() const;
  
  //! Reset to initial state just after construction
  void reset();
  
  //get the structure
  rutz::shared_ptr<T> getStructure();
  
protected:
  OModelParam<SimTime> itsSCtimestep;//!< simulation timestep
  OModelParam<Dims> itsSCdims;//!< SC dimensions
  NModelParam<bool> itsUseSpaceVariantBoundary; //!< use space variant boundary
  OModelParam<std::string> itsDecoderType; //!< type of decoder 
  
  //display options
  OModelParam<nsu::Location> itsProbe;
  OModelParam<uint> itsPlotLength;
  OModelParam<uint> its2DPlotDepth;
  OModelParam<uint> itsProbeDepth;
  OModelParam<bool> itsDisplayOutput;
  OModelParam<Range<double> > itsDisplayRange;
  OModelParam<Dims> its2DPlotSize;
    
  //our structure and plot organizer
  rutz::shared_ptr<T> itsStructure;   
  rutz::shared_ptr<nsu::StructurePlot> itsPlot; //for saving output

  //our last inputs
  ImageSet<double> itsInput;
  std::vector<double> itsInputGain;
  Dims itsInputDims;

  //construct our model
  virtual void setModel(const std::string& model, const Dims& dims, const SimTime& starttime = SimTime::ZERO());
};

#endif
#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
