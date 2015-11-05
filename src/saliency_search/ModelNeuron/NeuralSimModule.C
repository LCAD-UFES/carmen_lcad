/*!@file ModelNeuron/NeuralSimModule.C Implementation for the superior colliculus model */

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
// Primary maintainer for this file: David J. Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu:/software/invt/trunk/saliency/src/ModelNeuron/NeuralSimModule.C $

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include "Component/OptionManager.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameOstream.H"
#include "Simulation/SimEventQueue.H"
#include "Util/log.H"

#include "ModelNeuron/NeuralSimModule.H"
#include "ModelNeuron/SimStructureOpts.H"
#include "ModelNeuron/NeuralDecoder.H"

// ######################################################################
// an interface to components build from the ModelNeuron tools
// ######################################################################
template <class T>
NeuralSimModule<T>::NeuralSimModule(OptionManager& mgr,
                                    const std::string& descrName,
                                    const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName), 
  itsSCtimestep(&OPT_SCSimTimeStep, this),
  itsSCdims(&OPT_SCDims, this),
  itsUseSpaceVariantBoundary("UseSpaceVariantBoundary", this, false),
  itsDecoderType(&OPT_SCDisplayDecoder, this),
  itsProbe(&OPT_SCProbe,this),
  itsPlotLength(&OPT_SCPlotLength,this),
  its2DPlotDepth(&OPT_SC2DPlotDepth,this),
  itsProbeDepth(&OPT_SCProbeDepth,this),
  itsDisplayOutput(&OPT_SCUseDisplayOutput,this),
  itsDisplayRange(&OPT_SCDisplayRange,this),
  its2DPlotSize(&OPT_SC2DPlotSize,this),
  itsStructure(), itsPlot(), itsInput(), itsInputGain(), itsInputDims()
{ }

// ######################################################################
template <class T>
NeuralSimModule<T>::~NeuralSimModule()
{ }

// ######################################################################
template <class T>
uint NeuralSimModule<T>::size() const
{
  return itsInput.size() - 1;
}

// ######################################################################
template <class T>
void NeuralSimModule<T>::setInput(const Image<float>& current, const int layer)
{ 
  Image<float> inp = current;
  
  //reset module if its currently not valid
  if (!itsStructure.is_valid())
    {
      if (itsSCdims.getVal() != Dims(0,0))
        setModel(tagName(), itsSCdims.getVal());  
      else
        setModel(tagName(), current.getDims());  
      
      itsInputDims = current.getDims();
    }

  //rescale input if needed
  inp = rescaleBilinear(inp, itsStructure->getOutDims());
  itsInput[layer+1] = inp;//implicit conversion frol Image<float> to image<double>
}

// ######################################################################
template <class T>
void NeuralSimModule<T>::setInputGain(const float& weight, const int layer)
{
  itsInputGain[layer+1] = (double)weight;
}

// ######################################################################
template <class T>
Image<float> NeuralSimModule<T>::getV(const int layer) const
{ 
  Image<float> out = itsStructure->getOutput(layer);//conversion from double to float

  //rescale to the inputu size if necessary
  out = rescaleBilinear(out, itsInputDims);
  return out;
}

// ######################################################################
template <class T>
ImageSet<float> NeuralSimModule<T>::getSubV() const
{
  ImageSet<float> out(itsStructure->numSubs());
  if (itsStructure->numSubs() > 0)  
    for (uint ii = 0; ii < itsStructure->numSubs(); ++ii)
      {
        out[ii] = itsStructure->getOutput(ii); //conversion from double to float
        rescaleBilinear(out[ii], itsInputDims); 
      }
  else
    {
      Image<float> temp = itsStructure->getOutput();
      rescaleBilinear(temp, itsInputDims);
      out.push_back(temp);//conversion from double to float
    } 
  return out;
}

// ######################################################################
template <class T>
void NeuralSimModule<T>::update(const SimTime& time)
{
  //reset module if its currently not valid
  if (!itsStructure.is_valid())
    LFATAL("The module must recieve input before it updated.");
  
  for (uint ii = 0; ii < itsInput.size(); ++ii)
    if (itsInput.getImage(ii).initialized())
      itsStructure->input(itsInput.getImage(ii)*itsInputGain[ii], ii - 1); 
  
  itsStructure->evolve(time);
}

// ######################################################################
template <class T>
void NeuralSimModule<T>::reset()
{ 
  if (itsStructure.is_valid())
    itsStructure->initialize();
  if (itsPlot.is_valid())
    itsPlot->reset();
  
  itsInput = ImageSet<double>(itsStructure->numSubs()+1);
}

// ######################################################################
template <class T>
void NeuralSimModule<T>::setModel(const std::string& model_name, const Dims& dims, const SimTime& starttime)
{
  //set our border policy based on wether we are using space variant boundaries or not
  BorderPolicy bp = (itsUseSpaceVariantBoundary.getVal()) ? CROSS_HEMI : NONE;
  
  //change any factory parameters
  uint w = dims.w(); uint h = dims.h();
  nsu::setParameter(nsu::SimStructure::Factory::instance(), bp, itsSCtimestep.getVal(), w, h);
  
  //reset the module
  LINFO("model type: %s", model_name.c_str());
  itsStructure.reset(nsu::SimStructure::Factory::instance().createConvert<T*>(model_name));
  itsStructure->setTime(starttime);
  
  //setup plotting range
  nsu::NormalizeType ntype;
  Range<double> itsRange = itsDisplayRange.getVal();
  if ((itsRange.min() < 0) && (itsRange.max() < 0))//if both are less than 0 scale
    {
      ntype = nsu::SCALE;
      itsRange = Range<double>(0.0,0.0);
    }
  else if ((itsRange.min() == 0) && (itsRange.max() == 0))//set to min/max of data
    ntype = nsu::RANGE;
  else //set to auto scale at each time
    ntype = nsu::SET_RANGE;
  
  //set a decode if desired and initialize plotting
  if (itsDecoderType.getVal().compare("None") != 0)
    {
      nsu::NeuralDecoder* nd = nsu::NeuralDecoder::Factory::instance().create(itsDecoderType.getVal());
      itsPlot.reset(new nsu::StructurePlot(*itsStructure, *nd, its2DPlotDepth.getVal(), ntype, 
                                           itsRange.min(), itsRange.max()));
      delete nd;
    }
  else
    itsPlot.reset(new nsu::StructurePlot(*itsStructure, its2DPlotDepth.getVal(), ntype, 
                                         itsRange.min(), itsRange.max()));

  //update our probe position and set sampling rate for display text
  itsPlot->setSamplingRate(itsStructure->getTimeStep()); 
  nsu::Location location(itsProbe.getVal()); 
  itsPlot->setProbe(location);

  //setup image set to hold input
  const uint depth = (itsStructure->numSubs() < 1) ? 2 : itsStructure->numSubs()+1;
  itsInput = ImageSet<double>(depth);
  itsInputGain = std::vector<double>(depth, 1.0);
}

// ######################################################################
template <class T>
Layout<PixRGB<byte> > NeuralSimModule<T>::getDisplay() const
{
  if (itsInput[0].initialized())
    return itsPlot->draw(*itsStructure, itsInput[0], its2DPlotSize.getVal().w(), 
                         its2DPlotSize.getVal().h(), Dims(0,1),  
                         itsPlotLength.getVal(), itsProbeDepth.getVal(),
                         itsDisplayOutput.getVal());
  else
    return itsPlot->draw(*itsStructure, its2DPlotSize.getVal().w(), 
                         its2DPlotSize.getVal().h(), Dims(0,1),  
                         itsPlotLength.getVal(), itsProbeDepth.getVal(),
                         itsDisplayOutput.getVal());
}

// ######################################################################
template <class T>
rutz::shared_ptr<T> NeuralSimModule<T>::getStructure()
{
  return itsStructure;
}

//#include "ModelNeuron/SC.H"
//template class NeuralSimModule<SCInterface>;
template class NeuralSimModule<nsu::SimStructure>;

#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
