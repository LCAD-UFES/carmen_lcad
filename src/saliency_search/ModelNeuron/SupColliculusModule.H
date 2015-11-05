/*!@file Neuro/SupColliculusModule.H models of the superior colliculus */

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
// $HeadURL: svn://isvn.usc.edu:/software/invt/trunk/saliency/src/Neuro/SupColliculusModule.H $

#ifndef SUPCOLLICULUSMODULE_H_DEFINED
#define SUPCOLLICULUSMODULE_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include "ModelNeuron/NeuralSimModule.H"
//#include "ModelNeuron/SC.H"
  #include "ModelNeuron/SimStructure.H"

namespace nsu
{
// ######################################################################
//! A class to represent different superior colliculus (multi-layered) models
// ######################################################################
class SupColliculusModule : public NeuralSimModule<nsu::SimStructure>//public NeuralSimModule<SCInterface>
{
public:
  //! Uninitialized constructor
  SupColliculusModule(OptionManager& mgr, 
                      const std::string& descrName = "Superior Colliculus",
                      const std::string& tagName = "SupColliculusModule") : NeuralSimModule<nsu::SimStructure>(mgr, descrName, tagName) { };
  
  //! set the default input output layers
    void setDefaultIO(const uint inp_layer, const uint out_layer) { };
  
  //! Destructor
    virtual ~SupColliculusModule() { };
  
protected:
    //  OModelParam<std::string> itsType;//type
    //  OModelParam<float> itsSgsGain;//gain
    //  OModelParam<float> itsSgiGain;//gain

    virtual void start1() { };

    void setModel(const std::string& model, const Dims& dims, const SimTime& starttime = SimTime::ZERO()) { };
};
}
#endif
#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
