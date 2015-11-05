/*!@file ModelNeuron/CompLayer.H Class declarations for structure that
consists of an Gaussian excitatory layer which projects to an inhibitory layer,
which projects in a gaussian pattern pack to the excitatory layer. Inspired by
the layout of the mamillian SC. */

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
// $HeadURL: svn://isvn.usc.edu:/software/invt/trunk/saliency/src/ModelNeuron/CompLayer.H $

#ifndef  NEURALMODEL_COMPLAYER_H_DEFINED
#define  NEURALMODEL_COMPLAYER_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include "ModelNeuron/Structure.H"
#include "ModelNeuron/Layer.H"
#include "ModelNeuron/Weights.H"

namespace nsu
{
  // ######################################################################
  //  CompLayer
  //  ######################################################################
  // A class representing a single lamina containing excitatory and
  // inhibitory neurons. A network containing interacting excitatory and
  // inhhibitory units where an excitatory layer (input layer) has local
  // recurrent excitation. Each unit in the exciatory layer connects
  // directly to a single unit in the inhibitory layer. The inhibitory
  // units connect in a gaussian pattern back to the excitatory
  // network. A variety of behaviors can be obtained such as bumbs,
  // waves, weak winner take all selection etc....
  //
  // It is hypothesized that each layer of the primate colliculus (Sgs,
  // Sgi) supperior colliculus is organized in such a way, with
  // different sized excitation and inhibition regions.
  //
  // Computational framework inspired by work from SI Amari. See SC
  // class for citations on inspiration for the construction of a model
  // Sgs and Sgi, and interlaminar connectivity.
  //  ######################################################################
  template <class EUnit, class IUnit>
  struct CompLayer : public Structure<SimLayer*>
  {
      //! constructor
      CompLayer(const double& excite_std, const double& inhibit_std,
                const double& excite_w, const double& feedforward, const double& feedback,
                const BorderPolicy bp, const SimTime& timestep, 
                const uint width, const uint height, const std::string name = "", const std::string units = "")
          : Structure<SimLayer*>(timestep, width, height, name, units)
      { 
        WeightsUniform ff(feedforward);
        WeightsBinomial fb(inhibit_std, feedback, false, bp);

        Layer<EUnit, WeightsBinomial> itsE(excite_std, excite_w, true, bp, timestep, width, height, "Excitatory");
        Layer<IUnit, WeightsEmpty> itsI(timestep, width, height, "Inhibitory");

        addSub(itsE); 
        addSub(itsI);

        addInteraction(0, 1, ff);
        addInteraction(1, 0, fb);

        setDefaultIO(0,0);
      }
  
      //default copy and assignment OK
  
      //!destructor 
      virtual ~CompLayer() { };  
  
      //set the modules
      void setModules(const SimUnit& excite_module, const SimUnit& inhibit_module)
      {
        editSub(0).setModule(excite_module);
        editSub(1).setModule(inhibit_module);
      }
  
      //!get the current display output 
      Image<double> getDisplayOutput(const int pos = -1) const
      {
        if (pos < 0)
        {
          Image<double> e = getSub(0).getDisplayOutput();
          const Image<double> i = getSub(1).getDisplayOutput();
          e -= i;
          return e;
        }
        else
          return getSub(pos).getDisplayOutput();
      }

      //!clone the unit
      CompLayer<EUnit, IUnit>* clone() const {return new CompLayer<EUnit,IUnit>(*this); }
  };
}
#endif
#endif  
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
