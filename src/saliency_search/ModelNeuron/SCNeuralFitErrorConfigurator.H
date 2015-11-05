//////////////////////////////////////////////////////////////////////////
// University of Southern California (USC) and the iLab at USC.         //
// See http://iLab.usc.edu for information about this project.          //
//////////////////////////////////////////////////////////////////////////
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
//////////////////////////////////////////////////////////////////////////
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
//////////////////////////////////////////////////////////////////////////
#ifndef MODELNEURON_SCNEURALFITERRORCONFIGURATOR_H_DEFINED
#define MODELNEURON_SCNEURALFITERRORCONFIGURATOR_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "ModelNeuron/SCNeuralFitError.H"
#include "nub/ref.h"

// ######################################################################
class SCNeuralFitErrorConfigurator : public ModelComponent
{
  public:
    //! Constructor
    SCNeuralFitErrorConfigurator(OptionManager& mgr,
                                 const std::string& descrName =
                                 "SC Neural Fit Error Configurator",
                                 const std::string& tagName =
                                 "SCNeuralFitErrorConfigurator");

    //! destructor
    virtual ~SCNeuralFitErrorConfigurator();

    //! Get the chosen SCNeuralFitError
    nub::ref<SCNeuralFitErrorAdapter> getSC() const;

  protected:
    OModelParam<std::string> itsSCtype; //!< type of saliency map

    virtual void paramChanged(ModelParamBase* const param,
                              const bool valueChanged,
                              ParamClient::ChangeStatus* status);

  private:
    nub::ref<SCNeuralFitErrorAdapter> itsSC; // the saliency map
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif
#endif 
