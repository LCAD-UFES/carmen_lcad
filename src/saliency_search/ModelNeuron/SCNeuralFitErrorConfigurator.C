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


#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include "ModelNeuron/SCNeuralFitErrorConfigurator.H"
#include "ModelNeuron/SCFitOpts.H"
#include "rutz/trace.h"
  
// ######################################################################
SCNeuralFitErrorConfigurator::SCNeuralFitErrorConfigurator(OptionManager& mgr,
                                                           const std::string& descrName,
                                                           const std::string& tagName) :
    ModelComponent(mgr, descrName, tagName),
    itsSCtype(&OPT_ModelErrorFunc, this),
    itsSC(new SCNeuralFitErrorStub(mgr))
{
  GVX_TRACE(__PRETTY_FUNCTION__);
  addSubComponent(itsSC);
}

// ######################################################################
SCNeuralFitErrorConfigurator::~SCNeuralFitErrorConfigurator()
{
  GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
nub::ref<SCNeuralFitErrorAdapter> SCNeuralFitErrorConfigurator::getSC() const
{
  GVX_TRACE(__PRETTY_FUNCTION__);
  return itsSC;
}

// ######################################################################
void SCNeuralFitErrorConfigurator::paramChanged(ModelParamBase* const param,
                                                const bool valueChanged,
                                                ParamClient::ChangeStatus* status)
{
  GVX_TRACE(__PRETTY_FUNCTION__);
  ModelComponent::paramChanged(param, valueChanged, status);

  // was that a change of our baby's name?
  if (param == &itsSCtype) {

    // if we had one, let's unregister it (when we later reset() the
    // nub::ref, the current SCNeuralFitError will unexport its
    // command-line options):
    removeSubComponent(itsSC);

    // instantiate a SC of the appropriate type:
    if (itsSCtype.getVal().compare("None") == 0 || itsSCtype.getVal().compare("Stub") == 0) // no SC
      itsSC.reset(new SCNeuralFitErrorStub(getManager()));

    else if (itsSCtype.getVal().compare("Slice") == 0)      
      itsSC.reset(new SliceFitError(getManager()));

    else if (itsSCtype.getVal().compare("Monkey") == 0) 
      itsSC.reset(new MonkeyFitError(getManager()));

    else if (itsSCtype.getVal().compare("FreeviewingCT") == 0) 
      itsSC.reset(new FreeviewingFitErrorCT(getManager()));
    
    else if (itsSCtype.getVal().compare("FreeviewingSV") == 0) 
      itsSC.reset(new FreeviewingFitErrorSV(getManager()));
    
    else
      LFATAL("Unknown SC type %s", itsSCtype.getVal().c_str());

    // add our baby as a subcomponent of us so that it will become
    // linked to the manager through us (hopefully we are registered
    // with the manager), which in turn will allow it to export its
    // command-line options and get configured:

    addSubComponent(itsSC);

    // tell the controller to export its options:
    itsSC->exportOptions(MC_RECURSE);

    // some info message:
    LINFO("Selected SC of type %s", itsSCtype.getVal().c_str());
  }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif
