/*!@file Neuro/VisualCortexSurprise.C a VisualCortex with SingleChannelSurprise
  channels */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/VisualCortexSurprise.C $
// $Id: VisualCortexSurprise.C 11584 2009-08-12 05:24:46Z itti $
//

#include "Neuro/VisualCortexSurprise.H"

#include "Channels/ChannelOpts.H"
#include "Channels/ComplexChannel.H"
#include "Channels/SingleChannel.H"
#include "Channels/SubmapAlgorithm.H"
#include "Component/ModelOptionDef.H"
#include "Component/OptionManager.H"
#include "Image/MathOps.H"
#include "Surprise/SingleChannelSurprise.H"
#include "Surprise/SurpriseOpts.H"

// Used by: VisualCortexSurprise
const ModelOptionDef OPT_VCXnormSurp =
  { MODOPT_ARG(int), "VCXnormSurp", &MOC_SURPRISE, OPTEXP_CORE,
    "Apply competitive spatial interactions to VisualCortex output "
    "when --maxnorm-type is Surprise. This will slightly sparsify the "
    "final surprise map output by the VisualCortex. Specify the number "
    "of FancyWeak iterations or 0 for no interactions.",
    "vcx-normsurp", '\0', "<int>", "2" };

// Used by: VisualCortexSurprise
const ModelOptionDef OPT_VCXSurpriseThresh =
  { MODOPT_ARG(float), "VCXSurpriseThresh", &MOC_SURPRISE, OPTEXP_CORE,
    "Threshold for nonlinearity applied to Visual Cortex surprise output "
    "(will be ignored if --surprise-exp=0.0)",
    "surprise-thresh", '\0', "<float>", "0.25" };

// Used by: VisualCortexSurprise
const ModelOptionDef OPT_VCXSurpriseSemisat =
  { MODOPT_ARG(float), "VCXSurpriseSemisat", &MOC_SURPRISE, OPTEXP_CORE,
    "Semisaturation constant for nonlinearity applied to Visual Cortex "
    "surprise output (will be ignored if --surprise-exp=0.0)",
    "surprise-semisat", '\0', "<float>", "0.35" };

// Used by: VisualCortexSurprise
const ModelOptionDef OPT_VCXSurpriseExp =
  { MODOPT_ARG(float), "VCXSurpriseExp", &MOC_SURPRISE, OPTEXP_CORE,
    "Exponent for nonlinearity applied to Visual Cortex surprise output, "
    "or 0.0 to not apply any nonlinearity.",
    "surprise-exp", '\0', "<float>", "4" };


// ######################################################################
VisualCortexSurprise::VisualCortexSurprise(OptionManager& mgr,
                                           const std::string& descrName,
                                           const std::string& tagName) :
  RawVisualCortex(mgr, descrName, tagName),
  itsStype(&OPT_VisualCortexSurpriseType, this),
  itsNormSurp(&OPT_VCXnormSurp, this),
  itsSurpriseThresh(&OPT_VCXSurpriseThresh, this),
  itsSurpriseSemisat(&OPT_VCXSurpriseSemisat, this),
  itsSurpriseExp(&OPT_VCXSurpriseExp, this)
{
  VisualCortexSurprise::registerSurpriseTypes(mgr);

  // make all of the SingleChannel objects change their
  // SubmapAlgorithm type:
  mgr.setOptionValString(&OPT_SubmapAlgoType, "Surprise" + itsStype.getVal());
}

// ######################################################################
VisualCortexSurprise::~VisualCortexSurprise()
{  }

// ######################################################################
void VisualCortexSurprise::registerSurpriseTypes(OptionManager& mgr)
{
  // register the various surprise types with the SubmapAlgorithm
  // factory, so that we can set OPT_SubmapAlgoType to trigger
  // SingleChannel to get its new SubmapAlgorithm from the factory:
  ComponentFactory<SubmapAlgorithm>& f = SubmapAlgorithm::getFactory();
  if (!f.is_valid_key("SurpriseGaussian"))
    f.registerType<SingleChannelSurprise<SurpriseModelSG> >("SurpriseGaussian", mgr);
  if (!f.is_valid_key("SurprisePoisson"))
    f.registerType<SingleChannelSurprise<SurpriseModelSP> >("SurprisePoisson", mgr);
  if (!f.is_valid_key("SurprisePoisson1"))
    f.registerType<SingleChannelSurprise<SurpriseModelSP1> >("SurprisePoisson1", mgr);
  if (!f.is_valid_key("SurprisePoissonConst"))
    f.registerType<SingleChannelSurprise<SurpriseModelSPC> >("SurprisePoissonConst", mgr);
  if (!f.is_valid_key("SurprisePoissonFloat"))
    f.registerType<SingleChannelSurprise<SurpriseModelSPF> >("SurprisePoissonFloat", mgr);
  if (!f.is_valid_key("SurpriseChiSquare"))
    f.registerType<SingleChannelSurprise<SurpriseModelCS> >("SurpriseChiSquare", mgr);
  if (!f.is_valid_key("SurpriseJointGG"))
    f.registerType<SingleChannelSurprise<SurpriseModelGG> >("SurpriseJointGG", mgr);
  if (!f.is_valid_key("SurpriseNathan"))
    f.registerType<SingleChannelSurprise<SurpriseModelPM> >("SurpriseNathan", mgr);
  if (!f.is_valid_key("SurpriseOutlier"))
    f.registerType<SingleChannelSurprise<SurpriseModelOD> >("SurpriseOutlier", mgr);
  if (!f.is_valid_key("SurpriseMultivariant"))
    f.registerType<SingleChannelSurprise<SurpriseModelPM> >("SurpriseMultivariant", mgr);
}

// ######################################################################
void VisualCortexSurprise::paramChanged(ModelParamBase* const param,
                                        const bool valueChanged,
                                        ParamClient::ChangeStatus* status)
{
  RawVisualCortex::paramChanged(param, valueChanged, status);

  // was that a change of surprise type?
  if (param == &itsStype)
    {
      LINFO("Switching to surprise models of type %s", itsStype.getVal().c_str());

      // make all of the SingleChannel objects change their
      // SubmapAlgorithm type:
      getManager().setOptionValString(&OPT_SubmapAlgoType, "Surprise" + itsStype.getVal());
    }
}

// ######################################################################
Image<float> VisualCortexSurprise::postProcessOutputMap(const Image<float>& outmap)
{
  if (itsNormType.getVal() != VCXNORM_SURPRISE)
    LFATAL("You must use --maxnorm-type=Surprise with VisualCortexSurprise");

  Image<float> result = outmap;

  if (itsUseMax.getVal() == false)
    {
      // get the sum of all subchan output maps and divide it by the
      // number of subchans that have non-zero weights, so that surprise
      // averages rather than sums over subchans:
      uint num_nzw = 0; for (uint i = 0; i < numChans(); ++i) if (getSubchanTotalWeight(i) != 0.0) ++num_nzw;
      if (num_nzw > 1)
        {
          LDEBUG("Averaging surprise outputs across %u top channels.", numChans());
          result *= 1.0 / numChans();
        }
    }

  // get rid of background firing activity due to internal relaxation:
  result -= itsSurpriseThresh.getVal();
  inplaceRectify(result);

  // Pass the final surprise through a sigmoid, if exp is non-zero:
  if (itsSurpriseExp.getVal() != 0.0)
    inplaceSigmoid(result, itsSurpriseExp.getVal(), itsSurpriseExp.getVal(),
                   powf(itsSurpriseSemisat.getVal(), itsSurpriseExp.getVal()));

  // Optional spatial competition may be applied to sparsify the output:
  if (itsNormSurp.getVal())
    {
      result = maxNormalize(result, 0.0f, 0.0f, VCXNORM_FANCYWEAK, itsNormSurp.getVal());
      LDEBUG("%s(0.0 .. 0.0) x %d", maxNormTypeName(VCXNORM_FANCYWEAK), itsNormSurp.getVal());
    }

  return result;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
