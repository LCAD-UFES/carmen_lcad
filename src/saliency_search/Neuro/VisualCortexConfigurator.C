/*!@file Neuro/VisualCortexConfigurator.C Class to select a VisualCortex at
  runtime */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/VisualCortexConfigurator.C $
// $Id: VisualCortexConfigurator.C 11580 2009-08-11 00:54:51Z itti $
//

#include "Neuro/VisualCortexConfigurator.H"

#include "Beowulf/Beowulf.H"
#include "Channels/RawVisualCortex.H"
#include "Channels/IntegerRawVisualCortex.H"
#include "Channels/ChannelOpts.H"
#include "Channels/ChannelVisitor.H"
#include "Channels/EntropyChannel.H"
#include "Channels/InformationChannel.H"
#include "Channels/InputHandlerThreaded.H"
#include "Channels/MichelsonChannel.H"
#include "Channels/MultiSpectralResidualChannel.H"
#include "Channels/PN03contrastChannel.H"
#include "Channels/ScorrChannel.H"
#include "Channels/SpectralResidualChannel.H"
#include "Channels/TcorrChannel.H"
#include "Channels/VarianceChannel.H"
#include "Component/OptionManager.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/VisualCortex.H"
#include "Neuro/VisualCortexBeo.H"
#include "Neuro/VisualCortexEyeMvt.H"
#include "Neuro/VisualCortexSurprise.H"
#include "Beowulf/BeowulfOpts.H"
#include "Util/StringUtil.H"

namespace
{
  class ThreadInstaller : public ChannelVisitor
  {
  public:
    ThreadInstaller()
    {}

    virtual ~ThreadInstaller()
    {}

    virtual void visitChannelBase(ChannelBase& chan)
    {
      LFATAL("I don't know how to configure channel '%s' for Threaded use",
             chan.descriptiveName().c_str());
    }

    virtual void visitSingleChannel(SingleChannel& chan)
    {
      chan.setInputHandler
        (rutz::make_shared(new InputHandlerThreaded));
    }

    virtual void visitComplexChannel(ComplexChannel& chan)
    {
      chan.setSubchanVisitor
        (rutz::make_shared(new ThreadInstaller));

      // now iterate over the subchannels:
      for (uint i = 0; i < chan.numChans(); ++i)
        chan.subChan(i)->accept(*this);
    }
  };
}

// ######################################################################
VisualCortexConfigurator::
VisualCortexConfigurator(OptionManager& mgr,
                         const std::string& descrName,
                         const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsVCtype(&OPT_VisualCortexType, this),
  itsVC(new VisualCortex(mgr)), // initialize with an empty VisualCortex
  itsBeo()
{
  addSubComponent(itsVC);
}

// ######################################################################
VisualCortexConfigurator::~VisualCortexConfigurator()
{  }

// ######################################################################
nub::ref<VisualCortex> VisualCortexConfigurator::getVC() const
{ return itsVC; }

// ######################################################################
nub::soft_ref<Beowulf> VisualCortexConfigurator::getBeo() const
{ return itsBeo; }

// ######################################################################
void VisualCortexConfigurator::paramChanged(ModelParamBase* const param,
                                            const bool valueChanged,
                                            ParamClient::ChangeStatus* status)
{
  ModelComponent::paramChanged(param, valueChanged, status);

  // was that a change of our baby's name?
  if (param == &itsVCtype) {
    // some info message:
    LINFO("Configuring VC of type %s", itsVCtype.getVal().c_str());
    OptionManager& mgr = this->getManager();

    // if we had one, let's unregister it (when we later reset() the
    // nub::ref, the current VisualCortex will unexport its
    // command-line options). NOTE: here we explicitly reset itsVC to
    // a placeholder dummy (base class) VisualCortex, to make sure all
    // options get unexported right now, as otherwise some conflict
    // will occur between the int definition of --vc-chans and the std
    // one, when switching from an Std VisualCortex to an Int one:
    LINFO("Resetting VisualCortex...");
    removeSubComponent(*itsVC);
    itsVC.reset(new VisualCortex(mgr));

    if (itsBeo.isValid()) {
      LINFO("Resetting VisualCortex Beowulf...");
      removeSubComponent(*itsBeo); itsBeo.reset(NULL);
    }

    // Look for modifiers, like "Thread:"
    std::vector<std::string> tok;
    split(itsVCtype.getVal(), ":", std::back_inserter(tok));
    bool threaded = false; const std::string vct = tok.back(); tok.pop_back();
    for (size_t i = 0; i < tok.size(); ++i)
      if (tok[i].compare("Thread") == 0) threaded = true;
      else LFATAL("Unknown vc-type modifier: %s", tok[i].c_str());

    // instantiate a VC of the appropriate type:

    // #################### Empty "Stub" visual cortex:
    if (vct.compare("None") == 0 || vct.compare("Stub") == 0)
      itsVC.reset(new VisualCortex(mgr));

    // #################### Standard channels:
    else if (vct.compare("Std") == 0)
      {
        nub::ref<VisualCortexStd> vcx(new VisualCortexStd(mgr, "Visual Cortex", "VisualCortex"));
        if (threaded) {
          vcx->itsVCX->setSubchanVisitor(rutz::make_shared(new ThreadInstaller));
          vcx->itsVCX->sortChannelsByNumSubmaps(true);
        }
        itsVC = vcx;
      }
    // #################### Human eye movement fake cortex:
    else if (vct.compare("EyeMvt") == 0)
      itsVC.reset(new VisualCortexEyeMvt(mgr));

    // #################### Entropy model:
    else if (vct.compare("Entrop") == 0) // entropy
      {
        nub::ref<VisualCortexStd> vcx(new VisualCortexStd(mgr));
        vcx->itsVCX->removeAllSubChans(); vcx->itsVCX->hideOption(&OPT_RawVisualCortexChans);
        nub::soft_ref<EntropyChannel> channel(new EntropyChannel(mgr));
        vcx->itsVCX->addSubChan(channel);
        mgr.setOptionValString(&OPT_LevelSpec, "4-4,0-0,4");
        itsVC = vcx;
      }

    // #################### Variance model:
    else if (vct.compare("Variance") == 0)
      {
        nub::ref<VisualCortexStd> vcx(new VisualCortexStd(mgr));
        vcx->itsVCX->removeAllSubChans(); vcx->itsVCX->hideOption(&OPT_RawVisualCortexChans);
        nub::soft_ref<VarianceChannel> channel(new VarianceChannel(mgr));
        vcx->itsVCX->addSubChan(channel);
        mgr.setOptionValString(&OPT_LevelSpec, "4-4,0-0,4");
        itsVC = vcx;
      }

    // #################### Michelson contrast model:
    else if (vct.compare("Michelson") == 0)
      {
        nub::ref<VisualCortexStd> vcx(new VisualCortexStd(mgr));
        vcx->itsVCX->removeAllSubChans(); vcx->itsVCX->hideOption(&OPT_RawVisualCortexChans);
        nub::soft_ref<MichelsonChannel> channel(new MichelsonChannel(mgr));
        vcx->itsVCX->addSubChan(channel);
        mgr.setOptionValString(&OPT_LevelSpec, "4-4,0-0,4");
        itsVC = vcx;
      }

    // #################### Tcorr model:
    else if (vct.compare("Tcorr") == 0)
      {
        nub::ref<VisualCortexStd> vcx(new VisualCortexStd(mgr));
        vcx->itsVCX->removeAllSubChans(); vcx->itsVCX->hideOption(&OPT_RawVisualCortexChans);
        nub::soft_ref<TcorrChannel> channel(new TcorrChannel(mgr));
        vcx->itsVCX->addSubChan(channel);
        mgr.setOptionValString(&OPT_LevelSpec, "4-4,0-0,4");
        itsVC = vcx;
      }

    // #################### Scorr model:
    else if (vct.compare("Scorr") == 0)
      {
        nub::ref<VisualCortexStd> vcx(new VisualCortexStd(mgr));
        vcx->itsVCX->removeAllSubChans(); vcx->itsVCX->hideOption(&OPT_RawVisualCortexChans);
        nub::soft_ref<ScorrChannel> channel(new ScorrChannel(mgr));
        vcx->itsVCX->addSubChan(channel);
        mgr.setOptionValString(&OPT_LevelSpec, "4-4,0-0,4");
        itsVC = vcx;
      }

    // #################### Information model:
    else if (vct.compare("Info") == 0)
      {
        nub::ref<VisualCortexStd> vcx(new VisualCortexStd(mgr));
        vcx->itsVCX->removeAllSubChans(); vcx->itsVCX->hideOption(&OPT_RawVisualCortexChans);
        nub::soft_ref<InformationChannel> channel(new InformationChannel(mgr));
        vcx->itsVCX->addSubChan(channel);
        mgr.setOptionValString(&OPT_LevelSpec, "4-4,0-0,4");
        itsVC = vcx;
      }

    // #################### PN03contrast model:
    else if (vct.compare("PN03contrast") == 0)
      {
        nub::ref<VisualCortexStd> vcx(new VisualCortexStd(mgr));
        vcx->itsVCX->removeAllSubChans(); vcx->itsVCX->hideOption(&OPT_RawVisualCortexChans);
        nub::soft_ref<PN03contrastChannel> channel(new PN03contrastChannel(mgr));
        vcx->itsVCX->addSubChan(channel);
        mgr.setOptionValString(&OPT_LevelSpec, "4-4,0-0,4");
        itsVC = vcx;
      }

    // #################### Spectral Residual model:
    else if (vct.compare("SpectralResidual") == 0)
      {
        nub::ref<VisualCortexStd> vcx(new VisualCortexStd(mgr));
        vcx->itsVCX->removeAllSubChans(); vcx->itsVCX->hideOption(&OPT_RawVisualCortexChans);
        nub::soft_ref<SpectralResidualChannel> channel(new SpectralResidualChannel(mgr));
        vcx->itsVCX->addSubChan(channel);
        mgr.setOptionValString(&OPT_LevelSpec, "4-4,0-0,4");
        itsVC = vcx;
      }

    // #################### Multi Spectral Residual model:
    else if (vct.compare("MultiSpectralResidual") == 0)
      {
        nub::ref<VisualCortexStd> vcx(new VisualCortexStd(mgr));
        vcx->itsVCX->removeAllSubChans(); vcx->itsVCX->hideOption(&OPT_RawVisualCortexChans);
        nub::soft_ref<MultiSpectralResidualChannel> channel(new MultiSpectralResidualChannel(mgr));
        vcx->itsVCX->addSubChan(channel);
        mgr.setOptionValString(&OPT_LevelSpec, "4-4,0-0,4");
        itsVC = vcx;
      }

    // #################### Standard beo-channels:
    else if (vct.compare("Beo") == 0)
      {
        itsBeo.reset(new Beowulf(mgr, "Visual Cortex Beowulf", "VisualCortexBeowulf", true));
        nub::ref<VisualCortexStd> vcx(new VisualCortexStd(mgr, "Visual Cortex", "VisualCortex"));
        setupVisualCortexBeo(*(vcx->itsVCX), itsBeo);
        itsVC = vcx;
      }

    // #################### Standard surprise-channels:
    else if (vct.compare("Surp") == 0)
      {
        nub::ref<VisualCortexSurprise> vcs(new VisualCortexSurprise(mgr));
        nub::ref<VisualCortexStd> vcx(new VisualCortexStd(mgr, "Visual Cortex", "VisualCortex", vcs));
        if (threaded) {
          vcx->itsVCX->setSubchanVisitor(rutz::make_shared(new ThreadInstaller));
          vcx->itsVCX->sortChannelsByNumSubmaps(true);
        }
        itsVC = vcx;
      }

    // #################### Standard int channels:
    else if (vct.compare("Int") == 0)
      {
        nub::ref<VisualCortexInt> vcx(new VisualCortexInt(mgr, "Visual Cortex", "VisualCortex"));
        if (threaded) {
          vcx->itsVCX->setSubchanVisitor(rutz::make_shared(new ThreadInstaller));
          vcx->itsVCX->sortChannelsByNumSubmaps(true);
        }
        itsVC = vcx;
      }
    // #################### EnvVisualCortex
    else if (vct.compare("Env") == 0)
      {
        nub::ref<VisualCortexEnv> vcx(new VisualCortexEnv(mgr, "Visual Cortex", "VisualCortex"));
        itsVC = vcx;
      }
    // #################### unknown
    else
      LFATAL("Unknown vc-type: %s", vct.c_str());

    // add our babies as a subcomponents of us so that they will
    // become linked to the manager through us (hopefully we are
    // registered with the manager), which in turn will allow them to
    // export their command-line options and get configured. Note: the
    // Beowulf needs to be start()'ed by the time the VC starts, so we
    // add it first here if we have one:
    if (itsBeo.isValid())
      {
        addSubComponent(itsBeo);

        // export options:
        itsBeo->exportOptions(MC_RECURSE);

        // make sure we are beowulf master:
        mgr.setOptionValString(&OPT_BeowulfMaster, "true");
      }

    // register our visual cortex as a subcomponent:
    addSubComponent(itsVC);

    // export its options:
    itsVC->exportOptions(MC_RECURSE);
  }
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
