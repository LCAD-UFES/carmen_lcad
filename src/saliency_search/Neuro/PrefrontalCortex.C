/*!@file Neuro/PrefrontalCortex.C a human PrefrontalCortex */

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
// Primary maintainer for this file: Lior Elazary
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/PrefrontalCortex.C $
// $Id: PrefrontalCortex.C 13065 2010-03-28 00:01:00Z itti $
//

#include "Neuro/PrefrontalCortex.H"

#include "Component/OptionManager.H"
#include "Component/ModelOptionDef.H"
#include "Channels/GuidedSearch.H"
#include "Channels/OptimalGains.H"
#include "Image/DrawOps.H"
#include "Learn/Bayes.H"
#include "Media/MediaSimEvents.H"
#include "Media/TestImages.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/NeuroSimEvents.H"
#include "Neuro/VisualCortex.H"
#include "ObjRec/BayesianBiaser.H"
#include "Simulation/SimEventQueue.H"

// ######################################################################
// ######################################################################
// ########## PrefrontalCortex implementation
// ######################################################################
// ######################################################################
PrefrontalCortex::PrefrontalCortex(OptionManager& mgr,
                                   const std::string& descrName,
                                   const std::string& tagName) :
  SimModule(mgr, descrName, tagName)
{ }

// ######################################################################
PrefrontalCortex::~PrefrontalCortex()
{ }

// ######################################################################
// ######################################################################
// ########## PrefrontalCortexConfigurator implementation
// ######################################################################
// ######################################################################
PrefrontalCortexConfigurator::
PrefrontalCortexConfigurator(OptionManager& mgr,
                   const std::string& descrName,
                   const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsType(&OPT_PrefrontalCortexType, this),
  itsPFC(new PrefrontalCortexStub(mgr))
{
  addSubComponent(itsPFC);
}

// ######################################################################
PrefrontalCortexConfigurator::~PrefrontalCortexConfigurator()
{  }

// ######################################################################
nub::ref<PrefrontalCortex> PrefrontalCortexConfigurator::getPFC() const
{ return itsPFC; }

// ######################################################################
void PrefrontalCortexConfigurator::paramChanged(ModelParamBase* const param,
                                      const bool valueChanged,
                                      ParamClient::ChangeStatus* status)
{
  ModelComponent::paramChanged(param, valueChanged, status);

  // was that a change of our baby's name?
  if (param == &itsType) {
    // let's unregister our existing PrefrontalCortex:
    removeSubComponent(*itsPFC);

    // instantiate a PrefrontalCortex of the appropriate type (when the old
    // PrefrontalCortex is destroyed, it will un-export its command-line
    // options):
    if (itsType.getVal().compare("Stub") == 0)              // stub
      itsPFC.reset(new PrefrontalCortexStub(getManager()));
    else if (itsType.getVal().compare("OG") == 0)           // Optimal Gains
      itsPFC.reset(new PrefrontalCortexOG(getManager()));
    else if (itsType.getVal().compare("GS") == 0)           // Guided Search
      itsPFC.reset(new PrefrontalCortexGS(getManager()));
    else if (itsType.getVal().compare("SB") == 0)           // SalBayes
      itsPFC.reset(new PrefrontalCortexSB(getManager()));
    else
      LFATAL("Unknown PrefrontalCortex type %s", itsType.getVal().c_str());

    // add our baby as a subcomponent of us so that it will become
    // linked to the manager through us (hopefully we are registered
    // with the manager), which in turn will allow it to export its
    // command-line options and get configured:
    addSubComponent(itsPFC);

    // tell the controller to export its options:
    itsPFC->exportOptions(MC_RECURSE);

    // some info message:
    LINFO("Selected PFC of type %s", itsType.getVal().c_str());
  }
}

// ######################################################################
// ######################################################################
// ########## PrefrontalCortexStub implementation
// ######################################################################
// ######################################################################
PrefrontalCortexStub::PrefrontalCortexStub(OptionManager& mgr,
                                           const std::string& descrName,
                                           const std::string& tagName) :
  PrefrontalCortex(mgr, descrName, tagName)
{ }

// ######################################################################
PrefrontalCortexStub::~PrefrontalCortexStub()
{ }

// ######################################################################
// ######################################################################
// ########## PrefrontalCortexOG implementation
// ######################################################################
// ######################################################################

static const ModelOptionDef OPT_PFCOG_stsdfilename =
  { MODOPT_ARG_STRING, "PFCOGstsdfilename", &MOC_PFC, OPTEXP_CORE,
    "Name of the file to save computed salienceT and salienceD values, "
    "so that they could be later combined across several images.",
    "stsd-filename", '\0', "<filename>", "" };

static const ModelOptionDef OPT_PFCOG_DoMax =
  { MODOPT_FLAG, "PFCOGdoMax", &MOC_PFC, OPTEXP_CORE,
    "Use the max value from the object as a feature or the mean value.",
    "pfc-do-max", '\0', "", "false" };

static const ModelOptionDef OPT_PFCOG_targetMaskObjName =
  { MODOPT_ARG_STRING, "PFCOGtargetMaskObjName", &MOC_PFC, OPTEXP_CORE,
    "Name of the object used to build the target mask. The mask is "
    "built by looking at all objects matching this name. If no name "
    "is specified then every object that is in the xml is used to "
    "build the target mask. ",
    "target-mask-objName", '\0', "<name>", "" };

static const ModelOptionDef OPT_PFCOG_distractorMaskObjName =
  { MODOPT_ARG_STRING, "PFCOGdistractorMaskObjName", &MOC_PFC, OPTEXP_CORE,
    "Name of the object used to build the distractor mask. The mask is "
    "built by looking at all objects matching this name. If no name "
    "is specified then the distractor is build by taking the "
    "complement of the target mask",
    "distractor-mask-objName", '\0', "<name>", "" };

namespace {
  // a SimEvent to compute salience of target and distractor
  class SimReqOGtrain : public SimReqVCXchanVis {
  public:
    SimReqOGtrain(SimModule* src, const Image<byte>& tmask,
                  const Image<byte>& dmask,
                  rutz::shared_ptr<ParamMap> pmap,
                  const std::string& fname, const bool domax) :
      SimReqVCXchanVis(src, rutz::shared_ptr<ChannelVisitor>
                       (new OptimalGainsFinder(tmask, dmask, pmap, domax))),
      itsPmap(pmap), itsFilename(fname)
    { }

    virtual ~SimReqOGtrain()
    { }

    virtual void postProcessing(RawVisualCortex *vcx)
    {
      if (itsFilename.empty() == false)
        {
          LINFO("Saving sT and sD values to %s", itsFilename.c_str());
          itsPmap->format(itsFilename);
        }
    }

  private:
    rutz::shared_ptr<ParamMap> itsPmap;
    const std::string itsFilename;
  };
};

// ######################################################################
PrefrontalCortexOG::PrefrontalCortexOG(OptionManager& mgr,
                                       const std::string& descrName,
                                       const std::string& tagName) :
  PrefrontalCortex(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventInputFrame),
  SIMCALLBACK_INIT(SimEventVisualCortexOutput),
  itsFilename(&OPT_PFCOG_stsdfilename, this),
  itsTargetMaskObjName(&OPT_PFCOG_targetMaskObjName, this),
  itsDistractorMaskObjName(&OPT_PFCOG_distractorMaskObjName, this),
  itsDoMax(&OPT_PFCOG_DoMax, this),
  itsTargetMask(), itsDistractorMask()
{ }

// ######################################################################
void PrefrontalCortexOG::
onSimEventInputFrame(SimEventQueue& q, rutz::shared_ptr<SimEventInputFrame>& e)
{
  GenericFrame gf = e->frame();
  rutz::shared_ptr<GenericFrame::MetaData>
    metaData = gf.getMetaData(std::string("SceneData"));
  if (metaData.get() != 0) {
    rutz::shared_ptr<TestImages::SceneData> sceneData;
    sceneData.dyn_cast_from(metaData);

    // Train on all objects in the scene
    // Build the target and distractor mask, if multiple objects then add
    // which ever matches the target string to the target mask, and distractor string to
    // distractor mask. If no strings are spacified, then build from all objects and/or distractor
    // that is complmant the object

    itsDistractorMask = Image<byte>(sceneData->dims, ZEROS);
    itsTargetMask = Image<byte>(sceneData->dims, ZEROS);

    for (uint i = 0; i < sceneData->objects.size(); i++) {
      TestImages::ObjData objData = sceneData->objects[i];

      // get the target mask, either it was provided as a mask image:
      if (objData.objmask.initialized())
      {
        LINFO("Drawing target mask from B/W mask file data...");
        itsTargetMask += objData.objmask;
      }

      // ... and/or with a polygon:
      if (objData.polygon.empty() == false) {
        LINFO("Drawing target mask from polygon data from %s...", objData.name.c_str());
        if (itsTargetMaskObjName.getVal().size() > 0 &&
            objData.name == itsTargetMaskObjName.getVal())
          drawFilledPolygon(itsTargetMask, objData.polygon, byte(255));

        if (itsTargetMaskObjName.getVal().size() == 0)
          drawFilledPolygon(itsTargetMask, objData.polygon, byte(255));
      }

      if (itsDistractorMaskObjName.getVal().size() > 0 &&
          objData.name == itsDistractorMaskObjName.getVal())
      {
        LINFO("Drawing distractor mask from polygon data from %s...", objData.name.c_str());
        drawFilledPolygon(itsDistractorMask, objData.polygon, byte(255));
      }
    }

    if (itsDistractorMaskObjName.getVal().size() == 0)
    {
      // Create a distractor mask from 255-targetMask:
      itsDistractorMask.clear(255);
      itsDistractorMask -= itsTargetMask;
    }
  }
}

// ######################################################################
void PrefrontalCortexOG::
onSimEventVisualCortexOutput(SimEventQueue& q, rutz::shared_ptr<SimEventVisualCortexOutput>& e)
{
  rutz::shared_ptr<ParamMap> pmap(new ParamMap());
  rutz::shared_ptr<SimReqVCXchanVis> ev(new SimReqOGtrain(this, itsTargetMask, itsDistractorMask, pmap,
                                                          itsFilename.getVal(), itsDoMax.getVal()));
  LINFO("Requesting training STSD data from VCX...");
  q.request(ev);

  // the resulting sT and sD values will be saved to our
  // filename in the form of a ParamMap.
}

// ######################################################################
PrefrontalCortexOG::~PrefrontalCortexOG()
{ }

// ######################################################################
// ######################################################################
// ########## PrefrontalCortexGS implementation
// ######################################################################
// ######################################################################
// ######################################################################
static const ModelOptionDef OPT_PFCGS_gainsfilename =
  { MODOPT_ARG_STRING, "PFCGSgainsfilename", &MOC_PFC, OPTEXP_CORE,
    "Name of the file to load biasing gains.",
    "gains-filename", '\0', "<filename>", "" };

namespace {
  // a SimEvent to do guided search from a pmap of gains
  class SimReqGSbias : public SimReqVCXchanVis {
  public:
    SimReqGSbias(SimModule* src, rutz::shared_ptr<ParamMap> pmap) :
      SimReqVCXchanVis(src, rutz::shared_ptr<ChannelVisitor>(new GuidedSearchBiaser(pmap)))
    { }

    virtual ~SimReqGSbias()
    { }
  };
};


// ######################################################################
PrefrontalCortexGS::PrefrontalCortexGS(OptionManager& mgr,
                                       const std::string& descrName,
                                       const std::string& tagName) :
  PrefrontalCortex(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventInputFrame),
  itsFilename(&OPT_PFCGS_gainsfilename, this)
{ }

// ######################################################################
PrefrontalCortexGS::~PrefrontalCortexGS()
{ }

// ######################################################################
void PrefrontalCortexGS::
onSimEventInputFrame(SimEventQueue& q, rutz::shared_ptr<SimEventInputFrame>& e)
{
  // load the paramap:
  rutz::shared_ptr<ParamMap> pmap = ParamMap::loadPmapFile(itsFilename.getVal());

  // post an event with biasing information:
  rutz::shared_ptr<SimReqVCXchanVis> ev(new SimReqGSbias(this, pmap));
  q.request(ev);
}


// ######################################################################
// ######################################################################
// ########## PrefrontalCortexSB implementation
// ######################################################################
// ######################################################################

static const ModelOptionDef OPT_PFCSalBayes_NetworkFile =
  { MODOPT_ARG_STRING, "PFCBaysNetFile", &MOC_PFC, OPTEXP_CORE,
    "Name of the file to read the computed Bayesian Network",
    "pfc-BayesNet-file", '\0', "<filename>", "SalBayes.net" };

static const ModelOptionDef OPT_PFCSalBayes_ObjToBias =
  { MODOPT_ARG(int), "PFCObjectToBiasFor", &MOC_PFC, OPTEXP_CORE,
    "The object ID from the database that we should be biasing for.",
    "pfc-obj-to-bias", '\0', "<int>", "0" };

namespace {
  // a SimEvent to do SalBayes biasing
  class SimReqSBbias : public SimReqVCXchanVis {
  public:
    SimReqSBbias(SimModule* src, Bayes& b, const int obj) :
      SimReqVCXchanVis(src, rutz::shared_ptr<ChannelVisitor>(new BayesianBiaser(b, obj, -1, true)))
    { }

    virtual ~SimReqSBbias()
    { }
  };
};

// ######################################################################
PrefrontalCortexSB::PrefrontalCortexSB(OptionManager& mgr,
                                       const std::string& descrName,
                                       const std::string& tagName) :
  PrefrontalCortex(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventInputFrame),
  itsBayesNetFilename(&OPT_PFCSalBayes_NetworkFile, this),
  itsObjToBias(&OPT_PFCSalBayes_ObjToBias, this)
{ }

// ######################################################################
void PrefrontalCortexSB::start1()
{
  itsBayesNet.reset(new Bayes(0, 0)); //This will be set by the load
  if (!itsBayesNet->load(itsBayesNetFilename.getVal().c_str()))
    LFATAL("Cannot load database file %s",
           itsBayesNetFilename.getVal().c_str());

  PrefrontalCortex::start1();
}

// ######################################################################
void PrefrontalCortexSB::
onSimEventInputFrame(SimEventQueue& q, rutz::shared_ptr<SimEventInputFrame>& e)
{
  const uint obj = uint(itsObjToBias.getVal());
  if (obj < itsBayesNet->getNumClasses()) {
    const char* oname = itsBayesNet->getClassName(obj);
    LINFO("Biasing VC for '%s'", oname);

    // Post the object we are biasing for so that interested people know:
    rutz::shared_ptr<SimEventObjectToBias>
      eotb(new SimEventObjectToBias(this, oname));
    q.post(eotb);

    // Post the bias visitor to blackboard
    rutz::shared_ptr<SimReqVCXchanVis> ebv(new SimReqSBbias(this, *itsBayesNet.get(), obj));
    q.request(ebv);
  } else {
    LINFO("Cannot find objid %i in the database", obj);
    LINFO("Available objects are:");
    for (uint i = 0; i < itsBayesNet->getNumClasses(); i++)
      LINFO("%i: %s", i, itsBayesNet->getClassName(i));
    LFATAL("Please specify a valid object id");
  }
}

// ######################################################################
PrefrontalCortexSB::~PrefrontalCortexSB()
{ }

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
