/*!@file ModelNeuron/FreeViewingModel.C Implementation */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/FreeViewingModel.C $

#include "ModelNeuron/FreeViewingModel.H"
#include "Component/ModelOptionDef.H"
#include "Simulation/SimEventQueue.H"
#include "Image/ColorOps.H"
#include "Image/MathOps.H"
#include "Image/ImageSetOps.H"
#include "Image/DrawOps.H"
#include "Image/Convolutions.H"
#include "Image/ShapeOps.H"
#include "Image/ColorMap.H"
#include "Image/Transforms.H"
#include "Image/LowPass.H"
#include "Raster/GenericFrame.H"
#include "Neuro/NeuroOpts.H"
#include "ModelNeuron/SCNorm.H"
#include "ModelNeuron/SCFitOpts.H"

const ModelOptionCateg MOC_SCFreeViewingModel = { MOC_SORTPRI_3, "SC FreeViewing model options" };

extern const ModelOptionDef OPT_SCPoolingSize = 
{ MODOPT_ARG(Dims), "SCPoolingSize", &MOC_SCFreeViewingModel, OPTEXP_CORE,
  "The size at which we pool over visual cortex and retinal output", 
  "sc-pooling", '\0', "<Dims>", "3x3" };

extern const ModelOptionDef OPT_DemoRetinalImageSize = 
{ MODOPT_ARG(Dims), "DemoRetinalImageSize", &MOC_SCFreeViewingModel, OPTEXP_CORE,
  "Size of retinal output for demo purposes", 
  "retinal-rescale", '\0', "<Dims>", "1920x1080" };

extern const ModelOptionDef OPT_FreeViewingDisplayChan = 
{ MODOPT_FLAG, "FreeViewingDisplayChan", &MOC_SCFreeViewingModel, OPTEXP_CORE,
  "save the freeviewing channel output", 
  "display-freeviewing-channels", '\0', "<bool>", "false" };

extern const ModelOptionDef OPT_FreeViewingDisplayInverse = 
{ MODOPT_FLAG, "FreeViewingDisplayInverse", &MOC_SCFreeViewingModel, OPTEXP_CORE,
  "display the inverse SC map", 
  "display-sc-inverse", '\0', "<bool>", "false" };

extern const ModelOptionDef OPT_FreeViewingDisplayDemo = 
{ MODOPT_FLAG, "FreeViewingDisplayDemo", &MOC_SCFreeViewingModel, OPTEXP_CORE,
  "display demo plot", 
  "display-sc-demo", '\0', "<bool>", "true" };

extern const ModelOptionDef OPT_FreeViewingUpdateNorm = 
{ MODOPT_FLAG, "FreeViewingUpdateNorm", &MOC_SCFreeViewingModel, OPTEXP_CORE,
  "update the norm file", 
  "update-normfile", '\0', "<bool>", "false" };

extern const ModelOptionDef OPT_FreeViewingLoadChanMgzFileName = 
{ MODOPT_ARG(std::string), "FreeViewingLoadChanMgzFileName", &MOC_SCFreeViewingModel, OPTEXP_CORE,
  "mgz file to load channel outputs", 
  "load-channel-filename", '\0', "<file name>", "" };

extern const ModelOptionDef OPT_FreeViewingSaveChanMgzFileName = 
{ MODOPT_ARG(std::string), "FreeViewingSaveChanMgzFileName", &MOC_SCFreeViewingModel, OPTEXP_CORE,
  "mgz file to save channel outputs", 
  "save-channel-filename", '\0', "<file name>", "" };

extern const ModelOptionDef OPT_FreeViewingNeuralFileName = 
{ MODOPT_ARG(std::string), "FreeViewingNeuralFileName", &MOC_SCFreeViewingModel, OPTEXP_CORE,
  "filename of neural data used for demo", 
  "neural-filename", '\0', "<file name>", "" };

extern const ModelOptionDef OPT_FreeViewingOutputFileName = 
{ MODOPT_ARG(std::string), "FreeViewingOutputFileName", &MOC_SCFreeViewingModel, OPTEXP_CORE,
  "The filename to save channel and saliency map values at probe position.", 
  "output-filename", '\0', "<file name>", "" };

extern const ModelOptionDef OPT_FreeViewingDownSampleFactor = 
{ MODOPT_ARG(uint), "FreeViewingDownSampleFactor", &MOC_SCFreeViewingModel, OPTEXP_CORE,
  "factor to downsample input image", 
  "downsample-factor", '\0', "<uint>", "0" };

extern const ModelOptionDef OPT_FreeViewingEsigChanMaxNorm = 
{ MODOPT_ARG(float), "FreeViewingEsigChanMaxNorm", &MOC_SCFreeViewingModel, OPTEXP_CORE,
  "sigma of excitatory channel max norm", 
  "channel-maxnorm-esig", '\0', "<float>", "2.0" };

extern const ModelOptionDef OPT_FreeViewingIsigChanMaxNorm = 
{ MODOPT_ARG(float), "FreeViewingIsigChanMaxNorm", &MOC_SCFreeViewingModel, OPTEXP_CORE,
  "sigma of inhibitory channel max norm", 
  "channel-maxnorm-isig", '\0', "<float>", "25.0" };

extern const ModelOptionDef OPT_FreeViewingStrChanMaxNorm = 
{ MODOPT_ARG(float), "FreeViewingStrChanMaxNorm", &MOC_SCFreeViewingModel, OPTEXP_CORE,
  "strength of channel max norm", 
  "channel-maxnorm-strength", '\0', "<float>", "1.0" };

extern const ModelOptionDef OPT_FreeViewingEsigSCMaxNorm = 
{ MODOPT_ARG(float), "FreeViewingEsigSCMaxNorm", &MOC_SCFreeViewingModel, OPTEXP_CORE,
  "excitatory sigma of SC max norm", 
  "sc-maxnorm-esig", '\0', "<float>", "2.0" };

extern const ModelOptionDef OPT_FreeViewingIsigSCMaxNorm = 
{ MODOPT_ARG(float), "FreeViewingIsigSCMaxNorm", &MOC_SCFreeViewingModel, OPTEXP_CORE,
  "inhibitory sigma of SC max norm", 
  "sc-maxnorm-isig", '\0', "<float>", "25.0" };

extern const ModelOptionDef OPT_FreeViewingStrSCMaxNorm = 
{ MODOPT_ARG(float), "FreeViewingStrSCMaxNorm", &MOC_SCFreeViewingModel, OPTEXP_CORE,
  "strength of SC max norm", 
  "sc-maxnorm-strength", '\0', "<float>", "1.0" };

extern const ModelOptionDef OPT_FreeViewingNormSum = 
{ MODOPT_FLAG, "FreeViewingNormSum", &MOC_SCFreeViewingModel, OPTEXP_CORE,
  "norm channel outputs by the sum of all channels at each location", 
  "norm-chan-sum", '\0', "<bool>", "false" };

extern const ModelOptionDef OPT_FreeViewingNormMax = 
{ MODOPT_FLAG, "FreeViewingNormMax", &MOC_SCFreeViewingModel, OPTEXP_CORE,
  "norm each channel by its max collected from a norm file; this normalizations happens before all others", 
  "norm-chan-max", '\0', "<bool>", "false" };

extern const ModelOptionDef OPT_FreeViewingMapFac = 
{ MODOPT_ARG(float), "FreeViewingMapFac", &MOC_SCFreeViewingModel, OPTEXP_CORE,
  "map factor for display", "map-factor", '\0', "<float>", "1.0" };

extern const ModelOptionDef OPT_FreeViewingChanOnly = 
{ MODOPT_FLAG, "FreeViewingChanOnly", &MOC_SCFreeViewingModel, OPTEXP_CORE,
  "compute the channels only, the saliency output will be zeros", 
  "channels-only", '\0', "<bool>", "false" };

// ######################################################################
// FreeViewingModel Implementation
// ######################################################################
FreeViewingModel::FreeViewingModel(OptionManager& mgr,
                                   const std::string& descrName,
                                   const std::string& tag) :
    SimModule(mgr, descrName, tag), 
    SIMCALLBACK_INIT(SimEventRetinaImage),
    SIMCALLBACK_INIT(SimEventSaveOutput),
    itsSCPoolingSize(&OPT_SCPoolingSize, this),
    itsSCChannelMaxNormEsig(&OPT_FreeViewingEsigChanMaxNorm, this),
    itsSCChannelMaxNormIsig(&OPT_FreeViewingIsigChanMaxNorm, this),
    itsSCChannelMaxNormStrength(&OPT_FreeViewingStrChanMaxNorm, this),
    itsSCSCMaxNormEsig(&OPT_FreeViewingEsigSCMaxNorm, this),
    itsSCSCMaxNormIsig(&OPT_FreeViewingIsigSCMaxNorm, this),
    itsSCSCMaxNormStrength(&OPT_FreeViewingStrSCMaxNorm, this),
    itsSCChanNormSum(&OPT_FreeViewingNormSum, this),
    itsSCChanNormMax(&OPT_FreeViewingNormMax, this),
    itsSCChannelsOnly(&OPT_FreeViewingChanOnly, this),
    itsRetinalDisplaySize(&OPT_DemoRetinalImageSize, this),
    itsDisplayChan(&OPT_FreeViewingDisplayChan, this),
    itsSCInverse(&OPT_FreeViewingDisplayInverse, this),
    itsSCDemo(&OPT_FreeViewingDisplayDemo, this),
    itsMapFac(&OPT_FreeViewingMapFac, this),
    itsNormFileName(&OPT_NormFileName, this), 
    itsUpdateNormFile(&OPT_FreeViewingUpdateNorm, this),
    itsLoadChanFile(&OPT_FreeViewingLoadChanMgzFileName, this), 
    itsSaveChanFile(&OPT_FreeViewingSaveChanMgzFileName, this), 
    itsStimLocation(&OPT_StimLocation, this),
    itsPixPerDeg(&OPT_PixelsPerDegree, this),
    itsFramingImageName(&OPT_InputFramingImageName, this),
    itsOutputFileBase(&OPT_FreeViewingOutputFileName, this),
    itsNeuralFileName(&OPT_FreeViewingNeuralFileName, this),
    itsDownSampleFactor(&OPT_FreeViewingDownSampleFactor, this), 
    itsSamplingRate("FreeViewingSamplingRate", this, SimTime::HERTZ(200)),
    
    itsWorker(1),
    
    frame(0),
    downsampleFac(0),
    itsRetinalDisplaySizeLoc(),    
    doNorm(false),
    itsSCChanMaxNormEsig(0.0F),
    itsSCChanMaxNormIsig(0.0F),
    itsSCChanMaxNormStrength(0.0F),
    itsSCMaxNormEsig(0.0F),
    itsSCMaxNormIsig(0.0F),
    itsSCMaxNormStrength(0.0F),
    itsChanNormSum(false),
    itsChanNormMax(false),
    itsOnlyChan(false),
    itsStimLoc(),
    itsSaveResults(false),
    itsMapFactor(0.0),
    itsSCRf(),
    itsRetinal(), 
    itsSCInput(), 
    itsSC(),
    itsChanOutputs(),
    itsChanRange(),
    itsPlot(),
    itsNeuralPlot(),
    itsNeuralData(),

    itsSpeEngine(new SpatioTemporalEnergyEngine()),
    itsChanDecoder(),
    itsChanEncoder(),
    itsSamplePoints(),
    itsSalResults(), 
    itsChanResults(),

    itsChannelNames(),
    itsChannels({nub::soft_ref<FreeViewingChannel>(new SCLuminanceChannel(mgr, "Luminance")),

          nub::soft_ref<FreeViewingChannel>(new SCRGChannel(mgr, "RG")), 
          nub::soft_ref<FreeViewingChannel>(new SCBYChannel(mgr, "BY")),

          nub::soft_ref<FreeViewingChannel>(new SCFlickerChannel(mgr, 0, "Flicker")),

          nub::soft_ref<FreeViewingChannel>(new SCEdgeChannel(mgr, 0, "Edge")),
          nub::soft_ref<FreeViewingChannel>(new SCEdgeChannel(mgr, 1, "Edge")),

          nub::soft_ref<FreeViewingChannel>(new SCMotionChannel(mgr, 0, "Motion")),
          nub::soft_ref<FreeViewingChannel>(new SCMotionChannel(mgr, 1, "Motion"))})
{
  for (uint ii = 0; ii < itsChannels.size(); ++ii)
    addSubComponent(itsChannels[ii]);
}

// ######################################################################
void FreeViewingModel::start1()
{
  SimModule::start1();
}

// ######################################################################
void FreeViewingModel::start2()
{
  SimModule::start2();

  //store a kernel representing the SC Rf structure
  itsSCRf = createSCRfKernel();

  //set local copies of variables
  itsRetinalDisplaySizeLoc = itsRetinalDisplaySize.getVal();
  downsampleFac = itsDownSampleFactor.getVal();

  uint dsf = (uint)pow(2.0, (double)downsampleFac);
  if (dsf > 1)
  {
    for (uint ii = 0; ii < itsChannels.size(); ++ii)
    {
      PixPerDeg d = itsChannels[ii]->getPPD();
      PixPerDeg nd(d.ppdx() / dsf, d.ppdy() / dsf);
      itsChannels[ii]->setPPD(nd);
      LINFO("Resetting ppd to %s", toStr(nd).c_str());
    }
  }

  //setup our channel names 
  for (uint ii = 0; ii < itsChannels.size(); ++ii)
  {
    std::vector<std::string> str = itsChannels[ii]->getSubChanNames();
    itsChannelNames.insert(itsChannelNames.end(), str.begin(), str.end());
  }

  //make room for the outputs
  itsChanOutputs.reset(itsChannelNames.size());

  //resize the work queue
  itsWorker.resize(itsChannels.size());

  //get our spe scales
  std::vector<uint> scales;
  for (uint ii = 0; ii < itsChannels.size(); ++ii)
  {
    nub::soft_ref<SCSpeChannelBase> r = dyn_cast_weak<SCSpeChannelBase>(itsChannels[ii]);
    if (r.is_valid())
    {
      r->setEngine(itsSpeEngine);
      uint const s = r->getScale();
      std::vector<uint>::iterator value = std::find(scales.begin(), scales.end(), s);
      if (value == scales.end())
        scales.push_back(s);
    }
  }

  std::sort(scales.begin(), scales.end());

  LINFO("Starting the engine with %d scales", (uint)scales.size());
  
  //start the spe engine
  if (scales.size() > 0)
    itsSpeEngine->start(scales);
  else
    itsSpeEngine.reset(NULL);


  //setup the normfile
  if (itsNormFileName.getVal().empty())
    doNorm = false;
  else
  {
    doNorm = (itsUpdateNormFile.getVal()) ? true : false;
    itsChanRange = loadChanNormFile(itsNormFileName.getVal(), itsChannelNames);
    
    if (itsChanRange.size() == 0)
      itsChanRange.resize(itsChannelNames.size());
  }
  
  //check out whether we want to load or save channels
  if (!itsLoadChanFile.getVal().empty())
    itsChanDecoder = rutz::make_shared(new MgzDecoder(itsLoadChanFile.getVal()));
  
  if (!itsSaveChanFile.getVal().empty())
    itsChanEncoder = rutz::make_shared(new MgzEncoder(itsSaveChanFile.getVal(), 9));

  if (itsFramingImageName.getVal().find("BlackImage") == std::string::npos)
  {
    LFATAL("must have a black framing image");
  }

  std::string size = itsFramingImageName.getVal().substr(10);
  Dims screenSize = fromStr<Dims>(size);
  if (dsf > 1)
  {
    screenSize = Dims(screenSize.w() / dsf, screenSize.h() / dsf);
    LINFO("Settings screen dims to resampled %s", toStr(screenSize).c_str());
  }

  //initialize the channels
  for (uint ii = 0; ii < itsChannels.size(); ++ii)
  {
    uint f = itsChannels[ii]->getFactor();
    Dims d(screenSize.w() / f, screenSize.h() / f);
    itsChannels[ii]->clear(d);
  }

  //print info on the SC pooling
  for (float ii = 10.0F; ii < 50.0F; ii+=10.0F)
  {
    const float factor = 3.0F;
    PixPerDeg pixdeg = itsChannels[0]->getPPD();
    Point2D<int> p = itsChannels[0]->degToSvCoords(Point2D<float>(ii, 0.0F));

    Point2D<int> sc1 = p;
    itsChannels[0]->fromSvCoords(sc1);
    Point2D<float> d1 = SCTransform::pix2Deg(sc1.i, sc1.j, pixdeg.ppdx(), pixdeg.ppdy(), screenSize.w(), screenSize.h());

    Point2D<int> sc2 = p;
    sc2.i+=factor;
    itsChannels[0]->fromSvCoords(sc2);
    Point2D<float> d2 = SCTransform::pix2Deg(sc2.i, sc2.j, pixdeg.ppdx(), pixdeg.ppdy(), screenSize.w(), screenSize.h());

    Point2D<int> sc3 = p;
    sc3.i+=factor;
    sc3.j+=factor;
    itsChannels[0]->fromSvCoords(sc3);
    Point2D<float> d3 = SCTransform::pix2Deg(sc3.i, sc3.j, pixdeg.ppdx(), pixdeg.ppdy(), screenSize.w(), screenSize.h());

    Point2D<int> sc4 = p;
    sc4.j+=factor;
    itsChannels[0]->fromSvCoords(sc4);
    Point2D<float> d4 = SCTransform::pix2Deg(sc4.i, sc4.j, pixdeg.ppdx(), pixdeg.ppdy(), screenSize.w(), screenSize.h());

    Point2D<int> sc5 = p;
    sc5.i-=factor;
    sc5.j+=factor;
    itsChannels[0]->fromSvCoords(sc5);
    Point2D<float> d5 = SCTransform::pix2Deg(sc5.i, sc5.j, pixdeg.ppdx(), pixdeg.ppdy(), screenSize.w(), screenSize.h());

    Point2D<int> sc6 = p;
    sc6.i-=factor;
    itsChannels[0]->fromSvCoords(sc6);
    Point2D<float> d6 = SCTransform::pix2Deg(sc6.i, sc6.j, pixdeg.ppdx(), pixdeg.ppdy(), screenSize.w(), screenSize.h());

    Point2D<int> sc7 = p;
    sc7.j-=factor;
    sc7.i-=factor;
    itsChannels[0]->fromSvCoords(sc7);
    Point2D<float> d7 = SCTransform::pix2Deg(sc7.i, sc7.j, pixdeg.ppdx(), pixdeg.ppdy(), screenSize.w(), screenSize.h());

    Point2D<int> sc8 = p;
    sc8.j-=factor;
    itsChannels[0]->fromSvCoords(sc8);
    Point2D<float> d8 = SCTransform::pix2Deg(sc8.i, sc8.j, pixdeg.ppdx(), pixdeg.ppdy(), screenSize.w(), screenSize.h());

    Point2D<int> sc9 = p;
    sc9.i+=factor;
    sc9.j-=factor;
    itsChannels[0]->fromSvCoords(sc9);
    Point2D<float> d9 = SCTransform::pix2Deg(sc9.i, sc9.j, pixdeg.ppdx(), pixdeg.ppdy(), screenSize.w(), screenSize.h());

    LINFO("Deg : %3.2f, %3.2f %s %s %s %s %s %s %s %s %s %s",ii, 0.0F, toStr(d1).c_str(), toStr(d2).c_str(), toStr(d3).c_str(), toStr(d4).c_str(), toStr(d5).c_str(), toStr(d6).c_str(), toStr(d7).c_str(), toStr(d8).c_str(), toStr(d9).c_str(), toStr(d2).c_str());
  }

  //setup the probe location
  itsStimLoc = itsStimLocation.getVal();
  
  //setup saving output 
  if (itsOutputFileBase.getVal().size() > 2)
  {
    itsSaveResults = true;
    itsChanResults.resize(itsChannelNames.size());
  }

  //setup plot at probe location
  itsPlot.reset(SimTime::SECS(2), 0.0F, 255.0F / itsMapFac.getVal(), itsSamplingRate.getVal());
  itsNeuralPlot.reset(SimTime::SECS(2), 0.0F, 200.0F, itsSamplingRate.getVal());

  //setup sc params
  itsSCChanMaxNormEsig = itsSCChannelMaxNormEsig.getVal();
  itsSCChanMaxNormIsig = itsSCChannelMaxNormIsig.getVal();
  itsSCChanMaxNormStrength = itsSCChannelMaxNormStrength.getVal();
  itsSCMaxNormEsig = itsSCSCMaxNormEsig.getVal();
  itsSCMaxNormIsig = itsSCSCMaxNormIsig.getVal();
  itsSCMaxNormStrength = itsSCSCMaxNormStrength.getVal();
  itsChanNormSum = itsSCChanNormSum.getVal();
  itsChanNormMax = itsSCChanNormMax.getVal();
  itsOnlyChan = itsSCChannelsOnly.getVal();

  itsMapFactor = itsMapFac.getVal();

  //neural data for plotting
  if (itsNeuralFileName.getVal().size() > 2)
  {
    std::ifstream neuralDataFile(itsNeuralFileName.getVal());

    if (!neuralDataFile.is_open())
      LFATAL("cannot locate neural data file %s", itsNeuralFileName.getVal().c_str());
    
    std::string line;
    while (getline(neuralDataFile, line))
    {
      std::vector<std::string> toks;
      split(line, "", back_inserter(toks));
      if (toks.size() != 1)
        LFATAL("Invalid line while reading the neural data file. The reader expects one floating point value per line.");
      
      float data = fromStr<float>(toks[0]);          
      itsNeuralData.push_back(data);
    }
    
    //close stream
    neuralDataFile.close();
    LINFO("neural data file '%s' loaded. %d samples ", itsNeuralFileName.getVal().c_str(), (uint)itsNeuralData.size());
  }

  //get our sampling points
  itsSamplePoints = getMapPoints(itsChannels[0]->getSVImageDims(), itsChannels[0]->degToSvCoords(itsStimLoc));

  //set our sc amd sc inputs
  itsSCInput = Image<float>(itsChannels[0]->getSVImageDims(), ZEROS);
  itsSC = Image<float>(itsChannels[0]->getSVImageDims(), ZEROS);
}

// ######################################################################
void FreeViewingModel::stop2()
{
  SimModule::stop2();

  if (doNorm)
    writeChanNormFile(itsNormFileName.getVal(), itsChannelNames, itsChanRange);

  if (itsChanEncoder.is_valid()) itsChanEncoder->close();

  if (itsSaveResults)
  {
    std::ofstream modelRespOutFile(itsOutputFileBase.getVal()); 
    if (!modelRespOutFile.is_open())
      LFATAL("Error: cannot write to file %s", itsOutputFileBase.getVal().c_str());
    
    for (uint ii = 0; ii < itsSalResults.size(); ++ii)
    {
      modelRespOutFile << itsSalResults[ii];
      for (uint jj = 0; jj < itsChanResults.size(); ++jj)
        modelRespOutFile << " " << itsChanResults[jj][ii];

      modelRespOutFile << std::endl;
    }
    modelRespOutFile.close();
  }
}

// ######################################################################
FreeViewingModel::~FreeViewingModel()
{
}

// ######################################################################
Image<float> FreeViewingModel::createSCRfKernel()
{
  //store a kernel representing the SC Rf structure
  Dims rf = itsSCPoolingSize.getVal();//the redius

  if (rf.w() != rf.h())
    LFATAL("RF must be isotropic");

  if ((rf.w() == 0) && (rf.h() == 0))
    return Image<float>();

  Image<float> ret(rf.w() * 2 + 1, rf.h() * 2 + 1, ZEROS);//add 1 to ensure odd size

  for (int x = 0; x < ret.getWidth(); ++x)
    for (int y = 0; y < ret.getHeight(); ++y)
    {
      float const r = sqrt((x-rf.w())*(x-rf.w()) + (y-rf.h())*(y-rf.h()));
      if (r <= rf.w())
        ret.setVal(x,y,1.0F);
    }
  
  return ret;
}

// ######################################################################
std::vector<Point2D<int> > FreeViewingModel::getMapPoints(Dims const & dims, Point2D<int> const & pos)
{
  std::vector<Point2D<int> > ret;
  Dims rf = itsSCPoolingSize.getVal();

  if ((rf.w() == 0) && (rf.h() == 0))
  {
    ret.push_back(pos);
    return ret;
  }

  for (int x = 0; x < dims.w(); ++x)
    for (int y = 0; y < dims.h(); ++y)
    {
      float const r = sqrt((x-pos.i)*(x-pos.i) + (y-pos.j)*(y-pos.j));
      if (r <= rf.w())
        ret.push_back(Point2D<int>(x,y));
    }
  
  return ret;
}

// ######################################################################
float FreeViewingModel::sampleFromMap(Image<float> const & map, std::vector<Point2D<int> > const & pnts)
{
  float sum = 0.0F;
  std::vector<Point2D<int> >::const_iterator ii = pnts.begin();
  while (ii != pnts.end())
    sum += map.getVal(*ii++);

  return sum;
}

// ######################################################################
void FreeViewingModel::onSimEventRetinaImage(SimEventQueue& q, rutz::shared_ptr<SimEventRetinaImage>& e)
{
  /*
    get the retinal image and downsample
  */  
  itsRetinal = e->frame().colorByte();

  for (uint ii = 0; ii < downsampleFac; ++ii)
    itsRetinal = lowPass5yDecY(lowPass5xDecX(itsRetinal));

  //if we don't have a channel decoder, then we must compute the features
  if (!itsChanDecoder.is_valid())
  {
    /*
      compute the DKL components
    */    
    Image<float> rgimg, byimg, lumimg;
    getDKLM(itsRetinal, rgimg, byimg, lumimg);
    
    //get the spatiotemporal engine updated
    if (itsSpeEngine.is_valid())
      itsSpeEngine->update(lumimg);
    
    /* 
       Pass the retinal image through the space variant channels (compute feature;
       compute DoG transform; take abs, rectify, opponency, etc)
    */
    std::vector<ImageSet<float> > results(itsChannels.size());
    {
      std::vector<std::function<void()> > jobs;
      for (uint ii = 0; ii < itsChannels.size(); ++ii)
      {
        std::function<void()> f = [this, ii, &results, &lumimg, &rgimg, &byimg]()
          { results[ii] = itsChannels[ii]->getOutput(lumimg, rgimg, byimg); };
        jobs.push_back(f);
      }
      itsWorker.push(std::move(jobs));
    }
      
    //organize the results
    uint idx = 0;
    for (uint jj = 0; jj < itsChannels.size(); ++jj)
      for (uint kk = 0; kk < results[jj].size(); ++kk)
      {
        itsChanOutputs[idx] = results[jj][kk];  
        
        //shoud we save raw channels?
        if (itsChanEncoder.is_valid())
          itsChanEncoder->writeFrame(GenericFrame(itsChanOutputs[idx], FLOAT_NORM_PRESERVE));

        //update the channels range if desired
        if (doNorm)
          itsChanRange[idx].merge(rangeOf(itsChanOutputs[idx]));

        //re-range the channels
        if (itsChanNormMax)
          itsChanOutputs[idx] = remapRange(itsChanOutputs[idx], itsChanRange[idx], Range<float>(0.0, 1.0));
        
        ++idx;
      }
  }
  else //just load the channels from the mgz and re-range
  {
    for (uint jj = 0; jj < itsChanOutputs.size(); ++jj)
    { 
      GenericFrame frame = itsChanDecoder->readFrame();
      if (!frame.initialized())
        LFATAL("couldn't read channel output from MGZ file");
      
      itsChanOutputs[jj] = frame.asGrayF32();

      //re-range the channels
      if (itsChanNormMax)
        itsChanOutputs[jj] = remapRange(itsChanOutputs[jj], itsChanRange[jj], Range<float>(0.0, 1.0));
    }
  }
  
  /*
    apply any cross channel normalization
  */
  if (itsChanNormSum)
  {
    float s = sum(sum(itsChanOutputs));
    s+= 0.000001F;
    for (uint jj = 0; jj < itsChanOutputs.size(); ++jj)
      itsChanOutputs[jj] /= s;
  }
  
  /*
    apply max norm at channel level
  */
  {
    std::vector<std::function<void()> > jobs;
    for (uint jj = 0; jj < itsChanOutputs.size(); ++jj)
    {
      std::function<void()> f = [this, jj]()
        { itsChanOutputs[jj] = SCmaxNormalizeFancy(itsChanOutputs[jj], itsSCChanMaxNormEsig, itsSCChanMaxNormIsig, itsSCChanMaxNormStrength); };
      
      jobs.push_back(f);
    }
    itsWorker.push(std::move(jobs));  
  }
  
  if (!itsOnlyChan)
  {
    /* 
       sum them all up to get cortex/retinal output
    */  
    itsSCInput = sum(itsChanOutputs);
    
    /*
      pool over space to get SC receptive field structure
    */
    if (itsSCRf.initialized())
      itsSCInput = optConvolve(itsSCInput, itsSCRf);
    
    /*
      apply max norm at the SC level
    */
    itsSC = SCmaxNormalizeFancy(itsSCInput, itsSCMaxNormEsig, itsSCMaxNormIsig, itsSCMaxNormStrength);
  }  
  
  /* collect values if we are saving */
  if (itsSaveResults)
  {  
    Point2D<int> svprobe = itsChannels[0]->degToSvCoords(itsStimLoc);
    for (uint jj = 0; jj < itsChanOutputs.size(); ++jj)
    {
      float const vchan = sampleFromMap(itsChanOutputs[jj], itsSamplePoints);//itsChanOutputs[jj].getVal(svprobe.i, svprobe.j);
      itsChanResults[jj].push_back(vchan);
    } 
    
    float const vsal = itsSC.getVal(svprobe.i, svprobe.j);
    itsSalResults.push_back(vsal);
    
    //store for plotting
    LINFO("sampled %3.2f at time %s. SC location: %s Visual Deg location: %s", vsal, toStr(q.now()).c_str(), toStr(svprobe).c_str(), toStr(itsStimLoc).c_str());  
    itsPlot.push(q.now(), vsal);

    if (itsNeuralData.size() > 0)
      itsNeuralPlot.push(q.now(), itsNeuralData[frame]);
  }
  else
    LINFO("Time : %s Frame : %d", toStr(q.now()).c_str(), frame);
  
  ++frame;
}

// ######################################################################
void FreeViewingModel::onSimEventSaveOutput(SimEventQueue& q, rutz::shared_ptr<SimEventSaveOutput>& e)
{
  if (itsSaveResults)
  {
    // get the OFS to save to, assuming sinfo is of type
    // SimModuleSaveInfo (will throw a fatal exception otherwise):
    nub::ref<FrameOstream> ofs = dynamic_cast<const SimModuleSaveInfo&>(e->sinfo()).ofs;
    if (ofs->isVoid())
      return;

    Image<PixRGB<byte> > retinalout = rescaleBilinear(itsRetinal, itsRetinalDisplaySizeLoc);

    if (itsSCDemo.getVal())
    {
      ColorMap cm = ColorMap::GREY();
      PixRGB<byte> col(0, 255, 155);
      PixRGB<byte> colt(0, 0, 0);

      drawLine(retinalout, Point2D<int>(retinalout.getWidth()-1, retinalout.getHeight()-1), Point2D<int>(retinalout.getWidth()-1,0), col, 1);
      drawLine(retinalout, Point2D<int>(0, retinalout.getHeight()-2), Point2D<int>(0,0), col, 2);
      drawLine(retinalout, Point2D<int>(0, 0), Point2D<int>(retinalout.getWidth()-2,0), col, 2);
      drawLine(retinalout, Point2D<int>(0, retinalout.getHeight()-2), Point2D<int>(retinalout.getWidth()-2,retinalout.getHeight()-2), col, 2);
      std::string title = "Retinal Image";      
      writeText(retinalout, Point2D<int>(0,0), title.c_str(), colt, col);

      itsSC *= itsMapFactor;
      Image<byte> itsSCByte = itsSC;
      Image<PixRGB<byte> > itsSCCol =  colorize(itsSCByte, cm);
      title = "SC Map";
      writeText(itsSCCol, Point2D<int>(0,0), title.c_str(), colt, col);

      Image<float> itsSCInv = rescaleBilinear(itsChannels[0]->inverseTransform(itsSC), itsRetinalDisplaySizeLoc);
      Image<byte> itsSCInvByte = itsSCInv;
      Image<PixRGB<byte> > itsSCInvCol =  colorize(itsSCInvByte, cm);

      drawLine(itsSCInvCol, Point2D<int>(itsSCInvByte.getWidth()-2, itsSCInvByte.getHeight()-1), Point2D<int>(itsSCInvByte.getWidth()-2,0), col, 2);
      drawLine(itsSCInvCol, Point2D<int>(0, itsSCInvByte.getHeight()-1), Point2D<int>(0,0), col, 1);
      drawLine(itsSCInvCol, Point2D<int>(0, 0), Point2D<int>(itsSCInvByte.getWidth()-2,0), col, 2);
      drawLine(itsSCInvCol, Point2D<int>(0, itsSCInvByte.getHeight()-2), Point2D<int>(itsSCInvByte.getWidth()-2,itsSCInvByte.getHeight()-2), col, 2);
      title = "Inverted SC Map";
      writeText(itsSCInvCol, Point2D<int>(0,0), title.c_str(), colt, col);

      Layout<PixRGB<byte> > layout(retinalout);
      layout = hcat(layout, itsSCInvCol);

      Layout<PixRGB<byte> > layoutb(itsSCCol);
      
      Image<PixRGB<byte> > pltm;
      pltm = itsPlot.draw(layout.getWidth() - itsSCCol.getWidth(), itsSCCol.getHeight(), 
                          "", "Sp/s", "", PixRGB<byte>(128,128,255), 4, 0);
      
      if (itsNeuralData.size() > 0)
      {
        Image<PixRGB<byte> > pltn;
        pltn = itsNeuralPlot.draw(layout.getWidth() - itsSCCol.getWidth(), itsSCCol.getHeight(), 
                                  "Spike Rate","Sp/s","Time",PixRGB<byte>(0,0,0),0, true);
        
        
        layoutb = hcat(layoutb, composite(pltn, pltm, PixRGB<byte>(255,255,255)));
      }
      else
        layoutb = hcat(layoutb, pltm);
      
      std::vector<float> hist(itsChanOutputs.size());
      std::transform(itsChanResults.begin(), itsChanResults.end(), hist.begin(), [](std::vector<float> const & h){return h.back();});

      layout = vcat(layout, layoutb);
      layout = vcat(layout, drawHistogram(hist, layout.getWidth(), 300, colt, PixRGB<byte>(128, 128, 255)));

      ofs->writeRgbLayout(layout, "SC_Demo");
    }
    else
    {
      ofs->writeRGB(retinalout, "Retinal");
      
      if (itsDisplayChan.getVal())  
      {
        //display channel maps  
        for (uint ii = 0; ii < itsChanOutputs.size(); ++ii)
        {
          Image<float> f = itsChanOutputs[ii];
          f *= 255.0F;
          Image<byte> b = f;
          ofs->writeFrame(GenericFrame(b), itsChannelNames[ii]);
        }
      }

      //display SC input map
      Image<float> f = itsSCInput;
      f *= itsMapFactor;
      Image<byte> b = f;
      ofs->writeFrame(GenericFrame(b), "VisualCortex");
        
      //display SC map
      f = itsSC;
      f *= itsMapFactor;
      b = f;
      ofs->writeFrame(GenericFrame(b), "SC");
      
      //display inverse sc map
      if (itsSCInverse.getVal())
      {
        f = rescaleBilinear(itsChannels[0]->inverseTransform(itsSC), itsRetinalDisplaySizeLoc);
        f *= itsMapFactor;
        Image<byte> b = f;
        ofs->writeFrame(GenericFrame(b), "SC_Inverse");
      }
      
      //display 1D plot
      Image<PixRGB<byte> > plot = itsPlot.draw(800,300,"Probe", "sp/s", "Time", PixRGB<byte>(0,0,0), 4, 0);
      ofs->writeRGB(plot, "Probe activity");
    }
  }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
