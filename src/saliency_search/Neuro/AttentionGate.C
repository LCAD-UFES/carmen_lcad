/*!@file Neuro/AttentionGuidanceMap.C Implementation for task-relevance map class */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/AttentionGate.C $
// $Id: AttentionGate.C 13065 2010-03-28 00:01:00Z itti $
//

#include "Neuro/AttentionGate.H"

#include "Channels/ChannelBase.H"
#include "Channels/ChannelMaps.H"
#include "Component/OptionManager.H"
#include "Image/Image.H"
#include "Image/MathOps.H"
#include "Image/ShapeOps.H"  // for rescale()
#include "Media/MediaSimEvents.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/VisualCortex.H"
#include "Simulation/SimEventQueue.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameOstream.H"
#include "Util/log.H"

// ######################################################################
// ######################################################################
// ########## AttentionGuidanceMap implementation
// ######################################################################
// ######################################################################

AttentionGate::
AttentionGate(OptionManager& mgr,
              const std::string& descrName,
              const std::string& tagName,
              const nub::soft_ref<VisualCortex> vcx) :
  SimModule(mgr, descrName, tagName),
  itsSaveResults(&OPT_AGsaveResults, this),
  itsVCX(vcx), itsLogSigO(0.10f), itsLogSigS(20.0f) // see Neuro/NeuroOpts.{H,C}
{ }

// ######################################################################
AttentionGate::~AttentionGate()
{ }

// ######################################################################
// ######################################################################
// ########## AttentionGateConfigurator implementation
// ######################################################################
// ######################################################################
AttentionGateConfigurator::
AttentionGateConfigurator(OptionManager& mgr,
                          const std::string& descrName,
                          const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsAGtype(&OPT_AttentionGateType, this),
  itsAG(new AttentionGateStd(mgr))
{
  addSubComponent(itsAG);
}

// ######################################################################
AttentionGateConfigurator::~AttentionGateConfigurator()
{  }

// ######################################################################
nub::ref<AttentionGate>
AttentionGateConfigurator::getAG() const
{ return itsAG; }

// ######################################################################
void AttentionGateConfigurator::
paramChanged(ModelParamBase* const param,
             const bool valueChanged,
             ParamClient::ChangeStatus* status)
{
  ModelComponent::paramChanged(param, valueChanged, status);

  // was that a change of our baby's name?
  if (param == &itsAGtype) {
    // if we had one, let's unregister it (when we later reset() the
    // nub::ref, the current AttentionGate will unexport its
    // command-line options):
    removeSubComponent(*itsAG);

    // instantiate a SM of the appropriate type:
    if (itsAGtype.getVal().compare("Std") == 0)          // standard
      itsAG.reset(new AttentionGateStd(getManager()));
    else if (itsAGtype.getVal().compare("None") == 0)    // stub
      itsAG.reset(new AttentionGateStub(getManager()));
    else
      LFATAL("Unknown AG type %s", itsAGtype.getVal().c_str());

    // add our baby as a subcomponent of us so that it will become
    // linked to the manager through us (hopefully we are registered
    // with the manager), which in turn will allow it to export its
    // command-line options and get configured:

    addSubComponent(itsAG);

    // tell the controller to export its options:
    itsAG->exportOptions(MC_RECURSE);

    // some info message:
    LINFO("Selected AG of type %s", itsAGtype.getVal().c_str());
  }
}

// ######################################################################
// ######################################################################
// ########## AttentionGateStd implementation
// ######################################################################
// ######################################################################

// ######################################################################
AttentionGateStd::
AttentionGateStd(OptionManager& mgr,
                 const std::string& descrName,
                 const std::string& tagName) :
  AttentionGate(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventInputFrame),
  SIMCALLBACK_INIT(SimEventAttentionGuidanceMapOutput),
  SIMCALLBACK_INIT(SimEventSaveOutput),
  itsSegmentDone(false),
  itsAGStageOneType(&OPT_AttentionGateStageOneType, this),
  itsAGStageTwoType(&OPT_AttentionGateStageTwoType, this),
  itsAGStageTwoEpochs(&OPT_AttentionGateStageTwoEpochs, this),
  itsMaxStageTwoFrames(10),
  itsStageTwoGetFeatures(AG_MAX),
  itsT(SimTime::ZERO()), itsTimeStep(SimTime::SECS(0.0001)),
  itsC(1.0), itsLeak(5000.0)
{ }

// ######################################################################
AttentionGateStd::~AttentionGateStd()
{ }

// ######################################################################
void AttentionGateStd::
onSimEventSaveOutput(SimEventQueue& q, rutz::shared_ptr<SimEventSaveOutput>& e)
{
  //LINFO("Run Attention Gate");
  // now post an event with our output:
  LINFO("Save Attention Gate");
  Image<float> ag  = this->getValue(q);
  Image<float> lam = this->getLastAttentionMap();
  Image<float> cam = this->getCurrentAttentionMap();

  if (ag.initialized())
    {
      //LINFO("Posting Attention Gate");
      ag = normalizeFloat(ag,FLOAT_NORM_0_255)/255.0F;
      ag = logSig(ag,itsLogSigO,itsLogSigS) * 255.0F;

      lam = normalizeFloat(lam,FLOAT_NORM_0_255)/255.0F;
      lam = logSig(lam,itsLogSigO,itsLogSigS) * 255.0F;

      cam = normalizeFloat(cam,FLOAT_NORM_0_255)/255.0F;
      cam = logSig(cam,itsLogSigO,itsLogSigS) * 255.0F;

      rutz::shared_ptr<SimEventAttentionGateOutput>
        ago(new SimEventAttentionGateOutput(this, ag, lam, cam, itsFrameNumber-1));
      q.post(ago);


      if (itsSaveResults.getVal())
        {
          // get the OFS to save to, assuming sinfo is of type
          // SimModuleSaveInfo (will throw a fatal exception otherwise):
          nub::ref<FrameOstream> ofs = dynamic_cast<const SimModuleSaveInfo&>(e->sinfo()).ofs;

          Image<float> lout = rescale(lam,itsSizeX,itsSizeY);

          // Save a masked copy of the input frame
          if(itsLastFrame.initialized())
            {
              Image<float> maskImage = normalizeFloat(lout,FLOAT_NORM_0_255)/255.0F;
              //maskImage = logSig(maskImage,0.25,10.0);
              Image<PixRGB<float> > outImageFloat = itsLastFrame * maskImage;
              Image<PixRGB<byte> >  outImage      = outImageFloat;

              ofs->writeRGB(outImage, "AG-LMASK", FrameInfo("Masked Image", SRC_POS));
            }


          if (ag.initialized())
            {
              const Image<float> out = rescale(ag,itsSizeX,itsSizeY);
              ofs->writeFloat(out, FLOAT_NORM_PRESERVE,
                              "AG",
                              FrameInfo("overall attention gate map", SRC_POS));
            }
          if(lout.initialized())
            {
              ofs->writeFloat(lout, FLOAT_NORM_PRESERVE,
                              "AG-LAM",
                              FrameInfo("last attention gate map", SRC_POS));
            }
          if(cam.initialized())
            {
              const Image<float> out = rescale(cam,itsSizeX,itsSizeY);
              ofs->writeFloat(out, FLOAT_NORM_PRESERVE,
                              "AG-CAM",
                              FrameInfo("current attention gate map", SRC_POS));
            }
          itsLastFrame = itsCurrFrame;
        }
    }
}

// ######################################################################
void AttentionGateStd::
onSimEventInputFrame(SimEventQueue& q, rutz::shared_ptr<SimEventInputFrame>& e)
{
  // get the actual input image/frame size from the world.
  // We will use this to scale output of saved files.
  if (itsSaveResults.getVal())
    {
      itsSizeX       = e->frame().getWidth();
      itsSizeY       = e->frame().getHeight();
      itsCurrFrame   = e->frame().asRgb();
      itsFrameNumber = (unsigned int)e->frameNum();
    }
}

// ######################################################################
void AttentionGateStd::
onSimEventAttentionGuidanceMapOutput(SimEventQueue& q, rutz::shared_ptr<SimEventAttentionGuidanceMapOutput>& e)
{
  Image<float> input = e->agm();

  if(!itsStageOneGate.initialized())
  {
    itsStageOneGate.resize(input.getWidth(),input.getHeight(),0.0F);
    itsStageTwoGate.resize(input.getWidth(),input.getHeight(),0.0F);
    itsCurrentAttentionMap.resize(input.getWidth(),input.getHeight(),0.0F);
    itsLastAttentionMap.resize(input.getWidth(),input.getHeight(),0.0F);
  }
  itsInput = input;

  if(itsAGStageTwoType.getVal().compare("None") != 0)
  {
    if(itsSegmentDone == false)
    {
      itsSegmenter.SIsetValThresh(std::vector<float>(1,256.0F),
                                  std::vector<float>(1,32.0F));
      itsSegmenter.SItoggleCandidateBandPass(false);
      int x = itsInput.getWidth();
      int y = itsInput.getHeight();
      itsSegmenter.SIsetFrame(&x,&y);
      itsSegmenter.SIsetAvg(10);
      itsSegmenter.SItoggleRemoveSingles(true);
      itsSegmenter.SIsetKillValue(1);
      itsSegmentDone = true;
    }
  }
}


// ######################################################################
void AttentionGateStd::reset1()
{
  itsStageOneGate.freeMem();
  itsStageTwoGate.freeMem();
  itsCurrentAttentionMap.freeMem();
  itsLastAttentionMap.freeMem();

  itsTotalEpochs = (uint)((itsAGStageTwoEpochs.getVal() * 2) + 1);
}

// ######################################################################
Image<float> AttentionGateStd::getValue(SimEventQueue& q)
{
  // Use the simple or complex stage one gate?
  if (itsAGStageOneType.getVal().compare("Simple") == 0)
    stageOneGateSimple(q);
  else if (itsAGStageOneType.getVal().compare("Complex") == 0)
    stageOneGateComplex(q);
  else
    {
      LINFO("Type of stage one attention gate given by");
      LFATAL("ag-so-type as '%s' is not valid", itsAGStageOneType.getVal().c_str());
    }

  if (itsAGStageTwoType.getVal().compare("Std") == 0)
    {
      stageTwoGate(q);
      return itsStageTwoGate;
    }
  else if (itsAGStageTwoType.getVal().compare("None") == 0)
    return itsStageOneGate;
  else
    {
      LINFO("Type of stage two attention gate given by");
      LFATAL("ag-st-type as '%s' is not valid", itsAGStageTwoType.getVal().c_str());
    }

  return itsStageOneGate;
}

// ######################################################################
void AttentionGateStd::stageOneGateSimple(SimEventQueue& q)
{
  const SimTime dt = SimTime::computeDeltaT((q.now() - itsT), itsTimeStep);

  //LINFO("Running stage one standard attention gate");

  // >>> Forward map operation (attention blocking)
  /* Here we use a leaky integrator map to create a forward mask
     which stores the blocking moving forward.
  */

  // The last attention map needs to be copied since it is no longer current
  itsLastAttentionMap    = itsCurrentAttentionMap;

  // Find what poked through
  const Image<float> poke = itsInput - itsStageOneGate;

  Image<float>::const_iterator p = poke.begin();
  Image<float>::iterator       a = itsCurrentAttentionMap.beginw();

  while(p != poke.end())
  {
    if(*p < 0) *a++ = 0;
    else       *a++ = *p;
    //LINFO("%f",*p);
    p++;
  }
  //LINFO("END");

  // leak the attention gate (yes it's hacky, but it works)
  const float delta       = (dt.secs() / itsC);
  const Image<float> leak = (itsStageOneGate * itsLeak);
  itsStageOneGate        -= leak * delta;

  // Update the attention gate as the max of itsStageOneGate and itsInput
  a = itsStageOneGate.beginw();
  Image<float>::iterator i = itsInput.beginw();

  // find the max and put in a
  while(a != itsStageOneGate.endw())
  {
    if(*a < *i) *a = *i;
    a++; i++;
  }

  // >>> Backward map operation (attention capture)
  /* Here we take the current attention map and push it backwards in time
     to see if we captured attention from the last frame.

     This method states that it is the amount of attention that pops through
     the gate which holds the full capture backwards. Thus, in order to capture
     backwards I must pop through the attention gate with more power than
     the last guy.

  */

  // Difference
  const Image<float> diff = itsLastAttentionMap - itsCurrentAttentionMap;

  // Clamp to >= 0
  Image<float>::const_iterator d = diff.begin();
  a = itsLastAttentionMap.beginw();
  while(d != diff.end())
  {
    if(*d < 0) *a++ = 0;
    else       *a++ = *d;
    d++;
  }

  itsT = q.now();
}

// ######################################################################
void AttentionGateStd::stageOneGateComplex(SimEventQueue& q)
{
  LFATAL("Under Construction");
}

// ######################################################################
void AttentionGateStd::stageTwoGate(SimEventQueue& q)
{
  // Stage Two, Integration
  /* This is tricky. In order to compute integration, feature maps must be
     stored for several epochs. This way we can backward compere items at
     Time (approx) -250 ms to the current frame to see if this frame enhances
     the frame that came before. Enhancement is considered if the features
     in the current frame have similar statistics such that they should enhance
     the other frame.

     Additionally, the first stage must be considered. It will gate what
     gets through. As such, only features that get through the first gate
     can enhance an image in the second stage.
  */

  //LFATAL("Under Construction");

  // Find the discrete attention regions in this image
  std::vector<Image<float> > input;
  float min,max,avg;
  getMinMaxAvg(itsLastAttentionMap,min,max,avg);
  Image<float> newImage = ((itsLastAttentionMap - min) / (max - min)) * 255.0f;
  input.push_back(newImage);
  itsSegmenter.SIsegment(&input,false);
  Image<bool> candidates = itsSegmenter.SIreturnCandidates();
  itsStageTwoSegments    = itsSegmenter.SIreturnBlobs();
  int         segnum     = itsSegmenter.SInumberBlobs();

  LINFO("Compute Min Max XY");
  computeMinMaxXY(itsLastAttentionMap,itsStageTwoSegments,candidates);

  LINFO("Extract Features");
  extractFeatureValues(q);

  LINFO("Compute Feature Distances");
  computeFeatureDistance();

  rutz::shared_ptr<SimEventAttentionGateStageTwoSegments>
    segs(new SimEventAttentionGateStageTwoSegments(this, candidates,
                                                   itsStageTwoObjects.back(),
                                                   segnum));

  q.post(segs);
}

// ######################################################################
void AttentionGateStd::computeMinMaxXY(const Image<float>& attentionMap,
                                       const Image<int>&   segments,
                                       const Image<bool>&  candidates)
{
  Image<float>::const_iterator attentionMapItr = attentionMap.begin();
  Image<int>::const_iterator   segmentsItr     = segments.begin();
  Image<bool>::const_iterator  candidatesItr   = candidates.begin();

  int maxObjects = 0;

  // How many potential objects do we have?
  while(candidatesItr != candidates.end())
  {
    if(*candidatesItr++)
    {
      if(*segmentsItr > maxObjects)
      {
        maxObjects = *segmentsItr;
      }
    }
    ++segmentsItr;
  }

  segmentsItr     = segments.begin();
  candidatesItr   = candidates.begin();

  itsStageTwoObjectX.resize(maxObjects+1,-1);
  itsStageTwoObjectY.resize(maxObjects+1,-1);
  itsStageTwoObjectVal.resize(maxObjects+1,0);
  itsStageTwoObjectID.resize(maxObjects+1,0);

  int       pos = 0;
  const int w   = candidates.getWidth();

  // For each segmented object, find its max value and location
  // according to the value of surprise.

  while(candidatesItr != candidates.end())
  {
    if(*candidatesItr++)
    {
      if(*attentionMapItr > itsStageTwoObjectVal[*segmentsItr])
      {
        itsStageTwoObjectVal[*segmentsItr] = *attentionMapItr;
        itsStageTwoObjectX[*segmentsItr]   = pos%w;
        itsStageTwoObjectY[*segmentsItr]   = pos/w;
        itsStageTwoObjectID[*segmentsItr]  = *segmentsItr;
      }
    }
    pos++; ++attentionMapItr; ++segmentsItr;
  }
}

// ######################################################################
void AttentionGateStd::extractFeatureValues(SimEventQueue& q)
{
  std::vector<int>::iterator stageTwoObjectXItr   = itsStageTwoObjectX.begin();
  std::vector<int>::iterator stageTwoObjectYItr   = itsStageTwoObjectY.begin();
  std::vector<int>::iterator stageTwoObjectIDItr  = itsStageTwoObjectID.begin();
  std::vector<float>::iterator stageTwoObjectValItr = itsStageTwoObjectVal.begin();

  const float h = (float)itsLastAttentionMap.getHeight();
  const float w = (float)itsLastAttentionMap.getWidth();

  std::vector<std::vector<float> > featureObjects;
  std::vector<int>                 featureX;
  std::vector<int>                 featureY;
  std::vector<int>                 featureID;
  std::vector<float>               featureVal;

  rutz::shared_ptr<SimReqVCXmaps> vcxm(new SimReqVCXmaps(this));
  q.request(vcxm); // VisualCortex is now filling-in the maps...
  rutz::shared_ptr<ChannelMaps> chm = vcxm->channelmaps();

  const uint numSubmaps = chm->numSubmaps();

  int n = 0;

  while(stageTwoObjectYItr != itsStageTwoObjectY.end())
  {
    const float i         = (float)*stageTwoObjectXItr++;
    const float j         = (float)*stageTwoObjectYItr++;
    const float val       = (float)*stageTwoObjectValItr++;
    const int   ID        =        *stageTwoObjectIDItr++;

    std::vector<float> features;

    // check to make sure we took in a value
    if(i >= 0.0f)
    {
      // Get the feature from the most surprising location. If a feature
      // map is larger, take the feature at the center of the overlap
      if(itsStageTwoGetFeatures == AG_CENTER)
      {
        //LINFO("Using %f , %f",i,j);
        for (uint k = 0; k < numSubmaps; k++)
        {
          const Image<float> submap     = chm->getRawCSmap(k);
          const float        sm_w       = (float)submap.getWidth();
          const float        sm_h       = (float)submap.getHeight();
          // Adjust location for different sized maps
          const int          sm_i       = (int)floor(i   * (sm_w/w) +
                                                     0.5 * (sm_w/w));
          const int          sm_j       = (int)floor(j   * (sm_h/h) +
                                                     0.5 * (sm_h/h));
          //LINFO("Getting w %f h %f - %d , %d",sm_w,sm_h,sm_i,sm_j);
          const float        featureVal = submap.getVal(sm_i,sm_j);
          features.push_back(featureVal);
        }
      }
      // Take the max feature from the location if a feature map is larger
      // than the surprise map
      else if(itsStageTwoGetFeatures == AG_MAX)
      {
        //LINFO("Using %f , %f",i,j);
        for (uint k = 0; k < numSubmaps; k++)
        {
          const Image<float> submap     = chm->getRawCSmap(k);
          const float        sm_w       = (float)submap.getWidth();
          const float        sm_h       = (float)submap.getHeight();

          // Are the feature submap and AG map the same size?
          if(sm_w == w)
          {
            const float featureVal = submap.getVal((int)i,(int)j);
            features.push_back(featureVal);
          }
          // is the feature submap larger?
          else if(sm_w > w)
          {
            // Compute the bounds high/low of i and j
            const int l_i = (int)floor(i *  (sm_w/w));
            const int h_i = l_i + (int)round(sm_w/w);

            const int l_j = (int)floor(j *  (sm_h/h));
            const int h_j = l_j + (int)round(sm_h/h);
            // get the max feature from within the bounds
            float maxVal = 0;

            for(int u = l_i; u < h_i; u++)
            {
              for(int v = l_j; v < h_j; v++)
              {
                const float featureVal = submap.getVal(u,v);
                if(featureVal > maxVal)
                  maxVal = featureVal;
                //LINFO("Getting u %d v %d - %f , %f - Val %f",
                //      u,v,i,j,featureVal);
              }
            }
            features.push_back(maxVal);
          }
          // is the feature submap smaller?
          else
          {
            const int sm_i = (int)floor(i   * (sm_w/w) +
                                        0.5 * (sm_w/w));
            const int sm_j = (int)floor(j   * (sm_h/h) +
                                        0.5 * (sm_h/h));
            const float        featureVal = submap.getVal(sm_i,sm_j);
            features.push_back(featureVal);
          }
        }
      }
      else
      {
        LFATAL("Unknown method given for stage two feature extraction");
      }
      // push back details of this object
      featureObjects.push_back(features);
      featureX.push_back((int)i);
      featureY.push_back((int)j);
      featureID.push_back(ID);
      featureVal.push_back(val);
      n++;
    }
  }

  // Create the container and store this frames stats in it
  SimEventAttentionGateStageTwoObjects stObjects;
  stObjects.features = featureObjects;       // features of objects
  stObjects.segments = itsStageTwoSegments;  // Segment image of objects
  stObjects.x        = featureX;             // X position of objects
  stObjects.y        = featureY;             // Y position of objects
  stObjects.id       = featureID;            // ID of objects
  stObjects.val      = featureVal;           // Surprise Value of Objects
  stObjects.n        = n;                    // Number of Objects

  // store the object features from this frame
  itsStageTwoObjects.push_back(stObjects);

  // trim to keep bounded in size (smaller than n frames)
  if(itsStageTwoObjects.size() > itsMaxStageTwoFrames)
    itsStageTwoObjects.pop_front();

}

// ######################################################################
void AttentionGateStd::computeFeatureDistance()
{
  // NOTE: this does not take into account whether objects overlap
  // that comes later.

  // figure out how many objects we are dealing with
  int otherObjects = 0;

  // make sure we have more than one frame of samples
  if(itsStageTwoObjects.size() > 1)
  {

    for(std::deque<SimEventAttentionGateStageTwoObjects>::iterator
          stageTwoObjectsItr = itsStageTwoObjects.begin();
        stageTwoObjectsItr != (itsStageTwoObjects.end() - 2);
        ++stageTwoObjectsItr)
    {
      otherObjects += stageTwoObjectsItr->n;
    }

    // create a matrix to hold the feature distances between objects
    Image<float> fdMatrix;
    const int thisObjects = (itsStageTwoObjects.back()).n;
    fdMatrix.resize(otherObjects,thisObjects);
    Image<float>::iterator fdMatrixItr = fdMatrix.beginw();

    std::vector<std::vector<float> >::iterator featuresA =
      (itsStageTwoObjects.back()).features.begin();

    while(featuresA != (itsStageTwoObjects.back()).features.end())
    {
      std::deque<SimEventAttentionGateStageTwoObjects>::iterator
        stageTwoObjectsBItr = itsStageTwoObjects.begin();
      //LINFO("THIS NEW OBJECT");
      //int n = 0;
      // Don't include our own objects
      while(stageTwoObjectsBItr != (itsStageTwoObjects.end() - 2))
      {
        std::vector<std::vector<float> >::iterator featuresB =
          stageTwoObjectsBItr->features.begin();
        //LINFO("THIS NEW SET");

        while(featuresB != stageTwoObjectsBItr->features.end())
        {
          // compute features distance between objects
          std::vector<float>::iterator fa = featuresA->begin();
          std::vector<float>::iterator fb = featuresB->begin();
          float ss = 0.0f;
          // take the euclidian sum of square distance
          while(fa != featuresA->end())
          {
            ss += pow(*fa++ - *fb++ ,2);
          }
          // normalize the distance since we can have different
          // number of features
          *fdMatrixItr++ = sqrt(ss/featuresA->size());
          //LINFO("Mat %d : %f",n++,*(fdMatrixItr - 1));
          ++featuresB;
        }
        ++stageTwoObjectsBItr;
      }
      ++featuresA;
    }

    // store the distance matrix at the end
    (itsStageTwoObjects.end() - 1)->fdistance = fdMatrix;
  }
}


// ######################################################################
Image<float> AttentionGateStd::getLastAttentionMap() const
{
  return itsLastAttentionMap;
}

// ######################################################################
Image<float> AttentionGateStd::getCurrentAttentionMap() const
{
  return itsCurrentAttentionMap;
}
// ######################################################################
// ######################################################################
// ########## AttentionGateStub implementation
// ######################################################################
// ######################################################################

// ######################################################################
AttentionGateStub::
AttentionGateStub(OptionManager& mgr,
                        const std::string& descrName,
                        const std::string& tagName) :
  AttentionGate(mgr, descrName, tagName)
{ }

// ######################################################################
AttentionGateStub::~AttentionGateStub()
{ }

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
