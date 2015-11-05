/*!@file Neuro/TaskRelevanceMap.C Implementation for task-relevance map class */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/TaskRelevanceMap.C $
// $Id: TaskRelevanceMap.C 14390 2011-01-13 20:17:22Z pez $
//

#include "Neuro/TaskRelevanceMap.H"

#include "Channels/ChannelOpts.H" // for OPT_LevelSpec
#include "Channels/InputFrame.H"
#include "Component/OptionManager.H"
#include "Image/Image.H"
#include "Image/MathOps.H"   // for absDiff()
#include "Image/ShapeOps.H"  // for rescale(), downSize()
#include "Image/CutPaste.H"   // for concatX()
#include "Image/Transforms.H"
#include "Image/DrawOps.H" // for drawFilledPolygon()
#include "Image/MathOps.H"
#include "Image/MatrixOps.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/NeuroSimEvents.H"
#include "Neuro/SaccadeController.H"
#include "Raster/Raster.H"
#include "Simulation/SimEventQueue.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameOstream.H"
#include "Util/Types.H"
#include "Util/log.H"
#include "Util/StringConversions.H"
#include "Util/StringUtil.H" // for toLowerCase()

#include "Simulation/SimEventQueueConfigurator.H"
#include "Neuro/GistEstimator.H"
#include "Neuro/gistParams.H"
#include "GUI/XWinManaged.H"
#include <vector>

#include <cstdio>

#define PI 3.141592

// ######################################################################
// ######################################################################
// ########## TaskRelevanceMap implementation
// ######################################################################
// ######################################################################

TaskRelevanceMap::TaskRelevanceMap(OptionManager& mgr,
                                   const std::string& descrName,
                                   const std::string& tagName) :
  SimModule(mgr, descrName, tagName)
{ }

// ######################################################################
TaskRelevanceMap::~TaskRelevanceMap()
{ }

// ######################################################################
// ######################################################################
// ########## TaskRelevanceMapConfigurator implementation
// ######################################################################
// ######################################################################
TaskRelevanceMapConfigurator::
TaskRelevanceMapConfigurator(OptionManager& mgr,
                             const std::string& descrName,
                             const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsTRMtype(&OPT_TaskRelevanceMapType, this),
  itsTRM(new TaskRelevanceMapStub(mgr))
{
  addSubComponent(itsTRM);
}

// ######################################################################
TaskRelevanceMapConfigurator::~TaskRelevanceMapConfigurator()
{  }

// ######################################################################
nub::ref<TaskRelevanceMap> TaskRelevanceMapConfigurator::getTRM() const
{ return itsTRM; }

// ######################################################################
void TaskRelevanceMapConfigurator::
paramChanged(ModelParamBase* const param,
             const bool valueChanged,
             ParamClient::ChangeStatus* status)
{
  ModelComponent::paramChanged(param, valueChanged, status);

  // was that a change of our baby's name?
  if (param == &itsTRMtype) {
    // if we had one, let's unregister it (when we later reset() the
    // nub::ref, the current TaskRelevanceMap will unexport its
    // command-line options):
    removeSubComponent(*itsTRM);

    // instantiate a SM of the appropriate type:
    if (itsTRMtype.getVal().compare("None") == 0 ||
        itsTRMtype.getVal().compare("Stub") == 0) // no TRM
      itsTRM.reset(new TaskRelevanceMapStub(getManager()));
    else if (itsTRMtype.getVal().compare("Std") == 0)          // standard
      itsTRM.reset(new TaskRelevanceMapStd(getManager()));
    else if (itsTRMtype.getVal().compare("KillStatic") == 0)   // kill-static
      itsTRM.reset(new TaskRelevanceMapKillStatic(getManager()));
    else if (itsTRMtype.getVal().compare("KillN") == 0)        // kill-n
      itsTRM.reset(new TaskRelevanceMapKillN(getManager()));
    else if (itsTRMtype.getVal().compare("GistClassify") == 0) // gist-classify
      itsTRM.reset(new TaskRelevanceMapGistClassify(getManager()));
    else if (itsTRMtype.getVal().compare("Tigs") == 0) // Tigs
      itsTRM.reset(new TaskRelevanceMapTigs(getManager()));
    else if (itsTRMtype.getVal().compare("Tigs2") == 0) // Tigs(gist and pca image)
      itsTRM.reset(new TaskRelevanceMapTigs2(getManager()));
    else if (itsTRMtype.getVal().compare("Social") == 0) // Social(requires XML of data)
      itsTRM.reset(new TaskRelevanceMapSocial(getManager()));
    else
      LFATAL("Unknown TRM type %s", itsTRMtype.getVal().c_str());

    // add our baby as a subcomponent of us so that it will become
    // linked to the manager through us (hopefully we are registered
    // with the manager), which in turn will allow it to export its
    // command-line options and get configured:

    addSubComponent(itsTRM);

    // tell the controller to export its options:
    itsTRM->exportOptions(MC_RECURSE);

    // some info message:
    LINFO("Selected TRM of type %s", itsTRMtype.getVal().c_str());
  }
}


// ######################################################################
// ######################################################################
// ########## TaskRelevanceMapStub implementation
// ######################################################################
// ######################################################################

// ######################################################################
TaskRelevanceMapStub::TaskRelevanceMapStub(OptionManager& mgr,
                                           const std::string& descrName,
                                           const std::string& tagName) :
  TaskRelevanceMap(mgr, descrName, tagName)
{ }

// ######################################################################
TaskRelevanceMapStub::~TaskRelevanceMapStub()
{ }


// ######################################################################
// ######################################################################
// ########## TaskRelevanceMapAdapter implementation
// ######################################################################
// ######################################################################
TaskRelevanceMapAdapter::
TaskRelevanceMapAdapter(OptionManager& mgr,
                        const std::string& descrName,
                        const std::string& tagName) :
  TaskRelevanceMap(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventRetinaImage),
  SIMCALLBACK_INIT(SimEventSaccadeStatusEye),
  SIMCALLBACK_INIT(SimEventClockTick),
  SIMCALLBACK_INIT(SimEventSaveOutput),
  itsLevelSpec(&OPT_LevelSpec, this), //Channels/ChannelOpts.{H,C}
  itsSaveResults(&OPT_TRMsaveResults, this) // see Neuro/NeuroOpts.{H,C}
{ }

// ######################################################################
TaskRelevanceMapAdapter::~TaskRelevanceMapAdapter()
{ }

// ######################################################################
void TaskRelevanceMapAdapter::reset1()
{ itsMap.freeMem(); }

// ######################################################################
void TaskRelevanceMapAdapter::
onSimEventRetinaImage(SimEventQueue& q, rutz::shared_ptr<SimEventRetinaImage>& e)
{
  const Dims d = e->frame().colorByte().getDims();
  const int sml = itsLevelSpec.getVal().mapLevel();

  const Dims mapdims(d.w() >> sml, d.h() >> sml);

  // great, here is a new input image. Initialize our map if needed:
  if (itsMap.getDims() != mapdims)
    {
      itsMap.resize(mapdims,true);
      itsMap.clear(1.0F); // neutral relevance
    }

  // now do any implementation-specific processing:
  this->inputFrame(e->frame());
}

// ######################################################################
void TaskRelevanceMapAdapter::
onSimEventSaccadeStatusEye(SimEventQueue& q, rutz::shared_ptr<SimEventSaccadeStatusEye>& e)
{
  if (e->saccadeStatus() == TSTATUS_BEGIN) this->saccadicSuppression(true);
  else if (e->saccadeStatus() == TSTATUS_END) this->saccadicSuppression(false);
}

// ######################################################################
void TaskRelevanceMapAdapter::
onSimEventClockTick(SimEventQueue& q, rutz::shared_ptr<SimEventClockTick>& e)
{
  // evolve one time step:
  this->integrate(q);

  // all right, post our current internals if we have any:
  if (itsMap.initialized())
    {
      rutz::shared_ptr<SimEventTaskRelevanceMapOutput>
        etrm(new SimEventTaskRelevanceMapOutput(this, itsMap));
      q.post(etrm);
    }
}

// ######################################################################
void TaskRelevanceMapAdapter::
onSimEventSaveOutput(SimEventQueue& q, rutz::shared_ptr<SimEventSaveOutput>& e)
{
  this->save1(e->sinfo());
}

// ######################################################################
void TaskRelevanceMapAdapter::save1(const ModelComponentSaveInfo& sinfo)
{
  if (itsSaveResults.getVal())
    {
      // get the OFS to save to, assuming sinfo is of type
      // SimModuleSaveInfo (will throw a fatal exception otherwise):
      nub::ref<FrameOstream> ofs =
        dynamic_cast<const SimModuleSaveInfo&>(sinfo).ofs;

      ofs->writeFloat(itsMap, FLOAT_NORM_PRESERVE, "TRM",
                      FrameInfo("task relevance map (top-down)", SRC_POS));
    }
}

// ######################################################################
// ######################################################################
// ########## TaskRelevanceMapStd implementation
// ######################################################################
// ######################################################################

// ######################################################################
TaskRelevanceMapStd::
TaskRelevanceMapStd(OptionManager& mgr, const std::string& descrName,
                    const std::string& tagName) :
  TaskRelevanceMapAdapter(mgr, descrName, tagName),
  itsLearnTRM(&OPT_LearnTRM, this),
  itsLearnUpdateTRM(&OPT_LearnUpdateTRM, this),
  itsBiasTRM(&OPT_BiasTRM, this)
{ }

// ######################################################################
TaskRelevanceMapStd::~TaskRelevanceMapStd()
{ }

// ######################################################################
void TaskRelevanceMapStd::inputFrame(const InputFrame& f)
{ }

/*FIXME
void TaskRelevanceMapStd::input(const Image<float>& current)
{
  itsMap = current;
  const Dims d = itsMap.getDims();

  // bias TRM if the option is specified
  if (strcmp(itsBiasTRM.getValString().c_str(), "") != 0)
    {
      LINFO("biasing TRM with %s", itsBiasTRM.getValString().c_str());
      const Image<float> newtrm =
        Raster::ReadGray(itsBiasTRM.getValString());
      itsMap = rescale(newtrm * (1.0F/255.0F), d);
    }
}
*/

// ######################################################################
void TaskRelevanceMapStd::saccadicSuppression(const bool on)
{ }

// ######################################################################
void TaskRelevanceMapStd::integrate(SimEventQueue& q)
{ }

/*FIXME
void TaskRelevanceMapStd::update(const Point2D<int>& fixation,
                                 const Image<byte>& objectMask,
                                 const Dims& inputDims,
                                 const float relevance)
{
  // get the object shape:
  Point2D<int> fixn;
  fixn.i = fixation.i * itsMap.getWidth() / inputDims.w();
  fixn.j = fixation.j * itsMap.getHeight() / inputDims.h();
  if ( ! objectMask.initialized())
    { LERROR("object mask not initialized!"); return; }
  Image<byte> fmask = rescale(objectMask, itsMap.getDims());
  Point2D<int> p;

  //FIXME: this should be an Image function
  // update relevance only within object
  for (p.i = 0; p.i < itsMap.getWidth(); p.i ++)
    for (p.j = 0; p.j < itsMap.getHeight(); p.j ++)
      if (fmask.getVal(p) == 255)
        {
          if (relevance == 0.0f) itsMap.setVal(p, itsMap.getVal(p) - 1.0f);
          else itsMap.setVal(p, itsMap.getVal(p) + relevance);
        }

  if (itsLearnTRM.getVal() == false)
    {
      // biasing mode / updating mode / none:
      // set min to 0 else normalizing will clear the entire TRM
      float min, max; getMinMax(itsMap, min, max);
      if (min > 0.0f) itsMap.setVal(0, 0, 0.0f);
    }

  // normalize TRM else AGM will explode
  inplaceNormalize(itsMap, 0.0f, 3.0f);
  LINFO("Evolved TRM with relevance = %f", relevance);
}
*/

// ######################################################################
// ######################################################################
// ########## TaskRelevanceMapKillStatic implementation
// ######################################################################
// ######################################################################

// ######################################################################
TaskRelevanceMapKillStatic::
TaskRelevanceMapKillStatic(OptionManager& mgr,
                           const std::string& descrName,
                           const std::string& tagName) :
  TaskRelevanceMapAdapter(mgr, descrName, tagName),
  itsKillStaticThresh(&OPT_TRMKillStaticThresh, this),
  itsKillStaticCoeff(&OPT_TRMKillStaticCoeff, this),
  itsStaticBuff()
{ }

// ######################################################################
TaskRelevanceMapKillStatic::~TaskRelevanceMapKillStatic()
{ }

// ######################################################################
void TaskRelevanceMapKillStatic::reset1()
{
  TaskRelevanceMapAdapter::reset1();
  itsStaticBuff.freeMem();
}

// ######################################################################
void TaskRelevanceMapKillStatic::inputFrame(const InputFrame& f)
{
  // NOTE: we are guaranteed that itsMap is initialized and of correct
  // size, as this is done by the TaskRelevanceMapAdapter before
  // calling us.

  // NOTE: this is duplicating some of the computation done in the
  // IntensityChannel, so there is a tiny performance hit (~0.25%) in
  // doing this operation twice; however the benefit is that we avoid
  // having to somehow pick information out of an IntensityChannel and
  // pass it to a TaskRelevanceMap -- that was making for a mess in
  // Brain. In the future, we could avoid the ineffiency by caching
  // the intensity pyramid in the InputFrame object, and then letting
  // IntensityChannel and TaskRelevanceMap both share access to that
  // pyramid.
  const Image<float> img =
    downSize(f.grayFloat(), itsMap.getWidth(), itsMap.getHeight(), 5);

  // is this our first time here?
  if (itsStaticBuff.initialized() == false)
    { itsStaticBuff = img; return; }

  // otherwise derive TRM from difference between current frame and
  // staticBuf:
  Image<float> diff = absDiff(itsStaticBuff, img);

  // useful range of diff is 0..255; anything giving an output less
  // that itsKillStaticThresh will be considered static; here let's
  // clamp to isolate that:
  inplaceClamp(diff, 0.0F, itsKillStaticThresh.getVal());

  // the more static, the greater the killing: so, first let's change
  // the range so that the min is at -itsKillStaticThresh and zero is
  // neutral:
  diff -= itsKillStaticThresh.getVal();

  // itsKillStaticCoeff determines how strong the killing should be:
  diff *= (1.0F / itsKillStaticThresh.getVal()) * // normalize to min at -1.0
    itsKillStaticCoeff.getVal();                  // apply coeff

  // mix our new TRM to the old one; we want to have a fairly high
  // mixing coeff for the new one so that we don't penalize recent
  // onset objects too much (i.e., were previously static and killed,
  // but are not anymore):
  itsMap = itsMap * 0.25F + (diff + 1.0F) * 0.75F;
  float mi, ma; getMinMax(itsMap, mi, ma);
  LINFO("TRM range = [%.2f .. %.2f] -- 1.0 is baseline", mi, ma);

  // update our cumulative buffer:
  itsStaticBuff = itsStaticBuff * 0.75F + img * 0.25F;

}

// ######################################################################
void TaskRelevanceMapKillStatic::saccadicSuppression(const bool on)
{
  // a saccade resets our cumulative buffer:
  itsStaticBuff.freeMem();
}

// ######################################################################
void TaskRelevanceMapKillStatic::integrate(SimEventQueue& q)
{ }

// ######################################################################
void TaskRelevanceMapKillStatic::save1(const ModelComponentSaveInfo& sinfo)
{
  TaskRelevanceMapAdapter::save1(sinfo);

  if (itsSaveResults.getVal())
    {
      // get the OFS to save to, assuming sinfo is of type
      // SimModuleSaveInfo (will throw a fatal exception otherwise):
      nub::ref<FrameOstream> ofs =
        dynamic_cast<const SimModuleSaveInfo&>(sinfo).ofs;

      ofs->writeFloat(itsStaticBuff, FLOAT_NORM_0_255, "TRM-SB",
                      FrameInfo("task relevance map static buffer", SRC_POS));
    }
}


// ######################################################################
// ######################################################################
// ########## TaskRelevanceMapKillN implementation
// ######################################################################
// ######################################################################

// ######################################################################
TaskRelevanceMapKillN::
TaskRelevanceMapKillN(OptionManager& mgr, const std::string& descrName,
                      const std::string& tagName) :
  TaskRelevanceMapAdapter(mgr, descrName, tagName),
  itsN(&OPT_TRMkillN, this), itsCache()
{ }

// ######################################################################
TaskRelevanceMapKillN::~TaskRelevanceMapKillN()
{ }

// ######################################################################
void TaskRelevanceMapKillN::reset1()
{
  TaskRelevanceMapAdapter::reset1();
  itsCache.clear();
}

// ######################################################################
void TaskRelevanceMapKillN::start1()
{
  itsCache.setMaxSize(itsN.getVal());

  TaskRelevanceMapAdapter::start1();
}

// ######################################################################
void TaskRelevanceMapKillN::inputFrame(const InputFrame& f)
{ }

// ######################################################################
void TaskRelevanceMapKillN::integrate(SimEventQueue& q)
{
  // anything from the ShapeEstimator?
  if (SeC<SimEventShapeEstimatorOutput> e =
      q.check<SimEventShapeEstimatorOutput>(this))
    {
      // get the smooth object mask:
      Image<float> mask = e->smoothMask();

      // downscale to our map's size:
      mask = downSize(mask, itsMap.getWidth(), itsMap.getHeight(), 5);

      // is the mask uniform (i.e., empty)?  In this case, just push
      // it in, otherwise let's segment the objects out:
      float mi, ma; getMinMax(mask, mi, ma);
      if (fabs(mi - ma) < 1.0e-10)
        itsCache.push_back(mask);
      else
        {
          // let's build a distance map out of the object and threhold it:
          Image<float> dmap = chamfer34(mask, 255.0F);
          inplaceClamp(dmap, 0.0F, 15.0F);
          dmap /= 15.0F;  // 0 inside obj, 1 outside

          // get this new input mask into our cache:
          itsCache.push_back(dmap);
        }

      // our current relevance map is the min over our cache:
      itsMap = itsCache.getMin();
    }
}

// ######################################################################
void TaskRelevanceMapKillN::saccadicSuppression(const bool on)
{ }

// ######################################################################
// ######################################################################
// ########## TaskRelevanceMapGistClassify implementation
// ######################################################################
// ######################################################################
// ######################################################################
TaskRelevanceMapGistClassify::
TaskRelevanceMapGistClassify(OptionManager& mgr, const std::string& descrName,
                      const std::string& tagName) :
  TaskRelevanceMapAdapter(mgr, descrName, tagName),
  itsClusterCenterFileName(&OPT_TRMClusterCenterFile, this),
  itsTemplateDir(&OPT_TRMTemplateDir, this),
  itsPCADims(&OPT_TRMGistPCADims, this),
  itsPCAMatrixName(&OPT_TRMGistPCAMatrix, this),
  itsCacheSize(&OPT_TRMCacheSize, this),
  itsUpdatePeriod(&OPT_TRMUpdatePeriod, this),
  itsMapComputedForCurrentFrame(false),
  itsTDMap()
{
  itsFrame = -1;
  itsTDMap.setMaxSize(itsCacheSize.getVal());
}

// ######################################################################
TaskRelevanceMapGistClassify::~TaskRelevanceMapGistClassify()
{ }

// ######################################################################
void TaskRelevanceMapGistClassify::reset1()
{
  TaskRelevanceMapAdapter::reset1();
}

// ######################################################################
void TaskRelevanceMapGistClassify::inputFrame(const InputFrame& f)
{
  getPCAMatrix();
  itsFrame++;
  itsMapComputedForCurrentFrame = false;
}

// ######################################################################
void TaskRelevanceMapGistClassify::integrate(SimEventQueue& q)
{
  if (SeC<SimEventGistOutput> e =
      q.check<SimEventGistOutput>(this))
    {
      if (itsMapComputedForCurrentFrame)
        return;

      //! to make the gist feature value in a reasonable range to get the
      //  variance and determine value
      itsGist = e->gv() / 10.0F;

      gistmatch(reshape(itsGist,
                        Dims(itsGist.getWidth() * itsGist.getHeight(), 1)));

      if (itsCacheSize.getVal() > 0)
        {
          itsTDMap.push_back(itsTmpTDMap);
          if (itsFrame % itsUpdatePeriod.getVal() ==0)
            itsCurrentTDMap = itsTDMap.mean();
        }
      else
        itsCurrentTDMap = itsTmpTDMap;


      itsMap = rescale(itsCurrentTDMap, itsMap.getDims());

      float mi, ma; getMinMax(itsMap, mi, ma);
      LINFO("\nFinal TRM range = [%.2f .. %.2f] -- 1.0 is baseline\n", mi, ma);

      itsMapComputedForCurrentFrame = true;
    }
}

// ######################################################################
void TaskRelevanceMapGistClassify::saccadicSuppression(const bool on)
{ }

// ######################################################################
void TaskRelevanceMapGistClassify::
gistmatch(Image<float> currGist)
{
  Image<float> gistDist =   computeGistDist(currGist);
  itsTmpTDMap = getTDMap(gistDist);

}
// ######################################################################
void TaskRelevanceMapGistClassify::
getPCAMatrix()
{
  FILE* itsFile = fopen(itsPCAMatrixName.getVal().c_str(), "rb");
  ASSERT(itsFile != 0);

  int gistDims = 0;
  if(fread(&gistDims, sizeof(int), 1, itsFile) != 1) LFATAL("fread failed");

  Image<float> matrixTmp(gistDims, itsPCADims.getVal(), NO_INIT);
  size_t sz = itsPCADims.getVal()*gistDims;
  if(fread(matrixTmp.beginw(), sizeof(float), sz, itsFile) != sz) LFATAL("fread failed"); 

  itsPCAMatrix = transpose(matrixTmp);

  LDEBUG("itsPCAMatrix first 5 num: %f, %f, %f, %f, %f", itsPCAMatrix.getVal(0,0),
        itsPCAMatrix.getVal(1,0), itsPCAMatrix.getVal(2,0),
        itsPCAMatrix.getVal(3,0), itsPCAMatrix.getVal(4,0));

  fclose(itsFile);
}



// ######################################################################
Image<float> TaskRelevanceMapGistClassify::
computeGistDist(Image<float> currGist)
{
   FILE* itsFile = fopen(itsClusterCenterFileName.getVal().c_str(), "rb");
   ASSERT(itsFile != 0);

  if(fread(&itsNumOfCategory, sizeof(int), 1, itsFile) != 1) LFATAL("fread failed");
  if(fread(&itsUsedDims, sizeof(int),1,itsFile) != 1) LFATAL("fread failed");
  ASSERT(itsNumOfCategory > 0 && itsUsedDims == itsPCADims.getVal());

  LDEBUG("there are %4d categories, pca_dims: %4d",itsNumOfCategory, itsUsedDims);

  Image<float> currGistCut(currGist.getDims().sz()/NUM_GIST_FEAT, 1, NO_INIT);
  Image<float> currGistPCA(itsPCADims.getVal(), 1, NO_INIT);

  //! only use the whole image's gist feature to do the classification
  for(int i=0; i<currGistCut.getDims().sz(); i++)
    currGistCut.setVal(i,0, currGist.getVal(i*NUM_GIST_FEAT, 0));

  LDEBUG("currGistCut dim: %d, PCA matrix height: %4d",
        currGistCut.getWidth(), itsPCAMatrix.getHeight());

  ASSERT(currGistCut.getWidth() == itsPCAMatrix.getHeight());

  currGistPCA = matrixMult(currGistCut, itsPCAMatrix);

  LDEBUG("currGistPCA : %f, %f, %f", currGistPCA.getVal(0,0),
         currGistPCA.getVal(1,0), currGistPCA.getVal(2,0));

  Image<float> gistDist(itsNumOfCategory, 1,NO_INIT );
  Image<float> gistCenter(itsUsedDims, 1, NO_INIT);
  Image<float> gistCenterCovarMatrix(itsUsedDims,itsUsedDims, NO_INIT);
  Image<float> gistCenterCovarMatrixInv(itsUsedDims, itsUsedDims, NO_INIT);
  Image<float> gistDiff(itsUsedDims,1,NO_INIT);

  float meanClusterGistDist=0.0F;
  float det = 0.0F;
  float coef1 = pow(2*PI, itsUsedDims/2.0F);
  float coef2 = 0.0F, coef3=0.0F, coef4=0.0F;

  for(int i=0; i<itsNumOfCategory; i++)
    {
      if(fread(gistCenter.beginw(), sizeof(float), itsUsedDims, itsFile) != (size_t)itsUsedDims) LFATAL("fread failed");
      if(fread(&meanClusterGistDist, sizeof(float), 1, itsFile) != 1) LFATAL("fread failed");
      if(fread(&det, sizeof(float), 1, itsFile) != 1) LFATAL("fread failed");
      size_t sz = itsUsedDims*itsUsedDims;
      if(fread(gistCenterCovarMatrix.beginw(), sizeof(float), sz, itsFile) != sz) LFATAL("fread failed");

      gistCenterCovarMatrixInv = matrixInv(gistCenterCovarMatrix);

      gistDiff = gistCenter - currGistPCA;

      coef2 =1.0F /( coef1 * pow(det, 0.5)+0.0000001F );

      Image<float> tmp = matrixMult(matrixMult(gistDiff,  gistCenterCovarMatrixInv),
                                    transpose(gistDiff));

      coef3 = exp(-0.5F * tmp.getVal(0,0));
      coef4 = coef2 * coef3 / (meanClusterGistDist+0.5F);

      LDEBUG("%d's is %f,  %f, %f , mean is %f,sumDiff is%f\n", i+1, tmp.getVal(0,0),
            coef3, coef4, meanClusterGistDist, sum(gistDiff*gistDiff));
      gistDist.setVal(i, 0, coef4);

    }
  fclose(itsFile);

  return gistDist;
}

// ######################################################################
Image<float> TaskRelevanceMapGistClassify::
getTDMap(Image<float> gistDist)
{
  std::string TDMapName;
  Image<float> TDMap;
  float coef = 0.0F;

  for(int i=1; i<= itsNumOfCategory; i++)
    {
      TDMapName = itsTemplateDir.getVal()+std::string("category")
        +convertToString(i)+std::string(".png");


      Image<float> TDMapTmp = Raster::ReadGray(TDMapName);

      if(1==i)
        TDMap = TDMapTmp * gistDist.getVal(i-1,0);
      else
        TDMap += TDMapTmp * gistDist.getVal(i-1,0);

      coef += gistDist.getVal(i-1,0);
    }

  TDMap = (TDMap + 0.0001F) / (coef + 0.0001F);
  // consider that failure in classify then use the uniform template

  float mi, ma; getMinMax(TDMap, mi, ma);
  LINFO("\ncoef= %.6f TRM range = [%.2f .. %.2f] -- 1.0 is baseline\n", coef, mi, ma);
  return TDMap;
}


// ######################################################################
// ######################################################################
// ########## TaskRelevanceMapTigs implementation
// ######################################################################
// ######################################################################
// ######################################################################
TaskRelevanceMapTigs::
TaskRelevanceMapTigs(OptionManager& mgr, const std::string& descrName,
                      const std::string& tagName) :
  TaskRelevanceMapAdapter(mgr, descrName, tagName),
  itsTigsMatrixName(&OPT_TRMTigsMatrix, this),
  itsPCADims(&OPT_TRMGistPCADims, this),
  itsPCAMatrixName(&OPT_TRMGistPCAMatrix, this),
  itsCacheSize(&OPT_TRMCacheSize, this),
  itsUpdatePeriod(&OPT_TRMUpdatePeriod, this),
  itsMapComputedForCurrentFrame(false),
  itsTDMap()
{
  itsFrame = -1;
  itsTDMap.setMaxSize(itsCacheSize.getVal());
  getPCAMatrix();
  getTigsMatrix();
  itsPCADims.setVal(5);  // fixed to 5 now
}

// ######################################################################
TaskRelevanceMapTigs::~TaskRelevanceMapTigs()
{ }

// ######################################################################
void TaskRelevanceMapTigs::reset1()
{
  TaskRelevanceMapAdapter::reset1();
}

// ######################################################################
void TaskRelevanceMapTigs::inputFrame(const InputFrame& f)
{
  itsFrame++;
  itsMapComputedForCurrentFrame = false;
}

// ######################################################################
void TaskRelevanceMapTigs::getPCAMatrix()
{
  FILE* itsFile = fopen(itsPCAMatrixName.getVal().c_str(), "rb");
  if(itsFile == 0)
    PLFATAL("Can not open PCAMatrix for read PCA");
  int gistDims = 0;
  if(fread(&gistDims, sizeof(int), 1, itsFile) != 1) LFATAL("fread failed");

  Image<float> matrixTmp(gistDims, itsPCADims.getVal(), NO_INIT);
  size_t sz = itsPCADims.getVal()*gistDims;
  if(fread(matrixTmp.beginw(), sizeof(float), sz, itsFile) != sz) LFATAL("fread failed");

  itsPCAMatrix = transpose(matrixTmp);

  fclose(itsFile);
}

// ######################################################################
void TaskRelevanceMapTigs::getTigsMatrix()
{
  FILE* itsFile = fopen(itsTigsMatrixName.getVal().c_str(),
                        "rb");
  if(itsFile == 0)
    PLFATAL("Can not open TigsMatrix for read Tigs coefficients");
  int w,h;
  if(fread(&h, sizeof(int), 1, itsFile) != 1) LFATAL("fread failed"); 
  if(fread(&w, sizeof(int), 1, itsFile) != 1) LFATAL("fread failed");

  Image<float> matrixTmp(h, w, NO_INIT);
  // keep in mind that the definition of Image is Image(width, height)
  if(fread(matrixTmp.beginw(), sizeof(float), w*h, itsFile) != size_t(w*h)) LFATAL("fread failed");

  itsTigsMatrix = transpose(matrixTmp);
  fclose(itsFile);
}

// ######################################################################
void TaskRelevanceMapTigs::integrate(SimEventQueue& q)
{
  if (SeC<SimEventGistOutput> e =
      q.check<SimEventGistOutput>(this))
    {
      if (itsMapComputedForCurrentFrame)
        return;

      itsGist = e->gv();
      itsTmpTDMap = getTDMap(itsGist);

      if (itsCacheSize.getVal() > 0)
        {
          itsTDMap.push_back(itsTmpTDMap);
          if (itsFrame % itsUpdatePeriod.getVal() ==0)
            itsCurrentTDMap = itsTDMap.mean();
        }
      else
        itsCurrentTDMap = itsTmpTDMap;


      itsMap = rescale(itsCurrentTDMap, itsMap.getDims());

      float mi, ma; getMinMax(itsMap, mi, ma);
      LINFO("\nFinal TRM range = [%.2f .. %.2f] -- 1.0 is baseline\n\n", mi, ma);

      itsMapComputedForCurrentFrame = true;
    }
}

// ######################################################################
void TaskRelevanceMapTigs::saccadicSuppression(const bool on)
{ }

// ######################################################################
Image<float> TaskRelevanceMapTigs::
getTDMap(Image<float> currGist)
{
  Image<float> currGistCut(34,1,NO_INIT);
  Image<float> currGistPCA(itsPCADims.getVal(), 1, NO_INIT);
  Image<float> tmp(300,1, NO_INIT); //300=20x15 (Rob's cvpr 2007 paper, 32x32 for 1 block)
  Image<float> tmp2;

  for(int i=0; i<34; i++)
    {
      currGistCut.setVal(i, 0, currGist.getVal(i*NUM_GIST_FEAT) );
    }
  currGistPCA = matrixMult(currGistCut, itsPCAMatrix);

  tmp =  matrixMult(currGistPCA, itsTigsMatrix);
  tmp2 = decode(tmp); // tranfer the 20x15 dims vector to an image

  return tmp2;
}

// ######################################################################
Image<float> TaskRelevanceMapTigs:: decode(Image<float> inputVec)
{
  const int w = 20;
  const int h = 15;
  Image<float> smallTDMap(w,h,NO_INIT);
  Image<float> bigTDMap;

  for(int i = 0; i< h; i++)
    {
      for(int j = 0; j < w; j++)
        {
          smallTDMap.setVal(j,i,inputVec.getVal(i*w+j,0) );
        }
    }
  inplaceNormalize(smallTDMap, 1.0F, 255.0F);
  bigTDMap = rescale(smallTDMap,itsMap.getDims());

  return bigTDMap;
}

// ######################################################################
void TaskRelevanceMapTigs::save1(const ModelComponentSaveInfo& sinfo)
{
  if (itsSaveResults.getVal())
    {
      // get the OFS to save to, assuming sinfo is of type
      // SimModuleSaveInfo (will throw a fatal exception otherwise):
      nub::ref<FrameOstream> ofs =
        dynamic_cast<const SimModuleSaveInfo&>(sinfo).ofs;

      ofs->writeFloat(itsCurrentTDMap, FLOAT_NORM_0_255, "TRM-SB",
                      FrameInfo("task relevance map static buffer", SRC_POS));
    }
}

// ######################################################################
// ######################################################################
// ########## TaskRelevanceMapTigs2 implementation
// ######################################################################
// ######################################################################
// ######################################################################
// for the training part see /lab/tmpib/u/gist-trm/trainging/Tigs2
TaskRelevanceMapTigs2::
TaskRelevanceMapTigs2(OptionManager& mgr, const std::string& descrName,
                      const std::string& tagName) :
  TaskRelevanceMapAdapter(mgr, descrName, tagName),
  itsTigsMatrixName(&OPT_TRMTigs2Matrix, this),
  itsGistPCADims(&OPT_TRMGistPCADims, this),
  itsImgPCADims(&OPT_TRMImgPCADims, this),
  itsGistPCAMatrixName(&OPT_TRMGistPCAMatrix, this),
  itsImgPCAMatrixName(&OPT_TRMImgPCAMatrix, this),
  itsCacheSize(&OPT_TRMCacheSize, this),
  itsUpdatePeriod(&OPT_TRMUpdatePeriod, this),
  itsMapComputedForCurrentFrame(false),
  itsTDMap()
{
  itsFrame = -1;
  itsTDMap.setMaxSize(itsCacheSize.getVal());
  itsGistPCADims.setVal(5); // fixed to 5 now
  itsImgPCADims.setVal(20); // fixed to 20 now
}

// ######################################################################
TaskRelevanceMapTigs2::~TaskRelevanceMapTigs2()
{ }

// ######################################################################
void TaskRelevanceMapTigs2::reset1()
{
  TaskRelevanceMapAdapter::reset1();
}

// ######################################################################
void TaskRelevanceMapTigs2::inputFrame(const InputFrame& f)
{
  getGistPCAMatrix();
  getImgPCAMatrix();
  getTigsMatrix();
  itsFrame++;

  Image<float> currFrame = rescale(f.grayFloat(),f.getDims()/16, RESCALE_SIMPLE_NOINTERP);
  itsCurrFrameVec = reshape
    (transpose(currFrame/255.0F), Dims(currFrame.getWidth()*currFrame.getHeight(),1));

  itsMapComputedForCurrentFrame = false;
}

// ######################################################################
void TaskRelevanceMapTigs2::getGistPCAMatrix()
{
  FILE* itsFile = fopen(itsGistPCAMatrixName.getVal().c_str(), "rb");
  if(itsFile == 0)
    PLFATAL("Can not open GistPCAMatrix for read PCA");

  int gistDims = 0;
  if(fread(&gistDims, sizeof(int), 1, itsFile) != 1) LFATAL("fread failed");

  Image<float> matrixTmp(gistDims, itsGistPCADims.getVal(), NO_INIT);
  size_t sz = itsGistPCADims.getVal()*gistDims;
  if(fread(matrixTmp.beginw(), sizeof(float), sz, itsFile) != sz) LFATAL("fread failed");
  itsGistPCAMatrix = transpose(matrixTmp);

  fclose(itsFile);
}

// ######################################################################
void TaskRelevanceMapTigs2::getImgPCAMatrix()
{
  FILE* itsFile = fopen(itsImgPCAMatrixName.getVal().c_str(), "rb");
  if(itsFile == 0)
    PLFATAL("Can not open ImgPCAMatrix for read PCA");
  int imgDims = 0;
  if(fread(&imgDims, sizeof(int), 1, itsFile) != 1) LFATAL("fread failed");

  Image<float> matrixTmp(imgDims, itsImgPCADims.getVal(), NO_INIT);
  size_t sz = itsImgPCADims.getVal()*imgDims;
  if(fread(matrixTmp.beginw(), sizeof(float), sz, itsFile) != sz) LFATAL("fread failed");

  itsImgPCAMatrix = transpose(matrixTmp);

  fclose(itsFile);
}

// ######################################################################
void TaskRelevanceMapTigs2::getTigsMatrix()
{
  FILE* itsFile = fopen(itsTigsMatrixName.getVal().c_str(), "rb");
  if(itsFile == 0)
    PLFATAL("Can not open TigsMatrix for read Tigs coefficients");

  int w,h;
  if(fread(&h, sizeof(int), 1, itsFile) != 1) LFATAL("fread failed");
  if(fread(&w, sizeof(int), 1, itsFile) != 1) LFATAL("fread failed");

  Image<float> matrixTmp(h, w, NO_INIT);
  // keep in mind that the definition of Image is Image(width, height)
  if(fread(matrixTmp.beginw(), sizeof(float), w*h, itsFile) != size_t(w*h)) LFATAL("fread failed");

  itsTigsMatrix = transpose(matrixTmp);

  fclose(itsFile);
}

// ######################################################################
void TaskRelevanceMapTigs2::integrate(SimEventQueue& q)
{
  if (SeC<SimEventGistOutput> e =
      q.check<SimEventGistOutput>(this))
    {
      if (itsMapComputedForCurrentFrame)
        return;

      itsGist = e->gv();

      itsTmpTDMap = getTDMap(itsGist/10.0F); // for consistent with the training

      if (itsCacheSize.getVal() > 0)
        {
          itsTDMap.push_back(itsTmpTDMap);
          if (itsFrame % itsUpdatePeriod.getVal() ==0)
            itsCurrentTDMap = itsTDMap.mean();
        }
      else
        itsCurrentTDMap = itsTmpTDMap;


      itsMap = rescale(itsCurrentTDMap, itsMap.getDims());

      float mi, ma; getMinMax(itsMap, mi, ma);
      LINFO("\nFinal TRM range = [%.2f .. %.2f] -- 1.0 is baseline\n\n", mi, ma);

      itsMapComputedForCurrentFrame = true;
    }
}

// ######################################################################
void TaskRelevanceMapTigs2::saccadicSuppression(const bool on)
{ }

// ######################################################################
Image<float> TaskRelevanceMapTigs2::
getTDMap(Image<float> currGist)
{
  Image<float> currGistCut(34,1,NO_INIT);
  Image<float> currGistPCA(itsGistPCADims.getVal(), 1, NO_INIT);
  Image<float> currImgPCA(itsImgPCADims.getVal(), 1, NO_INIT);
  Image<float> tmp(300,1, NO_INIT); //300=20x15 (Rob's cvpr 2007 paper, 32x32 for 1 block)
  Image<float> tmp2;

  for(int i=0; i<34; i++)
    {
      currGistCut.setVal(i, 0, currGist.getVal(i*NUM_GIST_FEAT) );
    }
  currGistPCA = matrixMult(currGistCut, itsGistPCAMatrix);

  currImgPCA = matrixMult(itsCurrFrameVec, itsImgPCAMatrix);
  LINFO("the dim of gist is %d %d, of img is %d %d \n",
        currGistCut.getHeight(), currGistCut.getWidth(),
        itsCurrFrameVec.getHeight(), itsCurrFrameVec.getWidth());

  Image<float> combo = concatX(currGistPCA,currImgPCA);

  tmp = matrixMult(combo, itsTigsMatrix);

  tmp2 = decode(tmp); // tranfer the 20x15 dims vector to an image

  return tmp2;
}

// ######################################################################
Image<float> TaskRelevanceMapTigs2:: decode(Image<float> inputVec)
{
  const int w = 20;
  const int h = 15;
  Image<float> smallTDMap(w,h,NO_INIT);
  Image<float> bigTDMap;

  for(int i = 0; i< h; i++)
    {
      for(int j = 0; j < w; j++)
        {
          smallTDMap.setVal(j,i,inputVec.getVal(i*w+j,0) );
        }
    }
  inplaceNormalize(smallTDMap, 1.0F, 255.0F);
  bigTDMap = rescale(smallTDMap,itsMap.getDims());

  return bigTDMap;
}

// ######################################################################
void TaskRelevanceMapTigs2::save1(const ModelComponentSaveInfo& sinfo)
{
  if (itsSaveResults.getVal())
    {
      // get the OFS to save to, assuming sinfo is of type
      // SimModuleSaveInfo (will throw a fatal exception otherwise):
      nub::ref<FrameOstream> ofs =
        dynamic_cast<const SimModuleSaveInfo&>(sinfo).ofs;

      ofs->writeFloat(itsCurrentTDMap, FLOAT_NORM_0_255, "TRM-SB",
                      FrameInfo("task relevance map static buffer", SRC_POS));
    }
}

// ######################################################################
// ######################################################################
// ########## TaskRelevanceMapSocial implementation
// ######################################################################
// ######################################################################

// ######################################################################
TaskRelevanceMapSocial::
TaskRelevanceMapSocial(OptionManager& mgr,
                           const std::string& descrName,
                           const std::string& tagName) :
  TaskRelevanceMapAdapter(mgr, descrName, tagName),
  itsXMLFName(&OPT_TRMSocialRegionFName, this),
  itsObjectsInfo(), itsObjectsNames(), itsNumObjects(0),
  itsFrame(0)
{ }

// ######################################################################
void TaskRelevanceMapSocial::start1()
{
  // copied from SVEyeRegion::start1
  if (itsXMLFName.getVal().empty())
    LFATAL("No XML file given - use --trmsocial-input-file option with trm-social");
  else
    {
    // load up the objects from the XML file
    itsObjectsInfo.reset(new TestImages(itsXMLFName.getVal().c_str(),
                                        TestImages::XMLFILE));

    // "count" the number of distinct objects by ID and get their names
    // this works b/c currently our script indexes objects sequentially 
    // from 0 ... n
    Scene thisScene;
    std::string nom;
    for (uint i = 0; i < itsObjectsInfo->getNumScenes(); i++)
      {
         thisScene = itsObjectsInfo->getSceneData(i);
        for (std::vector<Object>::iterator iObj = thisScene.objects.begin();
             iObj != thisScene.objects.end(); iObj++)
          {
            //if the array is too small, resize the array
            if((*iObj).id >= itsNumObjects) 
              {
                itsNumObjects = (*iObj).id+1;
                itsObjectsNames.resize(itsNumObjects,std::string());
              }

            // get rid of leading and trailing underscores
            nom = (*iObj).name;
            if(!nom.empty() && (nom[nom.length()-1] == '_')) 
              nom.erase(nom.length()-1);
            if(!nom.empty() && (nom[0] == '_')) nom.erase(0,1);

            //if this is a new name, use the id to write the name
            if(itsObjectsNames[(*iObj).id].empty()==true) 
              {
                itsObjectsNames[(*iObj).id] = nom;
                LINFO("assigning obj %s to obj id #%d from frame %u",
                      nom.c_str(),(*iObj).id,i);
              }

            // if this id is already used for a different object, throw an error
            if (itsObjectsNames[(*iObj).id].compare(nom) != 0)
              LFATAL("XML file %s has name conflict for object #%u, %s <-> %s",
                     itsXMLFName.getVal().c_str(), (*iObj).id, 
                     itsObjectsNames[(*iObj).id].c_str(), nom.c_str());
          }
     
       }
    }

  TaskRelevanceMapAdapter::start1(); 
}

// ######################################################################
TaskRelevanceMapSocial::~TaskRelevanceMapSocial()
{ }

// ######################################################################
void TaskRelevanceMapSocial::inputFrame(const InputFrame& f)
{
  const Dims mapdims = f.getDims();
  const int sml = itsLevelSpec.getVal().mapLevel();
  const float EyeVal = 128.0F, MouthVal = 64.0F, FaceVal = 32.0F, BodyVal = 16.0F, PersonVal = 4.0F, BkgdVal = 1.0F;

  Image<float> BigMap;
  BigMap.resize(mapdims, true); 
  BigMap.clear(1.0F);
  
  Scene sceneData =
    itsObjectsInfo->getSceneData(itsFrame);

  // loop through all the regions on a given frame and assign z-stack order
  std::vector<float> weights(itsNumObjects, 1.0F);
  for (std::vector<Object>::iterator itrObject = sceneData.objects.begin(); itrObject != sceneData.objects.end(); itrObject++) {

    uint idx = (*itrObject).id;
    std::string ObjName = toLowerCase(itsObjectsNames[idx]);
    if (ObjName.find("eye") != std::string::npos) {weights[idx] = EyeVal;}
    else if (ObjName.find("mouth") != std::string::npos) {weights[idx] = MouthVal;}
    else if (ObjName.find("head") != std::string::npos || 
             ObjName.find("face") != std::string::npos) {weights[idx] = FaceVal;}
    else if (ObjName.find("body") != std::string::npos) {weights[idx] = BodyVal;}
    else if (ObjName.find("person") != std::string::npos ||
             ObjName.find("people") != std::string::npos ||
             ObjName.find("man") != std::string::npos ||
             ObjName.find("woman") != std::string::npos) {weights[idx] = PersonVal;}
    else {weights[idx] = BkgdVal;}
  }
  uint i,j,tmp;
  // sort z-stack weights

  const uint numInScene = sceneData.objects.size();
  std::vector<uint> zorder(numInScene);
  for (i = 0; i < numInScene; i++) zorder[i] = i;
  for (i = 0; i < numInScene; i++)  
    for (j = 0; j < numInScene-i-1; j++) 
      if(weights[sceneData.objects[zorder[j]].id] > 
         weights[sceneData.objects[zorder[j+1]].id]) {
        tmp = zorder[j];
        zorder[j] = zorder[j+1];
        zorder[j+1] = tmp;
      }  
  
  // fill BigMap from bottom of z-stack to top
  // todo: enforce C0/C1 continuity by some poisson problem?
  for (i = 0; i < numInScene; i++) {
    Object iObj = sceneData.objects[zorder[i]]; 
    drawFilledPolygon(BigMap, iObj.polygon, weights[iObj.id]);
  }
  
  itsMap = rescale(BigMap, mapdims.w() >> sml, mapdims.h() >> sml);
  itsFrame++;
}

// ######################################################################
void TaskRelevanceMapSocial::saccadicSuppression(const bool on)
{ }

// ######################################################################
void TaskRelevanceMapSocial::integrate(SimEventQueue& q)
{ }

// ######################################################################
void TaskRelevanceMapSocial::reset1()
{
  TaskRelevanceMapAdapter::reset1();
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
