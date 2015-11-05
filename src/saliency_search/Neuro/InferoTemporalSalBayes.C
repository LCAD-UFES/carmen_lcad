/*!@file Neuro/InferoTemporalSalBayes.C Object recognition module with SalBayes */

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
// Primary maintainer for this file: Lior Elazary
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/InferoTemporalSalBayes.C $
// $Id: InferoTemporalSalBayes.C 12337 2009-12-19 02:45:23Z itti $
//

#include "Neuro/InferoTemporalSalBayes.H"

#include "Component/OptionManager.H"
#include "Component/ModelOptionDef.H"
#include "Channels/ChannelMaps.H"
#include "Channels/ChannelOpts.H"
#include "Image/MathOps.H"
#include "Image/ShapeOps.H"
#include "Image/CutPaste.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/NeuroSimEvents.H"
#include "Neuro/Brain.H"
#include "Neuro/VisualCortex.H"
#include "Simulation/SimEventQueue.H"
#include "Media/MediaSimEvents.H"

#include <cstdlib>
#include <iostream>

static const ModelOptionDef OPT_ITSB_FOVSize =
  { MODOPT_ARG(Dims), "FoveaSize", &MOC_ITC, OPTEXP_CORE,
    "Use the given fovea size for constructing a feature vector.",
    "it-fov-size", '\0', "<w>x<h>", "75x75" };

static const ModelOptionDef OPT_ITCMode =
  { MODOPT_ARG_STRING, "ITC Mode", &MOC_ITC, OPTEXP_CORE,
    "The mode of ITC . Train: is for training from some data, Rec is for recognition.",
    "it-mode", '\0', "<Train|Rec>", "Rec" };

static const ModelOptionDef OPT_ITCSalBayes_NetFile =
{ MODOPT_ARG_STRING, "ITC BaysNet File", &MOC_ITC, OPTEXP_CORE,
  "Name of the file to save/read the computed Bayesian Network.",
  "it-bayesnet-file", '\0', "<filename>", "SalBayes.net" };

const ModelOptionDef OPT_ITSiftObjectDBFileName =
  { MODOPT_ARG_STRING, "ITC SiftObjectDBFileName", &MOC_ITC, OPTEXP_CORE,
    "Filename for the sift object database. Specifying no file will disable the sift alg.",
    "it-object-db", '\0', "<filename>", "" };

const ModelOptionDef OPT_ITUseSift =
  { MODOPT_ARG(int), "ITC use sift", &MOC_ITC, OPTEXP_CORE,
    "Use sift recognition on the n most probable objects obtained from SalBayes. "
    "That is, the sift algorithm will only run on the top n objects returned from SalBayes."
    "0 disables sift and just return the most probable object",
    "it-use-sift", '\0', "<int>", "10" };

const ModelOptionDef OPT_ITUseMaxNMatches =
  { MODOPT_ARG(bool), "ITC use max num of matches", &MOC_ITC, OPTEXP_CORE,
    "When determining a which object in the database matches, use the maximum "
    "number of keypoints matches instead of sorting by distance of keypoints.",
    "it-use-max-num-matches", '\0', "<true|false>", "true" };

// ######################################################################
// functor to assist with VisualObjectMatch sorting:
class moreVOM
{
public:
  moreVOM(const float kcoeff, const float acoeff) :
    itsKcoeff(kcoeff), itsAcoeff(acoeff)
  { }

  bool operator()(const rutz::shared_ptr<VisualObjectMatch>& x,
                  const rutz::shared_ptr<VisualObjectMatch>& y)
  { return ( x->getScore(itsKcoeff, itsAcoeff) >
             y->getScore(itsKcoeff, itsAcoeff) ); }

private:
  float itsKcoeff, itsAcoeff;
};


// ######################################################################
InferoTemporalSalBayes::InferoTemporalSalBayes(OptionManager& mgr,
                                     const std::string& descrName,
                                     const std::string& tagName) :
  InferoTemporal(mgr, descrName, tagName),
  itsLevelSpec(&OPT_LevelSpec, this),
  itsFoveaSize(&OPT_ITSB_FOVSize, this),
  itsITCMode(&OPT_ITCMode, this),
  itsBayesNetFilename(&OPT_ITCSalBayes_NetFile, this),
  itsUseSift(&OPT_ITUseSift, this),
  itsSiftObjectDBFile(&OPT_ITSiftObjectDBFileName, this),
  itsUseMaxNMatches(&OPT_ITUseMaxNMatches, this),
  itsSiftObjectDB(new VisualObjectDB())
{}

// ######################################################################
void InferoTemporalSalBayes::start1()
{
  // if no filename given for our object DB, start empty, otherwise load it:
  if (!itsSiftObjectDBFile.getVal().empty())
    itsSiftObjectDB->loadFrom(itsSiftObjectDBFile.getVal());

  InferoTemporal::start1();
}

// ######################################################################
void InferoTemporalSalBayes::stop1()
{}

// ######################################################################
InferoTemporalSalBayes::~InferoTemporalSalBayes()
{}

// ######################################################################
void InferoTemporalSalBayes::attentionShift(SimEventQueue& q,
                                            const Point2D<int>& location)
{
  if (itsBayesNet.is_valid() == false)
    {
      rutz::shared_ptr<SimReqVCXmaps> vcxm(new SimReqVCXmaps(this));
      q.request(vcxm); // VisualCortex is now filling-in the maps...
      rutz::shared_ptr<ChannelMaps> chm = vcxm->channelmaps();

      itsBayesNet.reset(new Bayes(chm->numSubmaps(), 0));
      itsBayesNet->load(itsBayesNetFilename.getVal().c_str());
    }


  // do we have 3D information about the scene?
  /*
  if(SeC<SimEventSaliencyCoordMap> e = q.check<SimEventSaliencyCoordMap>(this))
  {
    itsHas3Dinfo     = true;
    its3DSalLocation = e->getMaxSalCurrent();
    its3DSalVal      = e->getMaxSalValue();
  }
  else
  {
    itsHas3Dinfo     = false;
  }
  */

  //extract features
  std::vector<double> fv = buildRawDV(q, location);

  rutz::shared_ptr<VisualObject> vobj;
  if (!itsSiftObjectDBFile.getVal().empty())
  {
    //Train the Sift Recognition
    Image<PixRGB<float> > objImg;
    if (SeC<SimEventRetinaImage> e = q.check<SimEventRetinaImage>(this))
      objImg = e->frame().colorByte();
    if (!objImg.initialized()) return;

      vobj.reset(new VisualObject("NewObject", "NULL", objImg,
            Point2D<int>(-1,-1),
            std::vector<float>(),
            std::vector< rutz::shared_ptr<Keypoint> >(),
            false)); //Use color?
  }

  if (itsITCMode.getVal().compare("Train") == 0)          // training
  {
    rutz::shared_ptr<TestImages::SceneData> sceneData;
    //Get the scene data, but dont mark it so we will get it on the next saccade
    std::string objName;
    if (SeC<SimEventInputFrame> e = q.check<SimEventInputFrame>(this,SEQ_UNMARKED,0))
    {

      GenericFrame gf = e->frame();
      if (gf.hasMetaData(std::string("SceneData")))
      {
        rutz::shared_ptr<GenericFrame::MetaData> metaData = gf.getMetaData(std::string("SceneData"));
        if (metaData.get() != 0)
        {
          sceneData.dyn_cast_from(metaData);
          objName = getObjNameAtLoc(sceneData->objects, location);
        }
      } else {
        LINFO("Enter name for new object or [RETURN] to skip training:");
        std::getline(std::cin, objName, '\n');
      }

    }

    if (objName.length() > 0)
    {
      LINFO("Train on %s", objName.c_str());
      itsBayesNet->learn(fv, objName.c_str());
      itsBayesNet->save(itsBayesNetFilename.getVal().c_str());

      //Train The Sift alg
      if (!itsSiftObjectDBFile.getVal().empty())
      {
        vobj->setName(objName);
        itsSiftObjectDB->addObject(vobj, false); //allow multiple object names
        itsSiftObjectDB->saveTo(itsSiftObjectDBFile.getVal());
      }
    }
  }
  else if (itsITCMode.getVal().compare("Rec") == 0)      // Recognition
  {
    if (itsUseSift.getVal() > 0 && !itsSiftObjectDBFile.getVal().empty())
      predictWithSift(fv, vobj, q);
    else
      predict(fv, q);
  }
  else
    LFATAL("Unknown IT Mode type %s", itsITCMode.getVal().c_str());
}
// ######################################################################

void InferoTemporalSalBayes::predict(std::vector<double> &fv, SimEventQueue& q)
{
  const int cls = itsBayesNet->classify(fv);

  if (cls != -1)
  {
    const double maxProb  = itsBayesNet->getMaxProb();
    const double normProb = itsBayesNet->getNormProb();

    std::string clsName(itsBayesNet->getClassName(cls));
    rutz::shared_ptr<TestImages::ObjData> objData(new TestImages::ObjData);
    objData->name     = clsName;
    objData->id       = cls;
    objData->maxProb  = maxProb;
    objData->normProb = normProb;

    rutz::shared_ptr<SimEventObjectDescription>
      objDataEvent(new SimEventObjectDescription(this, objData));
    q.post(objDataEvent);
  }
}
// ######################################################################

void InferoTemporalSalBayes::predictWithSift(std::vector<double> &fv,
  rutz::shared_ptr<VisualObject> &vobj,
  SimEventQueue& q)
{
  int cls = -1;
  uint mink = 6U; //min # of keypoints
  float kcoeff = 0.5; //keypoint distance score default 0.5F
  float acoeff = 0.5; //affine distance score default 0.5F
  //float minscore=1.0F; //minscore  default 1.0F

  std::vector< rutz::shared_ptr<VisualObjectMatch> > matches;
  matches.clear();

  std::string maxObjName;
  uint maxKeyMatches = 0;

  std::vector<Bayes::ClassInfo> classInfo = itsBayesNet->classifyRange(fv, cls);
  for(uint i=0; i< classInfo.size() && i < (uint)itsUseSift.getVal(); i++)
  {
    std::string clsName(itsBayesNet->getClassName(classInfo[i].classID));
    //check all objects containing this name
    //TODO: this should be hashed for greater efficiency
    for(uint j=0; j<itsSiftObjectDB->numObjects(); j++)
    {
      rutz::shared_ptr<VisualObject> knownVObj = itsSiftObjectDB->getObject(j);
      if (clsName.compare(knownVObj->getName()) == 0)
      {
        // attempt a match:
        rutz::shared_ptr<VisualObjectMatch>
          match(new VisualObjectMatch(vobj, knownVObj,
                VOMA_SIMPLE,
                6U)); //keypoint selection threshold, from lowes code

        if (itsUseMaxNMatches.getVal())
        {
          //Find the max based on num of matches
          if (match->size() > maxKeyMatches)
          {
            maxObjName = clsName;
            maxKeyMatches = match->size();
          }
        } else {
          // apply some standard pruning:
          match->prune(std::max(25U, mink * 5U), mink);

          //// if the match is good enough, store it:
          if (match->size() >= mink &&
              //    match->getScore(kcoeff, acoeff) >= minscore &&
              match->checkSIFTaffine())
          {
            matches.push_back(match);
          }
        }

      }
    }
    std::sort(matches.begin(), matches.end(), moreVOM(kcoeff, acoeff));
  }

  rutz::shared_ptr<TestImages::ObjData> objData(new TestImages::ObjData);
  if (itsUseMaxNMatches.getVal())
  {
    objData->name = maxObjName;
    objData->id = (unsigned int) -1;
  } else {
    if (matches.size() > 0)
    {
      //Get the first match
      objData->name = matches[0]->getVoTest()->getName();
      objData->id = (unsigned int) -1;
    }
  }
  rutz::shared_ptr<SimEventObjectDescription>
    objDataEvent(new SimEventObjectDescription(this, objData));
  q.post(objDataEvent);
}

// ######################################################################
std::string InferoTemporalSalBayes::getObjNameAtLoc(const std::vector<TestImages::ObjData> &objects, const Point2D<int>& loc)
{

  for(uint obj=0; obj<objects.size(); obj++)
  {
    TestImages::ObjData objData = objects[obj];

    //find the object dimention from the polygon
    if (objData.polygon.size() > 0)
    {
      Point2D<int> upperLeft = objData.polygon[0];
      Point2D<int> lowerRight = objData.polygon[0];

      for(uint i=0; i<objData.polygon.size(); i++)
      {
        //find the bounds for the crop
        if (objData.polygon[i].i < upperLeft.i) upperLeft.i = objData.polygon[i].i;
        if (objData.polygon[i].j < upperLeft.j) upperLeft.j = objData.polygon[i].j;

        if (objData.polygon[i].i > lowerRight.i) lowerRight.i = objData.polygon[i].i;
        if (objData.polygon[i].j > lowerRight.j) lowerRight.j = objData.polygon[i].j;
      }

      //check if point is within the polygon
      for(int y=upperLeft.j; y<lowerRight.j; y++)
        for(int x=upperLeft.i; x<lowerRight.i; x++)
        {
          if (pnpoly(objData.polygon, loc))
            return objData.name;
        }
    }

  }
  return std::string("Unknown");
}

// ######################################################################
std::vector<double> InferoTemporalSalBayes::buildRawDV(SimEventQueue& q, const Point2D<int>& foveaLoc)
{
  bool salientLocationWithinSubmaps = true;
  Point2D<int> objSalientLoc(-1,-1);  //the feature location

  const int smlevel = itsLevelSpec.getVal().mapLevel();

  int x=int(foveaLoc.i / double(1 << smlevel) + 0.49);
  int y=int(foveaLoc.j / double(1 << smlevel) + 0.49);

  //LINFO("Getting from location %d,%d",x,y);

  int foveaW = int(itsFoveaSize.getVal().w() / double(1 << smlevel) + 0.49);
  int foveaH = int(itsFoveaSize.getVal().h() / double(1 << smlevel) + 0.49);

  int tl_x = x - (foveaW/2);
  int tl_y = y - (foveaH/2);

  rutz::shared_ptr<SimReqVCXmaps> vcxm(new SimReqVCXmaps(this));
  q.request(vcxm); // VisualCortex is now filling-in the maps...
  rutz::shared_ptr<ChannelMaps> chm = vcxm->channelmaps();

  Dims mapDims = chm->getMap().getDims();

  //Shift the fovea location so we dont go outside the image
  //Sift the fovea position if nessesary
  if (tl_x < 0) tl_x = 0; if (tl_y < 0) tl_y = 0;
  if (tl_x+foveaW > mapDims.w()) tl_x = mapDims.w() - foveaW;
  if (tl_y+foveaH > mapDims.h()) tl_y = mapDims.h() - foveaH;

  if (!salientLocationWithinSubmaps)
  {
    //Find the most salient location within the fovea
    Image<float> SMap = chm->getMap();

    Image<float> tmp = SMap; //TODO need to resize to fovea
    //Find the max location within the fovea

    float maxVal; Point2D<int> maxLoc;
    findMax(tmp, maxLoc, maxVal);
    //convert back to original SMap cordinates
   // objSalientLoc.i=tl_x+maxLoc.i;
   // objSalientLoc.j=tl_y+maxLoc.j;
    objSalientLoc.i=x;
    objSalientLoc.j=y;
  }

  //Go through all the submaps building the DV

  std::vector<double> FV;
  uint numSubmaps = chm->numSubmaps();
  for (uint i = 0; i < numSubmaps; i++)
  {
    //Image<float> submap = itsComplexChannel->getSubmap(i);
    Image<float> submap = chm->getRawCSmap(i);

    // resize submap to fixed scale if necessary:
    if (submap.getWidth() > mapDims.w())
      submap = downSize(submap, mapDims);
    else if (submap.getWidth() < mapDims.w())
      submap = rescale(submap, mapDims); //TODO convert to  quickInterpolate


    if (salientLocationWithinSubmaps) //get the location from the salient location within each submap
    {
      Image<float> tmp = submap;
      //get only the fovea region

      if (foveaW < tmp.getWidth()) //crop if our fovea is smaller
        tmp = crop(tmp, Point2D<int>(tl_x, tl_y), Dims(foveaW, foveaH));
     // tmp = maxNormalize(tmp, 0.0F, 10.0F, VCXNORM_MAXNORM);  //find salient locations

      //Find the max location within the fovea
      float maxVal; Point2D<int> maxLoc; findMax(tmp, maxLoc, maxVal);
      //LINFO("%i: Max val %f, loc(%i,%i)", i, maxVal, maxLoc.i, maxLoc.j);

      objSalientLoc.i=tl_x+maxLoc.i;
      objSalientLoc.j=tl_y+maxLoc.j;

    }

    if (objSalientLoc.i < 0) objSalientLoc.i = 0;
    if (objSalientLoc.j < 0) objSalientLoc.j = 0;

    if (objSalientLoc.i > submap.getWidth()-1) objSalientLoc.i = submap.getWidth()-1;
    if (objSalientLoc.j > submap.getHeight()-1) objSalientLoc.j = submap.getHeight()-1;



   // LINFO("Location from %i,%i: (%i,%i)", objSalientLoc.i, objSalientLoc.j,
    //    submap.getWidth(), submap.getHeight());
    float featureVal = submap.getVal(objSalientLoc.i,objSalientLoc.j);
    FV.push_back(featureVal);
 //   SHOWIMG(rescale(submap, 255, 255));

  }

  return FV;

}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
