/*!@file Neuro/SimulationViewerEyeRegion.C comparison between region data from an ObjRec data (from an XML file) and human eye movements */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SimulationViewerEyeRegion.C $
// $Id: SimulationViewerEyeRegion.C 14376 2011-01-11 02:44:34Z pez $
//

#include "Neuro/SimulationViewerEyeRegion.H"

#include "Component/OptionManager.H"
#include "Component/ModelOptionDef.H"
#include "Transport/TransportOpts.H"
#include "Channels/ChannelBase.H"
#include "Image/DrawOps.H"
#include "Image/ColorOps.H"    // for toRGB()
#include "Image/CutPaste.H"    // for concatX()
#include "Image/FilterOps.H"   // for lowPass3()
#include "Image/MathOps.H"     // for alphaBlend()
#include "Image/ShapeOps.H"    // for rescale()
#include "Image/Transforms.H"  // for composite()
#include "Neuro/NeuroOpts.H"
#include "Neuro/NeuroSimEvents.H"
#include "Psycho/EyeData.H"
#include "Simulation/SimEventQueue.H"
#include "Util/sformat.H"

#include "rutz/trace.h"

#include <fstream>
//#include <iostream>

const ModelOptionDef OPT_SimViewXMLInputFile =
  { MODOPT_ARG(std::string), "SimViewXMLInputFile", &MOC_INPUT, OPTEXP_CORE,
    "XML file to gather region data from.",
    "xml-file", '\0', "string", ""};

const ModelOptionDef OPT_SimViewSelectedObjects =
  { MODOPT_ARG(std::string), "SimViewSelectedObjects", &MOC_DISPLAY, OPTEXP_CORE,
    "Only display objects matching the given name.",
    "objs-filter", '\0', "string", "" };

const ModelOptionDef OPT_SimViewObjDrawMode =
  { MODOPT_ARG(std::string), "SimViewObjDrawMode", &MOC_DISPLAY, OPTEXP_CORE,
    "Chooses which objects are displayed.  Note that these options currently "
    "depend upon --display-patch and --display-foa.  \n"
    "\t'selected' objects are objects which are selected by the option --objs-filter.\n"
    "\t'targeted' objects are objects that saccades are directed towards.\n"
    "\t'traced' objects are objects that are followed by an eyetrace.\n"
    "\t'none' and 'all' are none and all labeled objects, respectively.",
    "obj-drawmode", '\0', "<none|selected|targeted|traced|all>", "all" };

const ModelOptionDef OPT_SimViewHighlightMode =
  { MODOPT_ARG(std::string), "SimViewHighlightMode", &MOC_DISPLAY, OPTEXP_CORE,
    "Chooses when objects are highlighted.  Note that these options currently "
    "depend upon --display-patch and --display-foa.  Also, highlighting is currently "
    "ambiguous with multiple eye-traces. \n"
    "\t'targeted' triggers highlighting only at the FOA.\n"
    "\t'traced' highlights the object that is at the point of gaze.",
    "obj-highlightmode", '\0', "<off|targeted|traced>", "targeted" };

// needs updating
const ModelOptionDef OPT_SimViewPrependHeader =
  { MODOPT_FLAG, "SimViewPrependHeader", &MOC_DISPLAY, OPTEXP_CORE,
    "Determines whether to add a header to svem or region output",
    "prepend-header", '\0', "", "false" };

// Used by: SimulationViewerEyeRegion
const ModelOptionDef OPT_SimViewRegionOutFile =
  { MODOPT_ARG_STRING, "SimViewRegionOutFile", &MOC_DISPLAY, OPTEXP_CORE,
    "File name for region-based output data (or empty to not save output data).",
    "region-out-fname", '\0', "<file>", "" };

// ######################################################################
SimulationViewerEyeRegion::
SimulationViewerEyeRegion(OptionManager& mgr, const std::string& descrName,
                       const std::string& tagName) :
  SimulationViewerEyeMvt(mgr, descrName, tagName),
  itsRegionOutFname(&OPT_SimViewRegionOutFile, this),
  itsXMLFname(&OPT_SimViewXMLInputFile, this),
  itsSelectedObjects(&OPT_SimViewSelectedObjects, this),
  itsObjectDrawMode(&OPT_SimViewObjDrawMode, this),
  itsHighlightMode(&OPT_SimViewHighlightMode, this),
  itsPrependHeader(&OPT_SimViewPrependHeader,this),
  itsLineThickness("SVLineThickness", this, 2),
  itsHitTransparency("SVHitTransparency", this, 0.65),
  itsRandomColoring("SVRandomColoring", this, true),
  itsRegionOutFile(0), itsRegions(), itsTargetsMask(), itsNumObjects(0),
  itsCurrRegionID(NULL_OBJ), 
  itsObjectEntry(new EyeData(1,1,1,SACSTATE_FIX,false)), 
  itsObjectOnset(SimTime::ZERO()), itsObjectFrameOnset(0), itsRandColors(), 
  itsObjectsNames(), itsRegHeaderWritten(false)
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
SimulationViewerEyeRegion::~SimulationViewerEyeRegion()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void SimulationViewerEyeRegion::start1()
{
  if (itsXMLFname.getVal().empty())
    LFATAL("No XML file given - use --xml-file option with sv-Type EyeRegion");
  else
    {
    // load up the objects from the XML file
    itsObjectsInfo.reset(new TestImages(itsXMLFname.getVal().c_str(),
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
                     itsXMLFname.getVal().c_str(), (*iObj).id, 
                     itsObjectsNames[(*iObj).id].c_str(), nom.c_str());
          }
     
       }
    }

  // open output file:
  if (!itsRegionOutFname.getVal().empty())
    {
      if (itsRegionOutFile) delete itsRegionOutFile;
      itsRegionOutFile = new std::ofstream(itsRegionOutFname.getVal().c_str());
      if (itsRegionOutFile->is_open() == false)
        LFATAL("Cannot open '%s' for writing", itsRegionOutFname.getVal().c_str());
    }

  // ensure that if this output file is open then --display-patch is also on.
  if (!(itsRegionOutFname.getVal().empty()) && itsDisplayPatch.getVal() == false)
        LFATAL("Cannot write %s unless --display-patch is set.",
               itsRegionOutFname.getVal().c_str());

  // initialize random colors

  if(itsRandomColoring.getVal())
    for (uint i = 0; i < itsNumObjects; i++)
      itsRandColors.push_back(PixRGB<byte>(randomUpToIncluding(255),
                                           randomUpToIncluding(255),
                                           randomUpToIncluding(255)));

  SimulationViewerEyeMvt::start1(); 
}

// ######################################################################
void SimulationViewerEyeRegion::stop1()
{
GVX_TRACE(__PRETTY_FUNCTION__);
 if (itsRegionOutFile) {
   writeROIOutput(rawToRet(itsObjectEntry->position()));
   delete itsRegionOutFile; 
   itsRegionOutFile = 0;
}

 SimulationViewerEyeMvt::stop1();
}

// ######################################################################
uint SimulationViewerEyeRegion::findHitRegion(const Point2D<int> pt, uint fNum)
{
  // TODO: how can we guarantee that the frame matches the annotation?
  // Reasonable assumption 1: the annotation corresponds to the video
  // Danger 1: different input frame sequence (e.g. frame rate)
  // Danger 2: holdup of frames from Retina

  if(itsObjectsInfo.get() == 0)
    LFATAL("No objects data - check xml file.");

  if(itsFrameNumber >= itsObjectsInfo->getNumScenes())
    {
      LINFO("Ran out of XML scene data...");
      return NULL_OBJ;
    }

  //should check pt must also be looking at current frame w.r.t cropping, etc.

  Scene sceneData =
    itsObjectsInfo->getSceneData(fNum);

  // find which object this saccade target hits
  uint pID = NULL_OBJ;
  double thisrank, maxrank=-1;

  for(std::vector<Object>::iterator itrObject =
        sceneData.objects.begin(); itrObject != sceneData.objects.end(); itrObject++)
    if(pnpoly((*itrObject).polygon,pt)) // is our object hit by the pt?
      {
        thisrank = rankForeground(*itrObject);
        if(thisrank > maxrank) // is our object in front?
          {
            pID = (*itrObject).id;
            maxrank = thisrank;
          }
      }
  return pID;
}


// ######################################################################
double SimulationViewerEyeRegion::rankForeground(Object obj)
{
  // This is an ad-hoc way to determine which object is most in front.

  const int h=1080,w=1920;
  const double scrsize= h*w;
  const size_t npos = std::string::npos;

  // simple way to assign foregroundness
  std::string name = toLowerCase(getObjName(obj.id));
  if(name.find("eyes") != npos || name.find("mouth") != npos) return 4.0;
  if(name.find("head") != npos || name.find("face") != npos) return 3.0;
  if(name.find("body") != npos) return 2.0;
  if(name.find("man") != npos || name.find("guy") != npos ||
     name.find("girl") != npos || name.find("person") != npos ||
     name.find("people") != npos || name.find("group") != npos)
     return area(obj.polygon)/scrsize+1; // weight based on area, normed by screen size - smaller is worse
  if(obj.id == NULL_OBJ) return 0;
  else return 1-area(obj.polygon)/scrsize; // smaller is better
  // none is assigned 
}

// ######################################################################
std::string SimulationViewerEyeRegion::listAllRegionsHit(const Point2D<int> pt, uint fNum)
{
  if(itsObjectsInfo.get() == 0)
    LFATAL("No objects data - check xml file.");

  if(itsFrameNumber >= itsObjectsInfo->getNumScenes())
    {
      LINFO("Ran out of XML scene data...");
      return std::string();
    }

  //TODO: should check pt is also looking at current frame w.r.t cropping, etc.

  Scene sceneData =
    itsObjectsInfo->getSceneData(fNum);

  // find which images this saccade target hits
  std::string hits;

  for(std::vector<Object>::iterator itrObject =
        sceneData.objects.begin(); itrObject != sceneData.objects.end(); itrObject++)
    if(pnpoly((*itrObject).polygon,pt))
      {
        if (!hits.empty()) hits += ";"; //delimiter
        hits += getObjName((*itrObject).id);
      }

  if (hits.empty()) 
    {
      LINFO("nothing hit!");  
      hits = "none";
    }
  return hits;
}
// ######################################################################
void SimulationViewerEyeRegion::extraSampleProcessing(const rutz::shared_ptr<EyeData> data)
{
  Point2D<int> currPos = rawToRet(data->position());
  uint hitObjID = findHitRegion(currPos);

  if(hitObjID != itsCurrRegionID)
    {
      // every time we switch objects we send output to a different file than the one for svem data
      // this gives an object by object account instead of a saccade by saccade account
      writeROIOutput(currPos);

      // TODO: incorporate output into one file, interweave headers? something perhaps more meaningful, less readable though.
      // probably not worth it as a separate option..
      // TODO: work into multiple eyetrace streams?

      Object oldObj = getSceneObj(itsCurrRegionID, itsObjectFrameOnset),
        newObj = getSceneObj(hitObjID);
      //            CLINFO("**** Object gaze shift found: %s (%0.3f)->%s (%0.3f) at %.1fms **** ",
      //   getObjName(oldObj.id).c_str(), rankForeground(oldObj),
      //   getObjName(newObj.id).c_str(), rankForeground(newObj),
      //   itsCurrTime.msecs());
      
      // update new information
      itsCurrRegionID = hitObjID;
      itsObjectOnset = itsCurrTime;
      itsObjectFrameOnset = itsFrameNumber;
      itsObjectEntry = data;
    }
}

// ######################################################################
void SimulationViewerEyeRegion::writeROIOutput(Point2D<int> currPos)
{
  // writes another file with more specific region data, namely:
  // distances to different objects
  // only works for one file at a time right now

  if(itsObjectsInfo.get() == 0)
    LFATAL("No objects data - check xml file.");
  if(itsFrameNumber >= itsObjectsInfo->getNumScenes())
    {
      LINFO("Ran out of XML scene data...");
      return;
    }

  // if we don't have the name for the file, exit
  if (itsRegionOutFname.getVal().empty()) return;


  if (!itsRegHeaderWritten) //write the header first
    {
      itsRegHeaderWritten = true;

      std::string header;
      header += "objname onset duration ";
      for(std::vector<std::string>::iterator iStr = itsObjectsNames.begin();
          iStr != itsObjectsNames.end(); iStr++)
        header += sformat("%sDist ", (*iStr).c_str());

      if (itsRegionOutFile)
        (*itsRegionOutFile) << header << std::endl;

  //      return; // don't want to process first object

    }

  std::string output;

  // first, the last region
  Object thisObj =
    getSceneObj(itsCurrRegionID,itsObjectFrameOnset);
  output += sformat("%s ", getObjName(thisObj.id).c_str());

  // second, the time entered to that region
  output += sformat("%.1fms ", itsObjectOnset.msecs());
  // third, the time elapsed in that region
  output += sformat("%.1fms ", (itsCurrTime-itsObjectOnset).msecs());

  // now, for distances to region.
  // we precompute the distances to each region (calculated from edges)
  // and sample at N evenly spaced frames to see where the average distance is
  const uint N = 5;
  uint iFrame;

  double dist;
  const double NOT_IN_FRAME = 10000; //out of range
  std::vector<std::vector<double> > distsNow
    (itsNumObjects, std::vector<double>(N,NOT_IN_FRAME));

  for(uint i = 0; i < N; i++)
    {
      iFrame = itsObjectFrameOnset + uint((itsFrameNumber-itsObjectFrameOnset)*i/N);
      Scene sceneData =
        itsObjectsInfo->getSceneData(iFrame);

      // find the distances to the objects in that frame
      std::vector<Object>::iterator itr;
      for(itr = sceneData.objects.begin(); itr != sceneData.objects.end(); itr++)
        {
          std::string objName = getObjName((*itr).id);
          std::vector<Point2D<int> > objPoly = (*itr).polygon; //grab its polygon
          uint ps = objPoly.size();
          uint objID = (*itr).id;

          // find min distance to polygon by iterating around edges
          double mindist = 10000;
          for (uint ii = 0, jj = ps-1; ii < ps; jj = ii++)
            {
              dist = currPos.distanceToSegment(objPoly[ii],objPoly[jj]);
              if(mindist > dist) mindist = dist;
            }

          // assign distance as inside or outside a polygon (-/+)
          distsNow[objID][i] = (pnpoly(objPoly,currPos)) ?
            -mindist : mindist;
        }
    }

  uint nPts;
  for(uint i = 0; i < itsNumObjects; i++) // report the average of those distances
    {
      nPts = 0;
      dist = 0;
      for(uint j = 0; j < N; j++)
        {
          // if the object is not in the frame we do not average it
          if(distsNow[i][j] != NOT_IN_FRAME) nPts++;
          dist += distsNow[i][j];
        }
      if (nPts == 0) output += "NaN "; // to be read by matlab
      else output += sformat("%0.3f ", dist/nPts);
    }

  if (itsRegionOutFile)
    (*itsRegionOutFile) << output << std::endl;
}

// ######################################################################
void SimulationViewerEyeRegion::drawEye(const rutz::shared_ptr<EyeData> rawPos, const uint tN)
{
  // convert to retinal coordinates (accounting for any shifting,
  // embedding, etc):
  // Point2D<int> currPos = rawToRet(rawPos->position());
 
 const PixRGB<byte> col = itsEyeStyles[tN].col;
  if(itsFrameNumber >= itsObjectsInfo->getNumScenes())
    {
      LINFO("Ran out of XML scene data... (%d/%d)",itsFrameNumber,
            itsObjectsInfo->getNumScenes());
      return;
    }

  // note: these functions are tolerant to
  // coordinates outside the actual image area
  // this may cause errors if the eye coordinate is outside

  if(itsHighlightMode.getVal().compare("traced") == 0)
    {
      Object currObj =
        getSceneObj(itsCurrRegionID);

      drawFilledPolygon(itsTargetsMask, currObj.polygon, col);
      drawRegions(currObj);
    }
  if(itsObjectDrawMode.getVal().compare("traced") == 0)
    {
      Object currObj =
        getSceneObj(itsCurrRegionID);
      drawRegions(currObj);
    }
  SimulationViewerEyeMvt::drawEye(rawPos, tN);
}

// ######################################################################
void SimulationViewerEyeRegion::drawFOA(const Point2D<int> target, const uint tN)
{
  const PixRGB<byte> col = itsEyeStyles[tN].col;
  if(itsFrameNumber >= itsObjectsInfo->getNumScenes())
    {
      LINFO("Ran out of XML scene data...");
      return;
    }

  if(itsHighlightMode.getVal().compare("targeted") == 0)
    {
      uint hitObjID = findHitRegion(target);
      Object thisObj =
        getSceneObj(hitObjID);
      drawFilledPolygon(itsTargetsMask, thisObj.polygon, col);
      drawRegions(thisObj);
    }
  if(itsObjectDrawMode.getVal().compare("targeted") == 0)
    {
      uint hitObjID = findHitRegion(target);
      Object thisObj =
        getSceneObj(hitObjID);
      drawRegions(thisObj);
    }

  SimulationViewerEyeMvt::drawFOA(target,tN);
}

// ######################################################################
void SimulationViewerEyeRegion::drawRegions(Object myObj)
{
  if(itsObjectsInfo.get() == 0)
    LFATAL("No objects data - check xml file.");
  

  if(itsFrameNumber >= itsObjectsInfo->getNumScenes())
    {
      LINFO("Ran out of XML scene data...");
      return;
    }

  Image<PixRGB<byte> > shapes(itsDrawings.getDims(),ZEROS);

  Scene sceneData = itsObjectsInfo->getSceneData(itsFrameNumber);
  if(myObj.id!=NULL_OBJ || itsObjectDrawMode.getVal().compare("all") != 0)
    {
      PixRGB<byte> lineCol(0,255,0);
      if (itsRandomColoring.getVal())
        lineCol = itsRandColors[myObj.id];


      drawOutlinedPolygon(shapes, myObj.polygon,
                          lineCol,
													Point2D<int>(0,0), 0, 1.0, 0, 0,
													itsLineThickness.getVal());

      // find appropriate (i.e. upperleft most) corner to write name on...
      // define upper left corner as farthest from 
      // lowerleft/upperright diagonal of polygon
      Point2D<int> cm = centroid(myObj.polygon);
      Point2D<int> upperleft = cm - Point2D<int>(-10,10);
      
      double thisDist, maxDist = 0; 
      Point2D<int> refPt = myObj.polygon[0];
      for(uint i = 0; i < myObj.polygon.size(); i++) {
        thisDist = myObj.polygon[i].distanceToLine(cm, upperleft, true);
        if (thisDist > maxDist) {
          maxDist = thisDist;
          refPt = myObj.polygon[i];
        }
      }

      writeText(itsDrawings, refPt+Point2D<int>(10,5), 
                getObjName(myObj.id).c_str(),
                PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),
                SimpleFont::FIXED(10), true);
    }
  else { // if no obj, show all objects in this frame
    Scene thisScene = itsObjectsInfo->getSceneData(itsFrameNumber);
    for (std::vector<Object>::iterator iObj = thisScene.objects.begin();
         iObj != thisScene.objects.end(); iObj++)
      drawRegions(*iObj);
  }
  itsDrawings = composite(itsDrawings,shapes);
}


// ######################################################################
Object SimulationViewerEyeRegion::getSceneObj(uint objID, uint FrameNum)
{
  Object non_object = Object();
  non_object.name = getObjName(NULL_OBJ);
  non_object.id = NULL_OBJ;
  if(objID == NULL_OBJ) return non_object; // no object targeted
  ASSERT(FrameNum < itsObjectsInfo->getNumScenes());

  Scene sceneData = itsObjectsInfo->getSceneData(FrameNum);
  for(std::vector<Object>::iterator itrObject =
        sceneData.objects.begin(); itrObject != sceneData.objects.end(); itrObject++)
    if ((*itrObject).id==objID) return (*itrObject);

  return non_object; // object not found
}

// ######################################################################
std::string SimulationViewerEyeRegion::getObjName(uint objID)
{
  if(objID == NULL_OBJ) return "none";
  else if(objID >= itsNumObjects) LFATAL("%d illegal index (max = %d, null = %d)", objID, itsNumObjects, NULL_OBJ);
  return itsObjectsNames[objID];
}

// ######################################################################
std::string SimulationViewerEyeRegion::craftSVEMOutput(const std::string tfn,
                        const rutz::shared_ptr<EyeData> data)
{
  // Write some measurements to a string. Here, we just put all the stats
  // into a text string, and we will send that off to be
  // written to disk.

  // The first time we run this, we build a header for the file
  // to know what stats to read. This is done in the craft submodules.

  // Note: This overloads the SVEyeMvt class method craftSVEMOutput.
  // It does so to insert a 
  // In the future we may implement a policy-based method to grab different output modules.

  std::string output = SimulationViewerEyeMvt::craftSVEMOutput(tfn,data);

  // add which objects are saccaded to/from
  output += craftRegionOutput(data);

  return output;
}

// ######################################################################
std::string SimulationViewerEyeRegion::
craftRegionOutput(const rutz::shared_ptr<EyeData> data)
{
  std::string output;
  // output region of origin, region of destination

  Point2D<int> position = rawToRet(data->position());
  Point2D<int> sacTarget = rawToRet(data->saccadeTarget());
  bool giveAllRegions = true; //TODO: should be ModelParam
  if(giveAllRegions)
    {
      uint destObjID = findHitRegion(sacTarget);
      output += sformat(" %s %s %s %s",
                        getObjName(itsCurrRegionID).c_str(),
                        listAllRegionsHit(position).c_str(),
                        getObjName(destObjID).c_str(),
                        listAllRegionsHit(sacTarget).c_str());

      if(!itsHeaderCrafted)
        {
          itsOutFields.push_back("bestregion_src");
          itsOutFields.push_back("region_src");
          itsOutFields.push_back("bestregion_target");
          itsOutFields.push_back("region_target");
        }
    }
  else
    {
      uint destObjID = findHitRegion(sacTarget);
      output += sformat(" %s %s",
                        getObjName(itsCurrRegionID).c_str(),

                        getObjName(destObjID).c_str());
      if(!itsHeaderCrafted)
        {
          itsOutFields.push_back("region_src");
          itsOutFields.push_back("region_target");
        }

    }
  return output;
}

// ######################################################################
Image< PixRGB<byte> > SimulationViewerEyeRegion::getTraj(SimEventQueue& q)
{
  LDEBUG("Buffering output frame %d...", itsFrameNumber);

GVX_TRACE(__PRETTY_FUNCTION__);
  // get the latest retina image:
  Image< PixRGB<byte> > input;
  if (SeC<SimEventRetinaImage> e = q.check<SimEventRetinaImage>(this, SEQ_ANY))
    {
      input = e->frame().colorByte();

      if(itsObjectDrawMode.getVal().compare("all") == 0) 
        drawRegions();
      else if(itsObjectDrawMode.getVal().compare("selected") == 0 &&
              !itsSelectedObjects.getVal().empty())
        {
          Scene sceneData = itsObjectsInfo->getSceneData(itsFrameNumber);
          for(uint i = 0; i < itsObjectsInfo->getNumObj(itsFrameNumber); i++)
            {
              Object objData = sceneData.objects[i];

              //TODO: write rules to determine which objects to draw
              //as external opt
              std::string objName = getObjName(objData.id);
              //if(objName.compare(itsSelectedObjects.getVal()) == 0) 
              drawRegions(objData);
            }
        }
    }
  else
    LFATAL("Could not find required SimEventRetinaImage");

  // draw the highlighted regions if there are any...
  if(itsTargetsMask.initialized()) { //TODO: add && itsTargetsMask is not blank for speed
    input = alphaBlend(itsTargetsMask, input,
                       itsHitTransparency.getVal());
  }

  // reinit these here, although this should be done in onSimEventClockTick
  itsTargetsMask.resize(input.getDims(), true);

  // make a composite of the input + the drawings:
  Image<PixRGB<byte> > comp = composite(itsDrawings, input);

  // return a plain traj?
  if (itsSaveTraj.getVal()) return comp;

  // let's get the current saliency map (normalized):
  const Dims dims = input.getDims();
  Image<float> sm = getMap(q, false);
  if (sm.initialized()) sm = rescaleOpt(sm, dims, itsDisplayInterp.getVal());
  else sm.resize(dims, true); // blank
  Image< PixRGB<byte> > smc = toRGB(Image<byte>(sm));

  // make a composite of the instantaneous SM + the drawings:
  Image< PixRGB<byte> > smcomp = composite(itsDrawings, smc);

  // otherwise, return mega combo; we have two formats: if we have a
  // max-cache, it will be a 4-image format, otherwise a 2-image
  // format:
  Image< PixRGB<byte> > ret;
  if (itsMaxCacheSize.getVal())
    {
      // 4-image format
      Image<float> maxsm =
        rescaleOpt(itsMaxCache.getMax(), dims, itsDisplayInterp.getVal());
      Image< PixRGB<byte> > maxsmc = toRGB(Image<byte>(maxsm));

      ret = concatX(concatY(input, smcomp),
                    concatY(comp, composite(itsDrawings, maxsmc)));

      drawGrid(ret, dims.w()-1, dims.h()-1, 3, 3, PixRGB<byte>(128, 128, 128));
    }
  else
    {
      // 2-image format:
      ret = concatX(comp, smcomp);
      drawLine(ret, Point2D<int>(dims.w()-1, 0), Point2D<int>(dims.w()-1, dims.h()-1),
               PixRGB<byte>(255, 255, 0), 1);
      drawLine(ret, Point2D<int>(dims.w(), 0), Point2D<int>(dims.w(), dims.h()-1),
               PixRGB<byte>(255, 255, 0), 1);
    }

  // make sure image is not unreasonably large:
  while (ret.getWidth() > itsMaxComboWidth.getVal())
    ret = decY(lowPass3y(decX(lowPass3x(ret))));

  return ret;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

