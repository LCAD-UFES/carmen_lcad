 /*!@file Neuro/InferoTemporalSIFT.C Object recognition module with SIFT */

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
// Primary maintainer for this file: Sophie Marat
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/InferoTemporalSIFT.C $
// $Id: InferoTemporalSIFT.C 14244 2010-11-17 02:56:14Z sophie $
//

#include "Neuro/InferoTemporalSIFT.H"

#include "Component/OptionManager.H"
#include "Component/ModelOptionDef.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/NeuroSimEvents.H"
#include "Neuro/Brain.H"
#include "Neuro/VisualCortex.H"
#include "Neuro/ShapeEstimator.H"
#include "Simulation/SimEventQueue.H"
#include "Media/MediaSimEvents.H"
#include "SIFT/Keypoint.H"
#include "SIFT/VisualObject.H"
#include "SIFT/VisualObjectDB.H"
#include "Image/MathOps.H"
#include "Image/ShapeOps.H"
#include "Image/CutPaste.H"
#include "Image/ColorOps.H"
#include "Raster/Raster.H"
#include "Image/DrawOps.H"
#include "GUI/XWindow.H"
#include "Channels/ChannelMaps.H"
#include "Channels/ChannelOpts.H"
#include "Channels/ChannelBase.H"
#include "Channels/RawVisualCortex.H"
#include "Component/ModelManager.H"
#include "Channels/IntensityChannel.H"
#include "Channels/ColorChannel.H"
#include "Channels/OrientationChannel.H"
#include "Channels/ChannelMaps.H"
#include "Util/FileUtil.H"
#include "Util/log.H"
#include "Util/sformat.H"
#include "Util/StringUtil.H"
#include "Raster/Raster.H"

#include "Image/Conversions.H"

#include "Neuro/GistEstimatorStd.H"
#include "Component/ModelManager.H"
#include "Channels/ChannelMaps.H"
#include "GUI/XWinManaged.H"
#include "Image/FilterOps.H"
#include "Image/MathOps.H"
#include "Image/ShapeOps.H"
#include "Neuro/gistParams.H"
#include "Raster/Raster.H"
#include "Simulation/SimEvents.H"
#include "Channels/SingleChannel.H"
#include "Util/Timer.H"

#include "Component/RawGistEstimatorStd.H"

#include "Learn/SVMClassifier.H"
#include "Neuro/SVMClassifierModule.H"


#include <cstdlib>
#include <iostream>  
#include <iomanip>
#include <fstream>
#include <cmath>
#include <sstream>
#include <dirent.h>


const ModelOptionDef OPT_ITSIFTDatabase =
  { MODOPT_ARG_STRING, "ITC Database dir", &MOC_ITC, OPTEXP_CORE, 
    "Directory of the SIFT Database",
    "it-SIFT-database-dir", '\0', "<filename>", " "};
const ModelOptionDef OPT_ITCMode =
  { MODOPT_ARG_STRING, "ITC Mode", &MOC_ITC, OPTEXP_CORE,
  "The mode of the ITC. Train: is for training from some data, Test is for recognition of the objects.", "it-mode", '\0', "<Train|Test>", ""};
const ModelOptionDef OPT_ITPathMatch = 
  { MODOPT_ARG_STRING, "ITC Path Match", &MOC_ITC, OPTEXP_CORE,
    "The path of the objects to consider for matching",
    "it-SIFT-path-match", '\0', "<filename>", " "};
const ModelOptionDef OPT_ITCoarseReco =
  { MODOPT_ARG(bool), "Use Gist Coarse Reco", &MOC_ITC, OPTEXP_CORE,
    "Use gist to do a coarse pre-recognition",
    "it-gist-reco", '\0', "<true|false>", "false" };

const ModelOptionDef OPT_ITSVMTrain = 
  { MODOPT_ARG_STRING, "ITC SVM Train file save", &MOC_ITC, OPTEXP_CORE,
    "The filename where the training exemple for the SVM will be saved",
    "it-SVM-train-file", '\0', "<filename>", " "};
const ModelOptionDef OPT_ITSVMId = 
  { MODOPT_ARG_STRING, "ITC Id for the SVM trainning", &MOC_ITC, OPTEXP_CORE,
    "the Id of the object trained on",
    "it-SVM-id", '\0', "<int>", ""};   
const ModelOptionDef OPT_ITSVMClass = 
  { MODOPT_ARG_STRING, "ITC Class for the SVM trainning", &MOC_ITC, OPTEXP_CORE,
    "the class of the object trained on, Used to complete the table Id:Class",
    "it-SVM-class", '\0', "<name>", " "};   
const ModelOptionDef OPT_ITSVMModel = 
  { MODOPT_ARG_STRING, "ITC SVM Model", &MOC_ITC, OPTEXP_CORE,
    "The SVM model to use for recognition",
    "it-SVM-model", '\0', "<filename>", " "};
const ModelOptionDef OPT_ITSVMRange = 
  { MODOPT_ARG_STRING, "ITC SVM Range", &MOC_ITC, OPTEXP_CORE,
    "The range of the SVM model to rescale data before recognition",
    "it-SVM-range", '\0', "<filename>", " "};
const ModelOptionDef OPT_ITSObjNameSVM = 
  { MODOPT_ARG_STRING, "ITC Obj Name", &MOC_ITC, OPTEXP_CORE,
    "The name of the object process for training svm",
    "it-SVM-obj-name", '\0', "<name>", " "};
const ModelOptionDef OPT_ITSFileRecoSave = 
  { MODOPT_ARG_STRING, "ITC save reco res", &MOC_ITC, OPTEXP_CORE,
    "The file where the result of the recognition will be saved",
    "it-reco-save-file", '\0', "<filename>", " "};

const ModelOptionDef OPT_ITSTable = 
  { MODOPT_ARG_STRING, "ITC save table", &MOC_ITC, OPTEXP_CORE,
    "Table of the different class and their Id",
    "it-table", '\0', "<filename>", " "};

 
// ######################################################################
namespace
{
  Image<PixRGB<byte> > getCroppedObject(const Image<PixRGB<byte> >& scene,
                                        const Image<float>& smoothMask)
  {
    if (!scene.initialized())
      return Image<PixRGB<byte> >();
    if (!smoothMask.initialized())
      return Image<PixRGB<byte> >();
    const float threshold = 1.0f;
    const Rectangle r = findBoundingRect(smoothMask, threshold);
    return crop(scene, r);
  }  
  
  Image<PixRGB<byte> > getCroppedObjectGist(const Image<PixRGB<byte> >& scene,
                                            const Image<float>& smoothMask)
  {
    if (!scene.initialized())
      return Image<PixRGB<byte> >();
    if (!smoothMask.initialized())
      return Image<PixRGB<byte> >();
    const float threshold = 1.0f;
    Dims dimGist(256, 256);
    const Rectangle r = findBoundingRect(smoothMask, threshold);
    Point2D<int> rCenter = r.center();
    Rectangle rGist = Rectangle::centerDims(rCenter, dimGist);
    Image<PixRGB<byte> > imCrop;
    if (scene.rectangleOk(rGist))
      {
        imCrop = crop(scene, rGist);
        LINFO("-----------------------------the rectangle fits the image------------------------");
        LINFO("-------the image crop dim are W=%d, H=%d------------", imCrop.getWidth(), imCrop.getHeight());
      }
    else
      {
        int cx = rCenter.i;
        int cy = rCenter.j;
        int ttop = std::max(0, cy -dimGist.min()/2);
        int lleft = std::max(0, cx - dimGist.min()/2);
        int bbot = std::min(scene.getHeight() - 1, cy + dimGist.min()/2);
        int rright = std::min(scene.getWidth() - 1, cx + dimGist.min()/2);
        Rectangle rGistcrop = Rectangle::tlbrI(ttop, lleft, bbot, rright);
        imCrop = crop(scene, rGistcrop);
        LINFO("-------the rectangle is too large, computing gist around object on image------------");
        LINFO("-------the image crop dim are W=%d, H=%d------------", imCrop.getWidth(), imCrop.getHeight());
      }
    Image<PixRGB<byte> > imCropr;
    imCropr = rescaleBilinear(imCrop, 256, 256);
    return imCropr;
  }  
}

// ######################################################################

InferoTemporalSIFT::InferoTemporalSIFT(OptionManager& mgr,
                                       const std::string& descrName,
                                       const std::string& tagName) :
  InferoTemporal(mgr, descrName, tagName),
  itsSIFTStoredDatabase(&OPT_ITSIFTDatabase, this),
  itsITCMode(&OPT_ITCMode, this),
  itsPathMatch(&OPT_ITPathMatch, this),
  itsCoarseReco(&OPT_ITCoarseReco, this),
  itsTrainSVM(&OPT_ITSVMTrain, this),
  itsSVMId(&OPT_ITSVMId, this),
  itsSVMClass(&OPT_ITSVMClass, this),
  itsSVMModel(&OPT_ITSVMModel, this),
  itsSVMRange(&OPT_ITSVMRange, this),
  itsNameObj(&OPT_ITSObjNameSVM, this), ///
  itsRecoSave(&OPT_ITSFileRecoSave, this),
  itsTable(&OPT_ITSTable, this), //
  itsObjectDB(new VisualObjectDB()),
  itsnewObjectDB(new VisualObjectDB()),
  itsPDFGist(new std::map<double, int>()),
  itsVisualCortex(new RawVisualCortex(mgr)),
  itsGistEstim(new RawGistEstimatorStd(mgr))//,
  //itsClassifier(new SVMClassifierModule(mgr, "", ""))
{
  addSubComponent(itsVisualCortex);// otherwise there is only one raw visual cortex and so add the new channel to general and only one !
  LINFO("------------------------------before Set Model Param Val--------------------------");
  LINFO("------------------------------itsRaw VisualCortex chans = %s--------------------------", itsVisualCortex->getModelParamString("RawVisualCortexChans").c_str());
  //itsVisualCortex->setModelParamString("RawVisualCortexChans", "ICO"); //, MC_RECURSE);
  LINFO("------------------------------itsRaw VisualCortex chans = %s--------------------------", itsVisualCortex->getModelParamString("RawVisualCortexChans").c_str());
  LINFO("------------------------------after Set Model Param Val--------------------------");
  addSubComponent(itsGistEstim);
  // addSubComponent(itsClassifier);
}

// ######################################################################
void InferoTemporalSIFT::start1()
{
  if(itsSIFTStoredDatabase.getVal().compare("") == 0)
    {
      LFATAL("Must specify directory SIFT Database <dbname.vdb> using it-SIFT-database-dir");
    }
  if (itsSIFTStoredDatabase.getVal().empty())
    LINFO("Starting with empty visualObjectDB"); // already done above
  else
    {
      itsObjectDB->loadFrom(itsSIFTStoredDatabase.getVal());
      if(itsPathMatch.getVal().compare(" ") == 0)
        {
          LINFO("No path specified to match the object aigainst the whole database will be used");
          itsnewObjectDB = itsObjectDB;
        } 
    }  
  
  //itsVisualCortex->stop();
  LINFO("------------------------------before Set Model Param Val--------------------------");
  itsVisualCortex->setModelParamString("RawVisualCortexChans", "ICO"); //, MC_RECURSE);
  itsVisualCortex->subComponent("orientation")->setModelParamString("NumOrientations", "4");
  LINFO("------------------------------after Set Model Param Val--------------------------");
  //itsVisualCortex->start();
  LINFO("------------------------------after Set itsVisualCortexStart--------------------------");
LINFO("------------------------------before Infero temporal start1--------------------------");
  LINFO("------------------------------itsRaw VisualCortex chans = %s--------------------------", itsVisualCortex->getModelParamString("RawVisualCortexChans").c_str());
  InferoTemporal::start1();
  LINFO("------------------------------itsRaw VisualCortex chans = %s--------------------------", itsVisualCortex->getModelParamString("RawVisualCortexChans").c_str());
  LINFO("------------------------------after Infero temporal start1--------------------------");


  //LINFO("------------------------------before Set Model Param Val--------------------------");
//itsVisualCortex->setModelParamString("RawVisualCortexChans", "ICO"); //, MC_RECURSE);
  //LINFO("------------------------------after Set Model Param Val--------------------------");
}

// ######################################################################
void InferoTemporalSIFT::stop1()
{
  // save database if we have a filename for it:
  if (itsSIFTStoredDatabase.getVal().empty() == false)
    itsObjectDB->saveTo(itsSIFTStoredDatabase.getVal());
}

// ######################################################################
InferoTemporalSIFT::~InferoTemporalSIFT()
{
}

// ######################################################################
void InferoTemporalSIFT::attentionShift(SimEventQueue& q,
                                            const Point2D<int>& location)

{
  Image<PixRGB<byte> > objImg; 
  Image<PixRGB<byte> > objImgGist;
  
  // get the lastest input frame from the retina:
  if (SeC<SimEventRetinaImage> e = q.check<SimEventRetinaImage>(this, SEQ_ANY))
    { 
      // SEQ_ANY is a flag passed to SimEventQueue for returning any event marked or not
      objImg = e->frame().colorByte();
      objImgGist = objImg;
    }
  else 
    LFATAL("Oooops, no input frame in the event queue?");
  
  // get the latest smooth mask from the shape estimator:
  Image<float> smoothMask;
  if (SeC<SimEventShapeEstimatorOutput>
      e = q.check<SimEventShapeEstimatorOutput>(this, SEQ_ANY,0))
    {
      smoothMask = e->smoothMask();
    }
  else
    LINFO("no mask available");
  
  // create visual object and extract keypoints:
  rutz::shared_ptr<VisualObject> vo; 
  rutz::shared_ptr<TestImages::ObjData> objData; 
  
  ////////////////////////////////////////////////////////////////////////////
  //TRAIN MODE
  if (itsITCMode.getVal().compare("Train") == 0)
    {
      rutz::shared_ptr<TestImages::SceneData> sceneData;

      //Get the scene data, but don't mark it so we will get it on the next saccade  
      if (SeC<SimEventInputFrame> e = q.check<SimEventInputFrame>(this))//, SEQ_UNMARKED,0)) //////////////
        {
          GenericFrame gf =e->frame(); 

          LINFO("--------------------------------------------------------------------------");         
          LINFO("----------------Received the frame----------------------------------------------------------");         
          LINFO("--------------------------------------------------------------------------");         

          // XML INPUT
          if(gf.hasMetaData(std::string("SceneData")))
            {
              rutz::shared_ptr<GenericFrame::MetaData> metaData;
              metaData = gf.getMetaData(std::string("SceneData"));
              if (metaData.get() != 0)
                {
                  sceneData.dyn_cast_from(metaData);
                  std::vector<TestImages::ObjData> objVect = sceneData->objects;
                  for(uint obj=0; obj<objVect.size(); obj++)
                    {
                      objData.reset(new TestImages::ObjData(objVect[obj]));
                      if (objData->polygon.size() > 0)
                        {
                          Point2D<int> upperLeft = objData->polygon[0];
                          Point2D<int> lowerRight = objData->polygon[0];
                          for(uint k=0; k<objData->polygon.size(); k++)
                            {
                              //find the bounds for the crop
                              if (objData->polygon[k].i < upperLeft.i) upperLeft.i = objData->polygon[k].i;
                              if (objData->polygon[k].j < upperLeft.j) upperLeft.j = objData->polygon[k].j;
                              if (objData->polygon[k].i > lowerRight.i) lowerRight.i = objData->polygon[k].i;
                              if (objData->polygon[k].j > lowerRight.j) lowerRight.j = objData->polygon[k].j;
                            }    
                          // crop around object using polygon
                          Image<PixRGB<byte> > objImgTr; 
                          objImgTr = crop(objImg, Rectangle::tlbrO(upperLeft.j,upperLeft.i,lowerRight.j,lowerRight.i));

                          //GIST
                          // Add the case for the gist on neighborhood
                          Dims dimGist(512, 512);
                          Rectangle r = Rectangle::tlbrO(upperLeft.j,upperLeft.i,lowerRight.j,lowerRight.i);


                          if(itsCoarseReco.getVal())
                            {
                              Point2D<int> rCenter = r.center();
                              Rectangle rGist = Rectangle::centerDims(rCenter, dimGist);
                              Image<PixRGB<byte> > objImgTrGist; 
                              objImgTrGist = crop(objImg, rGist);                           
                              computeGist(objImgTrGist);                           
                            }

                          //SIFT
                          vo.reset(new VisualObject("NewObject", "NewObjet", objImgTr));
                          vo->setName(objData->name);
                          vo->setImageFname(itsPathMatch.getVal().c_str() + objData->name + ".png");
                          if (itsObjectDB->addObject(vo))
                            LINFO("Added VisualObject '%s' to database", vo->getName().c_str());
                          else 
                            LERROR("FAILED adding VisualObject '%s' to database -- IGNORING",
                                   vo->getName().c_str());
                        }
                    }
                  rutz::shared_ptr<SimEventITOutput>
                    objDataEvent(new SimEventITOutput(this, objData));
                  q.post(objDataEvent);
                }
            }
          else //IMAGE INPUT WITHOUT XML
            {
              // crop around object using mask
              objData.reset(new TestImages::ObjData);
              objImg = getCroppedObject(objImg, smoothMask);
              if (!objImg.initialized()) 
                {
                  return; // no object image, so just do nothing
                }

              //GIST
              if(itsCoarseReco.getVal())
                {
                  objImgGist = getCroppedObjectGist(objImgGist, smoothMask);
                  computeGist(objImgGist);
                }

              //SIFT
              std::string objName;
              if (itsNameObj.getVal().empty())
                {
                  LINFO("Enter name for new object:"); //or [RETURN] to skip trainning:");
                  std::getline(std::cin, objName, '\n');
                }
              else
                objName = itsNameObj.getValString();///
              
              if (objName.length() >0)
                {
                  vo.reset(new VisualObject("NewObject", "NewObjet", objImg));
                  LINFO("Train on %s", objName.c_str());
                  vo->setName(objName);
                  //vo->setImageFname(objName + ".png");
                  vo->setImageFname(itsPathMatch.getVal().c_str() + objName + ".png"); //for placing the obj.png at the right place in the taxonomy
                  //to draw the object considered
                  const float threshold = 1.0f;
                  const Rectangle r = findBoundingRect(smoothMask, threshold);
                  std::vector<Point2D<int> > polyvect(5);
                  polyvect[0] = r.topLeft();
                  polyvect[1] = r.topRight();
                  polyvect[2] = r.bottomRight();
                  polyvect[3] = r.bottomLeft();
                  polyvect[4] = r.topLeft();
                  objData->polygon  = polyvect; //std::vector<Point2D<int> >
                  objData->name     =objName;
                  objData->dims     = objImg.getDims();
                }
              if (itsObjectDB->addObject(vo))
                LINFO("Added VisualObject '%s' to database", vo->getName().c_str());
              else
                LERROR("FAILED adding VisualObject '%s' to database -- IGNORING",
                       vo->getName().c_str());

              rutz::shared_ptr<SimEventITOutput>
                objDataEvent(new SimEventITOutput(this, objData));
              q.post(objDataEvent);
            } 
        }

      std::ofstream tableN(itsTable.getValString().c_str(), std::ios::out | std::ios::app);
      if (tableN)
        {
          tableN << itsSVMClass.getValString() << " " << ":" << " " << itsSVMId.getValString() << " " << std::endl;
        }
      tableN.close();
      //Add a test to add only neww entry in the table


   
    }//END TRAINING MODE
  
  //TESTING MODE
  else if(itsITCMode.getVal().compare("Test") == 0)
    {
      LINFO("--------------------------Enter testing Mode---------------------");

      // crop around object using mask to compute SIFT around WTA winner
      objImg = getCroppedObject(objImg, smoothMask);
      if (!objImg.initialized()) 
        {
          return; // no object image, so just do nothing
        }

      //GIST
      if(itsCoarseReco.getVal())
        {
          objImgGist = getCroppedObjectGist(objImgGist, smoothMask);
          computeGist(objImgGist);      
          bool testFind = 0;
          std::map<double, int>::reverse_iterator iteratorPDFGist = itsPDFGist->rbegin();
          float tresholdSIFT = 1.50; //arbitrary 
          std::string nameMatchSIFT;
          while (testFind == 0 && iteratorPDFGist != itsPDFGist->rend())
            {
              gistSelect(iteratorPDFGist);
              
              //SIFT
              objData.reset(new TestImages::ObjData);;
              vo.reset(new VisualObject("NewObject", "NewObjet", objImg));
              //get the matching objects:
              std::vector< rutz::shared_ptr<VisualObjectMatch> > matches;
              const uint nmatches = itsnewObjectDB->getObjectMatchesParallel(vo, matches, VOMA_KDTREEBBF); 
              //use kd-tree fast but approximate for matching
              //const uint nmatches = itsObjectDB->getObjectMatches(vo, matches, VOMA_KDTREEBBF); 
              LINFO("the number of potemtial matches are = %d", nmatches);
              if (nmatches == 0U)
                {
                  iteratorPDFGist ++;
                  nameMatchSIFT = "No_Match";
                }
              else
                {
                  rutz::shared_ptr<VisualObjectMatch> vomOne = matches[0];
                  if (vomOne->getScore() > tresholdSIFT)
                    {
                      testFind=1;
                      rutz::shared_ptr<VisualObject> objSIFT = vomOne->getVoTest();
                      nameMatchSIFT = objSIFT->getName();
                      LINFO("----------------------object recognized with score %f-----------------", vomOne->getScore());
                    }
                  else
                    {
                      iteratorPDFGist ++;
                      nameMatchSIFT = "No_Match";
                      LINFO("------------------the SIFT score is not sufficient---------------");
                    }
                }
              
              if (testFind==1 || iteratorPDFGist==itsPDFGist->rend());
              {
                //to draw the polygon 
                const float threshold = 1.0f;
                const Rectangle r = findBoundingRect(smoothMask, threshold);
                std::vector<Point2D<int> > polyvect(5);
                polyvect[0] = r.topLeft();
                polyvect[1] = r.topRight();
                polyvect[2] = r.bottomRight();
                polyvect[3] = r.bottomLeft();
                polyvect[4] = r.topLeft();
                objData->polygon = polyvect; //std::vector<Point2D<int> >
   
                //if no match, forget it:
                if (nmatches ==0U)
                  {
                    objData->name = "No Match"; //string
                    objData->maxProb = 0;
                    objData->dims = objImg.getDims();
                  }
                else
                  { 
                    //record the result of the 1rst match found 
                    rutz::shared_ptr<VisualObjectMatch> voms = matches[0];
                    rutz::shared_ptr<VisualObject> objs = voms->getVoTest(); //get the visual object
                    objData->name        = objs->getName(); //string
                    objData->maxProb     = voms->getScore(); //double
                    objData->dims        = objImg.getDims(); //dims
                  }               
                rutz::shared_ptr<SimEventITOutput>
                  objDataEvent(new SimEventITOutput(this, objData));
                q.post(objDataEvent);
              }
            }

          std::ofstream recoOutputFile(itsRecoSave.getValString().c_str(), std::ios::out | std::ios::app);
          if(recoOutputFile)
            {
              recoOutputFile << nameMatchSIFT << std::endl;
              recoOutputFile.close();            
            }

        }
      else
        {
          //SIFT without gist
          std::string nameMatchSIFT;
          objData.reset(new TestImages::ObjData);;
          vo.reset(new VisualObject("NewObject", "NewObjet", objImg));
          //get the matching objects:
          std::vector< rutz::shared_ptr<VisualObjectMatch> > matches;
          const uint nmatches = itsnewObjectDB->getObjectMatchesParallel(vo, matches, VOMA_KDTREEBBF); 
          //use kd-tree fast but approximate for matching
          //const uint nmatches = itsObjectDB->getObjectMatches(vo, matches, VOMA_KDTREEBBF); 
          const float threshold = 1.0f;
          const Rectangle r = findBoundingRect(smoothMask, threshold);
          std::vector<Point2D<int> > polyvect(5);
          polyvect[0] = r.topLeft();
          polyvect[1] = r.topRight();
          polyvect[2] = r.bottomRight();
          polyvect[3] = r.bottomLeft();
          polyvect[4] = r.topLeft();
          objData->polygon = polyvect; //std::vector<Point2D<int> >

          LINFO("the number of potemtial matches are = %d", nmatches);
          if (nmatches == 0U)
            {
              nameMatchSIFT = "No_Match";
            }
          else
            {
              rutz::shared_ptr<VisualObjectMatch> vomOne = matches[0];
              rutz::shared_ptr<VisualObject> objSIFT = vomOne->getVoTest();
              nameMatchSIFT = objSIFT->getName();
              LINFO("----------------------object recognized with score %f-----------------", vomOne->getScore());
              
              //to draw the polygon 
              const float threshold = 1.0f;
              const Rectangle r = findBoundingRect(smoothMask, threshold);
              std::vector<Point2D<int> > polyvect(5);
              polyvect[0] = r.topLeft();
              polyvect[1] = r.topRight();
              polyvect[2] = r.bottomRight();
              polyvect[3] = r.bottomLeft();
              polyvect[4] = r.topLeft();
              objData->polygon = polyvect; //std::vector<Point2D<int> >
            }
          
          //if no match, forget it:
          if (nmatches ==0U)
            {
              objData->name = "No Match"; //string
              objData->maxProb = 0;
              objData->dims = objImg.getDims();
            }
          else
            { 
              //record the result of the 1rst match found 
              rutz::shared_ptr<VisualObjectMatch> voms = matches[0];
              rutz::shared_ptr<VisualObject> objs = voms->getVoTest(); //get the visual object
              objData->name        = objs->getName(); //string
              objData->maxProb     = voms->getScore(); //double
              objData->dims        = objImg.getDims(); //dims
            }   
          rutz::shared_ptr<SimEventITOutput>
            objDataEvent2(new SimEventITOutput(this, objData));
          q.post(objDataEvent2);
          std::ofstream recoOutputFile(itsRecoSave.getValString().c_str(), std::ios::out | std::ios::app);
          if(recoOutputFile)
            {
              recoOutputFile << nameMatchSIFT << std::endl;
              recoOutputFile.close();            
            }
        }
    }
  else
    LFATAL("Unknown IT Mode type %s", itsITCMode.getVal().c_str());
}

 // ######################################################################
 std::string InferoTemporalSIFT::getObjNameAtLoc(const std::vector<TestImages::ObjData> &objects, const Point2D<int>& loc)
 {
   //will be usefull for comparison with the ground truth given with a XML file given
   for(uint obj=0; obj<objects.size(); obj++)
     {
       TestImages::ObjData objData = objects[obj];
       //uint nbobjtoreco = objects.size();
       
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

 //vo.reset(new VisualObject("NewObject", "NewObjet", objImg));

 // ######################################################################
void InferoTemporalSIFT::getObjDBToMatch(const char *dir)
 {
   LINFO("filename is %s",  dir);
   if (isDirectory(dir))
     {
       LINFO("---------------------------------Directory not folder--------------------------");
       DIR *dp = opendir(dir);
       dirent *dirp;
       while( (dirp = readdir(dp)) )
         {
           if(dirp->d_name[0] != '.')
             {
               //std::string fil = dir;
               //fil.append(dirp->d_name);
               std::string fil = sformat("%s%s", dir, dirp->d_name);
               if (isDirectory(dirp))
                 {
                   getObjDBToMatch(fil.c_str());
                 }
               else
                 {
                   std::string lobjName = fil.c_str();
                   LINFO("--------------------the path name loaded is %s----------------------", fil.c_str());
                   //the depth of the taxonomy should be 2 file/class/item
                   //objname in the format taxonomy:class:item
                   lobjName = lobjName.erase(lobjName.rfind("."), lobjName.size()); //remove .png
                   std::string item = lobjName;
                   item = item.erase(item.find("/"), item.rfind("/")+1);
                   std::string auxname = lobjName;
                   auxname = auxname.erase(auxname.rfind("/"), auxname.size());
                   std::string classobj = auxname;
                   classobj = classobj.erase(classobj.find("/"), classobj.rfind("/")+1);
                   auxname = auxname.erase(auxname.rfind("/"), auxname.size());
                   std::string taxoname = auxname;
                   taxoname = taxoname.erase(taxoname.find("/"), taxoname.rfind("/")+1);
                   //objName = objName.erase(objName.find("/"), objName.rfind("/")+1);
                   
                   std::string objName = taxoname;
                   objName.append(":");
                   objName.append(classobj);
                   objName.append(":");
                   objName.append(item); 
                   //create a new visual object database with only the object that belong to the path
                   //itsnewObjectDB->addObject(itsObjectDB->getObject(objName));

                   rutz::shared_ptr<VisualObject> myvo;
                   myvo = itsObjectDB->getObject(objName);
                   //LINFO("--------------------------getobj 1 num %u------------------------", itsObjectDB->numObjects());
                   itsnewObjectDB->addObject(myvo);
                 }
             } //check add all the object to the visualObjectDB
         }
       closedir(dp);
     }
   else
     {
       std::string filb = sformat("%s", dir);
       std::string objName = filb.c_str();
       objName = objName.erase(objName.rfind("."), objName.size());
       objName = objName.erase(objName.find("/"), objName.rfind("/")+1);
       //create a new visual object database with only the object that belong to the path
       itsnewObjectDB->addObject(itsObjectDB->getObject(objName));
     }
 }

// ######################################################################
void InferoTemporalSIFT::computeGist(Image<PixRGB<byte> > objImg)
{

  LINFO("--------------------------Enter compute Gist ---------------------");
  LINFO("------------------------------------------------------------------");
  //itsVisualCortex->subComponent("orientation")->setModelParamVal("NumOrientations", 4);  
LINFO("-------------------------Before resetting itsVisualCortex----------------------------------------");
LINFO("------------------------------itsRaw VisualCortex chans = %s--------------------------", itsVisualCortex->getModelParamString("RawVisualCortexChans").c_str());
  itsVisualCortex->reset(MC_RECURSE);
  LINFO("-------------------------After resetting itsVisualCortex----------------------------------------");
LINFO("------------------------------itsRaw VisualCortex chans = %s--------------------------", itsVisualCortex->getModelParamString("RawVisualCortexChans").c_str());
  LINFO("------------------------------------------------------------------");
  //itsVisualCortex->setModelParamVal("RawVisualCortexChans", "ICO", MC_RECURSE);
 
  InputFrame imgIn = InputFrame::fromRgb(&objImg); 
  //INFO("--------------------------S test Cropped obj 4 ---------------------");
  //LINFO("--------------------------Size objImg H=%d W=%d ---------------------", objImg.getHeight(), objImg.getWidth());

  Image<float> vcomap = itsVisualCortex->getVCOutput(objImg);
 LINFO("Sophie:------------------------the Raw VC Output has for dim H=%d and W=%d----------------------", vcomap.getHeight(), vcomap.getWidth());

  // WORKS but comment to check the pyramide size
  rutz::shared_ptr<ChannelMaps> chMaps(new ChannelMaps(itsVisualCortex.get()));
  LINFO("------------get all the %u channel maps at the same time---------",chMaps->numSubchans());

  //debug
  for(uint i=0; i < chMaps->numSubchans(); i++)
    {
      //Grab the current channel
      rutz::shared_ptr<ChannelMaps> currChan = chMaps->subChanMaps(i);
      //Determine the name of the channel
      std::vector<std::string> chanNameVec;
      split(currChan->getMap().name(), ":", std::back_inserter(chanNameVec));
      std::string chanName = chanNameVec[1];
      LINFO("------------------------------------channel Map name is %s------------------------", chanName.c_str());
      LINFO("------------------------------------channel Map has %u subchans ------------------------", currChan->numSubchans());
      for(uint k=0; k < currChan->numSubchans(); k++)
        {
          rutz::shared_ptr<ChannelMaps> subsubchan = currChan->subChanMaps(k);
          //Determine the name of the channel
          std::vector<std::string> subchanNameVec;
          split(subsubchan->getMap().name(), ":", std::back_inserter(subchanNameVec));
          std::string subchanName = subchanNameVec[1];
          LINFO("------------------------------------sub channel Map name is %s------------------------", subchanName.c_str());
        }
      
    }





  
  // gist feature vector
  Image<float> gistVector;
  gistVector = itsGistEstim->compute(chMaps);
  int idOb;
  std::string objName;

  LINFO("----------the gist has been computed-------------------------------");
  
  if(itsITCMode.getVal().compare("Train") == 0)
    {
      //to save the image crop used to compute the Gist
      /*std::string fileGist;
      fileGist = "/lab/sophie/SIFT_Gist_Darpa/results/Crop_Gist/";
      fileGist.append(itsNameObj.getValString());
      fileGist.append("_cropGist.png");
      Raster::WriteRGB(objImg,fileGist);
      */

      std::string objId;    
      idOb = atof(itsSVMId.getValString().c_str());
    }
  else
    {
      idOb = 0;
      //objName = "";
    }
  std::vector<float> vectGist(714);
  getVectorColumn(gistVector, &vectGist, 0);
    

  //Classifier
  SVMClassifier objClassifier;  
  if(itsITCMode.getVal().compare("Train") == 0)
    {
      objClassifier.train(itsTrainSVM.getValString(), idOb, vectGist);
    }
  if(itsITCMode.getVal().compare("Test") == 0)
    {   
      LINFO("------------------");
      objClassifier.readModel(itsSVMModel.getValString());
      objClassifier.readRange(itsSVMRange.getValString());
      double labelP;
      double probaP;
      labelP = objClassifier.predict(vectGist, &probaP);
      
      LINFO("----------------------------the label of the object is %f, with a probablility=%f--------------", labelP, probaP);
      
      std::map<int, double> pdf = objClassifier.predictPDF(vectGist);
      std::vector<int> labelVal;
      for(std::map<int,double>::iterator pdfIt = pdf.begin(); pdfIt != pdf.end(); ++pdfIt)
        {
          labelVal.push_back(pdfIt->first);
          itsPDFGist->insert(std::make_pair(pdfIt->second, pdfIt->first));
          //LINFO("-----------------pdfitsecond %f   pdfitfirst %d -------------", pdfIt->second, pdfIt->first);// debug
        }
      //bool testmap2 = itsPDFGist->empty();
      //LINFO("------------------------------map is empty %d --------------------------", testmap2);
      unsigned int nbLabel = labelVal.size();
      LINFO("------------------------There is %d differnt label--------------------", nbLabel);
      std::ofstream recoOutputFile(itsRecoSave.getValString().c_str(), std::ios::out | std::ios::app);
      if(recoOutputFile)
        {
          recoOutputFile << itsSVMId.getValString() << " " << itsNameObj.getValString() << " " << labelP << " " ;
                
          for(unsigned int it=0; it < labelVal.size(); ++it)
            {
              int itlabel = labelVal[it];
              double valproba = pdf[itlabel];
              recoOutputFile << itlabel << ":" << valproba << " ";
              //LINFO("-----------------the object %d is belonging to category %d with proba=%f-------------", idOb, itlabel, valproba);
            } 
          recoOutputFile.close();
        }
   
      //test new map
      std::vector<int> Valbis;
      for (std::map<double, int>::iterator ite = itsPDFGist->begin(); ite != itsPDFGist->end(); ++ite)
        {
          Valbis.push_back(ite->first);
          // LINFO("-----------------the value %f is from the category  %d ------------",ite->first, ite->second);        
        }
     
    } 
}

// ######################################################################

void  InferoTemporalSIFT::gistSelect(std::map<double, int>::reverse_iterator iteratorPDFGist)
{
   std::map<int, std::string> tableIdClass;
  std::ifstream tableNa(itsTable.getValString().c_str(), std::ios::in);
   
  if (tableNa)
    {
      while( !tableNa.eof() )
        {       
          int Id;
          std::string categ;
          std::string aux;
          // std::getline(std::cin, objName, '\n');
          tableNa >> categ >> aux >> Id;
          tableIdClass.insert(std::make_pair(Id, categ));
          // LINFO("---------------------------------categ=%s---------------------", categ.c_str());
          //LINFO("---------------------------------id=%d---------------------", Id);
        }
      tableNa.close();
    }

  std::string filenamerecogist;
  
  //loop on map to find the matching point
  //possibility to add trick as the map will be sort by Id
  bool resF = 0;
  
std::map<int,std::string>::iterator tabIt = tableIdClass.begin(); 
 while (tabIt != tableIdClass.end() && resF == 0)
   {
     LINFO("------------------------gist said %d--------------------------", iteratorPDFGist->second);
     LINFO("------------------------comparing to %d %s--------------------------", tabIt->first, tabIt->second.c_str());

     if(iteratorPDFGist->second == tabIt->first)
       {
         resF = 1;
         filenamerecogist = itsPathMatch.getValString();
         //filenamerecogist.append("/");
         //FOR THE DEMO NOT REAL TAXONOMY!!
         //if (tabIt->second =="candy" || tabIt->second =="tea")
         //  filenamerecogist = filenamerecogist.append("food/");
         //else if (tabIt->second=="case" || tabIt->second=="lotion")
         //  filenamerecogist = filenamerecogist.append("object/");
         //else
         //  filenamerecogist = filenamerecogist.append("document/");
         
         filenamerecogist.append(tabIt->second);  //itsSVMClass.getValString());
         filenamerecogist.append("/");
         //LINFO("-------------------------------filenameRecogist-----------------------%s----------------------------", filenamerecogist.c_str());
            }
     else if(iteratorPDFGist->second < tabIt->first)
       {
         LINFO("------------------------ERROR NO MATCH FOUND CHECK THE TABLE IS NOT CORRECT---------------------");
         tabIt ++;
       }
     else
       {
         tabIt ++;
         LINFO("------just test the other class----");
       }
   }
 
 LINFO("--------------try to match with SIFT-------");
 
 getObjDBToMatch(filenamerecogist.c_str());
    
   
    
    
  }


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

