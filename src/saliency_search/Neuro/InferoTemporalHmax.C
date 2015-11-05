 /*!@file Neuro/InferoTemporalHmax.C Object recognition module with Hmax */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/InferoTemporalHmax.C $
// $Id: InferoTemporalHmax.C 14157 2010-10-22 00:54:14Z dparks $
//

#include "Neuro/InferoTemporalHmax.H"
#include "HMAX/HmaxFL.H"
#include "Learn/SVMClassifier.H"
#include "Component/OptionManager.H"
#include "Component/ModelOptionDef.H"
#include "Image/MathOps.H"
#include "Image/ShapeOps.H"
#include "Image/CutPaste.H"
#include "Image/ColorOps.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/NeuroSimEvents.H"
#include "Neuro/Brain.H"
#include "Neuro/VisualCortex.H"
#include "Simulation/SimEventQueue.H"
#include "Media/MediaSimEvents.H"

#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <fstream>


const ModelOptionDef OPT_ITHMAXC1PatchesDir =
  { MODOPT_ARG_STRING, "ITC feature patches dir", &MOC_ITC, OPTEXP_CORE,
    "Directory of ordered patch files named C1Patches.<PATCH_SIZES>.<NUM_PATCHES_PER_SIZE>.<PATCH_ORIENTATIONS>.pnm, "
    "where PATCH_SIZES iterates over the number of patch scale sizes (all patches are square),  "
    "where NUM_PATCHES_PER_SIZE iterates over the number of patches for each size,  "
    "where PATCH_ORIENTATIONS iterates over the number of orientations,  "
    "each iterator goes from 0 to X-1",
    "it-hmax-c1patches-dir", '\0', "<dirname>", "" };


const ModelOptionDef OPT_ITHMAXFeatureVectorFileName =
  { MODOPT_ARG_STRING, "ITC HMAX Feature Vector File Name", &MOC_ITC, OPTEXP_CORE,
    "Output the feature vectors with their ids into a file",
    "it-hmax-feature-vector-filename", '\0', "<filename>", "" };


const ModelOptionDef OPT_ITHMAXDisableClassifier =
  { MODOPT_FLAG, "ITC HMAX Classifier Disable", &MOC_ITC, OPTEXP_CORE,
    "Disable the internal classifier module inside of the ITHmax class, designed to be used when system has an external" 
    "classifier to avoid conflicts",
    "it-hmax-disable-classifier", '\0', "<boolean>", "false" };



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
}


// ######################################################################
InferoTemporalHmax::InferoTemporalHmax(OptionManager& mgr,
                                     const std::string& descrName,
                                     const std::string& tagName) :
  InferoTemporal(mgr, descrName, tagName),
  itsHMAXStoredPatchesDir(&OPT_ITHMAXC1PatchesDir, this),
  itsHMAXFeatureVectorFileName(&OPT_ITHMAXFeatureVectorFileName, this),
  itsHMAXDisableClassifier(&OPT_ITHMAXDisableClassifier,this),
  itsClassifier(NULL)
{
}

// ######################################################################
void InferoTemporalHmax::start1()
{
  // Initialize hmax with feature learning
  std::vector<int> scss(9);
  scss[0] = 1; scss[1] = 3; scss[2] = 5; scss[3] = 7; scss[4] = 9;
  scss[5] = 11; scss[6] = 13; scss[7] = 15; scss[8] = 17;
  std::vector<int> spss(8);
  spss[0] = 8; spss[1] = 10; spss[2] = 12; spss[3] = 14;
  spss[4] = 16; spss[5] = 18; spss[6] = 20; spss[7] = 22;
  int nori = 4;

  if(itsHMAXStoredPatchesDir.getVal().compare("") == 0) {
    LFATAL("Must specify directory containing C1 Patches using --it-hmax-c1patches-dir");
  }

  if(!itsHMAXDisableClassifier.getVal())
    {
      itsClassifier = rutz::shared_ptr<SVMClassifierModule>(new SVMClassifierModule(getManager(),"",""));
    }
  else
    {
      printf("Did not initialize classifier inside of hmax\n");
    }



  // Initialize hmax
  itsHmax.init(nori,spss,scss);
  // Load the patches
  itsHmax.readInC1Patches(itsHMAXStoredPatchesDir.getVal());


  // Clear out the feature vector file if desired
  // if(itsHMAXFeatureVectorFileName.getVal().compare("") != 0) {
  //   std::ofstream c2File;
  //   c2File.open(itsHMAXFeatureVectorFileName.getVal().c_str(),std::ios::out);
  //   c2File.close();
  // }


  InferoTemporal::start1();
}

// ######################################################################
void InferoTemporalHmax::stop1()
{
  if(!itsHMAXDisableClassifier.getVal())
    {
      itsClassifier = rutz::shared_ptr<SVMClassifierModule>(NULL);
    }
}

// ######################################################################
InferoTemporalHmax::~InferoTemporalHmax()
{}

// ######################################################################
void InferoTemporalHmax::attentionShift(SimEventQueue& q,
                                            const Point2D<int>& location)
{
  int id = -1;
  std::string name = "";
  Image<float> inputf;
  Image<PixRGB<float> > objImg;

  if(!itsClassifier.is_valid())
    LFATAL("Classifier was disabled, so attention shift is not supported");

  // get the lastest input frame from the retina:
  if (SeC<SimEventRetinaImage> e = q.check<SimEventRetinaImage>(this))
    objImg = e->frame().colorByte();
  else
    LFATAL("Oooops, no input frame in the event queue?");

  // get the latest smooth mask from the shape estimator:
  Image<float> smoothMask;
  if (SeC<SimEventShapeEstimatorOutput>
      e = q.check<SimEventShapeEstimatorOutput>(this))
  {
    smoothMask = e->smoothMask();
    // crop around object using mask?
    //if (itsUseAttention.getVal())
    objImg = getCroppedObject(objImg, smoothMask);
  }
  if (!objImg.initialized()) return; // no object image, so just do nothing

  // Convert color image to grayscale using NTSC coordinates
  inputf = luminanceNTSC(objImg);
  // Pull the object data, if any
  if(itsClassifier->getMode().compare("Train") == 0)
    {
      SVMObject so;
      if(SeC<SimEventObjectDescription> d = q.check<SimEventObjectDescription>(this)) {
        rutz::shared_ptr<TestImages::ObjData> objData = d->getObjData();
        // Extract the id out of the object data
        id = objData->id;
        name = objData->name;
      }
    }
  // Calculate feature vector
  std::vector<float> featureVector = calculateFeatureVector(inputf);
  // Call classifier to determine label from feature vector
  SVMObject so = itsClassifier->determineLabel(featureVector,id,name);

  // Post the object 
  if (itsClassifier->getMode().compare("Rec") == 0)      // Recognition
  {
    rutz::shared_ptr<TestImages::ObjData> objData(new TestImages::ObjData);
    objData->id = so.id;
    if(so.initialized())
      objData->name = so.name;
    else
      objData->name = "";
    objData->maxProb  = so.confidence; //FIX ME!
    objData->normProb = so.confidence; //FIX ME!

    LINFO("OBJECT RECOGNITION: Object identified as %s[%d]\n",objData->name.c_str(),objData->id);

    rutz::shared_ptr<SimEventObjectDescription>
      objDataEvent(new SimEventObjectDescription(this, objData));

    q.post(objDataEvent);
  }
}

std::vector<float> InferoTemporalHmax::calculateFeatureVector(Image<float> img)
{
  // Allocate memory for c2 layer feature values
  float ** c2Res = _createFeatureVector();
  itsHmax.getC2(img,c2Res);
  std::vector<float> ret = _convertFeatureVector(c2Res);
  _freeFeatureVector(c2Res);

  return ret;
}

void InferoTemporalHmax::writeOutFeatureVector(std::vector<float> featureVector, int id)
{
  std::ofstream c2File;
  c2File.open(itsHMAXFeatureVectorFileName.getVal().c_str(),std::ios::app);
  if (c2File.is_open()) {
    c2File << id << " ";
    for(uint i=0;i<featureVector.size();i++) {
        c2File << std::setiosflags(std::ios::fixed) << std::setprecision(4) <<
          (i+1) << ":" << featureVector[i] << " ";
    }
    c2File << std::endl;
  }
  c2File.close();
}

float** InferoTemporalHmax::_createFeatureVector()
{
  // Allocate memory for c2 layer feature values
  uint c2dim1 = itsHmax.getC1PatchSizes().size();
  uint c2dim2 = itsHmax.getC1PatchesPerSize();
  float **c2Res = new float*[c2dim1];
  for(unsigned int i=0;i<c2dim1;i++) {
    c2Res[i] = new float[c2dim2];
  }
  return c2Res;
}

std::vector<float> InferoTemporalHmax::_convertFeatureVector(float **c2Res)
{
  std::vector<float> ret;
  // Allocate memory for c2 layer feature values
  uint c2dim1 = itsHmax.getC1PatchSizes().size();
  uint c2dim2 = itsHmax.getC1PatchesPerSize();
  for(unsigned int i=0;i<c2dim1;i++) {
    for(unsigned int j=0;j<c2dim2;j++) {
      ret.push_back(c2Res[i][j]);
    }
  }
  return ret;
}

void InferoTemporalHmax::_freeFeatureVector(float **c2Res)
{
  uint c2dim1 = itsHmax.getC1PatchSizes().size();
  for(unsigned int i=0;i<c2dim1;i++) {
    delete[] c2Res[i];
  }
  delete [] c2Res;
}


SVMObject InferoTemporalHmax::determineLabel(Image<float> objImg, int id, std::string name)
{
  SVMObject so;
  //extract features
  std::vector<float> featureVector=calculateFeatureVector(objImg);
  so=itsClassifier->determineLabel(featureVector,id,name);
  // Write out the data to a feature vector file if desired
  if(itsHMAXFeatureVectorFileName.getVal().compare("") != 0) {
    writeOutFeatureVector(featureVector,so.id);
  }
  return so;
}





// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

