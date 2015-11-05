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
// $HeadURL: svn://dparks@isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/InferoTemporalHmax.C $
// $Id: InferoTemporalHmax.C 13497 2010-05-28 00:37:41Z dparks $
//

#include "Neuro/InferoTemporalCudaHmax.H"
#include "CUDA/CudaHmaxCBCL.H"
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

const ModelOptionDef OPT_ITCUDAHMAXC0PatchesFileName =
  { MODOPT_ARG_STRING, "ITC C0 feature patches file", &MOC_ITC, OPTEXP_CORE,
    "File containing c0 patches in the following format:\n"
    "NUM_PATCHES\n"
    "DEPTH WIDTH HEIGHT\n"
    "D0W0H0 D0W0H1 D0W0H2 ...\n"
    "D0W1H0 ...\n",
    "it-cudahmax-c0patches-filename", '\0', "<filename>", "" };

const ModelOptionDef OPT_ITCUDAHMAXC1PatchesFileName =
  { MODOPT_ARG_STRING, "ITC C1 feature patches file", &MOC_ITC, OPTEXP_CORE,
    "File containing c0 patches in the following format:\n"
    "NUM_PATCHES\n"
    "DEPTH WIDTH HEIGHT\n"
    "D0W0H0 D0W0H1 D0W0H2 ...\n"
    "D0W1H0 ...\n",
    "it-cudahmax-c1patches-filename", '\0', "<filename>", "" };



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
InferoTemporalCudaHmax::InferoTemporalCudaHmax(OptionManager& mgr,
                                     const std::string& descrName,
                                     const std::string& tagName) :
  InferoTemporalHmax(mgr, descrName, tagName),
  itsCUDAHMAXStoredC0PatchesFile(&OPT_ITCUDAHMAXC0PatchesFileName, this),
  itsCUDAHMAXStoredC1PatchesFile(&OPT_ITCUDAHMAXC1PatchesFileName, this)
{

}

// ######################################################################
void InferoTemporalCudaHmax::start1()
{
  // Make sure old parameter is not present
  if(itsHMAXStoredPatchesDir.getVal().compare("") != 0) {
    LFATAL("This option is not valid for CUDA based Hmax use --it-cudahmax-c1patches-filename instead");
  }
  // Make sure C0 and C1 patch files are specified
  if(itsCUDAHMAXStoredC0PatchesFile.getVal().compare("") == 0) {
    LFATAL("Must specify filename containing C0 Patches using --it-cudahmax-c0patches-filename");
  }
  if(itsCUDAHMAXStoredC1PatchesFile.getVal().compare("") == 0) {
    LFATAL("Must specify filename containing C1 Patches using --it-cudahmax-c1patches-filename");
  }  

  // Load the patches
  hmax.loadC0(itsCUDAHMAXStoredC0PatchesFile.getVal());
  hmax.loadC1(itsCUDAHMAXStoredC1PatchesFile.getVal());
  InferoTemporal::start1();
}


// ######################################################################
InferoTemporalCudaHmax::~InferoTemporalCudaHmax()
{}



std::vector<float> InferoTemporalCudaHmax::_convertFeatureVector(float *c2Res, int numC2)
{
  std::vector<float> ret;
  // Allocate memory for c2 layer feature values
  for(int i=0;i<numC2;i++) {
      ret.push_back(c2Res[i]);
  }
  return ret;
}

void InferoTemporalCudaHmax::_freeFeatureVector(float *c2Res)
{
}

// Override these inherited functions from InferoTemporaHmax
void InferoTemporalCudaHmax::_freeFeatureVector(float **c2Res)
{
  // Override
}

std::vector<float> InferoTemporalCudaHmax::_convertFeatureVector(float **c2Res)
{
  // Override
  std::vector<float> ignore;
  LFATAL("Should never call this convertFeatureVector()");
  return ignore;
}

std::vector<float> InferoTemporalCudaHmax::calculateFeatureVector(Image<float> objImg)
{
  // Allocate memory for c2 layer feature values
  float *c2Res;
  int numC2;
  //extract features
  hmax.getC2(objImg.getArrayPtr(),objImg.getWidth(),objImg.getHeight());
  c2Res = hmax.getC2Features();
  numC2 = hmax.numC2Features();
  std::vector<float> ret = _convertFeatureVector(c2Res,numC2);
  hmax.clearC2();

  return ret;
}





// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

