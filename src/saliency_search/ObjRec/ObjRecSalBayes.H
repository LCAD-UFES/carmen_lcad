/*!@file ObjRec/ObjRecSalBayes.H Obj Reconition using SalBayes
 */

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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ObjRec/ObjRecSalBayes.H $
// $Id: ObjRecSalBayes.H 9412 2008-03-10 23:10:15Z farhan $
//

#ifndef ObjRecSalBayes_H_DEFINED
#define ObjRecSalBayes_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Image/Image.H"
#include "Image/ImageSet.H"
#include "Image/Pixels.H"
#include "Channels/DescriptorVec.H"
#include "Learn/Bayes.H"
#include "Neuro/StdBrain.H"
#include "Component/ModelManager.H"
#include "Image/Image.H"
#include "Image/ImageSet.H"
#include "Image/ShapeOps.H"
#include "Image/CutPaste.H"
#include "Image/DrawOps.H"
#include "Image/FilterOps.H"
#include "Image/ColorOps.H"
#include "Image/Transforms.H"
#include "Image/MathOps.H"
#include "Neuro/StdBrain.H"
#include "Neuro/VisualCortex.H"
#include "Neuro/NeuroOpts.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Media/TestImages.H"
#include "Media/SceneGenerator.H"
#include "Media/MediaSimEvents.H"
#include "Channels/DescriptorVec.H"
#include "Channels/ComplexChannel.H"
#include "Channels/SubmapAlgorithmBiased.H"
#include "Simulation/SimEventQueue.H"
#include "Simulation/SimulationOpts.H"
#include "Simulation/SimEventQueueConfigurator.H"
#include "Neuro/NeuroSimEvents.H"
#include "GUI/DebugWin.H"
#include "ObjRec/MaskBiaser.H"

#include <vector>
#include <limits>

class ObjRecSalBayes : public ModelComponent
{
public:

  //Constructor
  ObjRecSalBayes(ModelManager& mgr,
          const std::string& descrName = "ObjRecSalBayes",
          const std::string& tagName = "ObjRecSalBayes");
  ~ObjRecSalBayes();

  //###########################################################

  void start2();

  void train(const Image<PixRGB<byte> > &img, const std::string label);
  void finalizeTraining();
  std::string predict(const Image<PixRGB<byte> > &img);

  Point2D<int> evolveBrain(const Image<PixRGB<byte> > &img);

  void extractFeatures(const Image<PixRGB<byte> > &img);


private:
  bool itsDebug;
  int itsFoveaRadius;
  SimEventQueue itsQ;
  ModelManager *itsMgr;

  nub::soft_ref<StdBrain> itsStdBrain;
  DescriptorVec *itsDescVec;
  Bayes *itsBayesNet;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
