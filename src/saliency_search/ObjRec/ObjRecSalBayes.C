/*!@file ObjRec/ObjRecSalBayes.C Obj Reconition using SalBayes
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ObjRec/ObjRecSalBayes.C $
// $Id: ObjRecSalBayes.C 10982 2009-03-05 05:11:22Z itti $
//


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
#include "ObjRec/ObjRecSalBayes.H"

// ######################################################################
ObjRecSalBayes::ObjRecSalBayes(ModelManager& mgr, const std::string& descrName,
    const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsDebug(false),
  itsQ(mgr)

{

  itsStdBrain = nub::soft_ref<StdBrain>(new StdBrain(mgr));
  addSubComponent(itsStdBrain);

  mgr.setOptionValString(&OPT_RawVisualCortexChans, "IOC");
  mgr.setOptionValString(&OPT_SaliencyMapType, "Fast");
  mgr.setOptionValString(&OPT_SMfastInputCoeff, "1");
  mgr.setOptionValString(&OPT_TaskRelevanceMapType, "None");

  mgr.setOptionValString(&OPT_WinnerTakeAllType, "Fast");
  //mgr.setOptionValString(&OPT_SimulationTimeStep, "0.2");

  //mgr.setModelParamVal("FOAradius", 50, MC_RECURSE);
  //mgr.setModelParamVal("FoveaRadius", 50, MC_RECURSE);

  mgr.setOptionValString(&OPT_IORtype, "Disc");

  itsFoveaRadius = 50; //mgr.getModelParamVal<int>("FoveaRadius", MC_RECURSE);

  itsMgr = &mgr;

}

void ObjRecSalBayes::start2()
{
  ComplexChannel *cc =
    &*dynCastWeak<ComplexChannel>(itsStdBrain->getVC());

  itsDescVec = new DescriptorVec(*itsMgr, "Descriptor Vector", "DecscriptorVec", cc);
  //Get  new classifier
  itsBayesNet = new Bayes(itsDescVec->getFVSize(), 0);

  itsDescVec->setFoveaSize(itsFoveaRadius);

}

ObjRecSalBayes::~ObjRecSalBayes()
{
}

void ObjRecSalBayes::extractFeatures(const Image<PixRGB<byte> > &img)
{
  const int learnNumSacc = 100;
  Point2D<int> winner = evolveBrain(img); //evolve the brain
  for (int sacc=0; sacc<learnNumSacc; sacc++) //learn the n most salient points in the image
  {
    //show winner
    if (itsDebug){
      Image<PixRGB<byte> > tmpImg = img;
      drawCircle(tmpImg, winner, 50, PixRGB<byte>(255, 0, 0), 3);
      SHOWIMG(tmpImg);
    }

    //get the descriptor
    itsDescVec->setFovea(winner);

    if (itsDebug){
      SHOWIMG(itsDescVec->getFoveaImage());
    }


    itsDescVec->buildRawDV(); //build the descriptor vector

    //get the resulting feature vector
    std::vector<double> FV = itsDescVec->getFV();


    printf("%i %i %i ", sacc, winner.i, winner.j);
    for(uint i=0; i<FV.size(); i++)
      printf("%f ", FV[i]);
    printf("\n");


    Image<PixRGB<byte> > nullImg;
    winner = evolveBrain(nullImg); //evolve the biased brain to get a new winner

  }

}




void ObjRecSalBayes::train(const Image<PixRGB<byte> > &img, const std::string label)
{

  const int learnNumSacc = 1;
  Point2D<int> winner = evolveBrain(img); //evolve the brain
  for (int sacc=0; sacc<learnNumSacc; sacc++) //learn the n most salient points in the image
   {
    //show winner
    if (itsDebug){
      Image<PixRGB<byte> > tmpImg = img;
      drawCircle(tmpImg, winner, 50, PixRGB<byte>(255, 0, 0), 3);
      SHOWIMG(tmpImg);
    }

    //get the descriptor
    itsDescVec->setFovea(winner);

    if (itsDebug){
      SHOWIMG(itsDescVec->getFoveaImage());
    }


    itsDescVec->buildRawDV(); //build the descriptor vector

    //get the resulting feature vector
    std::vector<double> FV = itsDescVec->getFV();


   /* char filename[255];
    sprintf(filename, "results/%i.ppm", id++);
    Raster::WriteRGB(itsDescVec.getFoveaImage(), filename);
    printf("DV %i %s %i %ix%i: ",id, objName, sacc, winner.i, winner.j);
    for(uint i=0; i<FV.size(); i++)
      printf("%f ", FV[i]);
    printf("\n");*/

    printf("OD: '%s'  %i %i %i ",label.c_str(), sacc, winner.i, winner.j);
    for(uint i=0; i<FV.size(); i++)
      printf("%f ", FV[i]);


    itsBayesNet->learn(FV, label.c_str());

     Image<PixRGB<byte> > nullImg;
     winner = evolveBrain(nullImg); //evolve the biased brain to get a new winner

   }

}

void ObjRecSalBayes::finalizeTraining()
{

}

std::string ObjRecSalBayes::predict(const Image<PixRGB<byte> > &img)
{

  double prob = 0, statSig = 0;
  Point2D<int> winner = evolveBrain(img); //evolve the brain

  //show winner
  if (itsDebug){
    Image<PixRGB<byte> > tmpImg = img;
    drawCircle(tmpImg, winner, 50, PixRGB<byte>(255, 0, 0), 3);
    SHOWIMG(tmpImg);
  }
  //get the descriptor
  itsDescVec->setFovea(winner);
  itsDescVec->buildRawDV(); //build the descriptor vector

  //get the resulting feature vector
  std::vector<double> FV = itsDescVec->getFV();

  // printf("%i %i ", winner.i, winner.j);
  //  for(uint i=0; i<FV.size(); i++)
  //     printf("%f ", FV[i]);

  //classify

  int cls = -1;
  cls = itsBayesNet->classify(FV);

  statSig = itsBayesNet->getStatSig(FV, 0); //get the statistical significance
  LINFO("Class %i prob: %f %f\n", cls, prob, statSig);

  if (cls == -1) //check for errors
    return std::string("NOMATCH");

  std::string clsName(itsBayesNet->getClassName(cls));

  return clsName;
}


Point2D<int> ObjRecSalBayes::evolveBrain(const Image<PixRGB<byte> > &img)
{


  LINFO("Evolve Brain");

  if (img.initialized())
  {
    //place the image in the inputFrame queue
    rutz::shared_ptr<SimEventInputFrame>
      e(new SimEventInputFrame(itsStdBrain.get(), GenericFrame(img), 0));
    itsQ.post(e);
    // brain->input(img, seq);
    itsDescVec->setInputImg(img);
  }

  SimTime end_time = itsQ.now() + SimTime::MSECS(3.0);

  while (itsQ.now() < end_time)
  {
    itsStdBrain->evolve(itsQ); //evolve the brain

    // Any new WTA winner?
    if (SeC<SimEventWTAwinner> e =
        itsQ.check<SimEventWTAwinner>(itsStdBrain.get()))
    {
      const Point2D<int> winner = e->winner().p;

      //get the saliency map output
      if (itsDebug)
      {
        if (SeC<SimEventSaliencyMapOutput> smo =
            itsQ.check<SimEventSaliencyMapOutput>(itsStdBrain.get(), SEQ_ANY))
        {
          Image<float> img = smo->sm();
          SHOWIMG(rescale(img, img.getWidth()*16, img.getHeight()*16));
        }
      }
      while (itsQ.now() < end_time)
        itsQ.evolve();

      return winner;
    }

    itsQ.evolve();

  }
  return Point2D<int>();
}



// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
