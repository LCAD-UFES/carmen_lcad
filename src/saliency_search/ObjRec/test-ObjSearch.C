/*! @file ObjRec/test-ObjSearch.C test various visual search alg */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ObjRec/test-ObjSearch.C $
// $Id: test-ObjSearch.C 10982 2009-03-05 05:11:22Z itti $
//


#include "Component/ModelManager.H"
#include "Image/Image.H"
#include "Image/ImageSet.H"
#include "Image/ShapeOps.H"
#include "Image/DrawOps.H"
#include "Image/FilterOps.H"
#include "Image/ColorOps.H"
#include "Image/Transforms.H"
#include "Image/MathOps.H"
#include "Neuro/StdBrain.H"
#include "Neuro/VisualCortex.H"
#include "Neuro/VisualCortexConfigurator.H"
#include "Neuro/NeuroOpts.H"
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
#include "Learn/Bayes.H"
#include "GUI/DebugWin.H"
#include "ObjRec/BayesianBiaser.H"


int classifyImage(Image<PixRGB<byte> > & img, DescriptorVec &descVec, Bayes &bayesNet);
int classifyLocation(Point2D<int> &loc, DescriptorVec &descVec, Bayes &bayesNet,
    double *prob, double *statSig, std::vector<Bayes::ClassInfo> &classesInfo);
void biasVC(ComplexChannel &vc, Bayes &bayesNet, int objId);
Point2D<int> evolveBrain(Image<PixRGB<byte> > &img, DescriptorVec& descVec, int ii=-1);
int checkWinnerLoc(TestImages &testImages, uint scene, Point2D<int> &winner, std::string biasedObj);
std::string getWinnerLoc(TestImages &testImages, uint scene, Point2D<int> &winner);

ModelManager *mgr;

const char* Labels[14]={"house", "residential", "commercial", "agriculture",  "road",  "airport",
  "bridge",  "submerged house", "submerged residential", "submerged commercial",
  "submerged agriculture", "submerged road", "submerged airport",  "submerged bridge"};
int foveaRadius = 0;


int main(const int argc, const char **argv)
{

  MYLOGVERB = LOG_INFO;
  mgr = new ModelManager("Test ObjRec");

  nub::soft_ref<SimEventQueueConfigurator>
    seqc(new SimEventQueueConfigurator(*mgr));
  mgr->addSubComponent(seqc);

  //our brain
  nub::ref<StdBrain>  brain(new StdBrain(*mgr));
  mgr->addSubComponent(brain);

  mgr->exportOptions(MC_RECURSE);
  mgr->setOptionValString(&OPT_RawVisualCortexChans, "IOC");
  //mgr.setOptionValString(&OPT_RawVisualCortexChans, "I");
  //mgr->setOptionValString(&OPT_RawVisualCortexChans, "GNO");
  //mgr.setOptionValString(&OPT_RawVisualCortexChans, "N");
  //manager.setOptionValString(&OPT_UseOlderVersion, "false");
  // set the FOA and fovea radii
  mgr->setOptionValString(&OPT_SaliencyMapType, "Fast");
  mgr->setOptionValString(&OPT_SMfastInputCoeff, "1");

  mgr->setOptionValString(&OPT_WinnerTakeAllType, "Fast");
  mgr->setOptionValString(&OPT_SimulationTimeStep, "0.2");

  mgr->setModelParamVal("FOAradius", 50, MC_RECURSE);
  mgr->setModelParamVal("FoveaRadius", 50, MC_RECURSE);


  mgr->setOptionValString(&OPT_IORtype, "Disc");


  if (mgr->parseCommandLine(
        (const int)argc, (const char**)argv, "<Network file> <scenes set xml file> <obj to bias>", 3, 3) == false)
    return 1;

  mgr->start();

  bool debug = 1;
  ComplexChannel *cc =
    &*dynCastWeak<ComplexChannel>(brain->getVC());

  //Get a new descriptor vector
  DescriptorVec descVec(*mgr, "Descriptor Vector", "DecscriptorVec", cc);
  //Get  new classifier
  Bayes bayesNet(descVec.getFVSize(), 1000);

  const char *bayesNetFile = mgr->getExtraArg(0).c_str();
  const char *imageSetFile = mgr->getExtraArg(1).c_str();
  bayesNet.load(bayesNetFile);

  //get command line options
  int objToBias = bayesNet.getClassId(mgr->getExtraArg(2).c_str());

  foveaRadius = mgr->getModelParamVal<int>("FoveaRadius", MC_RECURSE);

  LINFO("******* Biasing for %i\n", objToBias);
  printf("Biasing for %i:%s\n", objToBias, mgr->getExtraArg(2).c_str());

  //load the images
  TestImages testImages(imageSetFile, TestImages::XMLFILE);

  descVec.setFoveaSize(foveaRadius);


  //bias for the object
  if (objToBias != -1)
  {
    //objToBias++;
    biasVC(*cc, bayesNet, objToBias);
  }


  int totalScenes = 0; //the number of scenes presented to the network

  for (uint scene=0; scene<testImages.getNumScenes(); scene++) //look at all the scenes
  {
    TestImages::SceneData sceneData = testImages.getSceneData(scene);

    totalScenes++;
    Image<PixRGB<byte> > sceneImg = testImages.getScene(scene);

    LINFO("Display scene %i", scene);
    Point2D<int> winner = evolveBrain(sceneImg, descVec); //evolve the biased brain

    int sacc = 1;
    printf("scene:%i:%s b:%i \n", scene, sceneData.filename.c_str(), objToBias);
    Image<PixRGB<byte> > tmp = sceneImg;
    Point2D<int> lastWinner = winner;

    char logfile[255];
    sprintf(logfile, "%s.dat", sceneData.filename.c_str());
    FILE *fp = fopen(logfile, "w");

    for(sacc=0; sacc<30; sacc++)
    {
      //classify the object under the fovea
      double prob = 0, statsSig = 0;
      std::vector<Bayes::ClassInfo> classesInfo;
      printf("Classify: %i\n", sacc);
      int cls = classifyLocation(winner, descVec, bayesNet,
          &prob, &statsSig, classesInfo);
      //int foundClsId = bayesNet.getClassId(objData.description.c_str());

      //printf("(%ix%i):", winner.i, winner.j);
      std::string trueClsName = getWinnerLoc(testImages, scene, winner);
      printf("%i:%i:%f:%f\n",
          bayesNet.getClassId(trueClsName.c_str()), cls, prob, statsSig);

      fprintf(fp, "%i %i %i %i ",
          winner.i, winner.j,
          bayesNet.getClassId(trueClsName.c_str()), cls);
      for (uint i=0; i<classesInfo.size(); i++)
        fprintf(fp, "%i %f %f ",
            classesInfo[i].classID, classesInfo[i].prob,
            classesInfo[i].statSig);
      fprintf(fp, "\n");

      // printf("lum %i Obj %i is class %i BiasedObj %i\n", lum, objId, cls, biasedObj);
     // LINFO("Object %i %s: Class %i:%s BiasingFor: %i",
      //    foundClsId, objData.description.c_str(),
      //    cls, bayesNet.getClassName(cls), objToBias);

     // if (debug)
     // {
        drawLine(tmp, lastWinner, winner, PixRGB<byte>(255, 0, 0), 3);
        if (bayesNet.getClassId(trueClsName.c_str()) ==  cls)
          drawCircle(tmp, winner, foveaRadius, PixRGB<byte>(0, 255, 0), 3);
        else
          drawCircle(tmp, winner, foveaRadius, PixRGB<byte>(255, 0, 0), 3);

        lastWinner = winner;
        //testImages.labelScene(scene, tmp);

     // }

     //if (objFound)
     //   break;

     Image<PixRGB<byte> > nullImg;
     winner = evolveBrain(nullImg, descVec); //evolve the biased brain to get a new winner

    }
    fclose(fp);


    if (debug)
    {
      //draw the labeled outline
      char info[255];
      if (objToBias != -1)
      {
        sprintf(info, "Biasing for %s\n", bayesNet.getClassName(objToBias));
        writeText(tmp, Point2D<int>(0,0), info, PixRGB<byte>(255), PixRGB<byte>(0));
      }

      testImages.labelScene(scene, tmp);
    }


    printf("] found in %i saccades\n", sacc);
    char filename[255];
    sprintf(filename, "%s_sacc.ppm", sceneData.filename.c_str());
    LINFO("Write image %s\n", filename);
    testImages.labelScene(scene, tmp);
    Raster::WriteRGB(tmp, filename);
    //SHOWIMG(tmp);

  }
  printf("Total scenes %i\n", totalScenes);

// stop all our ModelComponents
  mgr->stop();

  return 0;

}

void biasVC(ComplexChannel &vc, Bayes &bayesNet, int objId)
{
  //Set mean and sigma to bias submap
  BayesianBiaser bb(bayesNet, objId, -1, true);
  vc.accept(bb);

  setSubmapAlgorithmBiased(vc);
}

Point2D<int> evolveBrain(Image<PixRGB<byte> > &img, DescriptorVec& descVec, int ii)
{

  nub::ref<StdBrain>  brain = dynCastWeak<StdBrain>(mgr->subComponent("Brain"));
  nub::ref<SimEventQueueConfigurator> seqc =
    dynCastWeak<SimEventQueueConfigurator>(mgr->subComponent("SimEventQueueConfigurator"));
  nub::soft_ref<SimEventQueue> seq  = seqc->getQ();

  LINFO("Evolve Brain");

  if (mgr->started()){    //give the image to the brain

    if (img.initialized())
      {
        //place the image in the inputFrame queue
        rutz::shared_ptr<SimEventInputFrame>
          e(new SimEventInputFrame(brain.get(), GenericFrame(img), 0));
        seq->post(e);
       // brain->input(img, seq);
        descVec.setInputImg(img);
      }

    SimTime end_time = seq->now() + SimTime::MSECS(3.0);

    while (seq->now() < end_time)
    {
      brain->evolve(*seq); //evolve the brain

      // Any new WTA winner?
      if (SeC<SimEventWTAwinner> e = seq->check<SimEventWTAwinner>(brain.get()))
      {
       const Point2D<int> winner = e->winner().p;

        //get the saliency map output
       // if (SeC<SimEventSaliencyMapOutput> smo =
       //     seq->check<SimEventSaliencyMapOutput>(brain.get(), SEQ_ANY))
       // {
       //   Image<float> img = smo->sm();
       //   //SHOWIMG(rescale(img, img.getWidth()*16, img.getHeight()*16));
       // }
        seq->evolve();
        return winner;
      }


      seq->evolve();
      LINFO("Evolve 1\n");

    }
  }

  return Point2D<int>();

}


int classifyImage(Image<PixRGB<byte> > & img, DescriptorVec &descVec, Bayes &bayesNet)
{
  Point2D<int> winner = evolveBrain(img, descVec); //evolve the brain

  //get the descriptor
  descVec.setFovea(winner);
  descVec.buildRawDV(); //build the descriptor vector

  //get the resulting feature vector
  std::vector<double> FV = descVec.getFV();

  // printf("%i %i ", winner.i, winner.j);
  //  for(uint i=0; i<FV.size(); i++)
  //     printf("%f ", FV[i]);

  //classify

  int cls = bayesNet.classify(FV);


  if (cls == -1) //check for errors
    return -1;
  else
    return cls;

}


int classifyLocation(Point2D<int> &loc, DescriptorVec &descVec, Bayes &bayesNet,
    double *prob, double *statSig, std::vector<Bayes::ClassInfo> &classesInfo)
{

  //get the descriptor
  descVec.setFovea(loc);
  descVec.buildRawDV(); //build the descriptor vector

  //get the resulting feature vector
  std::vector<double> FV = descVec.getFV();

  //classify
  //  printf("FV: ");
  //  for(uint i=0; i<FV.size(); i++)
  //    printf("%f ", FV[i]);
  //  printf("\n");

  int cls = -1;
  if (prob != NULL)
    classesInfo = bayesNet.classifyRange(FV, cls);
    //cls = bayesNet.classify(FV, prob);
  else
    cls = bayesNet.classify(FV);

  if (cls == -1) //check for errors
    return -1;
  else
    return cls;

}

int checkWinnerLoc(TestImages &testImages, uint scene, Point2D<int> &winner, std::string biasedObj)
{

  LINFO("Checkign for %s at %ix%i", biasedObj.c_str(), winner.i, winner.j);
  for(uint obj=0; obj<testImages.getNumObj(scene); obj++)
  {
    TestImages::ObjData objData = testImages.getObjectData(scene, obj, false);

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
          if(testImages.pnpoly(objData.polygon, winner)) //if the point is outsize the image
          {
            if (objData.description == biasedObj)
            {
              printf("Match %s with %s\n", objData.description.c_str(), biasedObj.c_str());
              return 1;
            }
          }
        }
    }

  }

  return 0;
}

std::string getWinnerLoc(TestImages &testImages, uint scene, Point2D<int> &winner)
{

  for(uint obj=0; obj<testImages.getNumObj(scene); obj++)
  {
    TestImages::ObjData objData = testImages.getObjectData(scene, obj, false);

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
          if(testImages.pnpoly(objData.polygon, winner)) //if the point is outsize the image
          {
            return objData.description.c_str();
            /*if (objData.description == biasedObj)
            {
              printf("Match %s with %s\n", objData.description.c_str(), biasedObj.c_str());
              return 1;
            }*/
          }
        }
    }

  }

  return std::string("NULL");
}
