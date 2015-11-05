/*! @file ObjRec/test-SceneRec.C test various scene rec  alg */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ObjRec/test-SceneRec.C $
// $Id: test-SceneRec.C 10982 2009-03-05 05:11:22Z itti $
//


#include "Channels/SubmapAlgorithmBiased.H"
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
#include "Neuro/VisualCortexConfigurator.H"
#include "Neuro/VisualCortex.H"
#include "Neuro/SaliencyMap.H"
#include "Neuro/NeuroOpts.H"
#include "Media/TestImages.H"
#include "Media/SceneGenerator.H"
#include "Channels/DescriptorVec.H"
#include "Channels/ComplexChannel.H"
#include "Simulation/SimEventQueue.H"
#include "Simulation/SimulationOpts.H"
#include "Simulation/SimEventQueueConfigurator.H"
#include "Learn/Bayes.H"
//#include "GUI/DebugWin.H"
#include "ObjRec/BayesianBiaser.H"


#define OBJSIZEX 256
#define OBJSIZEY 256
#define NOBJ 25


int train(TestImages &testImages, DescriptorVec &descVec, Bayes &bayesNet);
int test(TestImages &testImages, DescriptorVec &descVec, Bayes &bayesNet);
int trainTest(TestImages &testImages, DescriptorVec &descVec, Bayes &bayesNet);
int classifyImage(Image<PixRGB<byte> > & img, DescriptorVec &descVec, Bayes &bayesNet);
void learnImage(Image<PixRGB<byte> > & img, int cls, DescriptorVec &descVec, Bayes &bayesNet,
                const char *objName = NULL);
int classifyLocation(Point2D<int> &loc, DescriptorVec &descVec, Bayes &bayesNet);
void learnLocation(Point2D<int> &loc, int cls, DescriptorVec &descVec, Bayes &bayesNet,
                   const char *objName = NULL);
void biasVC(ComplexChannel &vc, Bayes &bayesNet, int objId);
Point2D<int> evolveBrain(Image<PixRGB<byte> > &img, DescriptorVec& descVec, int ii=-1);

//#include "ObjRec/evalCoil.H"
//#include "ObjRec/evalCSCLAB.H"
//#include "ObjRec/evalLABELME.H"
#include "ObjRec/evalALOI.H"


ModelManager *mgr;


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

  mgr->setModelParamVal("FOAradius", OBJSIZEX/2, MC_RECURSE);
  mgr->setModelParamVal("FoveaRadius", OBJSIZEY/2, MC_RECURSE);

  mgr->setOptionValString(&OPT_IORtype, "Disc");


  if (mgr->parseCommandLine(
        (const int)argc, (const char**)argv, "BayesNet biasObj bias train=1/search=0", 4, 4) == false)
      exit(0);

  mgr->start();

  ComplexChannel *cc =
    &*dynCastWeak<ComplexChannel>(brain->getVC());

  //Get a new descriptor vector
  DescriptorVec descVec(*mgr, "Descriptor Vector", "DecscriptorVec", cc);

  //get command line options
  const char *bayesNetFile = mgr->getExtraArg(0).c_str();
  int biasedObj = mgr->getExtraArgAs<int>(1)-1;
  bool setBias = mgr->getExtraArgAs<int>(2);
  int train = mgr->getExtraArgAs<int>(3);


  //Get  new classifier
  printf("Biasing for %i", biasedObj);

  Bayes bayesNet(descVec.getFVSize(), NOBJ);
  bayesNet.load(bayesNetFile);


  //Bias the visual cortex to find the obj
  if (setBias)
    biasVC(*cc, bayesNet, biasedObj);

  //evalCOIL(descVec, bayesNet, NOBJ);
  // evalCSCLAB(descVec, bayesNet, NOBJ);
  // evalLABELME(descVec, bayesNet, NOBJ);
  evalALOI(descVec, bayesNet, NOBJ, biasedObj, train);

  if (train)
    bayesNet.save(bayesNetFile);


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
        brain->input(img, seq);
        descVec.setInputImg(img);
      }

    bool keep_going = true;
    while (keep_going){
      brain->evolve(*seq);
      const SimStatus status = seq->evolve();
      if (status == SIM_BREAK) {
        LINFO("V %d\n", (int)(seq->now().msecs()) );
        keep_going = false;
      }
      if (brain->gotCovertShift()) // new attended location
        {

          const Point2D<int> winner = brain->getLastCovertPos();
          const float winV = brain->getLastCovertAgmV();

          LINFO("##### Winner (%d,%d) at %fms : %.4f #####\n",
                winner.i, winner.j, seq->now().msecs(), winV * 1000.0f);

          //Image<float> img = brain->getSM()->getV(false);
         // Image<float> img = brain->getVC()->getOutput();
         // SHOWIMG(img);
          /* char filename[255];
             sprintf(filename, "SceneSMap%i.ppm", ii++);
             Raster::WriteRGB(img, filename);*/

          return winner;

          keep_going = false;

        }
      if (seq->now().secs() > 3.0) {
        LINFO("##### Time limit reached #####");
        keep_going = false;
      }
      LINFO("Evolve brain");
    }

  }

  return Point2D<int>();

}


void biasVC(ComplexChannel &vc, Bayes &bayesNet, int objId)
{
  //Set mean and sigma to bias submap
  BayesianBiaser bb(bayesNet, objId, -1, true);
  vc.accept(bb);

  setSubmapAlgorithmBiased(vc);
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

void learnImage(Image<PixRGB<byte> > & img, int cls, DescriptorVec &descVec, Bayes &bayesNet,
                const char *objName)
{
  Point2D<int> winner = evolveBrain(img, descVec); //evolve the brain

  //get the descriptor
  descVec.setFovea(winner);
  descVec.buildRawDV(); //build the descriptor vector

  //get the resulting feature vector
  std::vector<double> FV = descVec.getFV();

  //     for(uint i=0; i<FV.size(); i++)
  //    LINFO("FV: %f", FV[i]);

  if (objName != NULL)
    bayesNet.learn(FV, objName);
  else
    bayesNet.learn(FV, cls);

}

int classifyLocation(Point2D<int> &loc, DescriptorVec &descVec, Bayes &bayesNet)
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

  int cls = bayesNet.classify(FV);


  if (cls == -1) //check for errors
    return -1;
  else
    return cls;

}

void learnLocation(Point2D<int> &loc, int cls, DescriptorVec &descVec, Bayes &bayesNet,
                   const char *objName)
{

  //get the descriptor
  descVec.setFovea(loc);
  descVec.buildRawDV(); //build the descriptor vector

  //get the resulting feature vector
  std::vector<double> FV = descVec.getFV();

  //      printf("FV: ");
  //      for(uint i=0; i<FV.size(); i++)
  //        printf("%f ", FV[i]);
  //      printf("\n");

  if (objName != NULL)
    bayesNet.learn(FV, objName);
  else
    bayesNet.learn(FV, cls);

}

