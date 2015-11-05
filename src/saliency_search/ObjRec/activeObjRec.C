/*! @file ObjRec/activeObjRec.C test various obj rec alg */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ObjRec/activeObjRec.C $
// $Id: activeObjRec.C 10982 2009-03-05 05:11:22Z itti $
//

#include "Channels/SubmapAlgorithmBiased.H"
#include "Component/ModelManager.H"
#include "Component/OptionManager.H"
#include "Image/Image.H"
#include "Image/ShapeOps.H"
#include "Image/DrawOps.H"
#include "Neuro/StdBrain.H"
#include "Neuro/VisualCortexConfigurator.H"
#include "Devices/FrameGrabberConfigurator.H"
#include "Devices/IEEE1394grabber.H"
#include "Transport/FrameIstream.H"
#include "Neuro/VisualCortex.H"
#include "Neuro/NeuroOpts.H"
#include "Media/TestImages.H"
#include "Channels/DescriptorVec.H"
#include "Channels/ComplexChannel.H"
#include "Learn/Bayes.H"
#include "ObjRec/BayesianBiaser.H"

#include "GUI/XWinManaged.H"

int train(nub::soft_ref<IEEE1394grabber> &gb, DescriptorVec &descVec, Bayes &bayesNet);
int test(nub::soft_ref<IEEE1394grabber> &gb, DescriptorVec &descVec, Bayes &bayesNet);
int classifyImage(Image<PixRGB<byte> > & img, DescriptorVec &descVec, Bayes &bayesNet);
void learnImage(Image<PixRGB<byte> > & img, int cls, DescriptorVec &descVec, Bayes &bayesNet);

void biasImage( bool biasVal, Bayes &bayesNet );

Point2D<int> evolveBrain(Image<PixRGB<byte> > &img);

ModelManager *mgr;
#define NOBJ 2

XWinManaged disp(Dims(320, 240), -1, -1, "Test Output 1");
XWinManaged foveaDisp(Dims(256, 256), -1, -1, "Test Output 1");

int main(const int argc, const char **argv)
{

  MYLOGVERB = LOG_INFO;
  mgr = new ModelManager("Test ObjRec");

  //our brain
  nub::ref<StdBrain>  brain(new StdBrain(*mgr));
  mgr->addSubComponent(brain);

  //nub::soft_ref<FrameGrabberConfigurator>
  //  gbc(new FrameGrabberConfigurator(*mgr));
  nub::soft_ref<IEEE1394grabber>
    gb(new IEEE1394grabber(*mgr, "colorcam", "cocam"));

  mgr->addSubComponent(gb);


  gb->setModelParamVal("FrameGrabberSubChan", 0);
  gb->setModelParamVal("FrameGrabberBrightness", 128);
  gb->setModelParamVal("FrameGrabberHue", 180);

  mgr->exportOptions(MC_RECURSE);
  //mgr.setOptionValString(&OPT_RawVisualCortexChans, "IOC");
  //mgr.setOptionValString(&OPT_RawVisualCortexChans, "I");
  mgr->setOptionValString(&OPT_RawVisualCortexChans, "GNO");
  //mgr.setOptionValString(&OPT_RawVisualCortexChans, "N");
  //manager.setOptionValString(&OPT_UseOlderVersion, "false");
  // set the FOA and fovea radii
  mgr->setOptionValString(&OPT_SaliencyMapType, "Fast");
  mgr->setOptionValString(&OPT_WinnerTakeAllType, "Fast");
  mgr->setOptionValString(&OPT_SimulationTimeStep, "0.2");

  mgr->setModelParamVal("FOAradius", 50, MC_RECURSE);
  mgr->setModelParamVal("FoveaRadius", 50, MC_RECURSE);


  mgr->setOptionValString(&OPT_IORtype, "None");

  if (mgr->parseCommandLine(
        (const int)argc, (const char**)argv, "", 0, 0) == false);

  mgr->start();

  // get the frame grabber to start streaming:

  //Timer masterclock;                // master clock for simulations
  //Timer timer;


  ComplexChannel *cc =
    &*dynCastWeak<ComplexChannel>(brain->getVC());

  //Get a new descriptor vector
  DescriptorVec descVec(*mgr, "Descriptor Vector", "DecscriptorVec", cc);

  //Get  new classifier
        Bayes bayesNet(descVec.getFVSize(), NOBJ);



  //////////////////////////////////////////////// Main test Loop //////////////////////////////////////


  LINFO("Training");
  train(gb, descVec, bayesNet);
  bayesNet.save("objRecCoil.net");
  //bayesNet.load("objRec.net");


  LINFO("Testing");
 // getchar();
  test(gb, descVec, bayesNet);


}


int train(nub::soft_ref<IEEE1394grabber> &gb, DescriptorVec &descVec, Bayes &bayesNet)
{

  //Train with only one image
  Dims trainSize(50, 50);
  descVec.setFoveaSize(trainSize);

  while(1)
  {
    Image< PixRGB<byte> > input = gb->readRGB();
 //   input = rescale(input, trainSize); //resize the image

    LINFO("Obj learning...");
    learnImage(input, 1, descVec, bayesNet);
  }

  return 0;
}

int test(nub::soft_ref<IEEE1394grabber> &gb, DescriptorVec &descVec, Bayes &bayesNet)
{

  return 1;
}


int classifyImage(Image<PixRGB<byte> > & img, DescriptorVec &descVec, Bayes &bayesNet)
{
  Point2D<int> winner = evolveBrain(img); //evolve the brain

  //get the descriptor
  descVec.setFovea(winner);
  descVec.buildDV(); //build the descriptor vector

  //get the resulting feature vector
  std::vector<double> FV = descVec.getFV();

  // for(uint i=0; i<FV.size(); i++)
  //     LINFO("FV: %f", FV[i]);

  //classify

  int cls = bayesNet.classify(FV);

  if (cls == -1) //check for errors
    return -1;
  else
    return cls;

}

void learnImage(Image<PixRGB<byte> > & img, int cls, DescriptorVec &descVec, Bayes &bayesNet)
{

      Point2D<int> winner = evolveBrain(img); //evolve the brain

      drawCircle(img, winner, 25, PixRGB<byte>(255, 255, 0));
      disp.drawImage(img);
      //get the descriptor

      Point2D<int> loc = disp.getLastMouseClick();
      if (loc.isValid())
      {
        Dims WindowDims = disp.getDims();
        float newi = (float)loc.i * (float)img.getWidth()/(float)WindowDims.w();
        float newj = (float)loc.j * (float)img.getHeight()/(float)WindowDims.h();
        loc.i = (int)newi;
        loc.j = (int)newj;
        descVec.setFovea(loc);
        foveaDisp.drawImage(descVec.getFoveaImage());

        descVec.buildDV(); //build the descriptor vector

        //get the resulting feature vector
        std::vector<double> FV = descVec.getFV();

        double confi;
        int cls = bayesNet.classify(FV, &confi);
        //     for(uint i=0; i<FV.size(); i++)
        //    LINFO("FV: %f", FV[i]);

        LINFO("cls %i confi %f", cls, confi);
        bayesNet.learn(FV, 0u);

        if (confi > -40)
          biasImage(true, bayesNet);

      }




      /*
      descVec.buildDV(); //build the descriptor vector

      //get the resulting feature vector
      std::vector<double> FV = descVec.getFV();

 //     for(uint i=0; i<FV.size(); i++)
  //    LINFO("FV: %f", FV[i]);

      bayesNet.learn(FV, cls);*/

}

Point2D<int> evolveBrain(Image<PixRGB<byte> > &img)
{

        nub::ref<StdBrain>  brain = dynCastWeak<StdBrain>(mgr->subComponent(0));

        if (mgr->started() && img.initialized()){         //give the image to the brain
                brain->time();

                brain->input(img);

                bool keep_going = true;
                while (keep_going){
                        const SimStatus status = brain->evolve();
                        if (status == SIM_BREAK) {
                                LINFO("V %d\n", (int)(brain->time().msecs()) );
                                keep_going = false;
                        }
                        if (brain->gotCovertShift()) // new attended location
                        {

                                const Point2D<int> winner = brain->getLastCovertPos();
                                const float winV = brain->getLastCovertAgmV();

                                LINFO("##### Winner (%d,%d) at %fms : %.4f #####",
                                                winner.i, winner.j, brain->time().msecs(), winV * 1000.0f);

        return winner;

                                keep_going = false;

                        }
                        if (brain->time().secs() > 3.0) {
                                LINFO("##### Time limit reached #####");
                                keep_going = false;
                        }
                        LINFO("Evolve brain");
                }

        }

  return Point2D<int>();

}

void biasImage( bool biasVal, Bayes &bayesNet )
{
  nub::ref<StdBrain>  brain = dynCastWeak<StdBrain>(mgr->subComponent(0));
  ComplexChannel* cc = &*dynCastWeak<ComplexChannel>(brain->getVC());

  //Set mean and sigma to bias submap
  BayesianBiaser bb(bayesNet, 0, -1, biasVal);
  cc->accept(bb);

  //set the bias
  setSubmapAlgorithmBiased(*cc);
}


