/*!@file Robots2/Beobot2/Navigation/Gist_Navigation/Gist_Navigation.C Ice Module to log data    */
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
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Gist_Navigation.C
// $ $Id: Gist_Navigation.C 12962 2010-03-06 02:13:53Z irock $
//
//////////////////////////////////////////////////////////////////////////

#include "Robots/Beobot2/Navigation/Gist_Navigation/Gist_Navigation.H"
#include "Ice/BeobotEvents.ice.H"

#include "Raster/Raster.H"
#include "Util/sformat.H"
#include "Image/Image.H"
#include "Image/DrawOps.H"
#include "Image/ColorOps.H"  // for luminance()
#include "Image/MathOps.H"  // for stdev()
#include "Image/MatrixOps.H"  // for matrixMult()
#include "Image/CutPaste.H"  // for inplacePaste()


#include "SIFT/Keypoint.H"
#include "SIFT/VisualObject.H"
#include "SIFT/VisualObjectMatch.H"
#include "Transport/FrameInfo.H"

#include "Ice/IceImageUtils.H"

#include "Gist/trainUtils.H"

#define  GIST_DATA_FOLDER   "../data/train/gistFeat/"
#define  TRAIN_DATA_FOLDER  "../data/train/"
#define  LEARN_RATE         0.025
#define  NUM_DIRECTIONS     13

// ######################################################################
Gist_Navigation::Gist_Navigation(OptionManager& mgr,
               const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsFftComputer(new GistEstimatorFFT(mgr)),
  itsOfs(new OutputFrameSeries(mgr)),
  itsFfn(new FeedForwardNetwork()),
  itsTimer(1000000),
  itsCurrImgID(-1),
  itsPrevProcImgID(-1)
{
  //addSubComponent(itsFftComputer);
  addSubComponent(itsOfs);

  // initialize the feed forward network
  initFFN();

  // setup the PCA eigenvector
  itsPcaIcaVector =
    setupPcaIcaMatrix("../data/train/E_FastICA.evec", 640, 40);
}

// ######################################################################
void Gist_Navigation::initFFN()
{
  // instantiate a 3-layer feed-forward network
  // initialize with the provided parameters
  //FFNtrainInfo pcInfo(manager.getExtraArg(0).c_str());
  //ffn_place->init3L(pcInfo.h1Name, pcInfo.h2Name, pcInfo.oName,
  //                  pcInfo.redFeatSize, pcInfo.h1size, pcInfo.h2size,
  //                  pcInfo.nOutput, 0.0, 0.0);

  itsFfn->init3L
    (sformat("%sh1.nnwt",TRAIN_DATA_FOLDER),
     sformat("%sh2.nnwt",TRAIN_DATA_FOLDER),
     sformat("%so.nnwt", TRAIN_DATA_FOLDER),
     40, 100, 50, NUM_DIRECTIONS, LEARN_RATE, 0.0);
}

// ######################################################################
Gist_Navigation::~Gist_Navigation()
{ }

// ######################################################################
void Gist_Navigation::start1()
{
}

// ######################################################################
void Gist_Navigation::registerTopics()
{
  // subscribe to all sensor data
  this->registerSubscription("CameraMessageTopic");
  this->registerSubscription("MotorMessageTopic");
  this->registerPublisher("MotorRequestTopic");
}

// ######################################################################
void Gist_Navigation::evolve()
{
  // check if the current image is updated
  its_Curr_Img_mutex.lock();
  bool newImageFlag = (itsPrevProcImgID < itsCurrImgID);
  its_Curr_Img_mutex.unlock();

  // if so, process
  if(newImageFlag)
    {
      itsTimer.reset();

      its_Curr_Img_mutex.lock();
      itsProcImg = itsCurrImg;
      itsPrevProcImgID = itsCurrImgID;
      its_Curr_Img_mutex.unlock();

      //process

      //     std::string saveFName(sformat("ImageLog/image_%015d.ppm",currRequestID));
      //     LINFO("saving: %s",saveFName.c_str());
      //     Raster::WriteRGB(ima, saveFName);
      //     std::string line =
      //       sformat("[%15.3f] CAM filename: %s", time/1000.0, saveFName.c_str());
      //     line += std::string("\n");

      // normalize image
      itsNormalizedProcImg = normalize(Image<float>(luminance(itsProcImg)));

      // compute FFT
      itsFftComputer->computeGistFeatureVector(itsNormalizedProcImg);
      itsFftFeatures = itsFftComputer->getGist();

      // reduce the dimension
      Image<double> in = matrixMult(itsPcaIcaVector, itsFftFeatures);

//       LINFO("A[%d %d]", itsPcaIcaVector.getHeight(), itsPcaIcaVector.getWidth());
//       LINFO("B[%d %d]", itsFftFeatures.getHeight(),  itsFftFeatures.getWidth());

//       for(int i = 0; i < itsPcaIcaVector.getHeight(); i++)
//         for(int j = 0; j < itsPcaIcaVector.getWidth(); j++)
//           LINFO("PCA[%3d %3d]: %f", i, j, itsPcaIcaVector.getVal(j,i));

//       for(int i = 0; i < itsFftFeatures.getHeight(); i++)
//         for(int j = 0; j < itsFftFeatures.getWidth(); j++)
//           LINFO("FFT[%3d %3d]: %f", i, j, itsFftFeatures.getVal(j,i));

//       LINFO("C[%d %d]", in.getHeight(), in.getWidth());

//       for(int i = 0; i < in.getHeight(); i++)
//         LINFO("dir[%3d]: %f", i, in.getVal(0,i));

//       Raster::waitForKey();

      // run the NN
      itsFfn->run3L(in);

      // Calculate the direction from NN probabilistically
      // ORIGINAL ACKERMAN CODE
      //double target = (double)rand()/(double)RAND_MAX;
      //double sum = 0.0; double tprob = 0.0;
      //uint dir;
      //for(dir = 0; dir < NUM_DIRECTIONS; dir++)
      //  sum += itsFfn->getOutput().getVal(0,dir);
      //for(dir = 0; dir < NUM_DIRECTIONS; dir++)
      //  {
      //    tprob += (itsFfn->getOutput().getVal(0,dir)/sum);
      //    if (tprob > target) break;
      //  }
      //LINFO("target: %f, sum: %f ", target, sum);
      //for(uint i = 0; i < NUM_DIRECTIONS; i++)
      //  {
      //    if(i == dir)
      //      LINFO("NN_Dir [%3d]: %.5f -> %f ***** GO HERE *****", i,
      //            itsFfn->getOutput().getVal(i),
      //            itsFfn->getOutput().getVal(i)/sum);
      //    else
      //      LINFO("NN_Dir [%3d]: %.5f -> %f", i,
      //            itsFfn->getOutput().getVal(i),
      //            itsFfn->getOutput().getVal(i)/sum);
      //  }


      itsNNrotCommand.resize(1, NUM_DIRECTIONS, ZEROS);
      itsRCrotCommand.resize(1, NUM_DIRECTIONS, ZEROS);
                        double sum = 0.0;
      // Calculate the direction from NN probabilistically
      uint dir = 0; double max = 0.0;
      for(uint i = 0; i < NUM_DIRECTIONS; i++)
        {
          itsNNrotCommand.setVal
            (0, i,
             .90 * itsFfn->getOutput().getVal(0,i) +
             .10 * (double)rand()/(double)RAND_MAX   );
          if(itsNNrotCommand.getVal(0, i) > max)
            { dir = i; max = itsNNrotCommand.getVal(0, i); }
                                        sum += itsNNrotCommand.getVal(0,i);
        }
                        //Normalize
      for(uint i = 0; i < NUM_DIRECTIONS; i++)
                        {
                                itsNNrotCommand.setVal(0,i,itsNNrotCommand.getVal(0,i)/sum);
                        }
//      for(uint i = 0; i < NUM_DIRECTIONS; i++)
//        {
//          if(i == dir)
//            LINFO("NN_Dir [%3d]: %.5f -> %.5f ***** GO HERE *****", i,
//                  itsFfn->getOutput().getVal(0,i),
//                  itsNNrotCommand.getVal(i));
//          else
//            LINFO("NN_Dir [%3d]: %.5f -> %.5f", i,
//                  itsFfn->getOutput().getVal(0,i),
//                  itsNNrotCommand.getVal(i));
//        }

      // get command from Remote Control (if available)
      int half_ND = NUM_DIRECTIONS/2;
      double rot   = double(half_ND - int(dir))/double(half_ND);
      double trans = 1.0;

      LINFO("[R:%f  T:%f]", rot, trans);

      // check if RC is biasing
      int rcMode;
      double rcTrans, rcRot;
      its_Curr_Mtr_mutex.lock();
      rcMode  = itsRemoteMode;
      rcRot   = itsRcRotSpeed;
      rcTrans = itsRcTransSpeed;
      its_Curr_Mtr_mutex.unlock();
      LINFO("RC[M:%d R:%f  T:%f]", rcMode, rcRot, rcTrans);

       for(uint i = 0; i < NUM_DIRECTIONS; i++)
         {
                                         itsRCrotCommand.setVal(0,i,0);
         }
      // learn (if rc is biasing)
      itsTrainMode = (rcMode == 3 && (rcRot > .1 || rcRot < -.1));
      if(itsTrainMode) // FIXXX: SETUP ENUMERATION OF RC_MODE
        {
          LINFO("\nTRAINING\n");

          uint bDir = uint((half_ND)*(1.0 - rcRot));
          if(bDir >=0 &&  bDir < NUM_DIRECTIONS)
            {
              itsRCrotCommand.setVal(0, bDir, 1.0);

              // give some weight to nearby orientations.
              if (bDir > 0)
                itsRCrotCommand.setVal(0, bDir-1, 0.5);
              if (bDir > 1)
                itsRCrotCommand.setVal(0, bDir-2, 0.25);
              if (bDir < NUM_DIRECTIONS - 1)
                itsRCrotCommand.setVal(0, bDir+1, 0.5);
              if (bDir < NUM_DIRECTIONS - 2)
                itsRCrotCommand.setVal(0, bDir+2, 0.25);
            }
          else LFATAL("CORRUPTED rcRot: %f", rcRot);

//                                        for(uint i = 0; i < NUM_DIRECTIONS; i++)
//                                        {
//                                                LINFO("RC_Dir [%3d]: %.5f", i, itsRCrotCommand.getVal(i));
//                                        }


          itsFfn->backprop3L(itsRCrotCommand);

          // save new weights
          itsFfn->write3L
            (sformat("%sh1.nnwt",TRAIN_DATA_FOLDER),
             sformat("%sh2.nnwt",TRAIN_DATA_FOLDER),
             sformat("%so.nnwt", TRAIN_DATA_FOLDER));

          // set the RC rotation as rotation command
          rot   = rcRot;
        }

      // TODO: RC motor cap related issues
      //       FIX BeoPilot RC command input

      // setup navigation command and send it to BeoPilot
      LINFO("Final[T:%f  R:%f]", trans, rot);
      updateMotor(trans, rot);

      //drawState();
      //itsOfs->writeRGB(itsDispImg, "Gist_nav", FrameInfo("Gist_nav",SRC_POS));
    }
}

// ######################################################################
// normalize image by subtracting it with mean and dividing by stdev
Image<float> Gist_Navigation::normalize(Image<float> img)
{
  // ORIGINAL ACKERMAN & ITTI 2005 normalize code
//     /************** 2: Do Image Preproc ****************/
//     // extract Value (brightness/luminance/intensity/whatever) from HSV
//     total = 0.0;
//     iptr  = bwimage;
//     while (data != ima.end())
//     {
//       double max = 0.0;
//       if (data->red()   > max) max = data->red();
//       if (data->green() > max) max = data->green();
//       if (data->blue()  > max) max = data->blue();
//       *iptr = max;
//       data++;
//       total += *iptr++;
//     }

//     //normalize
//     mean = total / ((double)IMAGESIZE);
//     std = 0.0;
//     for(i = 0; i < IMAGESIZE; i++)
//       std += pow(bwimage[i]-mean,2);
//     std /= IMAGESIZE-1;
//     std = sqrt(std);
//     for(i = 0; i < IMAGESIZE; i++)
//     {
//       bwimage[i] -= mean;
//       bwimage[i] /= std;
//     }

  // our approach
  Image<float> tImg = img;
  double tMean  = float(mean(img));
  double tStdev = float(stdev(img));
  tImg -= tMean;
  tImg /= tStdev;

  return tImg;
}

// ######################################################################
void Gist_Navigation::drawState()
{
  uint w = itsProcImg.getWidth();
  uint h = itsProcImg.getHeight();

  itsDispImg.resize(w*2, 2*h, NO_INIT);

  // original image
  inplacePaste(itsDispImg, itsProcImg, Point2D<int>(0, 0));

  // normalized image
  Image<float> dispNorm = itsNormalizedProcImg;
  inplaceNormalize(dispNorm, 0.0f, 255.0f);
  Image<PixRGB<byte> > dn = Image<PixRGB<byte> >(toRGB(dispNorm));
  inplacePaste(itsDispImg, dn, Point2D<int>(w, 0));

  // Fourier Transform Image
  Image<float> fftImg = itsFftComputer->getFftImage();
  inplaceNormalize(fftImg, 0.0f, 255.0f);
  Image<PixRGB<byte> > fft = Image<PixRGB<byte> >(toRGB(fftImg));
  inplacePaste(itsDispImg, fft, Point2D<int>(0, h));

        std::vector<SimpleMeter> NNrotCommandMeters(13,SimpleMeter(20,60,0,100));
        std::vector<SimpleMeter> RCrotCommandMeters(13,SimpleMeter(20,60,0,100));

        char buffer[128];
        for(uint i = 0; i < NUM_DIRECTIONS; i++)
        {

                sprintf(buffer, "%3d",int(itsNNrotCommand.getVal(i)*100));

                Point2D<int> pastePoint = Point2D<int>(
                                        10+NNrotCommandMeters[i].getWidth()*i*1.05+w,
                                        2*h -15
                                        );

                writeText(itsDispImg, pastePoint, buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));

                pastePoint = Point2D<int>(
                                10+NNrotCommandMeters[i].getWidth()*i*1.05+w,
                                2*h - NNrotCommandMeters[i].getHeight() - 30
                                );
                inplacePaste(itsDispImg, NNrotCommandMeters[i].render(itsNNrotCommand.getVal(i)*100), pastePoint );

                sprintf(buffer, "%3d",int(itsRCrotCommand.getVal(i)*100));

                pastePoint = Point2D<int>(
                                        10+NNrotCommandMeters[i].getWidth()*i*1.05+w,
                                        2*h - RCrotCommandMeters[i].getHeight() - 45
                                        );

                writeText(itsDispImg, pastePoint, buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));

                pastePoint = Point2D<int>(
                                10+RCrotCommandMeters[i].getWidth()*i*1.05+w,
                                2*h - 2*RCrotCommandMeters[i].getHeight() - 60
                                );
                inplacePaste(itsDispImg, NNrotCommandMeters[i].render(itsRCrotCommand.getVal(i)*100), pastePoint );
        }

}

// ######################################################################
void Gist_Navigation::updateMessage
(const RobotSimEvents::EventMessagePtr& eMsg, const Ice::Current&)
{
  // camera message
  if(eMsg->ice_isA("::BeobotEvents::CameraMessage"))
  {
    // store the image
    BeobotEvents::CameraMessagePtr cameraMsg =
      BeobotEvents::CameraMessagePtr::dynamicCast(eMsg);

    int currRequestID = cameraMsg->RequestID;
    Image<PixRGB<byte> > img = Ice2Image<PixRGB<byte> >(cameraMsg->image);

    LDEBUG("Got a CameraMessage with Request ID = %d", currRequestID);

    its_Curr_Img_mutex.lock();
    itsCurrImg = img;
    itsCurrImgID = cameraMsg->RequestID;
    its_Curr_Img_mutex.unlock();
  }
  // motor message
  else if(eMsg->ice_isA("::BeobotEvents::MotorMessage"))
  {
    BeobotEvents::MotorMessagePtr mtrMsg =
      BeobotEvents::MotorMessagePtr::dynamicCast(eMsg);
    LDEBUG("Got a MotorMessage with Request ID = %d: RC Trans %f ,Rc Rot %f",
          mtrMsg->RequestID, itsRcTransSpeed, itsRcRotSpeed);
    its_Curr_Mtr_mutex.lock();
    itsRemoteMode = mtrMsg->rcMode;
    itsRcTransSpeed = mtrMsg->rcTransVel;
    itsRcRotSpeed = mtrMsg->rcRotVel;
    its_Curr_Mtr_mutex.unlock();
  }
}

// ######################################################################
void Gist_Navigation::updateMotor(double tran, double rot)
{
    BeobotEvents::MotorRequestPtr msg = new BeobotEvents::MotorRequest;
    msg->transVel = tran;
    msg->rotVel   = rot;
    this->publish("MotorRequestTopic", msg);
//    LINFO("[%d] Publish motor request", itsPrevProcImgID);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
