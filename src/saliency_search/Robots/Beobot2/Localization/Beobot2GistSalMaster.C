/*!@file Robots2/Beobot2/Localization/Beobot2GistSalMaster.C
  Vision robot localization using a combination saliency and gist.
  Run app-Beobot2GistSalMaster at X1 to run Gist-Saliency model
  Run app-Beobot2_GistSalLocalizerMaster at X1 to run SIFT recognition master    
  Run app-Beobot2_GistSalLocalizerWorker at X[2 ... 8] to run SIFT recognition workers    
  see Siagian_Itti09tr                                                  */
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/Beobot2/Localization/Beobot2GistSalMaster.C $
// $Id: Beobot2GistSalMaster.C 15441 2012-11-14 21:28:03Z kai $
//

#include "Robots/Beobot2/Localization/Beobot2GistSalMaster.H"

#include "Image/MathOps.H"      // for findMax
#include "Image/CutPaste.H"     // for inplacePaste()
#include "Image/DrawOps.H"
#include "Image/FilterOps.H"
#include "Image/PyramidOps.H"
#include "Image/ShapeOps.H"
#include "Image/Transforms.H"


#ifndef BEOBOT2GISTSALMASTERI_C
#define BEOBOT2GISTSALMASTERI_C

#define ERR_INTERVAL 5000

//! get all the necessary information from the visual cortex
void getBbmtResults
( nub::soft_ref<BeobotBrainMT> bbmt,
  Image<PixRGB<byte> > &currIma, Image<double> &cgist,
  Image<float> &currSalMap, ImageSet<float> &cmap,
  std::vector<Point2D<int> > &salpt, std::vector<std::vector<double> > &feat,
  std::vector<Rectangle> &objRect,
  BeobotEvents::GistSalMessagePtr msg);

//! display results
void dispResults
( Image< PixRGB<byte> > disp, rutz::shared_ptr<XWinManaged> win,
  Image< PixRGB<byte> > ima, Image< PixRGB<byte> > prevIma,
  std::vector<Point2D<int> > clmpt, std::vector<Point2D<int> > nlmpt,
  Image<float> currSalMap,
  std::vector<Point2D<int> > salpt, std::vector<Rectangle> objRect);

// ######################################################################
Beobot2GistSalMasterI::Beobot2GistSalMasterI
(OptionManager& mgr, const std::string& descrName, 
 const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsIfs(new InputFrameSeries(mgr)),
  itsBbmt(new BeobotBrainMT(mgr)),
  itsCurrImageID(-1),
  itsLastProcessedImageMessageID(-1),
  itsInputFrameRate(0),
  itsInputTimer(1000000)
{
  addSubComponent(itsIfs);
  addSubComponent(itsBbmt);

  itsCmap.reset(NUM_CHANNELS);

  // assume we do not have a camera inputting image
  itsReceivedCameraMessage = false;
}

// ######################################################################
void Beobot2GistSalMasterI::start1()
{
  //int w = itsIfs->getWidth(), h = itsIfs->getHeight();
  int w = 160, h = 120;
  std::string dims = convertToString(Dims(w, h));
  LINFO("image size: [%4dx%4d]", w, h);

  // image buffer and window for display:
  //itsDisp = Image<PixRGB<byte> >(w * 5, h, ZEROS);
  // //itsDisp = Image<PixRGB<byte> >(w, h, ZEROS);
  //itsMwin.reset
  //  (new XWinManaged(itsDisp.getDims(), 0, 0, "Master window"));

  itsNextFrameFlag = true;
  itsInputTimer.reset();
  itsTimes.clear();
}

// ######################################################################
Beobot2GistSalMasterI::~Beobot2GistSalMasterI()
{ }

// ######################################################################
void Beobot2GistSalMasterI::registerTopics()
{
  // publish gist and salient region information
  this->registerPublisher("GistSalMessageTopic");
  this->registerPublisher("AbortMessageTopic");

  this->registerSubscription("NextFrameMessageTopic");
  this->registerSubscription("CameraMessageTopic");
}

// ######################################################################
void Beobot2GistSalMasterI::evolve()
{
  // check if we have the next frame to process
  if(!haveNextFrame()) return;

  // get the next frame and start processing
  int msg_id = -1; Image<PixRGB<byte> > ima = getNextFrame(msg_id); 

  // check if the input sequence is out of frames
  if(!ima.initialized()) { printStatistics(); abort(); }

  // process saliency and gist
  itsBbmt->input(ima);

  // wait until the saliency and gist computation is ready
  Timer bbmtTimer(1000000);
  // this takes 40 ms or so
  while(!itsBbmt->outputReady()) usleep(100);
  float bbmtTime = bbmtTimer.get()/1000.0F;
  LINFO("BBMT Time: %f", bbmtTime);

  // prepare the Sal Regions and Gist features message
  BeobotEvents::GistSalMessagePtr msg =
    new BeobotEvents::GistSalMessage;

  // get all the necesary information from V1:
  Image<PixRGB<byte> > currIma; Image<float> currSalMap;
  Image<double> cgist;
  std::vector<Point2D<int> > salpt;
  std::vector<std::vector<double> >feat;
  std::vector<Rectangle> objRect;
  getBbmtResults(itsBbmt, currIma, cgist, currSalMap,
                 itsCmap, salpt, feat, objRect, msg);
  msg->RequestID = msg_id;  

  // display the results
  // //itsMwin->drawImage(currIma,0,0);
  //itsMwin->setTitle(sformat("msg_id: %d", msg_id).c_str());      
  //dispResults(itsDisp, itsMwin, currIma, prevIma, 
  //            itsClmpt, itsNlmpt,
  //            currSalMap, salpt, objRect);
  
  its_bbmt_processing_mutex.lock();
  itsLastProcessedImageMessageID = msg_id;
  its_bbmt_processing_mutex.unlock();

  // publish the message
  LINFO("Publishing Message with ID: %d", msg_id);
  publish("GistSalMessageTopic", msg);

  // print statistics once in a while
  if(msg_id > 0 && msg_id%20 == 0) printStatistics();
}

// ######################################################################
bool Beobot2GistSalMasterI::haveNextFrame()
{
  its_bbmt_processing_mutex.lock();
  int last_proc_msg_id = itsLastProcessedImageMessageID;
  its_bbmt_processing_mutex.unlock();

  its_Current_Image_mutex.lock();
  bool receivedCameraMessage = itsReceivedCameraMessage;
  int msg_id = itsCurrImageID;  
  its_Current_Image_mutex.unlock();

  // process from outside camera image
  if(receivedCameraMessage) return (msg_id > last_proc_msg_id);
 
  // process from inside
  if((itsInputFrameRate == 0 && itsNextFrameFlag) ||
     (itsInputFrameRate != 0 &&
      itsInputTimer.get() > (itsInputFrameRate - ERR_INTERVAL)))
    {
      return true;
    }
  else return false;
}

// ######################################################################
Image<PixRGB<byte> > Beobot2GistSalMasterI::getNextFrame(int& msg_id)
{
  Image<PixRGB<byte> > ima; 

  // print last processing loop first
  its_bbmt_processing_mutex.lock();
  int last_proc_msg_id = itsLastProcessedImageMessageID;
  its_bbmt_processing_mutex.unlock();
  if(last_proc_msg_id != -1)
    {
      float loopTime = itsInputTimer.get()/1000.0F;
      itsTimes.push_back(loopTime);
      LINFO("[%5d] Time: %f", last_proc_msg_id, loopTime);
    }

  // if from outside camera 
  its_Current_Image_mutex.lock();
  bool receivedCameraMessage = itsReceivedCameraMessage;

  if(receivedCameraMessage)
    { ima = itsCurrImage; msg_id = itsCurrImageID; }
  its_Current_Image_mutex.unlock();

  // if from inside   
  if(!receivedCameraMessage)
    {
      // NOTE: will block up if called faster than framerate
      itsIfs->updateNext(); ima = itsIfs->readRGB();
      if(ima.getWidth() != 160 && ima.getHeight() != 120)
        ima = rescale(ima, 160,120);
      
      its_Current_Image_mutex.lock();
      itsNextFrameFlag = false;
      itsCurrImageID++;
      msg_id = itsCurrImageID;
      itsCurrImage = ima; 
      its_Current_Image_mutex.unlock();
    }

  itsInputTimer.reset();
  return ima;
}

// ######################################################################
void Beobot2GistSalMasterI::printStatistics()
{
  // display the loop statistics
  double meanTime, minTime, maxTime, stdevTime;
  Image<double> t1(1, itsTimes.size(),NO_INIT);
  for(uint i = 0; i < itsTimes.size(); i++)
    t1.setVal(0, i, double(itsTimes[i]));

  meanTime = mean(t1);
  getMinMax(t1, minTime, maxTime);
  stdevTime = stdev(t1);

  LINFO("Time: %f (%f - %f) std: %f", 
        meanTime, minTime, maxTime, stdevTime);
}

// ######################################################################
void Beobot2GistSalMasterI::abort()
{
  // send an abort message
  BeobotEvents::AbortMessagePtr msg = new BeobotEvents::AbortMessage;
  LINFO("Publishing AbortMessage");
  publish("AbortMessageTopic", msg);
  
  sleep(5); exit(0);
}

// ######################################################################
void Beobot2GistSalMasterI::updateMessage
(const RobotSimEvents::EventMessagePtr& eMsg, const Ice::Current&)
{
  // get a next frame message from the localizer
  if(eMsg->ice_isA("::BeobotEvents::NextFrameMessage"))
    {
      LINFO("NextFrameMessage - go to the next frame");
      if(itsInputFrameRate == 0)
        {
          its_Current_Image_mutex.lock();
          itsNextFrameFlag = true;
          its_Current_Image_mutex.unlock();
        }
    }

  // if we get a camera message
  else if(eMsg->ice_isA("::BeobotEvents::CameraMessage"))
    {
      // get the input image
      BeobotEvents::CameraMessagePtr cameraMsg =
        BeobotEvents::CameraMessagePtr::dynamicCast(eMsg);
      int currRequestID = cameraMsg->RequestID;
      Image<PixRGB<byte> > ima = 
        Ice2Image<PixRGB<byte> >(cameraMsg->image);

      if(ima.getWidth() != 160 && ima.getHeight() != 120)
        ima = rescale(ima, 160,120);

      // save it for evolve
      its_Current_Image_mutex.lock();
      if(!itsReceivedCameraMessage)
        { itsTimes.clear(); itsReceivedCameraMessage = true; }
      itsCurrImage             = ima; 
      itsCurrImageID           = currRequestID;      
      its_Current_Image_mutex.unlock();

      //LINFO("CameraMessage[%6d]: %fms", currRequestID, tim.get()/1000.0F);
    }

}

// ######################################################################
bool checkInput(int opMode, bool resetNextLandmark, uint64 inputFrameRate,
                uint64 inputTime)
{
  // FIXXX currently unused

  // if train mode: need to have a ventral reset signal
  if(opMode == TRAIN_MODE && resetNextLandmark) return true;

  // if test mode and using infinite time: need reset signal
  if(opMode == TEST_MODE && inputFrameRate == 0 &&
     resetNextLandmark) return true;

  // else test mode and need to get in after time is up
  if(opMode == TEST_MODE && inputFrameRate != 0 &&
     (inputFrameRate - ERR_INTERVAL) < inputTime) return true;

  return false;
}

// ######################################################################
void getBbmtResults
( nub::soft_ref<BeobotBrainMT> bbmt,
  Image<PixRGB<byte> > &currIma, Image<double> &cgist,
  Image<float> &currSalMap, ImageSet<float> &cmap,
  std::vector<Point2D<int> > &salpt,  std::vector<std::vector<double> > &feat,
  std::vector<Rectangle> &objRect,
  BeobotEvents::GistSalMessagePtr msg)
{
  // current image, gist vector, and saliency map
  currIma    = bbmt->getCurrImage();
  cgist      = bbmt->getGist();
  currSalMap = bbmt->getSalMap();

  // current conspicuity maps
  for(uint i = 0; i < NUM_CHANNELS; i++) cmap[i] = bbmt->getCurrCMap(i);

  salpt.clear(); objRect.clear();
  uint numpt = bbmt->getNumSalPoint();
  for(uint i = 0; i < numpt; i++)
    {
      salpt.push_back(bbmt->getSalPoint(i));
      objRect.push_back(bbmt->getObjRect(i));

      // store the salient point location
      BeobotEvents::SalientRegion salReg;
      salReg.salpt.i = salpt[i].i;
      salReg.salpt.j = salpt[i].j;
      salReg.objRect.tl.i = objRect[i].topLeft().i;
      salReg.objRect.tl.j = objRect[i].topLeft().j;
      salReg.objRect.br.i = objRect[i].bottomRight().i;
      salReg.objRect.br.j = objRect[i].bottomRight().j;
      LDEBUG("[%4d] sp[%4d,%4d] rect[%4d,%4d,%4d,%4d]",
             i, salpt[i].i, salpt[i].j,
             objRect[i].topLeft().i,objRect[i].topLeft().j,
             objRect[i].bottomRight().i,objRect[i].bottomRight().j);

      std::vector<double> features; bbmt->getSalientFeatures(i, features);
      feat.push_back(features);

      // store the salient feature vector
      for(uint j = 0; j < features.size(); j++)
        {
          salReg.salFeatures.push_back(feat[i][j]);
          LDEBUG("[%4d]:%7f", j, feat[i][j]);
        }
      msg->salientRegions.push_back(salReg);
    }

  // store the input image
  msg->currIma = Image2Ice(currIma);

  // store the conspicuity maps
  for(uint i = 0; i < NUM_CHANNELS; i++)
    msg->conspicuityMaps.push_back(Image2Ice(cmap[i]));

  // store the gist in the message
  for(uint i = 0; i < cgist.size(); i++)
    msg->gistFeatures.push_back(cgist[i]);
}

// ######################################################################
void dispResults
( Image< PixRGB<byte> > disp, rutz::shared_ptr<XWinManaged> win,
  Image< PixRGB<byte> > ima, Image< PixRGB<byte> > prevIma,
  std::vector<Point2D<int> > clmpt, std::vector<Point2D<int> > nlmpt,
  Image<float> currSalMap,
  std::vector<Point2D<int> > salpt, std::vector<Rectangle> objRect)
{
  int w = ima.getWidth();
  int h = ima.getHeight();
  const int foa_size = std::min(w, h) / 12;

  // display input image:
  inplacePaste(disp, ima, Point2D<int>(0, 0));

  // display the saliency map
  Image<float> dispsm = quickInterpolate(currSalMap * SMFAC, 1 << sml);
  inplaceNormalize(dispsm, 0.0f, 255.0f);
  Image<PixRGB<byte> > sacImg = Image<PixRGB<byte> >(toRGB(dispsm));
  for(uint i = 0; i < objRect.size(); i++)
    drawRect(sacImg,objRect[i],PixRGB<byte>(255,0,0));
  inplacePaste(disp, sacImg, Point2D<int>(w, 0));

  // draw coordinates of fixation for both input and sal image
  for(uint i = 0; i < salpt.size(); i++)
    {
      Point2D<int> salpt2(salpt[i].i + w, salpt[i].j);
      drawDisk(disp, salpt[i], foa_size/6+2, PixRGB<byte>(20, 50, 255));
      drawDisk(disp, salpt[i], foa_size/6,   PixRGB<byte>(255, 255, 20));
      drawDisk(disp, salpt2,   foa_size/6+2, PixRGB<byte>(20, 50, 255));
      drawDisk(disp, salpt2,   foa_size/6,   PixRGB<byte>(255, 255, 20));
    }

  // draw the SE bounding box
  Image< PixRGB<byte> > roiImg = ima;
  for(uint i = 0; i < objRect.size(); i++)
    {
      drawRect(roiImg, objRect[i], PixRGB<byte>(255,255,0));
      drawDisk(roiImg, salpt[i], 3, PixRGB<byte>(255,0,0));
      std::string ntext(sformat("%d", i));
      writeText(roiImg, salpt[i], ntext.c_str());
    }
  inplacePaste(disp, roiImg, Point2D<int>(w*2, 0));

  // if there is a previous image
  if(prevIma.initialized())
    {
      // display the current landmark tracked
      Image< PixRGB<byte> > clmDisp = prevIma;
        for(uint i = 0; i < clmpt.size(); i++)
          {
            if(clmpt[i].isValid())
              {
                drawDisk(clmDisp, clmpt[i], 3, PixRGB<byte>(0,255,0));
                std::string ntext(sformat("%d", i));
                writeText(clmDisp, clmpt[i], ntext.c_str());
              }
          }
        inplacePaste(disp, clmDisp, Point2D<int>(w*3, 0));

        // display the next landmark tracked
        Image< PixRGB<byte> > nlmDisp = prevIma;
        for(uint i = 0; i < nlmpt.size(); i++)
          {
            if(nlmpt[i].isValid())
              {
                drawDisk(nlmDisp, nlmpt[i], 3, PixRGB<byte>(0,255,255));
                std::string ntext(sformat("%d", i));
                writeText(nlmDisp, nlmpt[i], ntext.c_str());
              }
          }
        inplacePaste(disp, nlmDisp, Point2D<int>(w*4, 0));
    }

  // display the image
  //win->drawImage(disp,0,0);
}

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
