/*!@file Robots2/Beobot2/Navigation/BeoRoadFinder.C 
  find road using image contours from Canny */
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
// Primary maintainer for this file: Christian Siagian <siagian@caltech.edu>
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Navigation/BeoRoadFinder.C
// $Id: $
//
//////////////////////////////////////////////////////////////////////////

// has to be the first one declared
#include "Image/OpenCVUtil.H"

#include "Robots/Beobot2/Navigation/BeoRoadFinder.H"
#include "Ice/BeobotEvents.ice.H"

#include "Raster/Raster.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/Raster.H"
#include "Image/CutPaste.H"     	 // for inplacePaste()
#include "Image/DrawOps.H" 	    	 // for writeText and SimpleFont
#include "Image/ColorOps.H" 	     	 // for luminance
#include "Image/MathOps.H"		 // for 
#include "Image/Normalize.H" 	    	 // for inplaceNormalize() 

#include <cstdio>	


#include "Transport/FrameInfo.H" //for ImageDisplayStream
#include "GUI/ImageDisplayStream.H"

#include "Util/Timer.H"
#include "Util/Geometry2DFunctions.H"

#include "Ice/IceImageUtils.H"

#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"

#include "Image/ShapeOps.H"

#define HORIZON_LINE                             130  
#define HORIZON_SUPPORT_LINE                     HORIZON_LINE+20
#define WIDTH                                    320
#define HEIGHT                                   240
#define VANISHING_POINT_SPACING                  20 
#define VANISHING_POINT_DISTANCE_THRESHOLD       WIDTH/8

// heading difference per unit pixel
// it's measured 27 degrees per half image of 160 pixels
#define HEADING_DIFFERENCE_PER_PIXEL  27.0/160.0*M_PI/180.0  // in radians 

// Beobot 2.0 pixel to cm road bottom conversion ratio
// that is, 1 pic = 9.525mm
#define BEOBOT2_PIXEL_TO_MM_ROAD_BOTTOM_RATIO 9.525

// ######################################################################
BeoRoadFinder::BeoRoadFinder
(OptionManager& mgr, const std::string& descrName, const std::string& tagName)
  :
  RobotBrainComponent(mgr, descrName, tagName),
  itsOfs(new OutputFrameSeries(mgr)),
  itsTimer(1000000),
  itsCurrentMessageID(0)
{
  itsTimer.reset();

  //itsOfs->addFrameDest("display");//Add default --out=display
  addSubComponent(itsOfs);

  itsCurrImageID = -1;
  itsProcImageID = -1;

  // set the vanishing point locations
  itsVanishingPoints.clear();
  int spacing = VANISHING_POINT_SPACING;  
  int hline   = HORIZON_LINE;
  for(int i = -4*spacing; i <= WIDTH+4*spacing; i += spacing)
    {
      itsVanishingPoints.push_back
        (VanishingPoint(Point2D<int>(i,hline),0.0F));
      LDEBUG("%d %d", i, hline);
    }

  // road recognition results 
  itsVanishingPoint           = Point2D<int>  (-1,-1);
  itsCenterPoint              = Point2D<float>(-1,-1);
  itsTargetPoint              = Point2D<float>(-1,-1);
  itsVanishingPointConfidence = 0.1;

  // currently not processing tracker
  itsTrackingFlag = false;

  // current accumulated trajectory
  itsAccumulatedTrajectory.i = 0.0F;
  itsAccumulatedTrajectory.j = 0.0F;

  // indicate how many unique lines have been identified
  // NOTE: never reset
  itsNumIdentifiedLines = 0;
}

// ######################################################################
BeoRoadFinder::~BeoRoadFinder()
{ }

// ######################################################################
void BeoRoadFinder::start1()
{ }

// ######################################################################
void BeoRoadFinder::registerTopics()
{
  // subscribe to a camera
  this->registerSubscription("CameraMessageTopic");
  this->registerSubscription("MotorMessageTopic");

  // sends out the screen shot to BeoVisualizer
  this->registerPublisher("VisualizerMessageTopic");

  this->registerPublisher("MotorRequestTopic");
  this->registerPublisher("TraversalMapMessageTopic");
}

// ######################################################################
void BeoRoadFinder::updateMessage
(const RobotSimEvents::EventMessagePtr& eMsg, const Ice::Current&)
{
  Timer tim(1000000); tim.reset();

  // camera message
  if(eMsg->ice_isA("::BeobotEvents::CameraMessage"))
    {
      its_Line_Tracker_mutex.lock();
      itsTrackingFlag = true;
      std::vector<Line> currentLines = itsCurrentLines;
      its_Line_Tracker_mutex.unlock();

      // store the image
      BeobotEvents::CameraMessagePtr cameraMsg =
        BeobotEvents::CameraMessagePtr::dynamicCast(eMsg);

      int currRequestID = cameraMsg->RequestID;
      Image<PixRGB<byte> > image = 
        Ice2Image<PixRGB<byte> >(cameraMsg->image);

      //LINFO("time: %f", tim.get()/1000.0F);

      // compute the canny image 
      IplImage *cvEdgeMap = 0;
      Image<byte> edgeMap = getCannyEdge(image, cvEdgeMap);

      //LINFO("time2: %f", tim.get()/1000.0F);

      // track the lines and send a traversal map
      if(currentLines.size() > 0)
        {
          trackVanishingLines(edgeMap, currentLines); //, true to print
   
          // compute the vanishing point, center point, target point
          float confidence;
          Point2D<int>   vp(-1,-1); 
          Point2D<float> cp(-1,-1);
          Point2D<float> tp =
            computeRoadCenterPoint(currentLines, vp, cp, confidence);

          its_Road_Information_mutex.lock(); 

          // update road model with the combined lines
          // NOTE: also change the line parameters to sync them
          //       this avoids drifts
          updateRoadModel(currentLines, currRequestID);

          itsVanishingPoint           = vp;
          itsCenterPoint              = cp;
          itsTargetPoint              = tp;
          itsVanishingPointConfidence = confidence;
          its_Road_Information_mutex.unlock(); 

          //LINFO("U[%4d] OUTPUT: vp(%4d %4d) cp(%4d, %4d) tp(%4d %4d)",
          //      currRequestID, vp.i, vp.j, cp.i, cp.j, tp.i, tp.j);

          // send motor command
          // COMMENT THIS OUT FOR BEONAVIGATOR 
          //publishMotorCommand(tp);

          publishTraversalMap(vp, cp, currRequestID);

          its_Line_Tracker_mutex.lock();
          itsCurrentLines.clear();
          for(uint i = 0; i < currentLines.size(); i++)
            itsCurrentLines.push_back(currentLines[i]);
          its_Line_Tracker_mutex.unlock();
        }
      else
        {
          //LINFO("no tracked lines");

          its_Road_Information_mutex.lock(); 
          itsVanishingPoint           = Point2D<int>  (-1,-1);
          itsCenterPoint              = Point2D<float>(-1,-1);
          itsTargetPoint              = Point2D<float>(-1,-1);
          itsVanishingPointConfidence = 0.1;
          its_Road_Information_mutex.unlock(); 
        }

      its_Line_Tracker_mutex.lock();
      itsTrackingFlag = false;
      its_Line_Tracker_mutex.unlock();

      // save it for evolve
      its_Current_Image_mutex.lock();
      itsCurrImage   = image; 
      itsCurrImageID = currRequestID;      
      itsEdgeMap     = edgeMap;
      itsCVedgeMap   = cvEdgeMap;
      itsEdgeMaps.push_back(edgeMap);
      its_Current_Image_mutex.unlock();

      //LINFO("CameraMessage[%6d]: %fms", currRequestID, tim.get()/1000.0F);
    }

  // change goal location
  else if(eMsg->ice_isA("::BeobotEvents::GoalLocationRequest"))
    {
      // we got new goal location request                     
      BeobotEvents::GoalLocationRequestPtr glrMsg =
        BeobotEvents::GoalLocationRequestPtr::dynamicCast(eMsg);

      // clear the road model lines
      its_Road_Information_mutex.lock(); 
      itsRoadModel.lines.clear();
      itsRoadModel.lastSeenHorizonPoint.clear();
      itsRoadModel.lastSeenLocation.clear();
      itsRoadModel.lastActiveIndex.clear();
      itsRoadModel.numMatches.clear();      
      its_Road_Information_mutex.unlock(); 
    }

  // motor Encoder message
  else if(eMsg->ice_isA("::BeobotEvents::MotorMessage"))
    {
      BeobotEvents::MotorMessagePtr mtrMsg =
        BeobotEvents::MotorMessagePtr::dynamicCast(eMsg);

      // store the information
      // its_Curr_Mtr_mutex.lock();
      // itsRemoteMode     =  mtrMsg->rcMode;
      // itsRcTransSpeed   =  mtrMsg->rcTransVel;
      // itsRcRotSpeed     =  mtrMsg->rcRotVel;
      // itsTravelLength   =  mtrMsg->trajLen;       // in <m>
      //itsCurrentIMUheading =  mtrMsg->imuHeading;
      // itsDiffHeading    =  mtrMsg->encoderOri;
      // its_Curr_Mtr_mutex.unlock();
      // LINFO("IMU heading: %f", mtrMsg->imuHeading/M_PI*180.0F);

      // LINFO("MotorMessage[%6d]: RC Trans %5.2f, Rot %5.2f "
      //       "heading: %f travLen: %f cHead: %f",
      //       mtrMsg->RequestID, mtrMsg->rcTransVel, mtrMsg->rcRotVel,
      //       mtrMsg->encoderOri, mtrMsg->trajLen, mtrMsg->imuHeading);

      its_Acc_Traj_mutex.lock();
      itsAccumulatedTrajectory.i +=  mtrMsg->encoderX;
      itsAccumulatedTrajectory.j +=  mtrMsg->encoderY;
      Point2D<float> accTraj = itsAccumulatedTrajectory;
      its_Acc_Traj_mutex.unlock();
      //LINFO("accumulated trajectory: %f %f", accTraj.i, accTraj.j);

      // estimate the current road heading
      // right now just set it when vanishing point is in the middle
      its_Road_Information_mutex.lock(); 
      Point2D<int> vp = itsVanishingPoint;
      if(vp.i == WIDTH/2) itsRoadHeading = mtrMsg->imuHeading;
      its_Road_Information_mutex.unlock(); 

      //if(vp.i == WIDTH/2)
      //  LINFO("heading: enc: %f IMU: %f ",
      //        mtrMsg->encoderOri/M_PI*180.0F, 
      //        mtrMsg->imuHeading/M_PI*180.0F);

      // FIXXX: this estimation should be more sophisticated 
      //        using histogram or kalman filter
      //double imu_heading = mtrMsg->imuHeading; 
      //double est_heading = 0.0F;
      //bool consistent = estimateCurrentHeading(imu_heading, est_heading);
      //LINFO("consistent: %d :: %f", consistent, est_heading);
    }
}

// ######################################################################
void BeoRoadFinder::evolve()
{
  //itsDebugWin.reset();
  // get all the pertinent information
  its_Current_Image_mutex.lock();
  Image<PixRGB<byte> > image = itsCurrImage; 
  int currImageID            = itsCurrImageID;
  Image<byte>  edgeMap       = itsEdgeMap;
  IplImage*    cvEdgeMap     = itsCVedgeMap;
  its_Current_Image_mutex.unlock();

  // only process if new image comes in
  if(currImageID != itsProcImageID)
    {
      //LINFO("currImageID: %d", currImageID);

      // compute road
      itsTimer.reset();

      computeHoughSegments(cvEdgeMap); 
      //uint64 t1 = itsTimer.get();

      its_Road_Information_mutex.lock(); 
      Point2D<int> prior_vp = itsVanishingPoint;
      // FIXXX make use of the confidence
      //       still need to check its accuracy
      //float prior_vp_conf   = itsVanishingPointConfidence;
      its_Road_Information_mutex.unlock(); 

      std::vector<Line> currentLines = computeVanishingLines(edgeMap, prior_vp);
      std::vector<Line> orgLines = currentLines;
      //LINFO("detect: %" ZU " lines", currentLines.size());
      //uint64 t2 = itsTimer.get();

      // wait until the tracking thread is done
      // get the trackers and 'disable' it during project forward
      std::vector<Image<byte> > edgeMaps;
      std::vector<Line> prevLines; 
      bool wait_for_tracker = true;
      while(wait_for_tracker)
        {
          its_Line_Tracker_mutex.lock();
          wait_for_tracker = itsTrackingFlag;

          //LINFO("wait: %d", wait_for_tracker);
          if(!wait_for_tracker)
            {
              // get the accumulated edgemaps and clear the vector
              its_Current_Image_mutex.lock();
              edgeMap = itsEdgeMap;
              image   = itsCurrImage;  

              for(uint i = 0; i < itsEdgeMaps.size(); i++)
                edgeMaps.push_back(itsEdgeMaps[i]);
              itsEdgeMaps.clear();
              currImageID = itsCurrImageID;
              its_Current_Image_mutex.unlock();

              prevLines.clear();
              for(uint i = 0; i < itsCurrentLines.size(); i++)
                prevLines.push_back(itsCurrentLines[i]);

              itsCurrentLines.clear();
            }
          its_Line_Tracker_mutex.unlock();

          usleep(100);  // wait for .5ms at a time
        }

      //uint64 t3b = itsTimer.get();
      //LINFO("waiting time (%" ZU " lines): %6.2f", 
      //      prevLines.size(), (t3b - t1)/1000.0F);

      // project forward and combine the lines
      projectForwardVanishingLines(currentLines, edgeMaps);

      //uint64 t3 = itsTimer.get();
      //LINFO("time: %6.2f (there are %" ZU " edgemaps and %" ZU " lines)", 
      //      (t3 - t3b)/1000.0F, edgeMaps.size(), prevLines.size());

      // integrate the two trackers
      std::vector<Line> combLines = combine(prevLines, currentLines);
    
      //uint64 t3c = itsTimer.get();
      //LINFO("combine: %6.2f", (t3c-t3)/1000.0F);

      // project forward for 1 last frame if needed
      edgeMaps.clear();
      its_Line_Tracker_mutex.lock();

      its_Current_Image_mutex.lock();
      edgeMap = itsEdgeMap;
      image   = itsCurrImage;

      for(uint i = 0; i < itsEdgeMaps.size(); i++)
        edgeMaps.push_back(itsEdgeMaps[i]);
      itsEdgeMaps.clear();
      currImageID = itsCurrImageID;
      its_Current_Image_mutex.unlock();

      // publish motor command only when there is a new frame 
      // and there are lines in the image
      if(edgeMaps.size() > 0 && combLines.size() > 0)
        {
          projectForwardVanishingLines(combLines, edgeMaps);

          // compute the vanishing point, center point, target point
          float confidence;
          Point2D<int> vp(-1,-1); 
          Point2D<float> cp(-1,-1);
          Point2D<float> tp =
            computeRoadCenterPoint(combLines, vp, cp, confidence);

          // update road model with the combined lines
          // NOTE: also change the line parameters to sync them
          //       this avoids drifts
          its_Road_Information_mutex.lock(); 
          updateRoadModel(combLines, currImageID);
          itsVanishingPoint           = vp;
          itsCenterPoint              = cp;
          itsTargetPoint              = tp;
          itsVanishingPointConfidence = confidence;
          its_Road_Information_mutex.unlock(); 

          //LINFO("E OUTPUT: vp(%4d %4d) cp(%4d, %4d) tp(%4d %4d)",
          //      vp.i, vp.j, cp.i, cp.j, tp.i, tp.j);

          // COMMENT THIS OUT FOR BEONAVIGATOR 
          //publishMotorCommand(vp);

          publishTraversalMap(vp, cp, currImageID);

          // std::string  res = 
          //   sformat("GT:ID %d,WIDTH 320,HEIGHT 240,VP %d,%d,"
          //           "LP %d,%d,RP %d,%d,LANG 0.0,RANG 0.0,CPUTIME 33.33\n", 
          //           currImageID,vp.i, vp.j, 
          //           int(cp.i+.5)-1,int(cp.j+.5), int(cp.i+.5)+1,int(cp.j+.5));

          // printf("%s", res.c_str()); 


          // std::string fname("result.txt");  FILE *bfp; 
          // if((bfp = fopen(fname.c_str(),"a+")) == NULL)
          //   LFATAL("not found");
          // fputs(res.c_str(), bfp);
          // fclose (bfp);
        }
      // else 
      //   {

      //     its_Road_Information_mutex.lock(); 
      //     Point2D<int>   vp = itsVanishingPoint;
      //     Point2D<float> cp = itsCenterPoint;
      //     its_Road_Information_mutex.unlock(); 

      //     std::string res = 
      //       sformat("GT:ID %d,WIDTH 320,HEIGHT 240,VP %d,%d,"
      //               "LP %d,%d,RP %d,%d,LANG 0.0,RANG 0.0,CPUTIME 33.33\n", 
      //               currImageID, vp.i,vp.j, 
      //               int(cp.i+.5)-1,int(cp.j+.5), int(cp.i+.5)+1,int(cp.j+.5));
    
      //     printf("%s", res.c_str());

      //     std::string fname("result.txt");  FILE *bfp; 
      //     if((bfp = fopen(fname.c_str(),"a+")) == NULL)
      //       LFATAL("not found");
      //     fputs(res.c_str(), bfp);
      //     fclose (bfp);          
      //   }

      // enable tracking again
      itsCurrentLines = combLines;

      its_Line_Tracker_mutex.unlock();

      //uint64 t4 = itsTimer.get();
      //LINFO("last: %6.2f (%" ZU " maps)", (t4-t3c)/1000.0F, edgeMaps.size());

      //LINFO("[%6d] time: H: %6.2f --> V: %6.2f --> c: %6.2f -:- T: %6.2f\n", 
      //      currImageID, t1/1000.0F, (t2 - t1)/1000.0F, (t3 - t2)/1000.0F,
      //      itsTimer.get()/1000.0F);          

      its_Road_Information_mutex.lock(); 
      double rHead = itsRoadHeading/M_PI*180.0F;
      its_Road_Information_mutex.unlock(); 

      Image<PixRGB<byte> > displayImage = 
        getDisplayImage(image, edgeMap, combLines, rHead); 




      //int vw = 720;//visualizer width  image
      //int vw2 = 560;//visualizer width canny
      //int vh = 360;//visualizer height
      //Image< PixRGB<byte> > visualizeImg = Image<PixRGB<byte> >(vw+vw2,vh,ZEROS);

      //Image< PixRGB<byte> > rfImg = crop(displayImage,Point2D<int>(0,23),Dims(vw-240,vh-120),true); 
      //Image< PixRGB<byte> > cannyImg = crop(displayImage,Point2D<int>(480,0),Dims(320,240),true); 
      //inplacePaste(visualizeImg, rfImg, Point2D<int>(0,0));
      //inplacePaste(visualizeImg, cannyImg, Point2D<int>(vw,0));

      //itsDebugWin.show(visualizeImg,"visualizer");
      Image< PixRGB<byte> > vImg = crop(displayImage,Point2D<int>(0,0),Dims(displayImage.getWidth(),displayImage.getHeight()-45),true); 
      //publishScreenShot(vImg);

      itsOfs->writeRGB(displayImage, "Road Finder EV");
      itsOfs->updateNext();
      //Raster::waitForKey();
    
      itsProcImageID = currImageID;
    }
}

//######################################################################
Point2D<float> BeoRoadFinder::computeRoadCenterPoint
(std::vector<Line>& lines, Point2D<int>& 
 vanishing_point, Point2D<float>& road_center_point, float& confidence)
{
  its_Road_Information_mutex.lock(); 
  Point2D<int>      prev_vanishing_point    = itsVanishingPoint;
  std::vector<bool> vanishingPointStability = itsVanishingPointStability;
  Point2D<float>    prev_center_point       = itsCenterPoint;
  Point2D<float>    prev_target_point       = itsTargetPoint;
  its_Road_Information_mutex.unlock(); 

  // LINFO("prev_vanishing_point: %d prev_center_point: %f prev_target_point: %f", 
  //       prev_vanishing_point.i, prev_center_point.i, prev_target_point.i);

  Point2D<float> target_point;

  uint num_healthy_lines = 0;
  uint num_new_lines     = 0;
  uint num_noisy_lines   = 0;

  // check if any healthy lines are active
  uint num_healthy_active = 0;

  for(uint i = 0; i < lines.size(); i++)
    {
      if(lines[i].scores.size() > 0)            
        {
          //LINFO("noisy: %d", i);         
          num_noisy_lines++;
        }
      else if(lines[i].start_scores.size() > 0) 
        {
          //LINFO("start: %d", i);
          num_new_lines++;
        }
      else                                      
        { 
          num_healthy_lines++; 
          if(lines[i].isActive) num_healthy_active++; 
          //LINFO("healthy: %d", i);
        }
    }
  // LINFO("#noisy: %d #new: %d #healthy: %d #healthy active: %d",
  //       num_noisy_lines, num_new_lines, num_healthy_lines, 
  //       num_healthy_active);

  if(num_healthy_lines == 0 &&
     num_new_lines     == 0 &&
     num_healthy_lines == 0    ) 
    {
      //LINFO("no lines to help");
      vanishing_point   = prev_vanishing_point; 
      //road_center_point = Point2D<float>(prev_vanishing_point.i, HEIGHT-1);
      road_center_point = Point2D<float>(WIDTH/2, HEIGHT-1);
      target_point      = road_center_point;
      confidence        = .4;
    }
  else 
    {
      std::vector<Line> temp_lines;  float weight = 0.0;    
      if(num_healthy_lines > 0)
        {
          for(uint i = 0; i < lines.size(); i++)
            if(lines[i].scores.size()       == 0 && 
               lines[i].start_scores.size() == 0    )  
              temp_lines.push_back(lines[i]);
          vanishing_point = getVanishingPoint(temp_lines,weight);
          //LINFO("NUM HEALTH: %d", num_healthy_lines);
        }
      else if(num_new_lines > 0)
        {
          for(uint i = 0; i < lines.size(); i++)
            if(lines[i].start_scores.size() > 0)
              temp_lines.push_back(lines[i]);
          vanishing_point = getVanishingPoint(temp_lines,weight);
          weight-= .1;

          //LINFO("NUM NEW: %d", num_new_lines);
        }
      else if(num_noisy_lines > 0)
        {
          for(uint i = 0; i < lines.size(); i++)
            if(lines[i].scores.size() > 0)
              temp_lines.push_back(lines[i]);
          vanishing_point = getVanishingPoint(temp_lines,weight);
          weight-= .2;

          //LINFO("NUM NOISY: %d", num_noisy_lines);
        }

      //road_center_point = Point2D<float>(vanishing_point.i, HEIGHT-1);
      road_center_point = Point2D<float>(WIDTH/2, HEIGHT-1);
      target_point      = road_center_point;
      confidence        = weight;
      //LINFO("VP: %d %d conf: %f", 
      //      vanishing_point.i, vanishing_point.j, confidence);
    }




  // // if the vanishing point is in the middle of the line
  // int llimit = WIDTH/2 - 20, rlimit = WIDTH/2 + 20;
  // bool in_middle = 
  //   (vanishing_point.i > llimit && vanishing_point.i < rlimit);
  // //LINFO("in middle: %d < %d < %d:::: %d",  
  // //      llimit, vanishing_point.i, rlimit, in_middle);

  // // update the stability
  // vanishingPointStability.push_back(in_middle);
  // std::vector<bool> temp_vps;
  // int n_stab = vanishingPointStability.size();
  // //LINFO("vp size: %d", n_stab);
  // if(n_stab > 15)
  //   {
  //     for(int i = n_stab-15; i < n_stab; i++)
  //       {
  //         temp_vps.push_back(vanishingPointStability[i]);
  //         //LINFO("[%3d]: %d", i, int(vanishingPointStability[i]));
  //       }
  //     vanishingPointStability = temp_vps;
  //   }

  // // check for stability of VP in the middle
  // uint num_middle = 0;
  // for(uint i = 0; i < vanishingPointStability.size(); i++)
  //   if(vanishingPointStability[i]) num_middle++;   

  // its_Road_Information_mutex.lock(); 
  // itsVanishingPointStability = vanishingPointStability;
  // its_Road_Information_mutex.unlock(); 

  // only use lines if the VP is in the middle
  // FIXXX: all the code below are disabled by using false in the if statement
  //LINFO("IN MIDDLE: %d", in_middle);
  //if(!in_middle) return target_point;

  //bool is_inmiddle_stable   = (num_middle >= 11);
  //bool is_inmiddle_unstable = (num_middle <=  5);

  // if the vp is in the middle very stably
  //if(is_inmiddle_stable &&
  //   num_healthy_lines  >  0 && num_healthy_active == 0    )

  // // if the vp in the middle is very unstable
  // if(is_inmiddle_unstable &&  
  //    vanishingPointStability.size() >= 15)
  //   {
  //     LINFO("DEACTIVATE");

  //     // clear all the angles
  //     for(uint i = 0; i < lines.size(); i++)
  //       lines[i].isActive = false;
  //     //Raster::waitForKey();
  //   }




  if((num_healthy_lines  > 0) && (num_healthy_active == 0))
   {
     //LINFO("ACTIVATE");

     // reset the angle to the center lines 
     for(uint i = 0; i < lines.size(); i++)
       {
         if(lines[i].scores.size()       == 0 &&
            lines[i].start_scores.size() == 0   )
           {
             lines[i].isActive      = true;              
             lines[i].angleToCenter = M_PI/2 - lines[i].angle;
             lines[i].pointToServo  = lines[i].onScreenRoadBottomPoint; 
             lines[i].offset        = 0.0f;
             // LINFO("[%3d] healthy line angle: %7.2f ptToServo: %7.2f %7.2f", 
             //       i, lines[i].angleToCenter/M_PI*180,
             //       lines[i].pointToServo.i, lines[i].pointToServo.j);
           }
       }
     //Raster::waitForKey();
   }

  // get the average active center angle
  float total_weight = 0.0f;
  float total_angle  = 0.0f; 
  int   num_a_line   = 0;

  float total_curr_offset  = 0.0f;
  for(uint i = 0; i < lines.size(); i++)
    {
      if(lines[i].scores.size()       == 0 && 
         lines[i].start_scores.size() == 0 &&  
         lines[i].isActive)
        {
          num_a_line++;

          // compute angle to road center
          float angle  = lines[i].angle;
          float cangle = lines[i].angleToCenter;
          float weight = lines[i].score;
            
          float ccangle = cangle + angle; 
            
          total_angle  += ccangle*weight;
          total_weight += weight;

          // compute center point
          float dist = HEIGHT - HORIZON_LINE;
          float tpi = 0.0;  
          float cos_ang = cos(M_PI/2 - ccangle);
          if(cos_ang != 0.0F) tpi = dist/cos_ang*sin(M_PI/2-ccangle);
          tpi += vanishing_point.i;

          // compute offset from point to servo
          float c_offset = 0.0f;

          Point2D<float> c_pt  = lines[i].onScreenRoadBottomPoint; 
          Point2D<float> pt_ts = lines[i].pointToServo;
          Point2D<float> h_pt  = lines[i].onScreenHorizonPoint;
          float offset = lines[i].offset;
          if(fabs(pt_ts.i) < 0.05 || fabs(pt_ts.i - WIDTH) < .05)
            {
              // figure out where the point would be 
              // for the specified 'j' component
              Point2D<float> h1(0    , pt_ts.j);
              Point2D<float> h2(WIDTH, pt_ts.j);
              c_pt = intersectPoint(h_pt,c_pt, h1,h2); 
            }
          c_offset = c_pt.i - pt_ts.i;

          total_curr_offset += (c_offset + offset)*weight;
            
          // LINFO("[%3d] angle: %7.2f cangle: %7.2f ccangle: %7.2f tp: %7.2f"
          //       "|| %7.2f - %7.2f = c_o: %7.2f + o: %7.2f = %7.2f W: %7.2f", 
          //       i, angle/M_PI*180, cangle/M_PI*180, ccangle/M_PI*180, tpi,
          //       c_pt.i, pt_ts.i, c_offset, offset, c_offset+offset, weight);
        }
    }

  // calculate confidence
  float avg_weight = 0.0F;
  if(num_a_line > 0) avg_weight = total_weight/num_a_line;
  confidence = avg_weight;
  // LINFO("avg_weight: %f", avg_weight);
  if(avg_weight == 0) return road_center_point;

  // angle-based road center estimation
  float avg_angle  = 0.0F;
  if(total_weight > 0) avg_angle = total_angle/total_weight;
  float dist = HEIGHT - HORIZON_LINE;
  float tpi = 0.0;  
  float cos_ang = cos(M_PI/2 - avg_angle);
  if(fabs(cos_ang) > 0.001F) tpi = dist/cos_ang*sin(M_PI/2-avg_angle);
  tpi += float(WIDTH)/2.0F;
  target_point = Point2D<float>(tpi, HEIGHT-1);
  // LINFO("avg_angle : %7.2f/%7.2f = %7.2f: tpi: %f", 
  //       total_angle/M_PI*180.0F, total_weight, avg_angle/M_PI*180, tpi);

  // offset point-based road center estimation
  float avg_offset = 0.0F; 
  if(total_weight > 0) avg_offset = total_curr_offset/total_weight;
  float cpi = float(WIDTH)/2.0F + avg_offset;
  road_center_point = Point2D<float>(cpi, HEIGHT-1);
  // LINFO("avg_offset: %7.2f/%7.2f = %7.2f: cpi: %f", 
  //       total_curr_offset, total_weight, avg_offset, cpi);

  // // adjust the active angles to take out the recent noise
  // for(uint i = 0; i < lines.size(); i++)
  //   {
  //     if(lines[i].scores.size()       == 0 && 
  //        lines[i].start_scores.size() == 0 &&  
  //        lines[i].isActive)
  //       {
  //         float angle  = lines[i].angle;
  //         float cangle = lines[i].angleToCenter;                  
  //         float ccangle = cangle + angle; 
            
  //         float diff_angle = avg_angle - ccangle;
  //         float new_angle  = cangle + diff_angle;

  //         lines[i].angleToCenter = new_angle;
            
  //         LINFO("[%3d] diff angle: %f new angle: %f", 
  //               i, diff_angle/M_PI*180, new_angle/M_PI*180);
  //       }
  //   }

  // if at least one line is active
  // activate all the healthy but inactive lines
  for(uint i = 0; i < lines.size(); i++)
    {
      if(avg_weight > 0.0 &&          
         lines[i].scores.size()       == 0 && 
         lines[i].start_scores.size() == 0 &&  
         !lines[i].isActive)
        {
          // calculate angle                  
          float angle = lines[i].angle;
          float ccangle = avg_angle - angle;
          lines[i].angleToCenter = ccangle;
          lines[i].pointToServo  = lines[i].onScreenRoadBottomPoint;
          lines[i].offset        = avg_offset;
          lines[i].isActive      = true; 
          //LINFO("[%3d] iangle: %f ccangle: %f", 
          //      i, angle/M_PI*180, ccangle/M_PI*180);
        }
    }

  return road_center_point;
}

//######################################################################
void BeoRoadFinder::updateRoadModel(std::vector<Line> &lines, int index)
{
  uint n_road_lines = itsRoadModel.lines.size();
  uint n_in_lines   = lines.size(); 

  // update only on the healthy input lines
  std::vector<bool> is_healthy(n_in_lines);
  for(uint i = 0; i < n_in_lines; i++)
    is_healthy[i] = 
      (lines[i].scores.size()       == 0 &&
       lines[i].start_scores.size() == 0   );

  std::vector<int> road_match_index(n_road_lines);
  for(uint j = 0; j < n_road_lines; j++) road_match_index[j] = -1;
  std::vector<int> in_match_index(n_in_lines);
  for(uint i = 0; i < n_in_lines; i++) in_match_index[i] = -1;

  std::vector<std::vector<float> > match_dists;
  for(uint i = 0; i < n_in_lines; i++)
    match_dists.push_back(std::vector<float>(n_road_lines));




  // LINFO("BEFORE M: %" ZU " lines", lines.size());	 
  // for(uint i = 0; i < n_road_lines; i++)	 
  //   {	 
  //     Point2D<float> pts    = lines[i].pointToServo; 
  //     float          offset = lines[i].offset; 
  //     LINFO("(%4d)[%8.3f %8.3f]: %10.4f",	 
  //           i, pts.i, pts.j, offset);	 
  //   }

  // LINFO("BEFORE Mr: %" ZU " lines", itsRoadModel.lines.size());	 
  // for(uint j = 0; j < n_road_lines; j++)	 
  //   {	 
  //     Point2D<float> pts      = itsRoadModel.lines[j].pointToServo; 
  //     float          offset   = itsRoadModel.lines[j].offset; 
  //     int            la_index = itsRoadModel.lastActiveIndex[j];
  //     Point2D<float> lshpt    = itsRoadModel.lastSeenHorizonPoint[j];
  //     Point2D<float> lspt     = itsRoadModel.lastSeenLocation[j];
  //     int            nmatch   = itsRoadModel.numMatches[j];
        
  //     LINFO("<%4d>[%8.3f %8.3f]: %10.4f || %d [%8.3f %8.3f] nm: %d",	 
  //           j, pts.i, pts.j, offset, la_index, lspt.i, lspt.j, nmatch);	 
  //   }      

  // go through each input and road line combination 
  // to get the match score
  // which is a simple closest point proximity
  for(uint i = 0; i < n_in_lines; i++)
    {
      if(!is_healthy[i]) continue;

      Point2D<float> ipt = lines[i].onScreenRoadBottomPoint; 
      Point2D<float> hpt = lines[i].horizonPoint;

      for(uint j = 0; j < n_road_lines; j++)
        {
          Point2D<float> lshpt = itsRoadModel.lastSeenHorizonPoint[j];
          Point2D<float> lsl   = itsRoadModel.lastSeenLocation[j];

          float dist  = lsl.distance(ipt);
          float hdist = hpt.distance(lshpt);

          match_dists[i][j] = dist;
          if(hdist > 50)
            match_dists[i][j] = dist + hdist; 
      
          // LINFO("lshpt: %f %f hpt: %f %f ==== ipt: %f %f -- rm.lsL %f %f", 
          //       lshpt.i, lshpt.j, hpt.i, hpt.j, ipt.i, ipt.j, lsl.i, lsl.j);
        }
    }

  // for(uint j = 0; j < n_road_lines; j++)
  //   {
  //     for(uint i = 0; i < n_in_lines; i++)
  //       printf("%6.2f ", match_dists[i][j]);
  //     printf("\n");
  //   }

  // calculate the best match and add it
  for(uint i = 0; i < n_in_lines; i++)
    {
      if(!is_healthy[i]) continue;

      // get the (best and second best) match and scores
      int   m1_index = -1;      float m1_dist = -1.0; 
      int   m2_index = -1;      float m2_dist = -1.0; 
      for(uint j = 0; j < n_road_lines; j++)
        {
          if(road_match_index[j] != -1) continue;

          float dist = match_dists[i][j];
          if(m1_index == -1 || dist < m1_dist)
            {
              m2_index = m1_index; m2_dist  = m1_dist;
              m1_index = j       ; m1_dist  = dist; 
            }
          else if(m2_index == -1 || dist < m2_dist)
            {
              m2_index = j       ; m2_dist  = dist; 
            }
        }

      // get the best match for road with many evidences
      int   ml1_index = -1;      float ml1_dist = -1.0; 
      int   ml2_index = -1;      float ml2_dist = -1.0; 
      for(uint j = 0; j < n_road_lines; j++)
        {
          int nmatch = itsRoadModel.numMatches[j];
          if(road_match_index[j] != -1 || nmatch < 10) continue;

          float dist = match_dists[i][j];
          if(ml1_index == -1 || dist < ml1_dist)
            {
              ml2_index = ml1_index; ml2_dist  = ml1_dist;
              ml1_index = j        ; ml1_dist  = dist; 
            }
          else if(ml2_index == -1 || dist < ml2_dist)
            {
              ml2_index = j       ; ml2_dist  = dist; 
            }
        }

      // LINFO("[%3d]: best: %d (%f) second: %d (%f)",
      //       i, m1_index, m1_dist, m2_index, m2_dist);
      // LINFO("[%3d]: lbest: %d (%f) lsecond: %d (%f)",
      //       i, ml1_index, ml1_dist, ml2_index, ml2_dist);

      // if there first 
      int j = -1;

      // check the large matches first
      if(ml1_dist != -1.0F)
        if(ml1_dist < 15.0F || 
           ((ml2_dist != -1.0 || ml1_dist/ml2_dist < .1) && ml1_dist < 30.0F))
          j = ml1_index;      
    
      // then the smaller matches
      if(j == -1 && m1_dist != -1.0F)
        if(m1_dist < 5.0F || 
           ((m2_dist != -1.0 || m1_dist/m2_dist < .1) && m1_dist < 20.0F)) 
          j = m1_index;      

      if(j != -1)
        {
          road_match_index[j] = i;              
          in_match_index[i]   = j;

          // use the model parameters
          lines[i].angleToCenter = itsRoadModel.lines[j].angleToCenter;
          lines[i].pointToServo  = itsRoadModel.lines[j].pointToServo; 
          lines[i].offset        = itsRoadModel.lines[j].offset;
          lines[i].index         = itsRoadModel.lines[j].index;
          lines[i].isActive = true;

          // update the road model 
          Point2D<float> hpt = lines[i].horizonPoint; 
          Point2D<float> ipt = lines[i].onScreenRoadBottomPoint; 

          itsRoadModel.lastSeenHorizonPoint[j] = hpt;
          itsRoadModel.lastSeenLocation[j]     = ipt;
          itsRoadModel.lastActiveIndex[j]      = index;

          int nmatch = itsRoadModel.numMatches[j];
          itsRoadModel.numMatches[j]   = nmatch+1;
        }
    }

  // LINFO("AFTER Mr: %" ZU " lines", itsRoadModel.lines.size());
  // for(uint j = 0; j < n_road_lines; j++)	 
  //   {	       
  //     Point2D<float> pts      = itsRoadModel.lines[j].pointToServo; 
  //     float          offset   = itsRoadModel.lines[j].offset; 
  //     int            la_index = itsRoadModel.lastActiveIndex[j];
  //     Point2D<float> lshpt    = itsRoadModel.lastSeenHorizonPoint[j];
  //     Point2D<float> lspt     = itsRoadModel.lastSeenLocation[j];
  //     int            nmatch   = itsRoadModel.numMatches[j];
        
  //     LINFO("<%4d>[%8.3f %8.3f]: %10.4f || %d [%8.3f %8.3f] %d",	 
  //           j, pts.i, pts.j, offset, la_index, lspt.i, lspt.j, nmatch);	 
  //   }      

  // ADD NOTE: secondary matching criteria:
  // for each healthy line not yet selected
  //   for each road model not yet selected
  //     if ratio with second best not selected > 3:1 

  // delete inactive road model lines
  std::vector<Line>::iterator l_itr = 
    itsRoadModel.lines.begin();
  std::vector<Point2D<float> >::iterator h_itr = 
    itsRoadModel.lastSeenHorizonPoint.begin();
  std::vector<Point2D<float> >::iterator p_itr = 
    itsRoadModel.lastSeenLocation.begin();
  std::vector<int>::iterator i_itr = 
    itsRoadModel.lastActiveIndex.begin();
  std::vector<int>::iterator n_itr = 
    itsRoadModel.numMatches.begin();
  //std::vector<Line>::iterator stop = 
  //  itsRoadModel.lines.end();

  while(l_itr != itsRoadModel.lines.end())
    {
      int lindex = *i_itr;
      if(index > lindex+300) 
        {
          l_itr = itsRoadModel.lines.erase(l_itr);
          h_itr = itsRoadModel.lastSeenHorizonPoint.erase(h_itr);
          p_itr = itsRoadModel.lastSeenLocation.erase(p_itr);
          i_itr = itsRoadModel.lastActiveIndex.erase(i_itr);
          n_itr = itsRoadModel.numMatches.erase(n_itr);
        }
      else { l_itr++; p_itr++; i_itr++; n_itr++; }
    }

  n_road_lines = itsRoadModel.lines.size();

  // LINFO("AFTER Dr: %" ZU " lines", itsRoadModel.lines.size());
  // for(uint j = 0; j < n_road_lines; j++)	 
  //   {	       
  //     Point2D<float> pts      = itsRoadModel.lines[j].pointToServo; 
  //     float          offset   = itsRoadModel.lines[j].offset; 
  //     int            la_index = itsRoadModel.lastActiveIndex[j];
  //     Point2D<float> lshpt    = itsRoadModel.lastSeenHorizonPoint[j];
  //     Point2D<float> lspt     = itsRoadModel.lastSeenLocation[j];
  //     int            nmatch   = itsRoadModel.numMatches[j];
        
  //     LINFO("<%4d>[%8.3f %8.3f]: %10.4f || %d [%8.3f %8.3f]: %d",	 
  //           j, pts.i, pts.j, offset, la_index, lspt.i, lspt.j, nmatch);	 
  //   }      



  // add all the lines not yet added to the road model  
  for(uint i = 0; i < n_in_lines; i++)
    {
      if(!is_healthy[i]) continue;
      if(in_match_index[i] != -1) continue;

      lines[i].index         = itsNumIdentifiedLines++; 

      Point2D<float> hpt = lines[i].horizonPoint; 
      Point2D<float> ipt = lines[i].onScreenRoadBottomPoint; 

      // update the road model
      itsRoadModel.lines.push_back(lines[i]);
      itsRoadModel.lastSeenHorizonPoint.push_back(hpt);
      itsRoadModel.lastSeenLocation.push_back(ipt);
      itsRoadModel.lastActiveIndex.push_back(index);
      itsRoadModel.numMatches.push_back(1);

      // LINFO("[%3d] healthy line angle: %7.2f ptToServo: %7.2f %7.2f", 
      //       i, lines[i].angleToCenter/M_PI*180,
      //       lines[i].pointToServo.i, lines[i].pointToServo.j);
    }



  // LINFO("AFTER Ar: %" ZU " lines", itsRoadModel.lines.size());	   
  // for(uint j = 0; j < n_road_lines; j++)	 
  //   {	 
  //     Point2D<float> pts      = itsRoadModel.lines[j].pointToServo; 
  //     float          offset   = itsRoadModel.lines[j].offset; 
  //     int            la_index = itsRoadModel.lastActiveIndex[j];
  //     Point2D<float> lshpt    = itsRoadModel.lastSeenHorizonPoint[j];
  //     Point2D<float> lspt     = itsRoadModel.lastSeenLocation[j];
  //     int            nmatch   = itsRoadModel.numMatches[j];
        
  //     LINFO("<%4d>[%8.3f %8.3f]: %10.4f || %d [%8.3f %8.3f]: nm: %d",	 
  //           j, pts.i, pts.j, offset, la_index, lspt.i, lspt.j, nmatch);	 
  //   }      
}

//######################################################################
Point2D<int> BeoRoadFinder::getVanishingPoint
(std::vector<Line> lines, float &confidence)
{
  // get the horizon points 
  // do a weighted average
  float total_weight = 0.0F;
  float total_hi     = 0.0F;
  int   num_hi       = 0;
  for(uint i = 0; i < lines.size(); i++)
    {
      Line l = lines[i];

      float hi     = l.horizonPoint.i;
      float weight = l.score;

      //if(weight > .5 && 
      //   (rbi >= -WIDTH/2 && rbi <= WIDTH+WIDTH/2))
      //  {
      total_hi     += hi*weight;
      total_weight += weight;
      num_hi++;
      //  }
    }

  float wavg_hi    = 0.0F;
  float avg_weight = 0.0F;
  if(num_hi > 0) 
    {
      wavg_hi    = total_hi/total_weight;
      avg_weight = total_weight/num_hi;
    }
  confidence = avg_weight;

  //LINFO("num hi: %d vanishing point: %7.2f (weights): %7.2f", 
  //      num_hi, wavg_hi, avg_weight);

  return Point2D<int>(wavg_hi,HORIZON_LINE);
}

//######################################################################
void BeoRoadFinder::publishMotorCommand(Point2D<int> max_vp)
{
  its_Acc_Traj_mutex.lock();
  Point2D<float> acc_traj = itsAccumulatedTrajectory;
  its_Acc_Traj_mutex.unlock();

  // compute rotational command
  float rot1 = (160.0 - max_vp.i)/240.0*.7+ .12;
  float lat_comp = 0.0F;

  // NOTE: odometry-based lateral compensation is not accurate
  // float lat_gain = .4;
  // float lat_comp = -acc_traj.j * lat_gain; 
  // if     (lat_comp >  lat_gain) lat_comp =  lat_gain; 
  // else if(lat_comp < -lat_gain) lat_comp = -lat_gain;

  float rot = rot1 + lat_comp;
  // LINFO("vp.i: %d atraj.j: %f = %5.2f + %5.2f = %5.2f", 
  //       max_vp.i, acc_traj.j, rot1, lat_comp, rot);

  BeobotEvents::MotorRequestPtr msg = new BeobotEvents::MotorRequest;
  msg->transVel = TYPICAL_MAXIMUM_SPEED;
  msg->rotVel   = rot;

  this->publish("MotorRequestTopic", msg);
}

//######################################################################
void BeoRoadFinder::publishTraversalMap
(Point2D<int> max_vp, Point2D<float> road_center_point, int index)
{
  its_Acc_Traj_mutex.lock();
  Point2D<float> acc_traj = itsAccumulatedTrajectory;
  its_Acc_Traj_mutex.unlock();

  // calculate heading difference 
  // given pixel distance from center of the image	
  // negate because we are estimating robot heading
  // NOTE: constants come from calibrated 54deg camera FOV 
  float rhead = -(160.0 - max_vp.i) * HEADING_DIFFERENCE_PER_PIXEL;

  // calculate metric (in mm) lateral deviation from center of road
  float lateral_deviation =
    (road_center_point.i - WIDTH/2)*BEOBOT2_PIXEL_TO_MM_ROAD_BOTTOM_RATIO;

  //LINFO("cp: %f sending: %f", road_center_point.i, lateral_deviation);

  //LINFO("vp: %d --> %f | lat: %d --> %f", 
  //      max_vp.i, rhead/M_PI*180.0,
  //      road_center_point.i - WIDTH/2, lateral_deviation);

  BeobotEvents::TraversalMapMessagePtr msg = 
    new BeobotEvents::TraversalMapMessage;
  msg->RequestID        = index; 
  msg->tmap             = Image2Ice<float>(getTraversalMap());
  msg->heading          = rhead;
  msg->lateralDeviation = lateral_deviation;
  this->publish("TraversalMapMessageTopic", msg);
}

//######################################################################
Image<float> BeoRoadFinder::getTraversalMap()
{
  Image<float> tmap
    (LOCAL_MAP_NUM_HORZ_GRIDS, LOCAL_MAP_NUM_VERT_GRIDS, ZEROS);
  tmap += LOCAL_MAP_DEFAULT_GRID_VALUE;

  uint w = tmap.getWidth();
  uint h = tmap.getHeight();
  Image<float>::iterator  itr  = tmap.beginw();//, stop = tmap.end();

  // never absolute because we are estimating based on image
  float maxv = 0.8;
  float minv = 0.2; 

  int ib     = 4;
  int wib    = w/ib;
  int wib_2  = wib/2;
  //int p      = 2;
  //float wibp = wib/p;  

  //  while (itr != stop)
  //  {
      for(uint j = 0; j < h; j++)
        {
          for(int i = 0; i < wib; i++)
            {
              if(i >= 6 && i <= 9)
                {
                  float ind = fabs(i+.5 - wib_2)-.5;
                  float pct = ind/wib_2; 
                  float val = minv + (maxv - minv)*(1.0 - pct); 

                  //if(j == 0) LINFO("[%3d]: %f --> %f --> %f", 
                  //                 i, ind, pct, val);
                  for(int ii = 0; ii < ib; ii++) *itr++ = val;
                }
              else for(int ii = 0; ii < ib; ii++) itr++;
            }
        }
   //  }

  return tmap;
}

//######################################################################
Image<byte> BeoRoadFinder::getCannyEdge
(Image<PixRGB<byte> > image, IplImage*& cvImage)
{
  // convert iNVT image to cvImage
  cvImage = cvCreateImage(cvGetSize(img2ipl(image)),8,1);

  // Canny Edge detection
  int sobelApertureSize = 7;
  int highThreshold = 400*sobelApertureSize*sobelApertureSize;
  int lowThreshold  = int(highThreshold*.4);
  cvCanny(img2ipl(luminance(image)), cvImage, lowThreshold, highThreshold,
          sobelApertureSize);

  // convert cvImage to iNVT image
  return ipl2gray(cvImage);
}

//######################################################################
void BeoRoadFinder::computeHoughSegments(IplImage *cvImage)
{
  // NOTE: has CV_HOUGH_STANDARD version in revision 15285

  CvMemStorage *s = cvCreateMemStorage(0);
  CvSeq *segments = 0;

  int threshold     = 10;
  int minLineLength =  5;
  int maxGap        =  2;
  segments = cvHoughLines2
    (cvImage,s,CV_HOUGH_PROBABILISTIC,1,CV_PI/180,
     threshold,minLineLength,maxGap);

  //MIN(segments->total,100)
  itsCurrentSegments.clear();

  //LINFO("Total Segments: %5d",segments->total);
  //uint total_used = 0;
  for(int i = 0;i < segments->total; i++)
    {
      CvPoint* line = (CvPoint*)cvGetSeqElem(segments,i);

      Point2D<int> pt1 = Point2D<int>(line[0].x,line[0].y);
      Point2D<int> pt2 = Point2D<int>(line[1].x,line[1].y);

      int dx = pt2.i - pt1.i;
      int dy = pt2.j - pt1.j;

      float length = pow(dx*dx+dy*dy, .5);
      float angle  = atan2(dy,dx) * 180.0F /M_PI;

      int horizon_y   = HORIZON_LINE;
      int horizon_s_y = HORIZON_SUPPORT_LINE;
      bool good_horizon_support = 
        (pt1.j > horizon_y   && pt2.j > horizon_y  ) && 
        (pt1.j > horizon_s_y || pt2.j > horizon_s_y)   ;

      bool non_vertical = 
         !((angle >  80.0F && angle <  100.0F) || 
           (angle < -80.0F && angle > -100.0F)   );

      if(length > 5.0F && good_horizon_support && non_vertical)
        {
          itsCurrentSegments.push_back(Segment(pt1,pt2,angle,length));
          // LINFO("[%3d]: l: %6.2f s: %6.2f nv: %d", 
          //       i, length, angle, non_vertical);
          //total_used++;
        }

    }
  //LINFO("Total Used : %5d", total_used);
}

//######################################################################
std::vector<Line> BeoRoadFinder::computeVanishingLines
(Image<byte> edgeMap, Point2D<int> vanishingPoint)
{
  Point2D<float> h1(0    , HORIZON_LINE);
  Point2D<float> h2(WIDTH, HORIZON_LINE);

  uint num_vp = itsVanishingPoints.size();
  std::vector<float> curr_vp_likelihood(num_vp);
  std::vector<std::vector<uint> > curr_vp_support(num_vp);
  for(uint i = 0; i < num_vp; i++) 
    {
      curr_vp_likelihood[i] = 0.0F;
      curr_vp_support.push_back(std::vector<uint>());
    }

  for(uint j = 0; j < itsCurrentSegments.size(); j++)
    {
      Segment s = itsCurrentSegments[j];

      Point2D<float> p1(s.p1);
      Point2D<float> p2(s.p2);
      if(p2.j > p1.j) 
        { 
          p1 = Point2D<float>(s.p2.i, s.p2.j); 
          p2 = Point2D<float>(s.p1.i, s.p1.j); 
        }

      //float angle  = s.angle;
      float length = s.length;      

      // compute intersection to vanishing point vertical          
      Point2D<float> p_int = intersectPoint(p1,p2, h1,h2);      
      // LINFO("Line: (%f,%f)-(%f,%f) -- (%f,%f)-(%f,%f)",  
      //       p1.i, p1.j, p2.i, p2.j, h1.i, h1.j, h2.i, h2.j);
      // LINFO("Intersection Point is (%f,%f)", p_int.i,p_int.j);      

      // for each vanishing point
      for(uint i = 0; i < itsVanishingPoints.size(); i++)
        {
          Point2D<int> vp = itsVanishingPoints[i].vp;
          int p_int_i = int(p_int.i);
          if(!((p1.i <= p2.i && p2.i <= p_int_i && p_int_i <= vp.i+10) || 
               (p1.i >= p2.i && p2.i >= p_int_i && p_int_i >= vp.i-10)   ))
            continue;

          float dist  = p_int.distance(Point2D<float>(vp.i, vp.j));
          float d_val = 1.0 - dist/(VANISHING_POINT_DISTANCE_THRESHOLD);
          if(d_val > 0.0F) 
            {
              // LINFO("[%3d %3d] dist: %6.2f --> c_val: %6.2f", 
              //       vp.i, vp.j, dist, d_val);

              curr_vp_support[i].push_back(j);
        
              // accumulate likelihood values
              // FIXXX would length^2 be better?
              curr_vp_likelihood[i] += d_val*length;
            }
        }
    }

  // integrate with previous values: FIXXX
  for(uint i = 0; i < itsVanishingPoints.size(); i++)
    {
      Point2D<int> vp = itsVanishingPoints[i].vp;

      float likelihood = curr_vp_likelihood[i];
      itsVanishingPoints[i].likelihood = likelihood;

      // compute prior
      float prior = 0.1;
      if(!(vanishingPoint.i == -1 && vanishingPoint.j == -1))
        {
          float di = fabs(vp.i - vanishingPoint.i);
          prior = 1.0 - di/(WIDTH/4); 
          if(prior < .1) prior = .1;
        }

      itsVanishingPoints[i].prior      = prior;
      itsVanishingPoints[i].posterior  = prior*likelihood;

      // LINFO("[%3d %3d]: l: %6.2f * pr: %6.2f = pos: %6.2f", 
      //       vp.i, vp.j, likelihood, prior, likelihood*prior);
      itsVanishingPoints[i].supportingSegments.clear();

      for(uint j = 0; j < curr_vp_support[i].size(); j++)
        itsVanishingPoints[i].supportingSegments.push_back
          (itsCurrentSegments[curr_vp_support[i][j]]);
    }

  uint max_i = 0;
  float max_p = itsVanishingPoints[max_i].posterior;  
  for(uint i = 0; i < itsVanishingPoints.size(); i++)
    {
      float posterior = itsVanishingPoints[i].posterior;
      if(max_p < posterior) 
        {
          max_p = posterior;
          max_i = i;
        }
    }

  // create vanishing lines

  // sort the supporting segments on length
  std::list<Segment> supporting_segments;
  uint n_segments = itsVanishingPoints[max_i].supportingSegments.size();
  for(uint i = 0; i < n_segments; i++)
    supporting_segments.push_back
      (itsVanishingPoints[max_i].supportingSegments[i]);
  supporting_segments.sort();  
  supporting_segments.reverse();

  std::vector<Line> current_lines;
  std::vector<bool> is_used(n_segments);
  for(uint i = 0; i < n_segments; i++) is_used[i] = false;

  // create lines
  std::list<Segment>::iterator 
    itr  = supporting_segments.begin(), stop = supporting_segments.end();
  uint index = 0;
  while (itr != stop)
    {
      Segment s2 = *itr; 
      //LINFO("[%d]: length: %f", index, s.length);
      itr++; 

      if(is_used[index]){ index++; continue; }
      is_used[index] = true;
      index++;

      // compute true angle
      //float angle = computeAngle(s2, edgeMap);
      //s2.angle = angle;

      // find other segments with this angle
      float total_length = 0.0F; uint  num_segments = 0;
      Line l = findLine2(s2, edgeMap, supporting_segments, 
                         is_used, total_length, num_segments);
      //LINFO("total length: %f num_segments: %d", total_length, num_segments);

      // check for line fitness
      Point2D<float> oshsp = l.onScreenHorizonSupportPoint;
      Point2D<float> osrbp = l.onScreenRoadBottomPoint;
      //float dist = oshsp.distance(osrbp);
      //float rbi  = l.roadBottomPoint.i;

      //if(dist < 60.0F && !(rbi >= -WIDTH/2 && rbi <= WIDTH+WIDTH/2))
      //  continue;

      Point2D<int> hpt(oshsp + .5);
      Point2D<int> rpt(osrbp + .5);

      float score = getLineFitness(hpt, rpt, edgeMap);
      l.score = score;
      l.start_scores.push_back(score);
      //LINFO("BEFORE score: %f dist: %f", score, dist);

          // // fit it one more time
          // std::vector<Line> tline; tline.push_back(l);
          // trackVanishingLines(edgeMap, tline);
          // l = tline[0];
    
          // oshsp = l.onScreenHorizonSupportPoint;
          // osrbp = l.onScreenRoadBottomPoint;
          // Point2D<int> thpt(oshsp + .5);
          // Point2D<int> trpt(osrbp + .5);

          // //LINFO("AFTER");
          // score = getLineFitness(thpt, trpt, edgeMap);
          // //LINFO("adding line. score: %f", score);


      //if( (total_length > 60 ||
      //     (total_length > 40 && num_segments >= 2) ||
      //     (total_length > 30 && num_segments >= 3))    && score >= .5)
      if(score >= .5)    
        {
          //LINFO("in");

          // // fit it one more time
          // std::vector<Line> tline; tline.push_back(l);
          // trackVanishingLines(edgeMap, tline);
          // l = tline[0];
    
          // oshsp = l.onScreenHorizonSupportPoint;
          // osrbp = l.onScreenRoadBottomPoint;
          // Point2D<int> thpt(oshsp + .5);
          // Point2D<int> trpt(osrbp + .5);

          // //LINFO("AFTER");
          // score = getLineFitness(thpt, trpt, edgeMap);
          //LINFO("adding line. score: %f \n\n\n\n\n\n", score);
          current_lines.push_back(l);          
        }
    }

  // save the vanishing point
  its_Road_Information_mutex.lock(); 
  itsVanishingPoint = itsVanishingPoints[max_i].vp;  
  its_Road_Information_mutex.unlock(); 

  return current_lines;
}

// ######################################################################
std::vector<Point2D<int> >  
BeoRoadFinder::getPixels
(Point2D<int> p1, Point2D<int> p2, Image<byte> edgeMap)
{
  std::vector<uint> startIndexes;
  return getPixels(p1,p2,edgeMap,startIndexes);
}

// ######################################################################
std::vector<Point2D<int> >  
BeoRoadFinder::getPixels
(Point2D<int> p1, Point2D<int> p2, Image<byte> edgeMap,
 std::vector<uint>& startIndexes)
{
  std::vector<Point2D<int> > points;

  // from Graphics Gems / Paul Heckbert
  int dx = p2.i - p1.i, ax = abs(dx) << 1, sx = signOf(dx);
  int dy = p2.j - p1.j, ay = abs(dy) << 1, sy = signOf(dy);
  int x = p1.i, y = p1.j;

  const int w = edgeMap.getWidth();
  const int h = edgeMap.getHeight();

  // flag to start new segment for the next hit
  bool start_segment = true;
  startIndexes.clear();

  if (ax > ay)
    {
      int d = ay - (ax >> 1);
      for (;;)
        {
          bool adding = false;

          if (x >= 0 && x < w && y >= 0 && y < h)
            {
              if(edgeMap.getVal(x,y) > 0)
                {
                  points.push_back(Point2D<int>(x,y)); 
                  adding = true;
                }
              else if(points.size() > 0)
                {
                  uint size = points.size();
                  Point2D<int> ppt = points[size-1];

                  // get points that are neighbors 
                  // of the previous point
                  for(int di = 0; di <= 1; di++)
                    for(int dj = 0; dj <= 1; dj++)
                      {
                        if(di == 0 && dj == 0) continue;
                        if(!edgeMap.coordsOk(x+di,y+dj)) continue;

                        if(edgeMap.getVal(x+di,y+dj) && 
                           abs((x+di)-ppt.i <= 1) && abs((y+dj)-ppt.j <= 1))
                          { 
                            points.push_back(Point2D<int>(x+di,y+dj));
                            adding = true;
                          }
                      }
                }

              // start new segment on the current pixel addition
              if(start_segment && adding)
                {
                  startIndexes.push_back(points.size()-1);
                  start_segment = false;
                }

              // close last segment by toggling start_segment flag
              else if(!start_segment && !adding)
                {
                  float dist = 
                    Point2D<int>(x,y).distance(points[points.size()-1]);
                  if(dist > 1.5) start_segment = true;
                }
            }

          if (x == p2.i) break;
          if (d >= 0) { y += sy; d -= ax; }
          x += sx; d += ay;
        }
    }
  else
    {
      int d = ax - (ay >> 1);
      for (;;)
        {
          bool adding = false;

          if (x >= 0 && x < w && y >= 0 && y < h)
            {
              if(edgeMap.getVal(x,y) > 0)
                {
                  points.push_back(Point2D<int>(x,y)); 
                  adding = true;
                }
              else if(points.size() > 0)
                {
                  uint size = points.size();
                  Point2D<int> ppt = points[size-1];

                  // get points that are neighbors 
                  // of the previous point
                  for(int di = 0; di <= 1; di++)
                    for(int dj = 0; dj <= 1; dj++)
                      {
                        if(di == 0 && dj == 0) continue;
                        if(!edgeMap.coordsOk(x+di,y+dj)) continue;

                        if(edgeMap.getVal(x+di,y+dj) && 
                           abs((x+di)-ppt.i <= 1) && abs((y+dj)-ppt.j <= 1))
                          {
                            points.push_back(Point2D<int>(x+di,y+dj));
                            adding = true;
                          }
                      }
                }

              // start new segment on the current pixel addition
              if(start_segment && adding)
                {
                  startIndexes.push_back(points.size()-1);
                  start_segment = false;
                }

              // close last segment by toggling start_segment flag
              else if(!start_segment && !adding)
                {
                  float dist = 
                    Point2D<int>(x,y).distance(points[points.size()-1]);
                  if(dist > 1.5) start_segment = true;
                }
            }

          if (y == p2.j) break;
          if (d >= 0) { x += sx; d -= ay; }
          y += sy; d += ax;
        }
    }
  return points;
}

// ######################################################################
std::vector<Point2D<int> >  
BeoRoadFinder::getPixelsQuick
(Point2D<int> p1, Point2D<int> p2, Image<byte> edgeMap)
{
  std::vector<Point2D<int> > points;

  // from Graphics Gems / Paul Heckbert
  int dx = p2.i - p1.i, ax = abs(dx) << 1, sx = signOf(dx);
  int dy = p2.j - p1.j, ay = abs(dy) << 1, sy = signOf(dy);
  int x = p1.i, y = p1.j;

  const int w = edgeMap.getWidth();
  const int h = edgeMap.getHeight();

  // flag to start new segment for the next hit
  if (ax > ay)
    {
      int d = ay - (ax >> 1);
      for (;;)
        {
          if (x >= 0 && x < w && y >= 0 && y < h)
            {
              if(edgeMap.getVal(x,y) > 0)
                  points.push_back(Point2D<int>(x,y)); 
            }

          if (x == p2.i) break;
          if (d >= 0) { y += sy; d -= ax; }
          x += sx; d += ay;
        }
    }
  else
    {
      int d = ax - (ay >> 1);
      for (;;)
        {
          if (x >= 0 && x < w && y >= 0 && y < h)
            {
              if(edgeMap.getVal(x,y) > 0)
                {
                  points.push_back(Point2D<int>(x,y)); 
                }
            }

          if (y == p2.j) break;
          if (d >= 0) { x += sx; d -= ay; }
          y += sy; d += ax;
        }
    }
  return points;
}


// ######################################################################
Line BeoRoadFinder::findLine2
(Segment s, Image<byte> edgeMap, 
 std::list<Segment> supportingSegments, std::vector<bool>& is_used,
 float& totalLength, uint& numSegments)
{
  Point2D<int> p1 = s.p1;
  Point2D<int> p2 = s.p2;

  Line l; 
  l.segments.push_back(s);
  l.length = s.length;
  std::vector<Point2D<int> > points = getPixels(p1,p2,edgeMap);

  //
  //float slope;
  //float y_intercept;
  //getLineEquation(p1,p2, slope, y_intercept);
  float distance_threshold  = 7.0F;
  float distance_threshold2 = 5.0F;

  // find points within distance
  uint index = 0;
  totalLength = s.length;
  numSegments = 1;

  std::list<Segment>::iterator itr  = supportingSegments.begin();
  for(uint i = 0; i < is_used.size(); i++)
    { 
      Segment s2 = (*itr);
      Point2D<int> p2_1 = s2.p1;
      Point2D<int> p2_2 = s2.p2;
      float length = s2.length;
    
      itr++; index++;
    
      if(is_used[i]) continue; 
    
      int mid_left_count  = 0;
      int mid_right_count = 0;

      bool is_inline       = true; 
      bool is_close_inline = true;
      std::vector<Point2D<int> > curr_points = 
        getPixels(p2_1,p2_2,edgeMap);
      for(uint j = 0; j < curr_points.size(); j++)
        {
          float dist = distance(p1,p2, curr_points[j]);
          int   wsid = side(p1,p2, curr_points[j]);

          //LINFO("[%3d] dist: %f wsid: %d", j, dist,wsid);

          if(wsid <= 0) mid_left_count ++;
          if(wsid >= 0) mid_right_count++;

          if(dist > distance_threshold2)
            { is_close_inline = false; }

          if(dist > distance_threshold)
            { is_inline = false; j = curr_points.size(); }
        }

      //LINFO("[%d] is_inline:: %d ml: %d mr: %d ", 
      //      index-1, is_inline, mid_left_count, mid_right_count);

      // include 
      if(is_close_inline ||
         (is_inline && mid_left_count >= 2 && mid_right_count >= 2))
        {
          for(uint j = 0; j < curr_points.size(); j++)
            points.push_back(curr_points[j]);
          is_used[i] = true;

          totalLength  += length;
          numSegments++;

          //l.length = totalLength;
          //l.segments.push_back(s2);
          //LINFO("YES!!!!!");

          std::vector<Point2D<int> > curr_points = 
            getPixels(p2_1,p2_2,edgeMap);
          for(uint j = 0; j < curr_points.size(); j++)
            points.push_back(curr_points[j]);
        }
    }

  updateLine(l, points, totalLength);
  Point2D<float> point1 = l.onScreenHorizonSupportPoint;
  Point2D<float> point2 = l.onScreenRoadBottomPoint;
  float distance = point1.distance(point2);
  float score = l.score/distance;
  l.score = score;

  //LINFO("totalLength: %f -> %d|| points size: %d --> score: %f", 
  //      totalLength, int(l.segments.size()), int(points.size()), score);

  return l;
}

// ######################################################################
void BeoRoadFinder::updateLine(Line& l, std::vector<Point2D<int> > points,
                               float score)
{
  //LINFO("size: %" ZU , points.size());
  if(points.size() == 0){ l.score = -1.0F; return; }

  // fit a line using all the points
  Point2D<float> lp1, lp2; fitLine(points, lp1, lp2);
  l.points = points;
  l.score  = score;

  Point2D<float> tr1 = intersectPoint
    (lp1, lp2, Point2D<float>(0, HORIZON_LINE),
     Point2D<float>(WIDTH,HORIZON_LINE));

  Point2D<float> tr2 = intersectPoint
    (lp1, lp2, Point2D<float>(0, HEIGHT-1),
     Point2D<float>(WIDTH,HEIGHT-1)); 

  Point2D<float> tr3 = intersectPoint
    (lp1, lp2, Point2D<float>(0, HORIZON_SUPPORT_LINE),
     Point2D<float>(WIDTH,HORIZON_SUPPORT_LINE));

  l.horizonPoint            = tr1;
  l.horizonSupportPoint     = tr3;
  l.roadBottomPoint         = tr2;

  if(tr2.i >= 0 && tr2.i <= WIDTH) l.onScreenRoadBottomPoint = tr2;
  else if(tr2.i < 0)
      l.onScreenRoadBottomPoint = intersectPoint
        (lp1, lp2, Point2D<float>(0, 0), Point2D<float>(0,HEIGHT));
  else if(tr2.i > WIDTH)
      l.onScreenRoadBottomPoint = intersectPoint
        (lp1, lp2, Point2D<float>(WIDTH, 0), Point2D<float>(WIDTH,HEIGHT));

  if(tr1.i >= 0 && tr1.i <= WIDTH) l.onScreenHorizonPoint = tr1;
  else if(tr1.i < 0)
      l.onScreenHorizonPoint = intersectPoint
        (lp1, lp2, Point2D<float>(0, 0), Point2D<float>(0,HEIGHT));
  else if(tr1.i > WIDTH)
      l.onScreenHorizonPoint = intersectPoint
        (lp1, lp2, Point2D<float>(WIDTH, 0), Point2D<float>(WIDTH,HEIGHT));

  if(tr3.i >= 0 && tr3.i <= WIDTH) l.onScreenHorizonSupportPoint = tr3;
  else if(tr3.i < 0)
      l.onScreenHorizonSupportPoint = intersectPoint
        (lp1, lp2, Point2D<float>(0, 0), Point2D<float>(0,HEIGHT));
  else if(tr3.i > WIDTH)
      l.onScreenHorizonSupportPoint = intersectPoint
        (lp1, lp2, Point2D<float>(WIDTH, 0), Point2D<float>(WIDTH,HEIGHT));

  Point2D<float> p1 = l.horizonPoint;
  Point2D<float> p2 = l.roadBottomPoint;
  //LINFO("points: [%d %d][%d %d]", p1.i, p1.j, p2.i, p2.j);

  float dy = p2.j - p1.j;
  float dx = p2.i - p1.i;
  //float dist  = p1.distance(p2);

  // set it to 0 to M_PI
  float angle = atan2(dy,dx); 
  if(angle < 0.0) angle = M_PI + angle;

  l.angle = angle;
}


// ######################################################################
Line BeoRoadFinder::findLine
(Segment s, Image<byte> edgeMap, 
 std::list<Segment> supportingSegments, std::vector<bool>& is_used)
{
  Point2D<int> pt1 = s.p1;
  Point2D<int> pt2 = s.p2;

  // convert to polar line  
  double theta  = 0.0F;
  double radius = 0.0F;
  getPolarLine(pt1, pt2, theta, radius);

  std::vector<Point2D<int> > points;
  std::vector<Point2D<int> > seg_points = getPixels(pt1,pt2,edgeMap);
  for(uint i = 0; i < seg_points.size(); i++)
    points.push_back(seg_points[i]);

  std::vector<std::pair<Point2D<int>, Point2D<int> > > point_indexes;

  // threshold is 2 pixels and 5 degree
  float theta_threshold  = 10*M_PI/180.0F;         
  float radius_threshold = 5.0;  

  LINFO("thres: theta:%f  rad: %f", theta_threshold, radius_threshold);

  Image<PixRGB<byte> > edge_map = toRGB(edgeMap);
  for(uint j = 0; j < points.size(); j++)
    {
      //LINFO("[%3d]: %d %d", j, points[j].i, points[j].j);
      edge_map.setVal(points[j], PixRGB<byte>(255,0,0));
    }

  LINFO("[%3d %3d) (%3d %3d) : theta: %f radius: %f",
        pt1.i, pt1.j, pt2.i, pt2.j, theta, radius);

  itsOfs->writeRGB(edge_map, "Testing1");
  itsOfs->updateNext();
  //Raster::waitForKey();

  LINFO("add it");

  uint index = 0;
  std::list<Segment>::iterator itr  = supportingSegments.begin();
  for(uint i = 0; i < is_used.size(); i++)
    {    
      Point2D<int> pt2_1 = (*itr).p1;
      Point2D<int> pt2_2 = (*itr).p2;
      itr++; index++;

      if(is_used[i]) continue; 

      // convert to polar
      double theta2  = 0.0F; 
      double radius2 = 0.0F; 
      getPolarLine(pt2_1,pt2_2,theta2, radius2);

      LINFO("[%3d]  (%3d %3d) (%3d %3d) : theta: %f radius: %f",
            index-1, pt2_1.i, pt2_1.j, pt2_2.i, pt2_2.j, theta2, radius2);

      LINFO("diff: %f - %f && %f - %f --> %f < %f| %f < %f ->%d %d", 
            theta, theta2, radius, radius2, 
            fabs(theta  - theta2 ), theta_threshold, 
            fabs(radius - radius2), radius_threshold,
            fabs(theta  - theta2 ) < theta_threshold,
            fabs(radius - radius2) < radius_threshold);

      //Image<PixRGB<byte> > edge_map = toRGB(edgeMap);
      //drawLine(edge_map,pt2_1,pt2_2, PixRGB<byte >(255,0,0),2;)

      //itsOfs->writeRGB(edge_map, "Testing1");
      //itsOfs->updateNext();
      //Raster::waitForKey();

      if(fabs(theta  - theta2 ) < theta_threshold  &&
         fabs(radius - radius2) < radius_threshold   )
        {
          std::vector<Point2D<int> > curr_points = 
            getPixels(pt2_1,pt2_2,edgeMap);
          for(uint j = 0; j < curr_points.size(); j++)
            points.push_back(curr_points[j]);
          is_used[i] = true;

          point_indexes.push_back
            (std::pair<Point2D<int>,Point2D<int> >
             (pt2_1,pt2_2));          
        }
    }

  LINFO("it's out: used");
  for(uint i = 0; i < is_used.size(); i++)
    {
      LINFO("[%3d]: %d", i, int(is_used[i]));

    }

  // draw the points
  edge_map = toRGB(edgeMap);
  drawLine(edge_map,pt1,pt2, PixRGB<byte >(255,0,0),2);
  for(uint j = 0; j < points.size(); j++)
    {
      //LINFO("[%3d]: %d %d", j, points[j].i, points[j].j);
      //edge_map.setVal(points[j], PixRGB<byte>(255,255,0));
      //drawLine(edge_map,pt2_1,pt2_2, PixRGB<byte >(255,0,0),2);
    }

  for(uint j = 0; j < point_indexes.size(); j++)
    {
      //LINFO("[%3d]: %d %d", j, points[j].i, points[j].j);
      //edge_map.setVal(points[j], PixRGB<byte>(255,255,0));
      drawLine(edge_map,
               point_indexes[j].first,point_indexes[j].second, 
               PixRGB<byte >(255,0,0),2);
    }


  LINFO("[%3d %3d) (%3d %3d) : theta: %f radius: %f",
        pt1.i, pt1.j, pt2.i, pt2.j, theta, radius);

  itsOfs->writeRGB(edge_map, "Testing1");
  itsOfs->updateNext();
  Raster::waitForKey();


  Line l;
  return l;
}

// ######################################################################
void BeoRoadFinder:: fitLine
(std::vector<Point2D<int> > points, Point2D<float>&p1,Point2D<float>&p2)
{
  float *line = new float[4];
  //float linearity = 0.0f;

  CvMemStorage* storage = cvCreateMemStorage(0);
  //	CvSeq* point_seq = cvCreateSeq( CV_32FC2, sizeof(CvSeq), sizeof(CvPoint2D32f), storage );
  CvPoint* cvPoints = (CvPoint*)malloc(points.size()*sizeof(Point2D<int>));
  for(uint i = 0; i < points.size(); i++){
    int x = points.at(i).i;
    int y = points.at(i).j;
    //cvSeqPush(point_seq,cvPoint2D32f(x,y));
    cvPoints[i].x = x;
    cvPoints[i].y = y;
  }
  //	linearity = myLinearity(point_seq);
  CvMat point_mat = cvMat(1,points.size(), CV_32SC2, cvPoints);
  cvFitLine(&point_mat, CV_DIST_L2,0, 0.01,0.01,line);
  //LINFO("v=(%f,%f),vy/vx=%f,(x,y)=(%f,%f), Linearity=%f\n",line[0],line[1],line[1]/line[0],line[2],line[3],linearity);


  cvReleaseMemStorage(&storage);

  // double a, b, c, d, e, f,x0,y0,x1,y1;              // y = a + b*x, b is slop
  // b = line[1]/ line[0];                  // y = c + d*x
  // a = line[3]- b*line[2];                // y = e + f*x
  // d = -1/b;
  // c = points[0].j - d*points[0].i;
  // f = d;
  // e = points[points.size()-1].j - f*points[points.size()-1].i;

  // x0 = (a-c)/(d-b);                  // x = (a-c)/(d-b)
  // y0 = c+d*x0;                   // y = a + b*x
  // x1 = (a-e)/(f-b);
  // y1 = e+f*x1;

  // p1.i = (int)x0;              
  // p1.j = (int)y0;           
  // p2.i = (int)x1;
  // p2.j = (int)y1;

  double d = sqrt((double)line[0]*line[0] + (double)line[1]*line[1]);  
  line[0] /= d;  
  line[1] /= d;  
  float t = (float)(WIDTH + HEIGHT);  
  p1.i = line[2] - line[0]*t;  
  p1.j = line[3] - line[1]*t;  
  p2.i = line[2] + line[0]*t;  
  p2.j = line[3] + line[1]*t;  

  // p1.i = cvRound(line[2] - line[0]*t);  
  // p1.j = cvRound(line[3] - line[1]*t);  
  // p2.i = cvRound(line[2] + line[0]*t);  
  // p2.j = cvRound(line[3] + line[1]*t);  
}

// ######################################################################
float BeoRoadFinder::getLineFitness
(Point2D<int> horizonPoint, Point2D<int> roadBottomPoint, 
 Image<byte> edgeMap, bool printResult)
{
  std::vector<Point2D<int> > points;
  return 
    getLineFitness(horizonPoint, roadBottomPoint, edgeMap, 
                   points, printResult);
}

// ######################################################################
float BeoRoadFinder::getLineFitness
(Point2D<int> horizonPoint, Point2D<int> roadBottomPoint, Image<byte> edgeMap,
 std::vector<Point2D<int> >& points, bool printResult)
{
  float score = 0;

  int min_effective_segment_size = 5;

  // go through the points in the line
  Point2D<int> p1 = horizonPoint;
  Point2D<int> p2 = roadBottomPoint;
  //LINFO("points: [%d %d][%d %d]", p1.i, p1.j, p2.i, p2.j);

  //float dy = p2.j - p1.j;
  //float dx = p2.i - p1.i;
  float dist  = p1.distance(p2);
  //float angle = atan2(dy,dx); if(angle < 0.0) angle = M_PI + angle;

  points.clear();
  std::vector<uint> start_indexes;
  points = getPixels(p1,p2, edgeMap, start_indexes);

  int sp = 4;
  std::vector<Point2D<int> > lpoints;
  lpoints = getPixelsQuick(p1+Point2D<int>(-sp,0),p2+Point2D<int>(-sp,0),edgeMap);
  std::vector<Point2D<int> > rpoints;
  rpoints = getPixelsQuick(p1+Point2D<int>(sp,0),p2+Point2D<int>(sp,0),edgeMap);

  uint num_segments = start_indexes.size();
  float max_length  = 0.0F;
  int max_i1 = 0, max_i2 = 0;
  int total         = 0; int max = 0;
  float eff_length  = 0;
  for(uint i = 1; i < num_segments; i++)
    {
      int size = start_indexes[i] - start_indexes[i-1];
      if(max < size) max = size;
      total += size;

      uint i1 = start_indexes[i-1];
      uint i2 = start_indexes[i  ]-1;
      Point2D<int> pt1 = points[i1]; 
      Point2D<int> pt2 = points[i2];
      float length = pt1.distance(pt2);
      if(max_length < length) 
        { max_length = length; max_i1 = i1; max_i2 = i2; }

      if(size >= min_effective_segment_size) eff_length+= length;

      //LINFO("[%3d](%3d) (%3d): (%4d %4d) (%4d %4d) si:%d --> len:%6.2f:(%d|%6.2f)", 
      //     i-1, start_indexes[i-1], start_indexes[i]-1, pt1.i, pt1.j, pt2.i, pt2.j, size, length, total, eff_length);
    }

  if(num_segments > 0) 
    {
      int size = int(points.size()) - int(start_indexes[num_segments-1]);

      if(max < size) max = size;
      total += size;

      uint i1 = start_indexes[num_segments-1 ];
      uint i2 = points.size()-1;

      Point2D<int> pt1 = points[i1];
      Point2D<int> pt2 = points[i2]; 
      float length = pt1.distance(pt2);

      if(max_length < length) 
        { max_length = length; max_i1 = i1; max_i2 = i2; }

      if(size >= min_effective_segment_size) eff_length+= length;

      //LINFO("[%3d](%3d) (%3d): (%4d %4d) (%4d %4d) si:%d --> len:%6.2f:(%d|%6.2f)", 
      //     num_segments-1, start_indexes[num_segments-1], int(points.size()-1), pt1.i, pt1.j, pt2.i, pt2.j, size, length, total, eff_length);
    }

  if(max_length > 0.0)
    {
      //std::vector<Point2D<int> > max_points;
      //for(int i = max_i1; i < max_i2; i++)
      //  max_points.push_back(points[i]);

      //Point2D<float> mp1, mp2; fitLine(max_points, mp1, mp2);
      //float mdy    = mp2.j - mp1.j;
      //float mdx    = mp2.i - mp1.i;  
      //float mangle = atan2(mdy,mdx); if (mangle < 0) mangle = M_PI + mangle;

      //float dangle = (mangle-angle)*180/M_PI;

      //uint lsize = lpoints.size();
      //uint rsize = rpoints.size();

      uint lsize = lpoints.size();
      uint rsize = rpoints.size();

      //|| dangle > 90.0 

      // can't be bigger than 15 degrees or bad point
      if(dist <= 50.0 || points.size() < 2*(lsize+rsize)) 
        {
          score = 0.0;

          // LINFO("score is zero");
          // LINFO("%3d %3d || l: %" ZU " r: %" ZU " m: %" ZU , 
          //       max_i1, max_i2, lpoints.size(), rpoints.size(), points.size()); 
          // LINFO("<%3d %3d || %3d %3d>: angle: %f vs. %f = %f", 
          //       points[max_i1].i, points[max_i1].j,
          //       points[max_i2].i, points[max_i2].j, angle/M_PI*180.0, mangle/M_PI*180.0, dangle );
        }
      else score = eff_length/dist;

      //LINFO("%3d %3d || l: %" ZU " r: %" ZU " m: %" ZU " el: %f / d: %f = s: %f", 
      //      max_i1, max_i2, lpoints.size(), rpoints.size(), points.size(), 
      //      eff_length, dist, score); 

      //LINFO("<%3d %3d || %3d %3d>: angle: %f vs. %f = %f", 
      //      points[max_i1].i, points[max_i1].j,
      //      points[max_i2].i, points[max_i2].j, angle/M_PI*180.0, mangle/M_PI*180.0, dangle );

      //LINFO("max_length : %f, num_segments: %d score: %f", max_length, num_segments, score);

    }


  if(printResult)
    {
      LINFO("[%4d %4d][%4d %4d] num segments: %d : size: %d", 
            p1.i, p1.j, p2.i, p2.j, num_segments, int(points.size()));

      LINFO("total: %d: max: %d --> eff length: %6.2f/%6.2f = score: %f", 
            total, max, eff_length, dist, score);

      Image<PixRGB<byte> > displayImage = edgeMap; 
      drawLine(displayImage, p1, p2, PixRGB<byte >(0,0,255),1); 
      for(uint i = 0; i < points.size(); i++)    
        drawDisk(displayImage, points[i], 1, PixRGB<byte>(0,255,0));

      for(uint i = 0; i < lpoints.size(); i++)    
        drawDisk(displayImage, lpoints[i], 1, PixRGB<byte>(255,0,0));

      for(uint i = 0; i < rpoints.size(); i++)    
        drawDisk(displayImage, rpoints[i], 1, PixRGB<byte>(255,0,0));

      itsOfs->writeRGB(zoomXY(displayImage, 4), "line fitness");
      itsOfs->updateNext();
      Raster::waitForKey();
    }
  return score;
}

// ######################################################################
void BeoRoadFinder::trackVanishingLines
(Image<byte> edgeMap, std::vector<Line> &currentLines, bool print)
{



  // LINFO("Start Tracking: %" ZU " lines", currentLines.size());
  // for(uint i = 0; i < currentLines.size(); i++) 
  //   {
  //     // draw the line
  //     Point2D<float> p1 = currentLines[i].onScreenHorizonSupportPoint;
  //     Point2D<float> p2 = currentLines[i].onScreenRoadBottomPoint;
  //     float score = currentLines[i].score;
  //     int ss = currentLines[i].start_scores.size();
  //     LINFO("(%4d)[%8.3f %8.3f][%8.3f %8.3f]: %d %10.4f", 
  //           i, p1.i, p1.j, p2.i, p2.j, ss, score);
  //   }





  // display the line and the canny image
  Image<PixRGB<byte> > displayImage = edgeMap; 

  for(uint i = 0; i < currentLines.size(); i++) 
    {
      // draw the line
      Point2D<float> p1 = currentLines[i].onScreenHorizonSupportPoint;
      Point2D<float> p2 = currentLines[i].onScreenRoadBottomPoint;

      Point2D<int> pi1(p1 + .5);
      Point2D<int> pi2(p2 + .5);

      //LINFO("Line %3d", i);
      // drawLine(displayImage, pi1, pi2, PixRGB<byte >(0,0,255),1); 
      // itsOfs->writeRGB(displayImage, "Tracker");
      // itsOfs->updateNext();
      // Raster::waitForKey();

      float max_score = 0.0F;
      int max_di1 = -10; int max_di2 = -10;
      std::vector<Point2D<int> > max_points;
      for(int di1 = -10; di1 <= 10; di1+= 2)
        for(int di2 = -10; di2 <= 10; di2+= 2) 
          {
            Point2D<int> pn1(pi1.i+di1, pi1.j);
            Point2D<int> pn2(pi2.i+di2, pi2.j);
            std::vector<Point2D<int> > points;

            float score = 
              getLineFitness(pn1,pn2, edgeMap, points);

            if(print)
              {
                //LINFO("(%4d)(%4d)[%4d %4d][%4d %4d](%5d) score: %10.4f", 
                //      di1, di2, pn1.i, pn1.j, pn2.i, pn2.j, int(points.size()), score);
                Line l; updateLine(l, points, score);

                Point2D<float> pl1 = l.onScreenHorizonSupportPoint;
                Point2D<float> pl2 = l.onScreenRoadBottomPoint;

                Point2D<float> pl3 = l.horizonSupportPoint;
                Point2D<float> pl4 = l.roadBottomPoint;

                //LINFO("osh[%7.2f %7.2f]|osr[%7.2f %7.2f]|h[%7.2f %7.2f]|r[%7.2f %7.2f]", 
                //                   pl1.i, pl1.j, pl2.i, pl2.j, pl3.i, pl3.j, pl4.i, pl4.j);

                displayImage = edgeMap;
                drawLine(displayImage, pn1, pn2, PixRGB<byte >(0,0,255),1); 
                for(uint j = 0; j < points.size(); j++)
                  {
                    drawDisk(displayImage, points[j], 1, PixRGB<byte>(255,0,0));
                    //LINFO("[%3d]: %4d %4d", j, points[j].i,points[j].j);
                  }

                itsOfs->writeRGB(displayImage, "Tracker");
                itsOfs->updateNext();
                Raster::waitForKey();
              }

            if(score > max_score)
              { 
                max_score = score; max_di1 = di1; max_di2 = di2; 
                max_points = points;
              }
          }

      // update the vanishing line
      if(max_score > 0)
        updateLine(currentLines[i], max_points, max_score);
      else currentLines[i].score = max_score;
      currentLines[i].segments.clear();     

      Point2D<float> rbp = currentLines[i].roadBottomPoint;
      //LINFO("[%4d]: %7.2f %7.2f: %10.4f", i, rbp.i, rbp.j, max_score);
    }

  // check for start values 
  std::vector<Line> temp_lines;
  for(uint i = 0; i < currentLines.size(); i++)
    {
      uint num_sscore = currentLines[i].start_scores.size();
      //LINFO("ss[%4d]: %4d", i, num_sscore);

      // still in starting stage
      if(num_sscore > 0)
        {
          currentLines[i].start_scores.push_back
            (currentLines[i].score);
          num_sscore++;

          //float first_value = currentLines[i].start_scores[0];

          uint num_high = 0;
          for(uint j = 0; j < num_sscore; j++)
            {
              if(currentLines[i].start_scores[j] > .5) num_high++;
            }

          // ready to be used
          if(num_high > 5) 
            {
              currentLines[i].start_scores.clear();
              //LINFO("consistent enough");
              num_sscore = 0;
            }
    
          if(num_sscore < 7) temp_lines.push_back(currentLines[i]);
          //else LINFO("\n\n!!!getting rid of line %d", i);
        }
      else temp_lines.push_back(currentLines[i]);
    }
  currentLines.clear();
  for(uint i = 0; i < temp_lines.size(); i++)
    currentLines.push_back(temp_lines[i]);

  // LINFO("Tracking: %" ZU " tlines.1", currentLines.size());
  // for(uint i = 0; i < currentLines.size(); i++) 
  //   {
  //     Point2D<float> p1 = currentLines[i].onScreenHorizonSupportPoint;
  //     Point2D<float> p2 = currentLines[i].onScreenRoadBottomPoint;
  //     float score = currentLines[i].score;
  //     LINFO("(%4d)[%8.3f %8.3f][%8.3f %8.3f]: %10.4f", 
  //           i, p1.i, p1.j, p2.i, p2.j, score);
  //   }

  // check lines to see if any lines are below the threshold
  temp_lines.clear();
  for(uint i = 0; i < currentLines.size(); i++)
    {           
      if(currentLines[i].score < 0.3 || 
         currentLines[i].scores.size() > 0)        
        currentLines[i].scores.push_back(currentLines[i].score);

      uint num_low = 0;
      int size = currentLines[i].scores.size();
      bool all_good_values = true;
      for(int j = 0; j < size; j++)
        if(currentLines[i].scores[j] < 0.3) 
          { num_low++; all_good_values = false; }
    
      // keep until 5 of 7 bad values
      if(num_low < 5)
        {     
          // update the values
          if(all_good_values) currentLines[i].scores.clear();
          else
            {
              std::vector<float> vals;
              if(size > 7)
                {
                  for(int j = size-7; j < size; j++) 
                    vals.push_back(currentLines[i].scores[j]);
                  currentLines[i].scores = vals;
                }
            }
          temp_lines.push_back(currentLines[i]);          
        }
      //else LINFO("\n\n!!!getting rid of line %d", i);
    }

  //LINFO("Tracking: %" ZU " tlines.2", temp_lines.size());

  currentLines.clear();
  for(uint i = 0; i < temp_lines.size(); i++)
    currentLines.push_back(temp_lines[i]);




  // LINFO("Tracking: %" ZU " filtered lines", currentLines.size());
  // for(uint i = 0; i < currentLines.size(); i++) 
  //   {
  //     // draw the line
  //     Point2D<float> p1 = currentLines[i].onScreenHorizonSupportPoint;
  //     Point2D<float> p2 = currentLines[i].onScreenRoadBottomPoint;
  //     int ss = currentLines[i].start_scores.size();
  //     float score = currentLines[i].score;
  //     LINFO("(%4d)[%8.3f %8.3f][%8.3f %8.3f]: %10.4f  ==> %d", 
  //           i, p1.i, p1.j, p2.i, p2.j, score, ss);
  //   }
}

// ######################################################################
void BeoRoadFinder::projectForwardVanishingLines
(std::vector<Line>& lines, std::vector<Image<byte> > edgeMaps)
{
  // project forward the lines
  // using all the frames that are just passed
  for(uint i = 0; i < edgeMaps.size(); i++)
    {
      trackVanishingLines(edgeMaps[i], lines);
    }
}

// ######################################################################
std::vector<Line> BeoRoadFinder::combine
(std::vector<Line> prevLines, std::vector<Line> currentLines)
{
  std::vector<Line> combLines;
  std::vector<bool> cline_isadded(currentLines.size());
  for(uint i = 0; i < cline_isadded.size(); i++) cline_isadded[i] = false;

  //LINFO("there are: %" ZU " prevLines and %" ZU " currentLines", 
  //      prevLines.size(), currentLines.size());

  prevLines = discardDuplicates(prevLines);

  // LINFO("there are: %" ZU " prevLines and %" ZU " currentLines (no duplicates)", 
  //       prevLines.size(), currentLines.size());

  // integrate the two trackers  
  for(uint j = 0; j < prevLines.size(); j++)
    {
      Point2D<float> pp1 = prevLines[j].onScreenHorizonSupportPoint;
      Point2D<float> pp2 = prevLines[j].onScreenRoadBottomPoint;         
      float score_pl = prevLines[j].score;

      float min_dist = -1.0F; int min_i = -1;
      std::vector<uint> match_index;
      for(uint i = 0; i < currentLines.size(); i++)
        {
          Point2D<float> cp1 = currentLines[i].onScreenHorizonSupportPoint;
          Point2D<float> cp2 = currentLines[i].onScreenRoadBottomPoint;
          //float score_cl = currentLines[i].score;
      
          // check the two ends of the vanishing points
          float dist1 = cp1.distance(pp1);
          float dist2 = cp2.distance(pp2);
          float dist = dist1+dist2;
    
          // LINFO("C[%3d] (%6.2f, %6.2f)-(%6.2f, %6.2f) -- "
          //       "P[%3d] (%6.2f, %6.2f)-(%6.2f, %6.2f) "
          //       ": CPscore: %f PVscore: %f ||%f + %f = %f",
          //       i, cp1.i, cp1.j, cp2.i, cp2.j, j, pp1.i, pp1.j, pp2.i, pp2.j,
          //       score_cl, score_pl, dist1, dist2, dist);

          // if the lines are close enough
          if(dist < 7.0F)
            {
              // LINFO("match: C[%d] with P[%d]", i,j);
              match_index.push_back(i);
              if(min_dist == -1.0F || min_dist > dist) 
                { min_dist = dist; min_i = i; }
            }
        }

      //LINFO("size m_index: %" ZU , match_index.size());
      // combine lines if there are duplicates
      if(match_index.size() > 0)
        {
          // if matched with more than 1 
          // pick the closest one 
          // and use the one with the higher score
          float score_mcl = currentLines[min_i].score;   
          Line l;
          if(score_pl > score_mcl)
            combLines.push_back(prevLines[j]);
          else
            {
              updateLine(l, currentLines[min_i].points, score_mcl);

              l.start_scores = prevLines[j].start_scores;               
              uint ss_size = l.start_scores.size(); 
              if(ss_size > 0) l.start_scores[ss_size-1] = score_mcl;
           
              l.scores = prevLines[j].start_scores; 
              uint s_size = l.scores.size(); 
              if(s_size > 0) l.scores[s_size-1] = score_mcl;

              l.isActive      = prevLines[j].isActive;
              l.angleToCenter = prevLines[j].angleToCenter;
              l.pointToServo  = prevLines[j].pointToServo;
              l.offset        = prevLines[j].offset;
              l.index         = prevLines[j].index;
              combLines.push_back(l);
            }

          // but all the other lines are discarded
          for(uint i = 0; i < match_index.size(); i++)
            {
              //LINFO("match index[%d] %d", i, match_index[i]);
              cline_isadded[match_index[i]] = true;
            }
          //LINFO("adding prevline[%d] with matches", j);
        }
      else 
        {
          combLines.push_back(prevLines[j]);
          //LINFO("adding prevline[%d] without matches", j);
        }
    }

  for(uint i = 0; i < cline_isadded.size(); i++)
    {
      //LINFO("added[%d]: %d", i, int(pline_isadded[i]));

      if(!cline_isadded[i]) 
        {
          combLines.push_back(currentLines[i]);
          //LINFO("adding currentline[%d]",i);
        }
    }

  // for(uint j = 0; j < combLines.size(); j++)
  //   {
  //     Point2D<float> pp1 = combLines[j].onScreenHorizonSupportPoint;
  //     Point2D<float> pp2 = combLines[j].onScreenRoadBottomPoint;         
  //     float score_pl = combLines[j].score;
    
  //     LINFO("C[%3d] (%6.2f, %6.2f)-(%6.2f, %6.2f): %f",
  //           j, pp1.i, pp1.j, pp2.i, pp2.j, score_pl);
  //   } 

  //LINFO("total combline: %" ZU , combLines.size());
  return combLines;
 }


// ######################################################################
std::vector<Line> BeoRoadFinder::discardDuplicates
(std::vector<Line> lines)
{
  std::vector<Line> newLines;
  std::vector<bool> line_isadded(lines.size());
  for(uint i = 0; i < line_isadded.size(); i++) line_isadded[i] = false;


  // go through each  
  for(uint j = 0; j < lines.size(); j++)
    {
      Point2D<float> pp1 = lines[j].onScreenHorizonSupportPoint;
      Point2D<float> pp2 = lines[j].onScreenRoadBottomPoint;         
      Line  line_added = lines[j];

      //      LINFO("P[%3d] (%6.2f, %6.2f)-(%6.2f, %6.2f): %f",
      //      j, pp1.i, pp1.j, pp2.i, pp2.j, line_added.score);
    }

  // go through each  
  for(uint j = 0; j < lines.size(); j++)
    {
      if(line_isadded[j]) continue;

      Point2D<float> pp1 = lines[j].onScreenHorizonSupportPoint;
      Point2D<float> pp2 = lines[j].onScreenRoadBottomPoint;         
      Line  line_added = lines[j];

      for(uint i = j+1; i < lines.size(); i++)
        {
          if(line_isadded[i]) continue;

          Point2D<float> cp1 = lines[i].onScreenHorizonSupportPoint;
          Point2D<float> cp2 = lines[i].onScreenRoadBottomPoint;
          float score_cl2 = lines[i].score;
      
          // check the two ends of the vanishing points
          float dist1 = cp1.distance(pp1);
          float dist2 = cp2.distance(pp2);
          float dist = dist1+dist2;

          // LINFO("C[%3d] (%6.2f, %6.2f)-(%6.2f, %6.2f) -- "
          //       "P[%3d] (%6.2f, %6.2f)-(%6.2f, %6.2f) "
          //       ": CPscore: %f PVscore: %f ||%f + %f = %f",
          //       i, cp1.i, cp1.j, cp2.i, cp2.j, j, pp1.i, pp1.j, pp2.i, pp2.j,
          //       line_added.score, score_cl2, dist1, dist2, dist);

          // if the lines are close enough
          if(dist < 3.0F)
            {
              line_isadded[i] = true;
              // LINFO("MATCH: C[%d] with P[%d]", i,j);

              if(line_added.score < score_cl2)
                line_added = lines[i];
            }
        }

      newLines.push_back(line_added);
    }

  // LINFO("new: %" ZU , newLines.size());
  return newLines;
 }

// ######################################################################
void BeoRoadFinder::publishScreenShot(Image<PixRGB<byte> > img)
{
  //BeobotEvents::VisualizerMessagePtr msg = 
  //  new BeobotEvents::VisualizerMessage;
  //msg->image     = Image2Ice(img); 
  //msg->BeoAppID  = BEO_ROADFINDER;
  //msg->RequestID = itsCurrentMessageID++;
  //this->publish("VisualizerMessageTopic", msg);

  std::string saveTName(sformat("./beoRF_%05d.png",itsCurrentMessageID++));

  //save screen shot to file
  Raster::WriteRGB(img, saveTName);
  LINFO("Save %s ",saveTName.c_str());

}

// ######################################################################
Image<PixRGB<byte> > BeoRoadFinder::getDisplayImage
(Image<PixRGB<byte> > ima, Image<byte> edgeMap, 
std::vector<Line> currentLines, double rHead)
{
  // define drawing colors
  PixRGB<byte> black (  0,  0,  0);
  PixRGB<byte> red   (255,  0,  0);
  PixRGB<byte> blue  (  0,  0,255);
  PixRGB<byte> green (  0,255,  0);
  PixRGB<byte> yellow(255,255,  0); 
  PixRGB<byte> orange(255,128,  0); //green: 69?
  PixRGB<byte> cyan  (  0,255,255);
  PixRGB<byte> white (255,255,255); 

  uint w = ima.getWidth();
  uint h = ima.getHeight();
  uint slack = w/4;
  Point2D<int> slack_pt(slack,0);

  Image<PixRGB<byte> > dispImage(w+2*slack+w, h+slack, ZEROS);

  Image<PixRGB<byte> > vp_vote_ima(w+2*slack, h, ZEROS);
  inplacePaste(vp_vote_ima, ima, Point2D<int>(slack,0));

  // find the most likely vanishing point location
  uint  max_il  = 0;
  float max_l  = itsVanishingPoints[max_il].likelihood;
  uint  max_ip = 0;
  float max_p  = itsVanishingPoints[max_ip].posterior;
  for(uint i = 0; i < itsVanishingPoints.size(); i++)
    {
      float likelihood = itsVanishingPoints[i].likelihood;
      float posterior = itsVanishingPoints[i].posterior;
      if(max_l < likelihood) { max_l  = likelihood; max_il = i; }
      if(max_p < posterior)  { max_p  = posterior;  max_ip = i; }
    }

  // drawLine(vp_vote_ima, Point2D<int>(0,131), Point2D<int>(479,131), green, 1); 
  // drawLine(vp_vote_ima, Point2D<int>(0,130), Point2D<int>(479,130), green, 1); 
  // drawLine(vp_vote_ima, Point2D<int>(0,129), Point2D<int>(479,129), green, 1); 

  // draw the vanishing point likelihoods
  for(uint i = 0; i < itsVanishingPoints.size(); i++)
    {
      float likelihood = itsVanishingPoints[i].likelihood;
      float posterior  = itsVanishingPoints[i].posterior;
      Point2D<int> vp  = itsVanishingPoints[i].vp;
      int l_size = likelihood/max_l*10;
      int p_size = posterior/max_l*10;

      if(l_size < 2) l_size = 2;
      if(p_size < 2) p_size = 2;

      Point2D<int> pt = vp+slack_pt;
      if(i == max_il) drawDisk(vp_vote_ima, pt, l_size, yellow); // orange
      else            drawDisk(vp_vote_ima, pt, l_size, yellow);

      if(i == max_ip) drawDisk(vp_vote_ima, pt, p_size, green );
      else            drawDisk(vp_vote_ima, pt, p_size, red   );

      //LINFO("here: %d %d", l_size, p_size);
    }

  // horizon support point
  //drawLine(vp_vote_ima, Point2D<int>(0,151), Point2D<int>(479,151), green, 1); 
  //drawLine(vp_vote_ima, Point2D<int>(0,150), Point2D<int>(479,150), green, 1); 
  //drawLine(vp_vote_ima, Point2D<int>(0,149), Point2D<int>(479,149), green, 1); 


  // for saving
  Image<PixRGB<byte> > edge_map    = toRGB(edgeMap);
  Image<PixRGB<byte> > segment_map = edge_map;

  // all the segments found
  std::vector<Segment> segments = itsCurrentSegments;
  // LINFO("clsize: %d size: %d", 
  //       int(currentLines.size()), int(itsCurrentSegments.size()));
  for(uint i = 0; i < segments.size(); i++)
    {
      Point2D<int> pt1 = segments[i].p1;
      Point2D<int> pt2 = segments[i].p2;
      drawLine(segment_map,pt1,pt2, red, 1);
      //drawLine(dispImage, pt1+slack_pt, pt2+slack_pt, green, 2); 
    }

  // draw the supporting segments
  std::vector<Segment> s_segments = 
    itsVanishingPoints[max_ip].supportingSegments;




  // float length = 0; uint lind = 0;
  // for(uint i = 0; i < s_segments.size(); i++)
  //   {
  //     Point2D<int> pt1 = s_segments[i].p1;
  //     Point2D<int> pt2 = s_segments[i].p2;
  //     float l = pt1.distance(pt2);
  //     if(length < l) { length = l; lind = i; }
  //   }





  // for(uint i = 0; i < s_segments.size(); i++)
  //   {
  //     Point2D<int> pt1 = s_segments[i].p1;
  //     Point2D<int> pt2 = s_segments[i].p2;
  //     //drawLine(segment_map,pt1,pt2, red, 2);
  //     drawLine(vp_vote_ima, pt1+slack_pt, pt2+slack_pt, red, 1); 
  //   }






  // for(uint i = 0; i < s_segments.size(); i++)
  //   {
  //     Point2D<int> pt1 = s_segments[i].p1;
  //     Point2D<int> pt2 = s_segments[i].p2;
  //     //drawLine(segment_map,pt1,pt2, red, 2);
  //     drawLine(vp_vote_ima, pt1+slack_pt, pt2+slack_pt, red, 1); 
  //   }






  // draw the tracked lines' pixels
  // for(uint i = 0;  i < currentLines.size(); i++)
  //   {  
  //     std::vector<Point2D<int> > points = currentLines[i].points;
  //     for(uint j = 0; j < points.size(); j++)
  //       drawDisk(segdisp_map, points[j], 1, red);
  //   }

  inplacePaste(dispImage, vp_vote_ima, Point2D<int>(0,0));
  inplacePaste(dispImage, segment_map, Point2D<int>(w+2*slack,0));

  // lateral position information
  its_Road_Information_mutex.lock(); 
  Point2D<int>   vp = itsVanishingPoint;
  Point2D<float> cp = itsCenterPoint;
  Point2D<float> tp = itsTargetPoint;
  //float confidence = itsVanishingPointConfidence;
  its_Road_Information_mutex.unlock(); 


  // for saving
  Image<PixRGB<byte> > line_map   = ima;

  // draw current tracked lines
  for(uint i = 0;  i < currentLines.size(); i++)
    {  

      // std::vector<Segment> l_segments = currentLines[i].segments;
      // for(uint j = 0; j < l_segments.size(); j++)
      //   {
      //     Point2D<int> pt1 = l_segments[j].p1;
      //     Point2D<int> pt2 = l_segments[j].p2;
      //     //drawLine(dispImage, pt1+slack_pt, pt2+slack_pt, cyan, 2); 
      //     //LINFO("  [%4d %4d][%4d %4d]", pt1.i, pt1.j, pt2.i, pt2.j);
      //   }

      Point2D<float> p1  = currentLines[i].horizonPoint;
      Point2D<float> p2  = currentLines[i].roadBottomPoint;
      Point2D<float> po1 = currentLines[i].onScreenHorizonSupportPoint;
      Point2D<float> po2 = currentLines[i].onScreenRoadBottomPoint;
      //LINFO("[%3d] p1: %f %f p2: %f %f po1: %f %f po2: %f %f",
      //      i, p1.i, p1.j, p2.i, p2.j, po1.i, po1.j, po2.i, po2.j);

      //float score   = currentLines[i].score;
      //int cline = int(score*5)+1;

      //LINFO("[%3d]: start: %d score: %d", i,
      //      int(currentLines[i].start_scores.size()), 
      //      int(currentLines[i].scores.size()      ) );
      PixRGB<byte> line_color;
      if(currentLines[i].start_scores.size() > 0) line_color = green;
      else if(currentLines[i].scores.size() > 0)  line_color = yellow;
      else                                        line_color = blue;

      //if(Point2D<float>(vp.i,vp.j).distance(p1) < 20)  continue;



      Point2D<int> pt1(p1+.5); Point2D<int> pt2(p2+.5);


      Point2D<int> pto1(po1+.5); Point2D<int> pto2(po2+.5);
      //drawLine(dispImage, pto1+slack_pt, pto2+slack_pt, red,1); 

      //Point2D<int> pp1(pto1.i-4, pto1.j); Point2D<int> pp2(pto2.i+4, pto2.j);
      //drawLine(dispImage,   pp1+slack_pt, pp2+slack_pt, yellow,1); 
      //drawLine(vp_vote_ima, pp1+slack_pt, pp2+slack_pt, yellow,1); 


      drawLine(dispImage, pto1+slack_pt, pto2+slack_pt, line_color,2); 
      drawLine(vp_vote_ima, pto1+slack_pt, pto2+slack_pt, line_color,2); 


      //drawLine(dispImage, pto1+slack_pt, pto2+slack_pt, red,1); 


      // for(int ii = -10; ii <= 10; ii+=2)
      //   {
      //     drawDisk(dispImage, pp1+slack_pt+Point2D<int>(ii,0), 1, red);
      //     drawDisk(dispImage, pp2+slack_pt+Point2D<int>(ii,0), 1, red);

      //     drawDisk(vp_vote_ima, pp1+slack_pt+Point2D<int>(ii,0), 1, red);
      //     drawDisk(vp_vote_ima, pp2+slack_pt+Point2D<int>(ii,0), 1, red);
      //   }

      // drawDisk(vp_vote_ima, pp1+slack_pt, 1, red);
      // drawDisk(vp_vote_ima, pp2+slack_pt, 1, red);
      // drawDisk(vp_vote_ima, pp1+slack_pt  , 1, cyan);
      // drawDisk(vp_vote_ima, pp2+slack_pt  , 1, cyan);


      // // highlight the segment points
      // for(uint j = mstart; j < mstart+mlength; j++)
      //   {
      //     drawDisk(dispImage, points[j]+slack_pt  , 2, cyan);
      //     //drawDisk(line_map, points[j], 2, cyan);

      //     drawDisk(vp_vote_ima, points[j]+slack_pt  , 2, cyan);
      //   }




      //bin/app-BeoCamera --icestorm-ip=bx7 --in=../data/logs/2012_08_31__10_58_48/image_000_#.ppm --out=none --input-frames=54160-MAX@1Hz --rescale-input=320x240
      // then move 11 more; remember to get a line first


      std::string text = sformat("%d:%3d", currentLines[i].index, pto2.i);
      writeText(dispImage,Point2D<int>(pto2.i,h+i*20)+slack_pt,
                text.c_str(), white, black, SimpleFont::FIXED(10));

      drawLine(line_map, pt1 , pt2 , yellow,1); //line_color 
      drawLine(line_map, pto1, pto2, red       ,1); 



      // uint sstart = 0; uint length = 0;
      // uint mstart = 0; uint mlength = 0;

      // LINFO("wow");

      // highlight the segment points
      std::vector<Point2D<int> > points = currentLines[i].points;
      for(uint j = 0; j < points.size()-1; j++)
        {
          //LINFO("[%3d]: %4d %4d", j, points[j].i, points[j].j);
   
          Point2D<int> pt1 = points[j];
          Point2D<int> pt2 = points[j+1];
    
          // LINFO("[%3d] pt1: %d %d ; pt2: %d %d --> %f l: %d", j, pt1.i, pt1.j, pt2.i, pt2.j,
          //       pt1.distance(pt2),length);
          // if(pt1.distance(pt2) < 3) length++; 
          // else 
          //   {
          //     LINFO("stop: %d %d", sstart,length);
          //     if(mlength < length) {mlength = length; mstart = sstart; }
          //     sstart = j+1; length = 0;
          //   }

          // if(length > mlength) { mstart = sstart; mlength = length; }

          drawDisk(dispImage, points[j]+slack_pt  , 2, red);
          //drawDisk(dispImage, points[j]+(slack_pt+slack_pt)+w, 1, red);

          //drawDisk(line_map, points[j], 2, cyan);
        }
      // LINFO("mstart: %d mlength: %d", mstart, mlength);



      //break; 

    }

  // for saving
  Image<PixRGB<byte> > track_map  = line_map;
  Image<PixRGB<byte> > refine_map = line_map;
  for(uint i = 0; i < s_segments.size(); i++)
    {
      Point2D<int> pt1 = s_segments[i].p1;
      Point2D<int> pt2 = s_segments[i].p2;
      //drawLine(segment_map,pt1,pt2, red, 2);
      drawLine(line_map, pt1, pt2, red, 1); 

    }

//  int vpi = vp.i; vp.i = vpi - 3 ;
//  int cpi = cp.i; cp.i = cpi - 35;
//  int tpi = tp.i; tp.i = tpi - 35;

  // draw the lateral position point
  Point2D<int> cp_i(cp.i+slack, cp.j); 
  Point2D<int> tp_i(tp.i+slack, tp.j);   
  Point2D<int> cp_i0(cp.i+slack, 220);
  Point2D<int> tp_i0(tp.i+slack, 220); 
  if(cp_i.isValid()) drawLine(dispImage, cp_i0,cp_i, yellow,2);
  if(tp_i.isValid()) drawLine(dispImage, tp_i0,tp_i, blue  ,2);

  //bin/app-BeoCamera --icestorm-ip=bx7 --in=../data/logs/2012_08_31__10_52_13/image_000_#.ppm --out=none --input-frames=44370-MAX@1Hz --rescale-input=320x240
  Point2D<int> spt(slack,0);
  if(vp.isValid())   drawDisk(refine_map, vp, 4, red);
  if(cp_i.isValid()) drawLine(refine_map, cp_i0-spt,cp_i-spt, yellow,2);
  if(tp_i.isValid()) drawLine(refine_map, tp_i0-spt,tp_i-spt, blue  ,2);


  std::string text = 
    sformat("vp: %4d B: %7.2f Y: %7.2f", vp.i, tp.i, cp.i);
  writeText(dispImage,Point2D<int>(0,0),text.c_str(), white, black,
            SimpleFont::FIXED(16));

   std::string name("input");
  // Raster::WriteRGB(ima     ,    sformat("%s.png"    ,             name.c_str()));
  // Raster::WriteRGB(edge_map,    sformat("%s_edgemap.png",         name.c_str()));
  // Raster::WriteRGB(segment_map, sformat("%s_segmentmap.png",      name.c_str()));
  //Raster::WriteRGB(vp_vote_ima, sformat("%s_vp_voting.png",       name.c_str()));
  // Raster::WriteRGB(line_map,    sformat("%s_line_extraction.png", name.c_str()));
  // Raster::WriteRGB(track_map,   sformat("%s_line_tracking.png",   name.c_str()));
  // Raster::WriteRGB(refine_map,  sformat("%s_vpld_refinement.png", name.c_str()));

  return dispImage;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
