/*!@file Robots2/Beobot2/HumanRobotInteraction/FaceDetector.H Ice Module to detect Faces    */
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
// Primary maintainer for this file: Dicky Sihite <sihite@usc.edu>
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/HumanRobotInteaction/FaceDetector.H
// $Id: FaceDetector.H 13902 2010-09-09 15:31:09Z lior $
//

#include "Image/OpenCVUtil.H"
#include <opencv/cv.h>
#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"

#include "Util/Timer.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include <IceUtil/Thread.h>

#include "Media/FrameSeries.H"

#include "Image/Image.H"

//#include "Util/sformat.H"
//#include "Transport/FrameInfo.H"

#include <vector>

#ifndef FaceDetectorI_H
#define FaceDetectorI_H

class FaceDetector : public RobotBrainComponent
{
public:

  FaceDetector(OptionManager& mgr,
         const std::string& descrName = "FaceDetector",
         const std::string& tagName = "FaceDetector");

  ~FaceDetector();

  virtual void evolve();

  //! Get a message
  virtual void updateMessage
  (const RobotSimEvents::EventMessagePtr& eMsg,
   const Ice::Current&);

  virtual void registerTopics();

  Image<PixRGB<byte> > findFaces(Image<PixRGB<byte> > ima);


  void start1();

private:
  //void writeToLogFile(std::string line);

  std::string itsLogFilename;
  IceUtil::Mutex its_logFilename_mutex;        //!< locking log filename

  //std::vector<FaceData> itsData;
  nub::soft_ref<OutputFrameSeries> itsOfs;
  Timer itsTimer;

  //--- edited
  Image<PixRGB<byte> > itsCurrImg;
  Image<PixRGB<byte> > itsProcImg;
  //Image<float> itsNormalizedProcImg;

  //Image<double> itsFftFeatures;

  IceUtil::Mutex its_Curr_Img_mutex;        //!< locking log filename
  IceUtil::Mutex its_Curr_Res_mutex;        //!< locking face result

  int itsCurrImgID;
  int itsPrevProcImgID;
  int itsRemoteMode;
  int currRequestID;
  //--- end of edit

  std::string itsStartTime;
  //std::string itsLogFoldername;
  int itsCurrMessageID;

  // results
  std::vector<Rectangle> itsCurrentFacesFound;


};

#endif


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
