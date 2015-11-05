/*!@file Robots2/Beobot2/Navigation/GistSal_Navigation/GistSal_Grapher.H Ice Module to Log data    */
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
// Primary maintainer for this file: Chin-Kai Chang <chinkaic@usc.edu>
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Navigation/GistSal_Navigation/GistSal_Grapher.H
// $Id: GistSal_Grapher.H 12962 2010-03-06 02:13:53Z irock $
//

#ifndef GistSal_Grapher_H
#define GistSal_Grapher_H

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"

#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "Util/Timer.H"

#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Image/Image.H"
#include "Image/DrawOps.H"
#include "Image/Point3D.H"
#include "GUI/XWinManaged.H"
#include "GUI/ImageDisplayStream.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/BeobotEvents.ice.H"
#include <IceUtil/Thread.h>

#include <vector>
typedef struct{
                float time,transVel,rotVel,encoderX,encoderY,encoderOri,rcTransVel,rcRotVel;
                int rcMode;
} motorData;
typedef struct{
                float time;
                std::string imagePath ;
} imageData;

typedef struct{
                float time;
                int segNum ;
                float lenTrav;
} locData;
typedef struct{
                float time;
                float lrfMin;
                float lrfMax;
} lrfData;
class GistSal_Grapher : public RobotBrainComponent
{
public:

  GistSal_Grapher(OptionManager& mgr,
         const std::string& descrName = "GistSal_Grapher",
         const std::string& tagName = "GistSal_Grapher");

  ~GistSal_Grapher();

  virtual void evolve();

  //! Get a message
  virtual void updateMessage
  (const RobotSimEvents::EventMessagePtr& eMsg,
   const Ice::Current&);

  virtual void registerTopics();

  bool initLogFile();

  void start1();

private:
  void writeToLogFile(std::string line);
        void searchLogFile();
        void updateGUI();

  std::string itsLogFilename;
  IceUtil::Mutex its_logFilename_mutex;        //!< locking log filename

  //std::vector<LoggerData> itsData;
  //nub::soft_ref<OutputFrameSeries> itsOfs;
  Timer itsTimer;
        Image<PixRGB<byte> > itsDispImage;
        XWinManaged itsXwin;
  std::string itsStartTime;
  std::string itsLogFoldername;
  int itsCurrMessageID;
        std::vector<motorData> itsLogMotorData;
        std::vector<locData> itsLogLocData;
        std::vector<lrfData> itsLogLrfData;
        std::vector<float> itsLogSegData;
};

#endif


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
