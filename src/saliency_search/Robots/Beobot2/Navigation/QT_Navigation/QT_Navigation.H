/*!@file Robots2/Beobot2/Navigation/QT_Navigation/QT_Navigation.H Ice Module to Log data    */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Navigation/QT_Navigation/QT_Navigation.H
// $Id: QT_Navigation.H 12962 2010-03-06 02:13:53Z irock $
//

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"

#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "Util/Timer.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include <IceUtil/Thread.h>

#include "Robots/Beobot2/BeoCommon.H"

#include "Media/FrameSeries.H"

#include "Image/Image.H"
#include "Image/Point3D.H"
#include "GUI/SimpleMeter.H"

#include "Neuro/GistEstimatorFFT.H"
#include "Gist/FFN.H"

#include <vector>
#include "SIFT/Keypoint.H"
#include "SIFT/VisualObject.H"
#include "SIFT/VisualObjectDB.H"
#include "SIFT/VisualObjectMatch.H"
#include "Transport/FrameInfo.H"

#ifndef QT_NAVIGATIONI_H
#define QT_NAVIGATIONI_H

typedef struct _RegionInformation
{
  uint start;
  uint end;

  float minDistance;
  uint minIndex;
}
RegionInformation;

class QT_Navigation : public RobotBrainComponent
{
public:

  QT_Navigation(OptionManager& mgr,
                const std::string& descrName = "QT_Navigation",
                const std::string& tagName = "QT_Navigation");

  ~QT_Navigation();

  virtual void evolve();

  //! Get a message
  virtual void updateMessage
  (const RobotSimEvents::EventMessagePtr& eMsg,
   const Ice::Current&);

  virtual void registerTopics();

  void start1();

private:
  Beobot2::MotorCommand computeQT_Navigation();

  void navigation();
  void loadFrame();
  bool computeSIFT();//return true when match is valid
  double computeDirection();//return error rate of direction
  double computeDirection2(int teachID);//return error rate of direction
  void recovery();
  int computeEntropy();
  void updateKeyframe(double error);

  void updateMotorPID(double tran, double rot,double error);
  void updateMotor(double tran,double rot);

  void loadDB(const std::string& path);
  void loadReplayDB();
  void saveDB(int start,int end,const std::string& path);
  void drawState();

  void initFFN();
  void updatePosition(double turn);
  Image<PixRGB<byte> > drawInfoImg();

  Image<PixRGB<byte> > itsCurrImg;
  Image<PixRGB<byte> > itsProcImg;
  Image<float> itsNormalizedProcImg;

  Image<double> itsFftFeatures;

  IceUtil::Mutex its_Curr_Img_mutex;        //!< locking log filename
  IceUtil::Mutex its_Curr_Mtr_mutex;        //!< locking log filename

  nub::soft_ref<GistEstimatorFFT> itsFftComputer;

  nub::soft_ref<OutputFrameSeries> itsOfs;

  //! Overall Display Image
  Image<PixRGB<byte> > itsDispImg;

  //! Location Map which maintain the track of data
  Image<PixRGB<byte> > itsMapImg;

  //! Teach and Replay Images for Display
  Image<PixRGB<byte> > itsTeachImg;
  Image<PixRGB<byte> > itsReplayImg;

  //! Two objects for SIFT key points
  rutz::shared_ptr<VisualObject> itsTeachVo;
  rutz::shared_ptr<VisualObject> itsReplayVo;

  //! list of match point from two objects
  rutz::shared_ptr<VisualObjectMatch> itsMatchList;

  //! list of objects as evidence of its exitance
  rutz::shared_ptr<VisualObjectDB> itsVisualObjectDB;
  rutz::shared_ptr<VisualObjectDB> itsVisualObjectDB2;

  rutz::shared_ptr<FeedForwardNetwork> itsFfn;
  Image<double> itsPcaIcaVector;
  Image<double> itsFfnResults;

  Timer itsTimer;
  int itsCurrImgID;
  int itsPrevProcImgID;
  int itsTeachImgID;
  int itsReplayImgID;
  double itsRcTransSpeed;
  double itsRcRotSpeed;
  double itsTransSpeed;
  double itsRotSpeed;
  int itsRemoteMode;

  bool itsTrainMode;
  Image<double> itsNNrotCommand;
  Image<double> itsRCrotCommand;

  double itsDirVector;         // the angle of current turning
  int itsDir;                  // current direction
  double itsMilestoneErr;      // current error
  double itsFalseErrRate;      // mean error / std error
  int itsDirCount;             // current direction count since last turning
  Point3D<double>itsPosition;
  Point3D<double>itsError;
  OModelParam<bool>        itsRealCamera;
  bool itsEstop;
  bool itsReplayDBLoaded;

  double itsIState;
  double itsDState;

  double itsPGain;
  double itsIGain;
  double itsDGain;
};
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
