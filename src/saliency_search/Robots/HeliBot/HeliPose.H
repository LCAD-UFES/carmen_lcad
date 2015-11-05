/*!@file Devices/HeliPose.H read HeliPose data  */

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
// Primary maintainer for this file: Lior Elazary
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/HeliBot/HeliPose.H $
// $Id: HeliPose.H 13901 2010-09-09 15:12:26Z lior $
//

#ifndef HeliPose_H_DEFINED
#define HeliPose_H_DEFINED

#include "Image/OpenCVUtil.H"
#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Util/Types.H"
#include "Image/Point3D.H"
#include "Image/Point2D.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Devices/IMU_SFE_Atomic.H"
#include <pthread.h>
#include <unistd.h>

class HeliPose : public ModelComponent {
public:

  struct Pose {
    Point3D<float> translation;
    Point3D<float> velocity;
    Point3D<float> rotation;
    float roll;
    float pitch;
    float yaw;
    float accelX;
    float accelY;
    float accelZ;
    bool valid;
  };

  //! Constructor
  HeliPose(OptionManager& mgr,
      nub::ref<InputFrameSeries>& ifs,
      nub::ref<OutputFrameSeries>& ofs,
      const std::string& descrName = "HeliPose",
      const std::string& tagName = "HeliPose");

  //! Destructor
  ~HeliPose();

  //! This method should not be called (implements thread code)
  void run_cameraPose(); //get the pose from the camera

  void run_imuPose(); //get the pose from the imu

  //! init the thread and connect to the HeliPose
  void init();

  //! Get sensor data
  Pose getPose();

  //!Get the extrinsic pose
  std::vector<Point2D<int> > getExtrinsic(Image<byte>& img);

  //!Get the extrinsic pose
  std::vector<Point2D<int> > getExtrinsic(Image<PixRGB<byte> >& img);

  //!Show a grid on the image based on the pose
  void displayExtrinsic(Image<byte>& img);

  Image<PixRGB<byte> > getImg();

  //! Calculate the IMu mean an std as rest
  void getIMUBias();


protected:
  //! get started (see ModelComponent.H)
  void start1();

  //! get started (see ModelComponent.H)
  void start2();

  //! get stopped (see ModelComponent.H)
  void stop1();

private:
  nub::soft_ref<InputFrameSeries> itsIfs;
  nub::soft_ref<OutputFrameSeries> itsOfs;
  nub::soft_ref<IMU_SFE_Atomic> itsIMU;
  pthread_t runner;
  pthread_mutex_t itsPoseLock;
  pthread_mutex_t itsImgLock;
  mutable bool running;
  mutable bool itsDebug;
  Image<PixRGB<byte> > itsCurrentImg;
  Pose itsCurrentPose;
  Point3D<float> itsRotationScale;
  Point3D<float> itsVelocityScale;
  Point3D<float> itsRotationBias;
  Point3D<float> itsVelocityBias;
  Point3D<float> itsRotationSigma;
  Point3D<float> itsVelocitySigma;


#ifdef HAVE_OPENCV
  CvMemStorage* itsStorage;
  CvMat* itsIntrinsicMatrix;
  CvMat* itsDistortionCoeffs;
  CvMat* itsCameraRotation;
  CvMat* itsCameraTranslation;
#endif


};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif
