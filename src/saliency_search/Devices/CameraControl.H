/*!@file Devices/CameraControl.H Controls a pan/tilt camera head */

// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/CameraControl.H $
// $Id: CameraControl.H 4751 2005-07-01 16:50:30Z rjpeters $

#ifndef CAMERACONTROL_H_DEFINED
#define CAMERACONTROL_H_DEFINED

#include "Component/ModelComponent.H"
#include "Devices/cameraConf.H"
#include "Devices/ssc.H"

//! This class is an interface to pan, tilt, and otherwise move the camera
class CameraControl : public ModelComponent
{
public:
  //! Initialize a camera control object
  CameraControl(OptionManager& mgr,
                const std::string& descrName = "Camera Controller",
                const std::string& tagName = "CameraControl",
                int setPort = 0, bool setCam = true,
                int pPort = 0, int tPort = 1, int Port = 1);
  //! Destructor
  ~CameraControl();
  //! move the camera to a point X,Y in an image
  float moveCamXYFrame(int X, int Y, int camNumber = 0, bool simluate = false);
  //! center the camera
  float moveCamTPFrame(float theta, float phi, int camNumber = 0);
  //! center the camera
  void centerCamera(int camNumber = 0);
  //! pan the camera a certain amount relative to the current pos in deg.
  void panRelative(float doPan);
  //! tilt the camera a certain amount relative to the current pos in deg.
  void tiltRelative(float doTilt);
  //! pan the camora to an absolute position in deg.
  void panAbsolute(float doPan);
  //! tilt the camera to an absolute position in deg.
  void tiltAbsolute(float doTilt);
  //! set the size of the image for moveCamXYFrame if not 640x480
  void setImageSize(int X, int Y);
  //! tells if calibration is to be used to different cameras
  void useCalibration(bool useIt);
  //! get the current position pan of the camera in deg.
  float getCurrentPan();
  //! get the current postion tilt of the camera in deg.
  float getCurrentTilt();
  //! get the current position pan of the camera in deg.
  float getSimPan();
  //! get the current postion tilt of the camera in deg.
  float getSimTilt();
private:
  nub::soft_ref<SSC> ssc;
  //! config param for camera
  cameraConf camera;
  float currentPan,currentTilt,simPan,simTilt;
  int serialPort, tiltPort, panPort;
  bool calibrate;

};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
