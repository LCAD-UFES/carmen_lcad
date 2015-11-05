/*!@file Devices/cameraConf.H A set of camera configuration parameters */

// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/cameraConf.H $
// $Id: cameraConf.H 6003 2005-11-29 17:22:45Z rjpeters $

#ifndef CAMERACONF_H_DEFINED
#define CAMERACONF_H_DEFINED

#include "Util/readConfig.H"

//! This class uses readConfig to extract config parameters for CameraControl
/*! This class reads config parameters from "camera.conf" or another
    specified config file and organizes them for use in CameraControl. */

class cameraConf
{
private:
  readConfig readconfig;
  void init();
public:
  //!default constructor, open config fileName and read
  cameraConf(std::string fileName);
  //!default constructor, open config "camera.conf" and read
  cameraConf();
  //!default destructor
  ~cameraConf();
  //! size of field of view in pixels
  unsigned int Xpixel;
  //! size of field of view in pixels
  unsigned int Ypixel;
  //! size of field of view in degrees
  float Xfield;
  //! size of field of view in degrees
  float Yfield;
        //! pixel size in millimeters
        float pixelSizeMM;
        //! focal length in millimeters
        float focalLengthMM;
        //! pan offset in millimeters
        float panOffsetMM;
        //! tilt offset in millimeters
        float tiltOffsetMM;

  //! camera center in degrees
  float Xcenter;
  //! camera center in degrees
  float Ycenter;
  //! servo pixels per degree
  float servoPixels;
  //! default travel speed from 0 to 1
  float travelSpeed;
  //! the time it takes to complete a 90 degree travel in milliseconds
  int travelTime;
  //! minimum travel time
  int minTravelTime;
  //! center camera at start 1 = yes
  unsigned int startCenterCamera;
  //! center camera at finish
  unsigned int finishCenterCamera;
  //! servo connections on SSC
  unsigned int SSCXconnector;
  //! servo connections on SSC
  unsigned int SSCYconnector;
  //! baud tranfer rate 2400 or 9600 only
  unsigned int SSCbaud;
  //! SSC serial port device;
  std::string SSCport;
  //! X Travel Limit in degrees
  float XlimitStart;
  //! X Travel Limit in degrees
  float XlimitEnd;
  //! Y Travel Limit in degrees
  float YlimitStart;
  //! Y Travel Limit in degrees
  float YlimitEnd;
  //! manual field adjustment
  float fieldAdjustmentX;
  //! manual field adjustment
  float fieldAdjustmentY;
  //! field size in pixels
  float pixelField;
  //! Pan calibration in degrees
  float panCalibrate;
  //! Tilt calibration in degrees
  float tiltCalibrate;
  //! minimum distance to move in seccade
  float minMove;
  std::vector<float> cameraCalibrationPan;
  std::vector<float> cameraCalibrationTilt;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
