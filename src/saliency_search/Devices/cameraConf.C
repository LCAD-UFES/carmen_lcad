/*!@file Devices/cameraConf.C [put description here] */

// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/cameraConf.C $
// $Id: cameraConf.C 6182 2006-01-31 18:41:41Z rjpeters $

#include "Devices/cameraConf.H"

cameraConf::cameraConf()
{
  readconfig.openFile("camera.conf");
  init();
}
cameraConf::cameraConf(std::string fileName)
{
  readconfig.openFile(fileName.c_str());
  init();
}

void cameraConf::init()
{
  // size of field of view in pixels
  Xpixel = (unsigned int)readconfig.getItemValueF("Xpixel");
  // size of field of view in pixels
  Ypixel = (unsigned int)readconfig.getItemValueF("Ypixel");
  // size of field of view in degrees
  Xfield = readconfig.getItemValueF("Xfield");
  // size of field of view in degrees
  Yfield = readconfig.getItemValueF("Yfield");

        // pixel size in millimeters
        pixelSizeMM = readconfig.getItemValueF("pixelSizeMM");

        // focal length in millimeters
        focalLengthMM = readconfig.getItemValueF("focalLengthMM");

        // pan offset in millimeters
        panOffsetMM = readconfig.getItemValueF("panOffsetMM");
        // tilt offset in millimeters
        tiltOffsetMM = readconfig.getItemValueF("tiltOffsetMM");

  // camera center in degrees
  Xcenter = readconfig.getItemValueF("Xcenter");
  // camera center in degrees
  Ycenter = readconfig.getItemValueF("Ycenter");
  // servo pixels per degree
  servoPixels = readconfig.getItemValueF("servoPixels");
  // default travel speed from 0 to 1
  travelSpeed = readconfig.getItemValueF("travelSpeed");
  // the time it takes to travel 90 degrees of arc (linear) in ms
  travelTime = (int)readconfig.getItemValueF("travelTime");
  // Minimum time for any travel
  minTravelTime = (int)readconfig.getItemValueF("minTravelTime");
  // center camera at start 1 = yes
  startCenterCamera = (unsigned int)readconfig.getItemValueF("startCenterCamera");
  // center camera at finish
  finishCenterCamera = (unsigned int)readconfig.getItemValueF("finishCenterCamera");
  // servo connections on SSC
  SSCXconnector = (unsigned int)readconfig.getItemValueF("SSCXconnector");
  // servo connections on SSC
  SSCYconnector = (unsigned int)readconfig.getItemValueF("SSCYconnector");
  // baud tranfer rate 2400 or 9600 only
  SSCbaud = (unsigned int)readconfig.getItemValueF("SSCbaud");
  // SSC serial port device;
  SSCport = readconfig.getItemValueS("SSCport");
  // X Travel Limit in degrees
  XlimitStart = readconfig.getItemValueF("XlimitStart");
  // X Travel Limit in degrees
  XlimitEnd = readconfig.getItemValueF("XlimitEnd");
  // Y Travel Limit in degrees
  YlimitStart = readconfig.getItemValueF("YlimitStart");
  // Y Travel Limit in degrees
  YlimitEnd = readconfig.getItemValueF("YlimitEnd");
  // manual field adjustment
  fieldAdjustmentX = readconfig.getItemValueF("fieldAdjustmentX");
  // manual field adjustment
  fieldAdjustmentY = readconfig.getItemValueF("fieldAdjustmentY");
  // field size in pixels
  pixelField = readconfig.getItemValueF("pixelField");
  // Pan calibration in degrees
  panCalibrate = readconfig.getItemValueF("panCalibrate");
  // Tilt calibration in degrees
  tiltCalibrate = readconfig.getItemValueF("tiltCalibrate");
  // Tilt calibration in degrees
  minMove = readconfig.getItemValueF("minMove");
  cameraCalibrationPan.resize(4,0);
  cameraCalibrationPan[0] = readconfig.getItemValueF("camera1Pan")-90;
  cameraCalibrationPan[1] = readconfig.getItemValueF("camera2Pan")-90;
  cameraCalibrationPan[2] = readconfig.getItemValueF("camera3Pan")-90;
  cameraCalibrationPan[3] = readconfig.getItemValueF("camera4Pan")-90;
  cameraCalibrationTilt.resize(4,0);
  cameraCalibrationTilt[0] = readconfig.getItemValueF("camera1Tilt")-90;
  cameraCalibrationTilt[1] = readconfig.getItemValueF("camera2Tilt")-90;
  cameraCalibrationTilt[2] = readconfig.getItemValueF("camera3Tilt")-90;
  cameraCalibrationTilt[3] = readconfig.getItemValueF("camera4Tilt")-90;

}

cameraConf::~cameraConf()
{}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
