/*!@file Devices/CameraControl.C Controls a pan/tilt camera head */

// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/CameraControl.C $
// $Id: CameraControl.C 6003 2005-11-29 17:22:45Z rjpeters $

#include "Devices/CameraControl.H"

#include "Devices/ssc.H"
#include "Util/Assert.H"
#include <math.h>
#include <signal.h>
#include <unistd.h>

CameraControl::CameraControl(OptionManager& mgr,
                             const std::string& descrName,
                             const std::string& tagName,
                             int setPort, bool setCam,
                             int pPort, int tPort, int Port) :
  ModelComponent(mgr, descrName, tagName),
  ssc(new SSC(mgr))
{
  addSubComponent(ssc);

  panPort = pPort; tiltPort = tPort; serialPort = Port;
  if(setCam)
  {
    centerCamera();
  }
  currentPan = camera.Xcenter;
  currentTilt = camera.Ycenter;
  calibrate = false;
}

/*============================================================*/

CameraControl::~CameraControl()
{ }

/*============================================================*/

float CameraControl::moveCamXYFrame(int X, int Y, int camNumber, bool simulate)
{
  ASSERT((X >= 0) && ((unsigned)X < camera.Xpixel));
  ASSERT((Y >= 0) && ((unsigned)Y < camera.Ypixel));

  //LINFO("ATTEMPTING TO MOVE");
  int gotoCY = abs(Y - (int)camera.Ypixel);

  float panConv = ((float)camera.Xfield/(float)camera.Xpixel);
  float tiltConv = ((float)camera.Yfield/(float)camera.Ypixel);

  float panOff = ((float)camera.Xpixel*.5)-X;
  float tiltOff = ((float)camera.Ypixel*.5)-gotoCY;

  float travelPan = ((panOff*panConv)*camera.fieldAdjustmentX);
  float travelTilt = ((tiltOff*tiltConv)*camera.fieldAdjustmentY);
  if((fabs(travelPan) > camera.minMove) || (fabs(travelTilt) > camera.minMove))
  {
    panConv = currentPan+travelPan;
    tiltConv = currentTilt+travelTilt;

    float travelPanEst = (camera.travelTime/90)*fabs(travelPan)
      + camera.minTravelTime;
    float travelTiltEst = (camera.travelTime/90)*fabs(travelTilt)
      + camera.minTravelTime;

    if(panConv < camera.XlimitStart)
    {
      panConv = camera.XlimitStart;
      return -1;
    }
    if(panConv > camera.XlimitEnd)
    {
      panConv = camera.XlimitEnd;
      return -2;
    }
    if(tiltConv < camera.YlimitStart)
    {
      tiltConv = camera.YlimitStart;
      return -3;
    }
    if(tiltConv > camera.YlimitEnd)
    {
      tiltConv = camera.YlimitEnd;
      return -4;
    }

    if(simulate == false)
    {
      //LINFO("MOVING XY TO %f %f",panConv,tiltConv);
      if(calibrate == true)
      {
        ssc->moveRawHack(panPort,
                         (int)((panConv
                                +camera.cameraCalibrationPan[camNumber])
                               *camera.servoPixels),serialPort);
        ssc->moveRawHack(tiltPort,
                         (int)((tiltConv
                                +camera.cameraCalibrationTilt[camNumber])
                               *camera.servoPixels),serialPort);
      }
      else
      {
        ssc->moveRawHack(panPort,
                         (int)(panConv*camera.servoPixels),serialPort);
        ssc->moveRawHack(tiltPort,
                         (int)(tiltConv*camera.servoPixels),serialPort);
      }
      currentPan = panConv;
      currentTilt = tiltConv;
    }
    simPan = panConv;
    simTilt = tiltConv;


    if(travelPanEst > travelTiltEst)
      return travelPanEst;
    else
      return travelTiltEst;
  }
  else
  {
    return -5;
  }
}

/*============================================================*/

float CameraControl::moveCamTPFrame(float theta, float phi, int camNumber)
{
  //LINFO("ATTEMPTING TO LOT MOVE");
  float panConv;
  float tiltConv;

  panConv = currentPan-theta;
  tiltConv = currentTilt-phi;

  float travelPanEst = (camera.travelTime/90)*fabs(panConv);
   // + camera.minTravelTime;
  float travelTiltEst = (camera.travelTime/90)*fabs(tiltConv);
    //+ camera.minTravelTime;

  //LINFO("MOVING AMOUNT %f, %f",  panConv,  tiltConv);
  if(theta < camera.XlimitStart)
  {
    return -1;
  }
  if(theta > camera.XlimitEnd)
  {
    return -2;
  }
  if(phi < camera.YlimitStart)
  {
    return -3;
  }
  if(phi > camera.YlimitEnd)
  {
    return -4;
  }

  //LINFO("MOVING TP TO %f %f",theta,phi);
  if(calibrate == true)
  {
    ssc->moveRawHack(panPort,
                     (int)((theta+camera.cameraCalibrationPan[camNumber])
                           *camera.servoPixels),serialPort);
    ssc->moveRawHack(tiltPort,
                     (int)((phi+camera.cameraCalibrationTilt[camNumber])
                           *camera.servoPixels),serialPort);
  }
  else
  {
    ssc->moveRawHack(panPort,
                     (int)(theta*camera.servoPixels),serialPort);
    ssc->moveRawHack(tiltPort,
                     (int)(phi*camera.servoPixels),serialPort);
  }
  currentPan = theta;
  currentTilt = phi;

  if(travelPanEst > travelTiltEst)
    return travelPanEst;
  else
    return travelTiltEst;

}

/*============================================================*/
void CameraControl::centerCamera(int camNumber)
{
  if(calibrate == true)
  {
    ssc->moveRawHack(panPort,
                     (int)((camera.Xcenter
                            +camera.cameraCalibrationPan[camNumber])
                           *camera.servoPixels),serialPort);
    ssc->moveRawHack(tiltPort,
                     (int)((camera.Ycenter
                            +camera.cameraCalibrationTilt[camNumber])
                           *camera.servoPixels),serialPort);
  }
  else
  {
    ssc->moveRawHack(panPort,
                     (int)(camera.Xcenter*camera.servoPixels),serialPort);
    ssc->moveRawHack(tiltPort,
                     (int)(camera.Ycenter*camera.servoPixels),serialPort);
  }
}

/*============================================================*/

void CameraControl::panRelative(float doPan)
{
  float calcDoPan = getCurrentPan() + doPan;
  ASSERT((calcDoPan > camera.XlimitStart) && (calcDoPan > camera.XlimitEnd));
  ssc->move(camera.SSCXconnector,calcDoPan);
}

/*============================================================*/

void CameraControl::tiltRelative(float doTilt)
{
  float calcDoTilt = getCurrentTilt() + doTilt;
  ASSERT((calcDoTilt > camera.YlimitStart) && (calcDoTilt > camera.YlimitEnd));
  ssc->move(camera.SSCYconnector,calcDoTilt);
}
/*============================================================*/

void CameraControl::panAbsolute(float doPan)
{
  ASSERT((doPan > camera.XlimitStart) && (doPan > camera.XlimitEnd));
  doPan = doPan - 10;
  ssc->move(camera.SSCXconnector,doPan);
}

/*============================================================*/

void CameraControl::tiltAbsolute(float doTilt)
{
  ASSERT((doTilt > camera.YlimitStart) && (doTilt > camera.YlimitEnd));
  doTilt = doTilt - 10;
  ssc->move(camera.SSCYconnector,doTilt);
}

/*============================================================*/

void CameraControl::setImageSize(int X, int Y)
{
  camera.Xpixel = X;
  camera.Ypixel = Y;
}

/*============================================================*/

void CameraControl::useCalibration(bool useIt)
{
  calibrate = useIt;
}

/*============================================================*/

float CameraControl::getCurrentPan()
{
  //return ((90-camera.Xcenter)+(ssc->getPosition(camera.SSCXconnector)/248.0F)*180.0F);
  return currentPan;
}

/*============================================================*/

float CameraControl::getCurrentTilt()
{
  //return ((90-camera.Ycenter)+(ssc->getPosition(camera.SSCYconnector)/248.0F)*180.0F);
  return currentTilt;
}

/*============================================================*/

float CameraControl::getSimPan()
{
  return simPan;
}

/*============================================================*/

float CameraControl::getSimTilt()
{
  return simTilt;
}
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
