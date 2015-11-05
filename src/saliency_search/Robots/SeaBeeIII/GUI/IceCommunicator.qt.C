#ifndef ICECOMMUNICATOR_C
#define ICECOMMUNICATOR_C

#include "QtUtil/ImageConvert4.H"
#include "Robots/SeaBeeIII/GUI/IceCommunicator.qt.H"
#include "Ice/ImageIce.ice.H"
#include "Ice/IceImageUtils.H"


#define MSG_BUFF_MAX 5

IceCommunicator::IceCommunicator(OptionManager& mgr,
                             const std::string& descrName,
                             const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName)
//  itsGUIRegistered(false),
//  itsFwdRetinaImagesSize(0),
//  itsFwdRetinaMsgCounter(0),
//  itsDwnRetinaImagesSize(0),
//  itsDwnRetinaMsgCounter(0),
//  itsOrangeSegEnabled(false),
//  itsRedSegImagesSize(0),
//  itsSalientPointsSize(0),
//  itsSalientPointsEnabled(false),
//  itsVisionMsgCounter(0),
//  itsBeeStemDataSize(0),
//  itsBeeStemMsgCounter(0),
//  itsCompassMeter(140,130),
//  itsDepthMeter(140,130),
//  itsPressureMeter(100,90),
//  itsCircleFillMeter(60,50,20)
{
//  itsTimer.reset();
//  itsFwdVisionImage = Image<PixRGB<byte> >();
//  itsDwnVisionImage = Image<PixRGB<byte> >();
}

void IceCommunicator::registerTopics()
{
  registerSubscription("BeeStemMessageTopic");
  registerSubscription("XBox360RemoteControlMessageTopic");
  registerSubscription("RetinaMessageTopic");
  registerSubscription("VisionRectangleMessageTopic");
  registerSubscription("SalientPointMessageTopic");
  registerSubscription("MovementControllerMessageTopic");
  registerPublisher("CameraConfigTopic");
  registerPublisher("BeeStemConfigTopic");
  registerPublisher("SeaBeeStateConditionMessageTopic");
}

void IceCommunicator::evolve()
{
  /*
  itsUpdateMutex.lock();
  if(itsGUIRegistered)
    {
      if(itsFwdRetinaImagesSize > 0)
        {
          updateFwdImg();

          if(itsOrangeSegImages.size() > 0 && itsOrangeSegEnabled)
            {
              Image<PixRGB<byte> > oj = itsOrangeSegImages.front();

              itsFwdVisionImage = oj;
              itsOrangeSegImages.pop_front();
            }

          if(itsSalientPoints.size() > 0 && itsSalientPointsEnabled)
            updateSaliencyImage();

          itsGUIForm->setFwdVisionImage(itsFwdVisionImage);

        }
      if(itsDwnRetinaImagesSize > 0)
        {
          updateDwnImg();

          itsGUIForm->setDwnVisionImage(itsDwnVisionImage);
        }

      if(itsBeeStemDataSize > 0)
        {
          updateBeeStemData();
        }
    }


  if(itsTimer.getSecs() >= 1.0)
    {
      itsGUIForm->setFwdRetinaMsgField(itsFwdRetinaMsgCounter);
      itsGUIForm->setDwnRetinaMsgField(itsDwnRetinaMsgCounter);
      itsGUIForm->setBeeStemMsgField(itsBeeStemMsgCounter);
      itsGUIForm->setVisionMsgField(itsVisionMsgCounter);
      Image<PixRGB<byte> > headingAxis = itsCircleFillMeter.render(itsFwdRetinaMsgCounter);
Image<PixRGB<byte> > depthAxis = itsCircleFillMeter.render(itsDwnRetinaMsgCounter);
Image<PixRGB<byte> > strafeAxis = itsCircleFillMeter.render(itsBeeStemMsgCounter);
 itsGUIForm->setAxesImages(headingAxis,depthAxis,strafeAxis);

      itsTimer.reset();
      itsFwdRetinaMsgCounter = 0;
      itsDwnRetinaMsgCounter = 0;
      itsBeeStemMsgCounter = 0;
      itsVisionMsgCounter = 0;
    }
  itsUpdateMutex.unlock();
  */
}

/*
// ######################################################################
void IceCommunicator::updateFwdImg()
{
  Image<PixRGB<byte> > img = itsFwdRetinaImages.front();
  itsGUIForm->setFwdImage(img);

  itsFwdVisionImage = img;

  itsFwdRetinaImages.pop_front();
  itsFwdRetinaImagesSize--;
}

// ######################################################################
void IceCommunicator::updateDwnImg()
{
  Image<PixRGB<byte> > img = itsDwnRetinaImages.front();
  itsGUIForm->setDwnImage(img);
  itsDwnVisionImage = img;

  itsDwnRetinaImages.pop_front();
  itsDwnRetinaImagesSize--;
}

// ######################################################################
void IceCommunicator::updateSaliencyImage()
{
  Point2D<int> pt = itsSalientPoints.front();

  //  drawTraj(itsFwdVisionImage,
  //&(itsSalientPoints[0]),&(itsSalientPoints[2]));

  //  LINFO("Point %d, %d\n",pt.i,pt.j);
  PixRGB<byte> color(0,0,0);
  drawCircle(itsFwdVisionImage, pt, 10, PixRGB<byte>(0,150,0), 1);
  drawCircle(itsFwdVisionImage, pt, 13, PixRGB<byte>(0,100,0), 1);
  drawCircle(itsFwdVisionImage, pt, 16, PixRGB<byte>(0,50,0), 1);
  drawCircle(itsFwdVisionImage, pt, 19, PixRGB<byte>(0,0,0), 1);


  drawDisk(itsFwdVisionImage, pt, 7, PixRGB<byte>(0,0,0));
  drawDisk(itsFwdVisionImage, pt, 4, PixRGB<byte>(0,255,0));

  itsSalientPoints.pop_front();
}

// ######################################################################
void IceCommunicator::updateBeeStemData()
{
  BeeStemData d = itsBeeStemData.front();

  Image<PixRGB<byte> > compassImg = itsCompassMeter.render(d.heading);
  Image<PixRGB<byte> > depthImg = itsDepthMeter.render(d.externalPressure);
  Image<PixRGB<byte> > pressureImg = itsPressureMeter.render(d.internalPressure);

  itsGUIForm->setCompassImage(compassImg);
  itsGUIForm->setDepthImage(depthImg);
  itsGUIForm->setPressureImage(pressureImg);
  itsGUIForm->setBeeStemData(d);

  itsBeeStemData.pop_front();
  itsBeeStemDataSize--;
}

void IceCommunicator::setOrangeSegEnabled(bool enabled)
{
  itsUpdateMutex.lock();
  itsOrangeSegEnabled = enabled;
  itsUpdateMutex.unlock();
}

void IceCommunicator::setSalientPointsEnabled(bool enabled)
{
  itsUpdateMutex.lock();
  itsSalientPointsEnabled = enabled;
  itsUpdateMutex.unlock();
}
*/

// ######################################################################
void IceCommunicator::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
                                  const Ice::Current&)
{

  //Get a retina message
  if(eMsg->ice_isA("::RobotSimEvents::RetinaMessage"))
  {
    RobotSimEvents::RetinaMessagePtr msg = RobotSimEvents::RetinaMessagePtr::dynamicCast(eMsg);
//    LINFO("msg_id: %s",msg->cameraID.c_str());

    if(Ice2Image<PixRGB<byte> >(msg->img).initialized())
    {
      Image<PixRGB<byte> > retinaImage = Ice2Image<PixRGB<byte> >(msg->img);
      QImage qretinaImage = convertToQImage4(retinaImage);

      if(msg->cameraID == "FwdCamera")
      {
        LINFO("Forward Camera Message");
        emit(NewFwdLeftImg(qretinaImage));
      }
      else if(msg->cameraID == "DwnCamera")
      {

      }
    }

  }
}
#endif

