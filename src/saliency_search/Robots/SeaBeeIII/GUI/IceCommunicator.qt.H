#ifndef ICECOMMUNICATOR_H
#define ICECOMMUNICATOR_H

#include <QtGui/QtGui>
#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/IceImageUtils.H"
#include <Ice/Ice.h>

#include "Robots/RobotBrain/RobotBrainComponent.H"


//#include "Util/Timer.H"
//#include "GUI/CompassMeter.H"
//#include "GUI/DepthMeter.H"
//#include "GUI/PressureMeter.H"
//#include "GUI/CircleFillMeter.H"

//
//class BeeStemData
//{
//public:
//  float heading;
//  float externalPressure;
//  float internalPressure;
//  int headingPIDOutput;
//  int depthPIDOutput;
//  bool killSwitch;
//};

class IceCommunicator : public QObject, public RobotBrainComponent
{
  Q_OBJECT

public:
  IceCommunicator(OptionManager& mgr,
                const std::string& descrName = "IceCommunicator",
                const std::string& tagName = "IceCommunicator");

  virtual void updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
                             const Ice::Current&);

  virtual void registerTopics();

  virtual void evolve();

signals:
  void NewFwdLeftImg(QImage);
  void NewFwdRightImg(QImage);
  void NewDwnLeftImg(QImage);
  void NewDwnRightImg(QImage);


//  void setOrangeSegEnabled(bool enabled);
//  void setSalientPointsEnabled(bool enabled);

private:
//  void updateFwdImg();
//
//  void updateDwnImg();
//
//  void updateBeeStemData();
//
//  void updateSaliencyImage();
//
//
//  SeaBee3MainDisplayForm *itsGUIForm;
//
//  bool itsGUIRegistered;
//  IceUtil::Mutex itsGUIFormMutex;
//
//  IceUtil::Mutex itsUpdateMutex; // Locks lists for incoming updates
//
//  std::list<Image<PixRGB<byte> > > itsFwdRetinaImages;
//  char itsFwdRetinaImagesSize;
//  char itsFwdRetinaMsgCounter;
//
//  std::list<Image<PixRGB<byte> > > itsDwnRetinaImages;
//  char itsDwnRetinaImagesSize;
//  char itsDwnRetinaMsgCounter;
//
//  std::list<Image<PixRGB<byte> > > itsOrangeSegImages;
//  bool itsOrangeSegEnabled;
//
//  std::list<Image<PixRGB<byte> > > itsRedSegImages;
//  char itsRedSegImagesSize;
//
//  std::list<Point2D<int> > itsSalientPoints;
//  int itsSalientPointsSize;
//  bool itsSalientPointsEnabled;
//
//  char itsVisionMsgCounter;
//
//  std::list<BeeStemData> itsBeeStemData;
//  char itsBeeStemDataSize;
//  char itsBeeStemMsgCounter;
//  CompassMeter itsCompassMeter;
//  DepthMeter itsDepthMeter;
//  PressureMeter itsPressureMeter;
//  CircleFillMeter itsCircleFillMeter;
//
//  Timer itsTimer;
//
//  Image<PixRGB<byte> > itsFwdVisionImage;
//  Image<PixRGB<byte> > itsDwnVisionImage;

};

#endif

