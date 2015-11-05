/*!@file PrimarySomatosensoryCortex.C drive the actual robot */

//////////////////////////////////////////////////////////////////// //
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
// Primary maintainer for this file: Lior Elazary <lelazary@yahoo.com>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/RobotBrain/LateralGeniculateNucleusI.C $
// $Id: LateralGeniculateNucleusI.C 10794 2009-02-08 06:21:09Z itti $
//

#include "Component/ModelManager.H"
#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Image/Image.H"
#include "Image/DrawOps.H"
#include "GUI/ImageDisplayStream.H"
#include "GUI/DebugWin.H"
#include "Robots/RobotBrain/LateralGeniculateNucleusI.H"
#include "Robots/RobotBrain/RobotCommon.H"

#include "Ice/IceImageUtils.H"

// ######################################################################
LateralGeniculateNucleusI::LateralGeniculateNucleusI(OptionManager& mgr,
    const std::string& descrName, const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName)
{
  itsOfs = nub::ref<OutputFrameSeries>(new OutputFrameSeries(mgr));
  addSubComponent(itsOfs);

  itsWindowSize = Dims(150,150);
}

// ######################################################################
void LateralGeniculateNucleusI::init(Ice::CommunicatorPtr ic, Ice::ObjectAdapterPtr adapter)
{
  Ice::ObjectPtr pscPtr = this;
  itsObjectPrx = adapter->add(pscPtr,
      ic->stringToIdentity("LateralGeniculateNucleus"));

  itsPublisher = RobotSimEvents::EventsPrx::uncheckedCast(
        SimEventsUtils::getPublisher(ic, "AttendedRegionMessageTopic")
        );

  Ice::ObjectPrx base = ic->stringToProxy("IRobotService:default -p 10000 -h " ROBOT_IP);
  itsRobot = Robots::IRobotPrx::checkedCast(base);

  if(!itsRobot) LFATAL("Invalid Robot Proxy");

  IceUtil::ThreadPtr thread = this;
  thread->start();

  usleep(10000);
}

// ######################################################################
LateralGeniculateNucleusI::~LateralGeniculateNucleusI()
{
  //SimEventsUtils::unsubscribeSimEvents(itsTopicsSubscriptions, itsObjectPrx);
}

// ######################################################################
Point2D<int> LateralGeniculateNucleusI::getMouseClick(nub::soft_ref<OutputFrameSeries> &ofs)
{
  const nub::soft_ref<ImageDisplayStream> ids =
    ofs->findFrameDestType<ImageDisplayStream>();

  const rutz::shared_ptr<XWinManaged> uiwin =
    ids.is_valid()
    ? ids->getWindow("RetinaImg")
    : rutz::shared_ptr<XWinManaged>();

  if (uiwin.is_valid())
    return uiwin->getLastMouseClick();
  else
    return Point2D<int>(-1,-1);
}

// ######################################################################
int LateralGeniculateNucleusI::getKey(nub::soft_ref<OutputFrameSeries> &ofs)
{
  const nub::soft_ref<ImageDisplayStream> ids =
    ofs->findFrameDestType<ImageDisplayStream>();

  const rutz::shared_ptr<XWinManaged> uiwin =
    ids.is_valid()
    ? ids->getWindow("RetinaImg")
    : rutz::shared_ptr<XWinManaged>();
  return uiwin->getLastKeyPress();
}


// ######################################################################
void LateralGeniculateNucleusI::run() {
  sleep(1); //needed to wait for ofs to initalize

  while(1) {
    ImageIceMod::ImageIce imgIce = itsRobot->getImageSensor(0, false);
    Image<PixRGB<byte> > retinaImg;

    if (imgIce.pixSize == 1)
      retinaImg = toRGB(Ice2Image<byte>(imgIce));
    else
      retinaImg = Ice2Image<PixRGB<byte> >(imgIce);

    if (!retinaImg.initialized())
      continue;

    Image<PixRGB<byte> > tmpImg = retinaImg;

    //drawLine(tmpImg,
    //    Point2D<int>(tmpImg.getWidth()/2, 0),
    //    Point2D<int>(tmpImg.getWidth()/2, tmpImg.getHeight()),
    //    PixRGB<byte>(255,0,0));

    itsOfs->writeRGB(tmpImg, "RetinaImg", FrameInfo("RetinaImg", SRC_POS));

    Point2D<int> clickLoc = getMouseClick(itsOfs);

    if (clickLoc.isValid())
    {

      // the location is at the center, move it to the top left
      // Point2D<int> topLeft(clickLoc.i-((itsWindowSize.w()-1)/2), clickLoc.j-((itsWindowSize.h()-1)/2));

      Point2D<int> topLeft = clickLoc;
      printf("Please Click The Bottom Right Of The Object\n");

      Point2D<int> bottomRight = getMouseClick(itsOfs);
      while(!bottomRight.isValid())
      {
        bottomRight = getMouseClick(itsOfs);
        usleep(10000);
      }

      Image<PixRGB<byte> > selectImage = retinaImg;


      if(bottomRight.i-topLeft.i <= 0 || bottomRight.j-topLeft.j <= 0)
      {
        printf("ERROR:Please click top left, then bottom right\n");
        continue;
      }

      Dims selectDims(bottomRight.i-topLeft.i, bottomRight.j-topLeft.j);

      drawRect(selectImage, Rectangle(topLeft, selectDims), PixRGB<byte>(255,0,0));
      itsOfs->writeRGB(selectImage, "RetinaImg", FrameInfo("RetinaImg", SRC_POS));

      Image<PixRGB<byte> > objImg = crop(retinaImg,topLeft, selectDims);
      itsOfs->writeRGB(objImg, "Object", FrameInfo("ObjectImg", SRC_POS));

      printf("Store Selection As Landmark? (y/n): ");
      std::string keepSelection;
      //std::getline(std::cin, keepSelection);
      cin >> keepSelection;

      if(keepSelection == "y" || keepSelection == "Y")
      {
        std::string objName;
        float objWidth;
        float objHeight;

        printf("Enter a label for this object: ");
       // std::getline(std::cin, objName);
        cin >> objName;
        printf("Enter the size of this object in millimeters: ");
        cin >> objWidth;
        cin >> objHeight;
        LINFO("Storing Object: '%s' at %fmm x %fmm\n", objName.c_str(), objWidth, objHeight);

        if (objName != "") //Send the message of the object
        {
          RobotSimEvents::AttendedRegionMessagePtr arMsg =
            new RobotSimEvents::AttendedRegionMessage;
          arMsg->objId = 1;
          arMsg->objWidth = objWidth;
          arMsg->objHeight = objHeight;
          arMsg->name  = objName;
         // arMsg->img   = Image2Ice(objImg);
          arMsg->img   = Image2Ice(retinaImg);

          //Set the attended region
          arMsg->attTopLeft.i = topLeft.i;
          arMsg->attTopLeft.j = topLeft.j;
          arMsg->attWidth     = selectDims.w();
          arMsg->attHeight    = selectDims.h();
          LINFO("SelectDims: %s :: (%d, %d)", convertToString(selectDims).c_str(),
              arMsg->attWidth,arMsg->attHeight);

          itsPublisher->updateMessage(arMsg);
          LDEBUG("Sent object Img");
        }
      }

      //Blow through any accumulated buffered mouse clicks
      while(getMouseClick(itsOfs).isValid());

    }
    else {

      RobotSimEvents::AttendedRegionMessagePtr arMsg =
        new RobotSimEvents::AttendedRegionMessage;
      arMsg->objId = -1;
      arMsg->name.clear();
      arMsg->img = Image2Ice(retinaImg);

      itsPublisher->updateMessage(arMsg);
    }

    usleep(10000);
  }
}

// ######################################################################
void LateralGeniculateNucleusI::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
    const Ice::Current&)
{
}
