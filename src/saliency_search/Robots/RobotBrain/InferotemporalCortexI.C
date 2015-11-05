/*!@file InferotemporalCortexI.C Recognize Objects */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/RobotBrain/InferotemporalCortexI.C $
// $Id: InferotemporalCortexI.C 10794 2009-02-08 06:21:09Z itti $
//

#include "Component/ModelManager.H"
#include "Image/Image.H"
#include "Image/DrawOps.H"
#include "GUI/DebugWin.H"
#include "Robots/RobotBrain/InferotemporalCortexI.H"


#include "Ice/IceImageUtils.H"

// ######################################################################
InferotemporalCortexI::InferotemporalCortexI(OptionManager& mgr,
    const std::string& descrName, const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsObjectMessage(new RobotSimEvents::ObjectMessage),
  itsAttendedRegionMessage(new RobotSimEvents::AttendedRegionMessage),
  itsTrainingRegionMessage(new RobotSimEvents::AttendedRegionMessage),
  itsUseColor(false)

{
  itsObjectMessage->id = -1;
  itsAttendedRegionMessage->objId = -1;
  itsAttendedRegionMessage->name.clear();
  itsAttendedRegionMessage->img.width = -1;
  itsAttendedRegionMessage->img.height = -1;

  itsTrainingRegionMessage->objId = -1;
  itsTrainingRegionMessage->name.clear();
  itsTrainingRegionMessage->img.width = -1;
  itsTrainingRegionMessage->img.height = -1;

  itsOfs = nub::ref<OutputFrameSeries>(new OutputFrameSeries(mgr));
  addSubComponent(itsOfs);

  initVDB();

  itsTrainingMode = false;

  //Parameters
  //Camera
  itsCurrentCameraParam.dims = Dims(320,240);
  float testObjWidth = 42;
  float testObjDistanceFromCam = 37;
  itsCurrentCameraParam.focalLength = (itsCurrentCameraParam.dims.w()*testObjDistanceFromCam)/testObjWidth;
  itsCurrentCameraParam.yaw = -0.5*M_PI/180.0;

  LINFO("Focal length %f",
      itsCurrentCameraParam.focalLength);


  //Match thresh
  itsMatchThresh = 4U * 4U;
}

// ######################################################################
InferotemporalCortexI::~InferotemporalCortexI()
{
  SimEventsUtils::unsubscribeSimEvents(itsTopicsSubscriptions, itsObjectPrx);
}

// ######################################################################
void InferotemporalCortexI::init(Ice::CommunicatorPtr ic, Ice::ObjectAdapterPtr adapter)
{
  Ice::ObjectPtr objPtr = this;
  itsObjectPrx = adapter->add(objPtr,
      ic->stringToIdentity("InferotemporalCortex"));


  IceStorm::TopicPrx topicPrx;

  itsTopicsSubscriptions.push_back(SimEventsUtils::TopicInfo("AttendedRegionMessageTopic", topicPrx));

  SimEventsUtils::initSimEvents(ic, itsObjectPrx, itsTopicsSubscriptions);

  itsEventsPub = RobotSimEvents::EventsPrx::uncheckedCast(
      SimEventsUtils::getPublisher(ic, "LandmarksMessageTopic")
      );

  IceUtil::ThreadPtr thread = this;
  thread->start();

  usleep(10000);
}


bool InferotemporalCortexI::initVDB()
{
  itsUseColor = false;
  itsVDBFile = "objects.vdb";
  LINFO("Loading VDB File: %s", itsVDBFile.c_str());
  itsVDB.loadFrom(itsVDBFile);
  LINFO("VDB File Loaded");

  return true;
}

// ######################################################################
void InferotemporalCortexI::run()
{

  while(1)
  {
    evolve();
    usleep(10000);
  }

}


// ######################################################################
void InferotemporalCortexI::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
    const Ice::Current&)
{

  //Get a retina message
  if(eMsg->ice_isA("::RobotSimEvents::AttendedRegionMessage"))
  {

    //Dont update any more images if we are training
    if (itsTrainingMode)
      return;
    RobotSimEvents::AttendedRegionMessagePtr arMsg = RobotSimEvents::AttendedRegionMessagePtr::dynamicCast(eMsg);

    {

      if (arMsg->name.size() && !itsTrainingMode)
      {
        itsTrainingMode = true;
        itsTrainingRegionMessage = arMsg;
      } else {
        itsAttendedRegionMessage = arMsg;
      }


    }

  }
}



// ######################################################################
void InferotemporalCortexI::evolve()
{
  std::vector< rutz::shared_ptr<VisualObjectMatch> > matches;

  IceUtil::Mutex::Lock lock(itsARMutex); //lock this call so we can work with the AR message safely

  if (itsAttendedRegionMessage->img.width == -1 ||
      itsAttendedRegionMessage->img.height == -1)
    return;


  Image<PixRGB<byte> > arImg;

  if(itsTrainingMode)
  {
    arImg =  Ice2Image<PixRGB<byte> >(itsTrainingRegionMessage->img);

    if (!arImg.initialized())
      return;


    rutz::shared_ptr<VisualObject>
      imageVO(new VisualObject("Retina Image", "NULL", arImg,
            Point2D<int>(-1,-1),
            std::vector<double>(),
            std::vector< rutz::shared_ptr<Keypoint> >(),
            itsUseColor));


    Image<PixRGB<byte> > keyImg = imageVO->getKeypointImage(1);

    //Loop through all of the keypoints in the image Visual Object, and push all of the ones that
    //fall in the attended region into this keypoints vector
    std::vector< rutz::shared_ptr<Keypoint> > keypoints;
    std::vector< rutz::shared_ptr<Keypoint> > imgKeypoints = imageVO->getKeypoints();

    //Construct a rectangle to represent the selected region from LGN
    Dims attDims(
        itsTrainingRegionMessage->attWidth,
        itsTrainingRegionMessage->attHeight
        );

    Point2D<int> attTopLeft(
        itsTrainingRegionMessage->attTopLeft.i,
        itsTrainingRegionMessage->attTopLeft.j
        );

    LINFO("Dims: %s TL: %s", convertToString(attDims).c_str(), convertToString(attTopLeft).c_str());

    Rectangle attRegion(attTopLeft, attDims);
    drawRect(keyImg, attRegion, PixRGB<byte>(255,0,255), 2);

    keypoints.clear();

    for(uint i=0; i<imgKeypoints.size(); i++)
    {
      Point2D<int> keyPt(imgKeypoints.at(i)->getX(), imgKeypoints.at(i)->getY());

      if(attRegion.contains(keyPt))
      {
        drawCircle(keyImg, keyPt, 2, PixRGB<byte>(0,0,255));
        rutz::shared_ptr<Keypoint> k(
            new Keypoint(
              imgKeypoints.at(i)->getOriFV(),
              imgKeypoints.at(i)->getX() - attTopLeft.i,
              imgKeypoints.at(i)->getY() - attTopLeft.j,
              imgKeypoints.at(i)->getS(),
              imgKeypoints.at(i)->getO(),
              imgKeypoints.at(i)->getM()
              )
            );
        keypoints.push_back(k);
      }
    }

    rutz::shared_ptr<VisualObject>
      vo(new VisualObject("Retina Image",
            "NULL",
            crop(arImg, attTopLeft, attDims),
            Point2D<int>(-1,-1),
            std::vector<double>(),
            keypoints,
            itsUseColor,
            false
            )
        );

    //Object size in mm
    Dims objSize(
       (int)((float)itsTrainingRegionMessage->objWidth  * itsCurrentCameraParam.focalLength / 1000),
       (int)((float)itsTrainingRegionMessage->objHeight * itsCurrentCameraParam.focalLength / 1000));


    learnObject(vo, itsVDB.numObjects(), itsTrainingRegionMessage->name, objSize);
    LINFO("Learned Object:Id = %d, Name=%s Size=%ix%i",
        itsVDB.numObjects(),
        itsTrainingRegionMessage->name.c_str(),
        objSize.w(), objSize.h());


    itsOfs->writeRGB(keyImg, "VObject", FrameInfo("VObject", SRC_POS));

    itsTrainingMode = false;
  }
  else
  {
    arImg = Ice2Image<PixRGB<byte> >(itsAttendedRegionMessage->img);

    if (!arImg.initialized())
      return;

    rutz::shared_ptr<VisualObject>
      vo(new VisualObject("Retina Image", "NULL", arImg,
            Point2D<int>(-1,-1),
            std::vector<double>(),
            std::vector< rutz::shared_ptr<Keypoint> >(),
            itsUseColor));

    Image<PixRGB<byte> > keyImg = vo->getKeypointImage(1);

    RobotSimEvents::LandmarksMessagePtr landmarksMessage = new RobotSimEvents::LandmarksMessage;

    //Remember: Ref is the retina image, and Test is the matched database object

    //Search for landmarks in the scene
    itsVDB.getObjectMatches(vo,matches);

    for(uint i=0; i<matches.size(); i++)
    {
      matches.at(i)->prune();

      if(matches.at(i)->checkSIFTaffine(3.14/4.0, 30.0, 1.0)) {
        std::string objectName = matches.at(i)->getVoTest()->getName();

        Point2D<int> tl, tr, br, bl;

        //Compute the outline of the matched object
        matches.at(i)->getTransfTestOutline(tl,tr,br,bl);

        //Draw the outline of the recognized object for debugging
        drawLine(keyImg, tl, tr, PixRGB<byte>(0,0,255));
        drawLine(keyImg, tr, br, PixRGB<byte>(0,0,255));
        drawLine(keyImg, br, bl, PixRGB<byte>(0,0,255));
        drawLine(keyImg, bl, tl, PixRGB<byte>(0,0,255));

        //Compute the bearing to the object
        int landmarkX = (tl.i + tr.i + br.i + bl.i)/4;
        float bearing = atan( ((itsCurrentCameraParam.dims.w()/2) - landmarkX) / itsCurrentCameraParam.focalLength );
        bearing += itsCurrentCameraParam.yaw;

        //Compute the range of the object
        float theta, sx, sy, str;
        matches.at(i)->getSIFTaffine().decompose(theta, sx, sy, str);
        float range = (sx + sy)/2.0;



        //Construct the landmarkInfo to send out
        RobotSimEvents::LandmarkInfo currLandmarkInfo;
        currLandmarkInfo.id      = -1;
        currLandmarkInfo.name      = objectName;
        currLandmarkInfo.prob    = -1;
        currLandmarkInfo.range   = range;
        currLandmarkInfo.bearing = bearing;

        //Push the landmark onto the list of landmarks to report
        landmarksMessage->landmarks.push_back(currLandmarkInfo);

        writeText(keyImg, tl,
            objectName.c_str(),
            PixRGB<byte>(0,0,0),
            PixRGB<byte>(255,255,255),
            SimpleFont::FIXED(6));
      //  writeText(keyImg, Point2D<int>(tl.i, tl.j+10),
      //      ("r:"+toStr<float>(range)).c_str(),
      //      PixRGB<byte>(0,0,0),
      //      PixRGB<byte>(255,255,255),
      //      SimpleFont::FIXED(6));
      //  writeText(keyImg, Point2D<int>(tl.i, tl.j+20),
      //      ("b:"+toStr<float>(bearing)).c_str(),
      //      PixRGB<byte>(0,0,0),
      //      PixRGB<byte>(255,255,255),
      //      SimpleFont::FIXED(6));
      }
    }
    itsEventsPub->updateMessage(landmarksMessage);

    writeText(keyImg, Point2D<int>(0,0),
        (toStr<int>(matches.size()) + " matches").c_str(),
        PixRGB<byte>(0,0,0),
        PixRGB<byte>(255,255,255),
        SimpleFont::FIXED(6));


    itsOfs->writeRGB(keyImg, "VObject", FrameInfo("VObject", SRC_POS));

  }



  //   rutz::shared_ptr<VisualObjectMatch>
  //     match(new VisualObjectMatch(vo, itsPrevVObj, VOMA_SIMPLE, 5U));

  //   Image<PixRGB<byte> > matchImg = match->getMatchImage(1);
  //   itsOfs->writeRGB(matchImg, "MatchImg", FrameInfo("MatchImg", SRC_POS));

  // RobotSimEvents::LandmarksMessagePtr landmarksMessage = new RobotSimEvents::LandmarksMessage;

  // //Treat each keypoint as a landmark
  // uint nkp = vo->numKeypoints();
  // LDEBUG("Found %i keypoints\n", nkp);
  // for(uint kpIdx=0; kpIdx<nkp; kpIdx++)
  // {
  //   rutz::shared_ptr<Keypoint> keyPoint = vo->getKeypoint(kpIdx);

  //   RobotSimEvents::LandmarkInfo landmarkInfo = getLandmarkInfo(keyPoint);

  //   //Show the found landmark as green point
  //   if (landmarkInfo.prob != -1)
  //     drawCircle(keyImg,
  //         Point2D<int>((int)keyPoint->getX(), (int)keyPoint->getY()), 3, PixRGB<byte>(0,255,0));
  //   landmarksMessage->landmarks.push_back(landmarkInfo);
  // }

  // itsOfs->writeRGB(keyImg, "VObject", FrameInfo("VObject", SRC_POS));

  // LDEBUG("Send %i landmarks", (int)landmarksMessage->landmarks.size());
  // itsEventsPub->updateMessage(landmarksMessage);
}



RobotSimEvents::LandmarkInfo InferotemporalCortexI::getLandmarkInfo(rutz::shared_ptr<Keypoint> keyPoint)
{
  float x = keyPoint->getX();
  //float y = keyPoint->getY();
  float bearing = atan( ((itsCurrentCameraParam.dims.w()/2) - x) / itsCurrentCameraParam.focalLength );

  //TODO use ICE smart pointers
  RobotSimEvents::LandmarkInfo landmarkInfo;
  float prob;
  landmarkInfo.id = findKeypointID(keyPoint, prob);
  landmarkInfo.prob = prob;
  landmarkInfo.range = -1;
  landmarkInfo.bearing = bearing;

  //LINFO("Key %i, prob %f", landmarkInfo.id, landmarkInfo.prob);
  return landmarkInfo;

}

int InferotemporalCortexI::findKeypointID(rutz::shared_ptr<Keypoint> keyPoint, float &prob)
{

  //If we have an empty database then init
  if (itsKeypointsDB.size() == 0)
  {
    itsKeypointsDB.push_back(keyPoint);
    return itsKeypointsDB.size()-1;
  }

  //Search for the keypoint in the database
  std::vector<rutz::shared_ptr<Keypoint> >::const_iterator
    keyDBIter = itsKeypointsDB.begin(),
              bestMatch = itsKeypointsDB.begin(),
              keyDBStop = itsKeypointsDB.end();


  int distsq1 = (*keyDBIter)->maxDistSquared();
  int distsq2 = (*keyDBIter)->maxDistSquared();

  int id = 0;
  int bestID = 0;
  while(keyDBIter != keyDBStop)
  {
    const int distsq = (*keyDBIter)->distSquared(keyPoint);

    // is this better than our best one?
    if (distsq < distsq1)
    {
      distsq2 = distsq1; // old best becomes second best
      distsq1 = distsq;  // we got a new best
      bestMatch = keyDBIter;     // remember the best keypoint
      bestID = id;
    }
    else if (distsq < distsq2)  // maybe between best and second best?
      distsq2 = distsq;

    ++keyDBIter;
    ++id;
  }


  // Check that best distance less than thresh of second best distance:
  //if (100U * distsq1 < itsMatchThresh * distsq2)
  if ( distsq1 < 100000 )
  {
    prob = distsq1;
    return (int)(bestMatch - itsKeypointsDB.begin());
  } else {
    //Add the keypoint to the database
    itsKeypointsDB.push_back(keyPoint);
    prob = -1;
    return itsKeypointsDB.size()-1;
  }

  return -1;


}


void InferotemporalCortexI::learnObject(rutz::shared_ptr<VisualObject> vo, const int objId, const std::string name, const Dims objSize)
{
  vo->setName(name);
  vo->setImageFname("NULL");
  vo->setObjectSize(objSize);
  itsVDB.addObject(vo, false); //allow multiple object names
  itsVDB.saveTo(itsVDBFile);
}


void InferotemporalCortexI::findObjects(const rutz::shared_ptr<VisualObject> vo)
{
  std::vector< rutz::shared_ptr<VisualObjectMatch> > matches;

  Image<PixRGB<byte> > keyImg = vo->getKeypointImage(1);
  //Image<PixRGB<byte> > keyImg = vo->getImage();

  const uint nmatches = itsVDB.getObjectMatches(vo, matches, VOMA_SIMPLE,
      100U, //max objs to return
      0.5F, //keypoint distance score default 0.5F
      0.5F, //affine distance score default 0.5F
      1.0F, //minscore  default 1.0F
      3U, //min # of keypoint match
      100U, //keypoint selection thershold
      false //sort by preattentive
      );

  float score = 0;
  float avgScore = 0, affineAvgDist = 0;
  int nkeyp = 0;
  int objId = -1;
  if (nmatches > 0)
  {
    rutz::shared_ptr<VisualObject> obj; //so we will have a ref to the last matches obj
    rutz::shared_ptr<VisualObjectMatch> vom;
    for (unsigned int i = 0; i < 1; ++i)
    {
      vom = matches[i];
      obj = vom->getVoTest();
      score = vom->getScore();
      nkeyp = vom->size();
      avgScore = vom->getKeypointAvgDist();
      affineAvgDist = vom->getAffineAvgDist();

      objId = atoi(obj->getName().c_str()+3);


      //calculate the actual distance (location of keypoints) between
      //keypoints. If the same patch was found, then the distance should
      //be close to 0
      double dist = 0;
      for (int keyp=0; keyp<nkeyp; keyp++)
      {
        const KeypointMatch kpm = vom->getKeypointMatch(keyp);

        float refX = kpm.refkp->getX();
        float refY = kpm.refkp->getY();

        LDEBUG("Point %f,%f",
            refX, refY);
        drawCircle(keyImg, Point2D<int>((int)refX, (int)refY), 3, PixRGB<byte>(0,255,0));

        float tstX = kpm.tstkp->getX();
        float tstY = kpm.tstkp->getY();
        dist += (refX-tstX) * (refX-tstX);
        dist += (refY-tstY) * (refY-tstY);
      }

      //Send ObjectInfo
      LDEBUG("### Object match with '%s' score=%f ID:%i Dist:%f",
          obj->getName().c_str(), vom->getScore(), objId, dist);

      Point2D<int> tl, tr, br, bl;
      vom->getTransfTestOutline(tl, tr, br, bl);

      itsObjectMessage->id = objId;
      itsObjectMessage->name = obj->getName();
      itsObjectMessage->score = vom->getScore();
      itsObjectMessage->nKeyp = nkeyp;
      itsObjectMessage->dist = dist;
      itsObjectMessage->tl.i = tl.i; itsObjectMessage->tl.j = tl.j;
      itsObjectMessage->tr.i = tr.i; itsObjectMessage->tr.j = tr.j;
      itsObjectMessage->br.i = br.i; itsObjectMessage->br.j = br.j;
      itsObjectMessage->bl.i = bl.i; itsObjectMessage->bl.j = bl.j;

      itsEventsPub->updateMessage(itsObjectMessage);

      //Draw the outline of the image
      drawLine(keyImg, tl, tr, PixRGB<byte>(0, 255, 0));
      drawLine(keyImg, tr, br, PixRGB<byte>(0, 255, 0));
      drawLine(keyImg, br, bl, PixRGB<byte>(0, 255, 0));
      drawLine(keyImg, bl, tl, PixRGB<byte>(0, 255, 0));

      //   printf("%i:%s %i %f %i %f %f %f\n", objNum, obj->getName().c_str(),
      //       nmatches, score, nkeyp, avgScore, affineAvgDist, sqrt(dist));

    }
  }
  itsOfs->writeRGB(keyImg, "VObject", FrameInfo("VObject", SRC_POS));
}



