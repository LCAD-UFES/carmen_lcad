/*!@file Media/SceneGenerator.C generate test scenes */

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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/SceneGenerator.C $
// $Id: SceneGenerator.C 15310 2012-06-01 02:29:24Z itti $
//

#include "Media/SceneGenerator.H"
#include "Raster/Raster.H"
#include "Image/CutPaste.H"
#include "Image/ShapeOps.H"
#include "Image/DrawOps.H"
#include "Util/FileUtil.H" // for isDirectory()

#include <dirent.h>

#include "GUI/XWinManaged.H"
#include "GUI/DebugWin.H"

#include <vector>

// #######################################################################
SceneGenerator::SceneGenerator(SceneType sceneType, int width, int height) :
  itsSceneType(sceneType), itsWidth(width), itsHeight(height),
  itsALOIImages(NULL),
  itsCoilImages(NULL),
  itsTestImages(NULL),
  itsTargetsLoc(0),
  itsObjLocation(),
  itsTargetObject(-1),
  itsObjectSize(80),
  itsTargetColor(PixRGB<byte>(0,0,0)),
  itsLum(-1),
  itsRot(-1),
  itsColor(-1),
  itsNoise(-1),
  itsBackgroundColor(PixRGB<byte>(0,0,0))
{

  initRandomNumbers();

  setSceneType(sceneType);
}

// #######################################################################
SceneGenerator::~SceneGenerator()
{
  if (itsALOIImages != NULL)
    delete itsALOIImages;

  if (itsCoilImages != NULL)
    delete itsCoilImages;

  if (itsTestImages != NULL)
    delete itsTestImages;

}

void SceneGenerator::setSceneType(SceneType sceneId)
{

  itsSceneType = sceneId;

  //Could remove the itsALOIImages before to save on memory
  switch(itsSceneType)
  {
    case SIMPLE_EDGES:
      itsBackgroundColor = PixRGB<byte>(0,0,0);
      break;
    case SIMPLE_SHAPES:
      itsBackgroundColor = PixRGB<byte>(0,0,0);
      itsTargetColor = PixRGB<byte>(100, 100, 100);
      itsColor = PixRGB<byte>(200, 200, 200);
      itsTargetObject = 0;
      itsNoise = 30;
      break;
    case ALOI_OBJECTS:
      if (itsALOIImages == NULL) //load the images
        itsALOIImages = new TestImages("/lab/ilab15/tmp/objectsDB/png/png",
            TestImages::ALOI);
      itsBackgroundColor = PixRGB<byte>(15, 15, 8);
      itsTargetObject = 0;
      break;
    case COIL_OBJECTS:
      if (itsCoilImages == NULL) //load the images
        itsCoilImages = new TestImages("/lab/ilab15/tmp/objectsDB/coil/coil-100",
            TestImages::COIL);
      itsBackgroundColor = PixRGB<byte>(15, 15, 8);
      itsTargetObject = 0;
      break;
    case SATELLITE_KATRINA:
      readDirScenes("/lab/lior/images/croped");
      break;
    case MIT_LABELME:
      if (itsTestImages != NULL) delete itsTestImages; //delete the old images
      readDirScenes("/lab/ilab15/tmp/objectsDB/mit/labelMe");
      break;


  }


}

// #######################################################################
void SceneGenerator::readDirScenes(const char *path)
{

  DIR *dp = opendir(path);
  dirent *dirp;
  while ((dirp = readdir(dp)) != NULL ) {
    switch(itsSceneType)
    {
      case SATELLITE_KATRINA:
        if (dirp->d_name[0] != '.')
          sceneFilenames.push_back(std::string(path) + '/' + std::string(dirp->d_name));
      break;

      case MIT_LABELME:
        //only add directories
        if (dirp->d_name[0] != '.' &&
            isDirectory(dirp))
          sceneFilenames.push_back(std::string(path) + '/' + std::string(dirp->d_name));
        break;
      default:
        break;
    }

  }
  LINFO("%" ZU " files in the directory\n", sceneFilenames.size());

}

// #######################################################################
void SceneGenerator::setTargetObject(int obj)
{
  itsTargetObject = obj;
}

// #######################################################################
Image<PixRGB<byte> > SceneGenerator::getScene(int nobj, int lum, int col, int rot)
{
  switch(itsSceneType)
  {
    case SIMPLE_EDGES:
      break;
    case SIMPLE_SHAPES:
      return generateShapeScene();
      break;
    case ALOI_OBJECTS:
      return generateAloiObjectsScene(nobj, lum, col, rot);
      break;
    case COIL_OBJECTS:
      return generateCoilObjectsScene(nobj);
      break;
    case SATELLITE_KATRINA:
      return getSceneFile();
    case MIT_LABELME:
      return getLabelMeScene();
      break;
  }

  return Image<PixRGB<byte> >(); //return an empty scene

}

// #######################################################################
Image<PixRGB<byte> > SceneGenerator::getSceneFile()
{

  static int index = 0;

  index++;
  index = index % sceneFilenames.size();

  return Raster::ReadRGB(sceneFilenames[index]);
}


// #######################################################################
Image<PixRGB<byte> > SceneGenerator::getLabelMeScene()
{

  static int index = 0;

  index++;
  index = index % sceneFilenames.size();

  if (itsTestImages != NULL) delete itsTestImages;
  itsTestImages = new TestImages(sceneFilenames[index].c_str(),
      TestImages::MIT_LABELME);

  Image<PixRGB<byte> > scene = itsTestImages->getScene(0);


  scene = rescale(scene, itsWidth, itsHeight);
  return scene;
}

// #######################################################################
Image<PixRGB<byte> > SceneGenerator::generateEdgeScene()
{


  return Image<PixRGB<byte> >(); //return an empty scene
}

// #######################################################################
Image<PixRGB<byte> > SceneGenerator::generateShapeScene()
{

  Image<PixRGB<byte> > sceneImg(itsWidth, itsHeight, NO_INIT);
  sceneImg.clear(itsBackgroundColor); //set the background;

  itsTargetsLoc.clear();

  PixRGB<byte> color;

  int idx = 0; int targetObjPos = randomUpToIncluding(8);
  for (int y=itsObjectSize+10; y<sceneImg.getHeight(); y += (itsObjectSize * 2))
    for(int x=itsObjectSize+10; x<sceneImg.getWidth(); x += (itsObjectSize * 2))
    {
      int objType = randomUpToIncluding(0);
      /*int lum = randomUpToIncluding(11) + 1;
      int col = randomUpToIncluding(13) + 1;
      int rot = randomUpToIncluding(10) + 1;*/

      if (idx == targetObjPos) //place the target object
      {
        itsTargetsLoc.push_back(Point2D<int>(x,y));
        objType = itsTargetObject;
        color = itsTargetColor;
        //Add some noise
      } else {
        color = itsColor;
      }

      int noise = randomUpToIncluding(itsNoise*2)-itsNoise;
      color[0] += noise;
      color[1] += noise;
      color[2] += noise;


      switch(objType)
      {
        case 0:
          drawDisk(sceneImg, Point2D<int>(x, y), itsObjectSize/2, color);
          break;
        case 1:
          drawCross(sceneImg, Point2D<int>(x, y), color, itsObjectSize/2);
          break;
        case 2:
          drawRectOR(sceneImg, Rectangle::tlbrI(y-(itsObjectSize/2),
                                         x-(itsObjectSize/2),
                                         y+(itsObjectSize/2),
                                         x+(itsObjectSize/2)),
              color, 1, M_PI/2);
          break;
        case 3:
          drawRectOR(sceneImg, Rectangle::tlbrI(y-(itsObjectSize/2),
                                         x-(itsObjectSize/2),
                                         y+(itsObjectSize/2),
                                         x+(itsObjectSize/2)),
              color);
          break;
        case 4:
          drawPatch(sceneImg, Point2D<int>(x, y), itsObjectSize/2, color);
          break;
      }

      idx++;
    }

  return sceneImg; //return an empty scene
}

// #######################################################################
Image<PixRGB<byte> > SceneGenerator::generateAloiObjectsScene(int nobj, int lum, int col, int rot)
{

  Image<PixRGB<byte> > sceneImg(itsWidth, itsHeight, NO_INIT);
  sceneImg.clear(itsBackgroundColor); //set the background;

  itsTargetsLoc.clear();
  itsObjLocation.clear();

  int idx = 0; int targetObjPos = -1; //randomUpToIncluding(24);
  /*int rot = -2;
  //int lum = -2;
  static int lum = 0;
  lum = (lum + 1)%12;

  static int col = 0;
  col = (col + 1)%24; col = -2;*/

  std::vector<int> objInScene;

  int objType = 0;
  for (int y=itsObjectSize+10; y<sceneImg.getHeight(); y += (itsObjectSize * 2))
    for(int x=itsObjectSize+10; x<sceneImg.getWidth(); x += (itsObjectSize * 2))
    {
       bool exsist = true;

       while(exsist)
       {
         objType = randomUpToIncluding(nobj-1);
         exsist = false;
         for(uint i=0; i<objInScene.size(); i++)
         {
           if (objType == objInScene[i])
           {
             exsist = true;
           }
         }
       }

       objInScene.push_back(objType);

      //objType = (objType+1)%nobj ;

    //  if (itsLum < 0)
     //   lum = randomUpToIncluding(11);

    //  if (itsColor[0] == 0)
    //    col = randomUpToIncluding(23);

     // if(itsRot < 0)
     //   rot = randomUpToIncluding(40);

      Image<PixRGB<byte> > obj;
      if (idx == targetObjPos || objType == itsTargetObject) //place the target object
      {
        itsTargetsLoc.push_back(Point2D<int>(x,y));
        itsObjLocation.push_back(ObjectInfo(Point2D<int>(x,y), itsTargetObject));
        obj = itsALOIImages->getObject(itsTargetObject, lum, col, rot);
      } else { //the distarctors
        itsObjLocation.push_back(ObjectInfo(Point2D<int>(x,y), objType));
        obj = itsALOIImages->getObject(objType, lum, col, rot);
      }

      obj = rescale(obj, itsObjectSize, itsObjectSize);
      inplacePaste(sceneImg, obj, Point2D<int>(x-itsObjectSize/2, y-itsObjectSize/2));
      idx++;
    }

 // SHOWIMG(sceneImg);
 // Raster::WriteRGB(sceneImg, "out.ppm");
  return sceneImg;


}

// #######################################################################
Image<PixRGB<byte> > SceneGenerator::generateCoilObjectsScene(int nobj)
{

  Image<PixRGB<byte> > sceneImg(itsWidth, itsHeight, NO_INIT);
  sceneImg.clear(itsBackgroundColor); //set the background;

  itsTargetsLoc.clear();

  int idx = 0; int targetObjPos = randomUpToIncluding(8);
  int lum = -2, col = -2, rot = -2;
  for (int y=itsObjectSize+10; y<sceneImg.getHeight(); y += (itsObjectSize * 2))
    for(int x=itsObjectSize+10; x<sceneImg.getWidth(); x += (itsObjectSize * 2))
    {
      int objType = randomUpToIncluding(98) + 1;

      if(itsColor[1] == 0)
        rot = randomUpToIncluding(71) + 1;

      Image<PixRGB<byte> > obj;
      if (idx == targetObjPos || objType == itsTargetObject) //place the target object
      {
        itsTargetsLoc.push_back(Point2D<int>(x,y));
        obj = itsCoilImages->getObject(itsTargetObject, lum, col, rot);
      } else { //the distarctors
        obj = itsCoilImages->getObject(objType, lum, col, rot);
      }

      obj = rescale(obj, itsObjectSize, itsObjectSize);
      inplacePaste(sceneImg, obj, Point2D<int>(x-itsObjectSize/2, y-itsObjectSize/2));
      idx++;
    }

  return sceneImg;


}
// #######################################################################
bool SceneGenerator::checkTargetPos(const Point2D<int> &pt, const Dims &searchWin)
{
  int x = pt.i;
  int y = pt.j;
  bool foundTarget = false;
  for(uint i=0; i<itsTargetsLoc.size(); i++){
    if (itsTargetsLoc[i].i >  x - (searchWin.w()/2) &&
        itsTargetsLoc[i].i < x + (searchWin.w()/2) &&
        itsTargetsLoc[i].j > y - (searchWin.h()/2) &&
        itsTargetsLoc[i].j < y + (searchWin.h()/2) ){
      LINFO("Object found at %ix%i T%i at %ix%i",
          x, y, i, itsTargetsLoc[i].i, itsTargetsLoc[i].j);
      foundTarget = true;
      break;
    }
  }

  return foundTarget;

}

// #######################################################################
int SceneGenerator::getObjFromPos(const Point2D<int> &pt, const Dims &searchWin)
{
  int x = pt.i;
  int y = pt.j;
  int  objId = -1;
  for(uint i=0; i<itsObjLocation.size(); i++){
    if (itsObjLocation[i].loc.i >  x - (searchWin.w()/2) &&
        itsObjLocation[i].loc.i < x + (searchWin.w()/2) &&
        itsObjLocation[i].loc.j > y - (searchWin.h()/2) &&
        itsObjLocation[i].loc.j < y + (searchWin.h()/2) ){
      objId = itsObjLocation[i].objId;
      break;
    }
  }

  return objId;

}

// #######################################################################
Point2D<int> SceneGenerator::getObjLocation(uint objId)
{
  ASSERT(objId < itsObjLocation.size());
  return itsObjLocation[objId].loc;

}

// #######################################################################
int SceneGenerator::getObjId(uint objId)
{
  ASSERT(objId < itsObjLocation.size());
  return itsObjLocation[objId].objId;

}

// #######################################################################
uint SceneGenerator::getNumObj()
{
  return itsObjLocation.size();
}

// #######################################################################
const Point2D<int> SceneGenerator::getTargetPos() const
{

  if (itsTargetsLoc.size() > 0)
    return itsTargetsLoc[0];
  else
    return Point2D<int>();
};

// #######################################################################
void SceneGenerator::setObjectSize(int size)
{
  itsObjectSize = size;
}


// #######################################################################
void SceneGenerator::setTargetColor(PixRGB<byte> color)
{
  itsTargetColor = color;
}

// #######################################################################
void SceneGenerator::setLum(int lum)
{
  itsLum = lum;
}

// #######################################################################
void SceneGenerator::setRotation(int rot)
{
  itsRot = rot;
}

// #######################################################################
void SceneGenerator::setColor(PixRGB<byte> color)
{
  itsColor = color;
}

// #######################################################################
void SceneGenerator::setNoise(int level)
{
  itsNoise = level;
}

// #######################################################################
void SceneGenerator::setBackgroundColor(PixRGB<byte> color)
{
  itsBackgroundColor = color;
}

// #######################################################################
int SceneGenerator::getObjectSize()
{
  return itsObjectSize;
}

int SceneGenerator::getTargetObject()
{
  return itsTargetObject;
}

// #######################################################################
PixRGB<byte> SceneGenerator::getTargetColor()
{
  return itsTargetColor;
}

// #######################################################################
int SceneGenerator::getLum()
{
  return itsLum;
}

// #######################################################################
int SceneGenerator::getRotation()
{
  return itsRot;
}

// #######################################################################
PixRGB<byte> SceneGenerator::getColor()
{
  return itsColor;
}

// #######################################################################
int SceneGenerator::getNoise()
{
  return itsNoise;
}

// #######################################################################
PixRGB<byte> SceneGenerator::getBackgroundColor()
{
  return itsBackgroundColor;
}

SceneGenerator::SceneType SceneGenerator::getSceneType()
{
  return itsSceneType;
}

uint SceneGenerator::getNumSceneTypes()
{
  return numSceneTypeNames;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
