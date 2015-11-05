/*!@file Media/SceneGenerator.H generate test scenes */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/SceneGenerator.H $
// $Id: SceneGenerator.H 9412 2008-03-10 23:10:15Z farhan $
//

#ifndef SCENEGENERATOR_H
#define SCENEGENERATOR_H

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/Raster.H"
#include "Media/TestImages.H"

#include <vector>
#include <string>

class SceneGenerator
{
public :
  enum SceneType {SIMPLE_EDGES, SIMPLE_SHAPES, ALOI_OBJECTS, COIL_OBJECTS, SATELLITE_KATRINA, MIT_LABELME};

  struct ObjectInfo
  {
    Point2D<int> loc;
    int objId;

    ObjectInfo(Point2D<int> l, int id): loc(l), objId(id) {}
  };

  //! Constructor
  SceneGenerator(SceneType sceneType, int width = 255, int height = 255);


  ~SceneGenerator();

  //! Get a generated scene
  Image<PixRGB<byte> > getScene(int nobj, int lum=-1, int col=-1, int rot=-1);

  //! Check whether the target is in this position given a search window
  bool checkTargetPos(const Point2D<int> &pt, const Dims &searchWin);

  //! get the obj in this position given a search window
  int getObjFromPos(const Point2D<int> &pt, const Dims &searchWin);

  //! get the target position (first one)
  const Point2D<int> getTargetPos() const;

  void setSceneType(SceneType sceneId);

  //! set the target object
  void setTargetObject(int obj);

  //! Set the object size
  void setObjectSize(int size);

  //! set the target color
  void setTargetColor(PixRGB<byte> color);

  //! set the objects illumination
  void setLum(int lum);

  //! set the objects colors
  void setColor(PixRGB<byte> color);

  //! set the noise
  void setNoise(int level);

  //! set the rotation
  void setRotation(int rot);

  //! Set the background color
  void setBackgroundColor(PixRGB<byte> color);


  //! get the target object
  int getTargetObject();

  //! get the object size
  int getObjectSize();

  //! get the target color
  PixRGB<byte> getTargetColor();

  //! get the objects illumination
  int getLum();

  //! get the objects colors
  PixRGB<byte> getColor();

  //! get the noise
  int getNoise();

  //! get the rotation
  int getRotation();

  //! get the background color
  PixRGB<byte> getBackgroundColor();

  //! get the scene type id
  SceneType getSceneType();

  //! get a label me scene
  Image<PixRGB<byte> > getLabelMeScene();

  //! get the object location (center of mass)
  Point2D<int> getObjLocation(uint objId);

  //! get the object ID
  int getObjId(uint objId);

  //! Get the number of objects in the array
  uint getNumObj();

  static const int numSceneTypeNames = 6; //set for the number of names

  //! Get the name of a scene type
  const char * getSceneTypeNames(SceneType id){
    ASSERT(id < numSceneTypeNames);
    static const char* SceneTypeNames[numSceneTypeNames] = {
      "Simple Edges",
      "Simple Shapes",
      "Aloi Objects",
      "Coil Objects",
      "Satellite: Katrina",
      "Mit LabelMe"
    };

    return SceneTypeNames[id];
  }

  uint getNumSceneTypes();

private:

  Image<PixRGB<byte> > generateAloiObjectsScene(int nobj, int lum, int col, int rot);
  Image<PixRGB<byte> > generateCoilObjectsScene(int nobj);
  Image<PixRGB<byte> > generateShapeScene();
  Image<PixRGB<byte> > generateEdgeScene();
  Image<PixRGB<byte> > getSceneFile();

  void readDirScenes(const char *path); //read a list of scenes from the file system




  SceneType itsSceneType;   //The type of scene
  int itsWidth;             //Width of scene
  int itsHeight;            //Height of scene
  TestImages *itsALOIImages; //the Test ALOI Images class
  TestImages *itsCoilImages; //the Test Coil Images class
  TestImages *itsTestImages; //the Test images class
  std::vector<Point2D<int> >  itsTargetsLoc; //the locations of the target in x,y
  std::vector<ObjectInfo> itsObjLocation; //the locations of the objext in the array
  int itsTargetObject;          //the target object
  int itsObjectSize;            //the size of the objects in the scene
  PixRGB<byte>    itsTargetColor;              //Target Color
  int itsLum;                   //Overall illumination per object
  int itsRot;                   //Overall illumination per object
  PixRGB<byte>    itsColor;     //Overall color illumination per object
  int itsNoise;                 //level of noise;
  PixRGB<byte>    itsBackgroundColor;               //background color;
  std::vector<std::string> sceneFilenames;   //A list of scenes filenames

};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
