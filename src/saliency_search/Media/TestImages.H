/*!@file Media/TestImages.H Header file for test Images */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/TestImages.H $
// $Id: TestImages.H 12055 2009-11-18 23:47:49Z lior $
//

#ifndef MEDIA_TESTIMAGES_H_DEFINED
#define MEDIA_TESTIMAGES_H_DEFINED

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/Raster.H"
#include "Raster/GenericFrame.H"

#include <vector>
#include <string>

#ifdef HAVE_LIBXML
#include <libxml/parser.h>
#include <libxml/tree.h>
#endif

//! TestImages
class TestImages
{
public :
  //! Types of libraries available
  enum  LIBRARY { ALOI, CSAIL, COIL, MIT_LABELME, CALTECH256,
                  CSCLAB, IMG_DIR, XMLFILE };

  //! Possible uses
  enum  USETYPE { TRAIN, TEST, ALL };

  struct ObjData {
    Image<PixRGB<byte> > img;
    unsigned int         id;
    double               maxProb;
    double               normProb;
    std::string          name;
    std::string          description;
    std::string          filename;
    Dims                 dims;
    std::vector<Point2D<int> > polygon;
    // if an objectmask file was given instead of a polygon, it will prevail
    Image<byte>          objmask;
    // if available [X, Y, Z, U, V] related to most salient location
    // on object, otherwise empty vector
    std::vector<int>     location3D;
  };

  struct SceneData : public GenericFrame::MetaData
  {
    Image<PixRGB<byte> > img;
    std::string description;
    std::string filename;
    Dims dims;
    std::string type;
    std::vector<ObjData> objects;
    USETYPE useType;
  };


  //! Constructor
  TestImages(const char* imagesPath, LIBRARY lib,
             const int numTraining = -1, const int numTesting = -1,
             const int numObjects = -1);

  ~TestImages();

  //! Return a single object
  //!if lum or col or rot are -1, then return a random of that type
  Image<PixRGB<byte> > getObject(int id=-1, int lum=-1, int col=-2, int rot=-2);

  //! return the scene data
  SceneData getSceneData(uint scene, USETYPE useType=ALL);

  //! Return an ObjData for a single object
  //! load the image when the object is requested to save on memory
  ObjData getObjectData(uint scene, uint obj, bool getImage = false);

  //! get the obj in this position
  ObjData getObjFromPos(uint scene, const Point2D<int> &pt);

  Image<PixRGB<byte> > getScene(uint sceneId, USETYPE useType=ALL);

  std::string getSceneFilename(uint sceneId);

  Image<PixRGB<byte> > getLabelMeScene(uint sceneId);
  Image<PixRGB<byte> > getCscLabScene(uint sceneId);

  //! generate a scene from its objects
  Image<PixRGB<byte> > generateScene(uint scene);

  uint getMaxLum();
  uint getMaxCol();
  uint getMaxRot();

  //! get the number of objects in a scene
  uint getNumObj(uint scene = 0);

  //! get the max number of scenes we have
  uint getNumScenes(USETYPE useType = ALL);

  //! get the object name
  const char* getObjName(uint id);

  //! get the polygon encompesing the obj
  std::vector<Point2D<int> > getObjPolygon(uint id);

  //! get an object mask (use current scene)
  Image<byte> getObjMask(uint id);

  //! get an object mask
  Image<byte> getObjMask(uint scene, uint obj);

  //! get an object mask
  Image<byte> getObjMask(uint scene, uint obj, const Dims& sceneDims);

  //! get a mask from all objects
  Image<byte> getAllObjMask();

  //! label a scene from avilable polygons
  int labelScene(uint scene, Image<PixRGB<byte> > &sceneImg);

  //! Set the root path
  void setRootPath(std::string path) { itsCurrentPath = path; }

private:
  // read a list of scenes from the file system
  void readDir(const char *path);

  void readCalTech256Dir(const char *path, const int trainingSize = -1,
                         const int testingSize = -1, bool shuffe = true);
  // get an object from the current scene
  Image<PixRGB<byte> > getCscLabObj(int id);

  //get an object from the current scene
  Image<PixRGB<byte> > getLabelMeObj(int id);

  //read a list of objects from an xml file
  void readObjXML(const char *path);

#ifdef HAVE_LIBXML
  // retrun the text associaded with the named node
  void getNodeMatchText(xmlDocPtr doc, xmlNodePtr nodePtr,
                        const char* nodeName, std::string &result);
#endif

  const char* itsImagesPath;
  LIBRARY itsLibType;

  int itsMaxLum;
  int itsMaxCol;
  int itsMaxRot;

  static const char* ALOI_LUM[];
  static const char* ALOI_COL[];
  static const char* ALOI_ROT[];

  int itsSceneId;   // the current scene id

  std::vector<std::vector<Point2D<int> > > itsObjPolygon;
  std::vector<std::string> itsObjNames;
  std::vector<std::string> itsSceneFilenames;
  std::vector<ObjData> itsObjects;
  Image<PixRGB<byte> > itsCurrentScene;
  std::string itsCurrentPath;

  std::vector<SceneData> itsScenes;
  std::vector<SceneData> itsTrainScenes;
  std::vector<SceneData> itsTestScenes;

  int itsNumTraining;
  int itsNumTesting;
  int itsNumObjects;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
