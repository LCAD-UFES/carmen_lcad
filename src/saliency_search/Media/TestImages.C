/*!@file Media/TestImages.C Test Images */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/TestImages.C $
// $Id: TestImages.C 15310 2012-06-01 02:29:24Z itti $
//

#include "Media/TestImages.H"

#include "GUI/DebugWin.H"
#include "Image/CutPaste.H"
#include "Image/DrawOps.H"
#include "Image/ShapeOps.H"
#include "Image/Transforms.H"
#include "Raster/Raster.H"
#include "Util/FileUtil.H" // for isDirectory()
#include "Util/sformat.H"
#include <dirent.h>
#include <list>
#include <algorithm>

//init the objects def for the ALOI lib
const char* TestImages::ALOI_LUM[] = {
  "i110", "i120", "i130", "i140", "i150", "i160", "i170", "i180",
  "i190", "i210", "i230", "i250"
};

const char *TestImages::ALOI_COL[] = {
  "l1c1", "l1c2", "l1c3", "l2c1", "l2c2", "l2c3", "l3c1", "l3c2",
  "l3c3", "l4c1", "l4c2", "l4c3", "l5c1", "l5c2", "l5c3", "l6c1",
  "l6c2", "l6c3", "l7c1", "l7c2", "l7c3", "l8c1", "l8c2", "l8c3"
};

const char *TestImages::ALOI_ROT[] = {
  "r0", "r5", "r10", "r15", "r20", "r25", "r30", "r35", "r40", "r45", "r50",
  "r55", "r60", "r65", "r70", "r75", "r80", "r85", "r90", "r95", "r100",
  "r105", "r110", "r115", "r120", "r125", "r130", "r135", "r140", "r145",
  "r150", "r155", "r160", "r165", "r170", "r175", "r180", "r185", "r190",
  "r195", "r200", "r205", "r210", "r215", "r220", "r225", "r230", "r235",
  "r240", "r245", "r250", "r255", "r260", "r265", "r270", "r275", "r280",
  "r285", "r290", "r295", "r300", "r305", "r310", "r315", "r320", "r325",
  "r330", "r335", "r340", "r345", "r350", "r355"
};

// #######################################################################
TestImages::TestImages(const char* imagesPath, LIBRARY lib,
    const int numTraining,
    const int numTesting,
    const int numObjects) :
  itsImagesPath(imagesPath), itsLibType(lib),
  itsObjPolygon(0,std::vector<Point2D<int> >(0)), itsObjNames(0),
  itsSceneFilenames(0), itsObjects(0),
  itsScenes(0),
  itsTrainScenes(0),
  itsTestScenes(0),
  itsNumTraining(numTraining),
  itsNumTesting(numTesting),
  itsNumObjects(numObjects)
{
  switch (itsLibType) {
  case ALOI:
    itsMaxLum = sizeof(ALOI_LUM)/4;
    itsMaxCol = sizeof(ALOI_COL)/4;
    itsMaxRot = sizeof(ALOI_ROT)/4;
    break;

  case CSAIL:
    break;

  case COIL:
    break;

  case MIT_LABELME:
    readDir(itsImagesPath);
    break;

  case CSCLAB:
    itsSceneId = -1;
    readDir(itsImagesPath);
    break;

  case CALTECH256:
    itsSceneId = -1;
    readCalTech256Dir(itsImagesPath, itsNumTraining, itsNumTesting);
    break;

  case IMG_DIR:
    readDir(itsImagesPath);
    break;

  case XMLFILE:
    readObjXML(itsImagesPath);
    break;

  default:
    LERROR("Unknown lib type");
  }
}

// ######################################################################
TestImages::~TestImages()
{  }

// ######################################################################
uint TestImages::getNumObj(uint scene)
{
  switch (itsLibType) {
  case ALOI:
    break;

  case CSAIL:
    break;

  case COIL:
    break;

  case MIT_LABELME:
    return itsObjNames.size();
    break;

  case CSCLAB:
    return itsObjNames.size();
    break;

  case IMG_DIR:
    return itsSceneFilenames.size();
    break;

  case XMLFILE:
    ASSERT(scene < itsScenes.size());
    return itsScenes[scene].objects.size();
    break;

  default:
    LERROR("Unknown lib type");
  }

  return 0;
}

// #######################################################################
void TestImages::readDir(const char *path)
{
  DIR *dp = opendir(path);
  if (dp != NULL) {
    dirent *dirp;
    while ((dirp = readdir(dp)) != NULL ) {
      if (dirp->d_name[0] != '.' &&
          isDirectory(dirp) == false)
        {
          std::string filePath = std::string(path) + '/' +
            std::string(dirp->d_name);
          if (Raster::fileExists(filePath,RASFMT_AUTO, true))
            itsSceneFilenames.push_back(filePath);
        }
    }
    LDEBUG("%" ZU " files in the directory\n", itsSceneFilenames.size());
  } else LFATAL("%s is not a directory", path);
}

// #######################################################################
void TestImages::readCalTech256Dir(const char *path, const int trainingSize,
                                   const int testingSize, bool shuffle)
{
  DIR *dp = opendir(path);
  int i = 0;
  if (dp != NULL)
    {
      dirent *dirp;
      while ((dirp = readdir(dp)) != NULL ) {
        if (dirp->d_name[0] != '.' &&
            isDirectory(dirp) == true)
          {
            std::string filePath = std::string(path) + '/' +
              std::string(dirp->d_name);

            // get the individual files
            itsSceneFilenames.clear();
            readDir(filePath.c_str());

            // if (shuffle)
            //  std::random_shuffle(itsSceneFilenames.begin(),
            //  itsSceneFilenames.end());

            int training = trainingSize;
            int testing = testingSize;
            if (training == -1)
              training = itsSceneFilenames.size()/2;

            int scene = 0;

            for(; scene < training && scene < (int)itsSceneFilenames.size();
                scene++)
              {
                SceneData sceneData;
                sceneData.description = std::string(dirp->d_name);
                sceneData.filename = itsSceneFilenames[scene];
                sceneData.useType = TRAIN;
                itsTrainScenes.push_back(sceneData);
              }

            if (testing == -1)
              testing = itsSceneFilenames.size()/2;

            testing += scene;

            for(; scene<testing && scene<(int)itsSceneFilenames.size(); scene++)
              {
                SceneData sceneData;
                sceneData.description = std::string(dirp->d_name);
                sceneData.filename = itsSceneFilenames[scene];
                sceneData.useType = TEST;
                itsTestScenes.push_back(sceneData);
              }
          }

        //only train on num of objects
        if (itsNumObjects != -1 && i++ > itsNumObjects)
          break;
      }
      LDEBUG("%" ZU " scenes in the directory\n", itsSceneFilenames.size());
  } else  LFATAL("%s is not a directory", path);
}

// ######################################################################
Image<PixRGB<byte> > TestImages::getObject(int id, int lum, int col, int rot)
{
  std::string filename;

  switch (itsLibType) {
  case ALOI:
    {
      static int idNumber = 0;
      static int lumNumber = 0;
      static int colNumber = 0;
      static int rotNumber = 0;

      if (id == -1) id = idNumber++;
      id = (id % 1000) + 1; //only 1000 objects are avilable

      if (lum == -1) lum = lumNumber++;
      if (col == -1) col = colNumber++;
      if (rot == -1) rot = rotNumber++;

      if (lum > -1) {
        lum = lum%itsMaxLum; //dont overflow
        filename = sformat("%s/%i/%i_%s.png", itsImagesPath,
                           id, id,ALOI_LUM[lum]);
      } else if (col > -1) {
        col = col%itsMaxCol;  //dont overflow
        filename = sformat("%s/%i/%i_%s.png", itsImagesPath,
                           id, id,ALOI_COL[col]);
      } else if (rot > -1) {
        rot = rot%itsMaxRot; //dont overflow
        filename = sformat("%s/%i/%i_%s.png", itsImagesPath,
                           id, id,ALOI_ROT[rot]);
      }
    }
    break;

  case CSAIL:
    break;

  case COIL:
    {
      static int idNumber = 0;
      static int rotNumber = 0;

      if (id == -1) id = idNumber++;
      id = id % 100 + 1; //only 100 objects are avilable

      if (rot == -1) rot = rotNumber += 5;
      else rot *= 5;

      rot = rot % 360;

      filename = sformat("%s/obj%i__%i.png", itsImagesPath, id, rot);
    }
    break;

  case CSCLAB:
    return getCscLabObj(id);
    break;

  case IMG_DIR:
    {
      static int idNumber = 0;

      if (id == -1) id = idNumber++;
      id = id % itsSceneFilenames.size();

      filename = sformat("%s/%s", itsImagesPath, itsSceneFilenames[id].c_str());
    }
    break;

  case MIT_LABELME:
    return getLabelMeObj(id);
    break;

  default:
    LERROR("Unknown lib type");
  }

  return  Raster::ReadRGB(filename);
}


// ######################################################################
TestImages::ObjData TestImages::getObjectData(uint scene, uint obj,
                                              bool getImage)
{
  ASSERT(scene < itsScenes.size() && obj < itsScenes[scene].objects.size());

  ObjData objData = itsScenes[scene].objects[obj];

  if (getImage) {
    // find the object dimention from the polygon:

    // tranform the outline (becuase of the crop)
    std::vector<Point2D<int> > newObjOutline;

    if (objData.polygon.size() > 0)
      {
        Point2D<int> upperLeft = objData.polygon[0];
        Point2D<int> lowerRight = objData.polygon[0];
        for(uint i=0; i<objData.polygon.size(); i++)
          {
            // find the bounds for the crop
            if (objData.polygon[i].i < upperLeft.i)
              upperLeft.i = objData.polygon[i].i;
            if (objData.polygon[i].j < upperLeft.j)
              upperLeft.j = objData.polygon[i].j;
            if (objData.polygon[i].i > lowerRight.i)
              lowerRight.i = objData.polygon[i].i;
            if (objData.polygon[i].j > lowerRight.j)
              lowerRight.j = objData.polygon[i].j;
          }

        // getting object from the scene
        objData.img = Raster::ReadRGB(itsScenes[scene].filename);
        // check for out of bounds and fix
        if (upperLeft.i < 0) upperLeft.i = 0;
        if (upperLeft.j < 0) upperLeft.j = 0;
        if (lowerRight.i < 0) lowerRight.i = 0;
        if (lowerRight.j < 0) lowerRight.j = 0;
        if (lowerRight.i > objData.img.getWidth())
          lowerRight.i = objData.img.getWidth();
        if (lowerRight.j > objData.img.getHeight())
          lowerRight.j = objData.img.getHeight();

        objData.dims = Dims(lowerRight.i-upperLeft.i-1,
                            lowerRight.j-upperLeft.j-1);

        for (uint i=0; i<objData.polygon.size(); i++)
          newObjOutline.push_back(Point2D<int>(objData.polygon[i].i -
                                               upperLeft.i,
                                               objData.polygon[i].j -
                                               upperLeft.j));

        // LINFO("Cropping %ix%i, size: %ix%i\n",
        //     upperLeft.i, upperLeft.j,
        //     objData.dims.h(), objData.dims.w());
        objData.img = crop(objData.img, upperLeft, objData.dims);
      }

    if (!objData.filename.empty())
      objData.img = Raster::ReadRGB(objData.filename);
  }
  return objData;
}

// ######################################################################
TestImages::SceneData TestImages::getSceneData(uint scene, USETYPE useType)
{
  switch (useType) {
  case TRAIN:
    ASSERT(scene < itsTrainScenes.size());
    return itsTrainScenes[scene];
    break;

  case TEST:
    ASSERT(scene < itsTestScenes.size());
    return itsTestScenes[scene];
    break;

  default:
    ASSERT(scene < itsScenes.size());
    return itsScenes[scene];
    break;
  }
}

// ######################################################################
TestImages::ObjData TestImages::getObjFromPos(uint scene,
                                              const Point2D<int> &pt)
{
  ASSERT(scene < itsScenes.size());

  ObjData objData;
  for(uint obj=0; obj<itsScenes[scene].objects.size(); obj++)
    {
      objData = itsScenes[scene].objects[obj];

      // find the object dimention from the polygon
      if (objData.polygon.size() > 0)
        {
          Point2D<int> upperLeft = objData.polygon[0];
          Point2D<int> lowerRight = objData.polygon[0];

          for(uint i=0; i<objData.polygon.size(); i++)
            {
              // find the bounds for the crop
              if (objData.polygon[i].i < upperLeft.i)
                upperLeft.i = objData.polygon[i].i;
              if (objData.polygon[i].j < upperLeft.j)
                upperLeft.j = objData.polygon[i].j;
              if (objData.polygon[i].i > lowerRight.i)
                lowerRight.i = objData.polygon[i].i;
              if (objData.polygon[i].j > lowerRight.j)
                lowerRight.j = objData.polygon[i].j;
            }

          // check if point is within the polygon
          for (int y = upperLeft.j; y < lowerRight.j; y++)
            for (int x = upperLeft.i; x < lowerRight.i; x++)
              if(pnpoly(objData.polygon, pt)) return objData;
          return objData;
        }
    }

  return objData;
}

// ######################################################################
Image<PixRGB<byte> > TestImages::generateScene(uint scene)
{
  ASSERT(scene < itsScenes.size());

  Image<PixRGB<byte> > sceneImg;
  if (itsScenes[scene].filename.empty())
    sceneImg = Image<PixRGB<byte> >(itsScenes[scene].dims, NO_INIT);
  else
    {
      sceneImg = Raster::ReadRGB(itsScenes[scene].filename);
      sceneImg = rescale(sceneImg, itsScenes[scene].dims); //resize the image
    }

  for(uint obj=0; obj<itsScenes[scene].objects.size(); obj++)
    {
      ObjData objData = getObjectData(scene,obj);
      if (objData.dims.w() > 0 && objData.dims.h() > 0)
        // use the first point to place the object
        if (objData.polygon.size() > 0)
          {
            if (itsScenes[scene].filename.empty()) {
              // this is a generated background
              inplacePaste(sceneImg, objData.img, objData.polygon[0]);
            } else
              pasteImage(sceneImg, objData.img, PixRGB<byte>(0,0,0),
                         objData.polygon[0], 0.5F);
      }
    }
  return sceneImg;
}

// ######################################################################
Image<PixRGB<byte> > TestImages::getCscLabObj(int id)
{
  std::vector<Point2D<int> > objOutline = getObjPolygon(id);

  std::string filename = std::string(itsImagesPath) + "/images/" +
    itsSceneFilenames[itsSceneId];
  filename.replace(filename.size()-4, 4, "-R.png"); //change extention to xml
  Image<PixRGB<byte> > objImg =  Raster::ReadRGB(filename);

  // crop the object in the scene
  Point2D<int> upperLeft = objOutline[0];
  Dims size(objOutline[2].i-objOutline[0].i,
      objOutline[2].j-objOutline[0].j);

  return(crop(objImg, upperLeft, size));
}

// ######################################################################
Image<PixRGB<byte> > TestImages::getLabelMeObj(int id)
{
  std::vector<Point2D<int> > objOutline = getObjPolygon(id);

  Image<PixRGB<byte> > objImg =
    Raster::ReadRGB(std::string(itsImagesPath) + '/' +
                    itsSceneFilenames[itsSceneId]);

  // crop the object in the scene
  Point2D<int> upperLeft = objOutline[0];
  Point2D<int> lowerRight = objOutline[0];
  for(uint i=0; i<objOutline.size(); i++)
  {
    // find the bounds for the crop
    if (objOutline[i].i < upperLeft.i) upperLeft.i = objOutline[i].i;
    if (objOutline[i].j < upperLeft.j) upperLeft.j = objOutline[i].j;

    if (objOutline[i].i > lowerRight.i) lowerRight.i = objOutline[i].i;
    if (objOutline[i].j > lowerRight.j) lowerRight.j = objOutline[i].j;

  }

  // tranforme the outline (becuase of the crop)
  std::vector<Point2D<int> > newObjOutline;
  for(uint i=0; i<objOutline.size(); i++)
    newObjOutline.push_back(Point2D<int>(objOutline[i].i - upperLeft.i,
                                    objOutline[i].j - upperLeft.j));

  Dims size(lowerRight.i-upperLeft.i, lowerRight.j - upperLeft.j);
  objImg = crop(objImg, upperLeft, size);

  //clear all pixels outsize the polygon
  for(int y=0; y<objImg.getHeight(); y++)
    for(int x=0; x<objImg.getWidth(); x++)
    {
      if (!pnpoly(newObjOutline, Point2D<int>(x, y)))
        objImg.setVal(x,y,PixRGB<byte>(0,0,0)); //clear the pixel
    }

  return(objImg);
}

// ######################################################################
uint TestImages::getMaxLum()
{
  return itsMaxLum;
}

// ######################################################################
uint TestImages::getMaxCol()
{
  return itsMaxCol;
}

// ######################################################################
uint TestImages::getMaxRot()
{
  switch (itsLibType){
    case ALOI:
      return itsMaxRot;
    case CSAIL:
      break;
    case COIL:
      return 360/5;
    default:
      LERROR("Unknown lib type");
  }
  return 0;
}

// ######################################################################
uint TestImages::getNumScenes(USETYPE useType)
{
  switch (itsLibType){
    case MIT_LABELME:
      return itsSceneFilenames.size();
    case CALTECH256:
      switch (useType)
      {
        case TRAIN:
          return itsTrainScenes.size();
          break;
        case TEST:
          return itsTestScenes.size();
          break;
        default:
          return itsScenes.size();
          break;
      }
    case XMLFILE:
      return itsScenes.size();
    default:
      LERROR("Unknown lib type");
  }
  return 0;
}

// ######################################################################
Image<PixRGB<byte> > TestImages::getScene(uint sceneId, USETYPE useType)
{
  switch (itsLibType){
    case ALOI:
      break;
    case CSAIL:
      break;
    case COIL:
      break;
    case MIT_LABELME:
      return getLabelMeScene(sceneId);
    case CSCLAB:
      return getCscLabScene(sceneId);
    case XMLFILE:
      ASSERT(sceneId < itsScenes.size());
      if ((int)itsScenes[sceneId].type.find("Embed") != -1 ||
          (int)itsScenes[sceneId].type.find("Array") != -1)
      {
        LDEBUG("Generating scene from objects");
        return generateScene(sceneId);
      } else  {
        return Raster::ReadRGB(itsScenes[sceneId].filename);
      }
    case CALTECH256:
      switch (useType)
      {
        case TRAIN:
          ASSERT(sceneId < itsTrainScenes.size());
          return Raster::ReadRGB(itsTrainScenes[sceneId].filename);
          break;
        case TEST:
          ASSERT(sceneId < itsTestScenes.size());
          return Raster::ReadRGB(itsTestScenes[sceneId].filename);
          break;
        default:
          ASSERT(sceneId < itsScenes.size());
          return Raster::ReadRGB(itsScenes[sceneId].filename);
          break;
      }
    default:
      LERROR("Unknown lib type");
  }

  return Image<PixRGB<byte> >();
}

// ######################################################################
std::string TestImages::getSceneFilename(uint sceneId)
{
  switch (itsLibType){
    case ALOI:
      break;
    case CSAIL:
      break;
    case COIL:
      break;
    case MIT_LABELME:
      return itsSceneFilenames[sceneId];
    case CSCLAB:
      return itsSceneFilenames[sceneId];
      break;
    case XMLFILE:
      return itsSceneFilenames[sceneId];
      break;
    default:
      LERROR("Unknown lib type");
  }

  return std::string();
}

// ######################################################################
Image<PixRGB<byte> > TestImages::getLabelMeScene(uint sceneId)
{
#ifdef HAVE_LIBXML
  ASSERT(sceneId < itsSceneFilenames.size());

  xmlDocPtr doc;

  //Get the xml file for the given scene
  std::string xmlFile = std::string(itsImagesPath);
  xmlFile.insert(xmlFile.rfind("/"), "/Annotations"); //set the Annotations dir
  xmlFile += "/" + itsSceneFilenames[sceneId];
  xmlFile.replace(xmlFile.size()-4, 4, ".xml"); //change extention to xml

  itsCurrentScene = Raster::ReadRGB(std::string(itsImagesPath) + '/' +
                                    itsSceneFilenames[sceneId]);

  //clear the objname and polygons
  itsObjNames.clear();
  itsObjPolygon.clear();

  //check if the file exists
  doc = xmlReadFile(xmlFile.c_str(), NULL, 0);
  if (doc == NULL)
  {
    LINFO("Failed to parse %s", xmlFile.c_str());
    return itsCurrentScene;
  }

  /*Get the root element node */
  xmlNode *root_element = xmlDocGetRootElement(doc);

  //look for object annotations
  xmlNodePtr cur = root_element->xmlChildrenNode; //dont care about toop level
  while (cur != NULL) {
    if ((!xmlStrcmp(cur->name, (const xmlChar *)"object"))) {

      xmlNodePtr object = cur->xmlChildrenNode;
      while(object != NULL)  //read the attributes and polygons
      {
        //Name
        if ((!xmlStrcmp(object->name, (const xmlChar *)"name"))) {
          xmlChar* name = xmlNodeListGetString(doc, object->xmlChildrenNode, 1);
          if (name != NULL)
          {
            itsObjNames.push_back(std::string((const char*)name));
            xmlFree(name);
          }
        }

        //Polygon
        //Read the points
        if ((!xmlStrcmp(object->name, (const xmlChar *)"polygon"))) {
          xmlNodePtr poly = object->xmlChildrenNode;

          std::vector<Point2D<int> > points;
          while(poly != NULL)  //read the attributes and polygons
          {
            //Get a point
            if ((!xmlStrcmp(poly->name, (const xmlChar *)"pt"))) {

              //get Point x and Point y

              int x = -1, y = -1;
              xmlNodePtr point = poly->xmlChildrenNode;
              while (point != NULL)
              {
                if ((!xmlStrcmp(point->name, (const xmlChar *)"x"))) {
                  xmlChar* xLoc =
                    xmlNodeListGetString(doc, point->xmlChildrenNode, 1);
                  x = atoi((const char*)xLoc);
                  xmlFree(xLoc);
                }

                if ((!xmlStrcmp(point->name, (const xmlChar *)"y"))) {
                  xmlChar* yLoc =
                    xmlNodeListGetString(doc, point->xmlChildrenNode, 1);
                  y = atoi((const char*)yLoc);
                  xmlFree(yLoc);
                }
                point = point->next; //next
              }
              points.push_back(Point2D<int>(x,y));
            }
            poly = poly->next; //next point
          }
          itsObjPolygon.push_back(points);
        }
        object = object->next; //next object
      }
    }
    cur = cur->next;
  }
  xmlFreeDoc(doc);
  xmlCleanupParser();

#else
  LFATAL("Need xmllib to parse scene files");
#endif

  return itsCurrentScene;
}

// ######################################################################
// get only the right images
Image<PixRGB<byte> > TestImages::getCscLabScene(uint sceneId)
{
#ifdef HAVE_LIBXML
  ASSERT(sceneId < itsSceneFilenames.size());

  xmlDocPtr doc;

  //Get the xml file for the given scene
  std::string xmlFile = std::string(itsImagesPath) + "/" +
    itsSceneFilenames[sceneId];

  doc = xmlReadFile(xmlFile.c_str(), NULL, 0);
  if (doc == NULL)
    LFATAL("Failed to parse %s", xmlFile.c_str());

  //clear the objname and polygons
  itsObjNames.clear();
  itsObjPolygon.clear();

  /*Get the root element node */
  xmlNode *root_element = xmlDocGetRootElement(doc);

  //look for object annotations
  xmlNodePtr cur = root_element->xmlChildrenNode; //dont care about toop level
  while (cur != NULL) {
    if ((!xmlStrcmp(cur->name, (const xmlChar *)"Object"))) {

      xmlChar* name = xmlGetProp(cur, (const xmlChar*)"id");
      itsObjNames.push_back(std::string((const char*)name));
      xmlFree(name);

      xmlNodePtr object = cur->xmlChildrenNode;
      while(object != NULL)  //read the attributes and polygons
      {
        //Get right Polygon
        if ((!xmlStrcmp(object->name, (const xmlChar *)"Box")) &&
            !xmlStrcmp(xmlGetProp(object, (const xmlChar*)"side"),
                       (const xmlChar*)"right"))
        {
          int left, top, width, height;
          xmlChar* data;

          data = xmlGetProp(object, (const xmlChar*)"left");
          left = atoi((const char*)data);
          xmlFree(data);

          data = xmlGetProp(object, (const xmlChar*)"top");
          top = atoi((const char*)data);
          xmlFree(data);

          data = xmlGetProp(object, (const xmlChar*)"width");
          width = atoi((const char*)data);
          xmlFree(data);

          data = xmlGetProp(object, (const xmlChar*)"height");
          height = atoi((const char*)data);
          xmlFree(data);

          //make a polygon
          std::vector<Point2D<int> > points;
          points.push_back(Point2D<int>(left, top));
          points.push_back(Point2D<int>(left+width, top));
          points.push_back(Point2D<int>(left+width, top+height));
          points.push_back(Point2D<int>(left, top+height));
          itsObjPolygon.push_back(points);
        }
        object = object->next; //next object
      }
    }
    cur = cur->next;
  }
  xmlFreeDoc(doc);
  xmlCleanupParser();

#else
  LFATAL("Need xmllib to parse scene files");
#endif

  itsSceneId = sceneId;
  std::string filename = std::string(itsImagesPath) + "/images/" +
    itsSceneFilenames[sceneId];
  filename.replace(filename.size()-4, 4, "-R.png"); //change extention to xml
  return  Raster::ReadRGB(filename);
}

// ######################################################################
const char* TestImages::getObjName(uint id)
{
  ASSERT(id < itsObjNames.size());
  return itsObjNames[id].c_str();
}

// ######################################################################
std::vector<Point2D<int> > TestImages::getObjPolygon(uint id)
{
  ASSERT(id < itsObjPolygon.size());
  return itsObjPolygon[id];
}

// ######################################################################
Image<byte> TestImages::getObjMask(uint obj)
{
  std::vector<Point2D<int> > objOutline = getObjPolygon(obj);
  Image<byte> objMask(itsCurrentScene.getDims(), ZEROS);
  drawFilledPolygon(objMask, objOutline, byte(255));

  return objMask;
}

// ######################################################################
Image<byte> TestImages::getObjMask(uint scene, uint obj)
{
  ASSERT(scene < itsScenes.size() && obj < itsScenes[scene].objects.size());

  SceneData sceneData = itsScenes[scene];
  ObjData objData = itsScenes[scene].objects[obj];
  std::vector<Point2D<int> > objOutline = objData.polygon;

  Image<byte> objMask(sceneData.dims, ZEROS);
  LDEBUG("Mask %ix%i\n", objMask.getWidth(), objMask.getHeight());
  drawFilledPolygon(objMask, objOutline, byte(255));

  return objMask;
}

// ######################################################################
Image<byte> TestImages::getObjMask(uint scene, uint obj, const Dims& sceneDims)
{
  ASSERT(scene < itsScenes.size() && obj < itsScenes[scene].objects.size());

  ObjData objData = itsScenes[scene].objects[obj];
  std::vector<Point2D<int> > objOutline = objData.polygon;

  Image<byte> objMask(sceneDims, ZEROS);
  LDEBUG("Mask %ix%i\n", objMask.getWidth(), objMask.getHeight());
  drawFilledPolygon(objMask, objOutline, byte(255));

  return objMask;
}

// ######################################################################
Image<byte> TestImages::getAllObjMask()
{
  Image<byte> objMask(2560, 1920, ZEROS);

  //clear all pixels outsize the polygon
  for (int y = 0; y < objMask.getHeight(); y++)
    for (int x = 0; x < objMask.getWidth(); x++)
      {
        for(uint obj = 0; obj < getNumObj(); obj++)
          {
            std::vector<Point2D<int> > objOutline = getObjPolygon(obj);
            if (pnpoly(objOutline, Point2D<int>(x, y)))
              {
                objMask.setVal(x, y, byte(255));
                break; //no need to check other objects.
              }
          }
      }

  return(objMask);
}

// ######################################################################
void TestImages::readObjXML(const char *filename)
{
#ifdef HAVE_LIBXML
  xmlDocPtr doc;

  // Get the xml file for the given scene:
  std::string xmlFile;

  if (itsCurrentPath.empty())
  {
    xmlFile = std::string(filename);
    itsCurrentPath = xmlFile.substr(0, xmlFile.rfind('/'));
  } else {
    xmlFile = itsCurrentPath + '/' + std::string(filename);
  }

  //Get the root dir
  LDEBUG("Reading %s", xmlFile.c_str());

  // check if the file exists:
  doc = xmlReadFile(xmlFile.c_str(), NULL, 0);
  if (doc == NULL) {
    LINFO("Failed to parse %s", xmlFile.c_str());
    return;
  }

  /* Get the root element node */
  xmlNode *root_element = xmlDocGetRootElement(doc);

  // look for object annotations
  xmlNodePtr cur = root_element; //->xmlChildrenNode; //dont care about top level

  //Skip the top level if scenes is in the name
  if ((!xmlStrcmp(cur->name, (const xmlChar *)"scenes")))
    cur = root_element->xmlChildrenNode; //dont care about top level

  while (cur != NULL) {
    // check for dir paths
    if ((!xmlStrcmp(cur->name, (const xmlChar *)"dir"))) {
      xmlChar* incPath = xmlGetProp(cur, (const xmlChar*)"path");
      LDEBUG("Setting Path to %s", (const char *)incPath);
      itsCurrentPath = std::string((const char *)incPath);
      xmlFree(incPath);
    }

    // check for include directives:
    if ((!xmlStrcmp(cur->name, (const xmlChar *)"include"))) {
      xmlChar* incFileName = xmlGetProp(cur, (const xmlChar*)"filename");
      LDEBUG("Including file %s", (const char *)incFileName);
      readObjXML((const char *)incFileName);
      xmlFree(incFileName);
    }

    // get the objects from the scene:
    if ((!xmlStrcmp(cur->name, (const xmlChar *)"annotation"))) {
      // get the  scene data:
      xmlNodePtr scenePtr = cur->xmlChildrenNode;

      std::string sceneFilename, sceneFolder, sceneDescription, sceneType;
      Dims sceneDims;
      std::vector<ObjData> objects;

      // new scene:
      while(scenePtr != NULL) { // read the attributes and polygons
        getNodeMatchText(doc, scenePtr, "filename", sceneFilename);
        getNodeMatchText(doc, scenePtr, "folder", sceneFolder);
        getNodeMatchText(doc, scenePtr, "description", sceneDescription);

        // get the scene size:
        if ((!xmlStrcmp(scenePtr->name, (const xmlChar *)"specs"))) {
          LDEBUG("Specs %s", sceneDescription.c_str());
          xmlChar* width = xmlGetProp(scenePtr, (const xmlChar*)"width");
          xmlChar* height = xmlGetProp(scenePtr, (const xmlChar*)"height");
          xmlChar* type = xmlGetProp(scenePtr, (const xmlChar*)"type");
          LDEBUG("Dims set %s %s", (char*)width, (char*)height);
          if (width != NULL && height != NULL)
            sceneDims = Dims(atoi((char *)width), atoi((char *)height));
          if (type != NULL)
            sceneType = std::string((const char *)type);
          xmlFree(width);
          xmlFree(height);
          xmlFree(type);
        }

        // get the object details from the scene:
        if ((!xmlStrcmp(scenePtr->name, (const xmlChar *)"object"))) {
          std::string objId, objName, objDescription, objFilename, objFolder;
          std::vector<Point2D<int> > objPolygon;
          Dims objDims(0,0); Image<byte> objMask;

          xmlNodePtr objectPtr = scenePtr->xmlChildrenNode;
          while(objectPtr != NULL) { // read the attributes and polygons
            getNodeMatchText(doc, objectPtr, "id", objId);
            getNodeMatchText(doc, objectPtr, "description", objDescription);
            getNodeMatchText(doc, objectPtr, "name", objName);
            getNodeMatchText(doc, objectPtr, "filename", objFilename);
            getNodeMatchText(doc, objectPtr, "folder", objFolder);


            // Polygon: Read the points
            if ((!xmlStrcmp(objectPtr->name, (const xmlChar *)"polygon"))) {
              xmlNodePtr poly = objectPtr->xmlChildrenNode;

              while(poly != NULL) { // read the attributes and polygons
                // Get a point
                if ((!xmlStrcmp(poly->name, (const xmlChar *)"pt"))) {
                  // get Point x and Point y:
                  std::string xStr,yStr;
                  xmlNodePtr point = poly->xmlChildrenNode;
                  while (point != NULL) {
                    getNodeMatchText(doc, point, "x", xStr);
                    getNodeMatchText(doc, point, "y", yStr);
                    point = point->next; //next
                  }
                  objPolygon.push_back(Point2D<int>(atoi(xStr.c_str()),
                                                    atoi(yStr.c_str())));
                }
                poly = poly->next; //next point
              }
            }

            //Read the bounding box
            if ((!xmlStrcmp(objectPtr->name, (const xmlChar *)"bndbox"))) {
              xmlNodePtr bboxPtr = objectPtr->xmlChildrenNode;
              std::string xmin, xmax, ymin, ymax;
              while(bboxPtr != NULL) { // read the box size
                getNodeMatchText(doc, bboxPtr, "xmin", xmin);
                getNodeMatchText(doc, bboxPtr, "xmax", xmax);
                getNodeMatchText(doc, bboxPtr, "ymin", ymin);
                getNodeMatchText(doc, bboxPtr, "ymax", ymax);
                bboxPtr = bboxPtr->next; //next point
              }
              objPolygon.push_back(Point2D<int>(atoi(xmin.c_str()),
                                                atoi(ymin.c_str())));
              objPolygon.push_back(Point2D<int>(atoi(xmax.c_str()),
                                                atoi(ymin.c_str())));
              objPolygon.push_back(Point2D<int>(atoi(xmax.c_str()),
                                                atoi(ymax.c_str())));
              objPolygon.push_back(Point2D<int>(atoi(xmin.c_str()),
                                                atoi(ymax.c_str())));
            }

            // object mask file?
            std::string objmaskfile;
            getNodeMatchText(doc, objectPtr, "objectmask", objmaskfile);
            if (objmaskfile.empty() == false) {
              if (objFolder.empty() == false)
                objmaskfile = objFolder+"/"+objmaskfile;
              Image<byte> timg = Raster::ReadGray(objmaskfile);
              if (objMask.initialized()) objMask += timg; // will clamp to 255
              else objMask = timg;
            }

            // next object data:
            objectPtr = objectPtr->next;
          }

         // LINFO("Read object %s (id %s) in %s",objName.c_str(),
         //     objId.c_str(),sceneFilename.c_str());

          // assign the object data to array:
          if (objPolygon.size() < 3 && objMask.initialized() == false)
            LINFO("Error: insufficent points in the polygon (%s)", xmlFile.c_str());
          else {
            ObjData objData;


            if (!objFilename.empty()) {
              if (itsCurrentPath.empty())
                objData.filename = objFolder + '/' + objFilename;
              else
                objData.filename = itsCurrentPath + '/' + objFolder +
                  '/' + objFilename;
            }
            objData.id = atoi(objId.c_str());

            // Replace spaces in objname with _
            for (std::string::iterator i = objName.begin();
                 i != objName.end(); i++)
              if (std::isspace(*i))  *i = '_';

            objData.name        = objName;
            objData.description = objDescription;
            objData.dims        = objDims;
            objData.polygon     = objPolygon;
            objData.objmask     = objMask;
            objects.push_back(objData);
          }
        }
        scenePtr = scenePtr->next; //next scene data
      }
      SceneData sceneData;

      //Hack for PASCAL dataset since the folder points to VOC2009/JPEGImages
      if (sceneFolder == "VOC2009")
        sceneFolder += "/JPEGImages";

      if (!sceneFilename.empty()) {
        if (itsCurrentPath.empty())
          sceneData.filename = sceneFolder + '/' + sceneFilename;
        else
          sceneData.filename = itsCurrentPath + '/' + sceneFolder +
            '/' + sceneFilename;
      }

      LDEBUG("Scene %s", sceneData.filename.c_str());
      sceneData.description = sceneDescription;
      sceneData.dims        = sceneDims;
      sceneData.type        = sceneType;
      sceneData.objects     = objects;

      itsScenes.push_back(sceneData);
    }
    cur = cur->next;
  }

  xmlFreeDoc(doc);
  xmlCleanupParser();

#else
  LFATAL("Need xmllib to parse scene files");
#endif

  return;

}

// ######################################################################
#ifdef HAVE_LIBXML
void TestImages::getNodeMatchText(xmlDocPtr doc, xmlNodePtr nodePtr,
                                  const char* nodeName, std::string &result)
{
  xmlChar *tmp = NULL;
  if (!xmlStrcmp(nodePtr->name, (const xmlChar *)nodeName))
    tmp = xmlNodeListGetString(doc, nodePtr->xmlChildrenNode, 1);

  if (tmp != NULL) {
    result = std::string((const char*)tmp);
    xmlFree(tmp);
  }
}
#endif // HAVE_LIBXML

// ######################################################################
int TestImages::labelScene(uint scene, Image<PixRGB<byte> > &sceneImg)
{
  //draw an outline of the object in the scene
  int lineWidth = int(sceneImg.getWidth()*0.005);

  for (uint obj=0; obj<getNumObj(scene); obj++) //look at all the objects in the scene
  {

    ObjData objData = getObjectData(scene, obj);

    std::vector<Point2D<int> > objPoly = objData.polygon;
    Point2D<int> p1 = objPoly[0];
    Point2D<int> centerLoc = p1;
    for(uint i=1; i<objPoly.size(); i++)
    {
      drawLine(sceneImg, p1, objPoly[i], PixRGB<byte>(255, 0, 0), lineWidth);
      p1 = objPoly[i];
      centerLoc.i += p1.i; centerLoc.j += p1.j;
    }
    drawLine(sceneImg, p1, objPoly[0], PixRGB<byte>(255, 0, 0), lineWidth); //close the polygon

    centerLoc.i /= objPoly.size();
    centerLoc.j /= objPoly.size();
    writeText(sceneImg, centerLoc, objData.description.c_str(), PixRGB<byte>(255), PixRGB<byte>(0));
  }

  return 1;

}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
