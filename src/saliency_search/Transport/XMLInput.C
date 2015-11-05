/*!@file Transport/XMLInput.C Use xml file as input */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/XMLInput.C $
// $Id: XMLInput.C 12962 2010-03-06 02:13:53Z irock $
//

#include "Transport/XMLInput.H"
#include "Transport/TransportOpts.H"
#include "Component/OptionManager.H"
#include "Component/ModelOptionDef.H"
#include "Image/ShapeOps.H"
#include "Image/CutPaste.H"
#include "Raster/GenericFrame.H"
#include "Image/DrawOps.H"

const ModelOptionDef OPT_XMLDrawPolygons =
  { MODOPT_ARG(bool), "XMLDrawPolygons", &MOC_INPUT, OPTEXP_CORE,
    "Draw the polygons on the frame with the name of the object ",
    "draw-polygons", '\0', "<true/false>", "false" };

const ModelOptionDef OPT_XMLGetObjects =
  { MODOPT_ARG(bool), "XMLGetObjects", &MOC_INPUT, OPTEXP_CORE,
    "Return frames with only the objects in them. Can be used for training or testing only objects.",
    "get-objects", '\0', "<true/false>", "false" };

const ModelOptionDef OPT_XMLFilterObjectName =
  { MODOPT_ARG(std::string), "XMLFilterObjectName", &MOC_INPUT, OPTEXP_CORE,
    "Only return objects matching the given name.",
    "filter-object-name", '\0', "string", "" };

const ModelOptionDef OPT_XMLRootPath =
  { MODOPT_ARG(std::string), "XMLRootPath", &MOC_INPUT, OPTEXP_CORE,
    "The root path from which all xml files will be read from.",
    "xml-root-path", '\0', "string", "" };

// ######################################################################
XMLInput::XMLInput(OptionManager& mgr) :
  FrameIstream(mgr, "XMLInput Input", "XMLInputInput"),
  itsDrawPolygons(&OPT_XMLDrawPolygons, this),
  itsGetObjects(&OPT_XMLGetObjects, this),
  itsFilterObjectName(&OPT_XMLFilterObjectName, this),
  itsRootPath(&OPT_XMLRootPath, this),
  itsFrameNum(0),
  itsCurrentSceneNum(0),
  itsObjectNum(0)
{
}

XMLInput::~XMLInput()
{
}

// ######################################################################
void XMLInput::setConfigInfo(const std::string& filename)
{
  // NOTE: if you modify any behavior here, then please update the
  // corresponding documentation for the global "--in" option inside
  // the OPT_InputFrameSource definition in Media/MediaOpts.C

  if (filename.size() == 0)
    return;

  LINFO("Reading xml file %s", filename.c_str());
  if (itsTestImages.get() == 0)
    itsTestImages.reset(new TestImages(filename.c_str(), TestImages::XMLFILE));

}

bool XMLInput::setFrameNumber(int n)
{
  ASSERT(n >= 0);
  itsFrameNum = n;

  return true;
}


// ######################################################################
GenericFrameSpec XMLInput::peekFrameSpec()
{
  GenericFrameSpec result;

  result.nativeType = GenericFrame::RGB_U8;
  result.videoFormat = VIDFMT_AUTO;
  result.videoByteSwap = false;
  result.dims = Dims(0,0);
  result.floatFlags = 0;

  return result;
}

// ######################################################################
GenericFrame XMLInput::readFrame()
{
  if (itsTestImages.get() == 0)
    LFATAL("No scene data. Need xml file");

  if (!itsGetObjects.getVal())
    itsCurrentSceneNum = itsFrameNum;

  //If we dont have the frame number, then return an empty image
  if (itsCurrentSceneNum >= itsTestImages->getNumScenes())
  {
    LINFO("No more scenes");
    return GenericFrame();
  }


  //Get the scene


  TestImages::SceneData sceneData = itsTestImages->getSceneData(itsCurrentSceneNum);
  rutz::shared_ptr<TestImages::SceneData> scene(new TestImages::SceneData);
  scene->description = sceneData.description;
  scene->filename = sceneData.filename;
  scene->type = sceneData.type;
  scene->useType = sceneData.useType;

//  LINFO("Scene %s", sceneData.filename.c_str());
  Image<PixRGB<byte> > sceneImg;
  if (itsGetObjects.getVal())
  {

    if (itsObjectNum < sceneData.objects.size())
    {
      TestImages::ObjData objData = sceneData.objects[itsObjectNum];
      std::vector<Point2D<int> > objPoly = objData.polygon;

      Image<PixRGB<byte> > img = itsTestImages->getScene(itsCurrentSceneNum);

      //Get the bounding box
      Rectangle rect = findBoundingRect(objPoly, img.getDims());
      sceneImg = crop(img, rect);

      scene->objects.push_back(objData);
      itsObjectNum++;
      if (itsObjectNum >= sceneData.objects.size())
      {
        itsCurrentSceneNum++;
        itsObjectNum = 0;
      }
    }



  } else {
    scene->objects = sceneData.objects;
    sceneImg = itsTestImages->getScene(itsCurrentSceneNum);

    if (itsDrawPolygons.getVal())
    {
      for(uint i=0; i<sceneData.objects.size(); i++)
      {
        TestImages::ObjData objData = sceneData.objects[i];

        if (itsFilterObjectName.getVal() == objData.name || itsFilterObjectName.getVal().empty())
        {
          std::vector<Point2D<int> > objPoly = objData.polygon;
          Point2D<int> p1 = objPoly[0];
          for(uint i=1; i<objPoly.size(); i++)
          {
            drawLine(sceneImg, p1, objPoly[i], PixRGB<byte>(0, 255, 0), 0);
            p1 = objPoly[i];
          }
          drawLine(sceneImg, p1, objPoly[0], PixRGB<byte>(0, 255, 0)); //close the polygon

          writeText(sceneImg, objPoly[0]+10, objData.name.c_str(), PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0));
        }

      }
    }

  }

  if (!sceneData.dims.isEmpty())
    sceneImg = rescale(sceneImg, sceneData.dims);
  scene->dims = sceneImg.getDims();


  GenericFrame frame(sceneImg);
  frame.addMetaData(std::string("SceneData"), scene);

  return frame;
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
