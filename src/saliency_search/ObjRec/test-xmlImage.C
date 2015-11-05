/*! @file ObjRec/test-xmlImage.C test reading image information from xml files */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
// by the University of Southern California (USC) and the iLab at USC.  //
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ObjRec/test-xmlImage.C $
// $Id: test-xmlImage.C 9412 2008-03-10 23:10:15Z farhan $
//


#include "Component/ModelManager.H"
#include "Image/Image.H"
#include "Image/ImageSet.H"
#include "Image/DrawOps.H"
#include "Media/TestImages.H"
#include "Raster/Raster.H"
#include "GUI/DebugWin.H"


int main(const int argc, const char **argv)
{

  ModelManager *mgr;
  MYLOGVERB = LOG_INFO;
  mgr = new ModelManager("Test ObjRec");

  if (mgr->parseCommandLine(
        (const int)argc, (const char**)argv, "<images set xml file>", 1, 1) == false)
    return 1;

  mgr->start();

  const char *imageSetFile = mgr->getExtraArg(0).c_str();
  //load the images
  TestImages testImages(imageSetFile, TestImages::XMLFILE);

  for (uint scene=0; scene<testImages.getNumScenes(); scene++) //look at all the scenes
  {
    Image<PixRGB<byte> > sceneImg = testImages.getScene(scene);
    TestImages::SceneData sceneData = testImages.getSceneData(scene);

    LINFO("Scene name %s", sceneData.filename.c_str());
    for (uint obj=0; obj<testImages.getNumObj(scene); obj++) //look at all the objects in the scene
    {
      TestImages::ObjData objData = testImages.getObjectData(scene, obj);
      LINFO("Image: %i %s %s", objData.id, objData.description.c_str(), objData.filename.c_str());
      printf("Image: %i %s %s\n", objData.id, objData.description.c_str(), objData.filename.c_str());
      //SHOWIMG(objData.img);


      //draw an outline of the object in the scene
      int lineWidth = int(sceneImg.getWidth()*0.005);


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
    SHOWIMG(sceneImg);
    //char filename[255];
    //sprintf(filename, "outlinedImages/%s.ppm", sceneData.filename.c_str());
    //LINFO("Write image %s\n", filename);
    //Raster::WriteRGB(sceneImg, filename);

  }

  // stop all our ModelComponents
  mgr->stop();

  return 0;

}

