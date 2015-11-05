/*! @file ObjRec/test-LabelMeSaliency.C get saliency information from  */
/* the label me data set */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ObjRec/test-LabelMeSaliencyJustObj.C $
// $Id: test-LabelMeSaliencyJustObj.C 10982 2009-03-05 05:11:22Z itti $
//


#include "Component/ModelManager.H"
#include "Image/Image.H"
#include "Image/ImageSet.H"
#include "Image/ShapeOps.H"
#include "Image/DrawOps.H"
#include "Image/FilterOps.H"
#include "Image/ColorOps.H"
#include "Image/Transforms.H"
#include "Image/MathOps.H"
#include "Neuro/StdBrain.H"
#include "Neuro/VisualCortex.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/SaliencyMap.H"
#include "Media/TestImages.H"
#include "Media/SceneGenerator.H"
#include "Channels/DescriptorVec.H"
#include "Channels/ComplexChannel.H"
#include "GUI/DebugWin.H"




ModelManager *mgr;

int main(const int argc, const char **argv)
{

  MYLOGVERB = LOG_INFO;
  mgr = new ModelManager("Test LabelMeSaliency");

  //our brain
  nub::ref<StdBrain>  brain2(new StdBrain(*mgr));
  mgr->addSubComponent(brain2);


  mgr->exportOptions(MC_RECURSE);
  mgr->setOptionValString(&OPT_RawVisualCortexChans, "IOC");
  //mgr.setOptionValString(&OPT_RawVisualCortexChans, "I");
  //mgr->setOptionValString(&OPT_RawVisualCortexChans, "GNO");
  //mgr.setOptionValString(&OPT_RawVisualCortexChans, "N");
  //manager.setOptionValString(&OPT_UseOlderVersion, "false");
  // set the FOA and fovea radii
  mgr->setOptionValString(&OPT_SaliencyMapType, "Fast");
  mgr->setOptionValString(&OPT_SMfastInputCoeff, "1");

  mgr->setOptionValString(&OPT_WinnerTakeAllType, "Fast");

  mgr->setModelParamVal("FOAradius", 100, MC_RECURSE);
  mgr->setModelParamVal("FoveaRadius", 100, MC_RECURSE);

  mgr->setOptionValString(&OPT_IORtype, "Disc");

  if (mgr->parseCommandLine(
        (const int)argc, (const char**)argv, "<path to images>", 1, 1) == false);

  mgr->start();
  nub::ref<StdBrain>  brain = dynCastWeak<StdBrain>(mgr->subComponent(0));

 //"/lab/ilab15/tmp/objectsDB/mit/labelMe/05june05_static_indoor",

  TestImages testImages(mgr->getExtraArg(0).c_str(), TestImages::MIT_LABELME);


  printf("## \"Filename\", \"Size\", \"Number of objects\", \"Obj ID\", \"Size\"\n");

  for(uint scene=0; scene<testImages.getNumScenes(); scene++)
  {

    //get the image
    LINFO("Get scene %i", scene);
    Image<PixRGB<byte> > img = testImages.getScene(scene);
    std::string sceneFile = testImages.getSceneFilename(scene);
    LINFO("Size %ix%i", img.getWidth(), img.getHeight());

    if (testImages.getNumObj() > 0)  //if we have any labled objects
    {
      for (uint obj=0; obj<testImages.getNumObj(); obj++)
      {
        Image<byte> objMask = testImages.getObjMask(obj);
        int objSize = -1;
        for(int y=0; y<objMask.getHeight(); y++)
          for(int x=0; x<objMask.getWidth(); x++)
            if (objMask.getVal(x,y) > 0)
              objSize++;

       // SHOWIMG(rescale(toRGB(objMask), 255, 255));
        std::vector<Point2D<int> > objPoly = testImages.getObjPolygon(obj);
        Point2D<int> p1 = objPoly[0];
        for(uint i=1; i<objPoly.size(); i++)
        {
          drawLine(img, p1, objPoly[i], PixRGB<byte>(255, 0, 0), 4);
          p1 = objPoly[i];
        }
        drawLine(img, p1, objPoly[0], PixRGB<byte>(255, 0, 0), 4); //close the polygon

        printf("\"%s\",\"%ix%i\",\"%i\",\"%i\",\"%i\"\n",
            sceneFile.c_str(), img.getWidth(), img.getHeight(),
            testImages.getNumObj(), obj, objSize);

      }
      SHOWIMG(img);

    } else {
      printf("##%s has no objects \n", sceneFile.c_str());
    }
  }

}

