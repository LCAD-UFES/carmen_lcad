/*!@file HMAX/test-hmaxRec.C aginst object recognition */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/HMAX/test-hmaxRec.C $
// $Id: test-hmaxRec.C 9412 2008-03-10 23:10:15Z farhan $
//

#include "HMAX/Hmax.H"
#include "Image/Image.H"
#include "Image/ColorOps.H"
#include "Image/MathOps.H"
#include "Image/CutPaste.H"
#include "Image/Transforms.H"
#include "Raster/Raster.H"
#include "Util/Types.H"
#include "Util/log.H"
#include "GUI/DebugWin.H"
#include "Media/TestImages.H"

#include <iostream>
#include <unistd.h>

// number of orientations to use in Hmax
#define NORI 4

int main(const int argc, const char **argv)
{
  int debug=0;
  if (argc != 2)
    { std::cerr<<"USAGE: test-hmaxRec <file>"<<std::endl; exit(1); }

  // get an Hmax object:
  std::vector<int> scss(5);
  scss[0] = 0; scss[1] = 2; scss[2] = 5; scss[3] = 8; scss[4] = 12;
  std::vector<int> spss(4);
  spss[0] = 4; spss[1] = 6; spss[2] = 9; spss[3] = 12;
  Hmax hmax(NORI, spss, scss);

  const char *imageSetFile = argv[1];

  //load the images
  TestImages testImages(imageSetFile, TestImages::XMLFILE);

  //int numMatches = 0; //the number of correct matches
  int totalObjects = 0; //the number of objects presented to the network

  for (uint scene=0; scene<testImages.getNumScenes(); scene++) //look at all the scenes
  {

    for (uint obj=0; obj<testImages.getNumObj(scene); obj++) //look at all the objects
    {
      TestImages::ObjData objData = testImages.getObjectData(scene, obj);

      //crop the obj image to 256x256;
      Point2D<int> upperLeft(0,0);

      if (objData.img.getWidth() > 256 || objData.img.getHeight() > 256)
        upperLeft = Point2D<int>((objData.img.getWidth()/2)-128, (objData.img.getHeight()/2)-128);

      LINFO("Upper left %ix%i", upperLeft.i, upperLeft.j);
        Image<PixRGB<byte> > inputImg = crop(objData.img, upperLeft, Dims(256,256), true);
      //center the object
      if (objData.img.getWidth() < 256 || objData.img.getHeight() < 256)
        inputImg = shift(inputImg, 128-(objData.img.getWidth()/2), 128-(objData.img.getHeight()/2));

      totalObjects++;

      // read the image:
      Image<byte> input = luminance(inputImg); //convert to gray

      Image<PixRGB<byte> > out = toRGB(input);
      if(debug) SHOWIMG(out);


      // pass image through Hmax model:
      Image<float> inputf = input;   // convert image to floats
      Image<float> c2resp = hmax.getC2(inputf);

      float mi, ma; getMinMax(c2resp, mi, ma);
      LINFO("%ix%i min=%f max=%f", c2resp.getWidth(), c2resp.getHeight(), mi, ma);

      printf("%i ", objData.id);
      for(int i=0; i<c2resp.getWidth()*c2resp.getHeight(); i++)
      {
        printf("%i:%f ", i,c2resp[i]);
      }
      printf("\n");


      //use svm to classify

      // display C2 response in an X window:
      if (debug)
      {
        c2resp = scaleBlock(c2resp, input.getDims());

        inplaceNormalize(c2resp, 0.0F, 255.0F);
        SHOWIMG(c2resp);
      }
    }
  }

  return 0;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
