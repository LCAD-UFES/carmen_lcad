/*!@file AppMedia/test-world1.C test simple world 1 */

// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ObjRec/test-world1.C $
// $Id: test-world1.C 10794 2009-02-08 06:21:09Z itti $


#include "Component/ModelManager.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Image/Image.H"
#include "Image/ImageSet.H"
#include "Image/DrawOps.H"
#include "Image/ShapeOps.H"
#include "Image/Layout.H"
#include <math.h>
#include <stdlib.h>
#include "ObjRec/ObjRec.H"
#include "GUI/ImageDisplayStream.H"
#include "GUI/XWinManaged.H"

int main(const int argc, const char **argv)
{
  MYLOGVERB = LOG_INFO;
  ModelManager *mgr = new ModelManager("Test ObjRec");

  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(*mgr));
  mgr->addSubComponent(ofs);

  nub::ref<InputFrameSeries> ifs(new InputFrameSeries(*mgr));
  mgr->addSubComponent(ifs);

  nub::ref<ObjRec> objRec(new ObjRec(*mgr));
  mgr->addSubComponent(objRec);

  mgr->exportOptions(MC_RECURSE);

  if (mgr->parseCommandLine(
        (const int)argc, (const char**)argv, "", 0, 0) == false)
    return 1;
  mgr->start();


  ifs->updateNext();
  //grab the images
  GenericFrame input = ifs->readFrame();
  Image<PixRGB<byte> > worldImg = rescale(input.asRgb(), 256, 256);
  objRec->setImageDims(worldImg.getDims());


  while(0)
  {

    double prob = objRec->predictWorld(worldImg);
    Image<PixRGB<byte> > worldPredictImg = objRec->getWorldPredictImage();

    worldPredictImg += worldImg;

    LINFO("World prob %f", prob);

    //Show the world
    Layout<PixRGB<byte> > outDisp;
    outDisp = vcat(outDisp, hcat(worldImg, worldPredictImg));

    Image<PixRGB<byte> > msgImg(worldImg.getWidth()*2, 20, ZEROS);
    char msg[255];
    sprintf(msg, "World p=%0.2e", prob);
    writeText(msgImg, Point2D<int>(0,0), msg, PixRGB<byte>(255), PixRGB<byte>(127));
    outDisp = vcat(outDisp, msgImg);

    ofs->writeRgbLayout(outDisp, "Result", FrameInfo("Result", SRC_POS));
  }

  double angle = 45;
  double length = 50;
  bool run = false;
  while(1)
  {

    //handle clicks
    const nub::soft_ref<ImageDisplayStream> ids =
      ofs->findFrameDestType<ImageDisplayStream>();

    const rutz::shared_ptr<XWinManaged> uiwin =
      ids.is_valid()
      ? ids->getWindow("world")
      : rutz::shared_ptr<XWinManaged>();


    if (uiwin.is_valid())
    {

      int key = uiwin->getLastKeyPress();
      if (key == 98) angle++;
      if (key == 104) angle--;

      if (key == 100) length++;
      if (key == 102) length--;

      if (key == 27) run=!run;

      if (key != -1)
        LINFO("Length %f  angle %f key %i", length, angle, key);

      Point2D<int> pos = uiwin->getLastMouseClick();
      if (pos.isValid())
      {
        double prob = objRec->evalLikelihood(worldImg, pos, angle, length);
        LINFO("Prob %f", prob);
      }
    }

    if (run)
      objRec->evalLikelihood(worldImg, Point2D<int>(0,0), angle, length);

    Image<PixRGB<byte> > worldPredictImg = objRec->getWorldPredictImage();

    if (worldPredictImg.initialized())
      worldPredictImg += worldImg;
    else
      worldPredictImg = worldImg;

    ofs->writeRGB(worldImg, "world", FrameInfo("world", SRC_POS));
    ofs->writeRGB(worldPredictImg, "predict", FrameInfo("predict", SRC_POS));


  }



  mgr->stop();

  exit(0);

}


