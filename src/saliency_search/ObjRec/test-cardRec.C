/*! @file ObjRec/test-cardRec.C test card recognition */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ObjRec/test-cardRec.C $
// $Id: test-cardRec.C 10794 2009-02-08 06:21:09Z itti $
//

#include "Component/ModelManager.H"
#include "GUI/DebugWin.H"
#include "Image/DrawOps.H"
#include "Image/Image.H"
#include "Image/MathOps.H"
#include "Image/OpenCVUtil.H"
#include "Image/Rectangle.H"
#include "Image/Transforms.H"
#include "Media/FrameSeries.H"
#include "Neuro/EnvSegmenterCannyContour.H"
#include "Raster/GenericFrame.H"
#include "Raster/Raster.H"
#include "SIFT/VisualObject.H"
#include "SIFT/VisualObjectDB.H"
#include "Transport/FrameInfo.H"
#include "Util/Timer.H"

#include <stdio.h>

#define USECOLOR false

VisualObjectDB itsObjectDB;
#ifndef HAVE_OPENCV
  LFATAL("OpenCV must be installed in order to use this function");
#else

std::string recCard(const Image<PixRGB<byte> > &img)
{
  std::string cardName;

  std::vector< rutz::shared_ptr<VisualObjectMatch> > matches;
  rutz::shared_ptr<VisualObject>
    vo(new VisualObject("PIC", "PIC", img,
          Point2D<int>(-1,-1),
          std::vector<double>(),
          std::vector< rutz::shared_ptr<Keypoint> >(),
          USECOLOR));

    const uint nm =
      itsObjectDB.getObjectMatches(vo, matches, VOMA_SIMPLE,
      100U, //max objs to return
      0.5F, //keypoint distance score default 0.5F
      0.5F, //affine distance score default 0.5F
      1.0F, //minscore  default 1.0F
      3U, //min # of keypoint match
      100U, //keypoint selection thershold
      false //sort by preattentive
      );

    LINFO("Found %i", nm);

    if (nm > 0)
    {
      cardName = matches[0]->getVoTest()->getName();
      LINFO("***** %u object recognition match(es) *****", nm);
      for (uint i = 0 ; i < nm; i ++)
        LINFO("   Match with '%s' [score = %f]",
            matches[i]->getVoTest()->getName().c_str(),
            matches[i]->getScore());
    }
    else
      LINFO("***** Could not identify attended object! *****");


  return cardName;
}

void trainCard(const Image<PixRGB<byte> > &img, const std::string &cardName)
{
  rutz::shared_ptr<VisualObject>
    vo(new VisualObject(cardName, "NULL", img,
          Point2D<int>(-1,-1),
          std::vector<double>(),
          std::vector< rutz::shared_ptr<Keypoint> >(),
          USECOLOR));

  itsObjectDB.addObject(vo, false);

  itsObjectDB.saveTo("cards.vdb");
}


int main(const int argc, const char **argv)
{
  MYLOGVERB = LOG_INFO;
  ModelManager *mgr = new ModelManager("Test ObjRec");

  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(*mgr));
  mgr->addSubComponent(ofs);

  nub::ref<InputFrameSeries> ifs(new InputFrameSeries(*mgr));
  mgr->addSubComponent(ifs);

  nub::ref<EnvSegmenterCannyContour> seg(new EnvSegmenterCannyContour(*mgr));
  mgr->addSubComponent(seg);

  mgr->exportOptions(MC_RECURSE);

  if (mgr->parseCommandLine(
        (const int)argc, (const char**)argv, "", 0, 0) == false)
    return 1;

  mgr->start();

  seg->setModelParamVal("CannyMinCos", 1.0);
  seg->setModelParamVal("CannyMaxArea", 6000);
  seg->setModelParamVal("CannyMaxArea", 12000);

  itsObjectDB.loadFrom("cards.vdb");
  while(1)
  {
    Image< PixRGB<byte> > inputImg;
    const FrameState is = ifs->updateNext();
    if (is == FRAME_COMPLETE)
      break;

    //grab the images
    GenericFrame input = ifs->readFrame();
    if (!input.initialized())
      break;
    inputImg = input.asRgb();

    Image<PixRGB<byte> > out;

    const Rectangle cardbox = seg->getFoa(inputImg, Point2D<int>(), NULL, &out);

    ofs->writeRGB(out, "input", FrameInfo("input", SRC_POS));

    if (cardbox.isValid())
    {
      Image<PixRGB<byte> > card =
        crop(inputImg, cardbox.getOverlap(inputImg.getBounds()));

      std::string cardName = recCard(card);

      if (cardName.length() == 0)
      {
        LINFO("Enter name for card:");
        std::getline(std::cin, cardName, '\n');

        if (cardName.length() > 0)
          trainCard(card, cardName);
      }

      writeText(card, Point2D<int>(0,0), cardName.c_str(),
          PixRGB<byte>(255), PixRGB<byte>(127));

      ofs->writeRGB(card, "card", FrameInfo("card", SRC_POS));
    }

    ofs->updateNext();
  }
  mgr->stop();

  return 0;

}

#endif
