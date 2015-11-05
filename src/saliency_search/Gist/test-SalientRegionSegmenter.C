/*!@file Gist/test-SalientRegionSegmenter.C test the Siagian Koch 2012cvpr 
   various shape estimator algorithm                                    */
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
// Primary maintainer for this file: Christian Siagian <siagian@caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/test-ContourBoundaryShapeEstimator.C $
// $Id: $
//
////////////////////////////////////////////////////////

#include "Component/ModelManager.H"
#include "Beobot/BeobotBrainMT.H"

#include "Media/FrameSeries.H"
#include "Transport/FrameIstream.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/Raster.H"
#include "GUI/XWinManaged.H"

#include "Image/DrawOps.H"
#include "Image/CutPaste.H"
#include "Image/ShapeOps.H"
#include "Image/FilterOps.H"
#include "Image/MathOps.H"

#include "Neuro/ShapeEstimator.H" 

#include <iostream>
#include <list>
#include <fstream>
#include <algorithm>

#define ALG_L_COL_RED      PixRGB<byte>(255,  0,  0)
#define ALG_L_COL_ORANGE   PixRGB<byte>(255,128,  0)
#define ALG_L_COL_YELLOW   PixRGB<byte>(255,255,  0)
#define ALG_L_COL_GREEN    PixRGB<byte>(  0,255,  0)
#define ALG_L_COL_CYAN     PixRGB<byte>(  0,255,255)
#define ALG_L_COL_MAGENTA  PixRGB<byte>(255,  0,255)
#define ALG_L_COL_BLUE     PixRGB<byte>(  0,  0,255)
#define ALG_L_COL_BROWN    PixRGB<byte>(139, 69, 19)

// compute the salient region
// using Siagian & Koch 2011 CBSE
void computeCBSE
(Image<PixRGB<byte> > image, bool saveOutput,
 std::string outputName, nub::ref<BeobotBrainMT> bbmt,
 rutz::shared_ptr<SalientRegionSegmenter> srs);

Point2D<int> correctSalientPointLocation
(Point2D<int> pt, Point2D<int> wci, Image<float> salMap);

//! debug window
rutz::shared_ptr<XWinManaged> win;

// ######################################################################
int main(const int argc, const char **argv)
{
  // instantiate a model manager:
  ModelManager manager("benchmark ShapeEstimator");

  // Instantiate our various ModelComponents:

  // get saliency
  nub::ref<BeobotBrainMT> bbmt(new BeobotBrainMT(manager));
  manager.addSubComponent(bbmt);

  rutz::shared_ptr<SalientRegionSegmenter> 
    srs(new SalientRegionSegmenter());

  nub::soft_ref<InputFrameSeries> ifs(new InputFrameSeries(manager));
  manager.addSubComponent(ifs);  

  nub::soft_ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);

  manager.exportOptions(MC_RECURSE);

  // Parse command-line:
  // benchmark name
  if (manager.parseCommandLine(argc, argv, "<outputFilename>", 0, 1) == false) return(1);

  bool saveOutput = false;
  std::string outFilename = "out";
  if(manager.numExtraArgs() >  0)
    {
      saveOutput = true;
      outFilename = manager.getExtraArg(0);
      LINFO("save to: %s", outFilename.c_str());
    }
 
  // let's do it!
  manager.start();

  ifs->updateNext();
  Image<PixRGB<byte> > image = ifs->readRGB();

  // Siagian&Koch 2011 Contour Boundary Shape Estimator
  computeCBSE(image, saveOutput, outFilename, bbmt, srs);
}

// ######################################################################
void computeCBSE
(Image<PixRGB<byte> > image, bool saveOutput, 
 std::string outputName, nub::ref<BeobotBrainMT> bbmt,
 rutz::shared_ptr<SalientRegionSegmenter> srs)
{
  uint width  = image.getWidth();
  uint height = image.getHeight();

  // compute saliency map
  bbmt->input(image); 
  while(!bbmt->outputReady()) usleep(1000);
  Image<float> salMap = bbmt->getSalMap();
              
  // get and correct location of salient point
  uint sw = salMap.getWidth();
  uint sh = salMap.getHeight();
            
  Image<float> currSalMap = salMap;
  Point2D<int> currwin; float maxval;
  findMax(currSalMap, currwin, maxval);  
  // get the winning channel information
  // RG: 0, BY: 1, I: 2, O0: 3, O45: 4, O90: 5, O135: 6
  Point2D<int> wci =  
    bbmt->getWinningChannelIndex(currwin);
  LINFO("wchan: %d, wschan: %d", wci.i, wci.j);
  
  // correct for it
  currwin = correctSalientPointLocation(currwin, wci, currSalMap);  
  currwin.i = int(currwin.i * width  / float(sw));
  currwin.j = int(currwin.j * height / float(sh));            
  srs->setImage(image);

  // while salient region 
  // does not land on already occupied location
  float cmval = maxval; uint nSalReg = 0; uint ntry = 0;
  Image<float> sreg = srs->getSalientRegion(currwin);
  while(ntry < 5)
    {
      LINFO("[%3d] %d %d", nSalReg, currwin.i, currwin.j);

      if(!sreg.initialized())
        {
          sreg = Image<float>(width, height, ZEROS);
          drawDisk(sreg, currwin, 2*4, 1.0F);
        }
      else if(saveOutput)
        {
          Image<byte> objMask = srs->getSalientRegionMask(nSalReg);
          Raster::WriteRGB
            (objMask, sformat("%s_objMask_%06d.png", outputName.c_str(), nSalReg));
          nSalReg++;
        }
      
      // suppress the previous salient region
      Image<float> sregMask = downSize(sreg, salMap.getDims());
      sregMask = convGauss<float>(sregMask,4,4,2);
      Image<float> supMask(sregMask);
      inplaceNormalize(supMask, 0.0F, 3.0F);
      inplaceClamp(supMask, 0.0F, 1.0F);
      supMask = (supMask * -1.0F) + 1.0F;
      currSalMap *= supMask;
      
      // display ////////////////////////////////////////////
      // win.reset
      //   (new XWinManaged(Dims(width*3,height*2), 0, 0, "bmCBSE"));
      //  Image<PixRGB<byte> > disp(width*3, height*2,ZEROS);
      //  inplacePaste(disp, image, Point2D<int>(0,0));
      //  Image<byte> tbSreg(sreg);
      //  inplacePaste(disp, makeRGB(tbSreg,tbSreg,tbSreg), 
      //               Point2D<int>(1*width,0));

      //  sregMask = zoomXY(sregMask, 4);
      //  inplaceNormalize(sregMask, 0.0F, 255.0F);
      //  Image<byte> timask(sregMask);
      //  inplacePaste(disp, makeRGB(timask,timask,timask), 
      //               Point2D<int>(2*width,0));    

      //  supMask = zoomXY(supMask, 4);
      //  inplaceNormalize(supMask, 0.0F, 255.0F);
      //  Image<byte> tsmask(supMask);
      //  inplacePaste(disp, makeRGB(tsmask,tsmask,tsmask), 
      //               Point2D<int>(0,height));

      //  Image<float> tsmap = zoomXY(salMap,4);
      //  inplaceNormalize(tsmap, 0.0F, 255.0F);                
      //  Image<byte> tbsmap(tsmap);
      //  inplacePaste(disp, makeRGB(tbsmap,tbsmap,tbsmap), 
      //               Point2D<int>(width,height));    
        
      //  Image<float> tcsmap = zoomXY(currSalMap,4);
      //  inplaceNormalize(tcsmap, 0.0F, 255.0F);                
      //  Image<byte> tbcsmap(tcsmap);
      //  inplacePaste(disp, makeRGB(tbcsmap,tbcsmap,tbcsmap), 
      //               Point2D<int>(2*width,height));    


      // win->drawImage(disp,0,0);
      // Raster::waitForKey();
      /////////////////////////////////////////////////////////

      // find next salient point
      findMax(currSalMap, currwin, cmval);

      // get the winning channel information
      // RG: 0, BY: 1, I: 2, O0: 3, O45: 4, O90: 5, O135: 6
      Point2D<int> wci =  
        bbmt->getWinningChannelIndex(currwin);
      LINFO("wchan: %d, wschan: %d", wci.i, wci.j);
                
      // correct for it
      currwin = correctSalientPointLocation
        (currwin, wci, currSalMap);
                
      currwin.i = int(currwin.i * width  / float(sw));
      currwin.j = int(currwin.j * height / float(sh));

      sreg = srs->getSalientRegion(currwin);
      ntry++;
    }

  // // display results
  // //drawCross(image, currwin, PixRGB<byte>(0, 0, 255), 2);
  // for(uint k = 0; k < bi.groundTruth.size();k++)
  //   drawRect(image, bi.groundTruth[k], PixRGB<byte>(255,0,0));
  // drawRect(image, srRect, PixRGB<byte>(0,255,0));
        
  // Image<PixRGB<byte> > dispImage(width*2, height*1,ZEROS);
  // inplacePaste(dispImage, image, Point2D<int>(0,0));
        
  // Image<byte> tbsr(srImage);
  // inplaceNormalize(tbsr, byte(0), byte(255));
  // inplacePaste(dispImage, makeRGB(tbsr,tbsr,tbsr), 
  //              Point2D<int>(width,0));

  // std::string dresfname =           
  //   tfname + std::string("_dres_") + tag + std::string(".png");
  // //Raster::WriteRGB(dispImage, outputFolder + '/' + dresfname);
  // Raster::WriteRGB(makeRGB(tbsr,tbsr,tbsr), outputFolder + '/' + dresfname);

  // win.reset
  //   (new XWinManaged(Dims(width*2,height*1), 0, 0, "bm SE"));
  // win->drawImage(dispImage,0,0);
  // Raster::waitForKey();
}

// ######################################################################
Point2D<int> correctSalientPointLocation
(Point2D<int> pt, Point2D<int> wci, Image<float> salMap)
{
  Point2D<int> ptCorr;

  // get channel name
  int chanName = wci.i;
  int scale    = wci.j;
  LINFO("chanName: %d, scale: %d", chanName, scale);

  Point2D<int> delta(0,0);

  // from salient point get most appropriate salient region 
  // for orientation saliency the border is salient, 
  // so we move the point perpendicular to the angle 
  if(chanName >= 3 && chanName <= 6)
    {
      // FIXXX: assume there's only 4 direction
      int dir = chanName - 3;

      Point2D<int> d1, d2;
      if(dir == 0)
        { d1 = Point2D<int>( 0, -1); d2 = Point2D<int>( 0,  1); }
      else if(dir == 1)
        { d1 = Point2D<int>( 1, -1); d2 = Point2D<int>(-1,  1); }
      else if(dir == 2)
        { d1 = Point2D<int>( 1,  0); d2 = Point2D<int>(-1,  0); }
      else if(dir == 3)
        { d1 = Point2D<int>( 1,  1); d2 = Point2D<int>(-1, -1); }
      d1 = d1*2;
      d2 = d2*2;
      Point2D<int> t1 = pt + d1;
      Point2D<int> t2 = pt + d2;

      // if points are in bounds and not yet assigned
      bool vt1 = (salMap.coordsOk(t1) && (t1.i != 0 || t1.j != 0) &&
                  salMap.getVal(t1) != 0);
      bool vt2 = (salMap.coordsOk(t2) && (t2.i != 0 || t2.j != 0) &&
                  salMap.getVal(t2) != 0);
      if(vt1)
        {
          if(!vt2) delta = d1;
          else
            {              
              // go to side with higher saliency
              // we might hit background and not object
              // either way we are reducing 
              // the number of unassigned regions
              float val1 = salMap.getVal(t1.i, t1.j);
              float val2 = salMap.getVal(t2.i, t2.j);
              if(val1 > val2) delta = d1; else delta = d2;

              LINFO("t1[%3d %3d] t2[%3d %3d] "
                    "v1:%f v2:%f", 
                    t1.i, t1.j, t2.i, t2.j, val1, val2);

            }
        } else if(vt2) delta = d2;

      // if both sides are assigned, 
      // we are in the crack between two salient regions
      // should have been corrected 
    }

  // FIXXX: need to incorporate scale???
  ptCorr = pt + delta;
  LINFO("pt: %3d %3d delta: %3d %3d --> corrected point: %d %d", 
        pt.i, pt.j, delta.i, delta.j, ptCorr.i, ptCorr.j);

  return ptCorr;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
