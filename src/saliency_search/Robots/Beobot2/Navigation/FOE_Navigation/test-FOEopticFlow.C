
/*!@file Robots/Beobot2/Navigation/FOE_Navigation/test-FOEopticFlow.C
  find the optic flow (Shi Tomasi) and the focus of expansion (Perrone 1992) */
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
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: $
// $Id: $

//

#define MAX_CORNERS        500   

// OpenCV must be first to avoid conflicting defs of int64, uint64
#include "Image/OpenCVUtil.H"

#include <cstdio>

#include "Component/ModelManager.H"
#include "Media/FrameSeries.H"

#include "Image/Image.H"
#include "Image/CutPaste.H"
#include "Image/ColorOps.H"
#include "Image/MathOps.H"
#include "Image/ShapeOps.H"
#include "Image/Pixels.H"
#include "Raster/Raster.H"

#include "SIFT/Keypoint.H"
#include "SIFT/VisualObject.H"
#include "SIFT/VisualObjectMatch.H"


#include "Util/Timer.H"

#include "Robots/Beobot2/Navigation/FOE_Navigation/FoeDetector.H"
#include "Robots/Beobot2/Navigation/FOE_Navigation/MotionOps.H"

// return the optic flow
//Image<float> getOpticFlow(Image<byte> image1, Image<byte> image2);
Image<float> getOpticFlow2(Image<float> image1, Image<float> image2);

// get the ground truth
std::vector<Point2D<int> > getGT(std::string gtFilename);

Image<byte>  calculateShift
(Image<byte> prevLum, Image<byte> lum, nub::ref<OutputFrameSeries> ofs);

Image<byte> shiftImage(SIFTaffine aff, Image<byte> ref, Image<byte> tst);

std::vector<std::vector<Point2D<int> > > getGTandData();

// ######################################################################
int main(const int argc, const char **argv)
{
  MYLOGVERB = LOG_INFO;  // suppress debug messages

  // instantiate a model manager:
  ModelManager manager("Test Optic Flow");

  nub::ref<InputFrameSeries> ifs(new InputFrameSeries(manager));
  manager.addSubComponent(ifs);

  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);

  nub::ref<FoeDetector> fd(new FoeDetector(manager));
  manager.addSubComponent(fd);

  manager.exportOptions(MC_RECURSE);

  if (manager.parseCommandLine((const int)argc, (const char**)argv,
                               "", 0, 0) == false)
    return(1);

  // get some options
  // if(manager.numExtraArgs() >  0)
  //   {
  //     outFilename = manager.getExtraArg(0);
  //     LINFO("save to: %s", outFilename.c_str());
  //   }

  // get ground truth file 
  //std::string gtFilename
  //  ("/lab/tmpib/u/siagian/neuroscience/Data/FOE/driving_nat_3.txt");
  //std::vector<Point2D<int> > gt = getGT(gtFilename);
  //int ldpos = gtFilename.find_last_of('.');
  //std::string prefix = gtFilename.substr(0, ldpos);

  std::string pprefix
    ("/lab/tmpib/u/siagian/neuroscience/Data/FOE/driving_nat_3");

  std::string folder("/lab/tmpib/u/siagian/neuroscience/Data/FOE/");
  std::string ciofmPrefix("DARPA_nv2_2011/Germany_K_CIOFM/SalMap_CIOFM");
  std::string ciofsPrefix("DARPA_nv2_2011/Germany_K_CIOFs/SalMap_CIOFs");

  // let's do it!
  manager.start();

  Timer timer(1000000);
  timer.reset();  // reset the timer
  int frame = 0;
  bool keepGoing = true;

  const FrameState is = ifs->updateNext();
  Image< PixRGB<byte> > prevImage;
  if (is == FRAME_COMPLETE) keepGoing = false;
  else prevImage = ifs->readRGB();

  uint width  = prevImage.getWidth();
  uint height = prevImage.getHeight();

  // for finding ground truth
  rutz::shared_ptr<XWinManaged> win
    (new XWinManaged(Dims(width, height), 0, 0, "GT"));

  // GROUND TRUTH 1
  //float totalErr = 0.0;
  // Image<PixRGB<byte> > d1 =prevImage;
  // win->drawImage(d1,0,0);
  // LINFO("please click for initial FOE location");
  // Raster::waitForKey();
  // Point2D<int> pt1 = win->getLastMouseClick();
  // LINFO("initial point: [%3d]: %d %d", frame, pt1.i, pt1.j);
  // drawCross(d1, pt1, PixRGB<byte>(255,0,0), 10, 2);
  // win->drawImage(d1,0,0);
  // Raster::waitForKey();
  // Point2D<int> lastPt = pt1;

  //std::vector<std::vector<Point2D<int> > > foePts =
  //  getGTandData();
  //Raster::waitForKey();
  // std::vector<PixRGB<byte> > colors(foePts.size());
  // colors[0] = PixRGB<byte>(255,   0,   0);
  // colors[1] = PixRGB<byte>(  0, 255,   0);
  // colors[2] = PixRGB<byte>(  0,   0, 255);
  // colors[3] = PixRGB<byte>(255, 255,   0);

  Image<byte> prevLum = Image<byte>(luminance(prevImage));
  win->setDims(Dims(640,720));
  while (keepGoing)
    {
      if (ofs->becameVoid())
        {
          LINFO("quitting because output stream was closed or became void");
          break;
        }

      const FrameState is = ifs->updateNext();
      if (is == FRAME_COMPLETE) break; // done receiving frames

      Image< PixRGB<byte> > currImage = ifs->readRGB();
      Image<byte> lum = Image<byte>(luminance(currImage));

      // saliency map result
     
      Image< PixRGB<byte> > ciofm = Raster::ReadRGB
        (folder+sformat("%s_%06d.ppm", ciofmPrefix.c_str(), frame));
      Image< PixRGB<byte> > ciofs = Raster::ReadRGB
        (folder+sformat("%s_%06d.ppm", ciofsPrefix.c_str(), frame));

      Image< PixRGB<byte> > salImg(640,720,ZEROS);
      inplacePaste(salImg, zoomXY(prevImage,2), Point2D<int>(0,0));
      inplacePaste(salImg, ciofm, Point2D<int>(0  ,480));
      inplacePaste(salImg, ciofs, Point2D<int>(320,480));

      drawLine(salImg, Point2D<int>(0,480), Point2D<int>(640,480), 
               PixRGB<byte>(255,255,255), 3);
      drawLine(salImg, Point2D<int>(320,480), Point2D<int>(320,720), 
               PixRGB<byte>(255,255,255), 3);

      win->drawImage(salImg,0,0); 
      Raster::waitForKey();
    
      Raster::WriteRGB
        (salImg, sformat("%s_res_%06d.ppm", pprefix.c_str(), frame));

      Raster::waitForKey();

      // =====================================================
      // Ground truthing
      // Image< PixRGB<byte> > dispGT = prevImage;
      // //drawCross(dispGT, gt[frame], PixRGB<byte>(0,255,0), 10, 2);
      // //LINFO("[%3d]: %d %d", frame, gt[frame].i, gt[frame].j);
      // //win->drawImage(dispGT,0,0);
      // //Raster::waitForKey();

      // Point2D<int> pt = win->getLastMouseClick();
      // while(!pt.isValid()) 
      //   pt = win->getLastMouseClick();
      // if(pt.isValid() && !(pt == lastPt)) lastPt = pt;

      // LINFO("[%3d]: pt %d %d", frame, pt.i, pt.j);
      // drawCross(dispGT, lastPt, PixRGB<byte>(255,0,0), 10, 2);
      // win->drawImage(dispGT,0,0);

      // FILE *gtfp; 
      // LINFO("[%3d] gt file: %s: %d %d", 
      //       frame, gtFilename.c_str(), lastPt.i, lastPt.j);
      // if((gtfp = fopen(gtFilename.c_str(),"at")) == NULL)
      //   LFATAL("not found");
      // fputs(sformat("%d %d \n", lastPt.i, lastPt.j).c_str(), gtfp);
      // fclose (gtfp);


      // for performance display
      // Image<PixRGB<byte> > pimg = prevImage;
      // for(uint i = 0; i < foePts.size(); i++)
      //   {
      //     if(!(foePts[i][frame].i == -1 && foePts[i][frame].j == -1)) 
      //       drawCross(pimg, foePts[i][frame], colors[i], 10, 2);
      //   }
      // Raster::WriteRGB(pimg, sformat("%s_res_%06d.ppm", pprefix.c_str(), frame));
      // ofs->writeRGB(pimg, "Performance");
      // Raster::waitForKey();

      // =====================================================

      // calculate planar shift using SIFT 
      //lum = calculateShift(prevLum, lum, ofs);

      // compute the optic flow
      //rutz::shared_ptr<OpticalFlow> flow =
      //  getLucasKanadeOpticFlow
      //(Image<byte>(luminance(prevImage)), 
      //Image<byte>(luminance(currImage)));

      // rutz::shared_ptr<OpticalFlow> flow =
      //   getLucasKanadeOpticFlow(prevLum, lum);
      // Image<PixRGB<byte> > opticFlow = drawOpticFlow(prevImage, flow);
      // Image< PixRGB<byte> > disp(4*width, height, ZEROS);
      // inplacePaste(disp, prevImage, Point2D<int>(0    ,0));
      // inplacePaste(disp, currImage, Point2D<int>(width,0));

      // // compute the focus of expansion (FOE)
      // Point2D<int> foe = 
      //   fd->getFoe(flow, FOE_METHOD_AVERAGE, false);
      // //fd->getFoe(flow, FOE_METHOD_TEMPLATE);
      // float err = foe.distance(gt[frame]); 
      // totalErr += err;
      // LINFO("Foe: %d %d: GT: %d %d --> %f --> avg: %f", 
      //       foe.i, foe.j, gt[frame].i, gt[frame].j, 
      //       err, totalErr/(frame+1));


      // drawCross(opticFlow, foe      , PixRGB<byte>(0,255,0), 10, 2);
      // //drawCross(opticFlow, gt[frame], PixRGB<byte>(255,0,0), 10, 2);
      // Image<float> foem = fd->getFoeMap();

      // inplacePaste(disp, opticFlow, Point2D<int>(width*2,0));
      // inplaceNormalize(foem, 0.0F, 255.0F);
      // inplacePaste(disp, toRGB(zoomXY(Image<byte>(foem),MAX_NEIGHBORHOOD)), 
      //              Point2D<int>(width*3,0));
      // ofs->writeRGB(disp, "Optic Flow");
      // const FrameState os = ofs->updateNext();

      // // write the file
      // //Image<PixRGB<byte> > simg = prevImage;
      // //drawCross(simg, foe      , PixRGB<byte>(0,255,0), 10, 2);
      // //drawCross(simg, gt[frame], PixRGB<byte>(255,0,0), 10, 2);
      // //Raster::WriteRGB(simg, sformat("%s_%06d.ppm", prefix.c_str(), frame));

      // //Raster::waitForKey();

      // if (os == FRAME_FINAL)
      //   break;

      prevImage = currImage;
      prevLum   = lum;
      frame++;

      //FILE *fp;      
      //LINFO("FOE %d %d", foe.i, foe.j);
      //if((fp = fopen("LK_CB_Germany.txt","at")) == NULL) LFATAL("not found");
      //fputs(sformat("%d %d \n", foe.i, foe.j).c_str(), fp);
      //fclose (fp);  

    }

  //LINFO("%d frames in %gs (%.2ffps)\n", 
  //      frame, timer.getSecs(), frame / timer.getSecs());

  // stop all our ModelComponents
  manager.stop();

  // all done!
  return 0;

}

// ######################################################################
std::vector<std::vector<Point2D<int> > > getGTandData()
{
  uint nSystems = 3;
  std::vector<std::vector<Point2D<int> > > data(nSystems+1);

  std::string folder("/lab/tmpib/u/siagian/neuroscience/Data/FOE/");

  // Browning
  // std::string gt("driving_nat_Browning_2.txt");
  // std::string f1("DARPA_nv2_2011/Browning_K_CIOFs/ST_PS_Browning.txt");
  // std::string f2("DARPA_nv2_2011/Browning_K_CIOFs/BMG_Browning.txt");
  // std::string f3("DARPA_nv2_2011/Browning_K_CIOFs/LK_CB_Browning.txt");

  // Las Vegas
  // std::string gt("driving_nat_2b_1.txt");
  // std::string f1("DARPA_nv2_2011/LasVegas_K_CIOFs/ST_PS_LasVegas.txt");
  // std::string f2("DARPA_nv2_2011/LasVegas_K_CIOFs/BMG_LasVegas.txt");
  // std::string f3("DARPA_nv2_2011/LasVegas_K_CIOFs/LK_CB_LasVegas.txt");
  
  // Germany  
  std::string gt("driving_nat_3_1.txt");
  std::string f1("DARPA_nv2_2011/Germany_K_CIOFs/ST_PS_Germany.txt");
  std::string f2("DARPA_nv2_2011/Germany_K_CIOFs/BMG_Germany.txt");
  std::string f3("DARPA_nv2_2011/Germany_K_CIOFs/LK_CB_Germany.txt");


  // get data
  data[0] = getGT(folder+gt);
  data[1] = getGT(folder+f1);
  data[2] = getGT(folder+f2);
  data[3] = getGT(folder+f3);

  // calculate errors as well
  for(uint i = 1; i < nSystems+1; i++) 
    {
      float totalErr = 0.0;
      uint totalPt = 0;
      for(uint j = 0; j < data[i].size(); j++)
        {
          // if systems is not ready 
          // just skip the point
          if(data[i][j].i == -1 && data[i][j].j == -1)
            continue;
              
          // if point is out of bounds just move it to the closest edge
          // also change the data
          Point2D<int> newPt = data[i][j];          
          if(newPt.i <    0) newPt.i =   0;
          if(newPt.i >= 320) newPt.i = 319;
          if(newPt.j <    0) newPt.j =   0;
          if(newPt.j >= 320) newPt.j = 319;
          data[i][j] = newPt;
          
          // compare with ground truth
          float err = newPt.distance(data[0][j]);
          
          LDEBUG("[%3d %3d] - [%3d %3d]: %f",
                 newPt.i, newPt.j, data[0][j].i, data[0][j].j,
                 err);
  
          totalErr += err; 
          totalPt++;
        }
 
      if(totalPt == 0) LINFO("s[%3d] err: 0/0 = 0", i);
      else
        LINFO("s[%3d] err: %f/%d = %f", 
              i, totalErr, totalPt, totalErr/totalPt);
    }
  return data;
}

// ######################################################################
std::vector<Point2D<int> > getGT(std::string gtFilename)
{
  std::vector<Point2D<int> > gt;

  FILE *fp;  char inLine[200]; //char comment[200];

  LINFO("ground truth file: %s",gtFilename.c_str());
  if((fp = fopen(gtFilename.c_str(),"rb")) == NULL)
    { LINFO("not found"); return gt; }

  // populate the trial information
  uint nFrames = 0;
  while(fgets(inLine, 200, fp) != NULL)
    {
      int x, y;
      sscanf(inLine, "%d %d", &x, &y);      
      LINFO("[%3d] x: %d y: %d", nFrames, x, y);
      gt.push_back(Point2D<int>(x,y));

      nFrames++;
    }
  Raster::waitForKey();
  return gt;
}

// ######################################################################
Image<float> getOpticFlow2(Image<float> image1, Image<float> image2)
{

  // Load two images and allocate other structures
  inplaceNormalize(image1, 0.0F, 255.0F);
  Image<byte> img1(image1);
  IplImage* imgA = img2ipl(img1); 
  inplaceNormalize(image2, 0.0F, 255.0F);
  Image<byte> img2(image2);
  IplImage* imgB = img2ipl(img2);

  CvSize img_sz = cvGetSize( imgA );
  int win_size = 15;
  
  IplImage* imgC =img2ipl(img1);

  // Get the features for tracking
  IplImage* eig_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
  IplImage* tmp_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
  
  int corner_count = MAX_CORNERS;
  CvPoint2D32f* cornersA = new CvPoint2D32f[ MAX_CORNERS ];
  
  cvGoodFeaturesToTrack( imgA, eig_image, tmp_image, 
                         cornersA, &corner_count,
			 0.05, 5.0, 0, 3, 0, 0.04 );
  

  cvFindCornerSubPix( imgA, cornersA, corner_count, 
                      cvSize( win_size, win_size ),
		      cvSize( -1, -1 ), 
                      cvTermCriteria( CV_TERMCRIT_ITER | 
                                      CV_TERMCRIT_EPS, 20, 0.03 ) );
  
  // Call Lucas Kanade algorithm
  char features_found[ MAX_CORNERS ];
  float feature_errors[ MAX_CORNERS ];
  
  CvSize pyr_sz = cvSize( imgA->width+8, imgB->height/3 );
  
  IplImage* pyrA = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
  IplImage* pyrB = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
  
  CvPoint2D32f* cornersB = new CvPoint2D32f[ MAX_CORNERS ];
  
  cvCalcOpticalFlowPyrLK( imgA, imgB, pyrA, pyrB, 
                          cornersA, cornersB, corner_count, 
			  cvSize( win_size, win_size ), 5, 
                          features_found, feature_errors,
			  cvTermCriteria( CV_TERMCRIT_ITER | 
                                          CV_TERMCRIT_EPS, 20, 0.3 ), 0 );
  
  // Make an image of the results
  
  for( int i=0; i < MAX_CORNERS; i++ )
    {
      LINFO("Error is %f/n", feature_errors[i]);
      //continue;

      //printf("Got it/n");
      CvPoint p0 = cvPoint( cvRound( cornersA[i].x ), 
                            cvRound( cornersA[i].y ) );
      CvPoint p1 = cvPoint( cvRound( cornersB[i].x ), 
                            cvRound( cornersB[i].y ) );
      cvLine( imgC, p0, p1, CV_RGB(255,0,0), 2 );
    }

  return ipl2gray(imgC);
}

// ######################################################################
Image<byte> calculateShift(Image<byte> prevLum, Image<byte> lum,
                           nub::ref<OutputFrameSeries> ofs)
{
  VisualObjectMatchAlgo voma(VOMA_SIMPLE);
//   if (strcmp(argv[1], "KDTree") == 0) voma = VOMA_KDTREE;
//   else if (strcmp(argv[1], "KDBBF") == 0) voma = VOMA_KDTREEBBF;
//   else if (strcmp(argv[1], "Simple") != 0)
//     LFATAL("Unknown matching method %s", argv[0]);

  // create visual objects:
  rutz::shared_ptr<VisualObject> vo1(new VisualObject("plum", "", prevLum));
  rutz::shared_ptr<VisualObject> vo2(new VisualObject("lum" , "", lum));

  // compute the matching keypoints:
  VisualObjectMatch match(vo1, vo2, voma);
  LDEBUG("Found %u matches", match.size());

  // let's prune the matches:
  uint np = match.prune();
  LDEBUG("Pruned %u outlier matches.", np);

  // show our final affine transform:
  SIFTaffine s = match.getSIFTaffine();
  LDEBUG("[tstX]   [ %- .3f %- .3f ] [refX]   [%- .3f]", s.m1, s.m2, s.tx);
  LDEBUG("[tstY] = [ %- .3f %- .3f ] [refY] + [%- .3f]", s.m3, s.m4, s.ty);


  LDEBUG("getKeypointAvgDist = %f", match.getKeypointAvgDist());
  LDEBUG("getAffineAvgDist = %f",   match.getAffineAvgDist());
  LDEBUG("getScore = %f", match.getScore());

  if (match.checkSIFTaffine() == false)
    LINFO("### Affine is too weird -- BOGUS MATCH");

  // get an image showing the matches:
  Image< PixRGB<byte> > mimg = match.getMatchImage(1.0F);
  Image< PixRGB<byte> > fimg = match.getFusedImage(0.25F);

  // LINFO("prevLum");
  // ofs->writeRGB
  //   (toRGB(prevLum), "test-FOE Main", FrameInfo("foe output", SRC_POS));
  // Raster::waitForKey();

  // LINFO("lum");
  // ofs->writeRGB
  //   (toRGB(lum), "test-FOE Main", FrameInfo("foe output", SRC_POS));
  // Raster::waitForKey();

  LINFO("Shift: %f %f", s.tx, s.ty);
  // ofs->writeRGB
  //   (fimg, "test-FOE Main", FrameInfo("foe output", SRC_POS));
  // Raster::waitForKey();

  Image<byte> res = shiftImage(s, prevLum, lum);

  // LINFO("shifted result");
  // ofs->writeRGB
  //   (toRGB(res), "test-FOE Main", FrameInfo("foe output", SRC_POS));
  // Raster::waitForKey();

  //Point2D<float> shift(s.tx,s.ty);
  return res;
}

// ######################################################################
 Image<byte> shiftImage(SIFTaffine aff, Image<byte> refi, Image<byte> tsti)
{
  // we loop over all pixel locations in the ref image, transform the
  // coordinates using the forward affine transform, get the pixel
  // value in the test image, and mix:

  uint w = refi.getWidth(), h = refi.getHeight();
  Image<byte> result(w, h, ZEROS);
  Image<byte>::const_iterator rptr = refi.begin();
  Image<byte>::iterator dptr = result.beginw();

  for (uint j = 0; j < h; j ++)
    for (uint i = 0; i < w; i ++)
      {
        float u, v;
        aff.transform(float(i), float(j), u, v);
        byte rval = *rptr++;

        if (tsti.coordsOk(u, v))
          {
            byte tval = tsti.getValInterp(u, v);
            //PixRGB<byte> mval = PixRGB<byte>(rval * mix + tval * (1.0F - mix));
            *dptr++ = tval;
          }
        else
          //*dptr++ = PixRGB<byte>(rval * mix);
          *dptr++ = byte(rval);
      }

  return result; 
 }

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
