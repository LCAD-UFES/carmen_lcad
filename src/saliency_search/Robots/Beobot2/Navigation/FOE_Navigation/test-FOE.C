/*!@file Robots/Beobot2/Navigation/FOE_Navigation/test-FOE.C 
  find the focus of expansion */
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

#include "Component/ModelManager.H"
#include "Component/OptionManager.H"
#include "Devices/FrameGrabberConfigurator.H"

#include "Media/FrameSeries.H"

#include "Image/ColorOps.H"
#include "Image/ShapeOps.H"
#include "Image/Image.H"
#include "Image/Layout.H"
#include "Raster/Raster.H"

#include "SIFT/Keypoint.H"
#include "SIFT/VisualObject.H"
#include "SIFT/VisualObjectMatch.H"

#include "Transport/FrameInfo.H"
#include "Util/Pause.H"
#include "Util/Timer.H"
#include "Util/Types.H"
#include "Util/csignals.H"
#include "Util/log.H"
#include <math.h>
#include <cstdio>
#include "Robots/Beobot2/Navigation/FOE_Navigation/FoeDetector.H"


#define DOT_NUM          200           // 200  1000

#define WIDTH            320           // 320  640
#define HEIGHT           240           // 240  480
#define FOE_X            2*WIDTH/4     // W/4 
#define FOE_Y            2*HEIGHT/4    // 2H/4 +4 

#define DOT_VEL          2.0/80.0      // Shuo: .035


// Shuo: in PsychToolbox: size refers to diameter, in our code radius
// Shuo: Abs max: 21, ave DS is 9.767 across all dots in each frames
#define DOT_ORG_DSIZE    .02    // .04
#define DOT_DSIZE        .07    // .035
#define MIN_DOT_SIZE     1   
#define MAX_DOT_SIZE     5   
#define ABS_MAX_DSIZE    50
#define NFRAME           60

#define HAVE_MOTION      1
#define HAVE_TEMP_SGRAD  1
#define HAVE_SPAT_SGRAD  1

#define NUM_PYR_LEVEL    2 // 3 for 640x480
#define NUM_DIRS         8
#define NUM_SPEEDS       3

// get the ground truth from a text file
std::vector<Point2D<int> > getGT(std::string gtFilename);

Image<byte>  calculateShift
(Image<byte> lum, Image<byte> prevLum, nub::ref<OutputFrameSeries> ofs);

// get the image for stimuli
Image<byte> getFoeDots
(uint step, bool haveMotion, bool  haveTempSGrad, bool haveSpatSGrad,
 float dx, float dy, float dotOrgDSize);

Image<byte> getPlanarMotionStimuli
(Image<byte> temp, uint step, float dx, float dy);

Image<byte> getBarStimuli(uint step);
Image<byte> getShapeStimuli(uint step);
Image<byte> getApertureProblemStimuli(uint step);

// this is stimuli like in Fukuchi,Tsuchiya,Koch VSS/JOV 2009/2010
// FOE is FOE_X,FOE_Y
Image<byte> getCleanFOE
(uint step, uint totalStep, uint mag, float dx, float dy, Image<byte> image);

Image<byte> getPlanarMotionImage
(uint step, uint totalStep, float dx, float dy, Image<byte> image);

Image<byte> shiftImage(SIFTaffine aff, Image<byte> ref, Image<byte> tst);

Image<byte> getImage
(std::string stimuli, std::vector<std::string> args, 
 nub::ref<FoeDetector> fd, uint step);

// ######################################################################
std::vector<Point2D<float> > dots;
std::vector<float> dotSizes;

std::vector<Point2D<float> > pdots;
std::vector<float> pdotSizes;

// ######################################################################
int main(const int argc, const char **argv)
{
  MYLOGVERB = LOG_INFO;  // suppress debug messages

  volatile int signum = 0;
  catchsignals(&signum);

  ModelManager manager("Test Motion Energy");

  nub::ref<InputFrameSeries> ifs(new InputFrameSeries(manager));
  manager.addSubComponent(ifs);

  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);

  nub::ref<FoeDetector> fd(new FoeDetector(manager));
  manager.addSubComponent(fd);

  if (manager.parseCommandLine((const int)argc, (const char**)argv,
                               "<stimuli> <options>", 0, 9) == false)
    return(1);

  fd->reset(NUM_PYR_LEVEL, NUM_DIRS, NUM_SPEEDS);

  std::string stimuli("Image");
  if(manager.numExtraArgs() > 0)
    stimuli = manager.getExtraArgAs<std::string>(0);
  LINFO("Stimuli: %s", stimuli.c_str());

  manager.start();

  Timer timer(1000000);
  timer.reset();  // reset the timer
  int frame = 0;

  PauseWaiter p;

  uint step; step = 0;

  // to get to the good part
  //for(uint i = 0; i < 50; i++) //was 25
  //  ifs->updateNext();

  // get ground truth file 
  std::string gtFilename
    ("/lab/tmpib/u/siagian/neuroscience/Data/FOE/driving_nat_Browning.txt");
  std::vector<Point2D<int> > gt = getGT(gtFilename);
  int ldpos = gtFilename.find_last_of('.');
  std::string prefix = gtFilename.substr(0, ldpos);

  // for finding ground truth
  rutz::shared_ptr<XWinManaged> win;
  
  float totalErr = 0.0;

  std::vector<std::string> args;
  for(uint i = 0; i < manager.numExtraArgs(); i++)
    args.push_back(manager.getExtraArgAs<std::string>(i)); 

  Image<byte> prevLum;
  Image<PixRGB<byte> > prevImage;
  Image<PixRGB<byte> > prevImage2;
  while (1)
    {
      if (signum != 0)
        {
          LINFO("quitting because %s was caught", signame(signum));
          break;
        }

      if (ofs->becameVoid())
        {
          LINFO("quitting because output stream was closed or became void");
          break;
        }

      if (p.checkPause())
        continue;

      const FrameState is = ifs->updateNext();
      if (is == FRAME_COMPLETE) break; // done receiving frames

      Image< PixRGB<byte> > input = ifs->readRGB();
      if(frame == 0) 
        {
          uint width  = input.getWidth();
          uint height = input.getHeight();
          win.reset(new XWinManaged(Dims(width, height), 0, 0, "GT"));
        }

      // empty image signifies end-of-stream
      if (!input.initialized()) break;
      Image<byte> lum = luminance(input);
      Point2D<float> pshift(0.0,0.0); 
      if(step != 0)
        {
          // calculate planar shift using SIFT 
          lum = calculateShift(lum,prevLum, ofs);
        }
      if( manager.numExtraArgs() > 0)
        lum = getImage(stimuli, args, fd, step);

      // for saving videos
      prevImage2 = prevImage;
      prevImage  = input;

      if (!lum.initialized()) break; step++;

      // compute the focus of expansion (FOE)
      Point2D<int> foe = fd->getFoe(lum, FOE_METHOD_TEMPLATE, false);
      //Point2D<int> foe = fd->getFoe(lum, FOE_METHOD_AVERAGE);
      LINFO("[%d]Foe: %d %d", frame, foe.i, foe.j);

      // illustration of the size of the receptive field
      if(!stimuli.compare("ShowRF"))
        {
          uint rfI = 44;
          uint rfJ = 152;
          lum.setVal(rfI, rfJ, 300.0F);      
          drawRect(lum, Rectangle::tlbrI(144,36,159,51), byte(255));
          drawRect(lum, Rectangle::tlbrI(148,40,155,47), byte(255));
          
          drawRect(lum, Rectangle::tlbrI(rfJ-8, rfI-8, rfJ+8, rfI+8), byte(255));
          drawRect(lum, Rectangle::tlbrI(rfJ-16,rfI-16,rfJ+16,rfI+16), byte(255));
        }

      ofs->writeGrayLayout(fd->getMTfeaturesDisplay(lum), "MT Features",
                           FrameInfo("motion energy output images", SRC_POS));

      // write the file
      if(frame >= 4)
        {
          float err = foe.distance(gt[frame-2]); 
          totalErr += err;
          LINFO("Foe: %d %d: GT: %d %d --> %f --> avg: %f", 
                foe.i, foe.j, gt[frame-2].i, gt[frame-2].j, 
                err, totalErr/(frame-3));

          Image<PixRGB<byte> > simg = prevImage2;
          drawCross(simg, foe        , PixRGB<byte>(0,255,0), 10, 2);
          drawCross(simg, gt[frame-2], PixRGB<byte>(255,0,0), 10, 2);
          win->drawImage(simg,0,0);
          //Raster::WriteRGB(simg, sformat("%s_STnPS_%06d.ppm", prefix.c_str(), frame-2));
        }

      //ofs->writeGrayLayout
      //  (lum, "test-FOE Main", FrameInfo("foe output", SRC_POS));
      const FrameState os = ofs->updateNext();
      //LINFO("frame[%d]: %8.3f %8.3f", frame, pshift.i, pshift.j); 
      Raster::waitForKey();

      if (os == FRAME_FINAL)
        break;

      prevLum  = lum;
      frame++;
    }

  LINFO("%d frames in %gs (%.2ffps)\n", 
        frame, timer.getSecs(), frame / timer.getSecs());

  // stop all our ModelComponents
  manager.stop();

  // all done!
  return 0;

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
      LDEBUG("[%3d] x: %d y: %d", nFrames, x, y);
      gt.push_back(Point2D<int>(x,y));

      nFrames++;
    }
  return gt;
}

// ######################################################################
Image<byte> getImage
  (std::string stimuli, std::vector<std::string> args, 
   nub::ref<FoeDetector> fd, uint step)
{
  Image<byte> lum;

  // Different stimuli:
  if(!stimuli.compare("Image")) 
    {
      float di =  0.0; float dj = 0.0;
      //float di =  -3.6; float dj = -.6; // first frame ACB
      if(args.size() > 2)
        {
          di = atof(args[1].c_str()); 
          dj = atof(args[2].c_str()); 
          //fd->setObserverRotation(di, dj);
        }
      LDEBUG("di: %f, dj: %f", di, dj);
    }
  else if(!stimuli.compare("BarStimuli"))
    lum = getBarStimuli(step); 
  else if(!stimuli.compare("ApertureProblem"))
    lum = getApertureProblemStimuli(step);
  else if(!stimuli.compare("CleanFoe"))
    {
      uint total = 30;  // 15 for the Fukuchi, et. al. paper
      if(args.size() > 1)
        total = uint(atoi(args[1].c_str()));
      float dx = 0.0;
      float dy = 0.0;
      if(args.size() > 3)
        {
          dx = atof(args[2].c_str());  
          dy = atof(args[3].c_str()); 
        }
      lum = getCleanFOE(step, total, 2.0, dx, dy, lum);
      
      //           float di =  0.0; float dj = 0.0;
      //           if(manager.numExtraArgs() > 5)
      //             {
      //               di = manager.getExtraArgAs<float>(4); 
      //               dj = manager.getExtraArgAs<float>(5); 
      //               fd->setObserverRotation(di, dj);
      //             }
      // planar (Yaw-Pitch) motion correction
      uint dir =  0.0; float speed = 0.0;
      if(args.size() > 5)
        {
          dir   = uint(atoi(args[4].c_str())); 
          speed = atof(args[5].c_str()); 
          fd->setObserverRotation(dir, speed);
        }
    }
  else if(!stimuli.compare("FoeDots"))      
    {
      bool haveMotion    = HAVE_MOTION;
      bool haveTempSGrad = HAVE_TEMP_SGRAD;
      bool haveSpatSGrad = HAVE_SPAT_SGRAD;
      if(args.size() > 5)
        {
          haveMotion    = bool(atoi(args[1].c_str())); 
          haveTempSGrad = bool(atoi(args[2].c_str())); 
          haveSpatSGrad = bool(atoi(args[3].c_str())); 
        }
      
      float dx = 0.0; float dy = 0.0;
      //float dx = pshift.i; float dy = pshift.j;
      if(args.size() > 5)
        {
          dx = atof(args[4].c_str()); 
          dy = atof(args[5].c_str()); 
        }
      
      float dotOrgDSize = DOT_ORG_DSIZE; 
      if(args.size() > 6)
        {
          dotOrgDSize =  atof(args[6].c_str());
        }        
      
      // planar (Yaw-Pitch) motion correction
      uint dir =  0.0; float speed = 0.0;
      if(args.size() > 8)
        {
          dir   = uint(atoi(args[7].c_str()));
          speed = atof(args[8].c_str()); 
          fd->setObserverRotation(dir, speed);
        }
      
      LINFO("[%3d] haveMotion: %d haveTempSGrad: %d haveSpatSGrad: %d "
            "dx dy: %7.3f %7.3f dotOrgDSize: %f comp: %d %f",
            step, haveMotion, haveTempSGrad, haveSpatSGrad, 
            dx, dy, dotOrgDSize, dir, speed);
      lum = getFoeDots(step, haveMotion, haveTempSGrad, haveSpatSGrad, 
                       dx, dy, dotOrgDSize);
      
          //float px = 2.0; float py = 0.0;
//           if(manager.numExtraArgs() > 2)
//             {
//               dx = manager.getExtraArgAs<float>(1); 
//               dy = manager.getExtraArgAs<float>(2); 
//             }
          //lum = getPlanarMotionStimuli(lum, step, px, py);
    }
  else if(!stimuli.compare("PlanarDots"))
    {
      Image<byte> temp(WIDTH, HEIGHT, ZEROS);
      
      float dx = 0.0; float dy = 0.0;
      if(args.size() > 2)
        {
          dx = atof(args[1].c_str()); 
          dy = atof(args[2].c_str()); 
        }
      lum = getPlanarMotionStimuli(temp, step, dx, dy);
    }
  else if(!stimuli.compare("PlanarImage")) 
    {
      uint total = 30;  // 15 for the Fukuchi, et. al. paper
      float dx =  0.0; float dy = 0.0;
      if(args.size() > 2)
        {
          dx = atof(args[1].c_str()); 
          dy = atof(args[2].c_str()); 
        }
      lum = getPlanarMotionImage(step, total, dx, dy, lum);
    }
  else LFATAL("Wrong option");

  return lum;
}

// ######################################################################
 Image<byte> calculateShift(Image<byte> lum, Image<byte> prevLum,
                            nub::ref<OutputFrameSeries> ofs)
{
  VisualObjectMatchAlgo voma(VOMA_SIMPLE);
//   if (strcmp(argv[1], "KDTree") == 0) voma = VOMA_KDTREE;
//   else if (strcmp(argv[1], "KDBBF") == 0) voma = VOMA_KDTREEBBF;
//   else if (strcmp(argv[1], "Simple") != 0)
//     LFATAL("Unknown matching method %s", argv[0]);

  // create visual objects:
  rutz::shared_ptr<VisualObject> vo1(new VisualObject("lum", "", lum));
  rutz::shared_ptr<VisualObject> vo2(new VisualObject("plum", "", prevLum));

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
  LDEBUG("getAffineAvgDist = %f", match.getAffineAvgDist());
  LDEBUG("getScore = %f", match.getScore());

  if (match.checkSIFTaffine() == false)
    LINFO("### Affine is too weird -- BOGUS MATCH");

  // get an image showing the matches:
  Image< PixRGB<byte> > mimg = match.getMatchImage(1.0F);
  Image< PixRGB<byte> > fimg = match.getFusedImage(0.25F);

  // LINFO("lum");
  // ofs->writeRGB
  //   (toRGB(lum), "test-FOE Main", FrameInfo("foe output", SRC_POS));
  // Raster::waitForKey();

  // LINFO("prevLum");
  // ofs->writeRGB
  //   (toRGB(prevLum), "test-FOE Main", FrameInfo("foe output", SRC_POS));
  // Raster::waitForKey();


  LINFO("Shift: %f %f", s.tx, s.ty);
  // ofs->writeRGB
  //   (fimg, "test-FOE Main", FrameInfo("foe output", SRC_POS));
  // Raster::waitForKey();

  Image<byte> res = shiftImage(s, lum, prevLum);

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
Image<byte> getApertureProblemStimuli(uint step)
{
  Image<byte> temp(WIDTH, HEIGHT, ZEROS);

  // create dot initially
  if(step == 0)
    {
      dots.resize(1); 
      // top left
      dots[0] = Point2D<float> (WIDTH/2, HEIGHT/4);      
    }

  uint len = 15;
  // APERTURE PROBLEM
  //===============================================
  dots[0] = Point2D<float>(dots[0].i + 2.0, dots[0].j);
  drawLine
    (temp, 
     Point2D<int>(dots[0].i-len, dots[0].j-len),
     Point2D<int>(dots[0].i+len, dots[0].j+len), byte(255), 1);
  
  return temp;
}

// ######################################################################
Image<byte> getShapeStimuli(uint step)
{
  Image<byte> temp(WIDTH, HEIGHT, ZEROS);
  // moving left
//   Rectangle r(Point2D<int>(100,100), Dims(20,20));
//   //Rectangle r(Point2D<int>(0+step*10,0+step*10), Dims(20,20));
//   if(step%2 ==0)
//     drawFilledRect(temp,r,byte(255));
//   //if(step%2 ==0)
//   //drawDisk(temp,Point2D<int>(100+step*1,100+step*1),5,byte(255));    
  return temp;
}

// ######################################################################
Image<byte> getBarStimuli(uint step)
{
  Image<byte> temp(WIDTH, HEIGHT, ZEROS);

  // initially create dots
  if(step == 0)
    {
      dots.resize(4); dotSizes.resize(4);

      // top left
      dots[0] = Point2D<float> (WIDTH/2, HEIGHT/4);      
      dotSizes[0] = 1.0;  
      dots[1] = Point2D<float> (WIDTH/4, HEIGHT/2);      
      dotSizes[1] = 1.0; 
      dots[2] = Point2D<float> (WIDTH/8, HEIGHT/4);      
      dotSizes[2] = 1.0;      
      dots[3] = Point2D<float> (WIDTH/4, HEIGHT/8);      
      dotSizes[3] = 1.0;

      // bottom right
//       dots[0] = Point2D<float> (7*WIDTH/8, 3*HEIGHT/4);      
//       dotSizes[0] = 1.0;  
//       dots[1] = Point2D<float> (3*WIDTH/4, 7*HEIGHT/8);      
//       dotSizes[1] = 1.0; 
//       dots[2] = Point2D<float> (WIDTH/2, 3*HEIGHT/4);      
//       dotSizes[2] = 1.0;      
//       dots[3] = Point2D<float> (3*WIDTH/4, HEIGHT/2);      
//       dotSizes[3] = 1.0; 

    }

  // move the dots if needed
  if(HAVE_MOTION)
    {
      //for(uint i = 0; i < dotNum; i++)
        dots[0] = Point2D<float>(dots[0].i + 2.0, dots[0].j);
        dots[1] = Point2D<float>(dots[1].i,       dots[1].j + 1.0);
        dots[2] = Point2D<float>(dots[2].i - 1.0, dots[2].j);
        dots[3] = Point2D<float>(dots[3].i,       dots[3].j - 1.0);

        //LINFO("loc: %f %f: size: %f", dots[i].i, dots[i].j, dotSizes[i]);      
    }
  // finally draw the dots
  //for(uint i = 0; i < dotNum; i++)
    //drawDisk(temp,Point2D<int>(dots[i].i, dots[i].j),dotSizes[i],byte(255));

    //drawDisk(temp,Point2D<int>(dots[0].i, dots[0].j), 1.0,byte(255));
    //drawDisk(temp,Point2D<int>(dots[1].i, dots[1].j), 1.0,byte(255));
    //drawDisk(temp,Point2D<int>(dots[2].i, dots[2].j), 1.0,byte(255));
    //drawDisk(temp,Point2D<int>(dots[3].i, dots[3].j), 1.0,byte(255));

  drawLine
    (temp, 
     Point2D<int>(dots[0].i, dots[0].j-15),
     Point2D<int>(dots[0].i, dots[0].j+15), byte(255), 1);  

  // control the thickness of the line
//     drawLine
//       (temp, 
//        Point2D<int>(dots[0].i, dots[0].j-15),
//        Point2D<int>(dots[0].i, dots[0].j+15), byte(255), 1);
//     drawLine
//       (temp, 
//        Point2D<int>(dots[0].i+1, dots[0].j-15),
//        Point2D<int>(dots[0].i+1, dots[0].j+15), byte(255), 1);
//     drawLine
//       (temp, 
//        Point2D<int>(dots[0].i-1, dots[0].j-15),
//        Point2D<int>(dots[0].i-1, dots[0].j+15), byte(255), 1);
//     drawLine
//       (temp, 
//        Point2D<int>(dots[0].i-2, dots[0].j-15),
//        Point2D<int>(dots[0].i-2, dots[0].j+15), byte(255), 1);
//     drawLine
//       (temp, 
//        Point2D<int>(dots[0].i-3, dots[0].j-15),
//        Point2D<int>(dots[0].i-3, dots[0].j+15), byte(255), 1);
//     drawLine
//       (temp, 
//        Point2D<int>(dots[0].i+2, dots[0].j-15),
//        Point2D<int>(dots[0].i+2, dots[0].j+15), byte(255), 1);
//     drawLine
//       (temp, 
//        Point2D<int>(dots[0].i-4, dots[0].j-15),
//        Point2D<int>(dots[0].i-4, dots[0].j+15), byte(255), 1);
//     drawLine
//       (temp, 
//        Point2D<int>(dots[0].i+3, dots[0].j-15),
//        Point2D<int>(dots[0].i+3, dots[0].j+15), byte(255), 1);

    //===============================================
//     drawLine
//       (temp, 
//        Point2D<int>(dots[1].i-15, dots[1].j),
//        Point2D<int>(dots[1].i+15, dots[1].j), byte(255), 1);

//     drawLine
//       (temp, 
//        Point2D<int>(dots[2].i, dots[2].j-15),
//        Point2D<int>(dots[2].i, dots[2].j+15), byte(255), 1);

//     drawLine
//       (temp, 
//        Point2D<int>(dots[3].i-15, dots[3].j),
//        Point2D<int>(dots[3].i+15, dots[3].j), byte(255), 1);


             //drawDisk(temp,Point2D<int>(dots[i].i, dots[i].j),dotSizes[i],byte(255));
  //temp.setVal(dots[i].i, dots[i].j, byte(128));

  //drawDisk(temp,Point2D<int>(FOE_X,FOE_Y),5,byte(128));    

  return temp;
}

// ######################################################################
Image<byte> getPlanarMotionStimuli
(Image<byte> temp, uint step, float dx, float dy)
{
  //if(step > NFRAME) return Image<byte>();

  uint dotNum = 200;//200;//DOT_NUM;

  float orgDotSize = 1.0;

  // create random dot initially
  if(step == 0)
    {
      pdots.resize(dotNum); pdotSizes.resize(dotNum);
      for(uint i = 0; i < dotNum; i++)
        {
          pdots[i] = Point2D<float>
            (WIDTH  * double(rand())/(RAND_MAX + 1.0),
             HEIGHT * double(rand())/(RAND_MAX + 1.0));
          pdotSizes[i] = orgDotSize; 
        }
    }
  
  // check for out-of-bounds dot needed to be shown
  for(uint i = 0; i < dotNum; i++)
    {
      // NOTE: can also kill dots randomly before going out of the image
      if(!temp.getBounds().contains(Point2D<int>(pdots[i].i, pdots[i].j))) 
        {
          float srx = 0.0;   float sry = 0.0;
          float rx  = WIDTH; float ry  = HEIGHT;
          if(dx < 0.0)     { srx = 7.0*WIDTH/8.0; rx  = WIDTH/8; }
          else if(dx > 0.0){ srx = 0.0;           rx  = WIDTH/8; }
          
          if(dy < 0.0)     { sry = 7.0*HEIGHT/8.0; ry  = HEIGHT/8; }
          else if(dy > 0.0){ sry = 0.0;            ry  = HEIGHT/8; }

          float sx = rx * double(rand())/(RAND_MAX + 1.0) + srx; 
          float sy = ry * double(rand())/(RAND_MAX + 1.0) + sry;
          
          pdots[i] = Point2D<float>(sx,sy);
          pdotSizes[i] = orgDotSize; 
          LDEBUG("new dots[%3d]: %7.3f %7.3f", i, sx, sy);
        }
      else
        {
          Point2D<int> pt(pdots[i].i, pdots[i].j);
          LDEBUG("[%3d] it's ok: (%7.3f %7.3f) -> %3d %3d", 
                 i, pdots[i].i, pdots[i].j, pt.i, pt.j);
        }
    }

  // planar motion
  if(dx != 0.0 || dy != 0.0)
    for(uint i = 0; i < dotNum; i++)
      {
        pdots[i] = Point2D<float>(pdots[i].i + dx, 
                                  pdots[i].j + dy);
      }

  // finally draw the dots
  for(uint i = 0; i < dotNum; i++)
    {
      LDEBUG("[%d] loc: %10.3f %10.3f: size: %7.3f", 
             i, pdots[i].i, pdots[i].j, pdotSizes[i]);
      drawDisk(temp,Point2D<int>(pdots[i].i, pdots[i].j),
               pdotSizes[i],byte(255));
      //temp.setVal(pdots[i].i, pdots[i].j, byte(128));
    }
  return temp;
}

// ######################################################################
Image<byte> orgPlanarImage;
Image<byte> getPlanarMotionImage
(uint step, uint totalStep, float dx, float dy, Image<byte> image)
{
  // just loop it 
  step = step % totalStep;

  // set original image on first step
  if(step == 0)
    { 
      orgPlanarImage = image;
    }
  image = orgPlanarImage;
  
  uint width  = image.getWidth();
  uint height = image.getHeight(); 
  float scale =  1.25;
  Image<byte> temp = rescale(image, scale*width, scale*height);
  float nwidth  = temp.getWidth();
  float nheight = temp.getHeight(); 

  float sleft = 0.0; if(dx < 0.0) sleft = nwidth  - 1 - width;
  float stop  = 0.0; if(dy < 0.0) stop  = nheight - 1 - height; 
  
  float left  = sleft + dx*step;
  float top   = stop  + dy*step; 

  //if(top  < 0.0) top  = 0.0;
  //if(left < 0.0) left = 0.0;

  Rectangle r = 
    Rectangle::tlbrI(top, left, top+height-1, left+width-1);

//   LINFO("[%3d/%3d] FOE(%7.3f %7.3f) %f p(%7.3f %7.3f) [[%7.3f %7.3f]] " 
//          "[%3d %3d %3d %3d] temp(%3d %3d) ((%3d %3d))", 
//         step, totalStep, 
//         foeX,foeY, scale, px, py, top, left,
//         r.top(), r.left(), r.bottomI(), r.rightI(), 
//         temp.getWidth(), temp.getHeight(),
//         r.width(),r.height());
  Image<byte> result = crop(temp, r);

  return result;
}

// ######################################################################
Image<byte> getFoeDots
(uint step, bool haveMotion, bool  haveTempSGrad, bool haveSpatSGrad,
 float dx, float dy, float dotOrgDSize)
{
  //if(step > NFRAME) return Image<byte>();

  Image<byte> temp(WIDTH, HEIGHT, ZEROS);

  float orgDotSize = 1.0;

  // create random dot initially
  if(step == 0)
    {
      dots.resize(DOT_NUM); dotSizes.resize(DOT_NUM);
      for(uint i = 0; i < DOT_NUM; i++)
        {
          dots[i] = Point2D<float>
            (WIDTH  * double(rand())/(RAND_MAX + 1.0),
             HEIGHT * double(rand())/(RAND_MAX + 1.0));

          // just do it in order to get identical average size
          //float range = MAX_DOT_SIZE - MIN_DOT_SIZE;
          //dotSizes[i] = i*range/(DOT_NUM-1.0)+ double(MIN_DOT_SIZE); 
          dotSizes[i] = orgDotSize; 
        }
    }

  // check for out-of-bounds dot needed to be shown
  for(uint i = 0; i < DOT_NUM; i++)
    {
      // NOTE: can also kill dots randomly before going out of the image
      // NOTE: how about adding new dots in the FOE quadrants
      if(!temp.getBounds().contains(Point2D<int>(dots[i].i, dots[i].j))) 
        {
          dots[i] = Point2D<float>
            (WIDTH  * double(rand())/(RAND_MAX + 1.0),
             HEIGHT * double(rand())/(RAND_MAX + 1.0) );

          // keep the sizes or 1.0?
          dotSizes[i] = orgDotSize; 
        }
    }

  // modify sizes according to rules
  for(uint i = 0; i < DOT_NUM; i++)
    {
      if(haveTempSGrad && haveSpatSGrad)
        {
          float dist = sqrt(pow((dots[i].i - FOE_X), 2.0) + 
                            pow((dots[i].j - FOE_Y), 2.0)  ); 
          if(haveMotion)
            {
              dotSizes[i] = dotOrgDSize * dist;
            }
          // growing, FOE coherent, but not moving 
          else
            {
              if(step == 0)
                {
                  dotSizes[i] = dotOrgDSize * dist;
                }
              else
                {
                  // increase until ABS_MAX_DSIZE, then stop
                  if(dotSizes[i] < ABS_MAX_DSIZE)
                    {
                      dotSizes[i] = (1.0 + DOT_DSIZE) *
                        dotOrgDSize * dist;
                    }
                  // else dot size stays the same
                }
            }
        }
      else if(haveTempSGrad && !haveSpatSGrad)
        {
          // all dot have same size just increase 
          float ds = MAX_DOT_SIZE - MIN_DOT_SIZE;
          dotSizes[i] = step*ds/(NFRAME - 1.0)+ double(MIN_DOT_SIZE); 
        }
      else if(!haveTempSGrad && haveSpatSGrad)
        {
          float dist = sqrt(pow((dots[i].i - FOE_X), 2.0) + 
                            pow((dots[i].j - FOE_Y), 2.0)  ); 
          dotSizes[i] = dotOrgDSize * dist;
        }
      // else just keep size 
    }

  // move the dots if needed
  if(haveMotion)
    for(uint i = 0; i < DOT_NUM; i++)
      {
        dots[i] = Point2D<float>
          (dots[i].i + DOT_VEL * (dots[i].i - FOE_X),
           dots[i].j + DOT_VEL * (dots[i].j - FOE_Y));
      }

  // add lateral motion
  if(dx != 0.0 || dy != 0.0)
    for(uint i = 0; i < DOT_NUM; i++)
      {
        dots[i] = Point2D<float>(dots[i].i + dx, dots[i].j + dy);
      }

  // finally draw the dots
  for(uint i = 0; i < DOT_NUM; i++)
    {
      //LINFO("loc: %10.3f %10.3f: size: %7.3f", 
      //      dots[i].i, dots[i].j, dotSizes[i]);
      drawDisk(temp,Point2D<int>(dots[i].i, dots[i].j),dotSizes[i],byte(255));
      //temp.setVal(dots[i].i, dots[i].j, byte(128));
    }

  // draw the FOE
  //drawDisk(temp,Point2D<int>(FOE_X,FOE_Y),2,byte(128));    

  return temp;
}


// ######################################################################
// this is stimuli like in Fukuchi,Tsuchiya,Koch VSS/JOV 2009/2010
// FOE is FOE_X,FOE_Y
Image<byte> orgImage;
Image<byte> getCleanFOE
(uint step, uint totalStep, uint mag, float dx, float dy, Image<byte> image)
{
  // just loop it 
  step = step % totalStep;

  // set original image on first step
  if(step == 0)
    { 
      orgImage = image;
      return image; 
    }
  image = orgImage;
  
  uint width  = image.getWidth();
  uint height = image.getHeight(); 

  float nsize = (step/(totalStep - 1.0));
  float scale =  1.0 / (1.0 - nsize*1.0/mag);
  Image<byte> temp = rescale(image, scale*width, scale*height);
  float nwidth  = temp.getWidth();
  float nheight = temp.getHeight(); 

  float px = float(FOE_X)/float(width);
  float py = float(FOE_Y)/float(height);

  float foeX  = px*nwidth; 
  float foeY  = py*nheight; 

  float left  = foeX - float(FOE_X) + dx*step;
  float top   = foeY - float(FOE_Y) + dy*step; 

  //if(top  < 0.0) top  = 0.0;
  //if(left < 0.0) left = 0.0;

  Rectangle r = 
    Rectangle::tlbrI(top, left, top+height-1, left+width-1);

  LINFO("[%3d/%3d] FOE(%7.3f %7.3f) %f p(%7.3f %7.3f) [[%7.3f %7.3f]] " 
         "[%3d %3d %3d %3d] temp(%3d %3d) ((%3d %3d))", 
        step, totalStep, 
        foeX,foeY, scale, px, py, top, left,
        r.top(), r.left(), r.bottomI(), r.rightI(), 
        temp.getWidth(), temp.getHeight(),
        r.width(),r.height());
  Image<byte> result = crop(temp, r);

  return result;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
