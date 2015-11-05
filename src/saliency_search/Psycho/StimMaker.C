/*!@file Psycho/StimMaker.C make different kind of visual test stimuli
*/

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
// Primary maintainer for this file: T. Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/StimMaker.C $
// $Id: StimMaker.C 9412 2008-03-10 23:10:15Z farhan $
//

#ifndef STIM_MAKER_C_DEFINED
#define STIM_MAKER_C_DEFINED

#include "Psycho/StimMaker.H"

StimMaker::StimMaker(const unsigned short ImageSizeX,
                     const unsigned short ImageSizeY,
                     const unsigned short frameCount,
                     const char bgColor)
{
  itsFrameCount = frameCount;
  itsSizeX      = ImageSizeX;
  itsSizeY      = ImageSizeY;
  itsSlowRate   = SM_SLOW_RATE;
  itsFastRate   = SM_FAST_RATE;
  itsStopRate   = SM_STOP_RATE;
  itsRandomSeed = 0;

  itsBlack.set(0.0F,0.0F,0.0F);
  itsColorVec.resize(SM_COLORS,itsBlack);

  itsRed.set(255.0F,0.0F,0.0F);
  itsOrange.set(255.0F,128.0F,0.0F);
  itsYellow.set(255.0F,255.0F,0.0F);
  itsGreen.set(0.0F,255.0F,0.0F);
  itsBlue.set(0.0F,0.0F,255.0F);
  itsPurple.set(255.0F,0.0F,255.0F);
  itsWhite.set(255.0F,255.0F,255.0F);

  itsColorVec[0] = itsRed;
  itsColorVec[1] = itsOrange;
  itsColorVec[2] = itsYellow;
  itsColorVec[3] = itsGreen;
  itsColorVec[4] = itsBlue;
  itsColorVec[5] = itsPurple;
  itsColorVec[6] = itsWhite;

  itsGTtargetColor.set(255.0F,255.0F,255.0F);
  itsGTtargetColorPatch1.set(0.0F,128.0F,128.0F);
  itsGTtargetColorPatch2.set(0.0F,0.0F,255.0F);
  itsGTtargetColorPatch1off.set(128.0F,128.0F,0.0F);
  itsGTtargetColorPatch2off.set(255.0F,0.0F,0.0F);
  itsGTdistColor.set(128.0F,128.0F,128.0F);

  SM_init(bgColor);
}

/*****************************************************************************/

StimMaker::~StimMaker()
{}

/*****************************************************************************/

// reset the frames and back ground truth to the background color
void StimMaker::SM_init(const char bgColor)
{
  Image<PixRGB<float> > baseImage;
  baseImage.resize(itsSizeX,itsSizeY);
  itsFrames.resize(itsFrameCount, baseImage);
  std::vector<Image<PixRGB<float> > >::iterator framesIter = itsFrames.begin();
  while(framesIter != itsFrames.end())
  {
    Image<PixRGB<float> >::iterator imgIter = framesIter->beginw();
    while(imgIter != framesIter->endw())
    {
      *imgIter = itsColorVec[itsBGcolor+1];
      ++imgIter;
    }
    ++framesIter;
  }

  itsGroundTruth.resize(itsFrameCount, baseImage);
  framesIter = itsGroundTruth.begin();
  while(framesIter != itsGroundTruth.end())
  {
    Image<PixRGB<float> >::iterator imgIter = framesIter->beginw();
    while(imgIter != framesIter->endw())
    {
      *imgIter = itsBlack;
      ++imgIter;
    }
    ++framesIter;
  }
  SM_setStandardBackGround(bgColor);

}

/*****************************************************************************/

// reset the frames and back ground truth to the background color
void StimMaker::SM_setStandardBackGround(const unsigned char bgColor)
{
  itsBGcolor         = bgColor;
  itsCustomBG        = itsColorVec[itsBGcolor];
  itsUseCustomColors = false;
}

/*****************************************************************************/

void StimMaker::SM_overRideRates(const unsigned char slowRate,
                                 const unsigned char fastRate,
                                 const unsigned char stopRate)
{
  itsSlowRate = slowRate;
  itsFastRate = fastRate;
  itsStopRate = stopRate;
}

/*****************************************************************************/

void StimMaker::SM_setCustomColors(const PixRGB<float> BGcolor,
                                   const PixRGB<float> TargetColor,
                                   const PixRGB<float> DistColor)
{
  itsCustomBG        = BGcolor;
  itsCustomTarg      = TargetColor;
  itsCustomDist      = DistColor;
  itsUseCustomColors = true;
}

/*****************************************************************************/
inline ushort StimMaker::SM_speedUpCheck(ushort currBlinkRate,
                                         const ushort frame,
                                         const ushort changeFrame,
                                         const unsigned char stimRate,
                                         const unsigned char useSmoothRateChange) const
{
  if(frame > changeFrame)
  {
    // we are slow, go fast
    if(stimRate == SM_SLOW_STIM)
    {
      if(currBlinkRate > itsFastRate)
      {
        if(useSmoothRateChange == SM_USE_SMOOTH_RATE_CHANGE)
          currBlinkRate -= SM_SMOOTH_ACCEL;
        else
          currBlinkRate = itsFastRate;
      }
      else
        if(currBlinkRate < itsFastRate)
          currBlinkRate = itsFastRate;
    }
    // we are steady, start into slow
    else if(stimRate == SM_NSPD_STIM)
    {
      if(currBlinkRate != itsSlowRate)
        currBlinkRate = itsSlowRate;
    }
  }
  return currBlinkRate;
}

/*****************************************************************************/
inline ushort StimMaker::SM_speedDownCheck(ushort currBlinkRate,
                                           const ushort frame,
                                           const ushort changeFrame,
                                           const unsigned char stimRate,
                                           const unsigned char useSmoothRateChange) const
{
  if(frame > changeFrame)
  {
    // we are fast, go slow
    if(stimRate == SM_FAST_STIM)
    {
      if(currBlinkRate < itsSlowRate)
      {
        if(useSmoothRateChange == SM_USE_SMOOTH_RATE_CHANGE)
          currBlinkRate += SM_SMOOTH_ACCEL;
        else
          currBlinkRate = itsSlowRate;
      }
      else
        if(currBlinkRate > itsSlowRate)
          currBlinkRate = itsSlowRate;
    }
    // we are slow, then stop
    else if(stimRate == SM_SLOW_STIM)
    {
      if(currBlinkRate != itsStopRate)
        currBlinkRate = itsStopRate;
    }
  }
  return currBlinkRate;

}



/*****************************************************************************/

inline void StimMaker::SM_drawGroundTruth(const ushort frame,
                                          const unsigned char stimSizeX,
                                          const unsigned char stimSizeY,
                                          const ushort PosX,
                                          const ushort PosY,
                                          const bool targetOn)
{
  const Point2D<int> P1(PosX,PosY);
  if(targetOn)
  {
    drawDisk(itsGroundTruth[frame],P1,stimSizeX,itsGTtargetColorPatch1);
    drawDisk(itsGroundTruth[frame],P1,stimSizeX/2,itsGTtargetColorPatch2);

  }
  else
  {
    drawDisk(itsGroundTruth[frame],P1,stimSizeX,itsGTtargetColorPatch1off);
    drawDisk(itsGroundTruth[frame],P1,stimSizeX/2,itsGTtargetColorPatch2off);
  }
}

/*****************************************************************************/

inline void StimMaker::SM_drawSingleTarget(const ushort frame,
                                           const unsigned char stimShape,
                                           const unsigned char stimColor,
                                           const unsigned char stimSizeX,
                                           const unsigned char stimSizeY,
                                           const ushort PosX,
                                           const ushort PosY,
                                           const float stimOri,
                                           const float shapePositionJitter,
                                           const float shapeOrientationJitter,
                                           const bool target)
{
  // Compute random shape if any

  unsigned char shape;

  if(stimShape == SM_STIM_RAND)
  {
    shape = 1 + (unsigned char)(SM_SHAPES*rand()/(RAND_MAX+1.0));
  }
  else
  {
    shape = stimShape;
  }

  // Compute random color if needed
  if(stimColor == SM_COLOR_RAND)
  {
    unsigned char color = 1 + (unsigned char)(SM_COLORS*rand()/(RAND_MAX+1.0));
    if(target)
      itsCustomTarg = itsColorVec[color];
    else
      itsCustomDist = itsColorVec[color];
  }
  else
  {
    if(stimColor != SM_COLOR_CUSTOM)
    {
      if(target)
        itsCustomTarg = itsColorVec[stimColor-1];
      else
        itsCustomDist = itsColorVec[stimColor-1];
    }
  }  // compute jitter if any
  float posJitterX,posJitterY,oriJitter;

  if(shapePositionJitter != 0.0F)
  {
    // some shapes are defined by a radius which is actaully
    // the diameter, as such we have to devide their size by 1/2
    float divider;
    if((stimShape == SM_STIM_DISK) ||
       (stimShape == SM_STIM_CROSS) ||
       (stimShape == SM_STIM_PATCH) ||
       (stimShape == SM_STIM_CIRC))
      divider = 4.0F;
    else
      divider = 2.0F;
    // jitter by as much as +/- .5 the size of the stim
    posJitterX = ((float)stimSizeX)*(shapePositionJitter*rand())
      /(RAND_MAX+1.0F) - stimSizeX/divider;
    posJitterY = ((float)stimSizeY)*(shapePositionJitter*rand())
      /(RAND_MAX+1.0F) - stimSizeY/divider;
  }
  else
  {
    posJitterX = 0.0F;
    posJitterY = 0.0F;
  }

  if(shapeOrientationJitter != 0.0F)
  {
    // jitter by as much as +/- Pi degrees
    oriJitter  = shapeOrientationJitter *
      (((2.0F*M_PI*rand())/(RAND_MAX+1.0F)) - M_PI);
  }
  else
  {
    oriJitter = 0.0F;
  }

  const float newPosX = posJitterX + PosX;
  const float newPosY = posJitterY + PosY;

  const float newOri  = stimOri + oriJitter;

  // Draw Disk
  if(shape == SM_STIM_DISK)
  {
    if(stimSizeX != stimSizeY)
      LINFO("WARNING: Disk size X and Y should be the same! Using X size.");
    if(stimOri != 0.0F)
      LINFO("WARNING: Disk cannot have orientation! Ignoring.");
    const Point2D<int> center((int)round(newPosX),(int)round(newPosY));
    if(target)
    {
      drawDisk(itsFrames[frame],center,stimSizeX/2,itsCustomTarg);
      drawDisk(itsGroundTruth[frame],center,stimSizeX/2,itsGTtargetColor);
    }
    else
    {
      drawDisk(itsFrames[frame],center,stimSizeX/2,itsCustomDist);
      drawDisk(itsGroundTruth[frame],center,stimSizeX/2,itsGTdistColor);
    }
  }
  // Draw Rect
  else if(shape == SM_STIM_RECT)
  {
    const int coordX1 = (int)floor(newPosX + (float)stimSizeX/2.0F);
    const int coordX2 = (int)floor(newPosX - (float)stimSizeX/2.0F);
    const int coordY1 = (int)floor(newPosY + (float)stimSizeY/2.0F);
    const int coordY2 = (int)floor(newPosY - (float)stimSizeY/2.0F);
    Rectangle rect = Rectangle::tlbrI(coordY2,coordX2,coordY1,coordX1);

    if(target)
    {
      drawRectOR(itsFrames[frame],rect,itsCustomTarg,2,newOri);
      drawRectOR(itsGroundTruth[frame],rect,itsGTtargetColor,2,newOri);
    }
    else
    {
      drawRectOR(itsFrames[frame],rect,itsCustomDist,2,newOri);
      drawRectOR(itsGroundTruth[frame],rect,itsGTdistColor,2,newOri);
    }
  }
  // Draw Line
  else if(shape == SM_STIM_LINE)
  {
    if(stimSizeX != stimSizeY)
      LINFO("WARNING: Line size X and Y should be the same! Using X size.");
    const float newX = stimSizeX/2 * sin(newOri);
    const float newY = stimSizeX/2 * cos(newOri);

    const int coordX1 = (int)floor(newPosX + newX);
    const int coordX2 = (int)floor(newPosX - newX);
    const int coordY1 = (int)floor(newPosY + newY);
    const int coordY2 = (int)floor(newPosY - newY);
    const Point2D<int> P1(coordX1,coordY1);
    const Point2D<int> P2(coordX2,coordY2);

    if(target)
    {
      drawLine(itsFrames[frame],P1,P2,itsCustomTarg,2);
      drawLine(itsGroundTruth[frame],P1,P2,itsGTtargetColor,2);
    }
    else
    {
      drawLine(itsFrames[frame],P1,P2,itsCustomDist,2);
      drawLine(itsGroundTruth[frame],P1,P2,itsGTdistColor,2);
    }
  }
  // Draw Cross
  else if(shape == SM_STIM_CROSS)
  {
    if(stimSizeX != stimSizeY)
      LINFO("WARNING: Cross size X and Y should be the same! Using X size.");
    const Point2D<int> P1((int)floor(newPosX),(int)floor(newPosY));
    if(target)
    {
      drawCrossOR(itsFrames[frame],P1,itsCustomTarg,stimSizeX/2,2,newOri);
      drawCrossOR(itsGroundTruth[frame],P1,itsGTtargetColor,stimSizeX/2,2,newOri);
    }
    else
    {
      drawCrossOR(itsFrames[frame],P1,itsCustomDist,stimSizeX/2,2,newOri);
      drawCrossOR(itsGroundTruth[frame],P1,itsGTdistColor,stimSizeX/2,2,newOri);
    }
  }
  // Draw Patch
  else if(shape == SM_STIM_PATCH)
  {
    if(stimOri != 0.0F)
      LINFO("WARNING: Patch cannot have orientation! Net yet supported. Ignoring.");
    if(stimSizeX != stimSizeY)
      LINFO("WARNING: Patch size X and Y should be the same! Using X size.");
    const Point2D<int> P1((int)floor(newPosX),(int)floor(newPosY));
    if(target)
    {
      drawPatch(itsFrames[frame],P1,stimSizeX/2,itsCustomTarg);
      drawPatch(itsGroundTruth[frame],P1,stimSizeX/2,itsGTtargetColor);
    }
    else
    {
      drawPatch(itsFrames[frame],P1,stimSizeX/2,itsCustomDist);
      drawPatch(itsGroundTruth[frame],P1,stimSizeX/2,itsGTdistColor);
    }
  }
  // Draw Arrow
  else if(shape == SM_STIM_ARROW)
  {
    if(stimSizeX != stimSizeY)
      LINFO("WARNING: Arrow size X and Y should be the same! Using X size.");
    const float newX = stimSizeX/2 * sin(newOri);
    const float newY = stimSizeX/2 * cos(newOri);

    const int coordX1 = (int)floor(newPosX + newX);
    const int coordX2 = (int)floor(newPosX - newX);
    const int coordY1 = (int)floor(newPosY + newY);
    const int coordY2 = (int)floor(newPosY - newY);
    const Point2D<int> P1(coordX1,coordY1);
    const Point2D<int> P2(coordX2,coordY2);

    if(target)
    {
      drawArrow(itsFrames[frame],P1,P2,itsCustomTarg,2);
      drawArrow(itsGroundTruth[frame],P1,P2,itsGTtargetColor,2);
    }
    else
    {
      drawArrow(itsFrames[frame],P1,P2,itsCustomDist,2);
      drawArrow(itsGroundTruth[frame],P1,P2,itsGTdistColor,2);
    }
  }
  // Draw Circle
  else if(shape == SM_STIM_CIRC)
  {
    if(stimSizeX != stimSizeY)
      LINFO("WARNING: Circle size X and Y should be the same! Using X size.");
    if(stimOri != 0.0F)
      LINFO("WARNING: Circle cannot have orientation! Ignoring.");

    const Point2D<int> center((int)round(newPosX),(int)round(newPosY));
    if(target)
    {
      drawCircle(itsFrames[frame],center,stimSizeX/2,itsCustomTarg);
      drawCircle(itsGroundTruth[frame],center,stimSizeX/2,itsGTtargetColor);
    }
    else
    {
      drawCircle(itsFrames[frame],center,stimSizeX/2,itsCustomDist);
      drawCircle(itsGroundTruth[frame],center,stimSizeX/2,itsGTdistColor);
    }
  }
  else
  {
    LFATAL("StimMaker tried to draw an unlisted shape %d. See list in StimMaker.H",shape);
  }
}

/*****************************************************************************/

void StimMaker::SM_makeUniformStim(const StimMakerParam &stim)
{
  itsRandomSeed = stim.SMP_randomSeed;
  // set the random seed so we can mix it up a little
  srand(itsRandomSeed);
  LINFO("Setting up stim params");
  // go through each frame
  uint frame = 0;
  // figure out where to place distractors
  const float divX     = ceil((float)itsSizeX/(float)stim.SMP_distPerRow);
  const float divY     = ceil((float)itsSizeY/(float)stim.SMP_distPerCol);
  const float startx   = round(divX/2);
  const float starty   = round(divY/2);
  const uint movieHalf = (uint)round((float)itsFrameCount/2.0F);
  const uint fastBlink = itsFastRate;
  const uint slowBlink = itsSlowRate;

  uint targBlinkRate;
  uint distBlinkRate;

  // slighly randomize the change time
  const uint changeFrame = (uint)floor((movieHalf/2)+(movieHalf*rand()/(RAND_MAX+1.0)));

  // define the speed at which the target blinks
  if(stim.SMP_targetRate == SM_FAST_STIM)
    targBlinkRate = fastBlink;
  else if(stim.SMP_targetRate == SM_SLOW_STIM)
    targBlinkRate = slowBlink;
  else
    targBlinkRate = 0;

  // define the speed at which the distractors blink
  if(stim.SMP_distRate == SM_FAST_STIM)
    distBlinkRate = fastBlink;
  else if(stim.SMP_distRate == SM_SLOW_STIM)
    distBlinkRate = slowBlink;
  else
    distBlinkRate = 0;

  //const float Randy = (1.0*rand()/(RAND_MAX+1.0));

  //float targAccelBase;
  //float distAccelBase;

  // Compute change rates as random over distractors or target
  /*
  if(targState > SM_STATE_STEADY)
  {
    targAccelBase = ((float)targBlinkRate/2.0F)*Randy;
  }

  if(distState > SM_STATE_STEADY)
  {
    distAccelBase = ((float)distBlinkRate/2.0F)*Randy;
  }
  */

  // allocate target
  uint currTargBlinkRate   = targBlinkRate;
  int currTargBlinkOffset  = 0;
  //uint currTargBlinkCount  = 0;
  uint currTargShape       = stim.SMP_targetShape;
  uint currTargColor       = stim.SMP_targetColor;
  uint currTargSizeX       = stim.SMP_targetSizeX;
  uint currTargSizeY       = stim.SMP_targetSizeY;
  bool currTargOn          = stim.SMP_targetOn;

  LINFO("Computing target values");
  // Start the target randomly?
  if(stim.SMP_useRandomStart == SM_USE_RANDOM_START)
  {
    const float distRand = (1.0*rand()/(RAND_MAX+1.0));
    currTargBlinkOffset = (uint)round((float)distBlinkRate*distRand);
    const float onRand = (1.0*rand()/(RAND_MAX+1.0));
    if(onRand > .5)
      currTargOn = true;
    else
      currTargOn = false;
  }
  // assign the target a random shape?
  if(stim.SMP_targetShape == SM_STIM_RAND)
  {
    const float distRand = 1.0F + (((float)(SM_SHAPES))*rand()/
                                      (RAND_MAX+1.0F));
    currTargShape = (uint)distRand;
  }
  // assign the target a random color?
  if(stim.SMP_targetColor == SM_COLOR_RAND)
  {
    const float distRand = 1.0F + (((float)(SM_COLORS-1.0F))*rand()/
                                      (RAND_MAX+1.0F));
    currTargColor = (uint)distRand;
  }
  // do we need to give a random size to the target?
  if(stim.SMP_targetSizeX == 0)
  {
    const float maxSizeX  = divX;
    const float distRandX = 1.0F + (((float)(maxSizeX-1.0F))*rand()/
                                       (RAND_MAX+1.0F));
    currTargSizeX = (uint)distRandX;
    const float maxSizeY  = divY;
    const float distRandY = 1.0F + (((float)(maxSizeY-1.0F))*rand()/
                                       (RAND_MAX+1.0F));
    currTargSizeY = (uint)distRandY;
  }

  LINFO("Setting up distractor vectors");
  // allocate vectors for distractors
  std::vector<ushort> distInit(stim.SMP_distPerCol,0);
  std::vector<int> distInitS(stim.SMP_distPerCol,0);
  std::vector<float> distInitF(stim.SMP_distPerCol,0);

  std::vector<std::vector<ushort> >
    currDistBlinkRate(stim.SMP_distPerRow,distInit);
  std::vector<std::vector<ushort> >
    currDistShape(stim.SMP_distPerRow,distInit);
  std::vector<std::vector<ushort> >
    currDistColor(stim.SMP_distPerRow,distInit);
  std::vector<std::vector<ushort> >
    currDistSizeX(stim.SMP_distPerRow,distInit);
  std::vector<std::vector<ushort> >
    currDistSizeY(stim.SMP_distPerRow,distInit);
  std::vector<std::vector<ushort> >
    currPosX(stim.SMP_distPerRow,distInit);
  std::vector<std::vector<ushort> >
    currPosY(stim.SMP_distPerRow,distInit);

  std::vector<std::vector<int> >
    currDistBlinkOffset(stim.SMP_distPerRow,distInitS);

  std::vector<std::vector<float> >
    currDistOrientation(stim.SMP_distPerRow,distInitF);

  std::vector<bool> distOnInit(stim.SMP_distPerCol,stim.SMP_distOn);
  std::vector<std::vector<bool> > currDistOn(stim.SMP_distPerRow,distOnInit);

  // (1) define the starting blink rate for each distractor
  // (2) define when its offset
  // (3) define its start state
  // (4) define shape
  // (5) define color

  LINFO("Setting up distractors");
  for(uint i = 0; i < stim.SMP_distPerRow; i++)
  {
    for(uint j = 0; j < stim.SMP_distPerCol; j++)
    {
      // determine start offset if random
      if(stim.SMP_useRandomStart == SM_USE_RANDOM_START)
      {
        const float distRand = (1.0F*rand()/(RAND_MAX+1.0));
        currDistBlinkOffset[i][j] = (uint)round((float)distBlinkRate*distRand);
        const float onRand = (1.0F*rand()/(RAND_MAX+1.0));
        if(onRand > 0.5F)
          currDistOn[i][j] = true;
        else
          currDistOn[i][j] = false;
      }
      else
      {
        currDistBlinkOffset[i][j] = 0;
      }

      // determine distractor shape if random
      if(stim.SMP_distShape == SM_STIM_RAND)
      {
        const float distRand = 1.0F + (((float)(SM_SHAPES))*rand()/
                                          (RAND_MAX+1.0F));
        currDistShape[i][j]  = (ushort)distRand;
      }
      else
      {
        currDistShape[i][j]  = stim.SMP_distShape;
      }

      // determine distractor color if random
      if(stim.SMP_distColor == SM_COLOR_RAND)
      {
        const float distRand = 1.0F + (((float)(SM_COLORS - 1.0F))*rand()/
                                       (RAND_MAX+1.0F));

        //std::cerr << ">>>>>NEW COLOR IS " << distRand << "\n";
        currDistColor[i][j]  = (ushort)distRand;
      }
      else
      {
        currDistColor[i][j] = stim.SMP_distColor;
      }

      // determine distractor size if random
      if(stim.SMP_distSizeX == 0)
      {
        const float maxSizeX  = divX;
        const float distRandX = 1.0F + (((float)(maxSizeX-1.0))*rand()/
                                          (RAND_MAX+1.0F));
        currDistSizeX[i][j]   = (ushort)distRandX;
        const float maxSizeY  = divY;
        const float distRandY = 1.0F + (((float)(maxSizeY-1.0))*rand()/
                                          (RAND_MAX+1.0F));
        currDistSizeY[i][j]   = (ushort)distRandY;
      }
      else
      {
        currDistSizeX[i][j]   = stim.SMP_distSizeX;
        currDistSizeY[i][j]   = stim.SMP_distSizeY;
      }

      currDistBlinkRate[i][j] = distBlinkRate;

      // determine if we need to jitter the position of the target
      if(stim.SMP_useHexagon == SM_USE_HEXAGON)
      {
        float newDiv = 0.0F;
        if(j%2 == 0)
          newDiv = divX/4;
        else
          newDiv = -1.0F*divX/4;
        // image pixel offset in X for target
        currPosX[i][j] = (uint)floor(newDiv + startx + i*divX);
        // image pixel offset in Y for target
        currPosY[i][j] = (uint)floor(starty + j*divY);
      }
      else
      {
        // image pixel offset in X for target
        currPosX[i][j] = (uint)floor(startx + i*divX);
        // image pixel offset in Y for target
        currPosY[i][j] = (uint)floor(starty + j*divY);
      }
      float posJitterX,posJitterY;
      if(stim.SMP_shapePositionJitterStatic != 0.0F)
      {
        float divider;
        if((j == stim.SMP_targetPosJ) && (i == stim.SMP_targetPosI))
        {
          if((currTargShape == SM_STIM_DISK)  ||
             (currTargShape == SM_STIM_CROSS) ||
             (currTargShape == SM_STIM_PATCH) ||
             (currTargShape == SM_STIM_CIRC))
            divider = 4.0F;
          else
            divider = 2.0F;

          posJitterX = ((float)stim.SMP_targetSizeX)
            *(stim.SMP_shapePositionJitterStatic*rand())
            /(RAND_MAX+1.0F) - stim.SMP_distSizeX/divider;
          posJitterY = ((float)stim.SMP_targetSizeY)
            *(stim.SMP_shapePositionJitterStatic*rand())
            /(RAND_MAX+1.0F) - stim.SMP_distSizeY/divider;

        }
        else
        {
          if((currDistShape[i][j] == SM_STIM_DISK)  ||
             (currDistShape[i][j] == SM_STIM_CROSS) ||
             (currDistShape[i][j] == SM_STIM_PATCH) ||
             (currDistShape[i][j] == SM_STIM_CIRC))
            divider = 4.0F;
          else
            divider = 2.0F;


          posJitterX = ((float)stim.SMP_targetSizeX)
            *(stim.SMP_shapePositionJitterStatic*rand())
            /(RAND_MAX+1.0F) - stim.SMP_distSizeX/divider;
          posJitterY = ((float)stim.SMP_targetSizeY)
            *(stim.SMP_shapePositionJitterStatic*rand())
            /(RAND_MAX+1.0F) - stim.SMP_distSizeY/divider;
        }
      }
      else
      {
        posJitterX = 0.0F;
        posJitterY = 0.0F;
      }
      currPosX[i][j] = (ushort)(round(posJitterX + currPosX[i][j]));
      currPosY[i][j] = (ushort)(round(posJitterY + currPosY[i][j]));

      // static jitter orientation if needed
      float oriJitter = 0.0F;
      if((j == stim.SMP_targetPosJ) && (i == stim.SMP_targetPosI))
      {
        if(stim.SMP_shapeOrientationJitterStatic != 0.0F)
        {
          oriJitter  = stim.SMP_shapeOrientationJitterStatic *
            (((2.0F*M_PI*rand())/(RAND_MAX+1.0F)) - M_PI);
        }
        currDistOrientation[i][j] = oriJitter + stim.SMP_targetOri;
      }
      else
      {
        if(stim.SMP_shapeOrientationJitterStatic != 0.0F)
        {
          oriJitter  = stim.SMP_shapeOrientationJitterStatic *
            (((2.0F*M_PI*rand())/(RAND_MAX+1.0F)) - M_PI);
        }
        currDistOrientation[i][j] = oriJitter + stim.SMP_distOri;
      }
    }
  }

  LINFO("Drawing stim images");
  std::vector<Image<PixRGB<float> > >::iterator framesIter = itsFrames.begin();
  while(framesIter != itsFrames.end())
  {
    // go over each stim
    // be sure not to write over the target

    for(uint i = 0; i < stim.SMP_distPerRow; i++)
    {
      for(uint j = 0; j < stim.SMP_distPerCol; j++)

      {
               //std::cerr << ".";
        // insert the target
        if((j == stim.SMP_targetPosJ) && (i == stim.SMP_targetPosI))
        {
          // is the target on or have no blink rate?
          if((currTargOn) || (currTargBlinkRate == 0))
          {
            SM_drawGroundTruth((ushort)frame,
                               (unsigned char)currTargSizeX,
                               (unsigned char)currTargSizeY,
                               (ushort)currPosX[i][j], (ushort)currPosY[i][j],
                               true);
            //std::cerr << "+";
            // draw the target on this image

            SM_drawSingleTarget((ushort)frame,
                                (unsigned char)currTargShape,
                                (unsigned char)currTargColor,
                                (unsigned char)currTargSizeX,
                                (unsigned char)currTargSizeY,
                                (ushort)currPosX[i][j], (ushort)currPosY[i][j],
                                currDistOrientation[i][j],
                                stim.SMP_shapePositionJitter,
                                stim.SMP_shapeOrientationJitter, true);
          }
          else
          {
            SM_drawGroundTruth((ushort)frame,
                               (unsigned char)currTargSizeX,
                               (unsigned char)currTargSizeY,
                               (ushort)currPosX[i][j], (ushort)currPosY[i][j],
                               false);
          }

          //std::cerr << "x";

          bool checkSpeed = false;
          if(currTargBlinkRate == 0)
            checkSpeed = true;
          else if(((frame+currTargBlinkOffset)
                   %currTargBlinkRate) == 0)
            checkSpeed = true;

          if(checkSpeed)
          {
            if(currTargBlinkRate != 0)
            {
              if(currTargOn)
                currTargOn = false;
              else
                currTargOn = true;
            }
            else
              currTargOn = true;
            // determin if we need to speed up the distractors
            // speed up or start (if possible)
            if(stim.SMP_targetState == SM_STATE_START)
            {
              const uint newRate = SM_speedUpCheck(
                                       currTargBlinkRate,
                                       frame,changeFrame,
                                       stim.SMP_targetRate,
                                       stim.SMP_useSmoothRateChange);
              if(newRate != currTargBlinkRate)
              {
                currTargBlinkOffset = -1*frame;
                currTargBlinkRate   = newRate;
              }
            }
            // determin if we need to slow down the distractors
            // slow down or stop (if possible)
            if(stim.SMP_targetState == SM_STATE_STOP)
            {
              const uint newRate = SM_speedDownCheck(
                                       currTargBlinkRate,
                                       frame,changeFrame,
                                       stim.SMP_targetRate,
                                       stim.SMP_useSmoothRateChange);
              if(newRate != currTargBlinkRate)
              {
                currTargBlinkOffset = -1*frame;
                currTargBlinkRate   = newRate;
              }
            }
          }
        }
        else
        {
          // is this target "on" in this frame
          // if the speed is set to zero, we are always "on"
          if((currDistOn[i][j]) || (currDistBlinkRate[i][j] == 0))
          {
            //std::cerr << "-";
            // draw the target on this image
            SM_drawSingleTarget((ushort)frame,
                                (unsigned char)currDistShape[i][j],
                                (unsigned char)currDistColor[i][j],
                                (unsigned char)currDistSizeX[i][j],
                                (unsigned char)currDistSizeY[i][j],
                                (ushort)currPosX[i][j], (ushort)currPosY[i][j],
                                currDistOrientation[i][j],
                                stim.SMP_shapePositionJitter,
                                stim.SMP_shapeOrientationJitter, false);
          }
          //std::cerr << "|";
          // turn target on or off at interval
          // If the current frame plus the offset is modulo 0 with the
          // rate of blink then turn it on or off
          /*
=======
          bool checkSpeed = false;
          if(currTargBlinkRate == 0)
            checkSpeed = true;
          else if(((frame+currTargBlinkOffset)
                   %currTargBlinkRate) == 0)
            checkSpeed = true;
>>>>>>> .r6413

          if(checkSpeed)
          {
            if(currTargBlinkRate != 0)
            {
              if(currTargOn)
                currTargOn = false;
              else
                currTargOn = true;
            }
            else
              currTargOn = true;
            // determin if we need to speed up the distractors
            // speed up or start (if possible)
            if(stim.SMP_targetState == SM_STATE_START)
            {
              const uint newRate = SM_speedUpCheck(
                                       currTargBlinkRate,
                                       frame,changeFrame,
                                       stim.SMP_targetRate,
                                       stim.SMP_useSmoothRateChange);
              if(newRate != currTargBlinkRate)
              {
                currTargBlinkOffset = -1*frame;
                currTargBlinkRate   = newRate;
              }
            }
            // determin if we need to slow down the distractors
            // slow down or stop (if possible)
            if(stim.SMP_targetState == SM_STATE_STOP)
            {
              const uint newRate = SM_speedDownCheck(
                                       currTargBlinkRate,
                                       frame,changeFrame,
                                       stim.SMP_targetRate,
                                       stim.SMP_useSmoothRateChange);
              if(newRate != currTargBlinkRate)
              {
                currTargBlinkOffset = -1*frame;
                currTargBlinkRate   = newRate;
              }
            }
          }

        }
        else
        {
          // is this target "on" in this frame
          // if the speed is set to zero, we are always "on"
          if((currDistOn[i][j]) || (currDistBlinkRate[i][j] == 0))
          {
            std::cerr << "-";
            // draw the target on this image
            SM_drawSingleTarget((ushort)frame,
                                (unsigned char)currDistShape[i][j],
                                (unsigned char)currDistColor[i][j],
                                (unsigned char)currDistSizeX[i][j],
                                (unsigned char)currDistSizeY[i][j],
                                (ushort)currPosX[i][j], (ushort)currPosY[i][j],
                                stim.SMP_distOri,
                                stim.SMP_shapePositionJitter,
                                stim.SMP_shapeOrientationJitter, false);
          }
          std::cerr << "|";
          */
          // turn target on or off at interval
          // If the current frame plus the offset is modulo 0 with the
          // rate of blink then turn it on or off


          bool checkSpeed = false;
          if(currDistBlinkRate[i][j] == 0)
            checkSpeed = true;
          else if(((frame+currDistBlinkOffset[i][j])
                   %currDistBlinkRate[i][j]) == 0)checkSpeed = true;

          if(checkSpeed)
          {

            if(currDistBlinkRate[i][j] != 0)
            {

              if(currDistOn[i][j])
                currDistOn[i][j] = false;
              else
                currDistOn[i][j] = true;
            }
            else
              currDistOn[i][j] = true;
            // determin if we need to speed up the distractors
            // speed up or start (if possible)

            if(stim.SMP_distState == SM_STATE_START)
            {

               const uint newRate = SM_speedUpCheck(
                                        currDistBlinkRate[i][j],
                                        frame,changeFrame,
                                        stim.SMP_distRate,
                                        stim.SMP_useSmoothRateChange);
              if(newRate != currDistBlinkRate[i][j])
              {
                  currDistBlinkOffset[i][j] = -1*frame;
                  currDistBlinkRate[i][j]   = newRate;
              }
            }

            // determin if we need to slow down the distractors
            // slow down or stop (if possible)
            if(stim.SMP_distState == SM_STATE_STOP)
            {

              const uint newRate = SM_speedDownCheck(
                                        currDistBlinkRate[i][j],
                                        frame,changeFrame,
                                        stim.SMP_distRate,
                                        stim.SMP_useSmoothRateChange);
              if(newRate != currDistBlinkRate[i][j])
              {
                currDistBlinkOffset[i][j] = -1*frame;
                currDistBlinkRate[i][j]   = newRate;
              }
            }
          }
        }
      }
    }

    frame++; ++framesIter;
    //std::cerr << "\n";
  }
  LINFO("Drawing complete");
}

/*****************************************************************************/

std::vector<Image<PixRGB<float> > > StimMaker::SM_getStim() const
{
  return itsFrames;
}

/*****************************************************************************/

std::vector<Image<PixRGB<float> > > StimMaker::SM_getGroundTruth() const
{
  return itsGroundTruth;
}

#endif // STIM_MAKER_H_DEFINED




