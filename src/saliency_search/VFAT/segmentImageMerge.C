/*!@file VFAT/segmentImageMerge.C Basic image segmenter blob finder using color */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/segmentImageMerge.C $
// $Id: segmentImageMerge.C 9412 2008-03-10 23:10:15Z farhan $
//

#include "VFAT/segmentImageMerge.H"

#include "Util/Assert.H"

#include <cstdio>
#include <cstdlib>
#include <iostream>

//! initial vergance values
#define CAMERAMU1         12.0F
#define CAMERAMU2         23.0F
#define CAMERAMU3         34.0F
#define CAMERASIGMA1      33.0F
#define CAMERASIGMA2      56.0F
#define CAMERASIGMA3      67.0F

//! width distance to center between cameras in inches
/* e.g. take the distance between cameras and divide by half */
#define DISTANCE          5

//! maximum lose of tracks before color is reset
#define LOTMAX            5

//! How many iteration to calculate over for movement statistics
#define ERRINTERVAL       5

void segmentImageMerge::colorProcessBlobs(int instance)
{
  float H1,S1,V1,Hs1,Ss1,Vs1;
  float mass;
  H1 = 0; S1 = 0; V1 = 0;
  Hs1 = 0; Ss1 = 0; Vs1 = 0;
  mass = 0;

  // iterate over all blobs and asses new HSV thresholds from
  // candidate blobs
  for(int i = 0; i < segment[instance].numberBlobs(); i++)
  {
    if(track[instance].isCandidate(i) == true)
    {
      float H2,S2,V2,Hs2,Ss2,Vs2;
      // get mean value for this candidate blob, these values will be used
      // to create new adaptive color
      segment[instance].getHSVvalueMean(i,&H2,&S2,&V2,&Hs2,&Ss2,&Vs2);

      // Add in HSV values for each blob times the blob size
      // to make larger blobs give more push on mean values
      H1 += H2 * segment[instance].getMass(i);
      S1 += S2 * segment[instance].getMass(i);
      V1 += V2 * segment[instance].getMass(i);
      Hs1 += Hs2 * segment[instance].getMass(i);
      Ss1 += Ss2 * segment[instance].getMass(i);
      Vs1 += Vs2 * segment[instance].getMass(i);
      // increment mass
      mass += segment[instance].getMass(i);

      // find boundaries of this blob
      int tt = segment[instance].getYmin(i);
      int bb = segment[instance].getYmax(i);
      int ll = segment[instance].getXmin(i);
      int rr = segment[instance].getXmax(i);


      // draw bounding box for this blob
      // Note: box must be of height > 1 and width > 1
      if((bb != tt) && (ll != rr))
        drawRect(*imageHold, Rectangle::tlbrI(tt*4,ll*4,bb*4,rr*4),
                 PixRGB<byte>(boxRed[instance],
                              boxGreen[instance],
                              boxBlue[instance]),1);

      // draw target circle for this blob
      drawCircle(*imageHold, Point2D<int>((int)segment[instance].getCenterX(i)*4
                                     ,(int)segment[instance].getCenterY(i)*4)
                 ,(int)sqrt((double)segment[instance].getMass(i)),
                 PixRGB<byte>(circleRed[instance],
                              circleGreen[instance],
                              circleBlue[instance]),2);

      drawCircle(*imageHold, Point2D<int>((int)segment[instance].getCenterX(i)*4
                                     ,(int)segment[instance].getCenterY(i)*4)
                 ,2,PixRGB<byte>(255,0,0),2);

    }
    else
    {
      if(track[instance].wasKilledByTrack(i) == false)
      {
              // find boundaries of this blob
        int tt = segment[instance].getYmin(i);
        int bb = segment[instance].getYmax(i);
        int ll = segment[instance].getXmin(i);
        int rr = segment[instance].getXmax(i);


        // draw bounding box for this blob
        // Note: box must be of height > 1 and width > 1
        if((bb != tt) && (ll != rr))
          drawRect(*imageHold, Rectangle::tlbrI(tt*4,ll*4,bb*4,rr*4),
                   PixRGB<byte>(circleRed[instance],
                                circleGreen[instance],
                                circleBlue[instance]),1);
      }
    }
  }

  // draw background grid in HSV bar graph
  if(fast != true)
    drawGrid(*auxHold, 25,25,1,1,PixRGB<byte>(50,50,50));

  // draw HSV bar graph if blobs have any mass
  if(mass != 0)
  {
    // figure out HSV bar values
    H1 = H1/mass;  S1 = S1/mass; V1 = V1/mass;
    Hs1 = Hs1/mass;  Ss1 = Ss1/mass; Vs1 = Vs1/mass;
    //std::cout << "H " << H1 << " S " << S1 << " V " << V1 << "\n";
    //std::cout << "Hs " << Hs1 << " Ss " << Ss1 << " Vs " << Vs1 << "\n";
    float htemp = H1-Hs1+6;
    float stemp = ((S1-Ss1)*100)+6;
    float vtemp = V1-Vs1+6;
    if(htemp <= 1) htemp = 1;
    if(stemp <= 1) stemp = 1;
    if(vtemp <= 1) vtemp = 1;
    if(fast != true)
    {

      // draw HSV mean value bars as a bunch of rectangles
      drawRect(*auxHold, Rectangle::tlbrI(5,5,((int)H1+6),20),
               PixRGB<byte>(255,0,0),1);
      drawRect(*auxHold, Rectangle::tlbrI(5,25,(int)(S1*100)+6,40),
               PixRGB<byte>(0,255,0),1);
      drawRect(*auxHold, Rectangle::tlbrI(5,45,((int)V1+6),60),
               PixRGB<byte>(0,0,255),1);

      // draw standard deviation bars
      drawRect(*auxHold, Rectangle::tlbrI((int)(htemp),10,
                                   (int)(H1+Hs1+6),15),
               PixRGB<byte>(255,0,0),1);
      drawRect(*auxHold, Rectangle::tlbrI((int)(stemp),30,
                                   (int)(((S1+Ss1)*100)+6),35),
               PixRGB<byte>(0,255,0),1);
      drawRect(*auxHold, Rectangle::tlbrI((int)(vtemp),50,
                                   (int)(V1+Vs1+6),55),
               PixRGB<byte>(0,0,255),1);

    // find total mass of all candidate blobs
    int massCalc = (int)((mass/(fimaHold->getWidth()*fimaHold->getHeight()))*450);
    drawRect(*auxHold, Rectangle::tlbrI(5,65,massCalc+6,80),
             PixRGB<byte>(255,0,255),1);
    }
  }

  if(fast != true)
  {
    // Draw hard constraint bars into HVS bar graph
    drawRect(*auxHold, Rectangle::tlbrI(((int)HL[instance]+5),12,
                                 ((int)HU[instance]+6),13),
             PixRGB<byte>(255,255,0),1);
    drawRect(*auxHold, Rectangle::tlbrI((int)(SL[instance]*100)+5,32,
                                 (int)(SU[instance]*100)+6,33),
             PixRGB<byte>(255,255,0),1);
    drawRect(*auxHold, Rectangle::tlbrI(((int)VL[instance]+5),52,
                                 ((int)VU[instance]+6),53),
             PixRGB<byte>(255,255,0),1);
  }

  // If loss of track is registered 5 times, reset color values to start up defualt values
  if(track[instance].returnLOT() == true)
  {
    //LINFO("LOT on %d, Number %d",instance, LOTcount[instance]);
    if(LOTcount[instance] > LOTMAX)
    {
      segment[instance].setHue(H[instance],Hstd[instance],0);
      segment[instance].setSat(S[instance],Sstd[instance],0);
      segment[instance].setVal(V[instance],Vstd[instance],0);
      LOTcount[instance] = 0;
      track[instance].reset();
    }
    else
    {
      LOTcount[instance]++;
    }
  }
  else
  {

    //LINFO("Get Value %d - %f,%f,%f,%f,%f,%f",instance,H1,S1,V1,Hs1,Ss1,Vs1);
    float hadj, sadj, vadj;

    //if adaptive thresholding is turned on, adjust color by standard deviation of color
    if(HASTD[instance] == true)
      hadj = Hs1*HA[instance];
    else
      hadj = HA[instance];

    if(SASTD[instance] == true)
      sadj = Ss1*SA[instance];
    else
      sadj = SA[instance];

    if(VASTD[instance] == true)
      vadj = Vs1*VA[instance];
    else
      vadj = VA[instance];

    // if adaptation is true, adapt new color to mean values with new standard deviation
    // but only within boundry constaints
    if(adpt[instance] == true)
    {
      if(H1 > HU[instance]) H1 = HU[instance];
      if(H1 < HL[instance]) H1 = HL[instance];
      if(S1 > SU[instance]) S1 = SU[instance];
      if(S1 < SL[instance]) S1 = SL[instance];
      if(V1 > VU[instance]) V1 = VU[instance];
      if(V1 < VL[instance]) V1 = VL[instance];
      //LINFO("Set Value %d - %f,%f,%f,%f,%f,%f",instance,
      //  H1,hadj,S1,sadj,V1,vadj);
      segment[instance].setHue(H1,hadj,0);
      segment[instance].setSat(S1,sadj,0);
      segment[instance].setVal(V1,vadj,0);
    }
  }
}

/************************/
/* START PUBLIC METHODS */
/***********************/

// When called at the start, this will resize all the vectors we use
segmentImageMerge::segmentImageMerge(int instances)
{
  instanceNumber = instances;
  H.resize(instances,0);
  S.resize(instances,0);
  V.resize(instances,0);
  Hstd.resize(instances,0);
  Sstd.resize(instances,0);
  Vstd.resize(instances,0);
  HA.resize(instances,3);
  SA.resize(instances,3);
  VA.resize(instances,3);
  HU.resize(instances,360);
  SU.resize(instances,1);
  VU.resize(instances,255);
  HL.resize(instances,0);
  SL.resize(instances,0);
  VL.resize(instances,0);
  delay.resize(instances,0);
  cameraMovePan.resize(instances,90);
  cameraMoveTilt.resize(instances,90);
  cameraGotoPan.resize(instances,90);
  cameraGotoTilt.resize(instances,90);
  cameraMu.resize((instances-1),0);
  cameraSigma.resize((instances-1),0);
  meanMove.resize(instances,0);

  stdMove.resize(instances,0);

  std::vector<float> temp;
  temp.resize(ERRINTERVAL,0);
  moveRecord.resize(instances,temp);
  moveRecordGrad.resize(instances,temp);
  LOTcount.resize(instances,0);
  height.resize(instances,0);
  width.resize(instances,0);
  gotoX.resize(instances,0);
  gotoY.resize(instances,0);
  circleRed.resize(instances,0);
  circleGreen.resize(instances,0);
  circleBlue.resize(instances,0);
  boxRed.resize(instances,0);
  boxGreen.resize(instances,0);
  boxBlue.resize(instances,0);
  didCircleColor.resize(instances,0);
  didBoxColor.resize(instances,0);
  didTrackColor.resize(instances,0);
  recordCounter.resize(instances,0);
  adpt.resize(instances,true);
  HASTD.resize(instances,false);
  SASTD.resize(instances,false);
  VASTD.resize(instances,false);
  moveCamera.resize(instances,false);
  temp.resize(instances);
  Timer Ttemp;
  tim.resize(instances,Ttemp);
  //segmentImage stmp;
  //segment.resize(instances,stmp);
  segment = new segmentImage[instances];
  segmentImageTrack sttmp;
  track.resize(instances,sttmp);
  for(int i = 0; i < instances; i++)
  {
    track[i].setUpVars(1000);
    track[i].setImage(&segment[i]);
  }
  cameraMu[0] = CAMERAMU1;
  cameraMu[1] = CAMERAMU2;
  cameraMu[2] = CAMERAMU3;
  cameraSigma[0] = CAMERASIGMA1;
  cameraSigma[1] = CAMERASIGMA2;
  cameraSigma[2] = CAMERASIGMA3;
}

segmentImageMerge::~segmentImageMerge()
{}

void segmentImageMerge::setCircleColor(int r, int g, int b, int instance)
{
  circleRed[instance] = r;
  circleBlue[instance] = g;
  circleGreen[instance] = b;
  didCircleColor[instance] = 1;
}

void segmentImageMerge::setBoxColor(int r, int g, int b, int instance)
{
  boxRed[instance] = r;
  boxBlue[instance] = g;
  boxGreen[instance] = b;
  didBoxColor[instance] = 1;
}

void segmentImageMerge::setTrackColor(float h, float hstd,
                                      float s, float sstd,
                                      float v, float vstd,
                                      int instance, bool adapt, int avg)
{
  H[instance] = h;
  S[instance] = s;
  V[instance] = v;
  Hstd[instance] = hstd;
  Sstd[instance] = sstd;
  Vstd[instance] = vstd;
  adpt[instance] = adapt;
  segment[instance].setHue(H[instance],Hstd[instance],0);
  segment[instance].setSat(S[instance],Sstd[instance],0);
  segment[instance].setVal(V[instance],Vstd[instance],0);
  didTrackColor[instance] = 1;
  segment[instance].setHSVavg(avg);
}

void segmentImageMerge::setAdapt(float ha, bool haSTD, float sa, bool saSTD,
                                 float va, bool vaSTD, int instance)
{
  HA[instance] = ha;
  SA[instance] = sa;
  VA[instance] = va;
  HASTD[instance] = haSTD;
  SASTD[instance] = saSTD;
  VASTD[instance] = vaSTD;
}

void segmentImageMerge::setAdaptBound(float Hupper, float Hlower,
                   float Supper, float Slower,
                   float Vupper, float Vlower,
                   int instance)
{
  HU[instance] = Hupper;
  SU[instance] = Supper;
  VU[instance] = Vupper;
  HL[instance] = Hlower;
  SL[instance] = Slower;
  VL[instance] = Vlower;
}

void segmentImageMerge::setCameraPosition(float pan, float tilt, int instance
                                          , bool stats)
{
  // record camera movement
  cameraMovePan[instance] = pan;
  cameraMoveTilt[instance] = tilt;
  if(stats == true)
  {
    int doThisItem;
    float move = sqrt(pow(pan,2)+pow(tilt,2));
    if(recordCounter[instance] != 0)
    {
      doThisItem = instance - 1;
    }
    else
    {
      doThisItem = ERRINTERVAL - 1;
    }

    // Calculate finate state gradiant from last iteration to this one and record
    moveRecordGrad[instance][recordCounter[instance]] =
      move - moveRecord[instance][recordCounter[doThisItem]] ;
    moveRecord[instance][recordCounter[instance]] = move;

    float sumP = 0;
    float SSP = 0;

    // calcuate mean movements of camera servos
    for(int i = 0; i < ERRINTERVAL; i++)
    {
      sumP += moveRecordGrad[instance][i];
    }
    meanMove[instance] = sumP/ERRINTERVAL;

    // calculate standard deviation of camera servos
    for(int i = 0; i < ERRINTERVAL; i++)
    {
      SSP += pow((meanMove[instance] - moveRecordGrad[instance][i]),2);
    }
    stdMove[instance] = sqrt(SSP/ERRINTERVAL);

    //LINFO("CAM %d Move STD %f",instance,stdMove[instance]);

    // increment counter
    if(recordCounter[instance] < ERRINTERVAL)
      recordCounter[instance]++;
    else
      recordCounter[instance] = 0;
  }
}

void segmentImageMerge::setFrame(int x1, int y1, int x2, int y2,
                                 int realX, int realY, int instance)
{
  segment[instance].setFrame(x1,y1,x2,y2,realX,realY);
}

/* This is a basic tracker access method that tracks on one image at a time */
void segmentImageMerge::trackImage(Image<PixRGB<byte> > input,
                                   Image<PixRGB<byte> > *image, int instance,
                                   Image<PixRGB<byte> > *auxImage, bool _fast)
{
  // Assert that parameters have been set up before starting
  fast = _fast;
  ASSERT(didCircleColor[instance] == 1);
  ASSERT(didBoxColor[instance] == 1);
  ASSERT(didTrackColor[instance] == 1);
  imageHold = image;
  auxHold = auxImage;

  Image< PixRGB<float> > fima;

  // decimate input image twice to speed things up
  fima = decXY(input);
  fima = decXY(fima);

  fimaHold = &fima;

  // segment the decimated image
  segment[instance].segment(fima);

  // get center of mass for blobs
  segment[instance].calcMassCenter();

  // edit blobs, weed out all the non-hackers who are not fit to carry a rifle
  track[instance].track();
  // apply adaptive color thesholding
  colorProcessBlobs(instance);

}

// END

// the statistical multi image tracker we will build
void segmentImageMerge::trackImageMulti(
                  std::vector<Image<PixRGB<byte> > > *image, int instances)
{
  fast = true;
  Image< PixRGB<float> > fima;

  for(int i = 0; i < instances; i++)
  {
    ASSERT(didCircleColor[i] == 1);
    ASSERT(didBoxColor[i] == 1);
    ASSERT(didTrackColor[i] == 1);

    imageHold = &(image->at(i));
    fima = decXY(image->at(i));
    fima = decXY(fima);

    // Color segment this instance
    segment[i].segment(fima);

    // get center of mass for blobs
    segment[i].calcMassCenter();

    // edit blobs, weed out all the non-hackers who are not fit to carry a rifle
    track[i].track(0);
  }

  moveMeanNormal = 0;
  moveStdNormal = 0;

  // Normalize over movement statisitcs to apply them in the next iterations
  for(int i = 0; i < instances; i++)
  {
    moveMeanNormal += meanMove[i];
    moveStdNormal += stdMove[i];
  }
  //avoid divide by zero error
  moveMeanNormal += .000001;
  moveStdNormal += .000001;
  updateVergance(48,36);

  for(int i = 0; i < instances; i++)
  {
    imageHold = &(image->at(i));
    //compute vergance springs for each camera
    verganceSpring(instances,i,true);

    // apply adaptive color thesholding
    colorProcessBlobs(i);
  }
}

void segmentImageMerge::mergeImages(Image<PixRGB<byte> > *image)
{
  mergeGotoX = 0; mergeGotoY = 0;
  int mergeCount = 0;
  for(int i = 0; i < instanceNumber; i++)
  {
    gotoX[i] = track[i].getObjectX();
    gotoY[i] = track[i].getObjectY();
    if(track[i].returnLOT() == false)
    {
      mergeGotoX += gotoX[i];
      mergeGotoY += gotoY[i];
      mergeCount++;
    }
  }
  if(mergeCount != 0)
  {
    mergeGotoX = mergeGotoX/mergeCount;
    mergeGotoY = mergeGotoY/mergeCount;
  }
  drawCircle(*image, Point2D<int>((int)mergeGotoX*4
                             ,(int)mergeGotoY*4)
             ,10,PixRGB<byte>(255,0,0),2);
}

void segmentImageMerge::updateVergance(float distance, float gaussBase)
{
  for(int i = 0; i < (instanceNumber-1); i++)
  {
    //this is the angle to the target from two apposing cameras.
    //Mu is then the differenc between these angles and 90 * 2
    cameraMu[i] = 2*(90-(((2*atan(distance/(DISTANCE*(i+1))))/3.14159)*90));
    // the base angle for something at three feet from target
    // i.e. make the gaussian three feet in diameters
    float baseAngle = 2*(90-(((2*atan((distance-gaussBase)
                                      /(DISTANCE*(i+1))))/3.14159)*90));
    cameraSigma[i] = fabs(baseAngle-cameraMu[i]);
    //LINFO("UPDATE VERGANCE camera %d, Mu %f STD %f",i,cameraMu[i],cameraSigma[i]);
  }
}

void segmentImageMerge::verganceSpring(int instances, int current, bool doTracked)
{

  float theta, phi;
  int seperation;


  int maxBlob = -1;
  float maxBlobVal = 0;

  // if we didn't lose track and we want to vergance on cameras
  // that are tracking...
  if((track[current].returnLOT() == false) && (doTracked == true))
  {
    moveCamera[current] = false;

    // for each blob this camera is tracking do
    for(int x = 0; x < segment[current].numberBlobs(); x++)
    {
      // check to make sure we havn't already disqualified this blob
      if(track[current].isCandidate(x) == true)
      {
        track[current].pVergance[x] = 0;
        // calculate the angle to the target blob being analized at the moment
        float gotoCY = fabs((480 - segment[current].getCenterY(x)*8)
                           - camera.Ypixel);

        float panConv = ((float)camera.Xfield/(float)camera.Xpixel);
        float tiltConv = ((float)camera.Yfield/(float)camera.Ypixel);

        float panOff = ((float)camera.Xpixel*.5)-
          segment[current].getCenterX(x)*8;
        float tiltOff = ((float)camera.Ypixel*.5)-gotoCY;

        float travelPan = cameraMovePan[current] +
          ((panOff*panConv)*camera.fieldAdjustmentX);
        float travelTilt = cameraMoveTilt[current] +
          ((tiltOff*tiltConv)*camera.fieldAdjustmentY);

        // cycle over other camera positions
        //and calculate the p of vergance for this camera
        for(int j = 0; j < instances; j++)
        {
          if(j != current)
          {
            if(j < current)
              theta = travelPan - cameraMovePan[j];
            else
              theta = cameraMovePan[j] - travelPan;

            phi = fabs(travelTilt - cameraMoveTilt[j]);
            seperation = abs(current - j);

            // p += vergance(tilt,cam(x))*vergance(pan,cam(x))
            track[current].pVergance[x] +=
              (Stats.gauss(theta,cameraMu[seperation-1],
                           cameraSigma[seperation-1])
               *Stats.gauss(phi,0.0F,21.0F))*(1-(stdMove[j]/moveStdNormal));

          }
        }
        // if I have the highest P of all the blobs in this
        // instance (camera) so far, I win. Take argmax
        if(track[current].pVergance[x] >= maxBlobVal)
        {
          if(maxBlob != -1)
          {
            // turn off this blob, it's no good
            track[current].setCandidate(maxBlob,false);
          }
          maxBlob = x;
          // set this blob as the best one
          maxBlobVal = track[current].pVergance[x];
        }
        else
        {
          // turn off this blob, it not better than anyone
          track[current].setCandidate(x,false);
        }
      }
    }
  }
  else
  {
    // this camera is in a LOT, send it to a vergance coordinate;
    if(LOTcount[current] > LOTMAX)
    {
      moveCamera[current] = true;
      float doPan = 0;
      float doTilt = 0;
      int normal = 0;

      // for all cameras not in LOT, go to the average vergance
      // over those cameras
      // e.g. you vergance should reflect the ones tracking
      for(int k = 0; k < instances; k++)
      {

        if((k != current) && (track[k].returnLOT() == false))
        {
          seperation = abs(current - k);
          // you should converge to another camera based upon the
          // P derived from the gradiant of its
          // movement. This is cameras that are more fluid have more influince.
          if(k < current)
          {
            doPan += cameraMovePan[k]*(1 - stdMove[k]/moveStdNormal)
              + cameraMu[seperation-1];

          }
          else
          {
            doPan += cameraMovePan[k]*(1 - stdMove[k]/moveStdNormal)
              - cameraMu[seperation-1];
          }
          doTilt += cameraMoveTilt[k]*(1 - stdMove[k]/moveStdNormal);
          normal++;
        }
      }
      if(normal != 0)
      {
        // if we can be biased by at least one camera do this
        cameraGotoPan[current] = doPan/normal;
        cameraGotoTilt[current] = doTilt/normal;
      }
    }
  }
}

void segmentImageMerge::getImageTrackXY(int *x, int *y, int instance)
{
  *x = gotoX[instance];
  *y = gotoY[instance];
}

void segmentImageMerge::getImageTrackXY2(int *x, int *y, int instance)
{
   *x = track[instance].getObjectX();
   *y = track[instance].getObjectY();
}

void segmentImageMerge::getImageTrackXYMerge(int *x, int *y)
{
  *x = mergeGotoX;
  *y = mergeGotoY;
}

bool segmentImageMerge::returnLOT(int instance)
{
  return track[instance].returnLOT();
}

float segmentImageMerge::returnCameraProb(int instance)
{
  return (1-stdMove[instance]/moveStdNormal);
}

bool segmentImageMerge::doMoveCamera(int instance, float *doPan, float *doTilt)
{
  *doPan = cameraGotoPan[instance];
  *doTilt = cameraGotoTilt[instance];

  // calculate gradiant variance
  return moveCamera[instance];
}

Image<byte> segmentImageMerge::returnCandidateImage(int instance)
{
  return segment[instance].returnNormalizedCandidates();
}

bool segmentImageMerge::StereoMatch(PixelPoint points[2],
                                    CameraParams params[2],
                                    Point3D* retPoint)
{
    float PI = 3.14159;
    float deg2rad = PI/180.0;

    //define the std deviations of the error function(gaussian) for
    //various parameters
    Point3D* P = (Point3D*)calloc(2, sizeof(Point3D));
    P[0] = Point3D(0.0, 0.0, 0.0);
    P[1] = Point3D(0.0, 0.0, 0.0);

    Point3D* f = (Point3D*)calloc(2, sizeof(Point3D));
    f[0] = Point3D(0.0, 0.0, 0.0);
    f[1] = Point3D(0.0, 0.0, 0.0);

    //get ideal case values and max error values
    for(int i=0; i<2; i++)
    {
      P[i].x = params[i].x + params[i].r*sin(params[i].theta*deg2rad)
        * cos(params[i].phi*deg2rad)
        + points[i].x * cos(params[i].theta*deg2rad)
        * cos(params[i].phi*deg2rad)
        - points[i].y * sin(params[i].phi*deg2rad);

      P[i].y = params[i].y + params[i].r*sin(params[i].theta*deg2rad)
        * sin(params[i].phi*deg2rad)
        + points[i].x * cos(params[i].theta*deg2rad)
        * sin(params[i].phi*deg2rad)
        + points[i].y * cos(params[i].phi*deg2rad);

      P[i].z = params[i].z + params[i].r*cos(params[i].theta*deg2rad)
        - points[i].x * sin(params[i].theta*deg2rad);

      f[i].x = params[i].x +params[i].r*sin(params[i].theta*deg2rad)
        * cos(params[i].phi*deg2rad)
        + params[i].f * sin(params[i].theta*deg2rad)
        * cos(params[i].phi*deg2rad);

      f[i].y = params[i].y + params[i].r*sin(params[i].theta*deg2rad)
        * sin(params[i].phi*deg2rad)
        + params[i].f * sin(params[i].theta*deg2rad)
        * sin(params[i].phi*deg2rad);

      f[i].z = params[i].z + params[i].r*cos(params[i].theta*deg2rad) +
               params[i].f * cos(params[i].theta*deg2rad);
    }

    float r1 = ((f[1].z-P[1].z)*(P[0].x-P[1].x)
                - (f[1].x-P[1].x)*(P[0].z-P[1].z))/
      ((f[1].x-P[1].x)*(f[0].z-P[0].z)
       - (f[1].z-P[1].z)*(f[0].x-P[0].x)+0.0001);

    float r2 = ((f[0].z-P[0].z)*(P[0].x-P[1].x)
                - (f[0].x-P[0].x)*(P[0].z-P[1].z))/
      ((f[1].x-P[1].x)*(f[0].z-P[0].z)
       - (f[1].z-P[1].z)*(f[0].x-P[0].x)+0.0001);

    float lhs = P[0].y + (f[0].y-P[0].y)*r1;
    float rhs = P[1].y + (f[1].y-P[1].y)*r2;

    //printf("Here!!!!\n");
    if(lhs-rhs>20 || lhs-rhs<-20)
      return false;

    retPoint->x = P[0].x + (f[0].x-P[0].x)*r1;
    retPoint->y = (lhs+rhs)/2.0;
    retPoint->z = P[0].z + (f[0].z-P[0].z)*r1;
    return true;
}

