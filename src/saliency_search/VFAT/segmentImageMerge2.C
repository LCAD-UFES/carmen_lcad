/*!@file VFAT/segmentImageMerge2.C Basic image segmenter blob finder using color */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/segmentImageMerge2.C $
// $Id: segmentImageMerge2.C 15310 2012-06-01 02:29:24Z itti $
//

#include "VFAT/segmentImageMerge2.H"

#include "Util/Assert.H"
#include "Raster/Raster.H"

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

void segmentImageMerge2::SIMcolorProcessBlobs(int instance)
{
  float H1,S1,V1,Hs1,Ss1,Vs1;
  float mass;
  H1 = 0; S1 = 0; V1 = 0;
  Hs1 = 0; Ss1 = 0; Vs1 = 0;
  mass = 0;

  // iterate over all blobs and asses new HSV thresholds from
  // candidate blobs
  for(int i = 0; i < SIM_segment[instance].SInumberBlobs(); i++)
  {
    if(SIM_track[instance].SITisCandidate(i) == true)
    {
      float H2,S2,V2,Hs2,Ss2,Vs2;
      // get mean value for this candidate blob, these values will be used
      // to create new adaptive color
      SIM_segment[instance].SIgetHSVvalueMean(i,&H2,&S2,&V2,&Hs2,&Ss2,&Vs2);

      // Add in HSV values for each blob times the blob size
      // to make larger blobs give more push on mean values
      H1 += H2 * SIM_segment[instance].SIgetMass(i);
      S1 += S2 * SIM_segment[instance].SIgetMass(i);
      V1 += V2 * SIM_segment[instance].SIgetMass(i);
      Hs1 += Hs2 * SIM_segment[instance].SIgetMass(i);
      Ss1 += Ss2 * SIM_segment[instance].SIgetMass(i);
      Vs1 += Vs2 * SIM_segment[instance].SIgetMass(i);
      // increment mass
      mass += SIM_segment[instance].SIgetMass(i);

      // find boundaries of this blob
      int tt = SIM_segment[instance].SIgetYmin(i);
      int bb = SIM_segment[instance].SIgetYmax(i);
      int ll = SIM_segment[instance].SIgetXmin(i);
      int rr = SIM_segment[instance].SIgetXmax(i);


      // draw bounding box for this blob
      // Note: box must be of height > 1 and width > 1
      if((bb != tt) && (ll != rr))
        drawRect(*SIM_imageHold, Rectangle::tlbrI(tt*4,ll*4,bb*4,rr*4),
                 PixRGB<byte>(SIM_boxRed[instance],
                              SIM_boxGreen[instance],
                              SIM_boxBlue[instance]),1);

      // draw target circle for this blob
      drawCircle(*SIM_imageHold, Point2D<int>((int)SIM_segment[instance].SIgetCenterX(i)
                                         *4
                                         ,(int)SIM_segment[instance].SIgetCenterY(i)*4)
                 ,(int)sqrt((double)SIM_segment[instance].SIgetMass(i)),
                 PixRGB<byte>(SIM_circleRed[instance],
                              SIM_circleGreen[instance],
                              SIM_circleBlue[instance]),2);

      drawCircle(*SIM_imageHold, Point2D<int>((int)SIM_segment[instance].SIgetCenterX(i)
                                         *4
                                         ,(int)SIM_segment[instance].SIgetCenterY(i)*4)
                 ,2,PixRGB<byte>(255,0,0),2);

    }
    else
    {
      if(SIM_track[instance].SITwasKilledByTrack(i) == false)
      {
              // find boundaries of this blob
        int tt = SIM_segment[instance].SIgetYmin(i);
        int bb = SIM_segment[instance].SIgetYmax(i);
        int ll = SIM_segment[instance].SIgetXmin(i);
        int rr = SIM_segment[instance].SIgetXmax(i);


        // draw bounding box for this blob
        // Note: box must be of height > 1 and width > 1
        if((bb != tt) && (ll != rr))
          drawRect(*SIM_imageHold, Rectangle::tlbrI(tt*4,ll*4,bb*4,rr*4),
                   PixRGB<byte>(SIM_circleRed[instance],
                                SIM_circleGreen[instance],
                                SIM_circleBlue[instance]),1);
      }
    }
  }

  // draw background grid in HSV bar graph
  if(SIM_fast != true)
    drawGrid(*SIM_auxHold, 25,25,1,1,PixRGB<byte>(50,50,50));

  // draw HSV bar graph if blobs have any mass
  if(mass != 0)
  {
    // figure out HSV bar values
    H1 = H1/mass;  S1 = S1/mass; V1 = V1/mass;
    Hs1 = Hs1/mass;  Ss1 = Ss1/mass; Vs1 = Vs1/mass;
    std::cout << "H " << H1 << " S " << S1 << " V " << V1 << "\n";
    std::cout << "Hs " << Hs1 << " Ss " << Ss1 << " Vs " << Vs1 << "\n";
    float htemp = H1-Hs1+6;
    float stemp = ((S1-Ss1)*100)+6;
    float vtemp = V1-Vs1+6;
    if(htemp <= 1) htemp = 1;
    if(stemp <= 1) stemp = 1;
    if(vtemp <= 1) vtemp = 1;
    if(SIM_fast != true)
    {

      // draw HSV mean value bars as a bunch of rectangles
      drawRect(*SIM_auxHold, Rectangle::tlbrI(5,5,((int)H1+6),20),
               PixRGB<byte>(255,0,0),1);
      drawRect(*SIM_auxHold, Rectangle::tlbrI(5,25,(int)(S1*100)+6,40),
               PixRGB<byte>(0,255,0),1);
      drawRect(*SIM_auxHold, Rectangle::tlbrI(5,45,((int)V1+6),60),
               PixRGB<byte>(0,0,255),1);

      // draw standard deviation bars
      drawRect(*SIM_auxHold, Rectangle::tlbrI((int)(htemp),10,
                                       (int)(H1+Hs1+6),15),
               PixRGB<byte>(255,0,0),1);
      drawRect(*SIM_auxHold, Rectangle::tlbrI((int)(stemp),30,
                                       (int)(((S1+Ss1)*100)+6),35),
               PixRGB<byte>(0,255,0),1);
      drawRect(*SIM_auxHold, Rectangle::tlbrI((int)(vtemp),50,
                                       (int)(V1+Vs1+6),55),
               PixRGB<byte>(0,0,255),1);

    // find total mass of all candidate blobs
    int massCalc = (int)((mass/(SIM_fimaHold->getWidth()
                                *SIM_fimaHold->getHeight()))*450);
    drawRect(*SIM_auxHold, Rectangle::tlbrI(5,65,massCalc+6,80),
             PixRGB<byte>(255,0,255),1);
    }
  }

  if(SIM_fast != true)
  {
    // Draw hard constraint bars into HVS bar graph
    drawRect(*SIM_auxHold, Rectangle::tlbrI(((int)SIM_HL[instance]+5),12,
                                     ((int)SIM_HU[instance]+6),13),
             PixRGB<byte>(255,255,0),1);
    drawRect(*SIM_auxHold, Rectangle::tlbrI((int)(SIM_SL[instance]*100)+5,32,
                                     (int)(SIM_SU[instance]*100)+6,33),
             PixRGB<byte>(255,255,0),1);
    drawRect(*SIM_auxHold, Rectangle::tlbrI(((int)SIM_VL[instance]+5),52,
                                     ((int)SIM_VU[instance]+6),53),
             PixRGB<byte>(255,255,0),1);
  }

  // If loss of track is registered 5 times, reset color
  // values to start up defualt values
  if(SIM_track[instance].SITreturnLOT() == true)
  {
    //LINFO("LOT on %d, Number %d",instance, LOTcount[instance]);
    if(SIM_LOTcount[instance] > LOTMAX)
    {
      SIMresetColor(instance);
    }
    else
    {
      SIM_LOTcount[instance]++;
    }
  }
  else
  {

    //LINFO("Get Value %d - %f,%f,%f,%f,%f,%f",instance,H1,S1,V1,Hs1,Ss1,Vs1);
    float hadj, sadj, vadj;

    // if adaptive thresholding is turned on, adjust color
    // by standard deviation of color
    if(SIM_HASTD[instance] == true)
      hadj = Hs1*SIM_HA[instance];
    else
      hadj = SIM_HA[instance];

    if(SIM_SASTD[instance] == true)
      sadj = Ss1*SIM_SA[instance];
    else
      sadj = SIM_SA[instance];

    if(SIM_VASTD[instance] == true)
      vadj = Vs1*SIM_VA[instance];
    else
      vadj = SIM_VA[instance];

    // if adaptation is true, adapt new color to mean values with
    // new standard deviation
    // but only within boundry constaints
    int count = 0;
    if(SIM_adpt[instance] == true)
    {
      if(H1 > SIM_HU[instance])
      {
        H1 = SIM_HU[instance];
        count++;
      }
      else
        if(H1 < SIM_HL[instance])
        {
          H1 = SIM_HL[instance];
          count++;
        }

      if(S1 > SIM_SU[instance])
      {
        S1 = SIM_SU[instance];
        count++;
      }
      else
        if(S1 < SIM_SL[instance])
        {
          S1 = SIM_SL[instance];
          count++;
        }

      if(V1 > SIM_VU[instance])
      {
        V1 = SIM_VU[instance];
        count++;
      }
      else
        if(V1 < SIM_VL[instance])
        {
          V1 = SIM_VL[instance];
          count++;
        }

      LINFO("Set Value %d - %f,%f,%f,%f,%f,%f",instance,
        H1,hadj,S1,sadj,V1,vadj);
      /*
      if(count > 1)
      {
        if(SIM_LOTcount[instance] > LOTMAX)
        {
          SIMresetColor(instance);
        }
        else
        {
          SIM_LOTcount[instance]++;
        }
      }
      */
      SIM_segment[instance].SIsetHue(H1,hadj,0);
      SIM_segment[instance].SIsetSat(S1,sadj,0);
      SIM_segment[instance].SIsetVal(V1,vadj,0);
    }
  }
}

void segmentImageMerge2::SIMresetColor(int instance)
{

  if(SIM_useCluster[instance] == false)
  {
    LINFO("LOT: Setting new colors from pre-defined for %d",instance);
    SIM_segment[instance].SIsetHue(SIM_H[instance],SIM_Hstd[instance]*5,0);
    SIM_segment[instance].SIsetSat(SIM_S[instance],SIM_Sstd[instance]*5,0);
    SIM_segment[instance].SIsetVal(SIM_V[instance],SIM_Vstd[instance]*5,0);
  }
  else
  {
    LINFO("LOT: Setting new colors from cluster for %d",instance);
    SIMclusterColor(*SIM_fimaHold,instance);
  }
  SIM_LOTcount[instance] = 0;
  SIM_track[instance].SITreset();
}

/************************/
/* START PUBLIC METHODS */
/***********************/

void segmentImageMerge2::resetAll(int instances) {

        delete SIM_segment;

        SIM_clusterSet = false;
  SIM_instanceNumber = instances;
  SIM_useCluster.resize(instances,false);
  SIM_H.resize(instances,0);
  SIM_S.resize(instances,0);
  SIM_V.resize(instances,0);
  SIM_Hstd.resize(instances,0);
  SIM_Sstd.resize(instances,0);
  SIM_Vstd.resize(instances,0);
  SIM_HA.resize(instances,3);
  SIM_SA.resize(instances,3);
  SIM_VA.resize(instances,3);
  SIM_HU.resize(instances,360);
  SIM_SU.resize(instances,1);
  SIM_VU.resize(instances,255);
  SIM_HL.resize(instances,0);
  SIM_SL.resize(instances,0);
  SIM_VL.resize(instances,0);
  SIM_delay.resize(instances,0);
  SIM_cameraMovePan.resize(instances,90);
  SIM_cameraMoveTilt.resize(instances,90);
  SIM_cameraGotoPan.resize(instances,90);
  SIM_cameraGotoTilt.resize(instances,90);
  SIM_cameraMu.resize((instances-1),0);
  SIM_cameraSigma.resize((instances-1),0);
  SIM_meanMove.resize(instances,0);

  SIM_stdMove.resize(instances,0);

  std::vector<float> temp;
  temp.resize(ERRINTERVAL,0);
  SIM_moveRecord.resize(instances,temp);
  SIM_moveRecordGrad.resize(instances,temp);
  SIM_LOTcount.resize(instances,0);
  SIM_height.resize(instances,0);
  SIM_width.resize(instances,0);
  SIM_gotoX.resize(instances,0);
  SIM_gotoY.resize(instances,0);
  SIM_circleRed.resize(instances,0);
  SIM_circleGreen.resize(instances,0);
  SIM_circleBlue.resize(instances,0);
  SIM_boxRed.resize(instances,0);
  SIM_boxGreen.resize(instances,0);
  SIM_boxBlue.resize(instances,0);
  SIM_didCircleColor.resize(instances,0);
  SIM_didBoxColor.resize(instances,0);
  SIM_didTrackColor.resize(instances,0);
  SIM_recordCounter.resize(instances,0);
  SIM_adpt.resize(instances,true);
  SIM_HASTD.resize(instances,false);
  SIM_SASTD.resize(instances,false);
  SIM_VASTD.resize(instances,false);
  SIM_moveCamera.resize(instances,false);
  temp.resize(instances);
  Timer Ttemp;
  SIM_tim.resize(instances,Ttemp);
  //segmentImage stmp;
  //segment.resize(instances,stmp);
  SIM_segment = new segmentImage2[instances];
  segmentImageTrack2 sttmp;
  SIM_track.resize(instances,sttmp);

  blobConf.openFile("blob.conf",true);
  SIM_blobProp.BP_LOTbound = (int)blobConf.getItemValueF("BP_LOTbound");
  SIM_blobProp.BP_bound = (int)blobConf.getItemValueF("BP_bound");
  SIM_blobProp.BP_softBound = (int)blobConf.getItemValueF("BP_softBound");
  SIM_blobProp.BP_lowBound = (int)blobConf.getItemValueF("BP_lowBound");
  SIM_blobProp.BP_traj = (int)blobConf.getItemValueF("BP_traj");
  SIM_blobProp.BP_sampleStart = (int)blobConf.getItemValueF("BP_sampleStart");
  SIM_blobProp.BP_maxTraj = (int)blobConf.getItemValueF("BP_maxTraj");
  SIM_blobProp.BP_maxSize = (int)blobConf.getItemValueF("BP_maxSize");
  SIM_blobProp.BP_minSize = (int)blobConf.getItemValueF("BP_minSize");
  SIM_blobProp.BP_maxFrameSize = blobConf.getItemValueF("BP_maxFrameSize");
  SIM_blobProp.BP_minMass = (int)blobConf.getItemValueF("BP_minMass");
  SIM_blobProp.BP_maxMass = (int)blobConf.getItemValueF("BP_maxMass");
  SIM_blobProp.BP_ratioMin = blobConf.getItemValueF("BP_ratioMin");
  SIM_blobProp.BP_softRatioMin = blobConf.getItemValueF("BP_softRatioMin");
  SIM_blobProp.BP_ratioMax = blobConf.getItemValueF("BP_ratioMax");
  SIM_blobProp.BP_softRatioMax = blobConf.getItemValueF("BP_softRatioMax");
  SIM_blobProp.BP_minClusterSize =
    (int)blobConf.getItemValueF("BP_minClusterSize");
  SIM_blobProp.BP_clusterWeightSpat =
    blobConf.getItemValueF("BP_clusterWeightSpat");
  //! set the weights in cluster for Hue
  SIM_blobProp.BP_clusterWeightH =
    blobConf.getItemValueF("BP_clusterWeightH");
  //! set the weights in cluster for Sat
  SIM_blobProp.BP_clusterWeightS =
    blobConf.getItemValueF("BP_clusterWeightS");
  //! set the weights in cluster for Val
  SIM_blobProp.BP_clusterWeightV =
    blobConf.getItemValueF("BP_clusterWeightV");
  //! set initial standard deviation for color tracking after Hue
  SIM_blobProp.BP_clusterColorStdH =
    blobConf.getItemValueF("BP_clusterColorStdH");
  //! set initial standard deviation for color tracking after Sat
  SIM_blobProp.BP_clusterColorStdS =
    blobConf.getItemValueF("BP_clusterColorStdS");
  //! set initial standard deviation for color tracking after Val
  SIM_blobProp.BP_clusterColorStdV =
    blobConf.getItemValueF("BP_clusterColorStdV");


  for(int i = 0; i < instances; i++)
  {
    SIM_track[i].SITsetBlobProp(&SIM_blobProp);
    SIM_track[i].SITsetUpVars(1000);
    SIM_track[i].SITsetImage(&SIM_segment[i]);
  }

  SIM_cameraMu[0] = CAMERAMU1;
  SIM_cameraMu[1] = CAMERAMU2;
  SIM_cameraMu[2] = CAMERAMU3;
  SIM_cameraSigma[0] = CAMERASIGMA1;
  SIM_cameraSigma[1] = CAMERASIGMA2;
  SIM_cameraSigma[2] = CAMERASIGMA3;

}

// When called at the start, this will resize all the vectors we use
segmentImageMerge2::segmentImageMerge2(int instances)
{
  SIM_clusterSet = false;
  SIM_instanceNumber = instances;
  SIM_useCluster.resize(instances,false);
  SIM_H.resize(instances,0);
  SIM_S.resize(instances,0);
  SIM_V.resize(instances,0);
  SIM_Hstd.resize(instances,0);
  SIM_Sstd.resize(instances,0);
  SIM_Vstd.resize(instances,0);
  SIM_HA.resize(instances,3);
  SIM_SA.resize(instances,3);
  SIM_VA.resize(instances,3);
  SIM_HU.resize(instances,360);
  SIM_SU.resize(instances,1);
  SIM_VU.resize(instances,255);
  SIM_HL.resize(instances,0);
  SIM_SL.resize(instances,0);
  SIM_VL.resize(instances,0);
  SIM_delay.resize(instances,0);
  SIM_cameraMovePan.resize(instances,90);
  SIM_cameraMoveTilt.resize(instances,90);
  SIM_cameraGotoPan.resize(instances,90);
  SIM_cameraGotoTilt.resize(instances,90);
  SIM_cameraMu.resize((instances-1),0);
  SIM_cameraSigma.resize((instances-1),0);
  SIM_meanMove.resize(instances,0);

  SIM_stdMove.resize(instances,0);

  std::vector<float> temp;
  temp.resize(ERRINTERVAL,0);
  SIM_moveRecord.resize(instances,temp);
  SIM_moveRecordGrad.resize(instances,temp);
  SIM_LOTcount.resize(instances,0);
  SIM_height.resize(instances,0);
  SIM_width.resize(instances,0);
  SIM_gotoX.resize(instances,0);
  SIM_gotoY.resize(instances,0);
  SIM_circleRed.resize(instances,0);
  SIM_circleGreen.resize(instances,0);
  SIM_circleBlue.resize(instances,0);
  SIM_boxRed.resize(instances,0);
  SIM_boxGreen.resize(instances,0);
  SIM_boxBlue.resize(instances,0);
  SIM_didCircleColor.resize(instances,0);
  SIM_didBoxColor.resize(instances,0);
  SIM_didTrackColor.resize(instances,0);
  SIM_recordCounter.resize(instances,0);
  SIM_adpt.resize(instances,true);
  SIM_HASTD.resize(instances,false);
  SIM_SASTD.resize(instances,false);
  SIM_VASTD.resize(instances,false);
  SIM_moveCamera.resize(instances,false);
  temp.resize(instances);
  Timer Ttemp;
  SIM_tim.resize(instances,Ttemp);
  //segmentImage stmp;
  //segment.resize(instances,stmp);
  SIM_segment = new segmentImage2[instances];
  segmentImageTrack2 sttmp;
  SIM_track.resize(instances,sttmp);

  blobConf.openFile("blob.conf",true);
  SIM_blobProp.BP_LOTbound = (int)blobConf.getItemValueF("BP_LOTbound");
  SIM_blobProp.BP_bound = (int)blobConf.getItemValueF("BP_bound");
  SIM_blobProp.BP_softBound = (int)blobConf.getItemValueF("BP_softBound");
  SIM_blobProp.BP_lowBound = (int)blobConf.getItemValueF("BP_lowBound");
  SIM_blobProp.BP_traj = (int)blobConf.getItemValueF("BP_traj");
  SIM_blobProp.BP_sampleStart = (int)blobConf.getItemValueF("BP_sampleStart");
  SIM_blobProp.BP_maxTraj = (int)blobConf.getItemValueF("BP_maxTraj");
  SIM_blobProp.BP_maxSize = (int)blobConf.getItemValueF("BP_maxSize");
  SIM_blobProp.BP_minSize = (int)blobConf.getItemValueF("BP_minSize");
  SIM_blobProp.BP_maxFrameSize = blobConf.getItemValueF("BP_maxFrameSize");
  SIM_blobProp.BP_minMass = (int)blobConf.getItemValueF("BP_minMass");
  SIM_blobProp.BP_maxMass = (int)blobConf.getItemValueF("BP_maxMass");
  SIM_blobProp.BP_ratioMin = blobConf.getItemValueF("BP_ratioMin");
  SIM_blobProp.BP_softRatioMin = blobConf.getItemValueF("BP_softRatioMin");
  SIM_blobProp.BP_ratioMax = blobConf.getItemValueF("BP_ratioMax");
  SIM_blobProp.BP_softRatioMax = blobConf.getItemValueF("BP_softRatioMax");
  SIM_blobProp.BP_minClusterSize =
    (int)blobConf.getItemValueF("BP_minClusterSize");
  SIM_blobProp.BP_clusterWeightSpat =
    blobConf.getItemValueF("BP_clusterWeightSpat");
  //! set the weights in cluster for Hue
  SIM_blobProp.BP_clusterWeightH =
    blobConf.getItemValueF("BP_clusterWeightH");
  //! set the weights in cluster for Sat
  SIM_blobProp.BP_clusterWeightS =
    blobConf.getItemValueF("BP_clusterWeightS");
  //! set the weights in cluster for Val
  SIM_blobProp.BP_clusterWeightV =
    blobConf.getItemValueF("BP_clusterWeightV");
  //! set initial standard deviation for color tracking after Hue
  SIM_blobProp.BP_clusterColorStdH =
    blobConf.getItemValueF("BP_clusterColorStdH");
  //! set initial standard deviation for color tracking after Sat
  SIM_blobProp.BP_clusterColorStdS =
    blobConf.getItemValueF("BP_clusterColorStdS");
  //! set initial standard deviation for color tracking after Val
  SIM_blobProp.BP_clusterColorStdV =
    blobConf.getItemValueF("BP_clusterColorStdV");


  for(int i = 0; i < instances; i++)
  {
    SIM_track[i].SITsetBlobProp(&SIM_blobProp);
    SIM_track[i].SITsetUpVars(1000);
    SIM_track[i].SITsetImage(&SIM_segment[i]);
  }

  SIM_cameraMu[0] = CAMERAMU1;
  SIM_cameraMu[1] = CAMERAMU2;
  SIM_cameraMu[2] = CAMERAMU3;
  SIM_cameraSigma[0] = CAMERASIGMA1;
  SIM_cameraSigma[1] = CAMERASIGMA2;
  SIM_cameraSigma[2] = CAMERASIGMA3;


}

segmentImageMerge2::~segmentImageMerge2()
{}

void segmentImageMerge2::SIMsetCircleColor(int r, int g, int b, int instance)
{
  SIM_circleRed[instance] = r;
  SIM_circleBlue[instance] = g;
  SIM_circleGreen[instance] = b;
  SIM_didCircleColor[instance] = 1;
}

void segmentImageMerge2::SIMsetBoxColor(int r, int g, int b, int instance)
{
  SIM_boxRed[instance] = r;
  SIM_boxBlue[instance] = g;
  SIM_boxGreen[instance] = b;
  SIM_didBoxColor[instance] = 1;
}

void segmentImageMerge2::SIMsetTrackColor(float h, float hstd,
                                      float s, float sstd,
                                      float v, float vstd,
                                      int instance, bool adapt, int avg)
{
  SIM_H[instance] = h;
  SIM_S[instance] = s;
  SIM_V[instance] = v;
  SIM_Hstd[instance] = hstd;
  SIM_Sstd[instance] = sstd;
  SIM_Vstd[instance] = vstd;
  SIM_adpt[instance] = adapt;
  SIM_segment[instance].SIsetHue(SIM_H[instance],SIM_Hstd[instance],0);
  SIM_segment[instance].SIsetSat(SIM_S[instance],SIM_Sstd[instance],0);
  SIM_segment[instance].SIsetVal(SIM_V[instance],SIM_Vstd[instance],0);
  SIM_didTrackColor[instance] = 1;
  SIM_segment[instance].SIsetHSVavg(avg);
}

void segmentImageMerge2::SIMsetAdapt(float ha, bool haSTD, float sa,
                                     bool saSTD,
                                     float va, bool vaSTD, int instance,
                                     bool useCluster)
{
  SIM_useCluster[instance] = useCluster;
  SIM_HA[instance] = ha;
  SIM_SA[instance] = sa;
  SIM_VA[instance] = va;
  SIM_HASTD[instance] = haSTD;
  SIM_SASTD[instance] = saSTD;
  SIM_VASTD[instance] = vaSTD;
}

void segmentImageMerge2::SIMsetAdaptBound(float Hupper, float Hlower,
                   float Supper, float Slower,
                   float Vupper, float Vlower,
                   int instance)
{
  SIM_HU[instance] = Hupper;
  SIM_SU[instance] = Supper;
  SIM_VU[instance] = Vupper;
  SIM_HL[instance] = Hlower;
  SIM_SL[instance] = Slower;
  SIM_VL[instance] = Vlower;
}

void segmentImageMerge2::SIMsetCameraPosition(float pan, float tilt,
                                             int instance, bool stats)
{
  // record camera movement
  SIM_cameraMovePan[instance] = pan;
  SIM_cameraMoveTilt[instance] = tilt;
  if(stats == true)
  {
    int doThisItem;
    float SIM_move = sqrt(pow(pan,2)+pow(tilt,2));
    if(SIM_recordCounter[instance] != 0)
    {
      doThisItem = instance - 1;
    }
    else
    {
      doThisItem = ERRINTERVAL - 1;
    }

    // Calculate finate state gradiant from last iteration to this one and record
    SIM_moveRecordGrad[instance][SIM_recordCounter[instance]] =
      SIM_move - SIM_moveRecord[instance][SIM_recordCounter[doThisItem]] ;
    SIM_moveRecord[instance][SIM_recordCounter[instance]] = SIM_move;

    float sumP = 0;
    float SSP = 0;

    // calcuate mean movements of camera servos
    for(int i = 0; i < ERRINTERVAL; i++)
    {
      sumP += SIM_moveRecordGrad[instance][i];
    }
    SIM_meanMove[instance] = sumP/ERRINTERVAL;

    // calculate standard deviation of camera servos
    for(int i = 0; i < ERRINTERVAL; i++)
    {
      SSP += pow((SIM_meanMove[instance] - SIM_moveRecordGrad[instance][i]),2);
    }
    SIM_stdMove[instance] = sqrt(SSP/ERRINTERVAL);

    //LINFO("CAM %d Move STD %f",instance,stdMove[instance]);

    // increment counter
    if(SIM_recordCounter[instance] < ERRINTERVAL)
      SIM_recordCounter[instance]++;
    else
      SIM_recordCounter[instance] = 0;
  }
}

void segmentImageMerge2::SIMsetFrame(int x1, int y1, int x2, int y2,
                                 int realX, int realY, int instance)
{
  SIM_segment[instance].SIsetFrame(x1,y1,x2,y2,realX,realY);
}

/* This is a basic tracker access method that tracks on one image at a time */
void segmentImageMerge2::SIMtrackImage(Image<PixRGB<byte> > input,
                                   Image<PixRGB<byte> > *image, int instance,
                                   Image<PixRGB<byte> > *auxImage, bool _fast)
{
  // Assert that parameters have been set up before starting
  SIM_fast = _fast;
  ASSERT(SIM_didCircleColor[instance] == 1);
  ASSERT(SIM_didBoxColor[instance] == 1);
  ASSERT(SIM_didTrackColor[instance] == 1);
  SIM_imageHold = image;
  SIM_auxHold = auxImage;

  Image< PixRGB<float> > fima;

  // decimate input image twice to speed things up
  fima = decXY(input);
  fima = decXY(fima);

  SIM_fimaHold = &fima;

  // segment the decimated image
  SIM_segment[instance].SIsegment(fima);

  // get center of mass for blobs
  SIM_segment[instance].SIcalcMassCenter();

  // edit blobs, weed out all the non-hackers who are not fit to carry a rifle
  //LINFO("INSTANCE %d",instance);
  SIM_track[instance].SITtrack();
  // apply adaptive color thesholding
  SIMcolorProcessBlobs(instance);

}

// END

// the statistical multi image tracker we will build
void segmentImageMerge2::SIMtrackImageMulti(
                  std::vector<Image<PixRGB<byte> > > *image, int instances)
{
  SIM_fast = true;
  Image< PixRGB<float> > fima;

  for(int i = 0; i < instances; i++)
  {
    ASSERT(SIM_didCircleColor[i] == 1);
    ASSERT(SIM_didBoxColor[i] == 1);
    ASSERT(SIM_didTrackColor[i] == 1);

    SIM_imageHold = &(image->at(i));
    fima = decXY(image->at(i));
    fima = decXY(fima);

    // Color segment this instance
    SIM_segment[i].SIsegment(fima);

    // get center of mass for blobs
    SIM_segment[i].SIcalcMassCenter();

    // edit blobs, weed out all the non-hackers who are not fit
    // to carry a rifle
    SIM_track[i].SITtrack(0);
  }

  SIM_moveMeanNormal = 0;
  SIM_moveStdNormal = 0;

  // Normalize over movement statisitcs to apply them in the next iterations
  for(int i = 0; i < instances; i++)
  {
    SIM_moveMeanNormal += SIM_meanMove[i];
    SIM_moveStdNormal += SIM_stdMove[i];
  }
  //avoid divide by zero error
  SIM_moveMeanNormal += .000001;
  SIM_moveStdNormal += .000001;
  SIMupdateVergance(48,36);

  for(int i = 0; i < instances; i++)
  {
    SIM_imageHold = &(image->at(i));
    //compute vergance springs for each camera
    SIMverganceSpring(instances,i,true);

    // apply adaptive color thesholding
    SIMcolorProcessBlobs(i);
  }
}

void segmentImageMerge2::SIMmergeImages(Image<PixRGB<byte> > *image)
{
  SIM_mergeGotoX = 0; SIM_mergeGotoY = 0;
  int mergeCount = 0;
  for(int i = 0; i < SIM_instanceNumber; i++)
  {
    SIM_gotoX[i] = SIM_track[i].SITgetObjectX();
    SIM_gotoY[i] = SIM_track[i].SITgetObjectY();
    if(SIM_track[i].SITreturnLOT() == false)
    {
      SIM_mergeGotoX += SIM_gotoX[i];
      SIM_mergeGotoY += SIM_gotoY[i];
      mergeCount++;
    }
  }
  if(mergeCount != 0)
  {
    SIM_mergeGotoX = SIM_mergeGotoX/mergeCount;
    SIM_mergeGotoY = SIM_mergeGotoY/mergeCount;
  }
  drawCircle(*image, Point2D<int>((int)SIM_mergeGotoX*4
                             ,(int)SIM_mergeGotoY*4)
             ,10,PixRGB<byte>(255,0,0),2);
}

void segmentImageMerge2::SIMupdateVergance(float distance, float gaussBase)
{
  for(int i = 0; i < (SIM_instanceNumber-1); i++)
  {
    //this is the angle to the target from two apposing cameras.
    //Mu is then the differenc between these angles and 90 * 2
    SIM_cameraMu[i] = 2*(90-(((2*atan(distance/(DISTANCE*(i+1))))/3.14159)*90));
    // the base angle for something at three feet from target
    // i.e. make the gaussian three feet in diameters
    float baseAngle = 2*(90-(((2*atan((distance-gaussBase)
                                      /(DISTANCE*(i+1))))/3.14159)*90));

    SIM_cameraSigma[i] = fabs(baseAngle-SIM_cameraMu[i]);
    //LINFO("UPDATE VERGANCE camera %d, Mu %f STD %f"
    //,i,cameraMu[i],cameraSigma[i]);
  }
}

void segmentImageMerge2::SIMverganceSpring(int instances, int current,
                                       bool doTracked)
{

  float theta, phi;
  int seperation;


  int maxBlob = -1;
  float maxBlobVal = 0;

  // if we didn't lose track and we want to vergance on cameras
  // that are tracking...
  if((SIM_track[current].SITreturnLOT() == false) && (doTracked == true))
  {
    SIM_moveCamera[current] = false;

    // for each blob this camera is tracking do
    for(int x = 0; x < SIM_segment[current].SInumberBlobs(); x++)
    {
      // check to make sure we havn't already disqualified this blob
      if(SIM_track[current].SITisCandidate(x) == true)
      {
        SIM_track[current].SIT_pVergance[x] = 0;
        // calculate the angle to the target blob being analized at the moment
        float gotoCY = fabs((480 - SIM_segment[current].SIgetCenterY(x)*8)
                           - SIM_camera.Ypixel);

        float panConv = ((float)SIM_camera.Xfield/(float)SIM_camera.Xpixel);
        float tiltConv = ((float)SIM_camera.Yfield/(float)SIM_camera.Ypixel);

        float panOff = ((float)SIM_camera.Xpixel*.5)
          - SIM_segment[current].SIgetCenterX(x)*8;
        float tiltOff = ((float)SIM_camera.Ypixel*.5)-gotoCY;

        float travelPan = SIM_cameraMovePan[current] +
          ((panOff*panConv)*SIM_camera.fieldAdjustmentX);
        float travelTilt = SIM_cameraMoveTilt[current] +
          ((tiltOff*tiltConv)*SIM_camera.fieldAdjustmentY);

        // cycle over other camera positions
        //and calculate the p of vergance for this camera
        for(int j = 0; j < instances; j++)
        {
          if(j != current)
          {
            if(j < current)
              theta = travelPan - SIM_cameraMovePan[j];
            else
              theta = SIM_cameraMovePan[j] - travelPan;

            phi = fabs(travelTilt - SIM_cameraMoveTilt[j]);
            seperation = abs(current - j);

            // p += vergance(tilt,cam(x))*vergance(pan,cam(x))
            SIM_track[current].SIT_pVergance[x] +=
              (SIM_Stats.gauss(theta,SIM_cameraMu[seperation-1]
                           ,SIM_cameraSigma[seperation-1])
               *SIM_Stats.gauss(phi,0.0F,21.0F))*
              (1-(SIM_stdMove[j]/SIM_moveStdNormal));

          }
        }
        // if I have the highest P of all the blobs in this
        // instance (camera) so far, I win. Take argmax
        if(SIM_track[current].SIT_pVergance[x] >= maxBlobVal)
        {
          if(maxBlob != -1)
          {
            // turn off this blob, it's no good
            SIM_track[current].SITsetCandidate(maxBlob,false);
          }
          maxBlob = x;
          // set this blob as the best one
          maxBlobVal = SIM_track[current].SIT_pVergance[x];
        }
        else
        {
          // turn off this blob, it not better than anyone
          SIM_track[current].SITsetCandidate(x,false);
        }
      }
    }
  }
  else
  {
    // this camera is in a LOT, send it to a vergance coordinate;
    if(SIM_LOTcount[current] > LOTMAX)
    {
      SIM_moveCamera[current] = true;
      float doPan = 0;
      float doTilt = 0;
      int normal = 0;

      // for all cameras not in LOT,
      // go to the average vergance over those cameras
      // e.g. you vergance should reflect the ones tracking
      for(int k = 0; k < instances; k++)
      {

        if((k != current) && (SIM_track[k].SITreturnLOT() == false))
        {
          seperation = abs(current - k);
          // you should converge to another camera based upon
          // the P derived from the gradiant of its
          // movement. This is cameras that are more fluid have more influince.
          if(k < current)
          {
            doPan += SIM_cameraMovePan[k]
              *(1- SIM_stdMove[k]/SIM_moveStdNormal)
              + SIM_cameraMu[seperation-1];

          }
          else
          {
            doPan += SIM_cameraMovePan[k]
              *(1 - SIM_stdMove[k]/SIM_moveStdNormal)
              - SIM_cameraMu[seperation-1];
          }
          doTilt += SIM_cameraMoveTilt[k]
            *(1 - SIM_stdMove[k]/SIM_moveStdNormal);
          normal++;
        }
      }
      if(normal != 0)
      {
        // if we can be biased by at least one camera do this
        SIM_cameraGotoPan[current] = doPan/normal;
        SIM_cameraGotoTilt[current] = doTilt/normal;
      }
    }
  }
}

void segmentImageMerge2::SIMgetImageTrackXY(int *x, int *y, int instance)
{
  *x = SIM_gotoX[instance];
  *y = SIM_gotoY[instance];
}

void segmentImageMerge2::SIMgetImageTrackXY2(int *x, int *y, int instance)
{
   *x = SIM_track[instance].SITgetObjectX();
   *y = SIM_track[instance].SITgetObjectY();
}

void segmentImageMerge2::SIMgetImageTrackXYMerge(int *x, int *y)
{
  *x = SIM_mergeGotoX;
  *y = SIM_mergeGotoY;
}

bool segmentImageMerge2::SIMreturnLOT(int instance)
{
  return SIM_track[instance].SITreturnLOT();
}

float segmentImageMerge2::SIMreturnCameraProb(int instance)
{
  return (1-SIM_stdMove[instance]/SIM_moveStdNormal);
}

bool segmentImageMerge2::SIMdoMoveCamera(int instance, float *doPan,
                                        float *doTilt)
{
  *doPan = SIM_cameraGotoPan[instance];
  *doTilt = SIM_cameraGotoTilt[instance];

  // calculate gradiant variance
  return SIM_moveCamera[instance];
}

Image<byte> segmentImageMerge2::SIMreturnCandidateImage(int instance)
{
  return SIM_segment[instance].SIreturnNormalizedCandidates();
}

void segmentImageMerge2::SIMSetCluster(int sizeX, int sizeY, int instances,
                                       float hweight, float sweight,
                                       float vweight)
{
  SIM_clusterSet = true;
  long totalSize = (sizeX/4)*(sizeY/4);
  SIM_Hweight = hweight;
  SIM_Sweight = sweight;
  SIM_Vweight = vweight;
  std::vector<double> tempVec(5,0.0F);
  std::vector<float> tempVec2(totalSize,0.0F);
  SIM_vectorizedImage.resize(totalSize,tempVec2);
  SIM_meanH.resize(instances,tempVec);
  SIM_meanS.resize(instances,tempVec);
  SIM_meanV.resize(instances,tempVec);
  SIM_stdH.resize(instances,tempVec);
  SIM_stdS.resize(instances,tempVec);
  SIM_stdV.resize(instances,tempVec);
  SIM_score.resize(instances,tempVec2);
  SIM_item.resize(totalSize,0);
  configIn.openFile("NPclassify.conf");
  polySet.openFile("polySet.conf");
  SIM_NP.NPsetup(configIn,polySet,false);
  SIM_NP.NPresizeSpace(totalSize,5);
}

void segmentImageMerge2::SIMclusterColor(Image<PixRGB<float> > image
                                         ,int instance)
{
  ASSERT(SIM_clusterSet && "You need to set up cluster params first");
  PixRGB<float> pix;
  Image<PixRGB<float> > newImage;
  newImage = decXY(image);
  newImage = decXY(newImage);
  Image<PixRGB<float> >::iterator inputIter = newImage.beginw();
  std::vector<std::vector<float> >::iterator imageVecIter
    = SIM_vectorizedImage.begin();

  std::vector<float>::iterator imageVecIterIter;
  std::vector<int>::iterator item;

  std::vector<double>::iterator meanHiter;
  meanHiter = SIM_meanH[instance].begin();
  std::vector<double>::iterator meanSiter;
  meanSiter = SIM_meanS[instance].begin();
  std::vector<double>::iterator meanViter;
  meanViter = SIM_meanV[instance].begin();
  std::vector<double>::iterator stdHiter;
  stdHiter = SIM_stdH[instance].begin();
  std::vector<double>::iterator stdSiter;
  stdSiter = SIM_stdS[instance].begin();
  std::vector<double>::iterator stdViter;
  stdViter = SIM_stdV[instance].begin();
  std::vector<float>::iterator scoreIter;
  scoreIter = SIM_score[instance].begin();

  // load the image into a vector of size samples*channels (e.g. HSV)
  int place = 0;
  int width = newImage.getWidth();
  float pixH, pixS, pixV;
  while(inputIter != newImage.endw())
  {
    imageVecIterIter = imageVecIter->begin();
    pix = *inputIter;
    PixHSV<float>(pix).getHSV(pixH,pixS,pixV);
    *imageVecIterIter = (pixH/360) * 100 * SIM_blobProp.BP_clusterWeightH;
    ++imageVecIterIter;
    *imageVecIterIter = pixS * 100 * SIM_blobProp.BP_clusterWeightS;
    ++imageVecIterIter;
    *imageVecIterIter = (pixV/255) * 100 * SIM_blobProp.BP_clusterWeightV;
    ++imageVecIterIter;
    *imageVecIterIter = (place/width)* 0.5 * SIM_blobProp.BP_clusterWeightSpat;
    ++imageVecIterIter;
    *imageVecIterIter = (place%width)* 0.5 * SIM_blobProp.BP_clusterWeightSpat;
    ++imageVecIter;
    ++inputIter;
    place++;
  }

  LINFO("RUNNING NEW VECTOR size %" ZU " x %" ZU ,SIM_vectorizedImage.size(),
        SIM_vectorizedImage[0].size());
  /*
  Image<double> temp;
  temp = SIM_vectorizedImage;
  Image<float> temp2;
  temp2 = temp;
  Raster::VisuFloat(image,0,"temp.pgm");
  Raster::VisuFloat(temp2,FLOAT_NORM_0_255,"temp.pgm");
  */
  // cluster input image
  //SIM_NP.NPresetSpace();
  SIM_NP.NPaddSpace(SIM_vectorizedImage);
  SIM_NP.NPclassifySpaceNew(false);
  LINFO("Get Return Info");
  //roots = SIM_NP.NPgetStems();
  //parents = SIM_NP.NPgetParents();
  //density = SIM_NP.NPgetDensity();
  double Hsum, HSS;
  double Ssum, SSS;
  double Vsum, VSS;
  float *H = &SIM_H[instance];
  float *S = &SIM_S[instance];
  float *V = &SIM_V[instance];
  SIM_winningScore = 0;
  SIM_winningClass = -1;
  // get the mean and standard deviation over all clusters
  for(int i = 0; i < SIM_NP.NPgetStemNumber(); i++)
  {
    int classSize = SIM_NP.NPgetClassSize(i);
    *scoreIter = 0;
    //if(SIM_NP.NPgetMinClassSize() <= classSize)
    if(SIM_blobProp.BP_minClusterSize <= classSize)
    {
      // for each class computer sum and sum squared values over HSV
      Hsum = 0.0F; HSS = 0.0F;
      item = SIM_item.begin();
      for(int cs = 0; cs < classSize; cs++, ++item)
      {
        *item = SIM_NP.NPgetClass(i,cs);
        Hsum += SIM_NP.NPgetFeature(*item,0);
        HSS += pow(SIM_NP.NPgetFeature(*item,0),2)/classSize;
      }
      *meanHiter = Hsum/classSize;
      *stdHiter = HSS - pow(*meanHiter,2);
      // for each class computer sum and sum squared values over HSV
      Ssum = 0.0F; SSS = 0.0F;
      item = SIM_item.begin();
      for(int cs = 0; cs < classSize; cs++, ++item)
      {
        Ssum += SIM_NP.NPgetFeature(*item,1);
        SSS += pow(SIM_NP.NPgetFeature(*item,1),2)/classSize;
      }
      *meanSiter = Ssum/classSize;
      *stdSiter = SSS - pow(*meanSiter,2);
      // for each class computer sum and sum squared values over HSV
      Vsum = 0.0F; VSS = 0.0F;
      item = SIM_item.begin();
      for(int cs = 0; cs < classSize; cs++, ++item)
      {
        Vsum += SIM_NP.NPgetFeature(*item,2);
        VSS += pow(SIM_NP.NPgetFeature(*item,2),2)/classSize;
      }
      *meanViter = Vsum/classSize;
      *stdViter = VSS - pow(*meanViter,2);
      // compute the P of this cluster matching the
      if((*stdHiter != 0) && (*stdSiter != 0) && (*stdViter != 0))
      {
        *scoreIter =
          ((SIMPgauss(*H,*meanHiter,*stdHiter)*SIM_Hweight)
           + (SIMPgauss(*S,*meanSiter,*stdSiter)*SIM_Sweight)
           + (SIMPgauss(*V,*meanViter,*stdViter)*SIM_Vweight))
          / (SIM_Hweight+SIM_Sweight+SIM_Vweight);
      }
      else
        *scoreIter = 0;
      // find out if the P is better for this class, if so, set it
      if(*scoreIter > SIM_winningScore)
      {
        SIM_winningScore = *scoreIter;
        SIM_winningClass = i;
      }
      LINFO("Competetors size %d H %f std %f S %f std %f V %f std %f P %f",
            classSize,
            *meanHiter,*stdHiter,*meanSiter,*stdSiter,
            *meanViter,*stdViter,*scoreIter);
    }
    ++meanHiter; ++meanSiter; ++meanViter;
    ++stdHiter; ++stdSiter; ++stdViter;
    ++scoreIter;
  }
  // if we have a winning class, set tracking colors to it
  if(SIM_winningClass != -1)
  {
    LINFO("WINNER %d H %f std %f S %f std %f V %f std %f",
          SIM_winningClass,
          SIM_meanH[instance][SIM_winningClass],
          SIM_stdH[instance][SIM_winningClass],
          SIM_meanS[instance][SIM_winningClass],
          SIM_stdS[instance][SIM_winningClass],
          SIM_meanV[instance][SIM_winningClass],
          SIM_stdV[instance][SIM_winningClass]);

    SIM_segment[instance].SIsetHue(SIM_meanH[instance][SIM_winningClass],
                                   SIM_stdH[instance][SIM_winningClass]
                                   * SIM_blobProp.BP_clusterColorStdH,0);

    SIM_segment[instance].SIsetSat(SIM_meanS[instance][SIM_winningClass],
                                   SIM_stdS[instance][SIM_winningClass]
                                   * SIM_blobProp.BP_clusterColorStdS,0);

    SIM_segment[instance].SIsetVal(SIM_meanV[instance][SIM_winningClass],
                                   SIM_stdV[instance][SIM_winningClass]
                                   * SIM_blobProp.BP_clusterColorStdV,0);
  }
  SIM_NP.NPresetSpace();
}

bool segmentImageMerge2::SIMstereoMatch(PixelPoint points[2],
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
      P[i].x = params[i].x + params[i].r
        * sin(params[i].theta*deg2rad) *cos(params[i].phi*deg2rad)
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
      f[i].z = params[i].z + params[i].r*cos(params[i].theta*deg2rad)
        + params[i].f * cos(params[i].theta*deg2rad);
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

double segmentImageMerge2::SIMPgauss(double X, double Xbar, double std)
{
  double first = 1/(sqrt(2*3.14159*pow(std,2)));
  double second = -1*(pow((X - Xbar),2)/(2*pow(std,2)));
  return first*exp(second);
}


