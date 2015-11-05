/*!@file VFAT/segmentImageMerge2.H Basic image segmenter blob finder using color */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/segmentImageMerge2.H $
// $Id: segmentImageMerge2.H 4663 2005-06-23 17:47:28Z rjpeters $
//

#ifndef SEGMENTIMAGEMERGE2_H
#define SEGMENTIMAGEMERGE2_H


#include "Devices/cameraConf.H"
#include "Image/Image.H"
#include "Util/Timer.H"
#include "Util/Types.H"
#include "Util/log.H"
#include "Util/readConfig.H"
#include "Util/stats.H"
#include "VFAT/NPclassify2.H"
#include "VFAT/PointClasses.H"
#include "VFAT/segmentImageTrack2.H"
#include <math.h>

class segmentImageMerge2
{
  // create a billion vectors for holding everything we need
  // this is lazy, but RAM is cheap ;)
private:
  segmentImage2 *SIM_segment;
  NPclassify2<float> SIM_NP;
  stats<float> SIM_Stats;
  cameraConf SIM_camera;
  blobProp SIM_blobProp;
  int SIM_instanceNumber;
  int SIM_mergeGotoX;
  int SIM_mergeGotoY;
  int SIM_winningClass;
  float SIM_Hweight, SIM_Sweight, SIM_Vweight;
  bool SIM_fast;
  bool SIM_clusterSet;
  float SIM_moveMeanNormal;
  float SIM_moveStdNormal;
  float SIM_winningScore;

  std::vector<float> SIM_H;
  std::vector<float> SIM_S;
  std::vector<float> SIM_V;
  std::vector<float> SIM_Hstd;
  std::vector<float> SIM_Sstd;
  std::vector<float> SIM_Vstd;
  std::vector<float> SIM_HA;
  std::vector<float> SIM_SA;
  std::vector<float> SIM_VA;
  std::vector<float> SIM_HU;
  std::vector<float> SIM_SU;
  std::vector<float> SIM_VU;
  std::vector<float> SIM_HL;
  std::vector<float> SIM_SL;
  std::vector<float> SIM_VL;
  std::vector<float> SIM_delay;
  std::vector<float> SIM_cameraMovePan;
  std::vector<float> SIM_cameraMoveTilt;
  std::vector<float> SIM_cameraGotoPan;
  std::vector<float> SIM_cameraGotoTilt;
  std::vector<float> SIM_cameraMu;
  std::vector<float> SIM_cameraSigma;
  std::vector<float> SIM_meanMove;
  std::vector<float> SIM_stdMove;

  std::vector<std::vector< float> > SIM_moveRecord;
  std::vector<std::vector< float> > SIM_moveRecordGrad;
  std::vector<std::vector< float> > SIM_imageVecIter;
  std::vector<std::vector< double> > SIM_meanH;
  std::vector<std::vector< double> > SIM_meanS;
  std::vector<std::vector< double> > SIM_meanV;
  std::vector<std::vector< double> > SIM_stdH;
  std::vector<std::vector< double> > SIM_stdS;
  std::vector<std::vector< double> > SIM_stdV;
  std::vector<std::vector< float> > SIM_score;

  std::vector<std::vector< float> > SIM_vectorizedImage;

  std::vector<int> SIM_LOTcount;
  std::vector<int> SIM_height;
  std::vector<int> SIM_width;
  std::vector<int> SIM_gotoX;
  std::vector<int> SIM_gotoY;
  std::vector<int> SIM_circleRed;
  std::vector<int> SIM_circleGreen;
  std::vector<int> SIM_circleBlue;
  std::vector<int> SIM_boxRed;
  std::vector<int> SIM_boxGreen;
  std::vector<int> SIM_boxBlue;
  std::vector<int> SIM_didCircleColor;
  std::vector<int> SIM_didBoxColor;
  std::vector<int> SIM_didTrackColor;
  std::vector<int> SIM_recordCounter;
  std::vector<int> SIM_item;

  std::vector<bool> SIM_adpt;
  std::vector<bool> SIM_HASTD;
  std::vector<bool> SIM_SASTD;
  std::vector<bool> SIM_VASTD;
  std::vector<bool> SIM_moveCamera;
  std::vector<bool> SIM_useCluster;

  std::vector<Timer> SIM_tim;
  std::vector<segmentImageTrack2> SIM_track;
  std::vector<Image<byte> > SIM_temp;
  void SIMcolorProcessBlobs(int instance);
  void SIMresetColor(int instance);
  Image<PixRGB<byte> > *SIM_imageHold;
  Image<PixRGB<byte> > *SIM_auxHold;
  Image<PixRGB<float> > *SIM_fimaHold;
  readConfig configIn;
  readConfig polySet;
  readConfig blobConf;
public:


  //! default constructor, also sets up number of tracker instances
  segmentImageMerge2(int instances);
  ~segmentImageMerge2();

        void resetAll(int instances);

  //! set the tracking color for mass circle
  /*!
    @param r this is the red color for the output circle
    @param g this is the green color for the output circle
    @param b this is the blue color for the output circle
    @param instance this is the tracker to apply these settings to
  */
  void SIMsetCircleColor(int r, int g, int b, int instance);

  //! set the tracking colot for the bounding box
  //! set the tracking color for mass circle
  /*!
    @param r this is the red color for the output bounding box
    @param g this is the green color for the output bounding box
    @param b this is the blue color for the output bounding box
    @param instance this is the tracker to apply these settings to
  */
  void SIMsetBoxColor(int r, int g, int b, int instance);

  //! set default tracking color values
  /*!
    @param h initial hue value
    @param hstd initial boundary in standard deviation for h
    @param s initial satuaration value
    @param sstd initial boundary in standard deviation for s
    @param v initial intensity value
    @param vstd initial boundary in standard deviation for v
    @param instance which tracker instance to apply these settings too
    @param adapt tells if you want these values to adapt
  */
  void SIMsetTrackColor(float h, float hstd,
                     float s, float sstd,
                     float v, float vstd,
                     int instance, bool adapt, int avg);

  //! set initial adaptive color thresholds for tracking
  void SIMsetAdapt(float ha, bool haSTD, float sa, bool saSTD,
                float va, bool vaSTD, int instance, bool useCluster);

  //! set up hard bounds for color adaptation
  /*!
    @param Hupper upper bound for adaptive hue
    @param Hlower lower bound for adaptive hue
    @param Supper upper bound for adaptive saturation
    @param Slower lower bound for adaptive saturation
    @param Vupper upper bound for adaptive intensity
    @param Vlower lower bound for adaptive intensity
    @param instance Which tracker to apply these settings to
  */
  void SIMsetAdaptBound(float Hupper, float Hlower,
                     float Supper, float Slower,
                     float Vupper, float Vlower,
                     int instance);

  //! tell this method the pan and tilt postition the camera is moving to
  void SIMsetCameraPosition(float pan, float tilt, int instance
                         , bool stats = false);

  //! set frame size for color tracker
  void SIMsetFrame(int x1, int y1, int x2, int y2,int realX, int realY,
                int instance);

  //! Put image one at a time into tracker and get back tracker image output
  /*!
    @param input this is the raw input image
    @param image this is a pointer to the output image
    @param outputI this is a pointer the the candidate pixel image
    @param instance This is the image instance to which you are using
    @param auxImage this is a pointer to the HSV threshold bar image
  */
  void SIMtrackImage(Image<PixRGB<byte> > input,
                  Image<PixRGB<byte> > *image, int instance,
                  Image<PixRGB<byte> > *auxImage, bool fast = false);

  //! put image set into tracker and let it sort all this stuff out
  /*!
    this method will track over multiple stereo pair linked cameras and
    produce blobs to track that agree with assumptions of stereo tracking
    for instance vergance and spacial locality.
    @param input this is the raw input image
    @param image this is a pointer to the output image
    @param outputI this is a pointer the the candidate pixel image
    @param instance This is the image instance to which you are using
    @param auxImage this is a pointer to the HSV threshold bar image
  */
  void SIMtrackImageMulti(
                       std::vector<Image<PixRGB<byte> > > *image,
                       int instances);
  //! merge qualities of images if needed
  void SIMmergeImages(Image<PixRGB<byte> > *image);
  //! update the vergance distance if needed for cameras
  /*! feed in the new distance to target and extact P over expected
     vergance
     @param distance is a measure in inches (sorry)
  */
  void SIMupdateVergance(float distance, float gaussBase = 36);

  //! apply probabalistic model over multiple stereo cameras for vergance
  /*! This method will calculate the most likely blob from a set of blobs based
    upon the vergance of the cameras. That is, joined stereo pairs should.
    experiance vergance to a target. This method creates virtual springs to
    pull the cameras into vergance with each other based upon the probability
    that a target will bring a camera into vergance with the other cameras.
    To use this method you must have defined a probabalistic model over
    vergance in the cameras.
    @param instance this is the instance of the track, in this case the camera
    @param doTracked this is true if you wish to apply to cameras that \
    are not currently in loss of track (LOT)
  */
  void SIMverganceSpring(int instance, int current, bool doTracked);
  //! get back X and Y coords for tracking
  void SIMgetImageTrackXY(int *x, int *y, int instance);
  //! get back X and Y coords for tracking
  void SIMgetImageTrackXY2(int *x, int *y, int instance);
  //! get back merged X and Y coords for tracking
  void SIMgetImageTrackXYMerge(int *x, int *y);
  //! return true if loss of track
  bool SIMreturnLOT(int instance);
  //! return the P of a good track
  float SIMreturnCameraProb(int instance);
  //! return if camera should move based upon maximum LOT's
  bool SIMdoMoveCamera(int instance, float *doPan, float *doTilt);
  //! return the image of candidate pixels
  Image<byte> SIMreturnCandidateImage(int instance);
  //! Set up color clustering for tracker
  /*! Call this method before calling SIMclsuterColor to set it up. Do
    this only once.
    @param sizeX This is the width of the image to be processed
    @param sizeY this is the height of the image to be processed
    @param instances How many tracker instances are being used
    @param hweight this is the weight to be given to hue in analysis
    @param sweight this is the weight to be given to saturation in analysis
    @param vweight this is the weight to be given to luminance in analysis
  */
  void SIMSetCluster(int sizeX, int sizeY, int instances,
                     float hweight, float sweight,
                     float vweight);
  //! find object in scene that's a best match using clustering
  /*! run this to find out which color to use for tracking.This will
    take and image and cluster it looking for the best color match
    in the image to track
  */
  void SIMclusterColor(Image<PixRGB<float> > image, int instance);
  //!Project corresponding points in 3D
  bool SIMstereoMatch(PixelPoint points[2],
                   CameraParams params[2],Point3D* retPoint);
  //! return the probability of X given Xbar and std using gaussian
  double SIMPgauss(double X, double Xbar, double std);

};

#endif
