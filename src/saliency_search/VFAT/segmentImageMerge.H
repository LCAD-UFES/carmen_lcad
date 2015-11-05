/*!@file VFAT/segmentImageMerge.H Basic image segmenter blob finder using color */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/segmentImageMerge.H $
// $Id: segmentImageMerge.H 4663 2005-06-23 17:47:28Z rjpeters $
//

#include "Devices/cameraConf.H"
#include "Image/Image.H"
#include "Util/Timer.H"
#include "Util/Types.H"
#include "Util/log.H"
#include "Util/stats.H"
#include "VFAT/PointClasses.H"
#include "VFAT/segmentImageTrack.H"
#include <math.h>

class segmentImageMerge
{
  // create a billion vectors for holding everything we need
  // this is lazy, but RAM is cheap ;)
private:
  segmentImage *segment;
  stats<float> Stats;
  cameraConf camera;
  int instanceNumber;
  int mergeGotoX;
  int mergeGotoY;
  bool fast;
  float moveMeanNormal;
  float moveStdNormal;

  std::vector<float> H;
  std::vector<float> S;
  std::vector<float> V;
  std::vector<float> Hstd;
  std::vector<float> Sstd;
  std::vector<float> Vstd;
  std::vector<float> HA;
  std::vector<float> SA;
  std::vector<float> VA;
  std::vector<float> HU;
  std::vector<float> SU;
  std::vector<float> VU;
  std::vector<float> HL;
  std::vector<float> SL;
  std::vector<float> VL;
  std::vector<float> delay;
  std::vector<float> cameraMovePan;
  std::vector<float> cameraMoveTilt;
  std::vector<float> cameraGotoPan;
  std::vector<float> cameraGotoTilt;
  std::vector<float> cameraMu;
  std::vector<float> cameraSigma;
  std::vector<float> meanMove;
  std::vector<float> stdMove;
  std::vector<std::vector< float> > moveRecord;
  std::vector<std::vector< float> > moveRecordGrad;


  std::vector<int> LOTcount;
  std::vector<int> height;
  std::vector<int> width;
  std::vector<int> gotoX;
  std::vector<int> gotoY;
  std::vector<int> circleRed;
  std::vector<int> circleGreen;
  std::vector<int> circleBlue;
  std::vector<int> boxRed;
  std::vector<int> boxGreen;
  std::vector<int> boxBlue;
  std::vector<int> didCircleColor;
  std::vector<int> didBoxColor;
  std::vector<int> didTrackColor;
  std::vector<int> recordCounter;

  std::vector<bool> adpt;
  std::vector<bool> HASTD;
  std::vector<bool> SASTD;
  std::vector<bool> VASTD;
  std::vector<bool> moveCamera;
  std::vector<Timer> tim;
  std::vector<segmentImageTrack> track;
  std::vector<Image<byte> > temp;
  void colorProcessBlobs(int instance);
  Image<PixRGB<byte> > *imageHold;
  Image<PixRGB<byte> > *auxHold;
  Image<PixRGB<float> > *fimaHold;
public:
  //! default constructor, also sets up number of tracker instances
  segmentImageMerge(int instances);
  ~segmentImageMerge();

  //! set the tracking color for mass circle
  /*!
    @param r this is the red color for the output circle
    @param g this is the green color for the output circle
    @param b this is the blue color for the output circle
    @param instance this is the tracker to apply these settings to
  */
  void setCircleColor(int r, int g, int b, int instance);

  //! set the tracking colot for the bounding box
  //! set the tracking color for mass circle
  /*!
    @param r this is the red color for the output bounding box
    @param g this is the green color for the output bounding box
    @param b this is the blue color for the output bounding box
    @param instance this is the tracker to apply these settings to
  */
  void setBoxColor(int r, int g, int b, int instance);

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
  void setTrackColor(float h, float hstd,
                     float s, float sstd,
                     float v, float vstd,
                     int instance, bool adapt, int avg);

  //! set initial adaptive color thresholds for tracking
  void setAdapt(float ha, bool haSTD, float sa, bool saSTD,
                float va, bool vaSTD, int instance);

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
  void setAdaptBound(float Hupper, float Hlower,
                     float Supper, float Slower,
                     float Vupper, float Vlower,
                     int instance);

  //! tell this method the pan and tilt postition the camera is moving to
  void setCameraPosition(float pan, float tilt, int instance
                         , bool stats = false);

  //! set frame size for color tracker
  void setFrame(int x1, int y1, int x2, int y2,int realX, int realY,
                int instance);

  //! Put image one at a time into tracker and get back tracker image output
  /*!
    @param input this is the raw input image
    @param image this is a pointer to the output image
    @param outputI this is a pointer the the candidate pixel image
    @param instance This is the image instance to which you are using
    @param auxImage this is a pointer to the HSV threshold bar image
  */
  void trackImage(Image<PixRGB<byte> > input,
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
  void trackImageMulti(
                       std::vector<Image<PixRGB<byte> > > *image,
                       int instances);
  //! merge qualities of images if needed
  void mergeImages(Image<PixRGB<byte> > *image);
  //! update the vergance distance if needed for cameras
  /*! feed in the new distance to target and extact P over expected
     vergance
     @param distance is a measure in inches (sorry)
  */
  void updateVergance(float distance, float gaussBase = 36);

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
  void verganceSpring(int instance, int current, bool doTracked);
  //! get back X and Y coords for tracking
  void getImageTrackXY(int *x, int *y, int instance);
  //! get back X and Y coords for tracking
  void getImageTrackXY2(int *x, int *y, int instance);
  //! get back merged X and Y coords for tracking
  void getImageTrackXYMerge(int *x, int *y);
  //! return true if loss of track
  bool returnLOT(int instance);
  //! return the P of a good track
  float returnCameraProb(int instance);
  //! return if camera should move based upon maximum LOT's
  bool doMoveCamera(int instance, float *doPan, float *doTilt);
  //! return the image of candidate pixels
  Image<byte> returnCandidateImage(int instance);
  //!Project corresponding points in 3D
  bool StereoMatch(PixelPoint points[2], CameraParams params[2],
                                                                         Point3D* retPoint);

};
