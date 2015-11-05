/*!@file VFAT/segmentImage.H Basic image segmenter blob finder using color */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/segmentImage.H $
// $Id: segmentImage.H 4663 2005-06-23 17:47:28Z rjpeters $
//

#include "Image/All.H"
#include "Image/Pixels.H"
//#include <vector>
#include <time.h>
#include <sys/time.h>

#define RGB  1
#define HSV  2
#define GREY 3

class segmentImage
{
private:
  Image<PixRGB<float> > workImage;
  Image<float> workImageGREY;
  Image<bool> candidatePixels;
  Image<bool> preCandidatePixels;
  Image<long> blobID;
  //! RGB values and upper and lower threshold values
  int red,green,blue,redLT,greenLT,blueLT,redUT,greenUT,blueUT;
  //! HSV vlaues and upper and lower threshold values
  double H,S,V,HLT,SLT,VLT,HUT,SUT,VUT;
  //! frame size that will be inspected
  int frameX1,frameY1,frameX2,frameY2;
  //! bools to determine if all values have been set to run image
  bool set1,set2,set3,set4;
  int doType;
  long num; // number of blob segments
  long masters; // number of masters;
  long mastersCount;
  long totalBlobs;
  long Hsamp,Ssamp,Vsamp,HSVcount,HSViter;
  // list of a pixels master
  std::vector<long> masterVec;
  std::vector<long> reOrderVec;
  std::vector<long> reverseOrderVec;
  std::vector<bool> reset;
  // list of blob properties
  std::vector<float> centerX;
  std::vector<float> centerY;
  // color properties for averageing
  std::vector<float> Havg;
  std::vector<float> Savg;
  std::vector<float> Vavg;
  std::vector<float> Hstdd;
  std::vector<float> Sstdd;
  std::vector<float> Vstdd;
  std::vector<float> HSVN;
  std::vector<long> Xsum;
  std::vector<long> Ysum;
  std::vector<long> mass;
  std::vector<int> xmin;
  std::vector<int> xmax;
  std::vector<int> ymin;
  std::vector<int> ymax;
  //! find any candidate pixel based upon pixel thresholding RGB
  void findCandidatesRGB();
  //! find any candidate pixel based upon pixel thresholding HSV
  void findCandidatesHSV();
  //! find any candidate pixel based upon pixel thresholding grey scale
  void findCandidatesGREY();
  //! remove single pixels without neighbors
  void removeSingles();
  //! scan candidate image and link continious pixels with a unique ID tag
  void discreteLinking();
  //! backward link pixels, find master, relabel masters
  void backwardLink(long slave, long master);
  //! combine slaves together into single blobs
  void combine();
  //! get information on blobs for debugging
  void getBlobs();
  //! Call to segmentation which calls most of these methods
  void doSegment();
public:
  //! create an object. Set true for RGB false for HSV
  /*! skews here are used to skew the curve towards one end of the threshold
     that is, you pick the ideal color value as val, the you pick the
     cut off threshold as thresh. You can then bias towads one end or the
     other by setting skew to +/- value, that value bing added to the
     upper or lower bound for the cut off depending on whether it is
     +/- that is, if its a neg. value then the lower bound is
     extended
  */
  segmentImage(int imageType);
  segmentImage();
  ~segmentImage();
  //! set the red value you are looking for with thresh error, and skew
  void setRed(int val, int thresh, int skew);
  //! set the green value you are looking for with thresh error, and skew
  void setGreen(int val, int thresh, int skew);
  //! set the blue value you are looking for with thresh error, and skew
  void setBlue(int val, int thresh, int skew);
  //! set the Hue value you are looking for with thresh error, and skew
  void setHue(double val, double thresh, double skew);
  //! set the Saturation value you are looking for with thresh error, and skew
  void setSat(double val, double thresh, double skew);
  //! set the Value (brightness) value you are looking for with thresh error
  void setVal(double val, double thresh, double skew);
  //! set the region of the image to inspect
  void setFrame(int x1, int y1, int x2, int y2, int realX, int realY);
  //! set up averaging for HSV color averaging
  void setHSVavg(long doAvg);
  //! segment image based upon parameters input
  void segment(Image<PixRGB<float> > &image);
  //! segment image based upon parameters input
  void segment(Image<float> &image);
  //! merge all blobs into one big blob, useful if you erase blobs
  /*! else just use returnCandidates */
  Image<long> createMother(Image<long> &img);
  //! return an image with labeled blobs. Use getBlobMap to map blobs
  Image<long> returnBlobs();
  //! return a bool map off all candidate pixels
  Image<bool> returnCandidates();
  //! return a normalized displayable map off all candidate pixels
  Image<float> returnNormalizedCandidates();
  //! return the image that is being worked on, to check if its ok
  Image<PixRGB<float> > returnWorkImage();
  //! return the image that is being worked on, to check if its ok
  Image<float> returnWorkImageGREY();
  //! return the total number of blobs
  int numberBlobs();
  //! return a map of blobs that gives the numeric ID of a blob from the image
  std::vector<long> getBlobMap();
  //! calculate basic mass/center blob properties
  void calcMassCenter();
  //! get blob center in X
  float getCenterX(long blob);
  //! get blob center in X
  float getCenterY(long blob);
  //! get blob mass
  long getMass(long blob);
  //! get X min for a blob
  int getXmin(long blob);
  //! get X max for a blob
  int getXmax(long blob);
  //! get Y min for a blob
  int getYmin(long blob);
  //! get Y max for a blob
  int getYmax(long blob);
  //! get the working image size in X
  int getImageSizeX();
  //! get the working image size in Y
  int getImageSizeY();
  //! get HSV mean values and standard deviations for a blob
  void getHSVvalue(long blob, float *H, float *S, float *V,
                   float *Hstd, float *Sstd, float *Vstd);
  //! do HVS color value means for x last iterations
  void  getHSVvalueMean(long blob, float *H, float *S, float *V,
                        float *Hstd, float *Sstd, float *Vstd);

};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
