/*!@file VFAT/segmentImage2.H Basic image segmenter blob finder using color */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/segmentImage2.H $
// $Id: segmentImage2.H 4663 2005-06-23 17:47:28Z rjpeters $
//

#include "Image/All.H"
#include "Image/Pixels.H"
//#include <vector>
#include <time.h>
#include <sys/time.h>

#define RGB  1
#define HSV  2
#define GREY 3

class segmentImage2
{
private:
  Image<PixRGB<float> > SI_workImage;
  Image<float> SI_workImageGREY;
  Image<bool> SI_candidatePixels;
  Image<bool> SI_preCandidatePixels;
  Image<long> SI_blobID;
  //! RGB values and upper and lower threshold values
  int SI_red,SI_green,SI_blue,SI_redLT,SI_greenLT;
  int SI_blueLT,SI_redUT,SI_greenUT,SI_blueUT;
  //! HSV vlaues and upper and lower threshold values
  double SI_H,SI_S,SI_V,SI_HLT,SI_SLT,SI_VLT,SI_HUT,SI_SUT,SI_VUT;
  //! frame size that will be inspected
  int SI_frameX1,SI_frameY1,SI_frameX2,SI_frameY2;
  //! bools to determine if all values have been set to run image
  bool SI_set1,SI_set2,SI_set3,SI_set4;
  int SI_doType;
  long SI_num; // number of blob segments
  long SI_masters; // number of masters;
  long SI_mastersCount;
  long SI_totalBlobs;
  long SI_Hsamp,SI_Ssamp,SI_Vsamp,SI_HSVcount,SI_HSViter;
  // list of a pixels master
  std::vector<long> SI_masterVec;
  std::vector<long> SI_reOrderVec;
  std::vector<long> SI_reverseOrderVec;
  std::vector<bool> SI_reset;
  // list of blob properties
  std::vector<float> SI_centerX;
  std::vector<float> SI_centerY;
  // color properties for averageing
  std::vector<float> SI_Havg;
  std::vector<float> SI_Savg;
  std::vector<float> SI_Vavg;
  std::vector<float> SI_Hstdd;
  std::vector<float> SI_Sstdd;
  std::vector<float> SI_Vstdd;
  std::vector<float> SI_HSVN;
  std::vector<long> SI_Xsum;
  std::vector<long> SI_Ysum;
  std::vector<long> SI_mass;
  std::vector<int> SI_xmin;
  std::vector<int> SI_xmax;
  std::vector<int> SI_ymin;
  std::vector<int> SI_ymax;
  //! find any candidate pixel based upon pixel thresholding RGB
  void SIfindCandidatesRGB();
  //! find any candidate pixel based upon pixel thresholding HSV
  void SIfindCandidatesHSV();
  //! find any candidate pixel based upon pixel thresholding grey scale
  void SIfindCandidatesGREY();
  //! remove single pixels without neighbors
  void SIremoveSingles();
  //! scan candidate image and link continious pixels with a unique ID tag
  void SIdiscreteLinking();
  //! backward link pixels, find master, relabel masters
  void SIbackwardLink(long slave, long master);
  //! combine slaves together into single blobs
  void SIcombine();
  //! get information on blobs for debugging
  void SIgetBlobs();
  //! Call to segmentation which calls most of these methods
  void SIdoSegment();
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
  segmentImage2(int imageType);
  segmentImage2();
  ~segmentImage2();
  //! set the red value you are looking for with thresh error, and skew
  void SIsetRed(int val, int thresh, int skew);
  //! set the green value you are looking for with thresh error, and skew
  void SIsetGreen(int val, int thresh, int skew);
  //! set the blue value you are looking for with thresh error, and skew
  void SIsetBlue(int val, int thresh, int skew);
  //! set the Hue value you are looking for with thresh error, and skew
  void SIsetHue(double val, double thresh, double skew);
  //! set the Saturation value you are looking for with thresh error, and skew
  void SIsetSat(double val, double thresh, double skew);
  //! set the Value (brightness) value you are looking for with thresh error
  void SIsetVal(double val, double thresh, double skew);
  //! set the region of the image to inspect
  void SIsetFrame(int x1, int y1, int x2, int y2, int realX, int realY);
  //! set up averaging for HSV color averaging
  void SIsetHSVavg(long doAvg);
  //! segment image based upon parameters input
  void SIsegment(Image<PixRGB<float> > &image);
  //! segment image based upon parameters input
  void SIsegment(Image<float> &image);
  //! merge all blobs into one big blob, useful if you erase blobs
  /*! else just use returnCandidates */
  Image<long> SIcreateMother(Image<long> &img);
  //! return an image with labeled blobs. Use getBlobMap to map blobs
  Image<long> SIreturnBlobs();
  //! return a bool map off all candidate pixels
  Image<bool> SIreturnCandidates();
  //! return a normalized displayable map off all candidate pixels
  Image<float> SIreturnNormalizedCandidates();
  //! return the image that is being worked on, to check if its ok
  Image<PixRGB<float> > SIreturnWorkImage();
  //! return the image that is being worked on, to check if its ok
  Image<float> SIreturnWorkImageGREY();
  //! return the total number of blobs
  int SInumberBlobs();
  //! return a map of blobs that gives the numeric ID of a blob from the image
  std::vector<long> SIgetBlobMap();
  //! calculate basic mass/center blob properties
  void SIcalcMassCenter();
  //! get blob center in X
  float SIgetCenterX(long blob);
  //! get blob center in X
  float SIgetCenterY(long blob);
  //! get blob mass
  long SIgetMass(long blob);
  //! get X min for a blob
  int SIgetXmin(long blob);
  //! get X max for a blob
  int SIgetXmax(long blob);
  //! get Y min for a blob
  int SIgetYmin(long blob);
  //! get Y max for a blob
  int SIgetYmax(long blob);
  //! get the working image size in X
  int SIgetImageSizeX();
  //! get the working image size in Y
  int SIgetImageSizeY();
  //! get HSV mean values and standard deviations for a blob
  void SIgetHSVvalue(long blob, float *H, float *S, float *V,
                   float *Hstd, float *Sstd, float *Vstd);
  //! do HVS color value means for x last iterations
  void  SIgetHSVvalueMean(long blob, float *H, float *S, float *V,
                        float *Hstd, float *Sstd, float *Vstd);

};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
