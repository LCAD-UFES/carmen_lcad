/*!@file VFAT/segmentImageMC.H Basic image segmenter blob finder using color */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/segmentImageMC.H $
// $Id: segmentImageMC.H 6006 2005-11-29 22:34:22Z rjpeters $
//

// ############################################################
// ############################################################
// ##### --- VFAT ---
// ##### Vision Feature Analysis Tool:
// ##### T. Nathan Mundhenk nathan@mundhenk.com
// ##### Laurent Itti itti@pollux.usc.edu
// #####
// ############################################################
// ############################################################

#ifndef SEGMENTIMAGEMC_H_DEFINED
#define SEGMENTIMAGEMC_H_DEFINED

#include "Image/All.H"
#include "Image/Pixels.H"
#include <time.h>
#include <sys/time.h>

//*******************************************
/* ABOUT THE USE OF TEMPLATES HERE

 the INT value is an integer number and should be an unsigned
 number (but isn't required to be). However, for images larger
 than 256x256 after decimation
 e.g. that contain more than 2^16 pixels, you should use a long instead
 this is because the number of blobs may be as large as the number
 of pixels.

 the FLOAT is for a floating point. A double may be used to obtain more
 accurate values at run time. This is at your discression

*/

//! Define the template declaration for this class
#define SI_TEMPLATE_CLASS <class FLOAT, class INT, unsigned int SI_channels>
//! further define templates for this class
#define SI_TEMPLATE       FLOAT,INT,SI_channels
//! This is a class to track hyper spectal blobs
/*! This class is most likely called by segmentImageTrackMC and will
    segment images by linking pixels and the corresponding image
    regions by linked stripes. It will also segregate each blob and
    track it seperately.
*/
template SI_TEMPLATE_CLASS class segmentImageMC
{
private:
  Image<PixRGB<byte> > *SI_workImage;
  Image<bool> SI_candidatePixels;
  Image<bool> SI_preCandidatePixels;
  Image<long> SI_blobID;
  typename std::vector<Image<FLOAT> > *SI_featureMaps;
  typename std::vector<Image<FLOAT> > SI_infeatureMaps;

  //! values for lower threshold values
  typename std::vector<FLOAT> SI_lowThresh;
  //! values for upper threshold values
  typename std::vector<FLOAT> SI_highThresh;
  //! frame size that will be inspected
  int SI_frameX1,SI_frameY1,SI_frameX2,SI_frameY2;
  //! bools to determine if all values have been set to run image
  bool SI_set1,SI_set2,SI_set3,SI_set4;
  //! set to false to not use band pass filter on candidates
  bool SI_useCandidateBandPass;
  int SI_doType;
  INT SI_num; // number of blob segments
  INT SI_masters; // number of masters;
  INT SI_mastersCount;
  INT SI_totalBlobs;
  INT SI_count;
  INT SI_iter;
  INT SI_maxIDVal;
  // list of a pixels master
  std::vector<long> SI_masterVec;
  typename std::vector<INT> SI_reOrderVec;
  typename std::vector<INT> SI_reverseOrderVec;
  std::vector<bool> SI_reset;
  // list of blob properties
  std::vector<FLOAT> SI_centerX;
  std::vector<FLOAT> SI_centerY;
  // color properties for averageing
  typename std::vector<std::vector<FLOAT> > SI_avg;
  typename std::vector<std::vector<FLOAT> > SI_std;
  typename std::vector<FLOAT> SI_tempAvg;
  typename std::vector<FLOAT> SI_tempStd;
  typename std::vector<INT> SI_N;
  typename std::vector<INT> SI_Xsum;
  typename std::vector<INT> SI_Ysum;
  typename std::vector<INT> SI_mass;
  std::vector<int> SI_xmin;
  std::vector<int> SI_xmax;
  std::vector<int> SI_ymin;
  std::vector<int> SI_ymax;
  //! find any candidate pixel based upon pixel thresholding RGB
  void SIfindCandidates();
  //! find any candidate pixel based upon pixel thresholding RGB
  void SIfindCandidatesNoBandPass();
  //! remove single pixels without neighbors
  void SIremoveSingles();
  //! remove single pixels without neighbors, iterator version with weird mem
  void SIremoveSinglesItr();
  //! scan candidate image and link continious pixels with a unique ID tag
  void SIdiscreteLinking();
  //! variant on discrete linking works orthogonally with iterators
  void SIdiscreteLinkingOrtho();
  //! backward link pixels, find master, relabel masters
  void SIbackwardLink(INT *slave, INT *master);
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
  segmentImageMC();
  ~segmentImageMC();
  //! set the Value (brightness) value you are looking for with thresh error
  void SIsetVal(typename std::vector<FLOAT> &val,
                typename std::vector<FLOAT> &thresh,
                typename std::vector<FLOAT> &skew);
  //! set the region of the image to inspect
  void SIsetFrame(int *x, int *y);
  //! Call during run if color is totally reset
  void SIresetCandidates(bool whichWay);
  //! set up averaging for color averaging adaptation
  void SIsetAvg(INT doAvg);
  //! reset averaging for color averaging adaptation
  void SIresetAvg();
  //! segment image based upon parameters input
  void SIsegment(Image<PixRGB<byte> > *image,
                 typename std::vector<Image<FLOAT> > *featureMap,
                 bool lowPass = false);
  //! toggle band pass filter on candidate pixels on/off
  void SItoggleCandidateBandPass(bool toggle);
  //! merge all blobs into one big blob, useful if you erase blobs
  /*! else just use returnCandidates */
  Image<INT> SIcreateMother(Image<INT> &img);
  //! return an image with labeled blobs. Use getBlobMap to map blobs
  Image<long> SIreturnBlobs();
  //! return a bool map off all candidate pixels
  Image<bool> SIreturnCandidates();
  //! return a normalized displayable map off all candidate pixels
  Image<FLOAT> SIreturnNormalizedCandidates();
  //! return the image that is being worked on, to check if its ok
  Image<PixRGB<FLOAT> > SIreturnWorkImage();
  //! return the total number of blobs
  INT SInumberBlobs();
  //! return a map of blobs that gives the numeric ID of a blob from the image
  std::vector<INT> SIgetBlobMap();
  //! calculate basic mass/center blob properties
  void SIcalcMassCenter();
  //! get blob center in X
  FLOAT SIgetCenterX(INT blob);
  //! get blob center in X
  FLOAT SIgetCenterY(INT blob);
  //! get blob mass
  INT SIgetMass(INT blob);
  //! get X min for a blob
  int SIgetXmin(INT blob);
  //! get X max for a blob
  int SIgetXmax(INT blob);
  //! get Y min for a blob
  int SIgetYmin(INT blob);
  //! get Y max for a blob
  int SIgetYmax(INT blob);
  //! get the working image size in X
  int SIgetImageSizeX();
  //! get the working image size in Y
  int SIgetImageSizeY();
  //! get HSV mean values and standard deviations for a blob
  void SIgetValue(INT *blob, typename std::vector<FLOAT> *mean,
                  typename std::vector<FLOAT> *std, INT *in);
  //! do HVS color value means for x last iterations
  void SIgetValueMean(INT *blobListSize,
                      std::vector<INT> *blobList,
                      typename std::vector<FLOAT> *mean,
                      typename std::vector<FLOAT> *std,
                      FLOAT *mass);
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif
