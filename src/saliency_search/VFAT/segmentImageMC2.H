/*!@file VFAT/segmentImageMC2.H Basic image segmenter blob finder using color */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/segmentImageMC2.H $
// $Id: segmentImageMC2.H 9898 2008-07-09 05:17:17Z mundhenk $
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

#ifndef SEGMENTIMAGEMC2_H_DEFINED
#define SEGMENTIMAGEMC2_H_DEFINED

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
template SI_TEMPLATE_CLASS class segmentImageMC2
{
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
  segmentImageMC2();
  ~segmentImageMC2();

  //! set the Value (brightness) value you are looking for with thresh error
  /*! This method will set the threshold to be val +/- thresh. Thus,
      consider val to be the mean color you wish to track with a standard
      deviation like boundary as thresh. To set the high and low values
      directly use SIsetValThresh as an alternative.
  */
  void SIsetVal(const typename std::vector<FLOAT> &val,
                const typename std::vector<FLOAT> &thresh);
  //! set the Value (brightness) value you are looking for with thresh error
  /*! Set threshold values directly as a boundary between high and low.
      If you wish to work more on mean values use SIsetVal instead.
  */
  void SIsetValThresh(const typename std::vector<FLOAT> &high,
                      const typename std::vector<FLOAT> &low);
  //! set the region of the image to inspect
  void SIsetFrame(int *x, int *y);
  //! Call during run if color is totally reset
  void SIresetCandidates(const bool whichWay);
  //! set up averaging for color averaging adaptation
  void SIsetAvg(const INT doAvg);
  //! reset averaging for color averaging adaptation
  void SIresetAvg();
  //! segment image based upon parameters input (depricated)
  void SIsegment(Image<PixRGB<byte> > *image,
                 typename std::vector<Image<FLOAT> > *featureMap,
                 const bool lowPass = false);
  //! segment image based upon parameters input
  void SIsegment(typename std::vector<Image<FLOAT> > *featureMap,
                 const bool lowPass = false);
  //! toggle band pass filter on candidate pixels on/off
  void SItoggleCandidateBandPass(const bool toggle);
  //! Turn on or off single removal: defaults to ON
  void SItoggleRemoveSingles(const bool toggle);
  //! override and set the kill value for removing singles
  void SIsetKillValue(const unsigned int kv);
  //! merge all blobs into one big blob, useful if you erase blobs
  /*! else just use returnCandidates */
  Image<INT> SIcreateMother(const Image<INT> &img) const;
  //! return an image with labeled blobs. Use getBlobMap to map blobs
  Image<int> SIreturnBlobs() const;
  //! return a bool map off all candidate pixels
  Image<bool> SIreturnCandidates() const;
  //! return a normalized displayable map off all candidate pixels
  Image<FLOAT> SIreturnNormalizedCandidates() const;
  //! return the image that is being worked on, to check if its ok
  Image<PixRGB<FLOAT> > SIreturnWorkImage() const;
  //! return the total number of blobs
  INT SInumberBlobs() const;
  //! return a map of blobs that gives the numeric ID of a blob from the image
  std::vector<INT> SIgetBlobMap() const;
  //! calculate basic mass/center blob properties
  void SIcalcMassCenter();
  //! get blob center in X
  FLOAT SIgetCenterX(const INT blob) const;
  //! get blob center in X
  FLOAT SIgetCenterY(const INT blob) const;
  //! get blob mass
  INT SIgetMass(const INT blob) const;
  //! get X min for a blob
  int SIgetXmin(const INT blob) const;
  //! get X max for a blob
  int SIgetXmax(const INT blob) const;
  //! get Y min for a blob
  int SIgetYmin(const INT blob) const;
  //! get Y max for a blob
  int SIgetYmax(const INT blob) const;
  //! get the working image size in X
  int SIgetImageSizeX() const;
  //! get the working image size in Y
  int SIgetImageSizeY() const;
  //! get HSV mean values and standard deviations for a blob
  void SIgetValue(INT *blob, typename std::vector<FLOAT> *mean,
                  typename std::vector<FLOAT> *std, INT *in);
  //! do HVS color value means for x last iterations
  void SIgetValueMean(INT *blobListSize,
                      std::vector<INT> *blobList,
                      typename std::vector<FLOAT> *mean,
                      typename std::vector<FLOAT> *std,
                      FLOAT *mass);
private:
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
  void SIbackwardLink(const INT slave, const INT master);
  //! basic link pixels
  void SIbasicLink(const INT node);
  //! combine slaves together into single blobs
  void SIcombine();
  //! get information on blobs for debugging
  void SIgetBlobs();
  //! Call to segmentation which calls most of these methods
  void SIdoSegment();
  Image<PixRGB<byte> > *SI_workImage;
  Image<bool>           SI_candidatePixels;
  Image<bool>           SI_preCandidatePixels;
  Image<int>            SI_blobID;
  // color properties for averageing
  typename std::vector<std::vector<FLOAT> > SI_avg;
  typename std::vector<std::vector<FLOAT> > SI_std;
  typename std::vector<Image<FLOAT> >      *SI_featureMaps;
  typename std::vector<Image<FLOAT> >       SI_infeatureMaps;
  typename std::vector<FLOAT> SI_tempAvg;
  typename std::vector<FLOAT> SI_tempStd;
  //! values for lower threshold values
  typename std::vector<FLOAT> SI_lowThresh;
  //! values for upper threshold values
  typename std::vector<FLOAT> SI_highThresh;
  //! list of blob properties
  typename std::vector<FLOAT> SI_centerX;
  typename std::vector<FLOAT> SI_centerY;
  typename std::vector<INT>   SI_N;
  typename std::vector<INT>   SI_Xsum;
  typename std::vector<INT>   SI_Ysum;
  typename std::vector<INT>   SI_mass;
  typename std::vector<INT>   SI_reOrderVec;
  typename std::vector<INT>   SI_reverseOrderVec;
  // list of a pixels master
  typename std::vector<int>   SI_masterVec;
  std::vector<short>          SI_xmin;
  std::vector<short>          SI_xmax;
  std::vector<short>          SI_ymin;
  std::vector<short>          SI_ymax;
  std::vector<bool>           SI_reset;
  INT SI_num; // number of blob segments
  INT SI_masters; // number of masters;
  INT SI_mastersCount;
  INT SI_totalBlobs;
  INT SI_count;
  INT SI_iter;
  INT SI_maxIDVal;
  int SI_doType;
  //! the typical kill value for removing single pixels
  unsigned int SI_killVal;
  //! frame size that will be inspected
  unsigned short SI_frameX1,SI_frameY1,SI_frameX2,SI_frameY2;
  //! bools to determine if all values have been set to run image
  bool SI_set1,SI_set2,SI_set3,SI_set4;
  //! set to false to not use band pass filter on candidates
  bool SI_useCandidateBandPass;
  //! Should singles be removed? Default: yes
  bool SI_removeSingles;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif
