/*!@file VFAT/featureClusterVision.H  Test the nonparametric classifier
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
// Primary maintainer for this file: T Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/featureClusterVision.H $
// $Id: featureClusterVision.H 9412 2008-03-10 23:10:15Z farhan $
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

#include "Component/ModelComponent.H"
#include "Component/OptionManager.H"
#include "Image/All.H"
#include "Image/Pixels.H"
#include "Image/Point2D.H"
#include "Media/FrameSeries.H"
#include "Neuro/SaliencyMap.H"
#include "Neuro/StdBrain.H"
#include "Neuro/VisualCortex.H"
#include "Raster/Raster.H"
#include "Raster/Raster.H"
#include "Util/Timer.H"
#include "Util/Types.H"
#include "Util/log.H"
#include "Util/readConfig.H"
#include "VFAT/NPclassify2.H"
#include "VFAT/findColorIndex.H"
#include "VFAT/readMatrix.H"
#include "rutz/shared_ptr.h"
#include <stdlib.h>
#include <string>
#include <sys/types.h>

//! main class for VFAT. This combines all the methods etc.
/*! this class takes NPclassify2, covEstimate, Brain/saliency
    as well as ICA/PCA methods to create a saliency based tracking
    and simple identification program.

    There are also many test methods for the different elements in
    this class
*/

template <class FLOAT>
class featureClusterVision : public ModelComponent
{
private:
  //! map of statistically salient points from montyMap
  std::vector<Point2D<int> > fCV_cmap;
  //! old copy of particles
  std::vector<Point2D<int> > fCV_cmapOld;
  //! map of sparse points (to cmap points) from sparcifier
  std::vector<Point2D<int>*> fCV_rmap;
  //! this number is used by the saccade data if needed
  std::vector<Point2D<int> > fCV_jumpTo;
  //! link map to feature values for given points
  typename std::vector<std::vector<FLOAT> > fCV_fmap;
  //! space after ICA and post processing to be clustered
  typename std::vector<std::vector<FLOAT> > fCV_space;
  //! This is a mixture of oreintation channles to rotation invariant texture
  typename std::vector<std::vector<FLOAT> > fCV_mixedRotation;
  //! This is a mixture of motion channles to rotation invariant texture
  typename std::vector<std::vector<FLOAT> > fCV_mixedMotion;
  //! This is a mixture of oreintation channles to rotation invariant texture
  typename std::vector<std::vector<PixH2SV2<FLOAT> > > fCV_mixedRotationH2SV2;
  //! This holds a set of features obtained from stand alone filters
  typename std::vector<std::vector<PixH2SV2<FLOAT> > > fCV_standAloneFeatures;
  //! This holds a set of features obtained from stand alone filters
  typename std::vector<std::vector<PixH2SV2<FLOAT> > > fCV_standAloneFeaturesSin;
  //! This holds a set of features obtained from stand alone filters
  typename std::vector<std::vector<PixH2SV2<FLOAT> > > fCV_standAloneFeaturesCos;
  //! list of covarience matrices
  typename std::vector<std::vector<FLOAT*> > fCV_unmixedMap;

  //! list of the classes for each point
  std::vector<std::vector<int*> >* fCV_classList;
  //! clustered feature list
  typename std::vector<std::vector<std::vector<FLOAT*> > > fCV_sortedSpace;
  //! gives the size of each matrix, that is features per channel
  std::vector<int> fCV_featureMatrixSizes;
  //! how many features per channel following ICA
  std::vector<int*> fCV_ICAfeaturesPerChannel;
  //! this is an index label for saccade data from file if used
  std::vector<int> fCV_indexNumber;
  //! this is a constant to normalize this feature from 0 to 1 or -1 to 1
  typename std::vector<FLOAT> fCV_featureNormConst;
  typename std::vector<FLOAT> fCV_featureTransConst;
  //! holds results of low pass at location
  typename std::vector<PixRGB<FLOAT> > fCV_lowPassVector;


  //! The unmixed features from ICAemacsc
  typename std::vector<Image<FLOAT> > fCV_Unmixed;
  //! ICA unmixing Matrices
  typename std::vector<Image<FLOAT> > fCV_ICAunmix;
  //! Sine type gabor filters
  typename std::vector<Image<FLOAT> > fCV_gaborFiltersSin;
  //! Cosine type gabor filters
  typename std::vector<Image<FLOAT> > fCV_gaborFiltersCos;
  //! This ia a pointer to a bunch of conspicuity maps for no-brain use
  typename std::vector<Image<FLOAT> > *fCV_cmaps;


  //! sorted classes by size
  std::vector<long> fCV_sortClassSize;
  std::vector<long> fCV_sortClassMember;
  //! A list of each features name
  std::vector<std::string> fCV_featureName;
  //! Post ICA feature name list
  std::vector<std::string*> fCV_featureNameICA;
  //! this is a filler string for pointers
  std::string fCV_NULLstring;
  //! the name of the low pass kernel file
  std::string fCV_lowPassKernelName;
  long fCV_sortCount;
  //! how many feature channels are you dealing with
  int fCV_channelNumbers;
  //! many total features are there
  int fCV_totalFeatures;
  //! how many total features are there potentially, including ignored features
  int fCV_totalPotFeatures;
  //! How many features are there per channel
  int fCV_featuresPerChannel;
  //! counters for features selected
  int fCV_countMM, fCV_countSM;
  //! image size values for salmap
  int fCV_sizeX, fCV_sizeY;
  FLOAT fCV_sizeXbias, fCV_sizeYbias;
  //! offset into feature set of orientations
  int fCV_oriOffset;
  //! offset into feature set of motion
  int fCV_motOffset;
  //! offset into V2 junction process
  int fCV_mixOffset;
  //! spatial component offset
  int fCV_spatOffset;
  //! offset into color features
  int fCV_colorOffset;
  //! offset into combined motion channel
  int fCV_motionCombinedOffset;
  //! reduced feature set size
  int fCV_reducedFeatureCount;
  //! how many sparce points do you want?
  int fCV_sparcePoints;
  //! holder for a new matrix size
  int fCV_newMatSize;
  //! this is the type of low pass filter to use for color extraction
  int fCV_lowPassType;
  //! amount of decimation to apply to monte carlo mapping
  int fCV_monteDec;

  //! size of data set of features
  unsigned int fCV_covDataSizeLast;
  unsigned int fCV_covDataSizeCurrent;
  unsigned int fCV_covDataSizeMatch;
  //! how many scale reductions to perform on image in gabor filter
  unsigned int fCV_gaborScales;
  //! Which feaures are turned on
  bool fCV_blueYellowOn, fCV_redGreenOn, fCV_flickerOn, fCV_lumOn, fCV_oriOn;
  bool fCV_motionOn, fCV_spatialOn, fCV_mixAlphaOn, fCV_mixBetaOn;
  bool fCV_mixGammaOn;
  bool fCV_redOn, fCV_greenOn, fCV_blueOn, fCV_yellowOn;
  bool fCV_hueOn, fCV_satOn, fCV_valOn;
  bool fCV_hue1On, fCV_hue2On;
  bool fCV_motionCombinedOn;
  //! tells if we should use a quarter size reduction image pyramid
  bool fCV_gaborUseQuarter;
  //! these are the weights for each feature
  FLOAT fCV_redGreenWeight, fCV_blueYellowWeight, fCV_flickerWeight;
  FLOAT fCV_lumWeight, fCV_oriWeight, fCV_motionWeight, fCV_spatialWeight;
  FLOAT fCV_mixAlphaWeight, fCV_mixBetaWeight, fCV_mixGammaWeight;
  FLOAT fCV_redWeight, fCV_greenWeight, fCV_blueWeight, fCV_yellowWeight;
  FLOAT fCV_hueWeight, fCV_satWeight, fCV_valWeight;
  FLOAT fCV_hue1Weight, fCV_hue2Weight;
  FLOAT fCV_motionCombinedWeight;
  //! these are a few pointers to some selected normalizers
  FLOAT *fCV_redNorm, *fCV_greenNorm, *fCV_blueNorm, *fCV_yellowNorm;
  FLOAT *fCV_hueNorm, *fCV_satNorm, *fCV_valNorm;
  FLOAT *fCV_hue1Norm, *fCV_hue2Norm;
  //! these are a few pointers to some selected normalizers
  FLOAT *fCV_redTrans, *fCV_greenTrans, *fCV_blueTrans, *fCV_yellowTrans;
  FLOAT *fCV_hueTrans, *fCV_satTrans, *fCV_valTrans;
  FLOAT *fCV_hue1Trans, *fCV_hue2Trans;
  //! how many features per channel on each channel after ICA
  int fCV_ICAfeaturesRedGreen, fCV_ICAfeaturesBlueYellow;
  int fCV_ICAfeaturesFlicker, fCV_ICAfeaturesLum;
  int fCV_ICAfeaturesMotion, fCV_ICAfeaturesOri;
  int fCV_ICAfeaturesSpatial, fCV_ICAfeaturesAlpha;
  int fCV_ICAfeaturesBeta, fCV_ICAfeaturesGamma;
  int fCV_ICAfeaturesRed, fCV_ICAfeaturesGreen;
  int fCV_ICAfeaturesBlue, fCV_ICAfeaturesYellow;
  int fCV_ICAfeaturesHue, fCV_ICAfeaturesSat;
  int fCV_ICAfeaturesVal;
  int fCV_ICAfeaturesHue1, fCV_ICAfeaturesHue2;
  int fCV_ICAfeaturesMotionCombined;
  //! these are max values on these features
  FLOAT fCV_maxMotVal, fCV_maxOriVal;
  //! this is the exponent to be applied to the saliency map
  FLOAT fCV_saliencyExp;
  //! amount of bias to pass from one cluster to the next (Markovian)
  FLOAT fCV_NPtemporalBias;
  //! bias for recycling particles in x*standard deviation
  FLOAT fCV_densityBias;
  //! bias over salmap low pass
  FLOAT fCV_salmapLowPassTemporalBias;
  //! the standard deviation for gabor filters
  FLOAT fCV_gaborStandardDev;
  //! the period for gabor filters
  FLOAT fCV_gaborPeriod;

  //! do the bias, or just set it
  bool fCV_doSLPTB;
  //! this is true if you want to print out clusters
  bool fCV_printOutClusters;
  //! this is true if you want to see the basic feature maps
  bool fCV_printOutFeatures;
  //! print output on the real image input
  bool fCV_doReal;
  //! is true if NPclassify is to be biased by the last iteration
  bool fCV_doNPbias;
  //! do you want to write out time stats to a file
  bool fCV_useTimerFile;
  //! do we use a brain for saliency and features?
  bool fCV_useBrain;

  //! Which alternation of covHolder are we using
  //bool fCV_useCovHolderUp;
  //! What is the current covholder we are using
  unsigned int fCV_currentCovHolder;
  //! on first iteration set all matched to themselves
  bool fCV_doMatchSelf;
  //! The real image to write to
  Image<PixRGB<FLOAT> > fCV_realImage;
  //! low pass version of real image
  Image<PixRGB<FLOAT> > fCV_realImageLowPass;
  //! H2SV2 version of real image
  Image<PixH2SV2<FLOAT> > fCV_realImageH2SV2;
  //! H2SV2 low pass version of real image
  Image<PixH2SV2<FLOAT> > fCV_realImageH2SV2LowPass;
  //! average temporal band pass over salmap
  Image<FLOAT> fCV_salmapLowPass;
  //! low pass kernel for individual use
  Image<FLOAT> fCV_lowPassKernel;
  //! output result image suitable for framing : classes
  Image<PixRGB<FLOAT> > fCV_outImageClasses;
  //! output result image suitable for framing : classes across time
  Image<PixRGB<FLOAT> > fCV_outImageTemporal;
  //! output result image suitable for framing : temporal target
  Image<PixRGB<FLOAT> > fCV_outImageTarget;
  //! salmap pointer to use in no-brain condition
  Image<FLOAT> *fCV_noBrainSalmap;

  //! the name of the real image
  std::string fCV_fileName;
  //! this is a map vector that points to the feature on values
  std::vector<bool*> fCV_featureOn;
  //! this is a list of particles which are to be kept
  std::vector<bool> fCV_keepParticle;
  //! the total number of channels turned on
  int fCV_channelsOn;
  //! this is a map vector that points to the weight values
  std::vector<FLOAT*> fCV_weights;
  //! brain
  nub::soft_ref<StdBrain> fCV_brain;
  //! point to input frames
  nub::soft_ref<InputFrameSeries> fCV_iframes;
  //! shared pointer to sal map
  nub::soft_ref<SaliencyMap> fCV_SM;
  //! shared pointer to visual cortex
  nub::soft_ref<VisualCortex> fCV_VC;
  //! NP classify object
  NPclassify2<FLOAT> fCV_NP;
  //! covEstimate Constructor
  covEstimate<double> fCV_CV;
  //! template for resizing covHolder on the fly
  covHolder<double> fCV_tcov;

  //! an array of covarriance data
  std::vector<covHolder<double> > *fCV_covHolderCurrent;
  std::vector<covHolder<double> > *fCV_covHolderLast;
  //std::vector<covHolder<FLOAT> > fCV_covHolderUp;
  //std::vector<covHolder<FLOAT> > fCV_covHolderDown;
  std::vector<covHolder<double> > fCV_covHolderMatch;
  std::vector<std::vector<covHolder<double> > > fCV_covHolder;
  //! private methods
  void fCVswitchCov();
  void fCVlowPass();
  void fCVfindFeaturesBrain();
  void fCVfindFeaturesNoBrain();
  void fCVfindFeaturesFromFile(std::string fileName);
  void fCVrunICA();
  void fCVrunNPclassify();
  void fCVrunCovEstimate();
  void fCVcheckParticles();
  void fCVmatchClassTemporal();
  void fCVsetImageParams();
  void fCVresizeMaps1(int sparcePoints);
  void fCVresizeMaps2(int sparcePoints, int newMatSize);
  void fCVcreateGaborFilters();
public:
  featureClusterVision(OptionManager& mgr,
                       const std::string& descrName,
                       const std::string& tagName,
                       nub::soft_ref<StdBrain>& _brain,
                       nub::soft_ref<InputFrameSeries>& _ifs,
                       const std::string& extraArg0);

  //! fCV without a std brain, user must supply saliency stuff externally
  featureClusterVision(OptionManager& mgr,
                       const std::string& descrName,
                       const std::string& tagName,
                       Image<FLOAT> *salMap,
                       typename std::vector<Image<FLOAT> > *cmaps,
                       nub::soft_ref<InputFrameSeries>& _ifs,
                       const std::string& extraArg0);
  ~featureClusterVision();

  //! called by default constructor to initialize this class
  void fCVsetUpfCV(OptionManager& mgr,
                   const std::string& descrName,
                   const std::string& tagName,
                   nub::soft_ref<InputFrameSeries>& _ifs,
                   const std::string& extraArg0);


  //! When called, this mixes the four channels together into a single channle
  /*! This works by finding the normalized difference between the output
    of two orthogonal channles. Then it averages the results over all the
    channels specified. This works to find the rotation invariant
    properties of orientation responses.
    @param data This is the raw channle input
    @param ch1a this is the first channle input
    @param ch1b this is the orthogonal channel to ch1a
    @param ch2a this is the second channel set to work in
    @param ch2b this is the orthogonal channel to ch2a
    @param outAlpha this is a vector of the mixed channels strength
    @param outBeta this is a vector of the mixed channels line clearity
    @param outGamma this is a vector of the mixed channels junction clearity
    @param norm1 this is a normalizer over ch1a and ch1b (e.g. Max value)
    @param norm2 this is a normalizer over ch2a and ch2b (e.g. Max value)
  */
  void fCVmixChannels(std::vector<std::vector<FLOAT> > *data,
                   int ch1a, int ch1b,
                   int ch2a, int ch2b,
                   typename std::vector<FLOAT> *outAlpha,
                   typename std::vector<FLOAT> *outBeta,
                   typename std::vector<FLOAT> *outGamma,
                   FLOAT norm1, FLOAT norm2,
                   int size);
  //! When called, this mixes the four channels together into a single channle
  /*! This works by finding the normalized difference between the output
    of two orthogonal channles. Then it averages the results over all the
    channels specified. This works to find the rotation invariant
    properties of orientation responses. This is like the other mixChannels
    however, this works off of Image class images.
    @param img0 An image filtered at 0 degrees
    @param img45 An image filtered at 45 degrees
    @param img90 An image filtered at 90 degrees
    @param img135 An image filtered at 135 degrees
    @param Alpha This is an image of lineyness in an image
    @param Beta This is an image of the crossyness in an image
    @param Gamma This is the junction results with crisp crosses
    @param norm1 This is a normalizer in (img0+img90)/norm
    @param norm2 This is a normalizer in (img45+img135)/norm
  */
  void fCVmixChannels(Image<FLOAT> &img0, Image<FLOAT> &img45,
                   Image<FLOAT> &img90, Image<FLOAT> &img135,
                   Image<FLOAT> *Alpha, Image<FLOAT> *Beta,
                   Image<FLOAT> *Gamma);
  //! This calles mixChannels over the entire dataset
  void fCVfindMixedChannels();
  //! when called this extracts the features at the most salient point
  /*! Most likely you will not need to ever call this unless you are just
    trying to get the most salient points plus the mixed orientations
  */
  void fCVcheckMixing();
  //! When called this will print out an ICA/PCA representation of each channle
  /*! This shows the end result of the ICA/PCA process on each feature
    channel
  */
  void fCVcheckICA();
  //! When called this will print out an ICA/PCA representation of each channle
  /*! This shows the end result of the ICA/PCA process on this feature
    channel
  */
  void fCVcheckICA(int channel, bool findMixed);
  //! check the output from the combined motion channel
  void fCVcheckMotionCombined(long frame);
  //! when called this will output features to a file and quit
  //! when called this will output post ICA features to a file and quit
  void fCVfeaturesToFile(std::string fileName, bool _new);
  //! check stand alone feature extraction
  void fCVrunStandAloneMSBatchFilter(std::string filename);
  //! check stand alone feature extraction
  void fCVrunStandAloneMSBatchTest(std::string filename);

  void fCVICAfeaturesToFile(std::string fileName);
  //! main method to call for most purposes
  void fCVclusterImage();
  //! A test method to use with raw saccade data
  void fCVsaccadeTest(std::string _maskFile, std::string _outFile, std::string _label,
                   std::string _fileName);
  //! test stand alone feature extraction
  void fCVstandAloneFeatureTest(std::string _fileName);
  //! fetches an images base statistics
  void fCVgetImageBaseStats(std::string _maskFile, std::string _imageFile,
                            std::string _outFile, std::string _label);
  //! fetches an images base statistics
  void fCVgetImageComplexStats(std::string _maskFile, std::string _imageFile,
                            std::string _outFile, std::string _label);

  //! upload a raw copy of the image
  void fCVuploadImage(Image<PixRGB<byte> > &input, std::string fileName);
  void fCVprintOutClusters();
  void fCVprintOutCovSlices(int sizeX,int sizeY);
  void fCVprintOutBayesClass();
  void fCVprintOutNeighborClass();
  void fCVdumpCovMatrix(std::string fileName);
  //! get the signature data from this class
  std::vector<covHolder<double> > fCVgetCovHolders();
  //! how many signatures do we have
  unsigned int fCVgetCovHolderSize();
  //! get back images of clusters for analysis
  void fCVgetClusterImages(Image<PixRGB<FLOAT> > *classImage,
                           Image<PixRGB<FLOAT> > *temporalImage,
                           Image<PixRGB<FLOAT> > *targetImage,
                           Image<FLOAT> *salMap);
  void fCVprocessOutSaccadeData(std::string maskFile, std::string outFile, std::string _label);
};





