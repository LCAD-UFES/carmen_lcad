/*!@file VFAT/featureClusterVision.C  Test the nonparametric classifier
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/featureClusterVision.C $
// $Id: featureClusterVision.C 14376 2011-01-11 02:44:34Z pez $
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

#include "VFAT/featureClusterVision.H"

#include "Image/Conversions.H"     // for vectorToImage()
#include "Image/Kernels.H"         // for gaborFilter()
#include "Image/MatrixOps.H"
#include "Util/Assert.H"
#include "Util/Timer.H"
#include "VFAT/VFATOpts.H"
#include "VFAT/covEstimate.H"
#include "VFAT/featureClusterFilters.H"

#define OVER 5
#define COVSIZE 605
#define COVHOLDER 10
#define DATABOUND 10000
#define VEC_RESIZE 100
#define FOOANDSTUFF 2
#define DUMPNP false
#define STUPIDMASKOFFSET 133

namespace
{
  // Moved getColor() and getGrey() here during Image class cleanup,
  // they could go somewhere else eventually but this seemed to be the
  // only place that was using them.

  template <class T>
  void getColor(const Image<T>& A, Image<PixRGB<T> >& B)
  {
    B.resize(A.getWidth(),A.getHeight());
    typename Image<T>::const_iterator BWptr     = A.begin();
    typename Image<PixRGB<T> >::iterator COLptr = B.beginw();

    while(BWptr != A.end())
      {
        *COLptr = PixRGB<T>(PixRGB<double>(PixHSV<double>(0,0,*BWptr*255.0F)));
        ++BWptr; ++COLptr;
      }
  }

  template <class T>
  void getGrey(const Image<PixRGB<float> >& A,Image<T>& B)
  {
    B.resize(A.getWidth(),A.getHeight());
    typename Image<T>::iterator BWptr  = B.beginw();
    typename Image<PixRGB<float> >::const_iterator COLptr = A.begin();
    float h,s,v;
    while(BWptr != B.endw())
      {
        PixRGB<float> pix = *COLptr;
        PixHSV<float>(pix).getHSV(h,s,v);
        *BWptr = (T)(v/255.0F);
        ++BWptr; ++COLptr;
      }
  }
}

// NOTE: The bulk of the code here is just to load values from config files
// and to set up basic data structures.
template <class FLOAT>
featureClusterVision<FLOAT>::
featureClusterVision(OptionManager& mgr,
                     const std::string& descrName,
                     const std::string& tagName,
                     nub::soft_ref<StdBrain>& _brain,
                     nub::soft_ref<InputFrameSeries>& _ifs,
                     const std::string& extraArg0)
  :
  ModelComponent(mgr, descrName, tagName)
{
  fCV_NULLstring = "NULL";
  std::cerr << "(0) GETTING shared pointers and MM devices\n";
  fCV_brain      = _brain;
  fCV_useBrain   = true;
  fCVsetUpfCV(mgr,descrName,tagName,_ifs,extraArg0);
}

template <class FLOAT>
featureClusterVision<FLOAT>::
featureClusterVision(OptionManager& mgr,
                     const std::string& descrName,
                     const std::string& tagName,
                     Image<FLOAT> *salMap,
                     std::vector<Image<FLOAT> > *cmaps,
                     nub::soft_ref<InputFrameSeries>& _ifs,
                     const std::string& extraArg0)
  :
  ModelComponent(mgr, descrName, tagName)
{
  fCV_NULLstring    = "NULL";
  fCV_cmaps         = cmaps;
  fCV_noBrainSalmap = salMap;
  std::cerr << "(0) GETTING shared pointers and MM devices\n";
  fCV_useBrain = false;
  fCVsetUpfCV(mgr,descrName,tagName,_ifs,extraArg0);
}

template <class FLOAT>
void featureClusterVision<FLOAT>::
fCVsetUpfCV(OptionManager& mgr,
            const std::string& descrName,
            const std::string& tagName,
            nub::soft_ref<InputFrameSeries>& _ifs,
            const std::string& extraArg0)
{
  fCV_iframes = _ifs;
  Image<PixRGB<byte> > fuck = Raster::ReadRGB(extraArg0, RASFMT_PNM);
  std::string NPconf = getManager().getOptionValString(&OPT_NPconfig);
  std::cerr << "NP config    file\t" << NPconf << "\n";
  std::string Pconf = getManager().getOptionValString(&OPT_Polyconfig);
  std::cerr << "Poly config  file\t" << Pconf << "\n";
  std::string Lconf = getManager().getOptionValString(&OPT_localconfig);
  std::cerr << "Local config file\t" << Lconf << "\n";
  std::string Mat = getManager().getOptionValString(&OPT_icamatrix);
  std::cerr << "ICA Matrix   file\t" << Mat << "\n";

  // config file for NP classify
  std::cerr << "(1) SETTING readConfig Objects";
  readConfig NPConfig(25);
  // config file for local method (this one)
  std::cerr << ".";
  readConfig localConfig(25);
  // config file for kernel parameters in NPconfig
  std::cerr << ".\n";
  readConfig KernelConfig(25);

  std::cerr << "(2) READING readConfig Files\n";
  // load config values for local config
  NPConfig.openFile((char*)NPconf.c_str());
  localConfig.openFile((char*)Lconf.c_str());
  KernelConfig.openFile((char*)Pconf.c_str());

  std::cerr << "(3) SETTING up NP classifier\n";
  // create NP classify object
  fCV_NP.NPsetup(NPConfig,KernelConfig,false);
  //resize vectors
  Point2D<int> tempPoint(0,0);
  std::cerr << "(4) SETTING local variables\n";
  fCV_channelNumbers = (int)localConfig.getItemValueF("channelNumbers");
  fCV_totalFeatures = (int)localConfig.getItemValueF("totalFeatures");
  fCV_totalPotFeatures = (int)localConfig.getItemValueF("totalPotFeatures");
  fCV_featuresPerChannel = (int)localConfig.getItemValueF("featuresPerChannel");
  // This is currently equal to 22
  int secretVariable = (fCV_totalPotFeatures/fCV_featuresPerChannel)+16;
  fCV_printOutFeatures = localConfig.getItemValueB("printOutFeatures");
  fCV_printOutClusters = localConfig.getItemValueB("printOutClusters");
  fCV_maxOriVal =  localConfig.getItemValueF("maxOriVal");
  fCV_maxMotVal =  localConfig.getItemValueF("maxMotVal");

  fCV_sizeX = fuck.getWidth(); // taken from input image
  fCV_sizeY = fuck.getHeight(); // taken from input image
  fCV_reducedFeatureCount =
    (int)localConfig.getItemValueF("reducedFeatureCount");
  fCV_sparcePoints = (int)localConfig.getItemValueF("sparcePoints");

  std::cerr << "(5) RESIZING local vectors and Images\n";
  std::cerr << "... cmap resize........\t" << (fCV_sizeX*fCV_sizeY) << "\n";
  fCV_cmap.resize(fCV_sizeX*fCV_sizeY,tempPoint);
  fCV_cmapOld.resize(fCV_sizeX*fCV_sizeY,tempPoint);
  fCV_keepParticle.resize(fCV_sizeX*fCV_sizeY,false);

  std::cerr << "... rmap resize........\t" << (fCV_sizeX*fCV_sizeY) << "\n";
  // initialized to 0 to avoid "warning: ‘temppPoint’ is used
  // uninitialized in this function" with g++ 4.1
  Point2D<int> *temppPoint = 0;
  fCV_rmap.resize(fCV_sizeX*fCV_sizeY,temppPoint);

  fCVresizeMaps1(fCV_sparcePoints);

  int fpc = fCV_reducedFeatureCount/fCV_channelNumbers;

  std::cerr << "... dub resize.........\t" << fCV_sizeX << " x "
            << fCV_sizeY << "\n";
  Image<FLOAT> dub;
  dub.resize(fCV_sizeX,fCV_sizeY);
  std::cerr << "... ICAunmix resize....\t" << secretVariable << "\n";
  fCV_ICAunmix.resize(secretVariable,dub);

  std::cerr << "... Unmixed resize.....\t" << secretVariable << "\n";
  fCV_Unmixed.resize(secretVariable,dub);

  std::cerr << "... featureMatrixSizes.\t" << secretVariable << "\n";
  fCV_featureMatrixSizes.resize(secretVariable,fpc);

  std::cerr << "(6) OPENING and reading ICA Matrix\n";

  Image<FLOAT> ttemp;

  for(int i = 0; i < secretVariable; i++)
  {
    std::cerr << "--\n";
    char str[100];
    sprintf(str,"%s.%d.dat",Mat.c_str(),(i+1));
    readMatrix rm(str);
    rm.echoMatrix();
    fCV_ICAunmix[i] = rm.returnMatrixAsImage();
    //ttemp = rm.returnMatrixAsImage();
    //fCV_ICAunmix[i] = transpose(ttemp);
  }
  std::cerr << "\n";
  fCV_currentCovHolder = 0;
  bool tbool = false;
  FLOAT tFLOAT = 0.0F;
  int tint = 0;
  std::cerr << "(7) resizing map for features and weights\n";
  fCV_featureOn.resize(secretVariable,&tbool);
  fCV_featureName.resize(secretVariable,"Undefined");
  fCV_weights.resize(secretVariable,&tFLOAT);
  fCV_featureNormConst.resize(secretVariable,1.0F);
  fCV_featureTransConst.resize(secretVariable,1.0F);
  fCV_ICAfeaturesPerChannel.resize(secretVariable,&tint);
  std::string temp = "groovy";
  // fCV_featureNameICA.resize(secretVariable,&temp);
  // load in these values as individual values but create a vector of
  // pointers for quick reference

  std::cerr << "(8) loading feature activation bools\n";
  fCV_blueYellowOn = localConfig.getItemValueB("blueYellowOn");
  fCV_featureOn[0] = &fCV_blueYellowOn;
  fCV_featureName[0] = "BlueYellow";
  fCV_redGreenOn = localConfig.getItemValueB("redGreenOn");
  fCV_featureOn[1] = &fCV_redGreenOn;
  fCV_featureName[1] = "RedGreen";
  fCV_flickerOn = localConfig.getItemValueB("flickerOn");
  fCV_featureOn[2] = &fCV_flickerOn;
  fCV_featureName[2] = "Flicker";
  fCV_lumOn = localConfig.getItemValueB("lumOn");
  fCV_featureOn[3] = &fCV_lumOn;
  fCV_featureName[3] = "Luminance";
  fCV_oriOn = localConfig.getItemValueB("oriOn");
  fCV_featureOn[4] = &fCV_oriOn;
  fCV_featureName[4] = "Orientation1";
  fCV_featureOn[5] = &fCV_oriOn;
  fCV_featureName[5] = "Orientation2";
  fCV_featureOn[6] = &fCV_oriOn;
  fCV_featureName[6] = "Orientation3";
  fCV_featureOn[7] = &fCV_oriOn;
  fCV_featureName[7] = "Orientation4";
  fCV_motionOn = localConfig.getItemValueB("motionOn");
  fCV_featureOn[8] = &fCV_motionOn;
  fCV_featureName[8] = "Motion1";
  fCV_featureOn[9] = &fCV_motionOn;
  fCV_featureName[9] = "Motion2";
  fCV_featureOn[10] = &fCV_motionOn;
  fCV_featureName[10] = "Motion3";
  fCV_featureOn[11] = &fCV_motionOn;
  fCV_featureName[11] = "Motion4";
  fCV_spatialOn = localConfig.getItemValueB("spatialOn");
  fCV_featureOn[12] = &fCV_spatialOn;
  fCV_featureName[12] = "SpatialXY";
  fCV_mixAlphaOn = localConfig.getItemValueB("mixAlphaOn");
  fCV_featureOn[13] = &fCV_mixAlphaOn;
  fCV_featureName[13] = "MixedAlpha";
  fCV_mixBetaOn = localConfig.getItemValueB("mixBetaOn");
  fCV_featureOn[14] = &fCV_mixBetaOn;
  fCV_featureName[14] = "MixedBeta";
  fCV_mixGammaOn = localConfig.getItemValueB("mixGammaOn");
  fCV_featureOn[15] = &fCV_mixGammaOn;
  fCV_featureName[15] = "MixedGamma";
  // New combined motion channel
  fCV_motionCombinedOn = localConfig.getItemValueB("motionCombinedOn");
  fCV_featureOn[16] = &fCV_motionCombinedOn;
  fCV_featureName[16] = "MotionCombined";
  // color channels
  fCV_redOn = localConfig.getItemValueB("redOn");
  fCV_featureOn[17] = &fCV_redOn;
  fCV_featureName[17] = "ColorRed";
  fCV_greenOn = localConfig.getItemValueB("greenOn");
  fCV_featureOn[18] = &fCV_greenOn;
  fCV_featureName[18] = "ColorGreen";
  fCV_blueOn = localConfig.getItemValueB("blueOn");
  fCV_featureOn[19] = &fCV_blueOn;
  fCV_featureName[19] = "ColorBlue";
  fCV_yellowOn = localConfig.getItemValueB("yellowOn");
  fCV_featureOn[20] = &fCV_yellowOn;
  fCV_featureName[20] = "ColorYellow";
  fCV_hueOn = localConfig.getItemValueB("hueOn");
  fCV_featureOn[21] = &fCV_hueOn;
  fCV_featureName[21] = "ColorHue";
  fCV_satOn = localConfig.getItemValueB("satOn");
  fCV_featureOn[22] = &fCV_satOn;
  fCV_featureName[22] = "ColorSaturation";
  fCV_valOn = localConfig.getItemValueB("valOn");
  fCV_featureOn[23] = &fCV_valOn;
  fCV_featureName[23] = "ColorValue";
  fCV_hue1On = localConfig.getItemValueB("hue1On");
  fCV_featureOn[24] = &fCV_hue1On;
  fCV_featureName[24] = "ColorHue1";
  fCV_hue2On = localConfig.getItemValueB("hue2On");
  fCV_featureOn[25] = &fCV_hue2On;
  fCV_featureName[25] = "ColorHue2";
  // load in these values as individual values but create a vector of
  // pointers for quick reference

  std::cerr << "(9) loading feature weights for clustering\n";
  fCV_blueYellowWeight = localConfig.getItemValueF("blueYellowWeight");
  fCV_weights[0] = &fCV_blueYellowWeight;
  fCV_featureNormConst[0] = localConfig.getItemValueF("blueYellowNorm");
  fCV_featureTransConst[0] = localConfig.getItemValueF("blueYellowTrans");
  fCV_redGreenWeight = localConfig.getItemValueF("redGreenWeight");
  fCV_weights[1] = &fCV_redGreenWeight;
  fCV_featureNormConst[1] = localConfig.getItemValueF("redGreenNorm");
  fCV_featureTransConst[1] = localConfig.getItemValueF("redGreenTrans");
  fCV_flickerWeight = localConfig.getItemValueF("flickerWeight");
  fCV_weights[2] = &fCV_flickerWeight;
  fCV_featureNormConst[2] = localConfig.getItemValueF("flickerNorm");
  fCV_featureTransConst[2] = localConfig.getItemValueF("flickerTrans");
  fCV_lumWeight = localConfig.getItemValueF("lumWeight");
  fCV_weights[3] = &fCV_lumWeight;
  fCV_featureNormConst[3] = localConfig.getItemValueF("lumNorm");
  fCV_featureTransConst[3] = localConfig.getItemValueF("lumTrans");
  fCV_oriWeight = localConfig.getItemValueF("oriWeight");
  fCV_oriOffset = 4;
  fCV_weights[4] = &fCV_oriWeight;
  fCV_featureNormConst[4] = localConfig.getItemValueF("oriNorm");
  fCV_featureTransConst[4] = localConfig.getItemValueF("oriTrans");
  fCV_weights[5] = &fCV_oriWeight;
  fCV_featureNormConst[5] = localConfig.getItemValueF("oriNorm");
  fCV_featureTransConst[5] = localConfig.getItemValueF("oriTrans");
  fCV_weights[6] = &fCV_oriWeight;
  fCV_featureNormConst[6] = localConfig.getItemValueF("oriNorm");
  fCV_featureTransConst[6] = localConfig.getItemValueF("oriTrans");
  fCV_weights[7] = &fCV_oriWeight;
  fCV_featureNormConst[7] = localConfig.getItemValueF("oriNorm");
  fCV_featureTransConst[7] = localConfig.getItemValueF("oriTrans");
  fCV_motionWeight = localConfig.getItemValueF("motionWeight");
  fCV_motOffset = 8;
  fCV_weights[8] = &fCV_motionWeight;
  fCV_featureNormConst[8] = localConfig.getItemValueF("motionNorm");
  fCV_featureTransConst[8] = localConfig.getItemValueF("motionTrans");
  fCV_weights[9] = &fCV_motionWeight;
  fCV_featureNormConst[9] = localConfig.getItemValueF("motionNorm");
  fCV_featureTransConst[9] = localConfig.getItemValueF("motionTrans");
  fCV_weights[10] = &fCV_motionWeight;
  fCV_featureNormConst[10] = localConfig.getItemValueF("motionNorm");
  fCV_featureTransConst[10] = localConfig.getItemValueF("motionTrans");
  fCV_weights[11] = &fCV_motionWeight;
  fCV_featureNormConst[11] = localConfig.getItemValueF("motionNorm");
  fCV_featureTransConst[11] = localConfig.getItemValueF("motionTrans");
  fCV_spatOffset = 12;
  fCV_spatialWeight = localConfig.getItemValueF("spatialWeight");
  fCV_weights[12] = &fCV_spatialWeight;
  fCV_featureNormConst[12] = localConfig.getItemValueF("spatialNorm");
  fCV_featureTransConst[12] = localConfig.getItemValueF("spatialTrans");
  fCV_mixOffset = 12;
  fCV_mixAlphaWeight = localConfig.getItemValueF("mixAlphaWeight");
  fCV_weights[13] = &fCV_mixAlphaWeight;
  fCV_featureNormConst[13] = localConfig.getItemValueF("alphaNorm");
  fCV_featureTransConst[13] = localConfig.getItemValueF("alphaTrans");
  fCV_mixBetaWeight = localConfig.getItemValueF("mixBetaWeight");
  fCV_weights[14] = &fCV_mixBetaWeight;
  fCV_featureNormConst[14] = localConfig.getItemValueF("betaNorm");
  fCV_featureTransConst[14] = localConfig.getItemValueF("betaTrans");
  fCV_mixGammaWeight = localConfig.getItemValueF("mixGammaWeight");
  fCV_weights[15] = &fCV_mixGammaWeight;
  fCV_featureNormConst[15] = localConfig.getItemValueF("gammaNorm");
  fCV_featureTransConst[15] = localConfig.getItemValueF("gammaTrans");
  // New combined motion channel weights
  fCV_motionCombinedOffset = 16;
  fCV_motionCombinedWeight = localConfig.getItemValueF("motionCombinedWeight");
  fCV_weights[16] = &fCV_motionCombinedWeight;
  fCV_featureNormConst[16] = localConfig.getItemValueF("motionCombinedNorm");
  fCV_featureTransConst[16] = localConfig.getItemValueF("motionCombinedTrans");
  // color weights
  fCV_colorOffset = 17;
  fCV_redWeight = localConfig.getItemValueF("redWeight");
  fCV_weights[17] = &fCV_redWeight;
  fCV_featureNormConst[17] = localConfig.getItemValueF("redNorm");
  fCV_featureTransConst[17] = localConfig.getItemValueF("redTrans");
  fCV_redNorm = &fCV_featureNormConst[17];
  fCV_redTrans = &fCV_featureTransConst[17];
  fCV_greenWeight = localConfig.getItemValueF("greenWeight");
  fCV_weights[18] = &fCV_greenWeight;
  fCV_featureNormConst[18] = localConfig.getItemValueF("greenNorm");
  fCV_featureTransConst[18] = localConfig.getItemValueF("greenTrans");
  fCV_greenNorm = &fCV_featureNormConst[18];
  fCV_greenTrans = &fCV_featureTransConst[18];
  fCV_blueWeight = localConfig.getItemValueF("blueWeight");
  fCV_weights[19] = &fCV_blueWeight;
  fCV_featureNormConst[19] = localConfig.getItemValueF("blueNorm");
  fCV_featureTransConst[19] = localConfig.getItemValueF("blueTrans");
  fCV_blueNorm = &fCV_featureNormConst[19];
  fCV_blueTrans = &fCV_featureTransConst[19];
  fCV_yellowWeight = localConfig.getItemValueF("yellowWeight");
  fCV_weights[20] = &fCV_yellowWeight;
  fCV_featureNormConst[20] = localConfig.getItemValueF("yellowNorm");
  fCV_featureTransConst[20] = localConfig.getItemValueF("yellowTrans");
  fCV_yellowNorm = &fCV_featureNormConst[20];
  fCV_yellowTrans = &fCV_featureTransConst[20];
  fCV_hueWeight = localConfig.getItemValueF("hueWeight");
  fCV_weights[21] = &fCV_hueWeight;
  fCV_featureNormConst[21] = localConfig.getItemValueF("hueNorm");
  fCV_featureTransConst[21] = localConfig.getItemValueF("hueTrans");
  fCV_hueNorm = &fCV_featureNormConst[21];
  fCV_hueTrans = &fCV_featureTransConst[21];
  fCV_satWeight = localConfig.getItemValueF("satWeight");
  fCV_weights[22] = &fCV_satWeight;
  fCV_featureNormConst[22] = localConfig.getItemValueF("satNorm");
  fCV_featureTransConst[22] = localConfig.getItemValueF("satTrans");
  fCV_satNorm = &fCV_featureNormConst[22];
  fCV_satTrans = &fCV_featureTransConst[22];
  fCV_valWeight = localConfig.getItemValueF("valWeight");
  fCV_weights[23] = &fCV_valWeight;
  fCV_featureNormConst[23] = localConfig.getItemValueF("valNorm");
  fCV_featureTransConst[23] = localConfig.getItemValueF("valTrans");
  fCV_valNorm = &fCV_featureNormConst[23];
  fCV_valTrans = &fCV_featureTransConst[23];
  fCV_hue1Weight = localConfig.getItemValueF("hue1Weight");
  fCV_weights[24] = &fCV_hue1Weight;
  fCV_featureNormConst[24] = localConfig.getItemValueF("hue1Norm");
  fCV_featureTransConst[24] = localConfig.getItemValueF("hue1Trans");
  fCV_hue1Norm = &fCV_featureNormConst[24];
  fCV_hue1Trans = &fCV_featureTransConst[21];
  fCV_hue2Weight = localConfig.getItemValueF("hue2Weight");
  fCV_weights[25] = &fCV_hue2Weight;
  fCV_featureNormConst[25] = localConfig.getItemValueF("hue2Norm");
  fCV_featureTransConst[25] = localConfig.getItemValueF("hue2Trans");
  fCV_hue2Norm = &fCV_featureNormConst[25];
  fCV_hue2Trans = &fCV_featureTransConst[21];

  std::cerr << "(10) loading feature ICA PCA reduced sizes\n";
  fCV_ICAfeaturesRedGreen =
    (int)localConfig.getItemValueF("ICAfeaturesRedGreen");
  fCV_ICAfeaturesPerChannel[0] = &fCV_ICAfeaturesRedGreen;
  fCV_ICAfeaturesBlueYellow =
    (int)localConfig.getItemValueF("ICAfeaturesBlueYellow");
  fCV_ICAfeaturesPerChannel[1] = &fCV_ICAfeaturesBlueYellow;
  fCV_ICAfeaturesFlicker =
    (int)localConfig.getItemValueF("ICAfeaturesFlicker");
  fCV_ICAfeaturesPerChannel[2] = &fCV_ICAfeaturesFlicker;
  fCV_ICAfeaturesLum =
    (int)localConfig.getItemValueF("ICAfeaturesLum");
  fCV_ICAfeaturesPerChannel[3] = &fCV_ICAfeaturesLum;
  fCV_ICAfeaturesOri =
    (int)localConfig.getItemValueF("ICAfeaturesOri");
  fCV_ICAfeaturesPerChannel[4] = &fCV_ICAfeaturesOri;
  fCV_ICAfeaturesPerChannel[5] = &fCV_ICAfeaturesOri;
  fCV_ICAfeaturesPerChannel[6] = &fCV_ICAfeaturesOri;
  fCV_ICAfeaturesPerChannel[7] = &fCV_ICAfeaturesOri;
  fCV_ICAfeaturesMotion =
    (int)localConfig.getItemValueF("ICAfeaturesMotion");
  fCV_ICAfeaturesPerChannel[8] = &fCV_ICAfeaturesMotion;
  fCV_ICAfeaturesPerChannel[9] = &fCV_ICAfeaturesMotion;
  fCV_ICAfeaturesPerChannel[10] = &fCV_ICAfeaturesMotion;
  fCV_ICAfeaturesPerChannel[11] = &fCV_ICAfeaturesMotion;
  fCV_ICAfeaturesSpatial =
    (int)localConfig.getItemValueF("ICAfeaturesSpatial");
  fCV_ICAfeaturesPerChannel[12] = &fCV_ICAfeaturesSpatial;
  fCV_ICAfeaturesAlpha =
    (int)localConfig.getItemValueF("ICAfeaturesAlpha");
  fCV_ICAfeaturesPerChannel[13] = &fCV_ICAfeaturesAlpha;
  fCV_ICAfeaturesBeta =
    (int)localConfig.getItemValueF("ICAfeaturesBeta");
  fCV_ICAfeaturesPerChannel[14] = &fCV_ICAfeaturesBeta;
  fCV_ICAfeaturesGamma =
    (int)localConfig.getItemValueF("ICAfeaturesGamma");
  fCV_ICAfeaturesPerChannel[15] = &fCV_ICAfeaturesGamma;
  // New combined motion channel ICA
  fCV_ICAfeaturesMotionCombined =
    (int)localConfig.getItemValueF("ICAfeaturesMotionCombined");
  fCV_ICAfeaturesPerChannel[16] = &fCV_ICAfeaturesMotionCombined;
  // color ICA
  fCV_ICAfeaturesRed = (int)localConfig.getItemValueF("ICAfeaturesRed");
  fCV_ICAfeaturesPerChannel[17] = &fCV_ICAfeaturesRed;
  fCV_ICAfeaturesGreen = (int)localConfig.getItemValueF("ICAfeaturesGreen");
  fCV_ICAfeaturesPerChannel[18] = &fCV_ICAfeaturesGreen;
  fCV_ICAfeaturesBlue = (int)localConfig.getItemValueF("ICAfeaturesBlue");
  fCV_ICAfeaturesPerChannel[19] = &fCV_ICAfeaturesBlue;
  fCV_ICAfeaturesYellow = (int)localConfig.getItemValueF("ICAfeaturesYellow");
  fCV_ICAfeaturesPerChannel[20] = &fCV_ICAfeaturesYellow;
  fCV_ICAfeaturesHue = (int)localConfig.getItemValueF("ICAfeaturesHue");
  fCV_ICAfeaturesPerChannel[21] = &fCV_ICAfeaturesHue;
  fCV_ICAfeaturesSat = (int)localConfig.getItemValueF("ICAfeaturesSat");
  fCV_ICAfeaturesPerChannel[22] = &fCV_ICAfeaturesSat;
  fCV_ICAfeaturesVal = (int)localConfig.getItemValueF("ICAfeaturesVal");
  fCV_ICAfeaturesPerChannel[23] = &fCV_ICAfeaturesVal;
  fCV_ICAfeaturesHue1 = (int)localConfig.getItemValueF("ICAfeaturesHue1");
  fCV_ICAfeaturesPerChannel[24] = &fCV_ICAfeaturesHue1;
  fCV_ICAfeaturesHue2 = (int)localConfig.getItemValueF("ICAfeaturesHue2");
  fCV_ICAfeaturesPerChannel[25] = &fCV_ICAfeaturesHue2;
  std::cerr << "(10) Resizing space for cluster\n";
  std::vector<bool*>::iterator featureOn_itr;

  int fCount = 0;
  int c = 0;
  for(featureOn_itr = fCV_featureOn.begin();
      featureOn_itr != fCV_featureOn.end();
      ++featureOn_itr)
  {
    if(**featureOn_itr == true)
    {
      fCount = fCount + *fCV_ICAfeaturesPerChannel[c];
      std::cerr << "FEATURES " << *fCV_ICAfeaturesPerChannel[c] << "\n";
    }
    c++;
  }

  fCV_newMatSize = fCount;
  fCV_featureNameICA.resize(secretVariable,&fCV_NULLstring);

  fCVresizeMaps2(fCV_sparcePoints,fCV_newMatSize);
  fCV_NPtemporalBias = localConfig.getItemValueF("NPtemporalBias");
  fCV_doNPbias       = false;
  fCV_saliencyExp    = localConfig.getItemValueF("saliencyExp");
  fCV_lowPassType    = (int)localConfig.getItemValueF("lowPassType");
  fCV_densityBias    = localConfig.getItemValueF("densityBias");
  fCV_useTimerFile   = localConfig.getItemValueB("useTimerFile");
  fCV_salmapLowPassTemporalBias =
    localConfig.getItemValueF("salmapLowPassTemporalBias");
  //fCV_useCovHolderUp = true;
  fCV_doMatchSelf = true;
  fCV_doSLPTB = false;

  //get the low pass kernel
  fCV_lowPassKernelName = localConfig.getItemValueS("lowPassKernelName");
  readMatrix kernelMat(fCV_lowPassKernelName.c_str());
  fCV_lowPassKernel = kernelMat.returnMatrixAsImage();

  fCV_gaborStandardDev = localConfig.getItemValueF("gaborStandardDev");
  fCV_gaborPeriod      = localConfig.getItemValueF("gaborPeriod");
  fCV_gaborScales = (unsigned int)localConfig.getItemValueF("gaborScales");
  fCV_gaborUseQuarter  = localConfig.getItemValueB("gaborUseQuarter");
  fCVcreateGaborFilters();
}

// ######################################################################
template <class FLOAT>
featureClusterVision<FLOAT>::~featureClusterVision()
{}

// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVresizeMaps1(int sparcePoints)
{
  FLOAT ZERO = 0.0F;
  std::cerr << "MAP RESIZE " << sparcePoints << "\n";
  std::cerr << "... fmap resize........\t" << fCV_totalFeatures << " x "
            << (sparcePoints+OVER) << "\n";
  //std::vector<FLOAT> tempVecDp;
  std::vector<FLOAT> tempVecD2(0,0.0F);
  fCV_fmap.resize(sparcePoints+OVER,tempVecD2);

  std::cerr << "... unMixedMap resize..\t" << (sparcePoints+OVER) << " x "
            << fCV_totalFeatures << "\n";
  std::vector<FLOAT*> tempVecPP(sparcePoints+OVER,(&ZERO));
  fCV_unmixedMap.resize(fCV_totalFeatures,tempVecPP);
  PixRGB<FLOAT> pixey;
  fCV_lowPassVector.resize(sparcePoints,pixey);

}

// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVresizeMaps2(int sparcePoints, int newMatSize)
{
  FLOAT ZERO = 0.0F;
  std::cerr << "... space resize.......\t" << newMatSize << " x "
            << (sparcePoints+OVER) << "\n";
  typename std::vector<FLOAT> tempVecD(newMatSize,0.0F);
  fCV_space.resize(sparcePoints+OVER,tempVecD);
  std::cerr << "(X) feature Cluster Vision Object Ready!\n";
  fCV_NP.NPresizeSpace((sparcePoints+OVER),newMatSize);
  fCV_CV.resize(newMatSize,sparcePoints+OVER,0.0F);

  typename std::vector<FLOAT> tempMix(sparcePoints+OVER,0.0F);
  fCV_mixedRotation.resize(3*fCV_featuresPerChannel,tempMix);
  fCV_mixedMotion.resize(3*fCV_featuresPerChannel,tempMix);
  std::cerr << "... sortedSpace resize.\t" << newMatSize << " x "
            << (sparcePoints+OVER) << " x "
            << (sparcePoints+OVER) << "\n";
  typename std::vector<FLOAT*> tempred2((sparcePoints+OVER),&ZERO);
  typename std::vector<std::vector<FLOAT*> >
    ntempred2(newMatSize,tempred2);
  fCV_sortedSpace.resize(sparcePoints+OVER,ntempred2);
  fCV_tcov.resize(newMatSize,sparcePoints+OVER,0.0F);
  fCV_tcov.baseID = 0;
  fCV_tcov.sortID = 0;
  fCV_tcov.matchID = 0;
  fCV_tcov.isMatched = false;
  fCV_tcov.isLarge = false;
  //fCV_tcov.featureName = &fCV_featureNameICA;

  LINFO("Setting up all tcov");
  int j = 0;
  for(unsigned int i = 0; i < fCV_weights.size(); i++)
  {
    if(*fCV_featureOn[i] == true)
    {
      if((signed)i != fCV_spatOffset)
      for(int k = 0; k < *fCV_ICAfeaturesPerChannel[i]; k++)
      {
        fCV_tcov.bias[j] = *fCV_weights[i];
        fCV_tcov.norm[j] = fCV_featureNormConst[i];
        fCV_tcov.trans[j] = fCV_featureTransConst[i];
        fCV_tcov.featureName[j] = fCV_featureName[i];
        j++;
      }
    }
  }
  LINFO("Setting tcov spatial labels");
  fCV_tcov.featureName[fCV_tcov.featureName.size() - 2] = "Spatial X";
  fCV_tcov.featureName[fCV_tcov.featureName.size() - 1] = "Spatial Y";

  LINFO("Setting tcov spatial matching");
  std::vector<covHolder<double> > covtemp;
  std::cerr << ".";
  covtemp.resize(COVSIZE,fCV_tcov);
  std::cerr << ".";
  fCV_covHolderMatch.resize(COVSIZE,fCV_tcov);
  std::cerr << ".";
  fCV_covHolder.resize(COVHOLDER,covtemp);
  std::cerr << ".";
  fCV_covDataSizeMatch = 0;
  std::cerr << ".\n";
  LINFO("MAP RESIZED 2");
  //fCV_covHolderUp.resize(COVSIZE,fCV_tcov);
  //fCV_covHolderDown.resize(COVSIZE,fCV_tcov);

}

// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVcreateGaborFilters()
{
  const float gaborMasks[4]   = {0.0F,45.0F,90.0F,135.0F};
  const FLOAT gaborSTD        = fCV_gaborStandardDev;
  const FLOAT gaborPrd        = fCV_gaborPeriod;
  Image<float> foo;
  fCV_gaborFiltersSin.resize(4,foo);
  fCV_gaborFiltersCos.resize(4,foo);

  unsigned int iii = 0;
  for(typename std::vector<Image<FLOAT> >::iterator igaborFilters
        = fCV_gaborFiltersSin.begin();
      igaborFilters != fCV_gaborFiltersSin.end(); ++igaborFilters, iii++)
  {
    *igaborFilters = gaborFilter<FLOAT>(gaborSTD,gaborPrd,0.0F
                                         ,gaborMasks[iii]);
    //Raster::VisuFloat(*igaborFilters,FLOAT_NORM_0_255,sformat("image.gabor.sin.%d.pgm",iii));
  }

  iii = 0;
  for(typename std::vector<Image<FLOAT> >::iterator igaborFilters
        = fCV_gaborFiltersCos.begin();
      igaborFilters != fCV_gaborFiltersCos.end(); ++igaborFilters, iii++)
  {
    *igaborFilters = gaborFilter<FLOAT>(gaborSTD,gaborPrd,45.0F
                                         ,gaborMasks[iii]);
    //Raster::VisuFloat(*igaborFilters,FLOAT_NORM_0_255,sformat("image.gabor.cos.%d.pgm",iii));
  }
}

// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVuploadImage(Image<PixRGB<byte> > &input,
                                                 std::string fileName)
{
  fCV_realImage = input;
  fCV_doReal = true;
  fCV_fileName = fileName;
  LINFO("HEIGHT %d WIDTH %d",fCV_realImage.getHeight(),
        fCV_realImage.getWidth());
  if(fCV_realImage.getHeight() < fCV_realImage.getWidth())
  {
    fCV_sizeXbias = 1.0F;
    fCV_sizeYbias = (FLOAT)fCV_realImage.getHeight()/
      (FLOAT)fCV_realImage.getWidth();
  }
  else
  {
    fCV_sizeXbias = (FLOAT)fCV_realImage.getWidth()/
      (FLOAT)fCV_realImage.getHeight();
    fCV_sizeYbias = 1.0F;
  }

}

// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVmixChannels(typename
                                       std::vector<std::vector<FLOAT> > *data,
                                       int ch1a, int ch1b,
                                       int ch2a, int ch2b,
                                       typename std::vector<FLOAT> *outAlpha,
                                       typename std::vector<FLOAT> *outBeta,
                                       typename std::vector<FLOAT> *outGamma,
                                       FLOAT norm1, FLOAT norm2,
                                       int size = 0)
{
  //LINFO("%d %d",outAlpha->size(),data->size());
  ASSERT((outAlpha->size() >= data->size()) && "Vector must be larger");
  ASSERT((outBeta->size() >= data->size()) && "Vector must be larger");
  ASSERT((outGamma->size() >= data->size()) && "Vector must be larger");
  //LINFO("MIXING CHANNELS");

  typename std::vector<FLOAT>::iterator iOutA;
  typename std::vector<FLOAT>::iterator iOutB;
  typename std::vector<FLOAT>::iterator iOutG;
  iOutA = outAlpha->begin();
  iOutB = outBeta->begin();
  iOutG = outGamma->begin();
  FLOAT c1mix,c2mix,c1Gmix,c2Gmix;
  if(size == 0)
    size = (signed)data->size();
  for(long i = 0; i < size; i++,
        ++iOutA, ++iOutB, ++iOutG)
  {
    //LINFO("%d",i);
    // compute lineyness at location (strength of line)
    c1mix = fabs((data->at(i)[ch1a])-(data->at(i)[ch1b]))/(norm1);
    c2mix = fabs((data->at(i)[ch2a])-(data->at(i)[ch2b]))/(norm2);
    // compute crossyness at location (strength of junction)
    c1Gmix = (((data->at(i)[ch1a])+(data->at(i)[ch1b]))/(norm1));
    c2Gmix = (((data->at(i)[ch2a])+(data->at(i)[ch2b]))/(norm2));
    // Indicates general intensity of the output
    *iOutA = (c1Gmix+c2Gmix)/2;
    //LINFO("ALPHA %f", *iOutA);
    // Finish computing crossyness by subtracting lineyness
    // indicates the crispness of lines
    *iOutB = fabs(c1mix-c2mix)/2;
    //c1Gmix = c1mix - c1Gmix;
    //c2Gmix = c2mix - c2Gmix;
    //LINFO("BETA %f", *iOutB);
    // indicates the crispness of junctions
    *iOutG = *iOutA - *iOutB;
    //LINFO("GAMMA %f",*iOutG);
  }
}

// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVmixChannels(Image<FLOAT> &img0,
                                              Image<FLOAT> &img45,
                                              Image<FLOAT> &img90,
                                              Image<FLOAT> &img135,
                                              Image<FLOAT> *Alpha,
                                              Image<FLOAT> *Beta,
                                              Image<FLOAT> *Gamma)
{
  FLOAT min0,max0,min45,max45,min90,max90,min135,max135;

  getMinMax(img0, min0,max0);    getMinMax(img45, min45,max45);
  getMinMax(img90, min90,max90); getMinMax(img135, min135,max135);

  FLOAT norm1 = max0  + max90;
  FLOAT norm2 = max45 + max135;

  // compute lineyness at this location
  Image<FLOAT> mix1  = abs(img0  - img90) /norm1;
  Image<FLOAT> mix2  = abs(img45 - img135)/norm2;

  // compute crossyness at this location
  Image<FLOAT> mixG1 = (img0  + img90) /norm1;
  Image<FLOAT> mixG2 = (img45 + img135)/norm1;

  // Average crossyness
  *Alpha = (mixG1 + mixG2)/2;

  // Average Lineyness
  *Beta  = abs(mix1 - mix2)/2;

  // Crispness of junctions
  *Gamma = *Alpha - *Beta;
}


// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVfindMixedChannels()
{
  if(fCV_spatialOn == true)
  {
    //LINFO("MIXING ORIENTATIONS");
    for(int i = 0; i < fCV_featuresPerChannel; i++)
    {
      fCVmixChannels(&fCV_fmap,((fCV_oriOffset*fCV_featuresPerChannel)+i),
                     ((fCV_oriOffset*fCV_featuresPerChannel)+
                      (2*fCV_featuresPerChannel)+i),
                     ((fCV_oriOffset*fCV_featuresPerChannel)+
                      (1*fCV_featuresPerChannel)+i),
                     ((fCV_oriOffset*fCV_featuresPerChannel)+
                      (3*fCV_featuresPerChannel)+i),
                     &fCV_mixedRotation[i],
                     &fCV_mixedRotation[i+fCV_featuresPerChannel],
                     &fCV_mixedRotation[i+fCV_featuresPerChannel*2],
                     fCV_maxOriVal,fCV_maxOriVal,fCV_countSM);
    }
  }

  if(fCV_motionOn == true)
  {
    //LINFO("MIXING MOTIONS");
    for(int i = 0; i < fCV_featuresPerChannel; i++)
    {
      fCVmixChannels(&fCV_fmap,(fCV_motOffset*fCV_featuresPerChannel+i),
                     (fCV_motOffset*fCV_featuresPerChannel+
                      (2*fCV_featuresPerChannel)+i),
                     (fCV_motOffset*fCV_featuresPerChannel+
                      (fCV_featuresPerChannel)+i),
                     (fCV_motOffset*fCV_featuresPerChannel+
                      (3*fCV_featuresPerChannel)+i),
                     &fCV_mixedMotion[i],
                     &fCV_mixedMotion[i+fCV_featuresPerChannel],
                     &fCV_mixedMotion[i+fCV_featuresPerChannel*2],
                     fCV_maxMotVal,fCV_maxMotVal,fCV_countSM);
    }
  }
}

// ######################################################################
// ######################################################################
// TEST METHODS
// ######################################################################
// ######################################################################

#if 0
// FIXME These functions doesn't compile cleanly anymore because
// VisualCortex::getFeatures() expects a std::vector<double>, but
// we're trying to pass it a std::vector<float>.
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVcheckMixing()
{
  ASSERT(fCV_useBrain == true);
  int mod = 0;
  if(fCV_spatialOn == true)
    mod = 2;
  //---------------Get Saliency Data---------------//
  std::cerr << "(1) GETTING saliency data \n";
  // get the current saliency map see Brain.C for examples
  /////fCV_SM = fCV_brain->getSM();
  /////fCV_VC = fCV_brain->getVC();
  Image<FLOAT> salmap;///////// = fCV_SM->getV(false);
  LFATAL("FIXME!!!!");

  salmap = rescaleBilinear(salmap,fCV_sizeX,fCV_sizeY);
  std::cerr << "...model objects fetched\n";

  typename std::vector<FLOAT> tempVecD2(0,0.0F);
  fCV_fmap.resize(0,tempVecD2);
  fCV_fmap.resize((salmap.getWidth()*salmap.getHeight()),tempVecD2);
  typename std::vector<FLOAT>
    tempMix((salmap.getWidth()*salmap.getHeight()),0.0F);
  fCV_mixedRotation.resize(0,tempMix);
  fCV_mixedMotion.resize(0,tempMix);
  fCV_mixedRotation.resize(3*fCV_featuresPerChannel,tempMix);
  fCV_mixedMotion.resize(3*fCV_featuresPerChannel,tempMix);

  // get sal values for each point into a vector fmap
  for(int i = 0; i < salmap.getWidth(); i++)
  {
    for(int j = 0; j < salmap.getHeight(); j++)
    {
      //void getFeatures(const Point2D<int>& locn, std::vector<FLOAT>& mean) const;
      // see VisualCortex.*
      //LINFO("SETTING %d",(j*salmap.getWidth())+i);
      fCV_VC->getFeatures(Point2D<int>(i,j),fCV_fmap[(j*salmap.getWidth())+i]);
    }
  }
  fCV_countSM = fCV_fmap.size();
  fCVfindMixedChannels();

  Image<FLOAT> outImage1;
  Image<FLOAT> outImage2;
  Image<FLOAT> outImage3;
  Image<FLOAT> ICAoutImage1;
  Image<FLOAT> ICAoutImage2;
  Image<FLOAT> ICAoutImage3;

  outImage1.resize(salmap.getWidth(),salmap.getHeight());
  outImage2.resize(salmap.getWidth(),salmap.getHeight());
  outImage3.resize(salmap.getWidth(),salmap.getHeight());
  ICAoutImage1.resize(salmap.getWidth(),salmap.getHeight());
  ICAoutImage2.resize(salmap.getWidth(),salmap.getHeight());
  ICAoutImage3.resize(salmap.getWidth(),salmap.getHeight());

  // check ICA on mixed channels
  Image<FLOAT> mmmap;
  Image<FLOAT> mmap;
  Image<FLOAT> foosh;
  mmmap = fCV_mixedRotation;
  mmap = mmmap;
  mmap = transpose(mmap);
  Image<FLOAT> ICAoutput1;
  Image<FLOAT> ICAoutput2;
  Image<FLOAT> ICAoutput3;
  unsigned int sm = (unsigned)fCV_countSM;
  unsigned int zero = 0;

  unsigned int st = 0;
  unsigned int sp = st + (fCV_featuresPerChannel-1);
  unsigned int width = fCV_ICAunmix[fCV_mixOffset+1].getWidth();
  ICAoutput1 = matrixMult(mmap,fCV_ICAunmix[fCV_mixOffset+1],st,sp,
                      zero,width,zero,sm);
  //foosh = ICAoutput1;
  //Raster::VisuFloat(foosh,FLOAT_NORM_0_255,"foo.pgm");
  st = sp + 1;
  sp = st + (fCV_featuresPerChannel-1);
  width = fCV_ICAunmix[fCV_mixOffset+2].getWidth();
  ICAoutput2 = matrixMult(mmap,fCV_ICAunmix[fCV_mixOffset+1],st,sp,
                      zero,width,zero,sm);

  st = sp + 1;
  sp = st + (fCV_featuresPerChannel-1);
  width = fCV_ICAunmix[fCV_mixOffset+3].getWidth();
  ICAoutput3 = matrixMult(mmap,fCV_ICAunmix[fCV_mixOffset+1],st,sp,
                      zero,width,zero,sm);

  for(int x = 0; x < ICAoutput1.getWidth(); x++)
  {
    for(int i = 0; i < salmap.getWidth(); i++)
    {
      for(int j = 0; j < salmap.getHeight(); j++)
      {
        ICAoutImage1.setVal(i,j,(ICAoutput1.getVal(x,
                                                   (j*salmap.getWidth())+i)));
      }
    }
    Raster::VisuFloat(ICAoutImage1,FLOAT_NORM_0_255,
                      sformat("%s.out.ImageA.ICA.%d.pgm",
                              fCV_fileName.c_str(),x));
  }

  for(int x = 0; x < ICAoutput2.getWidth(); x++)
  {
    for(int i = 0; i < salmap.getWidth(); i++)
    {
      for(int j = 0; j < salmap.getHeight(); j++)
      {
        ICAoutImage2.setVal(i,j,(ICAoutput2.getVal(x,
                                                   (j*salmap.getWidth())+i)));
      }
    }
    Raster::VisuFloat(ICAoutImage2,FLOAT_NORM_0_255,
                      sformat("%s.out.ImageB.ICA.%d.pgm",
                              fCV_fileName.c_str(),x));
  }

  for(int x = 0; x < ICAoutput3.getWidth(); x++)
  {
    for(int i = 0; i < salmap.getWidth(); i++)
    {
      for(int j = 0; j < salmap.getHeight(); j++)
      {
        ICAoutImage3.setVal(i,j,(ICAoutput3.getVal(x,
                                                   (j*salmap.getWidth())+i)));
      }
    }
    Raster::VisuFloat(ICAoutImage3,FLOAT_NORM_0_255,
                      sformat("%s.out.ImageG.ICA.%d.pgm",
                              fCV_fileName.c_str(),x));
  }

  for(int x = 0; x < fCV_featuresPerChannel; x++)
  {
    for(int i = 0; i < salmap.getWidth(); i++)
    {
      for(int j = 0; j < salmap.getHeight(); j++)
      {
        outImage1.setVal(i,j,
                         fCV_mixedRotation[x][(j*salmap.getWidth())+i]);
        outImage2.setVal(i,j,
                         fCV_mixedRotation[x+fCV_featuresPerChannel]
                         [(j*salmap.getWidth())+i]);
        outImage3.setVal(i,j,
                         fCV_mixedRotation[x+fCV_featuresPerChannel*2]
                         [(j*salmap.getWidth())+i]);
      }
    }

    //Raster::VisuRGB(outputClass,"CLASS_%d.pnm",i);
    Raster::VisuFloat(outImage1,FLOAT_NORM_0_255,
                      sformat("%s.out.ImageA.%d.pgm",
                              fCV_fileName.c_str(),x));
    Raster::VisuFloat(outImage2,FLOAT_NORM_0_255,
                      sformat("%s.out.ImageB.%d.pgm",
                              fCV_fileName.c_str(),x));
    Raster::VisuFloat(outImage3,FLOAT_NORM_0_255,
                      sformat("%s.out.ImageG.%d.pgm",
                              fCV_fileName.c_str(),x));
  }
}

// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVcheckICA()
{
  for(int i = 0; i < (fCV_totalPotFeatures/fCV_featuresPerChannel); i++)
  {
    LINFO("CHECKING CHANNEL %d",i);
    if(*fCV_featureOn[i] == true)
    {
      LINFO("CHANNEL %d ON",i);
      fCVcheckICA(i,true);
    }
  }
}

// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVcheckICA(int channel, bool findMixed)
{
  ASSERT(fCV_useBrain == true);
  int mod = 0;
  if(fCV_spatialOn == true)
    mod = 2;

  //---------------Get Saliency Data---------------//
  std::cerr << "(1) GETTING saliency data \n";
  // get the current saliency map see Brain.C for examples
  ///////fCV_SM = fCV_brain->getSM();
  //////fCV_VC = fCV_brain->getVC();
  Image<FLOAT> salmap;//////////////// = fCV_SM->getV(false);
  LFATAL("FIXME");

  salmap = rescaleBilinear(salmap,fCV_sizeX,fCV_sizeY);
  std::cerr << "...model objects fetched\n";
  typename std::vector<FLOAT> tempVecD2(0,0.0F);
  fCV_fmap.resize(0,tempVecD2);
  fCV_fmap.resize((salmap.getWidth()*salmap.getHeight()),tempVecD2);
  LINFO("MEMORY ALLOCATED for map %d x %d",
        (salmap.getWidth()*salmap.getHeight()),0);

  typename std::vector<FLOAT>
    tempMix((salmap.getWidth()*salmap.getHeight()),0.0F);
  fCV_mixedRotation.resize(0,tempMix);
  fCV_mixedMotion.resize(0,tempMix);
  fCV_mixedRotation.resize(3*fCV_featuresPerChannel,tempMix);
  fCV_mixedMotion.resize(3*fCV_featuresPerChannel,tempMix);


  // get sal values for each point into a vector fmap
  for(int i = 0; i < salmap.getWidth(); i++)
  {
    for(int j = 0; j < salmap.getHeight(); j++)
    {
      //void getFeatures(const Point2D<int>& locn, std::vector<FLOAT>& mean) const;
      // see VisualCortex.*
      //LINFO("SETTING %d",(j*salmap.getWidth())+i);
      fCV_VC->getFeatures(Point2D<int>(i,j),fCV_fmap[(j*salmap.getWidth())+i]);
    }
  }
  LINFO("FEATURES fetched");
  fCV_countSM = fCV_fmap.size();
  if(findMixed == true)
    fCVfindMixedChannels();
  Image<FLOAT> ifmap;
  ifmap = fCV_fmap;
  unsigned long zero = 0;
  unsigned long pStart = fCV_featuresPerChannel*channel;
  unsigned long pStop = pStart+fCV_featuresPerChannel-1;
  unsigned long width = fCV_ICAunmix[channel].getWidth();
  unsigned long sm = fCV_countSM;
  Image<FLOAT> unmixedImage;
  unmixedImage.resize(fCV_ICAunmix[channel].getWidth(),ifmap.getHeight());
  unmixedImage = matrixMult(ifmap,fCV_ICAunmix[channel],pStart,pStop,
                            zero,width,zero,sm);
  Image<FLOAT> outImage;
  outImage.resize(salmap.getWidth(),salmap.getHeight());
  LINFO("WRITTING images");
  for(int i = 0; i < unmixedImage.getWidth(); i++)
  {
    LINFO("Image %d",i);
    for(int j = 0; j < unmixedImage.getHeight(); j++)
    {
      int posX = j%outImage.getWidth();
      int posY = j/outImage.getWidth();
      outImage.setVal(posX,posY,unmixedImage.getVal(i,j));
    }
    Raster::VisuFloat(outImage,FLOAT_NORM_0_255
                      ,sformat("out.ICAresponse.%d.%d.pgm",channel,i));
    LINFO("DONE");
  }
}

// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVcheckMotionCombined(long frame)
{
  ASSERT(fCV_useBrain == true);
  char c[100];
  if(frame < 10)
    sprintf(c,"00000%d",(int)frame);
  else if(frame < 100)
    sprintf(c,"0000%d",(int)frame);
  else if(frame < 1000)
    sprintf(c,"000%d",(int)frame);
  else if(frame < 10000)
    sprintf(c,"00%d",(int)frame);
  else if(frame < 100000)
    sprintf(c,"0%d",(int)frame);
  else
    sprintf(c,"%d",(int)frame);

  int mod = 0;
  if(fCV_spatialOn == true)
    mod = 2;

  //---------------Get Saliency Data---------------//
  std::cerr << "(1) GETTING saliency data \n";
  // get the current saliency map see Brain.C for examples
  ///////fCV_SM = fCV_brain->getSM();
  //////fCV_VC = fCV_brain->getVC();
  Image<FLOAT> salmap;/////////////// = fCV_SM->getV(false);
  LFATAL("FIXME!");

  salmap = rescaleBilinear(salmap,fCV_sizeX,fCV_sizeY);
  std::cerr << "...model objects fetched\n";
  typename std::vector<FLOAT> tempVecD2(0,0.0F);
  fCV_fmap.resize(0,tempVecD2);
  fCV_fmap.resize((salmap.getWidth()*salmap.getHeight()),tempVecD2);
  LINFO("MEMORY ALLOCATED for map %d x %d",
        (salmap.getWidth()*salmap.getHeight()),0);

  // get sal values for each point into a vector fmap
  for(int i = 0; i < salmap.getWidth(); i++)
  {
    for(int j = 0; j < salmap.getHeight(); j++)
    {
      //void getFeatures(const Point2D<int>& locn, std::vector<FLOAT>& mean) const;
      // see VisualCortex.*
      //LINFO("SETTING %d",(j*salmap.getWidth())+i);
      fCV_VC->getFeatures(Point2D<int>(i,j),fCV_fmap[(j*salmap.getWidth())+i]);
    }
  }
  LINFO("FEATURES fetched");
  fCV_countSM = fCV_fmap.size();

  Image<FLOAT> ifmap;
  ifmap = fCV_fmap;
  unsigned long zero = 0;
  unsigned long pStart = fCV_motOffset*fCV_featuresPerChannel;
  unsigned long pStop = pStart+((fCV_featuresPerChannel*4)-1);
  unsigned long width = fCV_ICAunmix[15].getWidth();
  unsigned long sm = fCV_countSM;

  Image<FLOAT> unmixedImage;
  unmixedImage.resize(fCV_ICAunmix[15].getWidth(),ifmap.getHeight());
  LINFO("RUNNING for Matrix Size %d x %d at position %d to %d",
        fCV_ICAunmix[15].getWidth(),fCV_ICAunmix[15].getHeight(),
        pStart,pStop);
  Image<FLOAT> shit;
  shit = fCV_ICAunmix[15];
  std::cerr << "filter\n";
  for(int i = 0; i < fCV_ICAunmix[15].getWidth(); i++)
  {
      for(int j = 0; j < fCV_ICAunmix[15].getHeight(); j++)
      {
        std::cerr << fCV_ICAunmix[15].getVal(i,j) << " ";
      }
      std::cerr << "\n";
  }
  //shit = ifmap;
  //Raster::VisuFloat(shit,FLOAT_NORM_0_255,"InputFeatures.pgm");
  unmixedImage = matrixMult(ifmap,fCV_ICAunmix[15],pStart,pStop,
                            zero,width,zero,sm);
  Image<PixRGB<FLOAT> > outImage;
  Image<PixRGB<FLOAT> > outImage2;
  Image<FLOAT> storeImage;
  Image<FLOAT> storeImage2;
  outImage.resize(salmap.getWidth(),salmap.getHeight());
  outImage2.resize(salmap.getWidth()*3,salmap.getHeight()*2);
  // TOTAL FUDGE REMOVE IF WANTED //
  storeImage.resize(salmap.getWidth(),salmap.getHeight());
  storeImage2.resize(salmap.getWidth()*3,salmap.getHeight()*2);

  LINFO("WRITTING images");
  int XX = 0;
  int YY = 0;

  for(int i = 0; i < unmixedImage.getWidth(); i++)
  {
    LINFO("Image %d",i);
    if(i == 0){ XX = 0; YY = 0;}
    if(i == 1){ XX = 1; YY = 0;}
    if(i == 2){ XX = 2; YY = 0;}
    if(i == 3){ XX = 0; YY = 1;}
    if(i == 4){ XX = 1; YY = 1;}
    if(i == 5){ XX = 2; YY = 1;}
    for(int j = 0; j < unmixedImage.getHeight(); j++)
    {
      int posX = j%outImage.getWidth();
      int posY = j/outImage.getWidth();
      //LINFO("%f",unmixedImage.getVal(i,j));
      storeImage.setVal(posX,posY,unmixedImage.getVal(i,j));
      storeImage2.setVal(posX+(XX*outImage.getWidth())
                         ,posY+(YY*outImage.getHeight())
                         ,unmixedImage.getVal(i,j));
    }
    outImage = normalizeRGPolar(storeImage,255.0,255.0);
    outImage2 = normalizeRGPolar(storeImage2,255.0,255.0);
    Raster::WriteRGB(outImage,sformat("out.motionCombined.%d.%s.ppm",i,c));

    LINFO("DONE");
  }
  Raster::WriteRGB(outImage2,sformat("out.motionCombinedAll.%s.ppm",c));
}
#endif


// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVfeaturesToFile(std::string fileName,
                                                 bool _new = false)
{
  if(_new == false)
  {
    fCVfindFeaturesBrain();
    fCVfindMixedChannels();
  }
  std::string newFile;
  newFile = fileName + ".out.features";
  std::ofstream outfile(newFile.c_str(),std::ios::app);
  typename std::vector<std::vector<FLOAT> >::iterator iFmap;
  typename std::vector<FLOAT>::iterator iiFmap;
  iFmap = fCV_fmap.begin();
  int i = 0;
  while(iFmap != fCV_fmap.end())
  {
    for(iiFmap = iFmap->begin(); iiFmap != iFmap->end(); ++iiFmap)
      outfile << *iiFmap << "\t";

    for(int x = 0; x < fCV_featuresPerChannel; x++)
      outfile << fCV_mixedRotation[x][i] << "\t";

    for(int x = 0; x < fCV_featuresPerChannel; x++)
      outfile << fCV_mixedRotation[x+fCV_featuresPerChannel][i] << "\t";

    for(int x = 0; x < fCV_featuresPerChannel; x++)
      outfile << fCV_mixedRotation[x+fCV_featuresPerChannel*2][i] << "\t";

    outfile << "\n";
    ++iFmap; i++;
  }
  outfile.close();
}

// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVrunStandAloneMSBatchFilter(
                                  std::string filename)
{

  Timer tim;
  tim.reset();
  int t1,t2,t3;
  int t0 = tim.get();  // to measure display time
  t1     = 0;
  t2     = t1 - t0;

  const unsigned int scales   = fCV_gaborScales;
  const bool useQuarter       = fCV_gaborUseQuarter;

  std::vector<PixH2SV2<FLOAT> > samples(fCV_countSM);

  // resize (scales * HSV2 * angles) + junctions;
  fCV_standAloneFeatures.resize((scales*4),samples);
  fCV_standAloneFeaturesSin.resize((scales*4),samples);
  fCV_standAloneFeaturesCos.resize((scales*4),samples);
  fCV_mixedRotationH2SV2.resize(scales,samples);

  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t\t>>>>>>>> Set Up Data Structures TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";

  const unsigned int count = fCV_countSM;

  LINFO("COUNT %d %d",count,fCV_countSM);

  //Image<PixH2SV<FLOAT> > imageH2SV = fCV_realImageLowPass;

  multiScaleBatchFilter(&fCV_realImageH2SV2, &fCV_rmap, &count
                        , &fCV_gaborFiltersSin, scales, useQuarter, true,
                        &fCV_standAloneFeaturesSin);
  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t\t>>>>>>>> Convolved Sin: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";
  multiScaleBatchFilter(&fCV_realImageH2SV2, &fCV_rmap, &count
                        , &fCV_gaborFiltersCos, scales, useQuarter, true,
                        &fCV_standAloneFeaturesCos);
  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t\t>>>>>>>> Convolved Cos: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";


  for(unsigned int i = 0; i < fCV_standAloneFeatures.size(); i++)
  {
    for(unsigned int j = 0; j < fCV_standAloneFeatures[i].size(); j++)
    {
      fCV_standAloneFeatures[i][j].setH1(
        fabs(fCV_standAloneFeaturesSin[i][j].H1()) +
        fabs(fCV_standAloneFeaturesCos[i][j].H1()));
      //LINFO("%f",fCV_standAloneFeatures[i][j].h1);
      fCV_standAloneFeatures[i][j].setH2(
        fabs(fCV_standAloneFeaturesSin[i][j].H2()) +
        fabs(fCV_standAloneFeaturesCos[i][j].H2()));
      //LINFO("%f",fCV_standAloneFeatures[i][j].h2);
      fCV_standAloneFeatures[i][j].setS(
        fabs(fCV_standAloneFeaturesSin[i][j].S()) +
        fabs(fCV_standAloneFeaturesCos[i][j].S()));
      //LINFO("%f",fCV_standAloneFeatures[i][j].s);
      fCV_standAloneFeatures[i][j].setV(
        fabs(fCV_standAloneFeaturesSin[i][j].V()) +
        fabs(fCV_standAloneFeaturesCos[i][j].V()));
      //LINFO("%f",fCV_standAloneFeatures[i][j].v);
    }
  }

  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t\t>>>>>>>> Mixed Features TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";

  multiScaleJunctionFilter(count, scales, &fCV_standAloneFeatures,
                           &fCV_mixedRotationH2SV2);

  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t\t>>>>>>>> Junctions Mixed TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";

  const std::string noutFile = filename + ".stand.alone.features.out.txt";

  LINFO("WRITING %s", noutFile.c_str());
  std::ofstream outFile(noutFile.c_str(),std::ios::app);
  for(unsigned int j = 0; j < fCV_standAloneFeatures[0].size(); j++)
  {
    for(unsigned int i = 0; i < fCV_standAloneFeatures.size(); i++)
    {
      outFile << fCV_standAloneFeatures[i][j].H1() << "\t";
      outFile << fCV_standAloneFeatures[i][j].H2() << "\t";
      outFile << fCV_standAloneFeatures[i][j].S()  << "\t";
      outFile << fCV_standAloneFeatures[i][j].V()  << "\t";
    }
    outFile << "\n";
  }

  const std::string noutFile2 = filename + ".stand.alone.junctions.out.txt";

  LINFO("WRITING %s", noutFile2.c_str());
  std::ofstream outFile2(noutFile2.c_str(),std::ios::app);
  for(unsigned int j = 0; j < fCV_mixedRotationH2SV2[0].size(); j++)
  {
    for(unsigned int i = 0; i < fCV_mixedRotationH2SV2.size(); i++)
    {
      outFile2 << fCV_mixedRotationH2SV2[i][j].H1()  << "\t";
      outFile2 << fCV_mixedRotationH2SV2[i][j].H2()  << "\t";
      outFile2 << fCV_mixedRotationH2SV2[i][j].S()   << "\t";
      outFile2 << fCV_mixedRotationH2SV2[i][j].V()   << "\t";
    }
    outFile2 << "\n";
  }

  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t\t>>>>>>>> Features writen to file TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";
}

// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVrunStandAloneMSBatchTest(
                                  std::string filename)
{
  Timer tim;
  tim.reset();
  int t1,t2,t3;
  int t0 = tim.get();  // to measure display time
  t1 = 0;
  t2 = t1 - t0;

  const float gaborMasks[4]   = {0.0F,45.0F,90.0F,135.0F};
  std::vector<std::string> gaborLabels(4,"");
  gaborLabels[0] = "0";  gaborLabels[1] = "45";
  gaborLabels[2] = "90"; gaborLabels[3] = "135";
  const FLOAT gaborSTD        = fCV_gaborStandardDev;
  const FLOAT gaborPrd        = fCV_gaborPeriod;
  const unsigned int scales   = fCV_gaborScales;
  const bool useQuarter       = fCV_gaborUseQuarter;
  LINFO("Gabor period %f std %f",gaborPrd,gaborSTD);

  // load ICA unmixing matrix for junctions
  readMatrix junctionMat("ICAunmix.junctions.new.6.dat");
  readMatrix junctionMatG("ICAunmix.junctions.new.G.dat");
  LINFO("read Matrix");
  Image<FLOAT> ICAMatrix = junctionMat.returnMatrixAsImage();
  Image<FLOAT> ICAMatrixG = junctionMatG.returnMatrixAsImage();
  LINFO("creating images");
  Image<FLOAT> junctionMatrix;
  Image<FLOAT> junctionMatrixG;

  junctionMatrix.resize(4*scales,fCV_realImage.getHeight()*
                        fCV_realImage.getWidth(),true);
  junctionMatrixG.resize(scales,fCV_realImage.getHeight()*
                         fCV_realImage.getWidth(),true);

  Image<FLOAT> reducedMatrix;

  Image<FLOAT> reducedMatrixG;

  std::vector<PixH2SV2<FLOAT> > samples(
                             (unsigned)fCV_realImage.getWidth() *
                             (unsigned)fCV_realImage.getHeight()
                             ,PixH2SV2<FLOAT>(0.0F));

  // resize (scales * HSV2 * angles) + junctions;
  fCV_standAloneFeatures.resize(scales*4,samples);
  fCV_mixedRotationH2SV2.resize(scales,samples);

  const Image<FLOAT> blankImage;

  std::vector<Image<FLOAT> > gaborFiltersSin(4,blankImage);
  std::vector<Image<FLOAT> > gaborFiltersCos(4,blankImage);

  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t\t>>>>>>>> Set Up Data Structures TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";
  unsigned int iii = 0;
  for(typename std::vector<Image<FLOAT> >::iterator igaborFilters
        = gaborFiltersSin.begin();
      igaborFilters != gaborFiltersSin.end(); ++igaborFilters, iii++)
  {
    *igaborFilters = gaborFilter<FLOAT>(gaborSTD,gaborPrd,0.0F
                                         ,gaborMasks[iii]);
    Raster::VisuFloat(*igaborFilters,FLOAT_NORM_0_255
                      ,sformat("image.gabor.sin.%d.pgm",iii));
  }

  iii = 0;
  for(typename std::vector<Image<FLOAT> >::iterator igaborFilters
        = gaborFiltersCos.begin();
      igaborFilters != gaborFiltersCos.end(); ++igaborFilters, iii++)
  {
    *igaborFilters = gaborFilter<FLOAT>(gaborSTD,gaborPrd,45.0F
                                         ,gaborMasks[iii]);
    Raster::VisuFloat(*igaborFilters,FLOAT_NORM_0_255
                      ,sformat("image.gabor.cos.%d.pgm",iii));
  }

  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t\t>>>>>>>> Set Up Data Gabors TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";
  fCV_countSM = (fCV_realImage.getWidth()) *
    (fCV_realImage.getHeight());
  Point2D<int> tempP;
  Point2D<int> *tempPP = &tempP;
  fCV_cmap.resize(fCV_countSM,tempP);
  fCV_rmap.resize(fCV_countSM,tempPP);

  std::vector<Point2D<int> >::iterator icmap  = fCV_cmap.begin();
  std::vector<Point2D<int>*>::iterator irmap = fCV_rmap.begin();
  for(int j = 0; j < fCV_realImage.getHeight(); j++)
  {
    for(int i = 0; i < fCV_realImage.getWidth(); i++)
    {
      icmap->i = i;
      icmap->j = j;
      *irmap   = &*icmap;
      ++icmap; ++irmap;
    }
  }

  typename std::vector<std::vector<PixH2SV2<FLOAT> > > fSin;
  typename std::vector<std::vector<PixH2SV2<FLOAT> > > fCos;

  typename std::vector<std::vector<FLOAT> > fSinG;
  typename std::vector<std::vector<FLOAT> > fCosG;
  typename std::vector<std::vector<FLOAT> > standAloneFeaturesG;
  typename std::vector<std::vector<FLOAT> > mixedRotationG;

  fSin = fCV_standAloneFeatures;
  fCos = fCV_standAloneFeatures;

  typename std::vector<FLOAT> tempG(fSin[0].size(),0.0F);
  fSinG.resize(fSin.size(),tempG);
  fCosG.resize(fSin.size(),tempG);
  standAloneFeaturesG.resize(fSin.size(),tempG);

  typename std::vector<FLOAT> tempGM(fCV_mixedRotationH2SV2[0].size(),0.0F);
  mixedRotationG.resize(fCV_mixedRotationH2SV2.size(),tempGM);

  Image<FLOAT> realImageG;
  realImageG.resize(fCV_realImageH2SV2.getWidth(),
                    fCV_realImageH2SV2.getHeight(),true);

  for(unsigned int i = 0; i < (unsigned)fCV_realImageH2SV2.getWidth(); i++)
  {
    for(unsigned int j = 0; j < (unsigned)fCV_realImageH2SV2.getHeight(); j++)
    {
      realImageG.setVal(i,j,(fCV_realImageH2SV2.getVal(i,j)).H1());
    }
  }

  unsigned int count = fCV_countSM;
  LINFO("COUNT %d %d",count,fCV_countSM);
  LINFO("Scales %d",scales);
  // Color
  multiScaleBatchFilter(&fCV_realImageH2SV2, &fCV_rmap, &count
                        , &gaborFiltersSin, scales, useQuarter, true,
                        &fSin);
  multiScaleBatchFilter(&fCV_realImageH2SV2, &fCV_rmap, &count
                        , &gaborFiltersCos, scales, useQuarter, true,
                        &fCos);
  // Value Only
  multiScaleBatchFilter(&realImageG, &fCV_rmap, &count
                        , &gaborFiltersSin, scales, useQuarter, true,
                        &fSinG);
  multiScaleBatchFilter(&realImageG, &fCV_rmap, &count
                        , &gaborFiltersCos, scales, useQuarter, true,
                        &fCosG);



  for(unsigned int i = 0; i < fCV_standAloneFeatures.size(); i++)
  {
    for(unsigned int j = 0; j < count; j++)
    {
      standAloneFeaturesG[i][j] = fabs(fSinG[i][j])
                                + fabs(fCosG[i][j]);

      fCV_standAloneFeatures[i][j].setH1(fabs(fCos[i][j].H1())
                                       + fabs(fSin[i][j].H1()));
      fCV_standAloneFeatures[i][j].setH2(fabs(fCos[i][j].H2())
                                       + fabs(fSin[i][j].H2()));
      fCV_standAloneFeatures[i][j].setS(fabs(fCos[i][j].S())
                                      + fabs(fSin[i][j].S()));
      fCV_standAloneFeatures[i][j].setV(fabs(fCos[i][j].V())
                                      + fabs(fSin[i][j].V()));
    }
  }

  multiScaleJunctionFilter(count, scales, &fCV_standAloneFeatures
                           ,&fCV_mixedRotationH2SV2);

  multiScaleJunctionFilter(count, scales, &standAloneFeaturesG
                           ,&mixedRotationG);

  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t\t>>>>>>>> Features Fetched TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";

  const std::string noutFile = filename + ".stand.alone.features.out.txt";

  LINFO("WRITING %s", noutFile.c_str());
  std::ofstream outFile(noutFile.c_str(),std::ios::app);
  Image<FLOAT> timageh1;
  Image<FLOAT> timageh2;
  Image<FLOAT> timages;
  Image<FLOAT> timagev;

  timageh1.resize(fCV_realImageLowPass.getWidth(),
                fCV_realImageLowPass.getHeight());
  timageh2.resize(fCV_realImageLowPass.getWidth(),
                fCV_realImageLowPass.getHeight());
  timages.resize(fCV_realImageLowPass.getWidth(),
                fCV_realImageLowPass.getHeight());
  timagev.resize(fCV_realImageLowPass.getWidth(),
                fCV_realImageLowPass.getHeight());

  for(unsigned int i = 0; i < fCV_standAloneFeatures.size(); i++)
  {
    typename Image<FLOAT>::iterator itimageh1 = timageh1.beginw();
    for(unsigned int j = 0; j < count; j++,
          ++itimageh1)
    {
      *itimageh1 = fCV_standAloneFeatures[i][j].H1();
    }

    typename Image<FLOAT>::iterator itimageh2 = timageh2.beginw();
    for(unsigned int j = 0; j < count; j++,
          ++itimageh2)
    {
      *itimageh2 = fCV_standAloneFeatures[i][j].H2();
    }

    typename Image<FLOAT>::iterator itimages  = timages.beginw();
    for(unsigned int j = 0; j < count; j++,
          ++itimages)
    {
      *itimages  = fCV_standAloneFeatures[i][j].S();
    }

    typename Image<FLOAT>::iterator itimagev  = timagev.beginw();
    for(unsigned int j = 0; j < count; j++,
          ++itimagev)
    {
      *itimagev  = fCV_standAloneFeatures[i][j].V();
    }

    Raster::VisuFloat(timageh1,FLOAT_NORM_0_255,sformat("imageOut.H1.%d.pgm",i));
    Raster::VisuFloat(timageh2,FLOAT_NORM_0_255,sformat("imageOut.H2.%d.pgm",i));
    Raster::VisuFloat(timages ,FLOAT_NORM_0_255,sformat("imageOut.S.%d.pgm" ,i));
    Raster::VisuFloat(timagev ,FLOAT_NORM_0_255,sformat("imageOut.V.%d.pgm" ,i));
  }

  for(unsigned int i = 0; i < fCV_mixedRotationH2SV2.size(); i++)
  {
    typename Image<FLOAT>::iterator itimage = timageh1.beginw();
    for(unsigned int j = 0; j < fCV_standAloneFeatures[i].size(); j++,
          ++itimage)
    {
      *itimage = fCV_mixedRotationH2SV2[i][j].H1();
    }

    itimage = timageh2.beginw();
    for(unsigned int j = 0; j < fCV_standAloneFeatures[i].size(); j++,
          ++itimage)
    {
      *itimage = fCV_mixedRotationH2SV2[i][j].H2();
    }

    itimage = timages.beginw();
    for(unsigned int j = 0; j < fCV_standAloneFeatures[i].size(); j++,
          ++itimage)
    {
      *itimage = fCV_mixedRotationH2SV2[i][j].S();
    }

    itimage = timagev.beginw();
    for(unsigned int j = 0; j < fCV_standAloneFeatures[i].size(); j++,
          ++itimage)
    {
      *itimage = fCV_mixedRotationH2SV2[i][j].V();
    }

    Raster::VisuFloat(timageh1,FLOAT_NORM_0_255,sformat("imageOut.J.H1.%d.pgm",i));
    Raster::VisuFloat(timageh2,FLOAT_NORM_0_255,sformat("imageOut.J.H2.%d.pgm",i));
    Raster::VisuFloat(timages ,FLOAT_NORM_0_255,sformat("imageOut.J.S.%d.pgm" ,i));
    Raster::VisuFloat(timagev ,FLOAT_NORM_0_255,sformat("imageOut.J.V.%d.pgm" ,i));
  }

  // load up the image matrix
  unsigned int y = 0;
  for(unsigned int j = 0; j < fCV_mixedRotationH2SV2[0].size(); j++)
  {
    unsigned int x = 0;
    unsigned int w = 0;
    for(unsigned int i = 0; i < fCV_mixedRotationH2SV2.size(); i++)
    {
      junctionMatrixG.setVal(w,y,mixedRotationG[i][j]); w++;
      junctionMatrix.setVal(x,y,fCV_mixedRotationH2SV2[i][j].H1()); x++;
      junctionMatrix.setVal(x,y,fCV_mixedRotationH2SV2[i][j].H2()); x++;
      junctionMatrix.setVal(x,y,fCV_mixedRotationH2SV2[i][j].S()); x++;
      junctionMatrix.setVal(x,y,fCV_mixedRotationH2SV2[i][j].V()); x++;
    }
    y++;
  }
  LINFO("%d x %d",junctionMatrix.getWidth(),junctionMatrix.getHeight());
  LINFO("%d x %d",ICAMatrix.getWidth(),ICAMatrix.getHeight());

  LINFO("%d x %d",junctionMatrixG.getWidth(),junctionMatrixG.getHeight());
  LINFO("%d x %d",ICAMatrixG.getWidth(),ICAMatrixG.getHeight());

  reducedMatrix  = matrixMult(junctionMatrix,ICAMatrix);
  reducedMatrixG = matrixMult(junctionMatrixG,ICAMatrixG);
  LINFO("%d x %d",reducedMatrix.getWidth(),reducedMatrix.getHeight());

  for(unsigned int i = 0; i < (unsigned)reducedMatrix.getWidth(); i++)
  {
    Image<FLOAT> outImage;
    outImage.resize(fCV_realImageLowPass.getWidth(),
                    fCV_realImageLowPass.getHeight(),true);
    typename Image<FLOAT>::iterator outImage_itr = outImage.beginw();
    for(unsigned int j = 0; j < (unsigned)reducedMatrix.getHeight(); j++,
          ++outImage_itr)
    {
      *outImage_itr = logsig(0.0F,0.25F,reducedMatrix.getVal(i,j));
    }
    //outImage = lowPass9(outImage);
    Raster::VisuFloat(outImage,FLOAT_NORM_0_255,sformat("imageOut.ICA.J.%d.pgm",i));

  }

  for(unsigned int i = 0; i < (unsigned)reducedMatrixG.getWidth(); i++)
  {
    Image<FLOAT> outImage;
    outImage.resize(fCV_realImageLowPass.getWidth(),
                    fCV_realImageLowPass.getHeight(),true);
    typename Image<FLOAT>::iterator outImage_itr = outImage.beginw();
    for(unsigned int j = 0; j < (unsigned)reducedMatrixG.getHeight(); j++,
          ++outImage_itr)
    {
      *outImage_itr = reducedMatrixG.getVal(i,j);
    }
    //outImage = lowPass9(outImage);
    Raster::VisuFloat(outImage,FLOAT_NORM_0_255,sformat("imageOut.ICA.J.G.%d.pgm",i));
  }

  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t\t>>>>>>>> Features writen to file TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";

}

// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVICAfeaturesToFile(std::string fileName)
{
  std::string newFile;
  newFile = fileName + ".out.ICA";
  std::ofstream outfile(newFile.c_str(),std::ios::app);
  typename std::vector<std::vector<FLOAT> >::iterator ispace;
  typename std::vector<FLOAT>::iterator iispace;
  ispace = fCV_space.begin();
  for(int i = 0; i < fCV_countSM; i++, ++ispace)
  {
    for(iispace = ispace->begin(); iispace != ispace->end(); ++iispace)
      outfile << *iispace << "\t";

    outfile << "\n";
  }
  outfile.close();
}

// ######################################################################
// ######################################################################
// MAIN WORKING PRIVATE METHODS
// ######################################################################
// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVswitchCov()
{

  // Alternate pointers between covHolder objects
  // this is done to keep a record of the last iter. results
  fCV_covHolderLast = fCV_covHolderCurrent;
  if(fCV_currentCovHolder == COVHOLDER-1)
  {
    fCV_currentCovHolder = 0;
  }
  else
  {
    fCV_currentCovHolder++;
  }
  fCV_covHolderCurrent = &fCV_covHolder[fCV_currentCovHolder];

  for(unsigned long i = 0; i < fCV_covHolderCurrent->size(); i++)
  {
    fCV_covHolderCurrent->at(i).isLarge = false;
  }
  fCV_covDataSizeLast = fCV_covDataSizeCurrent;
  fCV_covDataSizeCurrent = 0;

}

// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVlowPass()
{
  if(fCV_lowPassType != 0)
  {
    std::cerr << "LOW PASSING IMAGE " << fCV_lowPassType << "\n";
    if(fCV_lowPassType == 1)
      fCV_realImageLowPass = lowPass3(fCV_realImage);
    else if(fCV_lowPassType == 2)
      fCV_realImageLowPass = lowPass5(fCV_realImage);
    else if(fCV_lowPassType == 3)
      fCV_realImageLowPass = lowPass9(fCV_realImage);
    else if(fCV_lowPassType > 3)
      fCV_realImageLowPass = fCV_realImage;
  }
  else
    fCV_realImageLowPass = fCV_realImage;
  fCV_realImageH2SV2        = fCV_realImage;
  fCV_realImageH2SV2LowPass = fCV_realImageLowPass;
  //Raster::VisuRGB(fCV_realImageLowPass,"LP.ppm");
  //Raster::VisuRGB(fCV_realImage,"real.ppm");
}

// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVfindFeaturesBrain()
{
  Timer tim;
  tim.reset();
  int t1,t2,t3;
  t2 = 0;
  int t0 = tim.get();  // to measure display time

  //---------------Get Saliency Data---------------//
  //std::cerr << "(1) GETTING saliency data \n";
  // get the current saliency map see Brain.C for examples
  ///////fCV_SM = fCV_brain->getSM();
  ///////fCV_VC = fCV_brain->getVC();
  Image<FLOAT> salmap;/////////////// = fCV_SM->getV(false);
  LFATAL("FIXME");
  Image<FLOAT> salmapOut = salmap;

  // apply temporal band pass to saliency map
  if(fCV_doSLPTB == true)
  {
    fCV_salmapLowPass = ((fCV_salmapLowPass*fCV_salmapLowPassTemporalBias) +
                         salmap)/(1.0F +fCV_salmapLowPassTemporalBias);
  }
  else
  {
    fCV_salmapLowPass = salmap;
    fCV_doSLPTB = true;
  }

  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t\t>>>>>>>> findFeatures lowPass init TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";

  LINFO("Salmap size %d x %d to %d x %d",salmap.getWidth()
        ,salmap.getHeight(),fCV_sizeX,fCV_sizeY);
  salmap = rescaleBilinear(fCV_salmapLowPass,fCV_sizeX/3,fCV_sizeY/3);

  //std::cerr << "...model objects fetched\n";

  // make sure the salmap is less than the size of the Cmap
  ASSERT(salmap.getSize() <= (signed)fCV_cmap.size());

  // find a sparcer represntation of salincy map by treating salmap as
  // a map of statistical values. See function in Image_MathOps.*
  Image<FLOAT> smap = salmap;

  //Raster::WriteRGB(smap1,sformat("%s.out.salmapBias.ppm",fCV_fileName.c_str()));
  Image<FLOAT> fooshit = rescaleBilinear(fCV_salmapLowPass,fCV_sizeX,fCV_sizeY);

  //Raster::VisuFloat(fooshit,FLOAT_NORM_0_255,
  //sformat("%s.out.salmap.pgm",fCV_fileName.c_str()));
  fCV_monteDec = 0;
  std::vector<Point2D<int>*>::iterator irmap = fCV_rmap.begin();
  std::vector<Point2D<int> >::iterator icmapOld = fCV_cmapOld.begin();

  for(int i = 0; i < fCV_countSM; i++,  ++irmap, ++icmapOld)
  {
    *icmapOld = **irmap/3;
  }
  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t\t>>>>>>>> findFeatures set params TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";
  fCV_countMM = findMonteMap(smap,&fCV_cmap,
                             fCV_monteDec,fCV_saliencyExp);

  //std::cerr << "...First pass found " << fCV_countMM << " points\n";

  // Make the cmap sparser by decimation (see Image_MathOps.*
  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t\t>>>>>>>> findFeatures find monte map TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";


  fCV_countSM = makeSparceMap(&fCV_cmap, &fCV_rmap, &fCV_cmapOld,
                              &fCV_keepParticle,
                              fCV_countMM, fCV_sparcePoints);
  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t\t>>>>>>>> findFeatures find sparce map TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";

  irmap = fCV_rmap.begin();
  for(int i = 0; i < fCV_countSM; i++,  ++irmap, ++icmapOld)
  {
    //LINFO("A2");
    //LINFO("%d %d %d",i,(*irmap)->i,(*irmap)->j);

    (*irmap)->i = (*irmap)->i * 3;
    (*irmap)->j = (*irmap)->j * 3;
    //LINFO("%d %d %d",i,(*irmap)->i,(*irmap)->j);
    //LINFO("SALMAP SIZE %d %d",salmap.getWidth(),salmap.getHeight());
  }

  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t\t>>>>>>>> findFeatures expand map TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";

  irmap = fCV_rmap.begin();
  typename std::vector<FLOAT> emptyVec(1,0.0F);
  typename std::vector<std::vector<FLOAT> > tempEmpty(fCV_countSM,emptyVec);
  fCV_fmap = tempEmpty;

  // get sal values for each point into a vector fmap

  if(fCV_lowPassType == 5)
  {
    if(fCV_lowPassVector.size() < fCV_rmap.size())
    {
      PixRGB<FLOAT> tempish;
      fCV_lowPassVector.resize(fCV_rmap.size(),tempish);
    }

    //pix = filterAtLocation(&fCV_realImageLowPass,&fCV_lowPassKernel,
    //                         &*fCV_rmap[i]);
    filterAtLocationBatch(&fCV_realImageLowPass,&fCV_lowPassKernel, &fCV_rmap,
                          &fCV_lowPassVector,fCV_countSM);
  }
  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t\t>>>>>>>> findFeatures lowPass batch TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";

  ///////////////////  fCV_VC->getFeaturesBatch(&fCV_rmap,&fCV_fmap,&fCV_countSM);
  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t\t>>>>>>>> findFeatures get features TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";

}

// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVfindFeaturesNoBrain()
{
  Timer tim;
  tim.reset();
  int t1,t2,t3;
  t2 = 0; t3 = 0;
  int t0 = tim.get();  // to measure display time

  //---------------Get Saliency Data---------------//
  //std::cerr << "(1) GETTING saliency data \n";
  // get the current saliency map see Brain.C for examples

  // apply temporal band pass to saliency map
  if(fCV_doSLPTB == true)
  {
    fCV_salmapLowPass = ((fCV_salmapLowPass*fCV_salmapLowPassTemporalBias) +
                         *fCV_noBrainSalmap)
                         /(1.0F +fCV_salmapLowPassTemporalBias);
  }
  else
  {
    fCV_salmapLowPass = *fCV_noBrainSalmap;
    fCV_doSLPTB = true;
  }

  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t\t>>>>>>>> findFeatures lowPass init TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";

  LINFO("Salmap size %d x %d to %d x %d",fCV_noBrainSalmap->getWidth()
        ,fCV_noBrainSalmap->getHeight(),fCV_sizeX,fCV_sizeY);
  Image<FLOAT> salmap = rescaleBilinear(fCV_salmapLowPass,fCV_sizeX/3,fCV_sizeY/3);

  //std::cerr << "...model objects fetched\n";

  // make sure the salmap is less than the size of the Cmap
  ASSERT(salmap.getSize() <= (signed)fCV_cmap.size());

  // find a sparcer represntation of salincy map by treating salmap as
  // a map of statistical values. See function in Image_MathOps.*
  Image<FLOAT> smap = salmap;

  //Raster::WriteRGB(smap1,sformat("%s.out.salmapBias.ppm",fCV_fileName.c_str()));
  //Raster::WriteRGB(smap2,sformat("%s.out.salmap.ppm",fCV_fileName.c_str()));
  fCV_monteDec = 0;
  std::vector<Point2D<int>*>::iterator irmap = fCV_rmap.begin();
  std::vector<Point2D<int> >::iterator icmapOld = fCV_cmapOld.begin();

  for(int i = 0; i < fCV_countSM; i++,  ++irmap, ++icmapOld)
  {
    *icmapOld = **irmap/3;
  }
  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t\t>>>>>>>> findFeatures set params TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";
  fCV_countMM = findMonteMap(smap,&fCV_cmap,
                             fCV_monteDec,fCV_saliencyExp);

  //std::cerr << "...First pass found " << fCV_countMM << " points\n";

  // Make the cmap sparser by decimation (see Image_MathOps.*
  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t\t>>>>>>>> findFeatures find monte map TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";


  fCV_countSM = makeSparceMap(&fCV_cmap, &fCV_rmap, &fCV_cmapOld,
                              &fCV_keepParticle,
                              fCV_countMM, fCV_sparcePoints);
  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t\t>>>>>>>> findFeatures find sparce map TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";

  irmap = fCV_rmap.begin();
  for(int i = 0; i < fCV_countSM; i++,  ++irmap, ++icmapOld)
  {
    //LINFO("A2");
    //LINFO("%d %d %d",i,(*irmap)->i,(*irmap)->j);

    (*irmap)->i = (*irmap)->i * 3;
    (*irmap)->j = (*irmap)->j * 3;
    //LINFO("%d %d %d",i,(*irmap)->i,(*irmap)->j);
    //LINFO("SALMAP SIZE %d %d",salmap.getWidth(),salmap.getHeight());
  }

  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t\t>>>>>>>> findFeatures expand map TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";


  typename std::vector<FLOAT> cmaps(fCV_cmaps->size(),0.0F);
  typename std::vector<std::vector<FLOAT> > tempEmpty(fCV_countSM,cmaps);
  fCV_fmap = tempEmpty;


  // get sal values for each point into a vector fmap

  //if(fCV_lowPassType == 5)
  //{
  // if(fCV_lowPassVector.size() < fCV_rmap.size())
  // {
  //   PixRGB<FLOAT> tempish;
  //   fCV_lowPassVector.resize(fCV_rmap.size(),tempish);
  // }
  // filterAtLocationBatch(&fCV_realImageLowPass,&fCV_lowPassKernel, &fCV_rmap,
  //                      &fCV_lowPassVector,fCV_countSM);
  //}
  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t\t>>>>>>>> findFeatures lowPass batch TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";

  irmap = fCV_rmap.begin();

  // get values from conspicuity maps with simple interpolation

  // first iterate over each feature location

  //#######################################################
  // get feaures stand alone


  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t\t>>>>>>>> findFeatures get features TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";

}

// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVfindFeaturesFromFile(std::string fileName)
{
  ASSERT(fCV_useBrain == true);

  //---------------Get Saliency Data---------------//
  //std::cerr << "(1) GETTING saliency data \n";
  // get the current saliency map see Brain.C for examples
  //////fCV_SM = fCV_brain->getSM();
  //////fCV_VC = fCV_brain->getVC();
  Image<FLOAT> salmap;////////////// = fCV_SM->getV(false);
  LFATAL("FIXME");
  Image<FLOAT> salmapOut = salmap;

  // apply temporal band pass to saliency map
  if(fCV_doSLPTB == true)
  {
    fCV_salmapLowPass = ((fCV_salmapLowPass*fCV_salmapLowPassTemporalBias) +
                         salmap)/(1.0F +fCV_salmapLowPassTemporalBias);
  }
  else
  {
    fCV_salmapLowPass = salmap;
    fCV_doSLPTB = true;
  }

  // make sure the salmap is less than the size of the Cmap
  ASSERT(salmap.getSize() <= (signed)fCV_cmap.size());

  // find a sparcer represntation of salincy map by treating salmap as
  // a map of statistical values. See function in Image_MathOps.*
  //Image<PixRGB<FLOAT> > realImageCopy =  fCV_realImage;
  //PixRGB<FLOAT> mrPixel;
  //mrPixel.setRed(255); mrPixel.setBlue(128); mrPixel.setGreen(0);
  std::string in;
  std::ifstream saccadeFile(fileName.c_str(),std::ios::in);
  std::string values[7];
  unsigned int count = 0;
  unsigned int sampleNumber = 0;
  unsigned int indexNumber = 0;
  Point2D<int> point;
  Point2D<int> *pointer;
  pointer = &point;
  int tensions = 0;
  fCV_indexNumber.resize(fCV_cmap.size(),tensions);
  fCV_jumpTo.resize(fCV_cmap.size(),point);
  while(saccadeFile >> in)
  {
    values[count] = in;
    if(count == 6)
    {
      count = 0;
      /*std::cout << values[0] << "\t"
                << values[1] << "\t"
                << values[2] << "\t"
                << values[3] << "\t"
                << values[4] << "\t"
                << values[5] << "\t"
                << values[6] << "\n";*/
      if((int)atof(values[2].c_str()) != 0)
      {
        if(fCV_cmap.size() == sampleNumber)
        {
          fCV_cmap.resize(fCV_cmap.size() + VEC_RESIZE,point);
          fCV_rmap.resize(fCV_cmap.size() + VEC_RESIZE,pointer);
          fCV_indexNumber.resize(fCV_cmap.size() + VEC_RESIZE,tensions);
          fCV_jumpTo.resize(fCV_cmap.size() + VEC_RESIZE,point);
        }
        fCV_cmap[sampleNumber].i      = (int)atof(values[0].c_str());
        fCV_cmap[sampleNumber].j      =
          (int)atof(values[1].c_str()) - STUPIDMASKOFFSET/2;
        //drawCircle(realImageCopy, fCV_cmap[sampleNumber],3,mrPixel,2);
        fCV_rmap[sampleNumber]        = &fCV_cmap[sampleNumber];
        fCV_indexNumber[sampleNumber] = indexNumber;
        fCV_jumpTo[sampleNumber].i    = (int)atof(values[3].c_str());
        fCV_jumpTo[sampleNumber].j    =
          (int)atof(values[4].c_str()) - STUPIDMASKOFFSET/2;
        sampleNumber++;
      }
      indexNumber++;
    }
    else
    {
      count++;
    }
  }
  //Raster::WriteRGB(realImageCopy,sformat("%s.out.eyeDataOver.ppm"
  //               ,fileName.c_str()));
  LINFO("FOUND %d samples from %s",sampleNumber,fileName.c_str());
  fCV_countSM = sampleNumber;

  // resize sample containers
  fCVresizeMaps1(sampleNumber);
  fCVresizeMaps2(sampleNumber,fCV_newMatSize);
  for(unsigned int i = 0; i < (unsigned)fCV_countSM; i++)
  {
    if(fCV_rmap[i]->i >= fCV_sizeX) fCV_rmap[i]->i = fCV_sizeX-1;
    if(fCV_rmap[i]->j >= fCV_sizeY) fCV_rmap[i]->j = fCV_sizeY-1;
    if(fCV_jumpTo[i].i >= fCV_sizeX) fCV_jumpTo[i].i = fCV_sizeX-1;
    if(fCV_jumpTo[i].j >= fCV_sizeY) fCV_jumpTo[i].j = fCV_sizeY-1;
    /*LINFO("VALS %d,%d", fCV_rmap[i]->i,fCV_rmap[i]->j);*/
  }
  typename std::vector<FLOAT> emptyVec(1,0.0F);
  typename std::vector<std::vector<FLOAT> > tempEmpty(fCV_countSM,emptyVec);
  fCV_fmap = tempEmpty;

  // get sal values for each point into a vector fmap
  /////////////////////  fCV_VC->getFeaturesBatch(&fCV_rmap,&fCV_fmap,&fCV_countSM);
}

// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVrunICA()
{


  //---------------Run ICA unmixing per Channel---------------//
  //std::cerr << "(2) RUNNING ICA unmixing\n";
  typename std::vector<Image<FLOAT> >::iterator image_itr;
  typename std::vector<Image<FLOAT> >::iterator unmixed_itr;
  typename std::vector<std::vector<FLOAT*> >::iterator unmixedMap_itr;
  typename std::vector<std::vector<FLOAT> >::iterator finalMap_itr;
  typename std::vector<std::vector<FLOAT> >::iterator space_itr;
  typename std::vector<FLOAT>::iterator space_itr_itr;
  typename std::vector<FLOAT*>::iterator weight_itr;
  typename std::vector<FLOAT>::iterator normalize_itr;
  typename std::vector<FLOAT>::iterator trans_itr;
  std::vector<int>::iterator index_itr;
  std::vector<int*>::iterator fpc_itr;
  typename Image<FLOAT>::iterator i_itr;
  std::vector<bool*>::iterator featureOn_itr;
  std::vector<std::string*>::iterator featureNameICA_itr;
  //index_itr = fCV_featureMatrixSizes.begin();
  unmixedMap_itr = fCV_unmixedMap.begin();
  unmixed_itr = fCV_Unmixed.begin();
  space_itr = fCV_space.begin();
  featureOn_itr = fCV_featureOn.begin();
  weight_itr = fCV_weights.begin();
  normalize_itr = fCV_featureNormConst.begin();
  trans_itr = fCV_featureTransConst.begin();
  fpc_itr = fCV_ICAfeaturesPerChannel.begin();
  featureNameICA_itr = fCV_featureNameICA.begin();
  // make sure channel size and matrix sizes match
  ASSERT(fCV_Unmixed.size() == fCV_featureMatrixSizes.size());
  ASSERT(fCV_Unmixed.size() == fCV_ICAunmix.size());
  // convert vector to image
  //LINFO("SIZEEE 2 %d x %d",fCV_fmap.size(),fCV_fmap[0].size());

  Image<FLOAT> ifmap = vectorToImage(fCV_fmap);
  Image<FLOAT> mmmap = vectorToImage(fCV_mixedRotation);
  Image<FLOAT> mmap = transpose(mmmap);

  // print out feature map if desired
  //if(fCV_printOutFeatures == true)
  //{
  //  Image<FLOAT> tfoo = ifmap;
  //  Raster::WriteGray(tfoo,sformat("%s.out.featureSet.pgm",fCV_fileName.c_str()));
  //}
  // create indexes into matrices
  unsigned int pStart = 0; unsigned int pStop = 0;
  unsigned int zero = 0;
  unsigned int sm = (unsigned)fCV_countSM;
  // multiply each feature set (channel) by it's unmixing matrix for ICA
  // store output
  int current = 0;
  int iter = 0;
  long colorOffset = 0;

  //LINFO("GO");


  for(image_itr = fCV_ICAunmix.begin(); image_itr != fCV_ICAunmix.end();
        ++image_itr, ++unmixed_itr, ++featureOn_itr, ++weight_itr, ++fpc_itr,
        ++normalize_itr, ++trans_itr, iter++)
  {
    bool doThis = false;
    unsigned int width = 0;
    unsigned int ICAwidth = 0;
    FLOAT translate = 0.0F;
    FLOAT normalize = 0.0F;
    FLOAT weight = 0.0F;
    pStop = pStart + (fCV_featuresPerChannel-1);
    if(iter < fCV_mixOffset)
    {
      width = (unsigned)image_itr->getWidth();
      /*std::cerr << "Iter " << iter
                << "...Matrix sizes 1) " << ifmap.getWidth() << " x "
                << ifmap.getHeight() << " 2) "
                << image_itr->getWidth() << " x "
                << image_itr->getHeight() << "\n"
                << "...Interval " << pStart << " - "
                << pStop << "\n";*/
    }
    else
    {
      width = (unsigned)(image_itr-1)->getWidth();
      /*std::cerr << "Iter " << iter+1
                << "...Matrix sizes 1) " << ifmap.getWidth() << " x "
                << ifmap.getHeight() << " 2) "
                << (image_itr-1)->getWidth() << " x "
                << (image_itr-1)->getHeight() << "\n"
                << "...Interval " << pStart << " - "
                << pStop << "\n";*/
    }

    if(**featureOn_itr == true)
    {
      if(iter < fCV_mixOffset)
      {
        //std::cerr << ">>>Regular Channel " << fCV_featureName[iter]
        //        << " is On - Offset: " << iter << "\n";
        *unmixed_itr = matrixMult(ifmap,*image_itr,pStart,pStop,
                                  zero,width,zero,sm);
        //std::cerr << "\tData width: " << unmixed_itr->getWidth() << "\n";
        /*
        for(int ii = 0; ii <  (image_itr-1)->getWidth(); ii++)
        {
          for(int jj = 0; jj < (image_itr-1)->getHeight(); jj++)
          {
            std::cout << (image_itr-1)->getVal(ii,jj);
          }
          std::cout << "\n";
        }
        */
        colorOffset += unmixed_itr->getWidth();
        for(int i = 0; i <  unmixed_itr->getWidth(); i++)
        {
          *featureNameICA_itr = &fCV_featureName[iter];
          ++featureNameICA_itr;
        }
        doThis = true;
        ICAwidth = unmixed_itr->getWidth();
        translate = *trans_itr;
        normalize = *normalize_itr;
        weight = **weight_itr;

      }
      else
      {
        if((iter <= fCV_mixOffset+3) && (iter != fCV_mixOffset))
        {
          //std::cerr << ">>>Mix Channel " << fCV_featureName[iter]
          //        << " is On - Offset: " << iter << "\n";
          unsigned int st = fCV_featuresPerChannel*(iter-fCV_mixOffset-1);
          unsigned int sp = st + (fCV_featuresPerChannel-1);
          //LINFO("START %d STOP %d width %d sm %d",st,sp,width,sm);
          //LINFO("SIZE %d x %d", mmap.getWidth(), mmap.getHeight());
          //LINFO("WIDTH %d",(image_itr-1)->getWidth());
          *unmixed_itr = matrixMult(mmap,*(image_itr-1),st,sp,
                                  zero,width,zero,sm);
          //std::cerr << "\tData width: " << unmixed_itr->getWidth() << "\n";
          /*
          for(int ii = 0; ii <  (image_itr-1)->getWidth(); ii++)
          {
            for(int jj = 0; jj < (image_itr-1)->getHeight(); jj++)
            {
              std::cout << (image_itr-1)->getVal(ii,jj);
            }
            std::cout << "\n";
          }
          */
          colorOffset += unmixed_itr->getWidth();
          for(int i = 0; i <  unmixed_itr->getWidth(); i++)
          {
            *featureNameICA_itr = &fCV_featureName[iter];
            ++featureNameICA_itr;
          }
          doThis = true;
          ICAwidth = unmixed_itr->getWidth();
          translate = *(trans_itr);
          normalize = *(normalize_itr);
          weight = **(weight_itr);

        }
        else if((iter <= fCV_motionCombinedOffset) && (iter != fCV_mixOffset))
        {
          //std::cerr << ">>>Combined Motion Channel is On - "
          //        << fCV_featureName[iter]
          //        << " Offset: " << iter << "\n";
          unsigned int st = fCV_motOffset*fCV_featuresPerChannel;
          unsigned int sp = st + ((fCV_featuresPerChannel*4)-1);
          //LINFO("START %d STOP %d width %d sm %d",st,sp,width,sm);
          *unmixed_itr = matrixMult(ifmap,*(image_itr-1),st,sp,
                                  zero,width,zero,sm);

          *unmixed_itr = logSig(*unmixed_itr,*(trans_itr),*(normalize_itr));

          colorOffset += unmixed_itr->getWidth();
          //std::cerr << "\tData width: " << unmixed_itr->getWidth() << "\n";
          /*
          for(int ii = 0; ii <  (image_itr-1)->getWidth(); ii++)
          {
            for(int jj = 0; jj < (image_itr-1)->getHeight(); jj++)
            {
              std::cout << (image_itr-1)->getVal(ii,jj);
            }
            std::cout << "\n";
          }
          */
          for(int i = 0; i <  unmixed_itr->getWidth(); i++)
          {
            *featureNameICA_itr = &fCV_featureName[iter];
            ++featureNameICA_itr;
          }
          doThis = true;
          ICAwidth = unmixed_itr->getWidth();
          translate = 0;
          normalize = 1;
          weight = **(weight_itr);

        }
        else
        {
          //std::cerr << "<Special Channel is Off>\n";
        }
      }

      //Image<FLOAT> thisSucks;
      //thisSucks = *unmixed_itr;
      //Raster::VisuFloat(thisSucks,FLOAT_NORM_0_255,sformat("ICA.%d.pgm",current));
      //std::cerr << "done\n";
    }
    else
    {
      //std::cerr << "<Channel is Off>\n";
    }
    pStart = pStop + 1;
    // rejoin output into a vector map (uck)
    // this takes each image matrix output and appends it onto a
    // full output vector
    if(doThis == true)
    {
      //if((iter < newOffset) && (iter != fCV_mixOffset))
      //{
        space_itr = fCV_space.begin();
        i_itr = unmixed_itr->beginw();
        //std::cerr << "REJOINING output from feautes in ICA\n";
        //std::cerr << "Translate " << translate << " Weight "
        //        << weight << " Normalize " << normalize << "\n";
        while(i_itr != unmixed_itr->endw())
        {
          space_itr_itr = space_itr->begin() + current;
          for(int j = 0; j < unmixed_itr->getWidth(); j++, ++space_itr_itr
                , ++i_itr)
          {
            *space_itr_itr = (*i_itr+translate)
              * weight * normalize;
          }
          ++space_itr;
        }
        //std::cerr << "\t.Added DATA - Width: " << ICAwidth
        //        << " At position: " << current << "\n";
        current += ICAwidth;

        //}
    }
  }


  if((fCV_redOn == true)  || (fCV_greenOn == true)  ||
     (fCV_blueOn == true) || (fCV_yellowOn == true) ||
     (fCV_hueOn == true)  || (fCV_satOn == true)    ||
     (fCV_valOn == true))
  {
    //std::cerr << ">>>Color Channel is On\n";
    PixRGB<FLOAT> pix;
    space_itr = fCV_space.begin();
    //LINFO("SPACE SIZE %d x %d",fCV_space.size(),fCV_space[0].size());
    //LINFO("OFFSET %d", colorOffset);
    if(fCV_redOn == true)
    {
      *featureNameICA_itr = &fCV_featureName[fCV_colorOffset];
      //std::cerr << ">>>Color Channel " << fCV_featureName[fCV_colorOffset]
      //        << " is On - Offset: " << colorOffset << "\n";
      ++featureNameICA_itr;

    }
    if(fCV_greenOn == true)
    {
      *featureNameICA_itr = &fCV_featureName[fCV_colorOffset+1];
      //std::cerr << ">>>Color Channel " << fCV_featureName[fCV_colorOffset+1]
      //                << " is On - Offset: " << colorOffset << "\n";
      ++featureNameICA_itr;
    }
    if(fCV_blueOn == true)
    {
      *featureNameICA_itr = &fCV_featureName[fCV_colorOffset+2];
      //std::cerr << ">>>Color Channel " << fCV_featureName[fCV_colorOffset+2]
      //                << " is On - Offset: " << colorOffset << "\n";
      ++featureNameICA_itr;
    }
    if(fCV_yellowOn == true)
    {
      *featureNameICA_itr = &fCV_featureName[fCV_colorOffset+3];
      //std::cerr << ">>>Color Channel " << fCV_featureName[fCV_colorOffset+3]
      //        << " is On - Offset: " << colorOffset << "\n";
      ++featureNameICA_itr;
    }
    if(fCV_hueOn == true)
    {
      *featureNameICA_itr = &fCV_featureName[fCV_colorOffset+4];
      //std::cerr << ">>>Color Channel " << fCV_featureName[fCV_colorOffset+4]
      //                << " is On - Offset: " << colorOffset << "\n";
      ++featureNameICA_itr;
    }
    if(fCV_satOn == true)
    {
      *featureNameICA_itr = &fCV_featureName[fCV_colorOffset+5];
      //std::cerr << ">>>Color Channel " << fCV_featureName[fCV_colorOffset+5]
      //                << " is On - Offset: " << colorOffset << "\n";
      ++featureNameICA_itr;
    }
    if(fCV_valOn == true)
    {
      *featureNameICA_itr = &fCV_featureName[fCV_colorOffset+6];
      //std::cerr << ">>>Color Channel " << fCV_featureName[fCV_colorOffset+6]
      //        << " is On - Offset: " << colorOffset << "\n";
      ++featureNameICA_itr;
    }
    if(fCV_hue1On == true)
    {
      *featureNameICA_itr = &fCV_featureName[fCV_colorOffset+7];
      //std::cerr << ">>>Color Channel " << fCV_featureName[fCV_colorOffset+7]
      //        << " is On - Offset: " << colorOffset << "\n";
      ++featureNameICA_itr;
    }
    if(fCV_hue2On == true)
    {
      *featureNameICA_itr = &fCV_featureName[fCV_colorOffset+8];
      //std::cerr << ">>>Color Channel " << fCV_featureName[fCV_colorOffset+8]
      //                << " is On - Offset: " << colorOffset << "\n";
      ++featureNameICA_itr;
    }

    if(fCV_lowPassType == 5)
    {
      if(fCV_lowPassVector.size() < fCV_rmap.size())
      {
        PixRGB<FLOAT> tempish;
        fCV_lowPassVector.resize(fCV_rmap.size(),tempish);
      }
      filterAtLocationBatch(&fCV_realImageLowPass,&fCV_lowPassKernel,
                            &fCV_rmap,
                            &fCV_lowPassVector,fCV_countSM);
    }
    for(int i = 0; i < fCV_countSM; i++, ++space_itr)
    {
      // did we low pass the whole image or are we just using
      // band pass parts
      if(fCV_lowPassType == 5)
      {
        pix = fCV_lowPassVector[i];
        //LINFO("VALS %d %f %f %f",i,pix.r,pix.g,pix.b);
      }
      else if(fCV_lowPassType == 4)
        pix = filterAtLocation(&fCV_realImageLowPass,&fCV_lowPassKernel,
                               &*fCV_rmap[i]);
      else
        pix = fCV_realImageLowPass.getVal(fCV_rmap[i]->i,fCV_rmap[i]->j);

      space_itr_itr = space_itr->begin() + colorOffset;

      if(fCV_redOn == true)
      {
        *space_itr_itr = ((pix.red()+(*fCV_redTrans))*(*fCV_redNorm))
          * fCV_redWeight;
        ++space_itr_itr;
      }
      if(fCV_greenOn == true)
      {
        *space_itr_itr = ((pix.green()+(*fCV_greenTrans))*(*fCV_greenNorm))
          * fCV_greenWeight;
        ++space_itr_itr;
      }
      if(fCV_blueOn == true)
      {
        *space_itr_itr = ((pix.blue()+(*fCV_blueTrans))*(*fCV_blueNorm))
          * fCV_blueWeight;
        ++space_itr_itr;
      }
      if(fCV_yellowOn == true)
      {
        *space_itr_itr = (((((pix.red() + pix.green()) -
                             fabs(pix.red() - pix.green()))/2.0F)
                           +(*fCV_yellowTrans))
                          *(*fCV_yellowNorm))
          * fCV_yellowWeight;
        ++space_itr_itr;
      }

      FLOAT pixH = 0, pixS, pixV;
      FLOAT pixH1 = 0, pixH2 = 0;

      if(fCV_hueOn == true)
        PixHSV<FLOAT>(pix).getHSV(pixH,pixS,pixV);
      else
        pix.getHSV(pixH1,pixH2,pixS,pixV);
      // LINFO("HSV Fetched");
      // WEE
      if(fCV_hueOn == true)
      {
        *space_itr_itr = ((pixH+(*fCV_hueTrans))*(*fCV_hueNorm))
          * fCV_hueWeight;
        ++space_itr_itr;
      }
      if(fCV_satOn == true)
      {
        *space_itr_itr = ((pixS+(*fCV_satTrans))*(*fCV_satNorm))
          * fCV_satWeight;
        ++space_itr_itr;
      }
      if(fCV_valOn == true)
      {
        *space_itr_itr = ((pixV+(*fCV_valTrans))*(*fCV_valNorm))
          * fCV_valWeight;
        ++space_itr_itr;
      }
      if(fCV_hue1On == true)
      {
        *space_itr_itr = ((pixH1+(*fCV_hue1Trans))*(*fCV_hue1Norm))
          * fCV_hue1Weight;
        ++space_itr_itr;
      }
      if(fCV_hue2On == true)
      {
        *space_itr_itr = ((pixH2+(*fCV_hue2Trans))*(*fCV_hue2Norm))
          * fCV_hue2Weight;
        ++space_itr_itr;
      }
    }
    //LINFO("DONE");
  }

  // if we want to include spatial factors then copy those into the space map
  if(fCV_spatialOn == true)
  {
    //int s1 = fCV_space[0].size() - 2;
    //int s2 = fCV_space[0].size() - 1;
    //std::cerr << ">>>Spatial Channel "
    //        << " is On - Offset: " << s1 << "\n";
    //std::cerr << ">>>Spatial Channel "
    //        << " is On - Offset: " << s2 << "\n";

    space_itr = fCV_space.begin();
    *featureNameICA_itr = &fCV_featureName[fCV_spatOffset];
    fCV_featureNormConst[fCV_featureNormConst.size() - 2] =
      fCV_sizeXbias;
    fCV_featureNormConst[fCV_featureNormConst.size() - 1] =
      fCV_sizeYbias;
    ++featureNameICA_itr;

    *featureNameICA_itr = &fCV_featureName[fCV_spatOffset];
    for(int i = 0; i < fCV_countSM; i++, ++space_itr)
    {
      space_itr_itr = space_itr->begin() + fCV_space[0].size() - 2;
      *space_itr_itr = ((fCV_rmap[i]->i)
                        /(FLOAT)fCV_sizeX)*fCV_spatialWeight*fCV_sizeXbias;
      //LINFO("OUT X VAL = %f", *space_itr_itr);
      space_itr_itr = space_itr->begin() + fCV_space[0].size() - 1;
      *space_itr_itr = ((fCV_rmap[i]->j)
                        /(FLOAT)fCV_sizeY)*fCV_spatialWeight*fCV_sizeYbias;
      //LINFO("OUT Y VAL = %f", *space_itr_itr);
    }
  }


  // print reduced feature map if desired
  /*
  if(fCV_printOutFeatures == true)
  {
    Image<FLOAT> foo_ish;
    foo_ish  = fCV_space;
    Image<FLOAT> foo_me;
    foo_me = foo_ish;
    Raster::WriteGray(foo_me,sformat("%s.out.featureSetICA.pgm",fCV_fileName.c_str()));
  }
  */
}

// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVrunNPclassify()
{
  //---------------Cluster Space with NPclassify---------------//

  //std::cerr << "(3) RUNNING NPclassify\n";
  //LINFO("RESETTING SPACE %d x %d",fCV_countSM,fCV_space[0].size());
  fCV_NP.NPresetSpace(fCV_countSM,(fCV_space[0].size()));
  //LINFO("ADDING SPACE");
  fCV_NP.NPaddSpace(fCV_space);
  //fCV_NP.NPechoSpace();

  if(fCV_doNPbias == true)
    if(fCV_NPtemporalBias != 0.0F)
      fCV_NP.NPsetConvolveBias(fCV_covHolderLast,fCV_covDataSizeLast
                               ,fCV_NPtemporalBias);

  fCV_NP.NPclassifySpaceNew(fCV_doNPbias);
  if(DUMPNP == true)
    fCV_NP.NPdumpLinkInfo(fCV_fileName);
  //LINFO("NP DONE");
  std::vector<std::vector<int*> >* foo;
  foo = fCV_NP.NPgetClass();
  fCV_classList = foo;
  fCV_doNPbias = true;

  //if(fCV_printOutClusters == true)
  //  fCVprintOutClusters();
}

// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVrunCovEstimate()
{
  //---------------Find Covariances with covEstimate---------------//

  //std::cerr << "(4) FINDING covariance\n";
  for(int i = 0; i < fCV_NP.NPgetStemNumber(); i++)
  {
    if(fCV_NP.NPgetClassSize(i) > fCV_NP.NPgetMinClassSize())
    {
      // sort space into sub spaces by class, find the gauss for sub spaces
      for(int j = 0; j < fCV_NP.NPgetClassSize(i); j++)
      {
        for(int k = 0; k < (signed)fCV_space[0].size(); k++)
        {
          //long item = fCV_NP.NPgetClass(i,j);
          //std::cerr << i << " " << j << " " << k << "\n";
          /*
          fCV_sortedSpace[i][j][k]
            = &fCV_space[*fCV_classList->at(i)[j]][k];
          */
          fCV_sortedSpace[i][k][j] = &fCV_space[*fCV_classList->at(i)[j]][k];

        }
      }
      if(fCV_covDataSizeCurrent == fCV_covHolderCurrent->size())
      {
          fCV_covHolderCurrent->resize(fCV_covHolderCurrent->size()+COVSIZE
                                     ,fCV_tcov);
      }
      //std::cerr << "SETTING: covmatrix\n";
      // insert Data
      fCV_covHolderCurrent->at(fCV_covDataSizeCurrent).isLarge = true;
      fCV_CV.setNewF(fCV_sortedSpace[i],
                     0.0F,fCV_NP.NPgetClassSize(i)
                     ,(signed)fCV_space[0].size(),
                     fCV_covHolderCurrent->at(fCV_covDataSizeCurrent),
                     false);
      fCV_covHolderCurrent->at(fCV_covDataSizeCurrent).baseID = i;
      fCV_covDataSizeCurrent++;
      //fCV_CV.printDebug();
      // (1) find means in space
      //std::cerr << "FINDING mean\n";
      fCV_CV.run();

      //if(i == 0)
      //        fCVprintOutCovSlices(fCV_sizeX,fCV_sizeY);
      //fCV_CV.printEigenVals();
    }
  }
}

// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVcheckParticles()
{
  //LINFO("CHECKING PARTICLES");
  std::vector<FLOAT>* meanClassDensity = fCV_NP.NPgetMeanClassDensity();
  std::vector<FLOAT>* stdClassDensity = fCV_NP.NPgetStdClassDensity();
  std::vector<FLOAT>* density = fCV_NP.NPgetDensityPtr();

  for(int i = 0; i < fCV_NP.NPgetStemNumber(); i++)
  {
    if(fCV_NP.NPgetClassSize(i) > fCV_NP.NPgetMinClassSize())
    {
      FLOAT thresh = meanClassDensity->at(i) +
        (stdClassDensity->at(i)*fCV_densityBias);
      //LINFO("MEAN %f STD %f BIAS %f",meanClassDensity->at(i),
      //            stdClassDensity->at(i),fCV_densityBias);
      //LINFO("THRESH %f",thresh);
      // sort space into sub spaces by class, find the gauss for sub spaces
      for(int j = 0; j < fCV_NP.NPgetClassSize(i); j++)
      {
        //LINFO("CHECKING %d",fCV_NP.NPgetClass(i,j));
        //LINFO("THRESH %f",thresh);
        //LINFO("DENSITY %f",density->at(fCV_NP.NPgetClass(i,j)));
        if(density->at(fCV_NP.NPgetClass(i,j)) > thresh)
        {
          fCV_keepParticle[fCV_NP.NPgetClass(i,j)] = true;
        }
        else
        {
          fCV_keepParticle[fCV_NP.NPgetClass(i,j)] = false;
        }
      }
    }
  }
}

// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVmatchClassTemporal()
{
  //LINFO("CLASS MATCH TEMPORAL");
  fCV_sortClassSize.resize(fCV_NP.NPgetStemNumber()+1,0);
  fCV_sortClassMember.resize(fCV_NP.NPgetStemNumber()+1,0);
  fCV_sortCount = 0;

  // sort classes based upon size

  for(int i = 0; i < fCV_NP.NPgetStemNumber(); i++)
  {
    // if I am the first class, set me as number 1
    if(fCV_sortCount == 0)
    {
      //LINFO("SETTING first class at %d size %d",i,NP.NPgetClassSize(i));
      fCV_sortClassSize[0] = fCV_NP.NPgetClassSize(i);
      fCV_sortClassMember[0] = i;
      fCV_sortCount++;
    }
    else
    {
      bool setThis = false;
      // look through the entire list in order
      long initSC = fCV_sortCount;
      for(long j = 0; j < initSC; j++)
      {
        // if I am bigger than someone, bump him and
        // everyone else back one, insert me
        if(fCV_NP.NPgetClassSize(i) > fCV_sortClassSize[j])
        {
          setThis = true;
          long tempClassSize;
          long tempClassNum;
          long newClassSize = fCV_NP.NPgetClassSize(i);
          long newClassNum = i;
          for(int k = j; k <= fCV_sortCount; k++)
          {
            tempClassSize = fCV_sortClassSize[k];
            tempClassNum = fCV_sortClassMember[k];
            fCV_sortClassSize[k] = newClassSize;
            fCV_sortClassMember[k] = newClassNum;
            newClassSize = tempClassSize;
            newClassNum = tempClassNum;
          }
          break;
        }
      }
      if(setThis == false)
      {
        fCV_sortClassSize[fCV_sortCount] = fCV_NP.NPgetClassSize(i);
        fCV_sortClassMember[fCV_sortCount] = i;
      }
      fCV_sortCount++;
    }
  }

  //LINFO("SETTING COVHOLDER");
  for(int i = 0; i < fCV_sortCount; i++)
  {
    long *thisClass = &fCV_sortClassMember[i];
    //LINFO("THIS CLASS %d",*thisClass);
    fCV_covHolderCurrent->at(*thisClass).sortID = i;
  }

  //LINFO("MATCHING ID");
  // set match ID at first iteration
  if(fCV_doMatchSelf == true)
  {
    for(unsigned int i = 0; i < fCV_covDataSizeCurrent; i++)
    {
      //LINFO("MATCHING SELF (real) %d",i);
      //LINFO("Size %d",fCV_covHolderCurrent->at(i).samples);
      fCV_covHolderCurrent->at(i).matchID = i;
    }
    fCV_doMatchSelf = false;
  }
  else
  {
    //LINFO("CALLING MATCH PMEAN new size %d, old size %d",
    //    fCV_covDataSizeCurrent,fCV_covDataSizeMatch);
    fCV_CV.matchPmeanAccum(fCV_covHolderCurrent,&fCV_covDataSizeCurrent,
                      &fCV_covHolderMatch,&fCV_covDataSizeMatch,
                      fCV_NP.NPgetMinClassSize());
  }
}

// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVsetImageParams()
{
  for(unsigned long i = 0; i < fCV_covDataSizeCurrent; i++)
  {
    //LINFO("%d : SIZE %d",i,fCV_covHolderCurrent->at(i).samples);
    //LINFO("%d : MATCHED at %d",i,
    //    fCV_covHolderCurrent->at(i).matchID);

    //unsigned int current = fCV_covHolderCurrent->at(i).matchID;
    if(fCV_covHolderCurrent->at(i).isLarge)
    {
      unsigned int X = 0;
      unsigned int Y = 0;
      unsigned int minX = fCV_sizeX;
      unsigned int minY = fCV_sizeY;
      unsigned int maxX = 0;
      unsigned int maxY = 0;

      for(int j = 0;
          j < fCV_NP.NPgetClassSize(fCV_covHolderCurrent->at(i).baseID); j++)
      {

        long item = fCV_NP.NPgetClass(fCV_covHolderCurrent->at(i).baseID
                                      ,j);

        int ii = (int)((fCV_NP.NPgetFeature(item,fCV_space[0].size()-2)
                        /(fCV_spatialWeight*fCV_sizeXbias))*fCV_sizeX);
        X += ii;
        if((unsigned)ii < minX) minX = (unsigned)ii;
        if((unsigned)ii > maxX) maxX = (unsigned)ii;

        int jj = (int)((fCV_NP.NPgetFeature(item,fCV_space[0].size()-1)
                        /(fCV_spatialWeight*fCV_sizeYbias))*fCV_sizeY);

        Y += jj;
        if((unsigned)jj < minY) minY = (unsigned)jj;
        if((unsigned)jj > maxY) maxY = (unsigned)jj;
      }
      fCV_covHolderCurrent->at(i).posX =
        X/fCV_NP.NPgetClassSize(fCV_covHolderCurrent->at(i).baseID);
      fCV_covHolderCurrent->at(i).posY =
        Y/fCV_NP.NPgetClassSize(fCV_covHolderCurrent->at(i).baseID);
      fCV_covHolderCurrent->at(i).maxX = maxX;
      fCV_covHolderCurrent->at(i).minX = minX;
      fCV_covHolderCurrent->at(i).maxY = maxY;
      fCV_covHolderCurrent->at(i).minY = minY;
    }
  }
}

// ######################################################################
// ######################################################################
// CALL ALL IMPORTANT METHODS IN ORDER
// This is the main execution path
// ######################################################################
// ######################################################################

template <class FLOAT>
void featureClusterVision<FLOAT>::fCVclusterImage()
{
  std::ofstream timerFile("./clusterTimer.txt",std::ios::app);
  Timer tim;
  tim.reset();
  int t1,t2,t3;
  int t0 = tim.get();  // to measure display time
  // (-1) switch the pointers between frames for i and i-1
  LINFO("SWITCH COV");
  fCVswitchCov();
  /////t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  /////t3 = t2 - t3;
  std::cerr << "\t>>>>>>>> fCV (SWITCH COV) TIME: " << t2 << "ms\n";
  if(fCV_useTimerFile == true)
    timerFile << t2 << "\t";
  // (0) Low Pass the real image if needed for color extraction
  LINFO("LOW PASSING IMAGE");
  fCVlowPass();
  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t>>>>>>>> fCV (LOW PASS) TIME: " << t2 << "ms "
            << " SLICE " << t3 << "ms\n";
  if(fCV_useTimerFile == true)
    timerFile << t2 << "\t" << t3 << "\t";
  // (1) get features from saliency map using MC method
  LINFO("FIND FEATURES");
  if(fCV_useBrain == true)
    fCVfindFeaturesBrain();
  else
    fCVfindFeaturesNoBrain();
  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t>>>>>>>> fCV (FIND FEATURES) TIME: " << t2 << "ms "
            << " SLICE " << t3 << "ms\n";
  if(fCV_useTimerFile == true)
    timerFile << t2 << "\t" << t3 << "\t";
  // (2) Mix orientation and motion channles into rotation invariant info
  LINFO("FIND MIXED CHANNELS");
  fCVfindMixedChannels();
  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t>>>>>>>> fCV (FIND MIXED CHAN) TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";
  if(fCV_useTimerFile == true)
    timerFile << t2 << "\t" << t3 << "\t";
  // (3) Reduce and purify data with ICA/PCA
  LINFO("RUN ICA");
  fCVrunICA();
  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t>>>>>>>> fCV (RUN ICA) TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";
  if(fCV_useTimerFile == true)
    timerFile << t2 << "\t" << t3 << "\t";
  // (4) cluster data sets
  LINFO("RUN NPCLASSIFY");
  fCVrunNPclassify();
  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t>>>>>>>> fCV (NP CLASSIFY) TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";
  if(fCV_useTimerFile == true)
    timerFile << t2 << "\t" << t3 << "\t";
  // (5) find statistical relevance
  LINFO("RUN COVESTIMATE");
  fCVrunCovEstimate();
  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;;
  std::cerr << ".\t>>>>>>>> fCV (COV ESTIMATE) TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";
  if(fCV_useTimerFile == true)
    timerFile << t2 << "\t" << t3 << "\t";
  // (6) process clusters, sort by size and match between iterations
  LINFO("RUN MATCH CLASS TEMPORAL");
  fCVmatchClassTemporal();
  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t>>>>>>>> fCV (MATCH CLASS TEMPORAL) TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";
  if(fCV_useTimerFile == true)
    timerFile << t2 << "\t" << t3 << "\t";
  LINFO("FIND IMAGE PARAMETERS");
  fCVsetImageParams();
  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t>>>>>>>> fCV (FIND IMAGE PARAMETERS) TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";
  if(fCV_useTimerFile == true)
    timerFile << t2 << "\t" << t3 << "\t";
  // (x) print out the basic stuff that we found
  LINFO("PRINT CLUSTERS");
  fCVprintOutClusters();
  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t>>>>>>>> fCV (PRINT CLUSTERS) TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";
  if(fCV_useTimerFile == true)
    timerFile << t2 << "\t" << t3 << "\t";
  // (7) Check with features that were extracted should be re-used
  LINFO("CHECK PARTICLES");
  fCVcheckParticles();
  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t>>>>>>>> fCV (CHECK PARTICLES) TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";
  if(fCV_useTimerFile == true)
    timerFile << t2 << "\t" << t3 << "\t";
  // (8) reset the NPclassifier
  LINFO("RESET SPACE");
  fCV_NP.NPresetSpace();
  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t>>>>>>>> fCV (RESET) TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";
  if(fCV_useTimerFile == true)
  {
    timerFile << t2 << "\t" << t3 << "\n";
    timerFile.close();
  }

  std::cerr << "DONE\n";

}

// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVsaccadeTest(std::string _maskFile,
                                                 std::string _outFile,
                                                 std::string _label,
                                                 std::string _fileName)
{

  Timer tim;
  tim.reset();
  int t1,t2,t3;
  int t0 = tim.get();  // to measure display time
  // (-1) switch the pointers between frames for i and i-1
  LINFO("SWITCH COV");
  fCVswitchCov();
  /////t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  /////t3 = t2 - t3;
  std::cerr << "\t>>>>>>>> fCV (SWITCH COV) TIME: " << t2 << "ms\n";
  // (0) Low Pass the real image if needed for color extraction
  LINFO("LOW PASSING IMAGE");
  fCVlowPass();
  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t>>>>>>>> fCV (LOW PASS) TIME: " << t2 << "ms "
            << " SLICE " << t3 << "ms\n";
  // (1) get features from saliency map using MC method
  LINFO("FIND FEATURES");
  fCVfindFeaturesFromFile(_fileName);
  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t>>>>>>>> fCV (FIND FEATURES) TIME: " << t2 << "ms "
            << " SLICE " << t3 << "ms\n";
  // (2) Mix orientation and motion channles into rotation invariant info
  LINFO("FIND MIXED CHANNELS");
  fCVfindMixedChannels();
  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t>>>>>>>> fCV (FIND MIXED CHAN) TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";
  // (3) Reduce and purify data with ICA/PCA
  LINFO("RUN ICA");
  fCVrunICA();
  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t>>>>>>>> fCV (RUN ICA) TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";
  // (4) dump out raw data
  LINFO("DUMP FEATUES AND SACCADE DATA");
  fCVprocessOutSaccadeData(_maskFile,_outFile,_label);
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t>>>>>>>> fCV (DUMP DATA) TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";

  std::cerr << "DONE\n";

}
// ######################################################################

template <class FLOAT>
void featureClusterVision<FLOAT>::fCVstandAloneFeatureTest(std::string _fileName)
{
  std::ofstream timerFile("./clusterTimer.txt",std::ios::app);
  Timer tim;
  tim.reset();
  int t1,t2,t3;
  int t0 = tim.get();  // to measure display time
  // (-1) switch the pointers between frames for i and i-1
  LINFO("SWITCH COV");
  fCVswitchCov();
  ///  t3 = t2; // what is that supposed to do?
  t1 = tim.get();
  t2 = t1 - t0;
  ///t3 = t2 - t3; // and what about that one??
  std::cerr << "\t>>>>>>>> fCV (SWITCH COV) TIME: " << t2 << "ms\n";
  if(fCV_useTimerFile == true)
    timerFile << t2 << "\t";
  // (0) Low Pass the real image if needed for color extraction
  LINFO("LOW PASSING IMAGE");
  fCVlowPass();
  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t>>>>>>>> fCV (LOW PASS) TIME: " << t2 << "ms "
            << " SLICE " << t3 << "ms\n";
  if(fCV_useTimerFile == true)
    timerFile << t2 << "\t" << t3 << "\t";
  // (1) get features from saliency map using MC method
  LINFO("FIND FEATURES");
  if(fCV_useBrain == true)
    fCVfindFeaturesBrain();
  else
    fCVfindFeaturesNoBrain();
  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t>>>>>>>> fCV (FIND FEATURES) TIME: " << t2 << "ms "
            << " SLICE " << t3 << "ms\n";
  if(fCV_useTimerFile == true)
    timerFile << t2 << "\t" << t3 << "\t";
  // (2) Mix orientation and motion channles into rotation invariant info
  LINFO("FIND MIXED CHANNELS");
  fCVfindMixedChannels();
  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t>>>>>>>> fCV (FIND MIXED CHAN) TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";
  if(fCV_useTimerFile == true)
    timerFile << t2 << "\t" << t3 << "\t";
  // (2) Mix orientation and motion channles into rotation invariant info
  LINFO("FIND MIXED CHANNELS");
  //fCVrunStandAloneMSBatchTest(_fileName);
  fCVrunStandAloneMSBatchFilter(_fileName);
  t3 = t2;
  t1 = tim.get();
  t2 = t1 - t0;
  t3 = t2 - t3;
  std::cerr << "\t>>>>>>>> fCV (RUN STAND ALONE) TIME: "<< t2 << "ms "
            << " SLICE " << t3 << "ms\n";
  if(fCV_useTimerFile == true)
    timerFile << t2 << "\t" << t3 << "\t";

}

// ######################################################################
// ######################################################################
// MORE TEST METHODS
// ######################################################################
// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVprintOutClusters()
{
  //LINFO("Draw Class Images");

  findColorIndex FAC;
  unsigned int zero = 0;

  PixRGB<FLOAT> mrPixel;
  FAC.FACgetColor12(&zero,&mrPixel);

  PixRGB<FLOAT> mrPixelMatch;
  FAC.FACgetColor12(&zero,&mrPixelMatch);

  PixRGB<FLOAT> mrPixelOver;
  FAC.FACgetColor12(&zero,&mrPixelOver);

  fCV_outImageClasses  = fCV_realImage;
  fCV_outImageTemporal = fCV_realImage;
  fCV_outImageTarget   = fCV_realImage;

  //LINFO("SETTING TEMPORAL");
  for(unsigned long i = 0; i < fCV_covDataSizeCurrent; i++)
  {
    //LINFO("%d : SIZE %d",i,fCV_covHolderCurrent->at(i).samples);
    //LINFO("%d : MATCHED at %d",i,
    //  fCV_covHolderCurrent->at(i).matchID);
    unsigned int current = fCV_covHolderCurrent->at(i).matchID;
    if(fCV_covHolderCurrent->at(i).isLarge)
    {
      bool small = true;
      if(current < 12)
        FAC.FACgetColor12(&current,&mrPixelMatch);
      else
      {
        small = false;
        current = current - 12;
        FAC.FACgetColor12(&current,&mrPixelMatch);
      }
      for(int j = 0;
          j < fCV_NP.NPgetClassSize(fCV_covHolderCurrent->at(i).baseID); j++)
      {
        long item = fCV_NP.NPgetClass(fCV_covHolderCurrent->at(i).baseID
                                      ,j);
        int ii = (int)((fCV_NP.NPgetFeature(item,fCV_space[0].size()-2)
                        /(fCV_spatialWeight*fCV_sizeXbias))*fCV_sizeX);
        int jj = (int)((fCV_NP.NPgetFeature(item,fCV_space[0].size()-1)
                        /(fCV_spatialWeight*fCV_sizeYbias))*fCV_sizeY);

        if(small == true)
          drawCircle(fCV_outImageTemporal, Point2D<int>(ii,jj),2,mrPixelMatch,2);
        else
          drawCircle(fCV_outImageTemporal, Point2D<int>(ii,jj),3,mrPixelMatch,3);

        if(j == 0)
        {
          char foo2;
          sprintf(&foo2,"%d",(int)fCV_covHolderCurrent->at(i).matchID);
          writeText(fCV_outImageTemporal, Point2D<int>(ii,jj),&foo2,
                    PixRGB<FLOAT>(255),PixRGB<FLOAT>(0));
        }
      }
    }
  }
  //Raster::VisuRGB(fCV_outImageTemporal,"FOOish.ppm");
  //*******************************************************
  // draw output images

  //LINFO("Creating Cluster Visual Output");

  for(int i = 0; i < fCV_sortCount; i++)
  {
    unsigned int number;
    long *thisClass = &fCV_sortClassMember[i];
    if(fCV_NP.NPgetClassSize(*thisClass) > fCV_NP.NPgetMinClassSize())
    {
      number = (unsigned)(i + 1);
      //LINFO("SETTING PIXEL %d",number);
      FAC.FACgetColor12(&number,&mrPixel);
      unsigned int current = fCV_covHolderCurrent->at(*thisClass).matchID;
      FAC.FACgetColor12(&current,&mrPixelMatch);

      Image<PixRGB<FLOAT> > outputClass;
      outputClass.resize(fCV_sizeX,fCV_sizeY);
      //std::cerr << "class size " <<  fCV_NP.NPgetClassSize(*thisClass) << "\n";
      for(int j = 0; j < fCV_NP.NPgetClassSize(*thisClass); j++)
      {

        // --- NOTE:
        // NPgetFeature = ((fCV_rmap[i]->i)
        // /(FLOAT)fCV_sizeX)*fCV_spatialWeight;

        long item = fCV_NP.NPgetClass(*thisClass,j);

        int ii = (int)((fCV_NP.NPgetFeature(item,fCV_space[0].size()-2)
                        /(fCV_spatialWeight*fCV_sizeXbias))*fCV_sizeX);
        int jj = (int)((fCV_NP.NPgetFeature(item,fCV_space[0].size()-1)
                        /(fCV_spatialWeight*fCV_sizeYbias))*fCV_sizeY);

        drawCircle(fCV_outImageClasses, Point2D<int>(ii,jj),2,mrPixel,2);

        if(j == 0)
        {
          if(fCV_NP.NPgetMinClassSize() <= fCV_NP.NPgetClassSize(*thisClass))
          {
            char foo;
            sprintf(&foo,"%d",(int)*thisClass);
            writeText(fCV_outImageClasses, Point2D<int>(ii,jj),&foo,
                      PixRGB<FLOAT>(255),PixRGB<FLOAT>(0));
          }
        }
      }
    }
  }


  for(unsigned long i = 0; i < fCV_covDataSizeCurrent; i++)
  {
    if(fCV_covHolderCurrent->at(i).isLarge)
    {
      unsigned int current = fCV_covHolderCurrent->at(i).matchID;
      bool small = true;
      if(current < 12)
        FAC.FACgetColor12(&current,&mrPixelMatch);
      else
      {
        small = false;
        current = current - 12;
        FAC.FACgetColor12(&current,&mrPixelMatch);
      }

      drawCross(fCV_outImageTarget,
                Point2D<int>(fCV_covHolderCurrent->at(i).posX,
                        fCV_covHolderCurrent->at(i).posY
                        ),mrPixelMatch,20,2);
      if(small == true)
        drawCross(fCV_outImageTarget,
                  Point2D<int>(fCV_covHolderCurrent->at(i).posX,
                          fCV_covHolderCurrent->at(i).posY
                          ),PixRGB<float>(0,0,0),20,1);
      else
        drawCross(fCV_outImageTarget,
                  Point2D<int>(fCV_covHolderCurrent->at(i).posX,
                          fCV_covHolderCurrent->at(i).posY
                          ),PixRGB<float>(255,255,255),20,1);
      /*
      drawRect(matchMap, Rectangle::tlbrI(fCV_covHolderCurrent->at(i).maxY,
                                  fCV_covHolderCurrent->at(i).minX,
                                  fCV_covHolderCurrent->at(i).minY,
                                  fCV_covHolderCurrent->at(i).maxX),
                        mrPixelMatch,2);
      drawRect(realImage, Rectangle::tlbrI(fCV_covHolderCurrent->at(i).maxY,
                                   fCV_covHolderCurrent->at(i).minX,
                                   fCV_covHolderCurrent->at(i).minY,
                                   fCV_covHolderCurrent->at(i).maxX),
                         mrPixelMatch,2);
      */
    }
  }
}

// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVprintOutCovSlices(int sizeX, int sizeY)
{
  LINFO("PRINT COV Slices");
  Image<FLOAT> outImage;
  int isize;
  if(fCV_sizeX > fCV_sizeY)
    isize = fCV_sizeX;
  else
    isize = fCV_sizeY;
  for(int k = 0; k < (signed)fCV_space[0].size(); k++)
  {
    outImage.resize(isize,isize,true);
    outImage.resize(isize,isize);
    //outImage.resize(sizeX,sizeY);
    if(k == ((signed)fCV_space[0].size()-1))
    {
      outImage = fCV_CV.returnCovSlice(0,k,outImage,true);
    }
    else
    {
      outImage = fCV_CV.returnCovSlice(k,k+1,outImage,true);
    }
    Raster::WriteGray(outImage,
                      sformat("%s.out.covSlice.%d.pgm"
                              ,fCV_fileName.c_str(),k));
  }
}

#if 0
// FIXME these functions doesn't compile cleanly anymore because
// covEstimate<double>::getP() and getD() expect a
// std::vector<double>, but we're trying to pass a std::vector<float>
// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVprintOutBayesClass()
{
  LINFO("PRINT BAYES CLASSES");
  findColorIndex FAC;
  typename std::vector<FLOAT> ptemp;
  ptemp.resize(fCV_covDataSizeCurrent,0.0F);
  typename std::vector<std::vector<FLOAT> > pVal;
  pVal.resize(fCV_countSM,ptemp);
  std::vector<int> classMember;
  classMember.resize(fCV_countSM,-1);
  typename std::vector<FLOAT> classWinner;
  classWinner.resize(fCV_countSM,0.0F);
  //LINFO("FINDING P VALS");
  for(int i = 0; i < fCV_countSM; i++)
  {
    for(unsigned int k = 0; k < fCV_covDataSizeCurrent; k++)
    {
      // LINFO("%d %d",fCV_space[i].size(),fCV_covHolderCurrent->at(k).mean.size());
      //LINFO("%d %d",i,k);
      pVal[i][k] = fCV_CV.getP(fCV_space[i],fCV_covHolderCurrent->at(k),2);
      //LINFO("PVAL %d,%d %f",i,k,pVal[i][k]);
      if(classMember[i] == -1)
      {
        classMember[i] = k;
        classWinner[i] = pVal[i][k];
      }
      else
        if(pVal[i][k] > classWinner[i])
        {
          classWinner[i] = pVal[i][k];
          classMember[i] = k;
        }
    }
  }
  //LINFO("DRAW IMAGE");
  Image<PixRGB<FLOAT> > realImage;
  Image<PixRGB<FLOAT> > blankImage;
  realImage = fCV_realImage;
  blankImage.resize(realImage.getWidth(),realImage.getHeight(),0.0F);
  PixRGB<FLOAT> mrPixel;
  unsigned int zero = 0;
  FAC.FACgetColor12(&zero,&mrPixel);
  for(int i = 0; i < fCV_countSM; i++)
  {
    unsigned int current = classMember[i];
    int sz = 2;
    if(current >= 12)
    {
      sz = 3;
      unsigned int c = current - 12;
      FAC.FACgetColor12(&c,&mrPixel);
    }
    else
      FAC.FACgetColor12(&current,&mrPixel);
    Point2D<int> P;
    P = *fCV_rmap[i];
    drawCircle(realImage, P,sz,mrPixel,2);
    drawCircle(blankImage, P,sz,mrPixel,2);
  }
  Raster::WriteRGB(realImage,sformat("%s.out.CLASS_MAP_BAYES.ppm"
                                         ,fCV_fileName.c_str()));
  Raster::WriteRGB(blankImage,sformat("%s.out.BLANK_MAP_BAYES.ppm"
                                          ,fCV_fileName.c_str()));
  //LINFO("BAYES DONE");
}

// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVprintOutNeighborClass()
{
  LINFO("PRINT Neighbor classes");
  findColorIndex FAC;
  typename std::vector<FLOAT> ptemp;
  ptemp.resize(fCV_covDataSizeCurrent,0.0F);
  std::vector<std::vector<FLOAT> > pVal;
  pVal.resize(fCV_countSM,ptemp);
  std::vector<int> classMember;
  classMember.resize(fCV_countSM,-1);
  typename std::vector<FLOAT> classWinner;
  classWinner.resize(fCV_countSM,0.0F);
  //LINFO("FINDING P VALS");
  for(int i = 0; i < fCV_countSM; i++)
  {
    for(unsigned int k = 0; k < fCV_covDataSizeCurrent; k++)
    {
      //LINFO("%d %d",fCV_space[i].size(),fCV_covHolder[k].mean.size());
      pVal[i][k] = fCV_CV.getD(fCV_space[i],fCV_covHolderCurrent->at(k),2);
      //LINFO("PVAL %d,%d %f",i,k,pVal[i][k]);
      if(classMember[i] == -1)
      {
        classMember[i] = k;
        classWinner[i] = pVal[i][k];
      }
      else
        if(pVal[i][k] < classWinner[i])
        {
          classWinner[i] = pVal[i][k];
          classMember[i] = k;
        }
    }
  }

  Image<PixRGB<FLOAT> > realImage;
  Image<PixRGB<FLOAT> > blankImage;
  realImage = fCV_realImage;
  blankImage.resize(realImage.getWidth(),realImage.getHeight(),0.0F);
  PixRGB<FLOAT> mrPixel;

  for(int i = 0; i < fCV_countSM; i++)
  {
    unsigned int current = classMember[i];
    FAC.FACgetColor12(&current,&mrPixel);
    Point2D<int> P;
    P = *fCV_rmap[i];
    drawCircle(realImage, P,2,mrPixel,2);
    drawCircle(blankImage, P,2,mrPixel,2);
  }

  Raster::WriteRGB(realImage,sformat("%s.out.CLASS_MAP_NEIGH.ppm"
                                         ,fCV_fileName.c_str()));
  Raster::WriteRGB(blankImage,sformat("%s.out.BLANK_MAP_NEIGH.ppm"
                                          ,fCV_fileName.c_str()));
  //LINFO("NEIGHBOR DONE");
}
#endif

// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVdumpCovMatrix(std::string fileName)
{
  for(unsigned int i = 0; i < fCV_covDataSizeCurrent; i++)
  {
    fCV_CV.dumpMatrix(fileName,fCV_covHolderCurrent->at(i),i,fCV_fileName);
  }
}

// ######################################################################
template <class FLOAT>
std::vector<covHolder<double> > featureClusterVision<FLOAT>::fCVgetCovHolders()
{
  return *fCV_covHolderCurrent;
}

// ######################################################################
template <class FLOAT>
unsigned int featureClusterVision<FLOAT>::fCVgetCovHolderSize()
{
  return fCV_covDataSizeCurrent;
}

// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVgetClusterImages(
                         Image<PixRGB<FLOAT> > *classImage,
                         Image<PixRGB<FLOAT> > *temporalImage,
                         Image<PixRGB<FLOAT> > *targetImage,
                         Image<FLOAT> *salMap)
{
  *classImage    = fCV_outImageClasses;
  *temporalImage = fCV_outImageTemporal;
  *targetImage   = fCV_outImageTarget;
  *salMap        = fCV_salmapLowPass;
}

// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVprocessOutSaccadeData(std::string _maskFile,
                                                    std::string _outFile,
                                                    std::string _label)
{
  Image<FLOAT> maskFile;
  Image<FLOAT> maskFileLP;
  std::ofstream outFile(_outFile.c_str(),std::ios::app);
  maskFile = Raster::ReadGray(_maskFile, RASFMT_PNM);
  Image<FLOAT> salmap;//////////////// = fCV_SM->getV(false);
  LFATAL("FIXME");
  salmap = rescaleBilinear(salmap,fCV_sizeX,fCV_sizeY);
  LINFO("DUMPING VALUES");

  // mask and real may not be the same size, then center
  //int maskOffset = (maskFile.getHeight() - fCV_realImage.getHeight())/2;
  maskFileLP = lowPass9(maskFile);
  unsigned int mwidth = (unsigned)maskFile.getWidth();
  FLOAT maxDist = sqrt(pow((FLOAT)fCV_sizeX,2)+pow((FLOAT)fCV_sizeY,2));
  //PixRGB<FLOAT> mrPixel;
  //mrPixel.setRed(255); mrPixel.setGreen(0); mrPixel.setBlue(255);
  //Image<PixRGB<FLOAT> > realImage = fCV_realImage;

  for(unsigned int i = 0; i < (unsigned)fCV_countSM; i++)
  {
    // Dump saccade placement
    outFile << _label             << "\t" << fCV_indexNumber[i] << "\t";
    outFile << fCV_cmap[i].i      << "\t" << fCV_cmap[i].j      << "\t";
    outFile << fCV_jumpTo[i].i    << "\t" << fCV_jumpTo[i].j    << "\t";
    // Dump Mask Value

    typename Image<FLOAT>::iterator maskFileItr = maskFile.beginw();
    FLOAT distance = 0;
    FLOAT minDist  = maxDist;
    unsigned int ii = 0;

    // how far is this coord from a mask coord
    while(maskFileItr != maskFile.endw())
    {
      if(*maskFileItr != 0.0F)
      {
        distance = sqrt(pow((FLOAT)(ii % mwidth) - (FLOAT)fCV_cmap[i].i,2) +
                        pow((FLOAT)(ii / mwidth) - (FLOAT)fCV_cmap[i].j,2));
        if(distance < minDist) minDist = distance;
      }
      ++maskFileItr; ++ii;
    }
    //mrPixel.setBlue(255.0F - (minDist/maxDist)*255.0F);
    //mrPixel.setRed(255.0F - (minDist/maxDist)*255.0F);
    //drawCircle(realImage, fCV_cmap[i],3,mrPixel,2);

    FLOAT masky = maskFileLP.getVal(fCV_cmap[i].i,fCV_cmap[i].j);
    LINFO("MASK VALUE %f d %f at %dx%d",masky,minDist,fCV_cmap[i].i,
          fCV_cmap[i].j);

    if(masky >= .000001F)

      outFile << "1\t";
    else
      outFile << "0\t";

    outFile << minDist << "\t";

    // dump saliency map value
    FLOAT saly = salmap.getVal(fCV_cmap[i].i,fCV_cmap[i].j);
    outFile << saly << "\t";

    outFile << "XXXXX\t";

    // Dump Raw Feature Values
    for(unsigned int j = 0; j < fCV_fmap[i].size(); j++)
    {
      outFile << fCV_fmap[i][j] << "\t";
    }

    outFile << "XXXXX\t";

    // Dump processed feature Data
    for(unsigned int j = 0; j < fCV_space[i].size(); j++)
    {
      outFile << fCV_space[i][j] << "\t";
    }

    outFile << "\n";
  }
  outFile.close();

  // Raster::WriteRGB(realImage,sformat("%s.%s.out.eyeDataOver.ppm",_label.c_str()
  //               ,_outFile.c_str()));

}


// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVgetImageBaseStats(std::string _maskFile,
                                                std::string _imageFile,
                                                std::string _outFile,
                                                std::string _label)
{
  LINFO("GETTING BASE STATS");
  LINFO("MASK FILE  %s",_maskFile.c_str());
  LINFO("IMAGE FILE %s",_imageFile.c_str());
  Image<FLOAT> maskFile;
  Image<PixRGB<FLOAT> > imageFile;
  maskFile = Raster::ReadGray(_maskFile, RASFMT_PNM);
  imageFile = Raster::ReadRGB(_imageFile, RASFMT_PNM);
  FLOAT maskOffset = (maskFile.getHeight() - imageFile.getHeight())/2;
  // color stats
  // colors on the target
  FLOAT H1onMean, H2onMean, SonMean, VonMean;
  FLOAT H1onSTD, H2onSTD, SonSTD, VonSTD;
  FLOAT H1onSum = 0; FLOAT H2onSum = 0;
  FLOAT SonSum  = 0; FLOAT VonSum  = 0;
  double H1onSS  = 0; double H2onSS  = 0;
  double SonSS   = 0; double VonSS   = 0;
  // colors off the target
  FLOAT H1offMean, H2offMean, SoffMean, VoffMean;
  FLOAT H1offSTD, H2offSTD, SoffSTD, VoffSTD;
  FLOAT H1offSum = 0; FLOAT H2offSum = 0;
  FLOAT SoffSum  = 0; FLOAT VoffSum  = 0;
  double H1offSS  = 0; double H2offSS  = 0;
  double SoffSS   = 0; double VoffSS   = 0;
  // total colors
  FLOAT H1Mean, H2Mean, SMean, VMean;
  FLOAT H1STD, H2STD, SSTD, VSTD;
  FLOAT H1Sum = 0; FLOAT H2Sum = 0;
  FLOAT SSum  = 0; FLOAT VSum  = 0;
  double H1SS  = 0; double H2SS  = 0;
  double SSS   = 0; double VSS   = 0;
  // mask stats
  FLOAT maskMeanX, maskMeanY, maskSTDX, maskSTDY;
  FLOAT maskSumX = 0; FLOAT maskSumY = 0;
  double maskSSX  = 0; double maskSSY  = 0;
  unsigned long maskMass    = 0;
  unsigned long nonMaskMass = 0;
  unsigned long totalMass   = 0;

  FLOAT pixH1, pixH2, pixS, pixV;
  // first find basic mask stats
  for(unsigned int x = 0; x < (unsigned)imageFile.getWidth(); x++)
  {
    for(unsigned int y = 0; y < (unsigned)imageFile.getHeight(); y++)
    {
      //LINFO("Coords %dx%d",x,(int)(y+maskOffset));
      FLOAT maskVal = maskFile.getVal(x,(int)(y+maskOffset));
      // on mask stats
      if(maskVal >= 0.000001F)
      {
        maskMass++;
      }
      else
      {
        nonMaskMass++;
      }
    }
  }

  totalMass = maskMass + nonMaskMass;

  for(unsigned int x = 0; x < (unsigned)imageFile.getWidth(); x++)
  {
    for(unsigned int y = 0; y < (unsigned)imageFile.getHeight(); y++)
    {
      PixRGB<FLOAT> thisColor = imageFile.getVal(x,y);
      FLOAT maskVal = maskFile.getVal(x,(int)(y+maskOffset));
      thisColor.getHSV(pixH1,pixH2,pixS,pixV);

      // on mask stats
      if(maskVal >= 0.000001F)
      {
        // SUM
        maskSumX += x;     maskSumY += y;
        H1onSum  += pixH1; H2onSum  += pixH2;
        SonSum   += pixS;  VonSum   += pixV;
        // SUM of SQUARES
        maskSSX += pow((FLOAT)x,2)/maskMass;
        maskSSY += pow((FLOAT)y,2)/maskMass;
        H1onSS  += pow(pixH1,2)   /maskMass;
        H2onSS  += pow(pixH2,2)   /maskMass;
        SonSS   += pow(pixS,2)    /maskMass;
        VonSS   += pow(pixV,2)    /maskMass;
      }
      else
      {
        // off mask stats
        // SUM
        H1offSum += pixH1; H2offSum += pixH2;
        SoffSum  += pixS;  VoffSum  += pixV;
        // SUM of SQUARES
        H1offSS  += pow(pixH1,2)/nonMaskMass;
        H2offSS  += pow(pixH2,2)/nonMaskMass;
        SoffSS   += pow(pixS,2) /nonMaskMass;
        VoffSS   += pow(pixV,2) /nonMaskMass;
      }
      // total stats
      // SUM
      H1Sum += pixH1; H2Sum += pixH2;
      SSum  += pixS;  VSum  += pixV;
      // SUM of SQUARES
      H1SS  += pow(pixH1,2)/totalMass;
      H2SS  += pow(pixH2,2)/totalMass;
      SSS   += pow(pixS,2) /totalMass;
      VSS   += pow(pixV,2) /totalMass;
    }
  }

  // Compute all final stats for this image

  // MASK STATS
  maskMeanX = maskSumX/maskMass; maskMeanY = maskSumY/maskMass;
  H1onMean  = H1onSum /maskMass; H2onMean  = H2onSum /maskMass;
  SonMean   = SonSum  /maskMass; VonMean   = VonSum  /maskMass;
  maskSTDX  = sqrt(maskSSX - pow(maskMeanX,2));
  maskSTDY  = sqrt(maskSSY - pow(maskMeanY,2));
  H1onSTD   = sqrt(H1onSS  - pow(H1onMean,2));
  H2onSTD   = sqrt(H2onSS  - pow(H2onMean,2));
  SonSTD    = sqrt(SonSS   - pow(SonMean,2));
  VonSTD    = sqrt(VonSS   - pow(VonMean,2));

  // NON MASK STATS
  H1offMean  = H1offSum /nonMaskMass; H2offMean  = H2offSum /nonMaskMass;
  SoffMean   = SoffSum  /nonMaskMass; VoffMean   = VoffSum  /nonMaskMass;
  H1offSTD   = sqrt(H1offSS  - pow(H1offMean,2));
  H2offSTD   = sqrt(H2offSS  - pow(H2offMean,2));
  SoffSTD    = sqrt(SoffSS   - pow(SoffMean,2));
  VoffSTD    = sqrt(VoffSS   - pow(VoffMean,2));

  // TOTAL STATS
  H1Mean  = H1Sum /totalMass; H2Mean  = H2Sum /totalMass;
  SMean   = SSum  /totalMass; VMean   = VSum  /totalMass;
  H1STD   = sqrt(H1SS  - pow(H1Mean,2));
  H2STD   = sqrt(H2SS  - pow(H2Mean,2));
  SSTD    = sqrt(SSS   - pow(SMean,2));
  VSTD    = sqrt(VSS   - pow(VMean,2));

  // DUMP STATS TO FILE
  std::string noutFile = _outFile + ".simple.stats.out.txt";
  LINFO("WRITING %s", noutFile.c_str());
  std::ofstream outFile(noutFile.c_str(),std::ios::app);
  // mass stats
  outFile << _label   << "\t";
  outFile << maskMass << "\t" << nonMaskMass << "\t" << totalMass << "\t";

  // MASK STATS
  outFile << maskMeanX << "\t" << maskSTDX << "\t"
          << maskMeanY << "\t" << maskSTDY << "\t"
          << H1onMean  << "\t" << H1onSTD  << "\t"
          << H2onMean  << "\t" << H2onSTD  << "\t"
          << SonMean   << "\t" << SonSTD   << "\t"
          << VonMean   << "\t" << VonSTD   << "\t";

  // NON MASK STATS
  outFile << H1offMean  << "\t" << H1offSTD  << "\t"
          << H2offMean  << "\t" << H2offSTD  << "\t"
          << SoffMean   << "\t" << SoffSTD   << "\t"
          << VoffMean   << "\t" << VoffSTD   << "\t";

  // TOTAL STATS
  outFile << H1Mean  << "\t" << H1STD  << "\t"
          << H2Mean  << "\t" << H2STD  << "\t"
          << SMean   << "\t" << SSTD   << "\t"
          << VMean   << "\t" << VSTD   << "\n";

  outFile.close();
}
// ######################################################################
template <class FLOAT>
void featureClusterVision<FLOAT>::fCVgetImageComplexStats(std::string _maskFile,
                                                   std::string _imageFile,
                                                   std::string _outFile,
                                                   std::string _label)
{
  unsigned int rotationChannels = 5;
  int rotations[5] = {0,45,90,135,-2};
  unsigned int junctionList1[2] = {0,1};
  unsigned int junctionList2[2] = {2,3};
  unsigned int scales = 6;
  FLOAT gaborSTD = 6.0F;
  FLOAT gaborPrd = 3.0F;
  LINFO("GETTING BASE STATS");
  LINFO("MASK FILE  %s",_maskFile.c_str());
  LINFO("IMAGE FILE %s",_imageFile.c_str());
  Image<FLOAT> maskFile;
  Image<PixRGB<FLOAT> > imageFile;
  Image<PixRGB<FLOAT> > CmaskFile;
  maskFile = Raster::ReadGray(_maskFile, RASFMT_PNM);
  getColor(maskFile,CmaskFile);
  //Raster::VisuRGB(CmaskFile,"CmaskFile.ppm");
  //CmaskFile = CmaskFile*255;
  imageFile = Raster::ReadRGB(_imageFile, RASFMT_PNM);
  FLOAT maskOffset = (maskFile.getHeight() - imageFile.getHeight())/2;
  typename std::vector<Image<FLOAT> > masks(scales,maskFile*255.0F);


  // Create a nested vector to contain all the images

  Image<FLOAT> holder;
  typename std::vector<Image<FLOAT> > HSVpart(4,holder);
  typename std::vector<std::vector<Image<FLOAT> > >
    rot(rotationChannels,HSVpart);
  typename std::vector<std::vector<std::vector<Image<FLOAT> > > >
    complexPyramid(scales,rot);

  // Create iterator pointers for the image components

  typename Image<FLOAT>::iterator H1itr;
  typename Image<FLOAT>::iterator H2itr;
  typename Image<FLOAT>::iterator Sitr;
  typename Image<FLOAT>::iterator Vitr;

  // This will pull out the Gabor responses for the image at different
  // angles and scales over H2SV components;

  Image<PixRGB<FLOAT> > currentImage = imageFile;

  // move over all scales
  unsigned int s = 0;
  for(typename std::vector<std::vector<std::vector<Image<FLOAT> > > >::iterator
        complexPyramidItr = complexPyramid.begin();
      complexPyramidItr != complexPyramid.end();
      ++complexPyramidItr, s++)
  {
    unsigned int i = 0;
    // move over all rotations
    for(typename std::vector<std::vector<Image<FLOAT> > >::iterator rotationItr =
          complexPyramidItr->begin(); rotationItr != complexPyramidItr->end();
        ++rotationItr, i++)
    {
      // simple gabor interations, else we look for junctions etc.
      if(rotations[i] >= 0)
      {
        LINFO("Running rotations scale %d rot %d",s,rotations[i]);
        // process the H2SV color components
        rotationItr->at(0).resize(currentImage.getWidth(),
                                  currentImage.getHeight());
        H1itr = rotationItr->at(0).beginw();

        rotationItr->at(1).resize(currentImage.getWidth(),
                                  currentImage.getHeight());
        H2itr = rotationItr->at(1).beginw();

        rotationItr->at(2).resize(currentImage.getWidth()
                                  ,currentImage.getHeight());
        Sitr = rotationItr->at(2).beginw();

        rotationItr->at(3).resize(currentImage.getWidth(),
                                  currentImage.getHeight());
        Vitr = rotationItr->at(3).beginw();
        // pull out color components into seperate images
        for(typename Image<PixRGB<FLOAT> >::iterator currentItr
              = currentImage.beginw();
            currentItr != currentImage.endw();
            ++currentItr,++H1itr,++H2itr,++Sitr,++Vitr)
          {
            currentItr->getHSV(*H1itr,*H2itr,*Sitr,*Vitr);
            //LINFO("PIX %f %f %f %f",*H1itr,*H2itr,*Sitr,*Vitr);
            // normalize value (intensity)
            *Vitr = (*Vitr) * (*fCV_valNorm);
          }
        // find the gabor interactions over all H2SV images at this
        // scale and rotation

        // SIN gabor
        Image<FLOAT> mrGabor   = gaborFilter2<FLOAT>(gaborSTD,gaborPrd,0.0F,
                                                     (FLOAT)rotations[i]);
        // COS gabor
        Image<FLOAT> mrGaborAF = gaborFilter2<FLOAT>(gaborSTD,gaborPrd
                                                     ,gaborPrd/2,
                                                     (FLOAT)rotations[i]);

        // find gabor interaction with H2SV images at this scale/rotation
        rotationItr->at(0) = abs(convolve(rotationItr->at(0),mrGabor,CONV_BOUNDARY_ZERO)) +
          abs(convolve(rotationItr->at(0),mrGaborAF,CONV_BOUNDARY_ZERO));
        inplaceAttenuateBorders(rotationItr->at(0), (int)gaborSTD);
        //Raster::VisuFloat(rotationItr->at(0),FLOAT_NORM_0_255,
        //           sformat("imgH1.%d.%d.pgm",s,rotations[i]));
        Raster::WriteGray(rotationItr->at(0)*255.0F,
                          sformat("imgH1.%d.%d.%s.out.pgm",
                                  s,rotations[i],_label.c_str()));

        rotationItr->at(1) = abs(convolve(rotationItr->at(1),mrGabor,CONV_BOUNDARY_ZERO)) +
          abs(convolve(rotationItr->at(1),mrGaborAF,CONV_BOUNDARY_ZERO));
        inplaceAttenuateBorders(rotationItr->at(1), (int)gaborSTD);
        //Raster::VisuFloat(rotationItr->at(1),FLOAT_NORM_0_255,
        //           sformat("imgH2.%d.%d.pgm",s,rotations[i]));
        Raster::WriteGray(rotationItr->at(1)*255.0F,
                          sformat("imgH2.%d.%d.%s.out.pgm",
                                  s,rotations[i],_label.c_str()));

        rotationItr->at(2) = abs(convolve(rotationItr->at(2),mrGabor,CONV_BOUNDARY_ZERO)) +
          abs(convolve(rotationItr->at(2),mrGaborAF,CONV_BOUNDARY_ZERO));
        inplaceAttenuateBorders(rotationItr->at(2), (int)gaborSTD);
        //Raster::VisuFloat(rotationItr->at(2),FLOAT_NORM_0_255,
        //           sformat("imgS.%d.%d.pgm",s,rotations[i]));
        Raster::WriteGray(rotationItr->at(2)*255.0F,
                          sformat("imgS.%d.%d.%s.out.pgm",
                                  s,rotations[i],_label.c_str()));

        rotationItr->at(3) = abs(convolve(rotationItr->at(3),mrGabor,CONV_BOUNDARY_ZERO)) +
          abs(convolve(rotationItr->at(3),mrGaborAF,CONV_BOUNDARY_ZERO));
        inplaceAttenuateBorders(rotationItr->at(3), (int)gaborSTD);
        //Raster::VisuFloat(rotationItr->at(3),FLOAT_NORM_0_255,
        //                   sformat("imgV.%d.%d.pgm",s,rotations[i]));
        Raster::WriteGray(rotationItr->at(3)*255.0F,
                          sformat("imgV.%d.%d.%s.out.pgm",
                                  s,rotations[i],_label.c_str()));
      }
      // here we use complex channels to look for junctions
      else
      {
        LINFO("Running junctions scale %d rot %d",s,rotations[i]);
        // first element H1 - its orthogonal counterpart
        // first compute lineyness
        // This is the same formula used in mixChannels at ln 597 (ish)

        Image<FLOAT> Alpha;
        Image<FLOAT> Beta;
        // Mix 0,45,90,135 degree angles and find junctions at this scale
        // H1
        fCVmixChannels(complexPyramidItr->at(junctionList1[0])[0],
                    complexPyramidItr->at(junctionList1[1])[0],
                    complexPyramidItr->at(junctionList2[0])[0],
                    complexPyramidItr->at(junctionList2[1])[0],
                    &Alpha,&Beta,&complexPyramidItr->at(i)[0]);
        //Raster::VisuFloat(complexPyramidItr->at(i)[0],FLOAT_NORM_0_255,
        //                   sformat("juncH1.%d.pgm",s));
        Raster::WriteGray(complexPyramidItr->at(i)[0]*255.0F,
                          sformat("imgH1.junc.%d.%s.out.pgm",
                                  s,_label.c_str()));

        // H2
        fCVmixChannels(complexPyramidItr->at(junctionList1[0])[1],
                    complexPyramidItr->at(junctionList1[1])[1],
                    complexPyramidItr->at(junctionList2[0])[1],
                    complexPyramidItr->at(junctionList2[1])[1],
                    &Alpha,&Beta,&complexPyramidItr->at(i)[1]);
        //Raster::VisuFloat(complexPyramidItr->at(i)[1],FLOAT_NORM_0_255,
        //           sformat("juncH2.%d.pgm",s));
        Raster::WriteGray(complexPyramidItr->at(i)[1]*255.0F,
                          sformat("imgH2.junc.%d.%s.out.pgm",
                                  s,_label.c_str()));
        // S
        fCVmixChannels(complexPyramidItr->at(junctionList1[0])[2],
                    complexPyramidItr->at(junctionList1[1])[2],
                    complexPyramidItr->at(junctionList2[0])[2],
                    complexPyramidItr->at(junctionList2[1])[2],
                    &Alpha,&Beta,&complexPyramidItr->at(i)[2]);
        //Raster::VisuFloat(complexPyramidItr->at(i)[2],FLOAT_NORM_0_255,
        //           sformat("juncS.%d.pgm",s));
        Raster::WriteGray(complexPyramidItr->at(i)[2]*255.0F,
                          sformat("imgS.junc.%d.%s.out.pgm",
                                  s,_label.c_str()));
        // V
        fCVmixChannels(complexPyramidItr->at(junctionList1[0])[3],
                    complexPyramidItr->at(junctionList1[1])[3],
                    complexPyramidItr->at(junctionList2[0])[3],
                    complexPyramidItr->at(junctionList2[1])[3],
                    &Alpha,&Beta,&complexPyramidItr->at(i)[3]);
        //Raster::VisuFloat(complexPyramidItr->at(i)[3],FLOAT_NORM_0_255,
        //                   sformat("juncV.%d.pgm",s));
        Raster::WriteGray(complexPyramidItr->at(i)[3]*255.0F,
                          sformat("imgV.junc.%d.%s.out.pgm",
                                  s,_label.c_str()));
      }
    }
    // Make image half size after each iteration
    currentImage = rescaleBilinear(currentImage,
                           currentImage.getWidth()/2,
                           currentImage.getHeight()/2);
    // resize the mask as well, but keep it similar in size to the
    // orignal image for the width
    if(s < scales-1)
    {
      //masks[s+1] = rescaleBilinear(maskFile,currentImage.getWidth()/2
      //                           ,masks[s].getHeight()/2);
      LINFO("DOWNSCALE");
      Image<PixRGB<FLOAT> > Cmask = downscaleFancy(CmaskFile,
                                                   currentImage.getWidth()
                                                   ,masks[s].getHeight()/2,1
                                                   ,true);
      //LINFO("COPY COLOR");
      //Raster::VisuRGB(Cmask,sformat("CMask.%d.ppm",s+1));

      getGrey(Cmask,masks[s+1]);
      masks[s+1] = masks[s+1]*255.0F;
      Raster::WriteGray(masks[s+1],
                        sformat("masks.%d.%s.out.pgm",
                                s,_label.c_str()));
      //LINFO("RASTER");
      //Raster::VisuRGB(masks[s+1],sformat("Mask.%d.ppm",s+1));
    }
  }

  std::string noutFile = _outFile + ".complex.stats.out.txt";

  LINFO("WRITING %s", noutFile.c_str());
  std::ofstream outFile(noutFile.c_str(),std::ios::app);
  s = 0;
  // get stats on image
  for(typename std::vector<std::vector<std::vector<Image<FLOAT> > > >::iterator
        complexPyramidItr = complexPyramid.begin();
      complexPyramidItr != complexPyramid.end();
      ++complexPyramidItr, s++)
  {
    unsigned int i = 0;
    // move over all rotations
    for(typename std::vector<std::vector<Image<FLOAT> > >::iterator rotationItr =
          complexPyramidItr->begin(); rotationItr != complexPyramidItr->end();
        ++rotationItr, i++)
    {
      LINFO("COMPUTING over scale %d rot %d",s,rotations[i]);
      maskOffset = (masks[s].getHeight() - rotationItr->at(0).getHeight())/2;
      H1itr = rotationItr->at(0).beginw();
      H2itr = rotationItr->at(1).beginw();
      Sitr  = rotationItr->at(2).beginw();
      Vitr  = rotationItr->at(3).beginw();

      FLOAT H1onMean, H2onMean, SonMean, VonMean;
      FLOAT H1onSTD, H2onSTD, SonSTD, VonSTD;
      FLOAT H1onSum = 0; FLOAT H2onSum = 0;
      FLOAT SonSum  = 0; FLOAT VonSum  = 0;
      double H1onSS  = 0; double H2onSS  = 0;
      double SonSS   = 0; double VonSS   = 0;
      // colors off the target
      FLOAT H1offMean, H2offMean, SoffMean, VoffMean;
      FLOAT H1offSTD, H2offSTD, SoffSTD, VoffSTD;
      FLOAT H1offSum = 0; FLOAT H2offSum = 0;
      FLOAT SoffSum  = 0; FLOAT VoffSum  = 0;
      double H1offSS  = 0; double H2offSS  = 0;
      double SoffSS   = 0; double VoffSS   = 0;
      // total colors
      FLOAT H1Mean, H2Mean, SMean, VMean;
      FLOAT H1STD, H2STD, SSTD, VSTD;
      FLOAT H1Sum = 0; FLOAT H2Sum = 0;
      FLOAT SSum  = 0; FLOAT VSum  = 0;
      double H1SS  = 0; double H2SS  = 0;
      double SSS   = 0; double VSS   = 0;
      // mask stats
      FLOAT maskMeanX, maskMeanY, maskSTDX, maskSTDY;
      FLOAT maskSumX = 0; FLOAT maskSumY = 0;
      double maskSSX  = 0; double maskSSY  = 0;
      unsigned long maskMass    = 0;
      unsigned long nonMaskMass = 0;
      unsigned long totalMass   = 0;


      // offset the mask iterator to align with the image
      typename Image<FLOAT>::iterator Mitr = masks[s].beginw() +
               (int)(masks[s].getWidth() * maskOffset);

      // find the mass of the mask at each scale
      LINFO("COMPUTING mass values");
      for(typename Image<FLOAT>::iterator currentItr =
            rotationItr->at(0).beginw();
          currentItr != rotationItr->at(0).endw(); ++Mitr, ++currentItr)
      {
        if(*Mitr > 0.01F)
        {
          maskMass++;
        }
        else
        {
          nonMaskMass++;
        }
        totalMass++;
      }

      LINFO("MASS mask %lx non-mask %lx total %lx",maskMass,nonMaskMass,
            totalMass);

      Mitr = masks[s].beginw() +
              (int)(masks[s].getWidth() * maskOffset);
      unsigned int x = 0;

      // for each image pixel and each of the H2SV components
      // compute the complex image stats

      for(typename Image<FLOAT>::iterator currentItr =
            rotationItr->at(0).beginw();
          currentItr != rotationItr->at(0).endw();
          ++currentItr,++H1itr,++H2itr,++Sitr,++Vitr, ++Mitr, x++)
      {
        //We fall on a mask
        if(*Mitr > 0.01F)
        {
          maskSumX += x % rotationItr->at(0).getWidth();
          maskSumY += x / rotationItr->at(0).getWidth();
          H1onSum   = *H1itr + H1onSum;
          H2onSum   = *H2itr + H2onSum;
          SonSum    = *Sitr  + SonSum;
          VonSum    = *Vitr  + VonSum;
          // SUM of SQUARES
          maskSSX += pow((FLOAT)(x % rotationItr->at(0).getWidth()),2)
            /maskMass;
          maskSSY += pow((FLOAT)(x / rotationItr->at(0).getWidth()),2)
            /maskMass;
          H1onSS   = (pow(*H1itr,2)   /maskMass) + H1onSS;
          H2onSS   = (pow(*H2itr,2)   /maskMass) + H2onSS;
          SonSS    = (pow(*Sitr,2)    /maskMass) + SonSS;
          VonSS    = (pow(*Vitr,2)    /maskMass) + VonSS;
        }
        // We don't fall on a mask
        else
        {
          // off mask stats
          // SUM
          H1offSum = *H1itr + H1offSum;
          H2offSum = *H2itr + H2offSum;
          SoffSum  = *Sitr  + SoffSum;
          VoffSum  = *Vitr  + VoffSum;
          // SUM of SQUARES
          H1offSS  = (pow(*H1itr,2)/nonMaskMass) + H1offSS;
          H2offSS  = (pow(*H2itr,2)/nonMaskMass) + H2offSS;
          SoffSS   = (pow(*Sitr,2) /nonMaskMass) + SoffSS;
          VoffSS   = (pow(*Vitr,2) /nonMaskMass) + VoffSS;
        }
        // total stats
        // SUM
        H1Sum = *H1itr + H1Sum;
        H2Sum = *H2itr + H2Sum;
        SSum  = *Sitr  + SSum;
        VSum  = *Vitr  + VSum;
        // SUM of SQUARES
        H1SS  = (pow(*H1itr,2)/totalMass) + H1SS;
        H2SS  = (pow(*H2itr,2)/totalMass) + H2SS;
        SSS   = (pow(*Sitr,2) /totalMass) + SSS;
        VSS   = (pow(*Vitr,2) /totalMass) + VSS;
      }

      LINFO("COMPUTING TOTAL STATS");

      // MASK STATS
      maskMeanX = maskSumX/maskMass; maskMeanY = maskSumY/maskMass;
      H1onMean  = H1onSum /maskMass; H2onMean  = H2onSum /maskMass;
      SonMean   = SonSum  /maskMass; VonMean   = VonSum  /maskMass;
      maskSTDX  = sqrt(maskSSX - pow(maskMeanX,2));
      maskSTDY  = sqrt(maskSSY - pow(maskMeanY,2));
      H1onSTD   = sqrt(H1onSS  - pow(H1onMean,2));
      H2onSTD   = sqrt(H2onSS  - pow(H2onMean,2));
      SonSTD    = sqrt(SonSS   - pow(SonMean,2));
      VonSTD    = sqrt(VonSS   - pow(VonMean,2));

      // NON MASK STATS
      H1offMean  = H1offSum /nonMaskMass; H2offMean  = H2offSum /nonMaskMass;
      SoffMean   = SoffSum  /nonMaskMass; VoffMean   = VoffSum  /nonMaskMass;
      H1offSTD   = sqrt(H1offSS  - pow(H1offMean,2));
      H2offSTD   = sqrt(H2offSS  - pow(H2offMean,2));
      SoffSTD    = sqrt(SoffSS   - pow(SoffMean,2));
      VoffSTD    = sqrt(VoffSS   - pow(VoffMean,2));

      // TOTAL STATS
      H1Mean  = H1Sum /totalMass; H2Mean  = H2Sum /totalMass;
      SMean   = SSum  /totalMass; VMean   = VSum  /totalMass;
      H1STD   = sqrt(H1SS  - pow(H1Mean,2));
      H2STD   = sqrt(H2SS  - pow(H2Mean,2));
      SSTD    = sqrt(SSS   - pow(SMean,2));
      VSTD    = sqrt(VSS   - pow(VMean,2));

      LINFO("WRITING STATS TO FILE");

      // dump all this fun stuff to a file
      outFile << _label   << "\t";
      outFile << s        << "\t" << rotations[i] << "\t";
      outFile << maskMass << "\t" << nonMaskMass  << "\t" << totalMass << "\t";

      // MASK STATS
      outFile << maskMeanX << "\t" << maskSTDX << "\t"
              << maskMeanY << "\t" << maskSTDY << "\t"
              << H1onMean  << "\t" << H1onSTD  << "\t"
              << H2onMean  << "\t" << H2onSTD  << "\t"
              << SonMean   << "\t" << SonSTD   << "\t"
              << VonMean   << "\t" << VonSTD   << "\t";

      // NON MASK STATS
      outFile << H1offMean  << "\t" << H1offSTD  << "\t"
              << H2offMean  << "\t" << H2offSTD  << "\t"
              << SoffMean   << "\t" << SoffSTD   << "\t"
              << VoffMean   << "\t" << VoffSTD   << "\t";

      // TOTAL STATS
      outFile << H1Mean  << "\t" << H1STD  << "\t"
              << H2Mean  << "\t" << H2STD  << "\t"
              << SMean   << "\t" << SSTD   << "\t"
              << VMean   << "\t" << VSTD   << "\n";
    }
  }
}

// ############################################################
// explicit instantiations
// ############################################################

template class featureClusterVision<float>;
