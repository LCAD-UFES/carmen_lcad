/*!@file Neuro/SimulationViewerEyeMvtNeuro.H get the saliency values at a
   probe location relative the the current eye position for comparing
   with a neural response*/

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
// by the University of Southern California (USC) and the iLab at USC.  //
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
// Primary maintainer for this file: David J Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SimulationViewerEyeMvtNeuro.H $

#ifndef SIMULATIONVIEWEREYEMVTNEURO_H_DEFINED
#define SIMULATIONVIEWEREYEMVTNEURO_H_DEFINED

#include "Component/ModelParam.H"
#include "Image/ImageCache.H"
#include "Image/LevelSpec.H"
#include "Image/Point2D.H"
#include "Image/LogPolarTransform.H"
#include "Neuro/SimulationViewer.H"
#include "Simulation/SimEvents.H"
#include "SimulationPlot1D.H"

#include<deque>

class ofstream;

// ######################################################################
//! Simulation viewer that measures salience at receptive field center of a neuron
// ######################################################################
class SimulationViewerEyeMvtNeuro : public SimulationViewerAdapter {
public:
  // ######################################################################
  /*! @name Constructors and destructors */
  //@{

  //! Constructor. See ModelComponent.H.
  SimulationViewerEyeMvtNeuro(OptionManager& mgr,
                              const std::string& descrName =
                              "EyeMvtNeuro Simulation Viewer",
                              const std::string& tagName =
                              "SimulationViewerEyeMvtNeuro");
  
  //! Destructor
  virtual ~SimulationViewerEyeMvtNeuro();
  
  //@}
  
protected:
  //! Callback for every clock tick
  SIMCALLBACK_DECLARE(SimulationViewerEyeMvtNeuro, SimEventClockTick);
  
  //! Callback for every time we should save our outputs
  SIMCALLBACK_DECLARE(SimulationViewerEyeMvtNeuro, SimEventSaveOutput);
  
  //! Save our various results
  void save1(const ModelComponentSaveInfo& sinfo);
  
  //! Get the attention/eye/head trajectory image
  virtual Image< PixRGB<byte> > getTraj();
  
  OModelParam<bool> itsSaveTraj;      //!< save trajectory?
  OModelParam<bool> itsSaveMegaCombo; //!< save mega combo?
  OModelParam<int> itsDelayCacheSize; //!< size of our delay cache
  OModelParam<int> itsEyePatchSize;      //!< size of marker at eye position
  OModelParam<LevelSpec> itsLevelSpec; //!< our levelspec
  OModelParam<bool> itsUseDiagColor;//!< use eye movement diagnostic colors
  OModelParam<int> itsMaxComboWidth; //!< max width of getTraj()
  OModelParam<bool> itsShiftInput;   //! shift input to eye position?
  NModelParam<bool> itsUseSpaceVariantBoundary; //!< use space variant boundary

  //SimulationViewerEyeMvtNeuro specific options
  OModelParam<std::string> itsOutFname;  //!< Our results file
  OModelParam<Point2D<float> > itsProbe;//!location of virtual probe
  OModelParam<float> itsRFSize;//!size of the neurons receptive field
  OModelParam<std::string> itsRFMaskName;//!name of the rf weighting mask
  OModelParam<std::string> itsNeuronFileName;//!a text file with spike data
  OModelParam<bool> itsDisplayTime;//!display the sample time
  OModelParam<int> itsDelaySpike;//!delay our neurons data by this # of samples
  OModelParam<SimTime> itsBufferLength;//!length of plotting buffer
  OModelParam<float> itsSalScale; //!< y-axis scale of 1D saliency plots
  OModelParam<float> itsNeuroScale; //!< y-axis scale of 1D saliency plots
  OModelParam<std::string> itsVisRFName; //!< name of visual rf output file
  OModelParam<std::string> itsMotRFName; //!< name of motor rf output file
  OModelParam<SimTime> itsMotWindow; //!< window for motor rf
  OModelParam<SimTime> itsVisWindow; //!< window for visual rf
  OModelParam<SimTime> itsVisOffset; //!< offset for visual rf
  OModelParam<bool> itsShowSCMap; //!< show anatomical SC map?
  OModelParam<Dims> itsSCMapDims; //!< dims of anatomical sc map

  virtual void start1(); //!< get started
  virtual void stop1(); //!< get stopped  

  //!process retinal images
  virtual void doEventRetinaImage(SimEventQueue& q, rutz::shared_ptr<SimEventRetinaImage>& e);

  //!transform image to the anatomical SC
  virtual Image<PixRGB<byte> > transformToAnatomicalSC(const Image<PixRGB<byte> > & img);

private:
  //read an image set from disk
  void readImageSet(ImageSet<float>& set, const std::string filename);

  //write an image set to dsik
  void saveImageSet(const ImageSet<float>& set,
                    const int flags, std::string fname,
                    const RasterFileFormat ftype = RASFMT_AUTO);

  //prepare the file name for writing or reading image sets
  void prepFileName(const std::string& name,
                    std::string& base, std::string& ext);

  //set pixel locations of values originally given in degrees
  void setDegtoPixels(const float& ppdx, const float& ppdy);

  ImageCacheMinMax<float> itsDelayCache;//sliding image cache of sal maps

  Image<float> itsHeadSM;  // the saliency map at the head of the delay cache
  Image< PixRGB<byte> > itsDrawings;  // our drawings - black is transparent!
  Image< PixRGB<byte> > itsSCMap;  // sc anatomical map
  Image<float> itsRFMask;//the RF weighting function if we set one

  ImageSet<float> itsVisRf, itsVisRfBuf;//for visual rf
  Image<float> itsMotRf; //for calculating  motor rf
  uint itsMotSamp;
  uint itsVisSamp;
  uint itsVisSampOff;

  std::ofstream *itsOutFile; //outout file
  std::ifstream *itsNeuronFile;//spike file

  SimTime itsRate; //!< the sampling rate
  float itsPpdx; //!<pixels per degree
  float itsPpdy; //!<pixels per degree
  Point2D<int> itsProbeP;//probe location in pixels
  Point2D<int> itsRawCenter;//center of raw input
  int itsRFSizeP;//rf size in pixels
  Dims itsInputDims; //our input image dims
  Dims itsOgInputDims; //input dims of input imagex


  std::deque<float> itsSpikeVals;//hold data values
  uint itsSpikePos; //position in our array
  SimulationPlot1D itsSalPb; //plot saliency values over a short interval
  SimulationPlot1D itsSpikePb; //plot spike  values over a short interval
  SimulationPlot1D itsHposPb; //plot horizontal eye position 
  SimulationPlot1D itsVposPb; //plot vertical eye position 
  std::string itsXlabel, itsTitle;  //labels for our plots

  LogPolarTransform itsSCTransform;
};
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
