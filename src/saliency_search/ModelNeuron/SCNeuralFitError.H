//!For fitting neural data to a model

//////////////////////////////////////////////////////////////////////////
// University of Southern California (USC) and the iLab at USC.         //
// See http://iLab.usc.edu for information about this project.          //
//////////////////////////////////////////////////////////////////////////
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
//////////////////////////////////////////////////////////////////////////
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
//////////////////////////////////////////////////////////////////////////
//
// Primary maintainer for this file: David J. Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/SCNeuralFitError.H $
#ifndef MODELNEURON_SCNEURALFITERROR_H_DEFINED
#define MODELNEURON_SCNEURALFITERROR_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include "Fitting/NeuralFitError.H"
#include "ModelNeuron/SimStructure.H"
#include "ModelNeuron/StructurePlot.H"
#include "ModelNeuron/FreeViewingData.H"
#include "Image/Image.H"
#include "Image/ImageSet.H"
#include "Image/Range.H"
#include "Neuro/SimulationPlot1D.H"
#include "ModelNeuron/SurpriseSC.H"

class SCTransformModule;
// ######################################################################
//! A partial implementation of NeuarlFitError for fitting an SC model to neural data 
// ######################################################################
class SCNeuralFitErrorAdapter: public NeuralFitError
{	
  public:
    //! constructor
    SCNeuralFitErrorAdapter(OptionManager &  mgr, 
                            std::string const & descrName, 
                            std::string const & tagName);
    
    //!toggle whether to the show the demo plot
    void showDemoPlot(bool const value);

    //!should we display a demo plots?
    bool const hasDemoPlot() const;
    
  protected:
    //! post process maps
    virtual Image<float> postProcessMap(const Image<float> & map);

    //!Derived classes may need to reset after processing a stimulus file.
    virtual void reset(const std::vector<double>& params);
    
    OModelParam<Dims> itsMapDimensions;
    OModelParam<Point2D<float> > itsStimLocation;
    OModelParam<float> itsReceptiveFieldSize;
    OModelParam<std::string> itsSCModelType;
    OModelParam<SimTime> itsModelTimeStep;
    OModelParam<Dims> itsDemoPlotDims;
    OModelParam<uint> itsModelPlotLength;
    OModelParam<uint> its2DPlotDepth;
    OModelParam<uint> itsProbeDepth;
    OModelParam<bool> itsDisplayOutputType;
    OModelParam<Range<double> > itsDisplayRange;
    OModelParam<ProbeType> itsProbeSampleType;

    NModelParam<bool> itsUseSpaceVariant;
    NModelParam<Dims> itsScreenPixels;

    rutz::shared_ptr<nsu::SimStructure> itsModel; //hold any models
    rutz::shared_ptr<nsu::StructurePlot> itsModelPlot; //for saving output

    double itsLastNeuralSample;
    double itsLastModelResponse;
    SimTime itsLastTime;
    Dims itsMapDims;
    Point2D<int> itsStimLoc;
    uint itsRFSize;
    PixelUnitTransform itsPixelTransform;
    ProbeType itsProbeType;

    Image<float> itsLastModelOutput;
    uint itsOutFrameCount;
    void start1();
    
  private:
    //!get some starting parameters
    virtual std::vector<double> startingParams();

    //!get some starting parameters
    virtual void paramRange(std::vector<double> & min, std::vector<double> & max);
    
    //! return any sc model related paramters
    std::vector<double> getSCParams();
    
    //!get some paramter ranges
    virtual void getSCParamRange(std::vector<double> & min, std::vector<double> & max);
    
    //! return any other paramters - default returns an empty vector
    virtual std::vector<double> getOtherParams();

    //! return any other paramter ranges - default returns an empty vector
    virtual void getOtherParamRange(std::vector<double> & min, std::vector<double> & max);

    //!get a new sc model with params (params will only hold sc model params)
    nsu::SimStructure* getSCModel(std::vector<double>const &  params); 
    
    //! compute model response to stimulus
    DataSet computeModelResponse(std::vector<double> const & params);

    //!read a stimulus file
    virtual void readStimulusFile(std::string const & fileName, const uint subsample) = 0;

    //!return the number of stimuli
    virtual uint getNumStims() const = 0;

    //!return the number of samples for stimulus at the index
    virtual uint getNumSamples(uint const index) const = 0;

    //!return the number of samples for stimulus at the index
    virtual Image<double> processImage(std::vector<double> const & params, uint const index, uint const sample) = 0;

    //! generate a plot on each timestep
    void demoPlot(const SimTime& time = SimTime::ZERO());

    //!combine all the demo elements into the layout
    virtual void combineDemoElements(Layout<PixRGB<byte> > & layout);

    uint numSCParams, numOtherParams;
    bool itsShowDemoPlot;
    Layout<PixRGB<byte> > itsLayout;
};

// ######################################################################
//! A do-nothing stub implementation
// ######################################################################
class SCNeuralFitErrorStub : public SCNeuralFitErrorAdapter
{
  public:
    //! Constructor
    
    SCNeuralFitErrorStub(OptionManager& mgr,
                         const std::string& descrName = "SC Neural Fit Error Stub",
                         const std::string& tagName = "SCNeuralFitErrorStub");
    
    //! Destructor
    virtual ~SCNeuralFitErrorStub();

    //no-op implementation
    virtual void readStimulusFile(std::string const & fileName, const uint subsample);

     //!return the number of stimuli
    virtual uint getNumStims() const;

    //!return the number of samples for stimulus at the index
    virtual uint getNumSamples(uint const index) const;

    //!return the number of samples for stimulus at the index
    virtual Image<double> processImage(std::vector<double> const & params, uint const index, uint const sample);
};

// ######################################################################
//! objective function for fitting an SC model to slice data
// ######################################################################
class SliceFitError : public SCNeuralFitErrorAdapter
{	
  public:
    //! constructor
    SliceFitError(OptionManager & mgr, 
                          std::string const & descrName = "Slice Fit Error function",
                          std::string const & tagName = "SliceFitError");    
    
  protected:
    void start1();

  private:
    //!read a stimulus file
    void readStimulusFile(std::string const & fileName, const uint subsample);

    //!return the number of stimuli
    virtual uint getNumStims() const;

    //!return the number of samples for stimulus at the index
    virtual uint getNumSamples(uint const index) const;

    //!return the number of samples for stimulus at the index
    virtual Image<double> processImage(std::vector<double> const & params, uint const index, uint const sample);
    
    //! get the other params
    virtual std::vector<double> getOtherParams();

    //! return any other paramter ranges - default returns an empty vector
    virtual void getOtherParamRange(std::vector<double> & min, std::vector<double> & max);

    //!combine all the demo elements into the layout
    virtual void combineDemoElements(Layout<PixRGB<byte> > & layout);

    std::vector<ImageSet<double> > itsStimData;

    Image<PixRGB<byte> > itsLastStim;
    Dims itsRescaleDims;
};

// ######################################################################
// objective function for fitting an SC model to monkey
// ######################################################################
class MonkeyFitError : public SCNeuralFitErrorAdapter
{	
  public:
    //! constructor
    MonkeyFitError(OptionManager & mgr, 
                   std::string const & descrNamei = "Monkey fit error function",
                   std::string const & tagName = "MonkeyFitError");    
    
  private:  
    //! class to represent a stimulus
    struct Point 
    {
        //stimulus info
        double x;
        double y;
        uint R;
        uint G;
        uint B;
        double diam;
    };
    
    //! class to represent a video frame composed of stimuli
    struct Frame
    {
        //fixation point
        Point fixation;
        //stimuli
        std::vector<Point> stimuli;
    };

    //!read a stimulus file and fill in frames info
    void readStimulusFile(std::string const & fileName, const uint subsample);
    
    //!return the number of stimuli
    virtual uint getNumStims() const;

    //!return the number of samples for stimulus at the index
    virtual uint getNumSamples(uint const index) const;

    //!return the number of samples for stimulus at the index
    virtual Image<double> processImage(std::vector<double> const & params, uint const index, uint const sample);

    //!get the other params
    virtual std::vector<double> getOtherParams();

    //! return any other paramter ranges - default returns an empty vector
    virtual void getOtherParamRange(std::vector<double> & min, std::vector<double> & max);

    std::vector<std::vector<Frame> > itsFrames;
};


// ######################################################################
//! An  implementation of NeuarlFitError for free viewing data
// ######################################################################
class FreeviewingFitErrorBase : public SCNeuralFitErrorAdapter
{	
  public:
    //! constructor
    FreeviewingFitErrorBase(OptionManager &  mgr, 
                            std::string const & descrName = "Free viewing fit error function", 
                            std::string const & tagName = "FreeViewingFitError");    

    ~FreeviewingFitErrorBase();

  protected:
    void start1();

    void start2();

    //reset some variables when a new condition starts
    virtual void reset(const std::vector<double>& params);
    
    //!get other paramters
    virtual std::vector<double> getOtherParams();

    //! return any other paramter ranges - default returns an empty vector
    virtual void getOtherParamRange(std::vector<double> & min, std::vector<double> & max);
   
    //!read a stimulus file and fill in frames info
    virtual void readStimulusFile(std::string const & fileName, const uint subsample);
    
    //!return the number of stimuli
    virtual uint getNumStims() const;
    
    //!return the number of samples for stimulus at the index
    virtual uint getNumSamples(uint const index) const;
    
    //!return the number of samples for stimulus at the index
    virtual Image<double> processImage(std::vector<double> const & params, uint const index, uint const sample);
    
    //! combine elements of a demo plot
    virtual void combineDemoElements(Layout<PixRGB<byte> > & layout);

    //! post process maps
    virtual Image<float> postProcessFeatureMap(const Image<float> & map);

    OModelParam<std::string> itsEyeFileNames;
    OModelParam<std::string> itsNormFileName;

    //sampling of free data
    OModelParam<SimTime> itsFreeDataTime;

    //feature map dims
    OModelParam<Dims> itsFeatureMapDims;

    OModelParam<float> itsChanGain;

    //all of our stims
    std::vector<FreeData> itsStimData;
    
    //! hold any channel range data
    std::vector<Range<float> > itsChanRange;

    //! should we normalize channels?
    bool doNormalize;

    //!hold the last frames maps and eye data and timestep
    Image<PixRGB<byte> > itsLastRetinal;
    ImageSet<float> itsLastChanMaps;
    Image<float> itsLastSCInput;
    rutz::shared_ptr<EyeData> itsLastEyeData;

    //!hold the horizontal and vertical eye position
    SimulationPlot1D itsHeye, itsVeye, itsNeuralPlot, itsModelPlot1D;
    uint itsCondition;
    Point2D<int> itsStimLocHD, itsStimLocMaps;
    uint itsRFSizeHD, itsRFSizeMaps;

    Range<float> itsRange;
};

// ######################################################################
//! An  implementation of NeuarlFitError for free viewing data
// ######################################################################
class FreeviewingFitErrorSV : public FreeviewingFitErrorBase
{	
  public:
    //! constructor
    FreeviewingFitErrorSV(OptionManager &  mgr, 
                        std::string const & descrName = "Free viewing fit error function", 
                        std::string const & tagName = "FreeViewingFitErrorSV");    

    ~FreeviewingFitErrorSV();

    //!get other paramters
    virtual std::vector<double> getOtherParams();

    //! return any other paramter ranges - default returns an empty vector
    virtual void getOtherParamRange(std::vector<double> & min, std::vector<double> & max);

    //! post process maps
    virtual Image<float> postProcessFeatureMap(const Image<float> & map);

    //!post process map
    virtual Image<float> postProcessMap(const Image<float> & map);

    //! reset
    virtual void reset(const std::vector<double>& params);

  protected:
    void start1();
    void start2();

    OModelParam<Dims> itsRetinalDims;

    OModelParam<bool> itsUseScMaxNorm;
    OModelParam<bool> itsUseChannelMaxNorm;

    OModelParam<bool> itsUseScSurprise;
    OModelParam<bool> itsUseChannelSurprise;

    OModelParam<bool> itsUseChanSurpKL;

  private:
    SurpriseMapSC itsChanSurp;
    SurpriseMapSC itsSCSurp;

    nub::ref<SCTransformModule> itsTransform;

    bool useChannelMaxNorm;
    bool useScMaxNorm;
    bool useChannelSurprise;
    bool useScSurprise;
    uint itsNMChanW;
    uint itsNMSCW;
    uint itsNMChanWPos;
    uint itsNMSCWPos;
    uint itsSurpChanUPos;
    uint itsSurpSCUPos;
};

// ######################################################################
//! An  implementation of NeuarlFitError for free viewing data
// ######################################################################
class FreeviewingFitErrorCT : public FreeviewingFitErrorBase
{	
  public:
    //! constructor
    FreeviewingFitErrorCT(OptionManager &  mgr, 
                        std::string const & descrName = "Free viewing fit error function", 
                        std::string const & tagName = "FreeViewingFitErrorCT");    

    ~FreeviewingFitErrorCT();

    //!get other paramters
    virtual std::vector<double> getOtherParams();

    //! return any other paramter ranges - default returns an empty vector
    virtual void getOtherParamRange(std::vector<double> & min, std::vector<double> & max);

    //! post process maps
    virtual Image<float> postProcessFeatureMap(const Image<float> & map);

    //!post process map
    virtual Image<float> postProcessMap(const Image<float> & map);

    //! reset
    virtual void reset(const std::vector<double>& params);

  protected:
    void start1();
    void start2();

    OModelParam<bool> itsUseScMaxNorm;
    OModelParam<bool> itsUseChannelMaxNorm;

    OModelParam<bool> itsUseScSurprise;
    OModelParam<bool> itsUseChannelSurprise;

  private:
    SurpriseMapSC itsChanSurp;
    SurpriseMapSC itsSCSurp;

    bool useChannelMaxNorm;
    bool useScMaxNorm;
    bool useChannelSurprise;
    bool useScSurprise;
    uint itsNMChanW;
    uint itsNMSCW;
    uint itsNMChanWPos;
    uint itsNMSCWPos;
    uint itsSurpChanUPos;
    uint itsSurpSCUPos;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
#endif
#endif
