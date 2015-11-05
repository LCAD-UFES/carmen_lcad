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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/SCNeuralFitError.C $

#include "ModelNeuron/SCNeuralFitError.H"
#include "ModelNeuron/SCFitOpts.H"
#include "ModelNeuron/SimStructureOpts.H"
#include "ModelNeuron/SimStructures.H"
#include "ModelNeuron/Location.H"
#include "ModelNeuron/SCNorm.H"
#include "Util/StringUtil.H"
#include "SpaceVariant/SpaceVariantOpts.H"
#include "SpaceVariant/SCTransformModule.H"

#include "Image/ImageSetOps.H"
#include "Image/fancynorm.H"
#include "Image/ShapeOps.H"
#include "Image/ColorOps.H"
#include "Image/DrawOps.H"
#include "Image/MathOps.H"
#include "Image/Transforms.H"
#include "Image/ColorMap.H"
#include "Image/Range.H"
#include "Raster/Raster.H"

#include <iostream>
#include <fstream>

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

// ######################################################################
SCNeuralFitErrorAdapter::SCNeuralFitErrorAdapter(OptionManager & mgr, std::string const & descrName, 
                                                 std::string const & tagName):
    NeuralFitError(mgr, descrName, tagName),
    itsMapDimensions(&OPT_SimDim, this),
    itsStimLocation(&OPT_StimLocation, this),
    itsReceptiveFieldSize(&OPT_RFSize, this),
    itsSCModelType(&OPT_SCModelType, this),
    itsModelTimeStep(&OPT_ModelTime, this),
    itsDemoPlotDims(&OPT_DemoPlotDims, this),
    itsModelPlotLength(&OPT_SCPlotLength,this),
    its2DPlotDepth(&OPT_SC2DPlotDepth,this),
    itsProbeDepth(&OPT_SCProbeDepth,this),
    itsDisplayOutputType(&OPT_SCUseDisplayOutput,this),
    itsDisplayRange(&OPT_SCDisplayRange,this),
    itsProbeSampleType(&OPT_ProbeType, this),
    itsUseSpaceVariant("UseSpaceVariant", this, false),
    itsScreenPixels("ScreenSizePixels", this, Dims(1920,1080)),
    itsModel(), itsModelPlot(), itsLastNeuralSample(0.0), itsLastModelResponse(0.0), itsLastTime(SimTime::ZERO()), itsMapDims(),
    itsStimLoc(), itsRFSize(0), itsPixelTransform(), itsProbeType(),
    itsLastModelOutput(), itsOutFrameCount(0),
    numSCParams(0), numOtherParams(0), itsShowDemoPlot(false),
    itsLayout()
{ 

}

// ######################################################################
void SCNeuralFitErrorAdapter::start1()
{
  itsStimLoc = (Point2D<int>)itsStimLocation.getVal();
  itsRFSize = (int)itsReceptiveFieldSize.getVal();
  itsMapDims = itsMapDimensions.getVal();
  itsProbeType = itsProbeSampleType.getVal();

  NeuralFitError::start1();
}

// ######################################################################
Image<float> SCNeuralFitErrorAdapter::postProcessMap(const Image<float> & map)
{
  return map;
}

// ######################################################################
std::vector<double> SCNeuralFitErrorAdapter::startingParams()
{
  std::vector<double> p, mp, op;

  mp = getSCParams();
  numSCParams = mp.size();

  op = getOtherParams();
  numOtherParams = op.size();

  p.insert(p.end(), op.begin(), op.end());
  p.insert(p.end(), mp.begin(), mp.end());

  return p;
}

// ######################################################################
void SCNeuralFitErrorAdapter::paramRange(std::vector<double> & min, std::vector<double> & max)
{
  std::vector<double> scpmin, scpmax, opmin, opmax;
  min = std::vector<double>();
  max = std::vector<double>();

  getSCParamRange(scpmin, scpmax);
  getOtherParamRange(opmin, opmax);

  min.insert(min.end(), opmin.begin(), opmin.end());
  min.insert(min.end(), scpmin.begin(), scpmin.end());

  max.insert(max.end(), opmax.begin(), opmax.end());
  max.insert(max.end(), scpmax.begin(), scpmax.end());
}

// ######################################################################
std::vector<double> SCNeuralFitErrorAdapter::getSCParams()
{
  std::string modelType = itsSCModelType.getVal();

  if (modelType.compare("SCsSigmoid") == 0) 
  {
    std::vector<double> p = {1.0, 4.0, 1.0, 1.0, -100.0, 5.0, 150.0, 6.0, 150.0, 25.0, 50.0, -0.05, 1.0, -1.0, 0.55, 12.0};
    return p;
  }
  else if (modelType.compare("SCsRectify") == 0)
  {
    std::vector<double> p = {1.0, 4.0, 1.0, 1.0, -100.0, 5.0, 150.0, 6.0, 150.0, 25.0, 50.0, -0.05, 1.0, -1.0};
    return p;
  }
  else if (modelType.compare("SCsSigmoidFixed") == 0) 
  {
    std::vector<double> p = {1.0, 4.0, 1.0, 1.0, -100.0, 25.0, 50.0, 0.55, 12.0};
    return p;
  }
  else if (modelType.compare("SCsRectifyFixed") == 0)
  {
    std::vector<double> p = {1.0, 4.0, 1.0, 1.0, -100.0, 25.0, 50.0};
    return p;
 }
	else if (modelType.compare("SCsSigmoidFixedNoBound") == 0) 
  {
    std::vector<double> p = {1.0, 4.0, 1.0, 1.0, -100.0, 25.0, 50.0, -1, 1, 0.55, 12.0};
    return p;
  }
  else if (modelType.compare("SCsRectifyFixedNoBound") == 0)
  {
    std::vector<double> p = {1.0, 4.0, 1.0, 1.0, -100.0, 25.0, 50.0, -1, 1};
    return p;
  }
  else if (modelType.compare("LowPassSCsSigmoid") == 0)
  {
    std::vector<double> p = {1.0, 4.0, 1.0, 1.0, -100.0, 25.0, 50.0, -0.05, 0.55, 12.0};
    return p;
  }
  else if(modelType.compare("LowPassSCsRectify") == 0)
  {
    std::vector<double> p = {1.0, 4.0, 1.0, 1.0, -100.0, 25.0, 50.0, -0.05};
    return p;
  }

  else if(modelType.compare("SliceAdaptation") == 0)
  {
    std::vector<double> p = {2.5, 2.5, 0.05, 1.25, -15.0, 300.0, 285.0, 1.15, 0.01};
    return p;
  }

  /*
  else if(modelType.compare("SliceAdaptation") == 0)
  {
    std::vector<double> p = {1.0, 4.0, 1.0, 1.0, -100.0, 25.0, 50.0, -0.05};
    return p;
  }
  else if(modelType.compare("MonkeyAdaptation") == 0)
  {
    std::vector<double> p = {1.0, 4.0, 1.0, 1.0, -100.0, 25.0, 50.0, -0.05};
    return p;
  }
  else if(modelType.compare("MonkeyStimField") == 0)
  {
    std::vector<double> p = {1.0, 4.0, 1.0, 1.0, -100.0, 25.0, 50.0, -0.05};
    return p;
  }
  else if(modelType.compare("Monkey") == 0)
  {
    std::vector<double> p = {1.0, 4.0, 1.0, 1.0, -100.0, 25.0, 50.0, -0.05};
    return p;
  }
  */
  else if (modelType.compare("None") == 0)
  {
    return std::vector<double>();
  }
  else
  {
    LFATAL(" not a valid SCModelType");
    return std::vector<double>();
  }
}

// ######################################################################
void SCNeuralFitErrorAdapter::getSCParamRange(std::vector<double> & min, std::vector<double> & max)
{
  std::string modelType = itsSCModelType.getVal();

  if (modelType.compare("SCsSigmoid") == 0) 
  {
    std::vector<double> pmin = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> pmax = {25.0, 10.0, 10.0, 10.0, -1000.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, -1.0, 2.0, -2.0, 1.0, 25.0};
    min = pmin;
    max = pmax;
  }
  else if (modelType.compare("SCsRectify") == 0)
  {
    std::vector<double> pmin = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> pmax = {25.0, 10.0, 10.0, 10.0, -1000.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, -1.0, 2.0, -2.0};
    min = pmin;
    max = pmax;
  }
  else if (modelType.compare("SCsSigmoidFixed") == 0) 
  {
    std::vector<double> pmin = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> pmax = {25.0, 10.0, 10.0, 10.0, -1000.0, 300.0, 300.0, 1.0, 25.0};
    min = pmin;
    max = pmax;
  }
  else if (modelType.compare("SCsRectifyFixed") == 0)
  {
    std::vector<double> pmin = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> pmax = {25.0, 10.0, 10.0, 10.0, -1000.0, 300.0, 300.0};
    min = pmin;
    max = pmax;
  }
  else if (modelType.compare("SCsSigmoidFixedNoBound") == 0) 
  {
    std::vector<double> pmin = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> pmax = {25.0, 10.0, 10.0, 10.0, -1000.0, 300.0, 300.0, -2.0, 2.0, 1.0, 25.0};
    min = pmin;
    max = pmax;
  }
  else if (modelType.compare("SCsRectifyFixedNoBound") == 0)
  {
    std::vector<double> pmin = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> pmax = {25.0, 10.0, 10.0, 10.0, -1000.0, 300.0, 300.0, -2.0, 2.0};
    min = pmin;
    max = pmax;
  }
  else if (modelType.compare("LowPassSCsSigmoid") == 0)
  {
    std::vector<double> pmin = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> pmax = {25.0, 10.0, 10.0, 10.0, -1000.0, 300.0, 300.0, -1.0, 1.00, 25.0};
    min = pmin;
    max = pmax;
  }
  else if(modelType.compare("LowPassSCsRectify") == 0)
  {
    std::vector<double> pmin = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> pmax = {25.0, 10.0, 10.0, 10.0, -1000.0, 300.0, 300.0, -1.0};
    min = pmin;
    max = pmax;
  }

  else if(modelType.compare("SliceAdaptation") == 0)
  {
    std::vector<double> pmin = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> pmax = {25.0, 10.0, 10.0, 10.0, -1000.0, 300.0, 300.0, -2.0, 2.0, 1.0, 25.0};
    min = pmin;
    max = pmax; 
  }

  /*
  else if(modelType.compare("SliceAdaptation") == 0)
  {
  }
  else if(modelType.compare("MonkeyAdaptation") == 0)
  {
  }
  else if(modelType.compare("MonkeyStimField") == 0)
  {

  }
  else if(modelType.compare("Monkey") == 0)
  {
  }
  */
  else if (modelType.compare("None") == 0)
  {
    min = std::vector<double>();
    max = std::vector<double>();
  }
  else
    LFATAL(" not a valid SCModelType");
}

// ######################################################################
std::vector<double> SCNeuralFitErrorAdapter::getOtherParams()
{
  return std::vector<double>();
}

// ######################################################################
void SCNeuralFitErrorAdapter::getOtherParamRange(std::vector<double> & min, std::vector<double> & max)
{
  min = std::vector<double>();
  max = std::vector<double>();
}

// ######################################################################
nsu::SimStructure* SCNeuralFitErrorAdapter::getSCModel(std::vector<double> const &  params)
{
  
  Dims const d  = itsMapDims;
  uint const w  = d.w();
  uint const h  = d.h();
  std::string modelType = itsSCModelType.getVal();
  BorderPolicy bp = (itsUseSpaceVariant.getVal()) ? BorderPolicy::CROSS_HEMI : BorderPolicy::NONE;
  SimTime time = itsModelTimeStep.getVal();

  if (modelType.compare("None") == 0)    
    return NULL;

    //Sigmoid model
  else if (modelType.compare("SCsSigmoid") == 0)    
  {
    if (params.size() != 16)
    {
      LFATAL("number of parameters is not correct");
      return NULL;
    }
    else
   return new nsu::SCsSigmoid(params[0], params[1], params[2], params[3], params[4], params[5], params[6], params[7], params[8], params[9], params[10], params[11], params[12], params[13], params[14], params[15], bp, time, w, h);
  }

  //Rectify model
  else if(modelType.compare("SCsRectify") == 0)
  {
    if (params.size() != 14)
    {
      LFATAL("number of parameters is not correct");
      return NULL;
    }
    else
      return new nsu::SCsRectify(params[0], params[1], params[2], params[3], params[4], params[5], params[6], params[7], params[8], params[9], params[10], params[11], params[12], params[13], bp, time, w, h);
  }

  //Sigmoid model fixed
  else if (modelType.compare("SCsSigmoidFixed") == 0)    
  {
    if (params.size() != 9)
    {
      LFATAL("number of parameters is not correct");
      return NULL;
    }
    else
      return new nsu::SCsSigmoid(params[0], params[1], params[2], params[3], params[4], 5.0, 150.0, 6.0, 150.0, params[5], params[6], -0.05, 1.0, -1.0, params[7], params[8], bp, time, w, h);
  }
  
  //Rectify model fixed
  else if(modelType.compare("SCsRectifyFixed") == 0)
  {
    if (params.size() != 7)
    {
      LFATAL("number of parameters is not correct");
      return NULL;
    }
    else
      return new nsu::SCsRectify(params[0], params[1], params[2], params[3], params[4], 5.0, 150.0, 6.0, 150.0, params[5], params[6], -0.05, 1.0, -1.0, bp, time, w, h);
  }

  //Sigmoid model fixed
 else if (modelType.compare("SCsSigmoidFixedNoBound") == 0)    
  {
    if (params.size() != 11)
    {
      LFATAL("number of parameters is not correct");
      return NULL;
    }
    else
      return new nsu::SCsSigmoid(params[0], params[1], params[2], params[3], params[4], 5.0, 150.0, 6.0, 150.0, params[5], params[6], -0.05,  params[7], params[8], params[9], params[10], bp, time, w, h);
  }
  
  //Rectify model fixed
  else if(modelType.compare("SCsRectifyFixedNoBound") == 0)
  {
    if (params.size() != 9)
    {
      LFATAL("number of parameters is not correct");
      return NULL;
    }
    else
      return new nsu::SCsRectify(params[0], params[1], params[2], params[3], params[4], 5.0, 150.0, 6.0, 150.0, params[5], params[6], -0.05, params[7], params[8], bp, time, w, h);
  }

  //Lowpass Sigmoid model
  else if (modelType.compare("LowPassSCsSigmoid") == 0)
  {
  if (params.size() != 10)
    {
      LFATAL("number of parameters is not correct");
      return NULL;
    }
    else
      return new nsu::LowpassSCsSigmoid(params[0], params[1], params[2], params[3], params[4], params[5], params[6], params[7], params[8], params[9], bp, time, w, h);
  }

  //Lowpass Rectify model
  else if(modelType.compare("LowPassSCsRectify") == 0)
  {
    if (params.size() != 8)
    {
      LFATAL("number of parameters is not correct");
      return NULL;
    }
    else
      return new nsu::LowpassSCsRectify(params[0], params[1], params[2], params[3], params[4], params[5], params[6], params[7], bp, time, w, h);
  }

  //slice adaptation
  //Sigmoid model fixed
 else if (modelType.compare("SliceAdaptation") == 0)    
  {
    if (params.size() != 9)
    {
      LFATAL("number of parameters is not correct");
      return NULL;
    }
    else
      return new nsu::SCsRectify(params[0], params[1], params[2], params[3], params[4], 5.0, 150.0, 6.0, 150.0, params[5], params[6], -0.05, params[7], params[8], bp, time, w, h);
  }

  else
  {
    LFATAL("Bogus model type, sucka!");
    return NULL;
  }
}

// ######################################################################
NeuralFitError::DataSet SCNeuralFitErrorAdapter::computeModelResponse(std::vector<double> const & params)
{
  uint const plotdepth = its2DPlotDepth.getVal();
  SimTime timestep = getTimeStep(); 
  NeuralFitError::DataSet out;
  
  //get the params for the model
  std::vector<double> mparams(params.begin()+numOtherParams, params.end());
  std::vector<double> oparams(params.begin(), params.begin() + numOtherParams);
  
  //loop over all the conditions/stimuli
  for (uint i = 0; i < getNumStims(); ++i)
  {
    //allow derived classes to reset variables on each new stimulus
    reset(oparams);

    if (hasDiagnostic())
      LINFO("processing stimulus: %d", i);
    
    //initialize a model
    itsLastTime = SimTime::ZERO();    
    itsModel.reset(getSCModel(mparams));
    itsOutFrameCount = 0;

    if (hasDemoPlot() && itsModel.is_valid())
    {
      Range<double> itsRange = itsDisplayRange.getVal();
      nsu::NormalizeType ntype;
      if ((itsRange.min() < 0) && (itsRange.max() < 0))//if both are less than 0 scale
      {
        ntype = nsu::SCALE;
        itsRange = Range<double>(0.0,0.0);
      }
      else if ((itsRange.min() == 0) && (itsRange.max() == 0))//set to min/max of data
      {
        ntype = nsu::RANGE;
      }
      else //set to auto scale at each time
      {
        ntype = nsu::SET_RANGE;
      }
      itsModelPlot.reset(new nsu::StructurePlot(*itsModel, plotdepth, ntype, itsRange.min(), itsRange.max()));
      itsModelPlot->setSamplingRate(timestep); 
      //nsu::Location location(itsStimLoc.i, itsStimLoc.j); 
      //location.setHyperCubeDims(itsMapDims.w(), itsMapDims.h());
      //itsModelPlot->setProbe(location);
    }

    //loop over all the images in one stimulus 
    std::vector<double> output;    
    for (uint j = 0; j < getNumSamples(i); ++j)
    {
      //update simulation time
      itsLastTime += timestep; 
      
      //get a processed input image
      Image<double> resp = processImage(oparams, i, j);
      
      ///compute the model response if we have one
      if (itsModel.is_valid())
      {
        itsModel->input(resp);
        itsModel->evolve(itsLastTime);
        resp = itsModel->getOutput();
      }

      //apply any post processing
      resp = postProcessMap(resp);
      itsLastModelOutput = resp;
      
      //store the data
      switch (itsProbeType)
      {
      case ProbeType::Probe:
        {
          itsLastModelResponse = resp.getVal(itsStimLoc);
          break;
        }
        
      case ProbeType::Avg:
        {
          itsLastModelResponse = (double)getLocalAvg(resp, itsStimLoc, itsRFSize);
          break;
        }
        
      case ProbeType::Max:
        {
          itsLastModelResponse = (double)getLocalMax(resp, itsStimLoc, itsRFSize);
          break;
        }
        
      default :
        LFATAL("Not a valid probe type");
      }

      output.push_back(itsLastModelResponse);

      //plot a demo if desired
      if (hasDemoPlot())
      {
        itsLastNeuralSample = getNeuralResponse()[i][j];
        demoPlot(itsLastTime);
      }
    }

    //include response to one stimulus into the dataset
    out.push_back(output);
  }
  
  return out;
}

// ######################################################################
void SCNeuralFitErrorAdapter::demoPlot(const SimTime& time)
{
  if (!getOutputFrameSeries()->becameVoid())
  {
    //    if (time.msecs() > itsOutFrameCount * 33.333F/2.0F)
    //    {
    getOutputFrameSeries()->updateNext();
    
    combineDemoElements(itsLayout);
    getOutputFrameSeries()->writeRgbLayout(itsLayout, "Demo plot");
    itsLayout = Layout<PixRGB<byte> >();
    
    if (getOutputFrameSeries()->shouldWait())
      Raster::waitForKey();
    
    ++itsOutFrameCount;
    //    }
  }
}

// ######################################################################
void SCNeuralFitErrorAdapter::showDemoPlot(bool const value)
{
  if (hasDiagnostic() && value)
    LFATAL("Cannot have the diagnostic plot and the demo plot at the same time");
  
  itsShowDemoPlot = value;
}

// ######################################################################
bool const SCNeuralFitErrorAdapter::hasDemoPlot() const
{
  return itsShowDemoPlot;
}

// ######################################################################
void SCNeuralFitErrorAdapter::reset(const std::vector<double>& params)
{
}

// ######################################################################
void SCNeuralFitErrorAdapter::combineDemoElements(Layout<PixRGB<byte> > & layout)
{
}

// ######################################################################
// Stub implementation
// ######################################################################
SCNeuralFitErrorStub::SCNeuralFitErrorStub(OptionManager& mgr,
                                           const std::string& descrName,
                                           const std::string& tagName ):
    SCNeuralFitErrorAdapter(mgr, descrName, tagName)
{
}

// ######################################################################
SCNeuralFitErrorStub::~SCNeuralFitErrorStub()
{
}

// ######################################################################
void SCNeuralFitErrorStub::readStimulusFile(std::string const & fileName, const uint subsample)
{
}

// ######################################################################
uint SCNeuralFitErrorStub::getNumStims() const
{
  return 0;
}

// ######################################################################
uint SCNeuralFitErrorStub::getNumSamples(uint const index) const
{
  return 0;
}

// ######################################################################
Image<double> SCNeuralFitErrorStub::processImage(std::vector<double> const & params, uint const index, uint const sample)
{
  return Image<double>();
}

// ######################################################################
// implementation for SliceFitError
// ######################################################################
SliceFitError::SliceFitError(OptionManager & mgr, 
                             std::string const & descrName, std::string const & tagName):
    SCNeuralFitErrorAdapter(mgr, descrName, tagName), itsStimData(), itsLastStim(), itsRescaleDims()
    
{
}

// ######################################################################
void SliceFitError::start1()
{

  //set sampling rate of the data file
  setSamplingRate(SimTime::HERTZ(500));

  //call base class version first
  SCNeuralFitErrorAdapter::start1();

  uint const sizex = itsDemoPlotDims.getVal().w() / 2;
  uint const factor = sizex / itsMapDims.w();
  uint const sizey = itsMapDims.h() * factor;
  itsRescaleDims = Dims(sizex, sizey);
}

// ######################################################################
std::vector<double> SliceFitError::getOtherParams()
{
  //a gain parameter
  std::vector<double> otherParam(1,1.0);
  return otherParam;
}

// ######################################################################
void SliceFitError::getOtherParamRange(std::vector<double> & min, std::vector<double> & max)
{
  min = std::vector<double>(1, 0.0);
  max = std::vector<double>(1, 25.0);
}

// ######################################################################
void SliceFitError::readStimulusFile(std::string const & fileName, const uint subsample)
{
  std::ifstream stimFile(fileName);
  if (!stimFile.is_open())
    LFATAL("Error: cannot open the file");

  ImageSet<double> stimVideo;//stimulus video
  int lineCnt = 0;
  std::string line;
  double sample;
  Image<double> oneFrame;
  
  //read each line of the input stimuls file placing the stimulus at the desired location
  while (getline(stimFile, line))
  {
    std::vector<std::string> toks;
    split(line, " ", back_inserter(toks));
    
    if (toks.size() != 1)
      LFATAL("invalid line in %s (expected 1 tokens, got %d:\n%s", fileName.c_str(), int(toks.size()), line.c_str()); 

    if (lineCnt % subsample == 0)
    {
      //extract data from each line 
      sample = fromStr<double>(toks[0]);
      
      oneFrame = Image<double>(itsMapDims, ZEROS);
      oneFrame.setVal(itsStimLoc, sample);
      
      //add this frame to the sequence of images 
      stimVideo.push_back(oneFrame);
    }
    lineCnt++; 
  }
    
  //close the file stream
  stimFile.close();
  
  itsStimData.push_back(stimVideo);
  LINFO("Stimulus file '%s' loaded. %d samples ", fileName.c_str(), stimVideo.size());
}

// ######################################################################
uint SliceFitError::getNumStims() const
{
  return itsStimData.size();
}

// ######################################################################
uint  SliceFitError::getNumSamples(uint const index) const
{
  return itsStimData[index].size();
}

// ######################################################################
Image<double> SliceFitError::processImage(std::vector<double> const & params, uint const index, uint const sample)
{
  if (params.size() != 1)
    LFATAL("Other param has wrong number of values");

  Image<double> stim = itsStimData[index][sample];
  stim *= params[0];

  if (hasDemoPlot())
  {
    Image<byte> bimg = stim * 255.0;
    bimg = rescaleBilinear(bimg, itsRescaleDims);
    itsLastStim = toRGB(bimg);
  }

  return stim;
}

// ######################################################################
void SliceFitError::combineDemoElements(Layout<PixRGB<byte> > & layout)
{
  Layout<PixRGB<byte> > stim(itsLastStim);
  
  if (itsModel.is_valid())
    LFATAL("Need a model other than 'None' to show the demo plot");
  
  Layout<PixRGB<byte> > sc = itsModelPlot->drawStructure(*itsModel, 
                                                    itsRescaleDims.w(), 
                                                    itsRescaleDims.h(),
                                                    its2DPlotDepth.getVal(),
                                                    itsDisplayOutputType.getVal());
  
  Layout<PixRGB<byte> > line = itsModelPlot->drawLinePlots(*itsModel, itsLastNeuralSample, 
                                                      itsDemoPlotDims.getVal().w(),
                                                      itsDemoPlotDims.getVal().h() - itsRescaleDims.h(), 
                                                      Dims(0,1), 
                                                      itsModelPlotLength.getVal(),
                                                      itsProbeDepth.getVal(),
                                                      itsDisplayOutputType.getVal());

  layout = vcat(hcat(stim, sc), line);
}

// ######################################################################
// implementation for MonkeyFitError
// ######################################################################
MonkeyFitError::MonkeyFitError(OptionManager & mgr, std::string const & descrName,
                               std::string const & tagName): 
    SCNeuralFitErrorAdapter(mgr, descrName, tagName), itsFrames()
{
  itsUseSpaceVariant.setVal(false);
}    
    
// ######################################################################
void MonkeyFitError::readStimulusFile(std::string const & fileName, const uint subsample)
{ 
  std::ifstream stimFile(fileName);
  if (!stimFile.is_open())
    LFATAL("Error: cannot open the file");
  
  int lineCnt = 0, numStim = 0;
  std::string line;
  Frame oneFrame;
  Point stim;
  std::vector<Frame> oneFileFrames;
  //read each line of the input stimuls file placing the stimulus at the desired location
  while (getline(stimFile, line))
  {
    std::vector<std::string> toks;
    split(line, " ", back_inserter(toks));
    
    //line --> fixPointX, fixPonitY, fixPointR, fixPointG, fixPointB, fixPointSize, stim1X, stim1Y, stim1R, stim1G, stim1B,
    //stim1size, ....stimNsize
    if (toks.size() < 12)
      LFATAL("invalid line in %s (minimum number of values per line is 12, got %d:\n%s", fileName.c_str(), int(toks.size()), line.c_str()); 
    
    //extract fixation point data from each line 
    oneFrame.fixation.x = fromStr<double>(toks[0]);
    oneFrame.fixation.y = fromStr<double>(toks[1]);
    oneFrame.fixation.R = fromStr<uint>(toks[2]);
    oneFrame.fixation.G = fromStr<uint>(toks[3]);
    oneFrame.fixation.B = fromStr<uint>(toks[4]);
    oneFrame.fixation.diam = fromStr<double>(toks[5]);
    numStim = toks.size() / 6 - 1; // 6 properties per simuls, subtract 1 for fixation point
    for(int i=0; i<numStim; ++i)
    {
      stim.x = fromStr<double>(toks[0]);
      stim.y = fromStr<double>(toks[1]);
      stim.R = fromStr<uint>(toks[2]);
      stim.G = fromStr<uint>(toks[3]);
      stim.B = fromStr<uint>(toks[4]);
      stim.diam = fromStr<double>(toks[5]);
      oneFrame.stimuli.push_back(stim);
    }
    //add this frame to the sequence 
    oneFileFrames.push_back(oneFrame);
    lineCnt++; 
  }
  //close the stream
  stimFile.close();
  
  itsFrames.push_back(oneFileFrames);
  LINFO("Stimulus file '%s' loaded. %d frames ", fileName.c_str(), (uint)itsFrames.size());
}

// ######################################################################
std::vector<double> MonkeyFitError::getOtherParams()
{
  //A gain parameter
  std::vector<double> otherParam(1,1.0);
  return otherParam;
}

// ######################################################################
void MonkeyFitError::getOtherParamRange(std::vector<double> & min, std::vector<double> & max)
{
  min = std::vector<double>(1, 0.0);
  max = std::vector<double>(1, 2.0);
}

// ######################################################################
uint MonkeyFitError::getNumStims() const
{
  return itsFrames.size();
}


// ######################################################################
uint MonkeyFitError::getNumSamples(uint const index) const
{
  return itsFrames[index].size();
}


// ######################################################################
Image<double> MonkeyFitError::processImage(std::vector<double> const & params, uint const index, uint const sample)
{

  //check to see the file format, is it in pixels or degrees....

  /*
  Frame fr = itsFrames[index][sample];
  std::vector<Point> stim = fr.stimuli;

  Image<double> oneFrame = Image<double>(itsMapDims, ZEROS);
  for (uint i=0; i<stim.size(); ++i)
  {
    oneFrame[stim[i].x][stim[i].y] = 1;
  }
  */
  return Image<double>();
}

// ######################################################################
// Freeviewing implementation
// ######################################################################
FreeviewingFitErrorBase::FreeviewingFitErrorBase(OptionManager& mgr,
                                         const std::string& descrName,
                                         const std::string& tagName ):
    SCNeuralFitErrorAdapter(mgr, descrName, tagName), 
    itsEyeFileNames(&OPT_EyeFileNames,this), 
    itsNormFileName(&OPT_NormFileName, this), 
    itsFreeDataTime(&OPT_FreeDataTime, this), 
    itsFeatureMapDims(&OPT_FeatureMapDims, this),
    itsChanGain(&OPT_ChannelGain, this), 
    itsStimData(), 
    itsChanRange(),
    doNormalize(false),
    itsLastRetinal(),
    itsLastChanMaps(),
    itsLastSCInput(),
    itsLastEyeData(), 
    itsHeye(), 
    itsVeye(), 
    itsNeuralPlot(),
    itsModelPlot1D(),
    itsCondition(0),
    itsStimLocHD(), itsStimLocMaps(), 
    itsRFSizeHD(0), itsRFSizeMaps(0), itsRange()
{
}

// ######################################################################
FreeviewingFitErrorBase::~FreeviewingFitErrorBase()
{
}

// ######################################################################
void FreeviewingFitErrorBase::start1()
{
  //set the sampling rate from the command line
  setSamplingRate(itsFreeDataTime.getVal());
  
  //call base class version first - readStimulusFile gets called here
  SCNeuralFitErrorAdapter::start1();

  if (getSubSampleRatio() > 1)
    LFATAL("Subsampling value of %d is not allowed. Subsampling is disabled for this error type. "
           "Resample the data files and rerun ezvision to compute features at a lower sampling rate", 
           getSubSampleRatio());
  
  //make sure every stimulus file has the same number of submaps
  LINFO("loaded %d mgz files", (int)itsStimData.size());
  std::vector<FreeData>::iterator data(itsStimData.begin()), end(itsStimData.end());
  uint const maps = (data++)->info.numMaps();
  while (data != end)
    if (maps != (data++)->info.numMaps())
      LINFO("Not all stimuli have the same number of feature maps");

  itsLastChanMaps = ImageSet<float>(maps);
  
  std::vector<std::string> eyeFnames;
  split(itsEyeFileNames.getVal(), ",", back_inserter(eyeFnames));
  
  if (eyeFnames.size() != getNumFiles())
    LFATAL("Need to have the same number of stimulus files, neural data files, and eye position files");

  //get the norm file
  data = itsStimData.begin();
  if (itsNormFileName.getVal().empty())
    doNormalize = false;
  else
  {
    itsChanRange = loadChanNormFile(itsNormFileName.getVal(), *data);
    doNormalize = true;
  }

  //loop through and load up all the eye fnames
  std::vector<std::string>::iterator name(eyeFnames.begin()), endeye(eyeFnames.end());
  NeuralFitError::DataSet const & neuralData = getNeuralResponse();
  uint c = 0;
  while (name != endeye)
  {
    getEyeTraceFreeViewingData(*data, *name);

    if (data->eye->period().hertz() != getSamplingRate().hertz())
      LFATAL("Eye tracking files must have the same sampling rate as the NModelParam "
             "itsSamplingRate which should be set by the constructor of each class "
             "that derives from NeuralFitError using setSamplingRate(SimTime).");

    if (data->samples != neuralData[c++].size())
      LFATAL("Neural data file and eye position file have a different number of samples");
    
    ++data; ++name;
  }
}

// ######################################################################
void FreeviewingFitErrorBase::start2()
{
  SCNeuralFitErrorAdapter::start2();
  
  itsPixelTransform = PixelUnitTransform(itsStimData[0].eye->ppd().ppdx(), 
                                         itsStimData[0].eye->ppd().ppdy(), 
                                         itsScreenPixels.getVal());  

  //setup our plot buffers
  SimTime const rate = getTimeStep();
  float const screendegx = itsPixelTransform.screen.w() / itsPixelTransform.ppdx / 2;
  float const screendegy = itsPixelTransform.screen.h() / itsPixelTransform.ppdy / 2;

  itsHeye.reset(SimTime::MSECS(itsModelPlotLength.getVal()), screendegx*-1.0F, screendegx, rate);
  itsVeye.reset(SimTime::MSECS(itsModelPlotLength.getVal()), screendegy*-1.0F, screendegy, rate);

  float maxval = 250.0F;
  NeuralFitError::NormalizeType type = getNormType();
  if ((type == MAX) || (type == MAXCOND))
    maxval = 1.0F;
  
  itsNeuralPlot.reset(SimTime::MSECS(itsModelPlotLength.getVal()), 0.0F, maxval, rate);
  itsModelPlot1D.reset(SimTime::MSECS(itsModelPlotLength.getVal()), 0.0F, itsDisplayRange.getVal().max(), rate);

  Range<double> d = itsDisplayRange.getVal();
  itsRange = Range<float>(d.min(), d.max());
  
  Point2D<float> stimloc = itsStimLocation.getVal();
  itsStimLocHD = itsPixelTransform.deg2Pix(stimloc);
  itsRFSizeHD = int(itsReceptiveFieldSize.getVal() * (itsPixelTransform.ppdx + itsPixelTransform.ppdy) / 2.0F);
  
  float const rati = ((float)itsStimLocHD.i / (float)itsPixelTransform.screen.w());
  float const ratj = ((float)itsStimLocHD.j / (float)itsPixelTransform.screen.h());
  float const ratrf = ((float)itsRFSizeHD / (float)itsPixelTransform.screen.w());

  Dims const featureDims = itsFeatureMapDims.getVal();
  itsStimLocMaps.i = int(rati * featureDims.w());
  itsStimLocMaps.j = int(ratj * featureDims.h());
  itsRFSizeMaps = int(ratrf * featureDims.w());

  itsStimLoc.i = int(rati * itsMapDims.w());
  itsStimLoc.j = int(ratj * itsMapDims.h());
  itsRFSize = int(ratrf * itsMapDims.w());
}

// ######################################################################
std::vector<double> FreeviewingFitErrorBase::getOtherParams()
{ 
  uint const maps = itsStimData[0].info.numMaps();

  //weighting for initial params
  std::vector<double> params(maps, 1.0 / (double)maps * itsChanGain.getVal());
  return params;
}

// ######################################################################
void FreeviewingFitErrorBase::getOtherParamRange(std::vector<double> & min, std::vector<double> & max)
{
  uint const maps = itsStimData[0].info.numMaps();
  
  //zero weighting for initial params
  min = std::vector<double>(maps, 0.0);
  max = std::vector<double>(maps, 1.0);
}

// ######################################################################
void FreeviewingFitErrorBase::readStimulusFile(std::string const & fileName, const uint subsample)
{ 
  //load the submap info and mgz file
  FreeData data;
  getMGZFreeViewingData(data, fileName);
  itsStimData.push_back(data);
}

// ######################################################################
uint FreeviewingFitErrorBase::getNumStims() const
{
  return itsStimData.size();
}

// ######################################################################
uint FreeviewingFitErrorBase::getNumSamples(uint const index) const
{
  return itsStimData[index].samples;
}

// ######################################################################
Image<float> FreeviewingFitErrorBase::postProcessFeatureMap(const Image<float> & map)
{
  return map;
}

// ######################################################################
Image<double> FreeviewingFitErrorBase::processImage(std::vector<double> const & params, uint const index, uint const sample)
{
  LINFO("processing frame : %d from condition : %d", sample, index);

  //load up all the maps for this frame - since users don't have access to this
  //function we don't need to do any checking to make sure that index and sample
  //haven't already been called and are being called in order. Note that we
  //assume that the maps have all been stored to the same dimensions and that if
  //no channel normalization file is supplied the maps are already normalized.

  //the first data.info.numMaps() params are for the maps. Grab any other parameters here
  itsCondition = index;

  //get the current movies data
  FreeData& data = itsStimData[itsCondition];

  //get the current eye position and state
  if (data.eye->hasData(sample))
    itsLastEyeData = data.eye->data(sample);
  else
    LFATAL("eye data exhausted early");
  

  std::vector<double>::const_iterator weight(params.begin()), end(params.begin()+data.info.numMaps());
  std::vector<Range<float> >::const_iterator range(itsChanRange.begin());
  
  uint c = 0;
  while (weight != end)
  {
    //get the current map
    GenericFrame frame = data.features->readFrame();
    if (!frame.initialized())
    {
      LINFO("Feature map mgz file exhausted early");
      return Image<double>(itsMapDims, ZEROS);
    }

    Image<float> featureMap = frame.asGrayF32();

    if (doNormalize)//normalize over min/max possible for this channel
      featureMap = remapRange(featureMap, *range, Range<float>(0.0, 1.0));
    
    //do any post processing
    featureMap = postProcessFeatureMap(featureMap);

    //apply a weight
    featureMap *= *weight; 

    //store the normalized, weighted maps
    itsLastChanMaps[c] = featureMap;
    ++weight; ++range; ++c;
  }

  //Linearly combine maps and rescale to SC input
  Image<float> scInput = sum(itsLastChanMaps);  
  scInput = rescaleNI(scInput, itsMapDims);
  itsLastSCInput = scInput;
  
  //update our retinal image if we have one
  if (data.retinal.is_valid() && hasDemoPlot())
  {
    GenericFrame frame = data.retinal->readFrame();
    if (!frame.initialized())
      LINFO("Retinal input mgz file exhausted early");
    else
      itsLastRetinal = frame.asRgb();
  }

  return scInput;
}

// ######################################################################
void FreeviewingFitErrorBase::reset(const std::vector<double>& params)
{
  if (hasDemoPlot())
  {
    itsHeye.reset();
    itsVeye.reset();
    itsNeuralPlot.reset();
    itsModelPlot1D.reset();
  }
}

// ######################################################################
void FreeviewingFitErrorBase::combineDemoElements(Layout<PixRGB<byte> > & layout)
{
  Dims const plotdims = itsDemoPlotDims.getVal();
  //uint const depth = its2DPlotDepth.getVal()+1;  
  uint const lineplotheight = plotdims.h() / 6;
  
  Layout<PixRGB<byte> > scplot;  
  //  if (itsModel.is_valid())//we have a valid model, itsModelPlot takes care of any RF stuff
  //    scplot = itsModelPlot->drawStructure(*itsModel, 
  //                                         plotdims.w() / 2 / depth, plotdims.h() / 2 / depth, depth,
  //                                         itsDisplayOutputType.getVal());

  //  else//we have feature output only, so plot a disk to show RF
  //  {
    Image<float> fimg = itsLastModelOutput;
    remapRange(fimg, itsRange, Range<float>(0.0, 255.0));
    Image<byte> bimg = fimg;
    Image<PixRGB<byte> > colimg = colorize(bimg, ColorMap::JET());
    drawCircle(colimg, itsStimLoc, itsRFSize, PixRGB<byte>(255,255,255), 4);
    colimg = rescaleBilinear(colimg, Dims(plotdims.w() / 2, plotdims.h() / 2));
    scplot = colimg;
    //  }
  
  //combo of movie frame and sc map
  if (itsLastRetinal.initialized())
  {
    drawCircle(itsLastRetinal, itsStimLocHD, (int)itsRFSizeHD, PixRGB<byte>(255,255,255), 4);
    layout = hcat(scplot, rescaleBilinear(itsLastRetinal, Dims(plotdims.w() / 2, plotdims.h() / 2)));
  }
  else
    layout = scplot;

  //get eye position in degrees
  Point2D<float> const pos = itsPixelTransform.pix2Deg(itsLastEyeData->position());
  itsHeye.push(itsLastTime, pos.i);
  itsVeye.push(itsLastTime, pos.j);
  
  Image<PixRGB<byte> > eyeposh = itsHeye.draw(layout.getWidth(), 
                                              lineplotheight,
                                              "Horizontal eye position", //title
                                              "Deg", //y label
                                              "",    //x label
                                              PixRGB<byte>(1,1,1),
                                              0, false);//number of x axis tick marks
  
  Image<PixRGB<byte> > eyeposv = itsVeye.draw(layout.getWidth(), 
                                              lineplotheight,
                                              "Vertical eye position", //probe location
                                              "Deg", //y label
                                              "",  //x label
                                              PixRGB<byte>(1,1,1),
                                              0, false);//number of x axis tick marks

  Layout<PixRGB<byte> > dataplot;  
  Image<PixRGB<byte> > pltn, pltm;
  itsNeuralPlot.push(itsLastTime, itsLastNeuralSample);
  pltn = itsNeuralPlot.draw(layout.getWidth(), 
                            lineplotheight,
                            "Spike Rate", //probe location
                            "Sp/S", //y label
                            "Time",  //x label
                            PixRGB<byte>(0,0,0),
                            0, false);//number of x axis tick marks
  
  itsModelPlot1D.push(itsLastTime, itsLastModelResponse);
  pltm  = itsModelPlot1D.draw(layout.getWidth(), 
                              lineplotheight,
                              "", //probe location
                              "Sp/S", //y label
                              "Time",  //x label
                              PixRGB<byte>(255,0,0),
                              0, true);//number of x axis tick marks
  
  dataplot = composite(pltn, pltm, PixRGB<byte>(255,255,255));
  
  layout = vcat(layout, vcat(eyeposh, eyeposv));
  layout = vcat(layout, dataplot);
  
  LINFO("Neural: %0.3F, Model: %0.3F", itsLastNeuralSample, itsLastModelResponse);
}

// ######################################################################
// Freeviewing SV implementation
// ######################################################################
FreeviewingFitErrorSV::FreeviewingFitErrorSV(OptionManager& mgr,
                                             const std::string& descrName,
                                             const std::string& tagName ):
    FreeviewingFitErrorBase(mgr, descrName, tagName), 
    itsRetinalDims(&OPT_RetinalDims, this),
    itsUseScMaxNorm(&OPT_UseScMaxNorm, this),
    itsUseChannelMaxNorm(&OPT_UseChannelMaxNorm, this),
    itsUseScSurprise(&OPT_UseScSurprise, this),
    itsUseChannelSurprise(&OPT_UseChannelSurprise, this),
    itsUseChanSurpKL(&OPT_UseSurpChanKL, this),
    itsChanSurp(), itsSCSurp(), 
    itsTransform(new SCTransformModule(getManager())),
    useChannelMaxNorm(false),
    useScMaxNorm(false),
    useChannelSurprise(false), 
    useScSurprise(false),
    itsNMChanW(0.0), itsNMSCW(0.0),
    itsNMChanWPos(0), itsNMSCWPos(0),
    itsSurpChanUPos(0), itsSurpSCUPos(0)
{
  itsUseSpaceVariant.setVal(true);
  this->addSubComponent(itsTransform);
}

// ######################################################################
void FreeviewingFitErrorSV::start1()
{
  itsScreenPixels.setVal(itsRetinalDims.getVal());
  FreeviewingFitErrorBase::start1();

  itsTransform->clear(itsRetinalDims.getVal());

  useChannelMaxNorm = itsUseChannelMaxNorm.getVal();
  useScMaxNorm = itsUseScMaxNorm.getVal();

  useChannelSurprise = itsUseChannelSurprise.getVal();
  useScSurprise = itsUseScSurprise.getVal();
}

// ######################################################################
void FreeviewingFitErrorSV::start2()
{
  FreeviewingFitErrorBase::start2();
  itsStimLoc = itsStimLocHD;
  itsRFSize  = itsRFSizeHD;
}

// ######################################################################
FreeviewingFitErrorSV::~FreeviewingFitErrorSV()
{
}

// ######################################################################
std::vector<double> FreeviewingFitErrorSV::getOtherParams()
{ 
  //weighting for initial params
  std::vector<double> params = FreeviewingFitErrorBase::getOtherParams();
  
  if (useChannelMaxNorm)
  {
    params.push_back(1.0);
    itsNMChanWPos = params.size() -1;
  }
  
  if (useScMaxNorm)
  {
    params.push_back(1.0);
    itsNMSCWPos = params.size() -1;
  }

  if (useChannelSurprise)
  {
    params.push_back(0.94);
    itsSurpChanUPos = params.size() -1;
  }

  if (useScSurprise)
  {
    params.push_back(0.99);
    itsSurpSCUPos = params.size() -1;
  }

  return params;
}

// ######################################################################
void FreeviewingFitErrorSV::getOtherParamRange(std::vector<double> & min, std::vector<double> & max)
{
  FreeviewingFitErrorBase::getOtherParamRange(min, max);
  
  if (useChannelMaxNorm)
  {
    min.push_back(0.0);
    max.push_back(1.0);
  }
  
  if (useScMaxNorm)
  {
    min.push_back(0.0);
    max.push_back(1.0);
  }

  if (useChannelSurprise)
  {
    min.push_back(0.0);
    max.push_back(0.99999999);
  }
  
  if (useScSurprise)
  {
    min.push_back(0.0);
    max.push_back(0.99999999);
  }
}

// ######################################################################
void FreeviewingFitErrorSV::reset(const std::vector<double>& params)
{
  if (useChannelMaxNorm)
    itsNMChanW = params[itsNMChanWPos];
  
  if (useScMaxNorm)
    itsNMSCW = params[itsNMSCWPos];

  if (useChannelSurprise)
  {
    SurpriseSC unit(params[itsSurpChanUPos], 0.0, 0.1, itsUseChanSurpKL.getVal());
    itsChanSurp = SurpriseMapSC(itsFeatureMapDims.getVal().w(), itsFeatureMapDims.getVal().h(), unit);
  }

  if (useScSurprise)
  {
    SurpriseSC unit(params[itsSurpSCUPos], 0.0, 0.0);
    itsSCSurp = SurpriseMapSC(itsFeatureMapDims.getVal().w(), itsFeatureMapDims.getVal().h(), unit);
  }
}

// ######################################################################
Image<float> FreeviewingFitErrorSV::postProcessFeatureMap(const Image<float> & map)
{
  Image<float> o = map;
  if (useChannelMaxNorm)
    o = SCmaxNormalizeFancy(o, 1, itsNMChanW);
  
  if (useChannelSurprise)
  {
    itsChanSurp.input(o);
    o = itsChanSurp.getOutput();
  }
  
  return o;
}

// ######################################################################
Image<float> FreeviewingFitErrorSV::postProcessMap(const Image<float> & map)
{
  Image<float> o = map;
  if (useScMaxNorm)
    o = SCmaxNormalizeFancy(o, 1, itsNMSCW);
  
  if (useScSurprise)
  {
    itsSCSurp.input(o);
    o = itsSCSurp.getOutput();
  }
  
  o = rescaleNI(o, itsFeatureMapDims.getVal());
  Image<float> r = itsTransform->inverseTransform(o);
  
  return r;
}

// ######################################################################
// Freeviewing SV implementation
// ######################################################################
FreeviewingFitErrorCT::FreeviewingFitErrorCT(OptionManager& mgr,
                                             const std::string& descrName,
                                             const std::string& tagName ):
    FreeviewingFitErrorBase(mgr, descrName, tagName), 
    itsUseScMaxNorm(&OPT_UseScMaxNorm, this),
    itsUseChannelMaxNorm(&OPT_UseChannelMaxNorm, this),
    itsUseScSurprise(&OPT_UseScSurprise, this),
    itsUseChannelSurprise(&OPT_UseChannelSurprise, this),
    itsChanSurp(), itsSCSurp(), 
    useChannelMaxNorm(false),
    useScMaxNorm(false),
    useChannelSurprise(false), 
    useScSurprise(false),
    itsNMChanW(0.0), itsNMSCW(0.0),
    itsNMChanWPos(0), itsNMSCWPos(0),
    itsSurpChanUPos(0), itsSurpSCUPos(0)
{
  itsUseSpaceVariant.setVal(false);
}

// ######################################################################
void FreeviewingFitErrorCT::start1()
{
  FreeviewingFitErrorBase::start1();

  useChannelMaxNorm = itsUseChannelMaxNorm.getVal();
  useScMaxNorm = itsUseScMaxNorm.getVal();

  useChannelSurprise = itsUseChannelSurprise.getVal();
  useScSurprise = itsUseScSurprise.getVal();
}

// ######################################################################
void FreeviewingFitErrorCT::start2()
{
  FreeviewingFitErrorBase::start2();
}

// ######################################################################
FreeviewingFitErrorCT::~FreeviewingFitErrorCT()
{
}

// ######################################################################
std::vector<double> FreeviewingFitErrorCT::getOtherParams()
{ 
  //weighting for initial params
  std::vector<double> params = FreeviewingFitErrorBase::getOtherParams();
  
  if (useChannelMaxNorm)
  {
    params.push_back(1.0);
    itsNMChanWPos = params.size() -1;
  }
  
  if (useScMaxNorm)
  {
    params.push_back(1.0);
    itsNMSCWPos = params.size() -1;
  }

  if (useChannelSurprise)
  {
    params.push_back(0.7);
    itsSurpChanUPos = params.size() -1;
  }

  if (useScSurprise)
  {
    params.push_back(0.99);
    itsSurpSCUPos = params.size() -1;
  }

  return params;
}

// ######################################################################
void FreeviewingFitErrorCT::getOtherParamRange(std::vector<double> & min, std::vector<double> & max)
{
  FreeviewingFitErrorBase::getOtherParamRange(min, max);
  
  if (useChannelMaxNorm)
  {
    min.push_back(0.0);
    max.push_back(1.0);
  }
  
  if (useScMaxNorm)
  {
    min.push_back(0.0);
    max.push_back(1.0);
  }

  if (useChannelSurprise)
  {
    min.push_back(0.0);
    max.push_back(0.99999999);
  }
  
  if (useScSurprise)
  {
    min.push_back(0.0);
    max.push_back(0.99999999);
  }
}

// ######################################################################
void FreeviewingFitErrorCT::reset(const std::vector<double>& params)
{
  if (useChannelMaxNorm)
    itsNMChanW = params[itsNMChanWPos];
  
  if (useScMaxNorm)
    itsNMSCW = params[itsNMSCWPos];

  if (useChannelSurprise)
  {
    SurpriseSC unit(params[itsSurpChanUPos], 0.0, 0.1);
    itsChanSurp = SurpriseMapSC(itsFeatureMapDims.getVal().w(), itsFeatureMapDims.getVal().h(), unit);
  }

  if (useScSurprise)
  {
    SurpriseSC unit(params[itsSurpSCUPos], 0.0, 0.0);
    itsSCSurp = SurpriseMapSC(itsFeatureMapDims.getVal().w(), itsFeatureMapDims.getVal().h(), unit);
  }
}

// ######################################################################
Image<float> FreeviewingFitErrorCT::postProcessFeatureMap(const Image<float> & map)
{
  Image<float> o = map;
  if (useChannelMaxNorm)
    o = SCmaxNormalizeFancy(o, 1, itsNMChanW);
  
  if (useChannelSurprise)
  {
    itsChanSurp.input(o);
    o = itsChanSurp.getOutput();
  }
  
  return o;
}

// ######################################################################
Image<float> FreeviewingFitErrorCT::postProcessMap(const Image<float> & map)
{
  Image<float> o = map;
  if (useScMaxNorm)
    o = SCmaxNormalizeFancy(o, 1, itsNMSCW);
  
  if (useScSurprise)
  {
    itsSCSurp.input(o);
    o = itsSCSurp.getOutput();
  }
  
  return o;
}


#endif
