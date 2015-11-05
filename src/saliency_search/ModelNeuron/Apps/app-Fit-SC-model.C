//fit the SC model to experimentla data
/*
 */
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/app-Fit-SC-model.C $

#include "Component/ModelManager.H"
#include "Component/ModelOptionDef.H"
#include "Image/Image.H"
#include "Image/ImageSet.H"
#include "Image/Point2D.H"
#include "Util/StringUtil.H"
#include "Util/StringConversions.H"
#include "Util/SimTime.H"
#include "Fitting/NelderMead.H"
#include "ModelNeuron/SCNeuralFitErrorConfigurator.H"
#include "ModelNeuron/SCNeuralFitError.H"
#include "ModelNeuron/SCFitOpts.H"

#include <iostream>
#include <fstream>

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

//////////////////////////////////////////////////////////////////////////
// initialization of work queue
//////////////////////////////////////////////////////////////////////////
nsu::SimulationWorkQueue nsu::SimStructure::itsWorkQueue(4);

//////////////////////////////////////////////////////////////////////////
// Fitting code params
//////////////////////////////////////////////////////////////////////////
const ModelOptionDef OPT_OutputFile = { MODOPT_ARG(std::string), "OutputFile", &MOC_FitSC, OPTEXP_CORE,
                                        "File in which to save results", "results-file", '\0', "<>", ""};

const ModelOptionDef OPT_ProgramMode = { MODOPT_ARG(std::string), "ProgramMode", &MOC_FitSC, OPTEXP_CORE,
                                         "The mode in which the program runs.", "mode", '\0', "<Fit, Eval, Demo>", "Fit"};

const ModelOptionDef OPT_ModelParams = { MODOPT_ARG(std::string), "ModelParams", &MOC_FitSC, OPTEXP_CORE,
                                         "User supplied parameters. If empty, the defaults will be chosen.", "params", '\0', "<param1, param2,..., ParamN>", ""};

//////////////////////////////////////////////////////////////////////////
// main
//////////////////////////////////////////////////////////////////////////
int submain(const int argc, const char** argv)
{
  MYLOGVERB = LOG_INFO;  // suppress debug messages

  using namespace ParamListHelper;


  ModelManager manager("Fit an SC model to neural data");

  //variables for all of our command line paramters
  OModelParam<std::string> itsOutputFile(&OPT_OutputFile, &manager);
  OModelParam<std::string> itsProgramMode(&OPT_ProgramMode, &manager);
  OModelParam<std::string> itsModelParams(&OPT_ModelParams, &manager);

  nub::ref<SCNeuralFitErrorConfigurator> SCConf(new SCNeuralFitErrorConfigurator(manager));
  manager.addSubComponent(SCConf);

  //parse command line and start manager
  if (manager.parseCommandLine(argc, argv, "", 0, 0) == false)
    return(1);

  manager.start();

  //get some starting parameters
  std::vector<double> startParams;
  if (itsModelParams.getVal().empty())
    startParams = SCConf->getSC()->getStartingParams();
  else
  {
    SCConf->getSC()->getStartingParams();
    std::vector<std::string> toks;
    split(itsModelParams.getVal(), ",", back_inserter(toks));
    for (std::string const & param : toks)
      startParams.push_back(fromStr<double>(param));
  }

  std::vector<double> result;
  double error;
  NelderMead::FinishCode code = NelderMead::FinishCode::NONE;
  uint numIterations = 0;

  nub::ref<SCNeuralFitErrorAdapter> errorFunc = SCConf->getSC();

  //Fitting mode
  if (itsProgramMode.getVal().compare("Fit") == 0)
  {
    LINFO("Entering Fitting mode");

    //get some parameters for the minimizer
    ParamList<double, double, uint, double> nmParams = SCConf->getSC()->getNelderMeadParams();

    //get some parameters for the minimizer
    std::vector<double> paramMin, paramMax;
    SCConf->getSC()->getParamRange(paramMin, paramMax);
    
    //start our minimizer
    std::function<double const (std::vector<double> const &)> efunc = [errorFunc](std::vector<double> const & params)->double const { return (*errorFunc)(params); };
    
    NelderMead simplex(startParams, paramMin, paramMax, at<0>(nmParams), efunc, at<1>(nmParams), at<2>(nmParams), at<3>(nmParams), false, SCConf->getSC()->hasDiagnostic());

    //minimze and get the solution
    NelderMead::Result final = simplex.minimize();
    NelderMead::VertexPair solution = final.params;
    result = solution.first;
    error = solution.second; 
    code = final.code;
    numIterations = final.iterations;
  }
  //Eval mode
  else if(itsProgramMode.getVal().compare("Eval") == 0)
  {
    LINFO("Entering Eval");

    SCConf->getSC()->showDemoPlot(false);
    error = (*errorFunc)(startParams);
    result = startParams;
  }  
  //Demo mode
  else if(itsProgramMode.getVal().compare("Demo") == 0)
  {
    LINFO("Entering Demo");
    
    SCConf->getSC()->showDemoPlot(true);
    error = (*errorFunc)(startParams);
    result = startParams;
  }
  
  //output file or terminal
  if (itsOutputFile.getVal().empty())
  {
    LINFO("Stop criteria : %s", NelderMead::finishCodeToString(code).c_str());
    LINFO("Error : %3.10f", error);
    LINFO("Iterations : %d", numIterations);
    
    std::string ps;
    for (double const & p : result)
      ps = ps + " " + toStr(p);
    LINFO("%s", ps.c_str());
  }
  else
  {
    std::ofstream of(itsOutputFile.getVal());
    of << "Stop criteria: " << NelderMead::finishCodeToString(code) << std::endl;
    of << "Error: " << error << std::endl;
    of << "Iterations: " << numIterations << std::endl;
   
    for (double const & p : result)
      of << p << " ";
    of << std::endl;
    
    of.close();
  }
  
  return 0;  
}
#else
int submain(const int argc, const char** argv) {return 0; }; 
#endif

int main(const int argc, const char **argv)
{
  try
  {
    return submain(argc, argv);
  }
  catch (...)
  {
    REPORT_CURRENT_EXCEPTION;
  }
  return 1;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
