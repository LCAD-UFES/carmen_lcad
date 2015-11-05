//!gather statistics on sc model features

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/app-SC-Feature-test.C $

#include "Component/ModelManager.H"
#include "Component/ModelOptionDef.H"
#include "ModelNeuron/FreeViewingData.H"

//////////////////////////////////////////////////////////////////////////
// Fitting code params
//////////////////////////////////////////////////////////////////////////
const ModelOptionCateg MOC_SCFeatureTest = { MOC_SORTPRI_3, "SC Count Frames Options" };

const ModelOptionDef OPT_MGZFiles = { MODOPT_ARG(std::string), "MGZFiles", &MOC_SCFeatureTest, OPTEXP_CORE,
                                      "A comma separated list of mgz files containing features", "mgz-files", '\0', "<name1,name2,...nameN>", ""};

//////////////////////////////////////////////////////////////////////////
// main
//////////////////////////////////////////////////////////////////////////
int submain(const int argc, const char** argv)
{
  MYLOGVERB = LOG_INFO;  // suppress debug messages

  ModelManager manager("SC Count Frames");

  //variables for all of our command line paramters
  OModelParam<std::string> itsMGZFiles(&OPT_MGZFiles, &manager);

  //parse command line and start manager
  if (manager.parseCommandLine(argc, argv, "", 0, 0) == false)
    return(1);

  manager.start();
  
  //load up all the Mgz files 
  std::vector<FreeData> freedata = loadMGZFromList(itsMGZFiles.getVal());

  //get some info about the channels
  SubMapInfo mapinfo = freedata[0].info;
  
  //get some constants
  uint const numMaps = freedata[0].info.numMaps();
  
  //loop through all the frames of each file, display, and calcualte min-max values over all features
  std::vector<FreeData>::iterator data(freedata.begin()), dataend(freedata.end());

  while (data != dataend)
  {
    uint featureframecount = 0;
    uint mapframecount = 0;
    uint retinalframecount = 0;
    bool feature_skipped = true;
    bool retinal_skipped = true;

    while (1)
    {
      if (feature_skipped)
      {
        uint ii = 0;
        for (; ii < numMaps; ++ii)
        {
          feature_skipped = data->features->skipFrame();

          if (feature_skipped)
            ++mapframecount;
          else
            break;
        }

        if (feature_skipped)
          ++featureframecount;
        else
          LINFO("features exhausted while reading : %d", ii);
      }
      
      if ((data->retinal.get() != NULL) && (retinal_skipped))
      {
        retinal_skipped = data->retinal->skipFrame();
        
        if (retinal_skipped == true)
          ++retinalframecount;
        else
          LINFO("retinal exhausted");
      }

      if (!feature_skipped && !retinal_skipped)
        break;
    }

    LINFO("maps : %d, map count : %d,  feature: %d, retinal: %d", numMaps, mapframecount, featureframecount, retinalframecount);
    ++data;
  }
    
  return 0;  
  }

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
