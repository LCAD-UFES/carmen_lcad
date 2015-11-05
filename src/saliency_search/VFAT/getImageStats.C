/*!@file VFAT/getImageStats.C  simplified version of vision.C with feature analysis
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
// Primary maintainer for this file: T. Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/getImageStats.C $
// $Id: getImageStats.C 6383 2006-03-24 00:39:47Z rjpeters $
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

#include "Component/ModelManager.H"
#include "Media/FrameSeries.H"
#include "Neuro/StdBrain.H"
#include "VFAT/featureClusterVision.H"

int main(const int argc, const char **argv)
{
  MYLOGVERB = LOG_INFO;  // suppress debug messages

  // Instantiate a ModelManager:
  ModelManager manager("Super Model");

  nub::soft_ref<InputFrameSeries> ifs(new InputFrameSeries(manager));
  manager.addSubComponent(ifs);

  nub::soft_ref<StdBrain> brain(new StdBrain(manager));
  manager.addSubComponent(brain);

  // feature analysis part of model
  const std::string name = "featureCluster";
  const std::string tag  = "fCV";


  if (manager.parseCommandLine(argc, argv,
                               "<image> <maskFile> <outFile> <label>",
                               4, 4) == false)
    return 1;
  std::string maskFile  = argv[2];
  std::string imageFile = argv[1];
  std::string outFile   = argv[3];
  std::string label     = argv[4];

  LINFO("RUNNING with %s %s %s %s",maskFile.c_str(),
        imageFile.c_str(), outFile.c_str(),label.c_str());

  nub::soft_ref<featureClusterVision<float> >
    fCV(new featureClusterVision<float>(manager,name,tag,brain,ifs,
                                        manager.getExtraArg(0)));
  manager.addSubComponent(fCV);

  /* Q: What happened to my getExtraArg() for my input/output?

     A: See docs at the head of src/Media/FrameSeries.H
  */

  // let's get all our ModelComponent instances started:
  manager.start();

  LINFO("RUNNING SIMPLE FEATURES");
  fCV->fCVgetImageBaseStats(maskFile,imageFile,outFile,label);
  LINFO("RUNNING COMPLEX FEATURES");
  fCV->fCVgetImageComplexStats(maskFile,imageFile,outFile,label);

}
