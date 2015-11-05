/*!@file VFAT/test-featureNPC.C  Test the non-parametric classifier */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/test-featureNPC.C $
// $Id: test-featureNPC.C 6003 2005-11-29 17:22:45Z rjpeters $
//

// ############################################################
// ############################################################
// ##### ---NPclassify---
// ##### non-parametric classifier:
// ##### T. Nathan Mundhenk nathan@mundhenk.com
// ##### Vidhya Navalpakkam - navalpak@usc.edu
// ##### partners full name - email
// ############################################################
// ############################################################

//This is the start of the execution path for the NPclassify test alg.

#include "Util/Timer.H"
#include "VFAT/NPclassify.H"

#include <fstream>
#include <iostream>

//! This is the configFile name
char* configFile;
//! This is the configFile object
readConfig configIn(25);
readConfig polySet(25);
//! number of items if training
int itemNumber;

int main(int argc, char* argv[])
{

  std::cerr << "STARTING\n";
  std::cerr << "Opening config file" << argv[1] << "\n";
  std::ifstream inFile(argv[1],std::ios::in);
  std::vector<double> dataIn(7,0.0F);
  std::vector< std::vector<double> > vectorIn(1000,dataIn);
  int column = 0;
  int row = 0;
  int family = 0;
  bool comment = false;

  std::string in;
  std::cerr << "Parsing config file" << argv[1] << "\n";
  while (inFile >> in)
  {

    if(!in.compare("#")) //comment code # found
    {
      if(!comment)
      {
        comment = true;
      }
      else      //end of comment
      {
        comment = false;
      }
    }
    if((!comment) && in.compare("#")) //real line found
    {
      if(column == 1)
      {
        //vectorIn[row][family] = atof(in.c_str());
        //std::cerr << "Adding " << in << "[" << row << "]"
        //     << "[" << family*2 << "]\n";
      }
      if(column == 3)
      {
        vectorIn[row][family] = atof(in.c_str());
        //std::cerr << "Adding " << in << "[" << row << "]"
        //     << "[" << (family*2)+1 << "]\n";
      }
      column++;
      if(column == 5)
      {
        column = 0;
        row++;
      }
      if(!in.compare("%"))
      {
        row = 0;
        column = 0;
        family++;
        std::cerr << "New Family " << family << "\n";
      }
    }
  }

  // start timer
  Timer tim;
  tim.reset();
  uint64 t0 = tim.get();  // to measure display time
  // get test image

  if(argc > 2)
    itemNumber = atoi(argv[2]);
  else
    itemNumber = -666;

  // create operating objects
  configIn.openFile("NPclassify.conf");
  polySet.openFile("polySet.conf");

  NPclassify NP(configIn,polySet,true);
  if(argc > 3)
  {
    NP.inputCommandLineSettings(atof(argv[3]),atof(argv[4]),atof(argv[5]),
                                atof(argv[6]),atoi(argv[7]),atoi(argv[8]),
                                atof(argv[9]),atof(argv[10]),atof(argv[11]));
  }

  //inport data

  std::vector<double> feature(2,0);
  std::vector<long> roots;
  std::vector<long> parents;
  std::vector<double> density;

  NP.addSpace(vectorIn,(row-1));
  NP.classifySpaceNew();
  uint64 t1 = tim.get();
  t0 = t1 - t0;

  roots = NP.getStems();
  parents = NP.getParents();
  density = NP.getDensity();

  std::vector<std::vector<long> > theReturn = NP.getChildren();
  for(int i = 0; i < NP.getStemNumber(); i++)
  {
    if(NP.getClassSize(i) > NP.getMinClassSize())
    {
      LINFO("CLASS %d size %ld",i,NP.getClassSize(i));
    }
  }
  //outport classification data

  NP.metaClassify(itemNumber);

  std::ofstream outfile("feature_trian.dat",std::ios::out);
  std::ofstream matfile("feature_trian_mat.dat",std::ios::out);
  for(int i = 0; i < NP.getStemNumber(); i++)
  {
    for(int j = 0; j < NP.getClassSize(i); j++)
    {
      long item = NP.getClass(i,j);
      outfile << item << "\t" << i << "\t" << j << "\t"
              << NP.getFeature(item,0) << "\t"
              << NP.getFeature(item,1) << "\n";
      matfile << item << "\t" << i << "\n";
    }
  }


}
