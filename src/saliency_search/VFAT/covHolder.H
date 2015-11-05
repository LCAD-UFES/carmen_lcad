/*!@file VFAT/covHolder.H  holds properaties of objects
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/covHolder.H $
// $Id: covHolder.H 6003 2005-11-29 17:22:45Z rjpeters $
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

#ifndef COVHOLDER_H_DEFINED
#define COVHOLDER_H_DEFINED

#include "VFAT/targetInfo.H"

#include <fstream>
#include <iostream>

//! A general purpose container for holding guassian signatures
/*! This is a general purpose container for classes from the feature
    classifier. Note that there are no pointers and all data is copied
    this allows this object to be pulled off from the other code if
    need be which makes it portable to other machines or memory spaces
    so long as the object type is known.

    NOTE: To see how means and averages are computed along with how each
    sample is matched temporally, see the file covEstimate.C and the
    method covEstimate<T>::matchPmeanAccum(...) . This method does a
    nearest neighbor matching and computes temporal dynamics for each
    class.
*/
template <class FLOAT> class covHolder : public vfatTargetInfo<FLOAT>
{
public:
  inline covHolder();
  inline ~covHolder();
  //! resize this container to diminsions x samples
  inline void resize(unsigned long _dim, unsigned long _samples,
              FLOAT initVal);
  //! dump this objects values to a file
  inline void dumpMeToFile(std::string myname, std::string frame, bool init);
  //! was this object matched to another object?
  bool isMatched;
  //! Does this object have enough samples
  bool isLarge;
  //! number of samples in the set (Population in essence)
  unsigned long samples;
  //! sorted unique base ID of this file (by size)
  unsigned long sortID;
  //! unique matched ID of this object
  unsigned long matchID;
  //! angle of intersecting line from 0,0 to x,y in cov_meanRatio
  std::vector<std::vector<FLOAT> > meanTheta;
  //! mean ratio of x to y for any data point
  std::vector<std::vector<FLOAT> > meanRatio;
};

// ######################################################################
template <class FLOAT>
inline covHolder<FLOAT>::covHolder() : vfatTargetInfo<FLOAT>()
{}

// ######################################################################
template <class FLOAT>
inline covHolder<FLOAT>::~covHolder()
{}

// ######################################################################
template <class FLOAT>
inline void covHolder<FLOAT>::resize(unsigned long _dim,
                                     unsigned long _samples,
                                     FLOAT initVal)
{
  std::cerr << "COVHOLDER RESET\n";
  samples = _samples;
  vfatTargetInfo<FLOAT>::dim = _dim;
  std::string foo = "foo";
  std::vector<FLOAT> tempTD(vfatTargetInfo<FLOAT>::dim,initVal);
  meanRatio.resize(vfatTargetInfo<FLOAT>::dim,tempTD);
  meanTheta.resize(vfatTargetInfo<FLOAT>::dim,tempTD);
  vfatTargetInfo<FLOAT>::featureName.resize(vfatTargetInfo<FLOAT>::dim,foo);
  vfatTargetInfo<FLOAT>::resize(_dim);
}
// ######################################################################
template <class FLOAT>
inline void covHolder<FLOAT>::dumpMeToFile(std::string myname,
                                           std::string frame,
                                           bool init)
{
  std::string newFile;
  std::string newFile2;
  newFile = myname + ".out.covHolder.S.Sig";
  newFile2 = myname + ".out.covHolder.F.Sig";
  LINFO("DUMPING COVMATRIX TO %s",newFile.c_str());

  if(init)
  {
    std::ofstream outfile(newFile.c_str(),std::ios::out);
    outfile << "**************************************************\n";
    outfile << "Note: color values are normalized between 0 and 1\n";
    outfile << "FRAME"
            << "\tBaseID"     << "\tSortID"
            << "\tMatchID"    << "\tSamples"
            << "\tDiminsions" << "\tLifeSpan"
            << "\tPositionX"  << "\tPositionY"
            << "\tMinX"       << "\tMaxX"
            << "\tMinY"       << "\tMaxY\n";
    outfile.close();
  }

  std::ofstream outfile(newFile.c_str(),std::ios::app);
  outfile << frame
          << "\t" << vfatTargetInfo<FLOAT>::baseID
          << "\t" << sortID
          << "\t" << matchID << "\t" << samples
          << "\t" << vfatTargetInfo<FLOAT>::dim
          << "\t" << vfatTargetInfo<FLOAT>::lifeSpan
          << "\t" << vfatTargetInfo<FLOAT>::posX
          << "\t" << vfatTargetInfo<FLOAT>::posY
          << "\t" << vfatTargetInfo<FLOAT>::minX
          << "\t" << vfatTargetInfo<FLOAT>::maxX
          << "\t" << vfatTargetInfo<FLOAT>::minY
          << "\t" << vfatTargetInfo<FLOAT>::maxY << "\n";
  outfile.close();

  if(init)
  {
    std::ofstream outfile2(newFile2.c_str(),std::ios::out);
    outfile << "Note: feature value = bias*(norm*(value + trans))\n";
    outfile << "Also: Spatial norm = 1/image size\n\n";

    outfile2 << "FRAME\t" << "BaseID\t" << "MatchID";
    for(unsigned long x = 0; x < vfatTargetInfo<FLOAT>::dim; x++)
    {
      outfile2 << "\tFEATURE"
               << "\tMean"
               << "\tStandardDev"
               << "\tMinValue"
               << "\tMaxValue"
               << "\tEigenVal"
               << "\tSpeed" << "\tSpeedAvg"
               << "\tAccel" << "\tAccelAvg"
               << "\tDistance"
               << "\tBias"
               << "\tNorm"
               << "\tTranslate";
    }
    outfile2 << "\n";
    outfile2.close();
  }

  std::ofstream outfile2(newFile2.c_str(),std::ios::app);
  outfile2 << frame << "\t" << vfatTargetInfo<FLOAT>::baseID << "\t"
           << matchID;
  for(unsigned long x = 0; x < vfatTargetInfo<FLOAT>::dim; x++)
  {
    outfile2 << "\t" << vfatTargetInfo<FLOAT>::featureName[x]
             << "\t" << vfatTargetInfo<FLOAT>::mean[x]
             << "\t" << vfatTargetInfo<FLOAT>::STD[x]
             << "\t" << vfatTargetInfo<FLOAT>::min[x]
             << "\t" << vfatTargetInfo<FLOAT>::max[x]
             << "\t" << vfatTargetInfo<FLOAT>::eigenVal[x]
             << "\t" << vfatTargetInfo<FLOAT>::speed[x]
             << "\t" << vfatTargetInfo<FLOAT>::avgspeed[x]
             << "\t" << vfatTargetInfo<FLOAT>::accel[x]
             << "\t" << vfatTargetInfo<FLOAT>::avgaccel[x]
             << "\t" << vfatTargetInfo<FLOAT>::distance[x]
             << "\t" << vfatTargetInfo<FLOAT>::bias[x]
             << "\t" << vfatTargetInfo<FLOAT>::norm[x]
             << "\t" << vfatTargetInfo<FLOAT>::trans[x];
  }
  outfile2 << "\n";
  outfile2.close();

  /*
  outfile << "MeanTheta Matrix\n";
  for(unsigned long x = 0; x < dim; x++)
  {
    for(unsigned long y = 0; y < dim; y++)
    {
      outfile << meanTheta[x][y] << " ";
    }
    outfile << "\n";
  }
  outfile << "MeanRatio Matrix\n";
  for(unsigned long x = 0; x < dim; x++)
  {
    for(unsigned long y = 0; y < dim; y++)
    {
      outfile << meanRatio[x][y] << " ";
    }
    outfile << "\n";
  }
  */
  outfile.close();
  LINFO("DONE");
}

#endif
