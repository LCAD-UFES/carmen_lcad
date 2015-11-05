/*!@file VFAT/blobProp.H Basic image segmenter blob finder using color
 part of color tracker*/

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/blobProp.H $
// $Id: blobProp.H 4663 2005-06-23 17:47:28Z rjpeters $
//

#ifndef BLOBPROP_H_DEFINED
#define BLOBPROP_H_DEFINED

class blobProp
{
public:
  // hard Blob constraints

  //# spatial location constraint
  //! region of search for blobs if loss of track
  int BP_LOTbound;
  //! region of search for blobs otherwise
  int BP_bound;

  //! extra leway in bounds set from outside class
  int BP_softBound;
  int BP_lowBound;
  int BP_traj;
  int BP_sampleStart;
  int BP_maxTraj;
  int BP_maxSize;
  int BP_minSize;
  //!this is the maximum size a blob can be by windows size (2 = 1/2)
  float BP_maxFrameSize;
  //! minimum blob size in pixel mass
  int BP_minMass;
  //! maximum blob size in pixel mass
  int BP_maxMass;
  //! Low bound ratio for blob shape in height/width
  float BP_ratioMin;
  //! extra leway on min ratio set from outside class
  float BP_softRatioMin;
  //! High bound ratio for blob shape in height/width
  float BP_ratioMax;
  //! extra leway on max ratio set from outside class
  float BP_softRatioMax;
  //! smallest size for a cluster to be considered a good sample
  int BP_minClusterSize;
  //! set the weights in cluster for spatial relation
  float BP_clusterWeightSpat;
  //! set the weights in cluster for Hue
  float BP_clusterWeightH;
  //! set the weights in cluster for Sat
  float BP_clusterWeightS;
  //! set the weights in cluster for Val
  float BP_clusterWeightV;
  //! set initial standard deviation for color tracking after Hue
  float BP_clusterColorStdH;
  //! set initial standard deviation for color tracking after Sat
  float BP_clusterColorStdS;
  //! set initial standard deviation for color tracking after Val
  float BP_clusterColorStdV;
  //! check that blobs are within mass constraint
  bool BP_checkMass;
  //! check that blobs are within an x frame from last track
  bool BP_checkFrameX;
  //! check that blobs are within a y frame from last track
  bool BP_checkFrameY;
  //! check that blobs are withing a certain ratio contraint
  bool BP_checkSizeRatios;
};

#endif
