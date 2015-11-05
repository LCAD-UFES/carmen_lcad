/*!@file VFAT/segHolder.H  holds properaties of objects
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/segHolder.H $
// $Id: segHolder.H 6003 2005-11-29 17:22:45Z rjpeters $
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

#ifndef SEGHOLDER_H_DEFINED
#define SEGHOLDER_H_DEFINED

#include "VFAT/targetInfo.H"


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
template <class FLOAT, class INT, unsigned short dim> class segHolder :
  public vfatTargetInfo<FLOAT>
{
public:
  static const unsigned short dims = dim;
  inline segHolder();
  inline ~segHolder();

  //! is this tracker assigned yet to a target?
  bool                        noTargetYet;
  //! are we in a loss of track?
  bool                        LOT;
  //! did the loss of track cause us to reset the tracker?
  bool                        LOTandReset;
  //! what is the expected position of the target?
  unsigned short              expectedX;
  //! what is the expected position of the target?
  unsigned short              expectedY;
  //! what is the expected boundary of the target?
  unsigned short              expectedXmax;
  //! what is the expected boundary of the target?
  unsigned short              expectedYmax;
  //! what is the expected boundary of the target?
  unsigned short              expectedXmin;
  //! what is the expected boundary of the target?
  unsigned short              expectedYmin;
  //! boundary for tracking
  short                       boundaryX;
    //! boundary for tracking
  short                       boundaryY;
  //! number of loss of tracks in a row
  unsigned int                LOTcount;
  //! total number of blobs found this iteration
  unsigned int                blobNumber;
  //! how many blobs were killed
  unsigned int                killedBlobs;
  //! if we have a loss of track, what type is it
  unsigned int                LOTtype;
  //! how long have we been tracking without reset?
  unsigned long               totalLifeSpan;
  //! a name for the loss of track in LOTtype
  std::string                 LOTtypeName;
  //! this maps channels between covHolder and segHolder
  std::vector<unsigned short> channelMap;
  //! how much should any channel adapt
  std::vector<FLOAT>          channelAdapt;
  //! modifier from covHolder to tracker for standard dev.
  std::vector<FLOAT>          STDmod;
  //! modifier from covHolder to tracker for upper bound
  std::vector<FLOAT>          upperBoundMod;
  //! modifier from covHolder to tracker for lower bound
  std::vector<FLOAT>          lowerBoundMod;
};

 // ######################################################################
template <class FLOAT, class INT, unsigned short dim>
inline segHolder<FLOAT,INT,dim>::segHolder() : vfatTargetInfo<FLOAT>(dim)
{
  channelMap.resize(dim,0);
  channelAdapt.resize(dim,0.0F);
  STDmod.resize(dim,0.0F);
  upperBoundMod.resize(dim,0.0F);
  lowerBoundMod.resize(dim,0.0F);
  noTargetYet   = true;
  totalLifeSpan = 0;
}

// ######################################################################
template <class FLOAT, class INT, unsigned short dim>
inline segHolder<FLOAT,INT,dim>::~segHolder()
{}

#endif
