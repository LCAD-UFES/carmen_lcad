/*!@file Robots2/Beobot2/Navigation/FOE_Navigation/MiddleTemporal.H */
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
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Navigation/FOE_Navigation/MiddleTemporal.H
// $ $Id: $
//
//////////////////////////////////////////////////////////////////////////

#include "Image/Image.H"
#include "Image/Layout.H"
#include "Raster/Raster.H"

#include "Image/PyramidOps.H"
#include "Image/ImageSet.H"
#include "Image/DrawOps.H"
#include "Image/PyramidTypes.H"
#include "Image/PyrBuilder.H"
#include "Image/MathOps.H"
#include "GUI/XWinManaged.H"

#ifndef MIDDLETEMPORAL_H
#define MIDDLETEMPORAL_H

#define MAX_NEIGHBORHOOD       4


class MiddleTemporal
{
public:

  MiddleTemporal();

  void reset(uint numPyrLevel, uint numDirs, uint numSpeeds);

  ~MiddleTemporal();

  //! returns the MT feature overlay on the input image
  //! for display purposes 
  Layout<byte> getMTfeaturesDisplay(Image<byte> image);

  //! compute the Medial Temporal motion features
  void computeMTfeatures
  (std::vector<std::vector<ImageSet<float> > > itsRawSpatioTemporalEnergy);

  std::vector<Image<float> > getMTfeatures();
  std::vector<Image<float> > getMToptimalShift();

private:

  //! just to reset the sizes of the maps
  void reset();

  //! find the optimal speeds for each direction
  void         computeMaxSpeed(uint index);
  Image<float> computeMaxSpeed(uint index, int scale);

  //! compute the max values of each location
  void computeSteMaxVals();
  void computeDirSteMaxVals();

  //! normalize the motion detection values to 1.0
  void normalizeOnMaxVal();

  //! compute the ooponency values
  void computeOpponencies();

  //! center surround lateral inhibition for each direction
  Image<float> getOnSpotInhibitSpatioTemporalEnergy(uint dir, uint scale);

  //! lateral inhibition for each direction
  //! from original location 
  Image<float> getNeighborInhibitSpatioTemporalEnergy(uint dir, uint scale);

  //! we weigh MT features for dominance to particular direction
  void weighMTfeaturesForDominance();

  //! final calculations to compute MT features
  void findMaxMotionVals();

  //! printing procedures
  void printItsSpatioTemporalEnergy
  (uint si, uint ei, uint sj, uint ej, bool stop = false, float div = 1.0);
  void displayItsSpatioTemporalEnergy();
  void printItsMTfeatures(uint si, uint ei, uint sj, uint ej, bool stop);
  void printItsMToptimalShift(uint si, uint ei, uint sj, uint ej, bool stop);
  void displayItsMTfeatures();
  void displayItsMToptimalShift();

  uint  itsNumPyrLevels;
  uint  itsNumDirs;
  uint  itsNumSpeeds;


  //! max values within a neighborhood
  //! for all directions
  ImageSet<float> itsSteMaxVals;
  //! max values within a neighborhood
  //! for each direction
  std::vector<ImageSet<float> > itsDirSteMaxVals;

  //! raw motion energy for each direction
  std::vector<std::vector<ImageSet<float> > > itsRawSpatioTemporalEnergy;
  std::vector<ImageSet<float> > itsSpatioTemporalEnergy;
  std::vector<ImageSet<float> > itsSpatioTemporalEnergyOptimalShift;

  //! filtered: collapsed to the number of directions 
  //! this already includes:
  //!   lateral inhibition
  //!   center surround opponencies
  std::vector <Image<float> > itsMTfeatures;
  std::vector <Image<float> > itsMToptimalShift;

  rutz::shared_ptr<XWinManaged> itsWin;
};
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
