/*!@file Transport/DotStimuli.H A FrameIstream subclass for
  generating dot stimuli */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
// by the University of Southern California (USC) and the iLab at USC.  //
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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/RandomInput.H $
// $Id: RandomInput.H 6449 2006-04-10 23:33:51Z rjpeters $
//

#ifndef TRANSPORT_DOTSTIMULI_H_DEFINED
#define TRANSPORT_DOTSTIMULI_H_DEFINED

#include "Image/Dims.H"
#include "Transport/FrameIstream.H"
#include "rutz/rand.h"
#include "Image/Point2D.H"

//! FrameIstream subclass that generates random images
/*! This can be a handy way to quickly test an executable, if you
    don't have any images lying around, etc. -- you can just specify
    --in=random:256x256 on the command-line to configure your
    InputFrameSeries to use a RandomInput source. */
class DotStimuli : public FrameIstream
{
public:
  //! Constructor
  DotStimuli(OptionManager& mgr);

  //! Virtual destructor
  virtual ~DotStimuli();

  //! Override from FrameIstream; parse the options
  virtual void setConfigInfo(const std::string& options);

  //! Peek at the specifications of the next frame to be read
  virtual GenericFrameSpec peekFrameSpec();

  //! Generate a random color image
  virtual GenericFrame readFrame();

  //! Specify the dimensions of the random images
  void setDims(const Dims& s);

  //! Set the seed for our random-number generator
  void setSeed(int s);

private:

  //! configure parameters for various stimuli
  void setConfigRandom(const std::string& params);
  void setConfigSingle(const std::string& params);
  void setConfigPlanar(const std::string& params);
  void setConfigAsymmetry(const std::string& params, uint mode);
  void setConfigFoe(const std::string& params);
  void setConfigRotation(const std::string& params);
    
  //! get the image for various stimuli
  Image<float> getRandomDotStimuli(uint step);
  Image<float> getSingleDotStimuli(uint step);
  Image<float> getPlanarDotStimuli(uint step);
  Image<float> getGridPlanarDotStimuli(uint step);
  Image<float> getRandomPlanarDotStimuli(uint step);
  Image<float> getFoeStimuli(uint step);
  Image<float> getRotationDotStimuli(uint step);
  
  Dims        itsDims;
  rutz::urand itsGenerator;

  //! Focus of Expansion information
  bool itsIsRandom;

  bool itsIsPlanar;
  float itsPlanarDx;
  float itsPlanarDy;

  bool itsIsSingleDot;
  Point2D<float> itsSingleDotLocation;
  float itsSingleDotSize;
  float itsSingleDotDx;
  float itsSingleDotDy;

  bool itsIsFoe;
  Point2D<int> itsFoe;

  bool itsIsRotation;
  Point2D<int> itsRotationCenter;


  uint itsAsymmetryMode;
  uint itsStep;

  //! Random dots
  std::vector<Point2D<float> > itsRandomDots;
  std::vector<float> itsRandomDotSizes;
 
  //! Focus of Expansion dots
  std::vector<Point2D<float> > itsFoeDots;
  std::vector<float> itsFoeDotSizes;
  bool itsFoeHaveMotion;
  bool itsFoeHaveTempSGrad;
  bool itsFoeHaveSpatSGrad;   

  //! planar dots
  std::vector<Point2D<float> > itsPlanarDots;
  std::vector<float> itsPlanarDotSizes;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_RANDOMINPUT_H_DEFINED
