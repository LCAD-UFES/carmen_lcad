/*!@file Transport/BarStimuli.H A FrameIstream subclass for
  generic bar stimuli */

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
// Primary maintainer for this file: Christian Siagian <siagian at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/BarStimuli.H $
// $Id: $
//

#ifndef TRANSPORT_BARSTIMULI_H_DEFINED
#define TRANSPORT_BARSTIMULI_H_DEFINED

#include "Image/Dims.H"
#include "Image/Point2D.H"
#include "Transport/FrameIstream.H"
#include "rutz/rand.h"

//! FrameIstream subclass that generates bar stimuli images
/*! This can be a handy way to quickly test an executable, if you
    don't have any images lying around, etc. Also has Aperture Problem
    stimuli -- you can just specify --in=bars:256x256 on the
    command-line to configure your InputFrameSeries to use a
    BarStimuli source. */
class BarStimuli : public FrameIstream
{
public:
  //! Constructor
  BarStimuli(OptionManager& mgr);

  //! Virtual destructor
  virtual ~BarStimuli();

  //! Override from FrameIstream; just calls setDims()
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
  void setConfigNormal(const std::string& params);
  void setConfigApertureProblem(const std::string& params);
    
  //! get the image for various stimuli
  Image<float> getNormalBarStimuli(uint step);
  Image<float> getApertureProblemStimuli(uint step);

  bool itsHaveMotion;

  bool itsIsNormal;
  bool itsIsApertureProblem;

  std::vector<Point2D<float> > itsBars;
  std::vector<float> itsBarLengths;

  uint itsStep;

  Dims        itsDims;
  rutz::urand itsGenerator;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_BARSTIMULI_H_DEFINED
