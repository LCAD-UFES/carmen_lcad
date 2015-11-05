/*!@file Transport/Stimulus2D.H Class declarations for a layer of neurons */

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
// Primary maintainer for this file: David Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/Stimulus2D.H $

#ifndef TRANSPORT_STIMULUS2D_H_DEFINED
#define TRANSPORT_STIMULUS2D_H_DEFINED

#include "Component/ModelParam.H"
#include "Transport/FrameIstream.H"
#include "Image/Image.H"
#include "Util/SimTime.H"


// ######################################################################
//! A class for to represent a stimulus
// ######################################################################
class Stimulus2D : public FrameIstream
{
public:
  //default constructor
  Stimulus2D(OptionManager& mgr,
             const std::string& descrName = "Stimulus2D Stream",
             const std::string& tagName = "Stim2DStream");

  //default destructor
  ~Stimulus2D()
  { };

  //!set the filename and load the data in this case.
  void setConfigInfo(const std::string& cfg);

  //!set the frame number
  bool setFrameNumber(int n);

  //! Return the specifications of the next frame
  GenericFrameSpec peekFrameSpec();

  //! Get the natural inter-frame time for this frame source
  SimTime getNaturalFrameTime() const;

  //! Read a frame from the input source
  GenericFrame readFrame();

  //! get the current time into the stimulus
  const SimTime getTime() const;

  //! get the total time of the stimulus
  const SimTime getTotalTime() const;

  //! get the stimulus sampling rate
  const SimTime getSR() const;

  //! get the size of the image the stimulus will be rendered to
  const Dims getDims() const;

  //! create and return the next image by drawing a disk, increment counters
  Image<float> next(const int radius);

  //! skip to this time step
  bool skipTo(SimTime time);

private:
  Stimulus2D(const Stimulus2D& rhs);
  Stimulus2D& operator=(const Stimulus2D& rhs);

  OModelParam<int> itsRadius;
  Image<float> itsStim;
  SimTime itsSR, itsT;
  Dims itsDims;
};

#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
