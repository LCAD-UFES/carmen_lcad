/*!@file Transport/GameOfLifeInput.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/GameOfLifeInput.H $
// $Id: GameOfLifeInput.H 8736 2007-08-31 19:01:40Z rjpeters $
//

#ifndef TRANSPORT_GAMEOFLIFEINPUT_H_DEFINED
#define TRANSPORT_GAMEOFLIFEINPUT_H_DEFINED

#include "Component/ModelParam.H"
#include "Image/Image.H"
#include "Transport/FrameIstream.H"
#include "rutz/rand.h"

#include <deque>

class GameOfLifeInput : public FrameIstream
{
public:
  /// Constructor
  GameOfLifeInput(OptionManager& mgr);

  /// Virtual destructor
  virtual ~GameOfLifeInput();

  //! Override from FrameIstream; just calls setDims()
  virtual void setConfigInfo(const std::string& dimsstring);

  //! Peek at the specifications of the next frame to be read
  virtual GenericFrameSpec peekFrameSpec();

  //! Generate a random color image
  virtual GenericFrame readFrame();

  //! Specify the dimensions of the random images
  void setDims(const Dims& s);

  //! Set the seed for our random-number generator
  void setSeed(int s);

private:
  OModelParam<double>      itsFillFraction;
  OModelParam<uint>        itsCellSize;

  Dims                     itsDims;
  rutz::urand              itsGenerator;
  Image<byte>              itsBoard;
  bool                     itsStarted;
  std::deque<Image<byte> > itsBoardHist;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_GAMEOFLIFEINPUT_H_DEFINED
