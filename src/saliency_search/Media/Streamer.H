/*!@file Media/Streamer.H Base class for simple input-output streaming applications */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/Streamer.H $
// $Id: Streamer.H 9841 2008-06-20 22:05:12Z lior $
//

#ifndef MEDIA_STREAMER_H_DEFINED
#define MEDIA_STREAMER_H_DEFINED

#include "Raster/GenericFrame.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameOstream.H"
#include "nub/ref.h"

class InputFrameSeries;
class ModelComponent;
class ModelManager;
class OptionManager;
class OutputFrameSeries;

/// Base class for simple input-output streaming applications
class Streamer
{
public:
  Streamer(const char* name);

  virtual ~Streamer();

  int run(const int argc, const char** argv,
          const char* extraArgsDescription = "",
          const int minExtraArgs = 0, const int maxExtraArgs = 0);

protected:
  void addComponent(const nub::ref<ModelComponent>& c);

  OptionManager& getManager();

private:
  Streamer(const Streamer&); // not implemented
  Streamer& operator=(const Streamer&); // not implemented

  int tryRun(const int argc, const char** argv,
             const char* extraArgsDescription,
             const int minExtraArgs, const int maxExtraArgs);

  /// Give base classes an opportunity to pull extra args out of ModelManager before start()
  virtual void handleExtraArgs(const ModelManager& mgr);

  virtual void onFrame(const GenericFrame& input,
                       FrameOstream& ofs,
                       const int frameNum) = 0;

  nub::ref<ModelManager> itsManager;
  nub::ref<InputFrameSeries> itsIfs;
  nub::ref<OutputFrameSeries> itsOfs;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // MEDIA_STREAMER_H_DEFINED
