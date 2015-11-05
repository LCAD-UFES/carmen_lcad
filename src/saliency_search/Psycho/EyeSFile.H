/*!@file Psycho/EyeSFile.H Read data from a .eyeS eye position file */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/EyeSFile.H $
// $Id: EyeSFile.H 14732 2011-04-14 18:47:43Z lior $
//

#ifndef PSYCHO_EYESFILE_H_DEFINED
#define PSYCHO_EYESFILE_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Image/Dims.H"
#include "Image/Point2D.H"
#include "Util/SimTime.H"

#include <iosfwd>

/// Read data from a .eyeS eye position file
class EyeSFile : public ModelComponent
{
public:
  EyeSFile(OptionManager& mgr);

  virtual ~EyeSFile();

  virtual void start2();

  virtual void stop1();

  Point2D<int> readUpTo(const SimTime& stime);

  //! Get an eye position from the list
  Point2D<float> getPos();

  //! Get the raw input dims
  Dims getRawInputDims() { return itsRawInputDims.getVal(); }

  int lineNumber() const;

private:
  OModelParam<std::string> itsEyeFname;
  OModelParam<int> itsEyeTrash;
  OModelParam<SimTime> itsEyePeriod;
  OModelParam<Dims> itsRawInputDims;
  std::ifstream* itsFile;
  int itsEyeSample;
  Point2D<int> itsPos;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_EYESFILE_H_DEFINED
