/*!@file Psycho/EyeLinkAscParser.H EyeLink eye-tracker parses the
  *.asc eyetracking file. Usually this comes from the *.edf file that the
  eyetracker produces, then processed by EDF2ASC executable.            */
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
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/EyeLinkAsc.H $
// $Id: $
//


// ######################################################################

#include "Image/Point2D.H"

#ifndef PSYCHO_EYELINKASCPARSER_H_DEFINED
#define PSYCHO_EYELINKASCPARSER_H_DEFINED

class EyeLinkAscParser
{
public:
  //! Constructor
  EyeLinkAscParser(std::string fileName);

  //! Destructor
  virtual ~EyeLinkAscParser();

public:

  std::string itsSubjectName;

  std::vector<int> itsTrialStart;
  std::vector<int> itsTrialEnd;

  std::vector<std::vector<int> > itsFlipTime;
  std::vector<std::vector<std::vector<Point2D<float> > > > itsFlipFixations;
  std::vector<std::vector<int> > itsFlipFrame;

  std::vector<int> itsDriftCorrectionStart;
  std::vector<int> itsDriftCorrectionDone;

  std::vector<std::vector<int> > itsSaccadeStart;
  std::vector<std::vector<int> > itsSaccadeEnd;
  std::vector<std::vector<int> > itsSaccadeDuration;
  std::vector<std::vector<Point2D<float> > > itsSaccadeStartLocation;
  std::vector<std::vector<Point2D<float> > > itsSaccadeEndLocation;
  std::vector<std::vector<float> >           itsSaccadeAmplitude; 
  std::vector<std::vector<float> >           itsSaccadePeakVel;
 
  std::vector<std::vector<int> > itsFixationStart;
  std::vector<std::vector<int> > itsFixationEnd;
  std::vector<std::vector<int> > itsFixationDuration;
  std::vector<std::vector<Point2D<float> > > itsFixationAvgLocation;
  std::vector<std::vector<float> >           itsFixationAvgPupilSize;
 
  std::vector<std::vector<int> > itsGazeTime;
  std::vector<std::vector<Point2D<float> > > itsLeftGaze;
  std::vector<std::vector<float> > itsLeftGazePupilSize;

  std::vector<std::vector<Point2D<float> > > itsRightGaze;
  std::vector<std::vector<float> > itsRightGazePupilSize;

private:

  //! Parse the input file
  void parse(std::string fileName);

  std::vector<std::string> tokenize(std::string line);
  std::string trim(std::string str);

};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
