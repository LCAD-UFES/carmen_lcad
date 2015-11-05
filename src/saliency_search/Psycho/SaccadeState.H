/*!@file Psycho/SaccadeState.H Saccade/fixation/pursuit states */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/SaccadeState.H $
// $Id: SaccadeState.H 9447 2008-03-13 01:18:27Z dberg $
//

#ifndef PSYCHO_SACCADESTATE_H_DEFINED
#define PSYCHO_SACCADESTATE_H_DEFINED


//! Enum for SaccadeController state
enum SaccadeState {
  SACSTATE_UNK = 0,  //!< Unknown state
  SACSTATE_FIX = 1,  //!< Fixation
  SACSTATE_SAC = 2,  //!< Saccade
  SACSTATE_SMO = 3,   //!< Smooth pursuit
  SACSTATE_COM = 4
};

//! Display state names in clear
inline const char *saccadeStateName(const SaccadeState p)
{
  static const char* n[5] = { "Unknown", "Fixation", "Saccade",
                              "SmoothPursuit", "Combined" };
  return n[int(p)];
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_SACCADESTATE_H_DEFINED
