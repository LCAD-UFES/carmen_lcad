/*!@file Transport/HalfFieldDeinterlacer.H "Deinterlace" frames by always returning either the top (or bottom) half-field */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/HalfFieldDeinterlacer.H $
// $Id: HalfFieldDeinterlacer.H 9547 2008-03-28 23:32:43Z rjpeters $
//

#ifndef TRANSPORT_HALFFIELDDEINTERLACER_H_DEFINED
#define TRANSPORT_HALFFIELDDEINTERLACER_H_DEFINED

#include "Transport/Deinterlacer.H"

///  "Deinterlace" frames by always returning either the top (or bottom) half-field
template <bool BottomField>
class HalfFieldDeinterlacer : public Deinterlacer
{
public:
  /// Constructor
  HalfFieldDeinterlacer(OptionManager& mgr);

  /// Destructor
  virtual ~HalfFieldDeinterlacer();

  /// Return the specifications of the next frame
  virtual GenericFrameSpec peekFrameSpec();

  /// Advise the deinterlacer of the current frame number; pass along to our delegate
  virtual bool setFrameNumber(int n);

  //! Get a streaming grab started
  virtual void startStream();

  /// Get the next half-field deinterlaced frame
  virtual GenericFrame readFrame();
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_HALFFIELDDEINTERLACER_H_DEFINED
