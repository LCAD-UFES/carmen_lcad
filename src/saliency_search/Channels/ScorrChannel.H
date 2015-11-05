/*!@file Channels/ScorrChannel.H */

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
// Primary maintainer for this file:
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/ScorrChannel.H $
// $Id: ScorrChannel.H 7434 2006-11-11 02:15:19Z rjpeters $
//

#ifndef SCORRCHANNEL_H_DEFINED
#define SCORRCHANNEL_H_DEFINED

#include "Channels/SingleChannel.H"

// ######################################################################
//! A spatial correlation channel
/*! Spatial correlation is computed at every location between the
  local patch and other patches at a given radius from the local
  patch. Radius is given in saliency map pixels. Note that in this
  channel, high 'salience' means low spatial correlation, i.e., lots
  of change. CAUTION: this channel will throw a fatal exception if
  your LevelSpec contains more than one level or if that level is
  different from the maplevel or if the deltas are non-zero. */
class ScorrChannel : public SingleChannel
{
public:
  //! Construct with standard params.
  ScorrChannel(OptionManager& mgr);

  //! Destructor
  virtual ~ScorrChannel();

  //! overload
  virtual bool outputAvailable() const;

protected:
  OModelParam<int> itsRadius; //!< Radius (in saliency map pixels)

  //! special input function
  virtual void doInput(const InputFrame& inframe);

  //! special output function
  virtual Image<float> getOutput();

  Image<float> itsMap;        //!< our temporal correlation map
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // SCORRCHANNEL_H_DEFINED
