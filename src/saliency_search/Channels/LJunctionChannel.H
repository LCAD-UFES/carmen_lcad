/*!@file Channels/LJunctionChannel.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/LJunctionChannel.H $
// $Id: LJunctionChannel.H 8160 2007-03-21 21:34:16Z rjpeters $
//

#ifndef LJUNCTIONCHANNEL_H_DEFINED
#define LJUNCTIONCHANNEL_H_DEFINED

#include "Channels/ComplexChannel.H"
#include "rutz/shared_ptr.h"

class OrientationChannel;

//######################################################################
//! An L junction channel that detects L junctions at different
//orientations
class LJunctionChannel :  public ComplexChannel
{
public:
  //! Construct with standard params.
  LJunctionChannel(OptionManager& mgr,
                   nub::soft_ref<OrientationChannel> oc);

  //! Destructor
  virtual ~LJunctionChannel();

  //! Overload so that we can reconfigure when our params get changed
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

protected:
  //! (re-)build our subchannels
  virtual void buildSubChans();

  //! Computes the L junctions at different orientations
  virtual void doInput(const InputFrame& inframe);

  OModelParam<uint> itsNumOrients;  //!< number of junction subchannels

  nub::soft_ref<OrientationChannel> itsOriChan;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // LJUNCTIONCHANNEL_H_DEFINED
