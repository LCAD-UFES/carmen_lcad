/*!@file Channels/IntensityBandChannel.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/IntensityBandChannel.H $
// $Id: IntensityBandChannel.H 8160 2007-03-21 21:34:16Z rjpeters $
//

#ifndef INTENSITYBANDCHANNEL_H_DEFINED
#define InINTENSITYBANDCHANNEL_H_DEFINED

#include "Channels/ComplexChannel.H"

class IntensityChannel;

// ######################################################################
//! A composite channel containing a set of intensity-band channels
/*! This is an example of a homogeneous complex channel. Thus, the
  JetSpec representation will have an index for the subchannel number,
  and further indices for the various Jet components (which are
  identical for all subchannels). */
class IntensityBandChannel : public ComplexChannel
{
public:
  //! Construct with standard params
  IntensityBandChannel(OptionManager& mgr);

  //! Destructor
  virtual ~IntensityBandChannel();

  //! Wraps ComplexChannel's subChan() and casts the result to IntensityChannel
  virtual IntensityChannel& band(const uint idx) const;

  //! Overload so that we can reconfigure when our params get changed
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

protected:
  OModelParam<uint> itsNumBands; //!< number of intensity bands
  OModelParam<float> itsSigma;   //!< width of each intensity band

  //! (re-)build our subchannels
  virtual void buildSubChans();

  //! IntensityBandChannel requires only the intensity image input
  virtual void doInput(const InputFrame& inframe);
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // INTENSITYBANDCHANNEL_H_DEFINED
