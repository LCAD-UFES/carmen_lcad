/*!@file Channels/OrientationChannel.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/OrientationChannel.H $
// $Id: OrientationChannel.H 10794 2009-02-08 06:21:09Z itti $
//

#ifndef ORIENTATIONCHANNEL_H_DEFINED
#define ORIENTATIONCHANNEL_H_DEFINED

#include "Channels/ComplexChannel.H"
#include "rutz/shared_ptr.h"

class GaborChannel;

//######################################################################
//! A composite channel containing a set of orientation-tuned gabor channels.
/*! This is an example of a homogeneous complex channel. Thus, the
  JetSpec representation will have an index for the subchannel number,
  and further indices for the various Jet components (which are
  identical for all subchannels). */
class OrientationChannel : public ComplexChannel
{
public:
  //! Construct with standard params
  OrientationChannel(OptionManager& mgr);

  //! Construct with standard params and tag override
  OrientationChannel(OptionManager& mgr, const char* tag, const char* desc, const char* gabortag);

  //! Destructor
  virtual ~OrientationChannel();

  //! Wraps ComplexChannel's subChan() and casts the result to GaborChannel.
  virtual GaborChannel& gabor(const uint idx) const;

  //! Overload so that we can reconfigure when our params get changed
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

  //! the possible interaction types between orientations
  enum InteractType { NONE = 0, SUB_MEAN = 1, CUSTOM = 2 };

  //! returns the currently set interaction type between the orientation
  InteractType getInteractType();

  //! sets the interaction type between the orientations
  void setInteractType(InteractType type);

  //! returns the coefficients vector for interactions between orientations
  std::vector<float> getInteractCoeffs();

  //! sets the coefficients vector for the interactions between orientations
  void setInteractCoeffs(std::vector<float>& coeffs);

  //! compute interactions, then call ComplexChannel::getOutput()
  virtual Image<float> getOutput();

protected:
  //! trigger the interaction computation between the gabor filters
  void doInteractions();

  OModelParam<uint> itsNumOrients;  //!< number of Gabor subchannels

  //! (re-)build our subchannels
  virtual void buildSubChans();

  //! parse the string for the interaction type between orientations
  virtual void parseInteractString(const std::string& value);

  //! OrientationChannel requires only luminance input.
  virtual void doInput(const InputFrame& inframe);

  //! type of interaction mode between orientations
  OModelParam<std::string> itsInteractString;

private:
  InteractType itsInteractType;
  std::vector<float> itsInteractCoeffs;
  bool itsDidInteractions;
  bool itsOverideTagName;
  std::string itsGaborOverideTag;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // ORIENTATIONCHANNEL_H_DEFINED
