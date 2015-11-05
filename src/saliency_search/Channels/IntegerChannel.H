/*!@file Channels/IntegerChannel.H Base class for channels that will use integer math */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/IntegerChannel.H $
// $Id: IntegerChannel.H 8233 2007-04-03 22:26:45Z rjpeters $
//

#ifndef CHANNELS_INTEGERCHANNEL_H_DEFINED
#define CHANNELS_INTEGERCHANNEL_H_DEFINED

#include "Channels/ChannelBase.H"
#include "Channels/IntegerInput.H"
#include "Channels/IntegerMathEngine.H"

struct integer_math;
template <class T> class PyramidCache;

/// Base class for channels that will use integer math
class IntegerChannel : public ChannelBase
{
public:
  //! Default constructor. See ModelComponent.H
  /*! @param mgr our ModelManager (see ModelManager.H)
      @param descrName descriptive name for human usage
      @param tagName name for ParamMap usage
      @param vs The VisualFeature implemented by the channel */
  IntegerChannel(OptionManager& mgr, const std::string& descrName,
                 const std::string& tagName, const VisualFeature vs,
                 nub::ref<IntegerMathEngine> eng);

  virtual Image<int> getRawCSmapInt(const uint idx) const = 0;

  virtual Image<int> getSubmapInt(const uint index) const = 0;

  virtual Image<int> getOutputInt() = 0;

  void inputInt(const IntegerInput& inp,
                const SimTime& t,
                PyramidCache<int>* cache,
                const Image<byte>& clipMask = Image<byte>());

  const integer_math* getImath() const;

  nub::ref<IntegerMathEngine> getMathEngine() const;

private:
  /// Dispatch to our integer version
  virtual void doInput(const InputFrame& inframe);

  virtual void doInputInt(const IntegerInput& inp,
                          const SimTime& t,
                          PyramidCache<int>* cache,
                          const Image<byte>& clipMask) = 0;

  nub::ref<IntegerMathEngine> itsMathEngine;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_INTEGERCHANNEL_H_DEFINED
