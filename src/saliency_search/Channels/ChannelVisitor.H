/*!@file Channels/ChannelVisitor.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/ChannelVisitor.H $
// $Id: ChannelVisitor.H 6679 2006-06-02 18:45:56Z rjpeters $
//

#ifndef CHANNELS_CHANNELVISITOR_H_DEFINED
#define CHANNELS_CHANNELVISITOR_H_DEFINED

class ChannelBase;
class SingleChannel;
class ComplexChannel;

/// Abstract base class for algorithms that traverse an entire tree of channel objects.
/** This is a straightforward implementation of the "Visitor" design
    pattern; see http://en.wikipedia.org/wiki/Visitor_pattern for
    details */
class ChannelVisitor
{
public:
  /// Virtual destructor for safe base class destruction
  virtual ~ChannelVisitor();

  /// Visit the given ChannelBase node.
  virtual void visitChannelBase(ChannelBase& chan) = 0;

  /// Visit the given SingleChannel node.
  virtual void visitSingleChannel(SingleChannel& chan) = 0;

  /// Visit the given ComplexChannel node.
  /** NOTE: ComplexChannel does NOT provide the iteration over its
      subchannels; instead, each ChannelVisitor subclass must iterate
      over ComplexChannel's subchannels, calling
      subChan(i)->accept(*this) for each one. Although this requires
      slightly more code for each ChannelVisitor subclass, it is also
      much more flexible since it lets the visitor be aware of the
      hierarchical structres of the various channels that it
      visits. */
  virtual void visitComplexChannel(ComplexChannel& chan) = 0;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_CHANNELVISITOR_H_DEFINED
