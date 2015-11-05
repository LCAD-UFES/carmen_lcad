/*!@file Channels/JetFiller.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/JetFiller.H $
// $Id: JetFiller.H 9412 2008-03-10 23:10:15Z farhan $
//

#ifndef CHANNELS_JETFILLER_H_DEFINED
#define CHANNELS_JETFILLER_H_DEFINED

#include "Channels/ChannelVisitor.H"
#include "Channels/Jet.H"
#include "Image/Point2D.H"

#include <vector>

/// Fill the components of the given Jet that relate to the called channel
/** All relevant components of the Jet are set to the channel's values
    at the specified location across all pyramid levels. The channel
    will store its data starting at the given index in the jet, and
    will return the number of entries filled. The index vector will be
    modified as the various Jet components are explored, but will be
    restored to its original state upon completion. */
class JetFiller : public ChannelVisitor
{
public:
  /// Construct
  JetFiller(const Point2D<int>& loc, Jet<float>& j, bool do_interp);

  /// Virtual destructor
  virtual ~JetFiller();

  /// Visit the given ChannelBase node.
  virtual void visitChannelBase(ChannelBase& chan);

  /// Visit the given SingleChannel node.
  virtual void visitSingleChannel(SingleChannel& chan);

  /// Visit the given ComplexChannel node.
  virtual void visitComplexChannel(ComplexChannel& chan);

private:
  Point2D<int> itsLoc;
  Jet<float>& itsJet;
  bool itsDoInterp;
  std::vector<int> itsIndex;
};

/// Fill the specification of a Jet with data that relate to the called channel
class JetSpecFiller : public ChannelVisitor
{
public:
  /// Construct
  JetSpecFiller();

  /// Virtual destructor
  virtual ~JetSpecFiller();

  /// Visit the given ChannelBase node.
  virtual void visitChannelBase(ChannelBase& chan);

  /// Visit the given SingleChannel node.
  virtual void visitSingleChannel(SingleChannel& chan);

  /// Visit the given ComplexChannel node.
  virtual void visitComplexChannel(ComplexChannel& chan);

  /// Get the resulting JetSpec
  rutz::shared_ptr<JetSpec> getJetSpec() const
  { return itsJetSpec; }

private:
  rutz::shared_ptr<JetSpec> itsJetSpec;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_JETFILLER_H_DEFINED
