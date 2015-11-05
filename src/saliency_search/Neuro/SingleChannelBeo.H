/*!@file Neuro/SingleChannelBeo.H wrapper class to run a SingleChannel on a
  distant CPU */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SingleChannelBeo.H $
// $Id: SingleChannelBeo.H 8195 2007-03-30 04:34:07Z rjpeters $
//

#ifndef SINGLECHANNELBEO_H_DEFINED
#define SINGLECHANNELBEO_H_DEFINED

#include "Channels/InputHandler.H"
#include "Channels/SingleChannel.H"
#include "Util/Types.H"
class Beowulf;

// Beowulf message IDs for our messages:
#define BEO_SCHANCONF   1000
#define BEO_SCHANINPUT  1001
#define BEO_SCHANOUTPUT 1002
#define BEO_SCHANALLOUT 1003

/// Wrapper to run a SingleChannel on a distant CPU
/** This class is an InputHandler derivative, to be used as an add-on
    to SingleChannel or subclasses, by installing an object of this
    class with SingleChannel::setInputHandler(). It uses a Beowulf to
    delegate the actual feature map computations to a distant CPU. The
    distant CPU should run a SingleChannelBeoServer, like in the the
    beochannel-server program; see SingleChannelBeoServer.H and
    beochannel-server.C for details. When we get an input image, we
    will send it off to the processing node specified at construction
    (and possibly modified later using standard ModelParam access
    functions), using the Beowulf object given at construction. Before
    sending off our first input, we will contact our processing node
    and send it all of our associated SingleChannel's ModelParam
    values in a configuration message; this will allow it to build the
    actual SingleChannel which we are associated with.

    The system can work in two modes: in full emulation mode, the
    output map, channel pyramid, submaps and clip pyramid are all
    returned by the processor node. This requires a lot of network
    bandwidth but should ensure results that are exactly identical to
    when using the normal channels. In quick mode, only the output
    submap is returned. This will work with standard processing that
    only deals with the output, but things like saving the submaps or
    biasing the visual cortex will break; so use the fast mode at your
    own risk ;-) Selection of which mode is used is made by the
    SingleChannelBeoServer; here we just store whatever the server
    sends us, be it full or partial results.

    PROGRAMMER NOTE: See the examples in ChannelsBeo.{H,C} for how to
    use SingleChannelBeo. */
class SingleChannelBeo : public InputHandler
{
public:
  //! Constructor
  /*! @param beow the Beowulf to use for data transfers */
  SingleChannelBeo(const nub::ref<Beowulf>& beow);

  //! Virtual destructor ensures proper destruction of derived classes.
  virtual ~SingleChannelBeo();

  //! Send the input image off to the processing node
  /*! You should call this from inside your overload of
      SingleChannel::doInput() */
  virtual void handleInput(SingleChannel& chan,
                           const Image<float>& bwimg,
                           const SimTime& t,
                           const Image<byte>& clipMask,
                           const rutz::shared_ptr<PyramidCache<float> >& cache);

  //! Have we received the results from our processing node yet?
  /*! This call will block until the results are received back from
    the processing node. You should call this from within your
    overload of SingleChannel::outputAvailable().  This is how we
    implement the remote processing with minimum differences between
    SingleChannel and objects that inherit from both SingleChannel and
    SingleChannelBeo; indeed, each time is result is requested from
    SingleChannel, it will (and should in future methods!) check first
    that some output is available. So, by intercepting this check we
    can wait until the results have been received from the processing
    node. */
  virtual void waitForOutput(SingleChannel& chan);

private:
  virtual rutz::shared_ptr<InputHandler> makeClone() const;

  SingleChannelBeo(const SingleChannelBeo&); // not implemented
  SingleChannelBeo& operator=(const SingleChannelBeo&); // not implemented

  nub::ref<Beowulf> itsBeo;   // our Beowulf for communications
  int itsNode;                // Beowulf node that will process our data
  int32 itsFrame;             // unofficial frame number, starts at zero
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
