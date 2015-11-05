/*!@file Neuro/SingleChannelBeo.C wrapper class to run a SingleChannel on a
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SingleChannelBeo.C $
// $Id: SingleChannelBeo.C 8195 2007-03-30 04:34:07Z rjpeters $
//

#include "Neuro/SingleChannelBeo.H"

#include "Beowulf/Beowulf.H"
#include "Beowulf/TCPmessage.H"
#include "Component/ParamMap.H"
#include "Util/Assert.H"

#include <sstream>

// ######################################################################
SingleChannelBeo::SingleChannelBeo(const nub::ref<Beowulf>& beow) :
  itsBeo(beow), itsNode(-2), itsFrame(-1)
{ }

// ######################################################################
SingleChannelBeo::~SingleChannelBeo()
{
  if (itsNode >= 0)
    itsBeo->releaseNode(itsNode);
}

// ######################################################################
void SingleChannelBeo::handleInput(SingleChannel& chan,
                                   const Image<float>& bwimg,
                                   const SimTime& t,
                                   const Image<byte>& clipMask,
                                   const rutz::shared_ptr<PyramidCache<float> >& cache)
{
  // if we haven't allocated a Beowulf node yet, then let's do so now:
  if (itsNode == -2)
    {
      if (itsBeo->getNodeNumber() != -1)
        LFATAL("I need to be Beowulf master");

      // ask the Beowulf for a node number:
      itsNode = itsBeo->requestNode();
      if (itsNode == -2)
        LFATAL("No more available nodes - You need a bigger Beowulf");

      LINFO("Delegating '%s' [%s] to Beowulf node %d",
            chan.descriptiveName().c_str(), chan.tagName().c_str(),
            itsNode);
    }

  // if this is the first time we are called, let's start by sending
  // off a configuration message to our processing node, so that it
  // can create a channel for us:
  if (itsFrame == -1) {
    // we have two types of config data: our ModelParam values, and our
    // map weights. We put each into a ParamMap:
    ParamMap param; chan.writeParamsTo(param);
    ParamMap weight; chan.writeTo(weight);

    // let's convert both ParamMaps to string:
    std::stringstream sparam, sweight;
    param.format(sparam); weight.format(sweight);

    // let's put our descriptive name, our tag name and both parameter
    // strings into a TCPmessage, and let's put our VisualFeature in
    // the 'frame' field, and let's send it off (NOTE: VisualFeature
    // also is in param, but having it here will make decoding of the
    // message easier).
    int32 vs = static_cast<int32>(chan.visualFeature());
    TCPmessage msg(vs, BEO_SCHANCONF);
    msg.addString(chan.descriptiveName().c_str());
    msg.addString(chan.tagName().c_str());
    msg.addString(sparam.str().c_str());
    msg.addString(sweight.str().c_str());

    itsBeo->send(itsNode, msg);
  }

  // increment our unofficial frame number; we have this counter just
  // because we have a frame field in TCPmessage, so we may as well
  // use it:
  itsFrame ++;

  // send t, bwimg and clipMask to processor node:
  TCPmessage msg(itsFrame, BEO_SCHANINPUT);
  msg.addDouble(t.secs()); msg.addImage(bwimg); msg.addImage(clipMask);
  itsBeo->send(itsNode, msg);

  // anybody needing our results will need to wait until they have
  // come back from the processor node:
  chan.killCaches();
}

// ######################################################################
void SingleChannelBeo::waitForOutput(SingleChannel& chan)
{
  // nothing to wait for if we haven't allocated a Beowulf node yet:
  if (itsNode == -2) return;

  // nothing to wait for if we have nothing in progress
  if (chan.hasOutputCache()) return;

  // See if we have a new message on our Beowulf:
  TCPmessage rmsg; bool gotnothing = true; int err = 0;
  int32 rframe, raction, rnode = itsNode;  // receive only from our server
  while(gotnothing) {
    err = 0;
    // wait up to 50ms
    if (itsBeo->receive(rnode, rmsg, rframe, raction, 50, &err)) {
      // message is supposed to contain: pyramid, submaps, possibly clipPyr:
      if (raction != BEO_SCHANOUTPUT && raction != BEO_SCHANALLOUT)
        LERROR("IGNORING message of wrong type from node %d", rnode);
      else {
        if (rframe != itsFrame)
          LERROR("Received results for frame %d while at frame %d??",
                 rframe, itsFrame);
        // the time:
        const double t = rmsg.getElementDouble();

        // the output map (always valid, but possibly blank):
        chan.storeOutputCache(rmsg.getElementFloatIma());

        // additional results?
        if (raction == BEO_SCHANALLOUT) {
          // the pyramid:
          ImageSet<float> pyr = rmsg.getElementFloatImaSet();
          // if the SingleChannel on the remote node had no output
          // available yet, we have an empty pyramid, which we don't
          // want to push into our pyramid queue; this way, we will also
          // have no pyramid available if the remote had none:
          if (pyr.size())
            chan.storePyramid(pyr, SimTime::SECS(t));

          // the submaps:
          pyr = rmsg.getElementFloatImaSet();
          if (pyr.size())
            chan.storeSubmapCache(pyr);

          // clip pyramid (if any):
          pyr = rmsg.getElementFloatImaSet();
          if (pyr.size())
            chan.storeClipPyramid(pyr);
        }

        // ok, we got our results, let's return:
        gotnothing = false;
      }
    }

    if (err != 0)
      LFATAL("error during Beowulf::receive()");
  }
}

// ######################################################################
rutz::shared_ptr<InputHandler> SingleChannelBeo::makeClone() const
{
  return rutz::make_shared(new SingleChannelBeo(itsBeo));
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
