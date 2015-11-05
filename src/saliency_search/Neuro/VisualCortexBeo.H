/*!@file Neuro/VisualCortexBeo.H a VisualCortex with SingleChannelBeo channels */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/VisualCortexBeo.H $
// $Id: VisualCortexBeo.H 10845 2009-02-13 08:49:12Z itti $
//

#ifndef VISUALCORTEXBEO_H_DEFINED
#define VISUALCORTEXBEO_H_DEFINED

class Beowulf;
class RawVisualCortex;
namespace nub { template <class T> class ref; }

/// Setup a VisualCortex for use with SingleChannelBeo channels
/** This setup does two things. First, it ensures that all
    SingleChannel objects have their InputHandler set to
    SingleChannelBeo. Second, it directs the VisualCortex to sort its
    subchannels according to their number of submaps, so that the most
    CPU-intensive channels (e.g. motion, orientation) will be
    processed first. See SingleChannelBeo.H for info.

    The Beowulf object passed here must be configured as master.

    You may still add whatever channels you want to this VisualCortex;
    just if you want them to be Beo too, make sure you first get our
    Beowulf using getBeowulf(), the next available node using
    getNextAvailableNode() and then initilize your BeoChannels using
    this Beowulf and node number.

    If you are going to use a beowulf-enabled VisualCortex, you should
    be running a process that implements a SingleChannelBeoServer on
    each slave node of your Beowulf. Such process is in
    beochannel-server.C for example. First start all your servers on
    all your server nodes, then, on the master, start your program
    that uses the VisualCortexBeo. It will configure the Beowulf,
    spawn the channels, and rip through your frames!
*/
void setupVisualCortexBeo(RawVisualCortex& vcx, nub::ref<Beowulf> beo);

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
