/*!@file Simulation/SimEventQueueFlag.H Flags for handling of SimEvent by SimEventQueue */

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
// Primary maintainer for this file: Laurent Itti <itti@pollux.usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Simulation/SimEventQueueFlag.H $
// $Id: SimEventQueueFlag.H 10794 2009-02-08 06:21:09Z itti $
//

#ifndef SIMULATION_SIMEVENTQUEUEFLAG_H_DEFINED
#define SIMULATION_SIMEVENTQUEUEFLAG_H_DEFINED

//! Flag type for SimEventQueue::check(), can be bitwise or'ed
typedef int SimEventQueueFlag;

//! No flag. Return any event (marked or not), don't mark nor delete it
const SimEventQueueFlag SEQ_ANY = 0;

//! Mark in the queue each event returned as being done for our caller
const SimEventQueueFlag SEQ_MARK = (1 << 0);

//! Only consider events that have not been marked for this caller
const SimEventQueueFlag SEQ_UNMARKED = (1 << 1);



//! Flag for SimEventQueueL::request(), can be bitwise or'ed
typedef int SimReqQueueFlag;

//! No flag
const SimReqQueueFlag SRQ_ANY = 0;

//! Trow a fatal exception if there is no handler for that request
const SimReqQueueFlag SRQ_NONE_FATAL = (1 << 0);

//! Trow a fatal exception if there is more than 1 handler for that request
const SimReqQueueFlag SRQ_SEVERAL_FATAL = (1 << 1);


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // SIMULATION_SIMEVENTQUEUEFLAG_H_DEFINED
