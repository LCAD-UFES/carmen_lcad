/**
   \file  Robots/LoBot/io/LoFireWireBus.H
   \brief Quick wrapper around libdc1394's handles, camera nodes, etc.
*/

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
// Primary maintainer for this file: Manu Viswanathan <mviswana at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/io/LoFireWireBus.H $
// $Id: LoFireWireBus.H 11256 2009-05-30 01:46:10Z mviswana $
//

#ifndef LOBOT_FIREWIRE_BUS_DOT_H
#define LOBOT_FIREWIRE_BUS_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/misc/singleton.hh"

// 1394 headers
#ifdef HAVE_IEEE1394

#include <libdc1394/dc1394_control.h>

#else // fake 1394 API just to let this class compile

typedef int raw1394handle_t ;
typedef int nodeid_t ;

inline void dc1394_free_camera_nodes(nodeid_t*){}

#endif

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::FireWireBus
   \brief Encapsulation of libdc1394 initialization, handles, etc.

   This class provides a wrapper around libdc1394's handles, camera
   nodes, bus initialization functions, and so on. It is implemented as a
   singleton so that any other part of Lobot/Robolocust can access it
   conveniently.

   Ideally, this wouldn't be a singleton because there could be multiple
   FireWire buses on the host system. But for the purposes of the
   Robolocust project, this amount of abstraction is more than
   sufficient.
*/
class FireWireBus : public singleton<FireWireBus> {
   // Quick wrapper around raw1394 handles
   class raw1394_handle {
      raw1394handle_t handle ;
   public :
      raw1394_handle(int card_number = 0) ;
      ~raw1394_handle() ;

      // conversion operator to allow instances of this class to be used
      // directly in APIs requiring a raw1394handle_t
      operator const raw1394handle_t&() const {return handle ;}
   } ;

   // Quick wrapper around the camera node list returned by
   // dc1394_get_camera_nodes()
   class dc_node_list {
      nodeid_t* cameras ;
      int       num_cameras ;
   public :
      dc_node_list(const raw1394_handle&) ;
      ~dc_node_list() ;
      void release() {dc1394_free_camera_nodes(cameras) ; cameras = 0 ;}

      int size() const {return num_cameras ;}
      const nodeid_t& operator[](int i) const ;
   } ;

   // Bus handles, camera nodes, etc.
   raw1394_handle m_handle ;
   dc_node_list   m_cameras ;

   // Initialization
   FireWireBus() ; // private because this is a singleton
   friend class singleton<FireWireBus> ;

public:
   /// Return the raw 1394 bus handle
   const raw1394handle_t& handle() const {return m_handle ;}

   /// Return the number of cameras currently connected to the 1394 bus
   int num_cameras() const {return m_cameras.size() ;}

   /// Return the camera node corresponding the the i-th camera on the bus
   const nodeid_t& get_camera_node(int i) const {return m_cameras[i] ;}

   /// Return the i-th camera node using the subscript operator rather
   /// than an explicit function call.
   const nodeid_t& operator[](int i) const {return get_camera_node(i) ;}

   /// Camera nodes can be released after the camera devices have been
   /// initialized. This allows clients to free up memory associated with
   /// these nodes if they feel they won't set camera parameters (or call
   /// other functions that require the camera nodes) during the
   /// remainder of their lives.
   ///
   /// NOTE: This method should not be called without due consideration
   /// has been given to the matter. That is: call it only if you're
   /// really sure you know what you're doing.
   void release_camera_nodes() {m_cameras.release() ;}

   /// Destroy 1394 bus handle, camera nodes, etc.
   ~FireWireBus() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif // #ifndef LOBOT_FIREWIRE_BUS_DOT_H

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
