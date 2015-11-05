/**
   \file Robots/LoBot/io/LoFireWireBus.C

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/io/LoFireWireBus.C $
// $Id: LoFireWireBus.C 11256 2009-05-30 01:46:10Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/io/LoFireWireBus.H"
#include "Robots/LoBot/misc/LoExcept.H"

//---------------------- ALTERNATIVE DEFINITION -------------------------

// In case libdc1394 and other IEEE-1394 libraries are missing
#ifndef HAVE_IEEE1394

namespace lobot {

// Constructors
FireWireBus::raw1394_handle::raw1394_handle(int)
   : handle(0)
{}

FireWireBus::dc_node_list::dc_node_list(const FireWireBus::raw1394_handle&)
   : cameras(0), num_cameras(0)
{}

FireWireBus::FireWireBus()
   : m_handle(0), m_cameras(m_handle)
{
   throw missing_libs(MISSING_LIBDC1394) ;
}

// Destructors
FireWireBus::raw1394_handle::~raw1394_handle(){}
FireWireBus::dc_node_list::~dc_node_list(){}
FireWireBus::~FireWireBus(){}

} // end of namespace encapsulating above empty definition

#else

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- INITIALIZATION -----------------------------

// Initializing raw 1394 handles
FireWireBus::raw1394_handle::raw1394_handle(int card_number)
   : handle(dc1394_create_handle(card_number))
{
   if (handle == NULL)
      throw bus_error(INIT_PROBLEM) ;
}

// Retrieving the IDs of the cameras currently connected to the FireWire
// bus.
FireWireBus::dc_node_list::dc_node_list(const FireWireBus::raw1394_handle& H)
   : cameras(0), num_cameras(0)
{
   cameras = dc1394_get_camera_nodes(H, & num_cameras, 0) ;
   if (num_cameras < 1)
      throw bus_error(NO_CAMERAS) ;

   int highest_node_id = raw1394_get_nodecount(H) - 1 ;
   for (int i = 0; i < num_cameras; ++i)
      if (cameras[i] == highest_node_id) { // prevent ISO transfer bug
         release() ;
         throw bus_error(HIGHEST_NODE) ;
      }
}

// Bus initialization simply involves obtaining a raw 1394 handle and
// retrieving the camera IDs.
FireWireBus::FireWireBus()
   : m_handle(0), m_cameras(m_handle)
{}

//------------------------ CAMERA NODE ACCESS ---------------------------

const nodeid_t& FireWireBus::dc_node_list::operator[](int i) const
{
   if (! cameras)
      throw bus_error(CAMERA_NODES_FREED) ;
   if (i < 0 || i >= num_cameras)
      throw bus_error(BAD_CAMERA_NODE_INDEX) ;
   return cameras[i] ;
}

//----------------------------- CLEAN-UP --------------------------------

FireWireBus::raw1394_handle::~raw1394_handle()
{
   dc1394_destroy_handle(handle) ;
}

FireWireBus::dc_node_list::~dc_node_list()
{
   release() ;
}

FireWireBus::~FireWireBus(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif // #ifndef HAVE_IEEE1394

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
