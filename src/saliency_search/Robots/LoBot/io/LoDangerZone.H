/**
   \file  Robots/LoBot/control/LoDangerZone.H
   \brief An object for monitoring lobot's danger zone.

   This file defines a class that reads the danger zone settings from the
   Robolocust config file and implements an update method that the main
   thread can use to keep the current state of the danger zone in sync
   with the laser range finder measurements. Other threads, viz., the
   Robolocust behaviours, can read the current danger zone state and take
   appropriate action using the different state access APIs implemented
   by the danger zone object.
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
// Primary maintainer for this file: mviswana usc edu
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/io/LoDangerZone.H $
// $Id: LoDangerZone.H 13037 2010-03-23 01:00:53Z mviswana $
//

#ifndef LOBOT_DANGER_ZONE_DOT_H
#define LOBOT_DANGER_ZONE_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/io/LoLRFData.H"
#include "Robots/LoBot/io/LoLaserRangeFinder.H"

#include "Robots/LoBot/misc/singleton.hh"
#include "Robots/LoBot/util/range.hh"

// Standard C++ headers
#include <string>
#include <vector>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::DangerZone
   \brief An object for monitoring the robot's danger zone.

   This class provides an API for keeping track of the robot's danger
   zone. The Robolocust danger zone works by dividing the laser range
   finder's FOV into several angular blocks. Each block has three
   fundamental settings:

      - extents
      - distance
      - threshold

   The extents define the block's angular range. A blocks's distance
   setting specifies the minimum distance between the robot and
   obstacles. A block's threshold setting specifies the minimum number of
   LRF distance measurements that must be less than its distance setting
   for the block to be actually considered as being "penetrated" or
   "active."

   For example, let us say we have a danger zone block with extents [-30,
   30], distance 350 and threshold 20. This means that when we have >= 20
   LRF readings that are <= 350mm in the angular range -30 degrees to +30
   degrees, an obstacle has penetrated this particular portion of the
   robot's danger zone and, thus, activated this block.

   The danger zone settings are specified in the Robolocust config file.

   The main thread is supposed to update the danger zone object after
   calling the laser range finder object's update() method.

   Other threads, viz., the behaviours that need to monitor the danger
   zone, may use the lobot::DangerZone state access APIs to ascertain the
   current conditions of the danger zone and take appropriate action.

   As it does for updating other objects, the main thread must use
   lobot::UpdateLock's write lock when updating the danger zone. Other
   threads must use lobot::UpdateLock's read lock when accessing the
   danger zone's current state.
*/
class DangerZone : public singleton<DangerZone> {
   // Prevent copy and assignment
   DangerZone(const DangerZone&) ;
   DangerZone& operator=(const DangerZone&) ;

   // Boilerplate code to make the generic singleton design pattern work
   friend class singleton<DangerZone> ;

   /// The danger zone works by dividing the laser range finder's FOV
   /// into several user-specified angular blocks. Each such block is
   /// associated with a user-specified distance and a threshold. When
   /// the LRF reports a distance measurement in a block less than that
   /// block's danger zone, that reading is added to a list of danger
   /// zone readings for that block. When the number of danger zone
   /// readings for block exceeds its threshold, that block is considered
   /// "active" or "penetrated."
   ///
   /// This inner class holds together angular blocks and their
   /// correspondinng danger zone specifications, thresholds, danger zone
   /// readings, etc.
public:
   class Block {
      /// The angular extents of the block. These extents are read from
      /// the config file.
      range<int> m_extents ;

      /// The user-specified minimum acceptable distance measurement for
      /// this block. When the LRF reports a distance less than the value
      /// of the danger zone, that reading will get recorded as a danger
      /// zone reading.
      int m_danger_zone ;

      /// The user-specified minimum number of danger zone readings for
      /// this block. Danger zone readings less than this number will not
      /// activate the block.
      int m_threshold ;

      /// The danger zone readings for this block. This data structure is
      /// cleared and updated as part of the main thread's update cycle
      /// (i.e., it is not a static setting read from the config file).
      //@{
   public:
      typedef LRFData::Reading Reading ;
      typedef std::vector<Reading> Readings ;
   private:
      Readings m_danger_zone_readings ;
      //@}

      /// Private constructors because only the DangerZone object can
      /// create new danger zone blocks.
      //@{
      Block(const range<int>& extents, int danger_zone, int threshold) ;
      Block(int start_angle, int end_angle, int danger_zone, int threshold) ;
      //@}

      /// Add a danger zone reading to the block.
      void add(const Reading& R) {m_danger_zone_readings.push_back(R) ;}

      /// Clear the block's danger zone readings.
      void clear() {m_danger_zone_readings.clear() ;}

      /// Helper function object to update a block.
      //@{
      class update {
         const LRFData& lrf ;
      public:
         update(const LRFData&) ;
         void operator()(const Block&) const ;
      } ;
      friend class update ;
      //@}

      // The outer class, viz., lobot::DangerZone, has special privileges
      // with danger zone blocks.
      friend class DangerZone ;

   public:
      /// Retrieving the angular block's extents, danger zone and
      /// threshold settings.
      //@{
      int start() const {return m_extents.min()  ;}
      int end()   const {return m_extents.max()  ;}
      int size()  const {return m_extents.size() ;}
      const range<int>& extents() const {return m_extents ;}

      int danger_zone() const {return m_danger_zone ;}
      int threshold()   const {return m_threshold   ;}
      //@}

      /// How many danger zone readings does this block currently have?
      int danger_level() const {return m_danger_zone_readings.size() ;}

      /// Has this block of the danger zone been penetrated by an
      /// obstacle?
      bool penetrated() const {return danger_level() >= threshold() ;}

      /// Iterators for going through the current danger zone readings.
      //@{
      Readings::const_iterator danger_begin() const {
         return m_danger_zone_readings.begin() ;
      }
      Readings::const_iterator danger_end() const {
         return m_danger_zone_readings.end() ;
      }
      //@}

      /// Debug support
      void dump(const std::string& caller) const ;
   } ;

   /// The Robolocust danger zone consists of several blocks.
   //@{
   typedef std::vector<Block> Blocks ;
private:
   Blocks m_blocks ;
   //@}

   /// How many blocks does the danger zone have?
public:
   static int num_blocks() {return instance().m_blocks.size() ;}

   /// Once the danger zone settings have been loaded from the config
   /// file, we find the max danger zone and store that for later use
   /// (e.g., some behaviours need this value for rendering purposes).
   /// Since the danger zone settings are constants, finding and storing
   /// this value avoids having to repeatedly find it each time it is
   /// needed.
private:
   float m_max ;
public:
   static float max() {return instance().m_max ;}

   /// Obviously, for this whole danger zone thing to work, we also need
   /// an LRF object from where we can get distance measurements. The
   /// main thread should specify this.
private:
   static const LaserRangeFinder* m_lrf ;
public:
   static void use(const LaserRangeFinder* lrf) {m_lrf = lrf ;}

   /// Everytime we update the danger zone, we record the LRF
   /// measurements that resulted in the current danger zone state.
private:
   LRFData* m_lrf_data ;

   /// A private constructor because this class is a singleton.
   DangerZone() ;

public:
   /// This method updates the danger zone using the current laser range
   /// finder measurements. It is meant to be called only by the main
   /// thread, which should use lobot::UpdateLock's write lock when
   /// calling this function.
   static void update() ;

   /// A convenience function to check if any block in the danger zone
   /// has been penetrated or not.
   ///
   /// NOTE: Client should use lobot::UpdateLock's read lock when calling
   /// this function.
   static bool penetrated() ;

   /// Iterators for walking through the list of danger zone blocks.
   ///
   /// NOTE: Client should use lobot::UpdateLock's read lock when calling
   /// thess functions.
   //@{
   static Blocks::const_iterator begin() {return instance().m_blocks.begin() ;}
   static Blocks::const_iterator end()   {return instance().m_blocks.end()   ;}
   //@}

   /// Retrieving the LRF measurements snapshot corresponding to the
   /// current danger zone state.
   ///
   /// NOTE: Client should use lobot::UpdateLock's read lock when calling
   /// this function.
   static const LRFData& lrf_data() {return *instance().m_lrf_data ;}

   /// Clean-up.
   ~DangerZone() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
