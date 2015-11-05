/**
   \file  Robots/LoBot/control/LoMetrics.H
   \brief A behaviour for collecting performance metrics.

   This file defines a class that implements a behaviour for gathering
   all sorts of useful data as lobot runs. For example, we can get
   information such as the robot's pose history, the time at which a goal
   is reached, average speed of the robot, number of emergency stop and
   extrication events, actual and predicted times-to-impact, and so on.
   All this data is periodically written to a log file, which can then be
   analyzed off-line.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoMetrics.H $
// $Id: LoMetrics.H 13620 2010-06-25 05:12:03Z mviswana $
//

#ifndef LOBOT_METRICS_BEHAVIOUR_DOT_H
#define LOBOT_METRICS_BEHAVIOUR_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoBehavior.H"
#include "Robots/LoBot/thread/LoMutex.H"
#include "Robots/LoBot/misc/factory.hh"

// Standard C++ headers
#include <sstream>
#include <string>
#include <vector>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::Metrics
   \brief A behaviour for gathering performance metrics about the robot.

   This class implements a behaviour that collects all sorts of useful
   data as lobot runs. For example, we can get information such as the
   robot's pose history, the time at which a goal is reached, average
   speed of the robot, number of emergency stop and extrication events,
   actual and predicted times-to-impact, and so on. All this data is
   periodically written to a log file, which can then be analyzed
   off-line.

   This behaviour does not control the robot in any way nor does it
   perform any sort of visualization.
*/
class Metrics : public Behavior {
   // Prevent copy and assignment
   Metrics(const Metrics&) ;
   Metrics& operator=(const Metrics&) ;

   // Handy type to have around in a derived class
   typedef Behavior base ;

   // Boilerplate code to make the generic factory design pattern work
   friend  class subfactory<Metrics, base> ;
   typedef register_factory<Metrics, base> my_factory ;
   static  my_factory register_me ;

   /// Different behaviours report metrics using different data types.
   /// However, all of them get converted to strings and buffered by this
   /// class. When the buffer grows to some predefined limit, it will be
   /// written out to the log file.
   ///
   /// This data structure acts as the above-mentioned buffer of log
   /// messages.
   std::vector<std::string> m_buffer ;

   /// Since multiple behaviours can write to the above buffer, we need
   /// to synchronize accesses to it.
   Mutex m_mutex ;

   /// Although this class is not a singleton, it behaves like one. Its
   /// public interface consists of a helper class for building
   /// individual log messages, which then get buffered by the metrics
   /// behaviour. The above-mentioned helper class uses the following
   /// instance pointer to get to a Metrics object.
   ///
   /// DEVNOTE: We implement this behaviour in this way so that clients
   /// don't have to rely on the lobot::App object to get a suitable
   /// instance of this class and so that lobot::App doesn't have to
   /// implement a special API to search for this behaviour in its list
   /// of behaviours. Instead, client behaviours can simply use the
   /// logging API and let this class figure out how to get to a usable
   /// instance.
   ///
   /// DEVNOTE 2: Since the singleton concept is not enforced, users can,
   /// in fact, start multiple metrics behaviours. However, only one of
   /// them will be used, viz., the last one to be created; the others
   /// will simply waste CPU time and memory.
   static Metrics* m_instance ;

   /// A private constructor because behaviours are instantiated with an
   /// object factory and not directly by clients.
   Metrics() ;

   /// Some things to do before commencing regular action processing.
   void pre_run() ;

   /// This method checks if the message buffer has exceeded some
   /// preconfigured threshold and, if so, dumps it to the log.
   void action() ;

   /// Clean-up: dump any pending messages to log.
   ~Metrics() ;

   // Public interface for metrics logging
public:
   /// This class defines a manipulator that can be used to end log
   /// entries and begin new ones. It is useful when we want to create an
   /// explicit instance of the Metrics::Log class (see below) and
   /// "reuse" that object for multiple log messages.
   struct endl {} ;

   /// This class defines a manipulator that can be used to break up long
   /// log entries over multiple lines. Each new line in an entry will be
   /// indented with eight spaces, while the previous ones end with a
   /// backslash. The first line in such a multiple line log entry will
   /// have a time-stamp prefix while the indented lines will not.
   struct newl {} ;

   /**
      \class lobot::Metrics::Log
      \brief A class for creating metrics related log messages and
      buffering them with the metrics behaviour for subsequent output to
      the log file.

      To create metrics related log messages, client behaviours should
      create a local instance of the Metrics::Log class and use the
      stream insertion operator to write messages to the metrics log
      file. The constructor of this class takes care of time-stamping
      each message and the destructor buffers the message with the
      Metrics behaviour. Here is the intended usage pattern for this
      class:

         void some_class::some_function()
         {
            Pose p = something_complicated() ;
            Metrics::Log() << "current pose: "
                           << p.x() << ' '
                           << p.y() << ' '
                           << p.t() ;
         }

      When the local Log object created in the chained operator<<
      statement goes out of scope, the message will be added to the
      Metrics behaviour's internal message buffer, which will then take
      care of periodically dumping all the messages to the log file.

      NOTE: It is not shown in the above example; however, clients may
      use I/O manipulators to get formatted output. But please don't end
      with newlines; those are added automatically when the message
      buffer is dumped to the log file.

      NOTE 2: To break up long lines, use the Metrics::newl "manipulator"
      like so:

         void some_class::some_function()
         {
            Metrics::Log() << "a fantastically long message that needs"
                           << Metrics::newl()
                           << "to be broken up across multiple lines" ;
         }

      The Metrics::newl manipulator will introduce a space followed by a
      backslash followed by a newline and then indent the next line in
      the log with eight spaces to indicate that this line is a
      continuation of the previous one.

      NOTE 3: Although the above examples illustrate using the metrics
      logging facility with an unnamed temporary object, it is also
      perfectly acceptable to explicitly create a Metrics::Log object and
      use it multiple times. However, keep in mind that all log messages
      will be treated as a single log entry and will show up on one line
      unless you use the Metrics::endl manipulator as shown below:

         void some_class::some_function()
         {
            Metrics::Log metlog ;
            metlog << "this is the first entry"  << Metrics::endl() ;
            metlog << "this is the second entry" << Metrics::endl() ;
            metlog << "oops, this and the next line" ;
            metlog << "are both part of the same log entry!" ;
         }
   */
   class Log {
      /// The metrics log messages are accumulated in an output string
      /// stream using the stream insertion operator. The destructor and
      /// the Metrics::endl manipulator will result in the contents of
      /// this string stream being buffered with the metrics behaviour.
      std::ostringstream str ;

      /// Each log entry will be automatically prefixed with a
      /// time-stamp. This flag keeps track of whether each use of the
      /// stream insertion operator requires a time-stamp to be output or
      /// not. When a Metrics::Log object is created, this flag will be
      /// initialized to true so that the very first operator<< results
      /// in a time-stamp. After that, this flag will be cleared so that
      /// subsequent uses of operator<< don't cause additional
      /// time-stamps.
      ///
      /// The Metrics::endl manipulator also results in setting this
      /// flag.
      bool need_ts ;

      /// A helper function to spit out a time-stamp.
      void ts() ;

      /// A helper function to send log messages to the metrics
      /// behaviour's internal message buffer.
      void send() ;

   public:
      /// When a Metrics::Log object is created, it will automatically
      /// cause a time-stamp to be prefixed to the message.
      Log() ;

      /// When a Metrics::Log object goes out of scope, it will queue the
      /// message with the metrics behaviour, which, in turn, will take
      /// care of periodically writing the messages to the log file.
      ~Log() ;

      /// The stream insertion operator for arbitrary data types.
      ///
      /// To create log entries, clients should instantiate the
      /// Metrics::Log class and "build" the full message by chaining
      /// multiple calls to this operator. When the Metrics::Log object
      /// goes out of scope, the message will be queued with the metrics
      /// behaviour.
      template<typename T> Log& operator<<(const T&) ;

      /// Conversion operator. This function converts Metrics::Log
      /// objects to std::ostringstream objects. This allows clients to
      /// simply reuse their operator<< definitions for std::ostream and
      /// not have to provide an overload for use with Metrics::Log.
      operator std::ostringstream&() {return str ;}

      // Dealing with the Metrics::endl and Metrics::newl manipulators
      friend Log& operator<<(Log&, const endl&) ;
      friend Log& operator<<(Log&, const newl&) ;
   } ;

   // Log objects will need to access the metrics behaviour's internal buffer
   friend class Log ;

   /// To make the metrics log more readable, each message's
   /// initial/opening phrase (such as "tracking pose" or "extricate") is
   /// left-justified within a field width specified in the config file.
   /// This helper function returns that setting.
   static int opening_phrase_width() ;

   /// A shorter alias for above function.
   static int opw() {return opening_phrase_width() ;}
} ;

//--------------------- TEMPLATE MEMBER FUNCTIONS -----------------------

// Stream insertion operator for arbitrary data types. Client modules
// need not define an operator<< for use especially with Metrics::Log
// (though they may if they so wish). The Metrics::Log-to-std::ostream
// conversion operator ensures that client modules that define an
// operator<< for use with std::ostream will work unchanged in
// conjunction with Metrics::Log.
template<typename T>
Metrics::Log& Metrics::Log::operator<<(const T& t)
{
   if (need_ts) {
      ts() ;
      need_ts = false ;
   }
   str << t ;
   return *this ;
}

//----------------------- NON-MEMBER FUNCTIONS --------------------------

/// This function ends a log entry and begins a new one. It is meant for
/// use with Metrics::Log objects created explicitly so that they may be
/// "reused" to output multiple log messages.
Metrics::Log& operator<<(Metrics::Log&, const Metrics::endl&) ;

/// This function breaks up long log entries into multiple lines by
/// inserting a newline and indenting the next line of a multiple line
/// log entry with two tabs.
Metrics::Log& operator<<(Metrics::Log&, const Metrics::newl&) ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
