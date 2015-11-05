/**
   \file  Robots/LoBot/misc/LoTTIMap.H
   \brief An object to store the LGMD spike rates, predicted
   times-to-impact and prediction confidence levels corresponding to each
   actual time-to-impact recorded in the log files for the Bayesian TTI
   prediction experiments.

   The Robolocust TTI algorithm for LGMD-based obstacle avoidance uses a
   Bayesian state estimator to predict the time-to-impact given an LGMD
   spike rate. To evaluate how well this prediction model works, we
   conducted several experiments under varying conditions.

   The log files collected during these experiments record various pieces
   of relevant information such as the actual time-to-impact between the
   robot and a wall straight ahead it towards which the robot is driving,
   the artificially generated LGMD spike rate corresponding to the actual
   TTI, the time-to-impact predicted by the Bayesian TTI model, the
   confidence level of this prediction, etc.

   The lobay program is designed to parse the above log files and output
   a results file that specifies the average spike rate, predicted TTI
   and prediction confidence for each actual TTI recorded in the log
   files making up a dataset.

   This file defines a class that keeps track of the mapping between the
   actual TTI values and the corresponding spike rates, predictions and
   confidence levels. Since different individual runs of the robot can
   produce slightly different records, we maintain a list of spike rates,
   TTI predictions and confidence levels per actual TTI.

   Once all the logs in a dataset have been loaded and parsed, we will
   write the final results to a file of the following format:

        TTI   LGMD Spike Rate   Predicted TTI   Confidence Level
        ---   ---------------   -------------   ----------------
              mean   stdev      mean   stdev    mean   stdev

   NOTE: The informational header shown above won't actually appear in
   the results file. Rather, the file will simply contain 7 columns of
   floating point numbers. The results file ought to be suitable as
   direct input for a program such as gnuplot.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/baylog/LoTTIMap.H $
// $Id: LoTTIMap.H 14083 2010-09-30 13:59:37Z mviswana $
//

#ifndef LOBOT_BAYLOG_TTI_MAP_DOT_H
#define LOBOT_BAYLOG_TTI_MAP_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/util/triple.hh"

// Standard C++ headers
#include <ostream>
#include <string>
#include <map>
#include <vector>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::TTIMap
   \brief A mapping between actual times-to-impact recorded in logs
   generated during the Bayesiant TTI prediction experiments and the
   corresponding LGMD spike rates, predicted times-to-impact and
   prediction confidence levels.

   This class maps the actual TTI values read from a Bayesian
   time-to-impact prediction experiment's log file to the corresponding
   LGMD spike rates, predicted TTI's and confidence levels (also recorded
   in the log).

   Sometimes, an experiment may record the same actual time-to-impact
   multiple times. Each such occurence may be associated with a slightly
   different set of values for the predicted TTI, prediction confidence
   and LGMD spike rate. Furthermore, the different logs making up a
   dataset might record the same values for actual TTI's with slightly
   different values for the corresponding predictions and LGMD spike
   rate.

   Therefore, for each actual TTI, we maintain a list of corresponding
   predictions, confidence levels and LGMD spike rates. To figure out the
   average-case behaviour of the Bayesian prediction model, we compute
   the means and standard deviations of all these lists for each actual
   TTI and store the results to a file.
*/
class TTIMap {
   // Prevent copy and assignment
   TTIMap(const TTIMap&) ;
   TTIMap& operator=(const TTIMap&) ;

   /// The whole idea behind this class is to keep track of a list of
   /// correspondences between actual TTI values recorded in log files
   /// and LGMD spike rates, predicted TTI values and prediction
   /// confidence levels. These types and data members take care of the
   /// details.
   //@{
   typedef std::vector<float> List ;
   typedef triple<List, List, List> Triple ;
   typedef std::map<std::string, Triple> Map ;
   typedef Map::value_type MapEntry ;
   Map m_map ;
   //@}

public:
   /// Constructor.
   TTIMap() ;

   /// This method adds a record parsed from a log file to the map.
   void add(float actual_tti,
            float lgmd_spike_rate,
            float predicted_tti, float prediction_confidence) ;

private:
   /// Helper function object to dump a map entry to the specified output
   /// stream.
   class dump {
      std::ostream& os ;
   public:
      dump(std::ostream&) ;
      void operator()(const MapEntry&) const ;
   } ;

   /// Output operator. It should be used after the whole map has been
   /// assembled from all the logs in a dataset.
   ///
   /// This function will compute the means and standard deviations of
   /// all the LGMD spike rates, predicted TTI values and prediction
   /// confidence levels corresponding to each actual TTI value and write
   /// the results to the supplied stream.
   friend std::ostream& operator<<(std::ostream&, const TTIMap&) ;
} ;

//----------------------- NON-MEMBER FUNCTIONS --------------------------

/// This function writes the given TTIMap to the specified stream,
/// computing the proper averages, etc. as required. It should only be
/// used after the TTIMap has been "fully assembled" by parsing all the
/// log files for a Bayesian TTI prediction experiment dataset.
std::ostream& operator<<(std::ostream&, const TTIMap&) ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
