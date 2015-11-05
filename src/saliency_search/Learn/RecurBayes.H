/*!@file Learn/RecurBayes.H RecurBayesian network classifier */

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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Learn/RecurBayes.H $
// $Id: RecurBayes.H 9108 2007-12-30 06:14:30Z rjpeters $
//

//This is a Naive RecurBayes for now
#ifndef LEARN_BAYES_H_DEFINED
#define LEARN_BAYES_H_DEFINED

#include "Util/Types.H" // for uint
#include <vector>
#include <string>

class RecurBayes
{
public:

  struct ClassInfo
  {
    ClassInfo(int id, double p, double sig) : classID(id), prob(p), statSig(sig) {} //constructor to set values
    int classID; //the class ID;
    double prob; //the probability of this class
    double statSig; //the statistical significance between the features value and the params
  };

  //! Construct a bayes classifer with a given number of features and
  //! number of classes
  RecurBayes(uint numClasses, uint numFeatures, uint nfix);

  //! Destructor
  ~RecurBayes();

  //! Learn to associate a feature vector with a particuler class name
  void learn(std::vector<double> &fv, const char *name, uint fix); //TODO make as a Template

  //! classify a given feature vector
  int classify(std::vector<double> &fv, double *prob = NULL, uint fix = 1); //TODO make as a template

  //! Get the mean for a particuler feature
  double getMean(uint cls, uint i, uint fix);

  //! Get the stdev Squared for a particuler feature
  double getStdevSq(uint cls, uint i, uint fix);

  //! Get the number of features
  uint getNumFeatures();

  //! Get the number of classes
  uint getNumClasses();

  //! Get the Freq of a given class
  uint getClassFreq(uint cls);

  //! Get the probability of a given class
  double getClassProb(uint cls);

  //! return the statistical significent of the FV for a given class
  double getStatSig(std::vector<double> &fv, uint cls, uint fix);

  //! Calculate a Normal Dist (use the srdev squared
  double gauss(double x, double mean, double stdevSq);

  //! Save the network to a file
  void save(const char *filename);

  //! Load the network from a file
  void load(const char *filename);

  //! set feature name (for debuging)
  void setFeatureName(uint index, const char *name);

  //! get feature name (for debuging)
  const char* getFeatureName(const uint index) const;

  //! Add class by name and return its Id
  int addClass(const char *name);

  //! Get the class name from a given Id
  const char* getClassName(const uint id);

  //! Get the class id from a given name
  int getClassId(const char *name);

private:

  uint itsNumFeatures;   //the number of features we have
  uint itsNumClasses;   //the Number of classes we have
  uint itsNumFix;
  std::vector<std::vector<std::vector<double> > > itsMean;  //the mean for each feature per class per fixation
  //the stdev squared for each feature
  std::vector<std::vector<std::vector<double> > > itsVar;  //the variance for each feature per class per fixation
  //TODO: its long int sufficent? is there a better way of calc the mean and stdev?
  std::vector<uint64> itsClassFreq;   //the Freq of a given class

  std::vector<std::string> itsFeatureNames; //THe name of the features
  std::vector<std::string> itsClassNames;   //The names of the clases

};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // LEARN_BAYES_H_DEFINED
