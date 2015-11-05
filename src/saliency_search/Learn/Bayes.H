/*!@file Learn/Bayes.H Bayesian network classifier */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Learn/Bayes.H $
// $Id: Bayes.H 10794 2009-02-08 06:21:09Z itti $
//

//This is a Naive Bayes for now
#ifndef LEARN_BAYES_H_DEFINED
#define LEARN_BAYES_H_DEFINED

#include "Util/Types.H" // for uint
#include <vector>
#include <string>

class Bayes
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
  Bayes(uint numFeatures, uint numClasses);

  //! Destructor
  ~Bayes();

  //! Learn to associate a feature vector with a particuler class
  void learn(const std::vector<double> &fv, const uint cls); //TODO make as a Template

  //! Learn to associate a feature vector with a particuler class name
  void learn(const std::vector<double> &fv, const char *name); //TODO make as a Template

  //! classify a given feature vector
  int classify(const std::vector<double> &fv, double *prob = NULL); //TODO make as a template

  //! classify a given feature vector (Return all classes and thier prob, cls contains the max)
  std::vector<ClassInfo> classifyRange(std::vector<double> &fv, int &retCls, const bool sort=true);

  //! Return the probability of all the classes given the feature vector
  std::vector<double> getClassProb(const std::vector<double> &fv);

  //! Get the mean for a particuler feature
  double getMean(const uint cls, const uint i) const;

  //! Get the stdev Squared for a particuler feature
  double getStdevSq(const uint cls, const uint i) const;

  //! set the mean
  void  setMean(const uint cls, const uint i, const double val);

  //! set the stdev Squared for a particuler feature
  void setStdevSq(const uint cls, const uint i, const double val);

  //! Get the number of features
  uint getNumFeatures() const;

  //! Get the number of classes
  uint getNumClasses() const;

  //! Get the Freq of a given class
  uint getClassFreq(const uint cls) const;

  //! Get the probability of a given class
  double getClassProb(const uint cls) const;

  //! return the statistical significent of the FV for a given class
  double getStatSig(const std::vector<double> &fv, const uint cls) const;

  //! Calculate a Normal Dist (use the srdev squared
  double gauss(const double x, const double mean, const double stdevSq) const;

  //! Save the network to a file
  void save(const char *filename);

  //! Load the network from a binary file
  bool load(const char *filename);

  //! Load the network from a text file
  void import(const char *filename);

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

  //! get the probability value associated with a classification
  double getMaxProb() const;

  //! get the normalized probability value associated with a classification
  double getNormProb() const;
private:
  uint   itsNumFeatures; //the number of features we have
  uint   itsNumClasses;  //the Number of classes we have
  double itsMaxProb;     // Stores the maximum probability with each object rec
  double itsSumProb;     // Used to derive a normalized P value
  double itsNormProb;    // normalized P of object
  std::vector<std::vector<double> > itsMean;  //the mean for each feature per class
  std::vector<std::vector<double> > itsStdevSq;  //the stdev squared for each feature
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
