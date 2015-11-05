/*!@file Learn/RecurRecurBayes.C Recursive RecurBayesian network classifier */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Learn/RecurBayes.C $
// $Id: RecurBayes.C 10794 2009-02-08 06:21:09Z itti $
//

//This is a Naive RecurBayes for now
#ifndef LEARN_BAYES_C_DEFINED
#define LEARN_BAYES_C_DEFINED

#include "Learn/RecurBayes.H"
#include "Util/Assert.H"
#include "Util/log.H"
#include <math.h>
#include <fcntl.h>
#include <limits>
#include <string>

// ######################################################################
RecurBayes::RecurBayes(uint numClasses, uint numFeatures, uint numFix):
  itsNumFeatures(numFeatures), itsNumClasses(numClasses), itsNumFix(numFix),
  itsMean(numClasses,
      std::vector<std::vector<double> >
          (numFix, std::vector<double>(numFeatures,0))),
  itsVar(numClasses,
      std::vector<std::vector<double> >
          (numFix, std::vector<double>(numFeatures,0.01))),
  itsClassFreq(numClasses,0),
  itsFeatureNames(numFeatures),
  itsClassNames(numClasses, "No Name")
{

}

// ######################################################################
RecurBayes::~RecurBayes()
{
}


// ######################################################################
void RecurBayes::learn(std::vector<double> &fv, const char *name, uint fix)
{
  //get the class id

  int cls = getClassId(name);
  if (cls == -1) //this is a new class, add it to the network
    cls = addClass(name);

  ASSERT(fv.size() == itsNumFeatures);

  //update the class freq
  ASSERT((uint)cls < itsNumClasses);
  itsClassFreq[cls]++;

  //compute the stddev and mean of each feature
  //This algorithm is due to Knuth (The Art of Computer Programming, volume 2:
  //  Seminumerical Algorithms, 3rd edn., p. 232. Boston: Addison-Wesley.)
  for (uint i=0; i<fv.size(); i++){
    double val = fv[i];
    double delta = val - itsMean[cls][fix][i];
    itsMean[cls][fix][i] += delta/itsClassFreq[cls];
    if (itsClassFreq[cls] > 3)
    {
      itsVar[cls][fix][i] = (itsVar[cls][fix][i]*(itsClassFreq[cls]-2))
        + delta*(val - itsMean[cls][fix][i]);
    }
    if (itsClassFreq[cls] > 1) //watch for divide by 0
      itsVar[cls][fix][i] /= double(itsClassFreq[cls]-1);
  }

}

// ######################################################################
double RecurBayes::getMean(uint cls, uint i, uint fix)
{
  ASSERT(cls < itsNumClasses && i < itsNumFeatures);
  return itsMean[cls][fix][i];
}

// ######################################################################
double RecurBayes::getStdevSq(uint cls, uint i, uint fix)
{
  ASSERT(cls < itsNumClasses && i < itsNumFeatures);
  return itsVar[cls][fix][i];
}

// ######################################################################
uint RecurBayes::getNumFeatures()
{
  return itsNumFeatures;
}

// ######################################################################
uint RecurBayes::getNumClasses()
{
  return itsNumClasses;
}

// ######################################################################
uint RecurBayes::getClassFreq(uint cls)
{
  ASSERT(cls < itsNumClasses);
  return itsClassFreq[cls];
}

// ######################################################################
double RecurBayes::getClassProb(uint cls)
{
  ASSERT(cls < itsNumClasses);

  //double totalFreq = 0;
  //for (uint i=0; i<itsNumClasses; i++)
  //  totalFreq += itsClassFreq[i];

  //return double(itsClassFreq[cls])/totalFreq;

  return double(1/itsNumClasses);
}

// ######################################################################
int RecurBayes::classify(std::vector<double> &fv, double *prob, uint fix)
{

  //the maximum posterior  (MAP alg):
  double maxProb = -std::numeric_limits<double>::max();
  int maxCls = -1;
  double sumClassProb = 0;

  for(uint cls=0; cls<itsNumClasses; cls++)
  {
    //Find the probability that the fv belongs to this class
    double probVal = log(getClassProb(cls)); //the prior probility
    for (uint i=0; i<itsNumFeatures; i++) //get the probilityposterior prob
    {
      if (itsMean[cls][fix][i] > 0)  //only process if mean > 0
      {
        double g = gauss(fv[i], itsMean[cls][fix][i], itsVar[cls][fix][i]);
        probVal += log(g);


       //  LINFO("Val %f Mean %f sigma %f g(%e) %e",
       //      fv[i], itsMean[cls][i], itsStdevSq[cls][i], g, probVal);
      }
    }

    LINFO("Class %i prob %f\n", cls, probVal);
    sumClassProb += probVal;
    if (probVal > maxProb){ //we have a new max
      maxProb = probVal;
      maxCls = cls;
    }
  }

  if (prob != NULL)
    *prob = exp(maxProb)/exp(sumClassProb);

  return maxCls;
}


// ######################################################################
double RecurBayes::getStatSig(std::vector<double> &fv, uint cls, uint fix)
{
  ASSERT(fv.size() == itsNumFeatures);

  double statSig = 0;

  //simple t test
  for (uint i=0; i<fv.size(); i++){
    //compute a t test for each feature
    double val = fv[i];
    if (itsVar[cls][fix][i] != 0.00)
    {
      double fsig = log(fabs(val - itsMean[cls][fix][i])) - log(itsVar[cls][fix][i]);
      statSig += fsig;
    }
  }

  return statSig;

}

//// ######################################################################
double RecurBayes::gauss(double x, double mean, double stdevSq)
{
 double delta = -(x - mean) * (x - mean);
 return exp(delta/(2*stdevSq))/(sqrt(2*M_PI*stdevSq));
}

//// ######################################################################
void RecurBayes::save(const char *filename)
{

  int fd;

  if ((fd = creat(filename, 0644)) == -1) {
    printf("Can not open %s for saving\n", filename);
    return;
  }

  //write the #  Features and Classes
  write(fd, &itsNumFeatures, sizeof(uint));
  write(fd, &itsNumClasses, sizeof(uint));

  //Write the class freq and names
  for(uint i=0; i<itsNumClasses; i++)
  {
    write(fd, &itsClassFreq[i], sizeof(uint64));

    uint clsNameLength = itsClassNames[i].size()+1; //1 for null str terminator
    write(fd, &clsNameLength, sizeof(uint));
    write(fd, itsClassNames[i].c_str(), sizeof(char)*clsNameLength);
  }


  //Write the mean and stdev
  for(uint cls=0; cls<itsNumClasses; cls++)
  {
    for (uint i=0; i<itsNumFeatures; i++) //get the posterior prob
    {
      int fix = 1;
      write(fd, &itsMean[cls][fix][i], sizeof(double));
      write(fd, &itsVar[cls][fix][i], sizeof(double));
    }
  }

  close(fd);


}

//// ######################################################################
void RecurBayes::load(const char *filename)
{

  int fd;

  if ((fd = open(filename, 0644)) == -1) {
    printf("Can not open %s for reading\n", filename);
    return;
  }

  itsNumClasses = 0;
  itsNumFeatures = 0;
  //read the #  Features and Classes
  read(fd, &itsNumFeatures, sizeof(uint));
  read(fd, &itsNumClasses, sizeof(uint));

  //read the class freq
  itsClassFreq.clear();
  itsClassFreq.resize(itsNumClasses);
  itsClassNames.resize(itsNumClasses);

  for(uint i=0; i<itsNumClasses; i++)
  {
    read(fd, &itsClassFreq[i], sizeof(uint64));

    uint clsNameLength;
    read(fd, &clsNameLength, sizeof(uint));
    char clsName[clsNameLength];
    read(fd, &clsName, sizeof(char)*clsNameLength);
    itsClassNames[i] = std::string(clsName);

  }


  //Write the mean and stdev
  itsMean.clear();
  itsMean.resize(itsNumClasses,
      std::vector<std::vector<double> >
          (itsNumFix, std::vector<double>(itsNumFeatures,0)));


  itsVar.clear();
  itsVar.resize(itsNumClasses,
      std::vector<std::vector<double> >
          (itsNumFix, std::vector<double>(itsNumFeatures,0.01)));

  for(uint cls=0; cls<itsNumClasses; cls++)
  {
    for (uint i=0; i<itsNumFeatures; i++) //get the posterior prob
    {
      read(fd, &itsMean[cls][i], sizeof(double));
      read(fd, &itsVar[cls][i], sizeof(double));
    }
  }

  close(fd);

}

// ######################################################################
void RecurBayes::setFeatureName(uint index, const char *name)
{
  ASSERT(index < itsNumFeatures);
  itsFeatureNames[index] = std::string(name);
}

// ######################################################################
const char* RecurBayes::getFeatureName(const uint index) const
{
  ASSERT(index < itsNumFeatures);
  return itsFeatureNames[index].c_str();
}


// ######################################################################
int RecurBayes::addClass(const char *name)
{
  //Add a new class

  //check if the class exsists
  if (getClassId(name) == -1)
  {
    itsClassNames.push_back(std::string(name));

    itsMean.push_back(std::vector<std::vector<double> >
        (itsNumFix, std::vector<double>(itsNumFeatures,0)));


    itsVar.push_back(std::vector<std::vector<double> >
        (itsNumFix, std::vector<double>(itsNumFeatures,0.01))),


    itsClassFreq.push_back(1);

    return itsNumClasses++;
  }

  return -1;

}

// ######################################################################
const char* RecurBayes::getClassName(const uint id)
{
  ASSERT(id < itsNumClasses);
  return itsClassNames[id].c_str();

}

// ######################################################################
int RecurBayes::getClassId(const char *name)
{
  //TODO: should use hash_map (but no hash_map on this machine :( )

  for(uint i=0; i<itsClassNames.size(); i++){
    if (!strcmp(itsClassNames[i].c_str(), name))
      return i;
  }

  return -1;  //no class found but that name

}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // LEARN_BAYES_C_DEFINED
