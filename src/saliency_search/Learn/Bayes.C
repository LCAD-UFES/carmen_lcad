/*!@file Learn/Bayes.C Bayesian network classifier */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Learn/Bayes.C $
// $Id: Bayes.C 15310 2012-06-01 02:29:24Z itti $
//

//This is a Naive Bayes for now
#include "Learn/Bayes.H"
#include "Util/Assert.H"
#include "Util/log.H"
#include <math.h>
#include <fcntl.h>
#include <limits>
#include <string>
#include <algorithm>

#include <cstdio>
#include <unistd.h> // for write()

// functor to assist with classInfo sorting:
class lessClassInfo
{
public:
  bool operator()(const Bayes::ClassInfo& classA,
                  const Bayes::ClassInfo& classB)
  { return (classA.prob) > (classB.prob); }
};

// ######################################################################
Bayes::Bayes(uint numFeatures, uint numClasses):
  itsNumFeatures(numFeatures), itsNumClasses(numClasses),
  itsMean(numClasses, std::vector<double>(numFeatures,0)),
  itsStdevSq(numClasses, std::vector<double>(numFeatures,0.01)),
  itsClassFreq(numClasses,0),
  itsFeatureNames(numFeatures),
  itsClassNames(numClasses, "No Name")
{

}

// ######################################################################
Bayes::~Bayes()
{}

// ######################################################################
void Bayes::learn(const std::vector<double> &fv, const uint cls)
{

  ASSERT(fv.size() == itsNumFeatures);

  //update the class freq
  ASSERT(cls < itsNumClasses);
  itsClassFreq[cls]++;

  //compute the stddev and mean of each feature
  //This algorithm is due to Knuth (The Art of Computer Programming, volume 2:
  //  Seminumerical Algorithms, 3rd edn., p. 232. Boston: Addison-Wesley.)
  for (uint i=0; i<fv.size(); i++){
    const double val = fv[i];
    const double delta = val - itsMean[cls][i];
    itsMean[cls][i] += delta/itsClassFreq[cls];
    if (itsClassFreq[cls] > 2) //watch for divide by 0
    {
      itsStdevSq[cls][i] = (itsStdevSq[cls][i]*(itsClassFreq[cls]-2))
        + delta*(val - itsMean[cls][i]);
      itsStdevSq[cls][i] /= double(itsClassFreq[cls]-1);
    }
  }
}

// ######################################################################
void Bayes::learn(const std::vector<double> &fv, const char *name)
{
  //get the class id

  int cls = getClassId(name);
  if (cls == -1) //this is a new class, add it to the network
    cls = addClass(name);

  if(fv.size() != itsNumFeatures)
  {
    LINFO("NOTE: deleting the .net file may fix this if you are");
    LINFO("training with a different set of features.");
    LFATAL("fv.size() != itsNumFeatures: %d != %d",(int)fv.size(),
           itsNumFeatures);
  }

  //ASSERT(fv.size() == itsNumFeatures);

  //update the class freq
  ASSERT((uint)cls < itsNumClasses);
  itsClassFreq[cls]++;

  //compute the stddev and mean of each feature
  //This algorithm is due to Knuth (The Art of Computer Programming, volume 2:
  //  Seminumerical Algorithms, 3rd edn., p. 232. Boston: Addison-Wesley.)
  /*
  for (uint i=0; i<fv.size(); i++){
    const double val = fv[i];
    const double delta = val - itsMean[cls][i];
    itsMean[cls][i] += delta/itsClassFreq[cls];
    if (itsClassFreq[cls] > 3)
    {
      itsStdevSq[cls][i] = (itsStdevSq[cls][i]*(itsClassFreq[cls]-2))
        + delta*(val - itsMean[cls][i]);
    }
    if (itsClassFreq[cls] > 1) //watch for divide by 0
      itsStdevSq[cls][i] /= double(itsClassFreq[cls]-1);
  }
  */

  //watch for divide by 0
  if(itsClassFreq[cls] > 3)
  {
    const double freq1 = 1.0F/(double)itsClassFreq[cls];
    const double freq2 = 1.0F/(double)(itsClassFreq[cls]-1);
    const uint64 freq  = itsClassFreq[cls];

    for (uint i=0; i<fv.size(); i++)
    {
      const double val   = fv[i];
      const double delta = val - itsMean[cls][i];

      itsMean[cls][i]    += delta * freq1;
      itsStdevSq[cls][i] = (itsStdevSq[cls][i]*(freq-2))
        + delta*(val - itsMean[cls][i]);
      itsStdevSq[cls][i] *= freq2;
    }
  }
  else if(itsClassFreq[cls] > 1)
  {
    const double freq1 = 1.0F/(double)itsClassFreq[cls];
    const double freq2 = 1.0F/(double)(itsClassFreq[cls]-1);

    for (uint i=0; i<fv.size(); i++)
    {
      const double val   = fv[i];
      const double delta = val - itsMean[cls][i];

      itsMean[cls][i]    += delta * freq1;
      itsStdevSq[cls][i] *= freq2;
    }
  }
  else
  {
    const double freq1 = 1.0F/(double)itsClassFreq[cls];

    for (uint i=0; i<fv.size(); i++)
    {
      const double val   = fv[i];
      const double delta = val - itsMean[cls][i];

      itsMean[cls][i]    += delta * freq1;
    }
  }

}

// ######################################################################
double Bayes::getMean(const uint cls, const uint i) const
{
  ASSERT(cls < itsNumClasses && i < itsNumFeatures);
  return itsMean[cls][i];
}

// ######################################################################
double Bayes::getStdevSq(const uint cls, const uint i) const
{
  ASSERT(cls < itsNumClasses && i < itsNumFeatures);
  return itsStdevSq[cls][i];
}

// ######################################################################
void  Bayes::setMean(const uint cls, const uint i, const double val)
{
  ASSERT(cls < itsNumClasses && i < itsNumFeatures);
  itsMean[cls][i] = val;

}

// ######################################################################
void Bayes::setStdevSq(const uint cls, const uint i, const double val)
{
  ASSERT(cls < itsNumClasses && i < itsNumFeatures);
  itsStdevSq[cls][i] = val;
}

// ######################################################################
uint Bayes::getNumFeatures() const
{
  return itsNumFeatures;
}

// ######################################################################
uint Bayes::getNumClasses() const
{
  return itsNumClasses;
}

// ######################################################################
uint Bayes::getClassFreq(const uint cls) const
{
  ASSERT(cls < itsNumClasses);
  return itsClassFreq[cls];
}

// ######################################################################
double Bayes::getClassProb(const uint cls) const
{
  ASSERT(cls < itsNumClasses);

  //double totalFreq = 0;
  //for (uint i=0; i<itsNumClasses; i++)
  //  totalFreq += itsClassFreq[i];

  //return double(itsClassFreq[cls])/totalFreq;

  return double(1/itsNumClasses);
}

// ######################################################################
int Bayes::classify(const std::vector<double> &fv, double *prob)
{

  //the maximum posterior  (MAP alg):
  itsMaxProb  = -std::numeric_limits<double>::max();
  itsSumProb  = 0.0F;
  itsNormProb = 0.0F;
  int maxCls = -1;
  //double sumClassProb = 0;

  for(uint cls=0; cls<itsNumClasses; cls++)
  {
    LINFO("Class %d of %d - %s",cls,itsNumClasses,itsClassNames[cls].c_str());
    //Find the probability that the fv belongs to this class
    double probVal = 0; ////log(getClassProb(cls)); //the prior probility
    for (uint i=0; i<itsNumFeatures; i++) //get the probilityposterior prob
    {
      if (itsMean[cls][i] > 0)  //only process if mean > 0
      {
        const double g = gauss(fv[i], itsMean[cls][i], itsStdevSq[cls][i]);
        probVal += log(g);

        //LINFO("Val %f Mean %f sigma %f g(%e) %e",
        //      fv[i], itsMean[cls][i], itsStdevSq[cls][i], g, probVal);
      }
    }

    //if (probVal == NAN || probVal == -INFINITY) probVal = 1; //log of 0
    //printf("Class %i %s prob %f\n", cls, getClassName(cls), probVal);

    //sumClassProb += probVal;

    itsSumProb += exp(probVal);
    if (probVal > itsMaxProb){ //we have a new max
      itsMaxProb = probVal;
      maxCls = cls;
    }
  }

  itsMaxProb  = exp(itsMaxProb);
  itsNormProb = itsMaxProb / itsSumProb;

  if (prob != NULL)
    *prob = itsMaxProb; //)/exp(sumClassProb);

  return maxCls;
}

// ######################################################################
std::vector<Bayes::ClassInfo> Bayes::classifyRange(std::vector<double> &fv,
                                                   int &retCls, const bool sortit)
{

  std::vector<ClassInfo> classInfoRet;

  //the maximum posterior  (MAP alg):
  itsMaxProb  = -std::numeric_limits<double>::max();
  itsSumProb  = 0.0F;
  itsNormProb = 0.0F;
  int maxCls = -1;

  for(uint cls=0; cls<itsNumClasses; cls++)
  {
    //Find the probability that the fv belongs to this class
    double probVal = 0; //log(getClassProb(cls)); //the prior probility
    for (uint i=0; i<itsNumFeatures; i++) //get the posterior prob
    {
      if (itsMean[cls][i] > 0)  //only process if mean > 0
      {
        const double g = gauss(fv[i], itsMean[cls][i], itsStdevSq[cls][i]);
        probVal += log(g);
      }
    }

    itsSumProb += exp(probVal);
    if (probVal > itsMaxProb){ //we have a new max
      itsMaxProb = probVal;
      maxCls = cls;
    }
    classInfoRet.push_back(ClassInfo(cls, probVal, getStatSig(fv, cls)));
  }

  itsMaxProb  = exp(itsMaxProb);
  itsNormProb = itsMaxProb / itsSumProb;

  retCls = maxCls;
  if (sortit)
    std::sort(classInfoRet.begin(), classInfoRet.end(), lessClassInfo());
  return classInfoRet;
}

// ######################################################################
std::vector<double> Bayes::getClassProb(const std::vector<double> &fv)
{

  std::vector<double> classProb(itsNumClasses);

  for(uint cls=0; cls<itsNumClasses; cls++)
  {
    //Find the probability that the fv belongs to this class
    double probVal = log(1/(float)itsNumClasses); //log(getClassProb(cls)); //the prior probility
    for (uint i=0; i<itsNumFeatures; i++) //get the posterior prob
    {
      if (itsMean[cls][i] != 0)  //only process if mean > 0
      {

        const double g = gauss(fv[i], itsMean[cls][i], itsStdevSq[cls][i]);
        probVal += log(g);
        //LINFO("%i: %f %f %f => %e,%e",
        //    cls, fv[i], itsMean[cls][i], itsStdevSq[cls][i],g, probVal);
      } else {
        probVal += log(0);
      }
    }

    classProb[cls] = probVal;
  }

  return classProb;
}

// ######################################################################
double Bayes::getStatSig(const std::vector<double> &fv, const uint cls) const
{
  ASSERT(fv.size() == itsNumFeatures);

  double statSig = 0;

  //simple t test
  for (uint i=0; i<fv.size(); i++){
    //compute a t test for each feature
    const double val = fv[i];
    if (itsStdevSq[cls][i] != 0.00)
    {
      const double fsig =
        log(fabs(val - itsMean[cls][i])) - log(itsStdevSq[cls][i]);
      statSig += fsig;
    }
  }

  return statSig;

}

//// ######################################################################
inline double Bayes::gauss(const double x, const double mean, const double stdevSq) const
{
 const double delta = -(x - mean) * (x - mean);
 return exp(delta/(2*stdevSq))/(sqrt(2*M_PI*stdevSq));
}

//// ######################################################################
void Bayes::save(const char *filename)
{

  int fd;

  if ((fd = creat(filename, 0644)) == -1) {
    printf("Can not open %s for saving\n", filename);
    return;
  }

  //write the #  Features and Classes
  if(write(fd, &itsNumFeatures, sizeof(uint)) != sizeof(uint)) LFATAL("Failed to write into: %s", filename);
  if(write(fd, &itsNumClasses, sizeof(uint)) != sizeof(uint)) LFATAL("Failed to write into: %s", filename);

  //Write the class freq and names
  for(uint i=0; i<itsNumClasses; i++)
  {
    if(write(fd, &itsClassFreq[i], sizeof(uint64)) != sizeof(uint64)) LFATAL("Failed to write into: %s", filename);
    uint clsNameLength = itsClassNames[i].size()+1; //1 for null str terminator
    if(write(fd, &clsNameLength, sizeof(uint)) != sizeof(uint)) LFATAL("Failed to write into: %s", filename);
    int sz = sizeof(char)*clsNameLength; 
    if(write(fd, itsClassNames[i].c_str(), sz) != sz) LFATAL("Failed to write into: %s", filename);
  }


  //Write the mean and stdev
  for(uint cls=0; cls<itsNumClasses; cls++)
  {
    for (uint i=0; i<itsNumFeatures; i++) //get the posterior prob
    {
      if(write(fd, &itsMean[cls][i], sizeof(double)) != sizeof(double)) LFATAL("Failed to write into: %s", filename);
      if(write(fd, &itsStdevSq[cls][i], sizeof(double)) != sizeof(double)) LFATAL("Failed to write into: %s", filename);
    }
  }

  close(fd);


}

//// ######################################################################
bool Bayes::load(const char *filename)
{
  int fd;

  if ((fd = open(filename, 0644)) == -1) {
    printf("Can not open %s for reading\n", filename);
    return false;
  }

  itsNumClasses = 0;
  itsNumFeatures = 0;
  //read the #  Features and Classes
  if(read(fd, &itsNumFeatures, sizeof(uint)) != sizeof(uint)) LFATAL("Failed to read from: %s", filename);
  if(read(fd, &itsNumClasses, sizeof(uint)) != sizeof(uint)) LFATAL("Failed to read from: %s", filename);

  //read the class freq
  itsClassFreq.clear();
  itsClassFreq.resize(itsNumClasses);
  itsClassNames.resize(itsNumClasses);

  for(uint i=0; i<itsNumClasses; i++)
  {
    if(read(fd, &itsClassFreq[i], sizeof(uint64)) != sizeof(uint64)) LFATAL("Failed to read from: %s", filename);

    uint clsNameLength;
    if(read(fd, &clsNameLength, sizeof(uint)) != sizeof(uint)) LFATAL("Failed to read from: %s", filename);
    char clsName[clsNameLength];
    int sz = sizeof(char)*clsNameLength;
    if(read(fd, &clsName, sz) != sz) LFATAL("Failed to read from: %s", filename);
    itsClassNames[i] = std::string(clsName);
  }


  //Write the mean and stdev
  itsMean.clear();
  itsMean.resize(itsNumClasses, std::vector<double>(itsNumFeatures,0));

  itsStdevSq.clear();
  itsStdevSq.resize(itsNumClasses, std::vector<double>(itsNumFeatures,0));

  for(uint cls=0; cls<itsNumClasses; cls++)
  {
    for (uint i=0; i<itsNumFeatures; i++) //get the posterior prob
    {
      if(read(fd, &itsMean[cls][i], sizeof(double)) != sizeof(double)) LFATAL("Failed to read from: %s", filename);
      if(read(fd, &itsStdevSq[cls][i], sizeof(double)) != sizeof(double)) LFATAL("Failed to read from: %s", filename);
    }
  }

  close(fd);

  return true;
}

//// ######################################################################
void Bayes::import(const char *filename)
{

  FILE *fd;

  if ((fd = fopen(filename, "r")) == NULL) {
    printf("Can not open %s for reading\n", filename);
    return;
  }


  //Read the mean and stdev
  for(uint cls=0; cls<itsNumClasses; cls++)
  {

    //read the means then the var
    for (uint i=0; i<itsNumFeatures; i++) //get the posterior prob
    {
      float mean = 0;
      int ret = fscanf(fd, "%f", &mean);
      if (ret == -1) //end of file or error, exit
      {
        fclose(fd);
        return;
      }
      itsMean[cls][i] = mean;
    }

    //read the means then the var
    for (uint i=0; i<itsNumFeatures; i++) //get the posterior prob
    {
      float stdevSq = 0;
      int ret = fscanf(fd, "%f", &stdevSq);
      if (ret == -1) //end of file or error, exit
      {
        fclose(fd);
        return;
      }
      itsStdevSq[cls][i] = stdevSq;
    }


  }

  fclose(fd);

}
// ######################################################################
void Bayes::setFeatureName(uint index, const char *name)
{
  ASSERT(index < itsNumFeatures);
  itsFeatureNames[index] = std::string(name);
}

// ######################################################################
const char* Bayes::getFeatureName(const uint index) const
{
  ASSERT(index < itsNumFeatures);
  return itsFeatureNames[index].c_str();
}

// ######################################################################
int Bayes::addClass(const char *name)
{
  //Add a new class

  //check if the class exsists
  if (getClassId(name) == -1)
  {
    itsClassNames.push_back(std::string(name));

    itsMean.push_back(std::vector<double>(itsNumFeatures,0));
    itsStdevSq.push_back(std::vector<double>(itsNumFeatures,0.01));
    itsClassFreq.push_back(1);
    return itsNumClasses++;
  }

  return -1;

}

// ######################################################################
const char* Bayes::getClassName(const uint id)
{
  ASSERT(id < itsNumClasses);
  return itsClassNames[id].c_str();

}

// ######################################################################
int Bayes::getClassId(const char *name)
{
  //TODO: should use hash_map (but no hash_map on this machine :( )

  for(uint i=0; i<itsClassNames.size(); i++){
    if (!strcmp(itsClassNames[i].c_str(), name))
      return i;
  }

  return -1;  //no class found but that name

}

// ######################################################################
double Bayes::getMaxProb() const
{ return itsMaxProb; }

// ######################################################################
double Bayes::getNormProb() const
{ return itsNormProb; }

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
