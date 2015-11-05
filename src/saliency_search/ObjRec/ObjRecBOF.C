/*!@file ObjRec/ObjRecBOF.C  ObjRec using bag of features */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ObjRec/ObjRecBOF.C $
// $Id: ObjRecBOF.C 13716 2010-07-28 22:07:03Z itti $
//

#include "ObjRec/ObjRecBOF.H"
#include "Component/OptionManager.H"
#include "Image/ColorOps.H"
#include "GUI/DebugWin.H"
#include "Image/DrawOps.H"
#include "Image/FilterOps.H"
#include "Image/ImageSet.H"
#include "Image/Kernels.H"
#include "Image/MathOps.H"
#include <dirent.h>



// ######################################################################
ObjRecBOF::ObjRecBOF(OptionManager& mgr, const std::string& descrName,
    const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsNumOriArray(36)
{

}

void ObjRecBOF::start2()
{


}

ObjRecBOF::~ObjRecBOF()
{
}


void ObjRecBOF::train(const Image<PixRGB<byte> > &img, const std::string label)
{

}

void ObjRecBOF::train(const std::string &name, int cls)
{

  //getSaliencyKeypoints(name.c_str());
  //getSIFTKeypoints(name.c_str());
  getIlabSIFTKeypoints(name.c_str());

}

void ObjRecBOF::getSaliencyKeypoints(const std::string &name)
{

  char filename[255];
  const char* dirname = "/lab/tmpib/u/objRec/bof/salBayes/fv";
  snprintf(filename, sizeof(filename), "%s/%s.jpg.dat", dirname, name.c_str());
  Object obj;
  obj.name = name;
  obj.keypoints = readSaliencyKeypoints(filename); //read the keypoints
  if (obj.keypoints.size() > 0)
    itsObjects.push_back(obj);
}

void ObjRecBOF::getSIFTKeypoints(const std::string &name)
{

  char filename[255];
  const char* dirname = "/lab/tmpib/u/objRec/bof/siftDemoV4/voc/keyp";
  snprintf(filename, sizeof(filename), "%s/%s.jpg.dat", dirname, name.c_str());
  Object obj;
  obj.name = name;
  obj.keypoints = readSIFTKeypoints(filename); //read the keypoints
  if (obj.keypoints.size() > 0)
    itsObjects.push_back(obj);
}

void ObjRecBOF::getIlabSIFTKeypoints(const std::string &name)
{

  char filename[255];
  const char* dirname = "/home/elazary/images/VOCdevkit/VOC2007/ilabSift";
  snprintf(filename, sizeof(filename), "%s/%s.jpg.dat", dirname, name.c_str());
  Object obj;
  obj.name = name;
  obj.keypoints = readIlabSIFTKeypoints(filename); //read the keypoints
  if (obj.keypoints.size() > 0)
    itsObjects.push_back(obj);
}

void ObjRecBOF::finalizeTraining()
{
  getCodeWords(200);
  printCodeWords();

}

void ObjRecBOF::finalizeTesting()
{
  assignCodeWords();
  printAssignedCodeWords();
}

void ObjRecBOF::getObjCodeWords(const std::string &name)
{
  char filename[255];
  const char* dirname = "/home/elazary/images/VOCdevkit/VOC2007/ilabSift";
  snprintf(filename, sizeof(filename), "%s/%s.jpg.dat", dirname, name.c_str());
  Object obj;
  obj.name = name;
  obj.keypoints = readIlabSIFTKeypoints(filename); //read the keypoints
  if (obj.keypoints.size() > 0)
  {
    for(uint kp=0; kp<obj.keypoints.size(); kp++)
    {
      int codeWord = assignCodeWord(obj.keypoints[kp].fv);
      obj.keypoints[kp].codeWord = codeWord;
    }
  }

  snprintf(filename, sizeof(filename), "%s.dat", obj.name.c_str());
  FILE* fp = fopen(filename, "w");
  if (!fp)
    LFATAL("Error writing %s", obj.name.c_str());

  for(uint kp=0; kp<obj.keypoints.size(); kp++)
  {
    fprintf(fp, "%f %f %i ",
        obj.keypoints[kp].x,
        obj.keypoints[kp].y,
        obj.keypoints[kp].codeWord);
    //for(uint i=0; i<itsObjects[obj].keypoints[kp].fv.size(); i++)
    //  fprintf(fp, "%f ", itsObjects[obj].keypoints[kp].fv[i]);
    fprintf(fp, "\n");
  }
  fclose(fp);

}

Image<float> ObjRecBOF::extractFeatures(const Image<PixRGB<byte> > &img)
{
  extractSIFTFeatures(img);
  //extractGaborFeatures(img);

  return Image<float>();
}

void ObjRecBOF::extractGaborFeatures(const Image<PixRGB<byte> > &img)
{

  int itsNumOri = 8;
  int itsNumScales = 2;

  float stdmin = 1.75F;
  float stdstep = 0.5F;
  int fsmin = 3;
  int fsstep = 1;

  ImageSet<float> itsFilters;
  //create the filters
  for(int scale = 0; scale < itsNumScales; scale++)
    for(int ori = 0; ori < itsNumOri; ori++)
    {

      Image<float> filter = dogFilter<float>(stdmin + stdstep * scale,
          (float)ori * 180.0F / (float)itsNumOri,
          fsmin + fsstep * scale);

      //  fsmin + fsstep * scale);
      // normalize to zero mean:
      filter -= mean(filter);

      // normalize to unit sum-of-squares:
     // filter /= sum(squared(filter));

      itsFilters.push_back(filter);
    }


  double normSum = 0; //the numalization constant
  Image<float> input = luminance(img); //convert to gray
  //Compute the various low level features
  ImageSet<float> featuresValues(itsFilters.size());
  for(uint i=0; i<itsFilters.size(); i++)
  {
    Image<float> tmp = convolve(input, itsFilters[i], CONV_BOUNDARY_CLEAN); //No normalization
    //Image<float> tmp = convolveHmax(input, itsFilters[i]); // normalize by image energy
    tmp = abs(tmp);
    normSum += sum(tmp);
    featuresValues[i] = tmp;
  }


  for(uint feature=0; feature<featuresValues.size(); feature++) //for each feature m
  {
    Image<float> featureVal = featuresValues[feature];

    std::vector<Histogram> levelHists;
    for(int level = 0; level < 4; level++)
    {
      int levelSize = 1<<level;
      Histogram hist(levelSize*levelSize);

      int xSpace = (featureVal.getWidth()/levelSize) + 1;
      int ySpace = (featureVal.getHeight()/levelSize) + 1;

      for(int y=0; y<featureVal.getHeight(); y++)
        for(int x=0; x<featureVal.getWidth(); x++)
        {
          int binPos = (int)(x/xSpace + levelSize*(y/ySpace));
          hist.addValue(binPos, featureVal.getVal(x,y));
        }
      hist.normalize(normSum); //normalize across the sum of all feature outputs
      levelHists.push_back(hist);
    }

    //print the histograms
    printf("%i %i ",feature, (int)levelHists.size());
    for(uint h=0; h<levelHists.size(); h++)
    {
      Histogram hist = levelHists[h];
      for(uint i=0; i<hist.size(); i++)
        printf("%f ", hist[i]);
    }
    printf("\n");

  }



}

void ObjRecBOF::extractSIFTFeatures(const Image<PixRGB<byte> > &img)
{

  Image<float> input = luminance(img); //convert to gray
  LINFO("Get sift vector");
  for(int y=10; y<input.getHeight()-10; y+=10)
    for(int x=10; x<input.getWidth()-10; x+=10)
    {
      std::vector<std::vector<byte> >  sd = getSiftDescriptor(input, x,y,2);
      for(uint j=0; j<sd.size(); j++)
      {
        printf("%i %i ", x, y);
        for(uint i=0; i<sd[j].size(); i++)
        {
          printf("%i ", sd[j][i]);
        }
        printf("\n");
      }
    }

}


void ObjRecBOF::extractCodeWords(const char* dirname)
{

  Image<float> debugImg(256,256, ZEROS);
  readSaliencyFeatures(dirname);

  //
  for(uint obj=0; obj<itsObjects.size(); obj++)
    for(uint kp=0; kp<itsObjects[obj].keypoints.size(); kp++)
    {
      float x = itsObjects[obj].keypoints[kp].fv[0];
      float y = itsObjects[obj].keypoints[kp].fv[1];
      //float val = debugImg.getVal((int)x,(int)y);
      debugImg.setVal((int)x,(int)y,128.0);
    }

  //printFeatures();
  getCodeWords(200);
  //readCodeWords("/lab/tmpib/u/objRec/salBayes/fv/codeWords.dat");
  //printCodeWords();
  for(uint j=0; j<itsCodeWords.size(); j++)
  {
      float x = itsCodeWords[j][0];
      float y = itsCodeWords[j][1];
      //float val = debugImg.getVal((int)x,(int)y);
      drawCircle(debugImg, Point2D<int>((int)x,(int)y), 3, 255.0F);
  }
  SHOWIMG(debugImg);

  //assignCodeWords();
  //printAssignedCodeWords();

}

void ObjRecBOF::printFeatures()
{

  for (uint i=0; i<itsObjects[0].keypoints.size(); i++)
  {
    printf("%i %f %f ", i,
        itsObjects[0].keypoints[i].x,
        itsObjects[0].keypoints[i].y);

    std::vector<double> fv = itsObjects[0].keypoints[i].fv;
    for (uint j=0; j<fv.size(); j++)
      printf("%f ", fv[j]);
    printf("\n");
  }

}

void ObjRecBOF::printCodeWords()
{

  for(uint j=0; j<itsCodeWords.size(); j++)
  {
    for(uint i=0; i<itsCodeWords[j].size(); i++)
      printf("%f ", itsCodeWords[j][i]);
    printf("\n");
  }
}

void ObjRecBOF::printAssignedCodeWords()
{
  for(uint obj=0; obj<itsObjects.size(); obj++)
  {
    char filename[255];
    snprintf(filename, sizeof(filename), "%s.dat", itsObjects[obj].name.c_str());
    FILE* fp = fopen(filename, "w");
    if (!fp)
      LFATAL("Error writing %s", itsObjects[obj].name.c_str());

    for(uint kp=0; kp<itsObjects[obj].keypoints.size(); kp++)
    {
      fprintf(fp, "%f %f %i ",
          itsObjects[obj].keypoints[kp].x,
          itsObjects[obj].keypoints[kp].y,
          itsObjects[obj].keypoints[kp].codeWord);
      //for(uint i=0; i<itsObjects[obj].keypoints[kp].fv.size(); i++)
      //  fprintf(fp, "%f ", itsObjects[obj].keypoints[kp].fv[i]);
      fprintf(fp, "\n");
    }
    fclose(fp);
  }
}

void ObjRecBOF::assignCodeWords()
{
  for(uint obj=0; obj<itsObjects.size(); obj++)
    for(uint kp=0; kp<itsObjects[obj].keypoints.size(); kp++)
    {
      int codeWord = assignCodeWord(itsObjects[obj].keypoints[kp].fv);
      itsObjects[obj].keypoints[kp].codeWord = codeWord;
    }
}

int ObjRecBOF::assignCodeWord(const std::vector<double> &fv)
{
  std::vector<double>::const_iterator s = fv.begin();
  int k_best = 0;
  double min_dist = std::numeric_limits<double>::max();

  //find the center with the lowest distance to this data vector
  for(uint k=0; k<itsCodeWords.size(); k++)
  {
    std::vector<double>::iterator c = itsCodeWords[k].begin(); //the mean vector for this label
    double dist = 0;

    uint j = 0;
    for(; j<= itsCodeWords[k].size() - 4; j += 4)
    {
      double t0 = c[j] - s[j];
      double t1 = c[j+1] - s[j+1];
      dist += t0*t0 + t1*t1;

      t0 = c[j+2] - s[j+2];
      t1 = c[j+3] - s[j+3];
      dist += t0*t0 + t1*t1;
    }

    for( ; j < itsCodeWords[k].size(); j++ )
    {
      double t = c[j] - s[j];
      dist += t*t;
    }

    if (min_dist > dist)
    {
      min_dist = dist;
      k_best = k;
    }
  }

  return k_best;
}

void ObjRecBOF::readCodeWords(const char* filename)
{
  LINFO("Reading %s", filename);
  FILE *fp;
  fp = fopen(filename, "r");
  if (!fp)
    LFATAL("Error reading %s\n", filename);

  int nCodeWords, nDim;

  if (fscanf(fp, "%d %d", &nCodeWords, &nDim) != 2)
    LFATAL("Invalid codeword file beginning.");

  itsCodeWords.clear();
  itsCodeWords.resize(nCodeWords);
  for (int i = 0; i < nCodeWords; i++) {

    std::vector<double> codeWord(nDim);

    for (int j = 0; j < nDim; j++) {
      float val;
      if (fscanf(fp, "%f", &val) != 1)
        LFATAL("Invalid code word.");
      codeWord[j] = val;
    }
    itsCodeWords[i] = codeWord;
  }

  fclose(fp);
}


void ObjRecBOF::getCodeWords(int numCodeWords)
{

  initRandomNumbers();

  int numOfKeypoints = 0;
  int nDims = 0;
  //find the number of data and the dimentions
  for(uint i=0; i<itsObjects.size(); i++)
  {
    numOfKeypoints += itsObjects[i].keypoints.size();
    if (itsObjects[i].keypoints.size() > 0)
      nDims = itsObjects[i].keypoints[0].fv.size();
  }
  ASSERT(numOfKeypoints != 0 && nDims != 0);


  std::vector<std::vector<double> > centers(numCodeWords, std::vector<double>(nDims));
  std::vector<std::vector<double> > oldCenters(numCodeWords, std::vector<double>(nDims));
  std::vector<int> counters(numCodeWords);

  std::vector<int> labels(numOfKeypoints);

  //initalize labels randomly
  for(uint i=0; i<labels.size(); i++)
    labels[i] = randomUpToNotIncluding(numCodeWords);

  //iterate to find clusters centers
  int maxIter = 10;
  double epsilon = 1.0;
  double max_dist = epsilon*2;

  for(int iter = 0; iter < maxIter; iter++)
  {
    //zero centers
    for(uint j=0; j<centers.size(); j++)
      for(uint i=0; i<centers[j].size(); i++)
         centers[j][i] = 0;

    //zero counters
    for(uint i=0; i<counters.size(); i++)
      counters[i] = 0;

    //compute centers from each sample
    uint idx = 0;
    for(uint obj=0; obj<itsObjects.size(); obj++)
    {
      uint numKp = itsObjects[obj].keypoints.size();
      for (uint kp=0; kp<numKp; kp++)
      {
        std::vector<double>::iterator s = itsObjects[obj].keypoints[kp].fv.begin(); //the data vector
        uint k = labels[idx];  //the label assigned to this vector
        std::vector<double>::iterator c = centers[k].begin(); //the mean vector for this label

        int j;
        for(j=0; j<= (int)centers[k].size() - 4; j += 4)
        {
          double t0 = c[j] + s[j];
          double t1 = c[j+1] + s[j+1];

          c[j] = t0;
          c[j+1] = t1;

          t0 = c[j+2] + s[j+2];
          t1 = c[j+3] + s[j+3];

          c[j+2] = t0;
          c[j+3] = t1;
        }
        for( ; j < (int)centers[k].size(); j++ )
          c[j] += s[j];

        counters[k]++; //increment the number of datapoints in this class
        idx++;
      }
    }

    if (iter > 0)
      max_dist = 0;

    for(uint k=0; k<centers.size(); k++)
    {
      std::vector<double>::iterator c = centers[k].begin(); //the mean vector for this label
      if (counters[k] != 0)  //if we have some members in the class
      {
        //scale the mean vector with the number of its members
        double scale = 1./counters[k];
        for(uint j = 0; j < centers[k].size(); j++ )
          c[j] *= scale;
      }
      else
      {
        //random initalization
        //pick a keypoint at random from any object
        int obj = randomUpToNotIncluding(itsObjects.size());
        int kp = randomUpToNotIncluding(itsObjects[obj].keypoints.size());

        //assign the cluster center from this keypoint
        for(uint j=0; j<centers[k].size(); j++)
          c[j] = itsObjects[obj].keypoints[kp].fv[j];
      }

      if (iter > 0)
      {
        //find the distance betwwen the old center hypothisis and the new one
        double dist = 0;
        std::vector<double>::iterator c_o = oldCenters[k].begin();
        for(uint j = 0; j < centers[k].size(); j++ )
        {
          double t = c[j] - c_o[j];
          dist += t*t;
        }
        if( max_dist < dist )
          max_dist = dist;
      }
    }

    //assign the labels to the new clusters
    idx = 0;
    for(uint obj=0; obj<itsObjects.size(); obj++)
    {
      uint numKp = itsObjects[obj].keypoints.size();
      for (uint kp=0; kp<numKp; kp++)
      {
        std::vector<double>::iterator s = itsObjects[obj].keypoints[kp].fv.begin(); //the data vector
        int k_best = 0;
        double min_dist = std::numeric_limits<double>::max();

        //find the center with the lowest distance to this data vector
        for(uint k=0; k<centers.size(); k++)
        {
          std::vector<double>::iterator c = centers[k].begin(); //the mean vector for this label
          double dist = 0;

          int j = 0;
          for(; j<= (int)centers[k].size() - 4; j += 4)
          {
            double t0 = c[j] - s[j];
            double t1 = c[j+1] - s[j+1];
            dist += t0*t0 + t1*t1;

            t0 = c[j+2] - s[j+2];
            t1 = c[j+3] - s[j+3];
            dist += t0*t0 + t1*t1;
          }

          for( ; j < (int)centers[k].size(); j++ )
          {
            double t = c[j] - s[j];
            dist += t*t;
          }

          if (min_dist > dist)
          {
            min_dist = dist;
            k_best = k;
          }
        }
        labels[idx] = k_best;
        idx++;
      }
    }

    if (max_dist < epsilon)
      break;

    //if we did not terminate, then set the swap old centers to the new centers
    for(uint j=0; j<centers.size(); j++)
    {
      for(uint i=0; i<centers[j].size(); i++)
      {
        double tmp = centers[j][i];
        centers[j][i] = oldCenters[j][i];
        oldCenters[j][i] = tmp;
      }
    }

  }

  //ensure that we do not have empty clusters
  for(uint i=0; i<counters.size(); i++)
    counters[i] = 0;

  uint idx = 0;
  for(uint obj=0; obj<itsObjects.size(); obj++)
  {
    uint numKp = itsObjects[obj].keypoints.size();
    for (uint kp=0; kp<numKp; kp++)
    {
      counters[labels[idx]]++;
      idx++;
    }
  }


  //for(uint i=0; i<labels.size(); i++)
  //  printf("%i %i\n", i, labels[i]);

  //for(uint i=0; i<counters.size(); i++)
  //{
  //  if (counters[i] == 0)
  //    LINFO("Counter is zero");
  //}

  itsCodeWords = centers;

}


/*
void ObjRecBOF::getCodeWords(int numCodeWords)
{
  int numOfKeypoints = 0;
  int nDims = 0;

  //find the number of data and the dimentions
  for(uint i=0; i<itsObjects.size(); i++)
  {
    numOfKeypoints += itsObjects[i].keypoints.size();
    if (itsObjects[i].keypoints.size() > 0)
      nDims = itsObjects[i].keypoints[0].fv.size();
  }


  //insert values into the data tables
  CvMat *data = cvCreateMat(numOfKeypoints, nDims, CV_32FC1);
  CvMat *labels = cvCreateMat(numOfKeypoints, 1, CV_32SC1);

  cvZero(labels);
  int idx=0;
  for(uint obj=0; obj<itsObjects.size(); obj++)
  {
    for (uint i=0; i<itsObjects[obj].keypoints.size(); i++)
    {
      std::vector<double> fv = itsObjects[obj].keypoints[i].fv;
      for (uint j=0; j<fv.size(); j++)
      {
        cvmSet(data, idx, j, fv[j]);
        //cvSetReal2D(data, idx, j, fv[j]);
      }
      idx++;
    }
  }

  //run kmeans
  cvKMeans2(data, numCodeWords, labels,
      cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0 ));

  for(int i=0; i<numOfKeypoints; i++)
  {
    printf("%i %i\n", i, labels->data.i[i]);
  }
  //get the clusters means
  CvMat *clusterMeans = cvCreateMat(numCodeWords, nDims, CV_32FC1);
  CvMat *nDataCount = cvCreateMat(numCodeWords, 1, CV_32FC1);
  cvZero(clusterMeans);
  cvZero(nDataCount);

  for(int i=0; i<numOfKeypoints; i++)
  {
    for(int j=0; j<nDims; j++)
    {
      double val = cvGetReal2D(clusterMeans, labels->data.i[i], j);
      double val2 = cvGetReal2D(data, i, j);
      cvSetReal2D(clusterMeans, labels->data.i[i], j, val+val2);

      // printf("%0.1f %0.1f %0.1f\n", val, val2, count);

    }
    double count = cvmGet(nDataCount, labels->data.i[i], 0);
    cvmSet(nDataCount, labels->data.i[i], 0, count+1);
  }

  //inset the cluster means into the codeWords
  itsCodeWords.resize(numCodeWords);
  for(int i=0; i<numCodeWords; i++)
  {
    double count = cvmGet(nDataCount, i, 0);
    std::vector<double> codes;
    codes.resize(nDims);
    for(int j=0; j<nDims; j++)
    {
      double sum = cvmGet(clusterMeans, i, j);
      //cvmSet(clusterMeans, i, j, sum/count);
      codes[j] = sum/count;
    }
    itsCodeWords[i] = codes;
  }


  cvReleaseMat(&data);
  cvReleaseMat(&labels);
  cvReleaseMat(&clusterMeans);
  cvReleaseMat(&nDataCount);
}
*/

void ObjRecBOF::readSaliencyFeatures(const char* dirname)
{
  DIR *dp = opendir(dirname);
  if (dp == NULL)
    LFATAL("Can not open %s", dirname);
  dirent *dirp;
  while ((dirp = readdir(dp)) != NULL ) {
    std::string dirName(dirp->d_name);
    if (dirName.find(".dat") != std::string::npos)
    {
      char filename[255];
      snprintf(filename, sizeof(filename), "%s/%s", dirname, dirp->d_name);
      Object obj;
      obj.name = std::string(dirp->d_name);
      obj.keypoints = readSaliencyKeypoints(filename); //read the keypoints
      itsObjects.push_back(obj);
    }
  }
  closedir(dp);
}

std::vector<ObjRecBOF::Keypoint> ObjRecBOF::readSaliencyKeypoints(const char *filename)
{

  std::vector<Keypoint> keypoints;
  LINFO("Reading %s", filename);
  FILE *fp;
  fp = fopen(filename, "r");
  if (!fp)
  {
    LINFO("Error reading %s", filename);
    return keypoints;
  }

  int len = 42;
  while (fp != NULL)
  {
    Keypoint key;
    int keyNum, x, y;
    if (fscanf(fp, "%i %i %i", &keyNum, &x, &y) != 3) break;
    key.x = (double)x;
    key.y = (double)y;
    key.codeWord = -1;

    for (int j = 0; j < len; j++) {
      float val = -1;
      if (fscanf(fp, "%f", &val) != 1)
        perror("Invalid keypoint file value.");
      key.fv.push_back(val);
    }
    keypoints.push_back(key);
  }

  fclose(fp);
  return keypoints;
}

std::vector<ObjRecBOF::Keypoint> ObjRecBOF::readSIFTKeypoints(const char *filename)
{

  std::vector<Keypoint> keypoints;
  printf("Reading %s\n", filename);
  FILE *fp;
  fp = fopen(filename, "r");
  if (!fp)
  {
    LINFO("Error reading %s", filename);
    return keypoints;
  }

  int i, j, num, len, val;

  if (fscanf(fp, "%d %d", &num, &len) != 2)
    perror("Invalid keypoint file beginning.");

  if (len != 128)
    perror("Keypoint descriptor length invalid (should be 128).");
  for (i = 0; i < num; i++) {

    Keypoint key;
    float x, y, scale, ori;
    if (fscanf(fp, "%f %f %f %f", &x, &y, &scale, &ori) != 4) break;
    key.x = x;
    key.y = y;
    key.scale = scale;
    key.ori = ori;

    for (j = 0; j < len; j++) {
      if (fscanf(fp, "%d", &val) != 1 || val < 0 || val > 255)
        perror("Invalid keypoint file value.");
      key.fv.push_back((double)val);
    }
    keypoints.push_back(key);
  }

  fclose(fp);

  return keypoints;
}

std::vector<ObjRecBOF::Keypoint> ObjRecBOF::readIlabSIFTKeypoints(const char *filename)
{

  std::vector<Keypoint> keypoints;
  printf("Reading %s\n", filename);
  FILE *fp;
  fp = fopen(filename, "r");
  if (!fp)
  {
    LINFO("Error reading %s", filename);
    return keypoints;
  }

  int len = 128;

  while (fp != NULL)
  {
    Keypoint key;
    int x, y;
    if (fscanf(fp, "%i %i", &x, &y) != 2) break;
    key.x = x;
    key.y = y;

    for (int j = 0; j < len; j++) {
      int val;
      if (fscanf(fp, "%d", &val) != 1 || val < 0 || val > 255)
        perror("Invalid keypoint file value.");
      key.fv.push_back((double)val);
    }
    keypoints.push_back(key);
  }

  fclose(fp);

  return keypoints;
}

std::vector<std::vector<byte> >  ObjRecBOF::getSiftDescriptor(const Image<float> &lum,
    const float x, const float y, const float s)
{

  Image<float> mag, ori;

  //gradientSobel(lum, mag, ori, 3);
  gradient(lum, mag, ori);

  Histogram OV(itsNumOriArray);

  // 1. Calculate main orientation vector
  calculateOrientationVector(x, y, s, mag, ori, OV);

  //Image<byte> histImg = OV.getHistogramImage(256, 256, 0, -1);

  // 2. Create feature vector and keypoint for each significant
  // orientation peak:
  return createVectorsAndKeypoints(x, y, s, mag, ori, OV);

}

void ObjRecBOF::calculateOrientationVector(const float x, const float y, const float s,
                const Image<float>& gradmag, const Image<float>& gradorie, Histogram& OV) {


        // compute the effective blurring sigma corresponding to the
        // fractional scale s:
        const float sigma = s;

        const float sig = 1.5F * sigma, inv2sig2 = - 0.5F / (sig * sig);
        const int dimX = gradmag.getWidth(), dimY = gradmag.getHeight();

        const int xi = int(x + 0.5f);
        const int yi = int(y + 0.5f);

        const int rad = int(3.0F * sig);
        const int rad2 = rad * rad;


        // search bounds:
        int starty = yi - rad; if (starty < 0) starty = 0;
        int stopy = yi + rad; if (stopy >= dimY) stopy = dimY-1;

        // 1. Calculate orientation vector
        for (int ind_y = starty; ind_y <= stopy; ind_y ++)
        {
                // given that y, get the corresponding range of x values that
                // lie within the disk (and remain within the image):
                const int yoff = ind_y - yi;
                const int bound = int(sqrtf(float(rad2 - yoff*yoff)) + 0.5F);
                int startx = xi - bound; if (startx < 0) startx = 0;
                int stopx = xi + bound; if (stopx >= dimX) stopx = dimX-1;

                for (int ind_x = startx; ind_x <= stopx; ind_x ++)
                {
                        const float dx = float(ind_x) - x, dy = float(ind_y) - y;
                        const float distSq = dx * dx + dy * dy;

                        // get gradient:
                        const float gradVal = gradmag.getVal(ind_x, ind_y);

                        // compute the gaussian weight for this histogram entry:
                        const float gaussianWeight = expf(distSq * inv2sig2);

                        // add this orientation to the histogram
                        // [-pi ; pi] -> [0 ; 2pi]
                        float angle = gradorie.getVal(ind_x, ind_y) + M_PI;

                        // [0 ; 2pi] -> [0 ; 36]
                        angle = 0.5F * angle * itsNumOriArray / M_PI;
                        while (angle < 0.0F) angle += itsNumOriArray;
                        while (angle >= itsNumOriArray) angle -= itsNumOriArray;

                        OV.addValueInterp(angle, gaussianWeight * gradVal);
                }
        }


        // smooth the orientation histogram 3 times:
        for (int i = 0; i < 3; i++) OV.smooth();
}

// ######################################################################


std::vector<std::vector<byte> > ObjRecBOF::createVectorsAndKeypoints(const float x,
    const float y, const float s,
    const Image<float>& gradmag, const Image<float>& gradorie, Histogram& OV)
{

  const float sigma = s; //itsSigma * powf(2.0F, s / float(itsLumBlur.size() - 3));

  // find the max in the histogram:
  float maxPeakValue = OV.findMax();

  const int xi = int(x + 0.5f);
  const int yi = int(y + 0.5f);

  uint numkp = 0;

  std::vector<std::vector<byte> > descriptor;

  // 2. Create feature vector and keypoint for each significant
  // orientation peak:
  for (int bin = 0; bin < itsNumOriArray; bin++)
  {
    // consider the peak centered around 'bin':
    const float midval = OV.getValue(bin);

    // if current value much smaller than global peak or = to zero, forget it:
    if (midval == 0 || midval < 0.8F * maxPeakValue) continue;

    // get value to the left of current value
    const float leftval = OV.getValue((bin == 0) ? itsNumOriArray-1 : bin-1);

    // get value to the right of current value
    const float rightval = OV.getValue((bin == itsNumOriArray-1) ? 0 : bin+1);

    // only consider local peaks:
    if (leftval > midval) continue;
    if (rightval > midval) continue;

    // interpolate the values to get the orientation of the peak:
    //  with f(x) = ax^2 + bx + c
    //   f(-1) = x0 = leftval
    //   f( 0) = x1 = midval
    //   f(+1) = x2 = rightval
    //  => a = (x0+x2)/2 - x1
    //     b = (x2-x0)/2
    //     c = x1
    // f'(x) = 0 => x = -b/2a
    const float a  = 0.5f * (leftval + rightval) - midval;
    const float b  = 0.5f * (rightval - leftval);
    float realangle = float(bin) - 0.5F * b / a;

    realangle *= 2.0F * M_PI / itsNumOriArray; // [0:36] to [0:2pi]
    realangle -= M_PI;                      // [0:2pi] to [-pi:pi]

    // ############ Create keypoint:

    // compute the feature vector:
    FeatureVector fv;

    const float sinAngle = sin(realangle), cosAngle = cos(realangle);

    // check this scale
    const int radius = int(5.0F * sigma + 0.5F); // NOTE: Lowe uses radius=8?
    const float gausssig = float(radius); // 1/2 width of descript window
    const float gaussfac = - 0.5F / (gausssig * gausssig);


    // Scan a window of diameter 2*radius+1 around the point of
    // interest, and we will cumulate local samples into a 4x4 grid
    // of bins, with interpolation. NOTE: rx and ry loop over a
    // square that is assumed centered around the point of interest
    // and rotated to the gradient orientation (realangle):

    int scale = abs(int(s));
    scale = scale > 5 ? 5 : scale;

    for (int ry = -radius; ry <= radius; ry++)
      for (int rx = -radius; rx <= radius; rx++)
      {
        // rotate the point:
        const float newX = rx * cosAngle - ry * sinAngle;
        const float newY = rx * sinAngle + ry * cosAngle;

        // get the coords in the image frame of reference:
        const float orgX = newX + float(xi);
        const float orgY = newY + float(yi);

        // if outside the image, forget it:
        if (gradmag.coordsOk(orgX, orgY) == false) continue;

        // find the fractional coords of the corresponding bin
        // (we subdivide our window into a 4x4 grid of bins):
        const float xf = 2.0F + 2.0F * float(rx) / float(radius);
        const float yf = 2.0F + 2.0F * float(ry) / float(radius);


        // find the Gaussian weight from distance to center and
        // get weighted gradient magnitude:
        const float gaussFactor = expf((newX*newX+newY*newY) * gaussfac);
        const float weightedMagnitude =
          gaussFactor * gradmag.getValInterp(orgX, orgY);

        // get the gradient orientation relative to the keypoint
        // orientation and scale it for 8 orientation bins:
        float gradAng = gradorie.getValInterp(orgX, orgY) - realangle;

        gradAng=fmod(gradAng, 2*M_PI); //bring the range from 0 to M_PI

        //convert from -M_PI to M_PI
        if (gradAng < 0.0) gradAng+=2*M_PI; //convert to -M_PI to M_PI
        if (gradAng >= M_PI) gradAng-=2*M_PI;
        //split to eight bins
        const float orient = (gradAng + M_PI) * 8 / (2 * M_PI);

        /*
        //reflect the angle to convert from 0 to M_PI
        if (gradAng >= M_PI) gradAng-=M_PI;
        //split to four bins
        const float orient = (gradAng + M_PI) * 4 / (2 * M_PI);
         */

        // will be interpolated into 2 x 2 x 2 bins:

        fv.addValue(xf, yf, orient, weightedMagnitude);

      }

    // normalize, clamp, scale and convert to byte:
    std::vector<byte> oriVec;

    fv.toByteKey(oriVec);

    double mag = fv.getMag();
    if (oriVec.size() > 0 && mag > 0)
      descriptor.push_back(oriVec);
    //The key point

    ++ numkp;

  }
  return descriptor;
}


std::string ObjRecBOF::test(const Image<PixRGB<byte> > &img)
{


  return std::string("Test");
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
