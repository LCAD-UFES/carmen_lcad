/*!@file Learn/SVMClassifier.C Support Vector Machine Classifier module */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Learn/SVMClassifier.C $
// $Id: SVMClassifier.C 14581 2011-03-08 07:18:09Z dparks $
//

#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <cstdlib>
#include <map>

#include "svm.h"
#include "SVMClassifier.H"
#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Component/OptionManager.H"


SVMClassifier::SVMClassifier(float gamma, int C)
{
  itsSVMModel = NULL;

  // Default parameters
  itsSVMParams.svm_type = C_SVC;
  itsSVMParams.kernel_type = RBF;
  itsSVMParams.degree = 3;
  itsSVMParams.gamma = gamma;        // 1/k
  itsSVMParams.coef0 = 0;
  itsSVMParams.nu = 0.5;
  itsSVMParams.cache_size = 100;
  itsSVMParams.C = C;
  itsSVMParams.eps = 1e-3;
  itsSVMParams.p = 0.1;
  itsSVMParams.shrinking = 1;
  itsSVMParams.probability = 1;
  itsSVMParams.nr_weight = 0;
  itsSVMParams.weight_label = NULL;
  itsSVMParams.weight = NULL;
  itsSVMRangeEnabled = false;
}

SVMClassifier::~SVMClassifier()
{
}


void SVMClassifier::readModel(std::string modelFileName)
{
  itsSVMModel = svm_load_model(modelFileName.c_str());
  if(itsSVMModel == NULL)
    LFATAL("Model file undefined (%s)", modelFileName.c_str());
}

void SVMClassifier::readRange(std::string rangeFileName)
{
  itsSVMRangeEnabled = true;
  FILE *frange;
  /* frange rewinded in finding max_index */
  int idx, c;
  double fmin, fmax;
  // Size to request memory in
  int block_size = 1024;
  int cur_size = block_size;
  // Current largest index
  int largest_index = 0;
  itsSVMFeatureRangeMax.clear();
  itsSVMFeatureRangeMin.clear();
  itsSVMFeatureRangeMax.resize(cur_size);
  itsSVMFeatureRangeMin.resize(cur_size);
  frange = fopen(rangeFileName.c_str(),"r");
  if(frange == NULL)
    {
      LFATAL("Unable to open SVM range file");
    }
  if((c = fgetc(frange)) == 'y')
    {
      LFATAL("Y Scaling is not implemented");
    }
  else
    ungetc(c, frange);

  if (fgetc(frange) == 'x') {
    if(fscanf(frange, "%lf %lf\n", &itsSVMFeatureRangeLower, &itsSVMFeatureRangeUpper) != 2) LFATAL("Failed to load from: %s", rangeFileName.c_str() );
    //printf("%f:%f\n",itsSVMFeatureRangeLower,itsSVMFeatureRangeUpper);
    while(fscanf(frange,"%d %lf %lf\n",&idx,&fmin,&fmax)==3)
      {
        //printf("%d L:%f\tH:%f....",idx,fmin,fmax);
        if(largest_index < idx) largest_index = idx;
        if(idx >= cur_size-1)
          {
            while(idx >= cur_size-1)
              cur_size += block_size;
            itsSVMFeatureRangeMax.resize(cur_size);
            itsSVMFeatureRangeMin.resize(cur_size);
          }
        itsSVMFeatureRangeMin[idx] = fmin;
        itsSVMFeatureRangeMax[idx] = fmax;
      }
  }
  itsSVMFeatureRangeMin.resize(largest_index+1);
  itsSVMFeatureRangeMax.resize(largest_index+1);
  fclose(frange);
  // for(int i=1;i<itsSVMFeatureRangeMin.size();i++)
  //   {
  //     printf("%d L:%f\tH:%f\n",i,itsSVMFeatureRangeMin[i],itsSVMFeatureRangeMax[i]);
  //   }

}



void SVMClassifier::writeModel(std::string modelFileName)
{
  svm_save_model(modelFileName.c_str(),itsSVMModel);
}

float SVMClassifier::rescaleValue(float value, unsigned int index)
{
  if(itsSVMRangeEnabled)
    {
      if(itsSVMFeatureRangeMax[index] == itsSVMFeatureRangeMin[index])
        value = itsSVMFeatureRangeMax[index];
      if(value == itsSVMFeatureRangeMin[index])
        value = itsSVMFeatureRangeLower;
      else if(value == itsSVMFeatureRangeMax[index])
        value = itsSVMFeatureRangeUpper;
      else
        value = itsSVMFeatureRangeLower + (itsSVMFeatureRangeUpper-itsSVMFeatureRangeLower) *
          (value-itsSVMFeatureRangeMin[index])/
          (itsSVMFeatureRangeMax[index]-itsSVMFeatureRangeMin[index]);
    }
  return value;
}

double SVMClassifier::predict(std::vector<float> &feature, double *probability)
{
  unsigned int ind=0;
  svm_node *node = new svm_node[feature.size()+1]; // One extra to signal end of list
  for(ind=0;ind<feature.size();ind++){
      node[ind].index = ind+1;
      node[ind].value = rescaleValue(feature[ind],ind+1);
      //printf("%f,%f ",feature[ind],node[ind].value);
  }
  // Set the last to -1 to indicate the end of the list
  node[ind].index = -1;
  node[ind].value = -1;
  int label = _predict(node,feature.size(),probability);
  delete [] node;
  return label;
}

double SVMClassifier::predict(float * &feature, unsigned int fdim, double *probability)
{
  unsigned int ind=0;
  svm_node *node = new svm_node[fdim+1]; // One extra to signal end of list
  for(ind=0;ind<fdim;ind++){
      node[ind].index = ind+1;
      node[ind].value = rescaleValue(feature[ind],ind+1);
      //printf("%f,%f ",feature[ind],node[ind].value);
  }
  // Set the last to -1 to indicate the end of the list
  node[ind].index = -1;
  node[ind].value = -1;
  int label = _predict(node,fdim,probability);
  delete [] node;
  return label;
}

double SVMClassifier::predict(float **&feature, unsigned int fdim1, unsigned int fdim2, double *probability)
{
  unsigned int ind = 0;
  svm_node *node = new svm_node[fdim1*fdim2+1]; // One extra to signal end of list
  for(unsigned int i=0;i<fdim1;i++){
    for(unsigned int j=0;j<fdim2;j++){
      node[ind].index = ind+1;
      node[ind].value = rescaleValue(feature[i][j],ind+1);
      //printf("%f,%f ",feature[i][j],node[ind].value);
      ind++;
    }
  }
  // Set the last to -1 to indicate the end of the list
  node[ind].index = -1;
  node[ind].value = -1;
  int label= _predict(node,fdim1*fdim2,probability);
  delete [] node;
  return label;
}

double SVMClassifier::_predict(struct svm_node *node, unsigned int fdim, double * probability)
{
  std::map<int, double> pdf = predictPDF(node);
  return _getBestLabel(pdf,probability);
}


std::map<int,double> SVMClassifier::predictPDF(std::vector<float> &feature)
{
  unsigned int ind=0;
  svm_node *node = new svm_node[feature.size()+1]; // One extra to signal end of list
  for(ind=0;ind<feature.size();ind++){
      node[ind].index = ind+1;
      node[ind].value = rescaleValue(feature[ind],ind+1);
      //printf("%f,%f ",feature[ind],node[ind].value);
  }
  // Set the last to -1 to indicate the end of the list
  node[ind].index = -1;
  node[ind].value = -1;
  std::map<int,double> pdf= predictPDF(node);
  delete [] node;
  return pdf;
}



std::map<int,double> SVMClassifier::predictPDF(const svm_node* dataPointNodes)
{
  std::map<int,double> pdf;
  int numberOfLabels = svm_get_nr_class(itsSVMModel);
  int *labels = new int[numberOfLabels];
  svm_get_labels(itsSVMModel,labels);
  double *probEst = new double[numberOfLabels];
  svm_predict_probability(itsSVMModel,dataPointNodes,probEst);
  // Need to find the index of the returned label
  for(int i=0;i<numberOfLabels;i++)
    {
      pdf[labels[i]] = probEst[i];
    }
  delete [] probEst;
  delete [] labels;
  return pdf;
}


double SVMClassifier::predict(Image<double> dataPoint, double *probability)
{
  if(!itsSVMModel)
  {
    LERROR("Model not created. Run SVMClassifier::train before running predict");
    return -1;
  }

  //Make sure that the data point is a column vector for now...
  //This can be generalized later
  ASSERT(dataPoint.getWidth() == 1);

  //Construct the svm node to predict
  svm_node dataPointNodes[dataPoint.getHeight()+1];
  for(int dimIdx=0; dimIdx<dataPoint.getHeight(); dimIdx++)
  {
    svm_node tmpNode;
    tmpNode.index = dimIdx+1;
    tmpNode.value = rescaleValue(dataPoint.getVal(0, dimIdx),dimIdx);
    dataPointNodes[dimIdx] = tmpNode;
  }
  svm_node endNode;
  endNode.index = -1;
  endNode.value = -1;
  dataPointNodes[dataPoint.getHeight()] = endNode;

  std::map<int, double> pdf = predictPDF(dataPointNodes);
  return _getBestLabel(pdf,probability);
}

int SVMClassifier::_getBestLabel(std::map<int,double> pdf, double *probability)
{
  double maxProb = -1;
  int bestLabel = 0;
  for(std::map<int, double>::iterator pdfIt = pdf.begin(); pdfIt != pdf.end(); ++pdfIt) 
    {
      int label = pdfIt->first;
      double prob = pdfIt->second;
      if(maxProb < prob)
	{
	  bestLabel = label;
	  maxProb = prob;
	}
    }
  if(probability)
    *probability = maxProb;
  return bestLabel;
}

void SVMClassifier::train(std::string outputFileName, int id, std::vector<float> &feature)
{
  std::ofstream outfile;
  outfile.open(outputFileName.c_str(),std::ios::out | std::ios::app);
  if (outfile.is_open()) {
    outfile << id << " ";
    for(unsigned int i=0;i<feature.size();i++) {
      outfile << std::setiosflags(std::ios::fixed) << std::setprecision(4) <<
	(i+1) << ":" << feature[i] << " ";
    }
    outfile << std::endl;
    outfile.close();
  }
  else {
    LFATAL("Could not open output file");
  }
}


void SVMClassifier::train(std::string outputFileName, int id, float *&feature, unsigned int fdim)
{
  std::ofstream outfile;
  outfile.open(outputFileName.c_str(),std::ios::out | std::ios::app);
  if (outfile.is_open()) {
    outfile << id << " ";
    for(unsigned int i=0;i<fdim;i++) {
      outfile << std::setiosflags(std::ios::fixed) << std::setprecision(4) <<
	(i+1) << ":" << feature[i] << " ";
    }
    outfile << std::endl;
    outfile.close();
  }
  else {
    LFATAL("Could not open output file");
  }
}

void SVMClassifier::train(std::string outputFileName, int id, float **&feature, unsigned int fdim1, unsigned int fdim2)
{
  std::ofstream outfile;
  outfile.open(outputFileName.c_str(),std::ios::out | std::ios::app);
  if (outfile.is_open()) {
    outfile << id << " ";
    for(unsigned int i=0;i<fdim1;i++) {
      for(unsigned int j=0;j<fdim2;j++) {
        outfile << std::setiosflags(std::ios::fixed) << std::setprecision(4) <<
          (i*fdim2+j+1) << ":" << feature[i][j] << " ";
      }
    }
    outfile << std::endl;
    outfile.close();
  }
  else {
    LFATAL("Could not open output file");
  }
}

void SVMClassifier::train(Image<double> trainingData, std::vector<double> dataClasses)
{
  ASSERT((uint)trainingData.getWidth() == dataClasses.size());

  //Setup the svm classifier
  svm_problem trainingProblem;

  //Tell the problem how many data points we have
  trainingProblem.l = dataClasses.size();

  //Copy the data classes into the training problem
  trainingProblem.y = new double[dataClasses.size()];
  std::copy(dataClasses.begin(), dataClasses.end(), trainingProblem.y);

  //Fill in the training data by creating a matrix of svm nodes
  trainingProblem.x = new svm_node*[trainingData.getWidth()];
  for(int ptIdx=0; ptIdx < trainingData.getWidth(); ptIdx++)
  {
    //Allocate the nodes for this data point
    trainingProblem.x[ptIdx] = new svm_node[trainingData.getHeight()+1];
    for(int dimIdx=0; dimIdx<trainingData.getHeight(); dimIdx++)
    {
      svm_node tmpNode;
      tmpNode.index = dimIdx+1;
      tmpNode.value = trainingData.getVal(ptIdx, dimIdx);

      trainingProblem.x[ptIdx][dimIdx] = tmpNode;
    }
    //Create the end-node so that libsvm knows that this data point entry is over
    svm_node endNode;
    endNode.index = -1;
    endNode.value = -1;
    trainingProblem.x[ptIdx][trainingData.getHeight()] = endNode;
  }
  _train(trainingProblem);
}


void SVMClassifier::train(std::vector<std::vector<float> > trainingData, std::vector<float> dataClasses)
{
  ASSERT(trainingData.size() == dataClasses.size());

  //Setup the svm classifier
  svm_problem trainingProblem;

  //Tell the problem how many data points we have
  trainingProblem.l = dataClasses.size();

  //Copy the data classes into the training problem
  trainingProblem.y = new double[dataClasses.size()];
  std::copy(dataClasses.begin(), dataClasses.end(), trainingProblem.y);

  //Fill in the training data by creating a matrix of svm nodes
  trainingProblem.x = new svm_node*[trainingData.size()];
  for(uint i=0; i < trainingData.size(); i++)
  {
    //Allocate the nodes for this data point
    trainingProblem.x[i] = new svm_node[trainingData[i].size()+1];
    for(uint dimIdx=0; dimIdx<trainingData[i].size(); dimIdx++)
    {
      svm_node tmpNode;
      tmpNode.index = dimIdx+1;
      tmpNode.value = trainingData[i][dimIdx];
      trainingProblem.x[i][dimIdx] = tmpNode;
    }
    //Create the end-node so that libsvm knows that this data point entry is over
    svm_node endNode;
    endNode.index = -1;
    endNode.value = -1;
    trainingProblem.x[i][trainingData[i].size()] = endNode;
  }
  _train(trainingProblem);
}


void SVMClassifier::_train(svm_problem& trainingProblem)
  {
  //Check to make sure that our parameters and training problem are sane
  //TODO: Do something with this check...
  const char* check = svm_check_parameter(&trainingProblem, &itsSVMParams);
  LDEBUG("SVM Pararameter Check: %s", check);


  itsSVMModel = svm_train(&trainingProblem, &itsSVMParams);
}

