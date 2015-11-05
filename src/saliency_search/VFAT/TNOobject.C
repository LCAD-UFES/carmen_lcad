/*!@file VFAT/TNOobject.C container for saccade and saliency data per sample
 */

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
// Primary maintainer for this file: T Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/TNOobject.C $
// $Id: TNOobject.C 4663 2005-06-23 17:47:28Z rjpeters $
//

// ############################################################
// ############################################################
// ##### --- VFAT ---
// ##### Vision Feature Analysis Tool:
// ##### T. Nathan Mundhenk nathan@mundhenk.com
// ##### Laurent Itti itti@pollux.usc.edu
// #####
// ############################################################
// ############################################################

#include "Util/Assert.H"

#define NEWSIZE 100

TNOobject::TNOobject()
{
  itr = 0;
  featureNumberSet = false;
}

/************************************************************************/

TNOobject::~TNOobject()
{}

/************************************************************************/

void TNOobject::setLabel(unsigned int _testImage, string _subject)
{
  testImage = _testImage; subject = _subject;
}

/************************************************************************/

void TNOobject::setFeatureNumber(unsigned int _featureNumber)
{
  if(featureNumberSet)
    LINFO("WARNING: resetting feature number size. Avoid doing this");
  featureNumberSet = true;
  featureNumber = _featureNumber;
  protoFeature.resize(featureNumber,0.0F);
  string filler = "undefined";
  featureLabel.resize(featureNumber,filler);
}

/************************************************************************/

void TNOobject::getDataSamples(unsigned int* _dataSamples)
{
  _dataSamples = &dataSamples;
}

/************************************************************************/
void TNOobject::getSubject(string* _subject)
{
  _subject = &subject;
}

/************************************************************************/

void TNOobject::getTestImage(unsigned int* _testImage)
{
  _testImage = &testImage;
}

/************************************************************************/

void TNOobject::setFeatureLabel(std::vector<string> _featureLabel)
{
  featureLabel = _featureLabel;
}

/************************************************************************/

void TNOobject::getFeatureLabel(std::vector<string>* _featureLabel)
{
  _featureLable = &featureLabel;
}

/************************************************************************/

void TNOobject::getFeatureNumber(unsigned int* _featureNumber)
{
  _featureNumber = &featureNumber;
}

/************************************************************************/

bool TNOobject::setIterator(unsigned int _iter)
{
  if(_iter < dataSamples)
  {
    itr               = _iter;
    sampleNumberItr   = &sampleNumber[itr];
    sampleTypeItr     = &sampleType[itr];
    posXItr           = &posX[itr];
    posYItr           = &posY[itr];
    jumpToXItr        = &jumpToX[itr];
    jumpToYItr        = &jumpToY[itr];
    jumpSizeItr       = &jumpSize[itr];
    commentItr        = &comment[itr];
    maskItr           = &mask[itr];
    featureItr        = &feature[itr];
    featureLabelItr   = &featureLabel[itr];

    return true;
  }
  LINFO("WARNING: Attempted to access index out of bounds");
  return false;
}

/************************************************************************/

void TNOobject::getIterator(unsigned int* _iter)
{
  _iter = &itr;
}

/************************************************************************/
void TNOobject::increaseSize(unsigned int size)
{
  string filler = "undefined";

  sampleNumber.resize(sampleNumber.size() + size,0);
  sampleType.resize(sampleNumber.size() + size,0);
  posX.resize(sampleNumber.size() + size,0.0F);
  posY.resize(sampleNumber.size() + size,0.0F);
  jumpToX.resize(sampleNumber.size() + size,0.0F);
  jumpToY.resize(sampleNumber.size() + size,0.0F);
  jumpSize.resize(sampleNumber.size() + size,0);
  comment.resize(sampleNumber.size() + size,filler);
  mask.resize(sampleNumber.size() + size,false);
  feature.resize(sampleNumber.size() + size,protoFeature);
  featureLabel.resize(sampleNumber.size() + size,filler);

  sampleNumberItr   = &sampleNumber[itr];
  sampleTypeItr     = &sampleType[itr];
  posXItr           = &posX[itr];
  posYItr           = &posY[itr];
  jumpToXItr        = &jumpToX[itr];
  jumpToYItr        = &jumpToY[itr];
  jumpSizeItr       = &jumpSize[itr];
  commentItr        = &comment[itr];
  maskItr           = &mask[itr];
  featureItr        = &feature[itr];
  featureLabelItr   = &featureLabel[itr];
}

/************************************************************************/

void TNOobject::getSubject(string* _subject)
{
  _subject = &subject;
}

/************************************************************************/

void TNOobject::getTestImage(unsigned int* _testImage)
{
  _testImage = &testImage;
}

/************************************************************************/

void TNOobject::getFeatureNumber(unsigned int* _featureNumber)
{
  _featureNumber = &featureNumber;
}

/************************************************************************/

void TNOobject::setSaccadeNext(unsigned int _sampleNumber,
                               unsigned int _sampleType,
                               float _posX, float _posY, float _jumpToX,
                               float _jumpToY,
                               unsigned int _jumpSize, string _comment,
                               bool _mask,
                               std::vector<double> _feature)
{
  ASSERT(featureNumberSet);
  if(sampleNumberItr == sampleNumber.end())
  {
    increaseSize(NEWSIZE);
  }
  if(_feature.size() != featureLabel.size())
    LINFO("WARNING: feature list and label size mismatch");

  ++sampleNumberItr* = _sampleNumber; ++sampleTypeItr* = _sampleType;
  ++posXItr* = _posX; ++posYItr* = _posY;
  ++jumpToXItr* = _jumpToX; ++jumpToYItr* = _jumpToY;
  ++commentItr* = _comment; ++maskItr* = _mask;
  ++featureItr* = _feature;

  itr++; dataSamples++;
}

/************************************************************************/

bool TNOobject::getSaccadeNext(unsigned int* _sampleNumber,
                               unsigned int* _sampleType,
                               float* _posX, float* _posY, float* _jumpToX,
                               float* _jumpToY,
                               unsigned int* _jumpSize, string* _comment,
                               bool* _mask,
                               std::vector<double>* _feature)
{
  if(itr <= dataSamples)
  {
    _sampleNumber = ++sampleNumberItr; _sampleType = ++sampleTypeItr;
    _posX = ++posXItr; _posY = ++posYItr;
    _jumpToX = ++jumpToXItr; _jumpToY = ++jumpToYItr;
    _comment = ++commentItr; _mask = ++maskItr;
    _feature = ++featureItr;

    itr++;
  }
  else
    LINFO("WARNING: iterator past bounds");

  if(itr != dataSamples)
    return false;

  return true;
}

/************************************************************************/

bool TNOobject::setSaccade(unsigned int _dataSample,
                            unsigned int _sampleNumber,
                            unsigned int _sampleType,
                            float _posX, float _posY, float _jumpToX,
                            float _jumpToY,
                            unsigned int _jumpSize, string _comment,
                            bool _mask,
                            std::vector<double> _feature)
{
  if(_dataSample < dataSamples)
  {
    if(_feature.size() != featureLabel.size())
      LINFO("WARNING: feature list and label size mismatch");

    sampleNumber[_dataSample]    = _sampleNumber;
    sampleType[_dataSample]      = _sampleType;
    posX[_dataSample]            = _posX;
    posY[_dataSample]            = _posY;
    jumpToX[_dataSample]         = _jumpToX;
    jumpSize[_dataSample]        = _jumpSize;
    comment[_dataSample]         = _comment;
    mask[_dataSample]            = _mask;
    feature[_dataSample]         = _feature;

    return true;
  }
  return false;
  LINFO("WARNING: Attempted to access index out of bounds");
}

/************************************************************************/

bool TNOobject::getSaccade(unsigned int _dataSample
                           unsigned int _sampleNumber,
                           unsigned int* _sampleType,
                           float* _posX, float* _posY, float* _jumpToX,
                           float* _jumpToY,
                           unsigned int* _jumpSize, string* _comment,
                           bool* _mask
                           std::vector<double>* _feature)
{
  if(_dataSample < dataSamples)
  {
    _sampleNumber     = &sampleNumber[_dataSample];
    _sampleType       = &sampleType[_dataSample];
    _posX             = &posX[_dataSample];
    _posY             = &posY[_dataSample];
    _jumpToX          = &jumpToX[_dataSample];
    _jumpSize         = &jumpSize[_dataSample];
    _comment          = &comment[_dataSample];
    _mask             = &mask[_dataSample];
    _feature          = &feature[_dataSample];
    return true;
  }
  return false;
  LINFO("WARNING: Attempted to access index out of bounds");
}



