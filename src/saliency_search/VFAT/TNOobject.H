/*!@file VFAT/TNOobject.H container for saccade and saliency data per sample
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/TNOobject.H $
// $Id: TNOobject.H 4663 2005-06-23 17:47:28Z rjpeters $
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

/************************************************************************/

/*! This class is a container for each basic sample and includes
    the saccade data, the mask data, the saliency features, ICA features
    and complex features.
*/

/************************************************************************/
class TNOobject
{
public:
  TNOobject();
  ~TNOobject();
  //! set the labels for this sample
  void setLabel(unsigned int _testImage, string _subject);
  //! give this sample a number of features, must be called
  void setFeatureNumber(unsigned int _featureNumber);
  //! how many data samples do we have in this object
  void getDataSamples(unsigned int* _dataSamples);
  //! who is the subject we are talking about
  void getSubject(string* _subject);
  //! which test image are we using
  void getTestImage(unsigned int* _testImage);
  //! what do we call the features we have put in
  void setFeatureLabel(std::vector<string> _featureLabel);
  //! what did we call our features
  void getFeatureLabel(std::vector<string>* _featureLabel);
  //! what is this sample number?
  void getFeatureNumber(unsigned int* _featureNumber);
  //! all data is stored with the key of saccade number
  //! set the current iterator position, is false if not success
  bool setIterator(unsigned int _iter);
  //! get the current iterator position
  void getIterator(unsigned int* _iter);
  //! set saccade data using iterators to fill up the data set
  void setSaccadeNext(unsigned int _sampleNumber, unsigned int _sampleType,
             float _posX, float _posY, float _jumpToX, float _jumpToY,
             unsigned int _jumpSize, string _comment, bool _mask,
             std::vector<double> _feature);
  //! get saccade data using iterators. Will return true at the end
  bool getSaccadeNext(unsigned int* _sampleNumber, unsigned int* _sampleType,
             float* _posX, float* _posY, float* _jumpToX, float* _jumpToY,
             unsigned int* _jumpSize, string* _comment, bool* _mask,
             std::vector<double>* _feature);
  //! Random access to set saccade data, bool is false if not success
  bool setSaccade(unsigned int _dataSample,
                  unsigned int _sampleNumber, unsigned int _sampleType,
                  float _posX, float _posY, float _jumpToX, float _jumpToY,
                  unsigned int _jumpSize, string _comment, bool _mask,
                  std::vector<double> _feature);
  //! Random access to get saccade data, bool is false if not success
  bool getSaccade(unsigned int _dataSample
                  unsigned int _sampleNumber, unsigned int* _sampleType,
                  float* _posX, float* _posY, float* _jumpToX, float* _jumpToY,
                  unsigned int* _jumpSize, string* _comment, bool* _mask
                  std::vector<double>* _feature);
private:
  //! did we set the number of features
  bool featureNumberSet;
  //! how many data samples do we have for this image and subject
  unsigned int dataSamples;
  //! what is the test image index number
  unsigned int testImage;
  //! how many saliency features are we storing in this object
  unsigned int featureNumber;
  //! keeps track of the iterator position over all objects
  unsigned int itr;
  //! what is the subject name for this image sample
  string subject;
  //! how many saccade samples do we have
  std::vector<unsigned int> sampleNumber;
  std::vector<unsigned int>::iterator sampleNumberItr;
  //! what is the type for this sample (blink, saccade, etc)
  std::vector<unsigned int> sampleType;
  std::vector<unsigned int>::iterator sampleTypeItr;
  //! what is the eye position X
  std::vector<float> posX;
  std::vector<float>::iterator posXItr;
  //! what is the eye position Y
  std::vector<float> posY;
  std::vector<float>::iterator posYItr;
  //! where are they jumping to X?
  std::vector<float> jumpToX;
  std::vector<float>::iterator jumpToXItr;
  //! where are they jumping to Y?
  std::vector<float> jumpToY;
  std::vector<float>::iterator jumpToYItr;
  //! many samples in durration is this jump (see eyeS data)
  std::vector<unsigned int> jumpSize;
  std::vector<unsigned int>::iterator jumpSizeItr;
  //! any additional comment about this data
  std::vector<string> comment;
  std::vector<string>::iterator commentItr;
  //! is this a mask location in the image?
  std::vector<bool> mask;
  std::vector<bool>::iterator maskItr;
  //! what is the saliency etc. feature set?
  std::vector<std::vector<double> > feature;
  std::vector<std::vector<double> >::iterator featureItr;
  //! prototype object for feature
  std::vector<double> protoFeature;
  std::vector<double>::iterator protoFeatureItr;
  //! what are the labels for this feature?
  std::vector<string> featureLabel;
}


