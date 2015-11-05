/*!@file VFAT/covEstimate.H quick estimator for coveriance matrix
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/covEstimate.H $
// $Id: covEstimate.H 6003 2005-11-29 17:22:45Z rjpeters $
//
// ############################################################
// ############################################################
// ##### --- VFAT ---
// ##### Vision Feature Analysis Tool:
// ##### T. Nathan Mundhenk nathan@mundhenk.com
// ##### Laurent Itt itti@pollux.usc.edu
// #####
// ############################################################
// ############################################################

#ifndef COVESTIMATE_H_DEFINED
#define COVESTIMATE_H_DEFINED

#include <vector>
#include "Image/Pixels.H"
#include "Image/Point2D.H"
#include "Image/All.H"
#include "VFAT/covHolder.H"

//! This method attmepts to estimate eignen values and vectors
/*! The basic idea behind this class is to estimate a gaussian envolope
    around some given data but do so by avoiding matrix inversion and
    multiplicaition. Instead, we estimate values and gain an order of
    complexity in speed.
*/

template <class T> class covEstimate
{
private:
  //! Pointer to raw data matrix in vector form
  std::vector<std::vector<T> > *cov_inputSpace;
  //! Pointer to map of raw data matrix in vector form
  std::vector<std::vector<T*> > *cov_inputSpacePtr;
  //! Pointer to map of raw data matrix in vector form, float cast form
  std::vector<std::vector<float*> > *cov_FinputSpacePtr;
  //! work matrix to rotate and translate
  std::vector<std::vector<T> >  cov_transSpace;
  //! temporary rotation space
  std::vector<std::vector<T> >  cov_rotSpace;
  //! container for mean translated during computation of P
  std::vector<T> cov_meanTranslate;
  //! container for mean Rotated during computation of P
  std::vector<T> cov_meanRot;
  //! sum of squares for independant data set after rotation/translation
  std::vector<T> cov_sumSquare;
  //! list of rotations from forward rotation, used to reverse rotate stdSpace
  std::vector<int> cov_list_i;
  //! list of rotations from forward rotation, used to reverse rotate stdSpace
  std::vector<int> cov_list_j;
  //! size of cov_list for reverse rotations
  int cov_listSize;
  //! size of input space, obtained from cov_inputSpace
  //unsigned int cov_iSpaceSize;
  //! size of transformation space, obtained from cov_transSpace
  //unsigned int cov_tSpaceSize;
  //! dimension of this space
  unsigned long cov_dim;
  //! samples in this space
  unsigned long cov_samples;
  //! tracks current status of algorithm
  float cov_currentStep;
  //! tells if we are working from a vector or a vector map (pointers)
  bool cov_usePtr;
  //! use special pointer for float entry when T = double
  bool cov_useFloatSpecial;
  //! tell if we are going to use the more simple method with dual interactions
  bool cov_doSimple;
  //! temp vector
  std::vector<T> cov_tempTS;
  //! (1) find mean value (centroid) in matrix
  void findMean();
  //! (1a) OPTIONAL find the min and max values
  void findMinMax();
  //! (2) translate space to new coordinates
  void translateSpace();
  //! (3) find covarience reducing rotations and rotate space
  void findRotatios(bool doSimple);
  //! (4) find variance of matrix using assumption of independance
  void findVariance();
  //! this is a pointer to the covHolder input
  covHolder<T> *cov_covHolder;
public:
  //! Default constructor, call and input pointer to first matrix
  covEstimate(std::vector<std::vector<T> > &_inputSpace,covHolder<T> &_ch);
  //! for backward compatibility
  covEstimate(std::vector<std::vector<T*> > &_inputSpace,covHolder<T> &_ch);
  //! Default constructor, call and input pointer to first matrix
  covEstimate();
  //! Default destructor, call and input pointer to first matrix
  ~covEstimate();
  //! given the stored covarienc matrix what is the prob. of some point P
  /*! @param sample This a new sample point in the same space as cov matrix
      @param stop How early to stop from end of list in diminsions
   */
  T getP(std::vector<T> sample, covHolder<T> &_ch, int stop);
  //! obtain the distance between this point and the mean found in covEstimate
  /*! @param sample This a new sample point in the same space as cov matrix
  */
  T getD(std::vector<T> sample, covHolder<T> &_ch, int stop);
  //! obtain the inverse distance between this point and the other
  T getD(std::vector<T> *point1,std::vector<T> *point2,
         std::vector<T> *biasSet, bool useGuass);
  //! match clusters from mean values
  /*! This will take in a set of mean values about each cluster and
      match clusters from one set to clusters in another set based upon
      mean values.
      @param ch1 this is a list of covHolders for set 1
      @param size1 this is the size of the first list
      @param ch2 this is a list of covHolders for set 2
      @param size2 this is the size of the second list
  */
  void matchPmean(std::vector<covHolder<T> > *chNew, long sizeNew,
                  std::vector<covHolder<T> > *chOld, long sizeOld,
                  long minSize);
  //! match clusters from mean values
  /*! This will take in a set of mean values about each cluster and
      match clusters from one set to clusters in another set based upon
      mean values. This one accumulates Classes.
      @param ch1 this is a list of covHolders for set 1
      @param size1 this is the size of the first list
      @param ch2 this is a list of covHolders for set 2
      @param size2 this is the size of the second list
  */
  void matchPmeanAccum(std::vector<covHolder<T> > *ch, unsigned int *sizeCH,
                  std::vector<covHolder<T> > *accum, unsigned int *sizeAccum,
                  long minSize);
  //! find the gaussian P for some data
  T gauss(T X, T mu, T std);
  //! run covestimator with input data
  void run();
  //! (1a) call to insert new matrix, force (clear) resize if nessesary
  void setNew(std::vector<std::vector<T> > &_inputSpace,
              bool doResize, covHolder<T> &_ch, T initVal);
  //! (1a) call to insert new matrix, force (clear) resize if nessesary
  void setNew(std::vector<std::vector<T*> > &_inputSpace,
              bool doResize, covHolder<T> &_ch, T initVal);
  //! (1a) call to insert new matrix, force (clear) resize if nessesary
  void setNew(std::vector<std::vector<T> > &_inputSpace,
              T initVal, int samples, int dim, covHolder<T> &_ch,
              bool _resize);
  //! (1a) call to insert new matrix, force (clear) resize if nessesary
  void setNew(std::vector<std::vector<T*> > &_inputSpace,
              T initVal, int samples, int dim, covHolder<T> &_ch,
              bool _resize);
  //! set new, but lets us set a float when T = double etc.
  void setNewF(std::vector<std::vector<float*> > &_inputSpace,
               T initVal, int samples, int dim, covHolder<T> &_ch,
               bool _resize);
  //! (aux) resize and clear all matrices
  /*! this should not be called directly, but may if you wish
     @param dimensions This is the dimension of the input matrix
     @param samples This is the number of samples taken
     @param zero This is what value you would like to set all elements to
  */
  void resize(unsigned int _dimensions, unsigned int _samples, T zero);
  //! (aux) print debugging data
  void printDebug();
  //! (aux) this will return an image slice of the matrix in its current form
  /*! This takes a 2D snapshot of the space along the two dimensions provided
    @param spaceSize, this is the "size" of the image to be returned as a square of size X x X

   */
  void printEigenVals();
  Image<float> returnMatrixSlice(int dim1, int dim2, int spaceSize);
  //! (aux) this shows how the covarience matrix fits over the data
  Image<float> returnCovSlice(int dim1, int dim2, int spaceSize,
                              bool inorm = false);
  Image<float> returnCovSlice(int dim1, int dim2, Image<float> &img,
                              bool inorm = false);

  //! dump the covMatrix to files, this will append the output to these files
  void dumpMatrix(std::string fileName, int index, std::string ID);
  //! dump the covMatrix to files, this will append the output to these files
  void dumpMatrix(std::string fileName, covHolder<T> &_ch, int index, std::string ID);
  //! return the sample size
  unsigned long getSampleSize();

};
#endif



