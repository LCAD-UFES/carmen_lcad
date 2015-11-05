/*!@file Channels/DescriptorVec.H descriptor vector generator for obj rec */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/DescriptorVec.H $
// $Id: DescriptorVec.H 9412 2008-03-10 23:10:15Z farhan $
//

#ifndef DESCRIPTORVEC_H_DEFINED
#define DESCRIPTORVEC_H_DEFINED

#include "Channels/InputFrame.H"
#include "Channels/ComplexChannel.H"
#include "Channels/SingleChannel.H"
#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Image/Pixels.H"
#include "Image/Image.H"
#include "SIFT/VisualObjectMatchAlgo.H"
#include "SIFT/Histogram.H"
#include "Image/FourierEngine.H"


// ######################################################################
//! Descriptor vector class
class DescriptorVec : public ModelComponent
{
public:
  //! Constructor, need complex channel
  DescriptorVec(OptionManager& mgr,
                const std::string& descrName,
                const std::string& tagName,
                ComplexChannel *cc );

  //! Destructor
  virtual ~DescriptorVec();

  //! Set the input image
  void setInputImg(const Image<PixRGB<byte> >& img);

  //! Set the fovea location to create the descriptor vector
  void setFovea(Point2D<int> loc);

  //! Get the fovea dims
  Dims getFoveaSize();

  //! Set the fovea dims
  void setFoveaSize(Dims &d);

  //! Set the fovea dims
  void setFoveaSize(int foveaRadius);

  //! Get the image under the fovea
  Image<PixRGB<byte> > getFoveaImage();

  //! Return the image of the histogram under the fovea
  Image<PixRGB<byte> > getHistogramImage();

  //! Build a SIFT descriptor from the the location of the fovea
  void buildSIFTDV();

  //! Build a particle count descriptor from the the location of the fovea
  void buildParticleCountDV();

  //! Build a Fourier Transform descriptor from the the location of the fovea
  void buildFftDV();

  //! Build a descriptor vector from the location of the fovea (complex channel)
  void buildDV();

  //! Build a descriptor vector from the location of the fovea (complex channel raw submaps)
  void buildRawDV();

  void buildLocalDV();

  //! Build a feature vector from a single channel
  void buildSingleChannelFV(SingleChannel &sc);

  //! Get the feature vector from the descriptor vector
  const std::vector<double>& getFV() const;

  //! Get the size of the feature vector
  uint getFVSize();

private:

  // Get the dominate orientation in the image
  // based on Lowes SIFT alg
  float getDominateOrientation(const Image<float> &img);

protected:
  ComplexChannel *itsComplexChannel;
  OModelParam<Dims> itsFoveaSize;    //! The size of our fovea
  Point2D<int> itsFoveaLoc;               //! THe location of the fovea
  Point2D<int> itsAttentionLoc;               //! THe location of the attention point
  Image<PixRGB<byte> > itsFoveaImg;  //! the image to the fovea
  Image<PixRGB<byte> > itsInputImg;  //! the input image
  Histogram     itsFeatureHistogram; //! The feature histogram
  mutable FourierEngine<double>* itsFEngine;
  Image<complexd> itsFTImg;
  std::vector<double> itsFV;         //! Our feature vector

  //!The feature vector computed, multidementional for each keypoint found
  //  std::vector<std::vector<double> > itsDescriptorVec;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
