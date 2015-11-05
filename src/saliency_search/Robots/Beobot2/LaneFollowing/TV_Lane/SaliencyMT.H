/*!@file /Robots/Beobot2/LaneFollowing/RG_Lane/SaliencyMT.H 
 * A class for quick-and-dirty saliency mapping */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
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
// Primary maintainer for this file: Zack Gossman <gossman@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/Beobot2/LaneFollowing/RG_Lane/SaliencyMT.H $
// $Id: SaliencyMT.H 7229 2006-10-01 14:40:23Z kai $
//

#ifndef BEOSALIENCYMT_H_DEFINED
#define BEOSALIENCYMT_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Image/ColorOps.H"
#include "Image/Image.H"
#include "Image/ImageSet.H"
#include "Image/MathOps.H"
#include "Image/DrawOps.H"
#include "Image/FilterOps.H"
#include "Image/Pixels.H"
#include "Image/PyramidOps.H"
#include "Image/ShapeOps.H"
#include "Image/Transforms.H"
#include "Image/fancynorm.H"
#include "Util/Types.H"
#include "Image/colorDefs.H"

#include <pthread.h>
#include <cstdlib>
#include <unistd.h>
#include <list>

// ######################################################################
struct jobData
{
  jobData() { };

  jobData(const int jt, const PyramidType pt, const float w,
          const float ori = 0.0F) :
    jobType(jt), ptyp(pt), weight(w), orientation(ori) { }

  int jobType;
  PyramidType ptyp;
  float weight;
  float orientation;
};


// ######################################################################
class SaliencyMT : public ModelComponent
{
public:

  // ######################################################################
  /*! @name Constructors and Destructors */
  //@{

  //! Constructor
  SaliencyMT(OptionManager& mgr,
             const std::string& descrName = "SaliencyMT",
             const std::string& tagName = "SaliencyMT");

  //! Destructor
  ~SaliencyMT();

  //@}

  // ######################################################################
  /*! @name member functions */
  //@{

  //! Setup up with image to be processed and build the job queue
  void newInput(const Image< PixRGB<byte> > img, bool procFlicker=true);

  //! Is the last given input done processing?
  bool outputReady();

  //! Get last computed output
  Image<float> getOutput();
  Image<float> getRGOutput();
  Image<float> getBYOutput();
  Image<float> getIntensityOutput();


  //! For internal thread use: Compute a conspicuity map from an image
  void computeCMAP();
  //@}

protected:
  OModelParam<uint> itsNumThreads; //!< Number of worker threads to use

  void start1(); //!< get started
  void stop2();  //!< get stopped

private:
  Image< PixRGB<byte> > colima;     // current color image
  Image<float> lum;        //curent luminance image
  Image<byte> r, g, b, y;  //curent RGBY images
  Image<float> prev;       // previous float image
  Image<float> outmap;     // output saliency map
  Image<float> skinima;    //skin hue map
  Image<float> cmap_rg;    //current Red-Green Map
  Image<float> cmap_by;    //current Blue-Yellow Map
  Image<float> cmap_intensity;    //current Blue-Yellow Map
  bool gotLum, gotRGBY, gotSkin;

  std::list<jobData> jobQueue;
  uint jobsTodo;

  //thread stuff
  pthread_t *worker;
  pthread_mutex_t jobLock, mapLock;
  pthread_cond_t jobCond;
  uint numWorkers;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
