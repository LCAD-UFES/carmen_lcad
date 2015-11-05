/*!@file Psycho/EyesalData.H Simple struct for eyesal output files data */

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
// Primary maintainer for this file: David Berg <dberg@usc.edu>
//

#ifndef PSYCHO_EYESALDATA_H_DEFINED

#include "Image/Pixels.H"
#include "Psycho/EyeData.H"
#include "Util/SimTime.H"
#include "rutz/shared_ptr.h"

#include <string>
#include <vector>


//! Simple class for an eyesal output file data, represents data and I/O
class EyesalData {
public:
  // ######################################################################
  /*! @name Constructors, destructor and initialization */
  //@{

  //! Emtpy Constructor for compatibility
  EyesalData();

  //! Constructor from a file on disk
  EyesalData(const std::string& filename);


  //@}

  // ######################################################################
  /*! @name Data access functions */
  //@{

    //! reset the file data
    void setFile(const std::string& filename);

  //! Do we have a sample for the given index?
  /*! This will return true as long as the index is not past the
    available amount of data. */
  bool hasData(const size_t index) const;

  //!return the normalized saliency values
  std::vector<float> getNormSal() const;

  //! return the normalized random value at index(usually 1:100)
  std::vector<float> getNormRand(const size_t index) const;

  //! return all the normalized random values
  std::vector< std::vector<float> > getAllNormRand() const;

  //! return all the normalized random values Transposed
  std::vector< std::vector<float> > getAllNormRandT() const;

  //!return the x and y position at the specified index
  Point2D<int> getXYpos(const size_t index) const;

  //!return the x and y positions as a vector
  std::vector<Point2D<int> > getXYpos() const;

  //!return the time at which the saccade landed at its target
  float getTime(const size_t index) const;

  //!return the times of saccadic endpoints as a vector
  std::vector<float> getTime() const;

  //!return the filename that the saccade at index occurred in
  std::string getFileName(const size_t index) const;

  //!return the filnames as a vector of string
  std::vector<std::string> getFileName() const;

  //! Get number of saccades in file
  size_t size() const;

  //@}

  // ######################################################################
  /*! @name Metadata access functions */
  //@{

  //! Get filename
  std::string filename() const;

  //! Get filename, without extension if any
  std::string basename() const;

  //@}

  // ######################################################################
  /*! @name Computations  */
  //@{

  //! get AUC of the data
  float getAUC() const;

 //@}

private:
  // forbid assignment and copy-construction:
  EyesalData& operator=(const EyesalData& e);
  EyesalData(const EyesalData& e);


  std::string itsFilename;
  // for internal use only!
  //Eyesal format:
  // x,y,fovx,fovy,pupil,amp,duration,sactime,val,min,max,avg,100 random
  struct RawEyesalData {
    std::string Filename;
    int x, y, fovx, fovy;
    float pupil, amp, duration, sactime,val,min,max,avg;
    float rand[300];
  };
  std::vector<RawEyesalData> itsData;
};




#define PSYCHO_EYESALDATA_H_DEFINED

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_EYESALDATA_H_DEFINED
