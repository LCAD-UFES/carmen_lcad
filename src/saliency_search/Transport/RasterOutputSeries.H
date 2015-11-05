/*!@file Transport/RasterOutputSeries.H */

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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/RasterOutputSeries.H $
// $Id: RasterOutputSeries.H 9547 2008-03-28 23:32:43Z rjpeters $
//

#ifndef TRANSPORT_RASTEROUTPUTSERIES_H_DEFINED
#define TRANSPORT_RASTEROUTPUTSERIES_H_DEFINED

#include "Component/ModelParam.H"
#include "Raster/RasterFileFormat.H"
#include "Transport/FrameOstream.H"

//! FrameOstream subclass that writes raster files
class RasterOutputSeries : public FrameOstream
{
public:
  //! Constructor
  RasterOutputSeries(OptionManager& mgr);

  //! Destructor
  virtual ~RasterOutputSeries();

  //! Override from FrameOstream; just calls setFileStem()
  virtual void setConfigInfo(const std::string& filestem);

  //! Set the current frame number and remember it
  virtual bool setFrameNumber(int n);

  //! Write a frame to disk
  virtual void writeFrame(const GenericFrame& frame,
                          const std::string& shortname,
                          const FrameInfo& auxinfo);

  //! No-op
  virtual void closeStream(const std::string& shortname);

  //! Specify the output filename stem
  void setFileStem(const std::string& s);

  //! Get the output raster file format for this series
  virtual RasterFileFormat getRasterFileFormat() const = 0;

private:
  //! compute filename corresponding to args and return as string
  /*! Use sformat() to generate shortname if you need printf-style
      formatting. */
  std::string computeOutputFileName(const std::string& shortname) const;

  //! text log file name
  OModelParam<std::string> itsLogFile;

  //! output filename prefix
  std::string itsPrefix;

  int itsFrameNumber;

  int itsNumericFieldWidth;
};

//! Implements RasterOutputSeries using a command-line option to determine the output format
class GenericRasterOutputSeries : public RasterOutputSeries
{
public:
  //! Constructor
  GenericRasterOutputSeries(OptionManager& mgr);

  //! Destructor
  virtual ~GenericRasterOutputSeries();

  //! Get the output raster file format for this series
  virtual RasterFileFormat getRasterFileFormat() const;

private:
  //! output file format
  OModelParam<RasterFileFormat> itsRasterFileFormat;
};


//! Implements RasterOutputSeries with a fixed file type
template <RasterFileFormat F>
class FixedRasterOutputSeries : public RasterOutputSeries
{
public:
  //! Constructor
  FixedRasterOutputSeries(OptionManager& mgr);

  //! Destructor
  virtual ~FixedRasterOutputSeries();

  //! Get the output raster file format for this series
  virtual RasterFileFormat getRasterFileFormat() const;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_RASTEROUTPUTSERIES_H_DEFINED
