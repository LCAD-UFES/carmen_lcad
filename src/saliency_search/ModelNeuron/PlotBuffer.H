/*!@file ModelNeuron/PlotBuffer.H Class declaration for PlotBuffer,
   which will hold and plot information from any SimUnit. */

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
// Primary maintainer for this file: David J. Berg <dberg@usc.edu>
// $HeadURL:svn://ilab.usc.edu/trunk/saliency/src/ModelNeuron/PlotBuffer.H$

#ifndef MODELNEURON_PLOTBUFFER_H_DEFINED
#define MODELNEURON_PLOTBUFFER_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include "Image/Layout.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/Range.H"
#include "Util/SimTime.H"

namespace nsu
{
  class SimUnit;

  // ######################################################################
  // ! A class for storing output of simulation units and plotting. 
  // ######################################################################
  class PlotBuffer
  {
    public:
      //! This will only construct the object. The object will be setup on
      //! the first call to push(). w and h set the maximum size of the
      //! display. Length is the number of samples to display in the
      //! window. Use 0 for length to display all samples.
      PlotBuffer();
  
      //!!destructor
      ~PlotBuffer() { };
  
      //!push data onto the plot buffer
      void push(const SimUnit& nsm, const uint length = 250,
                const uint depth = 4, const bool usedisplayout = false);

      //!push data onto the plot buffer, with addional data to be plooted atop the top level plot
      void push(const SimUnit& nsm, const double& data, const uint length = 250,
                const uint depth = 4, const bool usedisplayout = false);
  
      //!reset the state to that just after construction
      void reset();
  
      //!plot and possibly clear the buffer
      Layout<PixRGB<byte> > draw(const bool clear = false, 
                                 const uint w = 640, const uint h = 480, 
                                 const Range<double>& range = Range<double>(0.0,0.0));
  
      //!plot and possibly clear the buffer
      void draw(Image<PixRGB<byte> >& img, const bool clear = false, 
                const uint w = 640, const uint h = 480, 
                const Range<double>& range = Range<double>(0.0,0.0));
  
      //!set the sampling rate of the input so the x-label can be set correctly
      void setSamplingRate(const SimTime& time);

      //!get the total number of units plotted by this buffer
      const uint getTotal();
  
      //!convert a SimeTime object to its SI representation as a
      //!string. For example a SimTime object with a value of .001s will
      //!return 'milliseconds', an object with a value of .005ms will
      //!return "5 microseconds".
      static std::string SimTimeToSI(const SimTime& time);
  
    private:
      //!actually add the data. called from push
      void addData(const SimUnit& nsm, const uint length, 
                   const uint depth, const bool usedisplayout = false);  

      //!count the total nodes of our objec hierarchy
      void getSubInfo(const SimUnit& nsm, const uint depth);

      std::vector<std::deque<double> > itsData;
      std::deque<double> itsExtraData;
      std::vector<std::string> names, units;
      std::string itsXlabel;
      uint itsCount, itsChildCount, itsTotal, itsDepth;
  };

  // ######################################################################
  // ! A class for storing a list of plotbuffers (lots of 1D plots)
  // ######################################################################
  class PlotBufferList
  {
    public:
      //!constrcutor
      PlotBufferList() : 
          itsPb() {};

      //!destructor
      ~PlotBufferList() {};

      //!add data from a vector of neural simulation modules
      void push(const std::vector<const SimUnit*>& nsm, 
                const uint length = 250, const uint depth = 4,
                const bool usedisplayout = false);

      //!add data from a vector of neural simulation modules
      void push(const std::vector<const SimUnit*>& nsm, const double& data, 
                const uint length = 250, const uint depth = 4,
                const bool usedisplayout = false);

      //!clear all plots in buffer
      void clear();

      //!get a layout of the plot, Dims(1,0) will contatinate images along
      //!the horizontal axis. Dims(0,1) will contatinate along the
      //!vertical axis. For other Dims configurations the function will
      //!attempt to fill a grid, working across columns until the data is
      //!exausted for the grid is filled.
      Layout<PixRGB<byte> > draw(bool clear = false, const uint w = 640, 
                                 const uint h = 480, const Dims& d = Dims(0,1), 
                                 const Range<double>& range = Range<double>(0.0, 0.0));
  
      //!draw into an image - see above
      void draw(Image<PixRGB<byte> >& img, bool clear = false, 
                const uint w = 640, const uint h = 480, const Dims& d = Dims(0,1), 
                const Range<double>& range = Range<double>(0.0, 0.0));
  
      //!set the sampling rate of the input so the x-axis can be properly labeled
      void setSamplingRate(const SimTime& samplerate);
  
    private:
      std::vector<PlotBuffer> itsPb;
  };
}

#endif
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
