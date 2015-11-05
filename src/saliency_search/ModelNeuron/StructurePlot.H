/*!@file ModelNeuron/StructurePlot.H Class declaration for StructurePlot
   which creates hierarchical 2d image represenation of a structures
   activity*/

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
// $HeadURL:svn://ilab.usc.edu/trunk/saliency/src/ModelNeuron/StructurePlot.H$

#ifndef MODELNEURON_STRUCTUREPLOT_H_DEFINED
#define MODELNEURON_STRUCTUREPLOT_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include "Image/Layout.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/DrawOps.H"
#include "Image/ShapeOps.H"
#include "Image/ColorMap.H"
#include "Image/ColorOps.H"
#include "Image/MathOps.H"
#include "Image/Range.H"
#include "Util/StringConversions.H"

#include "ModelNeuron/LayerDecoder.H"
#include "ModelNeuron/Location.H"
#include "ModelNeuron/PlotBuffer.H"
#include "ModelNeuron/SimStructure.H"

namespace nsu
{
  // ######################################################################
  // ! A class for displaying a hierarchy of activity from a
  // SimStructure's sub units.
  // ######################################################################
  enum NormalizeType {SCALE = 0, RANGE = 1, SET_RANGE = 2};

  class StructurePlot
  {
    public:
      //!Constructor 
      StructurePlot(const SimStructure& structure,
                    const NeuralDecoder& decoder, 
                    const uint depth = 2,
                    const NormalizeType normtype = SCALE, 
                    const double& rangemin = 0, const double& rangemax = 0);

      //!Constructor 
      StructurePlot(const SimStructure& structure,
                    const uint depth = 2,
                    const NormalizeType normtype = SCALE, 
                    const double& rangemin = 0, const double& rangemax = 0);
  
      //!Destructor
      ~StructurePlot();
  
      //!plot the structure
      Layout<PixRGB<byte> > draw(SimStructure& structure,                //the structure to draw
                                 const uint w = 320, const uint h = 240, //the output width
                                 const Dims dims = Dims(0,1),            //(1,0) for column, 0,1 for row plot
                                 const uint length = 250,                //length in samples
                                 const uint plotdepth = 4,               //depth in structure
                                 const bool usedisplayoutput = false);   //use computation, or display

      //!plot the structure
      Layout<PixRGB<byte> > draw(SimStructure& structure,                //the structure to draw
                                 const Image<double>& data,              //any data we want to plot alongsize
                                 const uint w = 320, const uint h = 240, //the output width
                                 const Dims dims = Dims(0,1),            //(1,0) for column, 0,1 for row plot
                                 const uint length = 250,                //length in samples
                                 const uint plotdepth = 4,               //depth in structure
                                 const bool usedisplayoutput = false);   //use computation, or display

      //!plot the structures 2d components
      Layout<PixRGB<byte> > drawStructure(SimStructure& structure, 
                                          const uint w, const uint h, 
                                          const uint depth,
                                          const bool usedisplayoutput);

      //!plot the structs 1D components
      Layout<PixRGB<byte> > drawLinePlots(SimStructure& structure,                //the structure to draw
                                          const uint w = 320, const uint h = 240, //the output width
                                          const Dims dims = Dims(0,1),            //(1,0) for column, 0,1 for row plot
                                          const uint length = 250,                //length in samples
                                          const uint plotdepth = 4,               //depth in structure
                                          const bool usedisplayoutput = false);   //use computation, or display

      //!plot the structs 1D components
      Layout<PixRGB<byte> > drawLinePlots(SimStructure& structure,                //the structure to draw
                                          const Image<double>& data,              //any data we want to plot alongsize
                                          const uint w = 320, const uint h = 240, //the output width
                                          const Dims dims = Dims(0,1),            //(1,0) for column, 0,1 for row plot
                                          const uint length = 250,                //length in samples
                                          const uint plotdepth = 4,               //depth in structure
                                          const bool usedisplayoutput = false);   //use computation, or display

      //!plot the structs 1D components
      Layout<PixRGB<byte> > drawLinePlots(SimStructure& structure,                //the structure to draw
                                          const double & data,                    //any data we want to plot alongsize
                                          const uint w = 320, const uint h = 240, //the output width
                                          const Dims dims = Dims(0,1),            //(1,0) for column, 0,1 for row plot
                                          const uint length = 250,                //length in samples
                                          const uint plotdepth = 4,               //depth in structure
                                          const bool usedisplayoutput = false);   //use computation, or display
  
      //!set the probe
      void setProbe(const Location& probe);

      //!set the sampling rate of the plotter so that time plots can be
      //!labeled correctly
      void setSamplingRate(const SimTime&  samplingrate);

      //reset to state after construction;
      void reset();
  
    private:
      //!get the total number of subs
      uint countSubs(const SimStructure& structure, uint depth);

      //!prohibit copy constructor and assignment
      StructurePlot(const StructurePlot& rhs);
      StructurePlot& operator=(const StructurePlot& rhs);

      NormalizeType itsNormType;
      std::vector<Range<double> > itsRange;
      std::vector<std::string> itsNames;
      std::vector<ColorMap> itsCol;
      int itsTotal, itsCount;
      std::vector<LayerDecoder> itsDecoder;
      Location itsProbe;
      PlotBufferList itsN;
      uint itsDepth;
  };

  // ######################################################################
  // ! inline implementation for StructurePlot
  // ######################################################################
  inline
  StructurePlot::StructurePlot(const SimStructure& structure,
                               const NeuralDecoder& decoder, 
                               const uint depth, 
                               const NormalizeType normtype, 
                               const double& rangemin, const double& rangemax) : 
      itsNormType(normtype), itsRange(), 
      itsNames(), itsCol(), itsTotal(countSubs(structure, depth)), itsCount(-1), 
      itsDecoder(), itsProbe(), itsN(), itsDepth(depth)
  {
    for (int ii = 0; ii < itsTotal; ++ii)
    {
      itsDecoder.push_back(LayerDecoder(decoder, structure.getOutDims())); 
      itsRange.push_back(Range<double>(rangemin, rangemax));
    }
  }

  // ######################################################################
  inline
  StructurePlot::StructurePlot(const SimStructure& structure,
                               const uint depth,
                               const NormalizeType normtype, 
                               const double& rangemin, const double& rangemax) : 
      itsNormType(normtype), itsRange(), 
      itsNames(), itsCol(), itsTotal(countSubs(structure, depth)), itsCount(-1), 
      itsDecoder(), itsProbe(), itsN(), itsDepth(depth)
  { 
    for (int ii = 0; ii < itsTotal; ++ii)
      itsRange.push_back(Range<double>(rangemin, rangemax));
  }

  // ######################################################################
  inline
  StructurePlot::~StructurePlot()  
  { }

  // ######################################################################
  inline
  void StructurePlot::setProbe(const Location& probe)
  {
    itsN.clear();
    itsProbe = probe; 
  };

  // ######################################################################
  inline
  void StructurePlot::setSamplingRate(const SimTime&  samplingrate)
  {
    itsN.setSamplingRate(samplingrate);
  }

  // ######################################################################
  inline
  Layout<PixRGB<byte> > 
  StructurePlot::draw(SimStructure& structure,
                      const uint w, const uint h, const Dims dims,
                      const uint length, const uint plotdepth,
                      const bool usedispout)
  {
    Layout<PixRGB<byte> > ds = drawStructure(structure, w, h, itsDepth, usedispout);
    Layout<PixRGB<byte> > out = drawLinePlots(structure, w*2, h, dims, length, plotdepth, usedispout);

    //compine the probe and layer plots. 
    out = hcat(ds, out);
    return out;
  }

  // ######################################################################
  inline
  Layout<PixRGB<byte> > 
  StructurePlot::draw(SimStructure& structure, const Image<double>& data,
                      const uint w, const uint h, const Dims dims,
                      const uint length, const uint plotdepth,
                      const bool usedispout)
  {
    Layout<PixRGB<byte> > ds = drawStructure(structure, w, h, itsDepth, usedispout);
    Layout<PixRGB<byte> > out = drawLinePlots(structure, data, w*2, h, dims, length, plotdepth, usedispout);
    
    //compine the probe and layer plots. 
    out = hcat(ds, out);
    return out;
  }

  // ######################################################################
  inline
  Layout<PixRGB<byte> >
  StructurePlot::drawStructure(SimStructure& structure,
                               const uint w, const uint h,
                               const uint depth, const bool usedispout)
  {
    Layout<PixRGB<byte> > layout;

    //perform depth first grab of output data
    if (depth > 0)
      for (uint i = 0; i < structure.numSubs(); ++i)
      {
        SimStructure& temp = structure.editSub(i);
        uint mydepth = depth - 1;
        layout = vcat(layout, drawStructure(temp, w, h, mydepth, usedispout));
      }

    //base case
    ++itsCount;

    //since output is typically one of the subs in a structure, don't plot if we have children, unless we are at our
    //plotting depth
    if ((structure.numSubs() <= 0) || (depth == 0))
    {
      Image<double> out = (usedispout) ? structure.getDisplayOutput() : structure.getOutput();
      
      if (itsDecoder.size() > 0)
      {
        itsDecoder[itsCount].push(out);
        out = itsDecoder[itsCount].getOutput();
      }

      //scale and convert type, rescale if necessary
      Image<byte> outb;
      if (itsNormType == SCALE) {
        inplaceNormalize(out, 0.0, 255.0);
        outb = out;
      }
      else if (itsNormType == RANGE) {
        itsRange[itsCount].merge(rangeOf(out));
        outb = remapRange(out, itsRange[itsCount], Range<double>(0.0, 255.0)); 
      }
      else if (itsNormType == SET_RANGE)
        outb = remapRange(out, itsRange[itsCount], Range<double>(0.0, 255.0)); 
      
      outb = rescale(outb, Dims(w,h));

      //add some text
      if ((itsNormType == RANGE) || (itsNormType == SET_RANGE))
      {
        std::string temp = itsNames[itsCount] + " " + 
          convertToString(itsRange[itsCount].min()) + ":" + 
          convertToString(itsRange[itsCount].max());
        writeText(outb, Point2D<int>(0,0), temp.c_str());
      }
      else
        writeText(outb, Point2D<int>(0,0), itsNames[itsCount].c_str());

      //clean up by adding some boarder and colorizing
      Rectangle r(Point2D<int>(0,0), Dims(outb.getWidth(),outb.getHeight()));
      drawRectSquareCorners(outb ,r , (byte)255, 2);

      Image<PixRGB<byte> > outbrgb = colorize(outb, itsCol[itsCount]);
      layout = vcat(outbrgb, layout);
    }

    if (itsCount == itsTotal-1)
      itsCount = -1;

    return layout;
  }

  // ######################################################################
  inline
  Layout<PixRGB<byte> > 
  StructurePlot::drawLinePlots(SimStructure& structure,
                               const uint w, const uint h, const Dims dims,
                               const uint length, const uint plotdepth,
                               const bool usedispout)
  {
    if (itsProbe.dims() < 1)
      LFATAL("Must set a probe to use this functon");
    
    std::vector<const SimUnit*> units; 
    structure.getSimUnit(itsProbe, units);
    itsN.push(units, length, plotdepth, usedispout);

    Layout<PixRGB<byte> > out = itsN.draw(false, w, h, dims, 
                                          (itsNormType == SCALE) 
                                          ? Range<double>(0.0, 0.0) 
                                          : itsRange.back());
    
    return out;
  }

  // ######################################################################
  inline
  Layout<PixRGB<byte> > 
  StructurePlot::drawLinePlots(SimStructure& structure, const Image<double>& data,
                               const uint w, const uint h, const Dims dims,
                               const uint length, const uint plotdepth,
                               const bool usedispout)
  {
    if (itsProbe.dims() < 2)
      LFATAL("Must set a probe of at least 2 dimensions to use this function");
    
    std::vector<const SimUnit*> units; 
    structure.getSimUnit(itsProbe, units);
    itsN.push(units, data.getVal(itsProbe.getDim(0), itsProbe.getDim(1)), length, plotdepth, usedispout);
    
    Layout<PixRGB<byte> > out = itsN.draw(false, w, h, dims, 
                                          (itsNormType == SCALE) 
                                          ? Range<double>(0.0, 0.0) 
                                          : itsRange.back());   
    
    return out;
  }

  // ######################################################################
  inline
  Layout<PixRGB<byte> > 
  StructurePlot::drawLinePlots(SimStructure& structure, const double & data,
                               const uint w, const uint h, const Dims dims,
                               const uint length, const uint plotdepth,
                               const bool usedispout)
  {
    if (itsProbe.dims() < 2)
      LFATAL("Must set a probe of at least 2 dimensions to use this function");
    
    std::vector<const SimUnit*> units; 
    structure.getSimUnit(itsProbe, units);
    itsN.push(units, data, length, plotdepth, usedispout);
    
    Layout<PixRGB<byte> > out = itsN.draw(false, w, h, dims, 
                                          (itsNormType == SCALE) 
                                          ? Range<double>(0.0, 0.0) 
                                          : itsRange.back());   
    
    return out;
  }

  // ######################################################################
  inline
  void StructurePlot::reset()
  {
    std::vector<LayerDecoder>::iterator iter(itsDecoder.begin());
    while (iter != itsDecoder.end())
      (iter++)->reset();
  
    itsN.clear();
  }

  // ######################################################################
  inline
  uint StructurePlot::countSubs(const SimStructure& structure, uint depth)
  {
    if (depth > 0)
      for (uint i = 0; i < structure.numSubs(); ++i)
      {
        const SimStructure& temp(structure.getSub(i));
        const uint mydepth = depth - 1;
        countSubs(temp, mydepth); //recursion = depth first count 
      }

    itsNames.push_back(structure.getName());

    //setup color
    if (structure.getName().find("Inh") == 0)
    {
      itsCol.push_back(ColorMap::GRADIENT(PixRGB<byte>(255,255,255), 
                                          PixRGB<byte>(0,0,255)));
      LINFO("Assigning colormap for %s: blue", structure.getName().c_str());
    }
  
    else if (structure.getName().find("Exc") == 0)
    {
      itsCol.push_back(ColorMap::GRADIENT(PixRGB<byte>(255,255,255), 
                                          PixRGB<byte>(255,0,0)));
      LINFO("Assigning colormap for %s: red", structure.getName().c_str());
    }
    else
    {
      itsCol.push_back(ColorMap::JET());
      LINFO("Assigning colormap for %s: jet", structure.getName().c_str());
    }
  
    return itsNames.size();
  }
}

#endif
#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
