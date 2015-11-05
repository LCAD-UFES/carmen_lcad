/*!@file ModelNeuron/FreeViewingChannels.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/FreeViewingChannels.H $

#ifndef FREEVIEWINGCHAN_H_DEFINED
#define FREEVIEWINGCHAN_H_DEFINED

#include "Image/ImageSet.H"
#include "ModelNeuron/SpatioTemporalEngine.H"
#include "SpaceVariant/SCTransformModule.H"

// ###################################################################### 
// A base class for freeviewing channels
// ######################################################################
class FreeViewingChannel : public SCTransformModule
{
  public:
    //! Constructor
    /*! See ModelComponent.H for details */
    FreeViewingChannel(OptionManager& mgr, const std::string& descrName = "", const std::string& tagName = "");

    //! destructor
    virtual ~FreeViewingChannel();

    //! pre-process, transform, and post-process. Override doPreProcess and doPostProcess
    virtual ImageSet<float> getOutput(const Image<float> & lumimg, const Image<float> & rgimg, const Image<float> & byimg);

    //!get the subchannel names
    virtual std::vector<std::string> getSubChanNames() const;

    //!number of sub chans
    uint const numSubChans() const;

    //! get the scale factor
    virtual uint const getFactor() const;

    //! preprocess the luminance, red-green opponency and blue-yellow raw inputs
    virtual ImageSet<float> doPreProcess(const Image<float> & lumimg, const Image<float> & rgimg, const Image<float> & byimg);
    
    //! transform
    virtual void doTransform(ImageSet<float> & inpimg);
    
    //! transform the i'th subchan
    void doTransform(ImageSet<float> & imgset, uint const ii);
    
    //! perform any post processing
    virtual void doPostProcess(ImageSet<float> & inpimg);
  protected:
    NModelParam<bool> takeAbs;
    NModelParam<bool> takeRectify;
    std::vector<std::string> itsSubChanNames;
};
 
// ###################################################################### 
// A base class for SpatioTemporalEnergy channels
// ######################################################################
class SCSpeChannelBase : public FreeViewingChannel
{
  public:
    //construct
    SCSpeChannelBase(OptionManager& mgr, uint const scale, const std::string& descrName = "", const std::string& tagName = "");
    
    //destruct
    virtual ~SCSpeChannelBase();

    //!set the engine
    void setEngine(rutz::shared_ptr<SpatioTemporalEnergyEngine> const & engine);

    //!get the scale
    uint const getScale() const;

    //! get the factor
    uint const getFactor() const;

    //preprocess    
    ImageSet<float> doPreProcess(const Image<float> & lumimg, const Image<float> & rgimg, const Image<float> & byimg);
    
  protected:
    void start1();

    SpatioTemporalEnergy itsFilters;
    rutz::shared_ptr<SpatioTemporalEnergyEngine> itsEngine;
    uint itsScale;
};

// ###################################################################### 
// The SC Freeviewing channels
// ######################################################################

// ###################################################################### 
// A luminace channel
// ######################################################################
class SCLuminanceChannel : public FreeViewingChannel
{
  public:
    //! Constructor
    /*! See ModelComponent.H for details */
    SCLuminanceChannel(OptionManager& mgr, const std::string& descrName = "SC Luminance Channel",
                             const std::string& tagName = "SCLuminanceChannel");
    
    //! Destructor
    virtual ~SCLuminanceChannel();

    //compute the luminace info
    ImageSet<float> doPreProcess(const Image<float> & lumimg, const Image<float> & rgimg, const Image<float> & byimg);
};

// ###################################################################### 
// R-G channel
// ######################################################################
class SCRGChannel : public FreeViewingChannel
{
  public:
    //! Constructor
    /*! See ModelComponent.H for details */
    SCRGChannel(OptionManager& mgr, const std::string& descrName = "SC RG Channel",
                      const std::string& tagName = "SCRGChannel");
    
    //! Destructor
    virtual ~SCRGChannel();

    //compute the color info
    ImageSet<float> doPreProcess(const Image<float> & lumimg, const Image<float> & rgimg, const Image<float> & byimg);
};

// ###################################################################### 
// B-Y channel
// ######################################################################
class SCBYChannel : public FreeViewingChannel
{
  public:
    //! Constructor
    /*! See ModelComponent.H for details */
    SCBYChannel(OptionManager& mgr, const std::string& descrName = "SC BY Channel",
                      const std::string& tagName = "SCBYChannel");
    
    //! Destructor
    virtual ~SCBYChannel();

    //compute the color info
    ImageSet<float> doPreProcess(const Image<float> & lumimg, const Image<float> & rgimg, const Image<float> & byimg);
};

// ######################################################################
class SCEdgeChannel : public SCSpeChannelBase
{
  public:
    //! Constructor
    /* See ModelComponent.H for details */
    SCEdgeChannel(OptionManager& mgr, uint const scale, const std::string& descrName = "SC Edge Channel",
                  const std::string& tagName = "SCEdgeChannel");
    
    //! Destructor
    virtual ~SCEdgeChannel() { };
    
  protected:
    void start1();
    
    NModelParam<uint> itsNumOrientations;
};

// ######################################################################
class SCFlickerChannel : public SCSpeChannelBase
{
  public:
    //! Constructor
    /* See ModelComponent.H for details */
    SCFlickerChannel(OptionManager& mgr, uint const scale, const std::string& descrName = "SC Flicker Channel",
                  const std::string& tagName = "SCFlickerChannel");
    
    //! Destructor
    virtual ~SCFlickerChannel() { };

  protected:
    void start1();
};

// ######################################################################
class SCMotionChannel : public SCSpeChannelBase
{
  public:
    //! Constructor
    /* See ModelComponent.H for details */
    SCMotionChannel(OptionManager& mgr, uint const scale, const std::string& descrName = "SC Motion Channel",
                  const std::string& tagName = "SCMotionChannel");
    
    //! Destructor
    virtual ~SCMotionChannel() { };

    ImageSet<float> doPreProcess(const Image<float> & lumimg, const Image<float> & rgimg, const Image<float> & byimg);
    
  protected:
    void start1();

    NModelParam<uint> itsNumTemporalOrientations;
    NModelParam<uint> itsNumOrientations;
    NModelParam<bool> itsRelativeMotion;
};

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif
