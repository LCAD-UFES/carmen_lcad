/*!@file ModelNeuron/NeuralDecoder.H Class declaration for neural
   decoders, which expect as input a spike train, and outputs a
   decoded signal such as a spike density function, inter spike
   interval etc...*/

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
// Primary maintainer for this file: David Berg <dberg@usc.edu>
// $HeadURL:svn://ilab.usc.edu/trunk/saliency/src/ModelNeuron/NeuralDecoder.H$

#ifndef MODELNEURON_NEURALDECODER_H_DEFINED
#define MODELNEURON_NEURALDECODER_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include "Util/SimTime.H"
#include "GenericUtils/GenericFactory.H"
#include "GenericUtils/CreateFunctor.H"

namespace nsu
{
  // ######################################################################
  //! An abstract class for a neural decoder. Neural decoders transform
  //! input and output of SimUnit's, possibly with a delay.
  // ######################################################################
  class NeuralDecoder
  {
    public:
      //typedef for registering decoders
      typedef CreateFunctor<NeuralDecoder, ParamList<SimTime> >  Creator;
      typedef GenericFactory<NeuralDecoder, std::string, Creator> Factory;

      //!constructor 
      NeuralDecoder(const SimTime& timeStep = SimTime::MSECS(1.0),
                    const SimTime& windowSize = SimTime::MSECS(100.0)) :
          itsWindowSize(windowSize),
          itsSamples( (uint)(itsWindowSize.secs()*timeStep.hertz()) ) { };
  
      //!desctrctor
      virtual ~NeuralDecoder() { };
  
      //!add some data to the decoder
      virtual void push(const double& in) = 0; 

      //!get the latest (and possibly time delayed) output of the decoder
      virtual const double getOutput() const = 0;

      //!make a copy of the this object
      virtual NeuralDecoder* clone() const = 0;

      //! reset the decoder internals to the state after construction
      virtual void reset() { };
  
    protected:
      //prohibit copy and assign
      NeuralDecoder(const NeuralDecoder& rhs);
      NeuralDecoder& operator=(const NeuralDecoder& rhs);

      SimTime itsWindowSize;
      uint itsSamples;
  };

  // ######################################################################
  //! A class to derive from to create new NeuralDecoders. New
  //! NeuralDecoder derived types can derive from this class to inherit
  //! the clone() function if desired.
  /*
    Programmer Note: This class uses the 'quriously recursive template
    pattern' to simplify creation of new classes for the programmer. As
    such, to create a new simulation module, classes should adhear the
    following convention:
  
    class mymodule : public NeuralDecoderDerived<mymodule>
    {
    mymodule(//params//) : NeuralDecoder<mymodule>(//params//) { };
    //...rest of functions
    };
  */
  // ######################################################################
  template <class Derived>
  class NeuralDecoderDerived : public NeuralDecoder
  {
    public:
      NeuralDecoder* clone() const 
      { return new Derived(dynamic_cast<const Derived&>(*this)); };               

    protected:
      NeuralDecoderDerived(const SimTime& timeStep = SimTime::MSECS(1.0),
                           const SimTime& windowSize = SimTime::MSECS(100.0)) : 
          NeuralDecoder(timeStep, windowSize) { };
  
      ~NeuralDecoderDerived() { };
  };

  // ######################################################################
  //! A decoder that just passes through the data, possibly with a delay
  // ######################################################################
  class HoldDecoder: public NeuralDecoderDerived<HoldDecoder>
  {
    public:
      //!constructor 
      HoldDecoder(const SimTime& timeStep = SimTime::MSECS(1.0),
                  const SimTime& windowSize = SimTime::ZERO()) :
          NeuralDecoderDerived<HoldDecoder>(timeStep, windowSize), itsLastSpike(0.0), 
          itsSignal(itsSamples, 0.0) { };
  
      //!desctructor
      ~HoldDecoder() { };
  
      //!add some data to the decoder
      void push(const double& in); 

      //!get the instantaneous Spikes/Second
      const double getOutput() const;

      //! reset decoder internals
      void reset();

    private:
      double itsLastSpike;
      std::deque<double> itsSignal;
  };

  // ######################################################################
  //! A histogram (non overlapping) decoder, spike count
  // ######################################################################
  class HistDecoder: public NeuralDecoderDerived<HistDecoder>
  {
    public:
      //!constructor 
      HistDecoder(const SimTime& timeStep = SimTime::MSECS(1.0),
                  const SimTime& windowSize = SimTime::MSECS(100.0)) :
          NeuralDecoderDerived<HistDecoder>(timeStep,windowSize), itsSpikeCount(0),
          itsLastCount(0.0), itsSampleCount(0) { };
  
      //!desctructor
      ~HistDecoder() { };
  
      //!add some data to the decoder
      void push(const double& in); 

      //!get the instantaneous Spikes/Second
      const double getOutput() const;

      //! reset decoder internals
      void reset();

    private:
      double itsSpikeCount, itsLastCount;
      uint itsSampleCount;
  };

  // ######################################################################
  //!A decoder using a sliding rectangular window, or moving average
  //######################################################################
  class RectDecoder: public NeuralDecoderDerived<RectDecoder>
  {
    public:
      //!constructor 
      RectDecoder(const SimTime& timeStep = SimTime::MSECS(1.0),
                  const SimTime& windowSize = SimTime::MSECS(100.0)) :
          NeuralDecoderDerived<RectDecoder>(timeStep, windowSize), 
          itsSpikeRate(0.0), itsSignal(itsSamples, 0.0) { };
  
      //!desctructor
      ~RectDecoder() { };
  
      //!add some data to the decoder
      void push(const double& in); 

      //!get the instantaneous Spikes/Second
      const double getOutput() const;

      //! reset decoder internals
      void reset();
  
    private:
      double itsSpikeRate;
      std::deque<double> itsSignal;
  };

  // ######################################################################
  //! A Exponential rate decoder
  // ######################################################################
  class ExpDecoder: public NeuralDecoderDerived<ExpDecoder>
  {
    public:
      //!constructor 
      ExpDecoder(const SimTime& timeStep = SimTime::MSECS(1.0),
                 const SimTime& windowSize = SimTime::MSECS(100.0), 
                 const double& alpha = 0.99) :
          NeuralDecoderDerived<ExpDecoder>(timeStep,windowSize), itsSpikeRate(0.0), 
          itsAlpha(alpha) { };
  
      //!desctructor
      ~ExpDecoder() { };

      //!add some data to the decoder
      void push(const double& in); 

      //!get the instantaneous Spikes/Second
      const double getOutput() const;

      //! reset decoder internals
      void reset();
  
    private:
      double itsSpikeRate;
      double itsAlpha;
  };

  // ######################################################################
  //! A Rate decoder using an alpha function
  // ######################################################################
  class AlphaDecoder: public NeuralDecoderDerived<AlphaDecoder>
  {
    public:
      //!constructor: in this case windowSize is the alpha parameter of
      //!the kernel function 1/alpha^2 * t * exp(-1/alpha * t), the actual
      //!length of the kernel is determined automatically
      AlphaDecoder(const SimTime& timeStep = SimTime::MSECS(1.0),
                   const SimTime& windowSize = SimTime::MSECS(15.0));
      //!destructor
      ~AlphaDecoder() { };
  
      //!add some data to the decoder
      void push(const double& in); 

      //!get the instantaneous Spikes/Second
      const double getOutput() const;

      //! reset decoder internals
      void reset();

    private:
      std::deque<double> itsSignal;
      std::vector<double> itsKernel;
  };

  // ######################################################################
  // register or types with a factory
  // ######################################################################
  namespace 
  {
    typedef NeuralDecoder::Factory NDFactory;
    typedef NeuralDecoder::Creator NDCreator;
    //define creation functions
    struct RegisterNeuralDecoder
    {
        RegisterNeuralDecoder()
        {
          const SimTime time = SimTime::MSECS(1.0);
          NDFactory::instance().add("HoldDecoder",  NDCreator::make<HoldDecoder>(time));
          NDFactory::instance().add("HistDecoder",  NDCreator::make<HistDecoder>(time));
          NDFactory::instance().add("RectDecoder",  NDCreator::make<RectDecoder>(time));
          NDFactory::instance().add("ExpDecoder",   NDCreator::make<ExpDecoder>(time));
          NDFactory::instance().add("AlphaDecoder", NDCreator::make<AlphaDecoder>(time));
        }
    };
    static RegisterNeuralDecoder registerneuraldecoder;  
  }

}//end namespace
#endif
#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */



