/*!@file ModelNeuron/NeuralColumn.H Class declaration for a neural
   column, which is a collection of neural simulation units that can
   interact in an arbitrary connection pattern. */

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
// $HeadURL:svn://ilab.usc.edu/trunk/saliency/src/ModelNeuron/NeuralColumn.H$

#ifndef MODELNEURON_NEURALCOLUMN_H_DEFINED
#define MODELNEURON_NEURALCOLUMN_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include "ModelNeuron/NeuralSimUtils.H"
#include "ModelNeuron/SimUnit.H"

namespace nsu
{
  // ######################################################################
  //NeuralColumn
  //######################################################################
  //! A container for a column of SimUnits. The user can specify
  // an arbitrary connection pattern between modules. The output will be 
  // a weighted sum of all units with output weights (see addWeight(),
  // below). Instead of overriding doIntegrate in derived modules, one 
  // can add column specific operations by overriding doExtraIntegrate. 
  // One can also override combineOutput to combine outputs from different
  // modules in a way other than a weighted sum. 
  //######################################################################
  template<class T>
  class NeuralColumn : public SimUnit
  {
      typedef SimUnit::RateType RateType;

    public:
      enum {IO = -1};

      //!constructor
      NeuralColumn(const SimTime& timestep, const RateType ratetype = SimUnit::NORMAL,
                   const std::string& name = "", const std::string& units = "");
  
      //!destructor
      virtual ~NeuralColumn();

      //!get the number of sub units
      const uint numSubs() const;
  
      //!get sub unit const version
      const SimUnit& getSub(const uint i) const;

      //!add a subunit, making several copies if desired
      void addSub(const SimUnit& sub, uint repeat = 1);

      //!adds a weighted connection between two units. Use -1 as the
      //!'from' argument to accept inputs to a unit. Use -1 as the 'to'
      //!argument to contributed to the weighted summed output
      void addWeight(const int from, const int to, const float weight);

    protected:
      //!get sub unit const version
      SimUnit& editSub(const uint i);


    private:
      struct Weights
      {
          int from, to;
          float str;
      };

      typedef typename nsu::vector<T> vector;
      typedef typename std::vector<Weights> weights_vector;
      typedef typename nsu::vector<T>::Type cast_type;

      //! integrate our internals a single time step (in msecs)
      const double doIntegrate(const SimTime& dt, const double& exc, const double& inh);  

      //!integrate any dynamics of our container a single time step (in msecs)
      virtual void doExtraIntegrate(const SimTime& dt, const double& exc, const double& inh);

      //!combine outputs of submodules for system output, default is to multiply paramteter 
      //! by value and add to total (weighted sum of submodules). 
      virtual void combineOutput(double& total, const double& value, const double& parameter);

      //!apply any post processing. Default returns input
      virtual double postProcess(const double& in);
    
      //! perform any additional initialization or reset
      void doInit();  
  
      //!perform a deep copy
      NeuralColumn* doClone() const;

      //! private implementations of addSub, where 'true' and 'false'
      //! indicate if the type is polymorphic.
      void addSub(const SimUnit& sub, nsu::Int2Type<false>);
      void addSub(const SimUnit& sub, nsu::Int2Type<true>);

      //!hold our units
      vector itsM;  

      //!weights
      weights_vector itsInputW, itsW, itsOutputW;
  };

  // ######################################################################
  //! implementation of NeuralColumn functions
  // ######################################################################
  template<class T> inline
  NeuralColumn<T>::NeuralColumn(const SimTime& timestep, 
                                const RateType ratetype,
                                const std::string& name, 
                                const std::string& units) :
      SimUnit(timestep, ratetype, name, units), itsM(), 
      itsInputW(), itsW(), itsOutputW()
  { }

  // ######################################################################
  template<class T> inline
  NeuralColumn<T>::~NeuralColumn()
  { }
  
  // ######################################################################
  template<class T> inline 
  const uint NeuralColumn<T>::numSubs() const
  {
    return (uint)itsM.size();
  }
  
  // ######################################################################
  template<class T> inline
  const SimUnit& NeuralColumn<T>::getSub(const uint i) const
  {
    if (i < itsM.size())
    {
      return itsM[i];
    }
    else
    {
      LFATAL("no units added to this column");
      return itsM[0];//will never execute
    }
  }

  // ######################################################################
  template<class T> inline
  SimUnit& NeuralColumn<T>::editSub(const uint i)
  {
    if (i < itsM.size())
    {
      return itsM[i];
    }
    else
    {
      LFATAL("no units added to this column");
      return itsM[0];//will never execute
    }
  }

  // ######################################################################
  template <class T> inline
  void NeuralColumn<T>::addSub(const SimUnit& sub, uint repeat)
  {
    for (uint i = 0; i < repeat; ++i)
      addSub(sub, nsu::Int2Type<nsu::vector<T>::isPointer >());
  }

  // ######################################################################
  template <class T> inline
  void NeuralColumn<T>::addSub(const SimUnit& sub, nsu::Int2Type<false>)
  {
    const cast_type temp = dynamic_cast<const cast_type&>(sub);
    itsM.push_back(temp);
  }

  // ######################################################################
  template <class T> inline
  void NeuralColumn<T>::addSub(const SimUnit& sub, nsu::Int2Type<true>) 
  {
    itsM.push_back(sub);
  }

  // ######################################################################
  template <class T> inline
  void NeuralColumn<T>::addWeight(const int from, const int to, 
                                  const float weight)
  {
    if ( (from == to) && (from == IO) )
      LFATAL("It doesn't make since to map inputs directly to outputs");
  
    if ( (from < (int)itsM.size()) && (to < (int)itsM.size()) )
    {
      Weights temp = {from, to, weight};
      if (to == IO)
        itsOutputW.push_back(temp);
      else if (from == IO)
        itsInputW.push_back(temp);
      else
        itsW.push_back(temp);
    }
    else
      LFATAL("A weight between these two modules cannot be added as at least "
             "one of them is out of range.");
  }

  // ######################################################################
  template <class T> inline
  const double NeuralColumn<T>::doIntegrate(const SimTime& dt, 
                                            const double& exc, const double& inh)
  {
    //add the inputs
    typename weights_vector::const_iterator iter(itsInputW.begin());
    while (iter != itsInputW.end())
    {
      itsM[iter->to].inputExc( exc * (iter)->str );
      itsM[iter->to].inputInh( inh * (iter)->str );
      ++iter;
    }

    //handle columnar interactions
    iter = itsW.begin();
    while (iter != itsW.end())
    {
      itsM[iter->to].input( itsM[iter->from].getOutput() * iter->str );
      ++iter;
    }

    //do any derived class specific integration
    doExtraIntegrate(dt, exc, inh);
  
    //integrate submodules
    typename vector::iterator ii(itsM.begin()), end(itsM.end());
    while (ii != end)  
      (ii++)->evolve(getTime());
  
    //set output
    double out = 0.0;
    iter = itsOutputW.begin();
    while (iter != itsOutputW.end())
    {
      combineOutput(out, itsM[(iter)->from].getOutput(), iter->str);
      ++iter;
    }  
    return postProcess(out);
  }

  // ######################################################################
  template <class T> inline
  void NeuralColumn<T>::doExtraIntegrate(const SimTime& dt, const double& exc, const double& inh)
  { }

  // ######################################################################
  template <class T> inline
  void NeuralColumn<T>::combineOutput(double& total, const double& value, const double& parameter)
  {
    total += value * parameter;
  }

  // ######################################################################
  template <class T> inline
  double NeuralColumn<T>::postProcess(const double& in)
  {
    return in;
  }

  // ######################################################################
  template <class T> inline
  void NeuralColumn<T>::doInit()
  {
    itsInputW.clear();
    itsW.clear();
    itsOutputW.clear();

    typename vector::iterator ii(itsM.begin()), end(itsM.end());
    while (ii != end)
      (ii++)->initialize();
  }

  // ######################################################################
  template <class T> inline 
  NeuralColumn<T>* NeuralColumn<T>::doClone() const
  {
    return new NeuralColumn(*this);
  }
}

#endif
#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
