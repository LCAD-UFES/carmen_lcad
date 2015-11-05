/*! create objects from a ParamList */
//////////////////////////////////////////////////////////////////////////
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
// See http://iLab.usc.edu for information about this project.          //
//////////////////////////////////////////////////////////////////////////
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
//////////////////////////////////////////////////////////////////////////
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
//////////////////////////////////////////////////////////////////////////
//
// Primary maintainer for this file: David J. Berg <dberg@usc.edu>
// $HeadURL:svn://ilab.usc.edu/trunk/saliency/src/GenericUtils/ObjCreator.H$

#ifndef UTIL_OBJCREATOR_H_DEFINED
#define UTIL_OBJCREATOR_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include "GenericUtils/ParamList.H"
//######################################################################
//ObjCreator - create an object from a paramlist
//
//Usage:
//
//  //create an object from a paramlist
//  ParamList<float, int> pl(4.4f, 5);
//  auto pl1 = make_paramlist(1.0f, 0);
//  Circle * c = ObjCreator<Circle>::create(pl);
//  LINFO("%3.2f, %d", c->f, c->p);
//  c = ObjCreator<Circle>::create(pl1);
//  LINFO("%3.2f, %d", c->f, c->p);
//
// Note: see interesting implementation note below
//######################################################################
template <class T, class ConvertType = T>
class ObjCreator
{
  public:
    template <typename... Params>
    static T* create(const ParamList<Params...>& paramlist)
    {
      //if all the types in the paramlist are guarnteed to be different 
      //then this single line:
      //
      //T* t = new T(at<Params>(paramlist)...);
      //
      //replaces all the nasty specializations we have below. It works
      //great. Unfortunatly, the at<> function in ParamList.H will
      //always match the first type, so if there are repeats we won't
      //get the correct parameters. I can't seem to come up with any
      //reasonable tricks to get the expansion to keep track of who its
      //matched.... One could still make it work if they typeified there
      //types so that each was unique, but this is a whole other pain I
      //don't want to deal with.
      T* t = creator<ParamList<Params...>::length, T>::create(paramlist);
      ConvertType* ct = dynamic_cast<ConvertType*>(t);
      return (ct == NULL) ? NULL : ct;
    };
  
  private:
    //This private class does all the real work and ObjCreator just
    //serves as an interface. The parameters stored in the ParamList are
    //passed to T's constructor. Only the specializations that
    //corresponds to the correct param list length will get compiled.
    template <uint num, class U> struct creator;
  
    template <class U> struct creator<0, U> {
        template<typename... Params>
        static U* create(const ParamList<Params...>& pl)
        { return new U(); }; };
  
    template <class U> struct creator<1, U> {
        template<typename... Params>
        static U* create(const ParamList<Params...>& pl)
        {return new U(ParamListHelper::at<0>(pl)); }; };
  
    template <class U> struct creator<2, U> {
        template<typename... Params>
        static U* create(const ParamList<Params...>& pl)
        { using namespace ParamListHelper;
          return new U(at<0>(pl),at<1>(pl)); }; };
  
    template <class U> struct creator<3, U> {
        template<typename... Params>
        static U* create(const ParamList<Params...>& pl)
        { using namespace ParamListHelper;
          return new U(at<0>(pl),at<1>(pl),at<2>(pl)); }; };
  
    template <class U> struct creator<4, U> {
        template<typename... Params>
        static U* create(const ParamList<Params...>& pl)
        { using namespace ParamListHelper;
          return new U(at<0>(pl),at<1>(pl),at<2>(pl),at<3>(pl)); }; };
  
    template <class U> struct creator<5, U> {
        template<typename... Params>
        static U* create(const ParamList<Params...>& pl)
        { using namespace ParamListHelper;
          return new U(at<0>(pl),at<1>(pl),at<2>(pl),at<3>(pl),at<4>(pl)); }; };

    template <class U> struct creator<6, U> {
        template<typename... Params>
        static U* create(const ParamList<Params...>& pl)
        { using namespace ParamListHelper;
          return new U(at<0>(pl),at<1>(pl),at<2>(pl),at<3>(pl),at<4>(pl),
                       at<5>(pl)); }; };

    template <class U> struct creator<7, U> {
        template<typename... Params>
        static U* create(const ParamList<Params...>& pl)
        { using namespace ParamListHelper;
          return new U(at<0>(pl),at<1>(pl),at<2>(pl),at<3>(pl),at<4>(pl),
                       at<5>(pl), at<6>(pl)); }; };

    template <class U> struct creator<8, U> {
        template<typename... Params>
        static U* create(const ParamList<Params...>& pl)
        { using namespace ParamListHelper;
          return new U(at<0>(pl),at<1>(pl),at<2>(pl),at<3>(pl),at<4>(pl),
                       at<5>(pl),at<6>(pl),at<7>(pl)); }; };

    template <class U> struct creator<9, U> {
        template<typename... Params>
        static U* create(const ParamList<Params...>& pl)
        { using namespace ParamListHelper;
          return new U(at<0>(pl),at<1>(pl),at<2>(pl),at<3>(pl),at<4>(pl),
                       at<5>(pl),at<6>(pl),at<7>(pl),at<8>(pl)); }; };

    template <class U> struct creator<10, U> {
        template<typename... Params>
        static U* create(const ParamList<Params...>& pl)
        { using namespace ParamListHelper;
          return new U(at<0>(pl),at<1>(pl),at<2>(pl),at<3>(pl),at<4>(pl),
                       at<5>(pl),at<6>(pl),at<7>(pl),at<8>(pl),at<9>(pl)); }; };

    template <class U> struct creator<11, U> {
        template<typename... Params>
        static U* create(const ParamList<Params...>& pl)
        { using namespace ParamListHelper;
          return new U(at<0>(pl),at<1>(pl),at<2>(pl),at<3>(pl),at<4>(pl),
                       at<5>(pl),at<6>(pl),at<7>(pl),at<8>(pl),at<9>(pl),
                       at<10>(pl)); }; };

    template <class U> struct creator<12, U> {
        template<typename... Params>
        static U* create(const ParamList<Params...>& pl)
        { using namespace ParamListHelper;
          return new U(at<0>(pl),at<1>(pl),at<2>(pl),at<3>(pl),at<4>(pl),
                       at<5>(pl),at<6>(pl),at<7>(pl),at<8>(pl),at<9>(pl),
                       at<10>(pl),at<11>(pl)); }; };

    template <class U> struct creator<13, U> {
        template<typename... Params>
        static U* create(const ParamList<Params...>& pl)
        { using namespace ParamListHelper;
          return new U(at<0>(pl),at<1>(pl),at<2>(pl),at<3>(pl),at<4>(pl),
                       at<5>(pl),at<6>(pl),at<7>(pl),at<8>(pl),at<9>(pl),
                       at<10>(pl),at<11>(pl),at<12>(pl)); }; };

    template <class U> struct creator<14, U> {
        template<typename... Params>
        static U* create(const ParamList<Params...>& pl)
        { using namespace ParamListHelper;
          return new U(at<0>(pl),at<1>(pl),at<2>(pl),at<3>(pl),at<4>(pl),
                       at<5>(pl),at<6>(pl),at<7>(pl),at<8>(pl),at<9>(pl),
                       at<10>(pl),at<11>(pl),at<12>(pl),at<13>(pl)); }; };


    template <class U> struct creator<15, U> {
        template<typename... Params>
        static U* create(const ParamList<Params...>& pl)
        { using namespace ParamListHelper;
          return new U(at<0>(pl),at<1>(pl),at<2>(pl),at<3>(pl),at<4>(pl),
                       at<5>(pl),at<6>(pl),at<7>(pl),at<8>(pl),at<9>(pl),
                       at<10>(pl),at<11>(pl),at<12>(pl),at<13>(pl),at<14>(pl)); }; };

    template <class U> struct creator<16, U> {
        template<typename... Params>
        static U* create(const ParamList<Params...>& pl)
        { using namespace ParamListHelper;
          return new U(at<0>(pl),at<1>(pl),at<2>(pl),at<3>(pl),at<4>(pl),
                       at<5>(pl),at<6>(pl),at<7>(pl),at<8>(pl),at<9>(pl),
                       at<10>(pl),at<11>(pl),at<12>(pl),at<13>(pl),at<14>(pl), 
                       at<15>(pl)); }; };

    template <class U> struct creator<17, U> {
        template<typename... Params>
        static U* create(const ParamList<Params...>& pl)
        { using namespace ParamListHelper;
          return new U(at<0>(pl),at<1>(pl),at<2>(pl),at<3>(pl),at<4>(pl),
                       at<5>(pl),at<6>(pl),at<7>(pl),at<8>(pl),at<9>(pl),
                       at<10>(pl),at<11>(pl),at<12>(pl),at<13>(pl),at<14>(pl), 
                       at<15>(pl),at<16>(pl)); }; };

    template <class U> struct creator<18, U> {
        template<typename... Params>
        static U* create(const ParamList<Params...>& pl)
        { using namespace ParamListHelper;
          return new U(at<0>(pl),at<1>(pl),at<2>(pl),at<3>(pl),at<4>(pl),
                       at<5>(pl),at<6>(pl),at<7>(pl),at<8>(pl),at<9>(pl),
                       at<10>(pl),at<11>(pl),at<12>(pl),at<13>(pl),at<14>(pl), 
                       at<15>(pl),at<16>(pl),at<17>(pl)); }; };

    template <class U> struct creator<19, U> {
        template<typename... Params>
        static U* create(const ParamList<Params...>& pl)
        { using namespace ParamListHelper;
          return new U(at<0>(pl),at<1>(pl),at<2>(pl),at<3>(pl),at<4>(pl),
                       at<5>(pl),at<6>(pl),at<7>(pl),at<8>(pl),at<9>(pl),
                       at<10>(pl),at<11>(pl),at<12>(pl),at<13>(pl),at<14>(pl), 
                       at<15>(pl),at<16>(pl),at<17>(pl),at<18>(pl)); }; };

    template <class U> struct creator<20, U> {
        template<typename... Params>
        static U* create(const ParamList<Params...>& pl)
        { using namespace ParamListHelper;
          return new U(at<0>(pl),at<1>(pl),at<2>(pl),at<3>(pl),at<4>(pl),
                       at<5>(pl),at<6>(pl),at<7>(pl),at<8>(pl),at<9>(pl),
                       at<10>(pl),at<11>(pl),at<12>(pl),at<13>(pl),at<14>(pl), 
                       at<15>(pl),at<16>(pl),at<17>(pl),at<18>(pl),at<19>(pl)); }; };

    template <class U> struct creator<21, U> {
        template<typename... Params>
        static U* create(const ParamList<Params...>& pl)
        { using namespace ParamListHelper;
          return new U(at<0>(pl),at<1>(pl),at<2>(pl),at<3>(pl),at<4>(pl),
                       at<5>(pl),at<6>(pl),at<7>(pl),at<8>(pl),at<9>(pl),
                       at<10>(pl),at<11>(pl),at<12>(pl),at<13>(pl),at<14>(pl), 
                       at<15>(pl),at<16>(pl),at<17>(pl),at<18>(pl),at<19>(pl), at<20>(pl)); }; };
};

#endif //INVT_USE_CPP11
#endif//UTIL_OBJCREATOR_H_DEFINED
