/*!@file ModelNeuron/Location.H Class to represent a location in a
   StrucureModule or Layer. */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/Location.H $


#ifndef MODELNEURON_LOCATION_H_DEFINED
#define MODELNEURON_LOCATION_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include "Util/log.H"
#include <vector>

namespace nsu
{
  // ######################################################################
  /* a class to represent a N-D location in a Structure or Layer
   
     This class represents an N-D location stored as an integer. One of the dimensions can be
     'Location::ALL', which will indicate that we want to represent all elements
     along that dimension. 

  */
  // ######################################################################
  class Location 
  {
    public:
      //to represent every location
      enum {ALL = -1, NONE = -2};
  
      //!default constructor
      Location();
  
      //!set Location for various dimensions from constructor
      template <typename... NDPoint>
      Location(const NDPoint... dims);
  
      //!set Location for from a vector
      Location(const std::vector<int>& pos);
    
      //!destructor
      ~Location();
  
      //!get the number of dimensions
      const size_t dims() const;
  
      //!get the value at N'th pos
      const int getDim(const uint pos) const;
  
      //!set/reset the position of the location
      template <typename... Dims>
      void setLocation(const Dims... dims);

      //!set Location for from a vector
      void setLocation(const std::vector<int>& pos);

      //!set Location for from a vector, but not the dimensions
      void setLocation(const Location& loc);
  
      //!get the positions in a linear format
      const std::vector<int>& getLinearPos() const;

      //!get the positions in a linear format given the dimensions of the hypercube
      //!in which the location exists
      template <typename... Dims>
      const std::vector<int>& getLinearPos(const Dims... dims);

      //!set the dimensions ahead of time to precompute positions
      template <typename... Dims>
      void setHyperCubeDims(const Dims... dims);

      //!set the dimensions ahead of time to precompute positions
      void setHyperCubeDims(const std::vector<int>& hypercubedims);

      //are we equal?
      bool operator==(const Location& rhs);  
    private:
      //compute the linear position given the dimensions
      void computePos();

      //add the dims
      template <typename Head, typename... Tail> 
      bool add(std::vector<int>& vector, const bool allowFreeDim, uint count, const Head head, const Tail... tail);
      bool add(std::vector<int>& vector, const bool allowFreeDim, uint count) { return false; };//base case in recursion

      //add the dims as a vector
      bool addVector(std::vector<int>& vector, const bool allowFreeDim, const std::vector<int>& dims);

      //assign, and return true if there was a change
      bool assign(const std::vector<int>& src, std::vector<int>& dst);

      std::vector<int> itsLocation, itsHyperCube, itsLinearPos;
      int itsFreeDim;
  };

  // ######################################################################
  // implementation of location
  // ######################################################################
  inline
  Location::Location() :itsLocation(), itsHyperCube(), itsLinearPos(), itsFreeDim(Location::NONE)
  { }

  // ######################################################################
  template <typename... Dims> inline
  Location::Location(const Dims... dims) : itsLocation(), itsHyperCube(), itsLinearPos(), itsFreeDim(Location::NONE)
  { 
    add(itsLocation, true, 0, dims...);
  }

  // ######################################################################
  inline
  Location::Location(const std::vector<int>& pos) : itsLocation(), itsHyperCube(), itsLinearPos(), itsFreeDim(Location::NONE)
  {
    addVector(itsLocation, true, pos);
  }

  // ######################################################################
  inline
  bool Location::operator==(const Location& rhs) 
  { 
    return ( (itsLocation == rhs.itsLocation) && 
             (itsHyperCube == rhs.itsHyperCube) && 
             (itsLinearPos == rhs.itsLinearPos) );
  };

  // ######################################################################
  inline
  Location::~Location() { };

  // ######################################################################
  inline
  const size_t Location::dims() const { return itsLocation.size(); }

  // ######################################################################
  inline
  const int Location::getDim(const uint pos) const {return itsLocation[pos]; }

  // ######################################################################
  template <typename... Dims> inline
  void Location::setLocation(const Dims... dims)
  {
    itsLocation.resize(0);
    itsFreeDim = Location::NONE;
    if (add(itsLocation, true, 0, dims...) && (itsLocation.size() > 0) && (itsHyperCube.size() > 0))
      computePos();
  }

  // ######################################################################
  inline
  void Location::setLocation(const std::vector<int>& pos)
  {
    itsFreeDim = Location::NONE;
    if (addVector(itsLocation, true, pos) && (itsLocation.size() > 0) && (itsHyperCube.size() > 0))
      computePos();
  }

  // ######################################################################
  inline
  void Location::setLocation(const Location& loc)
  {
    itsFreeDim = loc.itsFreeDim;
    if (assign(loc.itsLocation, itsLocation) && (itsLocation.size() > 0) && (itsHyperCube.size() > 0))
      computePos();
  }

  // ######################################################################
  inline
  const std::vector<int>& Location::getLinearPos() const
  {
    return itsLinearPos;
  }

  // ######################################################################
  template <typename... Dims> inline
  const std::vector<int>& Location::getLinearPos(const Dims... dims)
  {
    setHyperCubeDims(dims...);
    return itsLinearPos;
  }

  // ######################################################################
  template <typename... Dims> inline
  void Location::setHyperCubeDims(const Dims... dims)
  {
    std::vector<int> temp;
    add(temp, false, 0, dims...); 
    if (assign(temp, itsHyperCube) && (itsLocation.size() > 0) && (itsHyperCube.size() > 0))
      computePos();
  }

  // ######################################################################
  inline
  void Location::setHyperCubeDims(const std::vector<int>& hypercubedims)
  {
    if (assign(hypercubedims,itsHyperCube) && (itsLocation.size() > 0) && (itsHyperCube.size() > 0))
      computePos();
  }

  // ######################################################################
  inline
  void Location::computePos()
  {
    itsLinearPos.clear();
    const int dsize = (int)itsHyperCube.size();
    const int psize = (int)itsLocation.size();

    //make sure everything has the right dimensions  
    const int diff = dsize - psize;  
    const uint shortest = (diff < 0) ? dsize : psize;

    const bool invalid = (diff < 0) ? diff * -1 > 2 : diff  > 2;
    if (invalid || ( (diff > 0) && (itsFreeDim != Location::NONE) ))
      LFATAL("HyperCube dimensions and location position mismatch");
  
    //get cumulative product of dimensions 
    uint dims[dsize]; dims[0] = 1;
    for (int c = 0; c < dsize-1; ++c)
      dims[c+1] = itsHyperCube[c]*dims[c];
  
    //if the point is fewer dims than the space it lives in, just travel along the dims in common/
    // if the point has more dims than the space it lives in, the cut the extra dims
    if ( (itsFreeDim < 0) || (itsFreeDim >= (int)itsHyperCube.size()) )//no freedims
    {
      uint pos = 0;
      for (uint c = 0; c < shortest; ++c)
        if (itsLocation[c] < itsHyperCube[c])
          pos += itsLocation[c]*dims[c];
        else 
          LFATAL("Location out of dimensions");
      
      itsLinearPos.push_back(pos);
    }
    else 
    {
      //loop over all positions in that dimension
      for (int i = 0; i < itsHyperCube[itsFreeDim]; ++i)
      {
        //convert everything to 1d representation
        uint pos = 0;
        for (uint c = 0; c < shortest; ++c)
          if (itsLocation[c] < itsHyperCube[c])
            pos += (c == (uint)itsFreeDim) ? i*dims[c] : itsLocation[c]*dims[c];
          else 
            LFATAL("Location out of dimensions");
          
        itsLinearPos.push_back(pos);
      }
    }    
  }

  // ######################################################################
  inline
  bool Location::addVector(std::vector<int>& vector, const bool allowFreeDim, const std::vector<int>& dims)
  {
    bool change = false;
    std::vector<int>::const_iterator iter(dims.begin());
    const uint end = vector.size();
    uint count = 0;
    while (iter != dims.end())
    {
      //fail if out of range
      if (!allowFreeDim && (*iter < 0) && (*iter > -2))
        LFATAL("cannot have negative dimensions.");
      
      //make sure we are not aleady set a free dim before setting a matched one
      if (*iter == Location::ALL) 
      {
        if (itsFreeDim != Location::NONE)
          LFATAL("cannot have more than one dimension in the Location as free.");
        
        itsFreeDim = count;
      }
      
      if (count < end) 
      {
        //if we are an existing vector check for equality
        if (vector[count] != *iter)
        {
          change = true;
          vector[count] = *iter;
        }
      }
      else 
      {
        //otherwise push back a new item
        vector.push_back(*iter);
        change = true;
      }
      
      ++count;
      ++iter;
    }
    return change;
  }

  // ######################################################################
  template <typename Head, typename... Tail> inline
  bool Location::add(std::vector<int>& vector, const bool allowFreeDim, uint count, const Head head, const Tail... tail)
  {
    //check for validity of value
    if (!allowFreeDim && ((int)head == Location::ALL))
      LFATAL("cannot have a free dimension.");
  
    //check for free dims
    if ((int)head == Location::ALL)
    {
      if (itsFreeDim == Location::NONE)
        itsFreeDim = count;
      else
        LFATAL("cannot have more than one dimension in the Location as free.");
    }

    bool change = false;
    if (count < vector.size()) 
    {
      //if we are an existing vector, check if the values are equal
      if (vector[count] != (int)head) 
      {
        change = true;
        vector[count] = head;
      }
    }
    else 
    {
      //else push it to the back
      vector.push_back(head);
      change = true;
    }

    const bool didchange = add(vector, allowFreeDim, ++count, tail...);
    return (didchange) ? true : change;
  }

  // ######################################################################
  inline
  bool Location::assign(const std::vector<int>& src, std::vector<int>& dst)
  {
    bool change = false;
    if (src.size() != dst.size())
    {
      dst = src;
      change = true;
    }
    else
    {
      std::vector<int>::const_iterator srciter(src.begin());
      std::vector<int>::iterator dstiter(dst.begin());
      while (srciter != src.end())
      {
        if (*srciter != *dstiter)
          change = true;
          
        *dstiter++ = *srciter++;
      }
    }
    return change;
  }

  // ######################################################################
  // free functions
  // ######################################################################
  //! format is "<int>,<int>"
  std::string convertToString(const Location& val);

  //! format is "<int>,<int>"
  void convertFromString(const std::string& str, Location& val);
}
#endif
#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
