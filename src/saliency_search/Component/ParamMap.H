/*!@file Component/ParamMap.H I/O operations for parameter files */

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
// Primary maintainer for this file: Rob Peters <rjpeters@klab.caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Component/ParamMap.H $
// $Id: ParamMap.H 14376 2011-01-11 02:44:34Z pez $
//

#ifndef PARAMMAP_H_DEFINED
#define PARAMMAP_H_DEFINED

#include "rutz/shared_ptr.h"
#include "Util/Types.H"

#include <iosfwd>
#include <vector>

// on some platforms sys/types.h must be included for 'uint' type.
#ifdef MACHINE_OS_DARWIN
#include <sys/types.h>
#endif


//! Allows reading and writing of parameters to and from files or streams
/*! ParamMap allows reading and writing of arbitrarily nested data structes
  to and from files or other streams. */
class ParamMap
{
  ParamMap(const ParamMap&); // not allowed
  ParamMap& operator=(const ParamMap&); // not allowed

public:
  //! Convenience function to create a new ParamMap from a .pmap file.
  static rutz::shared_ptr<ParamMap> loadPmapFile(const std::string& fname);

  //! Compatibility function to create a new ParamMap from a .conf file.
  /*! See readConfig for details on the format of a .conf file. Basically,
      it is similar to the .pmap format, except that comments must be
      bracketed by whitespace-separated '#' characters on both sides, and
      that there is no nesting of data structures. */
  static rutz::shared_ptr<ParamMap> loadConfFile(const std::string& fname);

  //! Convenience function to create a new ParamMap from an equivalent
  // map
  //template <class T>
  //  static rutz::shared_ptr<ParamMap> loadC_Map(const std::map <std::string, T>);

  //! Return codes when getting params from the map.
  enum ReturnCode
    {
      MISSING, //!< the param was not found
      UNCHANGED, //!< the param was found, but the value was unchanged
      CHANGED, //!< the param was found and its value was changed
    };

  //! Default construct with an empty map.
  ParamMap();

  //! Construct and load() from \a fname.
  ParamMap(const std::string& fname);

  //! Destructor.
  ~ParamMap();

  //! A class for iterating over the ParamMap's keys (i.e. param names).
  class key_iterator;

  //! Get an iterator to the beginning of the keys sequence.
  key_iterator keys_begin() const;

  //! Get an iterator to one-past-the-end of the keys sequence.
  key_iterator keys_end() const;

  //! Load in a parameter map from the given input stream.
  void load(std::istream& istrm);

  //! Convenience overload of the other load().
  /*! In this case we just build an istream internally from the given
      filename, and then load() from that.*/
  void load(const std::string& fname);

  //! Write a formatted parameter map to the given output stream.
  /*! The indentlev parameter is only used internally by ParamMap, and can
      be safely ignored by external callers. */
  void format(std::ostream& ostrm, int indentlev = 0) const;

  //! Convenience overload of the other format().
  /*! In this case we just build an ostream internally from the given
      filename, and then format() from that.*/
  void format(const std::string& fname) const;

  //! Query whether the parameter map has a particular named parameter.
  bool hasParam(const std::string& paramname) const;

  //! Query whether the parameter map is a leaf or not
  bool isLeaf(const std::string& paramname) const;

  uint getsize() const;
  // ######################################################################
  /*! @name Functions to extract named values from the ParamMap

      There are several kinds of functions that differ in how they handle
      named parameters that are not found in the ParamMap. The first kind
      look for a named parameter, and trigger and error if that parameter
      is not found. The second kind take a default value as an additional
      argument, and return that value if the named parameter is not
      found. The third kind take a destination by reference, and leave that
      destination unchanged if the parameter is not found, and additional
      return a ReturnCode specifying with the parameter was missing,
      found-and-changed, or found-and-unchanged. */
  //@{

  //! Get a named parameter as a nested parameter map.
  /*! Triggers an error if there is no parameter with the given name. */
  rutz::shared_ptr<ParamMap> getSubpmap(const std::string& paramname) const;

  //! Get a named parameter as a string value.
  /*! Triggers an error if there is no parameter with the given name. */
  std::string getStringParam(const std::string& paramname) const;

  //! Get a named parameter as a 'double' value.
  /*! Triggers an error if there is no parameter with the given name. */
  double getDoubleParam(const std::string& paramname) const;

  //! Get a named parameter as an 'int' value.
  /*! Triggers an error if there is no parameter with the given name. */
  int getIntParam(const std::string& paramname) const;

  //! Get a named parameter as a string value, or return the default value.
  /*! The default value will be returned if the named parameter is not
    found.*/
  std::string getStringParam(const std::string& paramname, const std::string& defval) const;

  //! Get a named parameter as a 'double' value, or return the default value.
  /*! The default value will be returned if the named parameter is not
    found.*/
  double getDoubleParam(const std::string& paramname, const double defval) const;

  //! Get a named parameter as an 'int' value, or return the default value.
  /*! The default value will be returned if the named parameter is not
    found.*/
  int getIntParam(const std::string& paramname, const int defval) const;

  //! Returns the submap for the given name.
  /*! If a submap for the given name exists, a rutz::shared_ptr to it
      is returned. If no submap for the given name has yet been
      created, one is created, and a rutz::shared_ptr to it is
      returned. This function differs from getSubpmap(), which only
      returns existing submaps. */
  rutz::shared_ptr<ParamMap> lookupSubpmap(const std::string& paramname);

  //! Get the named parameter, or do nothing if the parameter is missing.
  /*! The ReturnCode describes what happened when looking up the param. */
  ReturnCode queryStringParam(const std::string& paramname, std::string& result) const;

  //! Get the named parameter, or do nothing if the parameter is missing.
  /*! The ReturnCode describes what happened when looking up the param. */
  ReturnCode queryDoubleParam(const std::string& paramname, double& result) const;

  //! Get the named parameter, or do nothing if the parameter is missing.
  /*! The ReturnCode describes what happened when looking up the param. */
  ReturnCode queryIntParam(const std::string& paramname, int& result) const;

  //@}

  // ######################################################################
  /*! @name Functions to place named values into the ParamMap */

  //! Add a nested parameter map with a given name.
  void putSubpmap(const std::string& paramname, const rutz::shared_ptr<ParamMap>& val);

  //! Add a string value with a given name.
  void putStringParam(const std::string& paramname, const std::string& val);

  //! Add a double-precision floating point value with a given name.
  void putDoubleParam(const std::string& paramname, double val);

  //! Add an integral value with a given name.
  void putIntParam(const std::string& paramname, int val);

  //! Replace a submap
  void replaceSubpmap(const std::string& paramname, const rutz::shared_ptr<ParamMap>& val);

  //! Replace string-param
  void replaceStringParam(const std::string& paramname, const std::string& val);

  //! Replace double-param
  void replaceDoubleParam(const std::string& paramname, double val);

  //! Replace int-param
  void replaceIntParam(const std::string& paramname, int val);

  //! Remove all params from the ParamMap
  void clear();

  //! Remove a single param from the ParamMap
  void erase(const std::string& paramname);

  //@}

  //! For testing/debugging only.
  /*! Prints the raw contents of this parameter map to standard output. */
  void print(const std::string& name) const;

private:
  struct Impl;
  Impl* const rep;
};

class ParamMap::key_iterator
{
public:
  //! Destructor.
  ~key_iterator();
  //! Copy constructor.
  key_iterator(const key_iterator& other);
  //! Assignment operator.
  key_iterator& operator=(const key_iterator&);

  //! Dereference to get the key (i.e., parameter name).
  const std::string& operator*() const;

  //! Pre-increment operator.
  key_iterator& operator++();

  //! Equality comparison.
  bool operator==(const key_iterator& other) const;

  //! Inequality comparison.
  bool operator!=(const key_iterator& other) const;

private:
  void swap(key_iterator& other);

  //! Default construct.
  key_iterator();

  friend class ParamMap;

  struct IterRep;
  IterRep* const rep;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !PARAMMAP_H_DEFINED
