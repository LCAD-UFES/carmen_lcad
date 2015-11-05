/*!@file Component/ModelOptionDef.H Types for defining command-line options. */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Component/ModelOptionDef.H $
// $Id: ModelOptionDef.H 6117 2006-01-19 23:16:28Z rjpeters $
//

#ifndef MODELOPTIONDEF_H_DEFINED
#define MODELOPTIONDEF_H_DEFINED

#include <typeinfo>

// ######################################################################
//! Model option kinds -- either flag, alias, or argument-requiring option
enum ModelOptionKind {
  MOK_ARG,
  MOK_FLAG,
  MOK_OBSOLETE,
  MOK_ALIAS
};

// ######################################################################
//! ModelOptionType -- represents a ModelOptionKind plus type info
struct ModelOptionType {
  ModelOptionKind kind;
  const std::type_info* argtype; //! type of argument expected, or null
};

/*! @name ModelOptionType initializers */
//@{

//! Builds a ModelOptionType initializer for an option with argument-type T
#define MODOPT_ARG(T) { MOK_ARG, &(typeid(T)) }

//! ModelOptionType initializer for flag options
extern const ModelOptionType MODOPT_FLAG;

//! ModelOptionType initializer for obsolete options
extern const ModelOptionType MODOPT_OBSOLETE;

//! ModelOptionType initializer for alias options
extern const ModelOptionType MODOPT_ALIAS;

//! ModelOptionType initializer for string options
extern const ModelOptionType MODOPT_ARG_STRING;

//@}

// ######################################################################
//! Sort-priority levels for ModelOptionCateg (lower number == higher priority)
enum ModelOptionCategPriority {
  MOC_SORTPRI_1,
  MOC_SORTPRI_2,
  MOC_SORTPRI_3,
  MOC_SORTPRI_4,
  MOC_SORTPRI_5
};

// ######################################################################
//! Holds information about a group of command-line options; used for printing --help message
struct ModelOptionCateg {
  ModelOptionCategPriority sortpriority;
  const char*              description;
};

// NOTE: If you want to create a new option category with a new
// ModelOptionCateg object, then you should add it to a local file
// nearby to where the options are going to be used, e.g. in the same
// source subdirectory (i.e., don't add the new ModelOptionCateg here
// in this file).

extern const ModelOptionCateg MOC_GENERAL; //!< General-use options
extern const ModelOptionCateg MOC_ALIAS;   //!< Option aliases

// ######################################################################
//! What to export when calling ModelComponent::exportOptions()
//@{

//! Export no option
const int OPTEXP_NONE =       0;

//! Export all options
const int OPTEXP_ALL =        0xffff;

/*! Export our core configuration options that do not require any other
    model component to be present in the model */
const int OPTEXP_CORE =      (1 << 0);

/*! Export our options that relate to saving results, and require that
  an OutputFrameSeries be available (and used!) in the model */
const int OPTEXP_SAVE =      (1 << 1);

/*! Export options that relate to the MbariResultViewer */
const int OPTEXP_MRV =       (1 << 2);

//@}

// ######################################################################
//! Specification of a known model option
struct ModelOptionDef
{
  ModelOptionType type;    //!< Type of the option
  const char* name;        //!< Option name
  const ModelOptionCateg* categ; //!< Option category
  const int exportFlag;    //!< one of the what-to-export flags
  const char* descr;       //!< Description of what option does
  const char* longoptname; //!< Long option name (without leading "--")
  char shortoptname;       //!< Short option name (without leading "-")
  const char* validvals;   //!< Textual description of valid values, or NULL
  const char* defval;      //!< Default value in text representation
};

#endif // !MODELOPTIONDEF_H_DEFINED

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
