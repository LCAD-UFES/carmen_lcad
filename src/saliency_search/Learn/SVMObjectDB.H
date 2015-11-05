/*!@file Learn/SVMObjectDB.H SVM Object Database module */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://dparks@isvn.usc.edu/software/invt/trunk/saliency/src/Learn/SVMObjectDB.H $
// $Id: SVMObjectDB.H 11135 2009-04-21 20:14:32Z mviswana $
//

#ifndef SVMOBJECTDB_DEFINED
#define SVMOBJECTDB_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Learn/SVMObject.H"
#include <set>
#include <iterator>

#ifdef HAVE_LIBXML

#include <libxml/parser.h>
#include <libxml/tree.h>

#else // fake libxml API so that builds don't break on hosts without it

typedef void* xmlDocPtr ;
typedef void* xmlNodePtr ;

#endif

class SVMObjectDB
{
private:
  std::set<SVMObject> objdb;

public:
  void getNodeContent(xmlDocPtr doc, xmlNodePtr nodePtr,
                        const char* nodeName, std::string &result);
  void loadObjDB(const std::string& filename);
  void writeObjDB(const std::string& filename);
  SVMObject updateObject(const int id, const std::string& name);
  SVMObject newObject(const int id, const std::string& name);
  SVMObject getObject(const int id);
  SVMObject getObject(const std::string& name);
};

#endif
