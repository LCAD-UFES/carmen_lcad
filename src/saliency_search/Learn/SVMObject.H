/*!@file Learn/SVMObject.H SVM Object module */

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
// $HeadURL: svn://dparks@isvn.usc.edu/software/invt/trunk/saliency/src/Learn/SVMObject.H $
// $Id: SVMObject.H 13406 2010-05-14 22:25:05Z dparks $
//


#ifndef SVMOBJECT_H_DEFINED
#define SVMOBJECT_H_DEFINED

class SVMObject
{
public:
  int id;
  std::string name;
  std::string description;
  double confidence;

  SVMObject() {
    id = -1;
    name = "";
    description = "";
    confidence = 0.0;
  }

  SVMObject(int newid, std::string newname, std::string newdescription, double confidence=0.0){
    id=id;
    name = name;
    description = description;
    confidence = confidence;
  }

  ~SVMObject(){
  }

  bool initialized() {
    if(id == -1)
      return false;
    return true;
  }

  bool operator==(const SVMObject& obj) const {
    if(obj.id == id)
      return true;
    return false;
  }

  bool operator!=(const SVMObject& obj) const {
    if(obj.id != id)
      return true;
    return false;
  }


  bool operator==(const int cmpId) const {
    if(id == cmpId)
      return true;
    return false;
  }

  bool operator<(const SVMObject& obj) const {
    if(id < obj.id)
      return true;
    return false;
  }
};

#endif
