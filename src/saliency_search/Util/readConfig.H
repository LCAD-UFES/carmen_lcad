/*!@file Util/readConfig.H CINNIC classes */

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
// Primary maintainer for this file: T. Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/readConfig.H $
// $Id: readConfig.H 6003 2005-11-29 17:22:45Z rjpeters $
//

#ifndef READCONFIG_H_DEFINED
#define READCONFIG_H_DEFINED

#include "Util/Types.H"
#include <cstdlib>
#include <string>
#include <vector>

// ############################################################
// ############################################################
// ##### ---CINNIC---
// ##### readConfig:
// ##### T. Nathan Mundhenk nathan@mundhenk.com
// ############################################################
// ############################################################

//#define vectorValue 25

//!open a config file and parse it for data retreaval
/*! This class works to open a file for parsing which has name value
  key pairs. Each pair is seperated and loaded into a vector. Values
  can then be retrieved by either calling a values item number or
  name. This will then return the value. Comments can be handeled
  in the config file so long as they are in the form '# words #'.
  That is place a # with a space before and after each comment.
*/
class readConfig
{
private:
  std::vector<std::string> first;
  std::vector<std::string> second;
  std::vector<bool> isItem;
  std::string in;
  std::string fileName;
  bool comment;
  bool item;
  int vectorSize;
  int SiZE;
  int vectorValue;
public:
  //! constructor with initial size = 25
  readConfig();
  //! constructor with initial size
  readConfig(int size);
  //! destructor for readConfig
  ~readConfig();
  //! open a config file with filename
  void openFile(const char* filename, bool echo = false);
  //! write the file back to disk as filename
  void writeFile();
  //! write the file back to disk with a new file name
  void writeFile(const char* filename);
  //! returns true if item number itemNumber exists
  bool readFileTrue(int itemNumber);
  //! return value of item as an integer
  int readFileValueI(int itemNumber);
  //! return value of item as a float
  float readFileValueF(int itemNumber);
  //! return value of item as a string
  std::string readFileValueS(int itemNumber);
  //! return value of item as a C style char*
  const char* readFileValueC(int itemNumber);
  //! return the item name
  std::string readFileValueName(int itemNumber);
  //! return item name as a C style char*
  const char* readFileValueNameC(int itemNumber);
  //! return item name as a float
  float readFileValueNameF(int itemNumber);
  //! return the float value of a number with name itemName
  float getItemValueF(std::string itemName);
  //! return the string value of a number with name itemName
  std::string getItemValueS(std::string itemName);
  //! return a bool representation of the value 1 = true else false
  bool getItemValueB(std::string itemName);
  //! return the char* value of a number with name itemName
  const char* getItemValueC(std::string itemName); //return the char*
                                              //value of a number
                                              //with name itemName
  //! Set the item with string name itemName to a float _set
  void setItemValue(std::string itemName, float _set);
  //! Set the item with string name itemName to a string _set
  void setItemValue(std::string itemName, std::string _set);
  //! Set the item with string name itemName to a const char* _set
  void setItemValue(std::string itemName, const char* _set);
  //! Set the item signified by its number to a float
  void setFileValue(int itemNumber, float _set);
  //! Set the item signified by its number to a string
  void setFileValue(int itemNumber, std::string _set);
  //! Set the item signified by its number to a const char*
  void setFileValue(int itemNumber, const char* _set);
  //! Set the item with string name itemName to a float _set
  int addItemValue(std::string itemName, float _set);
  //! Set the item with string name itemName to a string _set
  int addItemValue(std::string itemName, std::string _set);
  //! Set the item with string name itemName to a string _set
  int addItemValue(std::string itemName, const char* _set);
  //! returns how many items exist
  int itemCount();
  //void closeFile(); //close the file
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif
