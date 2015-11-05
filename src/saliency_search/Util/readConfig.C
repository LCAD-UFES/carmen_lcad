/*!@file Util/readConfig.C CINNIC classes */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/readConfig.C $
// $Id: readConfig.C 9337 2008-02-27 12:13:43Z beobot $
//

#include "Util/readConfig.H"

#include "Util/Assert.H"
#include "Util/log.H"
#include "Util/sformat.H"

#include <fstream>

using std::string;

// note: as far as I know, memory management for vectors is all automatic;
// so no need to resize them

readConfig::readConfig()
{
  vectorValue = 25;
  first.resize(vectorValue);
  second.resize(vectorValue);
  isItem.resize(vectorValue);
}


readConfig::readConfig(int size)
{
  vectorValue = size;
  first.resize(vectorValue);
  second.resize(vectorValue);
  isItem.resize(vectorValue);
}

readConfig::~readConfig()
{
}

void readConfig::openFile(const char* filename, bool echo)
{
  //  LINFO("readConfig: Parsing file...\t%s\n", filename);
  SiZE = 0;
  fileName = filename;
  std::ifstream inFile(filename,std::ios::in);
  vectorSize = vectorValue;
  comment = false;
  item = true;
  while (inFile >> in)
  {
    if(!in.compare("#")) //comment code # found
    {
      if(!comment)
      {
        comment = true;
      }
      else      //end of comment
      {
        comment = false;
      }
    }
    if((!comment) && in.compare("#")) //real line found
    {
      if(item)
      {
        if(SiZE >= (vectorSize-1)) //resize vector if
                                   //more then vectorSize lines in file
        {
          //LINFO("Resizing configfile to %d",vectorSize);
          vectorSize+=vectorValue;
          first.resize(vectorSize);
          second.resize(vectorSize);
          isItem.resize(vectorSize);
        }
        if(echo == true)
          LINFO("item:%d %s ", SiZE, in.c_str());
        first[SiZE] = in;
        isItem[SiZE] = true;
        item = false;
      }
      else
      {
        if(echo == true)
          LINFO("%s\n", in.c_str());
        second[SiZE] = in;
        item = true;
        SiZE++;
      }
    }
  }
}

void readConfig::writeFile()
{
  std::ofstream outfile(fileName.c_str(),std::ios::out);
  for(int i = 0; i < SiZE; i++)
  {
    if(isItem[i]) //check to make sure this is an item
    {
      outfile << first[i] << " " << second[i] << "\n";
    }
  }
  outfile.close();
}

void readConfig::writeFile(const char* filename)
{
  std::ofstream outfile(filename,std::ios::out);
  for(int i = 0; i < SiZE; i++)
  {
    if(isItem[i]) //check to make sure this is an item
    {
      outfile << first[i] << " " << second[i] << "\n";
    }
  }
  outfile.close();
}

bool readConfig::readFileTrue(int itemNumber)
{
  if(itemNumber < vectorSize) //make sure we don't go beyond vector size
  {
    return isItem[itemNumber];
  }
  else
  {
    return false; //out of bounds of vector
  }
}

int readConfig::readFileValueI(int itemNumber)
{
  ASSERT(itemNumber < SiZE);
  return atoi(second[itemNumber].c_str());
}

float readConfig::readFileValueF(int itemNumber)
{
  ASSERT(itemNumber < SiZE);
  return atof(second[itemNumber].c_str());
}

string readConfig::readFileValueS(int itemNumber)
{
  ASSERT(itemNumber < SiZE);
  return second[itemNumber];
}

const char* readConfig::readFileValueC(int itemNumber)
{
  ASSERT(itemNumber < SiZE);
  return second[itemNumber].c_str();
}

string readConfig::readFileValueName(int itemNumber)
{
  ASSERT(itemNumber < SiZE);
  return first[itemNumber];
}

const char* readConfig::readFileValueNameC(int itemNumber)
{
  ASSERT(itemNumber < SiZE);
  return first[itemNumber].c_str();
}

float readConfig::readFileValueNameF(int itemNumber)
{
  ASSERT(itemNumber < SiZE);
  return atof(first[itemNumber].c_str());
}

float readConfig::getItemValueF(string itemName)
{
  int x = 0;
  for(int i = 0; i < SiZE; i++)
  {
    if(!first[i].compare(itemName))
    {
      x = 1;
      return atof(second[i].c_str());
    }
  }
  if(x != 1)
    LFATAL("Requested item \"%s\" not found in config file \"%s\"",
           itemName.c_str(),fileName.c_str());
  return 0.0F;
}

string readConfig::getItemValueS(string itemName)
{
  int x = 0;
  for(int i = 0; i < SiZE; i++)
  {
    if(!first[i].compare(itemName))
    {
      x = 1;
      return second[i];
    }
  }
  if(x != 1)
    LFATAL("Requested item \"%s\" not found in config file \"%s\"",
           itemName.c_str(),fileName.c_str());
  return("Fluffy Bunny");
}

bool readConfig::getItemValueB(string itemName)
{
  int x = 0;
  for(int i = 0; i < SiZE; i++)
  {
    if(!first[i].compare(itemName))
    {
      x = 1;
      if(((int)atof(second[i].c_str())) == 1)
        return true;
      else
        return false;
    }
  }
  if(x != 1)
    LFATAL("Requested item \"%s\" not found in config file \"%s\"",
           itemName.c_str(),fileName.c_str());
  return false;
}

const char* readConfig::getItemValueC(string itemName)
{
  int x = 0;
  for(int i = 0; i < SiZE; i++)
  {
    if(!first[i].compare(itemName))
    {
      x = 1;
      return second[i].c_str();
    }
  }
  if(x != 1)
    LFATAL("Requested item \"%s\" not found in config file \"%s\"",
           itemName.c_str(),fileName.c_str());
  return("Blue Smurf");
}

void readConfig::setItemValue(string itemName, float _set)
{
  int x = 0;
  for(int i = 0; i < SiZE; i++)
  {
    if(!first[i].compare(itemName))
    {
      x = 1;
      second[i] = sformat("%f",_set);
    }
  }
  if(x != 1)
    LFATAL("Requested item \"%s\" not found in config file",itemName.c_str());
}

void readConfig::setItemValue(string itemName, string _set)
{
  int x = 0;
  for(int i = 0; i < SiZE; i++)
  {
    if(!first[i].compare(itemName))
    {
      x = 1;
      second[i] = _set;
    }
  }
  if(x != 1)
    LFATAL("Requested item \"%s\" not found in config file",itemName.c_str());
}

void readConfig::setItemValue(string itemName, const char* _set)
{
  int x = 0;
  for(int i = 0; i < SiZE; i++)
  {
    if(!first[i].compare(itemName))
    {
      x = 1;
      second[i] = _set;
    }
  }
  if(x != 1)
    LFATAL("Requested item \"%s\" not found in config file",itemName.c_str());
}

void readConfig::setFileValue(int itemNumber, float _set)
{
  ASSERT(itemNumber < SiZE);
  second[itemNumber] = sformat("%f",_set);
}

void readConfig::setFileValue(int itemNumber, string _set)
{
  ASSERT(itemNumber < SiZE);
  second[itemNumber] = _set;
}

void readConfig::setFileValue(int itemNumber, const char* _set)
{
  ASSERT(itemNumber < SiZE);
  second[itemNumber] = _set;
}

int readConfig::addItemValue(string itemName, float _set)
{
  return addItemValue(itemName,sformat("%f",_set));
}

int readConfig::addItemValue(string itemName, string _set)
{
  if(SiZE >= (vectorSize - 1))
  {
    LINFO("Resizing configfile to %d",vectorSize);
    vectorSize+=vectorValue;
    first.resize(vectorSize);
    second.resize(vectorSize);
    isItem.resize(vectorSize);
  }
  first[SiZE] = itemName;
  second[SiZE] = _set;
  isItem[SiZE] = true;
  SiZE++;
  return SiZE;
}

int readConfig::addItemValue(string itemName, const char* _set)
{
  string str = _set;
  return addItemValue(itemName,str);
}

int readConfig::itemCount()
{
  return SiZE;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
