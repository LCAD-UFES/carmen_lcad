/*!@file Psycho/EyesalData.C */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
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
// Primary maintainer for this file: David Berg <dberg@usc.edu>

#ifndef PSYCHO_EYESALDATA_C_DEFINED
#define PSYCHO_EYESALDATA_C_DEFINED

#include "Psycho/EyesalData.H"
#include "Util/StringConversions.H"
#include "Util/StringUtil.H"

#include <fstream>


// ######################################################################
EyesalData::EyesalData() :
  itsFilename(""), itsData()
{
}


EyesalData::EyesalData(const std::string& filename) :
  itsFilename(filename), itsData()
{
    setFile(filename);
}

// ######################################################################
//Access functions
// ######################################################################


void EyesalData::setFile(const std::string& filename)
{
// let's read the entire file:
  const char *fn = filename.c_str();
  std::ifstream fil(fn);
  if (fil.is_open() == false) PLFATAL("Cannot open '%s'", fn);

  const std::string delim(" \t");
  std::string line; int linenum = -1;

  //ok, we have opened a file so lets read some data lines
  while (getline(fil, line))
    {
      // one more line that we have read:
      ++linenum;

      // skip initial whitespace:
      std::string::size_type pos = line.find_first_not_of(delim, 0);
      if (pos == line.npos) continue; // line was all whitespace
      RawEyesalData mydata;

      // let's tokenize the line:
      std::vector<std::string> tok;
      split(line, " \t", std::back_inserter(tok));

      //get our saccade data
      mydata.Filename = tok[0];
      mydata.x = fromStr<int>(tok[1]);
      mydata.y = fromStr<int>(tok[2]);
      mydata.fovx = fromStr<int>(tok[3]);
      mydata.fovy  = fromStr<int>(tok[4]);
      mydata.pupil = fromStr<float>(tok[5]);
      mydata.amp = fromStr<float>(tok[6]);
      mydata.duration = fromStr<float>(tok[7]);
      mydata.sactime = fromStr<float>(tok[8]);
      mydata.val = fromStr<float>(tok[9]);
      mydata.min = fromStr<float>(tok[10]);
      mydata.max = fromStr<float>(tok[11]);
      mydata.avg = fromStr<float>(tok[12]);

      if (tok.size() != 313)
        LFATAL("Error parsing '%s', line %d", fn, linenum);

      for (size_t jj = 13; jj < tok.size(); jj++){
          mydata.rand[jj-13] = fromStr<float>(tok[jj]);
      }

      itsData.push_back(mydata);//we got all the data so store it

    }//end while getlines

}


bool EyesalData::hasData(const size_t index) const
{
  return index < itsData.size();
}


Point2D<int> EyesalData::getXYpos(const size_t index) const
{
  Point2D<int> tmp(itsData[index].x,itsData[index].y);
  return tmp;
}

std::vector<float> EyesalData::getNormSal() const
{
    std::vector<float> normsal;
    for (std::vector<RawEyesalData>::const_iterator sal_iter = itsData.begin();
         sal_iter != itsData.end(); ++sal_iter)
        normsal.push_back(sal_iter->val/sal_iter->max);
    return normsal;


}

std::vector<float> EyesalData::getNormRand(const size_t index) const
{
    std::vector<float> normrnd;
    for (std::vector<RawEyesalData>::const_iterator rnd_iter = itsData.begin();
         rnd_iter != itsData.end(); ++rnd_iter)
        normrnd.push_back(rnd_iter->rand[index*3] / rnd_iter->max);
    return normrnd;

}

std::vector< std::vector<float> > EyesalData::getAllNormRand() const
{
    std::vector< std::vector<float> > temp;
    for (size_t count = 0; count < 100; ++count)
    {
        std::vector<float> test = getNormRand(count);
        temp.push_back(test);
    }
    return temp;
}

std::vector< std::vector<float> > EyesalData::getAllNormRandT() const
{
    std::vector< std::vector<float> > temp = getAllNormRand();
    std::vector< std::vector<float> > out(temp[0].size());
    for (size_t count = 0; count < temp[0].size(); ++count){
        out[count].resize(100);
        for (size_t rndc = 0; rndc < 100; ++rndc){
            out[count][rndc] = temp[rndc][count];
        }
    }

    return out;
}

std::vector<Point2D<int> > EyesalData::getXYpos() const
{
  std::vector<Point2D<int> > tmpv;
  for (size_t jj = 0; jj < size(); jj++)
    {
      Point2D<int> tmp(itsData[jj].x,itsData[jj].y);
      tmpv.push_back(tmp);
    }
  return tmpv;
}

float EyesalData::getTime(const size_t index) const
{
  return itsData[index].sactime;
}


std::vector<float> EyesalData::getTime() const
{
  std::vector< float > tmpv;
  for (size_t jj = 0; jj < size(); jj++)
    {
      float tmp = itsData[jj].sactime;
      tmpv.push_back(tmp);
    }
  return tmpv;

}

std::string EyesalData::getFileName(const size_t index) const
{
  return itsData[index].Filename;
}

std::vector<std::string> EyesalData::getFileName() const
{
  std::vector< std::string > tmpv;
  for (size_t jj = 0; jj < size(); jj++)
    {
      std::string tmp = itsData[jj].Filename;
      tmpv.push_back(tmp);
    }
  return tmpv;

}size_t EyesalData::size() const
{
  return itsData.size();

}


std::string EyesalData::filename() const
{
  return itsFilename;
}

std::string EyesalData::basename() const
{
  size_t idx = itsFilename.rfind('.');
  size_t ids = itsFilename.rfind('/');
  if (idx == itsFilename.npos)
    idx = itsFilename.size();//no extension
  if (ids == itsFilename.npos)
    ids = 0;//no path
  return itsFilename.substr(ids+1+2, itsFilename.size()-idx-1);
}

// ######################################################################
//some simple computations
// ######################################################################


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_EYESALDATA_C_DEFINED
