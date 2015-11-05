//!Structures and methods for free viewing data

//////////////////////////////////////////////////////////////////////////
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/FreeViewingData.H $
#ifndef MODELNEURON_FREEVIEWINGDATA_H_DEFINED
#define MODELNEURON_FREEVIEWINGDATA_H_DEFINED

#include "Util/StringUtil.H"
#include "Media/MgzDecoder.H"
#include "Psycho/EyeTrace.H"
#include "Media/FrameSeries.H"
#include "Image/Range.H"
#include "Image/Point2D.H"

#include <iostream>
#include <fstream>
#include <sys/stat.h>

#include <fcntl.h> // for open()
#include <cerrno> // for errno 
#include <cstdio> // for perror()

// ######################################################################
// sampling strategy
// ######################################################################
enum ProbeType {Probe, Avg, Max};

//#######################################################################
std::string convertToString(const ProbeType val);

//#######################################################################
void convertFromString(const std::string& str, ProbeType& val);

struct vectoruint : public std::vector<uint> 
{
    vectoruint() = default;
    vectoruint(std::vector<uint> v) : std::vector<uint>(v) { };
};
//#######################################################################
//! vector uint overload: format is "<int>,<int>,..."
std::string convertToString(const vectoruint & val);

//#######################################################################
//! vector uint overload: format is "<int>,<int>,..."
void convertFromString(const std::string& str, vectoruint& val);

//#######################################################################
//! a class to transform from pixels to degrees and vice versa
//#######################################################################
struct PixelUnitTransform
{
    PixelUnitTransform() : ppdx(0.0F), ppdy(0.0F), screen() { };

    PixelUnitTransform(float const & _ppdx, float const & _ppdy, Dims const & _screen) 
        : ppdx(_ppdx), ppdy(_ppdy), screen(_screen) { };
    
    Point2D<float> pix2Deg(Point2D<int> const & p) const 
    {
      float const pixi = p.i - (screen.w() / 2.0F - 1.0F);
      float const pixj = -1.0F * p.j + (screen.h() / 2.0F - 1.0F);
      return Point2D<float>(pixi / ppdx, pixj / ppdy);
    }

    Point2D<int> deg2Pix(Point2D<float> const & p) const
    {
      float const hdposi = (p.i * ppdx) + (screen.w() / 2.0F - 1.0F);
      float const hdposj = -1.0F * ((p.j * ppdy) - (screen.h() / 2.0F - 1.0F));
      return Point2D<int>((int)hdposi, (int)hdposj);
    }
    
    float ppdx;
    float ppdy;
    Dims screen;
};


//#######################################################################
//!a struct to hold information about the feature maps
//#######################################################################
struct SubMapInfo
{
    SubMapInfo() : itsInfo(), itsGroup() { }

    std::string const getMapName(uint const index) const 
    { return itsInfo[index].name; }
    
    uint const getMapLevel(uint const index) const 
    { return itsInfo[index].level; }
    
    uint const getMapDelta(uint const index) const 
    { return itsInfo[index].delta; }

    uint const getChannelGroup(uint const index) const 
    { return itsGroup[index]; }

    uint const getMaxChannelGroup() const 
    { return itsGroup.back(); }
    
    void addMap(std::string _name, uint _level, uint _delta) 
    { 
      MapInfo info; 
      info.name = _name; info.level = _level; info.delta = _delta; 

      if (itsInfo.size() > 0)        
      {
        if (info.name.compare(itsInfo.back().name) == 0)
          itsGroup.push_back(itsGroup.back());
        else
          itsGroup.push_back(itsGroup.back()+1);
              
      }
      else
        itsGroup.push_back(0);

      itsInfo.push_back(info); 
    }
    
    uint const numMaps() const { return itsInfo.size(); };

    bool const operator==(SubMapInfo const & rhs)
    {
      if (itsInfo.size() != rhs.itsInfo.size())
        return false;
      
      std::vector<MapInfo>::const_iterator ii(itsInfo.begin()), end(itsInfo.end());
      std::vector<MapInfo>::const_iterator jj(rhs.itsInfo.begin());
      
      while (ii != end)
      {
        if (ii->name != jj->name)
          return false;
        
        if (ii->name != jj->name)
          return false;
        
        if (ii->name != jj->name)
          return false;

        ++ii; ++jj;
      }
      return true;
    }

    bool const operator!=(SubMapInfo const & rhs)
    { return !operator==(rhs); }
    
  private:
    struct MapInfo
    {
        std::string name;
        uint level, delta;
    };
    
    std::vector<MapInfo> itsInfo;
    std::vector<uint> itsGroup;
};

//#######################################################################
//! a struct to hold data needed for freeviewing experiment
//#######################################################################
struct FreeData
{
    FreeData() : retinal(), features(), eye(), info(), samples(0) { }
    
    rutz::shared_ptr<MgzDecoder> retinal;
    rutz::shared_ptr<MgzDecoder> features;
    rutz::shared_ptr<EyeTrace> eye;
    SubMapInfo info;
    std::string name;
    uint samples;
};

//#######################################################################
//Free functions
//#######################################################################

//!parse a .SubMapInfo file
// ######################################################################
inline
void getSubMapInfoFreeViewingData(FreeData & freedata, std::string const & filename)
{
  std::ifstream submapFile(filename.c_str());
  if (!submapFile.is_open())
    LFATAL("Error: cannot open the file");
  
  std::string line;
  
  SubMapInfo infoList;
  while (getline(submapFile, line))
  {
    std::vector<std::string> toks;
    split(line, " ", back_inserter(toks));
    
    infoList.addMap(toks[0], fromStr<uint>(toks[2]), fromStr<uint>(toks[4]));
  }
  submapFile.close();
  freedata.info = infoList;
}

// ######################################################################
inline
void getMGZFreeViewingRetinal(FreeData & freedata, std::string const & retinalfilename)
{
  struct stat info;
  LINFO("%s", retinalfilename.c_str());
  int ret = stat(retinalfilename.c_str(), &info);
  if(ret == 0) 
    freedata.retinal = rutz::make_shared(new MgzDecoder(retinalfilename));
}

// ######################################################################
inline
void getMGZFreeViewingData(FreeData & freedata, std::string const & mgzfilename)
{
  //first parse the filename and get the mgz file and the submap info file name
  std::vector<std::string> toks;
  split(mgzfilename, ".", back_inserter(toks));
  freedata.name = toks[0];
  std::string infofilename = toks[0] + "-SubMapInfo";

  toks.resize(0);
  split(mgzfilename, "/", back_inserter(toks));
  std::string mgzfbase = toks.back();
  std::string path = join(toks.begin(), toks.end()-1, "/");

  toks.resize(0);
  split(mgzfbase, "-", back_inserter(toks));
  mgzfbase = toks.back();
  mgzfbase = toks[toks.size()-2] + "-" + mgzfbase;
 
  std::string retinalfilename = "/" + path + "/" + mgzfbase + ".RETOUT-";
  
  //load the submap info and mgz file and any retinal output
  getSubMapInfoFreeViewingData(freedata, infofilename);
  freedata.features = rutz::make_shared(new MgzDecoder(mgzfilename));

  //get the retinal
  getMGZFreeViewingRetinal(freedata, retinalfilename);
}

// ######################################################################
inline
void getEyeTraceFreeViewingData(FreeData & freedata, std::string const & etfilename)
{
  freedata.eye = rutz::make_shared(new EyeTrace(etfilename, PixRGB<byte>(0,0,0)));
  freedata.samples = freedata.eye->size();    
}

// ######################################################################
inline
std::vector<FreeData> loadMGZFromList(std::string const & filenames)
{
  //load up all the mgz data
  std::vector<std::string> toks;
  split(filenames, ",", back_inserter(toks));
  
  //allocate memory for our data  
  std::vector<FreeData> freedata(toks.size());
  
  std::vector<std::string>::const_iterator fname(toks.begin()), end(toks.end());
  std::vector<FreeData>::iterator data(freedata.begin()), dataend(freedata.end());
  
  //load each file
  getMGZFreeViewingData(*data, *fname);
  SubMapInfo mapinfo = data->info;
  ++data; ++fname;
  
  while (fname != end)
  {
    getMGZFreeViewingData(*data, *fname);
    if (mapinfo != data->info)
      LFATAL("The MGZ files must all have the same types of channels");
    
    ++data; ++fname;
  }
  
  return freedata;
}

//load up a normalization file
// ######################################################################
inline 
std::vector<Range<float> > loadChanNormFile(std::string const & filename, FreeData const & freedata, bool const lock = true)
{
  std::vector<Range<float> > ranges(freedata.info.numMaps());  
  
  //if no file name return
  if (filename.empty())
    return ranges;

  //lock a lock file
  std::string lockfile = filename + ".lock";
  if (lock)
  {
    int fd = -1;
    while (fd < 0)
    {
      LINFO("Locking file");
      fd = open(lockfile.c_str(), O_WRONLY | O_CREAT | O_EXCL, S_IRUSR | S_IWUSR);
      if ((fd < 0) && (errno != EEXIST))
        LFATAL("Could not lock the file");
    }
  }
  
  //see if the file exists
  struct stat info;
  int ret = stat(filename.c_str(), &info);
  if (ret != 0) 
    return ranges;
    
  //error if we can't open
  std::ifstream normFile(filename.c_str());
  if (!normFile.is_open())
    LFATAL("Error: cannot open the file");
  
  //gobble up the file
  uint c = 0;
  std::string line;
  while (getline(normFile, line))
  {
    std::vector<std::string> toks;
    split(line, " ", back_inserter(toks));
    std::string channame = toks[0];

    if (c >= freedata.info.numMaps())
    {
      LINFO("Channel normalization file does not have the correct number of channels.");
      return std::vector<Range<float> >();
    }

    if (freedata.info.getMapName(c).compare(channame) != 0)
    {
      LINFO("Channel normalization file doesn't have the same channel names");
      return std::vector<Range<float> >();
    }
    
    Range<float> range(fromStr<float>(toks[1]), fromStr<float>(toks[2]));
    ranges[c] = range;
    ++c;
  }
  
  if (c != freedata.info.numMaps())
  { 
    LINFO("Channel normalization file does not have the correct number of channels.");
    return std::vector<Range<float> >();
  }
  
  //unlock lock file
  remove(lockfile.c_str());
  LINFO("Releasing file");
  
  return ranges;
}

//load up a normalization file
// ######################################################################
inline 
std::vector<Range<float> > loadChanNormFile(std::string const & filename, std::vector<std::string> const & chanNames, bool const lock = true)
{
  std::vector<Range<float> > ranges(chanNames.size());  

  //if no file name return
  if (filename.empty())
    return ranges;
  
  //lock a lock file
  std::string lockfile = filename + ".lock";
  if (lock)
  {
    int fd = -1;
    while (fd < 0)
    {
      LINFO("Locking file");
      fd = open(lockfile.c_str(), O_WRONLY | O_CREAT | O_EXCL, S_IRUSR | S_IWUSR);
      if ((fd < 0) && (errno != EEXIST))
        LFATAL("Could not lock the file");
    }
  }
  
  //see if the file exists
  struct stat info;
  int ret = stat(filename.c_str(), &info);
  if (ret != 0) 
  {
    LINFO("Norm file doesn't exist, returning uninitialzed range");

    //unlock lock file
    remove(lockfile.c_str());
    LINFO("Releasing file");
    return ranges;
  }
    
  //error if we can't open
  std::ifstream normFile(filename.c_str());
  if (!normFile.is_open())
  {
    //unlock lock file
    remove(lockfile.c_str());
    LINFO("Releasing file");
    
    LFATAL("Error: cannot open the file");
  }
  
  //gobble up the file
  uint c = 0;
  std::string line;
  while (getline(normFile, line))
  {
    std::vector<std::string> toks;
    split(line, " ", back_inserter(toks));
    std::string channame = toks[0];

    if (c >= chanNames.size())
    {
      //unlock lock file
      remove(lockfile.c_str());
      LINFO("Releasing file");
      
      LINFO("Channel normalization file does not have the correct number of channels");
      return std::vector<Range<float> >();
    }

    if (chanNames[c].compare(channame) != 0)
    {
      //unlock lock file
      remove(lockfile.c_str());
      LINFO("Releasing file");

      LINFO("Channel normalization file doesn't have the same channel names");
      return std::vector<Range<float> >();
    }
    
    Range<float> range(fromStr<float>(toks[1]), fromStr<float>(toks[2]));
    ranges[c] = range;
    ++c;
  }
  
  if (c != chanNames.size())
  {
    //unlock lock file
    remove(lockfile.c_str());
    LINFO("Releasing file");

    LINFO("Channel normalization file does not have the correct number of channels");
    return std::vector<Range<float> >();
  }

  //unlock lock file
  remove(lockfile.c_str());
  LINFO("Releasing file");
  
  return ranges;
}

//load up a normalization file
// ######################################################################
inline
void writeChanNormFile(std::string const & filename, FreeData const & freedata, std::vector<Range<float> > const & ranges)
{
  //get the norm file
  if (filename.empty())
    return;

  //lock a lock file
  int fd = -1;
  std::string lockfile = filename + ".lock";
  while (fd < 0)
  {
    LINFO("Locking file");
    fd = open(lockfile.c_str(), O_WRONLY | O_CREAT | O_EXCL, S_IRUSR | S_IWUSR);
    if ((fd < 0) && (errno != EEXIST))
      LFATAL("Could not lock the file");
  }

  //get the files current contents if any, otherwise return a vector of initialized Range objects
  std::vector<Range<float> > currrange = loadChanNormFile(filename, freedata, false);
  
  std::ofstream normFile(filename.c_str());
  if (!normFile.is_open())
    LFATAL("Error: cannot open the file");
  
  if (freedata.info.numMaps() != ranges.size())
    LFATAL("The vectors of ranges must have the same number of elements as there are sub maps when writing out a norm file");
  
  for (uint ii = 0; ii < freedata.info.numMaps(); ++ii)
  {
    currrange[ii].merge(ranges[ii]);
    normFile << freedata.info.getMapName(ii) << " " << currrange[ii].min() << " " << currrange[ii].max() << std::endl;
  }
  
  normFile.close();
  
  //unlock lock file
  remove(lockfile.c_str());
}

// ######################################################################
inline
void writeChanNormFile(std::string const & filename, std::vector<std::string> const & channel_names, std::vector<Range<float> > const & ranges)
{
  //get the norm file
  if (filename.empty())
    return;

  //lock a lock file
  int fd = -1;
  std::string lockfile = filename + ".lock";
  while (fd < 0)
  {
    LINFO("Locking file");
    fd = open(lockfile.c_str(), O_WRONLY | O_CREAT | O_EXCL, S_IRUSR | S_IWUSR);
    if ((fd < 0) && (errno != EEXIST))
      LFATAL("Could not lock the file");
  }

  //get the files current contents if any, otherwise return a vector of initialized Range objects
  std::vector<Range<float> > currrange = loadChanNormFile(filename, channel_names, false);
  if (currrange.size() == 0)
    currrange.resize(channel_names.size());
  
  std::ofstream normFile(filename.c_str());
  if (!normFile.is_open())
    LFATAL("Error: cannot open the file");
  
  if (channel_names.size() != ranges.size())
    LFATAL("The vectors of ranges must have the same number of elements as the channel names");
  
  for (uint ii = 0; ii < channel_names.size(); ++ii)
  {
    currrange[ii].merge(ranges[ii]);
    normFile << channel_names[ii] << " " << currrange[ii].min() << " " << currrange[ii].max() << std::endl;
  }
  
  normFile.close();
  
  //unlock lock file
  remove(lockfile.c_str());
}
#endif
