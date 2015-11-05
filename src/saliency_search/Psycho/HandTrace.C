/*!@file Psycho/HandTrace.C */
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
// Primary maintainer for this file: Dicky Nauli Sihite <sihite@usc.edu>
// $HeadURL: 
// $Id: 


#ifndef PSYCHO_HANDTRACE_C_DEFINED
#define PSYCHO_HANDTRACE_C_DEFINED

#include "Psycho/HandTrace.H"
#include "Util/StringConversions.H"
#include "Util/StringUtil.H"
#include "Util/log.H"
#include "rutz/compat_cmath.h" // for isnan()

#include <fstream>

// ######################################################################
// ## namespace
// ######################################################################
namespace
{
  /* Look for auxiliary data files associated with the main .hanD
   * file, and read data from them if present. Specifically, if
   * itsFilename is e.g. "foo.hanD", then we will look these files:
   *
   * foo.hanD.ntrash: to contain a value for itsTrash
   * foo.hanD.rate: to contain a value for itsPeriod
   *
   * Note that the .rate file should contain a suffix to indicate what
   * units the value is in, e.g. "240.19Hz" or "4.16337ms".
   *
   * Returns true if all pieces of metadata were succesfully read. */
  bool getMetadataFromAuxFiles(const std::string& mainfilename,
                               size_t& ntrashOut, SimTime& periodOut)
  {
    bool gotperiod = false, gottrash = false;
    
    // ######################################################################
    { // These brackets are for making local variable
      std::ifstream f((mainfilename + ".ntrash").c_str());
      if (f.is_open()) {
        size_t ntrash = 0;
        f >> ntrash;
        if (f.fail()) LFATAL("couldn't read ntrash value from %s.ntrash",
                             mainfilename.c_str());
        
        ntrashOut = ntrash;
        gottrash = true;
        
        LINFO("read ntrash=%" ZU " from %s.ntrash",
              ntrashOut, mainfilename.c_str());
      }
    }
    
    // ######################################################################
    {
      std::ifstream f((mainfilename + ".rate").c_str());
      if (f.is_open()) {
        std::string rate;
        f >> rate;
        if (f.fail())
          LFATAL("couldn't read period/rate value from %s.rate",
                 mainfilename.c_str());
        
        periodOut = SimTime::fromString(rate);
        gotperiod = true;
        
        LINFO("read rate=%fHz from %s.rate",
              periodOut.hertz(), mainfilename.c_str());
      }   
    }
    
    return gottrash && gotperiod;
  }
}

// ######################################################################
// ##  HandTrace Class
// ######################################################################
HandTrace::HandTrace(const std::string& filename, const PixRGB<byte>& color) :
  itsFilename(filename), itsColor(color), itsPeriod(SimTime::ZERO()),
  itsTrash(0), itsNumEvents(0), nativeX(-1), nativeY(-1), itsData()
{
  // open the file
  const char *fn = filename.c_str();
  std::ifstream fil(fn);
  if (fil.is_open() == false) PLFATAL("Cannot open '%s'", fn);

  // text parsing variables
  std::string line; int linenum = -1;
  const std::string delim(" \t");
  bool gotperiod = false, gottrash = false, gotcols = false;
  uint samp_count = 0, trashed = 0;

  /* We need to get metadata, look for the following:
   * 1. Auxiliary files
   *    Extra files contains metadata values. If none found,
   *    continue to look for:
   * 2. Inside the main file
   *    Look for 'key=value' metadata lines
   */

  // 1. Aux files
  //    Read additional files for metadata
  const bool got_all_metadata_from_aux_files =
    getMetadataFromAuxFiles(itsFilename, itsTrash, itsPeriod);

  // 2. Inside main file
  //    If no aux files was found, read the main file and grab only metadata
  while (!got_all_metadata_from_aux_files && getline(fil, line)) {
    // one more line that we have read
    ++linenum;
    
    // skip initial whitespace and tab
    //std::string::size_type pos = line.find_first_not_of(delim, 0);
    
    //if (pos == line.npos) continue; // line was all whitespace/empty
    //if (line[pos] == '#') continue; // line is a comment
    
    // is it some metadata: "key = value"?
    if (line.find('=') != line.npos) {
      // let's tokenize it:
      std::vector<std::string> tok;
      split(line, "= \t", std::back_inserter(tok));
      
      // do we know that key?
      if (tok[0].compare("period") == 0) {
        itsPeriod = SimTime::fromString(tok[1]);
        gotperiod = true; }
      else if (tok[0].compare("trash") == 0) { 
        itsTrash = fromStr<int>(tok[1]);
        gottrash = true; }
      else if (tok[0].compare("cols") == 0) { 
        tok.erase(tok.begin()); // get rid of "cols"
        itsFields = tok;
        gotcols = true;}
      else if (tok[0].compare("res") == 0) {
        std::vector<std::string> res;
        split(tok[1], "x", std::back_inserter(res));
        nativeX = fromStr<int>(res[0]);
        nativeY = fromStr<int>(res[1]);
        }
      else {
        LFATAL("Unknown parameter '%s', file '%s' line %d",
               tok[0].c_str(), fn, linenum); }
      
      // done with this line, let's keep going:
      continue;
    }

    // check columns
    if (!gotcols) {
      std::string flds = "x y b";
      split(flds, " ", std::back_inserter(itsFields));
      gotcols = true;
    }

    // We got everything and since we already read this line, just proc it
    if (gotperiod && gottrash) {
      if (trashed < itsTrash) { ++trashed;  continue; }
      else {pushData(line);}
      samp_count++;
      break;
    }

    // if we reach this point, either we have hit some junk, or a
    // data line but we are missing some parameters. Not good:
    LFATAL("I need to have period and trash information before "
           "data starts, file '%s' line %d", fn, linenum);
  }
  
  // We got all the necessary metadata
  LINFO("%s: period = %.3fms, trash = %" ZU " samples.",
        fn, itsPeriod.msecs(), itsTrash);

  // Reset the variables back to default to prevent unwanted error
  line = ""; linenum = -1;
  
  // Now we have all the metadata. Let's get the data next.
  // Note that there is a bit of redundancy between the loop here and the
  // previous one, but it runs faster this way because from now on we
  // are sure that all the metadata is available:
  while (getline(fil, line)) {
    // one more line that we have read:
    ++linenum;
    
    // skip initial whitespace:
    //std::string::size_type pos = line.find_first_not_of(delim, 0);

    //if (pos == line.npos) continue; // line was all whitespace
    //if (line[pos] == '#') continue; // line is a comment

    // maybe we want to trash it right here:
    if (trashed < itsTrash) { ++trashed;  continue; }
    
    // else let's read the data
    else {pushData(line);}
    
    // update our sample count
    samp_count++;
  }
  
  LINFO("%s: %zu samples, %u events.", fn, itsData.size(), itsNumEvents);
}

// ######################################################################
HandTrace::~HandTrace()
{ }

// ######################################################################
bool HandTrace::pushData(const std::string line)
{
  /* Parses the line from a .hanD file and decides whether
   * to store extra data or not.
   * There are few types of fields(?):
   * 1. the standard (key) fields (x,y,buttons)
   * 2. event related fields: these fields receive a '*' designation
   *    and will be treated as "events"
   * 3. extra (non-key), non-event related fields 
   * 
   * Note: this code depends on some defaults that are set in .hanD
   */

  // Parse values from the line
  std::vector<std::string> strVals;
  split(line, " ", std::back_inserter(strVals));

  /* We expect the log file to have the following:
   * x y b0 b1 b2 b3 ...  *Event
   * the number of buttons available is not static/constant,
   * but they are expected to remain constant during the whole parsing
   * x and y is expected to be an integer, meanwhile buttons are expected
   * to be number 0 for false or any other for true condition
   * an event should be after the stuffs and always started with star
   */

  // Is the line an event field ?
  bool hasEventData = false;  uint i, iExtraData=0; //iExtraData shows where the extra data begins
  for (i = 0; i < strVals.size(); i++){
    // field should be indicative of an event and value should be valid
    if (isEventField(strVals[i]) && (fromStr<double>(strVals[i])==0.0)) {
      hasEventData = true; iExtraData=i; break;
    }
  }

  //! parse the line now for normal data
  //Create empty data
  int x_ = -1, y_ = -1;
  std::vector<bool> b_;
  int mx_ = -1, my_ = -1, nmx_ = nativeX, nmy_ = nativeY;
  bool mlb_ = false, mmb_ = false, mrb_ = false;
  //char * kbch_ = (char*)("");

  int no_btn=0; uint incr = 0; std::string kbData_("");


  // Doing the fields checking
  for (uint fldno = 0; fldno < itsFields.size(); fldno++) {
    //LINFO("BLAH %d %d %s %d",fldno,incr,itsFields[fldno].c_str(),
    //      int(strVals.size()));
    switch (itsFields[fldno][0]) {
    case 'x': //Joystick's x axis
      x_ = fromStr<int>(strVals[fldno+incr]); break;
    case 'y': //Joystick's y axis
      y_ = fromStr<int>(strVals[fldno+incr]); break;
    case 'b': //Joystick's buttons
      if (itsFields[fldno].size() > 1) { // we specify number of buttons
        std::string tmpline=itsFields[fldno];
        no_btn=fromStr<int>(tmpline.erase(0,1));
        incr += no_btn-1; // minus 1 coz we processed 1st data
        for (i = fldno; i < fldno+no_btn; i++) {
          b_.push_back(strVals[i].compare("0") != 0); }        
      } else { // none specified, we take the rest of data as buttons
        for (i = fldno; i < strVals.size(); i++) {
          b_.push_back(strVals[i].compare("0") != 0); }        
      } break;
    case 'm': // Mouse items
      switch (itsFields[fldno][1]) {
      case 'x': mx_ = fromStr<int>(strVals[fldno+incr]); break;
      case 'y': my_ = fromStr<int>(strVals[fldno+incr]); break;
      case 'l': mlb_ = (strVals[fldno+incr].compare("0") != 0); break;
      case 'm': mmb_ = (strVals[fldno+incr].compare("0") != 0); break;
      case 'r': mrb_ = (strVals[fldno+incr].compare("0") != 0); break;
      default: LFATAL("unknown field %s", itsFields[fldno+incr].c_str()); break;
      }
      break;
    case 'n': // Native resolution
      LFATAL("put native resolution in 'res=123x456'");
      break;
    case 'k': // Keyboard, treat the rest of data as string
      for (i = fldno+incr; i < strVals.size(); i++) {
        kbData_ += strVals[i]; kbData_ += " ";}
      break;
    default:
      LFATAL("unknown field %s", itsFields[fldno+incr].c_str());
      break;
    }
  }

  // parse the line for extra data
  rutz::shared_ptr<ParamMap> dataBuffer(new ParamMap);
  std::vector<std::string> temp;
  if (hasEventData){
    for (i = iExtraData; i < strVals.size(); i++){
      split(strVals[i], "=", std::back_inserter(temp));
      dataBuffer->putDoubleParam(unspecial(temp[0]),fromStr<double>(temp[1]));
    }
  }
  
  // if we have event-related data
  if(hasEventData) {
    // add to list of events
    itsEvents.push_back(dataBuffer);
    itsNumEvents++;
  }

  // Put all the data we gathered
  RawHandData ed = {x_, y_, b_, mx_, my_, nmx_, nmy_,
                    //mlb_, mmb_, mrb_, kbch_, dataBuffer};
                    mlb_, mmb_, mrb_, kbData_, dataBuffer};
  itsData.push_back(ed);
  
  return true;
}

// ######################################################################
bool HandTrace::isEventField(const std::string &field) const {
  return (field[0] == '*'); }

// ######################################################################
std::string HandTrace::special(std::string field) const {
  if(field[0]=='*') return field;
  else return "*"+field; }

// ######################################################################
std::string HandTrace::unspecial(std::string field) const {
  if(field[0]=='*') return field.erase(0,1);
  else return field; }

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_HANDTRACE_C_DEFINED
