/*!@file Psycho/EyeTrace.C */

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
// Primary maintainer for this file: John Shen <shenjohn@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/EyeTrace.C $
// $Id: EyeTrace.C 15310 2012-06-01 02:29:24Z itti $


#ifndef PSYCHO_EYETRACE_C_DEFINED
#define PSYCHO_EYETRACE_C_DEFINED

#include "Psycho/EyeTrace.H"
#include "Util/StringConversions.H"
#include "Util/StringUtil.H"
#include "Util/log.H"
#include "rutz/compat_cmath.h" // for isnan()

#include <fstream>

// ######################################################################
EyeTrace::EyeTrace(const std::string& filename, const PixRGB<byte>& color) :
  itsFilename(filename), itsColor(color), itsPeriod(SimTime::ZERO()),
  itsTrash(0), itsPPD(), itsNumEvents(0), itsData()
{
  // let's read the entire file:
  const char *fn = filename.c_str();
  std::ifstream fil(fn);
  if (fil.is_open() == false) PLFATAL("Cannot open '%s'", fn);

  std::string line; int linenum = -1;
  const std::string delim(" \t");
  bool gotperiod = false, gottrash = false, gotppd = false, gotcols = false;
  int ncols = 0;
  //itsInSac = false;
  uint samp_count = 0, trashed = 0;
  rutz::shared_ptr<ParamMap> lastSacData(new ParamMap);

  //look in the header for metadata
  while (getline(fil, line))
    {
      // one more line that we have read:
      ++linenum;

      // skip initial whitespace:
      std::string::size_type pos = line.find_first_not_of(delim, 0);

      if (pos == line.npos) continue; // line was all whitespace

      if (line[pos] == '#') continue; // line is a comment

      // is it some metadata: "key = value"?
      if (line.find('=') != line.npos)
        {
          // let's tokenize it:
          std::vector<std::string> tok;
          split(line, "= \t", std::back_inserter(tok));

          if (tok.size() != 2 && tok[0].compare("cols") != 0)
            {
              LFATAL("Error parsing '%s', line %d", fn, linenum);
            }
          // do we know that key?
          if (tok[0].compare("period") == 0)
            { itsPeriod = SimTime::fromString(tok[1]); gotperiod = true; }
          else if (tok[0].compare("trash") == 0)
            { itsTrash = fromStr<int>(tok[1]); gottrash = true; }
          else if (tok[0].compare("ppd") == 0)
            { 
              
              std::vector<std::string> ltok;
              split(tok[1], ",", std::back_inserter(ltok));
              if (ltok.size() > 1)
                itsPPD = PixPerDeg(fromStr<float>(ltok[0]),
                                   fromStr<float>(ltok[1])); 
              else
                itsPPD = PixPerDeg(fromStr<float>(ltok[0]), 0.0F);
              gotppd = true; 
            }
          else if (tok[0].compare("cols") == 0)
            {
              tok.erase(tok.begin()); // get rid of "cols"
              itsFields = tok;
              gotcols = true;
            }
          else LFATAL("Unknown parameter '%s', file '%s' line %d",
                      tok[0].c_str(), fn, linenum);

          // Enforce that we should have all our parameters parsed before
          // the data starts:
          if (gotperiod && gottrash && gotppd) break;

          // done with this line, let's keep going:
          continue;
        }
      // if we reach this point, either we have hit some junk, or a
      // data line but we are missing some parameters. Not good:
      LFATAL("I need to have period, trash, and ppd information before "
             "data starts, file '%s' line %d", fn, linenum);
    }

  LINFO("%s: period = %.3fms, ppdx = %.1f ppdy = %.1f pix/deg, trash = %" ZU " samples.",
        fn, itsPeriod.msecs(), itsPPD.ppdx(), itsPPD.ppdy(), itsTrash);

  // all right, we have all the metadata, let's now get the data. Note
  // that there is a bit of redundancy between the loop here and the
  // previous one, but it runs faster this way because from now on we
  // can assume that all the metadata is available:
  while (getline(fil, line))
    {
      // one more line that we have read:
      ++linenum;

      // skip initial whitespace:
      std::string::size_type pos = line.find_first_not_of(delim, 0);

      if (pos == line.npos) continue; // line was all whitespace

      if (line[pos] == '#') continue; // line is a comment

      // is it the column metadata (that came last in the header): "cols = value"?
      if (line.find("cols =") != line.npos)
        {
          // let's tokenize it:
          std::vector<std::string> tok;
          split(line, "= \t", std::back_inserter(tok));

          tok.erase(tok.begin());  // get rid of "cols"
          itsFields = tok;
          gotcols = true;
          continue;
        }

      // are we still unsure of the columns? then we first need to infer the columns
      if(!gotcols)
        {
          // extract values from line
          std::vector<std::string> values;
          split(line, " ", std::back_inserter(values));
          ncols = values.size();

          std::string flds = "";

          // case out for the number of columns.  
          // this is for legacy data with few # of columns 
          switch(ncols)
            {
            case 2: {flds = "x y"; break;}
            case 3: {flds = "x y status"; break;}
            case 4: {flds = "x y pd status"; break;}
            case 7: {flds = "x y status *targetx *targety *ampl *interval"; break;}
            case 8: {flds = "x y pd status *targetx *targety *ampl *interval"; break;}
            default:
              LFATAL("Error parsing data in '%s',line %d: %d of columns gives unknown data",
                              fn, linenum, ncols);
            }

          split(flds, " ", std::back_inserter(itsFields));
          gotcols = true;
        }

      // maybe we want to trash it right here:
      if (trashed < itsTrash) { ++trashed;  continue; }

      // else let's read the data
      else {pushData(line);}

      // update our sample count
      samp_count++;
    } // while (getline(fil,line))

  LINFO("%s: cols = %s",
        fn, join(itsFields.begin(),itsFields.end()," ").c_str() );

  LINFO("%s: %zu samples, %u events.", fn, itsData.size(),
        itsNumEvents);
}

// ######################################################################
EyeTrace::~EyeTrace()
{ }

// ######################################################################
bool EyeTrace::hasData(const size_t index, const SimTime& t) const
{ return ( index < itsData.size() && itsPeriod * int(index) < t ); }

// ######################################################################
bool EyeTrace::hasData(const size_t index) const
{ return ( index < itsData.size() ); }

// ######################################################################
rutz::shared_ptr<EyeData> EyeTrace::data(const size_t index) const
{
  if (index >= itsData.size()) LFATAL("Index past end of trace");

  // compute the saccade/blink states. Here is what David's matlab
  // code (see saliency/matlab/Eye-Markup/) spits out:
  // fixation:                   0 blue
  // saccade:                    1 green
  // blink/Artifact:             2 red
  // Saccade during Blink:       3 green 
  // smooth pursuit:             4 magenta
  // drift/misclassification:    5 black
  // combined saccades           6 green

  SaccadeState ss; bool bs;
  switch(itsData[index].status)
    {
    case 0: ss = SACSTATE_FIX; bs = false; break;
    case 1: ss = SACSTATE_SAC; bs = false; break;
    case 2: ss = SACSTATE_FIX; bs = true;  break;
    case 3: ss = SACSTATE_SAC; bs = true;  break;
    case 4: ss = SACSTATE_SMO; bs = false; break;
    case 5: ss = SACSTATE_UNK; bs = false; break;
    case 6: ss = SACSTATE_COM; bs = false; break;

    default:
      LFATAL("Bogus status code '%d', file '%s' index %" ZU " (after trashing)",
             itsData[index].status, itsFilename.c_str(), index);
      ss = SACSTATE_UNK; bs = false; // compiler doesn't realize this won't run
      break;
    }

  rutz::shared_ptr<EyeData> ret;

  // correct this part

  ret.reset(new EyeData(itsData[index].x, itsData[index].y,
                          itsData[index].diam, ss, bs,
                          itsData[index].extraData));

  return ret;
}

// ######################################################################
const SaccadeState EyeTrace::getStatus(const int sta) const
{
  SaccadeState ss;
  switch(sta)
    {
    case 0: ss = SACSTATE_FIX; break;
    case 1: ss = SACSTATE_SAC; break;
    case 2: ss = SACSTATE_FIX; break;
    case 3: ss = SACSTATE_SAC; break;
    case 4: ss = SACSTATE_SMO; break;
    case 5: ss = SACSTATE_UNK; break;
    case 6: ss = SACSTATE_COM; break;

    default:
        LFATAL("Bogus status code '%d', file '%s'",
               sta, itsFilename.c_str());
        ss = SACSTATE_UNK; // compiler doesn't realize this won't run
      break;
    }
  return ss;

}

bool EyeTrace::pushData(const std::string line)
{
  // Parses the line from an EyeMarkup file
  // and decides whether or not to store extra data 
  // There are three types of fields: 
  // (1) the standard (key) fields (x,y,pdiam,status)
  // (2) event related fields: these fields receive a '*' designation
  // and will be treated as "events" (formerly saccades only)
  // (3) extra (non-key), non-event related fields 

  // NB: this code depends on some defaults that are set in EyeMarkup 

  //////////////////////////////////////
  // Parse values from the line
  std::vector<std::string> strVals;
  split(line, " ", std::back_inserter(strVals));

  if(itsFields.size() != strVals.size())
    {} // TODO: should throw some exception here for safety

  // Parse data from line into doubles
  rutz::shared_ptr<ParamMap> dataBuffer(new ParamMap);

  // sentinel value that Eye-Markup uses for when there is no event data
  const double SENTINEL = 0.0;
  uint i = 0;
  std::vector<double> values(strVals.size(),SENTINEL);
  for (i = 0; i < strVals.size(); i++)
    if(strVals[i].compare("NaN") != 0) // handle NaNs
      values[i] = fromStr<double>(strVals[i]);

  //////////////////////////////////////
  // pre-test if there is any event-related data
  // We have event data only when we see non-zero event marked fields
  bool hasEventData = false;
  for (i = 0; i < itsFields.size(); i++)
      // field should be indicative of an event and value should be valid
      if (isEventField(itsFields[i]) && values[i] != SENTINEL) 
        {
          hasEventData = true; 
          break;
        }

  //////////////////////////////////////
  // assign values to their fields
  // Special handling for standard data:
  // here we set up default values for x,y,pd, and status
  // Note: sensitive to the names used in "cols" header
  // TODO: generalize this so that key fields can be added
  
  std::vector<std::string> key_fields;
  const std::string fieldnames("x y pd status");
  split(fieldnames," ", std::back_inserter(key_fields));
  std::vector<double> defaults(key_fields.size(),0); 
  
  // assign key fields their defaults in our buffer
  for (i = 0; i < key_fields.size(); i++)
       dataBuffer->putDoubleParam(key_fields[i], defaults[i]);

  // assign values from the input line
  for (i = 0; i < itsFields.size(); i++)
    {
      // if this is an event-related field without data, skip it
      if (isEventField(itsFields[i]) && !hasEventData) continue;
      // see if this field is already in the set of key fields
      if(find(key_fields.begin(),key_fields.end(),itsFields[i])
         !=key_fields.end() )         
          dataBuffer->replaceDoubleParam(itsFields[i], values[i]);
      else
          dataBuffer->putDoubleParam(itsFields[i],values[i]);
    }

  //////////////////////////////////////
  // package the data
  RawEyeData ed = {float(dataBuffer->getDoubleParam("x")),
                   float(dataBuffer->getDoubleParam("y")),
                   float(dataBuffer->getDoubleParam("pd")),
                   int(dataBuffer->getDoubleParam("status")),
                   dataBuffer};
  // from the extra data structure, get rid of the standard data
  for (i = 0; i < key_fields.size(); i++)
      ed.extraData->erase(key_fields[i]);

  // if we have event-related data
  if(hasEventData)
    {
      // add to list of events
      itsEvents.push_back(ed.extraData);
      itsNumEvents++;
    }

  itsData.push_back(ed);
  return true;
}

// ######################################################################
bool EyeTrace::isEventField(const std::string field) const
{ return field[0] == '*'; }

// ######################################################################
size_t EyeTrace::size() const
{ return itsData.size(); }

// ######################################################################
size_t EyeTrace::numEvents() const
{ return itsNumEvents; }

// ######################################################################
size_t EyeTrace::numSaccades() const
{ LERROR("This function is no longer accurate, use numEvents instead."); 
    return itsNumEvents; } 
// NB: new markup means that we may not be indicating
// saccades here

// ######################################################################
SimTime EyeTrace::period() const
{ return itsPeriod; }

// ######################################################################
std::string EyeTrace::filename() const
{ return itsFilename; }

// ######################################################################
PixRGB<byte> EyeTrace::color() const
{ return itsColor; }

// ######################################################################
std::string EyeTrace::basename() const
{
  size_t idx = itsFilename.rfind('.');
  if (idx != itsFilename.npos) return itsFilename.substr(0, idx);

  return itsFilename; // no extension?
}

// ######################################################################
PixPerDeg EyeTrace::ppd() const
{ return itsPPD; }

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_EYETRACE_C_DEFINED
