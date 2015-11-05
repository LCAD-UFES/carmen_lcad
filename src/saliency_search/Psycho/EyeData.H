/*!@file Psycho/EyeData.H Struct for eye-tracker data with a wrapped ParamMap */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/EyeData.H $
//

#ifndef PSYCHO_EYEDATA_H_DEFINED
#define PSYCHO_EYEDATA_H_DEFINED

#include "Image/Point2D.H"
#include "Psycho/SaccadeState.H"
#include "Util/Types.H"
#include "Util/StringUtil.H"
#include "Util/log.H"
#include "rutz/compat_cmath.h" // for isnan()
#include "rutz/shared_ptr.h"
#include "Component/ParamMap.H"
#include <cmath>

//! Simple struct for eye-tracker data
/*! Different eye trackers may or may not fill all data
  members. Members that should always be filled are x and y. Note that
  this class is costly in terms of storage, so typically one would
  want to use it only to pass data samples around, but not to store a
  long series of samples in memory. For example, Psycho/EyeTrace.H
  uses a more compact format to internally store eye data, and uses
  EyeData objects to deliver that data to whoever wants it. */
class EyeData 
{
public:
  //! Fully initialized constructor, no extra data
  inline EyeData(const float x_, const float y_, const float pdiam_,
                 const SaccadeState sstate_, const bool bstate_);

  //! Fully initialized constructor, extra data in ParamMap
  inline EyeData(const float x_, const float y_, const float pdiam_,
                         const SaccadeState sstate_, const bool bstate_,
                         rutz::shared_ptr<ParamMap> dat);
  
  // initialized constructor, extra data
  inline EyeData(const float x_, const float y_, const float pdiam_,
                        const SaccadeState sstate_, const bool bstate_,
                        const float dx_, const float dy_, const float ampl_,
                        const float dur_, const float time_,
                       const unsigned int num_); 

  //  inline EyeData(const float x_, const float y_, const float pdiam_,
  //             const SaccadeState sstate_, const bool bstate_,
  //	 rutz::shared_ptr<ParamMap> dat);

  //  inline EyeData(rutz::shared_ptr<ParamMap> dat);
  // inline EyeData(std::map<std::string, float> dat);

  inline ~EyeData();

  //@}
  // ######################################################################
  /*! @name Helper functions */
  //@{

 //! Are we in unknown state (uninitialized, loss of tracking, etc)
  inline bool isInUnknown() const;

  //! Are we in fixation?
  inline bool isInFixation() const;

  //! Are we in saccade?
  inline bool isInSaccade() const;

  //! Are we in smooth pursuit?
  inline bool isInSmoothPursuit() const;

  //! Are we in blink?
  inline bool isInBlink() const;

  //! Are we in a series of combined saccades
  inline bool isInCombinedSaccade() const;

  //! Get the saccade state directly
  inline SaccadeState saccadeState() const;

  //! Do we have valid pupil diameter data?
  inline bool hasPupilDiam() const;

  //! Do we have valid extra metadata?
  /*! Only a very few samples in an eye trace have this extra data,
    possibly none. The data may be present at the onset of each
    event and contains information about each event. */
  inline bool hasMetaData(const std::string field) const;

  inline bool hasSpecialMetaData(const std::string field) const;

  //! Do we have valid extra saccade target data?
  /*! Only a very few samples in an eye trace have this extra data,
    possibly none. The data may be present at the onset of each
    saccade and contains information about the target of the saccade. */
  //!NB: This is rewritten now for any sample that has extra data,
  // not just saccades. 
  inline bool hasSaccadeTargetData() const;

  //! Do we have valid (x,y) coordinates?
  inline bool isValid() const;

  //! Are our coordinates within some dims?
  inline bool isWithin(const Dims& dims) const;

  //! Is this field name a special field?
  inline bool isSpecialField(std::string field) const;

  //@}

  // ######################################################################
  /*! @name Access functions */
  //@{

  //! Get eye position, not rounded
  inline void getPosition(float& xx, float& yy) const;

  //! Get eye position, rounded to nearest int coordinates
  inline Point2D<int> position() const;

  //! Get pupil diameter
  /*! Note that this may be a NaN or other trash if there is no
    available data about pupil diameter, check with hasPupilDiameter()
    before if you want to ensure that you get a valid number. */
  inline float pupilDiameter() const;

  //! Get all metadata
  inline rutz::shared_ptr<ParamMap> getMetaData() const;
  
  //! Get list of metadata fields
  inline std::vector<std::string> getMetaDataList() const;

  //! Get one field of metadata
  inline double getMetaDataField(std::string field) const;

  //! Get saccade target, not rounded
  /*! Note that this will contain trash if there is no saccade target
    data. Check with hasMetaData() first to avoid this. */
  inline void getSaccadeTarget(float& targx, float& targy) const;

  //! Get rounded saccade target
  /*! Note that this will contain trash if there is no saccade target
    data. Check with hasMetaData() first to avoid this. */
  inline Point2D<int> saccadeTarget() const;

  //! Get saccade amplitude
  /*! Note that this will contain trash if there is no saccade target
    data. Check with hasMetaData() first to avoid this. */
  inline float saccadeAmplitude() const;

  //! Get saccade duration
  /*! Note that this will contain trash if there is o saccade target
    data. Check with hasMetaData() first to avoid this. */
  inline float saccadeDuration() const;

  //! Get the time at the endpoint of the saccade
  /*! Note that this will contain trash if there is no saccade target
    data. Check with hasMetaData() first to avoid this. */
  inline float saccadeTime() const;

  //! Get saccade number
  /*! Note that this will contain trash if there is no saccade target
    data. Check with hasMetaData() first to avoid this.
    Returned saccade number is zero-based. */
  inline unsigned int saccadeNumber() const;

  //! Output the data for debug purposes only
  /*! Commented out for build purposes. Include fstream to bring it back. */
  //inline void outputData(std::ostream &out) const;
private:
  //Optional data in tree of ParamMaps
  const float x, y;          //!< eye position coordinates, in pixels
  const float pdiam;         //!< pupil diameter, in pixels
  const SaccadeState sstate; //!< See definitions in SaccadeState.H
  const bool bstate;         //!< Blink state
 
  const rutz::shared_ptr<ParamMap> itsExtraData;
  inline std::string special(std::string field) const;
  inline std::string unspecial(std::string field) const;

};

// ######################################################################
// ######################################################################
// ##########               inlined functions                ############
// ######################################################################
// ######################################################################
inline EyeData::EyeData(const float x_, const float y_, const float pdiam_,
                        const SaccadeState sstate_, const bool bstate_) :
  x(x_), y(y_), pdiam(pdiam_), sstate(sstate_), bstate(bstate_),
  itsExtraData(new ParamMap)
{ }

inline EyeData::EyeData(const float x_, const float y_, const float pdiam_,
                 const SaccadeState sstate_, const bool bstate_,
                                        rutz::shared_ptr<ParamMap> dat) :
 x(x_), y(y_), pdiam(pdiam_), sstate(sstate_), bstate(bstate_),
  itsExtraData(dat)
{}
inline EyeData::EyeData(const float x_, const float y_, const float pdiam_,
                        const SaccadeState sstate_, const bool bstate_,
                        const float dx_, const float dy_, const float ampl_,
                        const float dur_, const float time_,
                        const unsigned int num_) :
        x(x_), y(y_), pdiam(pdiam_), sstate(sstate_), bstate(bstate_),
	itsExtraData(new ParamMap)
{
  itsExtraData->putDoubleParam("targetx",dx_);
  itsExtraData->putDoubleParam("targety",dy_);
  itsExtraData->putDoubleParam("ampl",ampl_);
  itsExtraData->putDoubleParam("dur",dur_);
  itsExtraData->putDoubleParam("time",time_);
  itsExtraData->putDoubleParam("num",num_);
}

inline EyeData::~EyeData()
{ }

inline bool EyeData::isInUnknown() const
{ return (sstate == SACSTATE_UNK); }

inline bool EyeData::isInFixation() const{ return (sstate == SACSTATE_FIX); }

inline bool EyeData::isInSaccade() const
{ return (sstate == SACSTATE_SAC) || (sstate == SACSTATE_COM); }

inline bool EyeData::isInSmoothPursuit() const
{ return (sstate == SACSTATE_SMO); }

inline bool EyeData::isInBlink() const
{ return bstate; }

inline bool EyeData::isInCombinedSaccade() const
{ return (sstate == SACSTATE_COM); }

inline SaccadeState EyeData::saccadeState() const
{ return sstate; }

inline bool EyeData::hasPupilDiam() const
{ return !isnan(pdiam); }

inline bool EyeData::hasMetaData(const std::string field = "any") const
{ if(field.compare("any") == 0) return itsExtraData.is_valid();
  else return (itsExtraData.is_valid() && 
               (itsExtraData->hasParam(field) ||
                itsExtraData->hasParam(special(field)) )); }

inline bool EyeData::hasSpecialMetaData(const std::string field) const
{ 
  return (hasMetaData("any") && itsExtraData->hasParam(special(field)));    
}

// in order to make this much more general, we should use a dictionary of similar terms in a file somewhere

inline bool EyeData::hasSaccadeTargetData() const
{
  std::vector<std::string> sac_args;
  // TODO: make enums/DEFINEs for which fields define a saccade
  // or for another region of interest
  const std::string fields("targetx targety");
  split(fields," ", std::back_inserter(sac_args));
  
  for(std::vector<std::string>::iterator iter = sac_args.begin();
      iter != sac_args.end();
      ++iter) 
    if (hasMetaData(*iter) == false) return false; 
  return true;
}

inline bool EyeData::isValid() const
{ return !(isnan(x) || isnan(y)); }

inline bool EyeData::isWithin(const Dims& dims) const
{
  if (isValid() == false) return false;
  const int i = int(x + 0.49999F), j = int(y + 0.49999F);
  return (i >= 0 && i < dims.w() && j >= 0 && j < dims.h());
}

inline bool EyeData::isSpecialField(std::string field) const
{
   return field[0]=='*';
}

inline void EyeData::getPosition(float& xx, float& yy) const
{ xx = x; yy = y; }

inline Point2D<int> EyeData::position() const
{ return Point2D<int>(int(x + 0.49999F), int(y + 0.49999F)); }

inline float EyeData::pupilDiameter() const
{ return pdiam; }

inline rutz::shared_ptr<ParamMap> EyeData::getMetaData() const
{ return itsExtraData; }

inline std::vector<std::string> EyeData::getMetaDataList() const 
{
  std::vector<std::string> argList;
  if(hasMetaData()) 
    for (ParamMap::key_iterator iter = getMetaData()->keys_begin();
         iter != getMetaData()->keys_end();
         ++iter)
      argList.push_back(*iter);
 
  return argList; }
    
inline double EyeData::getMetaDataField(std::string field) const
{ 
  if (hasSpecialMetaData(field)) 
    return itsExtraData->getDoubleParam(special(field));
  else if(hasMetaData(field)) 
    return itsExtraData->getDoubleParam(field);
  else LFATAL("No data field %s in EyeData",field.c_str());
  return -1;
}

inline void EyeData::getSaccadeTarget(float& targx, float& targy) const
{ targx = getMetaDataField("targetx"); 
  targy = getMetaDataField("targety"); }

inline Point2D<int> EyeData::saccadeTarget() const
{ 
  if(hasSaccadeTargetData())
    {
      float dx, dy;
      getSaccadeTarget(dx, dy);
      return Point2D<int>(int(dx + 0.49999F), int(dy + 0.49999F)); 
    }
  else
    { return Point2D<int>(1,1);} // TODO: should throw exception here
}

inline float EyeData::saccadeAmplitude() const
{ return getMetaDataField("amp"); }

inline float EyeData::saccadeDuration() const
{ return itsExtraData->getDoubleParam("dur"); }

inline float EyeData::saccadeTime() const
{ return itsExtraData->getDoubleParam("time"); }

inline unsigned int EyeData::saccadeNumber() const
{ return uint(itsExtraData->getDoubleParam("num")); }

// Commented out for build purposes.
/* 
inline void EyeData::outputData(std::ostream &out) const
{ 
  out << "(x,y,pd) = (" << x << "," << y << "," << pdiam << ")\t" 
      << "status = " << sstate-1 << "\n";  //-1 is for enum->0based status idx
  if(hasMetaData())
    {
      std::vector<std::string> metaFields = getMetaDataList();
      for(std::vector<std::string>::iterator iter = metaFields.begin();
          iter != metaFields.end();
          ++iter) 
        out << *iter << " = " << getMetaDataField(*iter) << "\n";
    }
};
*/
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

inline std::string EyeData::special(std::string field) const
{
  if(field[0]=='*') return field;
  else return "*"+field;
}

inline std::string EyeData::unspecial(std::string field) const
{
  if(field[0]=='*') return field.erase(0,1);
  else return field;
}
#endif // PSYCHO_EYEDATA_H_DEFINED
