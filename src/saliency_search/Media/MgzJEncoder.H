/*!@file Media/MgzJEncoder.H Low-level class to decode mgzj files */

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
// Primary maintainer for this file: Randolph Voorhies <voories at usc dot edu>

#ifndef MEDIA_MGZJENCODER_H_DEFINED
#define MEDIA_MGZJENCODER_H_DEFINED

#include "Image/Range.H"
#include "Raster/GenericFrame.H"
#include "Transport/LowLevelEncoder.H"
#include "Util/Types.H"

#include <iostream>
#include <fstream>

class MgzJEncoder : public LowLevelEncoder
{
  public:
    MgzJEncoder(const std::string& fname, const int complev);

    ~MgzJEncoder();

    int close();

    void writeFrame(const GenericFrame& frame);

    //Data structure for a frame entry
    //WARNING: Do not modify this structure in any way! Doing so will make
    //future .mgzJ files completely incompatible with old ones
    struct journalEntry 
    {
      uint64 start_byte;
      uint64 end_byte;
      byte  pix_type;
      uint32 width;
      uint32 height;
      uint32 flags;
      byte byte_swap;
    };
  private:

    std::ofstream itsFile;
    std::vector<journalEntry> itsJournal;
    int itsComplev; 


};




#endif //MEDIA_MGZJENCODER_H_DEFINED

