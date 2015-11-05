/*!@file Media/MgzJEncoder.C Low-level class to decode mgzj files */

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

#include "Media/MgzJEncoder.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Util/log.H"
#include "Util/sformat.H"
#include "rutz/trace.h"

#include <zlib.h>
#include <sstream>

// ######################################################################
MgzJEncoder::MgzJEncoder(const std::string& fname, const int complev)
{
  LINFO("Opening File: %s", fname.c_str());
  itsFile.open(fname.c_str(), std::ios::out | std::ios::binary);
  itsComplev = complev;
}

// ######################################################################
MgzJEncoder::~MgzJEncoder()
{
  this->close();
}

// ######################################################################
int MgzJEncoder::close()
{
  if(itsFile.is_open())
  {
    LINFO("Please wait... Writing %lu entries to the file journal", (unsigned long)itsJournal.size());
    //Get the current position of the put pointer - this will be the start location of the journal
    uint64 journal_position = itsFile.tellp();

    //Allocate the memory for our journal. 
    //  0: Number of entries     (8 bytes)
    //  8: Flags                 (8 bytes)
    // 16: JOURNAL               (? bytes)
    // ??: Journal Start         (8 bytes)
    size_t journal_size = 8 + 8 + itsJournal.size() * sizeof(MgzJEncoder::journalEntry) + 8;
    byte journal[journal_size];

    //Copy the number of journal entries to the journal buffer
    uint64 num_entries = itsJournal.size();
    memcpy(journal, (char*)&num_entries, 8);

    //Copy the format flags to the journal buffer.
    //This is just reserved space, in case we need to later add some more meta information,
    //such as some versioning of the journal entries, or journal compression, etc...
    uint64 flags = 0;
    memcpy(journal+8, (char*)&flags, 8);

    //Serialize all of our journal entries and write them to disk
    //TODO: Investigate just using a big memcpy or std::copy here, as general
    //consensus says that vectors are always contiguous
    for(size_t jIdx=0; jIdx<itsJournal.size(); jIdx++)
    {

      //Calculate the position of this new journal entry
      size_t jPos = 16+jIdx*sizeof(MgzJEncoder::journalEntry);

      //Copy this entry into the journal buffer
      memcpy(journal+jPos, (char *)&itsJournal[jIdx], sizeof(MgzJEncoder::journalEntry));
    }

    //The last 64 bits of the file will be the file position of the start of the journal
    memcpy(journal + 8 + 8 + itsJournal.size() * sizeof(MgzJEncoder::journalEntry), &journal_position, 8);

    //Write the journal to disk
    itsFile.write((char*)journal, journal_size);

    itsFile.close();
  }
  return 1;
}

// ######################################################################
void MgzJEncoder::writeFrame(const GenericFrame& frame)
{
  //Create a journal entry for this new frame
  MgzJEncoder::journalEntry entry;
  entry.pix_type   = frame.nativeType();
  entry.width      = frame.getWidth();
  entry.height     = frame.getHeight();

  //Fill in the special fields if necessary
  if(frame.nativeType() == GenericFrame::VIDEO)
  {
    entry.flags     = frame.asVideo().getMode();
    entry.byte_swap = frame.asVideo().getByteSwap();
  }
  else
  {
    entry.flags      = frame.floatFlags();
  }

  int    num_pix           = frame.getWidth() * frame.getHeight();
  size_t num_bytes         = -1;
  unsigned char* frameBase = NULL;
  switch(frame.nativeType())
  {
    case GenericFrame::GRAY_U8:
      num_bytes = sizeof(byte) * num_pix;
      frameBase = (unsigned char*)frame.asGrayU8().getArrayPtr();
      break;
    case GenericFrame::GRAY_U16:
      num_bytes = sizeof(uint16) * num_pix;
      frameBase = (unsigned char*)frame.asGrayU16().getArrayPtr();
      break;
    case GenericFrame::GRAY_F32:
      num_bytes = sizeof(float) * num_pix;
      frameBase = (unsigned char*)frame.asGrayF32().getArrayPtr();
      break;
    case GenericFrame::RGB_U8:
      num_bytes = sizeof(PixRGB<byte>) * num_pix;
      frameBase = (unsigned char*)frame.asRgbU8().getArrayPtr();
      break;
    case GenericFrame::RGB_U16:
      num_bytes = sizeof(PixRGB<uint16>) * num_pix;
      frameBase = (unsigned char*)frame.asRgbU16().getArrayPtr();
      break;
    case GenericFrame::RGB_F32:
      num_bytes = sizeof(PixRGB<float>) * num_pix;
      frameBase = (unsigned char*)frame.asRgbF32().getArrayPtr();
      break;
    case GenericFrame::VIDEO:
      num_bytes = frame.asVideo().getBufSize();
      frameBase = (unsigned char*)frame.asVideo().getBuffer();
      break;
    case GenericFrame::NONE:
      //nothing to do here
      break;
    default:
      LFATAL("Cannot write frames of type %s", frame.nativeTypeName().c_str());
  }
  
  //Allocate an output buffer, allowing for some extra space for a header in case
  //our input data is totally incompressable
  size_t outputBufferSize = num_bytes + 1000;
  unsigned char * outputBuffer = new unsigned char[outputBufferSize];

  //Setup our DEFLATE compression stream
  z_stream strm;
  strm.zalloc = Z_NULL;
  strm.zfree  = Z_NULL;
  strm.opaque = Z_NULL;

  int msg = deflateInit(&strm, itsComplev);
  if(msg != Z_OK)
  { 
    std::stringstream reason;
    reason << "Could not initialize mgzj encoder (";
    switch(msg)
    {
      case Z_MEM_ERROR:
        reason << "Z_MEM_ERROR: Insufficient Memory";
        break;
      case Z_STREAM_ERROR:
        reason << "Z_STREAM_ERROR: Likely an invalid compression level (You chose" << itsComplev << ")";
        break;
      case Z_VERSION_ERROR:
        reason << "Z_VERSION_ERROR: You're using an incompatible version of zlib.h";
        break;
      default:
        reason << "Unknown Error!";
        break;
    }
    reason << ")";
    LFATAL("%s", reason.str().c_str()); 
  }

  //Setup the input and output buffers
  strm.avail_in  = num_bytes;
  strm.next_in   = frameBase;
  strm.avail_out = outputBufferSize;
  strm.next_out  = outputBuffer;

  //Perform the actual decompression and just die on any errors
  msg = deflate(&strm, Z_FINISH);
  (void)deflateEnd(&strm);
  if(msg != Z_STREAM_END)
  { 
    std::stringstream reason;
    reason << "Failure to deflate (";
    switch(msg)
    {
      case Z_OK:
        reason << "Z_OK";
        break;
      case Z_STREAM_ERROR:
        reason << "Z_STREAM_ERROR";
        break;
      case Z_BUF_ERROR:
        reason << "Z_BUF_ERROR";
        break;
      default:
        reason << "Unknown Error!";
        break;
    }
    reason << ")";
    LFATAL("%s", reason.str().c_str()); 
  }
  size_t compressedSize = outputBufferSize - strm.avail_out;

  //Fill in the rest of the journal entry for this frame, and push it onto the
  //journal
  entry.start_byte = itsFile.tellp();
  entry.end_byte   = entry.start_byte + compressedSize;
  itsJournal.push_back(entry);

  //Write the compressed frame to disk
  itsFile.write((const char*)outputBuffer, compressedSize);

  delete [] outputBuffer;
}



