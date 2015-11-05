/*!@file Media/MgzJDecoder.C Low-level class to decode mgzj files */

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

#include "Media/MgzJDecoder.H"
#include <sys/errno.h>

// ######################################################################
MgzJDecoder::MgzJDecoder(const std::string& fname)
  :
  itsFileName(fname)
{
  init();
}

// ######################################################################
MgzJDecoder::~MgzJDecoder()
{
  shutdown();
}


// ######################################################################
void MgzJDecoder::init()
{
  LDEBUG("Opening File: %s", itsFileName.c_str());

  //Open the file in binary input mode, and position the get pointer
  //at the end of the file
  itsFile.open(itsFileName.c_str(), std::ios::in|std::ios::binary|std::ios::ate);

  if(!itsFile.is_open())
    LFATAL("Could not open file: %s", itsFileName.c_str());

  //Get the size of the file
  std::ifstream::pos_type fsize = itsFile.tellg();

  //Find the position of the journal start
  uint64 journal_start;
  itsFile.seekg(-8, std::ios::end);
  itsFile.read((char*)&journal_start, 8);

  //Read in the 16 bytes of meta-meta information
  byte meta_meta_buffer[16];
  itsFile.seekg(journal_start, std::ios::beg);
  itsFile.read((char*)meta_meta_buffer, 16);

  //Get the number of journal entries
  uint64 num_entries;
  memcpy((char*)&num_entries, meta_meta_buffer, 8);

  //Get the flags (for now just assert that they are 0. Anything else
  //should be an error)
  uint64 flags;
  memcpy((char*)&flags, meta_meta_buffer+8, 8);
  ASSERT(flags == 0);

  //Calculate the file offset where the actual journal entries begin
  uint64 journal_data_start = journal_start + 16;

  //Allocate a buffer to store the whole journal, and read it in
  uint64 journal_buf_size = (uint64)fsize - journal_data_start - 8;
  byte journal[journal_buf_size];
  itsFile.seekg(journal_data_start, std::ios::beg);
  itsFile.read((char*)journal, journal_buf_size);

  //Calculate the size of each entry
  uint64 entry_size = journal_buf_size / num_entries;

  //Loop through the buffer to deserialize the journal entries
  itsJournal.clear();
  for(uint64 buf_pos=0; buf_pos < journal_buf_size; buf_pos += entry_size)
  {
    MgzJEncoder::journalEntry entry;
    memcpy((char*)&entry, journal+buf_pos, sizeof(MgzJEncoder::journalEntry));
    itsJournal.push_back(entry);
  }

}

void MgzJDecoder::shutdown()
{
  if(itsFile.is_open())
    itsFile.close();
}

// ######################################################################
GenericFrame MgzJDecoder::readFrame()
{

  // Grab the journal entry for this frame and allocate an appropriate GenericFrame
  MgzJEncoder::journalEntry entry = itsJournal.at(itsFrameNum);
  const Dims dims(entry.width, entry.height);
  const GenericFrame::NativeType pix_type = GenericFrame::NativeType(entry.pix_type);
  const int num_pix = dims.sz();
  GenericFrame frame;

  //Read in the compressed image to a buffer
  uint64 comp_image_buf_size = entry.end_byte - entry.start_byte;
  byte * comp_image_buf = new byte[comp_image_buf_size];
  itsFile.seekg(entry.start_byte, std::ios::beg);
  itsFile.read((char*)comp_image_buf, comp_image_buf_size);

  //Prepare zlib to do the decompression
  z_stream strm;
  strm.zalloc   = Z_NULL;
  strm.zfree    = Z_NULL;
  strm.opaque   = Z_NULL;
  strm.avail_in = 0;
  strm.next_in  = Z_NULL;
  int ret = inflateInit(&strm);
  if(ret != Z_OK) 
   LFATAL("Could not initialize zlib!"); 

  strm.avail_in = comp_image_buf_size;
  strm.next_in  = comp_image_buf;
  switch(pix_type)
  {
    case GenericFrame::GRAY_U8:
      {
        Image<byte> img(dims, NO_INIT);
        strm.avail_out = num_pix * sizeof(byte);
        strm.next_out  = (unsigned char*)img.getArrayPtr();
        ret = inflate(&strm, Z_FINISH);
        if(ret != Z_STREAM_END)
        { LFATAL("Could Not Inflate Frame! %d, %s", ret, strm.msg); }
        frame = GenericFrame(img);

        break;
      }
    case GenericFrame::GRAY_U16:
      {
        Image<uint16> img(dims, NO_INIT);
        strm.avail_out = num_pix * sizeof(uint16);
        strm.next_out  = (unsigned char*)img.getArrayPtr();
        ret = inflate(&strm, Z_FINISH);
        if(ret != Z_STREAM_END)
        { LFATAL("Could Not Inflate Frame! %d, %s", ret, strm.msg); }
        frame = GenericFrame(img);

        break;
      }
    case GenericFrame::GRAY_F32:
      {
        Image<float> img(dims, NO_INIT);
        strm.avail_out = num_pix * sizeof(float);
        strm.next_out  = (unsigned char*)img.getArrayPtr();
        ret = inflate(&strm, Z_FINISH);
        if(ret != Z_STREAM_END)
        { LFATAL("Could Not Inflate Frame! %d, %s", ret, strm.msg); }
        frame = GenericFrame(img, entry.flags);

        break;
      }
    case GenericFrame::RGB_U8:
      {
        Image<PixRGB<byte> > img(dims, NO_INIT);
        strm.avail_out = num_pix * sizeof(PixRGB<byte>);
        strm.next_out  = (unsigned char*)img.getArrayPtr();
        ret = inflate(&strm, Z_FINISH);
        if(ret != Z_STREAM_END)
        { LFATAL("Could Not Inflate Frame! %d, %s", ret, strm.msg); }
        frame = GenericFrame(img);

        break;
      }
    case GenericFrame::RGB_U16:
      {
        Image<PixRGB<uint16> > img(dims, NO_INIT);
        strm.avail_out = num_pix * sizeof(PixRGB<uint16>);
        strm.next_out  = (unsigned char*)img.getArrayPtr();
        ret = inflate(&strm, Z_FINISH);
        if(ret != Z_STREAM_END)
        { LFATAL("Could Not Inflate Frame! %d, %s", ret, strm.msg); }
        frame = GenericFrame(img);

        break;
      }
    case GenericFrame::RGB_F32:
      {
        Image<PixRGB<float> > img(dims, NO_INIT);
        strm.avail_out = num_pix * sizeof(PixRGB<float>);
        strm.next_out  = (unsigned char*)img.getArrayPtr();
        ret = inflate(&strm, Z_FINISH);
        if(ret != Z_STREAM_END)
        { LFATAL("Could Not Inflate Frame! %d, %s", ret, strm.msg); }
        frame = GenericFrame(img, entry.flags);

        break;
      }
      case GenericFrame::VIDEO:
      {
        const size_t vidSize = getFrameSize(VideoFormat(entry.flags), dims);
        ArrayHandle<byte> vidBuffer(new ArrayData<byte>(Dims(vidSize,1), NO_INIT));
        strm.avail_out = vidSize;
        strm.next_out = (unsigned char*)vidBuffer.uniq().dataw();
        ret = inflate(&strm, Z_FINISH);
        if(ret != Z_STREAM_END)
        { LFATAL("Could Not Inflate Frame! %d, %s", ret, strm.msg); }
        frame = GenericFrame(VideoFrame(vidBuffer, dims, VideoFormat(entry.flags), bool(entry.byte_swap)));
        break;
      }
    default:
      LFATAL("Could Not Open Frame Of Type: %d!", pix_type);
  }
  
  inflateEnd(&strm);
  delete [] comp_image_buf;
  return frame;
}

// ######################################################################
int MgzJDecoder::getNumFrames()
{
  return itsJournal.size();
}

// ######################################################################
bool MgzJDecoder::setFrameNumber(unsigned int n)
{
  if(n < itsJournal.size())
  {
    itsFrameNum = n;
    return true;
  }
  LINFO("Could not set frame number to %d (only %lu frames available)", n, (unsigned long)itsJournal.size());
  return false;
}



