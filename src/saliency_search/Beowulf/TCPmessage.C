/*!@file Beowulf/TCPmessage.C Direct message passing over TCP connections */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Beowulf/TCPmessage.C $
// $Id: TCPmessage.C 11538 2009-07-30 06:23:37Z itti $
//

#include "Beowulf/TCPmessage.H"

#include "Beowulf/TCPdefs.H"
#include "Image/Image.H"
#include "Image/ImageSet.H"
#include "Image/Pixels.H"
#include "Util/log.H"

#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <unistd.h>

//! use message ID for ID-logging; see log.H:
#define MYLOGID itsHead.itsId

// ######################################################################
TCPmessage::TCPmessage() :
  itsMsg(new ArrayData<char>())
{ itsBusy = false; reset(0, 0, 0.0F); }

// ######################################################################
TCPmessage::TCPmessage(const int32 msgid, const int32 msgaction,
                       const float msgeti) :
  itsMsg(new ArrayData<char>())
{ itsBusy = false; reset(msgid, msgaction, msgeti); }

// ######################################################################
TCPmessage::TCPmessage(const int32 msgid, const int32 msgaction,
                       const float msgeti,
                       const void *buf, const int bufsize) :
  itsMsg(new ArrayData<char>())
{
  itsBusy = false; reset(msgid, msgaction, msgeti);
  resize(bufsize, false);
  memcpy(getMsg(), buf, bufsize);
  itsUpkidx = 0; itsHead.itsSize = bufsize;
}

// ######################################################################
TCPmessage::TCPmessage(const TCPmessage& m) :
  itsMsg(m.itsMsg)
{ itsBusy = m.itsBusy; itsHead = m.itsHead; itsUpkidx = m.itsUpkidx; }

// ######################################################################
TCPmessage& TCPmessage::operator=(const TCPmessage& m)
{
  ArrayHandle<char> msgcopy(m.itsMsg); itsMsg.swap(msgcopy);
  itsBusy = m.itsBusy; itsHead = m.itsHead; itsUpkidx = m.itsUpkidx;
  return *this;
}

// ######################################################################
TCPmessage::~TCPmessage()
{ }

// ######################################################################
void TCPmessage::addImage(const Image< PixRGB<byte> >& im)
{
  int32 stuff[3];
  stuff[0] = TCPMSG_COLBYTEIMA; stuff[1] = im.getWidth();
  stuff[2] = im.getHeight(); pack(stuff, 3);
  pack(im.getArrayPtr(), im.getSize());
}

// ######################################################################
void TCPmessage::addImage(const Image<byte>& im)
{
  int32 stuff[3];
  stuff[0] = TCPMSG_BYTEIMA; stuff[1] = im.getWidth();
  stuff[2] = im.getHeight(); pack(stuff, 3);
  pack(im.getArrayPtr(), im.getSize());
}

// ######################################################################
void TCPmessage::addImage(const Image<float>& im)
{
  int32 stuff[3];
  stuff[0] = TCPMSG_FLOATIMA; stuff[1] = im.getWidth();
  stuff[2] = im.getHeight(); pack(stuff, 3);
  pack(im.getArrayPtr(), im.getSize());
}

// ######################################################################
void TCPmessage::addImageSet(const ImageSet<float>& im)
{
  int32 stuff[2];
  stuff[0] = TCPMSG_FLOATIMASET; stuff[1] = im.size(); pack(stuff, 2);
  for (uint i = 0; i < im.size(); i ++) addImage(im[i]);
}

// ######################################################################
void TCPmessage::addFixation(const Fixation& fix)
{
  int32 typ = TCPMSG_FIXATION;
  pack(&typ, 1); pack(&fix.i, 1); pack(&fix.j, 1); pack(&fix.frame, 1);
}

// ######################################################################
void TCPmessage::addString(const char* str)
{
  int32 typ = TCPMSG_STRING; pack(&typ, 1); pack(str, strlen(str) + 1);
}

// ######################################################################
void TCPmessage::addInt32(const int32 val)
{
  int32 typ = TCPMSG_INT32; pack(&typ, 1); pack(&val, 1);
}

// ######################################################################
void TCPmessage::addInt64(const int64 val)
{
  int32 typ = TCPMSG_INT64; pack(&typ, 1); pack(&val, 1);
}

// ######################################################################
void TCPmessage::addFloat(const float val)
{
  int32 typ = TCPMSG_FLOAT; pack(&typ, 1); pack(&val, 1);
}

// ######################################################################
void TCPmessage::addDouble(const double val)
{
  int32 typ = TCPMSG_DOUBLE; pack(&typ, 1); pack(&val, 1);
}

// ######################################################################
void TCPmessage::reset(const int32 msgid, const int32 msgaction,
                       const float msgeti)
{
  freeMem();           // delete old message if any
  itsHead.itsId = msgid; itsHead.itsAction = msgaction;
  itsHead.itsETI = msgeti; itsHead.itsSize = 0;
}

// ######################################################################
void TCPmessage::freeMem()
{
  while (itsBusy) { LERROR("freeMem() while busy! Sleeping..."); sleep(1); }
  ArrayHandle<char> h;  // handle with empty array
  itsMsg.swap(h);
  itsHead.itsId = 0; itsHead.itsAction = 0; itsHead.itsSize = 0;
  itsHead.itsETI = 0.0F; itsBusy = false; itsUpkidx = 0;
  itsHeadIdx = 0;
}

// ######################################################################
void TCPmessage::getElementRaw(void **elem, int32& typ)
{
  if (itsUpkidx >= itsHead.itsSize)
    LFATAL("Trying to unpack past message end");

  unpack(&typ, 1);  // get element type
  switch(typ)
    {
    case TCPMSG_COLBYTEIMA:
      {
        Image<PixRGB<byte> >* im =
          new Image<PixRGB<byte> >(decodeColByteIma());
        *elem = static_cast<void*>(im);
        break;
      }
    case TCPMSG_BYTEIMA:
      {
        Image<byte>* im = new Image<byte>(decodeByteIma());
        *elem = static_cast<void*>(im);
        break;
      }
    case TCPMSG_FLOATIMA:
      {
        Image<float>* im = new Image<float>(decodeFloatIma());
        *elem = static_cast<void*>(im);
        break;
      }
    case TCPMSG_FLOATIMASET:
      {
        ImageSet<float>* ims = new ImageSet<float>(decodeFloatImaSet());
        *elem = static_cast<void*>(ims);
        break;
      }
    case TCPMSG_FIXATION:
      {
        Fixation* fix = new Fixation(decodeFixation());
        *elem = static_cast<void*>(fix);
        break;
      }
    case TCPMSG_STRING:
      {
        int32 s = strlen(getMsg() + itsUpkidx) + 1;
        char *str = new char[s];
        unpack(str, s); *elem = (void *)str;
        break;
      }
    case TCPMSG_INT32:
      {
        int32 *val = new int32; unpack(val, 1);
        *elem = (void *)val;
        break;
      }
    case TCPMSG_FLOAT:
      {
        float *val = new float; unpack(val, 1);
        *elem = (void *)val;
        break;
      }
    case TCPMSG_DOUBLE:
      {
        double *val = new double; unpack(val, 1);
        *elem = (void *)val;
        break;
      }
    case TCPMSG_INT64:
      {
        int64 *val = new int64; unpack(val, 1);
        *elem = (void *)val;
        break;
      }
    default:
      LFATAL("Bogus element %d", typ);
    }
}

// ######################################################################
Image<PixRGB<byte> > TCPmessage::getElementColByteIma()
{
  unpackAndVerifyType(TCPMSG_COLBYTEIMA);
  return decodeColByteIma();
}

// ######################################################################
Image<byte> TCPmessage::getElementByteIma()
{
  unpackAndVerifyType(TCPMSG_BYTEIMA);
  return decodeByteIma();
}

// ######################################################################
Image<float> TCPmessage::getElementFloatIma()
{
  unpackAndVerifyType(TCPMSG_FLOATIMA);
  return decodeFloatIma();
}

// ######################################################################
ImageSet<float> TCPmessage::getElementFloatImaSet()
{
  unpackAndVerifyType(TCPMSG_FLOATIMASET);
  return decodeFloatImaSet();
}

// ######################################################################
int32 TCPmessage::getElementInt32()
{
  unpackAndVerifyType(TCPMSG_INT32);
  int32 result; unpack(&result, 1); return result;
}

// ######################################################################
int64 TCPmessage::getElementInt64()
{
  unpackAndVerifyType(TCPMSG_INT64);
  int64 result; unpack(&result, 1); return result;
}

// ######################################################################
double TCPmessage::getElementDouble()
{
  unpackAndVerifyType(TCPMSG_DOUBLE);
  double result; unpack(&result, 1); return result;
}

// ######################################################################
float TCPmessage::getElementFloat()
{
  unpackAndVerifyType(TCPMSG_FLOAT);
  float result; unpack(&result, 1); return result;
}

// ######################################################################
Fixation TCPmessage::getElementFixation()
{
  unpackAndVerifyType(TCPMSG_FIXATION);
  return decodeFixation();
}

// ######################################################################
std::string TCPmessage::getElementString()
{
  unpackAndVerifyType(TCPMSG_STRING);
  const size_t sz = strlen(getMsg() + itsUpkidx);
  std::string result;
  result.resize(sz);
  unpack(result.data(), sz);
  char nullterm = 1;
  unpack(&nullterm, 1);
  ASSERT(nullterm == '\0');
  return result;
}

namespace
{
  const char* tcpmsgFieldTypeName(int32 typ)
  {
    switch (typ)
      {
#define DO_CASE(x) case x: return #x

        DO_CASE(TCPMSG_COLBYTEIMA); break;
        DO_CASE(TCPMSG_FIXATION); break;
        DO_CASE(TCPMSG_STRING); break;
        DO_CASE(TCPMSG_BYTEIMA); break;
        DO_CASE(TCPMSG_FLOATIMA); break;
        DO_CASE(TCPMSG_FLOATIMASET); break;
        DO_CASE(TCPMSG_INT32); break;
        DO_CASE(TCPMSG_FLOAT); break;
        DO_CASE(TCPMSG_DOUBLE); break;
        DO_CASE(TCPMSG_INT64); break;

#undef DO_CASE
      }

    // default:
    return "UNKNOWN";
  }
}

// ######################################################################
void TCPmessage::unpackAndVerifyType(int32 expected_type)
{
  if (itsUpkidx >= itsHead.itsSize)
    LFATAL("Trying to unpack past message end");

  int32 typ; unpack(&typ, 1);
  if (typ != expected_type)
    LFATAL("expected TCPmessage field %s, but got %s",
           tcpmsgFieldTypeName(expected_type),
           tcpmsgFieldTypeName(typ));
}

// ######################################################################
Image<PixRGB<byte> > TCPmessage::decodeColByteIma()
{
  int32 xx[2]; unpack(xx, 2);
  Image< PixRGB<byte> > im(xx[0], xx[1], NO_INIT);
  unpack(im.getArrayPtr(), xx[0] * xx[1]);
  return im;
}

// ######################################################################
Image<byte> TCPmessage::decodeByteIma()
{
  int32 xx[2]; unpack(xx, 2);
  Image<byte> im(xx[0], xx[1], NO_INIT);
  unpack(im.getArrayPtr(), xx[0] * xx[1]);
  return im;
}

// ######################################################################
Image<float> TCPmessage::decodeFloatIma()
{
  int32 xx[2]; unpack(xx, 2);
  Image<float> im(xx[0], xx[1], NO_INIT);
  unpack(im.getArrayPtr(), xx[0] * xx[1]);
  return im;
}

// ######################################################################
ImageSet<float> TCPmessage::decodeFloatImaSet()
{
  int32 numimg; unpack(&numimg, 1);
  ImageSet<float> ims(numimg);
  for (int i = 0; i < numimg; i ++)
    // store the image into the set
    ims[i] = this->getElementFloatIma();
  return ims;
}

// ######################################################################
Fixation TCPmessage::decodeFixation()
{
  Fixation fix;
  unpack(&fix.i, 1);
  unpack(&fix.j, 1);
  unpack(&fix.frame, 1);
  return fix;
}

// ######################################################################
int TCPmessage::readHeaderFrom(const int fd)
{
  if (itsHeadIdx == 0) freeMem(); // delete old message if any
  int toread = int(sizeof(TCPmessageHeader) - itsHeadIdx);
  if (toread <= 0) { LERROR("Internal error!"); return TCPBUG; }
  int nb = read(fd, (char*)(&itsHead) + itsHeadIdx, toread);
  if (nb == toread) { // ok, done reading
    if (itsHead.itsSize > 0)
      {
        resize(itsHead.itsSize, false);  // allocate memory for body
        itsBusy = true;                  // ready to receive the body
        itsHeadIdx = 0;                  // not reading header anymore
      }
    return TCPDONE;                       // done with header part
  } else if (nb > 0) {  // got only partial header data
    itsHeadIdx += nb; return TCPWAITREAD;
  } else if (nb == -1) {                                        // error?
    if (errno == EAGAIN) return TCPWAITREAD;   // ok, nothing received; wait
    else { PLDEBUG("Read error"); return TCPBUG; }
  } else if (nb == 0) {
    LDEBUG("Peer closed connection. Abort."); return TCPBUG;
  } else { LERROR("What is that?"); return TCPBUG; }
}

// ######################################################################
int TCPmessage::readFrom(const int fd)
{
  if (!itsBusy) { LERROR("not busy?"); itsBusy = true; itsUpkidx = 0; }
  int n = itsHead.itsSize - itsUpkidx; // number of bytes to read
  if (n <= 0) LFATAL("Bogus read request for %d bytes",  n);
  int nb = read(fd, getMsg() + itsUpkidx, n);
  //LDEBUG("%d/%d bytes from %d", nb + itsUpkidx, itsHead.itsSize, fd);
  if (nb == n) {                                      // ok, done reading
    itsBusy = false; itsUpkidx = 0;
    //LDEBUG("Receive complete");
    return TCPDONE;
  } else if (nb > 0) {             // ok, we received some; wait for more
    itsUpkidx += nb; return TCPWAITREAD;
  } else if (nb == -1) {                                        // error?
    if (errno == EAGAIN) return TCPWAITREAD;   // ok, nothing received; wait
    else { PLDEBUG("Read error"); return TCPBUG; }
  } else if (nb == 0) {
    LDEBUG("Peer closed connection. Abort."); return TCPBUG;
  } else { LERROR("What is that?"); return TCPBUG; }
}

// ######################################################################
int TCPmessage::writeHeaderTo(const int fd)
{
  if (!itsBusy) { itsBusy = true; itsUpkidx = 0; }      // start transfer
  int nb = write(fd, &itsHead, sizeof(TCPmessageHeader));
  if (nb == sizeof(TCPmessageHeader)) {                 // ok, done writing
    // do we have an empty body?
    if (itsHead.itsSize <= 0) itsBusy = false;
    return TCPDONE;
  } else {
    // ideally we should wait & retry but this seems to never happen
    LERROR("Could not write header in one shot");
    return TCPBUG;
  }
}

// ######################################################################
int TCPmessage::writeTo(const int fd)
{
  if (!itsBusy) { itsBusy = true; itsUpkidx = 0; }      // start transfer
  int n = itsHead.itsSize - itsUpkidx; // number of bytes to write
  if (n <= 0) LFATAL("Bogus write request for %d bytes",  n);
  int nb = write(fd, getMsg() + itsUpkidx, n);
  //LDEBUG("%d/%d bytes to %d", nb + itsUpkidx, itsHead.itsSize, fd);
  if (nb == n) {                                      // ok, done writing
    //LDEBUG("Done writing %d/%d", itsHead.itsId, itsHead.itsAction);
    itsBusy = false; itsUpkidx = 0; return TCPDONE;
  } else if (nb > 0) {          // we could write some, but not all; wait
    itsUpkidx += nb; return TCPWAITWRITE;
  } else if (nb == -1) {                   // we could not write anything
    if (errno == EAGAIN) return TCPWAITWRITE;         // ok, we'll try again
    else { PLDEBUG("Write error"); return TCPBUG; }
  } else if (nb == 0) {
    LDEBUG("Peer closed connection. Abort."); return TCPBUG;
  } else { LERROR("What is that?"); return TCPBUG; }
}

// ######################################################################
void TCPmessage::addShmInfo(const int shmid, const int siz)
{ int32 typ = TCPMSG_SHMINFO; pack(&typ, 1); pack(&shmid, 1); pack(&siz, 1); }

// ######################################################################
void TCPmessage::addShmDone(const int shmid)
{ int32 typ = TCPMSG_SHMDONE; pack(&typ, 1); pack(&shmid, 1); }

// ######################################################################
bool TCPmessage::checkShmInfo(int& shmid, int& siz) const
{
  if (itsHead.itsSize < (int)sizeof(int32)) return false; // no message body
  const int32 *ptr = (const int32 *)getMsg();
  if (ptr[0] == TCPMSG_SHMINFO) { shmid = ptr[1]; siz = ptr[2]; return true; }
  return false;
}

// ######################################################################
bool TCPmessage::checkShmDone(int& shmid) const
{
  if (itsHead.itsSize < (int)sizeof(int32)) return false; // no message body
  const int32 *ptr = (const int32 *)getMsg();
  if (ptr[0] == TCPMSG_SHMDONE) { shmid = ptr[1]; return true; }
  return false;
}

// ######################################################################
void TCPmessage::attach(char *msgbuf, const int siz)
{
  ArrayHandle<char> h(new ArrayData<char>(Dims(siz, 1), msgbuf, WRITE_THRU));
  itsMsg.swap(h); itsHead.itsSize = siz; itsBusy = false; itsUpkidx = 0;
  //LDEBUG("attached to %d bytes at 0x%lx", siz, msgbuf);
}

// ######################################################################
void TCPmessage::detach()
{
  while (itsBusy) { LERROR("detach() while busy! Sleeping..."); sleep(1); }
  ArrayHandle<char> empty; itsMsg.swap(empty);
  itsBusy = false; itsHead.itsSize = 0; itsUpkidx = 0;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
