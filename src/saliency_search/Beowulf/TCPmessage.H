/*!@file Beowulf/TCPmessage.H Direct message passing over TCP connections */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Beowulf/TCPmessage.H $
// $Id: TCPmessage.H 9412 2008-03-10 23:10:15Z farhan $
//

#ifndef TCPMESSAGE_H_DEFINED
#define TCPMESSAGE_H_DEFINED

#include "Util/log.H"
#include "Util/Types.H"  // for int32, etc
#include "Image/ArrayData.H"
#include "Image/Point2D.H"

#include <string.h>
#include <stdlib.h>
#include <sys/shm.h>

template <class T> class Image;
template <class T> class ImageSet;
template <class T> class PixRGB;

//! types of possible message fields:
#define TCPMSG_END             0
#define TCPMSG_SHMINFO         1
#define TCPMSG_SHMDONE         2

#define TCPMSG_COLBYTEIMA      100
#define TCPMSG_FIXATION        101
#define TCPMSG_STRING          102
#define TCPMSG_BYTEIMA         103
#define TCPMSG_FLOATIMA        104
#define TCPMSG_FLOATIMASET     105
#define TCPMSG_INT32           106
#define TCPMSG_FLOAT           107
#define TCPMSG_DOUBLE          108
#define TCPMSG_INT64           109

struct Fixation
{
  Fixation() : i(-1), j(-1), frame(-1) {}

  Fixation(const Point2D<int>& p, int f) : i(p.i), j(p.j), frame(f) {}

  int32 i;
  int32 j;
  int32 frame;
};

//! This class defines a modular message to be passed over TCP connections.
/*! The basic message structure is as follows:
  - one TCPmessageHeader (see private section of TCPmessage definition)
  - [repeat as needed:]
  - field type (int32)
  - field data (size and type depending on field type)
  - field type = TCPMSG_END
  CAUTION: to optimize speed, there is no semaphore when modifying
  the contents of the message (e.g., adding fields) or when parsing it.
  So, be careful with multi-threaded uses!
*/
class TCPmessage
{
public:
  //! uninitialized constructor (for pointers)
  TCPmessage();

  //! constructor with empty message body
  /*! @param msgid a message identification (user-defined)
    @param msgaction additional field where users can specify an
    action to be executed when this message is received
    @param msgeti estimated time to idle, in seconds. This is used for
    automatic load balancing, where the Beowulf class will dispatch
    processing to nodes with shorter eti's in priority, when a
    load-balanced send is requested. If you are not doing load
    balancing, just use the default of 0.0. */
  TCPmessage(const int32 msgid, const int32 msgaction,
             const float msgeti = 0.0F);

  //! constructor with copy of given message body
  /*! NOTE: we do a deep copy of the given buffer, so that it can be
    freed immediately upon return from this constructor */
  TCPmessage(const int32 msgid, const int32 msgaction, const float msgeti,
             const void *buf, const int bufsize);

  //! copy constructor
  TCPmessage(const TCPmessage& m);

  //! assignment
  TCPmessage& operator=(const TCPmessage& m);

  //! destructor
  ~TCPmessage();

  //! method to add elements into a message to be sent out
  void addImage(const Image< PixRGB<byte> >& im);
  //! method to add elements into a message to be sent out
  void addImage(const Image<byte>& im);
  //! method to add elements into a message to be sent out
  void addImage(const Image<float>& im);
  //! method to add elements into a message to be sent out
  void addImageSet(const ImageSet<float>& im);
  //! method to add elements into a message to be sent out
  void addFixation(const Fixation& fix);
  //! method to add elements into a message to be sent out
  void addString(const char* str);
  //! method to add elements into a message to be sent out
  void addInt32(const int32 val);
  //! method to add elements into a message to be sent out
  void addFloat(const float val);
  //! method to add elements into a message to be sent out
  void addDouble(const double val);
  //! method to add elements into a message to be sent out
  void addInt64(const int64 val);

  //! modify message ID
  inline void setID(const int32 id);
  //! get message ID
  inline int32 getID() const;

  //! modify message action
  inline void setAction(const int32 ac);
  //! get message action
  inline int32 getAction() const;

  //! modify message eti (estimated time to idle)
  inline void setETI(const float eti);
  //! get message eti (estimated time to idle)
  inline float getETI() const;

  //! get total message size (mostly for debuging purposes)
  inline int32 getSize() const;

  //! check whether message is busy (sending or receiving data):
  inline bool isBusy() const;

  //! empty old contents and get ready for a re-use:
  void reset(const int32 msgid, const int32 msgaction,
             const float msgeti = 0.0F);

  //! free memory:
  void freeMem();

  //! DONT'T USE THIS; use one of the type-specific getElement functions below
  /*! OK, only use this function if you need to handle multiple
      possible message types; if you know for sure which type of
      element you expect, then it is safer and more efficient to call
      one of the type-specific functions like getElementByteIma() or
      getElementInt32(); see below.

      If you do need to use this raw function, then after calling it,
      typ will contain TCPMSG_xxx so that the type of elem can be
      inferred. CAUTION: object will be allocated with new; you must
      delete it when done with it!
  */
  void getElementRaw(void **elem, int32& typ);

  //! Get a color byte image from the message stream
  /*! The current message element must have type TCPMSG_COLBYTEIMA */
  Image<PixRGB<byte> > getElementColByteIma();

  //! Get a byte image from the message stream
  /*! The current message element must have type TCPMSG_BYTEIMA */
  Image<byte> getElementByteIma();

  //! Get a float image from the message stream
  /*! The current message element must have type TCPMSG_FLOATIMA */
  Image<float> getElementFloatIma();

  //! Get a float image set from the message stream
  /*! The current message element must have type TCPMSG_FLOATIMASET */
  ImageSet<float> getElementFloatImaSet();

  //! Get an int32 value from the message stream
  /*! The current message element must have type TCPMSG_INT32 */
  int32 getElementInt32();

  //! Get an int64 value from the message stream
  /*! The current message element must have type TCPMSG_INT64 */
  int64 getElementInt64();

  //! Get a double value from the message stream
  /*! The current message element must have type TCPMSG_DOUBLE */
  double getElementDouble();

  //! Get a float value from the message stream
  /*! The current message element must have type TCPMSG_FLOAT */
  float getElementFloat();

  //! Get a Fixation from the message stream
  /*! The current message element must have type TCPMSG_FIXATION */
  Fixation getElementFixation();

  //! Get a string value from the message stream
  /*! The current message element must have type TCPMSG_STRING */
  std::string getElementString();

private:
  //! This private struct is the common header in all TCPmessage objects
  struct TCPmessageHeader
  {
    int32 itsId;      //!< user-defined message ID
    int32 itsAction;  //!< user-defined message action
    float itsETI;     //!< user-defined estimated time to idle
    int32 itsSize;    //!< total size of message data, not including header
  };

  //! pack some data into message:
  template <class T> inline void pack(const T *data, const int nitem);

  //! resize message to new size:
  inline void resize(const int target_size, const bool do_copy = true);

  //! unpack some data from message (memory must be pre-allocated):
public:
  template <class T> inline void unpack(const T *data, const int nitem);
private:

  //! get a type field from the buffer and make sure it's what we expected
  /*! will generate an LFATAL() if the type doesn't match expected_type */
  void unpackAndVerifyType(int32 expected_type);

  //! decode a color byte image from the message buffer
  /*! this assumes that you have already extracted the message type
      and verified that it is TCPMSG_COLBYTEIMA */
  Image<PixRGB<byte> > decodeColByteIma();

  //! decode a byte image from the message buffer
  /*! this assumes that you have already extracted the message type
      and verified that it is TCPMSG_BYTEIMA */
  Image<byte> decodeByteIma();

  //! decode a float image from the message buffer
  /*! this assumes that you have already extracted the message type
      and verified that it is TCPMSG_FLOATIMA */
  Image<float> decodeFloatIma();

  //! decode a float image from the message buffer
  /*! this assumes that you have already extracted the message type
      and verified that it is TCPMSG_FLOATIMASET */
  ImageSet<float> decodeFloatImaSet();

  //! decode a Fixation from the message buffer
  /*! this assumes that you have already extracted the message type
      and verified that it is TCPMSG_FIXATION */
  Fixation decodeFixation();

  //! read header in from a socket:
  int readHeaderFrom(const int fd);

  //! read more data in from a socket:
  int readFrom(const int fd);

  //! write header to a socket:
  int writeHeaderTo(const int fd);

  //! write more data to a socket:
  int writeTo(const int fd);

  //! make our private methods accessible to TCPcliServ:
  friend class TCPcliServ;

  //! add shared memory segment info into message
  void addShmInfo(const int shmid, const int siz);

  //! add shared memory segment release order into message
  void addShmDone(const int shmid);

  //! check if message refers to a shared memory segment (true if so)
  bool checkShmInfo(int& shmid, int& siz) const;

  //! check if message refers to a shared memory segment (true if so)
  bool checkShmDone(int& shmid) const;

  //! get pointer to message body (dangerous!)
  inline const char *getMsg() const;

  //! get pointer to message body (dangerous!)
  inline char *getMsg();

  //! attach to an existing message buffer
  /*! CAUTION: you need to detach() before going out of scope
    otherwise your buffer will be free'd. CAUTION: msgbuf must contain
    a valid finalized message size. */
  void attach(char *msgbuf, const int siz);

  //! detach from attached message buffer (no memory freeing)
  void detach();

  bool itsBusy;             //!< being sent/received
  int itsUpkidx;            //!< index for unpacking message contents
  int itsHeadIdx;           //!< index for receiving header data
  TCPmessageHeader itsHead; //!< message header
  ArrayHandle<char> itsMsg; //!< message data (ref-counted)
};

// ######################################################################
// ##### INLINED METHODS:
// ######################################################################

// ######################################################################
inline void TCPmessage::setID(const int32 id)
{ itsHead.itsId = id; }

// ######################################################################
inline int32 TCPmessage::getID() const
{ return itsHead.itsId; }

// ######################################################################
inline void TCPmessage::setAction(const int32 action)
{ itsHead.itsAction = action; }

// ######################################################################
inline int32 TCPmessage::getAction() const
{ return itsHead.itsAction; }

// ######################################################################
inline void TCPmessage::setETI(const float eti)
{ itsHead.itsETI = eti; }

// ######################################################################
inline float TCPmessage::getETI() const
{ return itsHead.itsETI; }

// ######################################################################
inline int32 TCPmessage::getSize() const
{ return itsHead.itsSize; }

// ######################################################################
inline bool TCPmessage::isBusy() const
{ return itsBusy; }

// ######################################################################
template <class T> inline
void TCPmessage::pack(const T *data, const int nitem)
{
  if (nitem == 0) return;  // nothing to do
  resize(itsHead.itsSize + nitem * sizeof(T));
  memcpy((void *)(getMsg() + itsHead.itsSize),
         (const void *)data, nitem * sizeof(T));
  itsHead.itsSize += nitem * sizeof(T);
}

// ######################################################################
inline void TCPmessage::resize(const int target_size, const bool do_copy)
{
  if (itsBusy) LFATAL("Attempted to resize() busy TCPmessage!");
  int siz = itsMsg.get().w() * itsMsg.get().h(); // current allocation

  // do a reallocation of memory if necessary:
  if (siz < target_size) {
    // use exponential size growth to ensure that append operations are
    // amortized in O(N):
    int newsiz = (siz * 3) / 2 + 2048;
    if (newsiz < target_size + 2048) newsiz = target_size + 2048;
    Dims newdims(newsiz, 1);
    ArrayData<char> *a = new ArrayData<char>(newdims);
    ArrayHandle<char> h(a); itsMsg.swap(h);

    // now copy the data if there was any:
    if (do_copy && itsHead.itsSize > 0)
      memcpy(getMsg(), h.get().data(), itsHead.itsSize);
  }
}

// ######################################################################
template <class T> inline
void TCPmessage::unpack(const T *data, const int nitem)
{
  if (nitem == 0) return;  // nothing to do
  memcpy((void *)data, (const void *)(getMsg() + itsUpkidx),
         nitem * sizeof(T));
  itsUpkidx += nitem * sizeof(T);
}

// ######################################################################
inline const char* TCPmessage::getMsg() const
{ return itsMsg.get().data(); }

// ######################################################################
inline char* TCPmessage::getMsg()
{ return itsMsg.uniq().dataw(); }

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
