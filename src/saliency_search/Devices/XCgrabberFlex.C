/*!@file Devices/XCgrabberFlex.C Interface with a Silicon imaging digital camera */

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
// Primary maintainer for this file: Farhan Baluch <fbaluch@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/XCgrabberFlex.C $
// $Id: XCgrabberFlex.C$
//

#include <cstddef>
#include <string>

#include "Component/OptionManager.H" // for option alias requests
#include "Devices/DeviceOpts.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/MathOps.H"
#include "Raster/GenericFrame.H"
#include "Raster/Raster.H"
#include "Util/Assert.H"
#include "Util/SimTime.H"
#include "Util/sformat.H"
#include "Video/VideoFrame.H"

#include "Devices/XCgrabberFlex.H"

#include <unistd.h> // for usleep()

// wait when polling for a frame (in us):
#define XCWAIT 50
//define the imaging board, (we have only one board)
#define UNITMAP 1
#define USEDBUFFER 16

// ######################################################################
XCgrabberFlex::XCgrabberFlex(OptionManager& mgr,
                                 const std::string& descrName,
                                 const std::string& tagName,
                                 const ParamFlag flags) :
  FrameIstream(mgr, descrName, tagName),
  itsDims(&OPT_FrameGrabberDims, this, Dims(1920, 1080), USE_MY_VAL),
  itsGrabMode(&OPT_FrameGrabberMode, this, VIDFMT_BAYER_GB, USE_MY_VAL),
  itsByteSwap(&OPT_FrameGrabberByteSwap, this, false, USE_MY_VAL),
  itsWhiteBalTarR(&OPT_FrameGrabberWhiteBalTargetR, this, false, USE_MY_VAL),
  itsWhiteBalTarG(&OPT_FrameGrabberWhiteBalTargetG, this, false, USE_MY_VAL),
  itsWhiteBalTarB(&OPT_FrameGrabberWhiteBalTargetB, this, false, USE_MY_VAL),
  itsWhiteBalRefR(&OPT_FrameGrabberWhiteBalReferenceR, this, false, USE_MY_VAL),
  itsWhiteBalRefG(&OPT_FrameGrabberWhiteBalReferenceG, this, false, USE_MY_VAL),
  itsWhiteBalRefB(&OPT_FrameGrabberWhiteBalReferenceB, this, false, USE_MY_VAL),
  itsGamma(&OPT_XCFrameGrabberGamma, this, 1.0, USE_MY_VAL | ALLOW_ONLINE_CHANGES),
  itsFPS(&OPT_FrameGrabberFPS, this, 30.0, USE_MY_VAL)
#ifdef HAVE_XCLIB
  ,itsFormatFile(&OPT_XCFormatFileName, this),
  itsCameraOk(false),
  itsImgBuf(NULL)
#endif
{
#ifdef HAVE_XCLIB
  memset(&itsXclib, 0, sizeof(itsXclib));
  itsXclib.ddch.len = sizeof(itsXclib);
  itsXclib.ddch.mos = XCMOS_LIBS;

  itsStatep = NULL;
  itsLastBuf = 0;
#endif
}

// ######################################################################
void XCgrabberFlex::start1()
{
#ifndef HAVE_XCLIB
  LFATAL("you must have XC support and the xclib library in order to use XCgrabberFlex");
#else
  // open the XC cameralink imaging board
  int i;
  if(!strcmp(itsFormatFile.getVal().c_str(),"noFile"))
    {
      LINFO("use default setup configure format");
      char* format =(char*)("default");
      i = xclib::xclib_open(&itsXclib, NULL,NULL, format, NULL);
    }
  else
    {
      LINFO("use input format file as configure file");
      char* formatFile = (char*)(itsFormatFile.getVal().c_str());
      i = xclib::xclib_open(&itsXclib, NULL, NULL, NULL, formatFile);
    }
  if(i != 0)
    {
      LINFO("error code %d\n", i);
      LFATAL("can not open the XC camera");
    }

  switch(itsGrabMode.getVal())
    {
    case VIDFMT_BAYER_GB12:        itsBitDepth = 12;  break;
    case VIDFMT_BAYER_GR12:        itsBitDepth = 12;  break;
    case VIDFMT_BAYER_RG12:        itsBitDepth = 12;  break;
    case VIDFMT_BAYER_BG12:        itsBitDepth = 12;  break;
    case VIDFMT_BAYER_GB:          itsBitDepth = 8;   break;
    case VIDFMT_BAYER_GR:          itsBitDepth = 8;   break;
    case VIDFMT_BAYER_RG:          itsBitDepth = 8;   break;
    case VIDFMT_BAYER_BG:          itsBitDepth = 8;   break;
    default:                       LFATAL("ERROR in specify the xc grab mode");
    }

  // list basic camera info
  struct xclib::pxdevinfo pxinfo;
  memset(&pxinfo, 0, sizeof(pxinfo));
  pxinfo.ddch.len = sizeof(pxinfo);
  pxinfo.ddch.mos = PXMOS_DEVINFO;

  itsXclib.pxdev.getDevInfo(&(itsXclib.pxdev), UNITMAP, 0, &pxinfo);
  LINFO("find %d baords, frame buffer memory = %.4f Kbytes",
        pxinfo.nunits,(double)pxinfo.memsize/1024);

  //white balance
  WhiteBalance();

  struct xclib::pxlibservice pxlib = itsXclib.pxlib;
  struct xclib::xcdevservice xcdev = itsXclib.xcdev;

  // initialize pxvidstate
  i = pxlib.allocStateCopy(&pxlib, 0, 0, &itsStatep);
  LINFO("allocate state copy (video state), result code: %d", i);

  i = pxlib.initStateCopy(&pxlib, 0, 0,
                         itsStatep, &pxinfo, (char*)("default"), PXMODE_DIGI);
  LINFO("init state copy (video state), result code: %d",i);
  pxlib.defineState(&pxlib, 0, itsStateid, itsStatep);

  //  itsStatep->vidres->x.vidoffset =  640;//1920/2-itsDims.getVal().w()/2;
  //itsStatep->vidres->x.vidoffsend = 1920; //1920/2+itsDims.getVal().w()/2;

  LINFO("pxvidimage dims = %d,%d\n",itsStatep->vidres->x.vidoffset,
        itsStatep->vidres->x.vidoffsend);

  //! show some info of pxvidstate structure
  /*
  LINFO("the pxvidimage bayerpattern: %d, %d, %d, %d, %d, %d\n",
        itsStatep->vidimage->bp.order,
        itsStatep->vidimage->bp.mode,
        itsStatep->vidimage->bp.arg[0],
        itsStatep->vidimage->bp.arg[1],
        itsStatep->vidimage->bp.arg[2],
        itsStatep->vidimage->bp.arg[3]);

  LINFO("the pxvidimage colorspace: %d, %d, %d",
        itsStatep->vidimage->cs.order,
        itsStatep->vidimage->cs.mode,
        (int)itsStatep->vidimage->cs.scale);

  LINFO("the pxvid image whitebalance: %d, %d, %d",
        itsStatep->vidimage->wb.order,
        itsStatep->vidimage->wb.mode,
        (int)itsStatep->vidimage->wb.gamma[0][0]);

  LINFO("the pxvid image sharp :%d, %d, %d, %d, %d",
        itsStatep->vidimage->sh.order,
        itsStatep->vidimage->sh.mode,
        itsStatep->vidimage->sh.scale,
        itsStatep->vidimage->sh.into[0],
        itsStatep->vidimage->sh.from[0]);

  for(int i=0; i<6; i++)
    for(int j=0; j<4; j++)
      {
        itsStatep->vidimage->wb.gamma[i][j] = 100;
        itsStatep->vidimage->wb.darkreference[i][j] = 30;
        itsStatep->vidimage->wb.darktarget[i][j] = 30;
        itsStatep->vidimage->wb.brightreference[i][j] = 120;
        itsStatep->vidimage->wb.brighttarget[i][j] = 200;
      }
  itsStatep->vidimage->wb.mode = 3;
  itsStatep->vidimage->wb.order = 1;

  itsStatep->vidimage->sh.order = 1;
  itsStatep->vidimage->sh.mode = 3;
  itsStatep->vidimage->sh.into[0] = 120;
  itsStatep->vidimage->sh.into[1] = 120;
  itsStatep->vidimage->sh.into[2] = 120;
  itsStatep->vidimage->sh.from[0] = 200;
  itsStatep->vidimage->sh.from[1] = 200;
  itsStatep->vidimage->sh.from[2] = 200;
  itsStatep->vidimage->sh.scale = 2;
  itsStatep->vidimage->sh.arg[0] = 1;
  itsStatep->vidimage->sh.arg[1] = 1;

  i = xcdev.setCameraConfig(&xcdev, UNITMAP, 0, 0, itsStatep, NULL);
  LINFO("set camera config res code: %d", i);
  LINFO("after wb, gamma is %d", (int)itsStatep->vidimage->wb.gamma[0][0]);

  i = pxlib.exportStateCopy(&pxlib,0, itsStateid, itsStatep, 0,(char*)
                            "trash_xc_format.txt",NULL,NULL,NULL);
  LINFO("export state res code: %d", i);
  */
  i = xcdev.setVideoConfig(&xcdev,UNITMAP, 0, 0, itsStatep, NULL);
  LINFO("set video configure code %d", i);


  i = xcdev.setLiveSeqBuf
    (&xcdev, UNITMAP, 0, 0, itsStatep, NULL, 1, USEDBUFFER, 1, 0, 1, 0);
  if(i != 0)
    {
      LINFO("start capture error code %d", i);
      LFATAL("the imaging board can not work on live mode\n");
    }

  // make sure the camera start to work for capture
  // get the captured buffer ID
  xclib::pxbuffer_t bufferID =
    (xclib::pxbuffer_t)xcdev.getLiveStatus(&xcdev, UNITMAP, 0, PXVIST_DONE | PXVIST_BUFFER);
  if(bufferID ==0)    LINFO("Grab not ready...");

  while( bufferID == 0 )
    { usleep(XCWAIT);
      bufferID = (xclib::pxbuffer_t)xcdev.getLiveStatus
        (&xcdev, UNITMAP, 0, PXVIST_DONE | PXVIST_BUFFER);
    }

  const unsigned int bufSz = itsDims.getVal().sz() * (int)ceil(itsBitDepth/8);
  itsImgBuf = (byte*)malloc(bufSz);

  itsCameraOk = true;

#endif // HAVE_XCLIB
}

// ######################################################################
void XCgrabberFlex::stop2()
{
#ifndef HAVE_XCLIB
  // don't LFATAL() in stop() since it may be called in a destructor chain
  LERROR("you must have XC  support and the xclib library in order to use XCgrabberFlex");
#else
  if (itsCameraOk)
    {
      xclib_close(&itsXclib);
      itsCameraOk = false;
    }
#endif // HAVE_XCLIB
}

// ######################################################################
XCgrabberFlex::~XCgrabberFlex()
{
#ifdef HAVE_XCLIB
      free(itsImgBuf);
#endif
}

// ######################################################################
void XCgrabberFlex::WhiteBalance()
{
#ifdef HAVE_XCLIB
  /*
  LINFO("Please put a white board in front of the camera then press any key");
  Raster::waitForKey();

  LINFO("ready for white balance");
  pxd_doSnap(UNITMAP, 1, 0);
  uint reference[3] = {0,0,0};
  uint target[3] = {0,0,0};

  if(0 != pxd_setImageBrightBalance(1,reference,target, 0.0))
    LFATAL("can not initial the bright balance");

  double masses[] = {0, 0, 0};
  int sz = 99;
  ushort* pixels = (ushort*)malloc(sz*sz*sizeof(ushort));
  int     midx, midy, i;

  midx = pxd_imageXdim()/2;
  midy = pxd_imageYdim()/2;
  pxd_readushort(UNITMAP,1,midx-sz/2,midy-sz/2,
                 midx+1+sz/2,midy+1+sz/2,pixels,sz*sz,(char*)"RofRGB");
  for (i = 0; i < sz*sz; i++)
    masses[0] += pixels[i];
  pxd_readushort(UNITMAP,1,midx-sz/2,midy-sz/2,
                 midx+1+sz/2,midy+1+sz/2,pixels,sz*sz,(char*)"GofRGB");
  for (i = 0; i < sz*sz; i++)
    masses[1] += pixels[i];
  pxd_readushort(UNITMAP,1,midx-sz/2,midy-sz/2,
                 midx+1+sz/2,midy+1+sz/2,pixels,sz*sz,(char*)"BofRGB");
  for (i = 0; i < sz*sz; i++)
    masses[2] += pixels[i];
  reference[0] = masses[0]/(sz*sz);
  reference[1] = masses[1]/(sz*sz);
  reference[2] = masses[2]/(sz*sz);
  target[0] = target[1] = target[2]
    = std::max(std::max(reference[0], reference[1]), reference[2]);

  if(0 != pxd_setImageBrightBalance(UNITMAP, reference, target, 0.60))
    LFATAL("can not do the bright balance");
  LINFO("white balance finished");

  free(pixels);
  */

#endif
}

// ######################################################################
GenericFrameSpec XCgrabberFlex::peekFrameSpec()
{
  GenericFrameSpec result;

  result.nativeType = GenericFrame::VIDEO;
  result.videoFormat = itsGrabMode.getVal();
  result.videoByteSwap = itsByteSwap.getVal();
  result.dims = itsDims.getVal();
  result.floatFlags = 0;

  return result;
}

// ######################################################################
SimTime XCgrabberFlex::getNaturalFrameTime() const
{
  return SimTime::HERTZ(itsFPS.getVal());
}



// ######################################################################
GenericFrame XCgrabberFlex::readFrame()
{
  return GenericFrame(this->grabRaw());
}

// ######################################################################
VideoFrame XCgrabberFlex::grabRaw()
{
#ifndef HAVE_XCLIB
  LFATAL("you must have XC support and the xclib library in order to use XCgrabberFlex");
  return VideoFrame();  /* can't happen */
#else
  ASSERT(itsCameraOk);
  int i = 0;

  struct xclib::xcdevservice xcdev = itsXclib.xcdev;
  struct xclib::pxlibservice pxlib = itsXclib.pxlib;

  // get the captured buffer ID
  xclib::pxbuffer_t bufferID = (xclib::pxbuffer_t)xcdev.getLiveStatus
    (&xcdev, UNITMAP, 0, PXVIST_DONE | PXVIST_BUFFER);

  while( bufferID == itsLastBuf)
    {
      bufferID = (xclib::pxbuffer_t)xcdev.getLiveStatus
        (&xcdev, UNITMAP, 0, PXVIST_DONE | PXVIST_BUFFER);
      usleep(100);
    }
  if(itsLastBuf != 0 && bufferID != (itsLastBuf)%USEDBUFFER + 1)
    {
      LINFO("last buf id= %4d, curr buf id= %4d",(int)itsLastBuf,(int)bufferID);
      LERROR("buffer error: buffer mis order");
    }

  pthread_mutex_lock(&qmutex_buf);
  itsLastBuf = bufferID;
  pthread_mutex_unlock(&qmutex_buf);

  // is the captured image base on byte or uint16 type
  int dataMode = (itsBitDepth == 8 ?  PXDATUINT8:PXDATUINT16);

  const unsigned int bufSz = itsDims.getVal().sz() * (int)ceil(itsBitDepth/8);
  const unsigned int imgSz = itsDims.getVal().sz();

  //! define the image from frame buffer
  struct xclib::pximage pximg;

  i = pxlib.initPximage(&pxlib, UNITMAP,
                        &pximg, 1, PXHINTBAYER, 0, itsStateid, bufferID, 0);

  pximg.wind.nw.x = 1920/2 - itsDims.getVal().w()/2;
  pximg.wind.nw.y = 1080/2 - itsDims.getVal().h()/2;
  pximg.wind.se.x = 1920/2 + itsDims.getVal().w()/2;
  pximg.wind.se.y = 1080/2 + itsDims.getVal().h()/2;

  LINFO("pximgsize %d,%d", pximg.wind.nw.x,pximg.wind.se.x);

  if (i<1)
    LFATAL("error, can not define a pximage, code: %d",i);


  if(pximg.ioset(&pximg, PXRXSCAN | PXIWRAP, dataMode, 0x01) < 0)
    {
      LFATAL("error in ioset, can not set frame buffer read");
      return VideoFrame();
    }

  if(imgSz !=  pximg.ioread(&pximg, PXRXSCAN | PXIWRAP, itsImgBuf,bufSz,0,0))
    {
      LFATAL("error in reading frame buffer(size error),"
             "expected size = %d", imgSz);
      return VideoFrame();
    }
  return VideoFrame(itsImgBuf,  bufSz, itsDims.getVal(),
                    itsGrabMode.getVal(), itsByteSwap.getVal(), false);

#endif // HAVE_XCLIB
}

// ######################################################################
void XCgrabberFlex::paramChanged(ModelParamBase* const param,
                                   const bool valueChanged,
                                   ParamClient::ChangeStatus* status)
{
#ifndef HAVE_XCLIB
  LFATAL("you must have XC support and the xclib library in order to use XCgrabberFlex");
#else
  FrameIstream::paramChanged(param, valueChanged, status);
#endif // HAVE_XCLIB
}

// ######################################################################
#ifdef HAVE_XCLIB
xclib::pxbuffer_t XCgrabberFlex::getCurrBufID()
{
  xclib::pxbuffer_t tmp;
  pthread_mutex_lock(&qmutex_buf);
  tmp = itsLastBuf;
  pthread_mutex_unlock(&qmutex_buf);
  return tmp;
}
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

