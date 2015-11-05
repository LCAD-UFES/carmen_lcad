/*!@file Devices/XCgrabber.C Interface with a Silicon imaging digital camera */

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
// Primary maintainer for this file: Zhicheng Li <zhicheng@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/XCgrabber.C $
// $Id: XCgrabber.C 15310 2012-06-01 02:29:24Z itti $
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

#include "Devices/XCgrabber.H"
#include <unistd.h> // for usleep()

// wait when polling for a frame (in us):
#define XCWAIT 50

// ######################################################################
XCgrabber::XCgrabber(OptionManager& mgr, const std::string& descrName,
                     const std::string& tagName, const ParamFlag flags) :
  FrameIstream(mgr, descrName, tagName),
  itsChannel(&OPT_FrameGrabberChannel, this, 0, USE_MY_VAL),
  itsDims(&OPT_FrameGrabberDims, this, Dims(1920, 1080), USE_MY_VAL),
  itsOffset(&OPT_FrameGrabberOffset, this, Dims(0, 0), USE_MY_VAL),
  itsNbuf(&OPT_FrameGrabberNbuf, this, 4, USE_MY_VAL),
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
  , itsFormatFile(&OPT_XCFormatFileName, this),
  itsCameraOk(false), itsLastBuf(0), itsImgBuf(NULL),
  itsStateid(PXMODE_DIGI), itsUnitMap(0), itsPximg(NULL)
#endif
{
#ifdef HAVE_XCLIB
  memset(&itsXclib, 0, sizeof(itsXclib));
  itsXclib.ddch.len = sizeof(itsXclib);
  itsXclib.ddch.mos = XCMOS_LIBS;
#endif
}

// ######################################################################
void XCgrabber::start1()
{
#ifndef HAVE_XCLIB
  LFATAL("you must have XC support and the xclib library in order to use XCgrabber");
#else

  // define the unit map: here we only support one unit, given by itsChannel:
  itsUnitMap = (1 << itsChannel.getVal());
  const int nbuf = itsNbuf.getVal(); // shortcut
  itsStateid = PXMODE_DIGI + itsChannel.getVal(); // black magic
  struct xclib::pxvidstate *statep;

  // open the XC cameralink imaging board
  if (itsFormatFile.getVal().empty()) {
    LINFO("Using default setup configure format");
    char* format = (char*)("default");
    if (int ret = xclib::xclib_open(&itsXclib, NULL, NULL, format, NULL))
      LFATAL("Cannot open XC library [%s]", err(ret));
  } else {
    LINFO("Using input format file '%s' as configure file", itsFormatFile.getVal().c_str());
    char* formatFile = (char*)(itsFormatFile.getVal().c_str());
    if (int ret = xclib::xclib_open(&itsXclib, NULL, NULL, NULL, formatFile))
      LFATAL("Cannot open XC library [%s]", err(ret));

    // now also actually load the mode defined in the config file (which should correspond to itsStateid) into the
    // board's hardware:
    if (int ret = itsXclib.xcdev.setVideoConfig(&itsXclib.xcdev, itsUnitMap, 0, itsStateid, NULL, NULL))
      LERROR("Set video configure error [%s]", err(ret));
    if (int ret = itsXclib.xcdev.setCameraConfig(&itsXclib.xcdev, itsUnitMap, 1, itsStateid, NULL, NULL))
      LERROR("Set camera config error [%s]", err(ret));
  }

  // shortcuts:
  struct xclib::pxlibservice *pxlib = &(itsXclib.pxlib);
  struct xclib::xcdevservice *xcdev = &(itsXclib.xcdev);

  // mask of which components in the pixies to grab (0x1 for the first component (gray/bayer), 0x7 for the first 3
  // components (rgb):
  int pixiemask = 0x1; // keep compiler happy

  // define bit depth, number of components, and pixiemask based on grab mode:
  switch(itsGrabMode.getVal()) {
  case VIDFMT_BAYER_GB12:  itsBitDepth = 12;  itsNumComp = 1; pixiemask = 0x1; break;
  case VIDFMT_BAYER_GR12:  itsBitDepth = 12;  itsNumComp = 1; pixiemask = 0x1; break;
  case VIDFMT_BAYER_RG12:  itsBitDepth = 12;  itsNumComp = 1; pixiemask = 0x1; break;
  case VIDFMT_BAYER_BG12:  itsBitDepth = 12;  itsNumComp = 1; pixiemask = 0x1; break;
  case VIDFMT_BAYER_GB:    itsBitDepth = 8;   itsNumComp = 1; pixiemask = 0x1; break;
  case VIDFMT_BAYER_GR:    itsBitDepth = 8;   itsNumComp = 1; pixiemask = 0x1; break;
  case VIDFMT_BAYER_RG:    itsBitDepth = 8;   itsNumComp = 1; pixiemask = 0x1; break;
  case VIDFMT_BAYER_BG:    itsBitDepth = 8;   itsNumComp = 1; pixiemask = 0x1; break;
  case VIDFMT_GREY:        itsBitDepth = 8;   itsNumComp = 1; pixiemask = 0x1; break;
  case VIDFMT_RGB24:       itsBitDepth = 8;   itsNumComp = 3; pixiemask = 0x7; break;
  default:                 LFATAL("ERROR unsupported video format for the XC grabber");
  }

  // list basic camera info:
  struct xclib::pxdevinfo pxinfo;
  memset(&pxinfo, 0, sizeof(pxinfo)); pxinfo.ddch.len = sizeof(pxinfo); pxinfo.ddch.mos = PXMOS_DEVINFO;
  if (int ret = itsXclib.pxdev.getDevInfo(&itsXclib.pxdev, itsUnitMap, 0, &pxinfo) != 0)
    LERROR("Error trying to get pxdev info [%s]", err(ret));

  LINFO("Framegrabber board family %d, model %d, submodel %d", pxinfo.family, pxinfo.model, pxinfo.submodel);
  LINFO("Found %d units, frame buffer memory = %.4f Kbytes", pxinfo.nunits, float(pxinfo.memsize) / 1024);
  LINFO("Driver ID: %s", pxinfo.driverid);
  LINFO("Library ID: %s", pxinfo.libraryid);

  // get some info about the xcdev:
  struct xclib::xcdevinfo xcinfo;
  memset(&xcinfo, 0, sizeof(xcinfo)); xcinfo.ddch.len = sizeof(xcinfo); xcinfo.ddch.mos = XCMOS_DEVINFO;
  if (int ret = itsXclib.xcdev.getDevInfo(&itsXclib.xcdev, itsUnitMap, 0, &xcinfo) != 0)
    LERROR("Error trying to get xcdev info [%s]", err(ret));
  LINFO("PCI Bus %d, virq %d", xcinfo.pcibus, xcinfo.virq);

  // initialize pxvidstate
  statep = NULL; // needs to be NULL, will be allocated by allocStateCopy
  if (int ret = pxlib->allocStateCopy(pxlib, 0, 0, &statep))
    LFATAL("Allocate state copy (video state) error [%s]", err(ret));

  if (itsFormatFile.getVal().empty() == false) {
    if (int ret = pxlib->importStateCopy(pxlib, 0, 0, statep, 0, (char *)(itsFormatFile.getVal().c_str()), NULL))
      LFATAL("Import state copy (video state) from '%s' error [%s]", itsFormatFile.getVal().c_str(), err(ret));
  } else {
    if (int ret = pxlib->initStateCopy(pxlib, 0, 0, statep, &pxinfo, (char*)("default"), PXMODE_DIGI))
      LFATAL("Init state copy (video state) error [%s]", err(ret));
  }

  //! show some info of pxvidstate structure
  LINFO("pxvidimage bayerpattern: %d, %d, %d, %d, %d, %d",
        statep->vidimage->bp.order,
        statep->vidimage->bp.mode,
        statep->vidimage->bp.arg[0],
        statep->vidimage->bp.arg[1],
        statep->vidimage->bp.arg[2],
        statep->vidimage->bp.arg[3]);

  LINFO("pxvidimage colorspace: %d, %d, %d",
        statep->vidimage->cs.order,
        statep->vidimage->cs.mode,
        int(statep->vidimage->cs.scale));

  LINFO("pxvidimage whitebalance: %d, %d, %d",
        statep->vidimage->wb.order,
        statep->vidimage->wb.mode,
        int(statep->vidimage->wb.gamma[0][0]));

  LINFO("pxvidimage sharp: %d, %d, %d, %d, %d",
        statep->vidimage->sh.order,
        statep->vidimage->sh.mode,
        statep->vidimage->sh.scale,
        statep->vidimage->sh.into[0],
        statep->vidimage->sh.from[0]);

  // mess around with capture parameters -- I think this actually has no effect on the board!!
  /*
  for (int ii = 0; ii < 6; ++ii)
    for (int jj = 0; jj < 4; ++jj)
      {
        statep->vidimage->wb.gamma[ii][jj] = 100;
        statep->vidimage->wb.darkreference[ii][jj] = 30;
        statep->vidimage->wb.darktarget[ii][jj] = 30;
        statep->vidimage->wb.brightreference[ii][jj] = 120;
        statep->vidimage->wb.brighttarget[ii][jj] = 200;
      }
  LINFO("Gamma is %d", (int)statep->vidimage->wb.gamma[0][0]);

  statep->vidimage->wb.mode = 3;
  statep->vidimage->wb.order = 1;

  statep->vidimage->sh.order = 1;
  statep->vidimage->sh.mode = 3;
  statep->vidimage->sh.into[0] = 120;
  statep->vidimage->sh.into[1] = 120;
  statep->vidimage->sh.into[2] = 120;
  statep->vidimage->sh.from[0] = 200;
  statep->vidimage->sh.from[1] = 200;
  statep->vidimage->sh.from[2] = 200;
  statep->vidimage->sh.scale = 2;
  statep->vidimage->sh.arg[0] = 1;
  statep->vidimage->sh.arg[1] = 1;

  // re-define a "state" (with our itsStateid number) that captures the current modified settings;
  // will be needed later:
  pxlib->defineState(pxlib, 0, itsStateid, statep);

  // update the hardware config using our stateid:
  if (int ret = xcdev->setCameraConfig(xcdev, itsUnitMap, 0, itsStateid, NULL, NULL))
    LERROR("Set camera config error [%s]", err(ret));
  if (int ret = xcdev->setVideoConfig(xcdev, itsUnitMap, 0, itsStateid, NULL, NULL))
    LERROR("Set video configure error [%s]", err(ret));

  // export our state (debug):
  pxlib->exportStateCopy(pxlib, 0, itsStateid, NULL, 0, (char*)"trash_xc_format.txt", NULL, NULL, NULL);
  */

  // setup the camera for live capture of sequences of buffers:
  if (int ret = xcdev->setLiveSeqBuf(xcdev, itsUnitMap, 0, itsStateid, NULL, NULL, 1, nbuf, 1, 0, 1, 0) != 0)
    LFATAL("start capture error [%s]; the imaging board cannot work in live mode", err(ret));
  LINFO("Live capture starting with %d buffers...", nbuf);

  // Define grab region of interest (sub-window):
  xclib::pxywindow pxywin;
  pxywin.nw.x = itsOffset.getVal().w();
  pxywin.nw.y = itsOffset.getVal().h();
  pxywin.se.x = itsDims.getVal().w() + itsOffset.getVal().w();
  pxywin.se.y = itsDims.getVal().h() + itsOffset.getVal().h();

  // is the captured image base on byte or uint16 type
  const int dataMode = (itsBitDepth == 8 ? PXDATUINT8 : PXDATUINT16);

  // define a bunch of pximage buffers; these will allow us to transfer data from the device memory into main memory:
  LINFO("Allocating %d pximage structs for fast I/O transfers", nbuf);
  itsPximg = new struct xclib::pximage*[nbuf];
  for (int i = 0; i < nbuf; ++i) {
    // allocate and initialize each pximage:
    itsPximg[i] = new (struct xclib::pximage);
    if (int ret = pxlib->initPximage(pxlib, itsUnitMap, itsPximg[i], 1, PXHINTFRAME, 0, itsStateid, i+1, 0) < 1)
      LFATAL("Cannot allocate pximage %d/%d [%s]", i, nbuf, err(ret));

    // set the ROI window:
    (void)itsPximg[i]->xwind(itsPximg[i], &pxywin, int('s'));

    // setup buffer I/O access:
    if (int ret = itsPximg[i]->ioset(itsPximg[i], PXRXSCAN | PXIWRAP, dataMode, pixiemask) < 0)
      LFATAL("Cannot setup I/O access for pximage buffer %d/%d [%s]", i, nbuf, err(ret));
  }

  // show frame and window dims:
  xclib::pxywindow *pxyw = itsPximg[0]->xwind(itsPximg[0], NULL, 'i');
  LINFO("Native frame:   [%d .. %d] x [%d .. %d]", pxyw->nw.x, pxyw->se.x, pxyw->nw.y, pxyw->se.y);
  pxyw = itsPximg[0]->xwind(itsPximg[0], NULL, 'w');
  LINFO("Capture window: [%d .. %d] x [%d .. %d]", pxyw->nw.x, pxyw->se.x, pxyw->nw.y, pxyw->se.y);

  // make sure the camera starts to work for capture. Get the captured buffer ID:
  itsLastBuf = (xclib::pxbuffer_t)xcdev->getLiveStatus(xcdev, itsUnitMap, 0, PXVIST_DONE | PXVIST_BUFFER);
  LINFO("Waiting for grabber to start streaming...");
  while (itsLastBuf == 0) {
    usleep(XCWAIT);
    itsLastBuf = (xclib::pxbuffer_t)xcdev->getLiveStatus(xcdev, itsUnitMap, 0, PXVIST_DONE | PXVIST_BUFFER);
  }
  LINFO("Grabber streaming ok.");

  const unsigned int bufSz = itsDims.getVal().sz() * int(ceil(itsBitDepth / 8) * itsNumComp);
  itsImgBuf = (byte*)malloc(bufSz);

  itsCameraOk = true;

#endif // HAVE_XCLIB
}

// ######################################################################
void XCgrabber::stop2()
{
#ifndef HAVE_XCLIB
  // don't LFATAL() in stop() since it may be called in a destructor chain
  LERROR("you must have XC  support and the xclib library in order to use XCgrabber");
#else

  if (itsCameraOk) { xclib::xclib_close(&itsXclib); itsCameraOk = false; }
  if (itsImgBuf) { free(itsImgBuf); itsImgBuf = NULL; }
  if (itsPximg) {
    for (int i = 0; i < itsNbuf.getVal(); ++i) delete itsPximg[i];
    delete [] itsPximg;
    itsPximg = NULL;
  }

#endif // HAVE_XCLIB
}

// ######################################################################
XCgrabber::~XCgrabber()
{ }

// ######################################################################
GenericFrameSpec XCgrabber::peekFrameSpec()
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
SimTime XCgrabber::getNaturalFrameTime() const
{
  return SimTime::HERTZ(itsFPS.getVal());
}

// ######################################################################
GenericFrame XCgrabber::readFrame()
{
  return GenericFrame(this->grabRaw());
}

// ######################################################################
VideoFrame XCgrabber::grabRaw()
{
#ifndef HAVE_XCLIB
  LFATAL("you must have XC support and the xclib library in order to use XCgrabber");
  return VideoFrame();  /* can't happen */
#else
  ASSERT(itsCameraOk);

  // shortcuts:
  struct xclib::xcdevservice *xcdev = &(itsXclib.xcdev);

  // get the captured buffer ID
  xclib::pxbuffer_t bufferID =
    (xclib::pxbuffer_t)xcdev->getLiveStatus(xcdev, itsUnitMap, 0, PXVIST_DONE | PXVIST_BUFFER);
  while (bufferID == itsLastBuf) {
    bufferID = (xclib::pxbuffer_t)xcdev->getLiveStatus(xcdev, itsUnitMap, 0, PXVIST_DONE | PXVIST_BUFFER);
    usleep(XCWAIT);
  }

  if (bufferID != (itsLastBuf % itsNbuf.getVal()) + 1)
    LERROR("Buffer mis-order (dropped frame)! last buf = %d, curr buf = %d", (int)itsLastBuf, (int)bufferID);

  pthread_mutex_lock(&qmutex_buf);
  itsLastBuf = bufferID;
  pthread_mutex_unlock(&qmutex_buf);

  const unsigned int bufSz = itsDims.getVal().sz() * int(ceil(itsBitDepth / 8)) * itsNumComp;
  const unsigned int imgSz = itsDims.getVal().sz();

  unsigned int readsz = itsPximg[bufferID-1]->ioread(itsPximg[bufferID-1], PXRXSCAN | PXIWRAP, itsImgBuf, bufSz, 0, 0);
  if (readsz != imgSz) {
    LFATAL("Error in reading frame buffer (size error), got %d, expected size = %d", readsz, imgSz);
    return VideoFrame();  // keep compiler happy
  }

  return VideoFrame(itsImgBuf, bufSz, itsDims.getVal(), itsGrabMode.getVal(), itsByteSwap.getVal(), false);

#endif // HAVE_XCLIB
}

// ######################################################################
void XCgrabber::paramChanged(ModelParamBase* const param,
                                   const bool valueChanged,
                                   ParamClient::ChangeStatus* status)
{
#ifndef HAVE_XCLIB
  LFATAL("you must have XC support and the xclib library in order to use XCgrabber");
#else
  FrameIstream::paramChanged(param, valueChanged, status);
#endif // HAVE_XCLIB
}

// ######################################################################
#ifdef HAVE_XCLIB
xclib::pxbuffer_t XCgrabber::getCurrBufID()
{
  xclib::pxbuffer_t tmp;
  pthread_mutex_lock(&qmutex_buf);
  tmp = itsLastBuf;
  pthread_mutex_unlock(&qmutex_buf);
  return tmp;
}
#endif

// ######################################################################
const char* XCgrabber::err(const int errcode)
{
#ifdef HAVE_XCLIB
  return itsXclib.pxaux.errorCodeString(&itsXclib.pxaux, errcode);
#else
  return NULL;
#endif
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

