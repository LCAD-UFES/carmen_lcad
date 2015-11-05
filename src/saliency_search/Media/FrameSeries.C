/*!@file Media/FrameSeries.C a series of frames */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
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
// Primary maintainer for this file: Rob Peters <rjpeters@klab.caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/FrameSeries.C $
// $Id: FrameSeries.C 15444 2012-12-01 04:06:55Z kai $
//

#include <cstddef>
#include <vector>

#include "Component/GlobalOpts.H" // for OPT_TestMode
#include "Component/ModelOptionDef.H"
#include "Component/OptionManager.H"
#include "Component/Plugin.H"
#include "Devices/DC1394Grabber2.H"
#include "Devices/Bumblebee2Grabber.H"
#include "Devices/DiskDataStream.H"
#include "Devices/IEEE1394grabber.H"
#include "Devices/QuickTimeGrabber.H"
#include "Devices/RTSPGrabber.H"
#include "Devices/V4L2grabber.H"
#include "Devices/V4Lgrabber.H"
#include "Devices/KinectGrabber.H"
#include "Devices/OpenNIGrabber.H"
#include "GUI/ImageDisplayStream.H"

#include "Devices/XCgrabber.H"
#include "Devices/XCgrabberFlex.H"
#include "Media/FrameSeries.H"

#ifdef INVT_HAVE_QT4
#  include "GUI/QtDisplayStream4.H" // use the Qt4 version of QtDisplayStream if we have Qt4 installed
#else
#ifdef INVT_HAVE_QT3
#  include "GUI/QtDisplayStream.H"
#endif
#endif

#include "GUI/SDLdisplayStream.H"
#include "Image/CutPaste.H"   // for inplaceEmbed()
#include "Image/Image.H"
#include "Image/Layout.H"
#include "Image/Pixels.H"
#include "Image/ShapeOps.H"   // for rescale()
#include "Media/FrameCounter.H"
#include "Media/MediaOpts.H"
#include "Media/MgzInputStream.H"
#include "Media/MgzJInputStream.H"
#include "Media/MgzOutputStream.H"
#include "Media/MgzJOutputStream.H"
#include "Media/MpegInputStream.H"
#include "Media/MpegOutputStream.H"
#include "Media/HttpOutputStream.H"
#include "Media/MrawvInputStream.H"
#include "Media/MrawvOutputStream.H"
#include "Media/NullOutputStream.H"
#include "Media/SequenceFileStream.H"
#include "Media/UcbMpegOutputStream.H"
#include "Raster/GenericFrame.H"
#include "Transport/BobDeinterlacer.H"
#include "Transport/BufferedFrameIstream.H"
#include "Transport/CoerceVideoFormatOfilt.H"
#include "Transport/ColorbarsInput.H"
#include "Transport/ColorizeOfilt.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameIstreamFactory.H"
#include "Transport/FrameOstreamFactory.H"
#include "Transport/GameOfLifeInput.H"
#include "Transport/HalfFieldDeinterlacer.H"
#include "Transport/HashOutputSeries.H"
#include "Transport/InfoOutputSeries.H"
#include "Transport/LuminanceOfilt.H"
#include "Transport/RandomInput.H"
#include "Transport/DotStimuli.H"
#include "Transport/BarStimuli.H"
#include "Transport/ShiftedImage.H"
#include "Transport/RasterInputSeries.H"
#include "Transport/RasterOutputSeries.H"
#include "Transport/RasterlistInputSeries.H"
#include "Transport/SplitRgbOfilt.H"
#include "Transport/StatsOutputSeries.H"
#include "Transport/TransportOpts.H"
#include "Transport/World2DInput.H"
#include "Transport/World3DInput.H"
#include "Transport/XMLInput.H"
#include "Transport/Stimulus2D.H"
#include "Util/AllocAux.H" // for invt_allocation_set_stats_units()
#include "Util/TextLog.H"
#include "Util/log.H"
#include "Util/sformat.H"

// Private command-line option defs

namespace
{
  bool isRectEmpty(const Rectangle& rect)
  {
    return
      (rect.isValid() == false)
      ||
      (rect.area() == 0);
  }

  template <class T>
  Image<T> doResizeImage(const Image<T>& input,
                         const Rectangle& rect,
                         const Dims& dims,
                         const int zoom,
                         const bool preserveAspect)
  {
    // if we got an empty image, that means we're at the end of our
    // input stream and so we just pass along the empty image as is:
    if (!input.initialized()) return input;

    if (dims.isEmpty() && zoom == 0) return input;

    Image<T> cropped = input;

    if (!isRectEmpty(rect))
      {
        cropped = crop(input, rect, true);
      }

    if (zoom < 0)
      {
        const Dims zoomout(std::max(cropped.getWidth() >> (-zoom), 1),
                           std::max(cropped.getHeight() >> (-zoom), 1));

        cropped = rescale(cropped, zoomout);
      }
    else if (zoom > 0)
      {
        cropped = zoomXY(cropped, 1 << zoom, 1 << zoom);
      }

    if (preserveAspect)
      {
        Image<T> res(dims, ZEROS);
        T bg = T(); bg += 64;
        inplaceEmbed(res, cropped, res.getBounds(), bg, true);
        return res;
      }

    if (dims.isNonEmpty())
      return rescale(cropped, dims);

    return cropped;
  }

  GenericFrame doResizeFrame(const GenericFrame& input,
                             const Rectangle& rect,
                             const Dims& dims,
                             const int zoom,
                             const bool preserveAspect)
  {
    /* Check if we can skip resizing altogether and just return a copy
       of the original image.

       NOTE that this check is critical in allowing efficient
       optimizations based on representations that would are lost when
       we are forced to resize. For example, SDLdisplayStream is
       optimized for displaying YUV VideoFrame objects, and
       ImageDisplayStream is optimized for displaying Layout
       objects. When we are forced to resize a VideoFrame or Layout,
       those optimizations are lost because the frame has to be
       rendered as a plain Image before resizing.
    */
    if (zoom == 0
        && (dims.isEmpty() || dims == input.getDims())
        && !preserveAspect)
      return input;

    switch (input.nativeType())
      {
      case GenericFrame::NONE:
        return GenericFrame();

      case GenericFrame::RGB_U8:
        return GenericFrame(doResizeImage(input.asRgbU8(),
                                          rect, dims, zoom, preserveAspect));

      case GenericFrame::RGBD:
        return GenericFrame(doResizeImage(input.asRgbU8(), rect, dims, zoom, preserveAspect),
                            doResizeImage(input.asGrayU16(), rect, dims, zoom, preserveAspect));

      case GenericFrame::RGB_F32:
        return GenericFrame(doResizeImage(input.asRgbF32(),
                                          rect, dims, zoom, preserveAspect),
                            input.floatFlags());

      case GenericFrame::GRAY_U8:
        return GenericFrame(doResizeImage(input.asGrayU8(),
                                          rect, dims, zoom, preserveAspect));

      case GenericFrame::GRAY_F32:
        return GenericFrame(doResizeImage(input.asGrayF32(),
                                          rect, dims, zoom, preserveAspect),
                            input.floatFlags());

      case GenericFrame::VIDEO:
        // NOTE: if we are going to resize a VideoFrame, we'll first
        // have to convert it to rgb and do the resizing in that
        // format
        return GenericFrame(doResizeImage(input.asRgb(),
                                          rect, dims, zoom, preserveAspect));

      case GenericFrame::RGB_U16:
        return GenericFrame();  break;
      case GenericFrame::GRAY_U16:
        return GenericFrame();  break;

      }

    ASSERT(0); /* can't happen */ return GenericFrame();
  }

  // this is a file-global variable that is needed when we use
  // pthread_once() to initialize the factories with istream_init()
  // and ostream_init(); those init functions need an OptionManager to
  // do the initialization but pthread_once() doesn't allow any args
  // to be passed to the init function, thus we have to communicate
  // the args through an outside variable
  OptionManager* init_manager = 0;

  pthread_once_t istream_init_once = PTHREAD_ONCE_INIT;

  void istream_init()
  {
    ASSERT(init_manager != 0);

    OptionManager& mgr = *init_manager;

    ComponentFactory<FrameIstream>& inTypeFactory = getFrameIstreamTypeFactory();

    // NOTE: If you add input sources or otherwise modify the
    // inTypeFactory, then please also update the documentation of the
    // main "--in" option in the definition of OPT_InputFrameSource in
    // Media/MediaOpts.C so that your changes will be visible to
    // users.

    inTypeFactory.registerType<RasterInputSeries>("raster", mgr);
    inTypeFactory.registerType<RasterlistInputSeries>("rasterlist", mgr);
    inTypeFactory.registerType<InputMPEGStream>("mpeg", mgr);
    inTypeFactory.registerType<InputMPEGStream>("movie", mgr);
    inTypeFactory.registerType<RandomInput>("random", mgr);
    inTypeFactory.registerType<DotStimuli>("dots", mgr);
    inTypeFactory.registerType<BarStimuli>("bars", mgr);
    inTypeFactory.registerType<ShiftedImage>("shiftedImage", mgr);
    inTypeFactory.registerType<GameOfLifeInput>("life", mgr);
    inTypeFactory.registerType<World2DInput>("World2D", mgr);
    inTypeFactory.registerType<World3DInput>("World3D", mgr);
    inTypeFactory.registerType<Stimulus2D>("stimulus2D", mgr);
    inTypeFactory.registerType<XMLInput>("xmlfile", mgr);
#ifdef HAVE_LINUX_VIDEODEV_H
    inTypeFactory.registerType<V4Lgrabber>("v4l", mgr);
#endif
#ifdef HAVE_LINUX_VIDEODEV2_H
    inTypeFactory.registerType<V4L2grabber>("v4l2", mgr);
#endif
    inTypeFactory.registerType<IEEE1394grabber>("ieee1394", mgr);
#ifdef HAVE_XCLIB
    inTypeFactory.registerType<XCgrabber>("XC", mgr);
#endif
#ifdef HAVE_XCLIB
    inTypeFactory.registerType<XCgrabberFlex>("XCFLEX", mgr);
#endif
    inTypeFactory.registerType<DC1394Grabber2>("dc1394v2", mgr);
    inTypeFactory.registerType<Bumblebee2Grabber>("bb2", mgr);
    inTypeFactory.registerType<QuickTimeGrabber>("qtgrab", mgr);
#ifdef INVT_HAVE_LIBFREENECT
    inTypeFactory.registerType<KinectGrabber>("kinect", mgr);
#endif
#ifdef INVT_HAVE_OPENNI
    inTypeFactory.registerType<OpenNIGrabber>("openni", mgr);
#endif
    inTypeFactory.registerType<RTSPGrabber>("rtsp", mgr);
    inTypeFactory.registerType<ColorbarsInput>("colorbars", mgr);
    inTypeFactory.registerType<BobDeinterlacer>("bob", mgr);
    inTypeFactory.registerType<HalfFieldDeinterlacer<true> >("bhf", mgr);
    inTypeFactory.registerType<HalfFieldDeinterlacer<false> >("thf", mgr);
    inTypeFactory.registerType<MgzInputStream>("mgz", mgr);
    inTypeFactory.registerType<MgzJInputStream>("mgzJ", mgr);
    inTypeFactory.registerType<BufferedFrameIstream>("buf", mgr);
    inTypeFactory.registerType<MrawvInputStream>("mraw", mgr);

    inTypeFactory.set_fallback
      (rutz::make_shared(new PluginFallback(init_manager, "FrameIstream")));

    ComponentFactory<FrameIstream>& inExtFactory = getFrameIstreamExtFactory();

    // NOTE: If you add input sources or otherwise modify the
    // inExtFactory, then please also update the documentation of the
    // main "--in" option in the definition of OPT_InputFrameSource in
    // Media/MediaOpts.C so that your changes will be visible to
    // users.

    inExtFactory.registerType<RasterInputSeries>("pnm", mgr);     // RASFMT_PNM
    inExtFactory.registerType<RasterInputSeries>("pgm", mgr);     // RASFMT_PNM
    inExtFactory.registerType<RasterInputSeries>("ppm", mgr);     // RASFMT_PNM
    inExtFactory.registerType<RasterInputSeries>("pbm", mgr);     // RASFMT_PNM
    inExtFactory.registerType<RasterInputSeries>("pfm", mgr);     // RASFMT_PFM
    inExtFactory.registerType<RasterInputSeries>("png", mgr);     // RASFMT_PNG
    inExtFactory.registerType<RasterInputSeries>("jpeg", mgr);    // RASFMT_JPEG
    inExtFactory.registerType<RasterInputSeries>("jpg", mgr);     // RASFMT_JPEG
    inExtFactory.registerType<RasterInputSeries>("dpx", mgr);     // RASFMT_DPX

    inExtFactory.registerType<RasterInputSeries>("grey", mgr);    // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("rgb555", mgr);  // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("rgb565", mgr);  // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("rgb24", mgr);   // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("rgb32", mgr);   // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv24", mgr);   // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuyv", mgr);    // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("uyvy", mgr);    // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv444", mgr);  // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv422", mgr);  // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv411", mgr);  // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv420", mgr);  // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv410", mgr);  // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv444p", mgr); // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv422p", mgr); // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv411p", mgr); // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv420p", mgr); // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv410p", mgr); // RASFMT_RAW_VIDEO

    inExtFactory.registerType<RasterInputSeries>("grey.gz", mgr);    // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("rgb555.gz", mgr);  // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("rgb565.gz", mgr);  // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("rgb24.gz", mgr);   // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("rgb32.gz", mgr);   // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv24.gz", mgr);   // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuyv.gz", mgr);    // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("uyvy.gz", mgr);    // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv444.gz", mgr);  // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv422.gz", mgr);  // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv411.gz", mgr);  // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv420.gz", mgr);  // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv410.gz", mgr);  // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv444p.gz", mgr); // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv422p.gz", mgr); // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv411p.gz", mgr); // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv420p.gz", mgr); // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv410p.gz", mgr); // RASFMT_RAW_VIDEO

    inExtFactory.registerType<RasterInputSeries>("grey.bz2", mgr);    // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("rgb555.bz2", mgr);  // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("rgb565.bz2", mgr);  // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("rgb24.bz2", mgr);   // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("rgb32.bz2", mgr);   // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv24.bz2", mgr);   // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuyv.bz2", mgr);    // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("uyvy.bz2", mgr);    // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv444.bz2", mgr);  // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv422.bz2", mgr);  // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv411.bz2", mgr);  // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv420.bz2", mgr);  // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv410.bz2", mgr);  // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv444p.bz2", mgr); // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv422p.bz2", mgr); // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv411p.bz2", mgr); // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv420p.bz2", mgr); // RASFMT_RAW_VIDEO
    inExtFactory.registerType<RasterInputSeries>("yuv410p.bz2", mgr); // RASFMT_RAW_VIDEO

    inExtFactory.registerType<InputMPEGStream>("avi", mgr);
    inExtFactory.registerType<InputMPEGStream>("mpg", mgr);
    inExtFactory.registerType<InputMPEGStream>("mpeg", mgr);
    inExtFactory.registerType<InputMPEGStream>("m4v", mgr);
    inExtFactory.registerType<InputMPEGStream>("mov", mgr);
    inExtFactory.registerType<InputMPEGStream>("flv", mgr);  // Flash video, supported by ffmpeg
    inExtFactory.registerType<InputMPEGStream>("dv", mgr);
    inExtFactory.registerType<InputMPEGStream>("asf", mgr);
    inExtFactory.registerType<InputMPEGStream>("wmv", mgr);
    inExtFactory.registerType<InputMPEGStream>("m2ts", mgr);

    inExtFactory.registerType<MgzInputStream>("mgz", mgr);
    inExtFactory.registerType<MgzJInputStream>("mgzJ", mgr);

    inExtFactory.registerType<MrawvInputStream>("mgrey", mgr);
    inExtFactory.registerType<MrawvInputStream>("mrgb555", mgr);
    inExtFactory.registerType<MrawvInputStream>("mrgb565", mgr);
    inExtFactory.registerType<MrawvInputStream>("mrgb24", mgr);
    inExtFactory.registerType<MrawvInputStream>("mrgb32", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv24", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuyv", mgr);
    inExtFactory.registerType<MrawvInputStream>("muyvy", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv444", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv422", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv411", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv420", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv410", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv444p", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv422p", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv411p", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv420p", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv410p", mgr);

    inExtFactory.registerType<MrawvInputStream>("mgrey.gz", mgr);
    inExtFactory.registerType<MrawvInputStream>("mrgb555.gz", mgr);
    inExtFactory.registerType<MrawvInputStream>("mrgb565.gz", mgr);
    inExtFactory.registerType<MrawvInputStream>("mrgb24.gz", mgr);
    inExtFactory.registerType<MrawvInputStream>("mrgb32.gz", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv24.gz", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuyv.gz", mgr);
    inExtFactory.registerType<MrawvInputStream>("muyvy.gz", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv444.gz", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv422.gz", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv411.gz", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv420.gz", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv410.gz", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv444p.gz", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv422p.gz", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv411p.gz", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv420p.gz", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv410p.gz", mgr);

    inExtFactory.registerType<MrawvInputStream>("mgrey.bz2", mgr);
    inExtFactory.registerType<MrawvInputStream>("mrgb555.bz2", mgr);
    inExtFactory.registerType<MrawvInputStream>("mrgb565.bz2", mgr);
    inExtFactory.registerType<MrawvInputStream>("mrgb24.bz2", mgr);
    inExtFactory.registerType<MrawvInputStream>("mrgb32.bz2", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv24.bz2", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuyv.bz2", mgr);
    inExtFactory.registerType<MrawvInputStream>("muyvy.bz2", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv444.bz2", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv422.bz2", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv411.bz2", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv420.bz2", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv410.bz2", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv444p.bz2", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv422p.bz2", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv411p.bz2", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv420p.bz2", mgr);
    inExtFactory.registerType<MrawvInputStream>("myuv410p.bz2", mgr);

    inExtFactory.registerType<XMLInput>("xml", mgr);
    inExtFactory.registerType<SequenceFileStream>("seq", mgr);
    inExtFactory.registerType<Stimulus2D>("stim2d", mgr);
  }

  pthread_once_t ostream_init_once = PTHREAD_ONCE_INIT;

  void ostream_init()
  {
    ASSERT(init_manager != 0);

    OptionManager& mgr = *init_manager;

    ComponentFactory<FrameOstream>& outTypeFactory = getFrameOstreamTypeFactory();

    // NOTE: If you add output destinations or otherwise modify the
    // outTypeFactory, then please also update the documentation of
    // the main "--out" option in the definition of
    // OPT_OutputFrameSink in Media/MediaOpts.C so that your changes
    // will be visible to users.

    outTypeFactory.registerType<GenericRasterOutputSeries>("raster", mgr);
    outTypeFactory.registerType<FixedRasterOutputSeries<RASFMT_PNM> >("pnm", mgr);
    outTypeFactory.registerType<FixedRasterOutputSeries<RASFMT_PNM> >("pgm", mgr);
    outTypeFactory.registerType<FixedRasterOutputSeries<RASFMT_PNM> >("ppm", mgr);
    outTypeFactory.registerType<FixedRasterOutputSeries<RASFMT_PNM> >("pbm", mgr);
    outTypeFactory.registerType<FixedRasterOutputSeries<RASFMT_PFM> >("pfm", mgr);
    outTypeFactory.registerType<FixedRasterOutputSeries<RASFMT_PNG> >("png", mgr);
    outTypeFactory.registerType<FixedRasterOutputSeries<RASFMT_RAW_VIDEO> >("rawvideo", mgr);
    outTypeFactory.registerType<FixedRasterOutputSeries<RASFMT_TXT> >("txt", mgr);
    outTypeFactory.registerType<FixedRasterOutputSeries<RASFMT_CCODE> >("ccode", mgr);

    outTypeFactory.registerType<DiskDataStream>("bkg-rawvideo", mgr);

    outTypeFactory.registerType<ImageDisplayStream>("display", mgr);
#ifdef HAVE_SDL_SDL_H
    outTypeFactory.registerType<SDLdisplayStream>("sdl", mgr);
#endif
#if defined(INVT_HAVE_QT3) || defined(INVT_HAVE_QT4)
    outTypeFactory.registerType<QtDisplayStream>("qt", mgr);
#endif
#ifdef INVT_HAVE_AVCODEC
    outTypeFactory.registerType<OutputMPEGStream>("mpeg", mgr);
    outTypeFactory.registerType<OutputMPEGStream>("movie", mgr);
    outTypeFactory.registerType<OutputHttpStream>("http", mgr);
#endif
#ifdef MPEGENCODE_PROG
    outTypeFactory.registerType<UcbMpegOutputStream>("ucbmpeg", mgr);
#endif
    outTypeFactory.registerType<HashOutputSeries>("hash", mgr);
    outTypeFactory.registerType<InfoOutputSeries>("info", mgr);
    outTypeFactory.registerType<StatsOutputSeries>("stats", mgr);
    outTypeFactory.registerType<MgzOutputStream>("mgz", mgr);
    outTypeFactory.registerType<MgzJOutputStream>("mgzJ", mgr);
    outTypeFactory.registerType<MrawvOutputStream>("mraw", mgr);
    outTypeFactory.registerType<NullOutputStream>("null", mgr);
    outTypeFactory.registerType<SplitRgbOfilt>("splitrgb", mgr);
    outTypeFactory.registerType<LuminanceOfilt>("luminance", mgr);
    outTypeFactory.registerType<ColorizeOfilt>("colorize", mgr);

    outTypeFactory.registerType<TCoerceVideoFormatOfilt<VIDFMT_GREY> >("coerce-grey", mgr);
    outTypeFactory.registerType<TCoerceVideoFormatOfilt<VIDFMT_RGB555> >("coerce-rgb555", mgr);
    outTypeFactory.registerType<TCoerceVideoFormatOfilt<VIDFMT_RGB565> >("coerce-rgb565", mgr);
    outTypeFactory.registerType<TCoerceVideoFormatOfilt<VIDFMT_RGB24> >("coerce-rgb24", mgr);
    outTypeFactory.registerType<TCoerceVideoFormatOfilt<VIDFMT_RGB32> >("coerce-rgb32", mgr);
    outTypeFactory.registerType<TCoerceVideoFormatOfilt<VIDFMT_YUV24> >("coerce-yuv24", mgr);
    outTypeFactory.registerType<TCoerceVideoFormatOfilt<VIDFMT_YUYV> >("coerce-yuyv", mgr);
    outTypeFactory.registerType<TCoerceVideoFormatOfilt<VIDFMT_UYVY> >("coerce-uyvy", mgr);
    outTypeFactory.registerType<TCoerceVideoFormatOfilt<VIDFMT_YUV444> >("coerce-yuv444", mgr);
    outTypeFactory.registerType<TCoerceVideoFormatOfilt<VIDFMT_YUV422> >("coerce-yuv422", mgr);
    outTypeFactory.registerType<TCoerceVideoFormatOfilt<VIDFMT_YUV411> >("coerce-yuv411", mgr);
    outTypeFactory.registerType<TCoerceVideoFormatOfilt<VIDFMT_YUV444P> >("coerce-yuv444p", mgr);
    outTypeFactory.registerType<TCoerceVideoFormatOfilt<VIDFMT_YUV422P> >("coerce-yuv422p", mgr);
    outTypeFactory.registerType<TCoerceVideoFormatOfilt<VIDFMT_YUV411P> >("coerce-yuv411p", mgr);
    outTypeFactory.registerType<TCoerceVideoFormatOfilt<VIDFMT_YUV420P> >("coerce-yuv420p", mgr);
    outTypeFactory.registerType<TCoerceVideoFormatOfilt<VIDFMT_YUV410P> >("coerce-yuv410p", mgr);

    outTypeFactory.set_fallback
      (rutz::make_shared(new PluginFallback(init_manager, "FrameOstream")));

    ComponentFactory<FrameOstream>& outExtFactory = getFrameOstreamExtFactory();

    // NOTE: If you add output destinations or otherwise modify the
    // outExtFactory, then please also update the documentation of the
    // main "--out" option in the definition of OPT_OutputFrameSink in
    // Media/MediaOpts.C so that your changes will be visible to
    // users.

    outExtFactory.registerType<FixedRasterOutputSeries<RASFMT_PNM> >("pnm", mgr);
    outExtFactory.registerType<FixedRasterOutputSeries<RASFMT_PNM> >("pgm", mgr);
    outExtFactory.registerType<FixedRasterOutputSeries<RASFMT_PNM> >("ppm", mgr);
    outExtFactory.registerType<FixedRasterOutputSeries<RASFMT_PNM> >("pbm", mgr);
    outExtFactory.registerType<FixedRasterOutputSeries<RASFMT_PFM> >("pfm", mgr);
    outExtFactory.registerType<FixedRasterOutputSeries<RASFMT_PNG> >("png", mgr);
    outExtFactory.registerType<FixedRasterOutputSeries<RASFMT_TXT> >("txt", mgr);
    outExtFactory.registerType<FixedRasterOutputSeries<RASFMT_CCODE> >("C", mgr);

#ifdef INVT_HAVE_AVCODEC
    outExtFactory.registerType<OutputMPEGStream>("mpg", mgr);
    outExtFactory.registerType<OutputMPEGStream>("mpeg", mgr);
    outExtFactory.registerType<OutputMPEGStream>("m4v", mgr);
#endif

    outExtFactory.registerType<MgzOutputStream>("mgz", mgr);
    outExtFactory.registerType<MgzJOutputStream>("mgzJ", mgr);
  }
}

// ######################################################################
// #################### InputFrameSeries
// ######################################################################

struct InputFrameSeries::Impl
{
  Impl(OptionManager& mgr, const FrameRange& range, bool wrap=false)
    :
    source(),
    sourceDescription(),
    echoes(),
    numEchoed(0),
    inputEof(false),
    lastUpdateTime(SimTime::ZERO()),
    counter(range, wrap)
  { 
  }

  nub::soft_ref<FrameIstream> source;
  std::string sourceDescription;
  std::vector<nub::ref<FrameOstream> > echoes;

  int numEchoed;

  bool inputEof;

  SimTime lastUpdateTime;

  FrameCounter counter;

  bool setFrameNumber(int n)
  {
    //If our source doesn't allow us to seek, then don't mess
    //around with anything - just return false to the user.
    if(!this->source->supportsSeek() && this->counter.currentFrame() > n) return false;

    return this->counter.setCurrent(n);
  }

  // auxiliary implementation function around which readRGB(),
  // readGray(), readFloat() are thin wrappers
  GenericFrame readFrame(InputFrameSeries* self)
  {
    // double check we have a valid source
    ASSERT(this->source.is_valid());

    const int fnum = this->counter.currentFrame();
    if (this->source->setFrameNumber(fnum))
      {
        const GenericFrame ima =
          doResizeFrame(this->source->readFrame(),
                        self->itsCropRect.getVal(),
                        self->itsDims.getVal(),
                        /* zoom */ 0,
                        self->itsPreserveAspect.getVal());

        if (ima.initialized())
          {
            LDEBUG("%s frame %d at %.2fms",
                   self->tagName().c_str(), fnum,
                   this->lastUpdateTime.msecs());

            textLog(self->itsLogFile.getVal(),
                    self->tagName().c_str(),
                    sformat("frame %d", fnum),
                    this->lastUpdateTime);

            // send the image out to all echo destinations:
            for (uint i = 0; i < this->echoes.size(); ++i)
              {
                this->echoes[i]->setFrameNumber(fnum);
                this->echoes[i]->writeFrame
                  (ima, "input-echo",
                   FrameInfo("copy of input frame", SRC_POS));
                ++this->numEchoed;
              }

            // set the default stats units so that it will be
            // available for later display of memory stats:
            invt_allocation_set_stats_units(ima.frameSpec().dims.sz());

            return ima;
          }
        else
          LINFO("input exhausted (readFrame() returned empty)");
      }
    else
      LINFO("input exhausted (setFrameNumber() returned false)");

    // something failed, so we're at eof; now keep track of whether
    // we've hit eof, so that we can use that information in update()
    this->inputEof = true;

    return GenericFrame();
  }
};

InputFrameSeries::InputFrameSeries(OptionManager& mgr,
                                   const std::string& descrName,
                                   const std::string& tag) :
  FrameIstream(mgr, "Input "+descrName, "Input"+tag),
  itsLogFile(&OPT_TextLogFile, this),
  itsTestMode(&OPT_TestMode, this),
  itsFrameRange(&OPT_InputFrameRange, this),
  itsFrameWrap(&OPT_InputFramesWrap, this),
  itsCropRect(&OPT_InputFrameCrop, this),
  itsDims(&OPT_InputFrameDims, this),
  itsPreserveAspect(&OPT_InputPreserveAspect, this),
  itsZeroNumberFrames(&OPT_ZeroNumberFrames, this),
  itsFrameSource(&OPT_InputFrameSource, this),
  itsInOut(&OPT_InputOutputComboSpec, this),
  itsInputEcho(&OPT_InputEchoDest, this),
  itsWaitForUser(&OPT_WaitForUser, this),
  itsKeepGoing(&OPT_KeepGoing, this),
  rep(new Impl(mgr, itsFrameRange.getVal(), itsFrameWrap.getVal()))
{
  init_manager = &mgr;
  pthread_once(&istream_init_once, &istream_init);
  pthread_once(&ostream_init_once, &ostream_init);
}

// ######################################################################
InputFrameSeries::~InputFrameSeries()
{
  delete rep;
}

// ######################################################################
void InputFrameSeries::reset1()
{
  // reset some stuff for FrameSeries
  rep->counter.reset(itsFrameRange.getVal(), false, itsFrameWrap.getVal());
  rep->numEchoed = 0;
  rep->inputEof = false;

  // propagate to our base class:
  FrameIstream::reset1();
}


// ######################################################################
void InputFrameSeries::setFrameDims(Dims d)
{
  OptionManager& mgr = getManager();
  mgr.setOptionValString(&OPT_InputFrameDims,
                         convertToString(d));
}

// ######################################################################
Dims InputFrameSeries::getFrameDims()
{
  return itsDims.getVal();
}



// ######################################################################
void InputFrameSeries::paramChanged(ModelParamBase* const param,
                                    const bool valueChanged,
                                    ParamClient::ChangeStatus* status)
{
  FrameIstream::paramChanged(param, valueChanged, status);

  if (param == &itsFrameSource)
    {
      if (itsFrameSource.getVal() != "")
        this->setFrameSource(itsFrameSource.getVal());
    }
  else if (param == &itsInOut)
    {
      if (itsInOut.getVal() != "")
        this->setFrameSource(itsInOut.getVal());
    }
  else if (param == &itsInputEcho)
    {
      OptionManager& mgr = getManager();

      if (itsInputEcho.getVal() == "")
        {
          // ignore
        }
      else if (itsInputEcho.getVal() == "none")
        {
          rep->echoes.clear();
        }
      else
        {
          nub::ref<FrameOstream> f = makeFrameOstream(itsInputEcho.getVal(), mgr);

          rep->echoes.push_back(f);
          this->addSubComponent(f);
          f->exportOptions(MC_RECURSE);
        }
    }
  else if (param == &itsFrameRange)
    {
      rep->counter.reset(itsFrameRange.getVal(), false);
    }
}

// ######################################################################
FrameState InputFrameSeries::update(const SimTime& stime)
{

  // check if we've hit eof, and handle it accordingly
  if (rep->inputEof)
    {
      if (itsKeepGoing.getVal())
        // ok, user wants to keep going even though we're out of input
        return FRAME_SAME;
      else
        // default case is to quit when we're out of input
        return FRAME_COMPLETE;
    }

  rep->numEchoed = 0;

  rep->lastUpdateTime = stime;

  const FrameState result = rep->counter.update(stime);

  if (result != FRAME_SAME)
    {
      const int fnum = rep->counter.currentFrame();

      LDEBUG("%s frame %d at %.2fms",
            rep->source->tagName().c_str(), fnum, stime.msecs());
    }

  return result;
}

// ######################################################################
FrameState InputFrameSeries::updateNext()
{
  return rep->counter.updateNext();
}

// ######################################################################
bool InputFrameSeries::shouldWait() const
{
  return rep->numEchoed > 0
    && itsWaitForUser.getVal() == true
    && !itsTestMode.getVal();
}

// ######################################################################
void InputFrameSeries::start1()
{
  if (!rep->source.is_valid())
    {
      LFATAL("\n\tOops, you didn't specify an input frame source.\n"
             "\tFor example, you can use --in=raster:path/to/filestem,\n"
             "\tor --in=mpeg:path/to/file.mpg, or --in=random:256x256.");
    }

  rep->counter.reset(itsFrameRange.getVal(), false, itsFrameWrap.getVal());

  const int fnum = rep->counter.currentFrame();
  if (!rep->source->setFrameNumber(fnum))
    LFATAL("couldn't initialize frame source %s to frame number %d",
           rep->sourceDescription.c_str(), fnum);

  rep->inputEof = false;
}

// ######################################################################
void InputFrameSeries::start2()
{
  FrameIstream::start2();

  // get the fully cooked input frame dims (after possible resizing
  // and framing):
  const Dims indims = this->peekDims();

  OptionManager& mgr = getManager();


  // Setting the command line option of dim-size here is hacky because 
  // the dim size is now assumed to come from the user even *after* a 
  // change of setFrameSource().  The dims from peekDims() should be used, 
  // and NOT this option... However, I don't know exactly where this is 
  // used yet, so I have an externally controlled fix of calling 
  // setFrameDims(Dims()) before setFrameSource() to destroy 
  // the value (and thus ignoring any user requests)
  // In reality this option must never be touched and peekDims()
  // exclusively used.  -DFP 10152010
  // set the input dims if someone wants them but they are not set:
  Dims idim(0, 0);
  convertFromString(mgr.getOptionValString(&OPT_InputFrameDims), idim);
  if (idim.isEmpty())
    mgr.setOptionValString(&OPT_InputFrameDims,
                           convertToString(indims));

  ASSERT(rep->source.is_valid());

  //Grab a copy of the FrameRange from our actual input source to see if it
  //has any hard restrictions on the frames it can retrieve.  If it does, then
  //create a union of the most restrictive of each of the parameters from the user
  //and the source.
  FrameRange repRng = rep->source->getFrameRange();
  FrameRange usrRng = this->getFrameRange();
  if(repRng != FrameRange())
  {
    std::vector<SimTime> delayTimes(usrRng.numDelayTimes());
    for(size_t i=0; i<usrRng.numDelayTimes(); i++)
      delayTimes.at(i) = usrRng.getDelayTime(i);

    FrameRange newRange(
          repRng.getFirst() > usrRng.getFirst() ? repRng.getFirst() : usrRng.getFirst(),
          usrRng.getStep(),
          repRng.getLast() < usrRng.getLast() ?   repRng.getLast()  : usrRng.getLast(),
          delayTimes,
          usrRng.isEventTriggered()
        );

    rep->counter.reset(newRange, true);
    itsFrameRange.setVal(newRange);
  }




  const GenericFrameSpec origspec = rep->source->peekFrameSpec();
  const GenericFrameSpec finalspec = this->peekFrameSpec();

  const std::string description =
    origspec == finalspec
    ? origspec.getDescription()
    : sformat("originally %s; converted to %s",
              origspec.getDescription().c_str(),
              finalspec.getDescription().c_str());

  LINFO("input frames: %s", description.c_str());
  textLog(itsLogFile.getVal(),
          "InputDims", sformat("%dx%d", indims.w(), indims.h()));
}

// ######################################################################
GenericFrameSpec InputFrameSeries::peekFrameSpec()
{
  // double check we have a valid source
  ASSERT(rep->source.is_valid());

  const int fnum = rep->counter.currentFrame();

  if (!rep->source->setFrameNumber(fnum))
    LFATAL("couldn't initialize frame source %s to frame number %d",
           rep->sourceDescription.c_str(), fnum);

  const GenericFrameSpec origspec = rep->source->peekFrameSpec();

  GenericFrameSpec result = origspec;

  // if we are doing resizing, our dims are the resized dims:
  if (itsDims.getVal().isNonEmpty())
    result.dims = itsDims.getVal();

  // otherwise if we are cropping, our dims are the cropped dims:
  else if (!isRectEmpty(itsCropRect.getVal()))
    result.dims = itsCropRect.getVal().dims();

  // now, if the dims have changed, then a VideoFrame will decay to
  // Image<PixRGB<byte> >
  if (result.dims != origspec.dims
      && origspec.nativeType == GenericFrame::VIDEO)
    result.nativeType = GenericFrame::RGB_U8;

  return result;
}

// ######################################################################
void InputFrameSeries::startStream()
{
  if (!this->started())
    LFATAL("startStream() must not be called until after start()");

  // double check we have a valid source
  ASSERT(rep->source.is_valid());

  rep->source->startStream();
}

// ######################################################################
GenericFrame InputFrameSeries::readFrame()
{
  if (!this->started())
    LFATAL("readFrame() must not be called until after start()");

  return rep->readFrame(this);
}

// ######################################################################
void InputFrameSeries::setFrameSource(const std::string& source)
{
  nub::soft_ref<FrameIstream> in =
    makeFrameIstream(source);

  if (!in.is_valid())
    {
      LFATAL("unknown value for %s: '%s'\n\tvalid values are %s",
             itsFrameSource.getOptionDef()->longoptname,
             source.c_str(),
             itsFrameSource.getOptionDef()->validvals);
    }

  if (rep->source.is_valid())
    this->removeSubComponent(*rep->source);
  rep->source = in;
  rep->sourceDescription = source;
  this->addSubComponent(rep->source);
  rep->source->exportOptions(MC_RECURSE);
}

// ######################################################################
int InputFrameSeries::frame() const
{
  return rep->counter.currentFrame();
}

// ######################################################################
nub::ref<FrameIstream> InputFrameSeries::getFrameSource() const
{
  if (!rep->source.is_valid())
    LFATAL("Oops! I don't have any frame source");

  return rep->source;
}

// ######################################################################
FrameRange InputFrameSeries::getFrameRange() const
{
  return itsFrameRange.getVal();
}

// ######################################################################
bool InputFrameSeries::setFrameNumber(int n)
{
  return rep->setFrameNumber(n);
}

// ######################################################################
const SimTime& InputFrameSeries::getNextFrameTime()
{
  return rep->counter.getNextFrameTime();
}


// ######################################################################
// #################### OutputFrameSeries
// ######################################################################

struct OutputFrameSeries::Impl
{
  Impl(OptionManager& mgr, const FrameRange& range)
    :
    vec(),
    explicitNone(false),
    counter(range),
    wasNonvoidAtStart(false)
  {}

  typedef std::vector<nub::ref<FrameOstream> > VecType;

  VecType vec;

  // by default, we will treat an empty destination list as an error
  // (to prevent somebody spending lots of CPU time only to find out
  // that their results haven't gone anywhere); but, if the user
  // explicitly specifies --out=none, then we'll allow an empty
  // destination list
  bool explicitNone;

  FrameCounter counter;

  bool wasNonvoidAtStart;
};

OutputFrameSeries::OutputFrameSeries(OptionManager& mgr,
                                     const std::string& descrName,
                                     const std::string& tag) :
  FrameOstream(mgr, "Output " + descrName, "Output"+tag),
  itsLogFile(&OPT_TextLogFile, this),
  itsTestMode(&OPT_TestMode, this),
  itsFrameRange(&OPT_OutputFrameRange, this),
  itsDims(&OPT_OutputFrameDims, this),
  itsPreserveAspect(&OPT_OutputPreserveAspect, this),
  itsZoom(&OPT_OutputZoom, this),
  itsZeroNumberFrames(&OPT_ZeroNumberFrames, this),
  itsFrameSink(&OPT_OutputFrameSink, this),
  itsInOut(&OPT_InputOutputComboSpec, this),
  itsWaitForUser(&OPT_WaitForUser, this),
  itsOutputReplicate(&OPT_OutputReplicate, this),
  itsNumWritten(0),
  rep(new Impl(mgr, itsFrameRange.getVal()))
{
  init_manager = &mgr;
  pthread_once(&istream_init_once, &istream_init);
  pthread_once(&ostream_init_once, &ostream_init);
}

// ######################################################################
OutputFrameSeries::~OutputFrameSeries()
{
  delete rep;
}

// ######################################################################
void OutputFrameSeries::reset1()
{
  // reset some stuff for FrameSeries
  rep->counter.reset(itsFrameRange.getVal(), true);
  itsNumWritten = 0;

  rep->explicitNone = false;
  rep->vec.clear();

  // propagate to our base class:
  FrameOstream::reset1();
}

// ######################################################################
void OutputFrameSeries::paramChanged(ModelParamBase* const param,
                                     const bool valueChanged,
                                     ParamClient::ChangeStatus* status)
{
  FrameOstream::paramChanged(param, valueChanged, status);

  if (param == &itsFrameSink)
    {
      this->addFrameDest(itsFrameSink.getVal());
    }
  else if (param == &itsInOut)
    {
      this->addFrameDest(itsInOut.getVal());
    }
  else if (param == &itsFrameRange)
    {
      rep->counter.reset(itsFrameRange.getVal(), false);
    }
}

// ######################################################################
FrameState OutputFrameSeries::update(const SimTime& stime,
                                     bool new_event)
{
  // reset our itsNumWritten:
  itsNumWritten = 0;

  const FrameState result = rep->counter.update(stime, new_event);

  if (result != FRAME_SAME)
    {
      const int fnum = rep->counter.currentFrame();

      LDEBUG("%-20s frame %d at %.2fms",
            this->tagName().c_str(), fnum, stime.msecs());

      textLog(itsLogFile.getVal(),
              this->tagName().c_str(),
              sformat("frame %d", fnum), stime);
    }

  return result;
}

// ######################################################################
FrameState OutputFrameSeries::updateNext()
{
  return rep->counter.updateNext();
}

// ######################################################################
bool OutputFrameSeries::shouldWait() const
{
  return itsNumWritten > 0
    && itsWaitForUser.getVal() == true
    && !itsTestMode.getVal();
}

// ######################################################################
void OutputFrameSeries::writeFrame(const GenericFrame& orig,
                                   const std::string& shortname,
                                   const FrameInfo& auxinfo)
{
  int fnum;
  if(itsZeroNumberFrames.getVal())
    fnum = 0;
  else
    fnum = rep->counter.currentFrame();

  const GenericFrame frame =
    doResizeFrame(orig, Rectangle(), itsDims.getVal(),
                  itsZoom.getVal(), itsPreserveAspect.getVal());

  // double-check that we avoid a silently empty output vector
  ASSERT(rep->vec.size() > 0 || rep->explicitNone);

  for (size_t i = 0; i < rep->vec.size(); ++i)
    {
      for (size_t k = 0; k < itsOutputReplicate.getVal(); ++k)
        {
          rep->vec[i]->setFrameNumber(k + fnum * itsOutputReplicate.getVal());
          rep->vec[i]->writeFrame(frame, shortname, auxinfo);
        }
      ++itsNumWritten;
    }
}

// ######################################################################
bool OutputFrameSeries::isVoid() const
{
  // look for any non-void child streams:
  for (size_t i = 0; i < rep->vec.size(); ++i)
    if (!rep->vec[i]->isVoid())
      return false;

  // no child streams were non-void, so we are void:
  return true;
}

// ######################################################################
bool OutputFrameSeries::becameVoid() const
{
  return rep->wasNonvoidAtStart && this->isVoid();
}

// ######################################################################
void OutputFrameSeries::closeStream(const std::string& shortname)
{
  for (size_t i = 0; i < rep->vec.size(); ++i)
    rep->vec[i]->closeStream(shortname);
}

// ######################################################################
void OutputFrameSeries::addFrameDest(const std::string& source)
{
  if (source == "")
    return;

  OptionManager& mgr = getManager();

  if (source == "none")
    {
      // get rid of any output destinations we may have already
      // created:
      while (rep->vec.size() != 0)
        {
          nub::ref<FrameOstream> f = rep->vec.back();
          rep->vec.pop_back();
          this->removeSubComponent(*f);
        }
      rep->explicitNone = true;
    }
  else
    {
      nub::ref<FrameOstream> f = makeFrameOstream(source, mgr);

      rep->vec.push_back(f);
      this->addSubComponent(f);
      f->exportOptions(MC_RECURSE);
    }
}

// ######################################################################
size_t OutputFrameSeries::getNumFrameDests() const
{
  return rep->vec.size();
}

// ######################################################################
nub::ref<FrameOstream> OutputFrameSeries::getFrameDest(size_t n) const
{
  ASSERT(n < rep->vec.size());

  return rep->vec[n];
}

// ######################################################################
void OutputFrameSeries::start2()
{
  rep->counter.reset(itsFrameRange.getVal(), true);

  if (rep->vec.empty() && !rep->explicitNone)
    LFATAL("\n\tOops, you didn't specify an output destination with\n"
           "\tan --out option. If this is actually the behavior\n"
           "\tyou want, you can do an explicit --out=none to avoid\n"
           "\tthis error message. Otherwise, you can use an option\n"
           "\tlike --out=raster:path/to/filestem or --out=mpeg:\n"
           "\tor --out=display: to request particular output format.");

  rep->wasNonvoidAtStart = !this->isVoid();
}

// ######################################################################
int OutputFrameSeries::frame() const
{
  if(itsZeroNumberFrames.getVal())
    return 0;
  else
    return rep->counter.currentFrame();
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
