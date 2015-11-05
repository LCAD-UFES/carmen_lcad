/*!@file Media/MediaOpts.C */

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
// Primary maintainer for this file:
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/MediaOpts.C $
// $Id: MediaOpts.C 15290 2012-05-13 14:06:48Z kai $
//

#ifndef MEDIA_MEDIAOPTS_C_DEFINED
#define MEDIA_MEDIAOPTS_C_DEFINED

#include "Media/MediaOpts.H"

#include "Component/ModelOptionDef.H"
#include "Image/Dims.H"
#include "Image/Point2D.H"
#include "Image/Rectangle.H"
#include "Media/FrameRange.H"
#include "Transport/TransportOpts.H"

// Format here is:
//
// { MODOPT_TYPE, "name", &MOC_CATEG, OPTEXP_CORE,
//   "description of what option does",
//   "long option name", 'short option name', "valid values", "default value" }
//

// alternatively, for MODOPT_ALIAS option types, format is:
//
// { MODOPT_ALIAS, "", &MOC_ALIAS, OPTEXP_CORE,
//   "description of what alias does",
//   "long option name", 'short option name', "", "list of options" }
//

// NOTE: do not change the default value of any existing option unless
// you really know what you are doing!  Many components will determine
// their default behavior from that default value, so you may break
// lots of executables if you change it.

// #################### FrameSeries handling options:

// Used by: InputFrameSeries, SaccadeControllers
const ModelOptionDef OPT_InputFrameDims =
  { MODOPT_ARG(Dims), "InputFrameDims", &MOC_INPUT, OPTEXP_CORE,
    "Rescale input frames to fixed dims, or 0x0 for no rescaling",
    "rescale-input", '\0', "<width>x<height>", "0x0" };


// Used by: OutputFrameSeries
const ModelOptionDef OPT_OutputFrameDims =
  { MODOPT_ARG(Dims), "OutputFrameDims", &MOC_OUTPUT, OPTEXP_CORE,
    "Rescale output frames to fixed dims, or 0x0 for no rescaling",
    "rescale-output", '\0', "<width>x<height>", "0x0" };

// Used by: OutputFrameSeries
const ModelOptionDef OPT_OutputPreserveAspect =
  { MODOPT_FLAG, "OutputPreserveAspect", &MOC_OUTPUT, OPTEXP_CORE,
    "Preserve output frame aspect ratio if rescaling to fixed dims",
    "preserve-output-aspect", '\0', "", "false" };

// Used by: OutputFrameSeries
const ModelOptionDef OPT_OutputZoom =
  { MODOPT_ARG(int), "OutputZoom", &MOC_OUTPUT, OPTEXP_CORE,
    "Number of levels for output images to be zoomed larger (for positive "
    "values) or smaller (for negative values); e.g. a zoom level of -3 would "
    "reduce the image size by a factor of 8 in both width and height, while "
    "a zoom level of +4 would increase the image size by a factor of 16 in "
    "both width and height. This option is similar to --rescale-output, but "
    "is more useful in the case where you just want to scale the output to "
    "half its normal size, without having to know exactly what the normal "
    "size would be. It is also useful in cases where you will be generating "
    "multiple output streams with different natural sizes; in that case, "
    "--rescale-output would rescale them all to the same size, while "
    "--zoom-output lets you apply a uniform zoom to each of them so that "
    "their relative sizes are maintained.",
    "zoom-output", '\0', "<int>", "0" };

// Used by: Mainly OutputFrameSeries
const ModelOptionDef OPT_ZeroNumberFrames =
  { MODOPT_ARG(bool), "ZeroNumberFrames", &MOC_INPUT, OPTEXP_CORE,
    "Force all input and output frames to have the number 000000",
    "zero-number-frames", '\0', "<true|false>", "false" };

// Used by: InputMPEGStream
const ModelOptionDef OPT_InputMPEGStreamPreload =
  { MODOPT_ARG(bool), "InputMPEGStreamPreload", &MOC_INPUT, OPTEXP_CORE,
    "Whether to preload the entire mpeg input movie.",
    "preload-mpeg", '\0', "<true|false>", "false" };

// Used by: InputFrameSeries
const ModelOptionDef OPT_InputFrameCrop =
  { MODOPT_ARG(Rectangle), "InputFrameCrop", &MOC_INPUT, OPTEXP_CORE,
    "Crop input frames, or 0,0,0,0 for no cropping",
    "crop-input", '\0', "x1,y1,x2,y2", "0,0,0,0" };

// Used by: InputFrameSeries
const ModelOptionDef OPT_InputFrameRange =
  { MODOPT_ARG(FrameRange), "InputFrameRange", &MOC_INPUT, OPTEXP_CORE,
    "Input frame range and inter-frame delay or rate. The frame range "
    "can include optional specifications for the first frame (default "
    "value=0), last frame (default=MAX), and/or frame step (default=1). "
    "A fixed framerate can be specified either as an inter-frame interval, "
    "with a suffix one of {s,ms,us,ns}, or as a frame rate, with a suffix of Hz. "
    "A variable framerate can be specified with the name of a file (.fl) "
    "that contains precise frame timing information, with lines of the form:\n"
    "\t<name> <time>\n"
    "where:\n"
    "\t<name> is a string (no spaces) and\n"
    "\t<time> is the elapsed time from stimulus onset (in ms)\n"
    "\tthe <time> in the first line should be 0\n"
    "\tthe <name> in the last line should be END\n"
    "\tthe <time> in the last line should mark the stimulus end time",
    "input-frames", 'M', "[[first[-step]]-[last]]@[delay_or_rate]",
    "0-1-MAX@30Hz" };

// Used by: InputFrameSeries
const ModelOptionDef OPT_InputFramesWrap =
  { MODOPT_FLAG, "InputFramesWrap", &MOC_INPUT, OPTEXP_CORE,
    "Whether to set the frame number back to the start if we reached the end. "
    "This needs the --input-frames to be set ",
    "input-frames-wrap", '\0', "", "false" };

// Used by: InputFrameSeries
const ModelOptionDef OPT_InputPreserveAspect =
  { MODOPT_FLAG, "InputPreserveAspect", &MOC_INPUT, OPTEXP_CORE,
    "Preserve input frame aspect ratio if rescaling to fixed dims",
    "preserve-input-aspect", '\0', "", "false" };



// Used by: InputFrameSeries
const ModelOptionDef OPT_InputEchoDest =
  { MODOPT_ARG_STRING, "InputEchoDest", &MOC_INPUT, OPTEXP_CORE,
    "Specify a destination for carbon-copies of the input frames. "
    "If more than one --in-echo is given, then the copies are sent "
    "to each destination in parallel.",
    "in-echo", '\0', "<raster|display|mpeg|none>", "none" };

// Used by: InputFrameSeries
const ModelOptionDef OPT_InputFrameSource =
  { MODOPT_ARG_STRING, "InputFrameSource", &MOC_INPUT, OPTEXP_CORE,
    "Specify a source of input frames. If this option is given "
    "multiple times, then only the last value will have any effect. "
    "The general expected form is [type]:[spec], where [type] specifies "
    "the type of input stream (such as raster files, a movie file, "
    "etc.), and [spec] provides additional information for the "
    "input stream (such as the name of the input files). In "
    "certain common cases, the [type] may be omitted if it can "
    "be inferred from the filename dot-extension spec, as in the "
    "case where the 'png' raster stream type can be inferred from "
    "the fact that the filename ends in '.png'. Also, for certain "
    "stream types, the [spec] is optional and may be omitted. "
    "There are several general input stream types: (1) raw input "
    "sources, which represent input images being read from some external "
    "source such as files on disk or input from a camera; (2) generated "
    "input sources, which represent images being generated online by "
    "the program, such as test stimuli; and (3) filters, which are "
    "input sources that generate images by transforming images that "
    "come from some other source. Filters can be chained together "
    "arbitrarily. Finally, some input sources may not be supported "
    "on every system, depending on the operating system and available "
    "libraries; unavailable input sources are still listed below "
    "but are specifically marked as being unavailable. Following is "
    "a list of possible stream types, their corresponding filename "
    "dot-extensions, and the format expected of their [spec]:\n"

    "\n\n"
    "RAW INPUT SOURCES\n"
    "        --in=raster:path/to/file#nnn#.[<width>x<height>].[ext]\n"
    "          Reads input from one or more raster files. The leading "
    "'raster:' may be omitted if the file has one of the known raster "
    "extensions: .pnm, .pgm, .ppm, .pbm, .pfm, .png, .jpeg, .jpg, .dpx, "
    ".rgb555, .rgb565, .rgb24, .rgb32, .yuv24, .yuyv, .uyvy, .yuv444, .yuv422, "
    ".yuv411, .yuv420, .yuv410, .yuv444p, .yuv422p, .yuv411p, .yuv420p, "
    ".yuv410p; or, any of the above with an additional .gz or .bz2 "
    "extension, in which case the image file will be transparently "
    "decompressed. If the given filename includes a pair of hashes "
    "('#'), then characters in between the hashes will be interpreted "
    "as a minimum numeric field width, and the frame number will be "
    "formatted as a zero-padded string to that minimum width, and "
    "will be substituted for the entire #nnn# sequence. As special "
    "cases, a single '#' is equivalent to '#6#', and '##' is equivalent "
    "to '#0#'. Thus a filename of foo#.png or foo#6#.png will cause "
    "the stream to read foo000000.png, foo000001.png, etc.; foo##.png "
    "or foo#0#.png will read foo0.png, foo1.png, etc.; and foo#3#.png "
    "will read foo000.png, foo001.png, etc. Note that for the "
    "raw formats that have no header information (e.g., rgb24, yuv420p), "
    "you need to specify the image dimensions somehow; the preferred "
    "method for doing this is to encode the dimensions as '.WWWxHHH' "
    "into a secondary dot-extension just prior to the pixel-format "
    "filename extension (this is the same filename format produced by "
    "--out=rawvideo). If no such .WWWxHHH is found in the filename, then "
    "the dimensions will be taken from the value passed to --yuv-dims, "
    "but this approach is deprecated.\n"

    "        --in=rasterlist:listfilename\n"
    "          Reads a list of raster file names from 'listfilename', "
    "with one file name per line. The corresponding images will be "
    "loaded as input frames, in the order in which they are listed in "
    "the list file. The raster files may be in any of the file formats "
    "accepted by --in=raster. Note that all of the files in the list "
    "must be of the same image type; that is, they must all have the "
    "same pixel format (such as RGB, grayscale, YUV420P, etc.), and "
    "they must all have the same image dimensions.\n"

    "        --in=movie:path/to/movie.[ext]\n"
    "          Reads input from a movie file. The leading 'movie:' may "
    "be omitted if the file has one of the known movie extensions: "
    ".avi, .mpg, .mpeg, .m4v, .mov, .flv, or .dv.\n"

    "        --in=mpeg:path/to/movie.[ext]\n"
    "          Equivalent to --in=movie; present for backward "
    "compatibility from when the only supported movie type was mpeg.\n"

    "        --in=mgz:path/to/file.mgz\n"
    "          Reads input frames from a file in our custom 'mgz' "
    "format, which is essentially a single file containing a "
    "gzip-compressed sequence of images. The leading 'mgz:' may be "
    "omitted if the given filename ends with a '.mgz' extension.\n"

    "        --in=mraw:path/to/file.[<width>x<height>].[ext]?.gz|.bz2?\n"
    "          Reads multiple raw-format input frames from a single "
    "file. Because this is a truly raw file format containing no header "
    "or formatting information, you must provide some auxiliary "
    "information by encoding it into the filename. Specifically, the "
    "file must end with two or three dot-extensions. The first "
    "dot-extension specifies the width and height dimensions of the "
    "video frames. The second specifies the pixel format (so the "
    "leading 'mraw:' may be omitted) as one of: .mrgb555, .mrgb565, "
    ".mrgb24, .mrgb32, .myuv24, .myuyv, .muyvy, .myuv444, .myuv422, .myuv411, "
    ".myuv420, .myuv410, .myuv444p, .myuv422p, .myuv411p, .myuv420p, "
    ".myuv410p. The optional third extension can be .gz or .bz2, in "
    "which case the video file will be transparently decompressed from "
    "gzip or bzip2 compression, respectively. For example, a valid "
    "usage would be \"--in=somemovie.640x480.myuv420p.gz\".\n"

    "        --in=xmlfile:path/to/file.xml\n"
    "          Reads input frames from an xml file which could contain "
    "MetaData. This can be used to train the biasing algorithm or for object "
    "recognition. The xml file can contain includes for other xml files. \n"

    "        --in=stimulus2D:path/to/file.stim2d\n"
    "          Reads a text file created by a matlab script createStim.m"
    " and generates input frames. createStim.m creates a text file which"
    " contains 1d time varying signals and the pixel location in an image"
    " they are assigned. Useful for creating 2d step, impulse etc. "
    " functions for testing systems with both spatial and temporal dynamics. \n"

    "        --in=v4l\n"
#ifndef HAVE_LINUX_VIDEODEV_H
    "          NOTE: THIS OPTION IS NOT SUPPORTED ON THIS SYSTEM\n"
#endif
    "          Reads input frames from a Video4Linux (V4L) framegrabber "
    "device. There is no [spec] option here; the framegrabber can be "
    "configured with ordinary command-line options which become "
    "available after --in=v4l is selected.\n"

    "        --in=v4l2\n"
#ifndef HAVE_LINUX_VIDEODEV2_H
    "          NOTE: THIS OPTION IS NOT SUPPORTED ON THIS SYSTEM\n"
#endif
    "          Reads input frames from a Video4Linux version 2 (V4L2) "
    "framegrabber device. There is no [spec] option here; the "
    "framegrabber can be configured with ordinary command-line "
    "options which become available after --in=v4l2 is selected.\n"

    "        --in=ieee1394\n"
#ifndef HAVE_IEEE1394
    "          NOTE: THIS OPTION IS NOT SUPPORTED ON THIS SYSTEM\n"
#endif
    "          Reads input frames from a Firewire camera using the "
    "libdc1394 library version 0.9 or 1.x. There is no [spec] option "
    "here; the camera can be configured with ordinary command-line "
    "options which become available after --in=ieee1394 is selected.\n"

    "        --in=dc1394v2\n"
#ifndef HAVE_DC1394V2
    "          NOTE: THIS OPTION IS NOT SUPPORTED ON THIS SYSTEM\n"
#endif
    "          Reads input frames from a Firewire camera using the "
    "libdc1394 library version 2.x. There is no [spec] option "
    "here; the camera can be configured with ordinary command-line "
    "options which become available after --in=dc1394v2 is selected.\n"

    "        --in=qtgrab\n"
#ifndef HAVE_QUICKTIME_QUICKTIME_H
    "          NOTE: THIS OPTION IS NOT SUPPORTED ON THIS SYSTEM\n"
#endif
    "          Reads input frames from a QuickTime camera using Apple's "
    "QuickTime library. There is no [spec] option here; the camera "
    "can be configured with ordinary command-line options which "
    "become available after --in=qtgrab is selected.\n"

    "        --in=rtsp://URL\n"
#ifndef INVT_HAVE_LIVEMEDIA
    "          NOTE: THIS OPTION IS NOT SUPPORTED ON THIS SYSTEM\n"
#endif
    "          Reads input frames from an rtsp source using livemedia libs. "
    "There is no [spec] option here; the camera "
    "can be configured with ordinary command-line options which "
    "become available after --in=rtsp is selected.\n"


    "\n\n"
    "GENERATED INPUT SOURCES\n"
    "        --in=random[:WxH]\n"
    "          Generates color input frames with R/G/B values drawn "
    "from a uniform random distribution on [0,255]. The optional WxH "
    "spec can be used to specify the dimensions of the generated "
    "images. The default image size is 320x240.\n"

    "        --in=dots[:WxH],MotionType\n"
    "          Generates artificial greyscale (but in PixRGB format) "
    "moving dots. [:WxH] indicate optional width and height of the stimulus "
    "The default image size is 320x240. "  
    "The motion type supported for now is: "
    "Foe[flags] for Focus of Expansion "
    "(flags: bit 2: haveMotion , bit1: have temporal gradient, bit 0: have spatial gradient)"
    "Planar for planar motion. "
    "We are working on rotation, random, and single dot motion. "
    "In addition user will be able to specify some of the critical parameters "
    "for each motion type. \n"

    "        --in=bars[:WxH],MotionType\n"
    "          Generates artificial greyscale (but in PixRGB format) "
    "bars. [:WxH] indicate optional width and height of the stimulus "
    "The default image size is 320x240. "  
    "The motion type supported for now is: "
    "Normal for a single vertical bar moving left to right, "
    "ApertureProblem for a 45deg bar moving left to right . "
    "In the future the user will be able to specify some parameters."
    "for each motion type. \n"

    "        --in=shiftedImage:path/image:MotionType\n"
    "          Generates moving color image stimuli "
    "The motion type supported for now is: "
    "foe for Focus of Expansion, "
    "planar for planar motion. "
    "We are working on rotation motion"
    "In addition user will be able to specify some of the critical parameters "
    "for each motion type. \n"

    "        --in=colorbars[:frameperiod]\n"
    "          Generates a series of 'color bars' test images which "
    "include bars of various colors, a set of grayscale bars, a frame "
    "counter, and a clock. The optional [frameperiod] can be used to "
    "specify the desired real-time inter-frame interval in any time "
    "units or Hz. Examples: --in=colorbars:100ms will generate one "
    "test frame every 100ms; --in=colorbars:4Hz will generate one "
    "test frame every 250ms (4Hz).\n"

    "        --in=life[:WxH]\n"
    "          Generates black+white input frames according to "
    "Conway's \"Game Of Life\", a cellular automata system. The "
    "optional WxH spec can be used to specify the dimensions of "
    "the generated images. The default image size is 320x240.\n"

    "        --in=World2D[:WxH]\n"
    "         Generates a simple world composed of 2D objects like "
    "square, lines, circles, and objects which are a combination of "
    "simpler objects. The optional WxH spec can be used to specify "
    "the dimensions of the generated images. The default image size "
    "is 256x256.\n "


    "\n\n"
    "INPUT FILTERS\n"
    "        --in=bob:<childtype:childspec>\n"
    "          This input type is a filter, whose [spec] is a nested "
    "type:spec specification. The filter applies bob deinterlacing to "
    "frames coming from the child input source. The bob deinterlacer "
    "forms deinterlaced images alternately from the 'top' and 'bottom' "
    "half-fields of the original image (corresponding to the even and "
    "odd-numbered scanlines). Each deinterlaced image is formed by "
    "linearly interpolating to fill in the missing scan lines. This "
    "type of deinterlacing generates twice as many frames, at twice the "
    "frame rate, as the original input source. This is called 'bob' "
    "deinterlacing because the resulting image sequence may appear "
    "to slightly bob up and down as it alternates between interpolated "
    "top fields and bottom fields. Examples: --in=bob:foo.mpg will "
    "deinterlace a movie; --in=bob:v4l will deinterlace the input "
    "coming from a Video4Linux framegrabber.\n"

    "        --in=bhf:<childtype:childspec>\n"
    "          This input type is a filter, whose [spec] is a nested "
    "type:spec specification. The filter applies linear interpolation "
    "deinterlacing to the bottom half-field of the original images.\n"

    "        --in=thf:<childtype:childspec>\n"
    "          This input type is a filter, whose [spec] is a nested "
    "type:spec specification. The filter applies linear interpolation "
    "deinterlacing to the top half-field of the original images.\n"

    "        --in=buf:<childtype:childspec>\n"
    "          This input type is a filter, whose [spec] is a nested "
    "type:spec specification. This input type buffers the input frames "
    "of its child input source into a frame queue using a background "
    "thread. Then, while callers in the main thread read frames out "
    "from the buffered input stream, the background thread continually "
    "tries to keep the queue filled. Command-line parameters are "
    "available to control the buffer size (--input-buffer-size), and "
    "also to control the desired behavior if the queue underflows "
    "(--underflow-strategy), i.e., if somebody in the main thread "
    "tries to read a frame while the queue is empty.\n"
    ,
    "in", '\0', "<[type]:[spec]>", "" };

// Used by: InputFrameSeries, OutputFrameSeries
const ModelOptionDef OPT_InputOutputComboSpec =
  { MODOPT_ARG_STRING, "InputOutputComboSpec", &MOC_INPUT, OPTEXP_CORE,
    "Specify both input and output with a single specification. Thus "
    "--io=type:foo is equivalent to --in=type:foo --out=type:foo. "
    "NOTE that this will only work if 'type' is valid for both input "
    "and output; currently that would include 'raster' and 'mpeg', but "
    "not 'display' (valid for output only) or 'random' (valid for "
    "input only).",
    // PROGRAMMER NOTE: We don't actually care what the /current/
    // value of this option is -- instead, we just catch each call
    // to this option and create a FrameOstream object as
    // appropriate.
    "io", '\0', "<intersection of valid values for --in and --out>", "" };

// Used by: OutputFrameSeries
const ModelOptionDef OPT_OutputFrameRange =
  { MODOPT_ARG(FrameRange), "OutputFrameRange", &MOC_OUTPUT, OPTEXP_CORE,
    "Output frame range and inter-frame delay or rate. See "
    "--input-frames for details on the expected format. "
    "Variable rates can be specified as with --input-frames, BUT NOTE "
    "that some output formats (e.g., movie encoders) may not preserve "
    "the variable framerate information.",
    "output-frames", 'R', "[[first[-step]]-[last]]@[delay_or_rate]",
    "0-1-MAX@30Hz" };

// Used by: OutputFrameSeries
const ModelOptionDef OPT_OutputFrameSink =
  { MODOPT_ARG_STRING, "OutputFrameSink", &MOC_OUTPUT, OPTEXP_CORE,
    "Specify a destination for output frames. This option may be "
    "called multiple times to build up an internal list of multiple "
    "parallel output destinations. If --out=none is given, then any "
    "previous --out selections are canceled. The general output "
    "paradigm is that the program may generate multiple streams of "
    "output, and each of those streams may be feed to one or more "
    "output destinations selected by --out options. For example, "
    "the program might generate output streams named 'MapA' and 'MapB', "
    "and if --out=pnm is selected, then ultimately the generated files "
    "will be named MapA000000.pnm, MapA000001.pnm, etc. and "
    "MapB000000.pnm, MapB000001.pnm, etc.\n"

    "\n\n"
    "The general expected form of the argument to a --out option is "
    "[type]:[spec], where [type] specifies the type of output stream "
    "(such as raster files, movie files, onscreen windows, etc.), and "
    "[spec] provides additional information for the output stream (such "
    "as a prefix for the output file names). In certain common cases, "
    "the explicit [type] may be omitted if it can be inferred from the "
    "filename dot-extension spec. Also, for certain stream types, the "
    "[spec] is optional and may be omitted. There are several general "
    "output stream types: (1) disk output destinations, in which "
    "the output is saved in some form to one or more files on disk; "
    "(2) GUI output destinations, in which the output images are "
    "displayed in some type of GUI window(s); (3) text output "
    "destinations, in which some information about the output images "
    "is printed to the console; and (4) filters, which are output "
    "destinations that merely transform the output on its way to a "
    "final destination (for example, splitting an RGB image into its "
    "individual color components). Finally, some output destinations "
    "may not be supported on every system, depending on the operating "
    "system and available libraries; unavailable output destinations "
    "are still included in the list below but are specifically marked "
    "as being unavailable. Following is a list of possible output "
    "stream types and the format expected of their [spec] options. "
    "In the descriptions below, the phrase KEY refers to the output "
    "stream name produced by the program (such as 'MapA' or 'MapB' "
    "in the previous example), and which is NOT controllable from the "
    "command-line.\n"

    "\n\n"
    "DISK OUTPUT DESTINATIONS\n"
    "        --out=raster:fileprefix\n"
    "        --out=raster:path/to/fileprefix#nnn#\n"
    "          Images will be saved to individual raster files on "
    "disk. The format of the image files will be deterined by the "
    "--output-format option (default=PNM). The filenames will consist "
    "of several parts, including the fileprefix given in the [spec], "
    "the KEY provided by the program, the frame number, and the filename "
    "extension appropriate to the image file format, in this format: "
    "[fileprefix]-[KEY][framenumber][.ext]. The hyphen between "
    "[fileprefix] and [KEY] is omitted if the fileprefix ends with a '/' "
    "(that is, it just specifies a directory prefix). By default, the "
    "frame number portion of the filename will be a 6-digit zero padded "
    "number, but the length of zero padding can also be specified in the "
    "fileprefix. If the fileprefix includes a pair of hashes ('#'), then "
    "the characters between the hashes will be interpreted as a minimum "
    "field width, and the frame number will be formatted as a string with "
    "zero-padding to fill that minimum width. As special cases, a single "
    "'#' is equivalent to '#6#', and '##' is equivalent to '#0#'. Note "
    "that the location of this hash sequence within the fileprefix is "
    "irrelevant; the frame number will always appear at the end of the "
    "filename, just before the filename extension. For example, "
    "an unadorned --out=raster will produce KEY000000.pnm, KEY000001.pnm, "
    "etc.; --out=raster:foo/bar will produce foo/bar-KEY000000.pnm, "
    "foo/bar-KEY000001.pnm, etc; --out=raster:foo/#2# will produce "
    "foo/KEY00.pnm, foo/KEY01.pnm, etc; --out=raster:## will produce "
    "KEY0.pnm, KEY1.pnm. If the fileprefix includes a directory "
    "component, then that directory must already exist.\n"

    "        --out=pnm[:fileprefix]\n"
    "        --out=pgm[:fileprefix]\n"
    "        --out=ppm[:fileprefix]\n"
    "        --out=pbm[:fileprefix]\n"
    "          Like --out=raster with the image file format set to PNM "
    "(Portable aNyMap). Despite the different option names here, the "
    "output files generated will always have a '.pnm' extension.\n"

    "        --out=pfm[:fileprefix]\n"
    "          Like --out=raster with the image file format set to PFM, "
    "which is our custom format to hold 32-bit floating-point values in a "
    "plain file format similar to PNM. The filename extension will be "
    "'.pfm'. These files can be read back in by various programs in the "
    "toolkit, and can be read into matlab with pfmread.m. This file "
    "format encodes only grayscale images, so color images will be "
    "converted to their luminance prior to saving in PFM format.\n"

    "        --out=png[:fileprefix]\n"
    "          Like --out=raster with the image file format set to PNG "
    "(Portable Network Graphics). The filename extension will be '.png'.\n"

    "        --out=jpg/jpeg[:fileprefix]\n"
    "          Like --out=raster with the image file format set to jpg. "
    "Default quality is 75%.The filename extension will be '.jpg' or '.jpeg'.\n"

    "        --out=rawvideo[:fileprefix]\n"
    "          Like --out=raster with the image file format set to "
    "RAWVIDEO. This is a raw file format in which the file contents "
    "include only the raw image pixels; the file itself does not "
    "indicate the image dimensions or the pixel type. Both of these "
    "pieces information are, however, encoded into the filename "
    "extension, which will be of the form '.WWWxHHH.PIXFMT', where "
    "WWW and HHH are the image width and height, and PIXFMT is one of "
    "'rgb555', 'rgb565', 'rgb24', 'rgb32', 'yuv24', 'yuyv', 'uyvy', 'yuv444', "
    "'yuv422', 'yuv411', 'yuv420', 'yuv410', 'yuv444p, 'yuv422p', "
    "'yuv411p, 'yuv420p', or 'yuv410p', as appropriate. These files "
    "can be subsequently read back in with the --in option.\n"

    "        --out=txt[:fileprefix]\n"
    "          Like --out=raster with the image file format set to TXT. "
    "The filename extension will be '.txt'. This is a plain-text file "
    "format in which image pixels are simply written as space-separated "
    "strings, with one row of pixels per line of text. Color pixels are "
    "written as individual R, G, B values. Thus, a line of pixels from a "
    "color image would be encoded as R1 G1 B1 R2 G2 B2 R3 G3 B3, etc. "
    "Files generated in this format can be read by matlab's load() "
    "function.\n"

    "        --out=ccode[:fileprefix]\n"
    "          Like --out=raster with the image file format set to "
    "CCODE. The filename extension will be '.C'. In this format, images "
    "are encoded as C++ source code suitable for pasting back into "
    "a source file in this toolkit. The generated source will include "
    "one int variable for the image width, one for the image height, and "
    "one C-style array containing the image pixels in row-major format.\n"

    "        --out=bkg-rawvideo:pfx1,pfx2,pfx3,...\n"
    "          Similar to --out=rawvideo, but in this case the actual file "
    "writing happens in a background thread. This is useful on multi-core "
    "machines in cases where the main thread is fully occupied with some "
    "other task or has other real-time constraints. The generated "
    "filenames are of the form [pfx][KEY][NNNNNN][.ext], where [pfx] "
    "cycles through the list of prefixes given in the [spec]. This allows "
    "output image files to be striped across multiple directories and "
    "even multiple physical disks, which can help to maximize usage of "
    "available disk bandwidth. For example, assuming 640x480 YUV420P "
    "files are being saved with --out=bkg-rawvideo:/disk1/a/,/disk2/b/, "
    "then the generated files will be "
    "/disk1/a/KEY000000.640x480.yuv420p, "
    "/disk2/b/KEY000001.640x480.yuv420p, "
    "/disk1/a/KEY000002.640x480.yuv420p, "
    "/disk2/b/KEY000003.640x480.yuv420p, etc.\n"

    "        --out=movie[:fileprefix]\n"
#ifdef INVT_HAVE_AVCODEC
    "          NOTE: THIS OPTION IS NOT SUPPORTED ON THIS SYSTEM\n"
#endif
    "          One movie file will be generated for each KEY, named as "
    "[fileprefix][KEY][.ext]. The filename extension will be appropriate "
    "to the movie codec selected with --output-codec. The video encoding "
    "is handled by the ffmpeg library.\n"

    "        --out=mpeg[:fileprefix]\n"
#ifdef INVT_HAVE_AVCODEC
    "          NOTE: THIS OPTION IS NOT SUPPORTED ON THIS SYSTEM\n"
#endif
    "          An alias for --out=movie.\n"

    "        --out=ucbmpeg[:fileprefix]\n"
#ifndef MPEGENCODE_PROG
    "          NOTE: THIS OPTION IS NOT SUPPORTED ON THIS SYSTEM\n"
#endif
    "          Like --movie, but the video encoding is handled by "
#ifdef MPEGENCODE_PROG
    MPEGENCODE_PROG
#else
    "ppmtompeg or mpeg_encode (but neither of those was found on "
    "this system)"
#endif
    ".\n"

    "        --out=mgz[:fileprefix]\n"
    "          Like --out=movie, but the output files will be saved "
    "in a custom 'mgz' format. The format is essentially a "
    "gzip-compressed sequence of raw images, all in a single file.\n"

    "        --out=mraw[:fileprefix]\n"
    "          Generates multi-frame raw files containing concatenated "
    "rawvideo frames. The resulting files are completely identical to a "
    "concatenation of the individual files generated with --out=rawvideo; "
    "however, if you are going to be saving hundreds or thousands of "
    "frames, then it may be more efficient (in terms of operating system "
    "and disk overhead) to save them into a single file with --out=mraw "
    "than to save into individual files with --out=rawvideo. Like "
    "--out=rawvideo, this multi-frame format is a truly raw format with "
    "only pixel data in the file itself; therefore, critical metadata "
    "(image dimensions and pixel format) are encoded into the filename. "
    "The filename will have an extension of the form '.WWWxHHH.mPIXFMT' "
    "(similar to the --out=rawvideo format, but note the extra 'm' "
    "denoting \"multi-frame\"); WWW and HHH specify the image dimensions "
    "and PIXFMT specifies the pixel format, as described for "
    "--out=rawvideo. The multi-frame files generated by --out=mraw can be "
    "subsequently read back in with a --in=mraw option. Note that "
    "although --in=mraw can read gzip- or bzip2-compressed files, "
    "--out=mraw cannot generate such files directly; rather, you can "
    "generate an an uncompressed file with --out=mraw, then compress it "
    "separately with gzip or bzip2, and then read the compressed file "
    "back in with --in=mraw.\n"

    "\n\n"
    "GUI OUTPUT DESTINATIONS\n"
    "        --out=display\n"
    "          Image files are sent to X11 windows on the display "
    "specified by the $DISPLAY environment variable, or :0.0 if $DISPLAY "
    "is unset. One window is generated for each KEY, and the KEY name "
    "is used as a window title.\n"

    "        --out=sdl\n"
#ifndef HAVE_SDL_SDL_H
    "          NOTE: THIS OPTION IS NOT SUPPORTED ON THIS SYSTEM\n"
#endif
    "          Image files are sent to an SDL window. This option "
    "can only be used when there is exactly one KEY. The parameters "
    "(size, etc.) of the SDL window can be controlled with other "
    "command-line parameters.\n"

    "        --out=qt\n"
#ifndef INVT_HAVE_QT3
    "          NOTE: THIS OPTION IS NOT SUPPORTED ON THIS SYSTEM\n"
#endif
    "          Image files are sent to a Qt window. This window holds "
    "all KEY streams using a tabbed interface: individual KEY streams "
    "can be selected for viewing by clicking on their names in the "
    "listbox on the left side of the window. The image display pane, "
    "on the right side of the window, shows the image itself (at bottom), "
    "along with auxiliary information about the image (at top), including "
    "its KEY name, a more detailed description of the image (if "
    "available), its dimensions, and its image/pixel format. The image "
    "display pane also includes a spinbox to zoom the image in/out, as "
    "well as a button that can be used to save any displayed image as a "
    "raster file. The main window also includes a pause button that can "
    "be used to stop a simulation so that the various images can "
    "reviewed.\n"

    "\n\n"
    "TEXT OUTPUT DESTINATIONS\n"
    "        --out=hash[:filename]\n"
    "          Write a simple checksum hash of each image to the given "
    "filename. If the filename is omitted, or is given as '' (the empty "
    "string) or '-' or 'stdout' or 'STDOUT', then the checksums will be "
    "written to stdout; likewise, if the filename is 'stderr' or 'STDERR', "
    "then the checksums will be written to stderr. In all other cases, the "
    "checksums will be written to the named file.\n"

    "        --out=info[:filename]\n"
    "          At program shutdown time, write a summary of the number, "
    "size, and type of frames written for each KEY. The summary is written "
    "to stdout, stderr, or a named file, based on the given filename, in "
    "the same way as described for --out=hash.\n"

    "        --out=stats[:filename]\n"
    "          Write a simple stats summary of output image, including "
    "the min/mean/stdev/max values for grayscale images, and the "
    "min/mean/stdev/max values for each R,G,B channel in color images. "
    "Also, at program shutdown time, write an overall stats summary for "
    "each KEY of all the images written for that KEY. The stats are "
    "written to stdout, stderr, or a named file, based on the given "
    "filename, in the same ways as described for --out=hash.\n"

    "\n\n"
    "OUTPUT FILTERS\n"

    "        --out=null\n"
    "          A special output destination that simply discards any "
    "frames it receives. Note that this is different from --out=none: "
    "whereas --out=none cancels any previous --out options, --out=null "
    "is just another output destination that happens to do nothing.\n"

    "        --out=splitrgb:destination\n"
    "          An output filter that splits color images into their "
    "individual R,G,B components, and then passes those components on to "
    "the destination specified by the [spec]. The [spec] should be "
    "another output destination, just as would normally be passed to "
    "--out. When the components are passed to the next destination, their "
    "KEY names are altered to include '-r', '-g', or '-b' suffixes, as "
    "appropriate. For example, --out=splitrgb:pnm would split color "
    "images into their components and then save those components to pnm "
    "files named KEY-r000000.pnm, KEY-g000000.pnm KEY-b000000.pnm etc. "
    "Grayscale images passing through splitrgb will also be \"split\" "
    "into components, but those components will just happen to be "
    "identical.\n"

    "        --out=luminance:destination\n"
    "          An output filter that converts color images into their "
    "luminance images, and then passes those luminance images on to the "
    "destination specified by the [spec]. As with --out=splitrgb, the "
    "[spec] is just another output destination, as would normally be "
    "passed to --out. Color images passing through --out=luminance will "
    "have their KEY names modified with a '-lum' suffix; grayscale images "
    "will pass through unaltered, and their KEY names will remain "
    "unaltered as well.\n"

    "        --out=colorize:destination\n"
    "          An output filter that colorizes grayscale images with a "
    "colormap. For now, the colormap is like matlab's 'jet' colormap and "
    "cannot be changed. Grayscale images passing through --out=colorize "
    "will have their KEY names modified with a '-colorized' suffix; "
    "images that are already in color will pass through unaltered, and "
    "their KEY names will remain unaltered as well.\n"

    "        --out=coerce-grey:destination\n"
    "        --out=coerce-rgb555:destination\n"
    "        --out=coerce-rgb565:destination\n"
    "        --out=coerce-rgb24:destination\n"
    "        --out=coerce-rgb32:destination\n"
    "        --out=coerce-yuv24:destination\n"
    "        --out=coerce-yuyv:destination\n"
    "        --out=coerce-uyvy:destination\n"
    "        --out=coerce-yuv444:destination\n"
    "        --out=coerce-yuv422:destination\n"
    "        --out=coerce-yuv411:destination\n"
    "        --out=coerce-yuv444p:destination\n"
    "        --out=coerce-yuv422p:destination\n"
    "        --out=coerce-yuv411p:destination\n"
    "        --out=coerce-yuv420p:destination\n"
    "        --out=coerce-yuv410p:destination\n"
    "          Output filters that coerce images into a given rawvideo "
    "format, if possible. Some coercions are atomic (implemented by a "
    "single conversion function), while the remaining coercions are "
    "implemented by the optimal sequence of atomic conversions. The "
    "optimal sequence for a given conversion is the one that meets "
    "the following criteria, in order of preference: (1) minimizes "
    "the number of lossy colorspace conversions, like rgb->yuv, "
    "yuv->rgb, rgb->grey, or yuv->grey; (2) minimizes the number of "
    "lossy spatial resolution conversions, like yuv444->yuv422 or "
    "yuv444->yuv410p; (3) minimizes the number of lossy bit depth "
    "conversions, like rgb24->rgb565 or rgb->555; (4) minimizes the "
    "total number of atomic conversion steps.\n"

    ,

    // PROGRAMMER NOTE: We don't actually care what the /current/
    // value of this option is -- instead, we just catch each call
    // to this option and create a FrameOstream object as
    // appropriate.
    "out", '\0', "<[type]:[spec]>", "" };

// Used by: InputFrameSeries, OutputFrameSeries
const ModelOptionDef OPT_WaitForUser =
  { MODOPT_FLAG, "WaitForUser", &MOC_OUTPUT, OPTEXP_CORE,
    "Whether to wait for a keypress after each input/output frame",
    "wait", '\0', "<true|false>", "false" };

// Used by: InputFrameSeries
const ModelOptionDef OPT_KeepGoing =
  { MODOPT_FLAG, "KeepGoing", &MOC_OUTPUT, OPTEXP_CORE,
    "Keep going even after input is exhausted",
    "keep-going", '+', "", "false" };

// Used by: OutputFrameSeries
const ModelOptionDef OPT_OutputReplicate =
  { MODOPT_ARG(uint), "OutputReplicate", &MOC_OUTPUT, OPTEXP_CORE,
    "How many times to replicate each output frame. This is useful "
    "if you want to generate a '15fps' mpeg1 movie; mpeg1 doesn't "
    "support 15fps, but it does support 30fps so if you use 30fps "
    "with output replication of 2, then you will effectively end "
    "up with a 15fps movie.",
    "output-replicate", '\0', "uint", "1" };

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // MEDIA_MEDIAOPTS_C_DEFINED
