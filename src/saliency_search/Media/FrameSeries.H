/*!@file Media/FrameSeries.H a series of frames */

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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/FrameSeries.H $
// $Id: FrameSeries.H 15330 2012-07-18 17:54:39Z dberg $
//

#ifndef FRAMESERIES_H_DEFINED
#define FRAMESERIES_H_DEFINED

#include "Component/ModelParam.H"
#include "Image/Dims.H"
#include "Image/Rectangle.H"
#include "Media/FrameRange.H"
#include "Media/FrameState.H"
#include "Transport/FrameIstream.H"
#include "Transport/FrameOstream.H"
#include "Util/Types.H"

class SimTime;
template <class T> class Image;
template <class T> class PixRGB;

// ######################################################################
//! A FrameSeries with additional facilities for reading input frames
/*! General notes for InputFrameSeries and OutputFrameSeries: these
    classes keep track of a range of frames ([first..last]) and an
    interframe delay in seconds (see FrameRange.H). The key behavior
    is the update() function, which takes the current simulation time
    in seconds and the simulation step in s, and returns a status code
    indicating whether the frame series (a) was unchanged, (b) will
    require that the next frame be loaded just before the simulation
    time is advanced by the current simulation step, or (c) is
    finished.

    InputFrameSeries implements a number of helper functions to
    facilitate reading frames in a standardized manner, with possible
    on-the-fly rescaling and visualization.

    NOTE: if we're in test mode (because --test-mode was given on the
    command line), we will turn off all displays even though they may
    be on here (actually what happens is that ImageDisplayStream
    refuses to do anything if in --test-mode).

    NOTE: when start() is called, we will set the "FOAradius",
    "FoveaRadius", "InputFrameDims", "SCinitialEyePosition" and
    "SCinitialHeadPosition" model parameters from input image dims, if
    someone wants them and they have not been set.


    NOTES on controlling InputFrameSeries and OutputFrameSeries from
    the command-line:

    Examples:
    \verbatim
      --in=raster:foo.ppm
      --in=raster:#.png   [to get foo000000.ppm, foo000001.ppm, etc.]
      --in=mpeg:/path/to/file.mpg
      --in=random:512x512
      --in=colorbars

      --out=display
      --out=qt
      --out=mpeg
      --out=png:/path/to/basestem
      --out=pnm:/path/to/basestem
      --out=none
      --out=hash

      --io=raster:path/to/framestem
      --io=mpeg:file.mpg
    \endverbatim

    Any time you use an InputFrameSeries or OutputFrameSeries, you can
    automatically use input/output in any of the formats that we
    support. That means:

    - your InputFrameSeries can take input from a series of raster
      files (the previous default behavior), or from any movie file
      that is readable by avcodec, or from an in-memory random number
      generator. Other input formats are possible, please write your
      own subclass of FrameIstream and add command-line support for it
      to InputFrameSeries.

    - your OutputFrameSeries can generate output going to series of
      raster files (the previous default behavior), or to mpeg files,
      or to onscreen windows, or to nowhere at all. Again, other
      output formats are possible, please write your own subclass of
      FrameOstream and add command-line support for it to
      OutputFrameSeries.

    The simplest behavior (which matches the historical behavior of
    the ezvision executable in which input and output both deal with
    raster file series using the same filename stem) can be gotten
    like this:

      ./bin/ezvision --bunch --of --options --io=raster:myinputfilestem

    To get to the more advanced features, you can use --in and --out
    instead of --io (note that --io=foo is just an alias for --in=foo
    --out=foo). For any of the options, you want to pass something of
    the form "type:spec", where 'type' is one of <raster|mpeg|random>
    for input options, or <raster|mpeg|display|none> for output
    options. The 'spec' part is type-dependent; for raster or mpeg the
    spec gives the filename or filename stem; for random, the spec is
    the desired image size as WWWxHHH (e.g. 512x512); for display and
    none, the spec is not required.

    As a shorthand convenience, you can eliminate either the type or
    spec if it can be inferred from the other part. E.g., if your spec
    ends in a raster file extension (.ppm, .pnm, .png, .pgm) or mpeg
    file extension (.mpg, .mpeg, .m4v), you can skip the 'type' and just
    do --in=file.ppm or --out=file.mpg, which will be interpreted as
    --in=raster:file.ppm or --out=mpeg:file.mpg. Likewise, for the types
    that don't require any extra spec info, you can just do
    --out=display or --out=none, which are shorthand for --out=display:
    and --out=none:.

    Note that you can only have one input source, so if you have
    multiple --in or --io options on your command-line, the last such
    option will override all previous ones.

    On the other hand, you can reasonably have multiple output
    destinations, so if there are multiple --out or --io options on
    the command-line, the output will be sent to all destinations in
    parallel. So, if you want to save your results to files but also
    watch them onscreen as the computation progresses, you could do
    --out=mpeg --out=display. As a special case, if you do --out=none,
    it will cancel any previous --out options (though later --out
    options will still take effect).

    Finally, you can use the --in-echo option to specify one or more
    destinations to which copies of the input should be sent. The
    --in-echo option accepts the same types of values as does --out,
    so e.g. if you want to see your input stream in an onscreen
    window, you can do --in=mpeg:myfile.mpg --in-echo=display.

    A couple other miscellaneous things:

    - the [no]display-input frames and [no]display-output-frames options
      have been superseded by the new, more flexible behavior -- if you
      want to mimic --nodisplay-input-frames or
      --nodisplay-output-frames, just do nothing since the default is
      for the frames to not be displayed onscreen. If you do want
      display, you can use --in-copy=display and/or --out=display.

    - there is a new command-line flag, --[no]wait, that controls the
      result of InputFrameSeries::shouldWait() and
      OutputFrameSeries::shouldWait(), which is used to determine
      whether we do a Raster::waitForKey() in the main loop after
      processing a new input or output frame; the default is --nowait,
      so if you want the pause, then you should do an explicit --wait
      on the command-line
*/
class InputFrameSeries : public FrameIstream
{
public:
  //! Constructor
  InputFrameSeries(OptionManager& mgr,
                   const std::string& descrName = "Frame Series",
                   const std::string& tagName = "FrameSeries");

  //! Destructor
  virtual ~InputFrameSeries();

  //! resets this FrameSeries to its state after construction
  virtual void reset1();

  //! Override the base version so that we can trap --in and --in-echo options
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

  //! Update internal state and return corresponding FrameState
  /*! @param stime current time in seconds
    @param sdelay time step in seconds */
  FrameState update(const SimTime& stime);

  //! Update by moving to the next frame and not worrying about time counts
  FrameState updateNext();

  //! Have we displayed frames and need to wait for a user keypress?
  /*! This is reset to false on each call to update(), and becomes
      true if (1) any frames were written AND (2) itsWaitForUser is
      true (which the use can control with --wait/--nowait). You can
      check it in order to decide whether you should pause and wait
      for the user after each series of displays at a given time
      step). */
  bool shouldWait() const;

  //! Get specifications of the image frames
  /*! It is okay to call this before the model is started().  If input
      resizing is being done, the returned dimensions will be the
      resized dims.  All in all, this will return the size of whatever
      you will get when calling readFrame(), readRGB(), etc. */
  virtual GenericFrameSpec peekFrameSpec();

  //! Optional call to efficiently prepare for frame streaming
  virtual void startStream();

  //! Read a GenericFrame
  virtual GenericFrame readFrame();

  virtual void setFrameDims(Dims d);
  virtual Dims getFrameDims();

  //! Set the frame source; same as doing --in=source on the command line
  void setFrameSource(const std::string& source);

  //! Get the current frame number
  int frame() const;

  //! Get a reference to our frame source
  nub::ref<FrameIstream> getFrameSource() const;

  //! Get a copy of the framerange
  FrameRange getFrameRange() const ;

  //! Force the frame number to a new value
  bool setFrameNumber(int n);

  //! Get the time at which the next frame will be ready
  const SimTime& getNextFrameTime();

private:
  //! post-commandline-parsing initialization
  /*! Sets the initial frame number. */
  virtual void start1();

  //! post-commandline-parsing initialization (see ModelComponent.H)
  /*! Will peek the first frame (without fully loading it) and set
      "FOAradius", "FoveaRadius", "InputFrameDims"
      "SCinitialEyePosition" and "SCinitialHeadPosition" model
      parameters from input image dims if someone wants them and they
      have not been set */
  virtual void start2();

  //! text log file name
  OModelParam<std::string> itsLogFile;

  //! if in test mode, then shouldWait() is always false
  OModelParam<bool> itsTestMode;

  //! range of frames and inter-frame delay in SECONDS
  OModelParam<FrameRange> itsFrameRange;

  //! Wether to set the frame back to the start if we reached the end
  OModelParam<bool> itsFrameWrap;

  //! Rectangle to which frames should be cropped
  OModelParam<Rectangle> itsCropRect;

  //! dims to which frames should be resized on read/write, or 0x0 for none
  OModelParam<Dims> itsDims;

  //! if true, aspect ratio of original image will be preserved when resizing
  OModelParam<bool> itsPreserveAspect;

  //! If true, all frame numbers are 000000, forces no numbering
  OModelParam<bool> itsZeroNumberFrames;

  //! catch --in command-line options
  OModelParam<std::string> itsFrameSource;

  //! catch --io command-line options
  OModelParam<std::string> itsInOut;

  //! catch --in-echo command-line options
  OModelParam<std::string> itsInputEcho;

  //! whether to wait for user's keypress between frames
  OModelParam<bool> itsWaitForUser;

  //! whether to keep going even after input is exhausted
  OModelParam<bool> itsKeepGoing;

  struct Impl;
  friend struct Impl;
  Impl* const rep;
};

// ######################################################################
//! A FrameSeries with additional facilities for writing output frames
/*! OutputFrameSeries implements a number of helper functions to
    facilitate writing frames in a standardized manner, with possible
    on-the-fly rescaling and visualization.

    SEE ADDITIONAL DOCUMENTATION in InputFrameSeries.
*/
class OutputFrameSeries : public FrameOstream
{
public:
  //! Constructor
  OutputFrameSeries(OptionManager& mgr,
                    const std::string& descrName = "Frame Series",
                    const std::string& tagName = "FrameSeries");

  //! Destructor
  virtual ~OutputFrameSeries();

  //! resets this FrameSeries to its state after construction
  virtual void reset1();

  //! Override the base version so that we can trap --out options
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

  //! Update internal state and return corresponding FrameState
  /*! @param stime current time in seconds
      @param sdelay time step in seconds
      @param new_event whether a new application-defined event has occurred
  */
  FrameState update(const SimTime& stime, bool new_event);

  //! Update by moving to the next frame and not worrying about time counts
  FrameState updateNext();

  //! Have we displayed frames and need to wait for a user keypress?
  /*! This is reset to false on each call to update(), and becomes
      true if (1) any frames were written AND (2) itsWaitForUser is
      true (which the use can control with --wait/--nowait). You can
      check it in order to decide whether you should pause and wait
      for the user after each series of displays at a given time
      step). */
  bool shouldWait() const;

  //! Write a frame to all frame destinations
  virtual void writeFrame(const GenericFrame& frame,
                          const std::string& shortname,
                          const FrameInfo& auxinfo = FrameOstream::defaultInfo);

  //! Check if we have no output destinations (e.g., user gave --out=none)
  /*! Clients can test isVoid() before they generate potentially
      expensive output images. This is just a performance optimization
      -- even if isVoid() is true, it's still safe to call writeRGB(),
      writeGray(), etc., but those calls will do nothing, and so any
      time spent computing the image will have been wasted. */
  virtual bool isVoid() const;

  //! Check if we have become void after starting out non-void
  /*! This is useful to test in main loops to know whether some
      substream has gotten closed (e.g. the user pressing the close
      button on an open window). */
  bool becameVoid() const;

  //! Call closeStream(shortname) on all of our substreams
  virtual void closeStream(const std::string& shortname);

  //! Add a frame destination; same as doing --out=source on the command line
  void addFrameDest(const std::string& source);

  //! Get the current frame number
  int frame() const;

  //! Get the number of frame destinations that we have
  size_t getNumFrameDests() const;

  //! Get a reference to the n-th frame destination
  nub::ref<FrameOstream> getFrameDest(size_t n) const;

  //! Try to return a frame destination matching type T, if any
  template <class T>
  nub::soft_ref<T> findFrameDestType() const
  {
    const size_t n = this->getNumFrameDests();
    for (size_t i = 0; i < n; ++i)
      {
        const nub::soft_ref<T> casted =
          dyn_cast_weak<T>(this->getFrameDest(i));

        if (casted.is_valid())
          return casted;
      }
    return nub::soft_ref<T>();
  }

private:
  //! post-commandline-parsing initialization (see ModelComponent.H)
  virtual void start2();

  //! text log file name
  OModelParam<std::string> itsLogFile;

  //! if in test mode, then shouldWait() is always false
  OModelParam<bool> itsTestMode;

  //! range of frames and inter-frame delay in SECONDS
  OModelParam<FrameRange> itsFrameRange;

  //! dims to which frames should be resized on read/write, or 0x0 for none
  OModelParam<Dims> itsDims;

  //! if true, aspect ratio of original image will be preserved when resizing
  OModelParam<bool> itsPreserveAspect;

  //! number of levels to zoom in (positive value) or out (negative value)
  OModelParam<int> itsZoom;

  //! If true, all frame numbers are 000000, forces no numbering
  OModelParam<bool> itsZeroNumberFrames;

  //! catch --out command-line options
  OModelParam<std::string> itsFrameSink;

  //! catch --io command-line options
  OModelParam<std::string> itsInOut;

  //! whether to wait for user's keypress between frames
  OModelParam<bool> itsWaitForUser;

  //! how many times to replicate each output frame
  OModelParam<uint> itsOutputReplicate;

  //! number of frames written since last update
  int itsNumWritten;

  struct Impl;
  Impl* const rep;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
