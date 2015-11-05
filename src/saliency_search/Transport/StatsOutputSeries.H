/*!@file Transport/StatsOutputSeries.H FrameOstream subclass that writes image statistics to an output file */

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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/StatsOutputSeries.H $
// $Id: StatsOutputSeries.H 9547 2008-03-28 23:32:43Z rjpeters $
//

#ifndef TRANSPORT_STATSOUTPUTSERIES_H_DEFINED
#define TRANSPORT_STATSOUTPUTSERIES_H_DEFINED

#include "Transport/FrameOstream.H"

//! FrameOstream subclass that writes image statistics to an output file
/*! To try out StatsOutputSeries, you can for example do the following
    using bin/stream to generate a listing on stderr of basic
    statistics for 5 random 300x300 color images:

      ./bin/stream --in=random:300x300 --out=stats:stderr \
                   --input-frames=0-4@1

    which will generate output like the following:

    000000 R=[0 .. 127.419667 +/- 73.714227 .. 255] G=[0 .. 127.312844 +/- 73.822887 .. 255] B=[0 .. 127.575222 +/- 73.854606 .. 255] % stream-output (300x300 Image<PixRGB<byte>>)
    000001 R=[0 .. 127.491533 +/- 73.670695 .. 255] G=[0 .. 127.532900 +/- 74.127892 .. 255] B=[0 .. 127.387644 +/- 73.752754 .. 255] % stream-output (300x300 Image<PixRGB<byte>>)
    000002 R=[0 .. 127.553444 +/- 74.032009 .. 255] G=[0 .. 127.531078 +/- 73.875733 .. 255] B=[0 .. 127.660456 +/- 73.902850 .. 255] % stream-output (300x300 Image<PixRGB<byte>>)
    000003 R=[0 .. 127.584222 +/- 73.662192 .. 255] G=[0 .. 127.564222 +/- 74.022741 .. 255] B=[0 .. 127.547267 +/- 73.920700 .. 255] % stream-output (300x300 Image<PixRGB<byte>>)
    000004 R=[0 .. 127.425100 +/- 73.940624 .. 255] G=[0 .. 127.340178 +/- 73.798693 .. 255] B=[0 .. 127.278911 +/- 73.749278 .. 255] % stream-output (300x300 Image<PixRGB<byte>>)
    OVERALL R=[0 .. 127.494793 .. 255] G=[0 .. 127.456244 .. 255] B=[0 .. 127.489900 .. 255] % summary of 5 frames

    where the first column is the frame number, followed by

    [min .. mean +/- stdev .. max]

    for each of the R,G,B components. The final line gives

    [min .. mean .. max]

    over all of the input frames.

    If the filename given to StatsOutputSeries is "", "-", "stdout", or
    "STDOUT" then the output goes to stdout, likewise "stderr" or
    "STDERR" sends output to stderr, and anything else will be
    interpreted as a literal filename to which output should be
    written.
*/
class StatsOutputSeries : public FrameOstream
{
public:
  //! Constructor
  StatsOutputSeries(OptionManager& mgr);

  //! Destructor
  virtual ~StatsOutputSeries();

  //! Override from FrameOstream; just calls setFileName()
  virtual void setConfigInfo(const std::string& filename);

  //! Set the current frame number and remember it
  virtual bool setFrameNumber(int n);

  //! Write image statistics to the output file
  virtual void writeFrame(const GenericFrame& frame,
                          const std::string& shortname,
                          const FrameInfo& auxinfo);

  //! No-op
  virtual void closeStream(const std::string& shortname);

  //! Specify the output filename
  void setFileName(const std::string& s);

private:
  virtual void stop2();

  StatsOutputSeries(const StatsOutputSeries&);
  StatsOutputSeries& operator=(const StatsOutputSeries&);

  struct Impl;
  Impl* rep;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_STATSOUTPUTSERIES_H_DEFINED
