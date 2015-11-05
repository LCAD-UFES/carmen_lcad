/*!@file Transport/HashOutputSeries.H write a series of image hashes to an output file */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/HashOutputSeries.H $
// $Id: HashOutputSeries.H 9547 2008-03-28 23:32:43Z rjpeters $
//

#ifndef TRANSPORT_HASHOUTPUTSERIES_H_DEFINED
#define TRANSPORT_HASHOUTPUTSERIES_H_DEFINED

#include "Transport/FrameOstream.H"

//! FrameOstream subclass that writes image hashes to an output file
/*! To try out HashOutputSeries, you can for example do the following
    using bin/stream to generate a listing on stderr of 100 32-bit
    hashes of 300x300 random images like this:

      ./bin/stream --in=random:300x300 --in-echo=hash:stderr \
                   --input-frames=0-99@1

    which will generate output like the following:

      000000  5b380a30  0  % input-echo
      000001  1d108dc8  0  % input-echo
      000002  5c0e58aa  0  % input-echo
      000003  cb34047b  0  % input-echo
      000004  f800c826  0  % input-echo
      000005  6386062a  0  % input-echo
      000006  30ed69d6  0  % input-echo

    where the first column is the frame number, the second column is
    the hash, and the third column is a 0 or 1 indicating whether
    we've seen that hash value previously in this run -- note that a
    value of 1 doesn't necessarily imply a hash-function collision; it
    could be that the input frames were exactly the same! The last
    column, after a matlab-comment character (%), is the image name as
    it was passed to writeRGB() or writeGray() or writeFloat().

    If the filename given to HashOutputSeries is "", "-", "stdout", or
    "STDOUT" then the output goes to stdout, likewise "stderr" or
    "STDERR" sends output to stderr, and anything else will be
    interpreted as a literal filename to which output should be
    written.
*/
class HashOutputSeries : public FrameOstream
{
public:
  //! Constructor
  HashOutputSeries(OptionManager& mgr);

  //! Destructor
  virtual ~HashOutputSeries();

  //! Override from FrameOstream; just calls setFileName()
  virtual void setConfigInfo(const std::string& filename);

  //! Set the current frame number and remember it
  virtual bool setFrameNumber(int n);

  //! Write a image hash to the output file
  virtual void writeFrame(const GenericFrame& frame,
                          const std::string& shortname,
                          const FrameInfo& auxinfo);

  //! No-op
  virtual void closeStream(const std::string& shortname);

  //! Specify the output filename
  void setFileName(const std::string& s);

private:
  HashOutputSeries(const HashOutputSeries&);
  HashOutputSeries& operator=(const HashOutputSeries&);

  struct Impl;
  Impl* rep;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_HASHOUTPUTSERIES_H_DEFINED
