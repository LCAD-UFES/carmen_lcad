/*!@file GUI/ImageDisplayStream.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/ImageDisplayStream.H $
// $Id: ImageDisplayStream.H 10954 2009-02-28 19:37:18Z lior $
//

#ifndef GUI_IMAGEDISPLAYSTREAM_H_DEFINED
#define GUI_IMAGEDISPLAYSTREAM_H_DEFINED

#include "Component/ModelParam.H"
#include "Transport/FrameOstream.H"

namespace rutz { template <class T> class shared_ptr; }
class XWinManaged;

//! A FrameOstream class that sends images to (multiple) on-screen window(s)
class ImageDisplayStream : public FrameOstream
{
public:
  //! Constructor
  ImageDisplayStream(OptionManager& mgr);

  //! Virtual destructor.
  virtual ~ImageDisplayStream();

  //! Write a frame to the named onscreen window
  /*! If no such window yet exists, it will be created on the fly.

      This implementation is optimized to take advantage of
      XWindow::drawRgbLayout() and XWindow::drawGrayLayout() if the
      given GenericFrame includes Layout information.
   */
  virtual void writeFrame(const GenericFrame& image,
                          const std::string& shortname,
                          const FrameInfo& auxinfo = FrameOstream::defaultInfo);

  //! Returns true if the user has clicked a close button on any open window
  virtual bool isVoid() const;

  //! Close the window corresponding to shortname, if any
  virtual void closeStream(const std::string& shortname);

  //! Return the window associated with wintitle, or null if there is no such window
  rutz::shared_ptr<XWinManaged> getWindow(const std::string& wintitle);

  //! Return the windows map in here
  std::vector<rutz::shared_ptr<XWinManaged> > getWindows();

private:
  OModelParam<bool> itsTestMode;
  OModelParam<int> itsMaxDisplay;
  OModelParam<bool> itsTraceXEvents;

  struct WindowMap;
  WindowMap* wmap;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // GUI_IMAGEDISPLAYSTREAM_H_DEFINED
