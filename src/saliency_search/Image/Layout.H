/*!@file Image/Layout.H Tiled layouts of arbitrary numbers of images of different sizes */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/Layout.H $
// $Id: Layout.H 10801 2009-02-09 02:07:52Z itti $
//

#ifndef IMAGE_LAYOUT_H_DEFINED
#define IMAGE_LAYOUT_H_DEFINED

#include "Image/CutPaste.H"
#include "Image/Image.H"
#include "Image/ImageSet.H"

#include <vector>

/// Represents tiled layouts of arbitrary numbers of images of different sizes
/** Previously, when we wanted to build up a big image of different
    individual images (e.g. for display), we had two choices:

    - use a series of nested concatX() and concatY() calls, which is
      convenient syntactically but is inefficient because each pixel
      is copied multiple times, once for each concat call
    - make a single output image and use a series of inplacePaste() or
      inplaceEmbed() calls to put the various pieces in place; this is
      efficient but requires manual tabulation of the various corner
      locations of the different tiles

    Now, with the Layout class, we get both efficiency and syntactical
    convenience. Instead of concatX() and concatY(), we use hcat() and
    vcat() which construct Layout objects (with arbitrary nesting);
    none of these calls involves any pixel copying. Instead we just
    keep track of the tile structure that is desired. Then when the
    final Layout has been constructed, it can be rendered into an
    Image in a single pass with the render() call.

    For example, where previously you would have done

      Image<byte> = concatX(e, concatY(concatX(a,b), concatX(c,d)));

    now you would do

      Image<byte> = hcat(e, vcat(hcat(a,b), hcat(c,d))).render();
*/
template <class T>
class Layout
{
public:
  enum Dir { H, V };

  inline Layout();

  inline Layout(const Image<T>& p);

  inline Layout(const Layout<T>& p1, const Layout<T>& p2, Dir d);

  inline Layout(const Image<T>* imgs, const size_t nimgs, Dir d);

  inline Layout(const ImageSet<T>& imgs, Dir d);

  inline Layout(const Layout<T>* imgs, const size_t nimgs, Dir d);

  bool initialized() const { return itsDims.isNonEmpty(); }

  Dims getDims() const { return itsDims; }
  int getWidth() const { return itsDims.w(); }
  int getHeight() const { return itsDims.h(); }

  Dir getDir() const { return itsDir; }

  size_t numParts() const { return itsParts.size(); }

  const Layout<T>& part(size_t i) const { return itsParts[i]; }

  const Image<T>& leafImage() const { return itsLeafImg; }

  inline Image<T> render() const;
  inline Image<T> render(const T& bgcol) const;

  inline void renderInto(Image<T>& x, const Point2D<int> p) const;

private:
  // NOTE: this implementation make sizeof(Layout)==32 on a 32-bit
  // machine; if this becomes a problem it may be possible to reduce
  // the footprint with some optimization tricks (such as: use a
  // custom linked list instead of the std::vector<>; condense
  // itsLeafImg and itsRenderCache into a single image; replace itsDir
  // with a single-bit bitfield)

  std::vector<Layout<T> > itsParts;
  Image<T> itsLeafImg;
  Dir itsDir;
  Dims itsDims;
  mutable Image<T> itsRenderCache;
};

// ######################################################################
// convenience functions for horizontal Layout concatenation

template <class T> inline
Layout<T> hcat(const Image<T>& p1, const Image<T>& p2)
{ return Layout<T>(p1, p2, Layout<T>::H); }

template <class T> inline
Layout<T> hcat(const Image<T>& p1, const Layout<T>& p2)
{ return Layout<T>(p1, p2, Layout<T>::H); }

template <class T> inline
Layout<T> hcat(const Layout<T>& p1, const Image<T>& p2)
{ return Layout<T>(p1, p2, Layout<T>::H); }

template <class T> inline
Layout<T> hcat(const Layout<T>& p1, const Layout<T>& p2)
{ return Layout<T>(p1, p2, Layout<T>::H); }

template <class T> inline
Layout<T> hcat(const Layout<T>& p1, const Layout<T>& p2, const Layout<T>& p3)
{ return Layout<T>(p1, p2, p3, Layout<T>::H); }

template <class T> inline
Layout<T> hcat(const Image<T>* imgs, const size_t nimgs)
{ return Layout<T>(imgs, nimgs, Layout<T>::H); }

template <class T> inline
Layout<T> hcat(const ImageSet<T>& imgs)
{ return Layout<T>(imgs, Layout<T>::H); }

template <class T> inline
Layout<T> hcat(const Layout<T>* imgs, const size_t nimgs)
{ return Layout<T>(imgs, nimgs, Layout<T>::H); }


// ######################################################################
// convenience functions for vertical Layout concatenation

template <class T> inline
Layout<T> vcat(const Image<T>& p1, const Image<T>& p2)
{ return Layout<T>(p1, p2, Layout<T>::V); }

template <class T> inline
Layout<T> vcat(const Image<T>& p1, const Layout<T>& p2)
{ return Layout<T>(p1, p2, Layout<T>::V); }

template <class T> inline
Layout<T> vcat(const Layout<T>& p1, const Image<T>& p2)
{ return Layout<T>(p1, p2, Layout<T>::V); }

template <class T> inline
Layout<T> vcat(const Layout<T>& p1, const Layout<T>& p2)
{ return Layout<T>(p1, p2, Layout<T>::V); }

template <class T> inline
Layout<T> vcat(const Layout<T>& p1, const Layout<T>& p2, const Layout<T>& p3)
{ return Layout<T>(p1, p2, p3, Layout<T>::V); }

template <class T> inline
Layout<T> vcat(const Image<T>* imgs, const size_t nimgs)
{ return Layout<T>(imgs, nimgs, Layout<T>::V); }

template <class T> inline
Layout<T> vcat(const ImageSet<T>& imgs)
{ return Layout<T>(imgs, Layout<T>::V); }

template <class T> inline
Layout<T> vcat(const Layout<T>* imgs, const size_t nimgs)
{ return Layout<T>(imgs, nimgs, Layout<T>::V); }



// ######################################################################
// convenience functions for combination h/v array Layout concatenation

template <class T> inline
Layout<T> arrcat(const Image<T>* imgs, const size_t nimgs, const size_t nx)
{
  std::vector<Layout<T> > rows;

  for (size_t n = 0; n < nimgs; n += nx)
    rows.push_back(hcat(imgs + n, std::min(nimgs - n, nx)));

  return vcat(&rows[0], rows.size());
}

template <class T> inline
Layout<T> arrcat(const ImageSet<T>& imgs, const size_t nx)
{
  const size_t nimgs = imgs.size();
  std::vector<Layout<T> > rows;

  for (size_t n = 0; n < nimgs; n += nx)
    rows.push_back(hcat(imgs.subSet(n, n + std::min(nimgs - n, nx))));

  return vcat(&rows[0], rows.size());
}

// ######################################################################
// Layout implementation
// ######################################################################
template <class T> inline
Layout<T>::Layout() : itsLeafImg(), itsDir(H), itsDims() {}

// ######################################################################
template <class T> inline
Layout<T>::Layout(const Image<T>& p) : itsLeafImg(p), itsDir(H), itsDims(p.getDims()) {}

// ######################################################################
template <class T> inline
Layout<T>::Layout(const Layout<T>& p1, const Layout<T>& p2, Dir d)
  :
  itsDir(d),
  itsDims(d == H
          ? Dims(p1.getDims().w() + p2.getDims().w(),
                 std::max(p1.getDims().h(), p2.getDims().h()))
          : Dims(std::max(p1.getDims().w(), p2.getDims().w()),
                 p1.getDims().h() + p2.getDims().h()))
{ itsParts.push_back(p1); itsParts.push_back(p2); }

// ######################################################################
template <class T> inline
Layout<T>::Layout(const Image<T>* imgs, const size_t nimgs, Dir d)
  :
  itsDir(d),
  itsDims()
{
  int w = 0; int h = 0;

  for (size_t i = 0; i < nimgs; ++i)
    {
      itsParts.push_back(imgs[i]);
      if (d == H)
        {
          w += imgs[i].getWidth();
          h = std::max(h, imgs[i].getHeight());
        }
      else
        {
          w = std::max(w, imgs[i].getWidth());
          h += imgs[i].getHeight();
        }
    }

  itsDims = Dims(w, h);
}

// ######################################################################
template <class T> inline
Layout<T>::Layout(const ImageSet<T>& imgs, Dir d)
  :
  itsDir(d),
  itsDims()
{
  int w = 0; int h = 0;
  const uint nimgs = imgs.size();

  for (size_t i = 0; i < nimgs; ++i)
    {
      itsParts.push_back(imgs[i]);
      if (d == H)
        {
          w += imgs[i].getWidth();
          h = std::max(h, imgs[i].getHeight());
        }
      else
        {
          w = std::max(w, imgs[i].getWidth());
          h += imgs[i].getHeight();
        }
    }

  itsDims = Dims(w, h);
}

// ######################################################################
template <class T> inline
Layout<T>::Layout(const Layout<T>* imgs, const size_t nimgs, Dir d)
  :
  itsDir(d),
  itsDims()
{
  int w = 0; int h = 0;

  for (size_t i = 0; i < nimgs; ++i)
    {
      itsParts.push_back(imgs[i]);
      if (d == H)
        {
          w += imgs[i].getDims().w();
          h = std::max(h, imgs[i].getDims().h());
        }
      else
        {
          w = std::max(w, imgs[i].getDims().w());
          h += imgs[i].getDims().h();
        }
    }

  itsDims = Dims(w, h);
}

// ######################################################################
template <class T> inline
Image<T> Layout<T>::render() const
{
  if (itsParts.size() == 0) return itsLeafImg;

  if (!itsRenderCache.initialized())
    {
      itsRenderCache = Image<T>(this->getDims(), ZEROS);

      this->renderInto(itsRenderCache, Point2D<int>(0,0));
    }

  return itsRenderCache;
}

// ######################################################################
template <class T> inline
Image<T> Layout<T>::render(const T& bgcol) const
{
  if (itsParts.size() == 0) return itsLeafImg;

  if (!itsRenderCache.initialized())
    {
      itsRenderCache = Image<T>(this->getDims(), NO_INIT);
      itsRenderCache.clear(bgcol);

      this->renderInto(itsRenderCache, Point2D<int>(0,0));
    }

  return itsRenderCache;
}

// ######################################################################
template <class T> inline
void Layout<T>::renderInto(Image<T>& x, const Point2D<int> p) const
{
  if (itsParts.size() == 0)
    {
      if (itsLeafImg.initialized())
        inplacePaste(x, itsLeafImg, p);
    }
  else if (itsDir == H)
    {
      Point2D<int> pp = p;
      for (size_t i = 0; i < itsParts.size(); ++i)
        {
          itsParts[i].renderInto(x, pp);
          pp.i += itsParts[i].getDims().w();
        }
    }
  else
    {
      Point2D<int> pp = p;
      for (size_t i = 0; i < itsParts.size(); ++i)
        {
          itsParts[i].renderInto(x, pp);
          pp.j += itsParts[i].getDims().h();
        }
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_LAYOUT_H_DEFINED
