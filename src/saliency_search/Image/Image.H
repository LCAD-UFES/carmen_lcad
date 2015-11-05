/*!@file Image/Image.H An image template class with many image processing
  functions */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/Image.H $
// $Id: Image.H 14755 2011-04-29 05:55:18Z itti $
//

#ifndef IMAGE_H_DEFINED
#define IMAGE_H_DEFINED

// Uncomment this to allow range checking in Image iterators:
//#define INVT_MEM_DEBUG

#include "Util/Assert.H"
#include "Util/Promotions.H"
#include "Image/ArrayData.H"
#include "Image/Point2D.H"
#include "Image/Rectangle.H"

#ifdef INVT_MEM_DEBUG
#include "Image/CheckedIterator.H"
#endif

#include <algorithm> // for std::min
#include <iterator>

//! Generic image template class
/*! This is a generic image template class that can handle grayscale
    as well as color or multispectral images. All Image methods should
    be instantiable for any type T that has the basic arithmetic
    operators. Note that some external Image functions will only work
    for scalar types (e.g. if comparison operators are needed), and
    other will only work for composite types such as PixRGB (e.g. if
    luminance() is needed).

    Note also that many other image manipulation functions are defined
    as non-member functions in Image/ColorOps.H, Image/FilterOps.H,
    Image/IO.H, Image/MathOps.H, Image/Omni.H, Image/ShapeOps.H, and
    Image/Transforms.H.
*/

template <class T>
class Image
{
public:

  // ############################################################
  /*! @name Constructors, destructors, assignment */
  //@{

  //! Construct from C array
  /*! Build from C array; an internal copy of the C array will be
      allocated, so the C array can (and should) be freed without
      affecting the Image. */
  inline Image(const T* inarray, int width, int height);

  //! Construct from C array
  /*! Build from C array; an internal copy of the C array will be
      allocated, so the C array can (and should) be freed without
      affecting the Image. */
  inline Image(const T* inarray, const Dims& dims);

  //! Allocates memory for given size, and optionally zero-clear that memory.
  inline Image(int width, int height, InitPolicy init);

  //! Constructor that only allocates memory for given size
  inline explicit Image(const Dims& dims, InitPolicy init);

  //! Construct an empty (0-by-0) image (useful for arrays of Images).
  inline Image();

  //! Copy constructor
  /*! e.g.:
      \code
      Image<byte> im(other);
      // or
      Image<byte> im = other; // with other also of type Image<byte>
      \endcode
  */
  inline Image(const Image<T>& A);

  //! Conversion copy constructor
  /*! e.g.:
      \code
      Image<byte> im(other);
      // or
      Image<byte> im = other; // with other of type Image<float>
      \endcode
  */
  template <class T2> inline Image(const Image<T2>& A);

  //! Assigment operator.
  /*! e.g.:
      \code
      Image<byte> im1, im2; im2 = im1;
      \endcode
  */
  inline Image<T>& operator=(const Image<T>& A);

  //! Conversion assigment operator.
  /*! e.g.:
      \code
      Image<byte> im1; Image<float> im2; im2 = im1;
      \endcode

  */
  template <class T2> inline Image<T>& operator=(const Image<T2>& A);

  //! Destructor
  inline ~Image();

  //! Free memory and switch to uninitialized state.
  /*! Note that it is \b NOT necessary to call this function to ensure
      proper cleanup, that will be done in the destructor by
      default. Rather, freeMem() is offered just as a performance
      optimization, to allow you to release a potentially large chunk
      of memory when you are finished using it. */
  inline void freeMem();

  //@}

  // ############################################################
  /*! @name Memory management functions */
  //@{

  //! Swap the contents of two images
  inline void swap(Image<T>& other);

  //! Use existing memory.
  /*! This is potentially dangerous and should really be avoided. The
      only case where this really is useful is to attach an image to
      an existing memory segment that is shared or into which data is
      streaming via DMA. */
  inline void attach(T* array, const int width, const int height);

  //! Detach previously attach()'ed image.
  /*! The main purpose of detach() is to make sure that this Image
      object does not continue to point at attach()'ed memory after
      that memory has been freed. Nevertheless, this function is not
      strictly necessary to ensure correct memory handling, since the
      Image destructor will only try to free memory if that memory is
      owned (i.e., not attach()'ed). All that it does is release any
      association with a previously attach()'ed memory block, by
      setting the current image to a new empty (zero-by-zero)
      ArrayData object. We assume that attached memory will be
      destroyed later. */
  inline void detach();

  //! Return a new image object with a deep copy of the underlying data
  /*! This function is necessary for safe use of
      attach()/detach(). That is, unfortunately attach()/detach() are
      not safe for use with shared image objects -- consider the
      following code:

      \code
      double d[4] = { 0.0, 1.0, 2.0, 3.0};

      // create an Image that is attach()'ed to the double array
      Image<double> a;
      a.attach(&d[0], 2, 2);

      const Image<double> b = a;
      // now 'b' thinks it has a safe lock on some const values:

      d[0] = -1.0;
      // OOPS! By changing values in the 'd' array directly, we'll now
      // have changed things to that b[0] == -1.0, even though 'b' was
      // declared as 'const'
      \endcode

      The solution to this problem is to prohibit the copy done in
      'b=a' above (this triggers an LFATAL() in
      ArrayData::acquire()). That assures us that any ArrayData that
      has a StoragePolicy of WRITE_THRU will be un-shareable.

      So, back to the point -- the correct way to write the code above
      would be to use deepcopy():

      \code
      double d[4] = { 0.0, 1.0, 2.0, 3.0};

      Image<double> a;
      a.attach(&d[0], 2, 2);

      const Image<double> b = a.deepcopy();

      d[0] = -1.0;
      // Now, 'b' is insulated from any changes to 'd' since we've
      // done a deep copy, so even now we'll still have b[0]==0.0
      \endcode
  */
  inline Image<T> deepcopy() const;

  //! Free mem and realloc new array (array contents are lost).
  /*! Use rescale() instead if you want to preserve image contents. */
  inline void resize(const Dims& dims, const bool clear = false);

  //! Free mem and realloc new array (array contents are lost).
  /*! Use rescale() instead if you want to preserve image contents. */
  inline void resize(const int width, const int height,
                     const bool clear = false);

  //@}

  // ############################################################
  /*! @name Iterators

      There are const and non-const versions of iterators, which are
      returned by begin()/end() and beginw()/endw(), respectively. The
      "w" in beginw()/endw() is a mnemonic for "write" or
      "writeable". Beware that the non-const versions of these
      functions will invoke the copy-on-write mechanism (see
      ArrayHandle), so a deep copy of all the image data will be made
      if the current image object is not the unique owner of its image
      data. Thus, begin()/end() should always be used unless write
      access is specifically needed. (It is for this reason that we
      don't overload the const/non-const functions with the same names
      (even though this is possible in C++, as done in the STL
      containers)).

      Debugging iterators (CheckedIterator) will be used if the macro
      INVT_MEM_DEBUG is #define'd. A checked iterator will check that
      it is within its proper bounds every time that it is
      dereferenced with either operator*() or operator->(). Note that
      the existence of the checked iterators means you cannot rely on
      an image iterator being a raw pointer. Instead, getArrayPtr() is
      available in case you unconditionally need a raw pointer to the
      image data.
   */
  //@{

#ifndef INVT_MEM_DEBUG
  // standard iterators are just pointers to the data:
  //! Read/write iterator.
  typedef T* iterator;
  //! Read-only iterator.
  typedef const T* const_iterator;
#else
  // range-checked iterators can detect attempts to access past image bounds:
  //! Read/write iterator.
  typedef CheckedIterator<T> iterator;
  //! Read-only iterator.
  typedef CheckedIterator<const T> const_iterator;
#endif

  //! Returns a read-only iterator to the beginning of the image data
  inline const_iterator begin() const;

  //! Returns a read-only iterator to one-past-the-end of the image data
  inline const_iterator end() const;

  //! Returns a read-write iterator to the beginning of the image data
  inline iterator beginw();

  //! Returns a read-write iterator to one-past-the-end of the image data
  inline iterator endw();

  //@}

  // ############################################################
  /*! @name Access functions*/
  //@{

  //! Check whether image is non-empty (i.e., non-zero height and width).
  inline bool initialized() const;

  //! Get image size (width * height)
  inline int getSize() const;

  //! Get image size (width * height)
  inline uint size() const;

  //! Get image width
  inline int getWidth() const;

  //! Get image height
  inline int getHeight() const;

  //! Get image width+height in Dims struct
  inline const Dims& getDims() const;

  //! Get image bounds as a rectangle with upper-left point at (0,0) and dims matching the image dims
  inline Rectangle getBounds() const;

  //! Check if *this is the same size as the other thing
  /*! The other thing can be any type that exposes getHeight() and
      getWidth() */
  template <class C>
  inline bool isSameSize(const C& other) const;

  //! Check if the image is 1D, i.e., width == 1 or height == 1
  inline bool is1D() const;

  //! Check if the image is a vector, i.e., width == 1
  inline bool isVector() const;

  //! Check if the image is a transposed vector, i.e., height == 1
  inline bool isTransposedVector() const;

  //! Check if the image is square, i.e., width == height
  inline bool isSquare() const;

  //! Access image elements through C array index interface
  inline T& operator[](const int index);

  //! Access image elements through C array index interface
  inline const T& operator[](const int index) const;

  //! Access image elements through C array index interface
  inline T& operator[](const Point2D<int>& p);

  //! Access image elements through C array index interface
  inline const T& operator[](const Point2D<int>& p) const;

  //! Get pixel value at index in image
  inline const T& getVal(const int index) const;

  //! Get pixel value at (x, y) in image
  inline const T& getVal(const int x, const int y) const;

  //! Get pixel value at specified coordinates in image
  inline const T& getVal(const Point2D<int>& p) const;

  //! Get value at (x,y), put in val rather that return it
  template <class T2> inline void getVal(const int x, const int y,
                                         T2& val) const;

  //! Get pixel value at (x, y) in image with bilinear interpolation
  inline T getValInterp(const float x, const float y) const;

  //! Get pixel value at (x, y) in image with bilinear interpolation
  inline T getValInterp(const Point2D<float>& p) const;

  //! Get pixel value with bilinear interpolation at a location (x,y) specified in a different Dims scale
  /*! We first scale the coordinates into the proper Dims for this
      image, and then use getValInterp() */
  inline T getValInterpScaled(const Point2D<int>& p, const Dims& pdims) const;

  //! Set value in image at index
  template <class T2> inline void setVal(const int index, const T2& value);

  //! Set value in image at (x, y)
  template <class T2> inline void setVal(const int x, const int y,
                                         const T2& value);

  //! Set value in image at Point2D<int>
  template <class T2> inline void setVal(const Point2D<int>& p, const T2& value);

  //! Returns read-only (const) pointer to internal image array
  inline const T* getArrayPtr() const;

  //! Returns read/write (non-const) pointer to internal image array
  inline T* getArrayPtr();

  //! Test whether point falls inside array boundaries
  inline bool coordsOk(const Point2D<int>& P) const;

  //! Test whether point falls inside array boundaries
  inline bool coordsOk(const int i, const int j) const;

  //! Test whether point falls inside array boundaries
  /*! This test is intended to be used before you attempt a getValInterp() */
  inline bool coordsOk(const Point2D<float>& p) const;

  //! Test whether point falls inside array boundaries
  /*! This test is intended to be used before you attempt a getValInterp() */
  inline bool coordsOk(const float i, const float j) const;

  //! Test whether rectangle fits in image
  inline bool rectangleOk(const Rectangle& rect) const;

  //@}

  // ############################################################
  /*! @name Operator overloads and basic image manipulations

      Note that many other image manipulation functions are available
      in Image_ColorOps.H, Image_FilterOps.H, Image_IO.H,
      Image_MathOps.H, Image_Omni.H, Image_ShapeOps.H,
      Image_Transforms.H, and Image_Conversions.H.
  */
  //@{

  //! Equality: true if the images are the same size and all pixels are equal.
  bool operator==(const Image<T>& that) const;

  //! Add constant to image, clamp result as necessary
  Image<T>& operator+=(const T& val);

  //! Subtract constant from image, clamp result as necessary
  Image<T>& operator-=(const T& val);

  //! Multiply image by constant, clamp result as necessary
  Image<T>& operator*=(const T& val);

  //! Divide image by constant, clamp result as necessary
  Image<T>& operator/=(const T& val);

  //! Bit-shift left by constant (type T must have operator<<())
  Image<T>& operator<<=(const unsigned int nbits);

  //! Bit-shift right by constant (type T must have operator>>())
  Image<T>& operator>>=(const unsigned int nbits);


  //! Add image to image, clamp result as necessary
  template <class T2> inline
  Image<T>& operator+=(const Image<T2>& A);

  //! Subtract image from image, clamp result as necessary
  template <class T2> inline
  Image<T>& operator-=(const Image<T2>& A);

  //! Multiply image by image, point-by-point, clamp result as necessary
  template <class T2> inline
  Image<T>& operator*=(const Image<T2>& A);

  //! Divide image by image, point-by-point, clamp result as necessary
  template <class T2> inline
  Image<T>& operator/=(const Image<T2>& A);

  //! Bitwise-or image by image, point-by-point, clamp result as necessary
  template <class T2> inline
  Image<T>& operator|=(const Image<T2>& A);


  //! Add scalar to each point in *this and return result
  template <class T2> inline
  Image<typename promote_trait<T,T2>::TP> operator+(const T2& val) const;

  //! Subtract scalar from each point in *this and return result
  template <class T2> inline
  Image<typename promote_trait<T,T2>::TP> operator-(const T2& val) const;

  //! Multiply scalar each point in *this by scalar and return result
  template <class T2> inline
  Image<typename promote_trait<T,T2>::TP> operator*(const T2& val) const;

  //! Divide each point in *this by scalar and return result
  template <class T2> inline
  Image<typename promote_trait<T,T2>::TP> operator/(const T2& val) const;

  //! Bit-shift left by constant and return result (type T must have operator<<())
  Image<T> operator<<(const unsigned int nbits) const;

  //! Bit-shift right by constant and return result (type T must have operator>>())
  Image<T> operator>>(const unsigned int nbits) const;


  //! Point-wise add img to image and return result
  template <class T2> inline
  Image<typename promote_trait<T,T2>::TP> operator+(const
                                                    Image<T2>& img) const;

  //! Point-wise subtract img from *this and return result
  template <class T2> inline
  Image<typename promote_trait<T,T2>::TP> operator-(const
                                                    Image<T2>& img) const;

  //! Point-wise multiply *this by img and return result
  template <class T2> inline
  Image<typename promote_trait<T,T2>::TP> operator*(const
                                                    Image<T2>& img) const;

  //! Point-wise divide *this by img and return result
  template <class T2> inline
  Image<typename promote_trait<T,T2>::TP> operator/(const
                                                    Image<T2>& img) const;


  //! clear contents (or set to given value)
  inline void clear(const T& val = T());

  //@}

  // ############################################################
  /*! @name Functions for testing/debugging only */
  //@{

  //! For testing/debugging only.
  /*! See if we are pointing to the same ArrayData<T> as is the other
      Image<T>. */
  bool hasSameData(const Image<T>& b) const;

  //! For testing/debugging only.
  /*! Returns the current reference count. */
  long refCount() const throw();

  //! For testing/debugging only.
  /*! Check if the ArrayHandle is shared. */
  bool isShared() const throw();

  //@}

private:
  // ############################################################
  // ##### Data:
  // ############################################################
  ArrayHandle<T> itsHdl;
  inline const ArrayData<T>& impl() const;
  inline ArrayData<T>& uniq();
};

// ######################################################################
// ######################################################################
// ######################################################################
// FREE FUNCTIONS:
// ######################################################################
// ######################################################################
// ######################################################################

//! Return a new Image with the same data, but interpreted as a different shape
/*! NOTE that this is very different from resize()! With reshape(), we
    are not changing the number of elements in the Image; rather we
    are just changing the way that the internal 1-D memory array is
    mapped to a logical 2-D Image. For example, you could reshape a
    10x5 Image to a 50x1 Image, or a 25x2 Image, or vice versa. You
    need to be aware the fact that Image uses a row-major storage
    format internally (i.e., as you step through the memory, you first
    traverse all the pixels in row 1, then all the pixels in row 2,
    etc.). So if you reshape a WxH Image to a WHx1 Image, your new 1-D
    Image will appear to have the rows of the original Image arranged
    end-to-end.

    Programmer note: In principle, we could support a reshape()
    operation that wouldn't require copying any data; in order to do
    that we'd need to split the Dims management out of ArrayData and
    ArrayHandle, so that a single ArrayData object could be used in
    multiple Image objects, each with potentially different Dims (but
    all with the same total number of pixels). However, for now we
    just use a simpler implementation which copies the data into the
    new Image.
*/
template <class T>
inline Image<T> reshape(const Image<T>& orig, const Dims& newdims)
{
  ASSERT(orig.getDims().sz() == newdims.sz());
  return Image<T>(orig.getArrayPtr(), newdims);
}

// ######################################################################
// ######################################################################
// ######################################################################
// Iterator traits
// ######################################################################
// ######################################################################
// ######################################################################
namespace std
{
  template <class T> struct iterator_traits< Image<T> >
  {
      typedef T difference_type;
      typedef T value_type;
      typedef T* pointer;
      typedef T& reference;
      typedef random_access_iterator_tag iterator_category;
  };
}

// ######################################################################
// ######################################################################
// ######################################################################
// INLINE FUNCTIONS:
// ######################################################################
// ######################################################################
// ######################################################################

// ######################################################################
// ##### Constructors & Destructors:
// ######################################################################

// ######################################################################
template <class T> inline
Image<T>::Image(const T* inarray, int width, int height) :
  itsHdl(new ArrayData<T>(Dims(width, height), inarray))
{}

// ######################################################################
template <class T> inline
Image<T>::Image(const T* inarray, const Dims& dims) :
  itsHdl(new ArrayData<T>(dims, inarray))
{}

// ######################################################################
template <class T> inline
Image<T>::Image(int width, int height, InitPolicy init) :
  itsHdl(new ArrayData<T>(Dims(width, height), init))
{}

// ######################################################################
template <class T> inline
Image<T>::Image(const Dims& dims, InitPolicy init) :
  itsHdl(new ArrayData<T>(dims, init))
{}

// ######################################################################
template <class T> inline
Image<T>::Image() :
  itsHdl(new ArrayData<T>())
{}

// ######################################################################
template <class T> inline
Image<T>::Image(const Image<T>& A) :
  itsHdl(A.itsHdl)
{}

// ######################################################################
template <class T> template <class T2> inline
Image<T>::Image(const Image<T2>& A) :
  itsHdl(new ArrayData<T>(A.getDims(), NO_INIT))
{
  typename Image<T2>::const_iterator sptr = A.begin();
  typename Image<T>::iterator aptr = beginw();
  typename Image<T>::iterator stop = endw();
  while (aptr != stop)
    {
      *aptr++ = clamped_convert<T>(*sptr++);
    }
}

// ######################################################################
template <class T> inline
Image<T>& Image<T>::operator=(const Image<T>& A)
{
  Image<T> A_copy( A );
  this->swap(A_copy);
  return *this;
}

// ######################################################################
template <class T> template <class T2> inline
Image<T>& Image<T>::operator=(const Image<T2>& A)
{
  // make a new image block if necessary
  if (this->getDims() != A.getDims() || itsHdl.isShared())
    *this = Image<T>(A.getDims(), NO_INIT);

  typename Image<T2>::const_iterator sptr = A.begin();
  typename Image<T>::iterator aptr = this->beginw();
  typename Image<T>::iterator stop = this->endw();
  while (aptr != stop) *aptr++ = clamped_convert<T>(*sptr++);
  return *this;
}

// ######################################################################
template <class T> inline
Image<T>::~Image()
{ /* memory deallocation is handled by ArrayData's destructor */ }

// ######################################################################
template <class T> inline
void Image<T>::freeMem()
{
  Image<T> empty;
  this->swap(empty);
}

// ######################################################################
// ##### Memory management functions:
// ######################################################################

// ######################################################################
template <class T> inline
void Image<T>::swap(Image<T>& other)
{
  itsHdl.swap(other.itsHdl);
}

// ######################################################################
template <class T> inline
void Image<T>::attach(T* array, const int w, const int h)
{
  ArrayHandle<T> attached(new ArrayData<T>(Dims(w, h), array, WRITE_THRU));
  itsHdl.swap(attached);
}

// ######################################################################
template <class T> inline
void Image<T>::detach()
{
  ArrayHandle<T> emptyHdl;
  itsHdl.swap(emptyHdl);
}

// ######################################################################
template <class T> inline
Image<T> Image<T>::deepcopy() const
{
  // see comment in class definition for why deepcopy() might be
  // needed
  return Image<T>(this->getArrayPtr(), this->getDims());
}

// ######################################################################
template <class T> inline
void Image<T>::resize(const Dims& dims, const bool do_clear)
{
  // This algorithm is slightly more convoluted than may appear necessary
  // at first, in order to be most efficient in the context of
  // copy-on-write. If the requested size is different than the current
  // size, it's very simple: we just make a new image of the right size,
  // clearing it if so requested. Otherwise, if we're keeping the same
  // size, we don't necessarily have to make a new data block; instead we
  // just delegate to clear(), which does the right thing depending on
  // whether we currently have a shared or unshared data block.

  if (dims != getDims())
    {
      ArrayHandle<T> resized(new ArrayData<T>(dims, do_clear ? ZEROS:NO_INIT));
      itsHdl.swap(resized);
    }
  else // we're keeping the same size, so just clear() if necessary
    {
      if (do_clear) clear( T() );
    }
}

// ######################################################################
template <class T> inline
void Image<T>::resize(const int width, const int height, const bool do_clear)
{
  resize(Dims(width, height), do_clear);
}

// ######################################################################
// ##### Iterators:
// ######################################################################

#ifndef INVT_MEM_DEBUG

template<class T> inline
typename Image<T>::const_iterator Image<T>::begin() const
{ return impl().data(); }

template<class T> inline
typename Image<T>::const_iterator Image<T>::end() const
{ return impl().end(); }

template<class T> inline
typename Image<T>::iterator Image<T>::beginw()
{ return uniq().dataw(); }

template<class T> inline
typename Image<T>::iterator Image<T>::endw()
{ return uniq().endw(); }

#else

template <class T> inline
typename Image<T>::const_iterator Image<T>::begin() const
{ return const_iterator(impl().data(), impl().data(), impl().end()); }

template <class T> inline
typename Image<T>::const_iterator Image<T>::end() const
{ return const_iterator(impl().end(), impl().data(), impl().end()); }

template <class T> inline
typename Image<T>::iterator Image<T>::beginw()
{ return iterator(uniq().dataw(), uniq().dataw(), uniq().endw()); }

template <class T> inline
typename Image<T>::iterator Image<T>::endw()
{ return iterator(uniq().endw(), uniq().dataw(), uniq().endw()); }

#endif

// ######################################################################
// ##### Access functions:
// ######################################################################

// ######################################################################
template <class T> inline
bool Image<T>::initialized() const
{ return getWidth() > 0 && getHeight() > 0; }

// ######################################################################
template <class T> inline
int Image<T>::getSize() const
{ return getDims().sz(); }

// ######################################################################
template <class T> inline
uint Image<T>::size() const
{ return getDims().sz(); }

// ######################################################################
template <class T> inline
int Image<T>::getWidth() const
{ return itsHdl.get().w(); }

// ######################################################################
template <class T> inline
int Image<T>::getHeight() const
{ return itsHdl.get().h(); }

// ######################################################################
template <class T> inline
const Dims& Image<T>::getDims() const
{ return itsHdl.get().dims(); }

// ######################################################################
template <class T> inline
Rectangle Image<T>::getBounds() const
{ return Rectangle(Point2D<int>(0,0), itsHdl.get().dims()); }

// ######################################################################
template <class T> template <class C> inline
bool Image<T>::isSameSize(const C& other) const
{ return getWidth() == other.getWidth() && getHeight() == other.getHeight(); }

// ######################################################################
template <class T> inline
bool Image<T>::is1D() const
{ return (getWidth() == 1) || (getHeight() == 1); }

// ######################################################################
template <class T> inline
bool Image<T>::isVector() const
{ return (getWidth() == 1); }

// ######################################################################
template <class T> inline
bool Image<T>::isTransposedVector() const
{ return (getHeight() == 1); }

// ######################################################################
template <class T> inline
bool Image<T>::isSquare() const
{ return (getWidth() == getHeight()); }

// ######################################################################
template <class T> inline
T& Image<T>::operator[](int index)
{
  ASSERT(index >= 0 && index < this->getSize());
  return beginw()[index];
}

// ######################################################################
template <class T> inline
const T& Image<T>::operator[](int index) const
{
  ASSERT(index >= 0 && index < this->getSize());
  return begin()[index];
}

// ######################################################################
template <class T> inline
T& Image<T>::operator[](const Point2D<int>& p)
{
  ASSERT(this->coordsOk(p));
  return this->beginw()[p.i + p.j * this->getWidth()];
}

// ######################################################################
template <class T> inline
const T& Image<T>::operator[](const Point2D<int>& p) const
{
  ASSERT(this->coordsOk(p));
  return this->begin()[p.i + p.j * this->getWidth()];
}

// ######################################################################
template <class T> inline
const T& Image<T>::getVal(const int index) const
{
  ASSERT(index >= 0 && index < this->getSize());
  return begin()[index];
}

// ######################################################################
template <class T> inline
const T& Image<T>::getVal(const int x, const int y) const
{
  ASSERT(coordsOk(x, y));
  return(begin()[x + y * getWidth()]);
}

// ######################################################################
template <class T> inline
const T& Image<T>::getVal(const Point2D<int>& p) const
{
  ASSERT(coordsOk(p));
  return(begin()[p.i + p.j * getWidth()]);
}

// ######################################################################
template <class T> template <class T2> inline
void Image<T>::getVal(const int x, const int y, T2& val) const
{
  ASSERT(coordsOk(x, y));
  val = begin()[x + y * getWidth()];
}

// ######################################################################
template <class T> inline
T Image<T>::getValInterp(const float x, const float y) const
{
  typename Image<T>::const_iterator const sptr = begin();
  const int wid = getWidth(), hei = getHeight();

  const int y0 = int(y);
  const int y1 = std::min(y0 + 1, hei - 1);
  const float fy = y - float(y0);

  const int wy0 = wid * y0;
  const int wy1 = wid * y1;

  const int x0 = int(x);
  const int x1 = std::min(x0 + 1, wid - 1);
  const float fx = x - float(x0);

  ASSERT(coordsOk(x0, y0));
  typename promote_trait<T, float>::TP const
    d00( sptr[x0 + wy0] ), d10( sptr[x1 + wy0] ),
    d01( sptr[x0 + wy1] ), d11( sptr[x1 + wy1] ),
    dx0( d00 + (d10 - d00) * fx ),
    dx1( d01 + (d11 - d01) * fx );

  return T( dx0 + (dx1 - dx0) * fy );  // no need to clamp
}

// ######################################################################
template <class T> inline
T Image<T>::getValInterp(const Point2D<float>& p) const
{
  return this->getValInterp(p.i, p.j);
}

// ######################################################################
template <class T> inline
T Image<T>::getValInterpScaled(const Point2D<int>& p,
                               const Dims& pdims) const
{
  const float x =
    std::max(0.0f, (float(p.i)+0.5f) * float(this->getWidth())/pdims.w() - 0.5f);
  const float y =
    std::max(0.0f, (float(p.j)+0.5f) * float(this->getHeight())/pdims.h() - 0.5f);

  return this->getValInterp(x, y);
}

// ######################################################################
template <class T> template <class T2> inline
void Image<T>::setVal(const int index, const T2& value)
{
  ASSERT(index >= 0 && index < this->getSize());
  beginw()[index] = clamped_convert<T>(value);
}

// ######################################################################
template <class T> template <class T2> inline
void Image<T>::setVal(const int x, const int y, const T2& value)
{
  ASSERT(coordsOk(x, y));
  beginw()[x + y * getWidth()] = clamped_convert<T>(value);
}

// ######################################################################
template <class T> template <class T2> inline
void Image<T>::setVal(const Point2D<int>& p, const T2& value)
{
  ASSERT(coordsOk(p));
  beginw()[p.i + p.j * getWidth()] = clamped_convert<T>(value);
}

// ######################################################################
template <class T> inline
const T* Image<T>::getArrayPtr() const
{
  return impl().data();
}

// ######################################################################
template <class T> inline
T* Image<T>::getArrayPtr()
{
  return uniq().dataw();
}

// ######################################################################
template <class T> inline
bool Image<T>::coordsOk(const Point2D<int>& P) const
{
  return (P.i >= 0 && P.j >= 0 && P.i < getWidth() && P.j < getHeight());
}

// ######################################################################
template <class T> inline
bool Image<T>::coordsOk(const int i, const int j) const
{
  return (i >= 0 && j >= 0 && i < getWidth() && j < getHeight());
}

// ######################################################################
template <class T> inline
bool Image<T>::coordsOk(const Point2D<float>& p) const
{
  return this->coordsOk(p.i, p.j);
}

// ######################################################################
template <class T> inline
bool Image<T>::coordsOk(const float i, const float j) const
{
  return (i >= 0.0F && j >= 0.0F &&
          i < float(getWidth() - 1) && j < float(getHeight() - 1));
}

// ######################################################################
template <class T> inline
bool Image<T>::rectangleOk(const Rectangle& rect) const
{
  return (rect.left() < getWidth() && rect.rightI() < getWidth() &&
          rect.top() < getHeight() && rect.bottomI() < getHeight() &&
          rect.left() >= 0 && rect.rightI() >= 0 &&
          rect.top() >= 0 && rect.bottomI() >= 0);
}

// ######################################################################
template <class T>
bool Image<T>::operator==(const Image<T>& that) const
{
  if (!this->isSameSize(that)) return false;

  return std::equal(this->begin(), this->end(), that.begin());
}

// ######################################################################
template <class T> inline
Image<T>& Image<T>::operator+=(const T& val)
{
  for (typename Image<T>::iterator itr = beginw(), stop = endw();
       itr != stop; ++itr)
    *itr = clamped_convert<T>( (*itr) + val );
  return *this;
}

// ######################################################################
template <class T> inline
Image<T>& Image<T>::operator-=(const T& val)
{
  for (typename Image<T>::iterator itr = beginw(), stop = endw();
       itr != stop; ++itr)
    *itr = clamped_convert<T>( (*itr) - val );
  return *this;
}

// ######################################################################
template <class T> inline
Image<T>& Image<T>::operator*=(const T& val)
{
  for (typename Image<T>::iterator itr = beginw(), stop = endw();
       itr != stop; ++itr)
    *itr = clamped_convert<T>( (*itr) * val );
  return *this;
}

// ######################################################################
template <class T> inline
Image<T>& Image<T>::operator/=(const T& val)
{
  for (typename Image<T>::iterator itr = beginw(), stop = endw();
       itr != stop; ++itr)
    *itr = clamped_convert<T>( (*itr) / val );
  return *this;
}

// ######################################################################
template <class T> inline
Image<T>& Image<T>::operator<<=(const unsigned int nbits)
{
  for (typename Image<T>::iterator itr = beginw(), stop = endw();
       itr != stop; ++itr)
    *itr <<= nbits;
  return *this;
}

// ######################################################################
template <class T> inline
Image<T>& Image<T>::operator>>=(const unsigned int nbits)
{
  for (typename Image<T>::iterator itr = beginw(), stop = endw();
       itr != stop; ++itr)
    *itr >>= nbits;
  return *this;
}

// ######################################################################
template <class T> template <class T2> inline
Image<T>& Image<T>::operator+=(const Image<T2>& A)
{
  ASSERT(isSameSize(A));
  typename Image<T2>::const_iterator itr2 = A.begin();
  for (typename Image<T>::iterator itr = beginw(), stop = endw();
       itr != stop; ++itr, ++itr2)
    *itr = clamped_convert<T>( (*itr) + (*itr2) );
  return *this;
}

// ######################################################################
template <class T> template <class T2> inline
Image<T>& Image<T>::operator-=(const Image<T2>& A)
{
  ASSERT(isSameSize(A));
  typename Image<T2>::const_iterator itr2 = A.begin();
  for (typename Image<T>::iterator itr = beginw(), stop = endw();
       itr != stop; ++itr, ++itr2)
    *itr = clamped_convert<T>( (*itr) - (*itr2) );
  return *this;
}

// ######################################################################
template <class T> template <class T2> inline
Image<T>& Image<T>::operator*=(const Image<T2>& A)
{
  ASSERT(isSameSize(A));
  typename Image<T2>::const_iterator itr2 = A.begin();
  for (typename Image<T>::iterator itr = beginw(), stop = endw();
       itr != stop; ++itr, ++itr2)
    *itr = clamped_convert<T>( (*itr) * (*itr2) );
  return *this;
}

// ######################################################################
template <class T> template <class T2> inline
Image<T>& Image<T>::operator/=(const Image<T2>& A)
{
  ASSERT(isSameSize(A));
  typename Image<T2>::const_iterator itr2 = A.begin();
  for (typename Image<T>::iterator itr = beginw(), stop = endw();
       itr != stop; ++itr, ++itr2)
    *itr = clamped_convert<T>( (*itr) / (*itr2) );
  return *this;
}

// ######################################################################
template <class T> template <class T2> inline
Image<T>& Image<T>::operator|=(const Image<T2>& A)
{
  ASSERT(isSameSize(A));
  typename Image<T2>::const_iterator itr2 = A.begin();
  for (typename Image<T>::iterator itr = beginw(), stop = endw();
       itr != stop; ++itr, ++itr2)
    *itr = clamped_convert<T>( (*itr) | (*itr2) );
  return *this;
}
// ######################################################################
template <class T> template <class T2> inline
Image<typename promote_trait<T,T2>::TP> Image<T>::operator+(const T2& val) const
{
  typedef typename promote_trait<T,T2>::TP TPRO;
  Image<TPRO> result(getWidth(), getHeight(), NO_INIT);
  typename Image<TPRO>::iterator ritr = result.beginw();
  for (typename Image<T>::const_iterator itr = begin(), stop = end();
       itr != stop; ++itr, ++ritr)
    *ritr = (*itr) + val;
  return result;
}

// ######################################################################
template <class T> template <class T2> inline
Image<typename promote_trait<T,T2>::TP> Image<T>::operator-(const T2& val) const
{
  typedef typename promote_trait<T,T2>::TP TPRO;
  Image<TPRO> result(getWidth(), getHeight(), NO_INIT);
  typename Image<TPRO>::iterator ritr = result.beginw();
  for (typename Image<T>::const_iterator itr = begin(), stop = end();
       itr != stop; ++itr, ++ritr)
    *ritr = (*itr) - val;
  return result;
}

// ######################################################################
template <class T> template <class T2> inline
Image<typename promote_trait<T,T2>::TP> Image<T>::operator*(const T2& val) const
{
  typedef typename promote_trait<T,T2>::TP TPRO;
  Image<TPRO> result(getWidth(), getHeight(), NO_INIT);
  typename Image<TPRO>::iterator ritr = result.beginw();
  for (typename Image<T>::const_iterator itr = begin(), stop = end();
       itr != stop; ++itr, ++ritr)
    *ritr = (*itr) * val;
  return result;
}

// ######################################################################
template <class T> template <class T2> inline
Image<typename promote_trait<T,T2>::TP> Image<T>::operator/(const T2& val) const
{
  typedef typename promote_trait<T,T2>::TP TPRO;
  Image<TPRO> result(getWidth(), getHeight(), NO_INIT);
  typename Image<TPRO>::iterator ritr = result.beginw();
  for (typename Image<T>::const_iterator itr = begin(), stop = end();
       itr != stop; ++itr, ++ritr)
    *ritr = (*itr) / val;
  return result;
}

// ######################################################################
template <class T> inline
Image<T> Image<T>::operator<<(const unsigned int nbits) const
{
  Image<T> result(getWidth(), getHeight(), NO_INIT);
  typename Image<T>::iterator ritr = result.beginw();
  for (typename Image<T>::const_iterator itr = this->begin(), stop = this->end();
       itr != stop; ++itr, ++ritr)
    *ritr = ((*itr) << nbits);
  return result;
}

// ######################################################################
template <class T> inline
Image<T> Image<T>::operator>>(const unsigned int nbits) const
{
  Image<T> result(getWidth(), getHeight(), NO_INIT);
  typename Image<T>::iterator ritr = result.beginw();
  for (typename Image<T>::const_iterator itr = this->begin(), stop = this->end();
       itr != stop; ++itr, ++ritr)
    *ritr = ((*itr) >> nbits);
  return result;
}

// ######################################################################
template <class T> template <class T2> inline
Image<typename promote_trait<T,T2>::TP> Image<T>::operator+(const Image<T2>& img) const
{
  ASSERT(isSameSize(img));
  typedef typename promote_trait<T,T2>::TP TPRO;
  Image<TPRO> result(getWidth(), getHeight(), NO_INIT);
  typename Image<TPRO>::iterator ritr = result.beginw();
  typename Image<T2>::const_iterator itr2 = img.begin();
  for (typename Image<T>::const_iterator itr = begin(), stop = end();
       itr != stop; ++itr, ++itr2, ++ritr)
    *ritr = (*itr) + (*itr2);
  return result;
}

// ######################################################################
template <class T> template <class T2> inline
Image<typename promote_trait<T,T2>::TP> Image<T>::operator-(const Image<T2>& img) const
{
  ASSERT(isSameSize(img));
  typedef typename promote_trait<T,T2>::TP TPRO;
  Image<TPRO> result(getWidth(), getHeight(), NO_INIT);
  typename Image<TPRO>::iterator ritr = result.beginw();
  typename Image<T2>::const_iterator itr2 = img.begin();
  for (typename Image<T>::const_iterator itr = begin(), stop = end();
       itr != stop; ++itr, ++itr2, ++ritr)
    *ritr = (*itr) - (*itr2);
  return result;
}

// ######################################################################
template <class T> template <class T2> inline
Image<typename promote_trait<T,T2>::TP> Image<T>::operator*(const Image<T2>& img) const
{
  ASSERT(isSameSize(img));
  typedef typename promote_trait<T,T2>::TP TPRO;
  Image<TPRO> result(getWidth(), getHeight(), NO_INIT);
  typename Image<TPRO>::iterator ritr = result.beginw();
  typename Image<T2>::const_iterator itr2 = img.begin();
  for (typename Image<T>::const_iterator itr = begin(), stop = end();
       itr != stop; ++itr, ++itr2, ++ritr)
    *ritr = (*itr) * (*itr2);
  return result;
}

// ######################################################################
template <class T> template <class T2> inline
Image<typename promote_trait<T,T2>::TP> Image<T>::operator/(const Image<T2>& img) const
{
  ASSERT(isSameSize(img));
  typedef typename promote_trait<T,T2>::TP TPRO;
  Image<TPRO> result(getWidth(), getHeight(), NO_INIT);
  typename Image<TPRO>::iterator ritr = result.beginw();
  typename Image<T2>::const_iterator itr2 = img.begin();
  for (typename Image<T>::const_iterator itr = begin(), stop = end();
       itr != stop; ++itr, ++itr2, ++ritr)
    *ritr = (*itr) / (*itr2);
  return result;
}

// ######################################################################
template <class T> inline
void Image<T>::clear(const T& val)
{
  // Check if we have a shared implementation... if yes, then for
  // efficiency we should release our copy before doing the iterative
  // loop, which would otherwise unnecessarily make a unique copy of
  // the shared data for us, which we would then ceremoniously ignore
  // as we clear it to a new value.
  if (itsHdl.isShared())
    *this = Image<T>(getDims(), NO_INIT);

  for (iterator itr = beginw(), stop = endw(); itr != stop; ++itr)
    *itr = val;
}

// ######################################################################
// ##### Functions for testing/debugging only:
// ######################################################################

template <class T> inline
bool Image<T>::hasSameData(const Image<T>& b) const
{ return itsHdl.hasSameData(b.itsHdl); }

// ######################################################################
template <class T> inline
long Image<T>::refCount() const throw() { return itsHdl.refCount(); }

// ######################################################################
template <class T> inline
bool Image<T>::isShared() const throw() { return itsHdl.isShared(); }

// ######################################################################
// ##### Private methods:
// ######################################################################

template <class T> inline
const ArrayData<T>& Image<T>::impl() const
{ return itsHdl.get(); }

// ######################################################################
template <class T> inline
ArrayData<T>& Image<T>::uniq()
{ return itsHdl.uniq(); }

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif
