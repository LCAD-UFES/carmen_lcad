/*!@file Image/ArrayData.H The guts of the 2D ref-counted copy-on-write array
    implementation */

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
// Primary maintainer for this file: Rob Peters <rjpeters@klab.caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/ArrayData.H $
// $Id: ArrayData.H 13229 2010-04-15 01:56:52Z dparks $
//

#ifndef ARRAYDATA_H_DEFINED
#define ARRAYDATA_H_DEFINED

#include "Image/Dims.H"
#include "Util/Alloc.H" // for invt_allocate()/invt_deallocate()
#include "Util/TypeTraits.H"
#include "Util/log.H"
#include "rutz/atomic.h"

#include <cstring> // for memset() and memcpy()
#include <new> // for placement new, in ArrayData's ctors

#ifdef INVT_USE_CUDA
#include "CUDA/CudaAlloc.H"
#include "CUDA/CudaDevices.H"
#endif

//! Specifies what to do with the T* used to init an ArrayData.
enum StoragePolicy
  {
    //! Make an initial copy of the data, then copy-on-write as usual.
    MAKE_OWN_COPY,

    //! Borrow the data, with write-through.
    /*! Writing to the Array will modify the T[] that initialized it. */
    WRITE_THRU
  };

//! Specifies how to initialize the pixels of a new image.
enum InitPolicy
  {
    //! All elements are initialized to the pixel type's zero or default value.
    ZEROS,

    //! Pixels are only minimally initialized.
    /*! For trivial types, this means the pixel values are not
        initialized at all; for non-trivial user-defined types, this
        means each pixel is copy-constructed from the type's default
        value. */
    NO_INIT
  };

//! Specifies where on a particular device the memory is stored
enum MemoryPolicy
  {
    //! On GPU, global (to a particular set of kernels on a single device)
    GLOBAL_DEVICE_MEMORY,
    //! On GPU, located in texture memory
    TEXTURE_DEVICE_MEMORY,
    //! On CPU/Host
    HOST_MEMORY,
  };

//! Device id for the host, to distinguish from CUDA devices
#define CUDA_HOST_DEVICE_NUM -1


//! Function to wrap use of invt_allocate to allow for CUDA memory policies
inline void* wrap_invt_allocate(size_t user_nbytes, const MemoryPolicy mp, const int dev)
  {
#ifdef INVT_USE_CUDA
    switch(mp)
    {
    case GLOBAL_DEVICE_MEMORY:
    case TEXTURE_DEVICE_MEMORY:
      return cuda_invt_allocate(user_nbytes,dev);
    case HOST_MEMORY:
      return invt_allocate(user_nbytes);
    default:
      LFATAL("Invalid memory policy %d",mp);
      return NULL;
    }
    LFATAL("Should be impossible to get here");
    return NULL;
#else
    // Optimization, don't look at memory policy
    return invt_allocate(user_nbytes);
#endif
  }

//! Function to wrap use of invt_deallocate to allow for CUDA memory policies
inline void wrap_invt_deallocate(void *mem, const MemoryPolicy mp, const int dev, const size_t nBytes)
  {
#ifdef INVT_USE_CUDA
    switch(mp)
    {
    case GLOBAL_DEVICE_MEMORY:
    case TEXTURE_DEVICE_MEMORY:
      cuda_invt_deallocate(mem, dev,nBytes);
      return;
    case HOST_MEMORY:
      invt_deallocate(mem);
      return;
    }
#else
    // Optimization, don't look at memory policy
    invt_deallocate(mem);
#endif
  }


void check_acquisition(StoragePolicy s, int count) throw();



//! Implements a ref-counted 2-D array, should only be used via ArrayHandle.
/*! ArrayData implements a 2-D array, by storing:
      - a Dims representing the width+height of the array,
      - a pointer to the dynamically allocated memory where the array
        data are stored,
      - a reference count, and
      - a StoragePolicy, indicating whether the ArrayData object
        "owns" the memory and should delete[] it eventually (in the
        case of MAKE_OWN_COPY), or whether someone else "owns" the
        memory and will eventually delete[] it themselves (in the case
        of WRITE_THRU)

    All of these data members are immutable, with the exception of the
    reference count. This reflects the semantics that an ArrayData
    object is a fixed-size block of data; if a higher-level clients
    wants to resize(), it does so by creating a new ArrayData of the
    desired size, and releasing the old ArrayData. This simplifies the
    memory management: within ArrayData, memory is new[]'ed only in
    the constructor(s), and delete[]'ed only in the destructor.

    ArrayData provides the basics needed to use STL-style iterator
    algorithms.  In particular it offers two pairs of functions to get
    half-open iterator ranges:

      - data() and end() for read-only iteration, and
      - dataw() and endw() for read-write iteration (where the "w"
        means "write" or "writable").
*/
template <class T>
class ArrayData
{
private:
  ArrayData(const ArrayData&); // not allowed; use clone() instead
  ArrayData& operator=(const ArrayData&); // not allowed; use clone() instead

public:

  // ############################################################
  // ##### Constructors
  // ############################################################

  //! Default construct an empty (i.e. 0-by-0) array.
  inline ArrayData();

  //! Construct with the specified dimensions and InitPolicy.
  inline ArrayData(const Dims& dims, const InitPolicy ip = NO_INIT);

  //! Construct with the specified dimensions and InitPolicy and Memory Policy
  inline ArrayData(const Dims& dims, const InitPolicy ip, const MemoryPolicy mp, const int device);

  //! Construct with the specified dimensions.
  /*! If the passed-in T* array is non-null, copy that data into the
      constructed object. */
  inline ArrayData(const Dims& dims, const T* aa);

  //! Construct with the specified dimensions, input data, and memory policy
  /*! If the passed-in T* array is non-null, copy that data into the
      constructed object.  Use the given memory policy */
  inline ArrayData(const Dims& dims, const T* aa, const MemoryPolicy mp, const int device, const MemoryPolicy srcmp, const int srcdevice);

  //! Construct with the specified dimensions and storage policy.
  /*! If the passed-in T* array is non-null, then if the storage
      policy is MAKE_OWN_COPY we make a copy of the data, but if the
      storage policy is WRITE_THRU then we "borrow" the data. */
  inline ArrayData(const Dims& dims, T* aa, const StoragePolicy s);

  //! Construct with the specified dimensions, storage and memory policies.
  /*! If the passed-in T* array is non-null, then if the storage
      policy is MAKE_OWN_COPY we make a copy of the data, but if the
      storage policy is WRITE_THRU then we "borrow" the data. */
  inline ArrayData(const Dims& dims, T* aa, const StoragePolicy s, const MemoryPolicy mp, const int device, const MemoryPolicy srcmp, const int srcdevice);

  //! Return a new'ed copy of *this.
  /*! The cloned copy will always be MAKE_OWN_COPY, even if *this is
      WRITE_THRU. */
  inline ArrayData* clone() const;

  //! Return a new'ed copy of *this using the given memory policy
  /*! The cloned copy will always be MAKE_OWN_COPY, even if *this is
      WRITE_THRU. */
  inline ArrayData* clone(const MemoryPolicy mp, const int device) const;

  //! Release the pointed-to data if our storage policy is MAKE_OWN_COPY.
  inline ~ArrayData() throw();

  // ############################################################
  // ##### Accessors
  // ############################################################

  //! Returns a read-only pointer to the data array.
  /*! With end(), forms an STL-style half-open const_iterator range. */
  inline const T* data() const throw();

  //! Returns a read-only pointer to one-past-the-end of the data array.
  /*! With data(), forms an STL-style half-open const_iterator range. */
  inline const T* end() const throw();

  //! Returns a read-write pointer to the data array (the "w" means "write").
  /*! With endw(), forms an STL-style half-open iterator range. */
  inline T* dataw() throw();

  //! Returns a read-write pointer to one-past-the-end of the data array.
  /*! With dataw(), forms an STL-style half-open iterator range. */
  inline T* endw() throw();

  //! Returns the dimensions (i.e. width, height) of the 2-D data array.
  inline const Dims& dims() const throw();

  //! Returns the width of the data array (i.e. the number of columns).
  inline int w() const throw();

  //! Returns the height of the data array (i.e. the number of rows).
  inline int h() const throw();

  // ############################################################
  // ##### Reference count manipulation
  // ############################################################

  //! Increment the reference count.
  inline void acquire() throw();

  //! Decrement the reference count and delete self if the count goes to zero.
  inline void release() throw();

  //! Query whether this object is shared (i.e. ref count > 1).
  inline bool isShared() const throw();

  //! For debugging/testing only.
  /*! Returns the current reference count. */
  inline int refCount() const throw();

  //! Return memory policy of underlying array data
  inline MemoryPolicy getMemoryPolicy() const;

  //! Return memory policy of underlying array data
  inline int getMemoryDevice() const;


private:
  rutz::atomic_int_t itsRefCount;
  MemoryPolicy const itsMemoryPolicy;  // Type of memory that is being used (global,texture,host)
  StoragePolicy const itsStoragePolicy;
  int const itsDevice;          // CUDA device (or host) where memory is located

  Dims const itsDims;           // width+height of data array
  T* const itsData;             // data array
};


//! This class provides ref-counting and copy-on-write for ArrayData's.
/*! NOTE: Most clients will not need to use ArrayHandle<T> directly,
    but can rather use Image<T>, which wraps and expands the
    ArrayHandle interface. Nevertheless, these notes may be of
    interest because the ref-counting and copy-on-write semantics that
    ArrayHandle<T> offers are transferred through to Image<T>.

    As long as clients access ArrayData<T> through an ArrayHandle<T>,
    the proper copy-on-write semantics are guaranteed. This is by
    virtue of the fact that the only way to extract a non-const
    ArrayData<T>& from an ArrayHandle<T> is to call uniq(), which in
    turn checks to see if the ArrayData is shared (i.e. ref count >
    1). If it is shared, then that ArrayData is clone()'d before
    returning a reference to it.

    The only caveat: you should not store non-const references to an
    ArrayData or to the elements of an ArrayData object for "long"
    periods of time. In this case, "long" means "longer than you are
    sure you have a unique reference", which in practice means "you
    can keep such references up until a new copy of the ArrayHandle<T>
    is made". This goes along with sensible coding practices anyway; a
    typical usage pattern would be:

      -# you have an ArrayHandle<T> (or an Image<T>, which holds an
         ArrayHandle<T>) that you want to modify
      -# get a pair of iterator's for that object so that you can
         perform your series of write operations
      -# don't use those iterators anymore, and then you can safely
         make copies of the freshly-modified ArrayHandle<T> or
         Image<T> object (for example to return a copy of it as the
         return value from a function)
*/
template<class T>
class ArrayHandle
{
public:
  //! Default construct with an empty array (i.e. 0-by-0).
  inline ArrayHandle();

  //! Construct with a new ArrayData, which the ArrayHandle now owns.
  inline explicit ArrayHandle(const ArrayData<T>* p) throw();

  //! Construct with a new ArrayData, which the ArrayHandle now owns.
  inline explicit ArrayHandle(const ArrayData<T>* p, const MemoryPolicy mp, const int dev) throw();

  //! Copy construct; no deep-copy is incurred, just bumps the ref-count.
  inline ArrayHandle(const ArrayHandle& r) throw();

  //! Destruct; delete's the ArrayData<T> if its ref count goes to zero.
  inline ~ArrayHandle() throw();

  //! Swap contents with another ArrayHandle.
  inline void swap(ArrayHandle<T>& other) throw();

  //! Assignment; no deep-copy is incurred, just bumps the ref-count.
  inline ArrayHandle& operator=(const ArrayHandle& r) throw();

  //! Get a read-only reference to the ArrayData object.
  /*! This is a "cheap" operation, since the ArrayData will never be
      copied in this operation. */
  inline const ArrayData<T>& get() const throw();

  //! Get a unique read-write reference to the ArrayData object.
  /*! This is a potentially "expensive" operation, since in order to
      get a unique ArrayData object, the current one may have to be
      clone()'d if it is shared (i.e. ref-count > 1). */
  inline ArrayData<T>& uniq(const MemoryPolicy mp, const int dev);

  //! Get a unique read-write reference to the ArrayData object.
  /*! This is a potentially "expensive" operation, since in order to
      get a unique ArrayData object, the current one may have to be
      clone()'d if it is shared (i.e. ref-count > 1). */
  inline ArrayData<T>& uniq();

  //! See if the ArrayData object is shared (i.e. ref-count > 1).
  inline bool isShared() const throw();

  //! For testing/debugging only.
  /*! See if we are pointing to the same ArrayData<T> as is the other
      ArrayHandle<T>. */
  inline bool hasSameData(const ArrayHandle<T>& b) const throw();

  //! For testing/debugging only.
  /*! Returns the current reference count of the our ArrayData<T>. */
  inline int refCount() const throw();

private:
  ArrayData<T>* px;     // contained pointer
};



// ######################################################################
// ######################################################################
// INLINE FUNCTIONS for ArrayData:
// ######################################################################
// ######################################################################

//! This is a helper template struct for ArrayData's constructors.
/*! It exposes functions for initializing raw memory in different
    ways. These functions can then be specialized for builtin or
    trivial types (i.e., types for which TypeTraits<T>::isTrivial is
    true) for maximum efficiency. We get this specialization by having
    an extra bool isTrivial template parameter for ArrayHelper, which
    allows us to select at compile-time based on the complexity of
    T. This is much better than using a run-time "if" statement in the
    constructors, since that incurs both a time cost (to evaluate an
    "if" statement) and a code space cost (since we're then including
    code that won't be used). */
template <class T, bool isTrivial = TypeTraits<T>::isTrivial>
struct ArrayHelper
{
  //! Construct \a n copies of \a val at \a space using placement new.
  /*! For general types, the minimal initialization that we can do is
      to zero_initialize() (i.e., unlike for builtin or trivial types,
      we can't just leave the memory sitting as it is,
      uninitialized). */
  static void minimal_initialize(T* trg, int n)
  {
    zero_initialize(trg, n);
  }

  static void minimal_initialize(T* trg, int n, const MemoryPolicy mp, const int dev)
  {
    zero_initialize(trg, n, mp, dev);
  }

  //! Initialize the memory to T's default or zero value.
  /*! For general types, the best way to do this is to make one
      default T object, and then copy construct each element of the
      array from that object using placement new. */
  static inline void zero_initialize(T* trg, int n)
  {
    T zero = T();
    T* cur = trg;
    try
      {
        for (; n > 0; --n, ++cur)
          new (cur) T(zero); // placement new
      }
    // if a constructor call throws, we have to clean up manually:
    catch (...)
      {
        while (cur != trg) (cur--)->~T(); // explicit destructor call
        invt_deallocate(trg);
        throw; // rethrow the exception
      }
  }

  static void zero_initialize(T* trg, int n, const MemoryPolicy mp, const int dev)
  {
#ifdef INVT_USE_CUDA
    T* tmp;
    switch(mp)
    {
    case GLOBAL_DEVICE_MEMORY:
    case TEXTURE_DEVICE_MEMORY:
      // Is there a better way to do this?
      tmp = new T[n];
      CudaDevices::memcpyHostToDevice(trg,tmp,n*sizeof(T),dev);
      delete tmp;
      break;
    case HOST_MEMORY:
      zero_initialize(trg,n);
      break;
    }
#else
    // Optimization, just assume the memory policy
    zero_initialize(trg,n);
#endif

  }

  //! Initialize the memory by copying elements from the \a src array.
  /*! For general types, we just copy construct each element in the
      target array from the corresponding element in the \a src array,
      using placement new. */
  static inline void copy_initialize(T* trg, int n, const T*src)
  {
        T* cur = trg;
        try
        {
          for (; n > 0; --n, ++cur)
            new (cur) T(*src++); // placement new
        }
        // if a constructor call throws, we have to clean up manually:
        catch (...)
        {
          while (cur != trg) (cur--)->~T(); // explicit destructor call
          invt_deallocate(trg);
          throw; // rethrow the exception
        }
  }

  // THIS IS AN UNSAFE DESIGN!  Complex types that allocate/deallocate resources in their constructors/destructors
  // are not properly handled when sent/received from CUDA devices under this framework.
  // The reason that non-primitive types are supported for CUDA right now, is because PixRGB and its ilk
  // are very useful, non-primitive type that will work with CUDA.
  // Maybe we should add a complext template or some other intelligence to demarcate the line between simple and complex classes
  // to fix this problem.  -- DFP 082009
  static void copy_initialize(T* trg, int n, const T*src, const MemoryPolicy trgmp, const int trgdev,
                              const MemoryPolicy srcmp, const int srcdev)
  {
#ifdef INVT_USE_CUDA
    T *tmp;
    switch(srcmp)
    {
    case GLOBAL_DEVICE_MEMORY:
    case TEXTURE_DEVICE_MEMORY:
      switch(trgmp)
      {
      case GLOBAL_DEVICE_MEMORY:
      case TEXTURE_DEVICE_MEMORY:
        // These types *SHOULD* be semi-trivial since they are going to the CUDA device
        CudaDevices::memcpyDeviceToDevice(trg,src,n*sizeof(T),trgdev,srcdev);
        break;
      case HOST_MEMORY:
        // Very inefficient, have to copy whole block from device to host, and then
        // call the constructor for each item in array using a new block of mem
        tmp = new T[n];
        CudaDevices::memcpyDeviceToHost(tmp,src,n*sizeof(T),srcdev);
        copy_initialize(trg,n,tmp);
        delete tmp;
        break;
      }

      break;
    case HOST_MEMORY:
      switch(trgmp)
      {
      case GLOBAL_DEVICE_MEMORY:
      case TEXTURE_DEVICE_MEMORY:
        CudaDevices::memcpyHostToDevice(trg,src,n*sizeof(T),trgdev);
        break;
      case HOST_MEMORY:
        copy_initialize(trg,n,src);
        break;
      }
      break;
    }
#else
    // Optimization, just assume the memory policy
    copy_initialize(trg,n,src);
#endif
  }



  //! Call destructors on the \a n objects stored in \a space.
  static inline void destruct(T* trg, int n)
  {
    while (--n >= 0)
      (trg+n)->~T(); // explicit destructor call
  }

  static void destruct(T* trg, int n, const MemoryPolicy mp, const int dev)
  {
#ifdef INVT_USE_CUDA
    switch(mp)
    {
    case GLOBAL_DEVICE_MEMORY:
    case TEXTURE_DEVICE_MEMORY:
      // What can we do here?
      break;
    case HOST_MEMORY:
      destruct(trg,n);
      break;
    }
#else
    // Optimization, just assume the memory policy
    destruct(trg,n);
#endif
  }

};

//! Specialization of ArrayHelper for builtin/trivial types.
/*! We can do things much faster with these kinds of types, using
    e.g. memset() and memcpy(), and we don't have to do default
    initialization at all. */
template <class T>
struct ArrayHelper<T, true>
{
  //! For trivial types, we don't have to do anything here.
  static void minimal_initialize(T* /*trg*/, int /*n*/) {}

  //! For trivial types, we don't have to do anything here.
  static void minimal_initialize(T* /*trg*/, int /*n*/, const MemoryPolicy /*mp*/, const int /*dev*/) {}

  //! For trivial types, we just memset() all the memory to zero.
  static inline void zero_initialize(T* trg, int n)
  {
    memset(trg, 0, n * sizeof(T));
  }

  static void zero_initialize(T* trg, int n, const MemoryPolicy mp, const int device)
  {
#ifdef INVT_USE_CUDA
    switch(mp)
    {
    case GLOBAL_DEVICE_MEMORY:
    case TEXTURE_DEVICE_MEMORY:
      CudaDevices::memset(trg,0,n*sizeof(T),device);
      break;
    case HOST_MEMORY:
      zero_initialize(trg, n);
      break;
    }
#else
    // Optimization, just assume the memory policy
    zero_initialize(trg, n);
#endif
  }

  //! For trivial types, we just memcpy() into the destination.
  static inline void copy_initialize(T* space, int n, const T* src)
  {
    memcpy(space, src, n * sizeof(T));
  }

  static void copy_initialize(T* trg, int n, const T*src, const MemoryPolicy trgmp, const int trgdev,
                              const MemoryPolicy srcmp, const int srcdev)
  {
#ifdef INVT_USE_CUDA
    switch(srcmp)
    {
    case GLOBAL_DEVICE_MEMORY:
    case TEXTURE_DEVICE_MEMORY:
      switch(trgmp)
      {
      case GLOBAL_DEVICE_MEMORY:
      case TEXTURE_DEVICE_MEMORY:
        CudaDevices::memcpyDeviceToDevice(trg,src,n*sizeof(T),trgdev,srcdev);
        break;
      case HOST_MEMORY:
        CudaDevices::memcpyDeviceToHost(trg,src,n*sizeof(T),srcdev);
        break;
      }
      break;
    case HOST_MEMORY:
      switch(trgmp)
      {
      case GLOBAL_DEVICE_MEMORY:
      case TEXTURE_DEVICE_MEMORY:
        CudaDevices::memcpyHostToDevice(trg,src,n*sizeof(T),trgdev);
        break;
      case HOST_MEMORY:
        copy_initialize(trg, n, src);
        break;
      }
      break;
    }
#else
    // Optimization, just assume the memory policy
    copy_initialize(trg, n, src);
#endif
  }

  //! For trivial types, no destruction is needed.
  static void destruct(T* /*space*/, int /*n*/) {}

  //! For trivial types, no destruction is needed.
  static void destruct(T* /*space*/, int /*n*/, const MemoryPolicy /*mp*/, const int /*dev*/) {}

};

// ######################################################################
template <class T> inline
ArrayData<T>::ArrayData() :
  itsRefCount(),
  itsMemoryPolicy(HOST_MEMORY),
  itsStoragePolicy(MAKE_OWN_COPY),
  itsDevice(CUDA_HOST_DEVICE_NUM),
  itsDims(0, 0),
  itsData(static_cast<T*>(wrap_invt_allocate(1,itsMemoryPolicy,itsDevice))) // just ask for one byte
{
  itsRefCount.atomic_set(0);
}

// ######################################################################
template <class T> inline
ArrayData<T>::ArrayData(const Dims& d, const InitPolicy ip) :
  itsRefCount(),
  itsMemoryPolicy(HOST_MEMORY),
  itsStoragePolicy(MAKE_OWN_COPY),
  itsDevice(CUDA_HOST_DEVICE_NUM),
  itsDims(d),
  itsData(static_cast<T*>(wrap_invt_allocate(itsDims.sz() * sizeof(T),itsMemoryPolicy,itsDevice)))
{
  if (ip == ZEROS)
    ArrayHelper<T>::zero_initialize(itsData, itsDims.sz(), itsMemoryPolicy, itsDevice);
  else
    ArrayHelper<T>::minimal_initialize(itsData, itsDims.sz(), itsMemoryPolicy, itsDevice);
  itsRefCount.atomic_set(0);
}


// ######################################################################
template <class T> inline
ArrayData<T>::ArrayData(const Dims& d, const InitPolicy ip, const MemoryPolicy mp, const int device) :
  itsRefCount(),
  itsMemoryPolicy(mp),
  itsStoragePolicy(MAKE_OWN_COPY),
  itsDevice(device),
  itsDims(d),
  itsData(static_cast<T*>(wrap_invt_allocate(itsDims.sz() * sizeof(T),itsMemoryPolicy,itsDevice)))
{
  if (ip == ZEROS)
    ArrayHelper<T>::zero_initialize(itsData, itsDims.sz(), itsMemoryPolicy, itsDevice);
  else
    ArrayHelper<T>::minimal_initialize(itsData, itsDims.sz(), itsMemoryPolicy, itsDevice);
  itsRefCount.atomic_set(0);
}

// ######################################################################
template <class T> inline
ArrayData<T>::ArrayData(const Dims& d, const T* aa) :
  itsRefCount(),
  itsMemoryPolicy(HOST_MEMORY),
  itsStoragePolicy(MAKE_OWN_COPY),
  itsDevice(CUDA_HOST_DEVICE_NUM),
  itsDims(d),
  itsData(static_cast<T*>(wrap_invt_allocate(itsDims.sz() * sizeof(T),itsMemoryPolicy,itsDevice)))
{
  ArrayHelper<T>::copy_initialize(itsData, itsDims.sz(), aa);

  itsRefCount.atomic_set(0);
}

// ######################################################################
template <class T> inline
ArrayData<T>::ArrayData(const Dims& d, const T* aa, const MemoryPolicy mp, const int device, const MemoryPolicy srcmp, const int srcdevice) :
  itsRefCount(),
  itsMemoryPolicy(mp),
  itsStoragePolicy(MAKE_OWN_COPY),
  itsDevice(device),
  itsDims(d),
  itsData(static_cast<T*>(wrap_invt_allocate(itsDims.sz() * sizeof(T),itsMemoryPolicy,itsDevice)))
{
  ArrayHelper<T>::copy_initialize(itsData, itsDims.sz(), aa, itsMemoryPolicy, itsDevice, srcmp, srcdevice);

  itsRefCount.atomic_set(0);
}

// ######################################################################
template <class T> inline
ArrayData<T>::ArrayData(const Dims& d, T* aa, const StoragePolicy s) :
  itsRefCount(),
  itsMemoryPolicy(HOST_MEMORY),
  itsStoragePolicy(s),
  itsDevice(CUDA_HOST_DEVICE_NUM),
  itsDims(d),
  itsData(s == MAKE_OWN_COPY
          ? static_cast<T*>(wrap_invt_allocate(itsDims.sz() * sizeof(T),itsMemoryPolicy,itsDevice))
          : aa)
{
  if (s == MAKE_OWN_COPY)
    {
      if (itsDims.sz() != 0 && aa != 0)
        ArrayHelper<T>::copy_initialize(itsData, itsDims.sz(), aa);
      else
        ArrayHelper<T>::minimal_initialize(itsData, itsDims.sz());
    }

  itsRefCount.atomic_set(0);
}


// ######################################################################
template <class T> inline
ArrayData<T>::ArrayData(const Dims& d, T* aa, const StoragePolicy s, const MemoryPolicy mp, const int device, const MemoryPolicy srcmp, const int srcdevice) :
  itsRefCount(),
  itsMemoryPolicy(mp),
  itsStoragePolicy(s),
  itsDevice(device),
  itsDims(d),
  itsData((s == MAKE_OWN_COPY)
          ? static_cast<T*>(wrap_invt_allocate(itsDims.sz() * sizeof(T),itsMemoryPolicy,itsDevice))
          : aa)
{
  if (s == MAKE_OWN_COPY)
    {
      if (itsDims.sz() != 0 && aa != 0)
        ArrayHelper<T>::copy_initialize(itsData, itsDims.sz(), aa, itsMemoryPolicy, itsDevice, srcmp, srcdevice);
      else
        ArrayHelper<T>::minimal_initialize(itsData, itsDims.sz(), itsMemoryPolicy, itsDevice);
    }
  else
    {
      if(itsMemoryPolicy!=srcmp || itsDevice!=srcdevice)
        LFATAL("WriteThru for ArrayData must be done using the same memory policy on the same device");
    }

  itsRefCount.atomic_set(0);
}

// ######################################################################
template <class T> inline
ArrayData<T>* ArrayData<T>::clone() const
{
  // When the new memory policy is not specified, we should either default to keeping it
  // in the same location, or default to the host. Currently, all memcpy's have to go
  // from dev->host->dev even if it is to/from the same device so it seems like
  // defaulting to the host makes sense for now
  return new ArrayData(itsDims, itsData, HOST_MEMORY, CUDA_HOST_DEVICE_NUM, itsMemoryPolicy, itsDevice);
}


// ######################################################################
template <class T> inline
ArrayData<T>* ArrayData<T>::clone(const MemoryPolicy mp, const int dev) const
{
  return new ArrayData(itsDims, itsData, mp, dev, itsMemoryPolicy, itsDevice);
}

// ######################################################################
template <class T> inline
ArrayData<T>::~ArrayData() throw()
{
  if (itsStoragePolicy == MAKE_OWN_COPY)
    {
      ArrayHelper<T>::destruct(itsData, itsDims.sz());
      wrap_invt_deallocate(itsData,itsMemoryPolicy,itsDevice,itsDims.sz()*sizeof(T));
    }
}

// ######################################################################
template <class T> inline
const T* ArrayData<T>::data() const throw()
{ return itsData; }

// ######################################################################
template <class T> inline
const T* ArrayData<T>::end() const throw()
{ return itsData + itsDims.sz(); }

// ######################################################################
template <class T> inline
T* ArrayData<T>::dataw() throw()
{ return itsData; }

// ######################################################################
template <class T> inline
T* ArrayData<T>::endw() throw()
{ return itsData + itsDims.sz(); }

// ######################################################################
template <class T> inline
const Dims& ArrayData<T>::dims() const throw()
{ return itsDims; }

// ######################################################################
template <class T> inline
int ArrayData<T>::w() const throw()
{ return itsDims.w(); }

// ######################################################################
template <class T> inline
int ArrayData<T>::h() const throw()
{ return itsDims.h(); }

// ######################################################################
template <class T> inline
void ArrayData<T>::acquire() throw()
{
  itsRefCount.atomic_incr();

  check_acquisition(itsStoragePolicy, itsRefCount.atomic_get());
}

// ######################################################################
template <class T> inline
void ArrayData<T>::release() throw()
{
  if ( itsRefCount.atomic_decr_test_zero() ) delete this;
}

// ######################################################################
template <class T> inline
bool ArrayData<T>::isShared() const throw()
{
  return (itsRefCount.atomic_get() > 1);
}

// ######################################################################
template <class T> inline
int ArrayData<T>::refCount() const throw()
{
  return itsRefCount.atomic_get();
}

// ######################################################################
template <class T> inline
MemoryPolicy ArrayData<T>::getMemoryPolicy() const
{
  return itsMemoryPolicy;
}

// ######################################################################
template <class T> inline
int ArrayData<T>::getMemoryDevice() const
{
  return itsDevice;
}


// ######################################################################
// ######################################################################
// INLINE FUNCTIONS for ArrayHandle:
// ######################################################################
// ######################################################################

// ######################################################################
template <class T> inline
ArrayHandle<T>::ArrayHandle() : px(new ArrayData<T>)
{
  px->acquire();
}


// ######################################################################
template <class T> inline
ArrayHandle<T>::ArrayHandle(const ArrayData<T>* p) throw() :
  // this cast is OK because we maintain const-correctness through
  // copy-on-write
  px(const_cast<ArrayData<T>*>(p))
{
  px->acquire();
}

// ######################################################################
template <class T> inline
ArrayHandle<T>::ArrayHandle(const ArrayData<T>* p, const MemoryPolicy mp, const int dev) throw() :
  // this cast is OK because we maintain const-correctness through
  // copy-on-write
  px(const_cast<ArrayData<T>*>(p->clone(mp,dev)))
{
  px->acquire();
}


// ######################################################################
template <class T> inline
ArrayHandle<T>::ArrayHandle(const ArrayHandle& r) throw() :
  px(r.px)
{
  px->acquire();
}

// ######################################################################
template <class T> inline
ArrayHandle<T>::~ArrayHandle() throw()
{
  px->release();
}

// ######################################################################
template <class T> inline
void ArrayHandle<T>::swap(ArrayHandle<T>& other) throw()
{
  ArrayData<T>* other_px = other.px;
  other.px = this->px;
  this->px = other_px;
}

// ######################################################################
template <class T> inline
ArrayHandle<T>& ArrayHandle<T>::operator=(const ArrayHandle& r) throw()
{
  ArrayHandle tmp(r); this->swap(tmp); return *this;
}

// ######################################################################
template <class T> inline
const ArrayData<T>& ArrayHandle<T>::get() const throw()
{ return *px; }

// ######################################################################
template <class T> inline
ArrayData<T>& ArrayHandle<T>::uniq(const MemoryPolicy mp, const int dev)
{
  if (px->isShared())
    {
      ArrayHandle copy(px->clone(mp,dev));
      this->swap(copy);
    }

  return *px;
}

// ######################################################################
template <class T> inline
ArrayData<T>& ArrayHandle<T>::uniq()
{
  if (px->isShared())
    {
      ArrayHandle copy(px->clone());
      this->swap(copy);
    }

  return *px;
}

// ######################################################################
template <class T> inline
bool ArrayHandle<T>::isShared() const throw()
{ return px->isShared(); }

// ######################################################################
template <class T> inline
bool ArrayHandle<T>::hasSameData(const ArrayHandle<T>& b) const throw()
{ return px==b.px; }

// ######################################################################
template <class T> inline
int ArrayHandle<T>::refCount() const throw()
{ return px->refCount(); }

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !ARRAY_IMPL_H_DEFINED
