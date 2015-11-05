#ifndef actinTypes_H_
#define actinTypes_H_
//     Copyright (c) 2009-2010 Energid Technologies. All rights reserved. ////
//
// Filename:    actinTypes.h
//
// Description: Platform independent variable types to support variety of
//              configurations.  If available, will use the boost libraries
//              to set types because they have an exhaustive compatibility
//              list.
//
/////////////////////////////////////////////////////////////////////////
#ifdef EC_HAVE_ACTIN
#  include <foundCore/ecConstants.h>
#else
#  include <string>
#  include <vector>
// Define some basic types equivalent on all platforms.
typedef bool                      EcBoolean;          ///< Boolean (0 or 1)
typedef float                     EcF32;              ///< 32-bit floating-point
typedef double                    EcF64;              ///< 64-bit floating-point
typedef EcF64                     EcReal;             ///< Alias for 64-bit float
typedef unsigned char             EcU8;               ///< 8-bit unsigned integer
typedef char                      EcInt8;             ///< 8-bit signed integer
typedef std::basic_string<char>   EcString;           ///< Character string

#  ifdef EC_HAVE_BOOST
// Boost does a really good job of properly defining size types for
// a wide variety of platforms.
#    include <boost/cstdint.hpp>
typedef boost::uint16_t           EcU16;              ///< 16-bit unsigned integer
typedef boost::int16_t            EcInt16;            ///< 16-bit signed integer
typedef boost::uint32_t           EcU32;              ///< 32-bit unsigned integer
typedef boost::int32_t            EcInt32;            ///< 32-bit signed integer
typedef boost::uint64_t           EcU64;              ///< 64-bit unsigned integer
typedef boost::int64_t            EcInt64;            ///< 64-bit signed integer
#  else // !EC_HAVE_BOOST
#    include <stdint.h>
// Assign 'typical' case for users that do not want boost.
// Platform dependent - use with caution.
typedef unsigned short            EcU16;              ///< 16-bit unsigned integer
typedef short                     EcInt16;            ///< 16-bit signed integer
#    if defined(WIN32) && !defined(_WIN64)
// Typical case for 32-bit Windows systems
typedef unsigned long             EcU32;              ///< 32-bit unsigned integer
typedef long                      EcInt32;            ///< 32-bit signed integer
typedef unsigned __int64          EcU64;              ///< 64-bit unsigned integer
typedef __int64                   EcInt64;            ///< 64-bit signed integer

#    elif defined(__uint32_t_defined) || defined(_UINT32_T)
// Debian and OSX definitions
typedef uint32_t                  EcU32;              ///< 32-bit unsigned integer
typedef int32_t                   EcInt32;            ///< 32-bit signed integer
typedef uint64_t                  EcU64;              ///< 64-bit unsigned integer
typedef int64_t                   EcInt64;            ///< 64-bit signed integer
#    elif ULONG_MAX == 0xffffffffUL
// 32-bit OS
typedef unsigned long             EcU32;              ///< 32-bit unsigned integer
typedef long                      EcInt32;            ///< 32-bit signed integer
typedef unsigned long long        EcU64;              ///< 64-bit unsigned integer
typedef long long                 EcInt64;            ///< 64-bit signed integer
#    else // ! ULONG_MAX == 0xffffffffUL
// 64-bit OS
typedef unsigned int              EcU32;              ///< 32-bit unsigned integer
typedef int                       EcInt32;            ///< 32-bit signed integer
typedef unsigned long             EcU64;              ///< 64-bit unsigned integer
typedef long                      EcInt64;            ///< 64-bit signed integer
#    endif // defined(WIN32) && !defined(_WIN64) 
#  endif // EC_HAVE_BOOST

// Add vector types.
typedef std::vector<EcF32>        EcF32Vector;        ///< Vector of floats
typedef std::vector<EcU32>        EcU32Vector;        ///< Vector of EcU32
typedef std::vector<EcReal>       EcRealVector;       ///< Vector of reals
typedef std::vector<EcRealVector> EcRealVectorVector; ///< Vector vector of reals
typedef std::vector<EcString>     EcStringVector;     ///< Vector of strings

// Define a few items
#  define EcFalse          false
#  define EcTrue           true
#  define EcNULL           0
#  define EcDELETE(x)      do { if(x) { delete    x; x = EcNULL; } } while(0)
#  define EcARRAYDELETE(x) do { if(x) { delete [] x; x = EcNULL; } } while(0)

// General sleep mechanism
#  ifdef WIN32
#    include <windows.h>
#    define EcSLEEPMS(ms)  Sleep(ms)
#  else
#    include <time.h>
#    define EcSLEEPMS(ms)  do { \
                              struct timespec ts={(ms)/1000,((ms)%1000)*1000000}; \
                              nanosleep(&ts, NULL); \
                           } while(0)
#  endif

// WIN32 DLL export declarations
#  ifdef WIN32
#    define EC_DECL_EXPORTS  __declspec(dllexport)
#    define EC_DECL_IMPORTS  __declspec(dllimport)
#  else
#    define EC_DECL_EXPORTS
#    define EC_DECL_IMPORTS
#  endif

#endif // EC_HAVE_ACTIN

#if WIN32 && (defined(actinSE_actinSE_EXPORTS) || defined(actinSE_actinSERender_EXPORTS) || defined(actinSE_actinSENetwork_EXPORTS))
#  define EC_ACTINSE_DECL          EC_DECL_EXPORTS
#  define EC_ACTINSE_TEMPLATE_DECL extern
#else
#  define EC_ACTINSE_DECL          EC_DECL_IMPORTS
#  define EC_ACTINSE_TEMPLATE_DECL
#endif


// Forward declarations of classes and types we use.
namespace boost { template <typename> class shared_ptr; }

namespace actinSE
{
class Array3;
class Orientation;
class CoordinateSystemTransformation;
class EndEffector;

/// Vector of 3 component array
typedef std::vector<Array3>             Array3Vector;
typedef std::vector<Array3Vector>       Array3VectorVector;
typedef std::vector<Array3VectorVector> Array3Vector3D;
/// Vector of end-effectors
typedef std::vector<EndEffector>        EndEffectorVector;
/// Shared pointer to end-effector
typedef boost::shared_ptr<EndEffector>  SharedEndEffector;
/// Vector of shared end-effectors
typedef std::vector<SharedEndEffector>  SharedEndEffectorVector;

struct ControlSystemImpl;
struct EndEffectorImpl;
template <typename T> struct NetworkInterfaceImpl;
}

#endif // actinTypes_H_
